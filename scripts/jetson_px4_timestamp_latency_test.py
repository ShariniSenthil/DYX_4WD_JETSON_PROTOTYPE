#!/usr/bin/env python3
"""
Read-only Jetson RPP -> MAVROS -> PX4 timestamp and latency recorder.

Measures:
  1. RPP source timestamp -> recorder receive age on Jetson.
  2. First matching RPP command -> cmd_vel_bridge PositionTarget delay.
  3. RPP and MAVROS setpoint topic rates.
  4. MAVROS/PX4 TIMESYNC round-trip time, offset stability and residual.
  5. PX4 telemetry header age for local odometry and IMU.
  6. Jetson wall-clock jumps relative to CLOCK_MONOTONIC.

It does not publish any control topic and cannot move the rover.

Important:
  - PX4 timestamps are normally boot-relative, while Jetson ROS timestamps are
    Unix/OS-clock based. Correct synchronization does not mean their raw numbers
    are equal. MAVROS estimated_offset_ns maps PX4 remote time into Jetson time.
  - Zero latency is physically impossible. This script measures the actual
    latency and reports min/mean/p95/p99/max.
  - Exact MAVLink command arrival inside PX4 must be checked against the PX4
    ULog from the same run. This recorder saves the clock mapping and command
    transition timestamps needed for that comparison.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import platform
import signal
import statistics
import subprocess
import sys
import time
from collections import Counter, deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Deque, Dict, Iterable, List, Optional

import rclpy
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import PositionTarget, TimesyncStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, TimeReference


NANOSECONDS_PER_SECOND = 1_000_000_000
MILLISECONDS_PER_SECOND = 1_000.0


def stamp_to_ns(stamp: Any) -> int:
    return (
        int(stamp.sec) * NANOSECONDS_PER_SECOND
        + int(stamp.nanosec)
    )


def utc_iso_from_ns(timestamp_ns: int) -> str:
    return datetime.fromtimestamp(
        timestamp_ns / NANOSECONDS_PER_SECOND,
        tz=timezone.utc,
    ).isoformat(timespec="microseconds")


def percentile(values: Iterable[float], percent: float) -> Optional[float]:
    ordered = sorted(float(value) for value in values if math.isfinite(value))
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]

    rank = (len(ordered) - 1) * percent / 100.0
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]

    fraction = rank - lower
    return ordered[lower] + fraction * (
        ordered[upper] - ordered[lower]
    )


def numeric_stats(values: Iterable[float]) -> Dict[str, Optional[float]]:
    cleaned = [float(value) for value in values if math.isfinite(value)]
    if not cleaned:
        return {
            "count": 0,
            "min": None,
            "mean": None,
            "p50": None,
            "p95": None,
            "p99": None,
            "max": None,
            "stddev": None,
        }

    return {
        "count": len(cleaned),
        "min": min(cleaned),
        "mean": statistics.fmean(cleaned),
        "p50": percentile(cleaned, 50.0),
        "p95": percentile(cleaned, 95.0),
        "p99": percentile(cleaned, 99.0),
        "max": max(cleaned),
        "stddev": (
            statistics.pstdev(cleaned)
            if len(cleaned) > 1
            else 0.0
        ),
    }


def topic_rate_hz(receive_monotonic_ns: List[int]) -> Optional[float]:
    if len(receive_monotonic_ns) < 2:
        return None
    duration = (
        receive_monotonic_ns[-1] - receive_monotonic_ns[0]
    ) / NANOSECONDS_PER_SECOND
    if duration <= 0.0:
        return None
    return (len(receive_monotonic_ns) - 1) / duration


def format_value(value: Optional[float], digits: int = 3) -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    return f"{value:.{digits}f}"


def run_text_command(command: List[str]) -> str:
    try:
        completed = subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=4.0,
        )
        return completed.stdout.strip()
    except Exception as error:
        return f"unavailable: {error}"


@dataclass
class RppSample:
    sequence: int
    source_stamp_ns: int
    receive_ros_ns: int
    receive_monotonic_ns: int
    north: float
    east: float
    speed: float
    paired: bool = False


class TimestampLatencyRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("jetson_px4_timestamp_latency_test")
        self.args = args

        self.start_realtime_ns = time.time_ns()
        self.start_monotonic_ns = time.monotonic_ns()
        self.stop_requested = False

        self.output_dir = Path(args.output_dir).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)

        timestamp_text = datetime.now().strftime("%Y%m%d_%H%M%S")
        prefix = args.output_prefix or (
            f"jetson_px4_timing_{timestamp_text}"
        )
        self.csv_path = self.output_dir / f"{prefix}.csv"
        self.summary_path = self.output_dir / f"{prefix}_summary.json"

        self.csv_handle = self.csv_path.open(
            "w",
            newline="",
            encoding="utf-8",
        )
        self.csv_writer = csv.DictWriter(
            self.csv_handle,
            fieldnames=[
                "event",
                "sequence",
                "receive_utc",
                "receive_realtime_ns",
                "receive_ros_ns",
                "receive_monotonic_ns",
                "source_stamp_ns",
                "source_age_ms",
                "north_mps",
                "east_mps",
                "speed_mps",
                "paired_rpp_sequence",
                "paired_rpp_stamp_ns",
                "rpp_to_bridge_header_ms",
                "rpp_to_bridge_receive_ms",
                "remote_timestamp_ns",
                "observed_offset_ns",
                "estimated_offset_ns",
                "timesync_sign",
                "timesync_rtt_ms",
                "timesync_residual_ms",
                "offset_error_ms",
                "note",
            ],
        )
        self.csv_writer.writeheader()

        best_effort_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Vector3Stamped,
            args.rpp_topic,
            self.on_rpp,
            best_effort_qos,
        )
        self.create_subscription(
            PositionTarget,
            args.bridge_topic,
            self.on_bridge,
            best_effort_qos,
        )
        self.create_subscription(
            TimesyncStatus,
            args.timesync_topic,
            self.on_timesync,
            best_effort_qos,
        )
        self.create_subscription(
            Odometry,
            args.odom_topic,
            self.on_odom,
            best_effort_qos,
        )
        self.create_subscription(
            Imu,
            args.imu_topic,
            self.on_imu,
            best_effort_qos,
        )
        self.create_subscription(
            TimeReference,
            args.time_reference_topic,
            self.on_time_reference,
            best_effort_qos,
        )

        self.rpp_sequence = 0
        self.bridge_sequence = 0
        self.timesync_sequence = 0
        self.odom_sequence = 0
        self.imu_sequence = 0
        self.time_reference_sequence = 0

        self.pending_rpp: Deque[RppSample] = deque(maxlen=500)

        self.rpp_receive_times_ns: List[int] = []
        self.bridge_receive_times_ns: List[int] = []
        self.timesync_receive_times_ns: List[int] = []
        self.odom_receive_times_ns: List[int] = []
        self.imu_receive_times_ns: List[int] = []

        self.rpp_age_ms: List[float] = []
        self.bridge_age_ms: List[float] = []
        self.rpp_to_bridge_header_ms: List[float] = []
        self.rpp_to_bridge_receive_ms: List[float] = []
        self.rpp_to_bridge_nonzero_header_ms: List[float] = []

        self.timesync_rtt_ms: List[float] = []
        self.timesync_residual_ms: List[float] = []
        self.timesync_abs_residual_ms: List[float] = []
        self.timesync_offset_error_ms: List[float] = []
        self.timesync_estimated_offset_ns: List[int] = []
        self.timesync_signs: Counter[int] = Counter()

        self.odom_age_ms: List[float] = []
        self.imu_age_ms: List[float] = []
        self.time_reference_age_ms: List[float] = []

        self.clock_jump_ms: List[float] = []
        self.previous_realtime_ns = time.time_ns()
        self.previous_monotonic_ns = time.monotonic_ns()

        self.last_live_print_monotonic_ns = 0
        self.missing_topics_reported = False

        self.create_timer(0.10, self.sample_host_clock)
        self.create_timer(args.print_interval, self.print_live_status)
        self.create_timer(1.0, self.check_duration)

        self.get_logger().warn(
            "READ-ONLY timing recorder started; no control topic is published"
        )
        self.get_logger().info(f"CSV: {self.csv_path}")
        self.get_logger().info(f"Summary: {self.summary_path}")

    def now_ros_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def common_receive_fields(self) -> Dict[str, Any]:
        realtime_ns = time.time_ns()
        monotonic_ns = time.monotonic_ns()
        ros_ns = self.now_ros_ns()
        return {
            "receive_utc": utc_iso_from_ns(realtime_ns),
            "receive_realtime_ns": realtime_ns,
            "receive_ros_ns": ros_ns,
            "receive_monotonic_ns": monotonic_ns,
        }

    def write_row(self, **values: Any) -> None:
        row = {
            field_name: ""
            for field_name in self.csv_writer.fieldnames
        }
        row.update(values)
        self.csv_writer.writerow(row)

    def on_rpp(self, message: Vector3Stamped) -> None:
        receive = self.common_receive_fields()
        source_stamp_ns = stamp_to_ns(message.header.stamp)
        age_ms = (
            receive["receive_ros_ns"] - source_stamp_ns
        ) / 1_000_000.0

        north = float(message.vector.x)
        east = float(message.vector.y)
        speed = math.hypot(north, east)

        self.rpp_sequence += 1
        sample = RppSample(
            sequence=self.rpp_sequence,
            source_stamp_ns=source_stamp_ns,
            receive_ros_ns=receive["receive_ros_ns"],
            receive_monotonic_ns=receive["receive_monotonic_ns"],
            north=north,
            east=east,
            speed=speed,
        )
        self.pending_rpp.append(sample)

        self.rpp_receive_times_ns.append(
            receive["receive_monotonic_ns"]
        )
        self.rpp_age_ms.append(age_ms)

        self.write_row(
            event="rpp",
            sequence=self.rpp_sequence,
            source_stamp_ns=source_stamp_ns,
            source_age_ms=f"{age_ms:.6f}",
            north_mps=f"{north:.9f}",
            east_mps=f"{east:.9f}",
            speed_mps=f"{speed:.9f}",
            **receive,
        )

    def find_matching_rpp(
        self,
        bridge_stamp_ns: int,
        north: float,
        east: float,
    ) -> Optional[RppSample]:
        maximum_age_ns = int(
            self.args.pair_window_ms * 1_000_000.0
        )

        for sample in reversed(self.pending_rpp):
            if sample.paired:
                continue

            delta_ns = bridge_stamp_ns - sample.source_stamp_ns
            if delta_ns < 0:
                continue
            if delta_ns > maximum_age_ns:
                break

            if (
                abs(sample.north - north)
                <= self.args.vector_epsilon_mps
                and abs(sample.east - east)
                <= self.args.vector_epsilon_mps
            ):
                return sample

        return None

    def on_bridge(self, message: PositionTarget) -> None:
        receive = self.common_receive_fields()
        source_stamp_ns = stamp_to_ns(message.header.stamp)
        age_ms = (
            receive["receive_ros_ns"] - source_stamp_ns
        ) / 1_000_000.0

        # cmd_vel_bridge publishes ROS PositionTarget in ENU:
        # velocity.x=East and velocity.y=North.
        east = float(message.velocity.x)
        north = float(message.velocity.y)
        speed = math.hypot(north, east)

        self.bridge_sequence += 1
        self.bridge_receive_times_ns.append(
            receive["receive_monotonic_ns"]
        )
        self.bridge_age_ms.append(age_ms)

        paired = self.find_matching_rpp(
            source_stamp_ns,
            north,
            east,
        )

        paired_sequence: Any = ""
        paired_stamp: Any = ""
        header_delta: Any = ""
        receive_delta: Any = ""

        if paired is not None:
            paired.paired = True
            paired_sequence = paired.sequence
            paired_stamp = paired.source_stamp_ns
            header_delta_value = (
                source_stamp_ns - paired.source_stamp_ns
            ) / 1_000_000.0
            receive_delta_value = (
                receive["receive_monotonic_ns"]
                - paired.receive_monotonic_ns
            ) / 1_000_000.0

            self.rpp_to_bridge_header_ms.append(
                header_delta_value
            )
            self.rpp_to_bridge_receive_ms.append(
                receive_delta_value
            )
            if paired.speed > self.args.nonzero_threshold_mps:
                self.rpp_to_bridge_nonzero_header_ms.append(
                    header_delta_value
                )

            header_delta = f"{header_delta_value:.6f}"
            receive_delta = f"{receive_delta_value:.6f}"

        self.write_row(
            event="bridge",
            sequence=self.bridge_sequence,
            source_stamp_ns=source_stamp_ns,
            source_age_ms=f"{age_ms:.6f}",
            north_mps=f"{north:.9f}",
            east_mps=f"{east:.9f}",
            speed_mps=f"{speed:.9f}",
            paired_rpp_sequence=paired_sequence,
            paired_rpp_stamp_ns=paired_stamp,
            rpp_to_bridge_header_ms=header_delta,
            rpp_to_bridge_receive_ms=receive_delta,
            **receive,
        )

    def on_timesync(self, message: TimesyncStatus) -> None:
        receive = self.common_receive_fields()
        source_stamp_ns = stamp_to_ns(message.header.stamp)
        remote_ns = int(message.remote_timestamp_ns)
        observed_offset_ns = int(message.observed_offset_ns)
        estimated_offset_ns = int(message.estimated_offset_ns)
        rtt_ms = float(message.round_trip_time_ms)

        # Different implementations describe offset direction differently.
        # Select the sign that maps PX4 remote/boot time closest to the local
        # ROS timestamp, and record that choice explicitly.
        plus_local_ns = remote_ns + estimated_offset_ns
        minus_local_ns = remote_ns - estimated_offset_ns
        plus_error_ns = source_stamp_ns - plus_local_ns
        minus_error_ns = source_stamp_ns - minus_local_ns

        if abs(plus_error_ns) <= abs(minus_error_ns):
            sign = 1
            residual_ns = plus_error_ns
        else:
            sign = -1
            residual_ns = minus_error_ns

        residual_ms = residual_ns / 1_000_000.0
        offset_error_ms = (
            observed_offset_ns - estimated_offset_ns
        ) / 1_000_000.0

        self.timesync_sequence += 1
        self.timesync_receive_times_ns.append(
            receive["receive_monotonic_ns"]
        )
        self.timesync_rtt_ms.append(rtt_ms)
        self.timesync_residual_ms.append(residual_ms)
        self.timesync_abs_residual_ms.append(abs(residual_ms))
        self.timesync_offset_error_ms.append(offset_error_ms)
        self.timesync_estimated_offset_ns.append(
            estimated_offset_ns
        )
        self.timesync_signs[sign] += 1

        self.write_row(
            event="timesync",
            sequence=self.timesync_sequence,
            source_stamp_ns=source_stamp_ns,
            source_age_ms=f"{(receive['receive_ros_ns'] - source_stamp_ns) / 1_000_000.0:.6f}",
            remote_timestamp_ns=remote_ns,
            observed_offset_ns=observed_offset_ns,
            estimated_offset_ns=estimated_offset_ns,
            timesync_sign=sign,
            timesync_rtt_ms=f"{rtt_ms:.6f}",
            timesync_residual_ms=f"{residual_ms:.6f}",
            offset_error_ms=f"{offset_error_ms:.6f}",
            **receive,
        )

    def on_odom(self, message: Odometry) -> None:
        receive = self.common_receive_fields()
        source_stamp_ns = stamp_to_ns(message.header.stamp)
        age_ms = (
            receive["receive_ros_ns"] - source_stamp_ns
        ) / 1_000_000.0

        self.odom_sequence += 1
        self.odom_receive_times_ns.append(
            receive["receive_monotonic_ns"]
        )
        self.odom_age_ms.append(age_ms)

        speed = math.hypot(
            float(message.twist.twist.linear.x),
            float(message.twist.twist.linear.y),
        )

        self.write_row(
            event="odom",
            sequence=self.odom_sequence,
            source_stamp_ns=source_stamp_ns,
            source_age_ms=f"{age_ms:.6f}",
            speed_mps=f"{speed:.9f}",
            **receive,
        )

    def on_imu(self, message: Imu) -> None:
        receive = self.common_receive_fields()
        source_stamp_ns = stamp_to_ns(message.header.stamp)
        age_ms = (
            receive["receive_ros_ns"] - source_stamp_ns
        ) / 1_000_000.0

        self.imu_sequence += 1
        self.imu_receive_times_ns.append(
            receive["receive_monotonic_ns"]
        )
        self.imu_age_ms.append(age_ms)

        self.write_row(
            event="imu",
            sequence=self.imu_sequence,
            source_stamp_ns=source_stamp_ns,
            source_age_ms=f"{age_ms:.6f}",
            **receive,
        )

    def on_time_reference(self, message: TimeReference) -> None:
        receive = self.common_receive_fields()
        source_stamp_ns = stamp_to_ns(message.header.stamp)
        age_ms = (
            receive["receive_ros_ns"] - source_stamp_ns
        ) / 1_000_000.0

        self.time_reference_sequence += 1
        self.time_reference_age_ms.append(age_ms)

        self.write_row(
            event="time_reference",
            sequence=self.time_reference_sequence,
            source_stamp_ns=source_stamp_ns,
            source_age_ms=f"{age_ms:.6f}",
            note=str(message.source),
            **receive,
        )

    def sample_host_clock(self) -> None:
        realtime_ns = time.time_ns()
        monotonic_ns = time.monotonic_ns()

        realtime_delta_ns = realtime_ns - self.previous_realtime_ns
        monotonic_delta_ns = (
            monotonic_ns - self.previous_monotonic_ns
        )
        jump_ms = (
            realtime_delta_ns - monotonic_delta_ns
        ) / 1_000_000.0
        self.clock_jump_ms.append(jump_ms)

        self.previous_realtime_ns = realtime_ns
        self.previous_monotonic_ns = monotonic_ns

    def elapsed_seconds(self) -> float:
        return (
            time.monotonic_ns() - self.start_monotonic_ns
        ) / NANOSECONDS_PER_SECOND

    def check_duration(self) -> None:
        if (
            self.args.duration > 0.0
            and self.elapsed_seconds() >= self.args.duration
        ):
            self.stop_requested = True
            rclpy.shutdown()

    def print_live_status(self) -> None:
        rpp_rate = topic_rate_hz(self.rpp_receive_times_ns)
        bridge_rate = topic_rate_hz(self.bridge_receive_times_ns)

        pair_stats = numeric_stats(
            self.rpp_to_bridge_nonzero_header_ms
            or self.rpp_to_bridge_header_ms
        )
        rtt_stats = numeric_stats(self.timesync_rtt_ms)
        residual_stats = numeric_stats(
            self.timesync_abs_residual_ms
        )

        print(
            (
                f"t={self.elapsed_seconds():6.1f}s | "
                f"RPP={format_value(rpp_rate, 2)}Hz | "
                f"PX4_SP={format_value(bridge_rate, 2)}Hz | "
                f"RPP->BRIDGE p95="
                f"{format_value(pair_stats['p95'])}ms | "
                f"TIMESYNC RTT p95="
                f"{format_value(rtt_stats['p95'])}ms | "
                f"SYNC residual|p95="
                f"{format_value(residual_stats['p95'])}ms | "
                f"samples: rpp={len(self.rpp_receive_times_ns)} "
                f"bridge={len(self.bridge_receive_times_ns)} "
                f"sync={len(self.timesync_receive_times_ns)}"
            ),
            flush=True,
        )

    def threshold_result(
        self,
        *,
        name: str,
        actual: Optional[float],
        operator: str,
        limit: float,
        minimum_samples: int,
        sample_count: int,
    ) -> Dict[str, Any]:
        if sample_count < minimum_samples or actual is None:
            return {
                "name": name,
                "status": "FAIL",
                "actual": actual,
                "operator": operator,
                "limit": limit,
                "sample_count": sample_count,
                "reason": "insufficient samples",
            }

        if operator == "<=":
            passed = actual <= limit
        elif operator == ">=":
            passed = actual >= limit
        else:
            raise ValueError(f"unsupported operator: {operator}")

        return {
            "name": name,
            "status": "PASS" if passed else "FAIL",
            "actual": actual,
            "operator": operator,
            "limit": limit,
            "sample_count": sample_count,
        }

    def build_summary(self) -> Dict[str, Any]:
        end_realtime_ns = time.time_ns()
        duration_s = self.elapsed_seconds()

        rpp_rate = topic_rate_hz(self.rpp_receive_times_ns)
        bridge_rate = topic_rate_hz(self.bridge_receive_times_ns)
        timesync_rate = topic_rate_hz(
            self.timesync_receive_times_ns
        )

        rpp_age_stats = numeric_stats(self.rpp_age_ms)
        bridge_age_stats = numeric_stats(self.bridge_age_ms)
        pair_header_stats = numeric_stats(
            self.rpp_to_bridge_header_ms
        )
        pair_nonzero_stats = numeric_stats(
            self.rpp_to_bridge_nonzero_header_ms
        )
        pair_receive_stats = numeric_stats(
            self.rpp_to_bridge_receive_ms
        )
        rtt_stats = numeric_stats(self.timesync_rtt_ms)
        residual_stats = numeric_stats(
            self.timesync_residual_ms
        )
        abs_residual_stats = numeric_stats(
            self.timesync_abs_residual_ms
        )
        offset_error_stats = numeric_stats(
            self.timesync_offset_error_ms
        )
        offset_stats_ms = numeric_stats(
            value / 1_000_000.0
            for value in self.timesync_estimated_offset_ns
        )
        odom_age_stats = numeric_stats(self.odom_age_ms)
        imu_age_stats = numeric_stats(self.imu_age_ms)
        clock_jump_abs_stats = numeric_stats(
            abs(value) for value in self.clock_jump_ms
        )

        selected_pair_stats = (
            pair_nonzero_stats
            if pair_nonzero_stats["count"]
            else pair_header_stats
        )

        checks = [
            self.threshold_result(
                name="rpp_rate_min_hz",
                actual=rpp_rate,
                operator=">=",
                limit=self.args.min_rpp_rate_hz,
                minimum_samples=10,
                sample_count=len(self.rpp_receive_times_ns),
            ),
            self.threshold_result(
                name="bridge_rate_min_hz",
                actual=bridge_rate,
                operator=">=",
                limit=self.args.min_bridge_rate_hz,
                minimum_samples=20,
                sample_count=len(self.bridge_receive_times_ns),
            ),
            self.threshold_result(
                name="rpp_receive_age_p95_ms",
                actual=rpp_age_stats["p95"],
                operator="<=",
                limit=self.args.max_rpp_receive_ms,
                minimum_samples=10,
                sample_count=int(rpp_age_stats["count"] or 0),
            ),
            self.threshold_result(
                name="rpp_to_bridge_p95_ms",
                actual=selected_pair_stats["p95"],
                operator="<=",
                limit=self.args.max_rpp_to_bridge_ms,
                minimum_samples=5,
                sample_count=int(
                    selected_pair_stats["count"] or 0
                ),
            ),
            self.threshold_result(
                name="timesync_rtt_p95_ms",
                actual=rtt_stats["p95"],
                operator="<=",
                limit=self.args.max_timesync_rtt_ms,
                minimum_samples=5,
                sample_count=int(rtt_stats["count"] or 0),
            ),
            self.threshold_result(
                name="timesync_abs_residual_p95_ms",
                actual=abs_residual_stats["p95"],
                operator="<=",
                limit=self.args.max_timesync_residual_ms,
                minimum_samples=5,
                sample_count=int(
                    abs_residual_stats["count"] or 0
                ),
            ),
            self.threshold_result(
                name="odom_age_p95_ms",
                actual=odom_age_stats["p95"],
                operator="<=",
                limit=self.args.max_telemetry_age_ms,
                minimum_samples=10,
                sample_count=int(odom_age_stats["count"] or 0),
            ),
            self.threshold_result(
                name="host_clock_jump_abs_max_ms",
                actual=clock_jump_abs_stats["max"],
                operator="<=",
                limit=self.args.max_clock_jump_ms,
                minimum_samples=10,
                sample_count=int(
                    clock_jump_abs_stats["count"] or 0
                ),
            ),
        ]

        overall_pass = all(
            check["status"] == "PASS"
            for check in checks
        )

        host_info = {
            "hostname": platform.node(),
            "platform": platform.platform(),
            "python": sys.version,
            "pid": os.getpid(),
            "timedatectl": run_text_command(
                [
                    "timedatectl",
                    "show",
                    "-p",
                    "NTPSynchronized",
                    "-p",
                    "NTP",
                    "-p",
                    "TimeUSec",
                ]
            ),
            "chronyc_tracking": run_text_command(
                ["chronyc", "tracking"]
            ),
        }

        return {
            "schema_version": 1,
            "start_utc": utc_iso_from_ns(
                self.start_realtime_ns
            ),
            "end_utc": utc_iso_from_ns(end_realtime_ns),
            "start_realtime_ns": self.start_realtime_ns,
            "end_realtime_ns": end_realtime_ns,
            "duration_s": duration_s,
            "csv_file": str(self.csv_path),
            "host": host_info,
            "topics": {
                "rpp": self.args.rpp_topic,
                "bridge": self.args.bridge_topic,
                "timesync": self.args.timesync_topic,
                "odom": self.args.odom_topic,
                "imu": self.args.imu_topic,
                "time_reference": (
                    self.args.time_reference_topic
                ),
            },
            "rates_hz": {
                "rpp": rpp_rate,
                "bridge": bridge_rate,
                "timesync": timesync_rate,
                "odom": topic_rate_hz(
                    self.odom_receive_times_ns
                ),
                "imu": topic_rate_hz(
                    self.imu_receive_times_ns
                ),
            },
            "latency_ms": {
                "rpp_source_to_recorder": rpp_age_stats,
                "bridge_source_to_recorder": bridge_age_stats,
                "rpp_to_first_matching_bridge_header": (
                    pair_header_stats
                ),
                "rpp_to_first_matching_bridge_nonzero": (
                    pair_nonzero_stats
                ),
                "rpp_to_first_matching_bridge_receive": (
                    pair_receive_stats
                ),
                "px4_odom_to_recorder": odom_age_stats,
                "px4_imu_to_recorder": imu_age_stats,
            },
            "timesync": {
                "rtt_ms": rtt_stats,
                "signed_residual_ms": residual_stats,
                "absolute_residual_ms": abs_residual_stats,
                "observed_minus_estimated_offset_ms": (
                    offset_error_stats
                ),
                "estimated_offset_ms": offset_stats_ms,
                "selected_offset_sign_counts": dict(
                    self.timesync_signs
                ),
                "mapping": (
                    "jetson_time_ns = remote_px4_ns "
                    "+ sign * estimated_offset_ns"
                ),
            },
            "host_clock": {
                "absolute_jump_ms": clock_jump_abs_stats,
            },
            "checks": checks,
            "overall_status": (
                "PASS" if overall_pass else "FAIL"
            ),
            "interpretation": {
                "raw_timestamp_equality_expected": False,
                "zero_latency_physically_possible": False,
                "live_test_scope": (
                    "Jetson RPP->bridge latency, MAVROS/PX4 "
                    "clock synchronization, and PX4 telemetry age"
                ),
                "ulog_required_for": (
                    "exact bridge/MAVLink command-arrival latency "
                    "inside PX4"
                ),
            },
        }

    def finalize(self) -> Dict[str, Any]:
        self.csv_handle.flush()
        os.fsync(self.csv_handle.fileno())
        self.csv_handle.close()

        summary = self.build_summary()
        self.summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True)
            + "\n",
            encoding="utf-8",
        )

        print("\n===== JETSON / PX4 TIMING RESULT =====")
        print(f"Overall: {summary['overall_status']}")
        for check in summary["checks"]:
            actual = check["actual"]
            print(
                f"{check['status']:4s} | "
                f"{check['name']}: "
                f"actual={format_value(actual)} "
                f"{check['operator']} {check['limit']} "
                f"(samples={check['sample_count']})"
            )

        print(f"\nCSV:     {self.csv_path}")
        print(f"Summary: {self.summary_path}")
        print(
            "Upload both files together with the PX4 ULog "
            "from this exact run."
        )
        return summary


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Record Jetson RPP, MAVROS bridge and PX4 time-sync "
            "latency without publishing control commands."
        )
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="recording duration in seconds; 0 means until Ctrl-C",
    )
    parser.add_argument(
        "--output-dir",
        default="~/rover_ws/logs/timing",
    )
    parser.add_argument("--output-prefix", default="")

    parser.add_argument(
        "--rpp-topic",
        default="/rpp/velocity_ned",
    )
    parser.add_argument(
        "--bridge-topic",
        default="/mavros/setpoint_raw/local",
    )
    parser.add_argument(
        "--timesync-topic",
        default="/mavros/timesync_status",
    )
    parser.add_argument(
        "--odom-topic",
        default="/mavros/local_position/odom",
    )
    parser.add_argument(
        "--imu-topic",
        default="/mavros/imu/data",
    )
    parser.add_argument(
        "--time-reference-topic",
        default="/mavros/time_reference",
    )

    parser.add_argument(
        "--pair-window-ms",
        type=float,
        default=100.0,
    )
    parser.add_argument(
        "--vector-epsilon-mps",
        type=float,
        default=0.001,
    )
    parser.add_argument(
        "--nonzero-threshold-mps",
        type=float,
        default=0.01,
    )

    parser.add_argument(
        "--min-rpp-rate-hz",
        type=float,
        default=18.0,
    )
    parser.add_argument(
        "--min-bridge-rate-hz",
        type=float,
        default=45.0,
    )
    parser.add_argument(
        "--max-rpp-receive-ms",
        type=float,
        default=10.0,
    )
    parser.add_argument(
        "--max-rpp-to-bridge-ms",
        type=float,
        default=30.0,
    )
    parser.add_argument(
        "--max-timesync-rtt-ms",
        type=float,
        default=10.0,
    )
    parser.add_argument(
        "--max-timesync-residual-ms",
        type=float,
        default=10.0,
    )
    parser.add_argument(
        "--max-telemetry-age-ms",
        type=float,
        default=50.0,
    )
    parser.add_argument(
        "--max-clock-jump-ms",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--print-interval",
        type=float,
        default=2.0,
    )
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()

    if args.duration < 0.0:
        raise SystemExit("--duration must be >= 0")
    if args.print_interval <= 0.0:
        raise SystemExit("--print-interval must be > 0")

    rclpy.init()
    node = TimestampLatencyRecorder(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        summary = node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if summary["overall_status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())