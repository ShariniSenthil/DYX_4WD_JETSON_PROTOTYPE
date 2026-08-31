#!/usr/bin/env python3
"""Field-test telemetry logger for the DYX 4WD marking rover.

This node is MONITORING ONLY. It never publishes a control topic and never
changes RPP, Mission Manager, PX4, RTK, or spray behavior.

It writes:
  telemetry.csv  - 20 Hz synchronized snapshots for quick analysis
  events.jsonl   - raw JSON status/event messages with receive timestamps

Start it through scripts/start_field_test_logging.sh so the same test also
records a rosbag containing the original ROS messages.
"""

from __future__ import annotations

import csv
import json
import math
import os
import signal
import sys
import time
from pathlib import Path
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Float64, String

from mavros_msgs.msg import GPSRAW, PositionTarget, State


def _finite(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _json_dict(text: str) -> dict[str, Any]:
    try:
        value = json.loads(text)
    except (TypeError, ValueError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class FieldTestLogger(Node):
    SAMPLE_HZ = 50.0

    def __init__(self, out_dir: Path) -> None:
        super().__init__("field_test_logger")

        self.out_dir = out_dir
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.out_dir / "telemetry.csv"
        self.events_path = self.out_dir / "events.jsonl"

        self._csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._event_file = self.events_path.open("a", encoding="utf-8")
        self._writer = csv.DictWriter(self._csv_file, fieldnames=self._columns())
        self._writer.writeheader()

        self._last_flush = time.monotonic()
        self._start_mono = time.monotonic()

        # Latest values. None means no valid sample has been received yet.
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw_deg = None
        self.odom_vx = None
        self.odom_vy = None
        self.odom_speed = None
        self.odom_yaw_rate = None

        self.gps_fix_type = None
        self.gps_satellites = None
        self.px4_connected = None
        self.px4_armed = None
        self.px4_mode = None

        self.rpp_vx = None
        self.rpp_vy = None
        self.rpp_vz = None
        self.rpp_command_speed = None
        self.rpp_xtrack_mm = None
        self.rpp_goal_distance_mm = None
        self.rpp_along_mm = None
        self.rpp_closest_mm = None
        self.rpp_accel_active = None
        self.rpp_accel_progress_m = None
        self.rpp_decel_active = None
        self.rpp_decel_progress_m = None
        self.rpp_decel_remaining_m = None
        self.rpp_terminal_precision_armed = None
        self.rpp_terminal_bearing_frozen = None
        self.rpp_terminal_correction_deg = None
        self.rpp_xtrack_speed_cap_active = None
        self.rpp_xtrack_speed_cap_mps = None
        self.rpp_accuracy: dict[str, Any] = {}
        self.rpp_debug: dict[str, Any] = {}

        self.setpoint_vx = None
        self.setpoint_vy = None
        self.setpoint_vz = None
        self.setpoint_yaw_rate = None

        self.mission_enable = None
        self.emergency_stop = None
        self.backend_heartbeat_healthy = None
        self.rtk_bridge_healthy = None
        self.rtk_correction_age_sec = None
        self.mission_status: dict[str, Any] = {}
        self.spray_status: dict[str, Any] = {}

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        retained_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(Odometry, "/mavros/local_position/odom", self._odom_cb, sensor_qos)
        self.create_subscription(GPSRAW, "/mavros/gpsstatus/gps1/raw", self._gps_cb, sensor_qos)
        self.create_subscription(State, "/mavros/state", self._state_cb, reliable_qos)
        self.create_subscription(PositionTarget, "/mavros/setpoint_raw/local", self._setpoint_cb, reliable_qos)

        self.create_subscription(Vector3Stamped, "/rpp/velocity_ned", self._rpp_velocity_cb, reliable_qos)
        self.create_subscription(Float64, "/rpp/command_speed_mps", lambda m: self._set("rpp_command_speed", m.data), reliable_qos)
        self.create_subscription(Float64, "/rpp/xtrack_mm", lambda m: self._set("rpp_xtrack_mm", m.data), retained_qos)
        self.create_subscription(Float64, "/rpp/goal_distance_mm", lambda m: self._set("rpp_goal_distance_mm", m.data), retained_qos)
        self.create_subscription(Float64, "/rpp/along_track_remaining_mm", lambda m: self._set("rpp_along_mm", m.data), retained_qos)
        self.create_subscription(Float64, "/rpp/closest_goal_distance_mm", lambda m: self._set("rpp_closest_mm", m.data), retained_qos)
        self.create_subscription(Bool, "/rpp/acceleration_active", lambda m: self._set("rpp_accel_active", m.data), retained_qos)
        self.create_subscription(Float64, "/rpp/acceleration_progress_m", lambda m: self._set("rpp_accel_progress_m", m.data), reliable_qos)
        self.create_subscription(Bool, "/rpp/deceleration_active", lambda m: self._set("rpp_decel_active", m.data), retained_qos)
        self.create_subscription(Float64, "/rpp/deceleration_progress_m", lambda m: self._set("rpp_decel_progress_m", m.data), reliable_qos)
        self.create_subscription(Float64, "/rpp/deceleration_remaining_m", lambda m: self._set("rpp_decel_remaining_m", m.data), reliable_qos)
        self.create_subscription(Bool, "/rpp/terminal_precision_armed", lambda m: self._set("rpp_terminal_precision_armed", m.data), retained_qos)
        self.create_subscription(Bool, "/rpp/terminal_bearing_frozen", lambda m: self._set("rpp_terminal_bearing_frozen", m.data), retained_qos)
        self.create_subscription(Float64, "/rpp/terminal_correction_deg", lambda m: self._set("rpp_terminal_correction_deg", m.data), reliable_qos)
        self.create_subscription(Bool, "/rpp/xtrack_speed_cap_active", lambda m: self._set("rpp_xtrack_speed_cap_active", m.data), reliable_qos)
        self.create_subscription(Float64, "/rpp/xtrack_speed_cap_mps", lambda m: self._set("rpp_xtrack_speed_cap_mps", m.data), reliable_qos)
        self.create_subscription(String, "/rpp/accuracy", self._rpp_accuracy_cb, retained_qos)
        self.create_subscription(String, "/rpp/debug", self._rpp_debug_cb, sensor_qos)

        self.create_subscription(Bool, "/mission_enable", lambda m: self._set("mission_enable", m.data), reliable_qos)
        self.create_subscription(Bool, "/emergency_stop", lambda m: self._set("emergency_stop", m.data), reliable_qos)
        self.create_subscription(Bool, "/cmd_vel_bridge/backend_heartbeat_healthy", lambda m: self._set("backend_heartbeat_healthy", m.data), retained_qos)
        self.create_subscription(Bool, "/rtk_correction_bridge/healthy", lambda m: self._set("rtk_bridge_healthy", m.data), retained_qos)
        self.create_subscription(Float32, "/rtk_correction_bridge/correction_age_sec", lambda m: self._set("rtk_correction_age_sec", m.data), reliable_qos)

        self.create_subscription(String, "/mission_manager/status", self._mission_status_cb, reliable_qos)
        self.create_subscription(String, "/mission_manager/point_event", lambda m: self._event("mission_manager/point_event", m.data), reliable_qos)
        self.create_subscription(String, "/spray/status", self._spray_status_cb, reliable_qos)

        self.timer = self.create_timer(1.0 / self.SAMPLE_HZ, self._sample)
        self.get_logger().info(f"Field logger writing to {self.out_dir}")

    @staticmethod
    def _columns() -> list[str]:
        return [
            "timestamp_unix_ns", "elapsed_sec",
            "odom_x_m", "odom_y_m", "odom_yaw_deg", "actual_vx_mps", "actual_vy_mps", "actual_speed_mps", "actual_yaw_rate_radps",
            "gps_fix_type", "gps_satellites", "px4_connected", "px4_armed", "px4_mode",
            "mission_state", "execution_mode", "current_point_id", "current_point_index", "current_point_state",
            "pause_reason", "resume_available", "mission_enable", "emergency_stop", "backend_heartbeat_healthy",
            "rtk_state", "rtk_fixed", "rtk_healthy", "rtk_motion_ok", "rtk_reason", "rtk_correction_age_sec",
            "gps_fix_status_age_sec", "rtk_health_status_age_sec", "rtk_age_status_age_sec",
            "arrival_settle_elapsed_sec", "arrival_settle_required_sec",
            "marking_active", "marking_radial_error_m", "marking_xtrack_m", "marking_along_error_m", "marking_combined_error_m",
            "rpp_goal_number", "rpp_cross_track_error_mm", "rpp_front_back_error_mm", "rpp_radial_error_mm", "rpp_closest_radial_error_mm", "rpp_accuracy_status",
            "rpp_debug_telemetry_sequence", "rpp_debug_control_sequence", "rpp_debug_control_sample_age_ms", "rpp_debug_odom_age_ms", "rpp_debug_control_dt_ms", "rpp_debug_control_compute_ms", "rpp_debug_deadline_missed", "rpp_debug_mode", "rpp_debug_reason",
            "rpp_debug_actual_speed_mps", "rpp_debug_command_speed_mps", "rpp_debug_heading_error_deg", "rpp_debug_cross_track_error_mm", "rpp_debug_along_remaining_mm", "rpp_debug_distance_to_goal_m",
            "rpp_command_speed_mps", "rpp_velocity_north_mps", "rpp_velocity_east_mps", "rpp_velocity_down_mps",
            "rpp_accel_active", "rpp_accel_progress_m", "rpp_decel_active", "rpp_decel_progress_m", "rpp_decel_remaining_m",
            "rpp_xtrack_mm_topic", "rpp_goal_distance_mm_topic", "rpp_along_remaining_mm_topic", "rpp_closest_goal_distance_mm_topic",
            "rpp_terminal_precision_armed", "rpp_terminal_bearing_frozen", "rpp_terminal_correction_deg",
            "rpp_xtrack_speed_cap_active", "rpp_xtrack_speed_cap_mps",
            "setpoint_vx_mps", "setpoint_vy_mps", "setpoint_vz_mps", "setpoint_yaw_rate_radps",
            "spray_controller_state", "spraying", "spray_duration_sec", "spray_elapsed_sec", "spray_remaining_sec", "spray_ready", "spray_fault_latched", "spray_fault_reason",
            "rtk_bridge_healthy",
        ]

    def _set(self, name: str, value: Any) -> None:
        setattr(self, name, value)

    def _event(self, topic: str, data: str) -> None:
        record = {
            "timestamp_unix_ns": time.time_ns(),
            "elapsed_sec": round(time.monotonic() - self._start_mono, 6),
            "topic": topic,
            "payload": _json_dict(data) or data,
        }
        self._event_file.write(json.dumps(record, separators=(",", ":"), ensure_ascii=False) + "\n")
        self._event_file.flush()

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist
        self.odom_x = _finite(p.x)
        self.odom_y = _finite(p.y)
        yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.odom_yaw_deg = math.degrees(yaw)
        self.odom_vx = _finite(t.linear.x)
        self.odom_vy = _finite(t.linear.y)
        if self.odom_vx is not None and self.odom_vy is not None:
            self.odom_speed = math.hypot(self.odom_vx, self.odom_vy)
        self.odom_yaw_rate = _finite(t.angular.z)

    def _gps_cb(self, msg: GPSRAW) -> None:
        self.gps_fix_type = int(msg.fix_type)
        self.gps_satellites = int(msg.satellites_visible)

    def _state_cb(self, msg: State) -> None:
        self.px4_connected = bool(msg.connected)
        self.px4_armed = bool(msg.armed)
        self.px4_mode = str(msg.mode)

    def _rpp_velocity_cb(self, msg: Vector3Stamped) -> None:
        self.rpp_vx = _finite(msg.vector.x)
        self.rpp_vy = _finite(msg.vector.y)
        self.rpp_vz = _finite(msg.vector.z)

    def _setpoint_cb(self, msg: PositionTarget) -> None:
        self.setpoint_vx = _finite(msg.velocity.x)
        self.setpoint_vy = _finite(msg.velocity.y)
        self.setpoint_vz = _finite(msg.velocity.z)
        self.setpoint_yaw_rate = _finite(msg.yaw_rate)

    def _rpp_accuracy_cb(self, msg: String) -> None:
        self.rpp_accuracy = _json_dict(msg.data)

    def _rpp_debug_cb(self, msg: String) -> None:
        self.rpp_debug = _json_dict(msg.data)

    def _mission_status_cb(self, msg: String) -> None:
        self.mission_status = _json_dict(msg.data)
        self._event("mission_manager/status", msg.data)

    def _spray_status_cb(self, msg: String) -> None:
        self.spray_status = _json_dict(msg.data)
        # Only write status transitions/high-value spray snapshots to JSONL by
        # preserving every raw message. rosbag remains the authoritative raw log.
        self._event("spray/status", msg.data)

    @staticmethod
    def _g(source: dict[str, Any], key: str, default: Any = None) -> Any:
        return source.get(key, default) if source else default

    def _sample(self) -> None:
        m = self.mission_status
        a = self.rpp_accuracy
        d = self.rpp_debug
        s = self.spray_status
        row = {
            "timestamp_unix_ns": time.time_ns(),
            "elapsed_sec": round(time.monotonic() - self._start_mono, 6),
            "odom_x_m": self.odom_x,
            "odom_y_m": self.odom_y,
            "odom_yaw_deg": self.odom_yaw_deg,
            "actual_vx_mps": self.odom_vx,
            "actual_vy_mps": self.odom_vy,
            "actual_speed_mps": self.odom_speed,
            "actual_yaw_rate_radps": self.odom_yaw_rate,
            "gps_fix_type": self.gps_fix_type,
            "gps_satellites": self.gps_satellites,
            "px4_connected": self.px4_connected,
            "px4_armed": self.px4_armed,
            "px4_mode": self.px4_mode,
            "mission_state": self._g(m, "state"),
            "execution_mode": self._g(m, "execution_mode"),
            "current_point_id": self._g(m, "current_point_id"),
            "current_point_index": self._g(m, "current_point_index"),
            "current_point_state": self._g(m, "current_point_state"),
            "pause_reason": self._g(m, "pause_reason"),
            "resume_available": self._g(m, "resume_available"),
            "mission_enable": self.mission_enable if self.mission_enable is not None else self._g(m, "mission_enable"),
            "emergency_stop": self.emergency_stop if self.emergency_stop is not None else self._g(m, "emergency_stop"),
            "backend_heartbeat_healthy": self.backend_heartbeat_healthy if self.backend_heartbeat_healthy is not None else self._g(m, "backend_heartbeat_healthy"),
            "rtk_state": self._g(m, "rtk_state"),
            "rtk_fixed": self._g(m, "rtk_fixed"),
            "rtk_healthy": self._g(m, "rtk_healthy"),
            "rtk_motion_ok": self._g(m, "rtk_motion_ok"),
            "rtk_reason": self._g(m, "rtk_reason"),
            "rtk_correction_age_sec": self._g(m, "rtk_correction_age_sec", self.rtk_correction_age_sec),
            "gps_fix_status_age_sec": self._g(m, "gps_fix_status_age_sec"),
            "rtk_health_status_age_sec": self._g(m, "rtk_health_status_age_sec"),
            "rtk_age_status_age_sec": self._g(m, "rtk_age_status_age_sec"),
            "arrival_settle_elapsed_sec": self._g(m, "arrival_settle_elapsed_sec"),
            "arrival_settle_required_sec": self._g(m, "arrival_settle_required_sec"),
            "marking_active": self._g(m, "marking_active"),
            "marking_radial_error_m": self._g(m, "marking_radial_error_m"),
            "marking_xtrack_m": self._g(m, "marking_xtrack_m"),
            "marking_along_error_m": self._g(m, "marking_along_error_m"),
            "marking_combined_error_m": self._g(m, "marking_combined_error_m"),
            "rpp_goal_number": self._g(a, "goal_number"),
            "rpp_cross_track_error_mm": self._g(a, "cross_track_error_mm"),
            "rpp_front_back_error_mm": self._g(a, "front_back_error_mm"),
            "rpp_radial_error_mm": self._g(a, "radial_error_mm"),
            "rpp_closest_radial_error_mm": self._g(a, "closest_radial_error_mm"),
            "rpp_accuracy_status": self._g(a, "accuracy_status"),
            "rpp_debug_telemetry_sequence": self._g(d, "telemetry_sequence"),
            "rpp_debug_control_sequence": self._g(d, "control_sequence"),
            "rpp_debug_control_sample_age_ms": self._g(d, "control_sample_age_ms"),
            "rpp_debug_odom_age_ms": self._g(d, "odom_age_ms"),
            "rpp_debug_control_dt_ms": self._g(d, "control_dt_ms"),
            "rpp_debug_control_compute_ms": self._g(d, "control_compute_ms"),
            "rpp_debug_deadline_missed": self._g(d, "control_deadline_missed"),
            "rpp_debug_mode": self._g(d, "control_mode"),
            "rpp_debug_reason": self._g(d, "reason"),
            "rpp_debug_actual_speed_mps": self._g(d, "actual_speed_mps"),
            "rpp_debug_command_speed_mps": self._g(d, "command_speed_mps"),
            "rpp_debug_heading_error_deg": self._g(d, "heading_error_deg"),
            "rpp_debug_cross_track_error_mm": self._g(d, "cross_track_error_mm"),
            "rpp_debug_along_remaining_mm": self._g(d, "along_remaining_mm"),
            "rpp_debug_distance_to_goal_m": self._g(d, "distance_to_goal_m"),
            "rpp_command_speed_mps": self.rpp_command_speed,
            "rpp_velocity_north_mps": self.rpp_vx,
            "rpp_velocity_east_mps": self.rpp_vy,
            "rpp_velocity_down_mps": self.rpp_vz,
            "rpp_accel_active": self.rpp_accel_active,
            "rpp_accel_progress_m": self.rpp_accel_progress_m,
            "rpp_decel_active": self.rpp_decel_active,
            "rpp_decel_progress_m": self.rpp_decel_progress_m,
            "rpp_decel_remaining_m": self.rpp_decel_remaining_m,
            "rpp_xtrack_mm_topic": self.rpp_xtrack_mm,
            "rpp_goal_distance_mm_topic": self.rpp_goal_distance_mm,
            "rpp_along_remaining_mm_topic": self.rpp_along_mm,
            "rpp_closest_goal_distance_mm_topic": self.rpp_closest_mm,
            "rpp_terminal_precision_armed": self.rpp_terminal_precision_armed,
            "rpp_terminal_bearing_frozen": self.rpp_terminal_bearing_frozen,
            "rpp_terminal_correction_deg": self.rpp_terminal_correction_deg,
            "rpp_xtrack_speed_cap_active": self.rpp_xtrack_speed_cap_active,
            "rpp_xtrack_speed_cap_mps": self.rpp_xtrack_speed_cap_mps,
            "setpoint_vx_mps": self.setpoint_vx,
            "setpoint_vy_mps": self.setpoint_vy,
            "setpoint_vz_mps": self.setpoint_vz,
            "setpoint_yaw_rate_radps": self.setpoint_yaw_rate,
            "spray_controller_state": self._g(s, "controller_state"),
            "spraying": self._g(s, "spraying"),
            "spray_duration_sec": self._g(s, "spray_duration_sec"),
            "spray_elapsed_sec": self._g(s, "spray_elapsed_sec"),
            "spray_remaining_sec": self._g(s, "spray_remaining_sec"),
            "spray_ready": self._g(s, "ready"),
            "spray_fault_latched": self._g(s, "fault_latched"),
            "spray_fault_reason": self._g(s, "fault_reason"),
            "rtk_bridge_healthy": self.rtk_bridge_healthy,
        }
        self._writer.writerow(row)
        if time.monotonic() - self._last_flush >= 1.0:
            self._csv_file.flush()
            self._event_file.flush()
            self._last_flush = time.monotonic()

    def close(self) -> None:
        try:
            self._csv_file.flush()
            self._csv_file.close()
        finally:
            self._event_file.flush()
            self._event_file.close()


def main() -> None:
    base = os.environ.get("ROVER_FIELD_LOG_DIR")
    if base:
        out_dir = Path(base).expanduser().resolve()
    else:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        out_dir = Path.home() / "rover_ws" / "field_logs" / f"field_{stamp}"

    rclpy.init(args=sys.argv)
    node = FieldTestLogger(out_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
