#!/usr/bin/env python3

"""Runtime C -> P1 -> P2 -> P3 -> P4 square-test supervisor.

This helper is intentionally test-only. It does not modify mission.csv.

After the production trajectory generator prepares the active mission, this
node takes the first four exact local marking coordinates and injects a
temporary mission-manager source path:

    fresh current rover position C
        -> straight interpolation <= 0.05 m
    exact P1 marking
        -> straight interpolation <= 0.05 m
    exact P2 marking

The original CSV coordinates remain the only marking points. All generated
points are navigation-only pass-through points.

The production mission manager owns the marking decision using:
- radial distance hypot(xtrack, along error) <= 0.030 m;
- stationary speed <= 0.010 m/s;
- continuous 3.00 second marking hold;
- P1-P4/P3/P4 COMPLETED point events.

This supervisor independently recomputes the same geometry from odometry.
It latches the safety topics and fails the test if a point is completed
outside the circular 30 mm waypoint radius or if the independent
3-second stationary hold is not satisfied.

The production backend still owns:
- /mission_enable;
- /emergency_stop;
- heartbeat gating;
- PX4 OFFBOARD/arm/start and safe stop.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import tempfile
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import (
    Bool,
    Int32MultiArray,
    String,
    UInt8MultiArray,
)


POINT_PASS_THROUGH = 0
POINT_MARKING = 2


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def finite(value: float) -> bool:
    return math.isfinite(float(value))


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=str(path.parent),
        prefix=f".{path.name}.",
        suffix=".tmp",
        delete=False,
    ) as handle:
        temporary = Path(handle.name)
        json.dump(
            payload,
            handle,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        handle.flush()
        os.fsync(handle.fileno())

    os.replace(temporary, path)


class CP1P2P3P4SquareTestSupervisor(Node):
    """Inject and verify the temporary four-marking runtime path."""

    TICK_SEC = 0.10
    SOURCE_TIMEOUT_SEC = 30.0
    MANAGER_LOAD_TIMEOUT_SEC = 20.0

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("c_p1_p2_p3_p4_square_test_supervisor")

        self.spacing_m = float(args.spacing_m)
        self.waypoint_radius_m = float(args.waypoint_radius_m)
        self.hold_required_sec = float(args.hold_required_sec)
        self.stationary_speed_mps = float(args.stationary_speed_mps)
        self.missed_point_abort_m = float(args.missed_point_abort_m)
        self.heading_tolerance_deg = float(args.heading_tolerance_deg)
        self.odom_timeout_sec = float(args.odom_timeout_sec)
        self.ready_file = Path(args.ready_file)
        self.result_file = Path(args.result_file)
        self.event_file = Path(args.event_file)

        self._validate_arguments()

        retained_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.ready_pub = self.create_publisher(
            Bool,
            "/trajectory_generator/ready",
            retained_qos,
        )
        self.nav_path_pub = self.create_publisher(
            NavPath,
            "/nav_path",
            retained_qos,
        )
        self.mission_waypoints_pub = self.create_publisher(
            NavPath,
            "/mission_waypoints",
            retained_qos,
        )
        self.path_types_pub = self.create_publisher(
            UInt8MultiArray,
            "/trajectory_generator/path_types",
            retained_qos,
        )
        self.marking_indices_pub = self.create_publisher(
            Int32MultiArray,
            "/trajectory_generator/marking_indices",
            retained_qos,
        )
        self.error_monitor_pub = self.create_publisher(
            String,
            "/test/marking_error_mm",
            command_qos,
        )
        self.estop_pub = self.create_publisher(
            Bool,
            "/emergency_stop",
            command_qos,
        )
        self.mission_enable_pub = self.create_publisher(
            Bool,
            "/mission_enable",
            command_qos,
        )

        self.create_subscription(
            NavPath,
            "/mission_waypoints",
            self._mission_waypoints_callback,
            retained_qos,
        )
        self.create_subscription(
            Bool,
            "/trajectory_generator/ready",
            self._trajectory_ready_callback,
            retained_qos,
        )
        self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self._odom_callback,
            odom_qos,
        )
        self.create_subscription(
            String,
            "/mission_manager/status",
            self._mission_status_callback,
            retained_qos,
        )
        self.create_subscription(
            String,
            "/mission_manager/point_event",
            self._point_event_callback,
            command_qos,
        )

        self.source_waypoints: list[tuple[float, float]] | None = None
        self.source_ready = False

        self.current_x: float | None = None
        self.current_y: float | None = None
        self.current_yaw: float | None = None
        self.current_speed_mps = math.inf
        self.last_odom_time = None
        self.last_monitor_publish_time = None
        self.safety_latched = False

        self.test_start: tuple[float, float] | None = None
        self.p1: tuple[float, float] | None = None
        self.p2: tuple[float, float] | None = None
        self.p3: tuple[float, float] | None = None
        self.p4: tuple[float, float] | None = None

        self.test_nav_path: NavPath | None = None
        self.test_waypoints: NavPath | None = None
        self.test_path_types: UInt8MultiArray | None = None
        self.test_marking_indices: Int32MultiArray | None = None
        self.expected_navigation_count = 0
        self.maximum_generated_spacing_m = 0.0

        self.phase = "WAIT_SOURCE"
        self.phase_counter = 0
        self.phase_started = self.get_clock().now()
        self.injected = False
        self.ready_written = False
        self.result_written = False

        self.manager_status: dict[str, Any] = {}
        self.test_running = False
        self.completed_points: set[int] = set()

        self.point_metrics: dict[int, dict[str, Any]] = {
            index: self._new_point_metrics()
            for index in range(4)
        }

        self.timer = self.create_timer(
            self.TICK_SEC,
            self._tick,
        )

        self._append_event(
            "SUPERVISOR_STARTED",
            {
                "spacing_m": self.spacing_m,
                "waypoint_radius_m": self.waypoint_radius_m,
                "hold_required_sec": self.hold_required_sec,
                "stationary_speed_mps": self.stationary_speed_mps,
                "heading_tolerance_deg": self.heading_tolerance_deg,
            },
        )

        self.get_logger().warn(
            "===== PRODUCTION C->P1->P2->P3->P4 SQUARE TEST SUPERVISOR STARTED ====="
        )
        self.get_logger().warn(
            "Waiting for prepared mission waypoints and fresh odometry"
        )

    def _validate_arguments(self) -> None:
        positive_values = {
            "spacing_m": self.spacing_m,
            "waypoint_radius_m": self.waypoint_radius_m,
            "hold_required_sec": self.hold_required_sec,
            "stationary_speed_mps": self.stationary_speed_mps,
            "missed_point_abort_m": self.missed_point_abort_m,
            "heading_tolerance_deg": self.heading_tolerance_deg,
            "odom_timeout_sec": self.odom_timeout_sec,
        }
        for name, value in positive_values.items():
            if not finite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")

        if self.spacing_m > 0.05 + 1.0e-9:
            raise ValueError("spacing_m must be <= 0.05")
        if abs(self.waypoint_radius_m - 0.03) > 1.0e-9:
            raise ValueError(
                "waypoint_radius_m must be exactly 0.03"
            )
        if self.stationary_speed_mps > 0.01 + 1.0e-9:
            raise ValueError(
                "stationary_speed_mps must be <= 0.01"
            )
        if self.hold_required_sec < 3.0:
            raise ValueError(
                "hold_required_sec must be >= 3.0"
            )

    @staticmethod
    def _new_point_metrics() -> dict[str, Any]:
        return {
            "hold_sample_count": 0,
            "maximum_hold_abs_xtrack_m": 0.0,
            "maximum_hold_abs_along_error_m": 0.0,
            "maximum_hold_combined_error_m": 0.0,
            "maximum_hold_radial_error_m": 0.0,
            "maximum_hold_heading_error_deg": 0.0,
            "maximum_hold_speed_mps": 0.0,
            "maximum_reported_hold_elapsed_sec": 0.0,
            "independent_valid_hold_started_ns": None,
            "maximum_independent_valid_hold_sec": 0.0,
            "completion_xtrack_m": None,
            "completion_along_error_m": None,
            "completion_combined_error_m": None,
            "completion_radial_error_m": None,
            "completion_heading_error_deg": None,
            "completion_speed_mps": None,
            "completed": False,
            "passed_position_hold": False,
            "passed_heading": False,
        }

    def _append_event(
        self,
        event: str,
        details: dict[str, Any],
    ) -> None:
        self.event_file.parent.mkdir(
            parents=True,
            exist_ok=True,
        )
        payload = {
            "event": event,
            "time_ns": self.get_clock().now().nanoseconds,
            **details,
        }
        with self.event_file.open(
            "a",
            encoding="utf-8",
        ) as handle:
            handle.write(
                json.dumps(
                    payload,
                    ensure_ascii=False,
                    sort_keys=True,
                )
            )
            handle.write("\n")

    @staticmethod
    def _yaw_from_quaternion(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> float:
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(sin_yaw, cos_yaw)

    def _odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        linear = message.twist.twist.linear

        values = (
            float(position.x),
            float(position.y),
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
            float(linear.x),
            float(linear.y),
        )
        if not all(finite(value) for value in values):
            return

        self.current_x = values[0]
        self.current_y = values[1]
        self.current_yaw = self._yaw_from_quaternion(
            *values[2:6]
        )
        self.current_speed_mps = math.hypot(
            values[6],
            values[7],
        )
        self.last_odom_time = self.get_clock().now()

        self._sample_active_hold()
        self._publish_error_monitor()
        self._evaluate_invalid_marking_stop()

    def _trajectory_ready_callback(self, message: Bool) -> None:
        if self.injected:
            return
        self.source_ready = bool(message.data)

    def _mission_waypoints_callback(
        self,
        message: NavPath,
    ) -> None:
        if self.injected or self.source_waypoints is not None:
            return
        if message.header.frame_id.strip() != "map":
            return
        if len(message.poses) < 4:
            return

        points: list[tuple[float, float]] = []
        for pose in message.poses:
            x = float(pose.pose.position.x)
            y = float(pose.pose.position.y)
            if not all(finite(value) for value in (x, y)):
                return
            points.append((x, y))

        self.source_waypoints = points
        self.get_logger().warn(
            f"Prepared source received: {len(points)} marking points"
        )

    def _mission_status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return

        self.manager_status = payload
        state = str(payload.get("state", "")).upper()

        if state == "RUNNING":
            self.test_running = True

        if state == "ERROR" and not self.result_written:
            error_message = str(
                payload.get("error")
                or payload.get("message")
                or "Mission manager entered ERROR"
            )

            transient_snapshot_error = (
                "Prepared path, path types and marking indices "
                "have different lengths"
                in error_message
            )

            test_preparation_active = (
                self.phase
                in {
                    "READY_FALSE",
                    "PUBLISH_DATA",
                    "READY_TRUE",
                    "WAIT_MANAGER",
                }
                and not self.test_running
            )

            if (
                transient_snapshot_error
                and test_preparation_active
            ):
                self.get_logger().warn(
                    "Ignoring transient mission-manager snapshot "
                    f"ERROR during {self.phase}; waiting for the "
                    "coherent four-point test mission: "
                    f"{error_message}"
                )
                return

            self._write_result(
                result="MANAGER_ERROR",
                message=error_message,
            )
            return

        self._sample_active_hold()
        self._publish_error_monitor()
        self._evaluate_invalid_marking_stop()

        if (
            self.phase == "WAIT_MANAGER"
            and state == "READY"
            and bool(payload.get("path_ready", False))
            and int(payload.get("total_points", 0)) == 4
            and int(
                payload.get("navigation_point_count", 0)
            ) == self.expected_navigation_count
        ):
            self._write_ready_file()
            self.phase = "MONITOR"
            self.phase_started = self.get_clock().now()

        if (
            state == "COMPLETED"
            and not self.result_written
            and set(self.completed_points) == {0, 1, 2, 3}
        ):
            self._finish_from_completed_points()

    def _point_event_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return

        event = str(payload.get("event", "")).upper()
        point_index = payload.get("point_index")

        if event != "COMPLETED":
            return
        if point_index not in (0, 1, 2, 3):
            return

        point_index = int(point_index)
        self.completed_points.add(point_index)

        self._record_completion_metrics(point_index)

        self._append_event(
            "POINT_COMPLETED",
            {
                "point_index": point_index,
                "point_id": payload.get("point_id"),
                "metrics": self.point_metrics[point_index],
            },
        )

        self.get_logger().warn(
            f"P{point_index + 1} COMPLETED by production mission manager"
        )

        if point_index == 3 and not self.result_written:
            self._finish_from_completed_points()

    def _odometry_is_fresh(self) -> bool:
        if (
            self.current_x is None
            or self.current_y is None
            or self.current_yaw is None
            or self.last_odom_time is None
        ):
            return False

        age = (
            self.get_clock().now() - self.last_odom_time
        ).nanoseconds / 1e9
        return age <= self.odom_timeout_sec

    @staticmethod
    def _orientation_for_bearing(
        pose: PoseStamped,
        bearing: float,
    ) -> None:
        half = 0.5 * bearing
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(half)
        pose.pose.orientation.w = math.cos(half)

    def _append_segment(
        self,
        *,
        path: NavPath,
        path_types: list[int],
        marking_indices: list[int],
        start: tuple[float, float],
        end: tuple[float, float],
        final_marking_index: int,
    ) -> None:
        delta_x = end[0] - start[0]
        delta_y = end[1] - start[1]
        distance = math.hypot(delta_x, delta_y)

        if not finite(distance) or distance <= 1.0e-6:
            raise ValueError("Test segment is zero-length")

        divisions = max(
            1,
            int(math.ceil(distance / self.spacing_m)),
        )
        actual_spacing = distance / divisions
        self.maximum_generated_spacing_m = max(
            self.maximum_generated_spacing_m,
            actual_spacing,
        )
        bearing = math.atan2(delta_y, delta_x)

        for division in range(1, divisions + 1):
            ratio = division / divisions
            is_final = division == divisions

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = start[0] + ratio * delta_x
            pose.pose.position.y = start[1] + ratio * delta_y
            pose.pose.position.z = 0.0
            self._orientation_for_bearing(pose, bearing)
            path.poses.append(pose)

            if is_final:
                path_types.append(POINT_MARKING)
                marking_indices.append(final_marking_index)
            else:
                path_types.append(POINT_PASS_THROUGH)
                marking_indices.append(-1)

    def _build_test_path(self) -> None:
        assert self.source_waypoints is not None
        assert self.current_x is not None
        assert self.current_y is not None

        if len(self.source_waypoints) < 4:
            raise ValueError("Prepared mission has fewer than four marking points")

        self.test_start = (
            float(self.current_x),
            float(self.current_y),
        )
        self.p1, self.p2, self.p3, self.p4 = self.source_waypoints[:4]
        points = [self.p1, self.p2, self.p3, self.p4]

        if math.hypot(
            self.p1[0] - self.test_start[0],
            self.p1[1] - self.test_start[1],
        ) <= self.waypoint_radius_m:
            raise ValueError(
                "Current rover position is already inside P1 tolerance"
            )

        path = NavPath()
        path.header.frame_id = "map"
        path_types: list[int] = []
        marking_indices: list[int] = []

        segment_starts = [self.test_start, self.p1, self.p2, self.p3]
        for point_index, (start, end) in enumerate(
            zip(segment_starts, points)
        ):
            self._append_segment(
                path=path,
                path_types=path_types,
                marking_indices=marking_indices,
                start=start,
                end=end,
                final_marking_index=point_index,
            )

        waypoints = NavPath()
        waypoints.header.frame_id = "map"
        incoming_bearings = [
            math.atan2(end[1] - start[1], end[0] - start[0])
            for start, end in zip(segment_starts, points)
        ]

        for point, bearing in zip(points, incoming_bearings):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            self._orientation_for_bearing(pose, bearing)
            waypoints.poses.append(pose)

        types_message = UInt8MultiArray()
        types_message.data = path_types
        indices_message = Int32MultiArray()
        indices_message.data = marking_indices

        self.test_nav_path = path
        self.test_waypoints = waypoints
        self.test_path_types = types_message
        self.test_marking_indices = indices_message
        self.expected_navigation_count = len(path.poses)
        self.injected = True

        c_to_p1 = math.hypot(
            self.p1[0] - self.test_start[0],
            self.p1[1] - self.test_start[1],
        )
        perimeter_lengths = [
            math.hypot(
                points[(i + 1) % 4][0] - points[i][0],
                points[(i + 1) % 4][1] - points[i][1],
            )
            for i in range(4)
        ]
        path_lengths = [c_to_p1, *perimeter_lengths[:3]]

        corner_angles: list[float] = []
        for i in range(4):
            previous = points[(i - 1) % 4]
            current = points[i]
            following = points[(i + 1) % 4]
            a = (previous[0] - current[0], previous[1] - current[1])
            b = (following[0] - current[0], following[1] - current[1])
            denominator = math.hypot(*a) * math.hypot(*b)
            cosine = max(-1.0, min(1.0, (a[0] * b[0] + a[1] * b[1]) / denominator))
            corner_angles.append(math.degrees(math.acos(cosine)))

        side_ratio = max(perimeter_lengths) / min(perimeter_lengths)
        geometry = {
            "side_lengths_m": perimeter_lengths,
            "corner_angles_deg": corner_angles,
            "side_ratio_max_min": side_ratio,
            "approximately_right_angled": all(
                abs(angle - 90.0) <= 8.0 for angle in corner_angles
            ),
            "approximately_equal_sided": side_ratio <= 1.10,
        }

        self._append_event(
            "TEST_PATH_BUILT",
            {
                "current_c": self.test_start,
                "points": points,
                "path_segment_lengths_m": path_lengths,
                "square_geometry": geometry,
                "navigation_point_count": self.expected_navigation_count,
                "maximum_spacing_m": self.maximum_generated_spacing_m,
            },
        )

        self.get_logger().warn("===== TEMPORARY FOUR-POINT TEST PATH BUILT =====")
        self.get_logger().warn(f"C->P1 distance       : {c_to_p1:.3f} m")
        for index, length in enumerate(perimeter_lengths, start=1):
            next_index = 1 if index == 4 else index + 1
            self.get_logger().warn(
                f"P{index}->P{next_index} side       : {length:.3f} m"
            )
        self.get_logger().warn(
            "Corner angles        : "
            + ", ".join(f"{angle:.1f}deg" for angle in corner_angles)
        )
        self.get_logger().warn(
            f"Side max/min ratio  : {side_ratio:.3f}"
        )
        if not geometry["approximately_equal_sided"]:
            self.get_logger().warn(
                "MISSION GEOMETRY NOTE: four corners are near right angles, "
                "but side lengths are not equal; this is rectangle-like, not an exact square"
            )
        self.get_logger().warn(
            f"Navigation points    : {self.expected_navigation_count}"
        )
        self.get_logger().warn(
            f"Maximum spacing      : {self.maximum_generated_spacing_m:.4f} m"
        )

    def _publish_ready(self, value: bool) -> None:
        message = Bool()
        message.data = bool(value)
        self.ready_pub.publish(message)

    def _publish_test_data(self) -> None:
        assert self.test_nav_path is not None
        assert self.test_waypoints is not None
        assert self.test_path_types is not None
        assert self.test_marking_indices is not None

        now = self.get_clock().now().to_msg()
        self.test_nav_path.header.stamp = now
        self.test_waypoints.header.stamp = now

        for pose in self.test_nav_path.poses:
            pose.header.stamp = now
        for pose in self.test_waypoints.poses:
            pose.header.stamp = now

        self.nav_path_pub.publish(self.test_nav_path)
        self.mission_waypoints_pub.publish(
            self.test_waypoints
        )
        self.path_types_pub.publish(
            self.test_path_types
        )
        self.marking_indices_pub.publish(
            self.test_marking_indices
        )

    def _write_ready_file(self) -> None:
        if self.ready_written:
            return
        assert self.test_start is not None
        points = [self.p1, self.p2, self.p3, self.p4]
        if any(point is None for point in points):
            return
        typed_points = [point for point in points if point is not None]
        segment_starts = [self.test_start, *typed_points[:-1]]
        segment_lengths = [
            math.hypot(end[0] - start[0], end[1] - start[1])
            for start, end in zip(segment_starts, typed_points)
        ]

        payload = {
            "ready": True,
            "current_c": {"x": self.test_start[0], "y": self.test_start[1]},
            "points": [
                {"id": f"P{index + 1}", "x": point[0], "y": point[1]}
                for index, point in enumerate(typed_points)
            ],
            "test_segment_lengths_m": segment_lengths,
            "navigation_point_count": self.expected_navigation_count,
            "maximum_generated_spacing_m": self.maximum_generated_spacing_m,
            "waypoint_radius_m": self.waypoint_radius_m,
            "hold_required_sec": self.hold_required_sec,
            "stationary_speed_mps": self.stationary_speed_mps,
            "heading_tolerance_deg": self.heading_tolerance_deg,
        }
        atomic_write_json(self.ready_file, payload)
        self.ready_written = True
        self._append_event("MANAGER_READY_WITH_TEST_PATH", payload)
        self.get_logger().warn(
            "Mission manager confirmed READY with the temporary "
            "C->P1->P2->P3->P4 path"
        )

    def _incoming_bearing(self, point_index: int) -> float | None:
        if self.test_start is None:
            return None
        points = [self.p1, self.p2, self.p3, self.p4]
        if not 0 <= point_index < 4 or points[point_index] is None:
            return None
        end = points[point_index]
        start = self.test_start if point_index == 0 else points[point_index - 1]
        if start is None or end is None:
            return None
        return math.atan2(end[1] - start[1], end[0] - start[0])

    def _point_coordinate(
        self,
        point_index: int,
    ) -> tuple[float, float] | None:
        points = [self.p1, self.p2, self.p3, self.p4]
        if 0 <= point_index < len(points):
            return points[point_index]
        return None

    def _current_point_measurement(
        self,
        point_index: int,
    ) -> dict[str, float] | None:
        if not self._odometry_is_fresh():
            return None

        coordinate = self._point_coordinate(point_index)
        incoming_bearing = self._incoming_bearing(point_index)

        if coordinate is None or incoming_bearing is None:
            return None
        assert self.current_x is not None
        assert self.current_y is not None
        assert self.current_yaw is not None

        unit_x = math.cos(incoming_bearing)
        unit_y = math.sin(incoming_bearing)

        target_to_rover_x = self.current_x - coordinate[0]
        target_to_rover_y = self.current_y - coordinate[1]

        xtrack_m = (
            unit_x * target_to_rover_y
            - unit_y * target_to_rover_x
        )
        along_error_m = -(
            unit_x * target_to_rover_x
            + unit_y * target_to_rover_y
        )
        combined_error_m = abs(xtrack_m) + abs(along_error_m)
        radial_error_m = math.hypot(
            target_to_rover_x,
            target_to_rover_y,
        )

        return {
            "xtrack_m": xtrack_m,
            "along_error_m": along_error_m,
            "combined_error_m": combined_error_m,
            "radial_error_m": radial_error_m,
            "heading_error_deg": abs(
                math.degrees(
                    wrap_pi(
                        incoming_bearing - self.current_yaw
                    )
                )
            ),
            "speed_mps": float(self.current_speed_mps),
        }

    def _active_point_index(self) -> int | None:
        point_index = self.manager_status.get("current_point_index")
        if point_index in (0, 1, 2, 3):
            return int(point_index)
        return None

    def _publish_error_monitor(self) -> None:
        if not self.test_running or self.result_written:
            return

        point_index = self._active_point_index()
        if point_index is None:
            return

        measurement = self._current_point_measurement(point_index)
        if measurement is None:
            return

        now = self.get_clock().now()
        if self.last_monitor_publish_time is not None:
            age = (now - self.last_monitor_publish_time).nanoseconds / 1e9
            if age < 0.20:
                return
        self.last_monitor_publish_time = now

        valid = (
            measurement["radial_error_m"]
            <= self.waypoint_radius_m
            and measurement["speed_mps"]
            <= self.stationary_speed_mps
        )
        payload = {
            "point": f"P{point_index + 1}",
            "xtrack_mm": round(measurement["xtrack_m"] * 1000.0, 1),
            "along_error_mm": round(
                measurement["along_error_m"] * 1000.0, 1
            ),
            "combined_mm": round(
                measurement["combined_error_m"] * 1000.0, 1
            ),
            "radial_mm": round(
                measurement["radial_error_m"] * 1000.0, 1
            ),
            "speed_mmps": round(
                measurement["speed_mps"] * 1000.0, 1
            ),
            "radius_limit_mm": round(
                self.waypoint_radius_m * 1000.0, 1
            ),
            "valid_for_hold": valid,
            "manager_marking_active": bool(
                self.manager_status.get("marking_active", False)
            ),
            "manager_hold_sec": round(
                float(
                    self.manager_status.get("hold_elapsed_sec", 0.0)
                    or 0.0
                ),
                2,
            ),
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self.error_monitor_pub.publish(message)

        self.get_logger().info(
            "MARKING ERROR MM | "
            f"point=P{point_index + 1} | "
            f"xtrack={payload['xtrack_mm']:+.1f} | "
            f"along={payload['along_error_mm']:+.1f} | "
            f"combined_info={payload['combined_mm']:.1f} | "
            f"radial={payload['radial_mm']:.1f}/"
            f"{payload['radius_limit_mm']:.1f} | "
            f"speed={payload['speed_mmps']:.1f}mm/s | "
            f"valid={'YES' if valid else 'NO'}"
        )

    def _latch_safety(self) -> None:
        self.safety_latched = True
        estop = Bool()
        estop.data = True
        disable = Bool()
        disable.data = False
        for _ in range(5):
            self.estop_pub.publish(estop)
            self.mission_enable_pub.publish(disable)

    def _safe_fail(
        self,
        *,
        result: str,
        message: str,
        point_index: int | None = None,
        measurement: dict[str, float] | None = None,
    ) -> None:
        if self.result_written:
            return
        self._latch_safety()
        details: dict[str, Any] = {}
        if point_index is not None:
            details["point_index"] = point_index
            details["point_id"] = f"P{point_index + 1}"
        if measurement is not None:
            details["measurement"] = measurement
        self._append_event("SAFETY_STOP", {"reason": result, **details})
        self._write_result(result=result, message=message)

    def _evaluate_invalid_marking_stop(self) -> None:
        """Fail safely after passing a marking point outside its 30 mm circle."""
        if not self.test_running or self.result_written:
            return

        point_index = self._active_point_index()
        if point_index is None:
            return

        measurement = self._current_point_measurement(point_index)
        if measurement is None:
            return

        if (
            measurement["along_error_m"] < -self.missed_point_abort_m
            and measurement["radial_error_m"] > self.waypoint_radius_m
        ):
            self._safe_fail(
                result="MISSED_MARKING_POINT",
                message=(
                    f"P{point_index + 1} passed by more than "
                    f"{self.missed_point_abort_m * 1000.0:.0f} mm "
                    "without completing the circular 30 mm waypoint hold"
                ),
                point_index=point_index,
                measurement=measurement,
            )

    def _sample_active_hold(self) -> None:
        payload = self.manager_status
        if not payload:
            return

        point_index = self._active_point_index()
        if point_index is None:
            return

        measurement = self._current_point_measurement(point_index)
        if measurement is None:
            return

        metrics = self.point_metrics[point_index]

        if not bool(payload.get("marking_active", False)):
            metrics["independent_valid_hold_started_ns"] = None
            return

        valid_sample = (
            measurement["radial_error_m"]
            <= self.waypoint_radius_m
            and measurement["speed_mps"]
            <= self.stationary_speed_mps
        )

        now_ns = self.get_clock().now().nanoseconds
        started_ns = metrics["independent_valid_hold_started_ns"]
        if valid_sample:
            if started_ns is None:
                metrics["independent_valid_hold_started_ns"] = now_ns
                started_ns = now_ns
            elapsed = (now_ns - int(started_ns)) / 1e9
            metrics["maximum_independent_valid_hold_sec"] = max(
                float(metrics["maximum_independent_valid_hold_sec"]),
                elapsed,
            )
        else:
            metrics["independent_valid_hold_started_ns"] = None

        metrics["hold_sample_count"] += 1
        metrics["maximum_hold_abs_xtrack_m"] = max(
            float(metrics["maximum_hold_abs_xtrack_m"]),
            abs(measurement["xtrack_m"]),
        )
        metrics["maximum_hold_abs_along_error_m"] = max(
            float(metrics["maximum_hold_abs_along_error_m"]),
            abs(measurement["along_error_m"]),
        )
        metrics["maximum_hold_combined_error_m"] = max(
            float(metrics["maximum_hold_combined_error_m"]),
            measurement["combined_error_m"],
        )
        metrics["maximum_hold_radial_error_m"] = max(
            float(metrics["maximum_hold_radial_error_m"]),
            measurement["radial_error_m"],
        )
        metrics["maximum_hold_heading_error_deg"] = max(
            float(metrics["maximum_hold_heading_error_deg"]),
            measurement["heading_error_deg"],
        )
        metrics["maximum_hold_speed_mps"] = max(
            float(metrics["maximum_hold_speed_mps"]),
            measurement["speed_mps"],
        )
        metrics["maximum_reported_hold_elapsed_sec"] = max(
            float(metrics["maximum_reported_hold_elapsed_sec"]),
            float(payload.get("hold_elapsed_sec", 0.0) or 0.0),
        )

    def _record_completion_metrics(
        self,
        point_index: int,
    ) -> None:
        measurement = self._current_point_measurement(point_index)
        metrics = self.point_metrics[point_index]

        if measurement is not None:
            metrics["completion_xtrack_m"] = measurement["xtrack_m"]
            metrics["completion_along_error_m"] = (
                measurement["along_error_m"]
            )
            metrics["completion_combined_error_m"] = (
                measurement["combined_error_m"]
            )
            metrics["completion_radial_error_m"] = (
                measurement["radial_error_m"]
            )
            metrics["completion_heading_error_deg"] = (
                measurement["heading_error_deg"]
            )
            metrics["completion_speed_mps"] = measurement["speed_mps"]

        metrics["completed"] = True

        completion_radial = float(
            metrics["completion_radial_error_m"]
            if metrics["completion_radial_error_m"] is not None
            else math.inf
        )
        completion_speed = float(
            metrics["completion_speed_mps"]
            if metrics["completion_speed_mps"] is not None
            else math.inf
        )
        independent_hold = float(
            metrics["maximum_independent_valid_hold_sec"]
        )
        completion_heading_error = float(
            metrics["completion_heading_error_deg"]
            if metrics["completion_heading_error_deg"] is not None
            else math.inf
        )

        metrics["passed_position_hold"] = (
            completion_radial
            <= self.waypoint_radius_m + 1.0e-6
            and completion_speed
            <= self.stationary_speed_mps + 1.0e-6
            and independent_hold
            >= self.hold_required_sec - 0.25
        )
        metrics["passed_heading"] = (
            completion_heading_error <= self.heading_tolerance_deg
        )

        if not metrics["passed_position_hold"]:
            self._safe_fail(
                result="INVALID_MARKING_COMPLETION",
                message=(
                    f"P{point_index + 1} completion rejected: "
                    f"radius={completion_radial * 1000.0:.1f} mm, "
                    f"speed={completion_speed * 1000.0:.1f} mm/s, "
                    f"independent_hold={independent_hold:.2f} s"
                ),
                point_index=point_index,
                measurement=measurement,
            )

    def _finish_from_completed_points(self) -> None:
        if self.result_written:
            return

        metrics = [self.point_metrics[index] for index in range(4)]
        all_completed = all(bool(item["completed"]) for item in metrics)
        position_hold_pass = all(
            bool(item["passed_position_hold"]) for item in metrics
        )
        heading_pass = all(bool(item["passed_heading"]) for item in metrics)

        if all_completed and position_hold_pass and heading_pass:
            result = "PASS"
            message = (
                "P1-P4 completed the circular 30 mm waypoint, "
                "3-second hold and heading checks"
            )
        elif all_completed and position_hold_pass:
            result = "FAIL_HEADING_ALIGNMENT"
            message = (
                "P1-P4 marking holds completed, but at least one "
                "master-antenna heading error exceeded the tolerance"
            )
        else:
            result = "FAIL_MARKING_VERIFICATION"
            message = (
                "P1-P4 completion or continuous circular position-hold "
                "verification failed"
            )

        self._write_result(result=result, message=message)

    def _write_result(
        self,
        *,
        result: str,
        message: str,
    ) -> None:
        if self.result_written:
            return

        payload = {
            "result": result,
            "message": message,
            "completed_points": sorted(self.completed_points),
            "points": {
                f"p{index + 1}": self.point_metrics[index]
                for index in range(4)
            },
            "manager_status": self.manager_status,
            "expected_navigation_count": self.expected_navigation_count,
            "maximum_generated_spacing_m": self.maximum_generated_spacing_m,
            "waypoint_radius_m": self.waypoint_radius_m,
            "hold_required_sec": self.hold_required_sec,
            "stationary_speed_mps": self.stationary_speed_mps,
            "heading_tolerance_deg": self.heading_tolerance_deg,
        }
        atomic_write_json(self.result_file, payload)
        self.result_written = True
        self._append_event("TEST_RESULT", payload)

        if result == "PASS":
            self.get_logger().warn(
                "===== C->P1->P2->P3->P4 SQUARE TEST PASSED ====="
            )
        else:
            self.get_logger().error(
                f"===== FOUR-POINT SQUARE TEST RESULT: {result} ====="
            )

    def _phase_age_sec(self) -> float:
        return (
            self.get_clock().now() - self.phase_started
        ).nanoseconds / 1e9

    def _set_phase(self, phase: str) -> None:
        self.phase = phase
        self.phase_counter = 0
        self.phase_started = self.get_clock().now()

    def _tick(self) -> None:
        if self.result_written:
            return

        if self.phase == "WAIT_SOURCE":
            if self._phase_age_sec() > self.SOURCE_TIMEOUT_SEC:
                self._write_result(
                    result="SOURCE_TIMEOUT",
                    message=(
                        "Timed out waiting for prepared first four "
                        "marking points and fresh odometry"
                    ),
                )
                return

            if (
                self.source_ready
                and self.source_waypoints is not None
                and len(self.source_waypoints) >= 4
                and self._odometry_is_fresh()
            ):
                try:
                    self._build_test_path()
                except ValueError as error:
                    self._write_result(
                        result="PATH_BUILD_ERROR",
                        message=str(error),
                    )
                    return
                self._set_phase("READY_FALSE")
            return

        if self.phase == "READY_FALSE":
            self._publish_ready(False)
            self.phase_counter += 1
            if self.phase_counter >= 5:
                self._set_phase("PUBLISH_DATA")
            return

        if self.phase == "PUBLISH_DATA":
            self._publish_ready(False)
            self._publish_test_data()
            self.phase_counter += 1
            if self.phase_counter >= 8:
                self._set_phase("READY_TRUE")
            return

        if self.phase == "READY_TRUE":
            self._publish_test_data()
            self._publish_ready(True)
            self.phase_counter += 1
            if self.phase_counter >= 8:
                self._set_phase("WAIT_MANAGER")
            return

        if self.phase == "WAIT_MANAGER":
            if self._phase_age_sec() > self.MANAGER_LOAD_TIMEOUT_SEC:
                self._write_result(
                    result="MANAGER_LOAD_TIMEOUT",
                    message=(
                        "Mission manager did not confirm the temporary "
                        "four-point path"
                    ),
                )
            return

        # MONITOR is callback-driven.


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Inject and verify a temporary production C->P1->P2->P3->P4 "
            "50 mm test path"
        )
    )
    parser.add_argument(
        "--spacing-m",
        type=float,
        default=0.05,
    )
    parser.add_argument(
        "--waypoint-radius-m",
        type=float,
        default=0.03,
    )
    parser.add_argument(
        "--hold-required-sec",
        type=float,
        default=3.0,
    )
    parser.add_argument(
        "--stationary-speed-mps",
        type=float,
        default=0.01,
    )
    parser.add_argument(
        "--missed-point-abort-m",
        type=float,
        default=0.10,
    )
    parser.add_argument(
        "--heading-tolerance-deg",
        type=float,
        default=4.0,
    )
    parser.add_argument(
        "--odom-timeout-sec",
        type=float,
        default=0.50,
    )
    parser.add_argument(
        "--ready-file",
        required=True,
    )
    parser.add_argument(
        "--result-file",
        required=True,
    )
    parser.add_argument(
        "--event-file",
        required=True,
    )
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()

    for path_value in (
        args.ready_file,
        args.result_file,
        args.event_file,
    ):
        path = Path(path_value)
        if path.exists():
            path.unlink()

    rclpy.init()
    node = CP1P2P3P4SquareTestSupervisor(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()