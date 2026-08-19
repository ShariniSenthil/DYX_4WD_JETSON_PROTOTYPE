#!/usr/bin/env python3

import hashlib
import math
import struct

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool


class MissionManager(Node):
    """
    Sequence the interpolated mission while preserving the original P1-P4
    points as marking targets.

    The dense interpolation points are path-shaping points, not stop points.
    For straight-segment tracking the active pass-through target is kept a
    configurable distance ahead of the rover. This prevents the controller
    from steering toward a point that has already moved behind the vehicle.

    Original marking points retain exact capture semantics:
        - enter the configured 0.05 m radius;
        - publish /marking_active=true;
        - hold for the full 3 second dwell;
        - then select a target about 0.75 m into the next segment.
    """

    CONTROL_HZ = 20.0
    PASS_MARGIN_M = 0.03

    def __init__(self):
        super().__init__("mission_manager")

        self.declare_parameter("local_frame", "map")
        self.declare_parameter("expected_marking_waypoints", 4)
        self.declare_parameter("navigation_tolerance_m", 0.05)
        self.declare_parameter("intermediate_switch_distance_m", 0.10)
        self.declare_parameter("pass_through_lookahead_distance_m", 0.60)
        self.declare_parameter("marking_lookahead_distance_m", 0.75)
        self.declare_parameter("post_marking_alignment_distance_m", 0.75)
        self.declare_parameter("marking_dwell_sec", 3.0)
        self.declare_parameter("waypoint_match_tolerance_m", 0.001)
        self.declare_parameter("odom_timeout_sec", 0.50)
        self.declare_parameter("max_path_points", 10000)

        self.local_frame = str(
            self.get_parameter("local_frame").value
        ).strip()
        self.expected_marking_waypoints = int(
            self.get_parameter("expected_marking_waypoints").value
        )
        self.navigation_tolerance = float(
            self.get_parameter("navigation_tolerance_m").value
        )
        self.intermediate_switch_distance = float(
            self.get_parameter("intermediate_switch_distance_m").value
        )
        self.pass_through_lookahead_distance = float(
            self.get_parameter("pass_through_lookahead_distance_m").value
        )
        self.marking_lookahead_distance = float(
            self.get_parameter("marking_lookahead_distance_m").value
        )
        self.post_marking_alignment_distance = float(
            self.get_parameter("post_marking_alignment_distance_m").value
        )
        self.marking_dwell_sec = float(
            self.get_parameter("marking_dwell_sec").value
        )
        self.waypoint_match_tolerance = float(
            self.get_parameter("waypoint_match_tolerance_m").value
        )
        self.odom_timeout_sec = float(
            self.get_parameter("odom_timeout_sec").value
        )
        self.max_path_points = int(
            self.get_parameter("max_path_points").value
        )

        self.validate_parameters()

        self.pending_path = None
        self.pending_marking_waypoints = None

        self.path_points = []
        self.marking_flags = []
        self.marking_numbers = []
        self.path_signature = None

        self.current_path_index = 0
        self.mission_complete = False
        self.dwell_start_time = None

        self.current_x = None
        self.current_y = None
        self.last_odom_time = None

        self.mission_enabled = False
        self.emergency_stop = True

        now = self.get_clock().now()
        self.last_wait_log_time = now
        self.last_status_log_time = now
        self.last_reject_log_time = now

        retained_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Path,
            "/nav_path",
            self.path_callback,
            retained_qos,
        )
        self.create_subscription(
            Path,
            "/mission_waypoints",
            self.marking_waypoints_callback,
            retained_qos,
        )
        self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self.odom_callback,
            odom_qos,
        )
        self.create_subscription(
            Bool,
            "/mission_enable",
            self.mission_enable_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/emergency_stop",
            self.emergency_stop_callback,
            10,
        )

        self.active_waypoint_pub = self.create_publisher(
            PoseStamped,
            "/active_waypoint",
            10,
        )
        self.marking_active_pub = self.create_publisher(
            Bool,
            "/marking_active",
            10,
        )
        self.mission_complete_pub = self.create_publisher(
            Bool,
            "/mission_complete",
            retained_qos,
        )

        self.publish_marking_active(False)
        self.publish_mission_complete(False)

        self.timer = self.create_timer(
            1.0 / self.CONTROL_HZ,
            self.control_loop,
        )

        self.get_logger().warn(
            "===== LOOKAHEAD MARKING MISSION MANAGER STARTED ====="
        )
        self.get_logger().warn(
            f"Marking tolerance : {self.navigation_tolerance:.3f} m"
        )
        self.get_logger().warn(
            f"Marking dwell     : {self.marking_dwell_sec:.1f} s"
        )
        self.get_logger().warn(
            "Pass-through lookahead: "
            f"{self.pass_through_lookahead_distance:.2f} m"
        )
        self.get_logger().warn(
            "Marking activation: "
            f"{self.marking_lookahead_distance:.2f} m"
        )
        self.get_logger().warn(
            "Post-marking target: "
            f"{self.post_marking_alignment_distance:.2f} m"
        )

    def validate_parameters(self):
        if not self.local_frame:
            raise ValueError("local_frame must not be empty")
        if self.expected_marking_waypoints < 2:
            raise ValueError("expected_marking_waypoints must be >= 2")
        if self.max_path_points < self.expected_marking_waypoints:
            raise ValueError("max_path_points is too small")

        positive_values = {
            "navigation_tolerance_m": self.navigation_tolerance,
            "intermediate_switch_distance_m": (
                self.intermediate_switch_distance
            ),
            "pass_through_lookahead_distance_m": (
                self.pass_through_lookahead_distance
            ),
            "marking_lookahead_distance_m": (
                self.marking_lookahead_distance
            ),
            "post_marking_alignment_distance_m": (
                self.post_marking_alignment_distance
            ),
            "marking_dwell_sec": self.marking_dwell_sec,
            "waypoint_match_tolerance_m": self.waypoint_match_tolerance,
            "odom_timeout_sec": self.odom_timeout_sec,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")

        if self.intermediate_switch_distance <= self.navigation_tolerance:
            raise ValueError(
                "intermediate_switch_distance_m must be greater than "
                "navigation_tolerance_m"
            )
        if (
            self.pass_through_lookahead_distance
            <= self.intermediate_switch_distance
        ):
            raise ValueError(
                "pass_through_lookahead_distance_m must be greater than "
                "intermediate_switch_distance_m"
            )
        if self.marking_lookahead_distance <= self.navigation_tolerance:
            raise ValueError(
                "marking_lookahead_distance_m must be greater than "
                "navigation_tolerance_m"
            )
        if (
            self.post_marking_alignment_distance
            <= self.intermediate_switch_distance
        ):
            raise ValueError(
                "post_marking_alignment_distance_m must be greater than "
                "intermediate_switch_distance_m"
            )

    def age_seconds(self, timestamp):
        if timestamp is None:
            return math.inf
        return (
            self.get_clock().now() - timestamp
        ).nanoseconds / 1e9

    def odometry_is_fresh(self):
        age = self.age_seconds(self.last_odom_time)
        return math.isfinite(age) and 0.0 <= age <= self.odom_timeout_sec

    def log_waiting(self, reason):
        now = self.get_clock().now()
        if (
            now - self.last_wait_log_time
        ).nanoseconds < 1_000_000_000:
            return
        self.last_wait_log_time = now
        self.get_logger().info(f"WAITING: {reason}")

    def log_rejected(self, reason):
        now = self.get_clock().now()
        if (
            now - self.last_reject_log_time
        ).nanoseconds < 2_000_000_000:
            return
        self.last_reject_log_time = now
        self.get_logger().error(f"MISSION REJECTED: {reason}")

    def validate_path_message(self, msg, label):
        frame_id = msg.header.frame_id.strip()
        if frame_id != self.local_frame:
            self.log_rejected(
                f"{label} frame must be {self.local_frame!r}, "
                f"got {frame_id!r}"
            )
            return None

        if len(msg.poses) == 0:
            self.log_rejected(f"{label} contains zero points")
            return None
        if len(msg.poses) > self.max_path_points:
            self.log_rejected(
                f"{label} contains {len(msg.poses)} points; "
                f"maximum is {self.max_path_points}"
            )
            return None

        points = []
        for index, pose_stamped in enumerate(msg.poses, start=1):
            pose_frame = pose_stamped.header.frame_id.strip()
            if pose_frame and pose_frame != self.local_frame:
                self.log_rejected(
                    f"{label} point {index} has wrong frame "
                    f"{pose_frame!r}"
                )
                return None

            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            if not all(math.isfinite(value) for value in (x, y)):
                self.log_rejected(
                    f"{label} point {index} contains non-finite coordinates"
                )
                return None
            points.append((x, y))

        return points

    def path_callback(self, msg):
        points = self.validate_path_message(msg, "/nav_path")
        if points is None:
            return
        self.pending_path = points
        self.try_load_combined_mission()

    def marking_waypoints_callback(self, msg):
        points = self.validate_path_message(msg, "/mission_waypoints")
        if points is None:
            return
        if len(points) != self.expected_marking_waypoints:
            self.log_rejected(
                f"expected {self.expected_marking_waypoints} marking "
                f"points, got {len(points)}"
            )
            return
        self.pending_marking_waypoints = points
        self.try_load_combined_mission()

    def match_marking_points(self, path_points, marking_waypoints):
        flags = [False] * len(path_points)
        numbers = [0] * len(path_points)
        search_start = 0

        for marking_number, marking_point in enumerate(
            marking_waypoints,
            start=1,
        ):
            matched_index = None
            for path_index in range(search_start, len(path_points)):
                distance = math.hypot(
                    path_points[path_index][0] - marking_point[0],
                    path_points[path_index][1] - marking_point[1],
                )
                if distance <= self.waypoint_match_tolerance:
                    matched_index = path_index
                    break

            if matched_index is None:
                return None, None

            flags[matched_index] = True
            numbers[matched_index] = marking_number
            search_start = matched_index + 1

        return flags, numbers

    @staticmethod
    def make_signature(path_points, marking_waypoints):
        digest = hashlib.sha256()
        for label, points in (
            (b"PATH", path_points),
            (b"MARK", marking_waypoints),
        ):
            digest.update(label)
            for x, y in points:
                digest.update(struct.pack("!dd", x, y))
        return digest.hexdigest()

    def try_load_combined_mission(self):
        if self.pending_path is None or self.pending_marking_waypoints is None:
            return

        flags, numbers = self.match_marking_points(
            self.pending_path,
            self.pending_marking_waypoints,
        )
        if flags is None:
            self.log_rejected(
                "original marking points were not found inside /nav_path"
            )
            return

        new_signature = self.make_signature(
            self.pending_path,
            self.pending_marking_waypoints,
        )
        if new_signature == self.path_signature:
            return

        if self.mission_enabled and not self.emergency_stop:
            self.log_rejected(
                "new mission received while autonomous motion is permitted"
            )
            return

        self.path_points = list(self.pending_path)
        self.marking_flags = flags
        self.marking_numbers = numbers
        self.path_signature = new_signature
        self.current_path_index = 0
        self.mission_complete = False
        self.dwell_start_time = None

        self.publish_marking_active(False)
        self.publish_mission_complete(False)

        marking_indices = [
            index + 1
            for index, flag in enumerate(flags)
            if flag
        ]

        self.get_logger().warn(
            "========== LOOKAHEAD MISSION ACCEPTED =========="
        )
        self.get_logger().warn(
            f"Total path points : {len(self.path_points)}"
        )
        self.get_logger().warn(
            f"Marking indices   : {marking_indices}"
        )
        self.get_logger().warn(
            f"Mission ID        : {new_signature[:12]}"
        )

    def odom_callback(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if not all(math.isfinite(value) for value in (x, y)):
            return

        self.current_x = x
        self.current_y = y
        self.last_odom_time = self.get_clock().now()

    def mission_enable_callback(self, msg):
        enabled = bool(msg.data)
        if enabled != self.mission_enabled:
            self.get_logger().warn(
                "MISSION ENABLED" if enabled else "MISSION DISABLED"
            )
        self.mission_enabled = enabled

        if not enabled:
            self.dwell_start_time = None
            self.publish_marking_active(False)

    def emergency_stop_callback(self, msg):
        active = bool(msg.data)
        if active != self.emergency_stop:
            self.get_logger().warn(
                "EMERGENCY STOP ACTIVE"
                if active
                else "EMERGENCY STOP RELEASED"
            )
        self.emergency_stop = active

        if active:
            self.dwell_start_time = None
            self.publish_marking_active(False)

    def publish_active_waypoint(self, x, y):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.local_frame
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.active_waypoint_pub.publish(msg)

    def publish_marking_active(self, active):
        msg = Bool()
        msg.data = bool(active)
        self.marking_active_pub.publish(msg)

    def publish_mission_complete(self, complete):
        msg = Bool()
        msg.data = bool(complete)
        self.mission_complete_pub.publish(msg)

    def distance_to_index(self, index):
        target_x, target_y = self.path_points[index]
        return math.hypot(
            target_x - self.current_x,
            target_y - self.current_y,
        )

    def next_marking_index(self, start_index):
        for index in range(start_index, len(self.path_points)):
            if self.marking_flags[index]:
                return index
        return None

    def promote_near_marking_target(self):
        if self.marking_flags[self.current_path_index]:
            return

        marking_index = self.next_marking_index(
            self.current_path_index + 1
        )
        if marking_index is None:
            return

        marking_distance = self.distance_to_index(marking_index)
        if marking_distance <= self.marking_lookahead_distance:
            old_index = self.current_path_index
            self.current_path_index = marking_index
            self.get_logger().warn(
                "MARKING APPROACH ACTIVATED | "
                f"path={old_index + 1}->{marking_index + 1} | "
                f"distance={marking_distance:.3f}m"
            )

    def advance_pass_through_targets(self):
        """Keep the active non-marking target ahead of the rover."""
        while (
            self.current_path_index < len(self.path_points) - 1
            and not self.marking_flags[self.current_path_index]
        ):
            current_index = self.current_path_index
            next_index = current_index + 1
            current_distance = self.distance_to_index(current_index)
            next_distance = self.distance_to_index(next_index)
            next_is_marking = self.marking_flags[next_index]

            close_enough = (
                current_distance <= self.intermediate_switch_distance
            )
            clearly_passed = (
                next_distance + self.PASS_MARGIN_M < current_distance
            )
            target_too_close_for_lookahead = (
                current_distance
                < self.pass_through_lookahead_distance
                and not next_is_marking
            )

            if not (
                close_enough
                or clearly_passed
                or target_too_close_for_lookahead
            ):
                break

            self.current_path_index = next_index
            reason = (
                "lookahead"
                if target_too_close_for_lookahead
                else "passed"
                if clearly_passed
                else "near"
            )
            self.get_logger().info(
                "PASS-THROUGH ADVANCE | "
                f"{current_index + 1}->{next_index + 1} | "
                f"reason={reason} | "
                f"old_dist={current_distance:.3f}m | "
                f"new_dist={next_distance:.3f}m"
            )

            if next_is_marking:
                break

    def advance_after_marking(self, completed_number):
        completed_index = self.current_path_index
        next_index = completed_index + 1

        self.dwell_start_time = None
        self.publish_marking_active(False)

        if next_index >= len(self.path_points):
            self.current_path_index = len(self.path_points)
            self.mission_complete = True
            self.publish_mission_complete(True)
            self.get_logger().warn(
                "========== MISSION COMPLETE =========="
            )
            return

        accumulated_distance = 0.0
        selected_index = next_index

        for candidate_index in range(next_index, len(self.path_points)):
            previous_x, previous_y = self.path_points[candidate_index - 1]
            candidate_x, candidate_y = self.path_points[candidate_index]
            accumulated_distance += math.hypot(
                candidate_x - previous_x,
                candidate_y - previous_y,
            )
            selected_index = candidate_index

            if self.marking_flags[candidate_index]:
                break
            if (
                accumulated_distance
                >= self.post_marking_alignment_distance
            ):
                break

        self.current_path_index = selected_index
        target_x, target_y = self.path_points[selected_index]
        skipped = selected_index - completed_index - 1

        self.get_logger().warn(
            f"POST-MARKING ALIGNMENT AFTER WP {completed_number} | "
            f"skipped={skipped} | "
            f"path={selected_index + 1}/{len(self.path_points)} | "
            f"lookahead={accumulated_distance:.3f}m | "
            f"target_E={target_x:.3f} | "
            f"target_N={target_y:.3f}"
        )

    def control_loop(self):
        if not self.path_points:
            self.publish_marking_active(False)
            self.log_waiting("waiting for mission paths")
            return

        if self.current_x is None or self.current_y is None:
            self.publish_marking_active(False)
            self.log_waiting("waiting for odometry")
            return

        if not self.odometry_is_fresh():
            self.publish_marking_active(False)
            self.log_waiting("odometry timeout")
            return

        if self.mission_complete:
            self.publish_marking_active(False)
            self.publish_mission_complete(True)
            self.log_waiting("mission complete")
            return

        if not (
            0 <= self.current_path_index < len(self.path_points)
        ):
            self.mission_complete = True
            self.publish_marking_active(False)
            self.publish_mission_complete(True)
            return

        self.advance_pass_through_targets()
        self.promote_near_marking_target()

        target_x, target_y = self.path_points[self.current_path_index]
        target_is_marking = self.marking_flags[self.current_path_index]
        marking_number = self.marking_numbers[self.current_path_index]
        distance = math.hypot(
            target_x - self.current_x,
            target_y - self.current_y,
        )

        if not math.isfinite(distance):
            self.publish_marking_active(False)
            self.log_waiting("invalid target distance")
            return

        self.publish_active_waypoint(target_x, target_y)

        if not self.mission_enabled:
            self.dwell_start_time = None
            self.publish_marking_active(False)
            self.log_waiting("mission disabled; target held")
            return

        if self.emergency_stop:
            self.dwell_start_time = None
            self.publish_marking_active(False)
            self.log_waiting("emergency stop active; target held")
            return

        now = self.get_clock().now()

        if target_is_marking:
            if self.dwell_start_time is None:
                if distance <= self.navigation_tolerance:
                    self.dwell_start_time = now
                    self.publish_marking_active(True)
                    self.get_logger().warn(
                        f"MARKING WP {marking_number} REACHED | "
                        f"distance={distance:.3f}m"
                    )
                else:
                    self.publish_marking_active(False)
            else:
                self.publish_marking_active(True)
                dwell_elapsed = (
                    now - self.dwell_start_time
                ).nanoseconds / 1e9

                if dwell_elapsed >= self.marking_dwell_sec:
                    self.get_logger().warn(
                        f"MARKING WP {marking_number} COMPLETE | "
                        f"hold={dwell_elapsed:.2f}s | "
                        f"current_distance={distance:.3f}m"
                    )
                    self.advance_after_marking(marking_number)
                    return
        else:
            self.dwell_start_time = None
            self.publish_marking_active(False)

        if (
            now - self.last_status_log_time
        ).nanoseconds >= 1_000_000_000:
            self.last_status_log_time = now
            target_type = (
                f"MARKING WP {marking_number}"
                if target_is_marking
                else "LOOKAHEAD PASS-THROUGH"
            )
            self.get_logger().info(
                f"ACTIVE {target_type} | "
                f"path={self.current_path_index + 1}/"
                f"{len(self.path_points)} | "
                f"dist={distance:.3f}m | "
                f"target_E={target_x:.3f} | "
                f"target_N={target_y:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_marking_active(False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()