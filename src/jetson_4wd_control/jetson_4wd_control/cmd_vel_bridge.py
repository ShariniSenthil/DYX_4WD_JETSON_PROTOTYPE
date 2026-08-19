"""Safety-gated PX4 velocity-vector bridge for the DYX rover.

RPP input:
    /rpp/velocity_ned (geometry_msgs/Vector3Stamped)
        vector.x = North velocity [m/s]
        vector.y = East velocity [m/s]
        vector.z = 0

PX4 output:
    /mavros/setpoint_raw/local

This bridge publishes only horizontal velocity vectors. It does not publish
AttitudeTarget messages and it does not use local-position yaw-rate fields.

With PX4 rover parameters RD_TRANS_DRV_TRN=45 and RD_TRANS_TRN_DRV=12,
PX4 performs its native differential pivot above 45 degrees and changes back
to straight driving at 12 degrees or less.

RPP owns acceleration, deceleration and all requested speed shaping. This bridge preserves
finite RPP speeds from 0.00 through 1.00 m/s and clamps only commands above the
configured safety maximum. It repeats the latest RPP command to PX4 at 50 Hz.
Literal zero remains the stop command for all safety conditions.
"""

from __future__ import annotations

import math
from typing import Any

import rclpy
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, UInt64


class CmdVelBridge(Node):
    """Gate RPP vectors and stream safe PX4 PositionTarget messages."""

    STREAM_HZ = 50.0
    ABSOLUTE_MAXIMUM_SPEED_MPS = 1.00
    COMMAND_EPSILON = 1.0e-6

    FRAME_LOCAL_NED = PositionTarget.FRAME_LOCAL_NED

    TYPE_MASK_VELOCITY_ONLY = (
        PositionTarget.IGNORE_PX
        | PositionTarget.IGNORE_PY
        | PositionTarget.IGNORE_PZ
        | PositionTarget.IGNORE_AFX
        | PositionTarget.IGNORE_AFY
        | PositionTarget.IGNORE_AFZ
        | PositionTarget.IGNORE_YAW
        | PositionTarget.IGNORE_YAW_RATE
    )

    def __init__(self) -> None:
        super().__init__("cmd_vel_bridge")

        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("backend_heartbeat_timeout_sec", 1.5)
        self.declare_parameter("maximum_speed_mps", 1.00)

        # Compatibility-only: retained because existing launch/test tooling
        # queries it. This bridge never publishes a yaw-rate setpoint.
        self.declare_parameter("maximum_yaw_rate_radps", 0.20)

        self.command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value
        )
        self.backend_heartbeat_timeout_sec = float(
            self.get_parameter("backend_heartbeat_timeout_sec").value
        )
        requested_maximum_speed = float(
            self.get_parameter("maximum_speed_mps").value
        )
        self.maximum_yaw_rate_compat = float(
            self.get_parameter("maximum_yaw_rate_radps").value
        )
        self.maximum_speed = min(
            requested_maximum_speed,
            self.ABSOLUTE_MAXIMUM_SPEED_MPS,
        )

        for name, value in {
            "command_timeout_sec": self.command_timeout_sec,
            "backend_heartbeat_timeout_sec": (
                self.backend_heartbeat_timeout_sec
            ),
            "maximum_speed_mps": self.maximum_speed,
            "maximum_yaw_rate_radps": self.maximum_yaw_rate_compat,
        }.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")

        if requested_maximum_speed > self.ABSOLUTE_MAXIMUM_SPEED_MPS:
            self.get_logger().warn(
                "maximum_speed_mps exceeds the 1.00 m/s safety cap; "
                "clamped to 1.00 m/s"
            )

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        safety_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        retained_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Vector3Stamped,
            "/rpp/velocity_ned",
            self._rpp_callback,
            command_qos,
        )
        self.create_subscription(
            State,
            "/mavros/state",
            self._state_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission_enable",
            self._mission_callback,
            safety_qos,
        )
        self.create_subscription(
            Bool,
            "/emergency_stop",
            self._estop_callback,
            safety_qos,
        )
        self.create_subscription(
            UInt64,
            "/rover_backend/heartbeat",
            self._heartbeat_callback,
            safety_qos,
        )

        self.setpoint_pub = self.create_publisher(
            PositionTarget,
            "/mavros/setpoint_raw/local",
            command_qos,
        )
        self.heartbeat_health_pub = self.create_publisher(
            Bool,
            "/cmd_vel_bridge/backend_heartbeat_healthy",
            retained_qos,
        )

        self.latest_north = 0.0
        self.latest_east = 0.0
        self.latest_command_time = None

        self.connected = False
        self.armed = False
        self.mode = ""
        self.mission_enabled = False
        self.emergency_stop = True

        self.latest_backend_heartbeat_time = None
        self.latest_backend_heartbeat_sequence = None
        self.last_heartbeat_health = None
        self.last_log_time = self.get_clock().now()

        self._publish_heartbeat_health(False, force=True)
        self.timer = self.create_timer(
            1.0 / self.STREAM_HZ,
            self._control_loop,
        )

        self.get_logger().warn(
            "===== PX4 NATIVE ROVER VELOCITY BRIDGE STARTED ====="
        )
        self.get_logger().warn(
            "RPP velocity bridge: 0.00..1.00 m/s at 50 Hz"
        )
        self.get_logger().warn(
            "PX4 native pivot contract: enter 45deg, drive at 12deg"
        )
        self.get_logger().warn(
            "No AttitudeTarget and no yaw-rate pivot commands"
        )

    def _age_seconds(self, timestamp: Any) -> float:
        if timestamp is None:
            return math.inf
        return (
            self.get_clock().now() - timestamp
        ).nanoseconds / 1e9

    def _heartbeat_healthy(self) -> bool:
        return (
            self._age_seconds(self.latest_backend_heartbeat_time)
            <= self.backend_heartbeat_timeout_sec
        )

    def _rpp_callback(self, message: Vector3Stamped) -> None:
        north = float(message.vector.x)
        east = float(message.vector.y)
        unused_z = float(message.vector.z)

        if not all(
            math.isfinite(value)
            for value in (north, east, unused_z)
        ):
            self.get_logger().error("Rejected non-finite RPP command")
            return

        if abs(unused_z) > self.COMMAND_EPSILON:
            self.get_logger().error(
                "Rejected non-zero RPP vector.z; pivot must use a "
                "horizontal velocity-vector bearing"
            )
            north = 0.0
            east = 0.0

        horizontal_speed = math.hypot(north, east)
        if horizontal_speed > self.maximum_speed:
            scale = self.maximum_speed / horizontal_speed
            north *= scale
            east *= scale
            self.get_logger().warn(
                "Clamped RPP command above 1.00 m/s | "
                f"requested={horizontal_speed:.3f}m/s"
            )
        elif horizontal_speed <= self.COMMAND_EPSILON:
            north = 0.0
            east = 0.0

        self.latest_north = north
        self.latest_east = east
        self.latest_command_time = self.get_clock().now()

    def _state_callback(self, message: State) -> None:
        self.connected = bool(message.connected)
        self.armed = bool(message.armed)
        self.mode = str(message.mode).strip().upper()

    def _mission_callback(self, message: Bool) -> None:
        self.mission_enabled = bool(message.data)

    def _estop_callback(self, message: Bool) -> None:
        self.emergency_stop = bool(message.data)

    def _heartbeat_callback(self, message: UInt64) -> None:
        self.latest_backend_heartbeat_time = self.get_clock().now()
        self.latest_backend_heartbeat_sequence = int(message.data)

    def _publish_heartbeat_health(
        self,
        healthy: bool,
        *,
        force: bool = False,
    ) -> None:
        healthy = bool(healthy)
        if (
            not force
            and self.last_heartbeat_health is not None
            and healthy == self.last_heartbeat_health
        ):
            return

        self.last_heartbeat_health = healthy
        message = Bool()
        message.data = healthy
        self.heartbeat_health_pub.publish(message)

    def _publish_local_velocity(
        self,
        north: float,
        east: float,
    ) -> None:
        message = PositionTarget()
        message.header.stamp = self.get_clock().now().to_msg()
        message.coordinate_frame = self.FRAME_LOCAL_NED
        message.type_mask = self.TYPE_MASK_VELOCITY_ONLY

        # MAVROS ROS fields are ENU: x=East, y=North.
        message.velocity.x = float(east)
        message.velocity.y = float(north)
        message.velocity.z = 0.0

        message.position.x = 0.0
        message.position.y = 0.0
        message.position.z = 0.0
        message.acceleration_or_force.x = 0.0
        message.acceleration_or_force.y = 0.0
        message.acceleration_or_force.z = 0.0
        message.yaw = 0.0
        message.yaw_rate = 0.0

        self.setpoint_pub.publish(message)

    def _control_loop(self) -> None:
        north = 0.0
        east = 0.0

        heartbeat_healthy = self._heartbeat_healthy()
        self._publish_heartbeat_health(heartbeat_healthy)

        if self.emergency_stop:
            reason = "emergency_stop"
        elif not self.mission_enabled:
            reason = "mission_disabled"
        elif not heartbeat_healthy:
            reason = "backend_heartbeat_timeout"
        elif not self.connected:
            reason = "px4_disconnected"
        elif not self.armed:
            reason = "px4_disarmed"
        elif self.mode != "OFFBOARD":
            reason = f"mode_{self.mode or 'unknown'}"
        elif self._age_seconds(self.latest_command_time) > self.command_timeout_sec:
            reason = "rpp_command_timeout"
        else:
            north = self.latest_north
            east = self.latest_east
            reason = (
                "publishing_rpp_velocity_vector"
                if math.hypot(north, east) > self.COMMAND_EPSILON
                else "publishing_rpp_stop"
            )

        self._publish_local_velocity(north, east)

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds >= 1_000_000_000:
            self.last_log_time = now
            heartbeat_age = self._age_seconds(
                self.latest_backend_heartbeat_time
            )
            heartbeat_age_text = (
                "never"
                if not math.isfinite(heartbeat_age)
                else f"{heartbeat_age:.2f}s"
            )
            sequence_text = (
                "none"
                if self.latest_backend_heartbeat_sequence is None
                else str(self.latest_backend_heartbeat_sequence)
            )
            self.get_logger().info(
                f"reason={reason} "
                f"connected={self.connected} "
                f"armed={self.armed} "
                f"mode={self.mode} "
                f"mission={self.mission_enabled} "
                f"estop={self.emergency_stop} "
                f"backendHeartbeat={heartbeat_healthy} "
                f"heartbeatAge={heartbeat_age_text} "
                f"heartbeatSeq={sequence_text} "
                f"vN={north:.3f} "
                f"vE={east:.3f}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(10):
            node._publish_local_velocity(0.0, 0.0)
        node._publish_heartbeat_health(False, force=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()