"""Safety-gated PX4 velocity-vector bridge for the DYX rover.

RPP input (restart-only selection; default A):
    B: /rpp/command (rpp_interfaces/RppCommand)
        validated horizontal velocity + authoritative absolute ENU yaw.
    A: /rpp/velocity_ned (geometry_msgs/Vector3Stamped)
        vector.x = North velocity [m/s]
        vector.y = East velocity [m/s]
        vector.z = 0

PX4 output:
    /mavros/setpoint_raw/local

Mode A publishes horizontal velocity with yaw ignored. Mode B publishes
horizontal velocity plus authoritative absolute ENU yaw. This bridge never
publishes AttitudeTarget or an active yaw-rate setpoint.

With PX4 rover parameters RD_TRANS_DRV_TRN=45 and RD_TRANS_TRN_DRV=12,
PX4 performs its native differential pivot above 45 degrees and changes back
to straight driving at 12 degrees or less.

RPP owns acceleration, deceleration and all requested speed shaping. This bridge preserves
finite RPP speeds from 0.00 through 1.00 m/s and clamps only commands above the
configured safety maximum. Each accepted RPP command is gated and forwarded to
PX4 as soon as it arrives (RPP runs at 50 Hz). The 50 Hz timer still evaluates
every safety gate and publishes immediately when the gate outcome changes; it
repeats the last command only as a keep-alive when no command has been
forwarded for KEEPALIVE_PERIODS stream periods (an RPP stall).
Literal zero remains the stop command for all safety conditions.

On every new MAVROS connection the bridge also asks PX4 to stream
LOCAL_POSITION_NED (the pose RPP steers from) at local_position_rate_hz.
"""

from __future__ import annotations

import math
from typing import Any

import rclpy
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import MessageInterval
from rcl_interfaces.msg import ParameterDescriptor
from rpp_interfaces.msg import RppCommand
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, UInt64

from jetson_4wd_control.mavlink_stream_rate import (
    MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    StreamRateRequester,
)


class CmdVelBridge(Node):
    """Gate RPP vectors and stream safe PX4 PositionTarget messages."""

    STREAM_HZ = 50.0
    # Timer keep-alive: repeat the last setpoint only if nothing was published
    # for this many stream periods. RPP normally publishes every period.
    KEEPALIVE_PERIODS = 1.25
    STREAM_RATE_RETRY_SEC = 2.0
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
    TYPE_MASK_VELOCITY_YAW_RATE = (
        TYPE_MASK_VELOCITY_ONLY
        & ~PositionTarget.IGNORE_YAW
        & ~PositionTarget.IGNORE_YAW_RATE
    )

    def __init__(self) -> None:
        super().__init__("cmd_vel_bridge")

        self.declare_parameter(
            "rpp_explicit_yaw_enabled",
            False,
            ParameterDescriptor(
                read_only=True,
                description="Restart-only RPP command transport; set in rover.launch.py",
            ),
        )
        self.rpp_explicit_yaw_enabled = bool(
            self.get_parameter("rpp_explicit_yaw_enabled").value
        )

        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("backend_heartbeat_timeout_sec", 1.5)
        self.declare_parameter("maximum_speed_mps", 1.00)

        # Bridge-side safety ceiling for Jetson yaw-rate commands.
        # PX4 RO_YAW_RATE_LIM remains the final FCU hard ceiling.
        self.declare_parameter("maximum_yaw_rate_radps", 0.45)

        # PX4 LOCAL_POSITION_NED rate requested on each MAVROS connection.
        # 0 leaves PX4's default stream rate untouched.
        self.declare_parameter("local_position_rate_hz", 0.0)

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
        explicit_yaw_command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
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

        if self.rpp_explicit_yaw_enabled:
            self.create_subscription(
                RppCommand,
                "/rpp/command",
                self._rpp_command_callback,
                explicit_yaw_command_qos,
            )
            self.get_logger().warn(
                "Bridge startup contract: B / EXPLICIT_YAW | "
                "Patch 4 validated atomic velocity+yaw forwarding active"
            )
        else:
            self.create_subscription(
                Vector3Stamped,
                "/rpp/velocity_ned",
                self._rpp_callback,
                command_qos,
            )
            self.get_logger().warn("Bridge startup contract: A / LEGACY_VELOCITY")
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
        self.latest_yaw_enu = 0.0
        self.latest_yaw_rate_enu = 0.0
        self.latest_yaw_valid = False
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

        # Phase-0 dropout instrumentation. The 1 Hz summary below cannot
        # observe a transient: at 50 Hz a one-cycle reason change is sampled
        # with probability 0.02. Mid-pivot yaw loss and stream stalls are both
        # transients, so they are edge-logged here instead.
        self._previous_reason = None
        self._previous_reason_time = self.get_clock().now()
        self._previous_yaw_valid = False
        self._previous_loop_time = None
        self._late_cycle_count = 0
        self._yaw_drop_count = 0

        self._last_publish_time = None
        self._last_published_reason = None

        self._message_interval_client = self.create_client(
            MessageInterval,
            "/mavros/set_message_interval",
        )
        self.local_position_rate = StreamRateRequester(
            message_id=MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            rate_hz=float(self.get_parameter("local_position_rate_hz").value),
            retry_sec=self.STREAM_RATE_RETRY_SEC,
            send=self._send_message_interval,
        )

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
        self._evaluate_and_publish(from_timer=False)

    def _clear_b_command(self, reason: str) -> None:
        self.latest_north = 0.0
        self.latest_east = 0.0
        self.latest_yaw_enu = 0.0
        self.latest_yaw_rate_enu = 0.0
        self.latest_yaw_valid = False
        self.latest_command_time = None
        self.get_logger().error(f"Rejected RPP atomic command: {reason}")
        self._evaluate_and_publish(from_timer=False)

    def _rpp_command_callback(self, message: RppCommand) -> None:
        try:
            north = float(message.velocity_north_mps)
            east = float(message.velocity_east_mps)
        except (AttributeError, TypeError, ValueError):
            self._clear_b_command("invalid velocity fields")
            return

        if not all(math.isfinite(value) for value in (north, east)):
            self._clear_b_command("non-finite velocity")
            return

        horizontal_speed = math.hypot(north, east)
        if horizontal_speed > self.maximum_speed:
            scale = self.maximum_speed / horizontal_speed
            north *= scale
            east *= scale
            self.get_logger().warn(
                "Clamped RPP atomic command above 1.00 m/s | "
                f"requested={horizontal_speed:.3f}m/s"
            )
        elif horizontal_speed <= self.COMMAND_EPSILON:
            north = 0.0
            east = 0.0

        horizontal_speed = math.hypot(north, east)
        yaw_valid = bool(message.yaw_valid)

        if yaw_valid:
            try:
                yaw_enu = float(message.yaw_enu_rad)
                yaw_rate_enu = float(message.yaw_rate_enu_radps)
            except (AttributeError, TypeError, ValueError):
                self._clear_b_command("invalid explicit yaw/yaw-rate field")
                return

            if not math.isfinite(yaw_enu):
                self._clear_b_command("non-finite explicit yaw")
                return
            if not math.isfinite(yaw_rate_enu):
                self._clear_b_command("non-finite explicit yaw-rate")
                return

            yaw_rate_enu = max(
                -self.maximum_yaw_rate_compat,
                min(self.maximum_yaw_rate_compat, yaw_rate_enu),
            )

        else:
            yaw_enu = 0.0
            yaw_rate_enu = 0.0
            if horizontal_speed > self.COMMAND_EPSILON:
                self._clear_b_command(
                    "moving command requires finite authoritative yaw+yaw-rate"
                )
                return

        self.latest_north = north
        self.latest_east = east
        self.latest_yaw_enu = yaw_enu
        self.latest_yaw_rate_enu = yaw_rate_enu
        self.latest_yaw_valid = yaw_valid
        self.latest_command_time = self.get_clock().now()
        self._evaluate_and_publish(from_timer=False)

    def _state_callback(self, message: State) -> None:
        self.connected = bool(message.connected)
        self.armed = bool(message.armed)
        self.mode = str(message.mode).strip().upper()
        self.local_position_rate.on_state(
            self.connected,
            self._message_interval_client.service_is_ready(),
            self.get_clock().now().nanoseconds / 1e9,
        )

    def _send_message_interval(self, message_id: int, rate_hz: float) -> bool:
        request = MessageInterval.Request()
        request.message_id = int(message_id)
        request.message_rate = float(rate_hz)
        epoch = self.local_position_rate.connection_epoch
        future = self._message_interval_client.call_async(request)

        def _done(done_future) -> None:
            try:
                success = bool(done_future.result().success)
            except Exception as error:  # noqa: BLE001 - logged, then retried
                self.get_logger().warn(
                    f"set_message_interval id={message_id} failed: {error}"
                )
                success = False
            self.local_position_rate.on_response(success, epoch)
            log = self.get_logger().info if success else self.get_logger().warn
            log(
                f"PX4 message {message_id} rate {rate_hz:.1f} Hz "
                f"{'accepted' if success else 'REJECTED; retrying'}"
            )

        future.add_done_callback(_done)
        return True

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
        *,
        yaw_enu_rad: float = 0.0,
        yaw_rate_enu_radps: float = 0.0,
        yaw_valid: bool = False,
    ) -> None:
        yaw_active = bool(yaw_valid) and self.rpp_explicit_yaw_enabled
        if not all(math.isfinite(value) for value in (north, east)):
            north = 0.0
            east = 0.0
            yaw_active = False

        if yaw_active and not all(
            math.isfinite(float(value))
            for value in (yaw_enu_rad, yaw_rate_enu_radps)
        ):
            north = 0.0
            east = 0.0
            yaw_active = False

        message = PositionTarget()
        message.header.stamp = self.get_clock().now().to_msg()
        message.coordinate_frame = self.FRAME_LOCAL_NED
        message.type_mask = (
            self.TYPE_MASK_VELOCITY_YAW_RATE
            if yaw_active
            else self.TYPE_MASK_VELOCITY_ONLY
        )

        # MAVROS ROS fields are ENU: x=East, y=North. Absolute yaw and signed
        # yaw-rate are also supplied in ROS ENU; MAVROS owns ENU->NED conversion.
        message.velocity.x = float(east)
        message.velocity.y = float(north)
        message.velocity.z = 0.0

        message.position.x = 0.0
        message.position.y = 0.0
        message.position.z = 0.0
        message.acceleration_or_force.x = 0.0
        message.acceleration_or_force.y = 0.0
        message.acceleration_or_force.z = 0.0
        message.yaw = float(yaw_enu_rad) if yaw_active else 0.0
        message.yaw_rate = float(yaw_rate_enu_radps) if yaw_active else 0.0

        self.setpoint_pub.publish(message)

    def _control_loop(self) -> None:
        """50 Hz timer: safety-gate evaluation and keep-alive."""
        self._evaluate_and_publish(from_timer=True)

    def _evaluate_and_publish(self, *, from_timer: bool) -> None:
        loop_time = self.get_clock().now()
        if from_timer and self._previous_loop_time is not None:
            interval = (
                loop_time - self._previous_loop_time
            ).nanoseconds / 1e9
            if interval > 2.0 / self.STREAM_HZ:
                self._late_cycle_count += 1
                self.get_logger().warn(
                    "BRIDGE STREAM LATE | "
                    f"interval={interval * 1000.0:.1f}ms "
                    f"expected={1000.0 / self.STREAM_HZ:.1f}ms "
                    f"lateCycles={self._late_cycle_count}"
                )
        if from_timer:
            self._previous_loop_time = loop_time

        north = 0.0
        east = 0.0
        yaw_enu = 0.0
        yaw_rate_enu = 0.0
        yaw_valid = False

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
            if self.rpp_explicit_yaw_enabled:
                yaw_enu = self.latest_yaw_enu
                yaw_rate_enu = self.latest_yaw_rate_enu
                yaw_valid = self.latest_yaw_valid
            reason = (
                "publishing_rpp_explicit_yaw"
                if yaw_valid
                else (
                    "publishing_rpp_velocity_vector"
                    if math.hypot(north, east) > self.COMMAND_EPSILON
                    else "publishing_rpp_stop"
                )
            )

        # The timer only re-sends when the gate outcome changed or the stream
        # has gone quiet (RPP stall). Normally each fresh RPP command has
        # already been forwarded by its own callback within the last period.
        if (
            from_timer
            and reason == self._last_published_reason
            and self._age_seconds(self._last_publish_time)
            < self.KEEPALIVE_PERIODS / self.STREAM_HZ
        ):
            return

        # In B mode a safety/inhibit boundary invalidates the entire atomic
        # command. A fresh RPP command is required before motion/yaw can resume.
        if (
            self.rpp_explicit_yaw_enabled
            and reason
            not in {"publishing_rpp_explicit_yaw", "publishing_rpp_stop"}
        ):
            self.latest_north = 0.0
            self.latest_east = 0.0
            self.latest_yaw_enu = 0.0
            self.latest_yaw_rate_enu = 0.0
            self.latest_yaw_valid = False
            self.latest_command_time = None

        self._publish_local_velocity(
            north,
            east,
            yaw_enu_rad=yaw_enu,
            yaw_rate_enu_radps=yaw_rate_enu,
            yaw_valid=yaw_valid,
        )
        self._last_publish_time = loop_time
        self._last_published_reason = reason

        if self._previous_yaw_valid and not yaw_valid:
            self._yaw_drop_count += 1
            self.get_logger().warn(
                "EXPLICIT YAW DROPPED | "
                f"reason={reason} "
                f"previousReason={self._previous_reason} "
                f"commandAge={self._age_seconds(self.latest_command_time):.3f}s "
                f"vN={north:.3f} vE={east:.3f} "
                f"yawDrops={self._yaw_drop_count}"
            )
        self._previous_yaw_valid = yaw_valid

        if reason != self._previous_reason:
            if self._previous_reason is not None:
                dwell = (
                    loop_time - self._previous_reason_time
                ).nanoseconds / 1e9
                self.get_logger().info(
                    "BRIDGE REASON CHANGE | "
                    f"{self._previous_reason} -> {reason} | "
                    f"previousHeld={dwell:.3f}s"
                )
            self._previous_reason = reason
            self._previous_reason_time = loop_time

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