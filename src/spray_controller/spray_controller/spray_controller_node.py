#!/usr/bin/env python3
"""Production spray-servo controller for the DYX 4WD marking rover.

PX4 contract
------------
Holybro Pixhawk 6X AUX5:
    PWM_AUX_FUNC5 = 301
    Peripheral via Actuator Set 1

MAVLink command:
    MAV_CMD_DO_SET_ACTUATOR = 187
    param1 = actuator-set-1 normalized value [-1, +1]

Mission contract
----------------
The mission_manager publishes /marking_active only while a REAL marking point
is inside tolerance and the rover is stationary. This node independently
re-validates mission state, point identity, PX4 state, safety topics and topic
freshness before moving the spray servo.

Safety properties
-----------------
* Non-blocking state machine (no sleep in callbacks/timers).
* Never reports SUCCESS until PRESS was accepted and RELEASE was accepted.
* PRESS is never automatically retried after an uncertain command timeout.
* RELEASE is idempotently retried until PX4 acknowledges it.
* Any E-stop, mission-disable, OFFBOARD loss, disarm, stale control input, or
  marking-hold loss during a spray causes an immediate release-recovery path.
* A persistent journal prevents a node restart from blindly re-spraying a
  point after a crash at an uncertain stage.
* Faults latch. Reset is allowed only in a safe state using /spray/reset_fault.
"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path
from typing import Any, Optional

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class SprayController(Node):
    """Fail-safe AUX5 spray-servo controller."""

    MAV_CMD_DO_SET_ACTUATOR = 187
    MAV_RESULT_ACCEPTED = 0
    PROTOCOL_VERSION = 1

    STATE_IDLE = "IDLE"
    STATE_WAIT_PRESS_ACK = "WAIT_PRESS_ACK"
    STATE_SPRAYING = "SPRAYING"
    STATE_WAIT_RELEASE_ACK = "WAIT_RELEASE_ACK"
    STATE_RECOVERY_RELEASE = "RECOVERY_RELEASE"
    STATE_WAIT_MARKING_CLEAR = "WAIT_MARKING_CLEAR"
    STATE_FAULT = "FAULT"

    JOURNAL_PRESS_COMMAND_SENT = "PRESS_COMMAND_SENT"
    JOURNAL_PRESSED = "PRESSED"
    JOURNAL_RELEASE_UNCONFIRMED = "RELEASE_UNCONFIRMED"
    JOURNAL_COMPLETED = "COMPLETED"
    JOURNAL_FAILED_SAFE = "FAILED_SAFE"

    def __init__(self) -> None:
        super().__init__("spray_controller")

        # ==========================================================
        # Parameters
        # ==========================================================
        self.declare_parameter("enabled", True)
        self.declare_parameter("press_value", 1.0)
        self.declare_parameter("release_value", 0.0)
        self.declare_parameter("spray_duration_sec", 3.0)
        self.declare_parameter("pre_spray_stable_sec", 0.25)

        self.declare_parameter("command_timeout_sec", 1.0)
        self.declare_parameter("release_retry_interval_sec", 0.25)
        self.declare_parameter("hard_press_timeout_sec", 5.0)

        self.declare_parameter("mavros_state_timeout_sec", 2.5)
        self.declare_parameter("mission_status_timeout_sec", 1.0)
        self.declare_parameter("marking_active_timeout_sec", 0.50)

        self.declare_parameter("require_px4_armed", True)
        self.declare_parameter("require_px4_offboard", True)

        self.declare_parameter(
            "journal_path",
            "~/.ros/dyx_spray_controller_journal.json",
        )

        self.enabled = bool(self.get_parameter("enabled").value)
        self.press_value = float(self.get_parameter("press_value").value)
        self.release_value = float(self.get_parameter("release_value").value)
        self.spray_duration_sec = float(self.get_parameter("spray_duration_sec").value)
        self.pre_spray_stable_sec = float(
            self.get_parameter("pre_spray_stable_sec").value
        )

        self.command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value
        )
        self.release_retry_interval_sec = float(
            self.get_parameter("release_retry_interval_sec").value
        )
        self.hard_press_timeout_sec = float(
            self.get_parameter("hard_press_timeout_sec").value
        )

        self.mavros_state_timeout_sec = float(
            self.get_parameter("mavros_state_timeout_sec").value
        )
        self.mission_status_timeout_sec = float(
            self.get_parameter("mission_status_timeout_sec").value
        )
        self.marking_active_timeout_sec = float(
            self.get_parameter("marking_active_timeout_sec").value
        )

        self.require_px4_armed = bool(self.get_parameter("require_px4_armed").value)
        self.require_px4_offboard = bool(
            self.get_parameter("require_px4_offboard").value
        )

        self.journal_path = Path(
            os.path.expanduser(str(self.get_parameter("journal_path").value))
        )

        self._last_config_request_id: Optional[str] = None
        self._last_config_result: Optional[str] = None
        self._last_config_reason: Optional[str] = None

        self._validate_parameters()

        # ==========================================================
        # QoS
        # ==========================================================
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        retained_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ==========================================================
        # Inputs
        # ==========================================================
        self.create_subscription(
            Bool,
            "/marking_active",
            self._marking_active_callback,
            command_qos,
        )

        self.create_subscription(
            Bool,
            "/mission_enable",
            self._mission_enable_callback,
            command_qos,
        )

        self.create_subscription(
            Bool,
            "/emergency_stop",
            self._emergency_stop_callback,
            command_qos,
        )

        self.create_subscription(
            String,
            "/mission_manager/status",
            self._mission_status_callback,
            retained_qos,
        )

        self.create_subscription(
            State,
            "/mavros/state",
            self._mavros_state_callback,
            state_qos,
        )

        # ==========================================================
        # Outputs
        # ==========================================================

        self.create_subscription(
            String,
            "/spray/config",
            self._spray_config_callback,
            retained_qos,
        )

        self.spray_active_pub = self.create_publisher(
            Bool,
            "/spray/active",
            retained_qos,
        )

        self.spray_complete_pub = self.create_publisher(
            String,
            "/spray/complete",
            retained_qos,
        )

        self.spray_result_pub = self.create_publisher(
            String,
            "/spray/result",
            retained_qos,
        )

        self.spray_status_pub = self.create_publisher(
            String,
            "/spray/status",
            retained_qos,
        )

        self.create_service(
            Trigger,
            "/spray/reset_fault",
            self._reset_fault_service,
        )

        # ==========================================================
        # MAVROS command client
        # ==========================================================
        self.command_client = self.create_client(
            CommandLong,
            "/mavros/cmd/command",
        )

        # ==========================================================
        # Runtime inputs/state
        # ==========================================================
        self.marking_active = False
        self.mission_enable = False
        self.emergency_stop = True

        self.mavros_connected = False
        self.px4_armed = False
        self.px4_mode = ""

        self.mission_state: Optional[str] = None
        self.mission_run_id: Optional[str] = None
        self.current_point_id: Optional[str] = None
        self.current_point_index: Optional[int] = None
        self.current_point_state: Optional[str] = None
        self.status_marking_active = False

        self.last_marking_active_rx: Optional[float] = None
        self.last_mission_status_rx: Optional[float] = None
        self.last_mavros_state_rx: Optional[float] = None

        self.marking_active_since: Optional[float] = None

        # ==========================================================
        # Spray transaction state
        # ==========================================================
        self.state = self.STATE_IDLE

        self.transaction_run_id: Optional[str] = None
        self.transaction_point_id: Optional[str] = None
        self.transaction_point_index: Optional[int] = None

        self.command_future = None
        self.command_kind: Optional[str] = None
        self.command_sent_at: Optional[float] = None

        self.press_may_be_active = False
        self.press_command_sent_at: Optional[float] = None
        self.spray_started_at: Optional[float] = None

        self.failure_after_release: Optional[str] = None
        self.next_release_retry_at = 0.0

        self.release_confirmed = False

        self.fault_latched = False
        self.fault_reason: Optional[str] = None

        self.result_sequence = 0
        self.last_result: Optional[dict[str, Any]] = None
        self._last_completed_replay_key: Optional[tuple[str, str]] = None

        self.journal = self._load_journal()

        if self.journal is not None:
            journal_state = str(self.journal.get("state", ""))

            if journal_state in {
                self.JOURNAL_PRESS_COMMAND_SENT,
                self.JOURNAL_PRESSED,
                self.JOURNAL_RELEASE_UNCONFIRMED,
            }:
                self.transaction_run_id = self._journal_str("mission_run_id")
                self.transaction_point_id = self._journal_str("point_id")
                self.transaction_point_index = self._journal_int("point_index")
                self.press_may_be_active = True
                self.failure_after_release = "NODE_RESTART_DURING_UNCERTAIN_SPRAY"
                self.state = self.STATE_RECOVERY_RELEASE
                self.next_release_retry_at = 0.0

                self.get_logger().error(
                    "Uncertain previous spray transaction found in journal; "
                    "release recovery is required before operation."
                )

            elif journal_state == self.JOURNAL_FAILED_SAFE:
                self.fault_latched = True
                self.fault_reason = str(
                    self.journal.get(
                        "reason",
                        "PREVIOUS_SPRAY_FAILED",
                    )
                )
                self.state = self.STATE_FAULT

        self._publish_active(self.press_may_be_active)
        self._publish_status()

        # 50 Hz safety/control state machine.
        self.create_timer(0.02, self._control_loop)

        # 5 Hz retained health/status heartbeat.
        self.create_timer(0.20, self._publish_status)

        self.get_logger().warn("===== PRODUCTION SPRAY CONTROLLER STARTED =====")
        self.get_logger().warn(
            "PX4 output contract: AUX5 / PWM_AUX_FUNC5=301 / "
            "Peripheral via Actuator Set 1"
        )
        self.get_logger().warn(
            f"Servo command: press={self.press_value:+.3f}, "
            f"duration={self.spray_duration_sec:.3f}s, "
            f"release={self.release_value:+.3f}"
        )
        self.get_logger().warn(f"Journal: {self.journal_path}")

    # ==============================================================
    # Parameter / utility helpers
    # ==============================================================

    def _validate_parameters(self) -> None:
        for name, value in (
            ("press_value", self.press_value),
            ("release_value", self.release_value),
        ):
            if not math.isfinite(value) or not -1.0 <= value <= 1.0:
                raise ValueError(f"{name} must be finite and in [-1.0, +1.0]")

        positive = {
            "spray_duration_sec": self.spray_duration_sec,
            "pre_spray_stable_sec": self.pre_spray_stable_sec,
            "command_timeout_sec": self.command_timeout_sec,
            "release_retry_interval_sec": self.release_retry_interval_sec,
            "hard_press_timeout_sec": self.hard_press_timeout_sec,
            "mavros_state_timeout_sec": self.mavros_state_timeout_sec,
            "mission_status_timeout_sec": self.mission_status_timeout_sec,
            "marking_active_timeout_sec": self.marking_active_timeout_sec,
        }

        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")

        if self.hard_press_timeout_sec <= self.spray_duration_sec:
            raise ValueError(
                "hard_press_timeout_sec must be greater than " "spray_duration_sec"
            )

    @staticmethod
    def _age(last_rx: Optional[float], now: float) -> float:
        if last_rx is None:
            return math.inf
        return max(0.0, now - last_rx)

    def _journal_str(self, key: str) -> Optional[str]:
        if self.journal is None:
            return None
        value = self.journal.get(key)
        if isinstance(value, str) and value:
            return value
        return None

    def _journal_int(self, key: str) -> Optional[int]:
        if self.journal is None:
            return None
        value = self.journal.get(key)
        if isinstance(value, int):
            return value
        return None

    # ==============================================================
    # Persistent transaction journal
    # ==============================================================

    def _load_journal(self) -> Optional[dict[str, Any]]:
        try:
            if not self.journal_path.exists():
                return None

            payload = json.loads(self.journal_path.read_text(encoding="utf-8"))

            if not isinstance(payload, dict):
                raise ValueError("journal root is not an object")

            if int(payload.get("version", -1)) != self.PROTOCOL_VERSION:
                raise ValueError("unsupported journal version")

            return payload

        except Exception as exc:
            # A corrupt safety journal must not be silently ignored.
            self.get_logger().error(f"Unable to read spray journal safely: {exc}")
            return {
                "version": self.PROTOCOL_VERSION,
                "state": self.JOURNAL_FAILED_SAFE,
                "reason": f"JOURNAL_READ_ERROR: {exc}",
                "timestamp_unix_ns": time.time_ns(),
            }

    def _write_journal(
        self,
        *,
        state: str,
        reason: Optional[str] = None,
    ) -> None:
        payload = {
            "version": self.PROTOCOL_VERSION,
            "state": state,
            "mission_run_id": self.transaction_run_id,
            "point_id": self.transaction_point_id,
            "point_index": self.transaction_point_index,
            "press_value": self.press_value,
            "release_value": self.release_value,
            "spray_duration_sec": self.spray_duration_sec,
            "reason": reason,
            "timestamp_unix_ns": time.time_ns(),
        }

        self.journal_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        temporary = self.journal_path.with_suffix(self.journal_path.suffix + ".tmp")

        encoded = json.dumps(
            payload,
            separators=(",", ":"),
            sort_keys=True,
        )

        with temporary.open("w", encoding="utf-8") as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())

        os.replace(
            temporary,
            self.journal_path,
        )

        self.journal = payload

    def _clear_journal(self) -> None:
        try:
            self.journal_path.unlink(missing_ok=True)
        except Exception as exc:
            raise RuntimeError(f"Unable to clear spray journal: {exc}") from exc
        self.journal = None

    # ==============================================================
    # Input callbacks
    # ==============================================================

    def _marking_active_callback(self, msg: Bool) -> None:
        now = time.monotonic()
        new_value = bool(msg.data)

        self.last_marking_active_rx = now

        if new_value and not self.marking_active:
            self.marking_active_since = now

        if not new_value and self.marking_active:
            self.marking_active_since = None

            if self.state in {
                self.STATE_WAIT_PRESS_ACK,
                self.STATE_SPRAYING,
            }:
                self._abort_to_release("MARKING_HOLD_LOST_DURING_SPRAY")

            elif self.state == self.STATE_WAIT_MARKING_CLEAR:
                self._reset_transaction_to_idle()

        self.marking_active = new_value

    def _mission_enable_callback(self, msg: Bool) -> None:
        self.mission_enable = bool(msg.data)

        if not self.mission_enable and self.state in {
            self.STATE_WAIT_PRESS_ACK,
            self.STATE_SPRAYING,
        }:
            self._abort_to_release("MISSION_DISABLED_DURING_SPRAY")

    def _emergency_stop_callback(self, msg: Bool) -> None:
        self.emergency_stop = bool(msg.data)

        if self.emergency_stop and self.state in {
            self.STATE_WAIT_PRESS_ACK,
            self.STATE_SPRAYING,
        }:
            self._abort_to_release("EMERGENCY_STOP_DURING_SPRAY")

    def _mission_status_callback(self, msg: String) -> None:
        now = time.monotonic()

        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        if not isinstance(payload, dict):
            return

        self.last_mission_status_rx = now

        state = payload.get("state")
        self.mission_state = str(state) if state is not None else None

        run_id = payload.get("mission_run_id")
        self.mission_run_id = (
            str(run_id) if isinstance(run_id, str) and run_id else None
        )

        point_id = payload.get("current_point_id")
        self.current_point_id = (
            str(point_id) if isinstance(point_id, str) and point_id else None
        )

        point_index = payload.get("current_point_index")
        self.current_point_index = (
            int(point_index) if isinstance(point_index, int) else None
        )

        point_state = payload.get("current_point_state")
        self.current_point_state = str(point_state) if point_state is not None else None

        self.status_marking_active = bool(payload.get("marking_active", False))

    def _mavros_state_callback(self, msg: State) -> None:
        self.last_mavros_state_rx = time.monotonic()

        self.mavros_connected = bool(msg.connected)
        self.px4_armed = bool(msg.armed)
        self.px4_mode = str(msg.mode or "")

        if self.state in {
            self.STATE_WAIT_PRESS_ACK,
            self.STATE_SPRAYING,
        } and not self._px4_gate_ok(time.monotonic()):
            self._abort_to_release("PX4_STATE_BECAME_UNSAFE_DURING_SPRAY")

    # ==============================================================
    # Safety gates
    # ==============================================================

    def _px4_gate_ok(self, now: float) -> bool:
        if self._age(self.last_mavros_state_rx, now) > self.mavros_state_timeout_sec:
            return False

        if not self.mavros_connected:
            return False

        if self.require_px4_armed and not self.px4_armed:
            return False

        if self.require_px4_offboard and self.px4_mode.upper() != "OFFBOARD":
            return False

        return True

    def _spray_config_callback(
        self,
        msg: String,
    ) -> None:
        """Safely update servo press/release positions at runtime."""

        try:
            payload = json.loads(msg.data)
        except Exception:
            self.get_logger().error("Rejected spray configuration: invalid JSON")
            return

        if not isinstance(payload, dict):
            self.get_logger().error(
                "Rejected spray configuration: payload is not an object"
            )
            return

        request_id = str(payload.get("request_id") or "").strip()

        # Configuration must NEVER change during a spray transaction.
        if (
            self.state != self.STATE_IDLE
            or self.press_may_be_active
            or self.marking_active
            or self.mission_enable
        ):
            self.get_logger().warn(
                "Rejected spray configuration because " "spray controller is not idle"
            )

            self._last_config_request_id = request_id
            self._last_config_result = "REJECTED"
            self._last_config_reason = "SPRAY_CONTROLLER_NOT_IDLE"

            self._publish_status()
            return

        try:
            press_value = float(payload["press_value"])

            release_value = float(payload["release_value"])

        except (
            KeyError,
            TypeError,
            ValueError,
        ):
            self._last_config_request_id = request_id
            self._last_config_result = "REJECTED"
            self._last_config_reason = "INVALID_VALUE"

            self._publish_status()
            return

        if not math.isfinite(press_value) or not -1.0 <= press_value <= 1.0:
            self._last_config_request_id = request_id
            self._last_config_result = "REJECTED"
            self._last_config_reason = "PRESS_VALUE_OUT_OF_RANGE"

            self._publish_status()
            return

        if not math.isfinite(release_value) or not -1.0 <= release_value <= 1.0:
            self._last_config_request_id = request_id
            self._last_config_result = "REJECTED"
            self._last_config_reason = "RELEASE_VALUE_OUT_OF_RANGE"

            self._publish_status()
            return

        self.press_value = press_value
        self.release_value = release_value

        self._last_config_request_id = request_id
        self._last_config_result = "ACCEPTED"
        self._last_config_reason = None

        self.get_logger().warn(
            "Spray servo configuration updated: "
            f"press={self.press_value:+.3f}, "
            f"release={self.release_value:+.3f}"
        )

        self._publish_status()

    def _spray_gate_ok(self, now: float) -> bool:
        if not self.enabled:
            return False

        if self.fault_latched:
            return False

        if not self.mission_enable or self.emergency_stop:
            return False

        if self.mission_state != "RUNNING":
            return False

        if not self.mission_run_id:
            return False

        if not self.current_point_id:
            return False

        if self.current_point_state != "ACTIVE":
            return False

        if not self.marking_active or not self.status_marking_active:
            return False

        if (
            self._age(self.last_marking_active_rx, now)
            > self.marking_active_timeout_sec
        ):
            return False

        if (
            self._age(self.last_mission_status_rx, now)
            > self.mission_status_timeout_sec
        ):
            return False

        if not self._px4_gate_ok(now):
            return False

        if not self.command_client.service_is_ready():
            return False

        return True

    # ==============================================================
    # MAVLink command helpers
    # ==============================================================

    def _make_actuator_request(
        self,
        value: float,
    ) -> CommandLong.Request:
        req = CommandLong.Request()
        req.broadcast = False
        req.command = self.MAV_CMD_DO_SET_ACTUATOR
        req.confirmation = 0

        # Actuator Set 1 -> param1. NaN explicitly leaves sets 2..6 alone.
        req.param1 = float(value)
        req.param2 = math.nan
        req.param3 = math.nan
        req.param4 = math.nan
        req.param5 = math.nan
        req.param6 = math.nan

        # Index 0 selects actuators 1..6. This is the first set.
        req.param7 = 0.0
        return req

    def _send_command(
        self,
        *,
        kind: str,
        value: float,
    ) -> bool:
        if not self.command_client.service_is_ready():
            return False

        self.command_future = self.command_client.call_async(
            self._make_actuator_request(value)
        )

        self.command_kind = kind
        self.command_sent_at = time.monotonic()
        return True

    @staticmethod
    def _response_accepted(response: Any) -> bool:
        if response is None:
            return False

        return (
            bool(getattr(response, "success", False))
            and int(getattr(response, "result", -1))
            == SprayController.MAV_RESULT_ACCEPTED
        )

    def _clear_command_future(self) -> None:
        self.command_future = None
        self.command_kind = None
        self.command_sent_at = None

    # ==============================================================
    # Transaction/result helpers
    # ==============================================================

    def _begin_transaction_from_current_point(self) -> None:
        self.transaction_run_id = self.mission_run_id
        self.transaction_point_id = self.current_point_id
        self.transaction_point_index = self.current_point_index

    def _transaction_matches_current(self) -> bool:
        return (
            self.transaction_run_id is not None
            and self.transaction_point_id is not None
            and self.transaction_run_id == self.mission_run_id
            and self.transaction_point_id == self.current_point_id
        )

    def _journal_completed_for_current_point(self) -> bool:
        if self.journal is None:
            return False

        return (
            self.journal.get("state") == self.JOURNAL_COMPLETED
            and self.journal.get("mission_run_id") == self.mission_run_id
            and self.journal.get("point_id") == self.current_point_id
        )

    def _publish_result(
        self,
        *,
        result: str,
        reason: Optional[str],
    ) -> None:
        self.result_sequence += 1

        payload = {
            "protocol_version": self.PROTOCOL_VERSION,
            "sequence": self.result_sequence,
            "result": result,
            "reason": reason,
            "mission_run_id": self.transaction_run_id,
            "point_id": self.transaction_point_id,
            "point_index": self.transaction_point_index,
            "press_value": self.press_value,
            "release_value": self.release_value,
            "spray_duration_sec": self.spray_duration_sec,
            "timestamp_unix_ns": time.time_ns(),
        }

        msg = String()
        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
            sort_keys=True,
        )
        self.spray_result_pub.publish(msg)
        self.last_result = payload

        if result == "SUCCESS" and self.transaction_point_id is not None:
            complete = String()
            complete.data = self.transaction_point_id
            self.spray_complete_pub.publish(complete)

    def _publish_active(self, active: bool) -> None:
        msg = Bool()
        msg.data = bool(active)
        self.spray_active_pub.publish(msg)

    def _controller_ready(self, now: float) -> bool:
        # Preflight readiness deliberately does NOT require armed/OFFBOARD,
        # because those gates may only be opened after the mission is started.
        return (
            self.enabled
            and not self.fault_latched
            and self.state
            not in {
                self.STATE_RECOVERY_RELEASE,
                self.STATE_FAULT,
            }
            and self.command_client.service_is_ready()
            and self.mavros_connected
            and self._age(
                self.last_mavros_state_rx,
                now,
            )
            <= self.mavros_state_timeout_sec
        )

    def _publish_status(self) -> None:
        now = time.monotonic()

        spray_elapsed_sec: Optional[float] = None
        spray_remaining_sec: Optional[float] = None
        if (
            self.state == self.STATE_SPRAYING
            and self.spray_started_at is not None
        ):
            spray_elapsed_sec = max(0.0, now - self.spray_started_at)
            spray_remaining_sec = max(
                0.0,
                self.spray_duration_sec - spray_elapsed_sec,
            )

        payload = {
            "protocol_version": self.PROTOCOL_VERSION,
            "timestamp_unix_ns": time.time_ns(),
            "ready": self._controller_ready(now),
            "enabled": self.enabled,
            "controller_state": self.state,
            "fault_latched": self.fault_latched,
            "fault_reason": self.fault_reason,
            "press_may_be_active": self.press_may_be_active,
            "release_confirmed": self.release_confirmed,
            "mission_run_id": self.mission_run_id,
            "current_point_id": self.current_point_id,
            "transaction_run_id": self.transaction_run_id,
            "transaction_point_id": self.transaction_point_id,
            "mavros_connected": self.mavros_connected,
            "px4_armed": self.px4_armed,
            "px4_mode": self.px4_mode,
            "mission_state": self.mission_state,
            "mission_enable": self.mission_enable,
            "emergency_stop": self.emergency_stop,
            "marking_active": self.marking_active,
            "status_marking_active": self.status_marking_active,
            "mavros_state_age_sec": (
                None
                if self.last_mavros_state_rx is None
                else round(
                    self._age(self.last_mavros_state_rx, now),
                    3,
                )
            ),
            "mission_status_age_sec": (
                None
                if self.last_mission_status_rx is None
                else round(
                    self._age(self.last_mission_status_rx, now),
                    3,
                )
            ),
            "marking_active_age_sec": (
                None
                if self.last_marking_active_rx is None
                else round(
                    self._age(self.last_marking_active_rx, now),
                    3,
                )
            ),
            "journal_state": (
                self.journal.get("state") if self.journal is not None else None
            ),
            "press_value": self.press_value,
            "release_value": self.release_value,
            "spray_duration_sec": self.spray_duration_sec,
            "spray_elapsed_sec": (
                round(spray_elapsed_sec, 3)
                if spray_elapsed_sec is not None
                else None
            ),
            "spray_remaining_sec": (
                round(spray_remaining_sec, 3)
                if spray_remaining_sec is not None
                else None
            ),
            "spraying": self.state == self.STATE_SPRAYING,
            "config_request_id": (self._last_config_request_id),
            "config_result": (self._last_config_result),
            "config_reason": (self._last_config_reason),
            "last_result": self.last_result,
        }

        msg = String()
        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
            sort_keys=True,
        )
        self.spray_status_pub.publish(msg)

    # ==============================================================
    # Fault/release recovery
    # ==============================================================

    def _abort_to_release(self, reason: str) -> None:
        if self.failure_after_release is None:
            self.failure_after_release = reason

        self._write_journal(
            state=self.JOURNAL_RELEASE_UNCONFIRMED,
            reason=reason,
        )

        self._begin_or_retry_release()

    def _begin_or_retry_release(self) -> None:
        now = time.monotonic()

        # A RELEASE command may be repeated safely. Never create a second
        # outstanding future at the same time.
        if self.command_future is not None:
            return

        if self._send_command(
            kind="RELEASE",
            value=self.release_value,
        ):
            self.state = self.STATE_WAIT_RELEASE_ACK
            return

        self.state = self.STATE_RECOVERY_RELEASE
        self.next_release_retry_at = now + self.release_retry_interval_sec

    def _release_succeeded(self) -> None:
        self.press_may_be_active = False
        self.release_confirmed = True
        self._publish_active(False)

        if self.failure_after_release is not None:
            reason = self.failure_after_release

            self._write_journal(
                state=self.JOURNAL_FAILED_SAFE,
                reason=reason,
            )

            self._publish_result(
                result="FAILED",
                reason=reason,
            )

            self.fault_latched = True
            self.fault_reason = reason
            self.state = self.STATE_FAULT

            self.get_logger().error(
                f"Spray transaction failed safely after RELEASE: {reason}"
            )
            return

        self._write_journal(
            state=self.JOURNAL_COMPLETED,
        )

        self._publish_result(
            result="SUCCESS",
            reason=None,
        )

        self.state = self.STATE_WAIT_MARKING_CLEAR

        self.get_logger().warn(
            "SPRAY SUCCESS | "
            f"{self.transaction_point_id} | "
            f"press={self.press_value:+.3f} | "
            f"duration={self.spray_duration_sec:.3f}s | "
            f"release={self.release_value:+.3f}"
        )

    def _set_fault_without_press(
        self,
        reason: str,
    ) -> None:
        self.fault_latched = True
        self.fault_reason = reason
        self.state = self.STATE_FAULT

        if (
            self.transaction_run_id is not None
            and self.transaction_point_id is not None
        ):
            self._write_journal(
                state=self.JOURNAL_FAILED_SAFE,
                reason=reason,
            )

            self._publish_result(
                result="FAILED",
                reason=reason,
            )

        self._publish_active(False)

        self.get_logger().error(f"SPRAY FAULT: {reason}")

    def _reset_transaction_to_idle(self) -> None:
        self.state = self.STATE_IDLE

        self.transaction_run_id = None
        self.transaction_point_id = None
        self.transaction_point_index = None

        self.command_future = None
        self.command_kind = None
        self.command_sent_at = None

        self.press_command_sent_at = None
        self.spray_started_at = None
        self.press_may_be_active = False

        self.failure_after_release = None
        self.next_release_retry_at = 0.0

        self.marking_active_since = None

        self._publish_active(False)

    # ==============================================================
    # Fault reset service
    # ==============================================================

    def _reset_fault_service(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if not self.fault_latched:
            response.success = True
            response.message = "Spray controller has no latched fault"
            return response

        if self.press_may_be_active:
            response.success = False
            response.message = "Cannot reset: servo release is not confirmed"
            return response

        if self.marking_active:
            response.success = False
            response.message = "Cannot reset while marking_active is true"
            return response

        if self.mission_enable and not self.emergency_stop:
            response.success = False
            response.message = "Cannot reset while mission is enabled without E-stop"
            return response

        try:
            self._clear_journal()
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response

        self.fault_latched = False
        self.fault_reason = None
        self.release_confirmed = True

        self._reset_transaction_to_idle()
        self._publish_status()

        response.success = True
        response.message = "Spray fault reset in safe state"
        return response

    # ==============================================================
    # Main control state machine
    # ==============================================================

    def _control_loop(self) -> None:
        now = time.monotonic()

        # ----------------------------------------------------------
        # Recovery after a node/process restart at an uncertain stage.
        # RELEASE is retried indefinitely until acknowledged.
        # ----------------------------------------------------------
        if self.state == self.STATE_RECOVERY_RELEASE:
            if self.command_future is None and now >= self.next_release_retry_at:
                self._begin_or_retry_release()
            return

        if self.state == self.STATE_FAULT:
            return

        # ----------------------------------------------------------
        # IDLE: wait for a stable REAL marking hold.
        # ----------------------------------------------------------
        if self.state == self.STATE_IDLE:
            if not self._spray_gate_ok(now):
                self.marking_active_since = now if self.marking_active else None
                return

            if self._journal_completed_for_current_point():
                key = (
                    str(self.mission_run_id),
                    str(self.current_point_id),
                )

                self._begin_transaction_from_current_point()

                if self._last_completed_replay_key != key:
                    self._publish_result(
                        result="SUCCESS",
                        reason="REPLAY_FROM_PERSISTENT_JOURNAL",
                    )
                    self._last_completed_replay_key = key

                self.state = self.STATE_WAIT_MARKING_CLEAR
                return

            if self.marking_active_since is None:
                self.marking_active_since = now
                return

            if now - self.marking_active_since < self.pre_spray_stable_sec:
                return

            self._begin_transaction_from_current_point()

            if not self._transaction_matches_current():
                self._set_fault_without_press("POINT_ID_CHANGED_BEFORE_PRESS")
                return

            # Persist the transaction BEFORE sending PRESS. If the process
            # dies in the tiny window around COMMAND_LONG transmission, the
            # restart path treats the press as uncertain, RELEASES, and faults
            # instead of blindly spraying the point a second time.
            self._write_journal(
                state=self.JOURNAL_PRESS_COMMAND_SENT,
            )

            # PRESS is intentionally sent at most once. If its ACK becomes
            # uncertain we RELEASE and fail instead of blindly pressing again.
            if not self._send_command(
                kind="PRESS",
                value=self.press_value,
            ):
                self._set_fault_without_press(
                    "MAVROS_COMMAND_SERVICE_UNAVAILABLE_BEFORE_PRESS"
                )
                return

            self.press_command_sent_at = self.command_sent_at
            self.press_may_be_active = True
            self.release_confirmed = False
            self._publish_active(True)

            self.state = self.STATE_WAIT_PRESS_ACK
            return

        # ----------------------------------------------------------
        # PRESS ACK
        # ----------------------------------------------------------
        if self.state == self.STATE_WAIT_PRESS_ACK:
            if not self._spray_gate_ok(now):
                self._abort_to_release("SAFETY_GATE_LOST_BEFORE_PRESS_ACK")
                return

            if (
                self.press_command_sent_at is not None
                and now - self.press_command_sent_at >= self.hard_press_timeout_sec
            ):
                self._abort_to_release("HARD_PRESS_WATCHDOG_BEFORE_PRESS_ACK")
                return

            if self.command_future is None:
                self._abort_to_release("PRESS_FUTURE_MISSING")
                return

            if not self.command_future.done():
                if (
                    self.command_sent_at is not None
                    and now - self.command_sent_at > self.command_timeout_sec
                ):
                    self._clear_command_future()
                    self._abort_to_release("PRESS_ACK_TIMEOUT_UNCERTAIN")
                return

            try:
                response = self.command_future.result()
            except Exception as exc:
                self._clear_command_future()
                self._abort_to_release(f"PRESS_SERVICE_EXCEPTION:{exc}")
                return

            accepted = self._response_accepted(response)
            result_code = getattr(response, "result", None)
            self._clear_command_future()

            if not accepted:
                self._abort_to_release(f"PRESS_REJECTED_RESULT_{result_code}")
                return

            self._write_journal(
                state=self.JOURNAL_PRESSED,
            )

            # The configured spray duration is the physical PRESS hold time.
            # Start the timer only after PX4/MAVROS has ACKed the PRESS command,
            # so command/transport latency is not subtracted from the 3-second
            # marking interval.
            self.spray_started_at = now
            self.state = self.STATE_SPRAYING
            return

        # ----------------------------------------------------------
        # Timed press.
        # ----------------------------------------------------------
        if self.state == self.STATE_SPRAYING:
            if not self._spray_gate_ok(now):
                self._abort_to_release("SAFETY_GATE_LOST_DURING_PRESS")
                return

            if self.press_command_sent_at is None:
                self._abort_to_release("PRESS_START_TIME_MISSING")
                return

            if self.spray_started_at is None:
                self._abort_to_release("SPRAY_HOLD_START_TIME_MISSING")
                return

            total_press_elapsed = now - self.press_command_sent_at
            spray_hold_elapsed = now - self.spray_started_at

            # Absolute fail-safe watchdog includes command/ACK latency.
            if total_press_elapsed >= self.hard_press_timeout_sec:
                self._abort_to_release("HARD_PRESS_WATCHDOG_EXPIRED")
                return

            # Normal release is based on the confirmed physical spray-hold
            # interval, not on the PRESS command transmission time.
            if spray_hold_elapsed >= self.spray_duration_sec:
                self.failure_after_release = None
                self._begin_or_retry_release()
            return

        # ----------------------------------------------------------
        # RELEASE ACK. RELEASE may be retried; this is safe/idempotent.
        # ----------------------------------------------------------
        if self.state == self.STATE_WAIT_RELEASE_ACK:
            if self.command_future is None:
                self.state = self.STATE_RECOVERY_RELEASE
                self.next_release_retry_at = now
                return

            if not self.command_future.done():
                if (
                    self.command_sent_at is not None
                    and now - self.command_sent_at > self.command_timeout_sec
                ):
                    self._clear_command_future()

                    self._write_journal(
                        state=self.JOURNAL_RELEASE_UNCONFIRMED,
                        reason=(self.failure_after_release or "RELEASE_ACK_TIMEOUT"),
                    )

                    self.state = self.STATE_RECOVERY_RELEASE
                    self.next_release_retry_at = now + self.release_retry_interval_sec
                return

            try:
                response = self.command_future.result()
            except Exception as exc:
                self._clear_command_future()

                self._write_journal(
                    state=self.JOURNAL_RELEASE_UNCONFIRMED,
                    reason=(
                        self.failure_after_release or f"RELEASE_SERVICE_EXCEPTION:{exc}"
                    ),
                )

                self.state = self.STATE_RECOVERY_RELEASE
                self.next_release_retry_at = now + self.release_retry_interval_sec
                return

            accepted = self._response_accepted(response)
            result_code = getattr(response, "result", None)
            self._clear_command_future()

            if not accepted:
                self._write_journal(
                    state=self.JOURNAL_RELEASE_UNCONFIRMED,
                    reason=(
                        self.failure_after_release
                        or f"RELEASE_REJECTED_RESULT_{result_code}"
                    ),
                )

                self.state = self.STATE_RECOVERY_RELEASE
                self.next_release_retry_at = now + self.release_retry_interval_sec
                return

            self._release_succeeded()
            return

        # ----------------------------------------------------------
        # Do not accept another point until current marking hold clears.
        # ----------------------------------------------------------
        if self.state == self.STATE_WAIT_MARKING_CLEAR:
            if not self.marking_active:
                self._reset_transaction_to_idle()

    # ==============================================================
    # Shutdown
    # ==============================================================

    def best_effort_release_on_shutdown(self) -> None:
        """Try once to place the servo in release position before shutdown."""
        try:
            if not self.command_client.service_is_ready():
                return

            future = self.command_client.call_async(
                self._make_actuator_request(self.release_value)
            )

            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=0.5,
            )

        except Exception:
            # Hardware/PX4 PWM_FAIL/PWM_DIS must remain the final protection
            # against process death or transport loss.
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SprayController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.best_effort_release_on_shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()