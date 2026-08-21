"""Mission manager for the DYX 4WD marking rover.

Architecture
------------
trajectory_generator owns static path geometry.
rpp_controller owns all vehicle motion: alignment, steering, path tracking,
acceleration, cruise, deceleration, re-alignment and final stop.
mission_manager owns only:
  * mission/path validation and marking order
  * AUTO/MANUAL sequencing
  * START/PAUSE/RESUME/NEXT/SKIP/STOP/CLEAR
  * hard E-stop and soft mission-enable gate
  * PX4 OFFBOARD/arming orchestration; STOP disarms but never requests MANUAL
  * RTK pre-check and runtime RTK pause/recovery indication
  * exact radial marking validation and stationary verification
  * spray request/result handshake
  * COMPLETED/FAILED/SKIPPED state
  * mission status and point events
  * x-track/along-track calculation for REPORTING ONLY

Important: this node never computes steering, lookahead, acceleration,
deceleration, pivot commands, heading alignment or cross-track correction.
"""

from __future__ import annotations

import copy
import json
import math
import threading
import time
import uuid
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GPSRAW, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry, Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32MultiArray, String, UInt8MultiArray
from std_srvs.srv import Trigger

from mission_manager.marking_arrival_policy import after_fail_mode_action


class MissionManager(Node):
    """Mission/safety/marking state machine. Motion control is RPP's job."""

    CONTROL_HZ = 20.0
    STATUS_HZ = 5.0
    SAFETY_PUBLISH_HZ = 2.0

    FCU_STATE_STALE_SEC = 3.0
    GPS_FIX_STALE_SEC = 2.0
    RTK_STATUS_STALE_SEC = 15.0
    MAX_RTK_CORRECTION_AGE_SEC = 15.0
    OFFBOARD_STREAM_SETTLE_SEC = 0.60
    SERVICE_DISCOVERY_TIMEOUT_SEC = 3.0
    SERVICE_RESPONSE_TIMEOUT_SEC = 5.0
    VEHICLE_STATE_CONFIRM_TIMEOUT_SEC = 3.0
    OFFBOARD_BEFORE_ARM_SETTLE_SEC = 0.50
    GOAL_PLANE_OVERSHOOT_TRIGGER_M = 0.020

    POINT_PASS_THROUGH = 0
    POINT_DUMMY_ALIGNMENT = 1
    POINT_MARKING = 2
    VALID_POINT_TYPES = {POINT_PASS_THROUGH, POINT_DUMMY_ALIGNMENT, POINT_MARKING}
    TERMINAL_POINT_STATES = {"COMPLETED", "SKIPPED", "FAILED"}

    def __init__(self) -> None:
        super().__init__("mission_manager")
        self._io_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()

        # ----------------------------------------------------------
        # Parameters: mission/safety/marking only
        # ----------------------------------------------------------
        self.declare_parameter("local_frame", "map")
        self.declare_parameter("marking_tolerance_m", 0.03)
        self.declare_parameter("arrival_settle_sec", 0.30)
        self.declare_parameter("marking_hold_sec", 3.00)
        self.declare_parameter("stationary_speed_tolerance_mps", 0.01)
        self.declare_parameter("dummy_arrival_tolerance_m", 0.03)
        self.declare_parameter("waypoint_match_tolerance_m", 0.002)
        self.declare_parameter("odom_timeout_sec", 0.50)
        self.declare_parameter("maximum_navigation_points", 200000)
        self.declare_parameter("maximum_marking_points", 10000)
        self.declare_parameter("spray_required", True)
        self.declare_parameter("spray_confirmation_timeout_sec", 5.0)
        self.declare_parameter("spray_status_timeout_sec", 2.0)

        self.local_frame = str(self.get_parameter("local_frame").value).strip()
        self.marking_tolerance_m = float(
            self.get_parameter("marking_tolerance_m").value
        )
        self.arrival_settle_sec = float(self.get_parameter("arrival_settle_sec").value)
        self.marking_hold_sec = float(self.get_parameter("marking_hold_sec").value)
        self.stationary_speed_tolerance_mps = float(
            self.get_parameter("stationary_speed_tolerance_mps").value
        )
        self.dummy_arrival_tolerance_m = float(
            self.get_parameter("dummy_arrival_tolerance_m").value
        )
        self.waypoint_match_tolerance_m = float(
            self.get_parameter("waypoint_match_tolerance_m").value
        )
        self.odom_timeout_sec = float(self.get_parameter("odom_timeout_sec").value)
        self.maximum_navigation_points = int(
            self.get_parameter("maximum_navigation_points").value
        )
        self.maximum_marking_points = int(
            self.get_parameter("maximum_marking_points").value
        )
        self.spray_required = bool(self.get_parameter("spray_required").value)
        self.spray_confirmation_timeout_sec = float(
            self.get_parameter("spray_confirmation_timeout_sec").value
        )
        self.spray_status_timeout_sec = float(
            self.get_parameter("spray_status_timeout_sec").value
        )
        self._validate_parameters()

        # ----------------------------------------------------------
        # QoS
        # ----------------------------------------------------------
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
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        mavros_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ----------------------------------------------------------
        # Inputs from trajectory_generator
        # ----------------------------------------------------------
        self.create_subscription(
            Path, "/nav_path", self._nav_path_callback, retained_qos
        )
        self.create_subscription(
            Path, "/mission_waypoints", self._mission_waypoints_callback, retained_qos
        )
        self.create_subscription(
            UInt8MultiArray,
            "/trajectory_generator/path_types",
            self._path_types_callback,
            retained_qos,
        )
        self.create_subscription(
            Int32MultiArray,
            "/trajectory_generator/marking_indices",
            self._marking_indices_callback,
            retained_qos,
        )
        self.create_subscription(
            Bool,
            "/trajectory_generator/ready",
            self._trajectory_ready_callback,
            retained_qos,
        )

        # ----------------------------------------------------------
        # Rover / PX4 / RTK / backend health
        # ----------------------------------------------------------
        self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self._odom_callback,
            odom_qos,
            callback_group=self._io_group,
        )
        self.create_subscription(
            State,
            "/mavros/state",
            self._mavros_state_callback,
            mavros_state_qos,
            callback_group=self._io_group,
        )
        self.create_subscription(
            GPSRAW,
            "/mavros/gpsstatus/gps1/raw",
            self._gps_status_callback,
            odom_qos,
            callback_group=self._io_group,
        )
        self.create_subscription(
            Bool,
            "/rtk_correction_bridge/healthy",
            self._rtk_health_callback,
            retained_qos,
            callback_group=self._io_group,
        )
        self.create_subscription(
            Float32,
            "/rtk_correction_bridge/correction_age_sec",
            self._rtk_correction_age_callback,
            retained_qos,
            callback_group=self._io_group,
        )
        self.create_subscription(
            Bool,
            "/cmd_vel_bridge/backend_heartbeat_healthy",
            self._backend_heartbeat_health_callback,
            retained_qos,
            callback_group=self._io_group,
        )
        self.create_subscription(
            String,
            "/mission_manager/execution_mode",
            self._execution_mode_callback,
            retained_qos,
        )
        self.create_subscription(
            String,
            "/spray/status",
            self._spray_status_callback,
            retained_qos,
            callback_group=self._io_group,
        )
        self.create_subscription(
            String,
            "/spray/result",
            self._spray_result_callback,
            retained_qos,
            callback_group=self._io_group,
        )
        self.create_subscription(
            String,
            "/rpp/terminal_result",
            self._rpp_terminal_result_callback,
            command_qos,
            callback_group=self._io_group,
        )

        # ----------------------------------------------------------
        # Outputs
        # ----------------------------------------------------------
        # mission_enable = soft movement gate owned only by mission_manager.
        # emergency_stop = hard safety latch.
        self.mission_enable_pub = self.create_publisher(
            Bool, "/mission_enable", command_qos
        )
        self.emergency_stop_pub = self.create_publisher(
            Bool, "/emergency_stop", command_qos
        )

        # Compatibility contract for current RPP/backend.
        # BOTH topics now carry only the current SEMANTIC goal (dummy/marking),
        # never a moving interpolation/lookahead point.
        self.active_waypoint_pub = self.create_publisher(
            PoseStamped, "/active_waypoint", command_qos
        )
        self.segment_goal_pub = self.create_publisher(
            PoseStamped, "/segment_goal", command_qos
        )

        self.runtime_path_pub = self.create_publisher(
            Path, "/runtime_nav_path", retained_qos
        )
        self.marking_active_pub = self.create_publisher(
            Bool, "/marking_active", command_qos
        )
        self.mission_complete_pub = self.create_publisher(
            Bool, "/mission_complete", retained_qos
        )
        self.status_pub = self.create_publisher(
            String, "/mission_manager/status", retained_qos
        )
        self.point_event_pub = self.create_publisher(
            String, "/mission_manager/point_event", command_qos
        )

        # ----------------------------------------------------------
        # Services
        # ----------------------------------------------------------
        self.create_service(Trigger, "/mission_manager/start", self._start_service)
        self.create_service(Trigger, "/mission_manager/pause", self._pause_service)
        self.create_service(Trigger, "/mission_manager/resume", self._resume_service)
        self.create_service(
            Trigger, "/mission_manager/next_point", self._next_point_service
        )
        self.create_service(
            Trigger, "/mission_manager/skip_point", self._skip_point_service
        )
        self.create_service(Trigger, "/mission_manager/stop", self._stop_service)
        self.create_service(Trigger, "/mission_manager/clear", self._clear_service)
        self.create_service(
            Trigger,
            "/mission_manager/emergency_stop",
            self._emergency_stop_service,
            callback_group=self._io_group,
        )
        self.create_service(
            Trigger,
            "/mission_manager/release_emergency_stop",
            self._release_emergency_stop_service,
            callback_group=self._io_group,
        )
        self._arming_client = self.create_client(
            CommandBool, "/mavros/cmd/arming", callback_group=self._io_group
        )
        self._mode_client = self.create_client(
            SetMode, "/mavros/set_mode", callback_group=self._io_group
        )

        # ----------------------------------------------------------
        # Prepared mission snapshot
        # ----------------------------------------------------------
        self._pending_nav_path: Optional[list[tuple[float, float]]] = None
        self._pending_mission_waypoints: Optional[list[tuple[float, float]]] = None
        self._pending_path_types: Optional[list[int]] = None
        self._pending_marking_indices: Optional[list[int]] = None
        self._trajectory_ready = False

        self._navigation_path: list[tuple[float, float]] = []
        self._mission_waypoints: list[tuple[float, float]] = []
        self._path_types: list[int] = []
        self._marking_indices: list[int] = []
        self._marking_path_index_by_number: list[int] = []
        self._semantic_path_indices: list[int] = []  # dummy + real marking only
        self._point_status: list[str] = []
        # Accuracy snapshots are captured exactly once for each uploaded CSV
        # point.  They are intentionally independent from the live working
        # metrics so safety-gate changes cannot erase the evidence used by the
        # final mission report.
        self._point_accuracy_snapshots: list[Optional[dict[str, Any]]] = []
        self._point_results: list[Optional[dict[str, Any]]] = []
        self._last_point_result: Optional[dict[str, Any]] = None
        self._last_termination_report: Optional[dict[str, Any]] = None

        # ----------------------------------------------------------
        # Mission runtime
        # ----------------------------------------------------------
        self._state = "EMPTY"
        self._execution_mode = "AUTO"
        self._pause_reason: Optional[str] = None
        self._resume_available = False
        self._last_message = "No prepared mission loaded"
        self._last_error: Optional[str] = None

        self._current_path_index = 0
        self._active_marking_number: Optional[int] = None
        self._last_completed_marking_number: Optional[int] = None
        self._mission_run_id = ""
        self._mission_started_unix_ns: Optional[int] = None

        # Precision point transaction:
        #   1) first exact arrival starts a FIXED 3-second verification window;
        #   2) at the end of that window the point is checked again;
        #   3) only if it is STILL inside 30 mm and stationary do we spray;
        #   4) after the spray transaction finishes, advance to the next point.
        #
        # arrival_settle_* is retained for status/backward compatibility.
        self._arrival_settle_started: Optional[float] = None
        self._arrival_settle_elapsed_sec = 0.0
        self._marking_hold_started: Optional[float] = None
        self._marking_hold_elapsed_sec = 0.0
        self._spray_request_started: Optional[float] = None

        # Final mission completion is reported first, then on the next control
        # tick the exact same STOP cleanup is executed automatically.
        self._auto_stop_pending = False

        self._marking_xtrack_m = math.inf
        self._marking_along_error_m = math.inf
        self._marking_combined_error_m = math.inf
        self._marking_radial_error_m = math.inf
        self._marking_error_valid = False

        # Latest explicit RPP terminal handshake for the current semantic goal.
        # It can start the same 3-second verification after an RPP miss, but it
        # never bypasses Mission Manager's own final position/speed re-check.
        self._rpp_terminal_result: Optional[dict[str, Any]] = None
        self._rpp_terminal_result_last_rx_monotonic: Optional[float] = None

        self._spray_controller_ready = False
        self._spray_controller_state: Optional[str] = None
        self._spray_fault_reason: Optional[str] = None
        self._spray_status_last_rx_monotonic: Optional[float] = None
        self._spray_status_timestamp_unix_ns: Optional[int] = None
        self._spray_success_keys: set[tuple[str, str]] = set()
        self._spray_failure_keys: dict[tuple[str, str], str] = {}

        # ----------------------------------------------------------
        # Rover state
        # ----------------------------------------------------------
        self._x: Optional[float] = None
        self._y: Optional[float] = None
        self._yaw: Optional[float] = None
        self._speed_mps = 0.0
        self._last_odom_monotonic: Optional[float] = None

        self._mission_enable = False
        self._emergency_stop = True
        self._fcu_connected = False
        self._px4_armed = False
        self._px4_mode = "UNKNOWN"
        self._last_fcu_rx_monotonic: Optional[float] = None

        self._gps_fix_type = 0
        self._last_gps_fix_rx_monotonic: Optional[float] = None
        self._rtk_healthy = False
        self._last_rtk_health_rx_monotonic: Optional[float] = None
        self._rtk_correction_age_sec: Optional[float] = None
        self._last_rtk_age_rx_monotonic: Optional[float] = None
        self._backend_heartbeat_healthy = False

        self._start_stage = "IDLE"
        self._start_failed_stage: Optional[str] = None
        self._start_debug: dict[str, Any] = {}
        self._last_status_publish_monotonic = 0.0

        self._publish_safety()
        self._publish_marking_active(False)
        self._publish_mission_complete(False)
        self._publish_runtime_path()
        self._publish_status(force=True)

        self.create_timer(1.0 / self.CONTROL_HZ, self._control_loop)
        self.create_timer(1.0 / self.SAFETY_PUBLISH_HZ, self._publish_safety)

        self.get_logger().warn("===== CLEAN MISSION MANAGER STARTED =====")
        self.get_logger().warn(
            f"Marking: first <= {self.marking_tolerance_m*1000.0:.0f} mm + "
            f"speed <= {self.stationary_speed_tolerance_mps:.3f} m/s starts "
            f"a fixed {self.marking_hold_sec:.2f}s PRE-MARK verification; "
            "spray starts only if the point is still valid at the end"
        )
        self.get_logger().warn(
            "RTK mission gate: fresh fix_type=6 => RUN; FLOAT/lost/stale GPS => PAUSE. "
            "Correction age/health, backend heartbeat and spray status are monitoring only."
        )
        self.get_logger().warn(
            f"RTK correction-age monitor threshold: {self.MAX_RTK_CORRECTION_AGE_SEC:.1f}s"
        )

    # ==============================================================
    # Basic math / validation
    # ==============================================================
    def _validate_parameters(self) -> None:
        if not self.local_frame:
            raise ValueError("local_frame must not be empty")
        values = {
            "marking_tolerance_m": self.marking_tolerance_m,
            "arrival_settle_sec": self.arrival_settle_sec,
            "marking_hold_sec": self.marking_hold_sec,
            "stationary_speed_tolerance_mps": self.stationary_speed_tolerance_mps,
            "dummy_arrival_tolerance_m": self.dummy_arrival_tolerance_m,
            "waypoint_match_tolerance_m": self.waypoint_match_tolerance_m,
            "odom_timeout_sec": self.odom_timeout_sec,
            "spray_confirmation_timeout_sec": self.spray_confirmation_timeout_sec,
            "spray_status_timeout_sec": self.spray_status_timeout_sec,
        }
        for name, value in values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
        if self.maximum_navigation_points <= 0 or self.maximum_marking_points <= 0:
            raise ValueError("maximum point counts must be > 0")

    @staticmethod
    def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.hypot(b[0] - a[0], b[1] - a[1])

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _age(timestamp: Optional[float]) -> float:
        if timestamp is None:
            return math.inf
        return max(0.0, time.monotonic() - timestamp)

    def _validate_path_message(
        self, message: Path, *, label: str, maximum_points: int
    ) -> Optional[list[tuple[float, float]]]:
        frame_id = message.header.frame_id.strip()
        if frame_id != self.local_frame:
            self.get_logger().error(
                f"Rejected {label}: frame must be {self.local_frame!r}, got {frame_id!r}"
            )
            return None
        if len(message.poses) > maximum_points:
            self.get_logger().error(
                f"Rejected {label}: {len(message.poses)} > maximum {maximum_points}"
            )
            return None
        points: list[tuple[float, float]] = []
        for number, pose_stamped in enumerate(message.poses, start=1):
            pose_frame = pose_stamped.header.frame_id.strip()
            if pose_frame and pose_frame != self.local_frame:
                self.get_logger().error(
                    f"Rejected {label}: point {number} frame={pose_frame!r}"
                )
                return None
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            if not math.isfinite(x) or not math.isfinite(y):
                self.get_logger().error(f"Rejected {label}: non-finite point {number}")
                return None
            points.append((x, y))
        return points

    # ==============================================================
    # Prepared mission loading and validation
    # ==============================================================
    def _nav_path_callback(self, message: Path) -> None:
        points = self._validate_path_message(
            message,
            label="/nav_path",
            maximum_points=self.maximum_navigation_points,
        )
        if points is None:
            return
        with self._lock:
            if not points:
                self._handle_source_clear("/nav_path cleared")
                return
            self._pending_nav_path = points
            self._try_load_prepared_mission()

    def _mission_waypoints_callback(self, message: Path) -> None:
        points = self._validate_path_message(
            message,
            label="/mission_waypoints",
            maximum_points=self.maximum_marking_points,
        )
        if points is None:
            return
        with self._lock:
            if not points:
                self._handle_source_clear("/mission_waypoints cleared")
                return
            self._pending_mission_waypoints = points
            self._try_load_prepared_mission()

    def _path_types_callback(self, message: UInt8MultiArray) -> None:
        values = [int(v) for v in message.data]
        if any(v not in self.VALID_POINT_TYPES for v in values):
            self.get_logger().error("Rejected path types: invalid value")
            return
        with self._lock:
            if not values:
                self._handle_source_clear("Path types cleared")
                return
            self._pending_path_types = values
            self._try_load_prepared_mission()

    def _marking_indices_callback(self, message: Int32MultiArray) -> None:
        values = [int(v) for v in message.data]
        if any(v < -1 for v in values):
            self.get_logger().error("Rejected marking indices: values must be >= -1")
            return
        with self._lock:
            if not values:
                self._handle_source_clear("Marking indices cleared")
                return
            self._pending_marking_indices = values
            self._try_load_prepared_mission()

    def _trajectory_ready_callback(self, message: Bool) -> None:
        with self._lock:
            self._trajectory_ready = bool(message.data)
            if not self._trajectory_ready:
                self._pending_nav_path = None
                self._pending_mission_waypoints = None
                self._pending_path_types = None
                self._pending_marking_indices = None
                if self._state in {
                    "RUNNING",
                    "PAUSED",
                    "WAITING_FOR_NEXT",
                }:
                    self._enter_error("Trajectory readiness lost during active mission")
                return
            self._try_load_prepared_mission()

    def _handle_source_clear(self, reason: str) -> None:
        self._trajectory_ready = False
        self._pending_nav_path = None
        self._pending_mission_waypoints = None
        self._pending_path_types = None
        self._pending_marking_indices = None
        if self._state in {"RUNNING", "PAUSED", "WAITING_FOR_NEXT"}:
            self._enter_error(f"Prepared trajectory cleared while active: {reason}")
            return
        self._clear_loaded_runtime(reason)

    def _try_load_prepared_mission(self) -> None:
        if not self._trajectory_ready:
            return
        pending = (
            self._pending_nav_path,
            self._pending_mission_waypoints,
            self._pending_path_types,
            self._pending_marking_indices,
        )
        if any(value is None for value in pending):
            return

        navigation_path = list(self._pending_nav_path or [])
        mission_waypoints = list(self._pending_mission_waypoints or [])
        path_types = list(self._pending_path_types or [])
        marking_indices = list(self._pending_marking_indices or [])

        if self._state in {"RUNNING", "PAUSED", "WAITING_FOR_NEXT"}:
            self.get_logger().error("Rejected replacement mission while active")
            return
        if not navigation_path or not mission_waypoints:
            self._enter_error("Prepared mission contains no points")
            return
        if not (len(navigation_path) == len(path_types) == len(marking_indices)):
            self.get_logger().warn(
                "Prepared snapshot incomplete: "
                f"path={len(navigation_path)} types={len(path_types)} "
                f"indices={len(marking_indices)} markings={len(mission_waypoints)}"
            )
            return

        marking_path_index_by_number = [-1] * len(mission_waypoints)
        semantic_indices: list[int] = []

        for path_index, (point_type, marking_index) in enumerate(
            zip(path_types, marking_indices)
        ):
            if point_type in {self.POINT_DUMMY_ALIGNMENT, self.POINT_MARKING}:
                semantic_indices.append(path_index)

            if point_type == self.POINT_MARKING:
                if not (0 <= marking_index < len(mission_waypoints)):
                    self._enter_error("Marking path point has invalid CSV index")
                    return
                if marking_path_index_by_number[marking_index] != -1:
                    self._enter_error(
                        "CSV marking point appears more than once in /nav_path"
                    )
                    return
                if (
                    self._distance(
                        navigation_path[path_index], mission_waypoints[marking_index]
                    )
                    > self.waypoint_match_tolerance_m
                ):
                    self._enter_error(
                        "Marking path coordinate does not match /mission_waypoints"
                    )
                    return
                marking_path_index_by_number[marking_index] = path_index
            elif marking_index != -1:
                self._enter_error("Navigation-only point contains CSV marking index")
                return

        if any(i < 0 for i in marking_path_index_by_number):
            self._enter_error("Prepared path does not contain every CSV marking point")
            return
        if marking_path_index_by_number != sorted(marking_path_index_by_number):
            self._enter_error("CSV marking order is not preserved in prepared path")
            return
        if not semantic_indices:
            self._enter_error("Prepared path contains no semantic goals")
            return

        self._navigation_path = navigation_path
        self._mission_waypoints = mission_waypoints
        self._path_types = path_types
        self._marking_indices = marking_indices
        self._marking_path_index_by_number = marking_path_index_by_number
        self._semantic_path_indices = semantic_indices
        self._point_status = ["PENDING"] * len(mission_waypoints)
        self._point_accuracy_snapshots = [None] * len(mission_waypoints)
        self._point_results = [None] * len(mission_waypoints)
        self._last_point_result = None
        self._reset_execution_progress()
        self._state = "READY"
        self._last_error = None
        self._last_message = "Prepared mission validated and ready"
        self._publish_marking_active(False)
        self._publish_mission_complete(False)
        self._publish_runtime_path()
        self._publish_goal()
        self._publish_status(force=True)

        self.get_logger().warn("===== PREPARED MISSION LOADED =====")
        self.get_logger().warn(f"Navigation points : {len(navigation_path)}")
        self.get_logger().warn(f"Semantic goals    : {len(semantic_indices)}")
        self.get_logger().warn(f"Marking points    : {len(mission_waypoints)}")

    # ==============================================================
    # Rover / health callbacks
    # ==============================================================
    def _odom_callback(self, message: Odometry) -> None:
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        vx = float(message.twist.twist.linear.x)
        vy = float(message.twist.twist.linear.y)
        with self._lock:
            self._x = float(p.x)
            self._y = float(p.y)
            self._yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
            self._speed_mps = math.hypot(vx, vy)
            self._last_odom_monotonic = time.monotonic()

    def _mavros_state_callback(self, message: State) -> None:
        with self._lock:
            self._fcu_connected = bool(message.connected)
            self._px4_armed = bool(message.armed)
            self._px4_mode = str(message.mode or "UNKNOWN").strip().upper()
            self._last_fcu_rx_monotonic = time.monotonic()

    def _gps_status_callback(self, message: GPSRAW) -> None:
        with self._lock:
            self._gps_fix_type = int(message.fix_type)
            self._last_gps_fix_rx_monotonic = time.monotonic()

    def _rtk_health_callback(self, message: Bool) -> None:
        with self._lock:
            self._rtk_healthy = bool(message.data)
            self._last_rtk_health_rx_monotonic = time.monotonic()

    def _rtk_correction_age_callback(self, message: Float32) -> None:
        with self._lock:
            age = float(message.data)
            self._rtk_correction_age_sec = age if math.isfinite(age) else None
            self._last_rtk_age_rx_monotonic = time.monotonic()

    def _backend_heartbeat_health_callback(self, message: Bool) -> None:
        with self._lock:
            self._backend_heartbeat_healthy = bool(message.data)

    def _execution_mode_callback(self, message: String) -> None:
        mode = str(message.data or "").strip().upper()
        if mode not in {"AUTO", "MANUAL"}:
            self.get_logger().warn(f"Ignoring invalid execution mode {mode!r}")
            return
        with self._lock:
            if self._state in {
                "RUNNING",
                "PAUSED",
                "WAITING_FOR_NEXT",
            }:
                self.get_logger().warn(
                    "Execution mode cannot change during active mission"
                )
                return
            self._execution_mode = mode
            self._last_message = f"Execution mode set to {mode}"
            self._publish_status(force=True)

    def _rpp_terminal_result_callback(self, message: String) -> None:
        """Receive RPP CAPTURED/MISSED for the active goal.

        CAPTURED does not start the 3 s hold. Mission Manager starts that
        hold only when it measures <=30 mm and stationary.
        MISSED, while stationary and outside 30 mm, fails the point now
        with no 3 s hold.
        """
        try:
            payload = json.loads(message.data)
        except Exception:
            return
        if not isinstance(payload, dict):
            return

        outcome = str(payload.get("outcome", "")).strip().upper()
        if outcome not in {"CAPTURED", "MISSED"}:
            return
        try:
            goal_x = float(payload.get("goal_x"))
            goal_y = float(payload.get("goal_y"))
        except (TypeError, ValueError):
            return
        if not all(math.isfinite(v) for v in (goal_x, goal_y)):
            return

        with self._lock:
            if not self._navigation_path:
                return
            if not (0 <= self._current_path_index < len(self._navigation_path)):
                return
            current = self._navigation_path[self._current_path_index]
            if math.hypot(goal_x - current[0], goal_y - current[1]) > 0.005:
                return
            self._rpp_terminal_result = copy.deepcopy(payload)
            self._rpp_terminal_result_last_rx_monotonic = time.monotonic()

    # ==============================================================
    # Spray controller handshake
    # ==============================================================
    def _spray_status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        with self._lock:
            self._spray_status_last_rx_monotonic = time.monotonic()
            timestamp_ns = payload.get("timestamp_unix_ns")
            self._spray_status_timestamp_unix_ns = (
                int(timestamp_ns) if isinstance(timestamp_ns, int) else None
            )
            self._spray_controller_ready = bool(payload.get("ready", False))
            state = payload.get("controller_state")
            self._spray_controller_state = str(state) if state is not None else None
            reason = payload.get("fault_reason")
            self._spray_fault_reason = str(reason) if reason else None

    def _spray_result_callback(self, message: String) -> None:
        """Monitor spray result without blocking rover path progression."""
        try:
            payload = json.loads(message.data)
        except Exception:
            return
        if not isinstance(payload, dict):
            return

        run_id = payload.get("mission_run_id")
        point_id = payload.get("point_id")
        result = str(payload.get("result", "")).upper()
        if not isinstance(run_id, str) or not run_id:
            return
        if not isinstance(point_id, str) or not point_id:
            return

        with self._lock:
            if run_id != self._mission_run_id:
                return

            key = (run_id, point_id)
            if result == "SUCCESS":
                self._spray_success_keys.add(key)
                self._spray_failure_keys.pop(key, None)
                self._last_message = (
                    f"Spray SUCCESS monitored for {point_id}; "
                    "mission progression is independent of spray result"
                )
                self._publish_status(force=True)
                return

            if result == "FAILED":
                reason = str(payload.get("reason", "UNKNOWN_SPRAY_FAILURE"))
                self._spray_failure_keys[key] = reason
                self._spray_success_keys.discard(key)
                self._last_message = (
                    f"Spray FAILED monitored for {point_id}: {reason}; "
                    "mission progression is not blocked"
                )
                self._emit_system_event(
                    "SPRAY_FAILED_MONITOR_ONLY",
                    self._last_message,
                )
                self._publish_status(force=True)

    def _spray_status_age_sec(self) -> float:
        if self._spray_status_last_rx_monotonic is None:
            return math.inf
        rx_age = self._age(self._spray_status_last_rx_monotonic)
        ts = self._spray_status_timestamp_unix_ns
        if ts is None:
            return rx_age
        wall_delta = (time.time_ns() - ts) / 1.0e9
        if wall_delta < -1.0:
            return math.inf
        return max(rx_age, max(0.0, wall_delta))

    def _spray_ready_for_mission_start(self) -> bool:
        if not self.spray_required:
            return True
        return (
            self._spray_controller_ready
            and self._spray_fault_reason is None
            and self._spray_status_age_sec() <= self.spray_status_timeout_sec
        )

    # ==============================================================
    # RTK safety
    # ==============================================================
    def _rtk_state(self) -> str:
        if self._gps_fix_type == 6:
            return "FIXED"
        if self._gps_fix_type == 5:
            return "FLOAT"
        if self._gps_fix_type >= 3:
            return "3D"
        if self._gps_fix_type == 2:
            return "2D"
        return "NO_FIX"

    def _rtk_motion_ok(self) -> tuple[bool, str]:
        """Allow mission motion only with a fresh MAVROS GPSRAW fix_type == 6.

        RTK correction-health and correction-age are still monitored and
        reported, but they do not independently stop the rover while the
        receiver/PX4 remains RTK FIXED.
        """
        gps_age = self._age(self._last_gps_fix_rx_monotonic)
        if gps_age > self.GPS_FIX_STALE_SEC:
            return False, f"RTK GPS fix status stale ({gps_age:.2f}s)"

        if self._gps_fix_type != 6:
            return False, (
                f"RTK {self._rtk_state()} " f"(fix_type={self._gps_fix_type})"
            )

        monitor_notes: list[str] = []

        health_age = self._age(self._last_rtk_health_rx_monotonic)
        if health_age > self.RTK_STATUS_STALE_SEC:
            monitor_notes.append(f"correction-health status stale {health_age:.1f}s")
        elif not self._rtk_healthy:
            monitor_notes.append("correction stream unhealthy")

        age_status_age = self._age(self._last_rtk_age_rx_monotonic)
        if age_status_age > self.RTK_STATUS_STALE_SEC:
            monitor_notes.append(f"correction-age status stale {age_status_age:.1f}s")

        correction_age = self._rtk_correction_age_sec
        if correction_age is None or correction_age < 0.0:
            monitor_notes.append("correction age unavailable")
        elif correction_age > self.MAX_RTK_CORRECTION_AGE_SEC:
            monitor_notes.append(
                f"correction age {correction_age:.1f}s > "
                f"{self.MAX_RTK_CORRECTION_AGE_SEC:.1f}s"
            )

        if monitor_notes:
            return (
                True,
                "RTK FIXED (fix_type=6); monitor only: " + "; ".join(monitor_notes),
            )

        if correction_age is None:
            return True, "RTK FIXED (fix_type=6)"

        return (
            True,
            f"RTK FIXED (fix_type=6); correction age={correction_age:.1f}s",
        )

    def _monitor_runtime_rtk(self) -> None:
        """Pause a RUNNING mission only when RTK FIXED is actually lost.

        Fresh fix_type=6 keeps running. FLOAT, lower fix types, or stale GPSRAW
        disable motion and pause. Correction-age/health remain monitor-only.
        """
        ok, reason = self._rtk_motion_ok()
        if self._state == "RUNNING" and not ok:
            self._mission_enable = False
            self._pause_reason = "RTK_LOST"
            self._resume_available = False
            self._state = "PAUSED"
            self._reset_arrival_state()
            self._publish_marking_active(False)
            self._publish_safety()
            self._last_message = f"{reason}; rover stopped and mission paused"
            self._emit_system_event("RTK_PAUSED", reason)
            self._publish_status(force=True)

    def _monitor_runtime_px4_control(self) -> None:
        """Stop mission motion if PX4 unexpectedly leaves OFFBOARD or disarms.

        This monitor NEVER forces PX4 back to OFFBOARD and NEVER requests
        MANUAL. It only removes Jetson motion/spray permission and pauses.
        """
        if self._state != "RUNNING":
            return

        state_age = self._age(self._last_fcu_rx_monotonic)
        reason: Optional[str] = None

        if state_age > self.FCU_STATE_STALE_SEC:
            reason = f"PX4 state stale ({state_age:.2f}s)"
        elif not self._fcu_connected:
            reason = "PX4 disconnected"
        elif self._px4_mode != "OFFBOARD":
            reason = f"PX4 mode changed to {self._px4_mode}"
        elif not self._px4_armed:
            reason = "PX4 unexpectedly disarmed"

        if reason is None:
            return

        self._mission_enable = False
        self._pause_reason = "PX4_CONTROL_LOST"
        self._resume_available = False
        self._state = "PAUSED"
        self._reset_arrival_state()
        self._publish_marking_active(False)
        self._publish_safety()
        self._last_message = (
            f"{reason}; mission paused. Mission Manager did not change PX4 mode"
        )
        self._emit_system_event("PX4_CONTROL_LOST", self._last_message)
        self._publish_status(force=True)

    def _motion_health_status(
        self, *, require_ready_state: bool, require_estop_released: bool = False
    ) -> tuple[bool, str]:
        """Simple Mission Manager movement gate for rover behavior testing.

        Blocking:
          * validated prepared mission exists;
          * lifecycle state is correct;
          * E-stop is released when movement is being re-enabled;
          * fresh MAVROS GPSRAW fix_type == 6.

        Monitoring only here:
          * RTK correction-health and correction-age;
          * backend heartbeat;
          * spray readiness/fault;
          * FCU freshness precheck;
          * odometry freshness precheck.

        PX4 still must accept OFFBOARD and ARM service commands.
        """
        if not self._navigation_path or not self._trajectory_ready:
            return False, "No validated prepared mission is ready"

        if require_ready_state and self._state != "READY":
            return False, f"Mission manager is not READY (state={self._state})"

        if require_estop_released and self._emergency_stop:
            return (
                False,
                "Emergency stop is active; release E-stop before enabling motion",
            )

        rtk_ok, rtk_reason = self._rtk_motion_ok()
        if not rtk_ok:
            return False, f"RTK FIXED required: {rtk_reason}"

        return True, (
            "Prepared mission + fresh RTK FIXED; "
            "remaining Mission Manager health fields are monitoring only"
        )

    def _monitor_pause_recovery(self) -> None:
        """Advertise Resume when the condition that caused the pause recovers."""
        if self._state != "PAUSED":
            return

        if self._pause_reason == "ESTOP" and self._emergency_stop:
            if self._resume_available:
                self._resume_available = False
                self._publish_status(force=True)
            return

        # Keep the existing runtime odometry protection: waypoint arrival
        # cannot be evaluated correctly from a frozen local position.
        if self._pause_reason == "ODOM_STALE":
            if self._age(self._last_odom_monotonic) > self.odom_timeout_sec:
                if self._resume_available:
                    self._resume_available = False
                    self._publish_status(force=True)
                return

        ok, reason = self._motion_health_status(
            require_ready_state=False,
            require_estop_released=True,
        )
        if not ok:
            if self._resume_available:
                self._resume_available = False
                self._last_message = (
                    f"Mission remains paused; Resume unavailable: {reason}"
                )
                self._publish_status(force=True)
            return

        if self._resume_available:
            return

        self._resume_available = True

        if self._pause_reason == "RTK_LOST":
            self._last_message = (
                "RTK FIXED (fix_type=6) recovered; mission remains paused. "
                "You can resume now"
            )
            self._emit_system_event(
                "RTK_RECOVERED",
                "Fresh fix_type=6 restored - ready to resume",
            )
        elif self._pause_reason == "ODOM_STALE":
            self._last_message = (
                "Local odometry recovered and RTK is FIXED; mission remains paused. "
                "You can resume now"
            )
        elif self._pause_reason == "ESTOP":
            self._last_message = (
                "Emergency stop released and RTK is FIXED; mission remains paused. "
                "You can resume now"
            )
        else:
            self._last_message = (
                "Mission remains paused; RTK is FIXED. You can resume now"
            )

        self._publish_status(force=True)

    def _publish_safety(self) -> None:
        m = Bool()
        m.data = bool(self._mission_enable)
        self.mission_enable_pub.publish(m)
        e = Bool()
        e.data = bool(self._emergency_stop)
        self.emergency_stop_pub.publish(e)

    def _set_safety(self, *, emergency_stop: bool, mission_enable: bool) -> None:
        self._emergency_stop = bool(emergency_stop)
        self._mission_enable = bool(mission_enable) and not self._emergency_stop
        if not self._mission_enable:
            self._publish_marking_active(False)
        self._publish_safety()

    def _require_motion_health(
        self, *, require_ready_state: bool, require_estop_released: bool = False
    ) -> None:
        ok, reason = self._motion_health_status(
            require_ready_state=require_ready_state,
            require_estop_released=require_estop_released,
        )
        if not ok:
            raise RuntimeError(reason)

    def _call_service(self, client: Any, request: Any, label: str) -> Any:
        if not client.wait_for_service(timeout_sec=self.SERVICE_DISCOVERY_TIMEOUT_SEC):
            raise RuntimeError(f"{label} service unavailable")
        future = client.call_async(request)
        deadline = time.monotonic() + self.SERVICE_RESPONSE_TIMEOUT_SEC
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError(f"{label} service timeout")
        result = future.result()
        if result is None:
            raise RuntimeError(f"{label} returned no response")
        return result

    def _request_px4_mode(self, mode: str) -> None:
        request = SetMode.Request()
        request.custom_mode = mode
        result = self._call_service(self._mode_client, request, f"set_mode({mode})")
        if not bool(result.mode_sent):
            raise RuntimeError(f"PX4 rejected mode {mode}")

    def _request_arm(self, arm: bool) -> None:
        request = CommandBool.Request()
        request.value = bool(arm)
        result = self._call_service(
            self._arming_client, request, "arming" if arm else "disarming"
        )
        if not bool(result.success):
            raise RuntimeError("PX4 rejected arm" if arm else "PX4 rejected disarm")

    def _wait_for_vehicle_state(
        self,
        *,
        expected_mode: Optional[str] = None,
        expected_armed: Optional[bool] = None,
    ) -> None:
        deadline = time.monotonic() + self.VEHICLE_STATE_CONFIRM_TIMEOUT_SEC
        expected_mode = expected_mode.upper() if expected_mode else None
        while rclpy.ok() and time.monotonic() < deadline:
            mode_ok = expected_mode is None or self._px4_mode == expected_mode
            arm_ok = expected_armed is None or self._px4_armed == expected_armed
            if mode_ok and arm_ok:
                return
            time.sleep(0.02)
        raise RuntimeError(
            f"PX4 state confirmation timeout: mode={self._px4_mode}, armed={self._px4_armed}"
        )

    def _best_effort_px4_disarm_only(self) -> tuple[bool, list[str]]:
        """Best-effort DISARM without changing PX4 flight mode.

        Mission Manager is never allowed to request MANUAL. Mode changes after
        STOP belong to the operator/RC or PX4 itself.
        """
        warnings: list[str] = []
        try:
            if self._px4_armed:
                self._request_arm(False)
                self._wait_for_vehicle_state(expected_armed=False)
        except Exception as exc:
            warnings.append(f"disarm warning: {exc}")
        # A service ACK (or the default False value before any state message)
        # is not evidence. Only a fresh, connected MAVROS state confirming
        # armed=False authorizes a "disarmed" claim.
        state_age = self._age(self._last_fcu_rx_monotonic)
        disarm_confirmed = (
            self._fcu_connected
            and state_age <= self.FCU_STATE_STALE_SEC
            and not self._px4_armed
        )
        if not disarm_confirmed:
            warnings.append(
                "disarm confirmation unavailable: "
                f"connected={self._fcu_connected}, armed={self._px4_armed}, "
                f"state_age_sec={state_age:.2f}"
            )
        return disarm_confirmed, warnings

    def _clear_after_stop(self, *, completed: bool, message: str) -> None:
        """Clear all Mission Manager mission/runtime state after STOP.

        When completed=True, /mission_complete remains latched TRUE until the
        next prepared mission is loaded. For an operator STOP it is FALSE.
        """
        self._trajectory_ready = False
        self._pending_nav_path = None
        self._pending_mission_waypoints = None
        self._pending_path_types = None
        self._pending_marking_indices = None

        self._navigation_path = []
        self._mission_waypoints = []
        self._path_types = []
        self._marking_indices = []
        self._marking_path_index_by_number = []
        self._semantic_path_indices = []
        self._point_status = []
        self._point_accuracy_snapshots = []
        self._point_results = []
        self._last_point_result = None

        self._current_path_index = 0
        self._active_marking_number = None
        self._last_completed_marking_number = None
        self._mission_run_id = ""
        self._mission_started_unix_ns = None

        self._spray_success_keys.clear()
        self._spray_failure_keys.clear()

        self._pause_reason = None
        self._resume_available = False
        self._last_error = None
        self._state = "EMPTY"
        self._start_stage = "IDLE"
        self._start_failed_stage = None
        self._start_debug = {}
        self._auto_stop_pending = False

        self._reset_arrival_state()
        self._last_message = message

        self._publish_marking_active(False)
        self._publish_runtime_path()
        self._publish_safety()
        if not completed:
            self._publish_mission_complete(False)
        self._publish_status(force=True)

    def _execute_stop_cleanup(
        self, *, completed: bool, source: str
    ) -> tuple[bool, list[str]]:
        """Physical STOP contract shared by operator STOP and auto-completion.

        Contract:
          mission_enable = false
          emergency_stop = true
          marking_active = false
          PX4 DISARM
          PX4 MODE UNCHANGED BY MISSION MANAGER
          clear Mission Manager runtime -> EMPTY only after DISARM confirmation
        """
        self._reset_point_timers()
        self._set_safety(emergency_stop=True, mission_enable=False)
        self._publish_marking_active(False)

        if completed:
            self._emit_system_event(
                "MISSION_AUTO_STOP",
                "All uploaded points processed; automatic STOP started",
            )
        else:
            self._emit_system_event(
                "MISSION_STOPPED",
                "Operator STOP requested before normal mission completion",
            )

        disarm_confirmed, warnings = self._best_effort_px4_disarm_only()

        with self._lock:
            if completed:
                if disarm_confirmed:
                    message = (
                        "Mission completed; automatic STOP executed; PX4 disarm "
                        "confirmed; PX4 mode unchanged by Mission Manager"
                    )
                else:
                    message = (
                        "Mission completed; automatic STOP executed; PX4 DISARM "
                        "NOT CONFIRMED; hard stop remains asserted; PX4 mode "
                        "unchanged by Mission Manager"
                    )
            else:
                if disarm_confirmed:
                    message = (
                        "Mission stopped by operator; PX4 disarm confirmed; "
                        "PX4 mode unchanged by Mission Manager"
                    )
                else:
                    message = (
                        "Mission stopped by operator; PX4 DISARM NOT CONFIRMED; "
                        "hard stop remains asserted; PX4 mode unchanged by "
                        "Mission Manager"
                    )
            if not disarm_confirmed:
                # The mission is not terminated until MAVROS confirms DISARM.
                # Retain all runtime/results so STOP can be retried and the
                # failure can be diagnosed while the hard stop stays asserted.
                if not completed:
                    self._state = "ERROR"
                self._last_error = message
                self._last_message = message + "; press STOP to retry disarm"
                self._emit_system_event(
                    "MISSION_TERMINATION_BLOCKED",
                    self._last_message,
                )
                self._publish_status(force=True)
                return False, warnings

            self._emit_mission_terminated(
                completed=completed,
                source=source,
                disarm_confirmed=True,
                warnings=warnings,
                message=message,
            )
            self._clear_after_stop(completed=completed, message=message)

        return True, warnings

    # ==============================================================
    # Services
    # ==============================================================
    def _start_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self._start_stage = "PRECHECK"
        self._start_failed_stage = None
        try:
            with self._lock:
                if self._state == "RUNNING":
                    raise RuntimeError(
                        "Previous mission is still RUNNING; STOP the current mission before START"
                    )
                if self._state == "PAUSED":
                    raise RuntimeError("Mission paused; use Resume")
                if self._state == "WAITING_FOR_NEXT":
                    raise RuntimeError("Manual mission waiting for NEXT")
                if self._state == "ERROR":
                    raise RuntimeError("Mission in ERROR; clear/prepare again")
                if self._px4_armed:
                    raise RuntimeError(
                        "START requires PX4 disarmed so sequence is OFFBOARD then ARM"
                    )
            self._require_motion_health(require_ready_state=True)

            self._start_stage = "ZERO_SETPOINT_SETTLE"
            self._set_safety(emergency_stop=False, mission_enable=False)
            time.sleep(self.OFFBOARD_STREAM_SETTLE_SEC)

            self._start_stage = "SWITCHING_OFFBOARD"
            if self._px4_mode != "OFFBOARD":
                self._request_px4_mode("OFFBOARD")

            # FIRST: confirm OFFBOARD while still disarmed.
            self._wait_for_vehicle_state(
                expected_mode="OFFBOARD",
                expected_armed=False,
            )

            # Keep the existing zero PositionTarget stream active while PX4
            # settles in OFFBOARD. Only after this do we send ARM.
            time.sleep(self.OFFBOARD_BEFORE_ARM_SETTLE_SEC)

            if self._px4_mode != "OFFBOARD":
                raise RuntimeError(
                    f"PX4 left OFFBOARD before ARM (mode={self._px4_mode})"
                )

            self._start_stage = "ARMING"
            self._request_arm(True)

            # SECOND: mission cannot run until BOTH are confirmed.
            self._wait_for_vehicle_state(
                expected_mode="OFFBOARD",
                expected_armed=True,
            )

            self._start_stage = "FINAL_CHECK"
            self._require_motion_health(require_ready_state=True)

            with self._lock:
                self._point_status = ["PENDING"] * len(self._mission_waypoints)
                self._point_accuracy_snapshots = [None] * len(self._mission_waypoints)
                self._point_results = [None] * len(self._mission_waypoints)
                self._last_point_result = None
                self._last_termination_report = None
                self._reset_execution_progress()
                self._mission_run_id = uuid.uuid4().hex
                self._mission_started_unix_ns = time.time_ns()
                self._spray_success_keys.clear()
                self._spray_failure_keys.clear()
                self._pause_reason = None
                self._resume_available = False
                self._state = "RUNNING"
                self._last_error = None
                self._last_message = f"Mission started in {self._execution_mode} mode"
                self._start_stage = "RUNNING"
                self._publish_mission_complete(False)
                self._publish_runtime_path()
                self._publish_goal()
            self._set_safety(emergency_stop=False, mission_enable=True)
            self._publish_status(force=True)
            response.success = True
            response.message = self._last_message
            return response
        except Exception as exc:
            failed_stage = self._start_stage
            self._set_safety(emergency_stop=True, mission_enable=False)
            cleanup_disarm_confirmed, cleanup = self._best_effort_px4_disarm_only()
            with self._lock:
                if (
                    self._navigation_path
                    and self._trajectory_ready
                    and self._state != "ERROR"
                ):
                    self._state = "READY"
                self._last_error = str(exc)
                self._last_message = f"Start blocked at {failed_stage}: {exc}"
                self._start_failed_stage = failed_stage
                self._start_stage = "FAILED"
                self._start_debug = {
                    "cleanup_disarm_confirmed": cleanup_disarm_confirmed,
                    "cleanup_warnings": cleanup,
                }
                self._publish_status(force=True)
            response.success = False
            response.message = self._last_message
            return response

    def _pause_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._lock:
            if self._state != "RUNNING":
                response.success = False
                response.message = "Only RUNNING mission can be paused"
                return response
            self._pause_reason = "OPERATOR"
            self._resume_available = True
            self._state = "PAUSED"
            self._reset_point_timers()
            self._set_safety(emergency_stop=False, mission_enable=False)
            self._last_message = "Mission paused by operator; progress preserved"
            self._publish_goal()
            self._publish_status(force=True)
        response.success = True
        response.message = "Mission paused"
        return response

    def _resume_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        try:
            with self._lock:
                if self._state != "PAUSED":
                    response.success = False
                    response.message = "Mission is not paused"
                    return response
            self._require_motion_health(
                require_ready_state=False, require_estop_released=True
            )
            self._set_safety(emergency_stop=False, mission_enable=False)
            time.sleep(self.OFFBOARD_STREAM_SETTLE_SEC)
            if self._px4_mode != "OFFBOARD":
                self._request_px4_mode("OFFBOARD")
            self._wait_for_vehicle_state(expected_mode="OFFBOARD")
            if not self._px4_armed:
                self._request_arm(True)
            self._wait_for_vehicle_state(expected_mode="OFFBOARD", expected_armed=True)
            self._require_motion_health(
                require_ready_state=False, require_estop_released=True
            )
            with self._lock:
                self._pause_reason = None
                self._resume_available = False
                self._state = "RUNNING"
                self._last_error = None
                self._last_message = "Mission resumed"
                self._reset_arrival_state()
                self._publish_goal()
            self._set_safety(emergency_stop=False, mission_enable=True)
            self._publish_status(force=True)
            response.success = True
            response.message = "Mission resumed"
            return response
        except Exception as exc:
            with self._lock:
                self._mission_enable = False
                self._resume_available = False
                self._last_message = f"Resume blocked: {exc}"
                self._publish_safety()
                self._publish_status(force=True)
            response.success = False
            response.message = self._last_message
            return response

    def _next_point_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Advance MANUAL mission after prepared-path + RTK FIXED gate passes."""
        try:
            with self._lock:
                if self._execution_mode != "MANUAL":
                    response.success = False
                    response.message = "NEXT is only used in MANUAL execution mode"
                    return response
                if self._state != "WAITING_FOR_NEXT":
                    response.success = False
                    response.message = "Mission is not waiting for NEXT"
                    return response
                if self._next_pending_marking_number(0) is None:
                    self._complete_mission()
                    response.success = True
                    response.message = "Mission completed"
                    return response

            # NEXT can re-enable movement, so it must pass the same simple
            # prepared-path + RTK FIXED gate used by RESUME.
            self._require_motion_health(
                require_ready_state=False, require_estop_released=True
            )
            self._set_safety(emergency_stop=False, mission_enable=False)
            time.sleep(self.OFFBOARD_STREAM_SETTLE_SEC)

            if self._px4_mode != "OFFBOARD":
                self._request_px4_mode("OFFBOARD")
            self._wait_for_vehicle_state(expected_mode="OFFBOARD")

            if not self._px4_armed:
                self._request_arm(True)
            self._wait_for_vehicle_state(expected_mode="OFFBOARD", expected_armed=True)

            # Re-check RTK FIXED immediately before enabling motion.
            self._require_motion_health(
                require_ready_state=False, require_estop_released=True
            )

            with self._lock:
                # State may have changed while service calls were in progress.
                if self._state != "WAITING_FOR_NEXT":
                    raise RuntimeError(
                        f"Mission state changed during NEXT (state={self._state})"
                    )
                self._state = "RUNNING"
                self._pause_reason = None
                self._resume_available = False
                self._last_error = None
                self._last_message = "Manual NEXT accepted; proceeding to next point"
                self._reset_arrival_state()
                self._publish_goal()

            self._set_safety(emergency_stop=False, mission_enable=True)
            self._publish_status(force=True)
            response.success = True
            response.message = "Next point enabled"
            return response

        except Exception as exc:
            with self._lock:
                # NEXT failure is a soft block: preserve WAITING_FOR_NEXT and
                # never turn motion on. Operator can fix health and press NEXT
                # again; no CLEAR or mission restart is required.
                self._mission_enable = False
                self._resume_available = False
                self._last_error = None
                self._last_message = f"NEXT blocked: {exc}"
                self._publish_safety()
                self._publish_status(force=True)
            response.success = False
            response.message = self._last_message
            return response

    def _skip_point_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Resolve the current point as SKIPPED, including an active point."""
        with self._lock:
            if self._state not in {
                "RUNNING",
                "PAUSED",
                "WAITING_FOR_NEXT",
            }:
                response.success = False
                response.message = "Mission is not active"
                return response

            previous_state = self._state
            previous_pause_reason = self._pause_reason
            previous_resume_available = self._resume_available

            marking_number = self._current_or_next_marking_number()
            if marking_number is None:
                response.success = False
                response.message = "No marking point available to skip"
                return response
            if self._point_status[marking_number] in self.TERMINAL_POINT_STATES:
                response.success = False
                response.message = "Current point already terminal"
                return response

            path_index = self._marking_path_index_by_number[marking_number]
            # SKIP removes spray permission immediately, even if requested
            # while the current point transaction is active.
            self._publish_marking_active(False)
            accuracy = self._capture_accuracy_snapshot(
                marking_number=marking_number,
                path_index=path_index,
                outcome="NOT_EVALUATED",
                reason="OPERATOR_SKIP",
            )
            point_id = f"P{marking_number+1:04d}"
            spray_key = (self._mission_run_id, point_id)
            if spray_key in self._spray_success_keys:
                spray = self._spray_outcome_snapshot(
                    attempted=True,
                    outcome="SUCCESS",
                )
            elif spray_key in self._spray_failure_keys:
                spray = self._spray_outcome_snapshot(
                    attempted=True,
                    outcome="FAILED",
                    reason=self._spray_failure_keys[spray_key],
                )
            elif self._spray_request_started is not None:
                spray = self._spray_outcome_snapshot(
                    attempted=True,
                    outcome="ABORTED_BY_SKIP",
                    reason="OPERATOR_SKIP",
                )
            else:
                spray = self._spray_outcome_snapshot(
                    attempted=False,
                    outcome="NOT_ATTEMPTED_SKIPPED",
                    reason="OPERATOR_SKIP",
                )

            self._point_status[marking_number] = "SKIPPED"
            point_result = self._store_point_result(
                marking_number=marking_number,
                path_index=path_index,
                point_outcome="SKIPPED",
                accuracy=accuracy,
                spray=spray,
            )
            self._emit_point_event(
                "SKIPPED",
                marking_number,
                path_index,
                reason="OPERATOR_SKIP",
                accuracy=accuracy,
                spray=spray,
                point_result=point_result,
            )
            if self._current_path_index <= path_index:
                self._current_path_index = self._next_semantic_index(path_index + 1)

            # This is an actual point advance, so live per-point working state
            # can now be reset. Immutable snapshots/results remain in history.
            self._reset_arrival_state()
            self._publish_marking_active(False)

            if self._finish_if_done():
                response.success = True
                response.message = f"P{marking_number+1:04d} skipped; mission completed"
                return response

            if self._execution_mode == "MANUAL":
                self._state = "WAITING_FOR_NEXT"
                self._pause_reason = None
                self._resume_available = False
                # SKIP selects the next point; it must never release an
                # independently asserted emergency-stop latch.
                self._set_safety(
                    emergency_stop=self._emergency_stop,
                    mission_enable=False,
                )
                self._last_message = (
                    f"{point_id} skipped; next point selected with motion "
                    "disabled; waiting for NEXT"
                )
            elif previous_state == "PAUSED":
                # Preserve the reason that caused the pause.
                self._state = "PAUSED"
                self._pause_reason = previous_pause_reason
                self._resume_available = previous_resume_available
                self._set_safety(
                    emergency_stop=self._emergency_stop,
                    mission_enable=False,
                )
                self._last_message = f"{point_id} skipped; mission remains paused"
            elif previous_state == "WAITING_FOR_NEXT":
                self._state = "WAITING_FOR_NEXT"
                self._set_safety(emergency_stop=False, mission_enable=False)
                self._last_message = f"{point_id} skipped; waiting for NEXT"
            else:
                # AUTO SKIP resumes the next point immediately when the control
                # contract is healthy; otherwise it remains safely PAUSED.
                rtk_ok, rtk_reason = self._rtk_motion_ok()
                if (
                    rtk_ok
                    and self._fcu_connected
                    and self._px4_mode == "OFFBOARD"
                    and self._px4_armed
                ):
                    self._state = "RUNNING"
                    self._pause_reason = None
                    self._resume_available = False
                    self._set_safety(
                        emergency_stop=False,
                        mission_enable=True,
                    )
                    self._last_message = f"{point_id} skipped; continuing automatically"
                else:
                    self._state = "PAUSED"
                    self._pause_reason = "SKIP_HEALTH_BLOCK"
                    self._resume_available = False
                    self._set_safety(
                        emergency_stop=False,
                        mission_enable=False,
                    )
                    self._last_message = (
                        f"P{marking_number+1:04d} skipped, but next-point motion "
                        f"is paused: RTK={rtk_reason}, mode={self._px4_mode}, "
                        f"armed={self._px4_armed}"
                    )

            self._publish_goal()
            self._publish_runtime_path()
            self._publish_status(force=True)

        response.success = True
        response.message = self._last_message
        return response

    def _stop_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Operator STOP at any point in the mission.

        STOP never requests MANUAL. It disables mission motion, asserts the
        hard-stop output, and requests PX4 DISARM. Runtime is cleared only
        after MAVROS confirms the disarm; PX4 mode ownership remains with the
        RC/operator/PX4.
        """
        was_completed = self._state == "COMPLETED"
        terminated, warnings = self._execute_stop_cleanup(
            completed=was_completed,
            source="OPERATOR",
        )

        response.success = terminated
        response.message = self._last_message
        if warnings:
            response.message += "; " + "; ".join(warnings)
        return response

    def _clear_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._lock:
            if self._state in {
                "RUNNING",
                "PAUSED",
                "WAITING_FOR_NEXT",
            }:
                response.success = False
                response.message = "Stop mission before Clear"
                return response
            self._trajectory_ready = False
            self._pending_nav_path = None
            self._pending_mission_waypoints = None
            self._pending_path_types = None
            self._pending_marking_indices = None
            self._clear_loaded_runtime(
                "Loaded trajectory cleared; mission.csv retained"
            )
        response.success = True
        response.message = "Loaded trajectory cleared; mission.csv retained"
        return response

    def _emergency_stop_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._lock:
            self._reset_point_timers()
            self._set_safety(emergency_stop=True, mission_enable=False)
            if self._state == "RUNNING":
                self._state = "PAUSED"
                self._pause_reason = "ESTOP"
                self._resume_available = False
            self._last_message = "Emergency stop ACTIVE; motion and spray disabled"
            self._emit_system_event("ESTOP_ACTIVE", self._last_message)
            self._publish_status(force=True)
        response.success = True
        response.message = self._last_message
        return response

    def _release_emergency_stop_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._lock:
            # Never resume automatically. Release only the hard stop latch;
            # _monitor_pause_recovery() will advertise Resume only after the
            # complete motion-health bundle is healthy again.
            self._set_safety(emergency_stop=False, mission_enable=False)
            if self._pause_reason == "ESTOP":
                self._resume_available = False
            self._last_message = (
                "Emergency stop released; mission remains paused/disabled"
            )
            self._emit_system_event("ESTOP_RELEASED", self._last_message)
            self._publish_status(force=True)
        response.success = True
        response.message = self._last_message
        return response

    # ==============================================================
    # Goal and sequence helpers
    # ==============================================================
    def _next_semantic_index(self, start: int) -> int:
        for index in self._semantic_path_indices:
            if index >= start:
                return index
        return len(self._navigation_path)

    def _current_or_next_marking_number(self) -> Optional[int]:
        if not self._navigation_path:
            return None
        for path_index in self._semantic_path_indices:
            if path_index < self._current_path_index:
                continue
            if self._path_types[path_index] != self.POINT_MARKING:
                continue
            number = self._marking_indices[path_index]
            if (
                0 <= number < len(self._point_status)
                and self._point_status[number] not in self.TERMINAL_POINT_STATES
            ):
                return number
        return None

    def _next_pending_marking_number(self, start: int) -> Optional[int]:
        for number in range(max(0, start), len(self._point_status)):
            if self._point_status[number] not in self.TERMINAL_POINT_STATES:
                return number
        return None

    def _advance_over_terminal_markings(self) -> None:
        while self._current_path_index < len(self._navigation_path):
            point_type = self._path_types[self._current_path_index]
            if point_type != self.POINT_MARKING:
                break
            marking_number = self._marking_indices[self._current_path_index]
            if not (0 <= marking_number < len(self._point_status)):
                self._enter_error("Invalid marking index at current semantic goal")
                return
            if self._point_status[marking_number] not in self.TERMINAL_POINT_STATES:
                break
            self._current_path_index = self._next_semantic_index(
                self._current_path_index + 1
            )

    def _current_point_id(self) -> Optional[str]:
        number = self._current_or_next_marking_number()
        return f"P{number+1:04d}" if number is not None else None

    def _reset_point_timers(self) -> None:
        """Reset transient timing/actuation state, never accuracy history."""
        self._arrival_settle_started = None
        self._arrival_settle_elapsed_sec = 0.0
        self._marking_hold_started = None
        self._marking_hold_elapsed_sec = 0.0
        self._spray_request_started = None

    def _reset_arrival_state(self) -> None:
        """Reset working state after an actual point/mission transition."""
        self._reset_point_timers()
        self._marking_error_valid = False
        self._marking_xtrack_m = math.inf
        self._marking_along_error_m = math.inf
        self._marking_combined_error_m = math.inf
        self._marking_radial_error_m = math.inf
        self._rpp_terminal_result = None
        self._rpp_terminal_result_last_rx_monotonic = None

    def _reset_execution_progress(self) -> None:
        self._current_path_index = self._next_semantic_index(0)
        self._active_marking_number = self._current_or_next_marking_number()
        self._last_completed_marking_number = None
        self._pause_reason = None
        self._resume_available = False
        self._reset_arrival_state()

    def _clear_loaded_runtime(self, message: str) -> None:
        self._navigation_path = []
        self._mission_waypoints = []
        self._path_types = []
        self._marking_indices = []
        self._marking_path_index_by_number = []
        self._semantic_path_indices = []
        self._point_status = []
        self._point_accuracy_snapshots = []
        self._point_results = []
        self._last_point_result = None
        self._state = "EMPTY"
        self._mission_run_id = ""
        self._mission_started_unix_ns = None
        self._current_path_index = 0
        self._active_marking_number = None
        self._last_error = None
        self._last_message = message
        self._reset_arrival_state()
        self._publish_marking_active(False)
        self._publish_mission_complete(False)
        self._publish_runtime_path()
        self._publish_status(force=True)

    # ==============================================================
    # Reporting geometry ONLY (never used to steer)
    # ==============================================================
    def _marking_error_components(
        self, marking_number: int
    ) -> tuple[float, float, float, float]:
        if self._x is None or self._y is None:
            return math.inf, math.inf, math.inf, math.inf
        target = self._mission_waypoints[marking_number]
        dx = self._x - target[0]
        dy = self._y - target[1]
        radial = math.hypot(dx, dy)

        # Accuracy is resolved in the final incoming NAVIGATION segment, not
        # the previous CSV-marking segment.  This handles P1 correctly because
        # trajectory_generator includes the prepared rover-start approach in
        # /nav_path, and it also handles dummy/extension geometry between CSV
        # points.  Scan backwards over any coincident samples defensively.
        if not (0 <= marking_number < len(self._marking_path_index_by_number)):
            return 0.0, radial, radial, radial

        path_index = self._marking_path_index_by_number[marking_number]
        start: Optional[tuple[float, float]] = None
        for candidate_index in range(path_index - 1, -1, -1):
            candidate = self._navigation_path[candidate_index]
            if self._distance(candidate, target) > 1.0e-9:
                start = candidate
                break

        if start is None:
            return 0.0, radial, radial, radial

        sx = target[0] - start[0]
        sy = target[1] - start[1]
        length = math.hypot(sx, sy)
        if length <= 1.0e-9:
            return 0.0, radial, radial, radial
        ux, uy = sx / length, sy / length
        # Error relative to target, resolved in incoming segment frame.
        along = dx * ux + dy * uy
        xtrack = -dx * uy + dy * ux
        combined = math.hypot(xtrack, along)
        return xtrack, along, combined, radial

    @staticmethod
    def _finite_number(value: float) -> Optional[float]:
        value = float(value)
        return value if math.isfinite(value) else None


#


    def _capture_accuracy_snapshot(
        self,
        *,
        marking_number: int,
        path_index: int,
        outcome: str,
        reason: Optional[str],
    ) -> dict[str, Any]:
        """Capture immutable point accuracy from RPP terminal_result ONLY.

        This function performs NO marking geometry calculation.

        - No hypot().
        - No local target-distance calculation.
        - No Mission Manager xtrack calculation.
        - No Mission Manager along-track calculation.
        - No tolerance decision.

        RPP already calculated and decided the terminal result.
        """

        if 0 <= marking_number < len(self._point_accuracy_snapshots):
            existing = self._point_accuracy_snapshots[marking_number]
            if existing is not None:
                return copy.deepcopy(existing)

        point_decision = str(outcome or "").strip().upper()

        # Operator SKIP intentionally has no terminal accuracy in the report.
        # This guarantees:
        #   Skipped -> Along — | Cross — | Overall —
        allow_rpp_terminal = point_decision != "NOT_EVALUATED"

        rpp = (
            self._rpp_terminal_result
            if allow_rpp_terminal and isinstance(self._rpp_terminal_result, dict)
            else None
        )

        expected_marking_number = marking_number + 1
        rpp_valid = False
        rpp_outcome: Optional[str] = None

        if rpp is not None:
            try:
                rpp_outcome = str(rpp.get("outcome") or "").strip().upper()
                rpp_valid = (
                    int(rpp.get("marking_number") or 0) == expected_marking_number
                    and bool(rpp.get("is_marking", False))
                    and rpp_outcome in {"CAPTURED", "MISSED"}
                    and str(rpp.get("measurement_source") or rpp.get("source") or "")
                    .strip()
                    .upper()
                    == "RPP_TERMINAL_RESULT"
                )
            except (TypeError, ValueError):
                rpp_valid = False

        def copied_finite(key: str) -> Optional[float]:
            if not rpp_valid or rpp is None:
                return None

            value = rpp.get(key)

            try:
                number = float(value)
            except (TypeError, ValueError):
                return None

            return number if math.isfinite(number) else None

        if rpp_valid and rpp is not None:
            cross_mm = copied_finite("cross_track_error_mm")
            along_mm = copied_finite("along_track_error_mm")
            overall_mm = copied_finite("overall_accuracy_mm")
            tolerance_mm = copied_finite("tolerance_mm")

            radial_m = copied_finite("radial_error_m")
            cross_m = copied_finite("cross_track_error_m")
            along_m = copied_finite("along_track_error_m")
            tolerance_m = copied_finite("tolerance_m")
            speed_mps = copied_finite("speed_mps")

            available = (
                cross_mm is not None and along_mm is not None and overall_mm is not None
            )

            snapshot: dict[str, Any] = {
                "measurement_source": "RPP_TERMINAL_RESULT",
                "available": available,
                # Mission Manager point-state context only.
                "outcome": point_decision,
                "reason": reason,
                # Exact RPP terminal state.
                "rpp_outcome": rpp_outcome,
                "rpp_reason": rpp.get("reason"),
                "point_id": f"P{marking_number+1:04d}",
                "point_index": marking_number,
                "path_index": path_index,
                # Exact timestamp produced by RPP.
                "timestamp_unix_ns": rpp.get("timestamp_unix_ns"),
                # COPY ONLY.
                "cross_track_error_m": cross_m,
                "cross_track_error_mm": cross_mm,
                "along_track_error_m": along_m,
                "along_track_error_mm": along_mm,
                "radial_error_m": radial_m,
                "radial_error_mm": overall_mm,
                "overall_accuracy_mm": overall_mm,
                "total_accuracy_mm": overall_mm,
                "tolerance_m": tolerance_m,
                "tolerance_mm": tolerance_mm,
                "within_tolerance": (
                    bool(rpp.get("within_tolerance"))
                    if rpp.get("within_tolerance") is not None
                    else None
                ),
                "speed_mps": speed_mps,
            }

        else:
            # No RPP terminal result means NO report accuracy.
            snapshot = {
                "measurement_source": "RPP_TERMINAL_RESULT",
                "available": False,
                "outcome": point_decision,
                "reason": reason,
                "rpp_outcome": None,
                "rpp_reason": None,
                "point_id": f"P{marking_number+1:04d}",
                "point_index": marking_number,
                "path_index": path_index,
                "timestamp_unix_ns": None,
                "cross_track_error_m": None,
                "cross_track_error_mm": None,
                "along_track_error_m": None,
                "along_track_error_mm": None,
                "radial_error_m": None,
                "radial_error_mm": None,
                "overall_accuracy_mm": None,
                "total_accuracy_mm": None,
                "tolerance_m": None,
                "tolerance_mm": None,
                "within_tolerance": None,
                "speed_mps": None,
            }

        if 0 <= marking_number < len(self._point_accuracy_snapshots):
            self._point_accuracy_snapshots[marking_number] = copy.deepcopy(snapshot)

        return copy.deepcopy(snapshot)

    def _spray_outcome_snapshot(
        self,
        *,
        attempted: bool,
        outcome: str,
        reason: Optional[str] = None,
    ) -> dict[str, Any]:
        elapsed_sec = 0.0
        if attempted and self._spray_request_started is not None:
            elapsed_sec = max(
                0.0,
                time.monotonic() - self._spray_request_started,
            )
        return {
            "attempted": bool(attempted),
            "outcome": str(outcome).upper(),
            "reason": reason,
            "required": self.spray_required,
            "elapsed_sec": elapsed_sec,
            "timestamp_unix_ns": time.time_ns(),
        }

    def _store_point_result(
        self,
        *,
        marking_number: int,
        path_index: int,
        point_outcome: str,
        accuracy: dict[str, Any],
        spray: dict[str, Any],
    ) -> dict[str, Any]:
        result = {
            "point_id": f"P{marking_number+1:04d}",
            "point_index": marking_number,
            "path_index": path_index,
            "point_outcome": str(point_outcome).upper(),
            "timestamp_unix_ns": time.time_ns(),
            "accuracy": copy.deepcopy(accuracy),
            "spray": copy.deepcopy(spray),
        }
        if 0 <= marking_number < len(self._point_results):
            self._point_results[marking_number] = copy.deepcopy(result)
        self._last_point_result = copy.deepcopy(result)
        return copy.deepcopy(result)

    # ==============================================================
    # Publishers / events / status
    # ==============================================================
    def _pose_for_index(self, path_index: int) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.local_frame
        if 0 <= path_index < len(self._navigation_path):
            x, y = self._navigation_path[path_index]
            msg.pose.position.x = x
            msg.pose.position.y = y
        # Orientation intentionally left identity/zero. Mission Manager does
        # NOT tell RPP what heading to use.
        msg.pose.orientation.w = 1.0
        return msg

    def _publish_goal(self) -> None:
        if not (0 <= self._current_path_index < len(self._navigation_path)):
            return
        if self._path_types[self._current_path_index] == self.POINT_PASS_THROUGH:
            return
        msg = self._pose_for_index(self._current_path_index)
        self.active_waypoint_pub.publish(msg)
        self.segment_goal_pub.publish(msg)

    def _publish_runtime_path(self) -> None:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.local_frame
        for index in range(
            max(0, self._current_path_index), len(self._navigation_path)
        ):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = self._navigation_path[index][0]
            pose.pose.position.y = self._navigation_path[index][1]
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.runtime_path_pub.publish(msg)

    def _publish_marking_active(self, active: bool) -> None:
        msg = Bool()
        msg.data = bool(active)
        self.marking_active_pub.publish(msg)

    def _publish_mission_complete(self, complete: bool) -> None:
        msg = Bool()
        msg.data = bool(complete)
        self.mission_complete_pub.publish(msg)



#
#
# Important: fallback accuracy is now "unavailable".
# It never reconstructs accuracy from Mission Manager live geometry.


    def _emit_point_event(
        self,
        event: str,
        marking_number: int,
        path_index: int,
        *,
        reason: Optional[str] = None,
        accuracy: Optional[dict[str, Any]] = None,
        spray: Optional[dict[str, Any]] = None,
        point_result: Optional[dict[str, Any]] = None,
    ) -> None:

        accuracy_payload = (
            copy.deepcopy(accuracy)
            if isinstance(accuracy, dict)
            else {
                "measurement_source": "RPP_TERMINAL_RESULT",
                "available": False,
                "outcome": "UNAVAILABLE",
                "reason": reason,
                "rpp_outcome": None,
                "rpp_reason": None,
                "point_id": f"P{marking_number+1:04d}",
                "point_index": marking_number,
                "path_index": path_index,
                "timestamp_unix_ns": None,
                "cross_track_error_m": None,
                "cross_track_error_mm": None,
                "along_track_error_m": None,
                "along_track_error_mm": None,
                "radial_error_m": None,
                "radial_error_mm": None,
                "overall_accuracy_mm": None,
                "total_accuracy_mm": None,
                "tolerance_m": None,
                "tolerance_mm": None,
                "within_tolerance": None,
                "speed_mps": None,
            }
        )

        spray_payload = (
            copy.deepcopy(spray)
            if spray is not None
            else {
                "attempted": self._spray_request_started is not None,
                "outcome": "UNSPECIFIED",
                "reason": None,
                "required": self.spray_required,
                "elapsed_sec": (
                    max(0.0, time.monotonic() - self._spray_request_started)
                    if self._spray_request_started is not None
                    else 0.0
                ),
            }
        )

        payload = {
            "event": event,
            "mission_run_id": self._mission_run_id or None,
            "point_id": f"P{marking_number+1:04d}",
            "point_index": marking_number,
            "path_index": path_index,
            "state": self._state,
            "timestamp_unix_ns": time.time_ns(),
            # Compatibility flat fields are COPY-ONLY aliases.
            "radial_error_m": accuracy_payload.get("radial_error_m"),
            "radial_error_mm": accuracy_payload.get("radial_error_mm"),
            "reason": reason,
            "xtrack_m": accuracy_payload.get("cross_track_error_m"),
            "xtrack_mm": accuracy_payload.get("cross_track_error_mm"),
            "along_error_m": accuracy_payload.get("along_track_error_m"),
            "along_error_mm": accuracy_payload.get("along_track_error_mm"),
            "overall_accuracy_mm": accuracy_payload.get("overall_accuracy_mm"),
            "accuracy": accuracy_payload,
            "spray": spray_payload,
            "spray_outcome": spray_payload.get("outcome"),
            "point_result": copy.deepcopy(point_result),
        }

        msg = String()
        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
            sort_keys=True,
        )

        self.point_event_pub.publish(msg)

    def _fail_marking_point(
        self, marking_number: int, path_index: int, reason: str
    ) -> None:
        """Record a terminal FAILED point before entering mission ERROR."""
        if 0 <= marking_number < len(self._point_status):
            if self._point_status[marking_number] not in self.TERMINAL_POINT_STATES:
                self._point_status[marking_number] = "FAILED"
        self._enter_error(reason)
        self._emit_point_event("FAILED", marking_number, path_index, reason=reason)
        # _enter_error() already publishes status, but publish again after the
        # point event so frontend/backend observers see the final FAILED count.
        self._publish_status(force=True)

    def _emit_system_event(self, event: str, message: str) -> None:
        payload = {
            "event": event,
            "mission_run_id": self._mission_run_id or None,
            "state": self._state,
            "message": message,
            "timestamp_unix_ns": time.time_ns(),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.point_event_pub.publish(msg)

    def _emit_mission_terminated(
        self,
        *,
        completed: bool,
        source: str,
        disarm_confirmed: bool,
        warnings: list[str],
        message: str,
    ) -> None:
        """Publish the final, post-disarm-attempt record before runtime is cleared."""
        terminated_at_unix_ns = time.time_ns()
        completed_count = self._point_status.count("COMPLETED")
        skipped_count = self._point_status.count("SKIPPED")
        failed_count = self._point_status.count("FAILED")
        total_count = len(self._point_status)
        resolved_count = completed_count + skipped_count + failed_count
        mission_summary = {
            "mission_run_id": self._mission_run_id or None,
            "state": self._state,
            "execution_mode": self._execution_mode,
            "normal_completion": bool(completed),
            "termination_source": source,
            "started_at_unix_ns": self._mission_started_unix_ns,
            "terminated_at_unix_ns": terminated_at_unix_ns,
            "point_status": list(self._point_status),
            "point_accuracy_snapshots": copy.deepcopy(self._point_accuracy_snapshots),
            "point_results": copy.deepcopy(self._point_results),
            "total_points": total_count,
            "resolved_points": resolved_count,
            "completed_points": completed_count,
            "skipped_points": skipped_count,
            "failed_points": failed_count,
            "remaining_points": max(0, total_count - resolved_count),
            "progress_percent": (
                round(100.0 * resolved_count / total_count, 2) if total_count else 0.0
            ),
            "disarm_confirmed": bool(disarm_confirmed),
            "px4_mode": self._px4_mode,
            "px4_armed": self._px4_armed,
        }
        payload = {
            "event": "MISSION_TERMINATED",
            "mission_run_id": self._mission_run_id or None,
            "state": self._state,
            "source": source,
            "completed": bool(completed),
            "disarm_confirmed": bool(disarm_confirmed),
            "px4_mode": self._px4_mode,
            "px4_armed": self._px4_armed,
            "message": message,
            "warnings": list(warnings),
            "mission_summary": mission_summary,
            "point_status": list(self._point_status),
            "point_accuracy_snapshots": copy.deepcopy(self._point_accuracy_snapshots),
            "point_results": copy.deepcopy(self._point_results),
            "timestamp_unix_ns": terminated_at_unix_ns,
        }
        self._last_termination_report = copy.deepcopy(payload)
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.point_event_pub.publish(msg)

    def _status_payload(self) -> dict[str, Any]:
        completed = self._point_status.count("COMPLETED")
        skipped = self._point_status.count("SKIPPED")
        failed = self._point_status.count("FAILED")
        accuracy_failed = sum(
            1
            for snapshot in self._point_accuracy_snapshots
            if snapshot is not None and snapshot.get("outcome") == "FAILED"
        )
        total = len(self._point_status)
        processed = completed + skipped + failed
        active_number = self._current_or_next_marking_number()
        active_id = f"P{active_number+1:04d}" if active_number is not None else None
        active_state = (
            self._point_status[active_number] if active_number is not None else None
        )
        spray_key = (
            (self._mission_run_id, active_id)
            if self._mission_run_id and active_id
            else None
        )
        rtk_ok, rtk_reason = self._rtk_motion_ok()
        return {
            "state": self._state,
            "message": self._last_message,
            "error": self._last_error,
            "mission_run_id": self._mission_run_id or None,
            "mission_started_unix_ns": self._mission_started_unix_ns,
            "execution_mode": self._execution_mode,
            "pause_reason": self._pause_reason,
            "resume_available": self._resume_available,
            "trajectory_ready": self._trajectory_ready,
            "path_ready": bool(self._navigation_path),
            "current_path_index": self._current_path_index,
            "navigation_point_count": len(self._navigation_path),
            "current_point_id": active_id,
            "current_point_index": active_number,
            "current_point_state": active_state,
            "total_points": total,
            "completed_points": completed,
            "skipped_points": skipped,
            "failed_points": failed,
            "accuracy_failed_points": accuracy_failed,
            "remaining_points": max(0, total - processed),
            "progress_percent": round(100.0 * processed / total, 2) if total else 0.0,
            "point_status": list(self._point_status),
            "point_accuracy_snapshots": copy.deepcopy(self._point_accuracy_snapshots),
            "point_results": copy.deepcopy(self._point_results),
            "last_point_result": copy.deepcopy(self._last_point_result),
            "last_termination_report": copy.deepcopy(self._last_termination_report),
            "mission_enable": self._mission_enable,
            "emergency_stop": self._emergency_stop,
            "px4_connected": self._fcu_connected,
            "px4_mode": self._px4_mode,
            "px4_armed": self._px4_armed,
            "gps_fix_type": self._gps_fix_type,
            "rtk_state": self._rtk_state(),
            "rtk_fixed": self._gps_fix_type == 6,
            "rtk_healthy": self._rtk_healthy,
            "rtk_motion_ok": rtk_ok,
            "rtk_reason": rtk_reason,
            "rtk_correction_age_sec": self._rtk_correction_age_sec,
            "rtk_correction_age_limit_sec": self.MAX_RTK_CORRECTION_AGE_SEC,
            "rtk_correction_age_over_limit": (
                self._rtk_correction_age_sec is not None
                and self._rtk_correction_age_sec > self.MAX_RTK_CORRECTION_AGE_SEC
            ),
            "rtk_gate_mode": "FRESH_FIX_TYPE_6_ONLY",
            "gps_fix_status_age_sec": round(
                self._age(self._last_gps_fix_rx_monotonic), 3
            ),
            "rtk_health_status_age_sec": round(
                self._age(self._last_rtk_health_rx_monotonic), 3
            ),
            "rtk_age_status_age_sec": round(
                self._age(self._last_rtk_age_rx_monotonic), 3
            ),
            "backend_heartbeat_healthy": self._backend_heartbeat_healthy,
            "spray_required": self.spray_required,
            "spray_gates_mission_progress": False,
            "spray_controller_ready": self._spray_controller_ready,
            "spray_controller_state": self._spray_controller_state,
            "spray_fault_reason": self._spray_fault_reason,
            "current_point_spray_confirmed": (
                spray_key in self._spray_success_keys if spray_key else False
            ),
            "marking_active": self._spray_request_started is not None,
            "arrival_settle_elapsed_sec": round(self._arrival_settle_elapsed_sec, 3),
            "arrival_settle_required_sec": self.arrival_settle_sec,
            "marking_hold_elapsed_sec": round(self._marking_hold_elapsed_sec, 3),
            "marking_hold_required_sec": self.marking_hold_sec,
            "verification_hold_elapsed_sec": round(self._marking_hold_elapsed_sec, 3),
            "verification_hold_required_sec": self.marking_hold_sec,
            # Compatibility names retained for current frontend/backend.
            # hold_* now represents the 3-second PRE-MARK verification.
            "hold_elapsed_sec": round(self._marking_hold_elapsed_sec, 3),
            "hold_required_sec": self.marking_hold_sec,
            "marking_error_mode": "RADIAL_2D",
            "marking_tolerance_m": self.marking_tolerance_m,
            "goal_plane_overshoot_trigger_m": self.GOAL_PLANE_OVERSHOOT_TRIGGER_M,
            "rpp_terminal_result": copy.deepcopy(self._rpp_terminal_result),
            "marking_xtrack_m": (
                round(self._marking_xtrack_m, 6)
                if math.isfinite(self._marking_xtrack_m)
                else None
            ),
            "marking_along_error_m": (
                round(self._marking_along_error_m, 6)
                if math.isfinite(self._marking_along_error_m)
                else None
            ),
            "marking_combined_error_m": (
                round(self._marking_combined_error_m, 6)
                if math.isfinite(self._marking_combined_error_m)
                else None
            ),
            "marking_radial_error_m": (
                round(self._marking_radial_error_m, 6)
                if math.isfinite(self._marking_radial_error_m)
                else None
            ),
            "marking_error_valid": self._marking_error_valid,
            "alignment_active": False,
            "start_stage": self._start_stage,
            "start_failed_stage": self._start_failed_stage,
            "start_debug": dict(self._start_debug),
        }

    def _publish_status(self, *, force: bool = False) -> None:
        now = time.monotonic()
        if (
            not force
            and now - self._last_status_publish_monotonic < 1.0 / self.STATUS_HZ
        ):
            return
        self._last_status_publish_monotonic = now
        msg = String()
        msg.data = json.dumps(
            self._status_payload(), separators=(",", ":"), sort_keys=True
        )
        self.status_pub.publish(msg)

    # ==============================================================
    # Mission completion / error
    # ==============================================================
    def _finish_if_done(self) -> bool:
        if self._next_pending_marking_number(0) is not None:
            return False
        self._complete_mission()
        return True



#
#
# This preserves the current AUTO/MANUAL transition behavior but the stored
# failure accuracy is RPP-only and spray is explicitly forbidden.


    def _resolve_accuracy_failure(
        self,
        *,
        marking_number: int,
        failure_reason: str,
    ) -> None:
        """Record RPP MISSED as FAILED with no spray."""

        point_id = f"P{marking_number+1:04d}"

        accuracy = self._capture_accuracy_snapshot(
            marking_number=marking_number,
            path_index=self._current_path_index,
            outcome="FAILED",
            reason=failure_reason,
        )

        spray = self._spray_outcome_snapshot(
            attempted=False,
            outcome="NOT_ATTEMPTED_RPP_MISSED",
            reason=failure_reason,
        )

        # Absolute guarantee: RPP MISSED never authorizes spray.
        self._publish_marking_active(False)
        self._spray_request_started = None

        self._point_status[marking_number] = "FAILED"

        point_result = self._store_point_result(
            marking_number=marking_number,
            path_index=self._current_path_index,
            point_outcome="FAILED",
            accuracy=accuracy,
            spray=spray,
        )

        overall_mm = accuracy.get("overall_accuracy_mm")
        overall_text = (
            f"{overall_mm:.1f}mm"
            if isinstance(overall_mm, (int, float)) and math.isfinite(float(overall_mm))
            else "unavailable"
        )

        self._last_message = (
            f"{point_id} FAILED from RPP MISSED "
            f"(overall={overall_text}); {failure_reason}"
        )

        self._emit_point_event(
            "ACCURACY_FAILED",
            marking_number,
            self._current_path_index,
            reason=failure_reason,
            accuracy=accuracy,
            spray=spray,
            point_result=point_result,
        )

        self._publish_marking_active(False)

        self._current_path_index = self._next_semantic_index(self._current_path_index + 1)

        self._reset_arrival_state()
        self._publish_runtime_path()

        if self._finish_if_done():
            return

        action = after_fail_mode_action(self._execution_mode)

        if action == "WAITING_FOR_NEXT":
            self._state = "WAITING_FOR_NEXT"
            self._pause_reason = None
            self._resume_available = False

            self._set_safety(
                emergency_stop=False,
                mission_enable=False,
            )

            self._last_message = (
                f"{point_id} FAILED from RPP MISSED "
                f"(overall={overall_text}); waiting for NEXT"
            )

        else:
            self._state = "RUNNING"
            self._pause_reason = None
            self._resume_available = False

            self._last_message = (
                f"{point_id} FAILED from RPP MISSED "
                f"(overall={overall_text}); continuing automatically"
            )

            self._publish_goal()

        self._publish_status(force=True)

    def _complete_mission(self) -> None:
        """Report normal completion, then schedule automatic STOP cleanup."""
        if self._auto_stop_pending:
            return

        # Completion is a terminal hard stop immediately; disarm is then
        # attempted outside this lock on the next timer tick.
        self._reset_point_timers()
        self._set_safety(emergency_stop=True, mission_enable=False)
        self._state = "COMPLETED"
        self._pause_reason = None
        self._resume_available = False
        self._last_error = None
        self._last_message = (
            "All uploaded points processed; mission completed; "
            "automatic STOP pending"
        )
        self._publish_marking_active(False)
        self._publish_mission_complete(True)
        self._emit_system_event("MISSION_COMPLETED", self._last_message)
        self._publish_status(force=True)

        # Do not disarm while _control_loop holds self._lock because MAVROS
        # state callbacks also need the lock. The next timer tick performs the
        # same STOP cleanup outside the control-loop lock.
        self._auto_stop_pending = True

    def _run_auto_stop_if_pending(self) -> bool:
        if not self._auto_stop_pending:
            return False

        # Clear the pending flag before doing service calls so this is one-shot.
        self._auto_stop_pending = False
        terminated, warnings = self._execute_stop_cleanup(
            completed=True,
            source="AUTO_COMPLETE",
        )
        if not terminated:
            self.get_logger().error(
                "Automatic STOP is hard-stopped but PX4 DISARM is not confirmed; "
                "mission state retained for operator retry"
            )
        if warnings:
            self.get_logger().warn(
                "Automatic STOP completed with warnings: " + "; ".join(warnings)
            )
        return True

    def _enter_error(self, reason: str) -> None:
        self._mission_enable = False
        self._state = "ERROR"
        self._last_error = reason
        self._last_message = reason
        self._pause_reason = None
        self._resume_available = False
        self._reset_arrival_state()
        self._publish_marking_active(False)
        self._publish_safety()
        self._publish_status(force=True)
        self.get_logger().error(reason)

    # ==============================================================
    # Control loop: sequencing + marking validation ONLY
    # ==============================================================
    def _control_loop(self) -> None:
        # Automatic normal-completion STOP must run outside the long-held
        # control-loop lock so MAVROS state callbacks can confirm DISARM.
        if self._run_auto_stop_if_pending():
            return

        with self._lock:
            self._publish_status()

            if not self._navigation_path:
                return

            self._monitor_runtime_rtk()
            self._monitor_runtime_px4_control()
            self._monitor_pause_recovery()

            if self._state not in {
                "RUNNING",
                "PAUSED",
                "WAITING_FOR_NEXT",
            }:
                return

            self._publish_goal()

            if self._state != "RUNNING":
                return
            if not self._mission_enable or self._emergency_stop:
                return
            if self._x is None or self._y is None:
                return
            if self._age(self._last_odom_monotonic) > self.odom_timeout_sec:
                # Odometry is a motion-safety condition. Pause rather than
                # continuing with stale position.
                self._state = "PAUSED"
                self._pause_reason = "ODOM_STALE"
                self._resume_available = False
                self._set_safety(emergency_stop=False, mission_enable=False)
                self._last_message = "Local odometry stale; mission paused"
                self._emit_system_event("ODOM_PAUSED", self._last_message)
                self._publish_status(force=True)
                return

            self._advance_over_terminal_markings()
            if self._state == "ERROR":
                return
            if self._finish_if_done():
                return
            if self._current_path_index >= len(self._navigation_path):
                self._enter_error("Path ended before all marking points were handled")
                return

            point_type = self._path_types[self._current_path_index]
            target = self._navigation_path[self._current_path_index]
            distance = math.hypot(target[0] - self._x, target[1] - self._y)

            # Pass-through points are never mission goals in this clean manager.
            if point_type == self.POINT_PASS_THROUGH:
                self._current_path_index = self._next_semantic_index(
                    self._current_path_index + 1
                )
                self._publish_goal()
                return

            # Dummy/extension: navigation-only semantic goal. No alignment,
            # heading or speed logic here. RPP owns how it reaches the dummy.
            if point_type == self.POINT_DUMMY_ALIGNMENT:
                self._publish_marking_active(False)
                self._reset_arrival_state()
                inside_extension_radius = distance <= self.dummy_arrival_tolerance_m
                extension_stationary = (
                    self._speed_mps <= self.stationary_speed_tolerance_mps
                )
                if inside_extension_radius and extension_stationary:
                    self._current_path_index = self._next_semantic_index(
                        self._current_path_index + 1
                    )
                    self._last_message = (
                        "Dummy/extension goal reached inside "
                        f"{self.dummy_arrival_tolerance_m*1000.0:.0f}mm and stationary; "
                        "next semantic goal selected"
                    )
                    self._publish_runtime_path()
                    self._publish_goal()
                    self._publish_status(force=True)
                elif inside_extension_radius:
                    self._last_message = (
                        "Dummy/extension inside "
                        f"{self.dummy_arrival_tolerance_m*1000.0:.0f}mm; waiting for "
                        f"speed <= {self.stationary_speed_tolerance_mps:.3f}m/s"
                    )
                return

            if point_type != self.POINT_MARKING:
                self._enter_error("Unknown semantic point type")
                return

            marking_number = self._marking_indices[self._current_path_index]
            if not (0 <= marking_number < len(self._point_status)):
                self._enter_error("Current marking index invalid")
                return

            self._active_marking_number = marking_number
            if self._point_status[marking_number] == "PENDING":
                self._point_status[marking_number] = "ACTIVE"

            #
            # In _control_loop(), after:
            #
            #     self._active_marking_number = marking_number
            #     if self._point_status[marking_number] == "PENDING":
            #         self._point_status[marking_number] = "ACTIVE"
            #
            # REPLACE EVERYTHING from the current:
            #
            #     (
            #         self._marking_xtrack_m,
            #         ...
            #     ) = self._marking_error_components(marking_number)
            #
            # THROUGH THE END OF CURRENT "PHASE B" -- immediately before:
            #
            #     # PHASE C: spray transaction.
            #
            # WITH THIS BLOCK.
            #
            # KEEP THE EXISTING PHASE C code after this block, with the two small edits
            # shown at the bottom of this file.

            # ======================================================
            # MARKING DECISION: RPP TERMINAL RESULT ONLY
            # ======================================================
            now = time.monotonic()
            point_id = f"P{marking_number+1:04d}"
            spray_key = (self._mission_run_id, point_id)

            rpp = (
                self._rpp_terminal_result
                if isinstance(self._rpp_terminal_result, dict)
                else None
            )

            rpp_terminal_outcome = (
                str(rpp.get("outcome") or "").strip().upper() if rpp is not None else ""
            )

            # These compatibility status fields are no longer calculated by
            # Mission Manager. They remain empty until/reset between points.
            self._marking_error_valid = rpp_terminal_outcome == "CAPTURED"

            # ------------------------------------------------------
            # PHASE A
            # Wait for RPP's one authoritative terminal decision.
            # ------------------------------------------------------
            if self._marking_hold_started is None:
                self._publish_marking_active(False)

                if rpp_terminal_outcome == "":
                    self._arrival_settle_started = None
                    self._arrival_settle_elapsed_sec = 0.0
                    self._marking_hold_elapsed_sec = 0.0

                    self._last_message = (
                        f"{point_id} active; waiting for RPP CAPTURED/MISSED"
                    )
                    return

                if rpp_terminal_outcome == "MISSED":
                    self._resolve_accuracy_failure(
                        marking_number=marking_number,
                        failure_reason=str((rpp or {}).get("reason") or "RPP_MISSED"),
                    )
                    return

                if rpp_terminal_outcome != "CAPTURED":
                    self._last_message = (
                        f"{point_id} waiting; invalid RPP terminal outcome "
                        f"{rpp_terminal_outcome!r}"
                    )
                    return

                # CAPTURED is already RPP's stationary final-stop decision.
                # Mission Manager does NOT calculate/check 30 mm again.
                accuracy = self._capture_accuracy_snapshot(
                    marking_number=marking_number,
                    path_index=self._current_path_index,
                    outcome="ACHIEVED",
                    reason=None,
                )

                if accuracy.get("available") is not True:
                    self._publish_marking_active(False)
                    self._enter_error(
                        f"{point_id} RPP CAPTURED without complete terminal accuracy"
                    )
                    return

                self._marking_hold_started = now
                self._arrival_settle_started = now
                self._arrival_settle_elapsed_sec = 0.0
                self._marking_hold_elapsed_sec = 0.0

                self._last_message = (
                    f"{point_id} RPP CAPTURED; starting "
                    f"{self.marking_hold_sec:.1f}s pre-mark hold; "
                    "spray remains OFF"
                )

                self._publish_status(force=True)
                return

            # ------------------------------------------------------
            # PHASE B
            # Fixed pre-mark hold AFTER RPP CAPTURED.
            #
            # No radial re-check.
            # No cross-track re-check.
            # No along-track re-check.
            # No second tolerance decision.
            # ------------------------------------------------------
            if self._spray_request_started is None:
                self._marking_hold_elapsed_sec = max(
                    0.0,
                    now - self._marking_hold_started,
                )

                self._arrival_settle_elapsed_sec = self._marking_hold_elapsed_sec

                self._publish_marking_active(False)

                accuracy = self._capture_accuracy_snapshot(
                    marking_number=marking_number,
                    path_index=self._current_path_index,
                    outcome="ACHIEVED",
                    reason=None,
                )

                if accuracy.get("available") is not True:
                    self._publish_marking_active(False)
                    self._enter_error(
                        f"{point_id} lost RPP terminal accuracy before spray"
                    )
                    return

                if self._marking_hold_elapsed_sec < self.marking_hold_sec:
                    remaining = max(
                        0.0,
                        self.marking_hold_sec - self._marking_hold_elapsed_sec,
                    )

                    overall_mm = accuracy.get("overall_accuracy_mm")

                    self._last_message = (
                        f"{point_id} RPP CAPTURED hold; "
                        f"{remaining:.2f}s remaining; "
                        f"overall={overall_mm}mm"
                    )
                    return

                # Hold complete. RPP CAPTURED authorizes spray.
                pending_spray = self._spray_outcome_snapshot(
                    attempted=self.spray_required,
                    outcome=("PENDING" if self.spray_required else "DISABLED"),
                )

                self._emit_point_event(
                    "ACCURACY_ACHIEVED",
                    marking_number,
                    self._current_path_index,
                    accuracy=accuracy,
                    spray=pending_spray,
                )

                self._spray_request_started = now

                if self.spray_required:
                    self._publish_marking_active(True)

                    overall_mm = accuracy.get("overall_accuracy_mm")

                    self._last_message = (
                        f"{point_id} RPP CAPTURED after "
                        f"{self.marking_hold_sec:.1f}s: "
                        f"overall={overall_mm}mm; "
                        "spray/mark triggered"
                    )

                    self._publish_status(force=True)
                    return

                # Spray disabled: fall through to existing PHASE C completion.
                self._publish_marking_active(False)

            # ==============================================================
            # TWO SMALL CURRENT PHASE-C EDITS
            # ==============================================================

            # CURRENT:
            #
            # accuracy = copy.deepcopy(self._point_accuracy_snapshots[marking_number])
            # if accuracy is None:
            #     accuracy = self._capture_accuracy_snapshot(...)
            #
            # REPLACE WITH:

            accuracy = self._capture_accuracy_snapshot(
                marking_number=marking_number,
                path_index=self._current_path_index,
                outcome="ACHIEVED",
                reason=None,
            )

            # CURRENT:
            #
            # final_mm = accuracy.get("radial_error_mm")
            #
            # REPLACE WITH:
            #
            # final_mm = accuracy.get("overall_accuracy_mm")
            # PHASE C: spray transaction. Current spray_controller needs
            # marking_active to remain TRUE through its stable/pre-spray and
            # press/release transaction, so keep it asserted until a result.
            spray_success = spray_key in self._spray_success_keys
            spray_failure = self._spray_failure_keys.get(spray_key)
            spray_elapsed = (
                max(0.0, now - self._spray_request_started)
                if self._spray_request_started is not None
                else 0.0
            )

            if self.spray_required:
                self._publish_marking_active(True)

                if (
                    not spray_success
                    and spray_failure is None
                    and spray_elapsed < self.spray_confirmation_timeout_sec
                ):
                    self._last_message = (
                        f"{point_id} verified; marking transaction active "
                        f"({spray_elapsed:.2f}s)"
                    )
                    return

                if not spray_success and spray_failure is None:
                    self._emit_system_event(
                        "SPRAY_TIMEOUT_MONITOR_ONLY",
                        (
                            f"{point_id} spray result timeout after "
                            f"{self.spray_confirmation_timeout_sec:.1f}s; "
                            "mission progression continues"
                        ),
                    )

            # Marking transaction has finished (SUCCESS, FAILED, timeout, or
            # spray disabled). Spray failure never aborts the whole mission.
            self._publish_marking_active(False)

            accuracy = self._capture_accuracy_snapshot(
                marking_number=marking_number,
                path_index=self._current_path_index,
                outcome="ACHIEVED",
                reason=None,
            )

            if not self.spray_required:
                spray = self._spray_outcome_snapshot(
                    attempted=False,
                    outcome="DISABLED",
                )
            elif spray_success:
                spray = self._spray_outcome_snapshot(
                    attempted=True,
                    outcome="SUCCESS",
                )
            elif spray_failure is not None:
                spray = self._spray_outcome_snapshot(
                    attempted=True,
                    outcome="FAILED",
                    reason=spray_failure,
                )
            else:
                spray = self._spray_outcome_snapshot(
                    attempted=True,
                    outcome="TIMEOUT",
                    reason=(
                        f"No spray result within "
                        f"{self.spray_confirmation_timeout_sec:.1f}s"
                    ),
                )

            final_mm = accuracy.get("overall_accuracy_mm")
            final_text = f"{final_mm:.1f}mm" if final_mm is not None else "unavailable"
            self._point_status[marking_number] = "COMPLETED"
            self._last_completed_marking_number = marking_number
            point_result = self._store_point_result(
                marking_number=marking_number,
                path_index=self._current_path_index,
                point_outcome="COMPLETED",
                accuracy=accuracy,
                spray=spray,
            )
            self._emit_point_event(
                "COMPLETED",
                marking_number,
                self._current_path_index,
                accuracy=accuracy,
                spray=spray,
                point_result=point_result,
            )

            self._current_path_index = self._next_semantic_index(
                self._current_path_index + 1
            )
            self._reset_arrival_state()
            self._publish_runtime_path()

            # If this was the final uploaded point, _complete_mission() reports
            # completion now and schedules automatic STOP/DISARM/clear.
            if self._finish_if_done():
                return

            if self._execution_mode == "MANUAL":
                self._state = "WAITING_FOR_NEXT"
                self._set_safety(
                    emergency_stop=False,
                    mission_enable=False,
                )
                self._last_message = (
                    f"{point_id} COMPLETED at {final_text}; waiting for NEXT"
                )
            else:
                self._state = "RUNNING"
                self._pause_reason = None
                self._resume_available = False
                self._last_message = (
                    f"{point_id} COMPLETED at {final_text}; " "continuing automatically"
                )
                self._publish_goal()

            self._publish_status(force=True)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._set_safety(emergency_stop=True, mission_enable=False)
            node._publish_marking_active(False)
            node._publish_mission_complete(False)
        finally:
            executor.shutdown(timeout_sec=2.0)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
