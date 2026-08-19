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
  * PX4 OFFBOARD/arming orchestration
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


class MissionManager(Node):
    """Mission/safety/marking state machine. Motion control is RPP's job."""

    CONTROL_HZ = 20.0
    STATUS_HZ = 5.0
    SAFETY_PUBLISH_HZ = 2.0

    FCU_STATE_STALE_SEC = 3.0
    GPS_FIX_STALE_SEC = 2.0
    RTK_STATUS_STALE_SEC = 5.0
    MAX_RTK_CORRECTION_AGE_SEC = 2.0
    OFFBOARD_STREAM_SETTLE_SEC = 0.60
    SERVICE_DISCOVERY_TIMEOUT_SEC = 3.0
    SERVICE_RESPONSE_TIMEOUT_SEC = 5.0
    VEHICLE_STATE_CONFIRM_TIMEOUT_SEC = 3.0

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

        # Arrival verification. This is NOT spray duration.
        self._arrival_settle_started: Optional[float] = None
        self._arrival_settle_elapsed_sec = 0.0
        # Marking hold starts only after exact arrival has been validated.
        # The rover remains stopped for marking_hold_sec total; spray is only
        # a short transaction inside that hold.
        self._marking_hold_started: Optional[float] = None
        self._marking_hold_elapsed_sec = 0.0
        self._spray_request_started: Optional[float] = None

        self._marking_xtrack_m = math.inf
        self._marking_along_error_m = math.inf
        self._marking_combined_error_m = math.inf
        self._marking_radial_error_m = math.inf
        self._marking_error_valid = False

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
            f"Marking: radial <= {self.marking_tolerance_m*1000.0:.0f} mm AND "
            f"speed <= {self.stationary_speed_tolerance_mps:.3f} m/s for "
            f"{self.arrival_settle_sec:.2f}s arrival settle, {self.marking_hold_sec:.2f}s marking hold"
        )
        self.get_logger().warn(
            "RTK runtime rule: FLOAT/lost => PAUSE; FIXED return => READY TO RESUME; no auto-resume"
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
                if self._state in {"RUNNING", "PAUSED", "WAITING_FOR_NEXT"}:
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
            if self._state in {"RUNNING", "PAUSED", "WAITING_FOR_NEXT"}:
                self.get_logger().warn(
                    "Execution mode cannot change during active mission"
                )
                return
            self._execution_mode = mode
            self._last_message = f"Execution mode set to {mode}"
            self._publish_status(force=True)

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
                self._last_message = f"Spray confirmed for {point_id}"
                self._publish_status(force=True)
            elif result == "FAILED":
                reason = str(payload.get("reason", "UNKNOWN_SPRAY_FAILURE"))
                self._spray_failure_keys[key] = reason
                if self._current_point_id() == point_id and self._state == "RUNNING":
                    marking_number = self._active_marking_number
                    if marking_number is None:
                        marking_number = self._current_or_next_marking_number()
                    if marking_number is not None:
                        self._fail_marking_point(
                            marking_number,
                            self._current_path_index,
                            f"Spray failed at {point_id}: {reason}",
                        )
                    else:
                        self._enter_error(f"Spray failed at {point_id}: {reason}")

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
        # Do not trust an old FIXED value. The GPS fix status itself must be fresh.
        if self._age(self._last_gps_fix_rx_monotonic) > self.GPS_FIX_STALE_SEC:
            return False, "RTK GPS fix status stale"
        if self._gps_fix_type != 6:
            return False, f"RTK {self._rtk_state()} (fix_type={self._gps_fix_type})"

        # The correction-health topic and correction-age topic are checked
        # independently so one fresh topic cannot hide a stale second topic.
        if self._age(self._last_rtk_health_rx_monotonic) > self.RTK_STATUS_STALE_SEC:
            return False, "RTK correction health status stale"
        if not self._rtk_healthy:
            return False, "RTK correction stream unhealthy"

        if self._age(self._last_rtk_age_rx_monotonic) > self.RTK_STATUS_STALE_SEC:
            return False, "RTK correction age status stale"
        age = self._rtk_correction_age_sec
        if age is None or age < 0.0 or age > self.MAX_RTK_CORRECTION_AGE_SEC:
            return False, f"RTK correction age invalid ({age})"

        return True, "RTK FIXED + fresh corrections"

    def _monitor_runtime_rtk(self) -> None:
        """Soft-pause a RUNNING mission when RTK motion health is lost.

        Recovery while PAUSED is handled by _monitor_pause_recovery(), which
        checks the complete pre-motion health bundle (RTK + odom + FCU +
        backend heartbeat + spray readiness) before advertising Resume.
        """
        ok, reason = self._rtk_motion_ok()
        if self._state == "RUNNING" and not ok:
            # Soft pause only. E-stop stays released. cmd_vel_bridge/RPP must
            # output zero while mission_enable is false.
            self._mission_enable = False
            self._pause_reason = "RTK_LOST"
            self._resume_available = False
            self._state = "PAUSED"
            self._reset_arrival_state()
            self._publish_marking_active(False)
            self._publish_safety()
            self._last_message = f"{reason}; mission paused automatically"
            self._emit_system_event("RTK_PAUSED", reason)
            self._publish_status(force=True)

    def _motion_health_status(
        self, *, require_ready_state: bool, require_estop_released: bool = False
    ) -> tuple[bool, str]:
        """Return the authoritative health result required before enabling motion.

        This is deliberately the same bundle used by START/RESUME/NEXT:
        validated mission, fresh FCU state, fresh odometry, RTK FIXED, fresh
        correction status, acceptable correction age, backend heartbeat and
        spray-controller readiness.
        """
        if not self._navigation_path or not self._trajectory_ready:
            return False, "No validated prepared mission is ready"
        if require_estop_released and self._emergency_stop:
            return (
                False,
                "Emergency stop is active; release E-stop before enabling motion",
            )
        if require_ready_state and self._state != "READY":
            return False, f"Mission manager is not READY (state={self._state})"
        if not self._fcu_connected:
            return False, "PX4/MAVROS disconnected"
        if self._age(self._last_fcu_rx_monotonic) > self.FCU_STATE_STALE_SEC:
            return False, "PX4/MAVROS state stale"
        if self._age(self._last_odom_monotonic) > self.odom_timeout_sec:
            return False, "Local odometry unavailable/stale"
        rtk_ok, rtk_reason = self._rtk_motion_ok()
        if not rtk_ok:
            return False, f"RTK FIXED required: {rtk_reason}"
        if not self._backend_heartbeat_healthy:
            return False, "Backend heartbeat unhealthy at cmd_vel_bridge"
        if not self._spray_ready_for_mission_start():
            reason = self._spray_fault_reason or "spray controller not ready/stale"
            return False, f"Spray controller not ready: {reason}"
        return True, "Motion health OK"

    def _monitor_pause_recovery(self) -> None:
        """Update Resume availability without ever restarting motion automatically."""
        if self._state != "PAUSED":
            return

        # A latched E-stop must first be explicitly released.
        if self._pause_reason == "ESTOP" and self._emergency_stop:
            if self._resume_available:
                self._resume_available = False
                self._publish_status(force=True)
            return

        ok, reason = self._motion_health_status(
            require_ready_state=False, require_estop_released=True
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
            self._last_message = "RTK and motion health recovered; mission remains paused. You can resume now"
            self._emit_system_event(
                "RTK_RECOVERED", "Full motion health restored - ready to resume"
            )
        elif self._pause_reason == "ODOM_STALE":
            self._last_message = "Odometry and motion health recovered; mission remains paused. You can resume now"
            self._emit_system_event(
                "ODOM_RECOVERED", "Full motion health restored - ready to resume"
            )
        elif self._pause_reason == "ESTOP":
            self._last_message = "Emergency stop released and motion health is valid; mission remains paused. You can resume now"
            self._emit_system_event(
                "ESTOP_RECOVERY_READY", "Full motion health restored - ready to resume"
            )
        else:
            self._last_message = (
                "Mission remains paused; motion health is valid. You can resume now"
            )
        self._publish_status(force=True)

    # ==============================================================
    # Safety / PX4 orchestration
    # ==============================================================
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
            self._reset_arrival_state()
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

    def _best_effort_px4_safe_cleanup(self) -> list[str]:
        warnings: list[str] = []
        try:
            if self._px4_armed:
                self._request_arm(False)
        except Exception as exc:
            warnings.append(f"disarm warning: {exc}")
        try:
            if self._px4_mode != "MANUAL":
                self._request_px4_mode("MANUAL")
        except Exception as exc:
            warnings.append(f"MANUAL warning: {exc}")
        return warnings

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
                    raise RuntimeError("Previous mission is still RUNNING; STOP the current mission before START")
                if self._state == "PAUSED":
                    raise RuntimeError("Mission paused; use Resume")
                if self._state == "WAITING_FOR_NEXT":
                    raise RuntimeError("Manual mission waiting for NEXT")
                if self._state == "ERROR":
                    raise RuntimeError("Mission in ERROR; clear/prepare again")
            self._require_motion_health(require_ready_state=True)

            self._start_stage = "ZERO_SETPOINT_SETTLE"
            self._set_safety(emergency_stop=False, mission_enable=False)
            time.sleep(self.OFFBOARD_STREAM_SETTLE_SEC)

            self._start_stage = "SWITCHING_OFFBOARD"
            if self._px4_mode != "OFFBOARD":
                self._request_px4_mode("OFFBOARD")
            self._wait_for_vehicle_state(expected_mode="OFFBOARD")

            self._start_stage = "ARMING"
            if not self._px4_armed:
                self._request_arm(True)
            self._wait_for_vehicle_state(expected_mode="OFFBOARD", expected_armed=True)

            self._start_stage = "FINAL_CHECK"
            self._require_motion_health(require_ready_state=True)

            with self._lock:
                self._point_status = ["PENDING"] * len(self._mission_waypoints)
                self._reset_execution_progress()
                self._mission_run_id = uuid.uuid4().hex
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
            cleanup = self._best_effort_px4_safe_cleanup()
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
                self._start_debug = {"cleanup_warnings": cleanup}
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
        """Advance a MANUAL mission only after the full motion-health gate passes."""
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

            # NEXT can re-enable movement, so it must pass exactly the same
            # health bundle used by RESUME before mission_enable becomes true.
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

            # Re-check immediately before enabling motion in case RTK/odom
            # changed while OFFBOARD/arming was being established.
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
        """Skip the current marking without imposing an RTK/motion-health gate.

        SKIP changes mission sequencing only. If the mission was already
        PAUSED, it stays PAUSED and motion stays disabled.
        """
        with self._lock:
            if self._state not in {"RUNNING", "PAUSED", "WAITING_FOR_NEXT"}:
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

            self._point_status[marking_number] = "SKIPPED"
            path_index = self._marking_path_index_by_number[marking_number]
            self._emit_point_event("SKIPPED", marking_number, path_index)
            if self._current_path_index <= path_index:
                self._current_path_index = self._next_semantic_index(path_index + 1)

            self._reset_arrival_state()
            self._publish_marking_active(False)

            if self._finish_if_done():
                response.success = True
                response.message = f"P{marking_number+1:04d} skipped; mission completed"
                return response

            if previous_state == "PAUSED":
                # Preserve the reason that caused the pause (RTK_LOST,
                # ODOM_STALE, ESTOP, OPERATOR, ...). SKIP must never turn
                # motion back on.
                self._state = "PAUSED"
                self._pause_reason = previous_pause_reason
                self._resume_available = previous_resume_available
                self._set_safety(
                    emergency_stop=self._emergency_stop,
                    mission_enable=False,
                )
            elif previous_state == "WAITING_FOR_NEXT":
                self._state = "WAITING_FOR_NEXT"
                self._set_safety(emergency_stop=False, mission_enable=False)
            elif self._execution_mode == "MANUAL":
                # A RUNNING MANUAL mission waits for the operator after SKIP.
                self._state = "WAITING_FOR_NEXT"
                self._pause_reason = None
                self._resume_available = False
                self._set_safety(emergency_stop=False, mission_enable=False)
            else:
                # RUNNING AUTO stays RUNNING. Runtime RTK/odom monitoring
                # continues to own motion safety.
                self._state = "RUNNING"

            self._last_message = f"P{marking_number+1:04d} skipped"
            self._publish_goal()
            self._publish_runtime_path()
            self._publish_status(force=True)

        response.success = True
        response.message = self._last_message
        return response

    def _stop_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self._set_safety(emergency_stop=True, mission_enable=False)
        cleanup = self._best_effort_px4_safe_cleanup()
        with self._lock:
            if self._navigation_path:
                self._point_status = ["PENDING"] * len(self._mission_waypoints)
            self._reset_execution_progress()
            self._mission_run_id = ""
            self._spray_success_keys.clear()
            self._spray_failure_keys.clear()
            self._state = (
                "READY" if self._navigation_path and self._trajectory_ready else "EMPTY"
            )
            self._last_error = None
            self._last_message = "Mission stopped; prepared mission retained"
            self._start_stage = "IDLE"
            self._publish_marking_active(False)
            self._publish_mission_complete(False)
            self._publish_runtime_path()
            self._publish_goal()
            self._publish_status(force=True)
        response.success = True
        response.message = self._last_message
        if cleanup:
            response.message += "; " + "; ".join(cleanup)
        return response

    def _clear_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._lock:
            if self._state in {"RUNNING", "PAUSED", "WAITING_FOR_NEXT"}:
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

    def _reset_arrival_state(self) -> None:
        self._arrival_settle_started = None
        self._arrival_settle_elapsed_sec = 0.0
        self._marking_hold_started = None
        self._marking_hold_elapsed_sec = 0.0
        self._spray_request_started = None
        self._marking_error_valid = False
        self._marking_xtrack_m = math.inf
        self._marking_along_error_m = math.inf
        self._marking_combined_error_m = math.inf
        self._marking_radial_error_m = math.inf

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
        self._state = "EMPTY"
        self._mission_run_id = ""
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

        if marking_number == 0:
            # P1 has no previous CSV marking segment. Reporting only.
            return 0.0, radial, radial, radial

        start = self._mission_waypoints[marking_number - 1]
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

    def _emit_point_event(
        self,
        event: str,
        marking_number: int,
        path_index: int,
        *,
        reason: Optional[str] = None,
    ) -> None:
        payload = {
            "event": event,
            "mission_run_id": self._mission_run_id or None,
            "point_id": f"P{marking_number+1:04d}",
            "point_index": marking_number,
            "path_index": path_index,
            "state": self._state,
            "timestamp_unix_ns": time.time_ns(),
            "radial_error_m": (
                round(self._marking_radial_error_m, 6)
                if math.isfinite(self._marking_radial_error_m)
                else None
            ),
            "reason": reason,
            "xtrack_m": (
                round(self._marking_xtrack_m, 6)
                if math.isfinite(self._marking_xtrack_m)
                else None
            ),
            "along_error_m": (
                round(self._marking_along_error_m, 6)
                if math.isfinite(self._marking_along_error_m)
                else None
            ),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.point_event_pub.publish(msg)

    def _fail_marking_point(
        self, marking_number: int, path_index: int, reason: str
    ) -> None:
        """Record a terminal FAILED point before entering mission ERROR."""
        if 0 <= marking_number < len(self._point_status):
            if self._point_status[marking_number] not in self.TERMINAL_POINT_STATES:
                self._point_status[marking_number] = "FAILED"
        self._enter_error(reason)
        self._emit_point_event(
            "FAILED", marking_number, path_index, reason=reason
        )
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

    def _status_payload(self) -> dict[str, Any]:
        completed = self._point_status.count("COMPLETED")
        skipped = self._point_status.count("SKIPPED")
        failed = self._point_status.count("FAILED")
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
            "remaining_points": max(0, total - processed),
            "progress_percent": round(100.0 * processed / total, 2) if total else 0.0,
            "point_status": list(self._point_status),
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
            # Compatibility names retained for current frontend/backend.
            # These now represent the REAL marking hold, not arrival settle.
            "hold_elapsed_sec": round(self._marking_hold_elapsed_sec, 3),
            "hold_required_sec": self.marking_hold_sec,
            "marking_error_mode": "RADIAL_2D",
            "marking_tolerance_m": self.marking_tolerance_m,
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

    def _complete_mission(self) -> None:
        self._set_safety(emergency_stop=False, mission_enable=False)
        self._state = "COMPLETED"
        self._pause_reason = None
        self._resume_available = False
        self._last_message = "Mission completed"
        self._publish_marking_active(False)
        self._publish_mission_complete(True)
        self._publish_status(force=True)

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
        with self._lock:
            self._publish_status()

            if not self._navigation_path:
                return

            # Runtime RTK monitor owns automatic RTK pause. While PAUSED, a
            # separate recovery monitor checks the complete motion-health
            # bundle and only advertises whether Resume is available.
            self._monitor_runtime_rtk()
            self._monitor_pause_recovery()

            if self._state not in {"RUNNING", "PAUSED", "WAITING_FOR_NEXT"}:
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
                inside_extension_radius = (
                    distance <= self.dummy_arrival_tolerance_m
                )
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

            (
                self._marking_xtrack_m,
                self._marking_along_error_m,
                self._marking_combined_error_m,
                self._marking_radial_error_m,
            ) = self._marking_error_components(marking_number)

            inside_30mm = (
                math.isfinite(self._marking_radial_error_m)
                and self._marking_radial_error_m <= self.marking_tolerance_m
            )
            stationary = self._speed_mps <= self.stationary_speed_tolerance_mps
            self._marking_error_valid = inside_30mm

            # Before spray is requested, BOTH conditions must remain true for
            # arrival_settle_sec continuously. Any drift/motion resets settle.
            if self._spray_request_started is None:
                if not (inside_30mm and stationary):
                    self._arrival_settle_started = None
                    self._arrival_settle_elapsed_sec = 0.0
                    self._publish_marking_active(False)
                    return

                now = time.monotonic()
                if self._arrival_settle_started is None:
                    self._arrival_settle_started = now
                    self._arrival_settle_elapsed_sec = 0.0
                    self._last_message = (
                        f"P{marking_number+1:04d} inside {self.marking_tolerance_m*1000.0:.0f}mm "
                        "and stationary; verifying stable arrival"
                    )
                    self._publish_status(force=True)
                    return

                self._arrival_settle_elapsed_sec = now - self._arrival_settle_started
                if self._arrival_settle_elapsed_sec < self.arrival_settle_sec:
                    return

                # Stable exact arrival accepted. Mission Manager now asks the
                # separate spray controller to perform its own timed transaction.
                self._marking_hold_started = now
                self._marking_hold_elapsed_sec = 0.0
                self._spray_request_started = now
                self._publish_marking_active(True)
                self._last_message = (
                    f"P{marking_number+1:04d} arrival validated: "
                    f"radial={self._marking_radial_error_m*1000.0:.1f}mm; "
                    f"3.0s marking hold started; spray enabled"
                )
                self._publish_status(force=True)

            point_id = f"P{marking_number+1:04d}"
            spray_key = (self._mission_run_id, point_id)

            # Once a REAL marking point has passed the exact 30 mm + stationary
            # arrival gate, the physical spray transaction is committed. Keep
            # /marking_active TRUE at CONTROL_HZ until spray SUCCESS/FAILED or a
            # real mission safety gate (E-stop, RTK pause, odom stale, mission
            # disable, PX4 loss) removes permission elsewhere in this manager.
            #
            # Do NOT fail an already-started spray merely because EKF/GNSS
            # position or velocity jitters outside the 30 mm / 0.01 m/s arrival
            # gate for one control sample. The rover is already commanded ZERO by
            # RPP while marking_active is true, and failing here used to create:
            #   P1 physically sprayed -> MM ERROR -> spray fault latch -> no P2.
            # The 30 mm tolerance remains mandatory BEFORE the spray is started.
            if self._spray_request_started is not None:
                self._publish_marking_active(True)

            # Keep the rover stopped for the complete marking hold. The spray
            # controller may finish its short 0.5 s PRESS early, but this point
            # cannot become COMPLETED until the full marking_hold_sec has elapsed.
            if self._marking_hold_started is not None:
                self._marking_hold_elapsed_sec = max(
                    0.0, time.monotonic() - self._marking_hold_started
                )

            if self.spray_required:
                failure = self._spray_failure_keys.get(spray_key)
                if failure is not None:
                    self._fail_marking_point(
                        marking_number,
                        self._current_path_index,
                        f"Spray failed at {point_id}: {failure}",
                    )
                    return
                if spray_key not in self._spray_success_keys:
                    if (
                        self._spray_request_started is not None
                        and time.monotonic() - self._spray_request_started
                        > self.spray_confirmation_timeout_sec
                    ):
                        self._publish_marking_active(False)
                        self._fail_marking_point(
                            marking_number,
                            self._current_path_index,
                            f"Spray confirmation timeout at {point_id}",
                        )
                    return

            # Spray SUCCESS may arrive well before the requested 3.0 s marking
            # hold is finished (normal spray ON time is only 0.5 s). Keep
            # /marking_active TRUE so RPP continues commanding exact ZERO until
            # the complete marking hold expires.
            if self._marking_hold_elapsed_sec < self.marking_hold_sec:
                remaining = max(0.0, self.marking_hold_sec - self._marking_hold_elapsed_sec)
                self._last_message = (
                    f"{point_id} spray complete; holding position "
                    f"{remaining:.2f}s more"
                )
                return

            # Full marking hold is complete and spray has succeeded. Only now
            # clear marking_active and advance to the next marking point.
            self._publish_marking_active(False)
            self._point_status[marking_number] = "COMPLETED"
            self._last_completed_marking_number = marking_number
            self._emit_point_event(
                "COMPLETED", marking_number, self._current_path_index
            )
            final_mm = self._marking_radial_error_m * 1000.0
            self._current_path_index = self._next_semantic_index(
                self._current_path_index + 1
            )
            self._reset_arrival_state()
            self._publish_runtime_path()

            if self._finish_if_done():
                return

            if self._execution_mode == "MANUAL":
                self._state = "WAITING_FOR_NEXT"
                self._set_safety(emergency_stop=False, mission_enable=False)
                self._last_message = (
                    f"{point_id} COMPLETED at {final_mm:.1f}mm; waiting for NEXT"
                )
            else:
                self._last_message = f"{point_id} COMPLETED at {final_mm:.1f}mm; continuing automatically"
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