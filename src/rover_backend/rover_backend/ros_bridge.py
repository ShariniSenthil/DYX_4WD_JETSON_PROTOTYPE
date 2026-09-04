"""ROS 2 bridge for the DYX 4WD Rover Backend.

This module is the only backend component that talks directly to ROS 2.
It:

- publishes the backend heartbeat required by cmd_vel_bridge;
- mirrors /mission_enable and /emergency_stop owned by mission_manager;
- subscribes to MAVROS, RTK, trajectory and mission-manager status;
- calls trajectory-generator and mission-manager Trigger services;
- forwards mission lifecycle commands to mission_manager;
- mirrors ROS state into the thread-safe RoverState used by REST/Socket.IO;
- always starts and stops in a safe, non-driving state.

It never reads or rewrites mission.csv and never performs path planning.
"""

from __future__ import annotations

import copy
import json
import logging
import math
import os
import struct
import tempfile
import threading
import time
import uuid
from pathlib import Path
from typing import Any
from typing import Callable
from typing import Optional

import rclpy

from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GPSRAW
from mavros_msgs.msg import Mavlink
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import UInt64
from mission_manager_interfaces.srv import ReleaseEmergencyStop
from std_srvs.srv import Trigger

from rover_backend.config import settings
from rover_backend.mission_report import MissionReportError
from rover_backend.mission_report import StaleMissionTerminalEvent
from rover_backend.mission_report import mission_report_store
from rover_backend.mission_store import mission_store
from rover_backend.rtk_mavros_readiness import (
    evaluate_mavros_rtcm_readiness,
)
from rover_backend.state import rover_state
from rover_backend.state import utc_now_iso

LOGGER = logging.getLogger(__name__)


RTCM_INJECTION_TOPIC = "/mavros/gps_rtk/send_rtcm"
PX4_EARTH_RADIUS_M = 6_371_000.0


class RosServiceOutcomeUnknownError(RuntimeError):
    """The client timed out after dispatch, so server completion is unknown."""

    outcome = "UNKNOWN"
    retry_safe = False


def _notify_authoritative_state_changed() -> None:
    """Notify realtime lazily to avoid a ros_bridge/realtime import cycle."""

    try:
        from rover_backend.realtime import notify_authoritative_state_changed

        notify_authoritative_state_changed()
    except ImportError:
        # Realtime may not be imported during isolated ROS-node tests.
        return


GPS_FIX_NAMES: dict[int, str] = {
    0: "NO_GPS",
    1: "NO_FIX",
    2: "2D_FIX",
    3: "3D_FIX",
    4: "DGPS",
    5: "RTK_FLOAT",
    6: "RTK_FIXED",
    7: "STATIC_FIXED",
    8: "PPP",
}


MISSION_MANAGER_COMMANDS = {
    "start",
    "pause",
    "resume",
    "next_point",
    "skip_point",
    "stop",
    "clear",
    "emergency_stop",
    "release_emergency_stop",
}


def _finite_float(
    value: Any,
    default: float | None = None,
) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default

    if not math.isfinite(result):
        return default

    return result


def _safe_int(
    value: Any,
    default: int = 0,
) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _px4_enu_to_geodetic(
    *,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
    east_m: float,
    north_m: float,
) -> tuple[float, float]:
    """Inverse PX4 MapProjection for one ROS/MAVROS ENU point.

    PX4 MapProjection::reproject() uses local NED horizontal coordinates:
        x = north
        y = east

    The rover /nav_path uses ROS/MAVROS ENU:
        x = east
        y = north

    Therefore north_m is PX4 x and east_m is PX4 y.
    """

    values = (
        origin_latitude_deg,
        origin_longitude_deg,
        east_m,
        north_m,
    )

    if not all(math.isfinite(value) for value in values):
        raise ValueError("PX4 projection values must be finite")

    if not -90.0 <= origin_latitude_deg <= 90.0:
        raise ValueError("PX4 origin latitude must be within [-90, 90]")

    if not -180.0 <= origin_longitude_deg <= 180.0:
        raise ValueError("PX4 origin longitude must be within [-180, 180]")

    reference_latitude = math.radians(origin_latitude_deg)
    reference_longitude = math.radians(origin_longitude_deg)

    x_rad = north_m / PX4_EARTH_RADIUS_M
    y_rad = east_m / PX4_EARTH_RADIUS_M
    central_angle = math.hypot(x_rad, y_rad)

    if central_angle <= 1.0e-15:
        return (
            origin_latitude_deg,
            origin_longitude_deg,
        )

    sin_c = math.sin(central_angle)
    cos_c = math.cos(central_angle)
    sin_reference = math.sin(reference_latitude)
    cos_reference = math.cos(reference_latitude)

    latitude = math.asin(
        cos_c * sin_reference
        + (x_rad * sin_c * cos_reference) / central_angle
    )

    longitude = reference_longitude + math.atan2(
        y_rad * sin_c,
        central_angle * cos_reference * cos_c
        - x_rad * sin_reference * sin_c,
    )

    return (
        math.degrees(latitude),
        math.degrees(longitude),
    )


def _reliable_qos(
    *,
    depth: int = 10,
    retained: bool = False,
) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL if retained else DurabilityPolicy.VOLATILE
        ),
    )


def _sensor_qos(
    *,
    depth: int = 10,
) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _json_object(
    raw_value: str,
) -> dict[str, Any] | None:
    try:
        value = json.loads(raw_value)
    except (TypeError, json.JSONDecodeError):
        return None

    if not isinstance(value, dict):
        return None

    return value


def _atomic_write_json(
    destination: Path,
    payload: dict[str, Any],
) -> None:
    """Atomically persist a small runtime-state JSON document."""

    destination.parent.mkdir(
        parents=True,
        exist_ok=True,
        mode=0o700,
    )

    encoded = (
        json.dumps(
            payload,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")

    descriptor: int | None = None
    temporary_path: str | None = None

    try:
        descriptor, temporary_path = tempfile.mkstemp(
            prefix=f".{destination.name}.",
            suffix=".tmp",
            dir=str(destination.parent),
        )

        with os.fdopen(descriptor, "wb") as handle:
            descriptor = None
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())

        try:
            os.chmod(temporary_path, 0o600)
        except PermissionError:
            pass

        os.replace(
            temporary_path,
            destination,
        )
        temporary_path = None

    finally:
        if descriptor is not None:
            try:
                os.close(descriptor)
            except OSError:
                pass

        if temporary_path is not None and os.path.exists(temporary_path):
            try:
                os.unlink(temporary_path)
            except OSError:
                pass


class RoverBackendRosNode(Node):
    """Backend ROS node and safety-command owner."""

    SAFETY_PUBLISH_HZ = 2.0

    ROS_MESSAGE_STALE_SEC = 5.0
    FCU_STATE_STALE_SEC = 3.0
    POSITION_STALE_SEC = 3.0
    RTK_STATUS_STALE_SEC = 15.0
    MAX_RTK_CORRECTION_AGE_SEC = 2.0

    SERVICE_DISCOVERY_TIMEOUT_SEC = 3.0
    SERVICE_RESPONSE_TIMEOUT_SEC = 5.0
    MANAGER_RESPONSE_TIMEOUT_SEC = {
        # START/RESUME/NEXT can each include OFFBOARD and ARM service
        # discovery/response waits plus state confirmations. These contracts
        # intentionally exceed Mission Manager's legitimate worst case.
        "start": 30.0,
        "resume": 30.0,
        "next_point": 30.0,
        # STOP can include DISARM service discovery/response and confirmation.
        "stop": 20.0,
        "pause": 8.0,
        "skip_point": 8.0,
        "clear": 8.0,
        # Keep hard-stop acknowledgement latency bounded independently of
        # ordinary long-running commands.
        "emergency_stop": 3.0,
        "release_emergency_stop": 5.0,
    }

    OFFBOARD_STREAM_SETTLE_SEC = 0.60
    COMMAND_SETTLE_SEC = 0.10
    EXECUTION_MODE_ACK_TIMEOUT_SEC = 1.50
    MANAGER_STATE_ACK_TIMEOUT_SEC = 1.50
    SPRAY_CONFIG_ACK_TIMEOUT_SEC = 1.50

    # These must match the configured AUX5 output endpoints in PX4.
    SPRAY_PWM_MIN_US = 1000.0
    SPRAY_PWM_MAX_US = 2000.0

    def __init__(self) -> None:
        super().__init__("rover_backend")

        if settings.backend_heartbeat_hz <= 0.0:
            raise ValueError("backend_heartbeat_hz must be greater than zero")

        self._command_lock = threading.RLock()
        self._runtime_lock = threading.RLock()
        # Mission Manager status arrives on a ROS executor thread. Durable
        # runtime persistence performs flush/fsync and must never block that
        # callback. Keep only the newest pending status while the worker is
        # writing an older snapshot.
        self._runtime_persist_condition = threading.Condition()
        self._runtime_persist_pending: tuple[str, dict[str, Any]] | None = None
        self._runtime_persist_stopping = False
        self._runtime_persist_thread = threading.Thread(
            target=self._runtime_persist_worker,
            name="mission-runtime-persist",
            daemon=True,
        )
        self._runtime_persist_thread.start()

        # Point events update the in-memory result map synchronously, but the
        # durable live-report checkpoint performs fsync and must not run in a
        # ROS subscription callback. A one-slot latest-state queue is enough:
        # each checkpoint reads the accumulated point_results map, so
        # coalescing cannot lose an already-recorded terminal point result.
        self._report_checkpoint_condition = threading.Condition()
        self._report_checkpoint_pending_mission_id: str | None = None
        self._report_checkpoint_stopping = False
        self._report_checkpoint_thread = threading.Thread(
            target=self._report_checkpoint_worker,
            name="mission-report-checkpoint",
            daemon=True,
        )
        self._report_checkpoint_thread.start()

        self._terminal_cleanup_lock = threading.Lock()
        self._terminal_cleanup_keys: set[str] = set()

        # The backend always starts safe.
        self._mission_enable = False
        self._emergency_stop = True
        self._heartbeat_sequence = 0

        self._fcu_connected = False
        self._armed = False
        self._mode = "UNKNOWN"
        self._gps_fix_type = 0
        self._rtk_healthy = False
        self._rtk_correction_age_sec: float | None = None

        self._px4_origin_latitude_deg: float | None = None
        self._px4_origin_longitude_deg: float | None = None

        # Cached MAVROS RTCM endpoint readiness. ROS graph inspection happens
        # only inside this ROS node; non-ROS supervisor code reads this cache.
        self._mavros_rtcm_ready = False
        self._mavros_rtcm_subscriber_count = 0

        self._trajectory_ready = False
        self._trajectory_error: str | None = None

        # Track whether mission_manager accepted all rebuilt
        # trajectory products.
        self._mission_manager_state = "EMPTY"
        self._mission_manager_ready = False
        self._mission_manager_error: str | None = None
        self._mission_execution_mode = "AUTO"
        # True from /trajectory_generator/prepare acceptance until the complete
        # multipart trajectory has been accepted by mission_manager. While this
        # is true, an intermediate mission_manager EMPTY snapshot must not make
        # the REST/UI mission look unloaded.
        self._preparation_in_progress = False
        # Latest status from the separate spray_controller node.
        self._spray_status: dict[str, Any] = {}
        self._spray_status_monotonic: float | None = None

        self._last_ros_message_monotonic: float | None = None
        self._last_fcu_message_monotonic: float | None = None
        self._last_position_message_monotonic: float | None = None
        self._last_rtk_message_monotonic: float | None = None
        self._last_rpp_debug_monotonic: float | None = None
        self._last_rpp_debug_sequence: int | None = None
        self._rpp_debug_dropped_frames = 0
        self._rpp_debug_callback_group = MutuallyExclusiveCallbackGroup()

        retained_qos = _reliable_qos(
            depth=1,
            retained=True,
        )

        command_qos = _reliable_qos(
            depth=1,
            retained=False,
        )

        normal_qos = _reliable_qos(
            depth=10,
            retained=False,
        )

        sensor_qos = _sensor_qos(
            depth=10,
        )
        debug_qos = _sensor_qos(
            depth=1,
        )

        # ======================================================
        # Safety and heartbeat publishers
        # ======================================================

        self._heartbeat_pub = self.create_publisher(
            UInt64,
            "/rover_backend/heartbeat",
            command_qos,
        )

        # Mission execution mode is independent of the PX4 vehicle mode.
        # AUTO runs continuously; MANUAL waits for NEXT/SKIP after each mark.
        self._execution_mode_pub = self.create_publisher(
            String,
            "/mission_manager/execution_mode",
            retained_qos,
        )
        self._spray_config_pub = self.create_publisher(
            String,
            "/spray/config",
            retained_qos,
        )

        # ======================================================
        # MAVROS and RTK subscriptions
        # ======================================================

        self.create_subscription(
            State,
            "/mavros/state",
            self._mavros_state_callback,
            normal_qos,
        )

        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/raw/fix",
            self._global_position_callback,
            sensor_qos,
        )

        self.create_subscription(
            GeoPointStamped,
            "/mavros/global_position/gp_origin",
            self._gp_origin_callback,
            retained_qos,
        )

        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._fused_global_position_callback,
            sensor_qos,
        )

        self.create_subscription(
            GPSRAW,
            "/mavros/gpsstatus/gps1/raw",
            self._gps_status_callback,
            sensor_qos,
        )

        self.create_subscription(
            Mavlink,
            "/uas1/mavlink_source",
            self._mavlink_estimator_status_callback,
            sensor_qos,
        )

        self.create_subscription(
            Float64,
            "/mavros/global_position/compass_hdg",
            self._heading_callback,
            sensor_qos,
        )

        self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self._local_odom_callback,
            sensor_qos,
        )

        self.create_subscription(
            BatteryState,
            "/mavros/battery",
            self._battery_callback,
            sensor_qos,
        )

        self.create_subscription(
            Bool,
            "/rtk_correction_bridge/healthy",
            self._rtk_health_callback,
            retained_qos,
        )

        self.create_subscription(
            Float32,
            "/rtk_correction_bridge/correction_age_sec",
            self._rtk_correction_age_callback,
            retained_qos,
        )

        self.create_subscription(
            String,
            "/rtk_correction_bridge/status",
            self._rtk_stream_status_callback,
            retained_qos,
        )

        self.create_subscription(
            Bool,
            "/cmd_vel_bridge/backend_heartbeat_healthy",
            self._heartbeat_health_callback,
            retained_qos,
        )

        self.create_subscription(
            Bool,
            "/mission_enable",
            self._mission_enable_state_callback,
            command_qos,
        )

        self.create_subscription(
            Bool,
            "/emergency_stop",
            self._emergency_stop_state_callback,
            command_qos,
        )

        # ======================================================
        # Trajectory and mission subscriptions
        # ======================================================

        self.create_subscription(
            Bool,
            "/trajectory_generator/ready",
            self._trajectory_ready_callback,
            retained_qos,
        )

        self.create_subscription(
            String,
            "/trajectory_generator/status",
            self._trajectory_status_callback,
            retained_qos,
        )

        self.create_subscription(
            NavPath,
            "/nav_path",
            self._nav_path_callback,
            retained_qos,
        )

        self.create_subscription(
            NavPath,
            "/mission_waypoints",
            self._mission_waypoints_callback,
            retained_qos,
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
            normal_qos,
        )

        self.create_subscription(
            Bool,
            "/marking_active",
            self._marking_active_callback,
            command_qos,
        )

        self.create_subscription(
            Bool,
            "/alignment_active",
            self._alignment_active_callback,
            command_qos,
        )

        self.create_subscription(
            PoseStamped,
            "/active_waypoint",
            self._active_waypoint_callback,
            command_qos,
        )

        self.create_subscription(
            String,
            "/rpp/accuracy",
            self._accuracy_callback,
            retained_qos,
        )
        self.create_subscription(
            String,
            "/rpp/debug",
            self._rpp_debug_callback,
            debug_qos,
            callback_group=self._rpp_debug_callback_group,
        )
        self.create_subscription(
            String,
            "/spray/status",
            self._spray_status_callback,
            retained_qos,
        )

        # ======================================================
        # Service clients
        # ======================================================

        self._trajectory_prepare_client = self.create_client(
            Trigger,
            "/trajectory_generator/prepare",
        )

        self._trajectory_clear_client = self.create_client(
            Trigger,
            "/trajectory_generator/clear",
        )

        self._mission_manager_clients = {
            command: self.create_client(
                Trigger,
                f"/mission_manager/{command}",
            )
            for command in sorted(MISSION_MANAGER_COMMANDS)
            if command != "release_emergency_stop"
        }

        self._release_emergency_stop_client = self.create_client(
            ReleaseEmergencyStop,
            "/mission_manager/release_emergency_stop",
        )

        # ======================================================
        # Timers and safe initialization
        # ======================================================

        self.create_timer(
            1.0 / settings.backend_heartbeat_hz,
            self._publish_heartbeat,
        )

        self.create_timer(
            1.0,
            self._stale_monitor,
        )
        self.create_timer(
            0.1,
            self._rpp_debug_stale_monitor,
            callback_group=self._rpp_debug_callback_group,
        )

        rover_state.mark_ros_node_started()
        rover_state.force_safe_runtime_state("BACKEND_ROS_BRIDGE_STARTUP")

        # Publish backend heartbeat immediately. mission_manager owns the
        # actual motion safety topics.
        self._publish_heartbeat()

        self.get_logger().warn("===== ROVER BACKEND ROS BRIDGE STARTED =====")
        self.get_logger().warn("Heartbeat topic: /rover_backend/heartbeat")

    # ==========================================================
    # Time and state helpers
    # ==========================================================

    @staticmethod
    def _monotonic_age(
        timestamp: float | None,
    ) -> float:
        if timestamp is None:
            return math.inf

        return max(
            0.0,
            time.monotonic() - timestamp,
        )

    def _mark_ros_message(self) -> None:
        self._last_ros_message_monotonic = time.monotonic()
        rover_state.mark_ros_message_received()

    def _stale_monitor(self) -> None:
        # This timer runs on the single rover-backend-ros-spin thread that
        # dispatches ALL of this node's ROS work (subscriptions, timers,
        # services). An uncaught exception here doesn't just skip one tick
        # -- it kills that thread permanently for the life of the process,
        # silently taking every ROS callback (including the E-stop/mission
        # command services) down with it while the HTTP/FastAPI side keeps
        # running and looks healthy. Fail closed and log instead of raising.
        try:
            self._stale_monitor_impl()
        except Exception:
            LOGGER.exception("_stale_monitor tick failed; continuing")

    def _stale_monitor_impl(self) -> None:
        ros_age = self._monotonic_age(self._last_ros_message_monotonic)
        fcu_age = self._monotonic_age(self._last_fcu_message_monotonic)
        position_age = self._monotonic_age(self._last_position_message_monotonic)
        rtk_age = self._monotonic_age(self._last_rtk_message_monotonic)

        rover_state.update(
            "ros",
            connected=(ros_age <= self.ROS_MESSAGE_STALE_SEC),
            error=(
                None
                if ros_age <= self.ROS_MESSAGE_STALE_SEC
                else "ROS messages are stale"
            ),
        )

        rover_state.update(
            "vehicle",
            connected=bool(self._fcu_connected and fcu_age <= self.FCU_STATE_STALE_SEC),
        )

        self._refresh_mavros_rtcm_readiness(
            fcu_age_sec=fcu_age,
        )

        if position_age > self.POSITION_STALE_SEC:
            rover_state.update(
                "vehicle",
                ground_speed_mps=0.0,
                linear_speed_mps=0.0,
            )

        if rtk_age > self.RTK_STATUS_STALE_SEC:
            self._rtk_healthy = False
            rover_state.update(
                "rtk",
                healthy=False,
                stream_connected=False,
                stream_state="STALE",
                status="STALE",
            )

    def _rpp_debug_stale_monitor(self) -> None:
        rpp_debug_age = self._monotonic_age(self._last_rpp_debug_monotonic)
        rover_state.update(
            "accuracy",
            rpp_debug_receive_age_ms=(
                rpp_debug_age * 1000.0 if math.isfinite(rpp_debug_age) else None
            ),
            rpp_debug_stream_fresh=(rpp_debug_age <= 0.25),
            rpp_debug_dropped_frames=self._rpp_debug_dropped_frames,
        )

    # ==========================================================
    # MAVROS and RTK callbacks
    # ==========================================================

    def _mavros_state_callback(
        self,
        message: State,
    ) -> None:
        self._mark_ros_message()
        self._last_fcu_message_monotonic = time.monotonic()

        self._fcu_connected = bool(message.connected)
        self._armed = bool(message.armed)
        self._mode = str(message.mode or "UNKNOWN").strip().upper()

        rover_state.update(
            "vehicle",
            connected=self._fcu_connected,
            armed=self._armed,
            mode=self._mode,
            system_status=int(message.system_status),
        )

        self._refresh_mavros_rtcm_readiness(
            fcu_age_sec=0.0,
        )

    def _refresh_mavros_rtcm_readiness(
        self,
        *,
        fcu_age_sec: float | None = None,
    ) -> None:
        """Refresh the cached MAVROS RTCM injection start gate."""

        if fcu_age_sec is None:
            fcu_age_sec = self._monotonic_age(
                self._last_fcu_message_monotonic
            )

        # No /mavros/state message has arrived yet -- _monotonic_age(None)
        # returns +inf. evaluate_mavros_rtcm_readiness() correctly rejects
        # a non-finite age; on a cold start _stale_monitor's first tick can
        # fire before MAVROS finishes connecting, so this is the normal
        # startup case, not an error. Treat it as "not ready yet" directly
        # instead of forwarding inf into the strict validator.
        if not math.isfinite(fcu_age_sec):
            with self._runtime_lock:
                self._mavros_rtcm_ready = False
                self._mavros_rtcm_subscriber_count = 0
            return

        try:
            subscriber_count = int(
                self.count_subscribers(
                    RTCM_INJECTION_TOPIC
                )
            )
        except Exception:
            # ROS graph discovery is an availability signal. If graph access
            # fails during startup/shutdown, fail closed rather than allowing
            # an RTK worker to start against an unknown endpoint.
            subscriber_count = 0

        ready = evaluate_mavros_rtcm_readiness(
            fcu_connected=bool(
                self._fcu_connected
            ),
            fcu_state_age_sec=float(
                fcu_age_sec
            ),
            rtcm_subscriber_count=(
                subscriber_count
            ),
            stale_sec=self.FCU_STATE_STALE_SEC,
        )

        with self._runtime_lock:
            self._mavros_rtcm_ready = ready
            self._mavros_rtcm_subscriber_count = (
                subscriber_count
            )

        rover_state.update(
            "rtk",
            mavros_ready=ready,
            mavros_rtcm_subscribers=(
                subscriber_count
            ),
        )

    def rtk_mavros_ready(self) -> bool:
        """Return cached MAVROS RTCM injection readiness."""

        with self._runtime_lock:
            return bool(
                self._mavros_rtcm_ready
            )

    def _global_position_callback(
        self,
        message: NavSatFix,
    ) -> None:
        self._mark_ros_message()
        latitude = _finite_float(message.latitude)
        longitude = _finite_float(message.longitude)
        altitude = _finite_float(message.altitude)

        if latitude is not None and not -90.0 <= latitude <= 90.0:
            latitude = None

        if longitude is not None and not -180.0 <= longitude <= 180.0:
            longitude = None

        rover_state.update(
            "gps",
            raw_latitude=latitude,
            raw_longitude=longitude,
            raw_altitude_m=altitude,
        )

    def _gp_origin_callback(
        self,
        message: GeoPointStamped,
    ) -> None:
        """Cache the exact PX4 EKF local-map geographic origin."""

        self._mark_ros_message()

        latitude = _finite_float(message.position.latitude)
        longitude = _finite_float(message.position.longitude)

        if (
            latitude is None
            or longitude is None
            or not -90.0 <= latitude <= 90.0
            or not -180.0 <= longitude <= 180.0
            or (latitude == 0.0 and longitude == 0.0)
        ):
            LOGGER.warning(
                "Ignoring invalid PX4 gp_origin: lat=%s lon=%s",
                latitude,
                longitude,
            )
            return

        with self._runtime_lock:
            self._px4_origin_latitude_deg = latitude
            self._px4_origin_longitude_deg = longitude

        rover_state.update(
            "mission",
            projection_origin_latitude_deg=latitude,
            projection_origin_longitude_deg=longitude,
            path_projection="PX4_MAP_PROJECTION_REPROJECT",
        )

        # /nav_path can arrive before gp_origin on process startup.
        # If a local preview is already cached, decorate it immediately.
        cached_preview = rover_state.section("mission").get(
            "navigation_path_preview",
            [],
        )

        if isinstance(cached_preview, list) and cached_preview:
            projected_preview: list[dict[str, Any]] = []

            for fallback_index, cached_point in enumerate(cached_preview):
                if not isinstance(cached_point, dict):
                    continue

                east_m = _finite_float(cached_point.get("x"))
                north_m = _finite_float(cached_point.get("y"))

                if east_m is None or north_m is None:
                    continue

                projected_preview.append(
                    self._navigation_preview_point(
                        index=_safe_int(
                            cached_point.get("index"),
                            fallback_index,
                        ),
                        east_m=east_m,
                        north_m=north_m,
                    )
                )

            rover_state.update(
                "mission",
                navigation_path_preview=projected_preview,
            )

    def _fused_global_position_callback(
        self,
        message: NavSatFix,
    ) -> None:
        """Use PX4 EKF fused global as the live map-position authority."""

        self._mark_ros_message()
        self._last_position_message_monotonic = time.monotonic()

        latitude = _finite_float(message.latitude)
        longitude = _finite_float(message.longitude)
        altitude = _finite_float(message.altitude)

        if latitude is not None and not -90.0 <= latitude <= 90.0:
            latitude = None

        if longitude is not None and not -180.0 <= longitude <= 180.0:
            longitude = None

        rover_state.update(
            "position",
            latitude=latitude,
            longitude=longitude,
            altitude_m=altitude,
            global_position_source="PX4_FUSED_GLOBAL",
        )

    def _gps_status_callback(
        self,
        message: GPSRAW,
    ) -> None:
        self._mark_ros_message()

        self._gps_fix_type = int(message.fix_type)

        fix_name = GPS_FIX_NAMES.get(
            self._gps_fix_type,
            f"UNKNOWN_{self._gps_fix_type}",
        )

        # MAVLink GPS_RAW_INT eph/epv values are DOP values scaled by 100.
        hdop = float(message.eph) / 100.0 if int(message.eph) > 0 else None
        vdop = float(message.epv) / 100.0 if int(message.epv) > 0 else None

        # GPS_RAW_INT horizontal / vertical position accuracy.
        #
        # MAVROS GPSRAW exposes h_acc and v_acc in millimetres.
        # Convert them to metres for backend/frontend telemetry.
        h_acc_raw = int(
            getattr(
                message,
                "h_acc",
                0,
            )
        )

        v_acc_raw = int(
            getattr(
                message,
                "v_acc",
                0,
            )
        )

        horizontal_accuracy_m = float(h_acc_raw) / 1000.0 if h_acc_raw > 0 else None

        vertical_accuracy_m = float(v_acc_raw) / 1000.0 if v_acc_raw > 0 else None

        rover_state.update(
            "gps",
            fix_type=self._gps_fix_type,
            fix_name=fix_name,
            satellites_visible=int(message.satellites_visible),
            horizontal_accuracy_m=(horizontal_accuracy_m),
            vertical_accuracy_m=(vertical_accuracy_m),
            hdop=hdop,
            vdop=vdop,
            rtk_fixed=(self._gps_fix_type == 6),
        )

        rover_state.update(
            "rtk",
            status=fix_name,
        )

        yaw_centidegrees = int(message.yaw)

        if yaw_centidegrees > 0:
            rover_state.update(
                "vehicle",
                heading_deg=(float(yaw_centidegrees) / 100.0) % 360.0,
            )

    def _mavlink_estimator_status_callback(
        self,
        message: Mavlink,
    ) -> None:
        """Decode PX4 MAVLink ESTIMATOR_STATUS (#230) accuracy telemetry."""

        if int(message.msgid) != 230:
            return

        # The raw MAVROS transport can expose frames that failed CRC or
        # signature validation. Never turn those bytes into telemetry.
        if int(message.framing_status) != int(Mavlink.FRAMING_OK):
            return

        self._mark_ros_message()

        try:
            words = [int(value) for value in message.payload64]
            payload = struct.pack(
                "<" + ("Q" * len(words)),
                *words,
            )

            payload_length = int(message.len)

            if payload_length < 0 or payload_length > len(payload):
                raise ValueError(
                    "ESTIMATOR_STATUS payload length exceeds MAVROS payload64"
                )

            # MAVLink 2 may trim trailing zero bytes. Padding restores the
            # fixed 42-byte wire layout before unpacking the final flags field.
            payload = payload[:payload_length].ljust(
                42,
                b"\x00",
            )

            (
                _time_usec,
                vel_ratio,
                pos_horiz_ratio,
                pos_vert_ratio,
                mag_ratio,
                hagl_ratio,
                tas_ratio,
                pos_horiz_accuracy,
                pos_vert_accuracy,
                flags,
            ) = struct.unpack(
                "<Q8fH",
                payload[:42],
            )
        except (
            struct.error,
            TypeError,
            ValueError,
            OverflowError,
        ):
            LOGGER.exception("Failed to decode MAVLink ESTIMATOR_STATUS")
            return

        horizontal_accuracy_m = _finite_float(pos_horiz_accuracy)
        vertical_accuracy_m = _finite_float(pos_vert_accuracy)

        if horizontal_accuracy_m is not None and horizontal_accuracy_m <= 0.0:
            horizontal_accuracy_m = None

        if vertical_accuracy_m is not None and vertical_accuracy_m <= 0.0:
            vertical_accuracy_m = None

        horizontal_accuracy_mm = (
            horizontal_accuracy_m * 1000.0
            if horizontal_accuracy_m is not None
            else None
        )
        vertical_accuracy_mm = (
            vertical_accuracy_m * 1000.0 if vertical_accuracy_m is not None else None
        )
        estimator_flags = int(flags)
        absolute_horizontal_valid = bool(estimator_flags & 16)
        absolute_vertical_valid = bool(estimator_flags & 32)
        gps_glitch = bool(estimator_flags & 1024)
        accel_error = bool(estimator_flags & 2048)
        available = (
            horizontal_accuracy_m is not None and vertical_accuracy_m is not None
        )
        healthy = (
            absolute_horizontal_valid
            and absolute_vertical_valid
            and not gps_glitch
            and not accel_error
        )

        # Keep the flat legacy API and nested GPS compatibility fields live.
        # Raw receiver h_acc/v_acc remain separate GPS_RAW_INT measurements.
        rover_state.update(
            "gps",
            px4_hrms_source="MAVLINK_ESTIMATOR_STATUS_230",
            px4_hrms_m=horizontal_accuracy_m,
            px4_hrms_mm=horizontal_accuracy_mm,
            px4_vrms_m=vertical_accuracy_m,
            px4_vrms_mm=vertical_accuracy_mm,
            px4_estimator_available=available,
            px4_estimator_flags=estimator_flags,
            px4_estimator_healthy=healthy,
        )

        rover_state.update(
            "estimator",
            available=available,
            source="MAVLINK_ESTIMATOR_STATUS_230",
            horizontal_accuracy_m=horizontal_accuracy_m,
            horizontal_accuracy_mm=horizontal_accuracy_mm,
            vertical_accuracy_m=vertical_accuracy_m,
            vertical_accuracy_mm=vertical_accuracy_mm,
            vel_ratio=_finite_float(vel_ratio),
            pos_horiz_ratio=_finite_float(pos_horiz_ratio),
            pos_vert_ratio=_finite_float(pos_vert_ratio),
            mag_ratio=_finite_float(mag_ratio),
            hagl_ratio=_finite_float(hagl_ratio),
            tas_ratio=_finite_float(tas_ratio),
            flags=estimator_flags,
            absolute_horizontal_valid=absolute_horizontal_valid,
            absolute_vertical_valid=absolute_vertical_valid,
            gps_glitch=gps_glitch,
            accel_error=accel_error,
            healthy=healthy,
        )

    def _heading_callback(
        self,
        message: Float64,
    ) -> None:
        self._mark_ros_message()

        heading = _finite_float(message.data)

        if heading is None:
            return

        rover_state.update(
            "vehicle",
            heading_deg=heading % 360.0,
        )

    def _local_odom_callback(
        self,
        message: Odometry,
    ) -> None:
        self._mark_ros_message()
        self._last_position_message_monotonic = time.monotonic()

        position = message.pose.pose.position
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular

        x = _finite_float(position.x)
        y = _finite_float(position.y)
        z = _finite_float(position.z)

        velocity_x = (
            _finite_float(
                linear.x,
                0.0,
            )
            or 0.0
        )
        velocity_y = (
            _finite_float(
                linear.y,
                0.0,
            )
            or 0.0
        )
        velocity_z = (
            _finite_float(
                linear.z,
                0.0,
            )
            or 0.0
        )
        angular_z = (
            _finite_float(
                angular.z,
                0.0,
            )
            or 0.0
        )

        ground_speed = math.hypot(
            velocity_x,
            velocity_y,
        )

        rover_state.update(
            "position",
            local_x_m=x,
            local_y_m=y,
            local_z_m=z,
            velocity_x_mps=velocity_x,
            velocity_y_mps=velocity_y,
            velocity_z_mps=velocity_z,
        )

        rover_state.update(
            "vehicle",
            ground_speed_mps=ground_speed,
            linear_speed_mps=ground_speed,
            angular_speed_rps=angular_z,
        )

    def _rpp_debug_callback(
        self,
        message: String,
    ) -> None:
        """Mirror exact RPP control telemetry without reconstruction."""

        self._mark_ros_message()
        payload = _json_object(message.data)
        if payload is None:
            return

        self._last_rpp_debug_monotonic = time.monotonic()
        telemetry_sequence = _safe_int(payload.get("telemetry_sequence"), -1)
        control_sequence = _safe_int(payload.get("control_sequence"), -1)
        if telemetry_sequence >= 0:
            previous_sequence = self._last_rpp_debug_sequence
            if (
                previous_sequence is not None
                and telemetry_sequence > previous_sequence + 1
            ):
                self._rpp_debug_dropped_frames += (
                    telemetry_sequence - previous_sequence - 1
                )
            if previous_sequence is None or telemetry_sequence != previous_sequence:
                self._last_rpp_debug_sequence = telemetry_sequence

        rover_state.update(
            "accuracy",
            rpp_debug_available=bool(payload.get("available", False)),
            rpp_debug_source="/rpp/debug",
            rpp_debug_schema_version=_safe_int(payload.get("schema_version"), 1),
            rpp_debug_telemetry_sequence=(
                telemetry_sequence if telemetry_sequence >= 0 else None
            ),
            rpp_debug_control_sequence=(
                control_sequence if control_sequence >= 0 else None
            ),
            rpp_debug_control_sample_age_ms=_finite_float(
                payload.get("control_sample_age_ms")
            ),
            rpp_debug_odom_age_ms=_finite_float(payload.get("odom_age_ms")),
            rpp_debug_control_dt_ms=_finite_float(payload.get("control_dt_ms")),
            rpp_debug_control_compute_ms=_finite_float(
                payload.get("control_compute_ms")
            ),
            rpp_debug_control_deadline_missed=bool(
                payload.get("control_deadline_missed", False)
            ),
            rpp_debug_reason=str(payload.get("reason") or "UNKNOWN"),
            rpp_control_mode=str(payload.get("control_mode") or "UNKNOWN"),
            rpp_goal_number=_safe_int(payload.get("goal_number"), 0),

            rpp_actual_speed_mps=_finite_float(
                payload.get("actual_speed_mps")
            ),
            rpp_command_speed_mps=_finite_float(
                payload.get("command_speed_mps")
            ),

            rpp_current_yaw_deg=_finite_float(
                payload.get("current_yaw_deg")
            ),
            rpp_path_bearing_deg=_finite_float(
                payload.get("path_bearing_deg")
            ),
            rpp_guidance_bearing_deg=_finite_float(
                payload.get("guidance_bearing_deg")
            ),
            rpp_heading_error_deg=_finite_float(
                payload.get("heading_error_deg")
            ),

            rpp_distance_to_goal_m=_finite_float(
                payload.get("distance_to_goal_m")
            ),

            rpp_cross_track_error_mm=_finite_float(
                payload.get("cross_track_error_mm")
            ),
            rpp_cross_track_side=str(
                payload.get("cross_track_side") or "UNKNOWN"
            ),

            rpp_along_remaining_mm=_finite_float(
                payload.get("along_remaining_mm")
            ),
            rpp_along_position=str(
                payload.get("along_position") or "UNKNOWN"
            ),
        )

    def _accuracy_callback(
        self,
        message: String,
    ) -> None:
        """Mirror RPP accuracy telemetry without affecting mission logic."""

        self._mark_ros_message()
        payload = _json_object(message.data)
        if payload is None:
            return

        cross_track_mm = _finite_float(payload.get("cross_track_error_mm"))
        front_back_mm = _finite_float(payload.get("front_back_error_mm"))
        radial_mm = _finite_float(payload.get("radial_error_mm"))
        closest_radial_mm = _finite_float(payload.get("closest_radial_error_mm"))
        target_mm = (
            _finite_float(
                payload.get("accuracy_target_mm"),
                30.0,
            )
            or 30.0
        )
        test_tolerance_mm = (
            _finite_float(
                payload.get("test_tolerance_mm"),
                50.0,
            )
            or 50.0
        )

        if radial_mm is None:
            accuracy_status = "UNAVAILABLE"
            accuracy_pass = False
            within_test_tolerance = False
        elif radial_mm <= target_mm:
            accuracy_status = "ACCURACY_PASS"
            accuracy_pass = True
            within_test_tolerance = True
        elif radial_mm <= test_tolerance_mm:
            accuracy_status = "TEST_PROCEED_BAND"
            accuracy_pass = False
            within_test_tolerance = True
        else:
            accuracy_status = "OUTSIDE_TOLERANCE"
            accuracy_pass = False
            within_test_tolerance = False

        rover_state.update(
            "accuracy",
            available=(
                cross_track_mm is not None
                and front_back_mm is not None
                and radial_mm is not None
            ),
            source="/rpp/accuracy",
            goal_number=_safe_int(
                payload.get("goal_number"),
                0,
            ),
            cross_track_error_m=_finite_float(payload.get("cross_track_error_m")),
            cross_track_error_mm=cross_track_mm,
            cross_track_abs_mm=(
                abs(cross_track_mm) if cross_track_mm is not None else None
            ),
            cross_track_side=str(payload.get("cross_track_side") or "UNKNOWN"),
            front_back_error_m=_finite_float(payload.get("front_back_error_m")),
            front_back_error_mm=front_back_mm,
            front_back_abs_mm=(
                abs(front_back_mm) if front_back_mm is not None else None
            ),
            front_back_position=str(payload.get("front_back_position") or "UNKNOWN"),
            radial_error_m=_finite_float(payload.get("radial_error_m")),
            radial_error_mm=radial_mm,
            closest_radial_error_m=_finite_float(payload.get("closest_radial_error_m")),
            closest_radial_error_mm=closest_radial_mm,
            accuracy_target_m=target_mm / 1000.0,
            accuracy_target_mm=target_mm,
            test_tolerance_m=test_tolerance_mm / 1000.0,
            test_tolerance_mm=test_tolerance_mm,
            accuracy_status=accuracy_status,
            accuracy_pass=accuracy_pass,
            within_test_tolerance=within_test_tolerance,
        )

    def _spray_status_callback(
        self,
        message: String,
    ) -> None:
        """Cache spray-controller status for REST configuration ACK/status."""

        self._mark_ros_message()

        payload = _json_object(message.data)
        if payload is None:
            return

        with self._runtime_lock:
            self._spray_status = dict(payload)
            self._spray_status_monotonic = time.monotonic()

    @classmethod
    def _spray_pwm_to_actuator_value(
        cls,
        pwm_us: float,
    ) -> float:
        pwm = _finite_float(pwm_us)

        if pwm is None:
            raise RuntimeError("Spray PWM must be a finite number")

        if pwm < cls.SPRAY_PWM_MIN_US or pwm > cls.SPRAY_PWM_MAX_US:
            raise RuntimeError(
                "Spray PWM must be between "
                f"{int(cls.SPRAY_PWM_MIN_US)} and "
                f"{int(cls.SPRAY_PWM_MAX_US)} microseconds"
            )

        span = cls.SPRAY_PWM_MAX_US - cls.SPRAY_PWM_MIN_US

        return 2.0 * ((pwm - cls.SPRAY_PWM_MIN_US) / span) - 1.0

    @classmethod
    def _spray_actuator_value_to_pwm(
        cls,
        value: Any,
    ) -> int | None:
        actuator_value = _finite_float(value)

        if actuator_value is None:
            return None

        if actuator_value < -1.0 or actuator_value > 1.0:
            return None

        span = cls.SPRAY_PWM_MAX_US - cls.SPRAY_PWM_MIN_US

        pwm = cls.SPRAY_PWM_MIN_US + ((actuator_value + 1.0) / 2.0) * span

        return int(round(pwm))

    def get_spray_config(
        self,
    ) -> dict[str, Any]:
        with self._runtime_lock:
            status = dict(self._spray_status)
            status_age_sec = self._monotonic_age(self._spray_status_monotonic)

        press_value = _finite_float(status.get("press_value"))
        release_value = _finite_float(status.get("release_value"))

        return {
            "available": bool(status),
            "status_age_sec": (
                None if not math.isfinite(status_age_sec) else round(status_age_sec, 3)
            ),
            "press_pwm_us": (self._spray_actuator_value_to_pwm(press_value)),
            "release_pwm_us": (self._spray_actuator_value_to_pwm(release_value)),
            "press_value": press_value,
            "release_value": release_value,
            "spray_duration_sec": _finite_float(status.get("spray_duration_sec")),
            "spray_elapsed_sec": _finite_float(status.get("spray_elapsed_sec")),
            "spray_remaining_sec": _finite_float(status.get("spray_remaining_sec")),
            "spraying": bool(status.get("spraying", False)),
            "marking_active": bool(status.get("marking_active", False)),
            "controller_state": status.get("controller_state"),
            "ready": bool(status.get("ready", False)),
            "fault_latched": bool(status.get("fault_latched", False)),
            "fault_reason": status.get("fault_reason"),
            "config_request_id": status.get("config_request_id"),
            "config_result": status.get("config_result"),
            "config_reason": status.get("config_reason"),
            "pwm_min_us": int(self.SPRAY_PWM_MIN_US),
            "pwm_max_us": int(self.SPRAY_PWM_MAX_US),
        }

    def set_spray_config(
        self,
        *,
        press_pwm_us: float,
        release_pwm_us: float,
    ) -> dict[str, Any]:
        """Publish spray PWM configuration and wait for controller ACK."""

        press_value = self._spray_pwm_to_actuator_value(press_pwm_us)
        release_value = self._spray_pwm_to_actuator_value(release_pwm_us)

        request_id = uuid.uuid4().hex

        payload = {
            "request_id": request_id,
            "press_value": press_value,
            "release_value": release_value,
        }

        message = String()
        message.data = json.dumps(
            payload,
            separators=(",", ":"),
            sort_keys=True,
        )

        self._spray_config_pub.publish(message)

        deadline = time.monotonic() + self.SPRAY_CONFIG_ACK_TIMEOUT_SEC

        while time.monotonic() < deadline:
            with self._runtime_lock:
                status = dict(self._spray_status)

            if str(status.get("config_request_id") or "") == request_id:
                result = str(status.get("config_result") or "").strip().upper()

                if result == "ACCEPTED":
                    return self.get_spray_config()

                reason = str(status.get("config_reason") or "UNKNOWN_REASON")

                raise RuntimeError("Spray configuration rejected: " f"{reason}")

            time.sleep(0.05)

        raise RuntimeError(
            "Timed out waiting for spray controller " "configuration acknowledgement"
        )

    def _battery_callback(
        self,
        message: BatteryState,
    ) -> None:
        self._mark_ros_message()

        percentage = _finite_float(message.percentage)

        if percentage is not None:
            if 0.0 <= percentage <= 1.0:
                percentage *= 100.0
            elif percentage < 0.0:
                percentage = None

        remaining_status = "UNKNOWN"
        if percentage is not None:
            if percentage <= 10.0:
                remaining_status = "CRITICAL"
            elif percentage <= 20.0:
                remaining_status = "LOW"
            else:
                remaining_status = "OK"

        rover_state.update(
            "battery",
            voltage_v=_finite_float(message.voltage),
            current_a=_finite_float(message.current),
            remaining_percent=percentage,
            temperature_c=_finite_float(message.temperature),
            status=remaining_status,
        )

    def _rtk_health_callback(
        self,
        message: Bool,
    ) -> None:
        self._mark_ros_message()
        self._last_rtk_message_monotonic = time.monotonic()

        self._rtk_healthy = bool(message.data)

        rover_state.update(
            "rtk",
            healthy=self._rtk_healthy,
        )

    def _rtk_correction_age_callback(
        self,
        message: Float32,
    ) -> None:
        self._mark_ros_message()
        self._last_rtk_message_monotonic = time.monotonic()

        self._rtk_correction_age_sec = _finite_float(
            message.data
        )

        if (
            self._rtk_correction_age_sec is None
            or self._rtk_correction_age_sec < 0.0
        ):
            self._rtk_correction_age_sec = None

        rover_state.update(
            "rtk",
            correction_age_sec=(
                self._rtk_correction_age_sec
            ),
        )

    def _rtk_stream_status_callback(
        self,
        message: String,
    ) -> None:
        """Mirror credential-free worker RTCM status into backend state."""

        self._mark_ros_message()
        self._last_rtk_message_monotonic = (
            time.monotonic()
        )

        payload = _json_object(
            message.data
        )

        if payload is None:
            return

        state_name = str(
            payload.get(
                "state",
                "UNKNOWN",
            )
        ).strip().upper()

        allowed_states = {
            "DISCONNECTED",
            "WAITING_FOR_FIRST_PUBLISHED_FRAME",
            "HEALTHY",
            "UNHEALTHY",
        }

        if state_name not in allowed_states:
            state_name = "UNKNOWN"

        connected = bool(
            payload.get(
                "connected",
                False,
            )
        )

        healthy = bool(
            connected
            and payload.get(
                "healthy",
                False,
            )
        )

        correction_age = _finite_float(
            payload.get(
                "correction_age_sec"
            )
        )

        if (
            correction_age is not None
            and correction_age < 0.0
        ):
            correction_age = None

        self._rtk_healthy = healthy
        self._rtk_correction_age_sec = (
            correction_age
        )

        gga = payload.get(
            "gga"
        )

        if not isinstance(
            gga,
            dict,
        ):
            gga = {}

        gga_state = str(
            gga.get(
                "state",
                "DISABLED",
            )
        ).strip().upper()

        allowed_gga_states = {
            "DISABLED",
            "WAITING_FOR_FIX",
            "NO_FIX",
            "STALE",
            "READY",
        }

        if gga_state not in allowed_gga_states:
            gga_state = "UNKNOWN"

        gga_source_age = _finite_float(
            gga.get(
                "source_age_sec"
            )
        )

        gga_sent_age = _finite_float(
            gga.get(
                "last_sent_age_sec"
            )
        )

        if (
            gga_source_age is not None
            and gga_source_age < 0.0
        ):
            gga_source_age = None

        if (
            gga_sent_age is not None
            and gga_sent_age < 0.0
        ):
            gga_sent_age = None

        rover_state.update(
            "rtk",
            stream_state=state_name,
            stream_connected=connected,
            healthy=healthy,
            correction_age_sec=(
                correction_age
            ),
            socket_bytes_received=max(
                0,
                _safe_int(
                    payload.get(
                        "socket_bytes_received"
                    ),
                    0,
                ),
            ),
            valid_frames=max(
                0,
                _safe_int(
                    payload.get(
                        "valid_frames"
                    ),
                    0,
                ),
            ),
            published_frames=max(
                0,
                _safe_int(
                    payload.get(
                        "published_frames"
                    ),
                    0,
                ),
            ),
            crc_failures=max(
                0,
                _safe_int(
                    payload.get(
                        "crc_failures"
                    ),
                    0,
                ),
            ),
            invalid_headers=max(
                0,
                _safe_int(
                    payload.get(
                        "invalid_headers"
                    ),
                    0,
                ),
            ),
            resync_bytes_discarded=max(
                0,
                _safe_int(
                    payload.get(
                        "resync_bytes_discarded"
                    ),
                    0,
                ),
            ),
            partial_frame_timeouts=max(
                0,
                _safe_int(
                    payload.get(
                        "partial_frame_timeouts"
                    ),
                    0,
                ),
            ),
            oversize_drops=max(
                0,
                _safe_int(
                    payload.get(
                        "oversize_drops"
                    ),
                    0,
                ),
            ),
            publish_errors=max(
                0,
                _safe_int(
                    payload.get(
                        "publish_errors"
                    ),
                    0,
                ),
            ),
            worker_mavros_subscribers=(
                _safe_int(
                    payload.get(
                        "mavros_subscribers"
                    ),
                    -1,
                )
            ),
            max_mavros_rtcm_frame_bytes=(
                _safe_int(
                    payload.get(
                        "max_mavros_rtcm_frame_bytes"
                    ),
                    0,
                )
                or None
            ),
            gga_enabled=bool(
                gga.get(
                    "enabled",
                    False,
                )
            ),
            gga_state=gga_state,
            gga_source_age_sec=(
                gga_source_age
            ),
            gga_last_sent_age_sec=(
                gga_sent_age
            ),
            gga_sent_total=max(
                0,
                _safe_int(
                    gga.get(
                        "sent_total"
                    ),
                    0,
                ),
            ),
            gga_send_errors=max(
                0,
                _safe_int(
                    gga.get(
                        "send_errors"
                    ),
                    0,
                ),
            ),
        )

    def _heartbeat_health_callback(
        self,
        message: Bool,
    ) -> None:
        rover_state.update(
            "safety",
            backend_heartbeat_healthy=bool(message.data),
        )

    def _navigation_preview_point(
        self,
        *,
        index: int,
        east_m: float,
        north_m: float,
    ) -> dict[str, Any]:
        """Return local ENU plus authoritative GPS for API/map preview."""

        point: dict[str, Any] = {
            "index": int(index),
            "x": float(east_m),
            "y": float(north_m),
        }

        with self._runtime_lock:
            origin_latitude = self._px4_origin_latitude_deg
            origin_longitude = self._px4_origin_longitude_deg

        if origin_latitude is None or origin_longitude is None:
            return point

        try:
            latitude, longitude = _px4_enu_to_geodetic(
                origin_latitude_deg=origin_latitude,
                origin_longitude_deg=origin_longitude,
                east_m=float(east_m),
                north_m=float(north_m),
            )
        except ValueError as error:
            LOGGER.warning(
                "Unable to project nav preview point %s: %s",
                index,
                error,
            )
            return point

        point.update(
            {
                "latitude": latitude,
                "longitude": longitude,
                "projection": "PX4_MAP_PROJECTION_REPROJECT",
            }
        )

        return point

    # ==========================================================
    # Trajectory and mission callbacks
    # ==========================================================

    def _trajectory_ready_callback(
        self,
        message: Bool,
    ) -> None:
        self._mark_ros_message()

        with self._runtime_lock:
            self._trajectory_ready = bool(message.data)

        rover_state.update(
            "mission",
            trajectory_ready=self._trajectory_ready,
        )

    def _trajectory_status_callback(
        self,
        message: String,
    ) -> None:
        self._mark_ros_message()

        payload = _json_object(message.data)
        if payload is None:
            return

        state_name = str(payload.get("state", "")).strip().upper()

        ready = bool(payload.get("ready", False))

        with self._runtime_lock:
            self._trajectory_ready = ready

            if state_name == "ERROR":
                self._trajectory_error = str(
                    payload.get("error")
                    or payload.get("message")
                    or "Trajectory preparation failed"
                )
            elif state_name == "READY":
                self._trajectory_error = None

        mission_updates: dict[str, Any] = {
            # trajectory_ready is the fixed-path/display authority.
            # mission["ready"] remains the stricter mission-manager START gate.
            "trajectory_ready": ready,
            "ready": ready,
            "navigation_point_count": max(
                0,
                _safe_int(
                    payload.get("navigation_point_count"),
                    0,
                ),
            ),
            "message": payload.get("message"),
            "error": (self._trajectory_error if state_name == "ERROR" else None),
        }

        if payload.get("extension_mode") is not None:
            mission_updates["extension_mode"] = payload.get("extension_mode")

        if payload.get("dummy_point_distance_m") is not None:
            mission_updates["dummy_point_distance_m"] = _finite_float(
                payload.get("dummy_point_distance_m")
            )

        if payload.get("row_transition_threshold_m") is not None:
            mission_updates["row_transition_threshold_m"] = _finite_float(
                payload.get("row_transition_threshold_m")
            )

        mission_updates["dummy_point_count"] = max(
            0,
            _safe_int(
                payload.get("dummy_point_count"),
                0,
            ),
        )

        if state_name == "PREPARING":
            mission_updates["state"] = "PREPARING"
            mission_updates["ready"] = False
        elif state_name == "READY":
            # trajectory_generator READY means the geometry exists, but the
            # mission is not startable until mission_manager has accepted all
            # four retained products (/nav_path, /mission_waypoints, path_types,
            # marking_indices). Keep the public lifecycle PREPARING here.
            mission_updates["state"] = "PREPARING"
            mission_updates["ready"] = False
            mission_updates["message"] = (
                "Trajectory generated; waiting for mission manager READY"
            )
        elif state_name == "ERROR":
            with self._runtime_lock:
                self._preparation_in_progress = False
            mission_updates["state"] = "ERROR"
            mission_updates["ready"] = False

        rover_state.update(
            "mission",
            **mission_updates,
        )

    def _nav_path_callback(
        self,
        message: NavPath,
    ) -> None:
        self._mark_ros_message()

        # The full path stays in ROS. API state stores only a bounded preview.
        preview_limit = 2000
        preview: list[dict[str, Any]] = []

        for index, pose in enumerate(message.poses[:preview_limit]):
            east_m = float(pose.pose.position.x)
            north_m = float(pose.pose.position.y)

            if not math.isfinite(east_m) or not math.isfinite(north_m):
                continue

            preview.append(
                self._navigation_preview_point(
                    index=index,
                    east_m=east_m,
                    north_m=north_m,
                )
            )

        rover_state.update(
            "mission",
            navigation_point_count=len(message.poses),
            navigation_path_preview=preview,
            navigation_path_preview_truncated=(len(message.poses) > preview_limit),
            path_frame_id=str(message.header.frame_id),
        )

    def _mission_waypoints_callback(
        self,
        message: NavPath,
    ) -> None:
        self._mark_ros_message()

        rover_state.update(
            "mission",
            total_points=len(message.poses),
        )

    def _mission_status_callback(
        self,
        message: String,
    ) -> None:
        self._mark_ros_message()

        payload = _json_object(message.data)
        if payload is None:
            return

        state_name = str(payload.get("state", "EMPTY")).strip().upper()

        execution_mode = (
            str(
                payload.get(
                    "execution_mode",
                    self._mission_execution_mode,
                )
            )
            .strip()
            .upper()
        )

        if execution_mode not in {"AUTO", "MANUAL"}:
            execution_mode = self._mission_execution_mode

        manager_ready = bool(payload.get("path_ready", False)) and bool(
            payload.get(
                "trajectory_ready",
                False,
            )
        )

        with self._runtime_lock:
            self._mission_manager_state = state_name
            self._mission_execution_mode = execution_mode

            self._mission_manager_ready = state_name == "READY" and manager_ready

            self._mission_manager_error = (
                str(
                    payload.get("error")
                    or payload.get("message")
                    or "Mission manager error"
                )
                if state_name == "ERROR"
                else None
            )

            preparation_in_progress = self._preparation_in_progress

            if self._mission_manager_ready:
                self._preparation_in_progress = False
                preparation_in_progress = False
            elif state_name == "ERROR":
                self._preparation_in_progress = False
                preparation_in_progress = False

        # trajectory_generator intentionally publishes empty retained products
        # at the start of a replacement generation. mission_manager therefore
        # briefly reports EMPTY. That EMPTY is an internal snapshot boundary,
        # not a user-visible unload. Keep PREPARING until the new complete
        # snapshot becomes READY.
        effective_state_name = state_name
        if preparation_in_progress and state_name == "EMPTY":
            effective_state_name = "PREPARING"
        elif preparation_in_progress and state_name == "READY" and not manager_ready:
            effective_state_name = "PREPARING"

        completed = max(
            0,
            _safe_int(
                payload.get("completed_points"),
                0,
            ),
        )
        skipped = max(
            0,
            _safe_int(
                payload.get("skipped_points"),
                0,
            ),
        )
        failed = max(
            0,
            _safe_int(
                payload.get("failed_points"),
                0,
            ),
        )
        total = max(
            0,
            _safe_int(
                payload.get("total_points"),
                0,
            ),
        )

        point_index_raw = payload.get("current_point_index")
        point_index = (
            _safe_int(point_index_raw) if point_index_raw is not None else None
        )

        # mission_manager/status can briefly publish navigation_point_count=0
        # while a new multipart trajectory snapshot is being accepted.
        #
        # /nav_path is the authoritative source for the actual generated
        # navigation-point count. Never overwrite an already-valid count
        # with an intermediate zero from mission_manager.
        manager_navigation_point_count = max(
            0,
            _safe_int(
                payload.get("navigation_point_count"),
                0,
            ),
        )

        mission_updates: dict[str, Any] = {
            "state": effective_state_name,
            "execution_mode": execution_mode,
            "ready": bool(effective_state_name == "READY" and manager_ready),
            "manager_ready": bool(self._mission_manager_ready),
            "total_points": total,
            "active_point_id": payload.get("current_point_id"),
            "active_point_index": point_index,
            "active_point_number": (
                point_index + 1 if point_index is not None else None
            ),
            "active_point_state": payload.get("current_point_state"),
            "completed_points": completed,
            "skipped_points": skipped,
            "failed_points": failed,
            "remaining_points": max(
                0,
                _safe_int(
                    payload.get("remaining_points"),
                    0,
                ),
            ),
            "progress_percent": (
                _finite_float(
                    payload.get("progress_percent"),
                    0.0,
                )
                or 0.0
            ),
            "marking_active": bool(payload.get("marking_active", False)),
            "pause_reason": payload.get("pause_reason"),
            "resume_available": bool(payload.get("resume_available", False)),
            # Mission Manager is the single authority for movement safety.
            # Mirror its RTK/localisation decision verbatim for backend/UI/logs.
            "gps_fix_type": _safe_int(payload.get("gps_fix_type"), 0),
            "rtk_state": payload.get("rtk_state"),
            "rtk_fixed": bool(payload.get("rtk_fixed", False)),
            "rtk_healthy": bool(payload.get("rtk_healthy", False)),
            "rtk_motion_ok": bool(payload.get("rtk_motion_ok", False)),
            "rtk_reason": payload.get("rtk_reason"),
            "rtk_correction_age_sec": _finite_float(
                payload.get("rtk_correction_age_sec")
            ),
            "gps_fix_status_age_sec": _finite_float(
                payload.get("gps_fix_status_age_sec")
            ),
            "rtk_health_status_age_sec": _finite_float(
                payload.get("rtk_health_status_age_sec")
            ),
            "rtk_age_status_age_sec": _finite_float(
                payload.get("rtk_age_status_age_sec")
            ),
            "backend_heartbeat_healthy": bool(
                payload.get("backend_heartbeat_healthy", False)
            ),
            "mission_enable": bool(payload.get("mission_enable", False)),
            "emergency_stop": bool(payload.get("emergency_stop", True)),
            "safety_generation": _safe_int(
                payload.get("safety_generation"),
                -1,
            ),
            "px4_connected": bool(payload.get("px4_connected", False)),
            "px4_mode": payload.get("px4_mode"),
            "px4_armed": bool(payload.get("px4_armed", False)),
            "spray_controller_ready": bool(
                payload.get("spray_controller_ready", False)
            ),
            "spray_controller_state": payload.get("spray_controller_state"),
            "spray_fault_reason": payload.get("spray_fault_reason"),
            "spray_enabled": bool(
                payload.get(
                    "spray_enabled",
                    payload.get("spray_required", False),
                )
            ),
            "spray_gates_mission_progress": bool(
                payload.get("spray_gates_mission_progress", False)
            ),
            "current_point_spray_confirmed": payload.get(
                "current_point_spray_confirmed"
            ),
            "start_stage": str(payload.get("start_stage") or "IDLE").strip().upper(),
            "start_failed_stage": payload.get("start_failed_stage"),
            "arrival_settle_elapsed_sec": max(
                0.0,
                _finite_float(
                    payload.get("arrival_settle_elapsed_sec"),
                    0.0,
                )
                or 0.0,
            ),
            "arrival_settle_required_sec": max(
                0.0,
                _finite_float(
                    payload.get("arrival_settle_required_sec"),
                    settings.arrival_settle_seconds,
                )
                or settings.arrival_settle_seconds,
            ),
            "hold_elapsed_sec": max(
                0.0,
                _finite_float(
                    payload.get(
                        "marking_hold_elapsed_sec",
                        payload.get(
                            "verification_hold_elapsed_sec",
                            payload.get("hold_elapsed_sec"),
                        ),
                    ),
                    0.0,
                )
                or 0.0,
            ),
            "hold_required_sec": max(
                0.0,
                _finite_float(
                    payload.get(
                        "marking_hold_required_sec",
                        payload.get(
                            "verification_hold_required_sec",
                            payload.get("hold_required_sec"),
                        ),
                    ),
                    settings.marking_hold_seconds,
                )
                or settings.marking_hold_seconds,
            ),
            "alignment_active": bool(
                payload.get(
                    "alignment_active",
                    False,
                )
            ),
            "point_status": payload.get(
                "point_status",
                [],
            ),
            # Survey-truth RECORDING HEALTH. Scalars only -- these say whether
            # the mission will be able to carry a physical measurement, so the
            # operator learns before starting that targets did not load or GNSS
            # is not streaming, instead of finding an empty report afterwards.
            # The per-point survey measurements themselves travel on the report
            # (accuracy.survey), not on this high-rate channel.
            "survey_truth_enabled": bool(
                payload.get("survey_truth_enabled", False)
            ),
            "survey_truth_ready": bool(payload.get("survey_truth_ready", False)),
            "survey_truth_targets_loaded": int(
                _finite_float(
                    payload.get("survey_truth_targets_loaded"),
                    0.0,
                )
                or 0.0
            ),
            "survey_truth_gnss_samples": int(
                _finite_float(
                    payload.get("survey_truth_gnss_samples"),
                    0.0,
                )
                or 0.0
            ),
            "survey_truth_coordinate_mode": payload.get(
                "survey_truth_coordinate_mode"
            ),
            "message": payload.get("message"),
            "error": payload.get("error"),
        }

        manager_run_id = str(payload.get("mission_run_id") or "").strip()
        if manager_run_id:
            mission_updates["mission_run_id"] = manager_run_id

        if preparation_in_progress and state_name == "EMPTY":
            # Preserve the uploaded mission metadata and the trajectory
            # generator's PREPARING message while mission_manager is clearing
            # its pending multipart snapshot.
            mission_updates.pop("total_points", None)
            mission_updates.pop("active_point_id", None)
            mission_updates.pop("active_point_index", None)
            mission_updates.pop("active_point_number", None)
            mission_updates.pop("active_point_state", None)
            mission_updates.pop("completed_points", None)
            mission_updates.pop("skipped_points", None)
            mission_updates.pop("failed_points", None)
            mission_updates.pop("remaining_points", None)
            mission_updates.pop("progress_percent", None)
            mission_updates.pop("point_status", None)
            mission_updates.pop("message", None)
            mission_updates.pop("error", None)

        if effective_state_name == "READY" and manager_ready:
            current_mission = rover_state.section("mission")
            if current_mission.get("prepared_at") is None:
                mission_updates["prepared_at"] = utc_now_iso()
            mission_updates["message"] = (
                payload.get("message") or "Prepared mission loaded and ready"
            )
            mission_updates["error"] = None

        # Only let mission_manager update the count when it has a real,
        # non-zero committed path. A zero must not erase the count that was
        # already received directly from /nav_path.
        if manager_navigation_point_count > 0:
            mission_updates["navigation_point_count"] = manager_navigation_point_count

        if effective_state_name == "RUNNING":
            current = rover_state.section("mission")
            if current.get("started_at") is None:
                mission_updates["started_at"] = utc_now_iso()

        elif effective_state_name == "PAUSED":
            mission_updates["paused_at"] = utc_now_iso()

        elif effective_state_name == "WAITING_FOR_NEXT":
            mission_updates["paused_at"] = None

        elif effective_state_name == "COMPLETED":
            mission_updates["completed_at"] = utc_now_iso()

        rover_state.update(
            "mission",
            **mission_updates,
        )

        # mission_manager is the sole owner of the motion safety gate.
        # Never perform durable file I/O from this ROS subscription callback.
        self._schedule_mission_runtime_persist(payload)
        _notify_authoritative_state_changed()



#
#
# Final report accuracy comes ONLY from Mission Manager's nested accuracy
# object, which itself is now a copy of RPP terminal_result.
#
# NO live /rpp/accuracy fallback.
# NO capture_* fallback.
# NO abs() or radial reconstruction for final point evidence.


    def _point_event_callback(
        self,
        message: String,
    ) -> None:
        """Store RPP-terminal point evidence without recalculating accuracy."""

        self._mark_ros_message()

        payload = _json_object(message.data)
        if payload is None:
            return

        payload["received_at"] = utc_now_iso()

        event_name = str(payload.get("event") or "").strip().upper()

        point_index_raw = payload.get("point_index")

        point_index = _safe_int(point_index_raw, -1) if point_index_raw is not None else -1

        point_number = point_index + 1 if point_index >= 0 else 0

        # ----------------------------------------------------------
        # RPP TERMINAL ACCURACY ONLY
        # ----------------------------------------------------------
        manager_accuracy = payload.get("accuracy")

        accuracy_snapshot: dict[str, Any] | None = None

        if isinstance(manager_accuracy, dict):
            candidate = copy.deepcopy(manager_accuracy)

            source = str(candidate.get("measurement_source") or "").strip().upper()

            if source == "RPP_TERMINAL_RESULT":
                # Copy only. No position calculations.
                accuracy_snapshot = candidate

                # Backend receive time is metadata only.
                accuracy_snapshot.setdefault(
                    "captured_at",
                    payload["received_at"],
                )

        if accuracy_snapshot is not None:
            payload["accuracy"] = copy.deepcopy(accuracy_snapshot)

            payload["overall_accuracy_mm"] = accuracy_snapshot.get("overall_accuracy_mm")

            overall_mm = _finite_float(accuracy_snapshot.get("overall_accuracy_mm"))

            payload["accuracy_remarks"] = (
                f"{overall_mm:.1f} mm" if overall_mm is not None else "Unavailable"
            )

        else:
            payload["accuracy"] = None
            payload["overall_accuracy_mm"] = None
            payload["accuracy_remarks"] = "Unavailable"

        # ----------------------------------------------------------
        # Existing spray evidence handling.
        # ----------------------------------------------------------
        manager_spray = payload.get("spray")

        if isinstance(manager_spray, dict):
            spray = copy.deepcopy(manager_spray)

            spray_attempted = bool(spray.get("attempted", False))

            spray_outcome = str(spray.get("outcome") or "UNKNOWN").strip().upper()

            spray_reason = spray.get("reason")

            spray_elapsed_sec = _finite_float(spray.get("elapsed_sec"))

        else:
            spray_attempted = bool(
                payload.get(
                    "spray_attempted",
                    False,
                )
            )

            spray_outcome = str(payload.get("spray_outcome") or "UNKNOWN").strip().upper()

            spray_reason = payload.get("spray_failure_reason")

            spray_elapsed_sec = _finite_float(payload.get("spray_elapsed_sec"))

            spray = {
                "attempted": spray_attempted,
                "outcome": spray_outcome,
                "reason": spray_reason,
                "elapsed_sec": spray_elapsed_sec,
            }

        spray_confirmed = (
            True
            if spray_outcome == "SUCCESS"
            else (
                False
                if spray_outcome
                in {
                    "FAILED",
                    "TIMEOUT",
                }
                else None
            )
        )

        mission = rover_state.section("mission")

        point_results = dict(mission.get("point_results") or {})

        point_id = str(payload.get("point_id") or "").strip()

        if point_id:
            existing = point_results.get(point_id)

            existing = existing if isinstance(existing, dict) else {}

            event_history = list(existing.get("event_history") or [])

            event_history.append(
                {
                    "event": event_name,
                    "state": payload.get("state"),
                    "reason": payload.get("reason"),
                    "timestamp_unix_ns": payload.get("timestamp_unix_ns"),
                    "received_at": payload["received_at"],
                    "accuracy": copy.deepcopy(accuracy_snapshot),
                    "spray": copy.deepcopy(spray),
                }
            )

            manager_result = payload.get("point_result")

            result = (
                copy.deepcopy(manager_result)
                if isinstance(
                    manager_result,
                    dict,
                )
                else {}
            )

            result.update(
                {
                    "point_id": point_id,
                    "point_index": point_index,
                    "event": event_name,
                    "mission_run_id": payload.get("mission_run_id"),
                    "point_outcome": str(result.get("point_outcome") or event_name)
                    .strip()
                    .upper(),
                    "spray": copy.deepcopy(spray),
                    "spray_attempted": spray_attempted,
                    "spray_outcome": spray_outcome,
                    "spray_confirmed": spray_confirmed,
                    "spray_failure_reason": spray_reason,
                    "spray_elapsed_sec": spray_elapsed_sec,
                    "spray_monitor_only": True,
                    "overall_accuracy_mm": (
                        accuracy_snapshot.get("overall_accuracy_mm")
                        if accuracy_snapshot is not None
                        else None
                    ),
                    "accuracy_remarks": payload.get(
                        "accuracy_remarks",
                        "Unavailable",
                    ),
                    # Exact RPP terminal evidence only.
                    "accuracy": copy.deepcopy(accuracy_snapshot),
                    "event_history": event_history,
                    "received_at": payload["received_at"],
                }
            )

            rpp_outcome = (
                str((accuracy_snapshot or {}).get("rpp_outcome") or "").strip().upper()
            )

            if rpp_outcome == "MISSED" or event_name == "ACCURACY_FAILED":
                result["accuracy_failure"] = copy.deepcopy(accuracy_snapshot)

            elif existing.get("accuracy_failure") is not None:
                result["accuracy_failure"] = copy.deepcopy(existing["accuracy_failure"])

            point_results[point_id] = result

        rover_state.update(
            "mission",
            last_point_event=payload,
            point_results=point_results,
        )
        _notify_authoritative_state_changed()

        if event_name == "MISSION_TERMINATED":
            self._schedule_terminal_mission_cleanup(payload)
        else:
            self._schedule_live_report_checkpoint()

    def _schedule_terminal_mission_cleanup(
        self,
        terminal_event: dict[str, Any],
    ) -> None:
        """Run terminal service/file work outside the ROS subscription callback."""

        cleanup_key = str(
            terminal_event.get("mission_run_id")
            or terminal_event.get("timestamp_unix_ns")
            or "unknown-terminal-event"
        )

        with self._terminal_cleanup_lock:
            if cleanup_key in self._terminal_cleanup_keys:
                return
            self._terminal_cleanup_keys.add(cleanup_key)

        cleanup_thread = threading.Thread(
            target=self._finalize_terminal_mission,
            args=(copy.deepcopy(terminal_event),),
            name=f"mission-terminal-cleanup-{cleanup_key[:12]}",
            daemon=True,
        )
        cleanup_thread.start()

    def _schedule_live_report_checkpoint(self) -> None:
        """Queue a coalesced live-report checkpoint outside ROS callbacks."""

        mission_id = str(rover_state.section("mission").get("mission_id") or "")
        if not mission_id:
            return

        with self._report_checkpoint_condition:
            if self._report_checkpoint_stopping:
                return
            self._report_checkpoint_pending_mission_id = mission_id
            self._report_checkpoint_condition.notify()

    def _report_checkpoint_worker(self) -> None:
        """Persist accumulated point results without blocking the executor."""

        while True:
            with self._report_checkpoint_condition:
                while (
                    self._report_checkpoint_pending_mission_id is None
                    and not self._report_checkpoint_stopping
                ):
                    self._report_checkpoint_condition.wait()

                if (
                    self._report_checkpoint_pending_mission_id is None
                    and self._report_checkpoint_stopping
                ):
                    return

                mission_id = self._report_checkpoint_pending_mission_id
                self._report_checkpoint_pending_mission_id = None

            # Serialize with upload replacement and terminal report cleanup.
            # Re-check identity and active artifacts after acquiring the
            # lifecycle lock so a delayed worker cannot recreate stale files.
            with mission_report_store.lifecycle_transaction():
                active_mission_id = str(
                    rover_state.section("mission").get("mission_id") or ""
                )
                if (
                    not mission_id
                    or active_mission_id != mission_id
                    or not settings.mission_file.is_file()
                    or not settings.mission_metadata_file.is_file()
                ):
                    continue

                try:
                    mission_report_store.checkpoint_live_report()
                except MissionReportError as error:
                    reason = f"Mission report checkpoint failed: {error}"
                    rover_state.update(
                        "report",
                        status="CHECKPOINT_FAILED",
                        error=reason,
                    )
                    self.get_logger().error(reason)

    def _shutdown_report_checkpoint_worker(self) -> None:
        """Flush the newest pending point checkpoint and stop its worker."""

        with self._report_checkpoint_condition:
            self._report_checkpoint_stopping = True
            self._report_checkpoint_condition.notify_all()

        self._report_checkpoint_thread.join(timeout=2.0)
        if self._report_checkpoint_thread.is_alive():
            LOGGER.warning(
                "Mission report checkpoint worker did not stop within 2 seconds"
            )

    def _finalize_terminal_mission(
        self,
        terminal_event: dict[str, Any],
    ) -> None:
        """Archive a terminal mission, then clear ROS and active files."""

        if not bool(terminal_event.get("disarm_confirmed", False)):
            reason = str(
                terminal_event.get("message")
                or (
                    "Mission Manager did not confirm PX4 disarm; active "
                    "mission artifacts were retained."
                )
            )
            mission_report_store.expose_failure(reason)
            self.get_logger().error(reason)
            return

        with mission_report_store.lifecycle_transaction():
            try:
                report = mission_report_store.write_terminal_report(terminal_event)
            except StaleMissionTerminalEvent as error:
                self.get_logger().warn(str(error))
                return
            except MissionReportError as error:
                reason = str(error)
                mission_report_store.expose_failure(reason)
                self.get_logger().error(reason)
                return

            trajectory_cleared = False
            accepted, clear_message = self._call_trigger(
                client=self._trajectory_clear_client,
                service_name="/trajectory_generator/clear",
            )

            if not accepted:
                reason = (
                    "Terminal report was preserved, but the trajectory "
                    f"could not be cleared: {clear_message}. "
                    "Active mission artifacts were retained."
                )
                try:
                    mission_report_store.update_cleanup(
                        report,
                        status="TRAJECTORY_CLEAR_FAILED",
                        complete=False,
                        trajectory_cleared=False,
                        active_artifacts_deleted=False,
                        error=reason,
                    )
                except MissionReportError:
                    self.get_logger().exception(
                        "Unable to record trajectory-clear failure in the mission report"
                    )
                mission_report_store.expose_failure(reason)
                self.get_logger().error(reason)
                return

            trajectory_cleared = True

            try:
                mission_store.delete()
            except RuntimeError as error:
                reason = (
                    "Terminal report was preserved and the trajectory was "
                    f"cleared, but active mission artifacts could not be deleted: {error}"
                )
                try:
                    mission_report_store.update_cleanup(
                        report,
                        status="ARTIFACT_DELETE_FAILED",
                        complete=False,
                        trajectory_cleared=trajectory_cleared,
                        active_artifacts_deleted=False,
                        error=reason,
                    )
                except MissionReportError:
                    self.get_logger().exception(
                        "Unable to record artifact-delete failure in the mission report"
                    )
                mission_report_store.expose_failure(reason)
                self.get_logger().error(reason)
                return

            try:
                mission_report_store.update_cleanup(
                    report,
                    status="READY",
                    complete=True,
                    trajectory_cleared=True,
                    active_artifacts_deleted=True,
                    error=None,
                )
            except MissionReportError:
                # The original report was already written before cleanup.  Do
                # not recreate deleted active artifacts merely because the
                # final cleanup marker could not be persisted.
                self.get_logger().exception(
                    "Mission artifacts were cleaned, but the report cleanup "
                    "marker could not be updated"
                )

    def _marking_active_callback(
        self,
        message: Bool,
    ) -> None:
        rover_state.update(
            "mission",
            marking_active=bool(message.data),
        )

    def _alignment_active_callback(
        self,
        message: Bool,
    ) -> None:
        rover_state.update(
            "mission",
            alignment_active=bool(message.data),
        )

    def _active_waypoint_callback(
        self,
        message: PoseStamped,
    ) -> None:
        x = _finite_float(message.pose.position.x)
        y = _finite_float(message.pose.position.y)

        if x is None or y is None:
            return

        rover_state.update(
            "mission",
            active_waypoint={
                "x": x,
                "y": y,
                "frame_id": str(message.header.frame_id),
            },
        )

    def _schedule_mission_runtime_persist(
        self,
        manager_payload: dict[str, Any],
    ) -> None:
        """Queue the newest runtime snapshot together with its mission identity."""

        mission_id = str(
            rover_state.section("mission").get("mission_id") or ""
        )
        snapshot = copy.deepcopy(manager_payload)

        with self._runtime_persist_condition:
            if self._runtime_persist_stopping:
                return

            # Mission identity is captured at enqueue time. A delayed Mission A
            # snapshot must never be persisted as Mission B after replacement.
            self._runtime_persist_pending = (mission_id, snapshot)
            self._runtime_persist_condition.notify()

    def _runtime_persist_worker(self) -> None:
        """Persist Mission Manager runtime snapshots outside ROS callbacks."""

        while True:
            with self._runtime_persist_condition:
                while (
                    self._runtime_persist_pending is None
                    and not self._runtime_persist_stopping
                ):
                    self._runtime_persist_condition.wait()

                if (
                    self._runtime_persist_pending is None
                    and self._runtime_persist_stopping
                ):
                    return

                pending = self._runtime_persist_pending
                self._runtime_persist_pending = None

            if pending is not None:
                mission_id, payload = pending
                self._persist_mission_runtime(mission_id, payload)

    def _shutdown_runtime_persist_worker(self) -> None:
        """Flush the newest pending runtime snapshot and stop the worker."""

        with self._runtime_persist_condition:
            self._runtime_persist_stopping = True
            self._runtime_persist_condition.notify_all()

        self._runtime_persist_thread.join(timeout=2.0)

        if self._runtime_persist_thread.is_alive():
            LOGGER.warning(
                "Mission runtime persistence worker did not stop within 2 seconds"
            )

    def _persist_mission_runtime(
        self,
        captured_mission_id: str,
        manager_payload: dict[str, Any],
    ) -> None:
        # Serialize runtime persistence against terminal cleanup and new-mission
        # lifecycle operations.
        with mission_report_store.lifecycle_transaction():
            current_mission_id = str(
                rover_state.section("mission").get("mission_id") or ""
            )

            # Never associate an old queued runtime snapshot with a mission that
            # replaced it while this worker was delayed.
            if captured_mission_id != current_mission_id:
                return

            # EMPTY status may remove stale runtime only when the captured
            # identity is also still EMPTY. It must never delete a newer
            # mission's runtime file.
            if (
                not captured_mission_id
                or not settings.mission_file.is_file()
                or not settings.mission_metadata_file.is_file()
            ):
                try:
                    settings.mission_runtime_file.unlink(missing_ok=True)
                except OSError:
                    LOGGER.exception(
                        "Unable to remove stale mission runtime state"
                    )
                return

            runtime_payload = {
                "schema_version": 1,
                "saved_at": utc_now_iso(),
                "mission_id": captured_mission_id,
                "runtime": manager_payload,
            }

            try:
                _atomic_write_json(
                    settings.mission_runtime_file,
                    runtime_payload,
                )
            except OSError:
                LOGGER.exception("Unable to persist mission runtime state")

    # ==========================================================
    # Safety mirroring and backend heartbeat
    # ==========================================================

    def _update_safety_mirror(self) -> None:
        heartbeat_healthy = bool(
            rover_state.section("safety").get(
                "backend_heartbeat_healthy",
                False,
            )
        )
        rover_state.set_safety_state(
            emergency_stop=bool(self._emergency_stop),
            mission_enable=bool(self._mission_enable),
            command_owner=("MISSION" if self._mission_enable else "NONE"),
            reason=(
                "MISSION_ENABLED"
                if self._mission_enable
                else ("EMERGENCY_STOP" if self._emergency_stop else "MISSION_DISABLED")
            ),
            heartbeat_healthy=heartbeat_healthy,
        )
        _notify_authoritative_state_changed()

    def _mission_enable_state_callback(self, message: Bool) -> None:
        self._mission_enable = bool(message.data)
        self._update_safety_mirror()

    def _emergency_stop_state_callback(self, message: Bool) -> None:
        self._emergency_stop = bool(message.data)
        self._update_safety_mirror()

    def _publish_heartbeat(self) -> None:
        self._heartbeat_sequence = (self._heartbeat_sequence + 1) % (2**64)
        message = UInt64()
        message.data = int(self._heartbeat_sequence)
        self._heartbeat_pub.publish(message)
        rover_state.update(
            "ros",
            last_heartbeat_at=utc_now_iso(),
        )

    # ==========================================================
    # Service-call helpers
    # ==========================================================

    def _call_service(
        self,
        *,
        client: Any,
        request: Any,
        service_name: str,
        success_attribute: str = "success",
        discovery_timeout_sec: float | None = None,
        response_timeout_sec: float | None = None,
        timeout_outcome_unknown: bool = False,
    ) -> tuple[bool, str]:
        discovery_timeout = (
            self.SERVICE_DISCOVERY_TIMEOUT_SEC
            if discovery_timeout_sec is None
            else float(discovery_timeout_sec)
        )
        response_timeout = (
            self.SERVICE_RESPONSE_TIMEOUT_SEC
            if response_timeout_sec is None
            else float(response_timeout_sec)
        )

        if not client.wait_for_service(timeout_sec=discovery_timeout):
            return (
                False,
                f"ROS service unavailable: {service_name}",
            )

        completion = threading.Event()
        response_holder: dict[str, Any] = {}

        try:
            future = client.call_async(request)
        except Exception as error:
            return (
                False,
                f"Unable to call {service_name}: {error}",
            )

        def _done_callback(
            completed_future: Any,
        ) -> None:
            try:
                response_holder["response"] = completed_future.result()
            except Exception as error:
                response_holder["error"] = str(error)
            finally:
                completion.set()

        future.add_done_callback(_done_callback)

        if not completion.wait(response_timeout):
            # Cancelling the local future cannot prove that the server did not
            # execute the request. It only prevents an obsolete client result
            # from being consumed if cancellation is supported.
            try:
                future.cancel()
            except Exception:
                pass
            if timeout_outcome_unknown:
                raise RosServiceOutcomeUnknownError(
                    f"Timed out waiting for {service_name} after dispatch; "
                    "execution outcome is unknown. Do not retry blindly; "
                    "verify authoritative mission/safety state first."
                )
            return (
                False,
                f"Timed out waiting for {service_name}",
            )

        if "error" in response_holder:
            return (
                False,
                f"{service_name} failed: " f"{response_holder['error']}",
            )

        response = response_holder.get("response")

        if response is None:
            return (
                False,
                f"{service_name} returned no response",
            )

        accepted = bool(
            getattr(
                response,
                success_attribute,
                False,
            )
        )

        message = str(
            getattr(
                response,
                "message",
                "OK" if accepted else "Rejected",
            )
        )

        return accepted, message

    def _call_trigger(
        self,
        *,
        client: Any,
        service_name: str,
        response_timeout_sec: float | None = None,
        timeout_outcome_unknown: bool = False,
    ) -> tuple[bool, str]:
        return self._call_service(
            client=client,
            request=Trigger.Request(),
            service_name=service_name,
            response_timeout_sec=(response_timeout_sec),
            timeout_outcome_unknown=timeout_outcome_unknown,
        )

    def _manager_command(
        self,
        command: str,
    ) -> tuple[bool, str]:
        if command == "release_emergency_stop":
            return (
                False,
                "release_emergency_stop requires a generation-aware request",
            )

        if command not in MISSION_MANAGER_COMMANDS:
            return (
                False,
                f"Unknown mission command: {command}",
            )

        return self._call_trigger(
            client=(self._mission_manager_clients[command]),
            service_name=(f"/mission_manager/{command}"),
            response_timeout_sec=self.MANAGER_RESPONSE_TIMEOUT_SEC[command],
            timeout_outcome_unknown=True,
        )

    def _publish_execution_mode(
        self,
        execution_mode: str,
    ) -> None:
        message = String()
        message.data = str(execution_mode).strip().upper()
        self._execution_mode_pub.publish(message)

    def _wait_for_execution_mode(
        self,
        execution_mode: str,
    ) -> None:
        expected = str(execution_mode).strip().upper()
        deadline = time.monotonic() + self.EXECUTION_MODE_ACK_TIMEOUT_SEC

        while time.monotonic() < deadline:
            with self._runtime_lock:
                observed = self._mission_execution_mode

            if observed == expected:
                return

            time.sleep(0.02)

        raise RuntimeError(
            "Timed out waiting for mission_manager to accept "
            f"execution mode {expected}"
        )

    # ==========================================================
    # Public mission operations used by API routes
    # ==========================================================

    def prepare_trajectory(
        self,
    ) -> dict[str, Any]:
        """Request trajectory preparation and return immediately.

        trajectory_generator owns the asynchronous RTK wait and publishes
        READY only after the path has actually been generated.
        """
        with self._runtime_lock:
            self._trajectory_ready = False
            self._trajectory_error = None
            self._mission_manager_ready = False
            self._mission_manager_error = None
            self._preparation_in_progress = True

        rover_state.set_mission_state(
            "PREPARING",
            message="Trajectory preparation requested; waiting for RTK FIXED",
            error=None,
        )

        # A new LOAD invalidates the previous fixed trajectory preview
        # immediately. trajectory_generator will republish /nav_path and
        # trajectory_ready=True only after the replacement P1->Pn geometry
        # has been completely generated.
        rover_state.update(
            "mission",
            trajectory_ready=False,
            navigation_point_count=0,
            navigation_path_preview=[],
            navigation_path_preview_truncated=False,
            path_frame_id=None,
        )

        accepted, service_message = self._call_trigger(
            client=self._trajectory_prepare_client,
            service_name="/trajectory_generator/prepare",
        )

        if not accepted:
            with self._runtime_lock:
                self._preparation_in_progress = False
            rover_state.set_mission_state(
                "ERROR",
                message=service_message,
                error=service_message,
            )
            raise RuntimeError(service_message)

        rover_state.update(
            "mission",
            message=service_message or "Trajectory preparation accepted",
            error=None,
        )

        return rover_state.section("mission")

    def set_execution_mode(
        self,
        execution_mode: str,
    ) -> dict[str, Any]:
        mode = str(execution_mode).strip().upper()

        if mode not in {"AUTO", "MANUAL"}:
            raise RuntimeError("execution_mode must be AUTO or MANUAL")

        with self._runtime_lock:
            manager_state = self._mission_manager_state

        if manager_state in {
            "RUNNING",
            "PAUSED",
            "WAITING_FOR_NEXT",
        }:
            raise RuntimeError(
                "Cannot change execution mode while mission is " f"{manager_state}"
            )

        self._publish_execution_mode(mode)
        self._wait_for_execution_mode(mode)

        rover_state.update(
            "mission",
            execution_mode=mode,
            message=f"Mission execution mode set to {mode}",
            error=None,
        )

        return rover_state.section("mission")

    def _proxy_manager_operation(
        self,
        command: str,
    ) -> dict[str, Any]:
        accepted, service_message = self._manager_command(command)
        if not accepted:
            # mission_manager returns the exact failed Start/Resume/etc reason.
            raise RuntimeError(service_message)

        rover_state.update(
            "mission",
            message=service_message,
            error=None,
        )
        _notify_authoritative_state_changed()
        return rover_state.section("mission")

    def start_mission(self) -> dict[str, Any]:
        # No path regeneration, RTK check, PX4 mode switch, arm command, or
        # safety-gate logic is allowed here. mission_manager owns all of it.
        return self._proxy_manager_operation("start")

    def pause_mission(self) -> dict[str, Any]:
        return self._proxy_manager_operation("pause")

    def resume_mission(self) -> dict[str, Any]:
        return self._proxy_manager_operation("resume")

    def next_point(self) -> dict[str, Any]:
        return self._proxy_manager_operation("next_point")

    def skip_point(self) -> dict[str, Any]:
        return self._proxy_manager_operation("skip_point")

    def stop_mission(self) -> dict[str, Any]:
        # Mission Manager owns disarm and emits MISSION_TERMINATED only after
        # its STOP contract finishes.  That event drives report-first cleanup.
        self._proxy_manager_operation("stop")
        return rover_state.section("mission")

    def clear_mission(self) -> dict[str, Any]:
        manager_ok, manager_message = self._manager_command("clear")
        trajectory_ok, trajectory_message = self._call_trigger(
            client=self._trajectory_clear_client,
            service_name="/trajectory_generator/clear",
        )

        with self._runtime_lock:
            self._trajectory_ready = False
            self._trajectory_error = None

        if not manager_ok:
            raise RuntimeError(manager_message)
        if not trajectory_ok:
            raise RuntimeError(trajectory_message)

        rover_state.clear_mission_runtime(retain_loaded_file=True)
        return rover_state.section("mission")

    def emergency_stop(self) -> dict[str, Any]:
        accepted, service_message = self._manager_command("emergency_stop")
        if not accepted:
            raise RuntimeError(service_message)
        rover_state.update("safety", reason="OPERATOR_EMERGENCY_STOP")
        _notify_authoritative_state_changed()
        return rover_state.section("safety")

    def release_emergency_stop(
        self,
        expected_generation: int,
    ) -> dict[str, Any]:
        if expected_generation < 0:
            raise RuntimeError(
                "Mission Manager safety generation is unavailable; "
                "refusing emergency-stop release"
            )

        request = ReleaseEmergencyStop.Request()
        request.expected_generation = int(expected_generation)

        accepted, service_message = self._call_service(
            client=self._release_emergency_stop_client,
            request=request,
            service_name="/mission_manager/release_emergency_stop",
            response_timeout_sec=self.MANAGER_RESPONSE_TIMEOUT_SEC[
                "release_emergency_stop"
            ],
            timeout_outcome_unknown=True,
        )

        if not accepted:
            raise RuntimeError(service_message)

        rover_state.update(
            "safety",
            reason="EMERGENCY_STOP_RELEASED",
        )
        _notify_authoritative_state_changed()
        return rover_state.section("safety")


class RosBridgeRuntime:
    """Start and stop the backend ROS executor in a dedicated thread."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._operation_lock = threading.RLock()
        # Ordinary commands remain serialized by _operation_lock. Safety
        # requests use their own ordering lock so E-stop can bypass a long
        # ordinary operation while a queued, older RELEASE is rejected.
        self._safety_command_lock = threading.RLock()
        self._safety_generation = 0
        self._node: RoverBackendRosNode | None = None
        self._executor: MultiThreadedExecutor | None = None
        self._thread: threading.Thread | None = None

    @property
    def node(self) -> RoverBackendRosNode:
        with self._lock:
            if self._node is None:
                raise RuntimeError("ROS bridge is not running")

            return self._node

    @property
    def running(self) -> bool:
        with self._lock:
            return bool(
                self._thread is not None
                and self._thread.is_alive()
                and self._node is not None
            )

    def start(self) -> None:
        with self._lock:
            if self.running:
                return

            if not rclpy.ok():
                rclpy.init(args=None)

            node = RoverBackendRosNode()
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(node)

            thread = threading.Thread(
                target=executor.spin,
                name="rover-backend-ros-spin",
                daemon=True,
            )

            self._node = node
            self._executor = executor
            self._thread = thread

            thread.start()

    def stop(self) -> None:
        with self._lock:
            node = self._node
            executor = self._executor
            thread = self._thread

            self._node = None
            self._executor = None
            self._thread = None

        if node is not None:
            try:
                node.emergency_stop()
            except Exception:
                LOGGER.exception(
                    "Unable to request mission_manager emergency stop during backend shutdown"
                )

        if executor is not None:
            try:
                executor.shutdown(timeout_sec=2.0)
            except Exception:
                LOGGER.exception("ROS executor shutdown failed")

        if thread is not None:
            thread.join(timeout=2.0)

        if node is not None:
            try:
                node._shutdown_runtime_persist_worker()
            except Exception:
                LOGGER.exception(
                    "Mission runtime persistence worker shutdown failed"
                )

        if node is not None:
            try:
                node._shutdown_report_checkpoint_worker()
            except Exception:
                LOGGER.exception(
                    "Mission report checkpoint worker shutdown failed"
                )

        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                LOGGER.exception("ROS node destruction failed")

        if rclpy.ok():
            rclpy.shutdown()

        rover_state.update(
            "ros",
            node_started=False,
            connected=False,
            error="Backend ROS bridge stopped",
        )
        rover_state.force_safe_runtime_state("BACKEND_ROS_BRIDGE_STOPPED")

    def rtk_mavros_ready(self) -> bool:
        """Return the ROS node's cached RTCM endpoint readiness."""

        with self._lock:
            node = self._node

        if node is None:
            return False

        return node.rtk_mavros_ready()

    def force_emergency_stop(self) -> dict[str, Any]:
        # Only protect the ordering token with the safety lock. Never hold a
        # mutex required by E-stop while waiting for a ROS service response.
        with self._safety_command_lock:
            self._safety_generation += 1

        return self.node.emergency_stop()

    def release_emergency_stop(self) -> dict[str, Any]:
        # Capture request ordering and the exact MissionManager safety epoch
        # observed when the operator issued RELEASE.
        with self._safety_command_lock:
            requested_generation = self._safety_generation
            expected_manager_generation = _safe_int(
                rover_state.section("mission").get("safety_generation"),
                -1,
            )

        if expected_manager_generation < 0:
            raise RuntimeError(
                "Mission Manager safety generation is unavailable; "
                "refusing emergency-stop release"
            )

        with self._operation_lock:
            # Reject a RELEASE superseded before ROS dispatch.
            with self._safety_command_lock:
                if requested_generation != self._safety_generation:
                    raise RuntimeError(
                        "Emergency-stop release rejected because a newer "
                        "E-stop assertion occurred while RELEASE was queued"
                    )

            # Do not hold the local safety lock across this blocking call.
            # MissionManager independently validates expected_generation under
            # its own safety/state lock before clearing the hard-stop latch.
            release_result = self.node.release_emergency_stop(
                expected_manager_generation
            )

            with self._safety_command_lock:
                superseded = (
                    requested_generation
                    != self._safety_generation
                )

            if superseded:
                # No compensation service call is required. If the newer E-stop
                # reached MissionManager first, the stale RELEASE was rejected
                # by generation. If RELEASE executed first, the newer E-stop
                # becomes the later authority.
                raise RuntimeError(
                    "Emergency-stop release was superseded by a newer "
                    "E-stop assertion"
                )

            return release_result

    def prepare_trajectory(self) -> dict[str, Any]:
        with self._operation_lock:
            return self.node.prepare_trajectory()

    def set_execution_mode(
        self,
        execution_mode: str,
    ) -> dict[str, Any]:
        with self._operation_lock:
            return self.node.set_execution_mode(execution_mode)

    def start_mission(self) -> dict[str, Any]:
        with self._operation_lock:
            return self.node.start_mission()

    def pause_mission(self) -> dict[str, Any]:
        with self._operation_lock:
            return self.node.pause_mission()

    def resume_mission(self) -> dict[str, Any]:
        with self._operation_lock:
            return self.node.resume_mission()

    def next_point(self) -> dict[str, Any]:
        with self._operation_lock:
            return self.node.next_point()

    def skip_point(self) -> dict[str, Any]:
        with self._operation_lock:
            return self.node.skip_point()

    def get_spray_config(
        self,
    ) -> dict[str, Any]:
        with self._operation_lock:
            return self.node.get_spray_config()

    def set_spray_config(
        self,
        *,
        press_pwm_us: float,
        release_pwm_us: float,
    ) -> dict[str, Any]:
        with self._operation_lock:
            return self.node.set_spray_config(
                press_pwm_us=press_pwm_us,
                release_pwm_us=release_pwm_us,
            )

    def stop_mission(self) -> dict[str, Any]:
        with self._operation_lock:
            return self.node.stop_mission()

    def clear_mission(self) -> dict[str, Any]:
        with self._operation_lock:
            return self.node.clear_mission()


ros_bridge = RosBridgeRuntime()
