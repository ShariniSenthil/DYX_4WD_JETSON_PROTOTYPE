#!/usr/bin/env python3

"""Thread-safe runtime state for the DYX 4WD Rover Backend.

The frontend reads this state through REST and Socket.IO.

The frontend does not directly control ROS topics. All ROS information is
received by ros_bridge.py and written into this shared state.

Safety defaults are intentionally restrictive:

    emergency_stop = True
    mission_enable = False
    mission_state = EMPTY

The rover must never resume movement automatically after a backend restart.
"""

from __future__ import annotations

import copy
import math
import threading

from datetime import datetime
from datetime import timezone
from typing import Any
from typing import Mapping

MISSION_STATES = {
    "EMPTY",
    "LOADED",
    "PREPARING",
    "READY",
    "RUNNING",
    "PAUSED",
    "WAITING_FOR_NEXT",
    "COMPLETED",
    "ERROR",
}


POINT_STATES = {
    "PENDING",
    "ACTIVE",
    "COMPLETED",
    "SKIPPED",
    "FAILED",
}


MISSION_EXECUTION_MODES = {
    "AUTO",
    "MANUAL",
}


def utc_now_iso() -> str:
    """Return a UTC timestamp suitable for JSON responses."""

    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def _safe_float(
    value: Any,
    default: float | None = None,
) -> float | None:
    """Convert a value into a finite float."""

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
    """Convert a value into an integer."""

    try:
        return int(value)

    except (TypeError, ValueError):
        return default


def _calculate_progress(
    completed_points: int,
    skipped_points: int,
    failed_points: int,
    total_points: int,
) -> float:
    """Calculate mission progress from terminal point states."""

    if total_points <= 0:
        return 0.0

    handled_points = max(
        0,
        completed_points + skipped_points + failed_points,
    )

    progress = handled_points / total_points * 100.0

    return round(
        max(
            0.0,
            min(100.0, progress),
        ),
        2,
    )


class RoverState:
    """Central thread-safe state shared by API, ROS and Socket.IO."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._revision = 0

        now = utc_now_iso()

        self._state: dict[str, dict[str, Any]] = {
            "backend": {
                "online": False,
                "service": "rover_backend",
                "version": None,
                "started_at": None,
                "updated_at": now,
                "last_error": None,
            },
            "ros": {
                "node_started": False,
                "connected": False,
                "last_message_at": None,
                "last_heartbeat_at": None,
                "updated_at": now,
                "error": None,
            },
            "vehicle": {
                "connected": False,
                "armed": False,
                "mode": "UNKNOWN",
                "system_status": 0,
                "heading_deg": None,
                "ground_speed_mps": 0.0,
                "linear_speed_mps": 0.0,
                "angular_speed_rps": 0.0,
                "updated_at": now,
            },
            "position": {
                "latitude": None,
                "longitude": None,
                "altitude_m": None,
                "local_x_m": None,
                "local_y_m": None,
                "local_z_m": None,
                "velocity_x_mps": 0.0,
                "velocity_y_mps": 0.0,
                "velocity_z_mps": 0.0,
                "updated_at": now,
            },
            "gps": {
                "fix_type": 0,
                "fix_name": "NO_FIX",
                "satellites_visible": 0,
                "horizontal_accuracy_m": None,
                "vertical_accuracy_m": None,
                "hdop": None,
                "vdop": None,
                "rtk_fixed": False,
                "px4_hrms_source": "MAVLINK_ESTIMATOR_STATUS_230",
                "px4_hrms_m": None,
                "px4_hrms_mm": None,
                "px4_vrms_m": None,
                "px4_vrms_mm": None,
                "px4_estimator_available": False,
                "px4_estimator_flags": 0,
                "px4_estimator_healthy": False,
                "updated_at": now,
            },
            "estimator": {
                "available": False,
                "source": "MAVLINK_ESTIMATOR_STATUS_230",
                "horizontal_accuracy_m": None,
                "horizontal_accuracy_mm": None,
                "vertical_accuracy_m": None,
                "vertical_accuracy_mm": None,
                "vel_ratio": None,
                "pos_horiz_ratio": None,
                "pos_vert_ratio": None,
                "mag_ratio": None,
                "hagl_ratio": None,
                "tas_ratio": None,
                "flags": 0,
                "absolute_horizontal_valid": False,
                "absolute_vertical_valid": False,
                "gps_glitch": False,
                "accel_error": False,
                "healthy": False,
                "updated_at": now,
            },
            "rtk": {
                # Legacy compatibility field. Dedicated RTK API exposes
                # correction stream and GNSS solution independently.
                "status": "UNAVAILABLE",

                "healthy": False,
                "correction_age_sec": None,

                "stream_state": "UNAVAILABLE",
                "stream_connected": False,
                "socket_bytes_received": 0,
                "valid_frames": 0,
                "published_frames": 0,
                "crc_failures": 0,
                "invalid_headers": 0,
                "resync_bytes_discarded": 0,
                "partial_frame_timeouts": 0,
                "oversize_drops": 0,
                "publish_errors": 0,

                "mavros_ready": False,
                "mavros_rtcm_subscribers": 0,
                "worker_mavros_subscribers": -1,
                "max_mavros_rtcm_frame_bytes": None,

                "gga_enabled": False,
                "gga_state": "DISABLED",
                "gga_source_age_sec": None,
                "gga_last_sent_age_sec": None,
                "gga_sent_total": 0,
                "gga_send_errors": 0,

                "updated_at": now,
            },
            "battery": {
                "voltage_v": None,
                "current_a": None,
                "remaining_percent": None,
                "temperature_c": None,
                "status": "UNKNOWN",
                "updated_at": now,
            },
            "accuracy": {
                "available": False,
                "source": "/rpp/accuracy",
                "goal_number": 0,
                "cross_track_error_m": None,
                "cross_track_error_mm": None,
                "cross_track_abs_mm": None,
                "cross_track_side": "UNKNOWN",
                "front_back_error_m": None,
                "front_back_error_mm": None,
                "front_back_abs_mm": None,
                "front_back_position": "UNKNOWN",
                "radial_error_m": None,
                "radial_error_mm": None,
                "closest_radial_error_m": None,
                "closest_radial_error_mm": None,
                "accuracy_target_m": 0.03,
                "accuracy_target_mm": 30.0,
                "test_tolerance_m": 0.05,
                "test_tolerance_mm": 50.0,
                "accuracy_status": "UNAVAILABLE",
                "accuracy_pass": False,
                "within_test_tolerance": False,

                # Exact /rpp/debug pass-through.
                "rpp_debug_available": False,
                "rpp_debug_source": "/rpp/debug",
                "rpp_debug_schema_version": 2,
                "rpp_debug_telemetry_sequence": None,
                "rpp_debug_control_sequence": None,
                "rpp_debug_control_sample_age_ms": None,
                "rpp_debug_receive_age_ms": None,
                "rpp_debug_odom_age_ms": None,
                "rpp_debug_control_dt_ms": None,
                "rpp_debug_control_compute_ms": None,
                "rpp_debug_control_deadline_missed": False,
                "rpp_debug_stream_fresh": False,
                "rpp_debug_dropped_frames": 0,
                "rpp_debug_reason": "UNKNOWN",
                "rpp_control_mode": "UNKNOWN",
                "rpp_goal_number": 0,

                "rpp_actual_speed_mps": None,
                "rpp_command_speed_mps": None,

                "rpp_current_yaw_deg": None,
                "rpp_path_bearing_deg": None,
                "rpp_guidance_bearing_deg": None,
                "rpp_heading_error_deg": None,

                "rpp_distance_to_goal_m": None,

                "rpp_cross_track_error_mm": None,
                "rpp_cross_track_side": "UNKNOWN",

                "rpp_along_remaining_mm": None,
                "rpp_along_position": "UNKNOWN",

                "updated_at": now,
            },
            "network": {
                "rover_ip": None,
                "frontend_connected": False,
                "socket_clients": 0,
                "last_client_seen_at": None,
                "updated_at": now,
            },
            "safety": {
                "emergency_stop": True,
                "mission_enable": False,
                "backend_heartbeat_healthy": False,
                "command_owner": "NONE",
                "safe": True,
                "reason": "BACKEND_STARTUP",
                "updated_at": now,
            },
            "mission": {
                "mission_id": None,
                "mission_run_id": None,
                "filename": None,
                "checksum_sha256": None,
                "coordinate_mode": None,
                "extension_mode": None,
                "dummy_point_distance_m": None,
                "row_transition_threshold_m": None,
                "execution_mode": "AUTO",
                "state": "EMPTY",
                "ready": False,
                # True only after trajectory_generator has committed the
                # fixed surveyed P1->Pn /nav_path.
                "trajectory_ready": False,
                "loaded": False,
                "total_points": 0,
                "navigation_point_count": 0,
                "dummy_point_count": 0,
                "active_point_id": None,
                "active_point_index": None,
                "active_point_number": None,
                "active_point_state": None,
                "completed_points": 0,
                "skipped_points": 0,
                "failed_points": 0,
                "remaining_points": 0,
                "progress_percent": 0.0,
                "marking_active": False,
                "pause_reason": None,
                "resume_available": False,
                "gps_fix_type": 0,
                "rtk_state": "NO_FIX",
                "rtk_fixed": False,
                "rtk_healthy": False,
                "rtk_motion_ok": False,
                "rtk_reason": "RTK status not received",
                "rtk_correction_age_sec": None,
                "gps_fix_status_age_sec": None,
                "rtk_health_status_age_sec": None,
                "rtk_age_status_age_sec": None,
                "backend_heartbeat_healthy": False,
                "mission_enable": False,
                "emergency_stop": True,
                "px4_connected": False,
                "px4_mode": "UNKNOWN",
                "px4_armed": False,
                "spray_controller_ready": False,
                "spray_controller_state": "UNKNOWN",
                "spray_fault_reason": None,
                "spray_enabled": False,
                "spray_gates_mission_progress": False,
                "current_point_spray_confirmed": None,
                "start_stage": "IDLE",
                "start_failed_stage": None,
                "arrival_settle_elapsed_sec": 0.0,
                "arrival_settle_required_sec": 0.30,
                # Survey-truth recording health, defaulted OFF so a backend
                # that has not yet heard from mission_manager never claims
                # physical truth is being captured when it may not be.
                "survey_truth_enabled": False,
                "survey_truth_ready": False,
                "survey_truth_targets_loaded": 0,
                "survey_truth_gnss_samples": 0,
                "survey_truth_coordinate_mode": None,
                "hold_elapsed_sec": 0.0,
                "hold_required_sec": 3.0,
                "alignment_active": False,
                "path_frame_id": None,
                "navigation_path_preview": [],
                "navigation_path_preview_truncated": False,
                "active_waypoint": None,
                "point_status": [],
                "uploaded_at": None,
                "prepared_at": None,
                "started_at": None,
                "paused_at": None,
                "completed_at": None,
                "message": "No mission loaded",
                "error": None,
                "last_point_event": None,
                "point_results": {},
                "terminal_cleanup_status": None,
                "terminal_cleanup_error": None,
                "updated_at": now,
            },
            "report": {
                "available": False,
                "terminal_available": False,
                "status": "UNAVAILABLE",
                "mission_id": None,
                "termination": None,
                "cleanup_complete": False,
                "error": None,
                "report_url": None,
                "download_url": None,
                "generated_at": None,
                "updated_at": now,
            },
        }

    # ==========================================================
    # Internal helpers
    # ==========================================================

    def _touch_locked(
        self,
        section_name: str,
    ) -> None:
        """Update timestamps and revision while lock is held."""

        now = utc_now_iso()

        self._state[section_name]["updated_at"] = now

        self._revision += 1

    def _validate_section_locked(
        self,
        section_name: str,
    ) -> None:
        if section_name not in self._state:
            raise KeyError(f"Unknown state section: {section_name}")

    def _normalise_mission_locked(
        self,
    ) -> None:
        """Keep mission totals and progress internally consistent."""

        mission = self._state["mission"]

        total_points = max(
            0,
            _safe_int(
                mission.get("total_points"),
                0,
            ),
        )

        completed_points = max(
            0,
            _safe_int(
                mission.get("completed_points"),
                0,
            ),
        )

        skipped_points = max(
            0,
            _safe_int(
                mission.get("skipped_points"),
                0,
            ),
        )

        failed_points = max(
            0,
            _safe_int(
                mission.get("failed_points"),
                0,
            ),
        )

        terminal_points = min(
            total_points,
            (completed_points + skipped_points + failed_points),
        )

        mission["total_points"] = total_points
        mission["completed_points"] = min(
            completed_points,
            total_points,
        )

        mission["skipped_points"] = min(
            skipped_points,
            total_points,
        )

        mission["failed_points"] = min(
            failed_points,
            total_points,
        )

        mission["remaining_points"] = max(
            0,
            total_points - terminal_points,
        )

        mission["progress_percent"] = _calculate_progress(
            completed_points=(mission["completed_points"]),
            skipped_points=(mission["skipped_points"]),
            failed_points=(mission["failed_points"]),
            total_points=total_points,
        )

        execution_mode = (
            str(
                mission.get(
                    "execution_mode",
                    "AUTO",
                )
            )
            .strip()
            .upper()
        )

        if execution_mode not in MISSION_EXECUTION_MODES:
            execution_mode = "AUTO"

        mission["execution_mode"] = execution_mode

        mission_state = (
            str(
                mission.get(
                    "state",
                    "EMPTY",
                )
            )
            .strip()
            .upper()
        )

        if mission_state not in MISSION_STATES:
            mission_state = "ERROR"
            mission["error"] = "Invalid mission state received"

        mission["state"] = mission_state

        if mission_state == "EMPTY":
            mission["ready"] = False
            mission["loaded"] = False

        elif mission_state in {
            "LOADED",
            "PREPARING",
            "READY",
            "RUNNING",
            "PAUSED",
            "WAITING_FOR_NEXT",
            "COMPLETED",
            "ERROR",
        }:
            mission["loaded"] = True

        if mission_state in {
            "READY",
            "RUNNING",
            "PAUSED",
            "WAITING_FOR_NEXT",
            "COMPLETED",
        }:
            mission["ready"] = True

        elif mission_state in {
            "EMPTY",
            "LOADED",
            "PREPARING",
            "ERROR",
        }:
            mission["ready"] = False

    # ==========================================================
    # Generic state access
    # ==========================================================

    def snapshot(self) -> dict[str, Any]:
        """Return a complete independent JSON-safe state snapshot."""

        with self._lock:
            result = copy.deepcopy(self._state)

            result["revision"] = self._revision

            result["generated_at"] = utc_now_iso()

            return result

    def section(
        self,
        section_name: str,
    ) -> dict[str, Any]:
        """Return an independent copy of one state section."""

        with self._lock:
            self._validate_section_locked(section_name)

            return copy.deepcopy(self._state[section_name])

    def update(
        self,
        section_name: str,
        **values: Any,
    ) -> dict[str, Any]:
        """Update selected fields in a state section."""

        with self._lock:
            self._validate_section_locked(section_name)

            section = self._state[section_name]

            for key, value in values.items():
                if key == "updated_at":
                    continue

                section[key] = copy.deepcopy(value)

            if section_name == "mission":
                self._normalise_mission_locked()

            self._touch_locked(section_name)

            return copy.deepcopy(section)

    def update_section(
        self,
        section_name: str,
        values: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Mapping-based equivalent of update()."""

        return self.update(
            section_name,
            **dict(values),
        )

    def replace(
        self,
        section_name: str,
        values: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Replace a complete state section."""

        with self._lock:
            self._validate_section_locked(section_name)

            replacement = copy.deepcopy(dict(values))

            replacement.pop(
                "updated_at",
                None,
            )

            replacement["updated_at"] = utc_now_iso()

            self._state[section_name] = replacement

            if section_name == "mission":
                self._normalise_mission_locked()

            self._revision += 1

            return copy.deepcopy(self._state[section_name])

    def replace_section(
        self,
        section_name: str,
        values: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Compatibility alias for replace()."""

        return self.replace(
            section_name,
            values,
        )

    # ==========================================================
    # Backend and ROS lifecycle
    # ==========================================================

    def mark_backend_online(
        self,
        *,
        version: str,
    ) -> None:
        now = utc_now_iso()

        self.update(
            "backend",
            online=True,
            version=version,
            started_at=now,
            last_error=None,
        )

    def mark_backend_offline(
        self,
        *,
        error: str | None = None,
    ) -> None:
        self.update(
            "backend",
            online=False,
            last_error=error,
        )

    def mark_ros_node_started(
        self,
    ) -> None:
        self.update(
            "ros",
            node_started=True,
            error=None,
        )

    def mark_ros_message_received(
        self,
    ) -> None:
        now = utc_now_iso()

        self.update(
            "ros",
            connected=True,
            last_message_at=now,
            error=None,
        )

    def mark_ros_disconnected(
        self,
        *,
        reason: str,
    ) -> None:
        self.update(
            "ros",
            connected=False,
            error=reason,
        )

    # ==========================================================
    # Safety
    # ==========================================================

    def force_safe_runtime_state(
        self,
        reason: str,
    ) -> None:
        """Force the backend state into the non-driving condition."""

        with self._lock:
            safety = self._state["safety"]

            safety.update(
                {
                    "emergency_stop": True,
                    "mission_enable": False,
                    "backend_heartbeat_healthy": False,
                    "command_owner": "NONE",
                    "safe": True,
                    "reason": str(reason),
                }
            )

            self._touch_locked("safety")

            mission = self._state["mission"]

            if mission.get("state") == "RUNNING":
                mission["state"] = "PAUSED"

            mission["marking_active"] = False
            mission["resume_available"] = False
            mission["arrival_settle_elapsed_sec"] = 0.0
            mission["hold_elapsed_sec"] = 0.0

            self._normalise_mission_locked()
            self._touch_locked("mission")

    def set_safety_state(
        self,
        *,
        emergency_stop: bool,
        mission_enable: bool,
        command_owner: str,
        reason: str,
        heartbeat_healthy: bool,
    ) -> None:
        """Update the commanded rover safety state."""

        emergency_stop = bool(emergency_stop)

        mission_enable = bool(mission_enable)

        # An asserted emergency stop always overrides mission enable.
        if emergency_stop:
            mission_enable = False

        safe = emergency_stop or not mission_enable

        self.update(
            "safety",
            emergency_stop=emergency_stop,
            mission_enable=mission_enable,
            backend_heartbeat_healthy=bool(heartbeat_healthy),
            command_owner=str(command_owner).strip().upper() or "NONE",
            safe=safe,
            reason=str(reason),
        )

    # ==========================================================
    # Mission state helpers
    # ==========================================================

    def load_mission(
        self,
        *,
        mission_id: str,
        filename: str,
        checksum_sha256: str,
        coordinate_mode: str,
        extension_mode: str,
        dummy_point_distance_m: float | None,
        row_transition_threshold_m: float,
        total_points: int,
        uploaded_at: str,
    ) -> None:
        """Set shared state after a valid CSV has been stored."""

        total_points = max(
            0,
            int(total_points),
        )

        self.update(
            "mission",
            mission_id=mission_id,
            mission_run_id=None,
            filename=filename,
            checksum_sha256=(checksum_sha256),
            coordinate_mode=(coordinate_mode),
            extension_mode=(extension_mode),
            dummy_point_distance_m=(_safe_float(dummy_point_distance_m)),
            row_transition_threshold_m=(_safe_float(row_transition_threshold_m)),
            state="LOADED",
            ready=False,
            loaded=True,
            total_points=total_points,
            navigation_point_count=0,
            dummy_point_count=0,
            active_point_id=None,
            active_point_index=None,
            active_point_number=None,
            active_point_state=None,
            completed_points=0,
            skipped_points=0,
            failed_points=0,
            remaining_points=total_points,
            progress_percent=0.0,
            marking_active=False,
            pause_reason=None,
            resume_available=False,
            arrival_settle_elapsed_sec=0.0,
            arrival_settle_required_sec=0.30,
            hold_elapsed_sec=0.0,
            hold_required_sec=3.0,
            alignment_active=False,
            path_frame_id=None,
            navigation_path_preview=[],
            navigation_path_preview_truncated=False,
            active_waypoint=None,
            point_status=[],
            last_point_event=None,
            point_results={},
            terminal_cleanup_status=None,
            terminal_cleanup_error=None,
            spray_enabled=False,
            spray_gates_mission_progress=False,
            current_point_spray_confirmed=None,
            uploaded_at=uploaded_at,
            prepared_at=None,
            started_at=None,
            paused_at=None,
            completed_at=None,
            message=("Mission uploaded and awaiting " "trajectory preparation"),
            error=None,
        )

    def set_mission_state(
        self,
        mission_state: str,
        *,
        message: str | None = None,
        error: str | None = None,
        **extra_values: Any,
    ) -> None:
        """Set a validated mission lifecycle state."""

        normalised_state = str(mission_state).strip().upper()

        if normalised_state not in MISSION_STATES:
            raise ValueError("Invalid mission state: " f"{mission_state}")

        values: dict[str, Any] = {
            "state": normalised_state,
            "error": error,
        }

        if message is not None:
            values["message"] = message

        values.update(extra_values)

        self.update(
            "mission",
            **values,
        )

    def set_active_point(
        self,
        *,
        point_id: str | None,
        point_index: int | None,
        point_state: str | None,
    ) -> None:
        """Update the active original CSV marking point."""

        normalised_point_state = None

        if point_state is not None:
            normalised_point_state = str(point_state).strip().upper()

            if normalised_point_state not in POINT_STATES:
                raise ValueError("Invalid point state: " f"{point_state}")

        active_point_number = None

        if point_index is not None:
            point_index = int(point_index)

            if point_index < 0:
                raise ValueError("point_index must be >= 0")

            active_point_number = point_index + 1

        self.update(
            "mission",
            active_point_id=point_id,
            active_point_index=point_index,
            active_point_number=(active_point_number),
            active_point_state=(normalised_point_state),
        )

    def set_mission_counts(
        self,
        *,
        completed_points: int,
        skipped_points: int,
        failed_points: int = 0,
        total_points: int | None = None,
    ) -> None:
        """Update mission totals using runtime point statuses."""

        values: dict[str, Any] = {
            "completed_points": max(
                0,
                int(completed_points),
            ),
            "skipped_points": max(
                0,
                int(skipped_points),
            ),
            "failed_points": max(
                0,
                int(failed_points),
            ),
        }

        if total_points is not None:
            values["total_points"] = max(
                0,
                int(total_points),
            )

        self.update(
            "mission",
            **values,
        )

    def set_marking_hold(
        self,
        *,
        active: bool,
        elapsed_sec: float,
        required_sec: float | None = None,
    ) -> None:
        """Update the current marking hold information."""

        elapsed = max(
            0.0,
            _safe_float(
                elapsed_sec,
                0.0,
            )
            or 0.0,
        )

        values: dict[str, Any] = {
            "marking_active": bool(active),
            "hold_elapsed_sec": elapsed,
        }

        if required_sec is not None:
            required = max(
                0.0,
                _safe_float(
                    required_sec,
                    0.0,
                )
                or 0.0,
            )
            values["hold_required_sec"] = required

        self.update(
            "mission",
            **values,
        )

    def clear_mission_runtime(
        self,
        *,
        retain_loaded_file: bool,
    ) -> None:
        """Clear execution progress.

        retain_loaded_file=True:
            Keep mission metadata and return to LOADED.

        retain_loaded_file=False:
            Clear all mission state and return to EMPTY.
        """

        with self._lock:
            mission = self._state["mission"]

            total_points = (
                max(
                    0,
                    _safe_int(
                        mission.get("total_points"),
                        0,
                    ),
                )
                if retain_loaded_file
                else 0
            )

            retained_values = {}

            if retain_loaded_file:
                for key in (
                    "mission_id",
                    "filename",
                    "checksum_sha256",
                    "coordinate_mode",
                    "extension_mode",
                    "dummy_point_distance_m",
                    "row_transition_threshold_m",
                    "execution_mode",
                    "uploaded_at",
                ):
                    retained_values[key] = copy.deepcopy(mission.get(key))

            replacement = {
                "mission_id": None,
                "mission_run_id": None,
                "filename": None,
                "checksum_sha256": None,
                "coordinate_mode": None,
                "extension_mode": None,
                "dummy_point_distance_m": None,
                "row_transition_threshold_m": None,
                "execution_mode": (
                    mission.get(
                        "execution_mode",
                        "AUTO",
                    )
                    if retain_loaded_file
                    else "AUTO"
                ),
                "state": ("LOADED" if retain_loaded_file else "EMPTY"),
                "ready": False,
                "loaded": bool(retain_loaded_file),
                "total_points": total_points,
                "navigation_point_count": 0,
                "dummy_point_count": 0,
                "active_point_id": None,
                "active_point_index": None,
                "active_point_number": None,
                "active_point_state": None,
                "completed_points": 0,
                "skipped_points": 0,
                "failed_points": 0,
                "remaining_points": (total_points),
                "progress_percent": 0.0,
                "marking_active": False,
                "pause_reason": None,
                "resume_available": False,
                "gps_fix_type": 0,
                "rtk_state": "NO_FIX",
                "rtk_fixed": False,
                "rtk_healthy": False,
                "rtk_motion_ok": False,
                "rtk_reason": "RTK status not received",
                "rtk_correction_age_sec": None,
                "gps_fix_status_age_sec": None,
                "rtk_health_status_age_sec": None,
                "rtk_age_status_age_sec": None,
                "backend_heartbeat_healthy": False,
                "mission_enable": False,
                "emergency_stop": True,
                "px4_connected": False,
                "px4_mode": "UNKNOWN",
                "px4_armed": False,
                "spray_controller_ready": False,
                "spray_controller_state": "UNKNOWN",
                "spray_fault_reason": None,
                "spray_enabled": False,
                "spray_gates_mission_progress": False,
                "current_point_spray_confirmed": None,
                "start_stage": "IDLE",
                "start_failed_stage": None,
                "arrival_settle_elapsed_sec": 0.0,
                "arrival_settle_required_sec": (
                    mission.get(
                        "arrival_settle_required_sec",
                        0.30,
                    )
                ),
                "hold_elapsed_sec": 0.0,
                "hold_required_sec": 3.0,
                "alignment_active": False,
                "path_frame_id": None,
                "navigation_path_preview": [],
                "navigation_path_preview_truncated": False,
                "active_waypoint": None,
                "point_status": [],
                "uploaded_at": None,
                "prepared_at": None,
                "started_at": None,
                "paused_at": None,
                "completed_at": None,
                "message": (
                    "Mission file retained; " "runtime progress cleared"
                    if retain_loaded_file
                    else "No mission loaded"
                ),
                "error": None,
                "last_point_event": None,
                "point_results": {},
                "terminal_cleanup_status": None,
                "terminal_cleanup_error": None,
                "updated_at": utc_now_iso(),
            }

            replacement.update(retained_values)

            self._state["mission"] = replacement

            self._normalise_mission_locked()
            self._revision += 1


rover_state = RoverState()
