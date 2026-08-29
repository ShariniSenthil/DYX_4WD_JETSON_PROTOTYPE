"""System, health, telemetry, network, RTK and safety API routes.

This module exposes read-only rover status and authenticated emergency-stop
controls. It does not read or modify mission.csv and performs no path
planning.
"""

from __future__ import annotations

import ipaddress
import math
import socket
import subprocess

from datetime import datetime
from datetime import timezone
from typing import Any

import psutil

from fastapi import APIRouter
from fastapi import Depends
from fastapi import HTTPException
from fastapi import status
from starlette.concurrency import run_in_threadpool

from rover_backend.auth import AuthenticatedSession
from rover_backend.auth import require_auth
from rover_backend.config import settings
from rover_backend.ros_bridge import ros_bridge
from rover_backend.state import rover_state
from rover_backend.state import utc_now_iso

system_router = APIRouter(
    tags=["system"],
)

# Compatibility alias used by older main.py versions. The final main.py will
# import system_router directly.
router = system_router


def _finite_float(
    value: Any,
    default: float | None = None,
) -> float | None:
    """Return a finite float or the supplied default."""

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
    """Return an integer without raising for malformed telemetry."""

    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _iso_age_seconds(
    value: Any,
) -> float | None:
    """Calculate the age of a UTC ISO-8601 timestamp."""

    if not isinstance(value, str) or not value.strip():
        return None

    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None

    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)

    age = (datetime.now(timezone.utc) - parsed.astimezone(timezone.utc)).total_seconds()

    return round(
        max(0.0, age),
        3,
    )


def _run_command(
    arguments: list[str],
) -> str | None:
    """Run a fixed local status command with a strict timeout."""

    try:
        result = subprocess.run(
            arguments,
            check=False,
            capture_output=True,
            text=True,
            timeout=1.0,
        )
    except (
        OSError,
        subprocess.SubprocessError,
    ):
        return None

    if result.returncode != 0:
        return None

    output = result.stdout.strip()
    return output or None


def collect_network_status() -> dict[str, Any]:
    """Collect the Jetson IPv4 interface and optional Wi-Fi information."""

    selected_interface: str | None = None
    selected_ip: str | None = None
    fallback_interface: str | None = None
    fallback_ip: str | None = None
    ipv4_addresses: list[dict[str, str | None]] = []

    try:
        interfaces = psutil.net_if_addrs()
        interface_stats = psutil.net_if_stats()
    except Exception:
        interfaces = {}
        interface_stats = {}

    for interface_name, addresses in interfaces.items():
        for address in addresses:
            if address.family != socket.AF_INET:
                continue

            try:
                parsed = ipaddress.ip_address(address.address)
            except ValueError:
                continue

            if parsed.is_loopback:
                continue

            ipv4_addresses.append(
                {
                    "interface": interface_name,
                    "address": address.address,
                    "netmask": address.netmask,
                }
            )

            if fallback_ip is None:
                fallback_interface = interface_name
                fallback_ip = address.address

            if address.address == settings.rover_ip:
                selected_interface = interface_name
                selected_ip = address.address

    if selected_ip is None:
        selected_interface = fallback_interface
        selected_ip = fallback_ip

    interface_is_up = bool(
        selected_interface
        and selected_interface in interface_stats
        and interface_stats[selected_interface].isup
    )

    wifi_ssid: str | None = None
    wifi_rssi_dbm: int | None = None

    is_wifi_interface = bool(
        selected_interface and selected_interface.lower().startswith(("wl", "wlan"))
    )

    if is_wifi_interface and selected_interface:
        wifi_ssid = _run_command(
            [
                "iwgetid",
                selected_interface,
                "--raw",
            ]
        )

        link_output = _run_command(
            [
                "iw",
                "dev",
                selected_interface,
                "link",
            ]
        )

        if link_output:
            for line in link_output.splitlines():
                stripped = line.strip()

                if not stripped.startswith("signal:"):
                    continue

                parts = stripped.split()

                if len(parts) < 2:
                    continue

                try:
                    wifi_rssi_dbm = int(round(float(parts[1])))
                except ValueError:
                    wifi_rssi_dbm = None

                break

    payload: dict[str, Any] = {
        "rover_ip": settings.rover_ip,
        "interface": selected_interface,
        "ip_address": selected_ip,
        "expected_ip_address": settings.rover_ip,
        "ip_matches_configuration": (selected_ip == settings.rover_ip),
        "connected": interface_is_up,
        "wifi_interface": is_wifi_interface,
        "wifi_connected": bool(interface_is_up and is_wifi_interface and wifi_ssid),
        "wifi_ssid": wifi_ssid,
        "wifi_rssi_dbm": wifi_rssi_dbm,
        "ipv4_addresses": ipv4_addresses,
        "updated_at": utc_now_iso(),
    }

    rover_state.update(
        "network",
        **payload,
    )

    return payload


def build_mission_status_payload() -> dict[str, Any]:
    """Build the stable mission-status contract used by REST and Socket.IO."""

    mission = rover_state.section("mission")
    report = rover_state.section("report")

    state_name = str(mission.get("state") or "EMPTY").strip().upper()

    return {
        "mission_id": mission.get("mission_id"),
        "mission_run_id": mission.get("mission_run_id"),
        "filename": mission.get("filename"),
        "checksum_sha256": mission.get("checksum_sha256"),
        "coordinate_mode": mission.get("coordinate_mode"),
        "extension_mode": mission.get("extension_mode"),
        "dummy_point_distance_m": mission.get("dummy_point_distance_m"),
        "row_transition_threshold_m": mission.get("row_transition_threshold_m"),
        "state": state_name,
        "state_lower": state_name.lower(),
        "loaded": bool(mission.get("loaded", False)),
        "ready": bool(mission.get("ready", False)),
        "total_points": max(
            0,
            _safe_int(
                mission.get("total_points"),
                0,
            ),
        ),
        "navigation_point_count": max(
            0,
            _safe_int(
                mission.get("navigation_point_count"),
                0,
            ),
        ),
        "dummy_point_count": max(
            0,
            _safe_int(
                mission.get("dummy_point_count"),
                0,
            ),
        ),
        "active_point_id": mission.get("active_point_id"),
        "active_point_index": mission.get("active_point_index"),
        "active_point_number": mission.get("active_point_number"),
        "active_point_state": mission.get("active_point_state"),
        "completed_points": max(
            0,
            _safe_int(
                mission.get("completed_points"),
                0,
            ),
        ),
        "skipped_points": max(
            0,
            _safe_int(
                mission.get("skipped_points"),
                0,
            ),
        ),
        "failed_points": max(
            0,
            _safe_int(
                mission.get("failed_points"),
                0,
            ),
        ),
        "remaining_points": max(
            0,
            _safe_int(
                mission.get("remaining_points"),
                0,
            ),
        ),
        "progress_percent": (
            _finite_float(
                mission.get("progress_percent"),
                0.0,
            )
            or 0.0
        ),
        "marking_active": bool(mission.get("marking_active", False)),
        "pause_reason": mission.get("pause_reason"),
        "resume_available": bool(mission.get("resume_available", False)),
        "gps_fix_type": int(mission.get("gps_fix_type", 0) or 0),
        "rtk_state": mission.get("rtk_state"),
        "rtk_fixed": bool(mission.get("rtk_fixed", False)),
        "rtk_healthy": bool(mission.get("rtk_healthy", False)),
        "rtk_motion_ok": bool(mission.get("rtk_motion_ok", False)),
        "rtk_reason": mission.get("rtk_reason"),
        "rtk_correction_age_sec": _finite_float(mission.get("rtk_correction_age_sec")),
        "gps_fix_status_age_sec": _finite_float(mission.get("gps_fix_status_age_sec")),
        "rtk_health_status_age_sec": _finite_float(
            mission.get("rtk_health_status_age_sec")
        ),
        "rtk_age_status_age_sec": _finite_float(mission.get("rtk_age_status_age_sec")),
        "backend_heartbeat_healthy": bool(
            mission.get("backend_heartbeat_healthy", False)
        ),
        "mission_enable": bool(mission.get("mission_enable", False)),
        "emergency_stop": bool(mission.get("emergency_stop", True)),
        "px4_connected": bool(mission.get("px4_connected", False)),
        "px4_mode": mission.get("px4_mode"),
        "px4_armed": bool(mission.get("px4_armed", False)),
        "spray_controller_ready": bool(mission.get("spray_controller_ready", False)),
        "spray_controller_state": mission.get("spray_controller_state"),
        "spray_fault_reason": mission.get("spray_fault_reason"),
        "spray_enabled": bool(mission.get("spray_enabled", False)),
        "spray_gates_mission_progress": bool(
            mission.get("spray_gates_mission_progress", False)
        ),
        "current_point_spray_confirmed": mission.get("current_point_spray_confirmed"),
        "start_stage": str(mission.get("start_stage") or "IDLE").strip().upper(),
        "start_failed_stage": mission.get("start_failed_stage"),
        "arrival_settle_elapsed_sec": (
            _finite_float(
                mission.get(
                    "arrival_settle_elapsed_sec",
                    0.0,
                ),
                0.0,
            )
            or 0.0
        ),
        "arrival_settle_required_sec": (
            _finite_float(
                mission.get(
                    "arrival_settle_required_sec",
                    settings.arrival_settle_seconds,
                ),
                settings.arrival_settle_seconds,
            )
            or settings.arrival_settle_seconds
        ),
        "alignment_active": bool(mission.get("alignment_active", False)),
        "hold_elapsed_sec": (
            _finite_float(
                mission.get("hold_elapsed_sec"),
                0.0,
            )
            or 0.0
        ),
        "hold_required_sec": (
            _finite_float(
                mission.get("hold_required_sec"),
                settings.marking_hold_seconds,
            )
            or settings.marking_hold_seconds
        ),
        "active_waypoint": mission.get("active_waypoint"),
        "point_status": mission.get(
            "point_status",
            [],
        ),
        "last_point_event": mission.get("last_point_event"),
        "point_results": mission.get(
            "point_results",
            {},
        ),
        "terminal_cleanup_status": mission.get("terminal_cleanup_status"),
        "terminal_cleanup_error": mission.get("terminal_cleanup_error"),
        "report": report,
        "uploaded_at": mission.get("uploaded_at"),
        "prepared_at": mission.get("prepared_at"),
        "started_at": mission.get("started_at"),
        "paused_at": mission.get("paused_at"),
        "completed_at": mission.get("completed_at"),
        "message": mission.get("message"),
        "error": mission.get("error"),
        "updated_at": mission.get("updated_at"),
    }


def build_telemetry_payload() -> dict[str, Any]:
    """Build the rover telemetry contract used by REST and Socket.IO."""

    snapshot = rover_state.snapshot()

    backend = snapshot["backend"]
    ros = snapshot["ros"]
    vehicle = snapshot["vehicle"]
    position = snapshot["position"]
    gps = snapshot["gps"]
    estimator = dict(snapshot.get("estimator", {}))
    rtk = snapshot["rtk"]
    battery = snapshot["battery"]
    accuracy = snapshot["accuracy"]
    safety = snapshot["safety"]
    mission = build_mission_status_payload()

    local_x = _finite_float(position.get("local_x_m"))
    local_y = _finite_float(position.get("local_y_m"))
    ground_speed = (
        _finite_float(
            vehicle.get("ground_speed_mps"),
            0.0,
        )
        or 0.0
    )
    heading = _finite_float(vehicle.get("heading_deg"))

    # Include nested production fields and a small flat compatibility layer.
    # The frontend adapter can migrate to nested fields without changing the
    # rover-side telemetry source again.
    return {
        "generated_at": snapshot.get(
            "generated_at",
            utc_now_iso(),
        ),
        "revision": snapshot.get("revision", 0),
        "backend": backend,
        "ros": ros,
        "vehicle": vehicle,
        "position": position,
        "gps": gps,
        "estimator": estimator,
        "rtk": rtk,
        "battery": battery,
        "accuracy": accuracy,
        "safety": safety,
        "mission": mission,
        "pos_e": local_x,
        "pos_n": local_y,
        "heading_deg": heading,
        "heading_ned_deg": heading,
        "speed_mps": ground_speed,
        "speed_m_s": ground_speed,
        "armed": bool(vehicle.get("armed", False)),
        "mode": str(vehicle.get("mode") or "UNKNOWN"),
        "connected": bool(vehicle.get("connected", False)),
        "latitude": position.get("latitude"),
        "longitude": position.get("longitude"),
        "altitude_m": position.get("altitude_m"),
        "lat": position.get("latitude"),
        "lon": position.get("longitude"),
        "alt": position.get("altitude_m"),
        "battery_v": battery.get("voltage_v"),
        "battery_pct": battery.get("remaining_percent"),
        "gps_fix": _safe_int(
            gps.get("fix_type"),
            0,
        ),
        "gps_fix_name": str(gps.get("fix_name") or "NO_FIX"),
        "gps_sat": _safe_int(
            gps.get("satellites_visible"),
            0,
        ),
        "hrms": gps.get("px4_hrms_m"),
        "vrms": gps.get("px4_vrms_m"),
        "rtk_fixed": bool(gps.get("rtk_fixed", False)),
        "rtk_healthy": bool(rtk.get("healthy", False)),
        "rtk_correction_age_sec": rtk.get("correction_age_sec"),
        "emergency_stop": bool(safety.get("emergency_stop", True)),
        "mission_enable": bool(safety.get("mission_enable", False)),
        "mission_state": mission["state"],
        "marking_active": mission["marking_active"],
        # Flat additive fields for the current frontend adapter.
        # Signed values remain visible even when they exceed 50 mm.
        "cross_track_error_mm": accuracy.get("cross_track_error_mm"),
        "cross_track_abs_mm": accuracy.get("cross_track_abs_mm"),
        "cross_track_side": accuracy.get("cross_track_side"),
        "front_back_error_mm": accuracy.get("front_back_error_mm"),
        "front_back_abs_mm": accuracy.get("front_back_abs_mm"),
        "front_back_position": accuracy.get("front_back_position"),
        "radial_error_mm": accuracy.get("radial_error_mm"),
        "closest_radial_error_mm": accuracy.get("closest_radial_error_mm"),
        "accuracy_status": accuracy.get("accuracy_status"),
        "accuracy_pass": bool(accuracy.get("accuracy_pass", False)),
        "within_test_tolerance": bool(
            accuracy.get(
                "within_test_tolerance",
                False,
            )
        ),

        # Exact RPP controller-view telemetry.
        "rpp_debug_available": bool(
            accuracy.get("rpp_debug_available", False)
        ),
        "rpp_control_mode": accuracy.get("rpp_control_mode"),
        "rpp_goal_number": accuracy.get("rpp_goal_number"),

        "rpp_actual_speed_mps": accuracy.get("rpp_actual_speed_mps"),
        "rpp_command_speed_mps": accuracy.get("rpp_command_speed_mps"),

        "rpp_current_yaw_deg": accuracy.get("rpp_current_yaw_deg"),
        "rpp_path_bearing_deg": accuracy.get("rpp_path_bearing_deg"),
        "rpp_guidance_bearing_deg": accuracy.get(
            "rpp_guidance_bearing_deg"
        ),
        "rpp_heading_error_deg": accuracy.get("rpp_heading_error_deg"),

        "rpp_distance_to_goal_m": accuracy.get(
            "rpp_distance_to_goal_m"
        ),

        "rpp_cross_track_error_mm": accuracy.get(
            "rpp_cross_track_error_mm"
        ),
        "rpp_cross_track_side": accuracy.get("rpp_cross_track_side"),

        "rpp_along_remaining_mm": accuracy.get(
            "rpp_along_remaining_mm"
        ),
        "rpp_along_position": accuracy.get("rpp_along_position"),
    }


def _require_ros_bridge() -> None:
    if not ros_bridge.running:
        raise HTTPException(
            status_code=(status.HTTP_503_SERVICE_UNAVAILABLE),
            detail=("The rover ROS bridge is not running."),
        )


@system_router.get("/api/health")
async def health() -> dict[str, Any]:
    """Unauthenticated discovery presence. UDP is only a hint."""

    return {
        "ok": True,
        "type": "rover_backend",
        "rover_id": settings.rover_id,
        "rover_name": settings.rover_name,
        "port": settings.backend_port,
        "version": settings.application_version,
    }


@system_router.get("/api/ping")
async def ping() -> dict[str, Any]:
    """Public endpoint used to identify the rover backend."""

    return {
        "success": True,
        "application": settings.application_name,
        "service": settings.service_name,
        "version": settings.application_version,
        "rover_id": settings.rover_id,
        "rover_name": settings.rover_name,
        "ip": settings.rover_ip,
        "port": settings.backend_port,
        "base_url": (f"http://{settings.rover_ip}:" f"{settings.backend_port}"),
        "timestamp": utc_now_iso(),
    }


@system_router.get("/api/healthz")
async def healthz(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Return backend, ROS, vehicle and safety health."""

    snapshot = rover_state.snapshot()

    backend = snapshot["backend"]
    ros = snapshot["ros"]
    vehicle = snapshot["vehicle"]
    safety = snapshot["safety"]
    mission = snapshot["mission"]

    backend_online = bool(backend.get("online", False))
    ros_node_started = bool(ros.get("node_started", False))
    ros_connected = bool(ros.get("connected", False))
    vehicle_connected = bool(vehicle.get("connected", False))

    safe_non_driving = bool(
        safety.get("emergency_stop", True) and not safety.get("mission_enable", False)
    )

    health_state = "OK"

    if not backend_online or not ros_node_started:
        health_state = "ERROR"
    elif not ros_connected or not vehicle_connected:
        health_state = "DEGRADED"

    return {
        "status": health_state,
        "backend_online": backend_online,
        "backend_version": backend.get("version"),
        "backend_started_at": backend.get("started_at"),
        "backend_uptime_sec": _iso_age_seconds(backend.get("started_at")),
        "ros_node_started": ros_node_started,
        "ros_connected": ros_connected,
        "ros_last_message_age_sec": (_iso_age_seconds(ros.get("last_message_at"))),
        "backend_heartbeat_last_age_sec": (
            _iso_age_seconds(ros.get("last_heartbeat_at"))
        ),
        "backend_heartbeat_healthy": bool(
            safety.get(
                "backend_heartbeat_healthy",
                False,
            )
        ),
        "vehicle_connected": vehicle_connected,
        "armed": bool(vehicle.get("armed", False)),
        "mode": str(vehicle.get("mode") or "UNKNOWN"),
        "safe_non_driving": safe_non_driving,
        "emergency_stop": bool(safety.get("emergency_stop", True)),
        "mission_enable": bool(safety.get("mission_enable", False)),
        "mission_state": str(mission.get("state") or "EMPTY").upper(),
        "generated_at": snapshot.get(
            "generated_at",
            utc_now_iso(),
        ),
        "revision": snapshot.get("revision", 0),
    }


@system_router.get("/api/telemetry/latest")
async def latest_telemetry(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Return the latest complete rover telemetry snapshot."""

    return build_telemetry_payload()


@system_router.get("/api/network")
async def network_status(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Return Jetson interface and configured rover-IP status."""

    return await run_in_threadpool(collect_network_status)


@system_router.get("/api/rtk/status")
async def rtk_status(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Return GPS fix and RTK correction-bridge health."""

    gps = rover_state.section("gps")
    rtk = rover_state.section("rtk")

    correction_age = _finite_float(rtk.get("correction_age_sec"))

    return {
        "healthy": bool(rtk.get("healthy", False)),
        "status": str(rtk.get("status") or "UNAVAILABLE"),
        "correction_age_sec": correction_age,
        "correction_fresh": bool(correction_age is not None and correction_age <= 2.0),
        "fix_type": _safe_int(
            gps.get("fix_type"),
            0,
        ),
        "fix_name": str(gps.get("fix_name") or "NO_FIX"),
        "rtk_fixed": bool(gps.get("rtk_fixed", False)),
        "satellites_visible": _safe_int(
            gps.get("satellites_visible"),
            0,
        ),
        "hdop": gps.get("hdop"),
        "vdop": gps.get("vdop"),
        "gps_updated_at": gps.get("updated_at"),
        "rtk_updated_at": rtk.get("updated_at"),
    }


@system_router.get("/api/safety/status")
async def safety_status(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Return the current backend-owned safety gate."""

    return {
        "success": True,
        "safety": rover_state.section("safety"),
    }


@system_router.post("/api/estop")
async def emergency_stop(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Immediately assert E-stop and disable mission movement."""

    _require_ros_bridge()

    try:
        safety = await run_in_threadpool(ros_bridge.force_emergency_stop)
    except RuntimeError as error:
        raise HTTPException(
            status_code=(status.HTTP_503_SERVICE_UNAVAILABLE),
            detail=str(error),
        ) from error

    return {
        "success": True,
        "message": "Emergency stop asserted.",
        "safety": safety,
    }


@system_router.post("/api/estop/release")
async def release_emergency_stop(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Release E-stop without starting or resuming the mission."""

    _require_ros_bridge()

    try:
        safety = await run_in_threadpool(ros_bridge.release_emergency_stop)
    except RuntimeError as error:
        raise HTTPException(
            status_code=(status.HTTP_503_SERVICE_UNAVAILABLE),
            detail=str(error),
        ) from error

    return {
        "success": True,
        "message": ("Emergency stop released; " "mission movement remains disabled."),
        "safety": safety,
    }
