"""Authenticated Socket.IO realtime transport for the DYX 4WD rover.

The REST API remains the authoritative command interface. This module only
streams backend state to authenticated frontend clients.

Emitted events:

    telemetry
        High-rate live values (vehicle, position, GNSS/RTK, battery, RPP debug,
        RPP live radial accuracy, stream freshness, safety). Its `mission`
        member is a bounded lifecycle projection; packet size does not grow
        with mission length or completed-point history.

    mission_status
        Mission lifecycle, emitted when a non-volatile lifecycle field changes
        plus a low-rate heartbeat (DYX_MISSION_STATUS_HEARTBEAT_SEC). With
        DYX_SOCKET_MISSION_STATUS_COMPACT=1 it is the compact
        mission_lifecycle@1 contract (no point_results/report history);
        otherwise the full REST contract, for older tablet builds.

    mission_progress
        Compact mission progress payload emitted when progress changes.

    safety_state
        Safety-gate state emitted whenever it changes.

    point_completed
    point_skipped
    point_failed
    point_event
        Events derived from /mission_manager/point_event through ros_bridge.

    mission_completed
        Emitted once when the mission enters COMPLETED.

    auth_revoked
        Emitted before a socket is disconnected after logout, expiry or
        server-side session revocation.

Every Socket.IO connection must provide the same persistent authentication
token used by the protected REST API. The token may be supplied through the
Socket.IO auth payload, X-Rover-Token, or Authorization: Bearer.
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading

from dataclasses import dataclass
from typing import Any

import socketio

from socketio.exceptions import ConnectionRefusedError as SocketConnectionRefusedError

from rover_backend.auth import authentication_store
from rover_backend.config import client_ip_is_allowed
from rover_backend.config import settings
from rover_backend.realtime_contract import ChangeDrivenEmitter
from rover_backend.realtime_contract import RealtimeMetrics
from rover_backend.realtime_contract import build_mission_lifecycle_payload
from rover_backend.realtime_contract import mission_lifecycle_signature
from rover_backend.realtime_contract import timed
from rover_backend.state import rover_state
from rover_backend.state import utc_now_iso
from rover_backend.system_routes import build_mission_status_payload
from rover_backend.system_routes import build_telemetry_payload
from rover_backend.trajectory_push import trajectory_snapshot

LOGGER = logging.getLogger(__name__)


# An empty origin list disables Engine.IO's browser-origin check. Network
# admission and token authentication are still enforced below. React Native
# clients commonly connect without a browser Origin header.
_SOCKET_CORS_ORIGINS: list[str] = (
    list(settings.cors_origins) if settings.cors_origins else []
)


sio = socketio.AsyncServer(
    async_mode="asgi",
    cors_allowed_origins=_SOCKET_CORS_ORIGINS,
    logger=False,
    engineio_logger=False,
    ping_interval=10,
    ping_timeout=10,
    max_http_buffer_size=1_000_000,
)


@dataclass(frozen=True, slots=True)
class SocketSession:
    """Authenticated identity associated with one Socket.IO connection."""

    token: str
    session_id: str
    username: str
    client_ip: str
    connected_at: str


_socket_sessions: dict[str, SocketSession] = {}
_socket_lock = asyncio.Lock()

_broadcast_task: asyncio.Task[None] | None = None
_stop_event: asyncio.Event | None = None
_state_change_event: asyncio.Event | None = None
_event_loop: asyncio.AbstractEventLoop | None = None

_lifecycle_lock = asyncio.Lock()
_revocation_callback_registered = False

# DYX_REALTIME_METRICS=1 logs one rate/size/timing summary per second.
realtime_metrics = RealtimeMetrics(enabled=settings.realtime_metrics_enabled)


async def _emit(event: str, payload: Any, **kwargs: Any) -> None:
    """Broadcast emit that feeds the development counters."""

    realtime_metrics.record_emit(event, payload)
    await sio.emit(event, payload, **kwargs)


def _socket_mission_payload(mission: dict[str, Any]) -> dict[str, Any]:
    """mission_status as sent on Socket.IO (REST keeps the full contract)."""

    if settings.socket_mission_status_compact:
        return build_mission_lifecycle_payload(
            mission, rover_state.section("mission")
        )
    return mission


def notify_authoritative_state_changed() -> None:
    """Wake realtime delivery from ROS/REST threads after state transitions."""

    loop = _event_loop
    state_change_event = _state_change_event
    if loop is None or loop.is_closed() or state_change_event is None:
        return
    try:
        loop.call_soon_threadsafe(state_change_event.set)
    except RuntimeError:
        # The ASGI loop is shutting down.
        return


_trajectory_emit_tasks: set[asyncio.Task[Any]] = set()


def _on_trajectory_event(event: dict[str, Any]) -> None:
    """Snapshot listener (ROS thread): hand the event to the Socket.IO loop."""

    loop = _event_loop
    if loop is None or loop.is_closed():
        return
    try:
        loop.call_soon_threadsafe(_schedule_trajectory_emit, event)
    except RuntimeError:
        # The ASGI loop is shutting down.
        return


def _schedule_trajectory_emit(event: dict[str, Any]) -> None:
    task = asyncio.ensure_future(_emit_trajectory_event(event))
    _trajectory_emit_tasks.add(task)

    def _done(finished: asyncio.Task[Any]) -> None:
        _trajectory_emit_tasks.discard(finished)
        if not finished.cancelled() and finished.exception() is not None:
            LOGGER.error(
                "trajectory event emit failed: %s",
                finished.exception(),
            )

    task.add_done_callback(_done)


async def _emit_trajectory_event(event: dict[str, Any]) -> None:
    # A ROS event may have been invalidated while queued for the ASGI loop.
    if trajectory_snapshot.event_is_current(event, rover_state.section("mission")):
        await sio.emit(event["event"], event["payload"])


async def _replay_trajectory(sid: str) -> None:
    live = trajectory_snapshot.live_payload(rover_state.section("mission"))
    if live is not None:
        await sio.emit("trajectory_path", live, to=sid)


# ---------------------------------------------------------------------------
# Connection helpers
# ---------------------------------------------------------------------------


def _normalise_ip(value: Any) -> str | None:
    if value is None:
        return None

    result = str(value).strip()

    if not result:
        return None

    if result.startswith("::ffff:"):
        result = result[7:]

    return result


def _client_ip(environ: dict[str, Any]) -> str | None:
    """Extract the peer IP from python-socketio's ASGI environment."""

    scope = environ.get("asgi.scope")

    if isinstance(scope, dict):
        client = scope.get("client")

        if isinstance(client, (tuple, list)) and len(client) >= 1:
            value = _normalise_ip(client[0])

            if value:
                return value

    return _normalise_ip(environ.get("REMOTE_ADDR"))


def _extract_socket_token(
    environ: dict[str, Any],
    auth: Any,
) -> str | None:
    """Extract the persistent rover token from supported locations."""

    if isinstance(auth, dict):
        for key in (
            "token",
            "access_token",
            "rover_token",
        ):
            value = auth.get(key)

            if isinstance(value, str) and value.strip():
                return value.strip()

    header_token = environ.get("HTTP_X_ROVER_TOKEN")

    if isinstance(header_token, str) and header_token.strip():
        return header_token.strip()

    authorization = environ.get("HTTP_AUTHORIZATION")

    if isinstance(authorization, str):
        scheme, separator, value = authorization.strip().partition(" ")

        if separator and scheme.lower() == "bearer" and value.strip():
            return value.strip()

    return None


async def _update_frontend_connection_state() -> None:
    async with _socket_lock:
        connected_count = len(_socket_sessions)

    rover_state.update(
        "network",
        frontend_connected=(connected_count > 0),
        socket_clients=connected_count,
        last_client_seen_at=(
            utc_now_iso()
            if connected_count > 0
            else rover_state.section("network").get("last_client_seen_at")
        ),
    )


async def _socket_record(
    sid: str,
) -> SocketSession | None:
    async with _socket_lock:
        return _socket_sessions.get(sid)


async def _all_socket_records() -> list[tuple[str, SocketSession]]:
    async with _socket_lock:
        return list(_socket_sessions.items())


# ---------------------------------------------------------------------------
# Socket.IO events
# ---------------------------------------------------------------------------


@sio.event
async def connect(
    sid: str,
    environ: dict[str, Any],
    auth: Any,
) -> bool:
    """Authenticate and admit one Socket.IO frontend connection."""

    client_ip = _client_ip(environ)

    if not client_ip_is_allowed(client_ip):
        LOGGER.warning(
            "Rejected Socket.IO client outside allowed network: ip=%s",
            client_ip,
        )

        raise SocketConnectionRefusedError("Client network is not allowed")

    token = _extract_socket_token(
        environ,
        auth,
    )

    if token is None:
        raise SocketConnectionRefusedError("Authentication token is required")

    session = authentication_store.authenticate_token(
        token,
        touch=True,
    )

    if session is None:
        raise SocketConnectionRefusedError("Session is invalid, expired or logged out")

    record = SocketSession(
        token=token,
        session_id=session.session_id,
        username=session.username,
        client_ip=client_ip or "unknown",
        connected_at=utc_now_iso(),
    )

    async with _socket_lock:
        _socket_sessions[sid] = record

    await _update_frontend_connection_state()

    LOGGER.info(
        "Socket.IO connected: sid=%s session=%s user=%s ip=%s",
        sid,
        record.session_id,
        record.username,
        record.client_ip,
    )

    # Give a newly connected frontend a complete state immediately instead
    # of waiting for the next periodic broadcast.
    mission = build_mission_status_payload()
    telemetry = build_telemetry_payload(mission)
    safety = rover_state.section("safety")

    await sio.emit(
        "telemetry",
        telemetry,
        to=sid,
    )

    await sio.emit(
        "mission_status",
        _socket_mission_payload(mission),
        to=sid,
    )

    await sio.emit(
        "mission_progress",
        _mission_progress_payload(mission),
        to=sid,
    )

    await sio.emit(
        "safety_state",
        safety,
        to=sid,
    )

    # Only a complete, verified snapshot is replayed; while the backend is
    # still assembling (e.g. just restarted) nothing partial is sent.
    await _replay_trajectory(sid)

    await sio.emit(
        "socket_ready",
        {
            "authenticated": True,
            "session_id": record.session_id,
            "username": record.username,
            "rover_id": settings.rover_id,
            "rover_name": settings.rover_name,
            "connected_at": record.connected_at,
        },
        to=sid,
    )

    return True


@sio.event
async def disconnect(
    sid: str,
) -> None:
    """Remove the local connection record after a client disconnects."""

    async with _socket_lock:
        record = _socket_sessions.pop(
            sid,
            None,
        )

    await _update_frontend_connection_state()

    if record is not None:
        LOGGER.info(
            "Socket.IO disconnected: sid=%s session=%s",
            sid,
            record.session_id,
        )


@sio.event
async def client_ping(
    sid: str,
    _payload: Any = None,
) -> dict[str, Any]:
    """Authenticated application-level liveness acknowledgement."""

    record = await _socket_record(sid)

    if record is None:
        raise SocketConnectionRefusedError("Socket session is unavailable")

    if not (authentication_store.session_is_active(record.token)):
        await _disconnect_socket(
            sid,
            reason="session_revoked",
        )

        raise SocketConnectionRefusedError("Authentication session is no longer active")

    rover_state.update(
        "network",
        last_client_seen_at=utc_now_iso(),
    )

    return {
        "success": True,
        "server_time": utc_now_iso(),
        "rover_id": settings.rover_id,
    }


# ---------------------------------------------------------------------------
# Event payloads and change detection
# ---------------------------------------------------------------------------


def _stable_signature(
    value: Any,
) -> str:
    """Create a deterministic comparison value for JSON-compatible data."""

    try:
        return json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            default=str,
        )
    except (TypeError, ValueError):
        return repr(value)


def _mission_progress_payload(
    mission: dict[str, Any],
) -> dict[str, Any]:
    """Return the compact progress contract consumed by mission screens."""

    return {
        "mission_id": mission.get("mission_id"),
        "state": mission.get(
            "state",
            "EMPTY",
        ),
        "total_points": mission.get(
            "total_points",
            0,
        ),
        "completed_points": mission.get(
            "completed_points",
            0,
        ),
        "skipped_points": mission.get(
            "skipped_points",
            0,
        ),
        "failed_points": mission.get(
            "failed_points",
            0,
        ),
        "remaining_points": mission.get(
            "remaining_points",
            0,
        ),
        "progress_percent": mission.get(
            "progress_percent",
            0.0,
        ),
        "active_point_id": mission.get("active_point_id"),
        "active_point_index": mission.get("active_point_index"),
        "active_point_number": mission.get("active_point_number"),
        "active_point_state": mission.get("active_point_state"),
        "marking_active": mission.get(
            "marking_active",
            False,
        ),
        "pause_reason": mission.get("pause_reason"),
        "resume_available": mission.get(
            "resume_available",
            False,
        ),
        "gps_fix_type": mission.get("gps_fix_type", 0),
        "rtk_state": mission.get("rtk_state"),
        "rtk_fixed": mission.get("rtk_fixed", False),
        "rtk_healthy": mission.get("rtk_healthy", False),
        "rtk_motion_ok": mission.get("rtk_motion_ok", False),
        "rtk_reason": mission.get("rtk_reason"),
        "rtk_correction_age_sec": mission.get("rtk_correction_age_sec"),
        "backend_heartbeat_healthy": mission.get(
            "backend_heartbeat_healthy",
            False,
        ),
        "mission_enable": mission.get("mission_enable", False),
        "emergency_stop": mission.get("emergency_stop", True),
        "arrival_settle_elapsed_sec": mission.get(
            "arrival_settle_elapsed_sec",
            0.0,
        ),
        "arrival_settle_required_sec": mission.get(
            "arrival_settle_required_sec",
            settings.arrival_settle_seconds,
        ),
        "alignment_active": mission.get(
            "alignment_active",
            False,
        ),
        "hold_elapsed_sec": mission.get(
            "hold_elapsed_sec",
            0.0,
        ),
        "hold_required_sec": mission.get(
            "hold_required_sec",
            settings.marking_hold_seconds,
        ),
        "report": mission.get("report"),
        "terminal_cleanup_status": mission.get("terminal_cleanup_status"),
        "terminal_cleanup_error": mission.get("terminal_cleanup_error"),
        "updated_at": mission.get("updated_at"),
    }


def _point_event_name(
    point_event: dict[str, Any],
) -> str:
    event_name = str(point_event.get("event", "")).strip().upper()

    return {
        "COMPLETED": "point_completed",
        "SKIPPED": "point_skipped",
        "FAILED": "point_failed",
    }.get(
        event_name,
        "point_event",
    )


# ---------------------------------------------------------------------------
# Authentication revocation
# ---------------------------------------------------------------------------


async def _disconnect_socket(
    sid: str,
    *,
    reason: str,
) -> None:
    """Notify and disconnect one socket without raising into callers."""

    record = await _socket_record(sid)

    if record is None:
        return

    try:
        await sio.emit(
            "auth_revoked",
            {
                "reason": reason,
                "session_id": (record.session_id),
            },
            to=sid,
        )
    except Exception:
        LOGGER.exception(
            "Failed to emit auth_revoked to sid=%s",
            sid,
        )

    try:
        await sio.disconnect(sid)
    except Exception:
        LOGGER.exception(
            "Failed to disconnect sid=%s",
            sid,
        )


async def disconnect_session(
    session_id: str,
    *,
    reason: str = "logout",
) -> None:
    """Disconnect every active socket belonging to one auth session."""

    records = await _all_socket_records()

    matching_sids = [sid for sid, record in records if record.session_id == session_id]

    if matching_sids:
        await asyncio.gather(
            *(
                _disconnect_socket(
                    sid,
                    reason=reason,
                )
                for sid in matching_sids
            ),
            return_exceptions=True,
        )


def _schedule_revoked_session_disconnect(
    session_id: str,
) -> None:
    """Thread-safe callback registered with AuthenticationStore."""

    loop = _event_loop

    if loop is None or loop.is_closed():
        return

    def create_disconnect_task() -> None:
        asyncio.create_task(
            disconnect_session(
                session_id,
                reason="logout",
            ),
            name=("rover-auth-revocation-" f"{session_id[:8]}"),
        )

    try:
        loop.call_soon_threadsafe(create_disconnect_task)
    except RuntimeError:
        # The ASGI event loop is shutting down.
        return


# ---------------------------------------------------------------------------
# Periodic broadcast
# ---------------------------------------------------------------------------


async def _revalidate_socket_sessions() -> None:
    records = await _all_socket_records()

    revoked_sids = [
        sid
        for sid, record in records
        if not (authentication_store.session_is_active(record.token))
    ]

    if revoked_sids:
        await asyncio.gather(
            *(
                _disconnect_socket(
                    sid,
                    reason="session_revoked",
                )
                for sid in revoked_sids
            ),
            return_exceptions=True,
        )


async def _broadcast_loop() -> None:
    stop_event = _stop_event
    state_change_event = _state_change_event

    if stop_event is None or state_change_event is None:
        return

    frequency_hz = max(
        0.2,
        float(settings.telemetry_broadcast_hz),
    )

    interval_seconds = 1.0 / frequency_hz

    validation_interval_ticks = max(
        1,
        int(round(frequency_hz * 5.0)),
    )

    validation_tick = 0

    mission_status_emitter = ChangeDrivenEmitter(
        heartbeat_sec=float(settings.mission_status_heartbeat_sec),
    )
    previous_progress_signature: str | None = None
    previous_safety_signature: str | None = None
    previous_point_event_signature: str | None = None
    previous_mission_state: str | None = None
    next_deadline = asyncio.get_running_loop().time()
    periodic_iteration = True

    while not stop_event.is_set():
        try:
            # Check even without path polling or connected clients. Keep the
            # snapshot lock (which can be held by projection) off the ASGI loop.
            await asyncio.to_thread(trajectory_snapshot.check_timeout)
            records = await _all_socket_records()

            if not records:
                # A client that connects later gets a fresh snapshot on
                # connect; do not let a stale signature suppress it.
                mission_status_emitter.reset()

            if records:
                with timed(realtime_metrics, "build_mission_status_payload"):
                    mission = build_mission_status_payload()

                with timed(realtime_metrics, "build_telemetry_payload"):
                    telemetry = build_telemetry_payload(mission)

                safety = rover_state.section("safety")

                if realtime_metrics.enabled:
                    point_results = mission.get("point_results")
                    realtime_metrics.set_gauge(
                        "point_results_count",
                        len(point_results) if isinstance(point_results, dict) else 0,
                    )
                    realtime_metrics.set_gauge(
                        "total_points", mission.get("total_points", 0)
                    )
                    realtime_metrics.set_gauge("socket_clients", len(records))

                # A. High-rate telemetry: live values, bounded size.
                await _emit(
                    "telemetry",
                    telemetry,
                )

                # B. Mission lifecycle: change-driven plus low-rate heartbeat.
                # Evaluated on every iteration, including state-change
                # wake-ups, so a safety change is never delayed to a heartbeat.
                with timed(realtime_metrics, "mission_lifecycle_signature"):
                    mission_signature = mission_lifecycle_signature(
                        mission, rover_state.section("mission")
                    )

                if mission_status_emitter.should_emit(mission_signature):
                    await _emit(
                        "mission_status",
                        _socket_mission_payload(mission),
                    )

                progress = _mission_progress_payload(mission)

                progress_signature = _stable_signature(progress)

                if progress_signature != previous_progress_signature:
                    previous_progress_signature = progress_signature

                    await _emit(
                        "mission_progress",
                        progress,
                    )

                safety_signature = _stable_signature(safety)

                if safety_signature != previous_safety_signature:
                    previous_safety_signature = safety_signature

                    await _emit(
                        "safety_state",
                        safety,
                    )

                point_event = mission.get("last_point_event")

                if isinstance(
                    point_event,
                    dict,
                ):
                    point_event_signature = _stable_signature(point_event)

                    if point_event_signature != previous_point_event_signature:
                        previous_point_event_signature = point_event_signature

                        await _emit(
                            _point_event_name(point_event),
                            point_event,
                        )

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

                if mission_state != previous_mission_state:
                    previous_mission_state = mission_state

                    await _emit(
                        "mission_state",
                        _socket_mission_payload(mission),
                    )

                    if mission_state == "COMPLETED":
                        await _emit(
                            "mission_completed",
                            _socket_mission_payload(mission),
                        )

            realtime_metrics.maybe_flush()

            validation_tick += 1

            if validation_tick >= validation_interval_ticks:
                validation_tick = 0
                await _revalidate_socket_sessions()

        except asyncio.CancelledError:
            raise

        except Exception:
            LOGGER.exception("Realtime broadcast iteration failed")

        if periodic_iteration:
            next_deadline += interval_seconds
        now = asyncio.get_running_loop().time()
        if next_deadline <= now:
            missed_intervals = int((now - next_deadline) / interval_seconds) + 1
            next_deadline += missed_intervals * interval_seconds
        try:
            await asyncio.wait_for(
                state_change_event.wait(),
                timeout=max(0.0, next_deadline - now),
            )
            state_change_event.clear()
            periodic_iteration = False
        except asyncio.TimeoutError:
            periodic_iteration = True
            continue


# ---------------------------------------------------------------------------
# ASGI lifecycle
# ---------------------------------------------------------------------------


async def start_realtime() -> None:
    """Start the singleton realtime broadcaster inside the ASGI event loop."""

    global _broadcast_task
    global _event_loop
    global _revocation_callback_registered
    global _state_change_event
    global _stop_event

    async with _lifecycle_lock:
        if _broadcast_task is not None and not _broadcast_task.done():
            return

        _event_loop = asyncio.get_running_loop()
        trajectory_snapshot.add_listener(_on_trajectory_event)

        _stop_event = asyncio.Event()
        _state_change_event = asyncio.Event()

        if not _revocation_callback_registered:
            authentication_store.register_revocation_callback(
                _schedule_revoked_session_disconnect
            )

            _revocation_callback_registered = True

        _broadcast_task = asyncio.create_task(
            _broadcast_loop(),
            name="rover-realtime-broadcast",
        )

        LOGGER.info(
            "Realtime broadcaster started at %.2f Hz",
            settings.telemetry_broadcast_hz,
        )


async def stop_realtime() -> None:
    """Stop broadcasts and disconnect all authenticated clients."""

    global _broadcast_task
    global _event_loop
    global _state_change_event
    global _stop_event

    async with _lifecycle_lock:
        task = _broadcast_task
        stop_event = _stop_event

        if stop_event is not None:
            stop_event.set()
        if _state_change_event is not None:
            _state_change_event.set()

        if task is not None:
            try:
                await asyncio.wait_for(
                    task,
                    timeout=3.0,
                )
            except asyncio.TimeoutError:
                task.cancel()

                await asyncio.gather(
                    task,
                    return_exceptions=True,
                )

        records = await _all_socket_records()

        if records:
            await asyncio.gather(
                *(sio.disconnect(sid) for sid, _record in records),
                return_exceptions=True,
            )

        async with _socket_lock:
            _socket_sessions.clear()

        rover_state.update(
            "network",
            frontend_connected=False,
            socket_clients=0,
        )

        trajectory_snapshot.remove_listener(_on_trajectory_event)

        _broadcast_task = None
        _stop_event = None
        _state_change_event = None
        _event_loop = None

        LOGGER.info("Realtime broadcaster stopped")


def make_asgi_app(
    fastapi_app: Any,
) -> socketio.ASGIApp:
    """Mount Socket.IO and the FastAPI application in one ASGI app."""

    return socketio.ASGIApp(
        sio,
        other_asgi_app=fastapi_app,
        socketio_path=(settings.socket_path.strip("/")),
    )
