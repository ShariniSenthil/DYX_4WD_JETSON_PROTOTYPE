"""ROS-free Socket.IO transport contract helpers for the DYX 4WD backend.

Everything here is pure: no ROS, no Socket.IO server, no FastAPI. That keeps
the realtime contract directly unit-testable on a workstation.

Data authority is not changed by anything in this module. It only decides
*what is sent when*; every accuracy number is copied from RPP / Mission
Manager unchanged.
"""

from __future__ import annotations

import json
import logging
import time

from dataclasses import dataclass
from dataclasses import field
from typing import Any
from typing import Callable

LOGGER = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Development instrumentation
# ---------------------------------------------------------------------------


@dataclass(slots=True)
class _EventCounter:
    events: int = 0
    total_bytes: int = 0
    max_bytes: int = 0

    def record(self, size_bytes: int | None) -> None:
        self.events += 1
        if size_bytes is not None:
            self.total_bytes += size_bytes
            self.max_bytes = max(self.max_bytes, size_bytes)


@dataclass(slots=True)
class _TimingCounter:
    calls: int = 0
    total_ms: float = 0.0
    max_ms: float = 0.0

    def record(self, elapsed_ms: float) -> None:
        self.calls += 1
        self.total_ms += elapsed_ms
        self.max_ms = max(self.max_ms, elapsed_ms)


def payload_size_bytes(payload: Any) -> int:
    """Approximate Socket.IO JSON size. Only called when metrics are on."""

    try:
        return len(
            json.dumps(
                payload,
                ensure_ascii=False,
                separators=(",", ":"),
                default=str,
            ).encode("utf-8")
        )
    except (TypeError, ValueError):
        return -1


@dataclass(slots=True)
class RealtimeMetrics:
    """Once-per-window counters for the realtime broadcaster.

    Disabled by default. When disabled every method is a cheap no-op, so the
    hot broadcast path pays nothing in production. Nothing is logged per
    event; one summary line is emitted per window.
    """

    enabled: bool = False
    window_sec: float = 1.0
    clock: Callable[[], float] = time.monotonic
    sink: Callable[[dict[str, Any]], None] | None = None

    _events: dict[str, _EventCounter] = field(default_factory=dict)
    _timings: dict[str, _TimingCounter] = field(default_factory=dict)
    _gauges: dict[str, Any] = field(default_factory=dict)
    _window_started: float | None = None
    last_summary: dict[str, Any] | None = None

    def record_emit(self, event: str, payload: Any) -> None:
        if not self.enabled:
            return
        self._events.setdefault(event, _EventCounter()).record(
            payload_size_bytes(payload)
        )

    def record_timing(self, name: str, elapsed_ms: float) -> None:
        if not self.enabled:
            return
        self._timings.setdefault(name, _TimingCounter()).record(elapsed_ms)

    def set_gauge(self, name: str, value: Any) -> None:
        if not self.enabled:
            return
        self._gauges[name] = value

    def maybe_flush(self) -> dict[str, Any] | None:
        """Emit and reset the window summary once `window_sec` has elapsed."""

        if not self.enabled:
            return None
        now = self.clock()
        if self._window_started is None:
            self._window_started = now
            return None
        elapsed = now - self._window_started
        if elapsed < self.window_sec:
            return None

        summary: dict[str, Any] = {
            "window_sec": round(elapsed, 3),
            "events": {
                name: {
                    "per_sec": round(counter.events / elapsed, 2),
                    "avg_bytes": (
                        round(counter.total_bytes / counter.events)
                        if counter.events
                        else 0
                    ),
                    "max_bytes": counter.max_bytes,
                    "bytes_per_sec": round(counter.total_bytes / elapsed),
                }
                for name, counter in sorted(self._events.items())
            },
            "timings_ms": {
                name: {
                    "calls": counter.calls,
                    "avg": (
                        round(counter.total_ms / counter.calls, 3)
                        if counter.calls
                        else 0.0
                    ),
                    "max": round(counter.max_ms, 3),
                }
                for name, counter in sorted(self._timings.items())
            },
            "gauges": dict(self._gauges),
        }
        self._events.clear()
        self._timings.clear()
        self._window_started = now
        self.last_summary = summary
        if self.sink is not None:
            self.sink(summary)
        else:
            LOGGER.info(
                "realtime metrics %s",
                json.dumps(summary, separators=(",", ":"), default=str),
            )
        return summary


class timed:
    """`with timed(metrics, "name"):` -- records elapsed ms when enabled."""

    __slots__ = ("_metrics", "_name", "_started")

    def __init__(self, metrics: RealtimeMetrics, name: str) -> None:
        self._metrics = metrics
        self._name = name
        self._started = 0.0

    def __enter__(self) -> "timed":
        if self._metrics.enabled:
            self._started = time.perf_counter()
        return self

    def __exit__(self, *_exc: Any) -> None:
        if self._metrics.enabled:
            self._metrics.record_timing(
                self._name,
                (time.perf_counter() - self._started) * 1000.0,
            )


# ---------------------------------------------------------------------------
# A. High-rate telemetry: bounded mission projection
# ---------------------------------------------------------------------------

# Exactly the mission fields the frontend telemetry adapter reads from
# telemetry.mission (px4TelemetryAdapter.toRoverTelemetry). Everything here is
# scalar, so telemetry size is independent of mission length and history.
TELEMETRY_MISSION_FIELDS: tuple[str, ...] = (
    "mission_id",
    "mission_run_id",
    "state",
    "loaded",
    "ready",
    "total_points",
    "navigation_point_count",
    "active_point_id",
    "active_point_index",
    "active_point_number",
    "active_point_state",
    "completed_points",
    "skipped_points",
    "failed_points",
    "remaining_points",
    "progress_percent",
    "marking_active",
)


def telemetry_mission_projection(mission_status: dict[str, Any]) -> dict[str, Any]:
    """Lifecycle subset embedded in high-rate telemetry. No point history."""

    return {key: mission_status.get(key) for key in TELEMETRY_MISSION_FIELDS}


# ---------------------------------------------------------------------------
# B. Mission lifecycle: compact, change-driven
# ---------------------------------------------------------------------------

MISSION_LIFECYCLE_FIELDS: tuple[str, ...] = (
    "mission_id",
    "mission_run_id",
    "filename",
    "state",
    "state_lower",
    "loaded",
    "ready",
    "accepted_for_start",
    "staged",
    "staged_id",
    "trajectory_ready",
    "total_points",
    "active_point_id",
    "active_point_index",
    "active_point_number",
    "active_point_state",
    "completed_points",
    "failed_points",
    "skipped_points",
    "remaining_points",
    "progress_percent",
    "pause_reason",
    "resume_available",
    "mission_enable",
    "emergency_stop",
    "gps_fix_type",
    "rtk_state",
    "rtk_fixed",
    "rtk_motion_ok",
    "rtk_reason",
    "px4_connected",
    "px4_mode",
    "px4_armed",
    "start_stage",
    "resume_stage",
    "stop_stage",
    "start_failed_stage",
    "alignment_active",
    "marking_active",
    "spray_controller_ready",
    "spray_controller_state",
    "spray_fault_reason",
    "current_point_spray_confirmed",
    "terminal_cleanup_status",
    "terminal_cleanup_error",
    # Per-point state strings (one short string per point). Bounded by the
    # mission's point count, not by history; the frontend reconciles missed
    # point events from it. Sent only when it changes.
    "point_status",
    "message",
    "error",
    "updated_at",
)

# Fields of the rover_state mission section that the REST mission-status
# contract does not carry but the lifecycle contract needs.
MISSION_SECTION_LIFECYCLE_FIELDS: tuple[str, ...] = (
    "execution_mode",
    "safety_generation",
)

# Continuously varying diagnostics. They never make a lifecycle packet
# "changed"; their latest values ride along on the next change or heartbeat.
VOLATILE_MISSION_FIELDS: frozenset[str] = frozenset(
    {
        "updated_at",
        "rtk_correction_age_sec",
        "gps_fix_status_age_sec",
        "rtk_health_status_age_sec",
        "rtk_age_status_age_sec",
        "arrival_settle_elapsed_sec",
        "hold_elapsed_sec",
        "survey_truth_gnss_samples",
    }
)

# Heavy/unbounded fields replaced in the signature by a cheap change proxy.
_HEAVY_MISSION_FIELDS: frozenset[str] = frozenset(
    {"point_results", "last_point_event", "report", "active_waypoint"}
)


def build_mission_lifecycle_payload(
    mission_status: dict[str, Any],
    mission_section: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Compact socket mission_status: lifecycle only, no growing history."""

    payload = {key: mission_status.get(key) for key in MISSION_LIFECYCLE_FIELDS}
    section = mission_section or {}
    for key in MISSION_SECTION_LIFECYCLE_FIELDS:
        payload[key] = section.get(key)
    payload["contract"] = "mission_lifecycle@1"
    return payload


def _last_point_event_identity(event: Any) -> list[Any] | None:
    if not isinstance(event, dict):
        return None
    return [
        event.get("mission_run_id"),
        event.get("point_id"),
        event.get("event"),
        event.get("received_at"),
    ]


def mission_lifecycle_signature(
    mission_status: dict[str, Any],
    mission_section: dict[str, Any] | None = None,
) -> str:
    """Deterministic change detector for mission_status emission.

    Cost is O(point count) for point_status only; point_results is never
    serialized. point_results only changes in the point-event callback, which
    always replaces last_point_event with a fresh received_at, so that event's
    identity plus the result count is a complete proxy.
    """

    comparable: dict[str, Any] = {
        key: value
        for key, value in mission_status.items()
        if key not in VOLATILE_MISSION_FIELDS and key not in _HEAVY_MISSION_FIELDS
    }
    point_results = mission_status.get("point_results")
    comparable["_point_results_count"] = (
        len(point_results) if isinstance(point_results, dict) else 0
    )
    comparable["_last_point_event"] = _last_point_event_identity(
        mission_status.get("last_point_event")
    )
    report = mission_status.get("report")
    if isinstance(report, dict):
        comparable["_report"] = {
            key: report.get(key)
            for key in ("available", "terminal_available", "status", "mission_id",
                        "cleanup_complete", "error", "generated_at")
        }
    section = mission_section or {}
    for key in MISSION_SECTION_LIFECYCLE_FIELDS:
        comparable[key] = section.get(key)
    return json.dumps(
        comparable,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        default=str,
    )


@dataclass(slots=True)
class ChangeDrivenEmitter:
    """Emit on signature change, or on a low-rate heartbeat.

    A change is emitted on the same broadcast iteration that observes it;
    the heartbeat only bounds how long an unchanged packet can go unsent.
    """

    heartbeat_sec: float
    clock: Callable[[], float] = time.monotonic
    _signature: str | None = None
    _last_emit: float | None = None

    def should_emit(self, signature: str) -> bool:
        now = self.clock()
        changed = signature != self._signature
        due = self._last_emit is None or (
            self.heartbeat_sec > 0 and now - self._last_emit >= self.heartbeat_sec
        )
        if changed or due:
            self._signature = signature
            self._last_emit = now
            return True
        return False

    def reset(self) -> None:
        self._signature = None
        self._last_emit = None


# ---------------------------------------------------------------------------
# C. Point result events
# ---------------------------------------------------------------------------

_POINT_EVENT_SOCKET_NAMES = {
    "COMPLETED": "point_completed",
    "SKIPPED": "point_skipped",
    "FAILED": "point_failed",
}


def point_event_socket_name(point_event: dict[str, Any]) -> str:
    event_name = str(point_event.get("event", "")).strip().upper()
    return _POINT_EVENT_SOCKET_NAMES.get(event_name, "point_event")


def build_point_result_event(
    point_event: dict[str, Any],
    point_result: dict[str, Any] | None,
    mission_id: Any,
) -> dict[str, Any]:
    """Socket payload for one Mission Manager point event.

    The original event fields are preserved (existing consumers read them).
    `point_result` is the exact immutable result the backend just stored in
    point_results[point_id] -- the same object the canonical report is built
    from -- minus its per-point event history. Accuracy is copied, never
    recomputed.
    """

    payload = dict(point_event)
    payload.setdefault("mission_id", mission_id)
    if isinstance(point_result, dict):
        result = {
            key: value
            for key, value in point_result.items()
            if key != "event_history"
        }
        result.setdefault("mission_id", mission_id)
        payload["point_result"] = result
    else:
        payload["point_result"] = None
    payload["contract"] = "point_result@1"
    return payload
