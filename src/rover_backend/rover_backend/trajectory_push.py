"""Verified snapshot of the generator's published path, pushed to the frontend.

Transport only. The trajectory generator remains the single path builder; this
module receives the same retained topics RPP and mission_manager receive, holds
them as a pending snapshot, and emits ONE event only when the components are
complete and consistent:

  * navigation path, markings, point types, marking indices and signature present
  * SHA-256 recomputed over them equals the received signature topic
  * that signature equals the READY status's ``path_signature``
  * status is READY with a matching ``navigation_point_count``
  * the PX4 geographic origin is known (lat/lon need it)

Retained topics arrive in any cross-topic order, so "latest path + latest READY"
is never combined; only a verified match is emitted. Pure Python (no ROS, no
Socket.IO) so it is unit-testable; ``ros_bridge`` feeds it and ``realtime``
forwards its events.
"""

from __future__ import annotations

import logging
import math
import threading
import time
import uuid
from typing import Any, Callable, Sequence

from mission_manager.path_contract import PendingPreparedPath
from mission_manager.path_contract import resolve_path_signature
from rover_backend.config import _read_bool

LOGGER = logging.getLogger(__name__)

SCHEMA_VERSION = 1
EVENT_PATH = "trajectory_path"
EVENT_CLEARED = "trajectory_path_cleared"

# Display encoding: 0.1 mm for local metres, 1e-9 deg (~0.1 mm) for lat/lon.
XY_DECIMALS = 4
LATLON_DECIMALS = 9

# Reasons a snapshot cannot become valid again without a new preparation. They
# are reported as "invalid / re-prepare required"; every other reason is
# ordinary lifecycle or a still-assembling snapshot ("pending").
HARD_INVALID_REASONS = frozenset(
    {
        "prepared_origin_unavailable",
        "prepared_origin_invalid",
        "prepared_origin_changed",
        "coordinate_mode_unavailable",
        "snapshot_projection_failed",
        "snapshot_assembly_timeout",
    }
)

# A READY generator whose snapshot has not verified within this time is no
# longer "loading": it is reported invalid so the operator can re-prepare.
ASSEMBLY_TIMEOUT_S = 10.0

Point = tuple[float, float]
Projector = Callable[[float, float, float, float], tuple[float, float]]
Listener = Callable[[dict[str, Any]], None]


def push_enabled() -> bool:
    """Rollback switch: DYX_TRAJECTORY_PUSH=0 disables all snapshot work."""

    return _read_bool("DYX_TRAJECTORY_PUSH", True)


class TrajectorySnapshotService:
    """Assemble, verify and sequence the frontend trajectory events."""

    def __init__(self, projector: Projector | None = None) -> None:
        self._lock = threading.RLock()
        self._listeners: list[Listener] = []
        self._projector = projector
        self.server_instance_id = uuid.uuid4().hex

        self._pending = PendingPreparedPath()
        self._status: dict[str, Any] | None = None
        self._origin: tuple[float, float] | None = None
        self._local_origin: tuple[float, float] | None = None
        self._nav_stamp: tuple[int, int] | None = None
        self._blocked_nav_stamp: tuple[int, int] | None = None
        self._suspended = False
        self._invalid_reason: str | None = None
        self._assembling_since: float | None = None
        self._clock: Callable[[], float] = time.monotonic
        self.assembly_timeout_s = ASSEMBLY_TIMEOUT_S

        self._seq = 0
        self._components_version = 0
        self._verified: tuple[int, str] | None = None
        self._live: dict[str, Any] | None = None
        self._live_key: tuple[Any, ...] | None = None
        self._last_clear_key: tuple[Any, ...] | None = None

    # ------------------------------------------------------------------ setup
    def set_projector(self, projector: Projector) -> None:
        self._projector = projector

    def add_listener(self, listener: Listener) -> None:
        with self._lock:
            if listener not in self._listeners:
                self._listeners.append(listener)

    def remove_listener(self, listener: Listener) -> None:
        with self._lock:
            if listener in self._listeners:
                self._listeners.remove(listener)

    def reset(self) -> None:
        """Forget everything (tests / explicit local reset). Emits no event."""

        with self._lock:
            self._pending = PendingPreparedPath()
            self._status = None
            self._origin = None
            self._local_origin = None
            self._nav_stamp = None
            self._blocked_nav_stamp = None
            self._suspended = False
            self._invalid_reason = None
            self._assembling_since = None
            self._components_version += 1
            self._verified = None
            self._live = None
            self._live_key = None
            self._last_clear_key = None

    # ------------------------------------------------------------- read access
    def live_payload(self, mission: dict[str, Any] | None = None) -> dict[str, Any] | None:
        """The last emitted, still-valid ``trajectory_path`` payload, or None."""

        if not push_enabled():
            return None
        with self._lock:
            if mission is not None and (
                mission.get("loaded") is not True
                or mission.get("trajectory_ready") is not True
                or self._live is None
                or mission.get("mission_id") != self._live.get("mission_id")
            ):
                return None
            return self._live

    def invalidate(self, reason: str, *, suspend: bool = True) -> None:
        """Revoke display state before a local prepare, replacement or clear.

        Suspension lasts through service failure. A successful prepare resumes
        assembly, but cannot reuse the previous stamped navigation message.
        """
        with self._lock:
            if self._nav_stamp is not None:
                self._blocked_nav_stamp = self._nav_stamp
            self._nav_stamp = None
            self._pending = PendingPreparedPath()
            self._status = None
            self._local_origin = None
            self._verified = None
            self._components_version += 1
            self._suspended = suspend
            self._invalid_reason = None
            self._assembling_since = None
            self._last_clear_key = None
            events = self._clear_locked(reason)
        self._dispatch(events)

    def resume(self) -> None:
        if not push_enabled():
            return
        with self._lock:
            self._suspended = False
            events = self._evaluate_locked()
        self._dispatch(events)

    def event_is_current(self, event: dict[str, Any], mission: dict[str, Any]) -> bool:
        """Recheck events queued onto the ASGI loop after invalidation."""
        if not push_enabled():
            return False
        with self._lock:
            payload = event["payload"]
            if payload.get("seq") != self._seq:
                return False
            if event["event"] == EVENT_CLEARED:
                return self._live is None
            return self.live_payload(mission) is payload

    def display_state(self, mission: dict[str, Any] | None = None) -> dict[str, Any]:
        """What a client without a live path should be told.

        ``live``       a verified path exists (see ``live_payload``)
        ``assembling`` READY/preparing but the snapshot is not verified yet
        ``invalid``    it cannot become valid without a new preparation
        ``idle``       nothing expected (no mission / generator not ready)
        ``disabled``   push turned off; callers use the legacy preview
        """

        def state(name: str, reason: str | None = None) -> dict[str, Any]:
            return {
                "state": name,
                "reason": reason,
                "requires_reprepare": name == "invalid",
                # Lets a client order a delayed poll against events it applied.
                "server_instance_id": self.server_instance_id,
                "seq": self._seq,
            }

        if not push_enabled():
            return state("disabled")
        self.check_timeout()
        with self._lock:
            if self.live_payload(mission) is not None:
                return state("live")
            if self._suspended:
                return state("assembling", "preparation_invalidated")
            if self._invalid_reason:
                return state("invalid", self._invalid_reason)
            if self._assembling_since is not None:
                return state("assembling", "snapshot_pending")
            return state("idle")

    def check_timeout(self) -> None:
        """Advance overdue assembly even when no client is polling the path."""
        if not push_enabled():
            return
        # Projection may hold this lock for a large path. A periodic check can
        # wait for the next tick; it must not delay telemetry delivery.
        if not self._lock.acquire(blocking=False):
            return
        try:
            events = []
            if (
                not self._suspended
                and self._live is None
                and self._invalid_reason is None
                and self._assembling_since is not None
                and self._clock() - self._assembling_since > self.assembly_timeout_s
            ):
                events = self._reject_locked("snapshot_assembly_timeout")
        finally:
            self._lock.release()
        self._dispatch(events)

    # ------------------------------------------------------------------ feeds
    def feed_nav(
        self, points: Sequence[Point] | None, stamp: tuple[int, int] | None = None
    ) -> None:
        self._feed_component("navigation_path", points, stamp=stamp)

    def feed_waypoints(self, points: Sequence[Point] | None) -> None:
        self._feed_component("mission_waypoints", points)

    def feed_types(self, values: Sequence[int] | None) -> None:
        self._feed_component("point_types", values)

    def feed_indices(self, values: Sequence[int] | None) -> None:
        self._feed_component("marking_indices", values)

    def feed_signature(self, signature: str | None) -> None:
        self._feed_component("path_signature", signature or None)

    def feed_status(self, status: dict[str, Any] | None) -> None:
        if not push_enabled():
            return
        with self._lock:
            self._status = dict(status) if isinstance(status, dict) else None
            if not self._status or self._status.get("state") != "READY":
                self._local_origin = None
            events = self._evaluate_locked()
        self._dispatch(events)

    def feed_origin(self, latitude_deg: float, longitude_deg: float) -> None:
        if not push_enabled():
            return
        with self._lock:
            self._origin = (float(latitude_deg), float(longitude_deg))
            events = self._evaluate_locked()
        self._dispatch(events)

    # --------------------------------------------------------------- internals
    def _feed_component(
        self, source: str, value: Any, *, stamp: tuple[int, int] | None = None
    ) -> None:
        if not push_enabled():
            return
        with self._lock:
            if source == "navigation_path":
                if self._blocked_nav_stamp is not None:
                    # Reject only the exact stale retained message (or one with
                    # no stamp to prove otherwise). Do NOT order by time: a
                    # clock step backwards must not block every later path.
                    if stamp is None or stamp == self._blocked_nav_stamp:
                        return
                    self._blocked_nav_stamp = None
                self._nav_stamp = stamp
            empty = value is None or (
                not isinstance(value, str) and len(value) == 0
            )
            if empty:
                self._pending.clear_source(source)
            else:
                stored = value if isinstance(value, str) else list(value)
                setattr(self._pending, source, stored)
            self._components_version += 1
            events = self._evaluate_locked(source_cleared=empty)
        self._dispatch(events)

    def _dispatch(self, events: list[dict[str, Any]]) -> None:
        if not events:
            return
        with self._lock:
            listeners = list(self._listeners)
        for event in events:
            for listener in listeners:
                try:
                    listener(event)
                except Exception:  # noqa: BLE001 - never break the ROS callback
                    LOGGER.exception("trajectory event listener failed")

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def _clear_locked(self, reason: str, *, force: bool = False) -> list[dict[str, Any]]:
        live = self._live
        self._live = None
        self._live_key = None
        status = self._status or {}
        mission_id = live.get("mission_id") if live else status.get("mission_id")
        signature = live.get("signature") if live else status.get("path_signature")
        key = (mission_id, signature, reason)
        if live is None and (not force or key == self._last_clear_key):
            return []
        self._last_clear_key = key
        return [
            {
                "event": EVENT_CLEARED,
                "payload": {
                    "schema_version": SCHEMA_VERSION,
                    "server_instance_id": self.server_instance_id,
                    "seq": self._next_seq(),
                    "mission_id": mission_id,
                    "signature": signature,
                    "reason": reason,
                    "requires_reprepare": reason in HARD_INVALID_REASONS,
                },
            }
        ]

    def _reject_locked(self, reason: str) -> list[dict[str, Any]]:
        """Clear and remember that this snapshot needs a new preparation."""

        self._invalid_reason = reason
        if reason != "snapshot_assembly_timeout":
            self._assembling_since = None
        # A soft clear may already have removed _live. Clients still need a
        # newer event when that pending state becomes invalid.
        return self._clear_locked(reason, force=True)

    def _pending_locked(self, reason: str | None = None) -> list[dict[str, Any]]:
        """Not verifiable yet: assembling (bounded by ``assembly_timeout_s``)."""

        if self._assembling_since is None:
            self._assembling_since = self._clock()
        if self._clock() - self._assembling_since > self.assembly_timeout_s:
            return self._reject_locked("snapshot_assembly_timeout")
        return self._clear_locked(reason) if reason else []

    def _evaluate_locked(self, *, source_cleared: bool = False) -> list[dict[str, Any]]:
        if self._suspended:
            self._assembling_since = None
            return self._clear_locked("preparation_invalidated")
        status = self._status
        ready = bool(
            status
            and str(status.get("state", "")).strip().upper() == "READY"
            and status.get("ready")
        )
        if not ready:
            self._invalid_reason = None
            self._assembling_since = None
            return self._clear_locked("generator_not_ready")
        # Re-derived on every evaluation, so a condition that heals (e.g. the
        # origin returns to the prepared one) stops being reported invalid.
        self._invalid_reason = None
        if source_cleared:
            return self._pending_locked("source_cleared")

        # A geometry signature covers local coordinates, not their geographic
        # origin. Never reproject a GPS path using a newer, unrelated origin.
        if status.get("coordinate_mode") == "gps":
            localization = status.get("localization")
            if not isinstance(localization, dict):
                return self._reject_locked("prepared_origin_unavailable")
            try:
                prepared_origin = (
                    float(localization["origin_latitude_deg"]),
                    float(localization["origin_longitude_deg"]),
                )
            except (KeyError, TypeError, ValueError):
                return self._reject_locked("prepared_origin_unavailable")
            if not all(math.isfinite(v) for v in prepared_origin):
                return self._reject_locked("prepared_origin_invalid")
            if self._origin is None:
                # The backend has not received gp_origin yet (e.g. just
                # restarted): pending, not a mismatch.
                return self._pending_locked()
            if prepared_origin != self._origin:
                return self._reject_locked("prepared_origin_changed")
        elif status.get("coordinate_mode") == "local":
            if self._local_origin is not None and self._origin != self._local_origin:
                return self._reject_locked("prepared_origin_changed")
        else:
            return self._reject_locked("coordinate_mode_unavailable")

        pending = self._pending
        nav = pending.navigation_path
        markings = pending.mission_waypoints
        types = pending.point_types
        indices = pending.marking_indices
        signature = pending.path_signature
        if any(v is None for v in (nav, markings, types, indices, signature)):
            return self._pending_locked("snapshot_incomplete")
        if self._origin is None:
            return self._pending_locked()

        count = len(nav)
        if not (count == len(types) == len(indices)):
            return self._pending_locked("snapshot_count_mismatch")
        try:
            status_count = int(status.get("navigation_point_count"))
        except (TypeError, ValueError):
            return self._pending_locked("status_count_invalid")
        if status_count != count or status.get("path_signature") != signature:
            return self._pending_locked("status_snapshot_mismatch")

        verified = (self._components_version, signature)
        if self._verified != verified:
            decision = resolve_path_signature(
                nav, markings, types, indices, signature, signature_required=True
            )
            if not decision.can_install:
                LOGGER.warning("trajectory snapshot rejected: %s", decision.reason)
                return self._pending_locked("snapshot_signature_mismatch")
            self._verified = verified

        key = (status.get("mission_id"), signature, self._origin)
        if self._live is not None and self._live_key == key:
            return []

        payload = self._build_payload_locked(status, nav, signature)
        if payload is None:
            return self._reject_locked("snapshot_projection_failed")
        self._live = payload
        self._live_key = key
        self._last_clear_key = None
        self._assembling_since = None
        if status.get("coordinate_mode") == "local":
            self._local_origin = self._origin
        return [{"event": EVENT_PATH, "payload": payload}]

    def _build_payload_locked(
        self,
        status: dict[str, Any],
        nav: Sequence[Point],
        signature: str,
    ) -> dict[str, Any] | None:
        projector = self._projector
        origin = self._origin
        if projector is None or origin is None:
            return None
        lat0, lon0 = origin

        x: list[float] = []
        y: list[float] = []
        lat: list[float] = []
        lon: list[float] = []
        try:
            for east, north in nav:
                point_lat, point_lon = projector(lat0, lon0, east, north)
                x.append(round(east, XY_DECIMALS))
                y.append(round(north, XY_DECIMALS))
                lat.append(round(point_lat, LATLON_DECIMALS))
                lon.append(round(point_lon, LATLON_DECIMALS))
        except (ValueError, TypeError, OverflowError) as error:
            LOGGER.warning("trajectory snapshot projection failed: %s", error)
            return None

        return {
            "schema_version": SCHEMA_VERSION,
            "server_instance_id": self.server_instance_id,
            "seq": self._next_seq(),
            "mission_id": status.get("mission_id"),
            "mission_checksum": status.get("mission_checksum"),
            "signature": signature,
            "count": len(x),
            "frame": "ENU",
            "origin": {"lat": lat0, "lon": lon0},
            "encoding": {"xy_decimals": XY_DECIMALS, "latlon_decimals": LATLON_DECIMALS},
            "x": x,
            "y": y,
            "lat": lat,
            "lon": lon,
            "built_at": time.time(),
        }


def legacy_points(payload: dict[str, Any]) -> list[dict[str, Any]]:
    """Old per-point shape (``/loaded-path`` consumers) from a live payload."""

    return [
        {
            "index": index,
            "x": x,
            "y": y,
            "latitude": lat,
            "longitude": lon,
            "projection": "PX4_MAP_PROJECTION_REPROJECT",
        }
        for index, (x, y, lat, lon) in enumerate(
            zip(payload["x"], payload["y"], payload["lat"], payload["lon"])
        )
    ]


trajectory_snapshot = TrajectorySnapshotService()
