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
import os
import threading
import time
import uuid
from typing import Any, Callable, Sequence

from mission_manager.path_contract import PendingPreparedPath
from mission_manager.path_contract import resolve_path_signature

LOGGER = logging.getLogger(__name__)

SCHEMA_VERSION = 1
EVENT_PATH = "trajectory_path"
EVENT_CLEARED = "trajectory_path_cleared"

# Display encoding: 0.1 mm for local metres, 1e-9 deg (~0.1 mm) for lat/lon.
XY_DECIMALS = 4
LATLON_DECIMALS = 9

Point = tuple[float, float]
Projector = Callable[[float, float, float, float], tuple[float, float]]
Listener = Callable[[dict[str, Any]], None]


def push_enabled() -> bool:
    """Rollback switch: DYX_TRAJECTORY_PUSH=0 disables all snapshot work."""

    return os.environ.get("DYX_TRAJECTORY_PUSH", "1").strip() != "0"


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

        self._seq = 0
        self._components_version = 0
        self._verified: tuple[int, str] | None = None
        self._live: dict[str, Any] | None = None
        self._live_key: tuple[Any, ...] | None = None

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
            self._components_version += 1
            self._verified = None
            self._live = None
            self._live_key = None

    # ------------------------------------------------------------- read access
    def live_payload(self) -> dict[str, Any] | None:
        """The last emitted, still-valid ``trajectory_path`` payload, or None."""

        with self._lock:
            return self._live

    # ------------------------------------------------------------------ feeds
    def feed_nav(self, points: Sequence[Point] | None) -> None:
        self._feed_component("navigation_path", points)

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
    def _feed_component(self, source: str, value: Any) -> None:
        if not push_enabled():
            return
        with self._lock:
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

    def _clear_locked(self, reason: str) -> list[dict[str, Any]]:
        live = self._live
        self._live = None
        self._live_key = None
        if live is None:
            return []
        return [
            {
                "event": EVENT_CLEARED,
                "payload": {
                    "schema_version": SCHEMA_VERSION,
                    "server_instance_id": self.server_instance_id,
                    "seq": self._next_seq(),
                    "mission_id": live.get("mission_id"),
                    "signature": live.get("signature"),
                    "reason": reason,
                },
            }
        ]

    def _evaluate_locked(self, *, source_cleared: bool = False) -> list[dict[str, Any]]:
        status = self._status
        ready = bool(
            status
            and str(status.get("state", "")).strip().upper() == "READY"
            and status.get("ready")
        )
        if not ready:
            return self._clear_locked("generator_not_ready")
        if source_cleared:
            return self._clear_locked("source_cleared")

        pending = self._pending
        nav = pending.navigation_path
        markings = pending.mission_waypoints
        types = pending.point_types
        indices = pending.marking_indices
        signature = pending.path_signature
        if any(v is None for v in (nav, markings, types, indices, signature)):
            return []
        if self._origin is None:
            return []

        count = len(nav)
        if not (count == len(types) == len(indices)):
            return []
        try:
            status_count = int(status.get("navigation_point_count"))
        except (TypeError, ValueError):
            return []
        if status_count != count or status.get("path_signature") != signature:
            return []

        verified = (self._components_version, signature)
        if self._verified != verified:
            decision = resolve_path_signature(
                nav, markings, types, indices, signature, signature_required=True
            )
            if not decision.can_install:
                LOGGER.warning("trajectory snapshot rejected: %s", decision.reason)
                return []
            self._verified = verified

        key = (status.get("mission_id"), signature, self._origin)
        if self._live is not None and self._live_key == key:
            return []

        payload = self._build_payload_locked(status, nav, signature)
        if payload is None:
            return []
        self._live = payload
        self._live_key = key
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
