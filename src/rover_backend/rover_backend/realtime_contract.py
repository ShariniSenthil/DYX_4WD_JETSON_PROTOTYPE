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
