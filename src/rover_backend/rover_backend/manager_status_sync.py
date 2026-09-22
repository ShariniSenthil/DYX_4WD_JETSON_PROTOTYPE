"""Synchronize a manager command's HTTP response with its own status.

Mission Manager increments a per-command acknowledgment counter and
force-publishes ``/mission_manager/status`` *before* a successful Trigger
returns. ROS does not order a topic delivery against a service response, so
the backend's status subscription may run slightly after the Trigger
completes. Without this, the HTTP response can be one authoritative status
behind the command it reports.

The wait is event-driven (``threading.Condition``), never a sleep or a poll.
Its timeout is a defensive bound only: a timeout never turns an executed
command into a failure, it just means the response carries the best snapshot
available and Socket.IO remains the recovery path.

ROS-free so it can be tested with real threads.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Mapping


@dataclass(frozen=True)
class AckMark:
    """Where one command's acknowledgment counter stood."""

    epoch: str | None
    count: int


@dataclass(frozen=True)
class SyncResult:
    command: str
    synchronized: bool
    # False when the manager has never published command acknowledgments
    # (older Mission Manager); the backend then returns immediately.
    supported: bool
    wait_ms: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "command": self.command,
            "synchronized": self.synchronized,
            "supported": self.supported,
            "wait_ms": round(self.wait_ms, 3),
        }


class ManagerStatusSync:
    """Condition-backed view of Mission Manager's command acknowledgments."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._epoch: str | None = None
        self._counts: dict[str, int] = {}
        self._supported = False

    def observe(self, payload: Mapping[str, Any]) -> None:
        """Record a status payload. Call after it is committed to rover_state.

        Counters are taken as published, not max-merged: one publisher, one
        topic, so status messages arrive in order. A manager restart changes
        the epoch, which the wait predicate treats as a new acknowledgment
        domain rather than a counter that went backwards.
        """
        acks = payload.get("command_acks")
        if not isinstance(acks, Mapping):
            return
        counts: dict[str, int] = {}
        for name, value in acks.items():
            try:
                counts[str(name)] = int(value)
            except (TypeError, ValueError):
                continue
        epoch_value = payload.get("command_ack_epoch")
        with self._condition:
            self._supported = True
            self._epoch = str(epoch_value) if epoch_value is not None else None
            self._counts = counts
            self._condition.notify_all()

    def mark(self, command: str) -> AckMark:
        """Capture the counter before dispatching ``command``."""
        with self._condition:
            return AckMark(epoch=self._epoch, count=self._counts.get(command, 0))

    def _acknowledged_after(self, command: str, before: AckMark) -> bool:
        if self._epoch != before.epoch:
            return self._counts.get(command, 0) > 0
        return self._counts.get(command, 0) > before.count

    def wait_for_ack(
        self,
        command: str,
        before: AckMark,
        timeout_sec: float,
    ) -> SyncResult:
        started = time.monotonic()
        with self._condition:
            if not self._supported:
                return SyncResult(command, False, False, 0.0)
            synchronized = self._condition.wait_for(
                lambda: self._acknowledged_after(command, before),
                timeout=max(0.0, float(timeout_sec)),
            )
        return SyncResult(
            command,
            bool(synchronized),
            True,
            (time.monotonic() - started) * 1000.0,
        )
