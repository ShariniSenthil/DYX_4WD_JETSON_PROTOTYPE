"""ROS-free deterministic RTK manager supervision core.

This module is the future backend-owned RTKManager's decision layer. It
answers whether a worker should exist, when it may start, which run_id owns
the current worker, whether the process adapter should spawn or stop, and
how restart, backoff, and terminal errors behave.

It does not spawn processes, open sockets, touch the filesystem, read the
wall clock, or depend on ROS, FastAPI, threads, or asyncio. Callers inject
monotonic ``now_sec`` and apply returned immutable action objects.

Invariants:

* At most one ``active_run_id`` exists at a time.
* ``STOPPED`` never has an active child.
* ``RUNNING`` has an active matching child that has reported READY.
* ``ERROR`` never auto-spawns and does not clear itself on repeated start.
* Desired ``STOPPED`` never auto-restarts, even if the child crash-exits.
* ``SpawnWorker`` / ``StopWorker`` are emitted at most once per run_id.
* Stale events for a non-active ``run_id`` are ignored without mutating
  child ownership.
* Injected time is finite and nondecreasing. A rejected backwards timestamp
  does not mutate manager state.
"""

from __future__ import annotations

import math
import uuid
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional, Union


# ---------------------------------------------------------------------------
# Time and numeric validation
# ---------------------------------------------------------------------------


def _require_finite_number(value: object, name: str) -> float:
    """Return ``value`` as float, rejecting bools, non-numerics, and non-finites."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError('%s must be a finite number' % name)
    number = float(value)
    if not math.isfinite(number):
        raise ValueError('%s must be finite' % name)
    return number


def _require_finite_now_sec(now_sec: float) -> float:
    """Return ``now_sec`` as float, rejecting non-numeric and non-finite values."""
    return _require_finite_number(now_sec, 'now_sec')


def _require_positive_finite(value: object, name: str) -> float:
    """Return a finite number strictly greater than zero."""
    number = _require_finite_number(value, name)
    if number <= 0.0:
        raise ValueError('%s must be a finite value > 0' % name)
    return number


def _require_bool(value: object, name: str) -> bool:
    if not isinstance(value, bool):
        raise TypeError('%s must be a bool' % name)
    return value


def _default_run_id_factory() -> str:
    return str(uuid.uuid4())


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------


class DesiredState(Enum):
    """Operator-requested lifecycle target."""

    STOPPED = 'STOPPED'
    RUNNING = 'RUNNING'


class ManagerState(Enum):
    """Explicit supervisor state. Names are the public status vocabulary."""

    STOPPED = 'STOPPED'
    WAITING_FOR_MAVROS = 'WAITING_FOR_MAVROS'
    STARTING = 'STARTING'
    RUNNING = 'RUNNING'
    RUNNING_MAVROS_STALE = 'RUNNING_MAVROS_STALE'
    BACKOFF = 'BACKOFF'
    STOPPING = 'STOPPING'
    ERROR = 'ERROR'


class WorkerExitReason(Enum):
    """Semantic child-exit reasons. Numeric OS exit-code mapping is Phase 2B."""

    CLEAN = 'CLEAN'
    RETRYABLE_FAILURE = 'RETRYABLE_FAILURE'
    CONFIG_INVALID = 'CONFIG_INVALID'
    OWNERSHIP_CONFLICT = 'OWNERSHIP_CONFLICT'
    AUTH_FAILED = 'AUTH_FAILED'
    MOUNTPOINT_REJECTED = 'MOUNTPOINT_REJECTED'


class ErrorReason(Enum):
    """Latched terminal manager error. Cleared only by a STOPPED reset cycle."""

    CONFIG_INVALID = 'CONFIG_INVALID'
    OWNERSHIP_CONFLICT = 'OWNERSHIP_CONFLICT'
    AUTH_FAILED = 'AUTH_FAILED'
    MOUNTPOINT_REJECTED = 'MOUNTPOINT_REJECTED'
    RESTART_BUDGET_EXHAUSTED = 'RESTART_BUDGET_EXHAUSTED'


TERMINAL_EXIT_REASONS = frozenset(
    {
        WorkerExitReason.CONFIG_INVALID,
        WorkerExitReason.OWNERSHIP_CONFLICT,
        WorkerExitReason.AUTH_FAILED,
        WorkerExitReason.MOUNTPOINT_REJECTED,
    }
)

_EXIT_REASON_TO_ERROR = {
    WorkerExitReason.CONFIG_INVALID: ErrorReason.CONFIG_INVALID,
    WorkerExitReason.OWNERSHIP_CONFLICT: ErrorReason.OWNERSHIP_CONFLICT,
    WorkerExitReason.AUTH_FAILED: ErrorReason.AUTH_FAILED,
    WorkerExitReason.MOUNTPOINT_REJECTED: ErrorReason.MOUNTPOINT_REJECTED,
}


class _StopKind(Enum):
    """Why the manager emitted StopWorker for the active run."""

    USER = 'USER'
    STARTUP_TIMEOUT = 'STARTUP_TIMEOUT'


# ---------------------------------------------------------------------------
# Immutable actions and snapshots
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class SpawnWorker:
    """Ask the process adapter to create a worker for ``run_id``."""

    run_id: str


@dataclass(frozen=True, slots=True)
class StopWorker:
    """Ask the process adapter to stop the worker owned by ``run_id``."""

    run_id: str


ManagerAction = Union[SpawnWorker, StopWorker]


@dataclass(frozen=True, slots=True)
class RestartPolicy:
    """Immutable restart / timeout policy.

    Development defaults:

    * ``initial_backoff_sec`` = 1.0
    * ``backoff_multiplier`` = 2.0
    * ``max_backoff_sec`` = 30.0
    * ``restart_window_sec`` = 60.0
    * ``max_restarts_in_window`` = 5
    * ``stable_run_reset_sec`` = 60.0
    * ``startup_timeout_sec`` = 10.0
    """

    initial_backoff_sec: float = 1.0
    backoff_multiplier: float = 2.0
    max_backoff_sec: float = 30.0
    restart_window_sec: float = 60.0
    max_restarts_in_window: int = 5
    stable_run_reset_sec: float = 60.0
    startup_timeout_sec: float = 10.0

    def __post_init__(self) -> None:
        initial = _require_positive_finite(
            self.initial_backoff_sec, 'initial_backoff_sec'
        )
        multiplier = _require_finite_number(
            self.backoff_multiplier, 'backoff_multiplier'
        )
        if multiplier < 1.0:
            raise ValueError('backoff_multiplier must be >= 1')
        max_backoff = _require_positive_finite(
            self.max_backoff_sec, 'max_backoff_sec'
        )
        if max_backoff < initial:
            raise ValueError('max_backoff_sec must be >= initial_backoff_sec')
        window = _require_positive_finite(
            self.restart_window_sec, 'restart_window_sec'
        )
        if isinstance(self.max_restarts_in_window, bool) or not isinstance(
            self.max_restarts_in_window, int
        ):
            raise TypeError('max_restarts_in_window must be an int >= 1')
        if self.max_restarts_in_window < 1:
            raise ValueError('max_restarts_in_window must be an int >= 1')
        stable = _require_positive_finite(
            self.stable_run_reset_sec, 'stable_run_reset_sec'
        )
        startup = _require_positive_finite(
            self.startup_timeout_sec, 'startup_timeout_sec'
        )
        object.__setattr__(self, 'initial_backoff_sec', initial)
        object.__setattr__(self, 'backoff_multiplier', multiplier)
        object.__setattr__(self, 'max_backoff_sec', max_backoff)
        object.__setattr__(self, 'restart_window_sec', window)
        object.__setattr__(self, 'stable_run_reset_sec', stable)
        object.__setattr__(self, 'startup_timeout_sec', startup)


@dataclass(frozen=True, slots=True)
class RtkManagerSnapshot:
    """Immutable status snapshot for later backend/REST exposure."""

    desired_state: DesiredState
    manager_state: ManagerState
    mavros_ready: bool
    active_run_id: Optional[str]
    child_started: bool
    child_ready: bool
    next_restart_at: Optional[float]
    consecutive_failures: int
    restart_count_in_window: int
    error_reason: Optional[ErrorReason]


# ---------------------------------------------------------------------------
# Core
# ---------------------------------------------------------------------------


class RtkManagerCore:
    """Pure desired-state supervisor for one RTK correction worker.

    Public methods return zero or more immutable process actions. Repeated
    calls with no new transition never re-emit the same spawn/stop action.
    """

    def __init__(
        self,
        policy: Optional[RestartPolicy] = None,
        run_id_factory: Optional[Callable[[], str]] = None,
    ) -> None:
        if policy is None:
            policy = RestartPolicy()
        if not isinstance(policy, RestartPolicy):
            raise TypeError('policy must be a RestartPolicy')
        if run_id_factory is None:
            run_id_factory = _default_run_id_factory
        if not callable(run_id_factory):
            raise TypeError('run_id_factory must be callable')
        self.policy = policy
        self.run_id_factory = run_id_factory
        self._desired = DesiredState.STOPPED
        self._state = ManagerState.STOPPED
        self._mavros_ready = False
        self._active_run_id: Optional[str] = None
        self._issued_run_ids: set[str] = set()
        self._child_started = False
        self._child_ready = False
        self._child_started_at: Optional[float] = None
        self._ready_at: Optional[float] = None
        self._stop_emitted = False
        self._stop_kind: Optional[_StopKind] = None
        self._consecutive_failures = 0
        self._auto_restart_times: list[float] = []
        self._next_restart_at: Optional[float] = None
        self._needs_automatic_restart = False
        self._error_reason: Optional[ErrorReason] = None
        self._last_now_sec: Optional[float] = None

    # -- public API --------------------------------------------------------

    @property
    def snapshot(self) -> RtkManagerSnapshot:
        """Return an immutable copy of supervisor status."""
        now = self._last_now_sec
        return RtkManagerSnapshot(
            desired_state=self._desired,
            manager_state=self._state,
            mavros_ready=self._mavros_ready,
            active_run_id=self._active_run_id,
            child_started=self._child_started,
            child_ready=self._child_ready,
            next_restart_at=self._next_restart_at,
            consecutive_failures=self._consecutive_failures,
            restart_count_in_window=self._restart_count_in_window(now),
            error_reason=self._error_reason,
        )

    def request_start(self, now_sec: float) -> tuple[ManagerAction, ...]:
        """Set desired RUNNING. Does not clear a latched terminal ERROR."""
        now = self._accept_now_sec(now_sec)
        self._last_now_sec = now
        self._desired = DesiredState.RUNNING
        if self._state == ManagerState.ERROR:
            return ()
        return tuple(self._maybe_launch(now, automatic=self._retry_pending()))

    def request_stop(self, now_sec: float) -> tuple[ManagerAction, ...]:
        """Set desired STOPPED. User STOP overrides restart policy."""
        now = self._accept_now_sec(now_sec)
        self._last_now_sec = now
        self._desired = DesiredState.STOPPED
        self._next_restart_at = None
        self._needs_automatic_restart = False
        if self._active_run_id is None:
            self._enter_stopped()
            return ()
        return tuple(self._emit_stop(self._active_run_id, _StopKind.USER))

    def set_mavros_ready(
        self,
        ready: bool,
        now_sec: float,
    ) -> tuple[ManagerAction, ...]:
        """Update the MAVROS start gate. Does not kill an already-started worker."""
        now = self._accept_now_sec(now_sec)
        ready_flag = _require_bool(ready, 'ready')
        self._last_now_sec = now
        return tuple(self._apply_mavros_ready(ready_flag, now))

    def on_child_started(
        self,
        run_id: str,
        now_sec: float,
    ) -> tuple[ManagerAction, ...]:
        """Record that the adapter created the process for ``run_id``."""
        now = self._accept_now_sec(now_sec)
        self._last_now_sec = now
        if not self._is_active_run(run_id):
            return ()
        if not self._child_started:
            self._child_started = True
            self._child_started_at = now
        return ()

    def on_child_ready(
        self,
        run_id: str,
        now_sec: float,
    ) -> tuple[ManagerAction, ...]:
        """Record worker READY. Stale or post-stop READY cannot restore RUNNING."""
        now = self._accept_now_sec(now_sec)
        self._last_now_sec = now
        if not self._is_active_run(run_id):
            return ()
        if self._stop_emitted or self._state == ManagerState.STOPPING:
            return ()
        if not self._child_started:
            self._child_started = True
            self._child_started_at = now
        if not self._child_ready:
            self._child_ready = True
            self._ready_at = now
        if self._mavros_ready:
            self._state = ManagerState.RUNNING
        else:
            self._state = ManagerState.RUNNING_MAVROS_STALE
        return ()

    def on_child_exit(
        self,
        run_id: str,
        reason: WorkerExitReason,
        now_sec: float,
    ) -> tuple[ManagerAction, ...]:
        """Handle a matching child exit. Stale run_ids are ignored."""
        now = self._accept_now_sec(now_sec)
        if not isinstance(reason, WorkerExitReason):
            raise TypeError('reason must be a WorkerExitReason')
        self._last_now_sec = now
        if not self._is_active_run(run_id):
            return ()
        return tuple(self._handle_child_exit(reason, now))

    def tick(self, now_sec: float) -> tuple[ManagerAction, ...]:
        """Advance timers: startup timeout and backoff deadline."""
        now = self._accept_now_sec(now_sec)
        self._last_now_sec = now
        actions: list[ManagerAction] = []
        actions.extend(self._maybe_startup_timeout(now))
        if self._state == ManagerState.BACKOFF:
            actions.extend(
                self._maybe_launch(now, automatic=True)
            )
        return tuple(actions)

    # -- time --------------------------------------------------------------

    def _accept_now_sec(self, now_sec: float) -> float:
        """Return ``now_sec`` if finite and nondecreasing; do not mutate state."""
        now = _require_finite_now_sec(now_sec)
        if self._last_now_sec is not None and now < self._last_now_sec:
            raise ValueError('now_sec must be nondecreasing')
        return now

    # -- desired-state / MAVROS gate --------------------------------------

    def _apply_mavros_ready(
        self,
        ready: bool,
        now: float,
    ) -> list[ManagerAction]:
        self._mavros_ready = ready
        if not ready:
            return self._on_mavros_lost()
        return self._on_mavros_recovered(now)

    def _on_mavros_lost(self) -> list[ManagerAction]:
        """MAVROS disappeared. Never kill or replace an already-started worker."""
        if self._state in (ManagerState.STARTING, ManagerState.RUNNING):
            self._state = ManagerState.RUNNING_MAVROS_STALE
        elif self._state == ManagerState.BACKOFF:
            self._state = ManagerState.WAITING_FOR_MAVROS
        return []

    def _on_mavros_recovered(self, now: float) -> list[ManagerAction]:
        if self._state == ManagerState.ERROR:
            return []
        if self._state == ManagerState.RUNNING_MAVROS_STALE:
            if self._child_ready:
                self._state = ManagerState.RUNNING
            elif self._active_run_id is not None and not self._stop_emitted:
                self._state = ManagerState.STARTING
            return []
        if self._desired != DesiredState.RUNNING:
            return []
        if self._state in (
            ManagerState.WAITING_FOR_MAVROS,
            ManagerState.BACKOFF,
            ManagerState.STOPPED,
        ):
            return self._maybe_launch(now, automatic=self._retry_pending())
        return []

    def _retry_pending(self) -> bool:
        return self._needs_automatic_restart or self._next_restart_at is not None

    # -- child lifecycle ---------------------------------------------------

    def _is_active_run(self, run_id: object) -> bool:
        return (
            isinstance(run_id, str)
            and run_id != ''
            and self._active_run_id is not None
            and run_id == self._active_run_id
        )

    def _handle_child_exit(
        self,
        reason: WorkerExitReason,
        now: float,
    ) -> list[ManagerAction]:
        stop_kind = self._stop_kind
        ready_at = self._ready_at
        self._clear_child()
        if self._desired == DesiredState.STOPPED:
            self._enter_stopped()
            return []
        if reason in TERMINAL_EXIT_REASONS:
            self._latch_error(_EXIT_REASON_TO_ERROR[reason])
            return []
        if stop_kind == _StopKind.USER:
            # User stop completed, but desired is RUNNING again: fresh start.
            self._needs_automatic_restart = False
            self._consecutive_failures = 0
            self._next_restart_at = None
            return self._maybe_launch(now, automatic=False)
        return self._handle_retryable_exit(now, ready_at)

    def _handle_retryable_exit(
        self,
        now: float,
        ready_at: Optional[float],
    ) -> list[ManagerAction]:
        """Unexpected CLEAN / RETRYABLE_FAILURE / startup-timeout while desired RUNNING."""
        self._needs_automatic_restart = True
        if (
            ready_at is not None
            and (now - ready_at) >= self.policy.stable_run_reset_sec
        ):
            self._consecutive_failures = 0
        self._consecutive_failures += 1
        self._schedule_backoff(now)
        if not self._mavros_ready:
            self._state = ManagerState.WAITING_FOR_MAVROS
            return []
        self._state = ManagerState.BACKOFF
        return []

    def _clear_child(self) -> None:
        self._active_run_id = None
        self._child_started = False
        self._child_ready = False
        self._child_started_at = None
        self._ready_at = None
        self._stop_emitted = False
        self._stop_kind = None

    # -- spawn / stop actions ---------------------------------------------

    def _maybe_launch(
        self,
        now: float,
        *,
        automatic: bool,
    ) -> list[ManagerAction]:
        if self._desired != DesiredState.RUNNING:
            return []
        if self._state == ManagerState.ERROR:
            return []
        if self._active_run_id is not None:
            return []
        if not self._mavros_ready:
            self._state = ManagerState.WAITING_FOR_MAVROS
            return []
        if self._next_restart_at is not None and now < self._next_restart_at:
            self._state = ManagerState.BACKOFF
            return []
        if automatic:
            self._prune_restart_times(now)
            if not self._budget_allows_restart(now):
                self._latch_error(ErrorReason.RESTART_BUDGET_EXHAUSTED)
                return []
        return self._spawn(now, automatic=automatic)

    def _spawn(self, now: float, *, automatic: bool) -> list[ManagerAction]:
        if self._active_run_id is not None:
            raise RuntimeError('invariant violated: spawn with active run_id')
        run_id = self.run_id_factory()
        if not isinstance(run_id, str):
            run_id = str(run_id)
        if run_id == '':
            raise ValueError('run_id_factory returned an empty run_id')
        if run_id in self._issued_run_ids:
            raise RuntimeError('run_id_factory reused a run_id')
        self._issued_run_ids.add(run_id)
        self._active_run_id = run_id
        self._child_started = False
        self._child_ready = False
        self._child_started_at = None
        self._ready_at = None
        self._stop_emitted = False
        self._stop_kind = None
        self._next_restart_at = None
        self._state = ManagerState.STARTING
        if automatic:
            self._auto_restart_times.append(now)
        return [SpawnWorker(run_id)]

    def _emit_stop(
        self,
        run_id: str,
        kind: _StopKind,
    ) -> list[ManagerAction]:
        if self._active_run_id != run_id:
            return []
        if self._stop_emitted:
            if kind == _StopKind.USER:
                self._stop_kind = _StopKind.USER
            return []
        self._stop_emitted = True
        self._stop_kind = kind
        self._state = ManagerState.STOPPING
        return [StopWorker(run_id)]

    def _maybe_startup_timeout(self, now: float) -> list[ManagerAction]:
        if self._active_run_id is None:
            return []
        if self._stop_emitted or self._state == ManagerState.STOPPING:
            return []
        if not self._child_started or self._child_ready:
            return []
        if self._child_started_at is None:
            return []
        elapsed = now - self._child_started_at
        if elapsed < self.policy.startup_timeout_sec:
            return []
        return self._emit_stop(
            self._active_run_id, _StopKind.STARTUP_TIMEOUT
        )

    # -- restart scheduling and budget ------------------------------------

    def _schedule_backoff(self, now: float) -> None:
        delay = self._backoff_delay_sec()
        self._next_restart_at = now + delay

    def _backoff_delay_sec(self) -> float:
        """Exponential delay for the current consecutive-failure count."""
        delay = self.policy.initial_backoff_sec
        steps = max(0, self._consecutive_failures - 1)
        for _ in range(steps):
            delay = delay * self.policy.backoff_multiplier
            if delay >= self.policy.max_backoff_sec:
                return self.policy.max_backoff_sec
        if delay > self.policy.max_backoff_sec:
            return self.policy.max_backoff_sec
        return delay

    def _budget_allows_restart(self, now: float) -> bool:
        return (
            self._restart_count_in_window(now)
            < self.policy.max_restarts_in_window
        )

    def _prune_restart_times(self, now: float) -> None:
        window = self.policy.restart_window_sec
        self._auto_restart_times = [
            stamp
            for stamp in self._auto_restart_times
            if (now - stamp) <= window
        ]

    def _restart_count_in_window(self, now: Optional[float]) -> int:
        if now is None:
            return len(self._auto_restart_times)
        window = self.policy.restart_window_sec
        return sum(
            1
            for stamp in self._auto_restart_times
            if (now - stamp) <= window
        )

    def _enter_stopped(self) -> None:
        self._state = ManagerState.STOPPED
        self._error_reason = None
        self._consecutive_failures = 0
        self._auto_restart_times = []
        self._next_restart_at = None
        self._needs_automatic_restart = False
        self._ready_at = None

    def _latch_error(self, reason: ErrorReason) -> None:
        self._state = ManagerState.ERROR
        self._error_reason = reason
        self._next_restart_at = None
        self._needs_automatic_restart = False
        self._active_run_id = None
        self._child_started = False
        self._child_ready = False
        self._child_started_at = None
        self._ready_at = None
        self._stop_emitted = False
        self._stop_kind = None
