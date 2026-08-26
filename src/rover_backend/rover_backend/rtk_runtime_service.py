"""Backend-owned single-thread RTK runtime supervision service.

All RtkManagerCore and RtkProcessAdapter lifecycle mutations are serialized
onto exactly one supervisor thread. ROS callbacks expose cached readiness and
FastAPI/API callers submit intent; neither calls the orchestrator directly.

The service owns no RTK profile persistence. A later profile store supplies
the WorkerConfig factory.
"""

from __future__ import annotations

import math
import queue
import sys
import threading
import time

from dataclasses import dataclass
from enum import Enum
from typing import Callable
from typing import Optional
from typing import Protocol
from typing import Sequence

from rover_backend.rtk_manager_core import (
    ManagerState,
    RtkManagerCore,
)
from rover_backend.rtk_process_adapter import (
    RtkProcessAdapter,
)
from rover_backend.rtk_process_protocol import (
    WorkerConfig,
)
from rover_backend.rtk_runtime_orchestrator import (
    RtkRuntimeOrchestrator,
    RtkRuntimeSnapshot,
)


DEFAULT_SUPERVISOR_POLL_SEC = 0.10
DEFAULT_COMMAND_TIMEOUT_SEC = 2.0
DEFAULT_START_TIMEOUT_SEC = 2.0
DEFAULT_SHUTDOWN_TIMEOUT_SEC = 8.0


class RtkRuntimeServiceError(RuntimeError):
    """Base runtime-service failure."""


class RtkRuntimeServiceNotRunningError(
    RtkRuntimeServiceError
):
    """A lifecycle command requires a running supervisor."""


class RtkRuntimeServiceShuttingDownError(
    RtkRuntimeServiceError
):
    """New lifecycle work is rejected once shutdown starts."""


class _CommandKind(Enum):
    START = "START"
    STOP = "STOP"


@dataclass(slots=True)
class _RuntimeCommand:
    kind: _CommandKind
    done: threading.Event
    error: Optional[BaseException] = None


class _RuntimeOrchestrator(Protocol):
    @property
    def snapshot(self) -> RtkRuntimeSnapshot: ...

    def open(self): ...

    def close(self) -> None: ...

    def request_start(
        self,
        now_sec: float,
    ): ...

    def request_stop(
        self,
        now_sec: float,
    ): ...

    def set_mavros_ready(
        self,
        ready: bool,
        now_sec: float,
    ): ...

    def tick(
        self,
        now_sec: float,
    ): ...


@dataclass(
    frozen=True,
    slots=True,
)
class RtkRuntimeServiceSnapshot:
    """Credential-free supervisor diagnostics."""

    running: bool
    shutdown_requested: bool
    owner_thread_id: Optional[int]
    mavros_ready: bool
    last_error_code: Optional[str]
    runtime: Optional[RtkRuntimeSnapshot]


class RtkRuntimeService:
    """Own one RTK orchestrator on one dedicated supervisor thread."""

    def __init__(
        self,
        orchestrator: _RuntimeOrchestrator,
        mavros_readiness_provider: Callable[[], bool],
        *,
        poll_interval_sec: float = DEFAULT_SUPERVISOR_POLL_SEC,
        command_timeout_sec: float = DEFAULT_COMMAND_TIMEOUT_SEC,
        start_timeout_sec: float = DEFAULT_START_TIMEOUT_SEC,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not callable(mavros_readiness_provider):
            raise TypeError(
                "mavros_readiness_provider must be callable"
            )

        if not callable(clock):
            raise TypeError(
                "clock must be callable"
            )

        self._require_positive_number(
            poll_interval_sec,
            "poll_interval_sec",
        )
        self._require_positive_number(
            command_timeout_sec,
            "command_timeout_sec",
        )
        self._require_positive_number(
            start_timeout_sec,
            "start_timeout_sec",
        )

        self._orchestrator = orchestrator
        self._mavros_readiness_provider = (
            mavros_readiness_provider
        )

        self._poll_interval_sec = float(
            poll_interval_sec
        )
        self._command_timeout_sec = float(
            command_timeout_sec
        )
        self._start_timeout_sec = float(
            start_timeout_sec
        )
        self._clock = clock

        self._state_lock = threading.RLock()

        self._commands: queue.Queue[
            _RuntimeCommand
        ] = queue.Queue()

        self._wake_event = threading.Event()
        self._started_event = threading.Event()

        self._thread: Optional[
            threading.Thread
        ] = None

        self._lifecycle_started = False
        self._shutdown_requested = False

        self._owner_thread_id: Optional[int] = (
            None
        )

        self._mavros_ready = False

        self._last_error_code: Optional[str] = (
            None
        )

        self._runtime_snapshot: Optional[
            RtkRuntimeSnapshot
        ] = None

        self._startup_error: Optional[
            BaseException
        ] = None

    @staticmethod
    def _require_positive_number(
        value: object,
        name: str,
    ) -> None:
        if (
            isinstance(value, bool)
            or not isinstance(
                value,
                (int, float),
            )
            or float(value) <= 0.0
        ):
            raise ValueError(
                f"{name} must be a number > 0"
            )

    @property
    def running(self) -> bool:
        with self._state_lock:
            thread = self._thread

            return bool(
                thread is not None
                and thread.is_alive()
                and self._owner_thread_id
                is not None
            )

    @property
    def snapshot(
        self,
    ) -> RtkRuntimeServiceSnapshot:
        with self._state_lock:
            return RtkRuntimeServiceSnapshot(
                running=self.running,
                shutdown_requested=(
                    self._shutdown_requested
                ),
                owner_thread_id=(
                    self._owner_thread_id
                ),
                mavros_ready=(
                    self._mavros_ready
                ),
                last_error_code=(
                    self._last_error_code
                ),
                runtime=(
                    self._runtime_snapshot
                ),
            )

    def start(self) -> None:
        """Start the one-shot supervisor lifetime."""

        with self._state_lock:
            if self.running:
                return

            if self._lifecycle_started:
                raise RtkRuntimeServiceError(
                    "RTK runtime service lifetime "
                    "has already been started"
                )

            self._lifecycle_started = True
            self._shutdown_requested = False
            self._startup_error = None
            self._last_error_code = None

            self._started_event.clear()
            self._wake_event.clear()

            thread = threading.Thread(
                target=self._thread_main,
                name="rover-rtk-supervisor",
                daemon=True,
            )

            self._thread = thread

            thread.start()

        if not self._started_event.wait(
            self._start_timeout_sec
        ):
            with self._state_lock:
                self._last_error_code = (
                    "SUPERVISOR_START_TIMEOUT"
                )

            raise RtkRuntimeServiceError(
                "RTK supervisor failed to start"
            )

        with self._state_lock:
            startup_error = (
                self._startup_error
            )

        if startup_error is not None:
            raise RtkRuntimeServiceError(
                "RTK supervisor startup failed"
            ) from startup_error

        # _started_event is published only after orchestrator.open() has
        # completed. Once that succeeds, subsequent supervisor failures are
        # runtime failures even if the thread exits before this caller gets
        # scheduled again. Do not race thread liveness against start().

    def request_start(self) -> None:
        """Submit desired RUNNING to the supervisor."""

        self._submit(
            _CommandKind.START
        )

    def request_stop(self) -> None:
        """Submit desired STOPPED to the supervisor."""

        self._submit(
            _CommandKind.STOP
        )

    def shutdown(
        self,
        timeout_sec: float = DEFAULT_SHUTDOWN_TIMEOUT_SEC,
    ) -> bool:
        """Stop, reap and close the owned runtime.

        Returns True only after the supervisor thread has exited cleanly.
        A False result means backend shutdown must continue fail-closed; process
        exit will still close the worker's parent-liveness descriptor.
        """

        self._require_positive_number(
            timeout_sec,
            "timeout_sec",
        )

        with self._state_lock:
            thread = self._thread

            if thread is None:
                self._shutdown_requested = True
                return True

            self._shutdown_requested = True

        self._wake_event.set()

        thread.join(
            timeout=float(timeout_sec)
        )

        if thread.is_alive():
            with self._state_lock:
                self._last_error_code = (
                    "SUPERVISOR_SHUTDOWN_TIMEOUT"
                )

            return False

        return True

    def _submit(
        self,
        kind: _CommandKind,
    ) -> None:
        with self._state_lock:
            if self._shutdown_requested:
                raise (
                    RtkRuntimeServiceShuttingDownError(
                        "RTK runtime service is "
                        "shutting down"
                    )
                )

            if not self.running:
                raise (
                    RtkRuntimeServiceNotRunningError(
                        "RTK runtime service is "
                        "not running"
                    )
                )

        command = _RuntimeCommand(
            kind=kind,
            done=threading.Event(),
        )

        self._commands.put(command)
        self._wake_event.set()

        if not command.done.wait(
            self._command_timeout_sec
        ):
            raise RtkRuntimeServiceError(
                "RTK supervisor command timed out"
            )

        if command.error is not None:
            raise RtkRuntimeServiceError(
                "RTK supervisor command failed"
            ) from command.error

    def _thread_main(self) -> None:
        """Own the orchestrator until clean stop, reap and close.

        Once adapter authority has been acquired this thread must never exit
        merely because ordinary runtime supervision raised. Any such failure
        is converted into fail-closed shutdown and the same owner thread keeps
        driving stop/poll/reap until orchestrator.close() is safe.
        """

        opened = False
        shutdown_stop_sent = False
        runtime_failed = False
        last_now: Optional[float] = None

        try:
            try:
                self._orchestrator.open()
                opened = True
            except BaseException as error:
                with self._state_lock:
                    self._startup_error = error
                    self._last_error_code = (
                        "SUPERVISOR_START_FAILED"
                    )

                return
            finally:
                with self._state_lock:
                    if opened:
                        self._owner_thread_id = (
                            threading.get_ident()
                        )

                # Successful open is the startup boundary. Runtime failures
                # after this point must not be reclassified as startup errors.
                self._started_event.set()

            while True:
                # The injected clock is part of normal deterministic testing,
                # but a broken provider must not be allowed to kill process
                # supervision. Fall back to the real monotonic clock and clamp
                # it so manager/adapter time can never move backwards.
                try:
                    candidate_now = float(
                        self._clock()
                    )

                    if not math.isfinite(
                        candidate_now
                    ):
                        raise ValueError(
                            "RTK supervisor clock "
                            "must be finite"
                        )

                    if (
                        last_now is not None
                        and candidate_now < last_now
                    ):
                        raise ValueError(
                            "RTK supervisor clock "
                            "moved backwards"
                        )

                    now = candidate_now

                except BaseException:
                    runtime_failed = True

                    fallback_now = float(
                        time.monotonic()
                    )

                    if last_now is not None:
                        fallback_now = max(
                            last_now,
                            fallback_now,
                        )

                    now = fallback_now

                    with self._state_lock:
                        self._shutdown_requested = (
                            True
                        )
                        self._last_error_code = (
                            "SUPERVISOR_CLOCK_FAILED"
                        )

                last_now = now

                if not runtime_failed:
                    try:
                        self._refresh_mavros_ready(
                            now
                        )

                        self._drain_commands(
                            now
                        )

                    except BaseException:
                        # A failure while forwarding readiness or processing
                        # queued lifecycle intent must not unwind this owner
                        # thread while a child/liveness FD may still exist.
                        runtime_failed = True

                        with self._state_lock:
                            self._shutdown_requested = (
                                True
                            )
                            self._last_error_code = (
                                "SUPERVISOR_RUNTIME_FAILED"
                            )

                with self._state_lock:
                    shutdown_requested = (
                        self._shutdown_requested
                    )

                if (
                    shutdown_requested
                    and not shutdown_stop_sent
                ):
                    try:
                        self._orchestrator.request_stop(
                            now
                        )
                    except BaseException:
                        # Keep retrying STOP on later iterations. Do not mark
                        # it sent unless the orchestrator accepted the intent.
                        runtime_failed = True

                        with self._state_lock:
                            self._last_error_code = (
                                "SUPERVISOR_STOP_FAILED"
                            )
                    else:
                        shutdown_stop_sent = True

                try:
                    # tick() is still required during failure shutdown because
                    # it drains worker status, escalates TERM -> KILL and reaps.
                    self._orchestrator.tick(
                        now
                    )
                except BaseException:
                    runtime_failed = True

                    with self._state_lock:
                        self._shutdown_requested = (
                            True
                        )
                        self._last_error_code = (
                            "SUPERVISOR_RUNTIME_FAILED"
                        )

                self._cache_runtime_snapshot()

                with self._state_lock:
                    shutdown_requested = (
                        self._shutdown_requested
                    )

                if (
                    shutdown_requested
                    and self._runtime_is_stopped()
                ):
                    try:
                        # close() is legal only after both manager and adapter
                        # ownership prove that the child has been reaped.
                        self._orchestrator.close()
                    except BaseException:
                        runtime_failed = True

                        with self._state_lock:
                            self._last_error_code = (
                                "SUPERVISOR_CLOSE_FAILED"
                            )
                    else:
                        opened = False
                        break

                try:
                    self._wake_event.wait(
                        self._poll_interval_sec
                    )
                except BaseException:
                    # Event waiting itself is never allowed to abandon an
                    # owned worker. Convert it to the same shutdown contract.
                    runtime_failed = True

                    with self._state_lock:
                        self._shutdown_requested = (
                            True
                        )
                        self._last_error_code = (
                            "SUPERVISOR_RUNTIME_FAILED"
                        )

                self._wake_event.clear()

        finally:
            # Releasing manager authority is deliberately NOT attempted here
            # when `opened` remains true. Adapter.close() rejects a live child.
            # Normal runtime exceptions are consumed above and remain inside
            # the supervision loop until stop/reap/close succeeds.
            self._complete_pending_commands()

            with self._state_lock:
                self._owner_thread_id = None

            self._started_event.set()

    def _refresh_mavros_ready(
        self,
        now: float,
    ) -> None:
        provider_failed = False

        try:
            ready = bool(
                self._mavros_readiness_provider()
            )
        except Exception:
            ready = False
            provider_failed = True

        with self._state_lock:
            previous = self._mavros_ready

        if ready != previous:
            self._orchestrator.set_mavros_ready(
                ready,
                now,
            )

            with self._state_lock:
                self._mavros_ready = ready

        if provider_failed:
            with self._state_lock:
                self._last_error_code = (
                    "MAVROS_READINESS_ERROR"
                )
        else:
            with self._state_lock:
                if (
                    self._last_error_code
                    == "MAVROS_READINESS_ERROR"
                ):
                    self._last_error_code = (
                        None
                    )

    def _drain_commands(
        self,
        now: float,
    ) -> None:
        while True:
            try:
                command = (
                    self._commands.get_nowait()
                )
            except queue.Empty:
                return

            try:
                with self._state_lock:
                    shutting_down = (
                        self._shutdown_requested
                    )

                if (
                    shutting_down
                    and command.kind
                    is _CommandKind.START
                ):
                    raise (
                        RtkRuntimeServiceShuttingDownError(
                            "RTK runtime service "
                            "is shutting down"
                        )
                    )

                if (
                    command.kind
                    is _CommandKind.START
                ):
                    self._orchestrator.request_start(
                        now
                    )

                elif (
                    command.kind
                    is _CommandKind.STOP
                ):
                    self._orchestrator.request_stop(
                        now
                    )

                else:
                    raise RuntimeError(
                        "unsupported RTK runtime command"
                    )

            except BaseException as error:
                command.error = error

            finally:
                command.done.set()

    def _cache_runtime_snapshot(
        self,
    ) -> None:
        try:
            snapshot = (
                self._orchestrator.snapshot
            )
        except Exception:
            return

        with self._state_lock:
            self._runtime_snapshot = snapshot

    def _runtime_is_stopped(
        self,
    ) -> bool:
        with self._state_lock:
            snapshot = (
                self._runtime_snapshot
            )

        if snapshot is None:
            return False

        return bool(
            snapshot.manager.manager_state
            is ManagerState.STOPPED
            and snapshot.manager.active_run_id
            is None
            and snapshot.process.active_run_id
            is None
            and snapshot.process.pid is None
        )

    def _complete_pending_commands(
        self,
    ) -> None:
        while True:
            try:
                command = (
                    self._commands.get_nowait()
                )
            except queue.Empty:
                return

            if command.error is None:
                command.error = (
                    RtkRuntimeServiceError(
                        "RTK supervisor stopped "
                        "before command completion"
                    )
                )

            command.done.set()


def default_rtk_worker_command(
) -> tuple[str, ...]:
    """Return an install-space-safe worker command.

    Using the backend interpreter with ``-m`` avoids PATH assumptions while
    preserving the environment inherited from the backend systemd process.
    """

    return (
        sys.executable,
        "-m",
        "rover_backend.rtk_worker_bootstrap",
    )


def build_rtk_runtime_service(
    *,
    config_factory: Callable[
        [str],
        WorkerConfig,
    ],
    mavros_readiness_provider: Callable[
        [],
        bool,
    ],
    worker_command: Optional[
        Sequence[str]
    ] = None,
    poll_interval_sec: float = (
        DEFAULT_SUPERVISOR_POLL_SEC
    ),
    stop_grace_sec: float = 5.0,
) -> RtkRuntimeService:
    """Build the production manager → adapter → worker supervision stack."""

    command = tuple(
        worker_command
        or default_rtk_worker_command()
    )

    core = RtkManagerCore()

    adapter = RtkProcessAdapter(
        command,
        stop_grace_sec=stop_grace_sec,
    )

    orchestrator = RtkRuntimeOrchestrator(
        core,
        adapter,
        config_factory,
    )

    return RtkRuntimeService(
        orchestrator,
        mavros_readiness_provider,
        poll_interval_sec=(
            poll_interval_sec
        ),
    )
