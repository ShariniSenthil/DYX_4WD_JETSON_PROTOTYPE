"""Synchronous orchestration between RTK manager policy and process adapter.

This layer owns no ROS objects, sockets, database connections, threads, or
wall clock. Callers inject monotonic time and a WorkerConfig factory.

Ordering invariant for ``tick``:

    adapter events -> manager event handlers -> manager timers -> actions

That ordering ensures a READY status received exactly at the startup timeout
boundary wins before the manager evaluates the timeout.
"""

from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional, Protocol, Sequence

from rover_backend.rtk_manager_core import (
    ManagerAction,
    RtkManagerCore,
    RtkManagerSnapshot,
    SpawnWorker,
    StopWorker,
    WorkerExitReason,
)
from rover_backend.rtk_process_adapter import (
    AdapterEvent,
    ChildProcessExited,
    ChildProcessStarted,
    ChildStatusReceived,
    ProcessAdapterProtocolError,
    RtkProcessAdapterSnapshot,
    SpawnFailedError,
)
from rover_backend.rtk_process_protocol import (
    ConfigValidationError,
    ProcessProtocolError,
    WorkerConfig,
    WorkerStatusEvent,
    WorkerStatusKind,
    encode_worker_config,
)


LOGGER = logging.getLogger(__name__)


class WorkerConfigBuildError(Exception):
    """The active backend profile cannot produce a valid worker config."""


class OrchestratorNotOpenError(RuntimeError):
    """Manager authority must be acquired before lifecycle mutation."""


class _ProcessAdapter(Protocol):
    def open(self): ...

    def close(self) -> None: ...

    def snapshot(self) -> RtkProcessAdapterSnapshot: ...

    def spawn(
        self,
        action: SpawnWorker,
        config: WorkerConfig,
    ) -> tuple[AdapterEvent, ...]: ...

    def stop(
        self,
        action: StopWorker,
        now_sec: float,
    ) -> tuple[AdapterEvent, ...]: ...

    def poll(
        self,
        now_sec: float,
    ) -> tuple[AdapterEvent, ...]: ...


@dataclass(frozen=True, slots=True)
class RtkRuntimeSnapshot:
    """Credential-free combined manager/process diagnostics."""

    manager: RtkManagerSnapshot
    process: RtkProcessAdapterSnapshot
    last_worker_status: Optional[WorkerStatusEvent]
    last_process_returncode: Optional[int]
    last_protocol_fault_run_id: Optional[str]


class RtkRuntimeOrchestrator:
    """Apply deterministic manager actions to one process adapter."""

    def __init__(
        self,
        core: RtkManagerCore,
        adapter: _ProcessAdapter,
        config_factory: Callable[[str], WorkerConfig],
    ) -> None:
        if not isinstance(core, RtkManagerCore):
            raise TypeError("core must be an RtkManagerCore")
        if not callable(config_factory):
            raise TypeError("config_factory must be callable")

        self.core = core
        self.adapter = adapter
        self.config_factory = config_factory

        self._last_worker_status: Optional[WorkerStatusEvent] = None
        self._last_process_returncode: Optional[int] = None
        self._last_protocol_fault_run_id: Optional[str] = None

    @property
    def snapshot(self) -> RtkRuntimeSnapshot:
        """Return combined state without exposing configuration secrets."""
        return RtkRuntimeSnapshot(
            manager=self.core.snapshot,
            process=self.adapter.snapshot(),
            last_worker_status=self._last_worker_status,
            last_process_returncode=self._last_process_returncode,
            last_protocol_fault_run_id=self._last_protocol_fault_run_id,
        )

    def open(self) -> "RtkRuntimeOrchestrator":
        """Acquire backend RTK manager authority."""
        self.adapter.open()
        return self

    def close(self) -> None:
        """Release manager authority after the child has been reaped."""
        self.adapter.close()

    def request_start(
        self,
        now_sec: float,
    ) -> tuple[AdapterEvent, ...]:
        """Set desired RUNNING and apply any resulting process actions."""
        self._require_open()
        actions = self.core.request_start(now_sec)
        return self._apply_actions(actions, now_sec)

    def request_stop(
        self,
        now_sec: float,
    ) -> tuple[AdapterEvent, ...]:
        """Set desired STOPPED and apply its stop action if required."""
        self._require_open()
        actions = self.core.request_stop(now_sec)
        return self._apply_actions(actions, now_sec)

    def set_mavros_ready(
        self,
        ready: bool,
        now_sec: float,
    ) -> tuple[AdapterEvent, ...]:
        """Update the MAVROS start gate and apply resulting actions."""
        self._require_open()

        previous = (
            self.core.snapshot.mavros_ready
        )

        actions = self.core.set_mavros_ready(
            ready,
            now_sec,
        )

        observed = self._apply_actions(
            actions,
            now_sec,
        )

        if bool(ready) != previous:
            LOGGER.warning(
                "RTK_SUPERVISOR "
                "event=MAVROS_READY_CHANGE "
                "ready=%s manager_state=%s",
                bool(ready),
                self.core.snapshot.manager_state.value,
            )

        return observed

    def tick(
        self,
        now_sec: float,
    ) -> tuple[AdapterEvent, ...]:
        """Poll child first, then advance manager timers."""

        self._require_open()

        observed: list[AdapterEvent] = []

        # Child status/exit gets first right of refusal at this timestamp.
        observed.extend(
            self._poll_adapter(now_sec)
        )

        actions = self.core.tick(now_sec)

        observed.extend(
            self._apply_actions(
                actions,
                now_sec,
            )
        )

        return tuple(observed)

    def _require_open(self) -> None:
        if not self.adapter.snapshot().is_open:
            raise OrchestratorNotOpenError(
                "RTK runtime orchestrator is not open"
            )

    def _poll_adapter(
        self,
        now_sec: float,
    ) -> list[AdapterEvent]:
        try:
            events = self.adapter.poll(
                now_sec
            )
        except ProcessAdapterProtocolError:
            self._handle_protocol_fault(
                now_sec
            )
            return []

        observed = list(events)

        actions = self._actions_for_events(
            events,
            now_sec,
        )

        observed.extend(
            self._apply_actions(
                actions,
                now_sec,
            )
        )

        return observed

    def _handle_protocol_fault(
        self,
        now_sec: float,
    ) -> None:
        """Terminate a child that violated the status wire protocol.

        The manager remains desired RUNNING. Once the child actually exits,
        its mapped RETRYABLE_FAILURE is fed back normally and restart policy
        decides whether another worker may start.
        """

        process = self.adapter.snapshot()
        run_id = process.active_run_id

        if run_id is None or process.pid is None:
            return

        self._last_protocol_fault_run_id = (
            run_id
        )

        LOGGER.error(
            "RTK_SUPERVISOR event=PROTOCOL_FAULT "
            "run_id=%s pid=%s",
            run_id,
            process.pid,
        )

        # This is deliberately an adapter-level stop rather than a manager
        # USER stop. The subsequent child exit remains an unexpected/retryable
        # failure from the manager's perspective.
        self.adapter.stop(
            StopWorker(run_id),
            now_sec,
        )

    def _apply_actions(
        self,
        actions: Sequence[ManagerAction],
        now_sec: float,
    ) -> tuple[AdapterEvent, ...]:
        pending = deque(actions)
        observed: list[AdapterEvent] = []

        # A valid manager transition cannot generate an unbounded synchronous
        # action chain. Keep a hard guard so future changes fail closed.
        action_count = 0

        while pending:
            action_count += 1

            if action_count > 64:
                raise RuntimeError(
                    "RTK orchestration action chain exceeded safety bound"
                )

            action = pending.popleft()

            if isinstance(
                action,
                SpawnWorker,
            ):
                events, follow_up = (
                    self._execute_spawn(
                        action,
                        now_sec,
                    )
                )

            elif isinstance(
                action,
                StopWorker,
            ):
                LOGGER.info(
                    "RTK_SUPERVISOR event=STOP_WORKER "
                    "run_id=%s",
                    action.run_id,
                )

                events = self.adapter.stop(
                    action,
                    now_sec,
                )
                follow_up = ()

            else:
                raise TypeError(
                    "unsupported RTK manager action"
                )

            observed.extend(events)

            pending.extend(
                follow_up
            )

            pending.extend(
                self._actions_for_events(
                    events,
                    now_sec,
                )
            )

        return tuple(observed)

    def _execute_spawn(
        self,
        action: SpawnWorker,
        now_sec: float,
    ) -> tuple[
        tuple[AdapterEvent, ...],
        tuple[ManagerAction, ...],
    ]:
        """Build config and execute one spawn action safely."""

        LOGGER.info(
            "RTK_SUPERVISOR event=SPAWN_REQUEST "
            "run_id=%s",
            action.run_id,
        )

        try:
            config = self.config_factory(
                action.run_id
            )
        except (
            WorkerConfigBuildError,
            ConfigValidationError,
        ):
            return (
                (),
                self.core.on_child_exit(
                    action.run_id,
                    WorkerExitReason.CONFIG_INVALID,
                    now_sec,
                ),
            )
        except Exception:
            # Infrastructure/config-source failure must still close the
            # manager-owned run lifecycle. Treat it as retryable rather than
            # leaving STARTING with no real child.
            return (
                (),
                self.core.on_child_exit(
                    action.run_id,
                    WorkerExitReason.RETRYABLE_FAILURE,
                    now_sec,
                ),
            )

        if not isinstance(
            config,
            WorkerConfig,
        ):
            return (
                (),
                self.core.on_child_exit(
                    action.run_id,
                    WorkerExitReason.CONFIG_INVALID,
                    now_sec,
                ),
            )

        if config.run_id != action.run_id:
            return (
                (),
                self.core.on_child_exit(
                    action.run_id,
                    WorkerExitReason.CONFIG_INVALID,
                    now_sec,
                ),
            )

        # Preflight deterministic protocol encoding before Popen. This lets
        # oversized/invalid serialized config become CONFIG_INVALID rather
        # than looking like an infrastructure spawn failure.
        try:
            encode_worker_config(
                config
            )
        except ProcessProtocolError:
            return (
                (),
                self.core.on_child_exit(
                    action.run_id,
                    WorkerExitReason.CONFIG_INVALID,
                    now_sec,
                ),
            )

        try:
            events = self.adapter.spawn(
                action,
                config,
            )
        except SpawnFailedError:
            process = (
                self.adapter.snapshot()
            )

            if (
                process.active_run_id
                == action.run_id
                and process.pid is not None
            ):
                # Popen succeeded but configuration delivery failed. The
                # adapter intentionally retains ownership while terminating
                # and reaping the real child. Do NOT tell the manager it has
                # exited yet or it could schedule a replacement too early.
                follow_up = (
                    self.core.on_child_started(
                        action.run_id,
                        now_sec,
                    )
                )

                return (
                    (),
                    follow_up,
                )

            # No process exists (pipe/Popen failure). The manager nevertheless
            # already owns this run_id because it emitted SpawnWorker, so close
            # that lifecycle explicitly as retryable.
            return (
                (),
                self.core.on_child_exit(
                    action.run_id,
                    WorkerExitReason.RETRYABLE_FAILURE,
                    now_sec,
                ),
            )

        return (
            events,
            (),
        )

    def _actions_for_events(
        self,
        events: Sequence[AdapterEvent],
        now_sec: float,
    ) -> tuple[ManagerAction, ...]:
        actions: list[ManagerAction] = []

        for event in events:
            actions.extend(
                self._actions_for_event(
                    event,
                    now_sec,
                )
            )

        return tuple(actions)

    def _actions_for_event(
        self,
        event: AdapterEvent,
        now_sec: float,
    ) -> tuple[ManagerAction, ...]:

        if isinstance(
            event,
            ChildProcessStarted,
        ):
            LOGGER.info(
                "RTK_SUPERVISOR event=CHILD_STARTED "
                "run_id=%s pid=%s",
                event.run_id,
                event.pid,
            )

            return self.core.on_child_started(
                event.run_id,
                now_sec,
            )

        if isinstance(
            event,
            ChildStatusReceived,
        ):
            status = event.event

            # Adapter already performs a run_id gate. Keep this second gate so
            # fake/test adapters and future implementations cannot poison
            # diagnostics or lifecycle with stale status.
            if (
                status.run_id
                != self.core.snapshot.active_run_id
            ):
                return ()

            self._last_worker_status = (
                status
            )

            if (
                status.kind
                is WorkerStatusKind.READY
            ):
                LOGGER.info(
                    "RTK_SUPERVISOR event=WORKER_READY "
                    "run_id=%s",
                    status.run_id,
                )

                return self.core.on_child_ready(
                    status.run_id,
                    now_sec,
                )

            if (
                status.kind
                is WorkerStatusKind.TERMINAL_ERROR
            ):
                LOGGER.error(
                    "RTK_SUPERVISOR "
                    "event=WORKER_TERMINAL_STATUS "
                    "run_id=%s detail_code=%s",
                    status.run_id,
                    status.detail_code,
                )

            # STARTED is diagnostic because Popen already provides the process
            # start event. TERMINAL_ERROR is also diagnostic; the authoritative
            # semantic failure is the stable process exit code.
            return ()

        if isinstance(
            event,
            ChildProcessExited,
        ):
            self._last_process_returncode = (
                event.returncode
            )

            actions = self.core.on_child_exit(
                event.run_id,
                event.reason,
                now_sec,
            )

            manager = self.core.snapshot

            if (
                manager.error_reason
                is not None
            ):
                log_method = LOGGER.error

            elif (
                event.reason
                is WorkerExitReason.CLEAN
            ):
                log_method = LOGGER.info

            else:
                log_method = LOGGER.warning

            log_method(
                "RTK_SUPERVISOR event=CHILD_EXIT "
                "run_id=%s returncode=%s "
                "reason=%s manager_state=%s "
                "error_reason=%s restart_count=%s "
                "consecutive_failures=%s "
                "next_restart_at=%s",
                event.run_id,
                event.returncode,
                event.reason.value,
                manager.manager_state.value,
                (
                    None
                    if manager.error_reason
                    is None
                    else manager.error_reason.value
                ),
                manager.restart_count_in_window,
                manager.consecutive_failures,
                manager.next_restart_at,
            )

            return actions

        raise TypeError(
            "unsupported RTK adapter event"
        )
