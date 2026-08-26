"""Tests for RTK manager-to-process runtime orchestration."""

from __future__ import annotations

from collections import deque

import pytest

from rover_backend.rtk_manager_core import (
    DesiredState,
    ErrorReason,
    ManagerState,
    RestartPolicy,
    RtkManagerCore,
    SpawnWorker,
    StopWorker,
    WorkerExitReason,
)
from rover_backend.rtk_process_adapter import (
    ChildProcessExited,
    ChildProcessStarted,
    ChildStatusReceived,
    ProcessAdapterProtocolError,
    RtkProcessAdapterSnapshot,
    SpawnFailedError,
)
from rover_backend.rtk_process_protocol import (
    WORKER_CONFIG_SCHEMA_VERSION,
    WORKER_STATUS_SCHEMA_VERSION,
    WorkerConfig,
    WorkerStatusEvent,
    WorkerStatusKind,
)
from rover_backend.rtk_runtime_orchestrator import (
    OrchestratorNotOpenError,
    RtkRuntimeOrchestrator,
    WorkerConfigBuildError,
)


SECRET = "SUPER_SECRET_RTK_PASSWORD_93a7"


def make_config(
    run_id: str,
    **overrides,
) -> WorkerConfig:
    values = {
        "schema_version":
            WORKER_CONFIG_SCHEMA_VERSION,
        "run_id": run_id,
        "caster_host": "caster.example.test",
        "caster_port": 2101,
        "mountpoint": "ROVER_RTCM3",
        "username": "rover",
        "password": SECRET,
        "rtcm_topic":
            "/mavros/gps_rtk/send_rtcm",
        "connect_timeout_sec": 5.0,
        "socket_timeout_sec": 1.0,
        "healthy_age_sec": 3.0,
        "stale_reconnect_sec": 10.0,
        "reconnect_delay_sec": 2.0,
        "first_data_timeout_sec": 8.0,
        "max_mavros_rtcm_frame_bytes": 720,
    }
    values.update(overrides)
    return WorkerConfig(**values)


def status(
    run_id: str,
    kind: WorkerStatusKind,
    detail_code=None,
):
    return ChildStatusReceived(
        WorkerStatusEvent(
            WORKER_STATUS_SCHEMA_VERSION,
            run_id,
            kind,
            detail_code,
        )
    )


class FakeAdapter:
    def __init__(self):
        self.is_open = False
        self.active_run_id = None
        self.pid = None
        self.spawn_calls = []
        self.stop_calls = []
        self.poll_events = deque()
        self.poll_error = None
        self.spawn_mode = "success"
        self.next_pid = 51000

    def open(self):
        self.is_open = True
        return self

    def close(self):
        if self.active_run_id is not None:
            raise RuntimeError(
                "active fake child"
            )
        self.is_open = False

    def snapshot(self):
        return RtkProcessAdapterSnapshot(
            is_open=self.is_open,
            active_run_id=self.active_run_id,
            pid=self.pid,
            config_writer_open=False,
            liveness_writer_open=(
                self.active_run_id is not None
            ),
            status_reader_open=(
                self.active_run_id is not None
            ),
            status_buffer_bytes=0,
            stop_requested=bool(
                self.stop_calls
            ),
            stop_deadline=None,
            kill_sent=False,
            exit_reported=False,
        )

    def spawn(self, action, config):
        self.spawn_calls.append(
            (action, config)
        )

        if self.spawn_mode == "fail_no_child":
            raise SpawnFailedError(
                "synthetic Popen failure"
            )

        self.next_pid += 1
        self.active_run_id = action.run_id
        self.pid = self.next_pid

        if self.spawn_mode == "fail_retained":
            raise SpawnFailedError(
                "synthetic config delivery failure"
            )

        return (
            ChildProcessStarted(
                action.run_id,
                self.pid,
            ),
        )

    def stop(self, action, now_sec):
        if (
            action.run_id
            == self.active_run_id
        ):
            self.stop_calls.append(
                (action, now_sec)
            )
        return ()

    def poll(self, now_sec):
        if self.poll_error is not None:
            error = self.poll_error
            self.poll_error = None
            raise error

        if not self.poll_events:
            return ()

        events = tuple(
            self.poll_events.popleft()
        )

        for event in events:
            if isinstance(
                event,
                ChildProcessExited,
            ):
                self.active_run_id = None
                self.pid = None

        return events


class RunIds:
    def __init__(self):
        self.value = 0

    def __call__(self):
        self.value += 1
        return "run-%d" % self.value


def make_runtime(
    *,
    adapter=None,
    config_factory=None,
    startup_timeout_sec=1.0,
):
    if adapter is None:
        adapter = FakeAdapter()

    if config_factory is None:
        config_factory = make_config

    core = RtkManagerCore(
        policy=RestartPolicy(
            startup_timeout_sec=(
                startup_timeout_sec
            ),
            initial_backoff_sec=1.0,
            backoff_multiplier=2.0,
            max_backoff_sec=30.0,
            restart_window_sec=60.0,
            max_restarts_in_window=5,
            stable_run_reset_sec=60.0,
        ),
        run_id_factory=RunIds(),
    )

    runtime = RtkRuntimeOrchestrator(
        core,
        adapter,
        config_factory,
    )

    return runtime, adapter


def test_requires_open_before_mutation():
    runtime, _ = make_runtime()

    with pytest.raises(
        OrchestratorNotOpenError
    ):
        runtime.request_start(0.0)


def test_running_request_waits_for_mavros():
    runtime, adapter = make_runtime()
    runtime.open()

    runtime.request_start(0.0)

    assert (
        runtime.snapshot.manager.desired_state
        is DesiredState.RUNNING
    )
    assert (
        runtime.snapshot.manager.manager_state
        is ManagerState.WAITING_FOR_MAVROS
    )
    assert adapter.spawn_calls == []


def test_mavros_ready_spawns_one_worker():
    runtime, adapter = make_runtime()
    runtime.open()

    runtime.request_start(0.0)

    events = runtime.set_mavros_ready(
        True,
        0.1,
    )

    assert len(adapter.spawn_calls) == 1
    assert isinstance(
        events[0],
        ChildProcessStarted,
    )

    snapshot = runtime.snapshot.manager

    assert snapshot.active_run_id == "run-1"
    assert snapshot.child_started is True
    assert snapshot.child_ready is False
    assert (
        snapshot.manager_state
        is ManagerState.STARTING
    )


def test_started_status_does_not_mean_ready():
    runtime, adapter = make_runtime()
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)
    runtime.request_start(0.1)

    adapter.poll_events.append(
        (
            status(
                "run-1",
                WorkerStatusKind.STARTED,
                "CONFIG_ACCEPTED",
            ),
        )
    )

    runtime.tick(0.2)

    assert (
        runtime.snapshot.manager.manager_state
        is ManagerState.STARTING
    )
    assert runtime.snapshot.manager.child_ready is False


def test_ready_status_enters_running():
    runtime, adapter = make_runtime()
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)
    runtime.request_start(0.1)

    adapter.poll_events.append(
        (
            status(
                "run-1",
                WorkerStatusKind.READY,
                "WORKER_READY",
            ),
        )
    )

    runtime.tick(0.2)

    assert (
        runtime.snapshot.manager.manager_state
        is ManagerState.RUNNING
    )
    assert runtime.snapshot.manager.child_ready is True


def test_mavros_loss_keeps_same_worker():
    runtime, adapter = make_runtime()
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)
    runtime.request_start(0.1)

    adapter.poll_events.append(
        (
            status(
                "run-1",
                WorkerStatusKind.READY,
            ),
        )
    )
    runtime.tick(0.2)

    runtime.set_mavros_ready(
        False,
        0.3,
    )

    assert (
        runtime.snapshot.manager.manager_state
        is ManagerState.RUNNING_MAVROS_STALE
    )
    assert runtime.snapshot.manager.active_run_id == "run-1"
    assert len(adapter.spawn_calls) == 1
    assert adapter.stop_calls == []


def test_mavros_recovery_restores_same_running_worker():
    runtime, adapter = make_runtime()
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)
    runtime.request_start(0.1)

    adapter.poll_events.append(
        (
            status(
                "run-1",
                WorkerStatusKind.READY,
            ),
        )
    )
    runtime.tick(0.2)

    runtime.set_mavros_ready(False, 0.3)
    runtime.set_mavros_ready(True, 0.4)

    assert (
        runtime.snapshot.manager.manager_state
        is ManagerState.RUNNING
    )
    assert runtime.snapshot.manager.active_run_id == "run-1"
    assert len(adapter.spawn_calls) == 1


def test_user_stop_calls_adapter_once():
    runtime, adapter = make_runtime()
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)
    runtime.request_start(0.1)

    runtime.request_stop(0.2)
    runtime.request_stop(0.3)

    assert len(adapter.stop_calls) == 1
    assert (
        runtime.snapshot.manager.manager_state
        is ManagerState.STOPPING
    )


def test_matching_exit_after_user_stop_enters_stopped():
    runtime, adapter = make_runtime()
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)
    runtime.request_start(0.1)
    runtime.request_stop(0.2)

    adapter.poll_events.append(
        (
            ChildProcessExited(
                "run-1",
                WorkerExitReason.CLEAN,
                0,
            ),
        )
    )

    runtime.tick(0.3)

    assert (
        runtime.snapshot.manager.manager_state
        is ManagerState.STOPPED
    )
    assert runtime.snapshot.manager.active_run_id is None


def test_unexpected_exit_enters_backoff():
    runtime, adapter = make_runtime()
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)
    runtime.request_start(0.1)

    adapter.poll_events.append(
        (
            ChildProcessExited(
                "run-1",
                WorkerExitReason.RETRYABLE_FAILURE,
                20,
            ),
        )
    )

    runtime.tick(0.2)

    snapshot = runtime.snapshot.manager

    assert (
        snapshot.manager_state
        is ManagerState.BACKOFF
    )
    assert snapshot.active_run_id is None
    assert snapshot.next_restart_at == pytest.approx(
        1.2
    )


def test_backoff_expiry_spawns_new_run():
    runtime, adapter = make_runtime()
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)
    runtime.request_start(0.1)

    adapter.poll_events.append(
        (
            ChildProcessExited(
                "run-1",
                WorkerExitReason.RETRYABLE_FAILURE,
                20,
            ),
        )
    )

    runtime.tick(0.2)
    runtime.tick(1.199)

    assert len(adapter.spawn_calls) == 1

    runtime.tick(1.2)

    assert len(adapter.spawn_calls) == 2
    assert (
        runtime.snapshot.manager.active_run_id
        == "run-2"
    )


def test_terminal_exit_latches_manager_error():
    runtime, adapter = make_runtime()
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)
    runtime.request_start(0.1)

    adapter.poll_events.append(
        (
            ChildProcessExited(
                "run-1",
                WorkerExitReason.AUTH_FAILED,
                23,
            ),
        )
    )

    runtime.tick(0.2)

    snapshot = runtime.snapshot.manager

    assert (
        snapshot.manager_state
        is ManagerState.ERROR
    )
    assert (
        snapshot.error_reason
        is ErrorReason.AUTH_FAILED
    )


def test_stale_ready_status_is_ignored():
    runtime, adapter = make_runtime()
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)
    runtime.request_start(0.1)

    adapter.poll_events.append(
        (
            status(
                "stale-run",
                WorkerStatusKind.READY,
            ),
        )
    )

    runtime.tick(0.2)

    assert runtime.snapshot.last_worker_status is None
    assert (
        runtime.snapshot.manager.manager_state
        is ManagerState.STARTING
    )


def test_invalid_config_factory_result_latches_config_error():
    runtime, adapter = make_runtime(
        config_factory=lambda _run_id: None
    )
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)

    runtime.request_start(0.1)

    assert adapter.spawn_calls == []
    assert (
        runtime.snapshot.manager.manager_state
        is ManagerState.ERROR
    )
    assert (
        runtime.snapshot.manager.error_reason
        is ErrorReason.CONFIG_INVALID
    )


def test_config_factory_explicit_error_latches_config_error():
    def fail(_run_id):
        raise WorkerConfigBuildError()

    runtime, adapter = make_runtime(
        config_factory=fail
    )
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)

    runtime.request_start(0.1)

    assert adapter.spawn_calls == []
    assert (
        runtime.snapshot.manager.error_reason
        is ErrorReason.CONFIG_INVALID
    )


def test_spawn_failure_without_child_becomes_retryable():
    adapter = FakeAdapter()
    adapter.spawn_mode = "fail_no_child"

    runtime, _ = make_runtime(
        adapter=adapter
    )
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)

    runtime.request_start(0.1)

    snapshot = runtime.snapshot.manager

    assert (
        snapshot.manager_state
        is ManagerState.BACKOFF
    )
    assert snapshot.active_run_id is None


def test_config_delivery_failure_keeps_child_owned_until_reap():
    adapter = FakeAdapter()
    adapter.spawn_mode = "fail_retained"

    runtime, _ = make_runtime(
        adapter=adapter
    )
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)

    runtime.request_start(0.1)

    snapshot = runtime.snapshot.manager

    assert snapshot.active_run_id == "run-1"
    assert snapshot.child_started is True
    assert (
        snapshot.manager_state
        is ManagerState.STARTING
    )

    adapter.poll_events.append(
        (
            ChildProcessExited(
                "run-1",
                WorkerExitReason.RETRYABLE_FAILURE,
                20,
            ),
        )
    )

    runtime.tick(0.2)

    assert (
        runtime.snapshot.manager.manager_state
        is ManagerState.BACKOFF
    )
    assert runtime.snapshot.manager.active_run_id is None


def test_poll_ready_precedes_startup_timeout_at_same_timestamp():
    runtime, adapter = make_runtime(
        startup_timeout_sec=1.0
    )
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)
    runtime.request_start(0.1)

    # Child started at 0.1. READY arrives exactly at elapsed=1.0.
    adapter.poll_events.append(
        (
            status(
                "run-1",
                WorkerStatusKind.READY,
            ),
        )
    )

    runtime.tick(1.1)

    assert (
        runtime.snapshot.manager.manager_state
        is ManagerState.RUNNING
    )
    assert adapter.stop_calls == []


def test_protocol_fault_stops_adapter_child_without_user_stop():
    runtime, adapter = make_runtime()
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)
    runtime.request_start(0.1)

    adapter.poll_error = (
        ProcessAdapterProtocolError(
            "synthetic malformed status"
        )
    )

    runtime.tick(0.2)

    assert len(adapter.stop_calls) == 1
    assert (
        adapter.stop_calls[0][0]
        == StopWorker("run-1")
    )

    assert (
        runtime.snapshot.manager.desired_state
        is DesiredState.RUNNING
    )

    assert (
        runtime.snapshot.last_protocol_fault_run_id
        == "run-1"
    )


def test_terminal_status_is_diagnostic_until_process_exit():
    runtime, adapter = make_runtime()
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)
    runtime.request_start(0.1)

    adapter.poll_events.append(
        (
            status(
                "run-1",
                WorkerStatusKind.TERMINAL_ERROR,
                "AUTH_FAILED",
            ),
        )
    )

    runtime.tick(0.2)

    assert (
        runtime.snapshot.manager.manager_state
        is ManagerState.STARTING
    )
    assert (
        runtime.snapshot.last_worker_status.detail_code
        == "AUTH_FAILED"
    )


def test_snapshot_never_contains_worker_config_or_password():
    runtime, _ = make_runtime()
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)
    runtime.request_start(0.1)

    snapshot = runtime.snapshot

    assert SECRET not in repr(snapshot)
    assert not hasattr(
        snapshot,
        "config",
    )


def test_worker_config_validation_error_latches_config_error():
    def invalid_config(run_id):
        return make_config(
            run_id,
            healthy_age_sec=5.0,
            stale_reconnect_sec=4.0,
        )

    runtime, adapter = make_runtime(
        config_factory=invalid_config
    )
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)

    runtime.request_start(0.1)

    assert adapter.spawn_calls == []
    assert runtime.snapshot.manager.active_run_id is None
    assert (
        runtime.snapshot.manager.manager_state
        is ManagerState.ERROR
    )
    assert (
        runtime.snapshot.manager.error_reason
        is ErrorReason.CONFIG_INVALID
    )


def test_unexpected_config_factory_failure_is_retryable_not_stuck():
    def failing_factory(_run_id):
        raise RuntimeError(
            "synthetic config source failure"
        )

    runtime, adapter = make_runtime(
        config_factory=failing_factory
    )
    runtime.open()
    runtime.set_mavros_ready(True, 0.0)

    runtime.request_start(0.1)

    assert adapter.spawn_calls == []

    snapshot = runtime.snapshot.manager

    assert snapshot.active_run_id is None
    assert (
        snapshot.manager_state
        is ManagerState.BACKOFF
    )
    assert snapshot.next_restart_at == pytest.approx(
        1.1
    )
