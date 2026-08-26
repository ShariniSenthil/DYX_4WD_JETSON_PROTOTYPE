"""Tests for the backend-owned POSIX RTK subprocess adapter."""

from __future__ import annotations

import math
import os
import random
import sys
import time
from pathlib import Path

import pytest

import rover_backend.rtk_process_adapter as adapter_module
from rover_backend.rtk_manager_core import (
    SpawnWorker,
    StopWorker,
    WorkerExitReason,
)
from rover_backend.rtk_process_adapter import (
    ActiveChildError,
    AdapterNotOpenError,
    ChildProcessExited,
    ChildProcessStarted,
    ChildStatusReceived,
    ProcessAdapterError,
    ProcessAdapterProtocolError,
    RtkProcessAdapter,
    SpawnFailedError,
)
from rover_backend.rtk_process_protocol import (
    MAX_WORKER_CONFIG_BYTES,
    MAX_WORKER_STATUS_BYTES,
    WORKER_CONFIG_SCHEMA_VERSION,
    WORKER_STATUS_SCHEMA_VERSION,
    OwnershipConflictError,
    WorkerConfig,
    WorkerStatusEvent,
    WorkerStatusKind,
    decode_worker_config,
    encode_worker_status,
    read_bounded_fd,
)


SECRET = "SUPER_SECRET_RTK_PASSWORD_93a7"
HELPER = Path(__file__).parent / "helpers" / "rtk_test_child.py"


def make_config(run_id: str = "run-A", **overrides) -> WorkerConfig:
    values = {
        "schema_version": WORKER_CONFIG_SCHEMA_VERSION,
        "run_id": run_id,
        "caster_host": "caster.example.test",
        "caster_port": 2101,
        "mountpoint": "ROVER_RTCM3",
        "username": "rover",
        "password": SECRET,
        "rtcm_topic": "/mavros/rtcm/send",
        "connect_timeout_sec": 5.0,
        "socket_timeout_sec": 10.0,
        "healthy_age_sec": 3.0,
        "stale_reconnect_sec": 15.0,
        "reconnect_delay_sec": 2.0,
        "first_data_timeout_sec": 12.0,
        "max_mavros_rtcm_frame_bytes": 720,
    }
    values.update(overrides)
    return WorkerConfig(**values)


def make_status(
    run_id: str = "run-A",
    kind: WorkerStatusKind = WorkerStatusKind.STARTED,
    detail_code: str | None = None,
) -> bytes:
    return encode_worker_status(
        WorkerStatusEvent(
            WORKER_STATUS_SCHEMA_VERSION,
            run_id,
            kind,
            detail_code,
        )
    )


def close_fd(fd: int | None) -> None:
    if fd is None:
        return
    try:
        os.close(fd)
    except OSError:
        pass


class FakeProcess:
    """Small Popen-compatible process with duplicated synthetic child ends."""

    next_pid = 41000

    def __init__(
        self,
        child_fds: tuple[int, int, int],
        *,
        terminate_exits: bool = False,
        kill_exits: bool = True,
    ) -> None:
        FakeProcess.next_pid += 1
        self.pid = FakeProcess.next_pid
        self.returncode: int | None = None
        self.terminate_calls = 0
        self.kill_calls = 0
        self.poll_calls = 0
        self.terminate_exits = terminate_exits
        self.kill_exits = kill_exits
        self.config_reader = os.dup(child_fds[0])
        self.liveness_reader = os.dup(child_fds[1])
        self.status_writer = os.dup(child_fds[2])

    def poll(self) -> int | None:
        self.poll_calls += 1
        return self.returncode

    def terminate(self) -> None:
        self.terminate_calls += 1
        if self.terminate_exits:
            self.returncode = -15

    def kill(self) -> None:
        self.kill_calls += 1
        if self.kill_exits:
            self.returncode = -9

    def close_child_fds(self) -> None:
        close_fd(self.config_reader)
        close_fd(self.liveness_reader)
        close_fd(self.status_writer)
        self.config_reader = None
        self.liveness_reader = None
        self.status_writer = None


class RecordingPopen:
    def __init__(
        self,
        *,
        terminate_exits: bool = False,
        kill_exits: bool = True,
    ) -> None:
        self.calls: list[tuple[list[str], dict]] = []
        self.processes: list[FakeProcess] = []
        self.terminate_exits = terminate_exits
        self.kill_exits = kill_exits

    def __call__(self, argv, **kwargs) -> FakeProcess:
        self.calls.append((list(argv), dict(kwargs)))
        process = FakeProcess(
            kwargs["pass_fds"],
            terminate_exits=self.terminate_exits,
            kill_exits=self.kill_exits,
        )
        self.processes.append(process)
        return process


class Harness:
    def __init__(
        self,
        tmp_path: Path,
        *,
        terminate_exits: bool = False,
        kill_exits: bool = True,
        stop_grace_sec: float = 5.0,
    ) -> None:
        self.popen = RecordingPopen(
            terminate_exits=terminate_exits,
            kill_exits=kill_exits,
        )
        self.adapter = RtkProcessAdapter(
            [sys.executable, "synthetic-test-child"],
            manager_lock_path=tmp_path / "manager.lock",
            stop_grace_sec=stop_grace_sec,
            popen_factory=self.popen,
        )
        self.adapter.open()

    def spawn(self, run_id: str = "run-A"):
        return self.adapter.spawn(SpawnWorker(run_id), make_config(run_id))

    @property
    def process(self) -> FakeProcess:
        return self.popen.processes[-1]

    def emit(self, payload: bytes) -> None:
        os.write(self.process.status_writer, payload)

    def cleanup(self) -> None:
        for process in self.popen.processes:
            process.close_child_fds()
        if self.adapter.process is not None:
            self.adapter._clear_reaped_child()
        self.adapter.close()


@pytest.fixture
def harness(tmp_path):
    value = Harness(tmp_path)
    try:
        yield value
    finally:
        value.cleanup()


def wait_for_events(
    adapter: RtkProcessAdapter,
    now_sec: float,
    predicate,
    timeout_sec: float = 3.0,
):
    deadline = time.monotonic() + timeout_sec
    collected = []
    now = now_sec
    while time.monotonic() < deadline:
        events = adapter.poll(now)
        collected.extend(events)
        if predicate(collected):
            return collected, now
        now += 0.01
        time.sleep(0.01)
    raise AssertionError("timed out waiting for synthetic child events")


# ---------------------------------------------------------------------------
# 1-3. Authority
# ---------------------------------------------------------------------------


def test_01_adapter_authority_lock_acquired(tmp_path):
    adapter = RtkProcessAdapter(
        [sys.executable, "child"], manager_lock_path=tmp_path / "manager.lock"
    )
    assert adapter.is_open is False
    assert adapter.open() is adapter
    assert adapter.open() is adapter
    assert adapter.is_open is True
    adapter.close()


def test_02_second_adapter_authority_conflicts(tmp_path):
    path = tmp_path / "manager.lock"
    first = RtkProcessAdapter([sys.executable, "child"], manager_lock_path=path)
    second = RtkProcessAdapter([sys.executable, "child"], manager_lock_path=path)
    first.open()
    try:
        with pytest.raises(OwnershipConflictError):
            second.open()
    finally:
        first.close()
        second.close()


def test_03_authority_released_after_close(tmp_path):
    path = tmp_path / "manager.lock"
    first = RtkProcessAdapter([sys.executable, "child"], manager_lock_path=path)
    second = RtkProcessAdapter([sys.executable, "child"], manager_lock_path=path)
    first.open()
    first.close()
    second.open()
    assert second.is_open is True
    second.close()


# ---------------------------------------------------------------------------
# 4-14. Spawn
# ---------------------------------------------------------------------------


def test_04_spawn_creates_exactly_three_pipes(tmp_path, monkeypatch):
    real_pipe = os.pipe
    calls = []

    def recording_pipe():
        result = real_pipe()
        calls.append(result)
        return result

    monkeypatch.setattr(adapter_module.os, "pipe", recording_pipe)
    value = Harness(tmp_path)
    try:
        value.spawn()
        assert len(calls) == 3
        assert len({fd for pair in calls for fd in pair}) == 6
    finally:
        value.cleanup()


def test_05_pass_fds_contains_child_ends_only(harness):
    harness.spawn()
    _, kwargs = harness.popen.calls[0]
    config_read, liveness_read, status_write = kwargs["pass_fds"]
    assert len(set(kwargs["pass_fds"])) == 3
    assert config_read != harness.adapter.liveness_write_fd
    assert liveness_read != harness.adapter.status_read_fd
    assert status_write != harness.adapter.status_read_fd


def test_06_popen_close_fds_true(harness):
    harness.spawn()
    assert harness.popen.calls[0][1]["close_fds"] is True


def test_07_shell_is_not_used(harness):
    harness.spawn()
    assert "shell" not in harness.popen.calls[0][1]


def test_08_password_absent_from_argv(harness):
    harness.spawn()
    argv = harness.popen.calls[0][0]
    assert SECRET not in repr(argv)
    assert argv[-6::2] == ["--config-fd", "--liveness-fd", "--status-fd"]


def test_09_password_absent_from_environment_additions(harness):
    harness.spawn()
    kwargs = harness.popen.calls[0][1]
    assert kwargs["env"] is None
    assert SECRET not in repr(kwargs)


def test_10_run_id_mismatch_rejected_before_popen(harness):
    with pytest.raises(ProcessAdapterError, match="run_id"):
        harness.adapter.spawn(SpawnWorker("run-B"), make_config("run-A"))
    assert harness.popen.calls == []
    assert harness.adapter.process is None


def test_11_second_spawn_rejected_before_popen(harness):
    harness.spawn()
    with pytest.raises(ActiveChildError):
        harness.adapter.spawn(SpawnWorker("run-B"), make_config("run-B"))
    assert len(harness.popen.calls) == 1


def test_12_popen_success_emits_child_started(harness):
    events = harness.spawn()
    assert events == (ChildProcessStarted("run-A", harness.process.pid),)


def test_13_popen_success_does_not_emit_ready(harness):
    events = harness.spawn()
    assert not any(isinstance(event, ChildStatusReceived) for event in events)
    assert "READY" not in repr(events)


def test_14_active_run_id_installed_once(harness):
    harness.spawn()
    pid = harness.adapter.process.pid
    harness.adapter.poll(0.0)
    assert harness.adapter.active_run_id == "run-A"
    assert harness.adapter.process.pid == pid
    assert len(harness.popen.processes) == 1


# ---------------------------------------------------------------------------
# 15-19. Config pipe
# ---------------------------------------------------------------------------


def test_15_child_receives_full_worker_config(harness):
    harness.spawn()
    payload = read_bounded_fd(harness.process.config_reader)
    assert decode_worker_config(payload) == make_config("run-A")


def test_16_parent_config_writer_closes_after_payload(harness):
    harness.spawn()
    assert harness.adapter.config_write_fd is None
    assert harness.adapter.snapshot().config_writer_open is False


def test_17_child_observes_config_eof(harness):
    harness.spawn()
    payload = read_bounded_fd(harness.process.config_reader)
    assert payload
    assert os.read(harness.process.config_reader, 1) == b""


def test_18_password_travels_only_in_config_payload(harness):
    harness.spawn()
    payload = read_bounded_fd(harness.process.config_reader)
    assert decode_worker_config(payload).password == SECRET
    assert SECRET.encode() in payload
    assert SECRET not in repr(harness.popen.calls[0])


def test_19_parent_retains_no_config_fd_after_delivery(harness):
    harness.spawn()
    snapshot = harness.adapter.snapshot()
    assert snapshot.config_writer_open is False
    assert harness.adapter.config_write_fd is None


# ---------------------------------------------------------------------------
# 20-30. Status stream
# ---------------------------------------------------------------------------


def test_20_started_accepted_for_matching_run(harness):
    harness.spawn()
    harness.emit(make_status())
    events = harness.adapter.poll(0.0)
    assert events[0].event.kind is WorkerStatusKind.STARTED


def test_21_ready_accepted_for_matching_run(harness):
    harness.spawn()
    harness.emit(make_status(kind=WorkerStatusKind.READY))
    events = harness.adapter.poll(0.0)
    assert events[0].event.kind is WorkerStatusKind.READY


def test_22_wrong_run_started_rejected(harness):
    harness.spawn()
    harness.emit(make_status("run-B"))
    assert harness.adapter.poll(0.0) == ()
    assert harness.adapter.active_run_id == "run-A"


def test_23_wrong_run_ready_rejected(harness):
    harness.spawn()
    harness.emit(make_status("run-B", WorkerStatusKind.READY))
    assert harness.adapter.poll(0.0) == ()
    assert harness.adapter.active_run_id == "run-A"


def test_24_fragmented_frame_reconstructed(harness):
    harness.spawn()
    payload = make_status(kind=WorkerStatusKind.READY)
    split = len(payload) // 2
    harness.emit(payload[:split])
    assert harness.adapter.poll(0.0) == ()
    harness.emit(payload[split:])
    events = harness.adapter.poll(0.0)
    assert events[0].event.kind is WorkerStatusKind.READY


def test_25_two_frames_in_one_read_are_separate(harness):
    harness.spawn()
    harness.emit(
        make_status(kind=WorkerStatusKind.STARTED)
        + make_status(kind=WorkerStatusKind.READY)
    )
    events = harness.adapter.poll(0.0)
    assert [event.event.kind for event in events] == [
        WorkerStatusKind.STARTED,
        WorkerStatusKind.READY,
    ]


def test_26_partial_frame_retained_until_complete(harness):
    harness.spawn()
    payload = make_status()
    harness.emit(payload[:-1])
    assert harness.adapter.poll(0.0) == ()
    assert bytes(harness.adapter.status_buffer) == payload[:-1]
    harness.emit(payload[-1:])
    assert len(harness.adapter.poll(0.0)) == 1
    assert harness.adapter.status_buffer == bytearray()


def test_27_oversize_unterminated_frame_rejected(harness):
    harness.spawn()
    harness.emit(b"X" * (MAX_WORKER_STATUS_BYTES + 1))
    with pytest.raises(ProcessAdapterProtocolError, match="unterminated"):
        harness.adapter.poll(0.0)
    assert harness.adapter.active_run_id == "run-A"


def test_28_malformed_frame_rejected(harness):
    harness.spawn()
    harness.emit(b"not-json\n")
    with pytest.raises(ProcessAdapterProtocolError, match="invalid"):
        harness.adapter.poll(0.0)
    assert harness.adapter.active_run_id == "run-A"


def test_29_status_eof_closes_parent_status_fd(harness):
    harness.spawn()
    close_fd(harness.process.status_writer)
    harness.process.status_writer = None
    assert harness.adapter.poll(0.0) == ()
    assert harness.adapter.status_read_fd is None


def test_30_accepted_status_is_credential_free(harness):
    harness.spawn()
    harness.emit(make_status(detail_code="CONFIG_DECODED"))
    event = harness.adapter.poll(0.0)[0]
    assert SECRET not in repr(event)
    assert set(event.event.__slots__) == {
        "schema_version",
        "run_id",
        "kind",
        "detail_code",
    }


# ---------------------------------------------------------------------------
# 31-34. Liveness ownership
# ---------------------------------------------------------------------------


def test_31_liveness_writer_stays_open_while_child_alive(harness):
    harness.spawn()
    fd = harness.adapter.liveness_write_fd
    assert fd is not None
    assert os.fstat(fd)
    harness.adapter.poll(0.0)
    assert harness.adapter.liveness_write_fd == fd


def test_32_child_observes_eof_when_parent_liveness_closes(harness):
    harness.spawn()
    harness.adapter._close_liveness_writer()
    assert os.read(harness.process.liveness_reader, 1) == b""


def test_33_config_and_liveness_pipes_are_distinct(harness):
    harness.spawn()
    child_fds = harness.popen.calls[0][1]["pass_fds"]
    assert child_fds[0] != child_fds[1]
    assert harness.process.config_reader != harness.process.liveness_reader


def test_34_status_and_liveness_pipes_are_distinct(harness):
    harness.spawn()
    child_fds = harness.popen.calls[0][1]["pass_fds"]
    assert child_fds[1] != child_fds[2]
    assert harness.adapter.status_read_fd != harness.adapter.liveness_write_fd


# ---------------------------------------------------------------------------
# 35-42. Stop, TERM, and KILL
# ---------------------------------------------------------------------------


def test_35_wrong_stopworker_ignored(harness):
    assert harness.adapter.stop(StopWorker("run-B"), 0.0) == ()
    harness.spawn()
    assert harness.adapter.stop(StopWorker("run-B"), 1.0) == ()
    assert harness.process.terminate_calls == 0
    assert harness.adapter.stop_requested is False


def test_36_matching_stopworker_terminates_once(harness):
    harness.spawn()
    assert harness.adapter.stop(StopWorker("run-A"), 1.0) == ()
    assert harness.process.terminate_calls == 1
    assert harness.adapter.stop_deadline == 6.0


def test_37_repeated_stopworker_does_not_terminate_twice(harness):
    harness.spawn()
    harness.adapter.stop(StopWorker("run-A"), 1.0)
    harness.adapter.stop(StopWorker("run-A"), 2.0)
    assert harness.process.terminate_calls == 1
    assert harness.adapter.stop_deadline == 6.0


def test_38_before_grace_deadline_does_not_kill(harness):
    harness.spawn()
    harness.adapter.stop(StopWorker("run-A"), 1.0)
    harness.adapter.poll(5.999)
    assert harness.process.kill_calls == 0


def test_39_exact_grace_deadline_kills_live_child(harness):
    harness.spawn()
    harness.adapter.stop(StopWorker("run-A"), 1.0)
    events = harness.adapter.poll(6.0)
    assert harness.process.kill_calls == 1
    assert isinstance(events[-1], ChildProcessExited)


def test_40_repeated_poll_does_not_duplicate_kill(tmp_path):
    value = Harness(tmp_path, kill_exits=False)
    try:
        value.spawn()
        value.adapter.stop(StopWorker("run-A"), 1.0)
        value.adapter.poll(6.0)
        value.adapter.poll(7.0)
        assert value.process.kill_calls == 1
        assert value.adapter.kill_sent is True
    finally:
        value.cleanup()


def test_41_real_cooperative_child_end_to_end(tmp_path):
    adapter = RtkProcessAdapter(
        [sys.executable, str(HELPER), "--status-mode", "combined"],
        manager_lock_path=tmp_path / "manager.lock",
        stop_grace_sec=0.5,
    )
    adapter.open()
    started = adapter.spawn(SpawnWorker("run-A"), make_config("run-A"))
    assert isinstance(started[0], ChildProcessStarted)
    statuses, now = wait_for_events(
        adapter,
        0.0,
        lambda events: any(
            isinstance(event, ChildStatusReceived)
            and event.event.kind is WorkerStatusKind.READY
            for event in events
        ),
    )
    assert [
        event.event.kind
        for event in statuses
        if isinstance(event, ChildStatusReceived)
    ] == [WorkerStatusKind.STARTED, WorkerStatusKind.READY]
    adapter.stop(StopWorker("run-A"), now + 0.01)
    exited, _ = wait_for_events(
        adapter,
        now + 0.02,
        lambda events: any(isinstance(event, ChildProcessExited) for event in events),
    )
    assert any(isinstance(event, ChildProcessExited) for event in exited)
    assert adapter.process is None
    assert adapter.active_run_id is None
    assert adapter.snapshot().liveness_writer_open is False
    adapter.close()
    assert adapter.is_open is False


def test_42_real_sigterm_ignoring_child_is_sigkilled(tmp_path):
    adapter = RtkProcessAdapter(
        [sys.executable, str(HELPER), "--ignore-sigterm"],
        manager_lock_path=tmp_path / "manager.lock",
        stop_grace_sec=0.05,
    )
    adapter.open()
    adapter.spawn(SpawnWorker("run-A"), make_config("run-A"))
    _, now = wait_for_events(
        adapter,
        0.0,
        lambda events: any(
            isinstance(event, ChildStatusReceived)
            and event.event.kind is WorkerStatusKind.READY
            for event in events
        ),
    )
    adapter.stop(StopWorker("run-A"), now + 1.0)
    assert adapter.poll(now + 1.049) == ()
    events, _ = wait_for_events(
        adapter,
        now + 1.05,
        lambda items: any(isinstance(item, ChildProcessExited) for item in items),
    )
    exit_event = next(item for item in events if isinstance(item, ChildProcessExited))
    assert exit_event.returncode < 0
    assert exit_event.reason is WorkerExitReason.RETRYABLE_FAILURE
    adapter.close()


# ---------------------------------------------------------------------------
# 43-55. Poll/reap and central exit mapping
# ---------------------------------------------------------------------------


def test_43_process_exit_is_reported_exactly_once(harness):
    harness.spawn()
    harness.process.returncode = 0
    first = harness.adapter.poll(0.0)
    second = harness.adapter.poll(0.0)
    assert sum(isinstance(event, ChildProcessExited) for event in first) == 1
    assert second == ()


def test_44_exit_zero_maps_clean(harness):
    harness.spawn()
    harness.process.returncode = 0
    assert harness.adapter.poll(0.0)[-1].reason is WorkerExitReason.CLEAN


def test_45_exit_20_maps_retryable_failure(harness):
    harness.spawn()
    harness.process.returncode = 20
    assert (
        harness.adapter.poll(0.0)[-1].reason
        is WorkerExitReason.RETRYABLE_FAILURE
    )


def test_46_exit_21_maps_config_invalid(harness):
    harness.spawn()
    harness.process.returncode = 21
    assert harness.adapter.poll(0.0)[-1].reason is WorkerExitReason.CONFIG_INVALID


def test_47_exit_22_maps_ownership_conflict(harness):
    harness.spawn()
    harness.process.returncode = 22
    assert (
        harness.adapter.poll(0.0)[-1].reason
        is WorkerExitReason.OWNERSHIP_CONFLICT
    )


def test_48_exit_23_maps_auth_failed(harness):
    harness.spawn()
    harness.process.returncode = 23
    assert harness.adapter.poll(0.0)[-1].reason is WorkerExitReason.AUTH_FAILED


def test_49_exit_24_maps_mountpoint_rejected(harness):
    harness.spawn()
    harness.process.returncode = 24
    assert (
        harness.adapter.poll(0.0)[-1].reason
        is WorkerExitReason.MOUNTPOINT_REJECTED
    )


def test_50_unknown_nonzero_maps_retryable_failure(harness):
    harness.spawn()
    harness.process.returncode = 77
    assert (
        harness.adapter.poll(0.0)[-1].reason
        is WorkerExitReason.RETRYABLE_FAILURE
    )


def test_51_signal_returncode_maps_retryable_failure(harness):
    harness.spawn()
    harness.process.returncode = -15
    assert (
        harness.adapter.poll(0.0)[-1].reason
        is WorkerExitReason.RETRYABLE_FAILURE
    )


def test_52_repeated_poll_does_not_duplicate_exit(harness):
    harness.spawn()
    harness.process.returncode = 0
    assert isinstance(harness.adapter.poll(0.0)[-1], ChildProcessExited)
    assert harness.adapter.poll(0.0) == ()
    assert harness.adapter.poll(1.0) == ()


def test_53_all_child_resources_cleared_after_reap(harness):
    harness.spawn()
    harness.process.returncode = 0
    harness.adapter.poll(0.0)
    snapshot = harness.adapter.snapshot()
    assert snapshot.active_run_id is None
    assert snapshot.pid is None
    assert snapshot.status_buffer_bytes == 0
    assert snapshot.stop_requested is False
    assert snapshot.exit_reported is True


def test_54_liveness_fd_closed_after_reap(harness):
    harness.spawn()
    fd = harness.adapter.liveness_write_fd
    harness.process.returncode = 0
    harness.adapter.poll(0.0)
    assert harness.adapter.liveness_write_fd is None
    with pytest.raises(OSError):
        os.fstat(fd)


def test_55_status_fd_closed_after_reap(harness):
    harness.spawn()
    fd = harness.adapter.status_read_fd
    harness.process.returncode = 0
    harness.adapter.poll(0.0)
    assert harness.adapter.status_read_fd is None
    with pytest.raises(OSError):
        os.fstat(fd)


# ---------------------------------------------------------------------------
# 56-60. Failure cleanup and stale lifecycle isolation
# ---------------------------------------------------------------------------


def test_56_popen_failure_closes_all_six_pipe_fds(tmp_path, monkeypatch):
    real_pipe = os.pipe
    created = []

    def recording_pipe():
        pair = real_pipe()
        created.extend(pair)
        return pair

    def failing_popen(*args, **kwargs):
        raise OSError("synthetic spawn failure")

    monkeypatch.setattr(adapter_module.os, "pipe", recording_pipe)
    adapter = RtkProcessAdapter(
        [sys.executable, "child"],
        manager_lock_path=tmp_path / "manager.lock",
        popen_factory=failing_popen,
    )
    adapter.open()
    with pytest.raises(SpawnFailedError, match="spawn"):
        adapter.spawn(SpawnWorker("run-A"), make_config("run-A"))
    assert len(created) == 6
    for fd in created:
        with pytest.raises(OSError):
            os.fstat(fd)
    adapter.close()


def test_57_popen_failure_leaves_no_active_child(tmp_path):
    def failing_popen(*args, **kwargs):
        raise OSError("synthetic spawn failure")

    adapter = RtkProcessAdapter(
        [sys.executable, "child"],
        manager_lock_path=tmp_path / "manager.lock",
        popen_factory=failing_popen,
    )
    adapter.open()
    with pytest.raises(SpawnFailedError):
        adapter.spawn(SpawnWorker("run-A"), make_config("run-A"))
    assert adapter.process is None
    assert adapter.active_run_id is None
    assert adapter.is_open is True
    adapter.close()


def test_58_config_write_failure_tracks_and_terminates_child(
    tmp_path, monkeypatch
):
    value = Harness(tmp_path)

    def failing_write(fd, payload):
        raise OSError("synthetic config write failure")

    monkeypatch.setattr(adapter_module, "write_all_fd", failing_write)
    try:
        with pytest.raises(SpawnFailedError, match="deliver") as captured:
            value.spawn()
        assert SECRET not in str(captured.value)
        assert value.adapter.process is value.process
        assert value.adapter.active_run_id == "run-A"
        assert value.process.terminate_calls == 1
        assert value.adapter.config_write_fd is None
        value.process.returncode = -15
        value.adapter.poll(0.0)
        assert value.adapter.process is None
    finally:
        value.cleanup()


def test_59_malformed_status_does_not_change_run_ownership(harness):
    harness.spawn()
    process = harness.adapter.process
    harness.emit(b"{}\n")
    with pytest.raises(ProcessAdapterProtocolError):
        harness.adapter.poll(0.0)
    assert harness.adapter.active_run_id == "run-A"
    assert harness.adapter.process is process


def test_60_stale_run_a_status_cannot_mutate_run_b_lifecycle(harness):
    harness.spawn("run-A")
    harness.process.returncode = 0
    harness.adapter.poll(0.0)
    harness.spawn("run-B")
    process_b = harness.process
    harness.emit(make_status("run-A", WorkerStatusKind.READY))
    assert harness.adapter.poll(1.0) == ()
    assert harness.adapter.active_run_id == "run-B"
    assert harness.adapter.process is process_b



def test_oversized_config_is_rejected_before_popen(tmp_path):
    value = Harness(tmp_path)
    try:
        run_id = "r" * MAX_WORKER_CONFIG_BYTES
        with pytest.raises(SpawnFailedError, match="encode"):
            value.adapter.spawn(
                SpawnWorker(run_id),
                make_config(run_id),
            )
        assert value.popen.calls == []
        assert value.adapter.process is None
        assert value.adapter.active_run_id is None
    finally:
        value.cleanup()


def test_oversize_unterminated_status_does_not_poison_reap(harness):
    harness.spawn()
    harness.emit(b"X" * (MAX_WORKER_STATUS_BYTES + 1))

    with pytest.raises(ProcessAdapterProtocolError, match="unterminated"):
        harness.adapter.poll(0.0)

    assert harness.adapter.status_buffer == bytearray()
    assert harness.adapter.active_run_id == "run-A"

    harness.process.returncode = 20
    events = harness.adapter.poll(0.0)

    assert isinstance(events[-1], ChildProcessExited)
    assert events[-1].reason is WorkerExitReason.RETRYABLE_FAILURE
    assert harness.adapter.process is None
    assert harness.adapter.active_run_id is None


def test_status_eof_with_partial_frame_is_rejected_once_then_reapable(harness):
    harness.spawn()
    payload = make_status(kind=WorkerStatusKind.READY)
    harness.emit(payload[:-1])

    close_fd(harness.process.status_writer)
    harness.process.status_writer = None

    with pytest.raises(ProcessAdapterProtocolError, match="partial frame"):
        harness.adapter.poll(0.0)

    assert harness.adapter.status_read_fd is None
    assert harness.adapter.status_buffer == bytearray()
    assert harness.adapter.active_run_id == "run-A"

    harness.process.returncode = 0
    events = harness.adapter.poll(0.0)

    assert isinstance(events[-1], ChildProcessExited)
    assert events[-1].reason is WorkerExitReason.CLEAN
    assert harness.adapter.process is None


def test_real_child_exits_cleanly_on_liveness_eof(tmp_path):
    adapter = RtkProcessAdapter(
        [sys.executable, str(HELPER)],
        manager_lock_path=tmp_path / "manager.lock",
        stop_grace_sec=0.5,
    )
    adapter.open()
    adapter.spawn(SpawnWorker("run-A"), make_config("run-A"))

    _, now = wait_for_events(
        adapter,
        0.0,
        lambda events: any(
            isinstance(event, ChildStatusReceived)
            and event.event.kind is WorkerStatusKind.READY
            for event in events
        ),
    )

    adapter._close_liveness_writer()

    events, _ = wait_for_events(
        adapter,
        now + 0.01,
        lambda items: any(
            isinstance(item, ChildProcessExited)
            for item in items
        ),
    )

    exit_event = next(
        item for item in events
        if isinstance(item, ChildProcessExited)
    )
    assert exit_event.returncode == 0
    assert exit_event.reason is WorkerExitReason.CLEAN
    assert adapter.process is None
    assert adapter.active_run_id is None

    adapter.close()


# ---------------------------------------------------------------------------
# 61-64. Injected time
# ---------------------------------------------------------------------------


def test_61_equal_now_is_accepted(harness):
    harness.spawn()
    harness.adapter.poll(2.0)
    harness.adapter.poll(2.0)
    harness.adapter.stop(StopWorker("run-B"), 2.0)


def test_62_increasing_now_is_accepted(harness):
    harness.spawn()
    harness.adapter.poll(1.0)
    harness.adapter.poll(2.0)
    harness.adapter.stop(StopWorker("run-B"), 3.0)


def test_63_decreasing_now_rejected_before_timer_mutation(harness):
    harness.spawn()
    harness.adapter.stop(StopWorker("run-A"), 5.0)
    deadline = harness.adapter.stop_deadline
    kill_calls = harness.process.kill_calls
    with pytest.raises(ValueError, match="nondecreasing"):
        harness.adapter.poll(4.0)
    assert harness.adapter.stop_deadline == deadline
    assert harness.process.kill_calls == kill_calls


def test_64_nonfinite_time_rejected(harness):
    harness.spawn()
    for now in (math.nan, math.inf, -math.inf):
        with pytest.raises(ValueError, match="finite"):
            harness.adapter.poll(now)
    assert harness.adapter.stop_requested is False


# ---------------------------------------------------------------------------
# 65-68. Secret containment
# ---------------------------------------------------------------------------


def test_65_password_absent_from_worker_command_and_final_argv(harness):
    harness.spawn()
    assert SECRET not in repr(harness.adapter.worker_command)
    assert SECRET not in repr(harness.popen.calls[0][0])


def test_66_password_absent_from_adapter_snapshot_and_repr(harness):
    harness.spawn()
    assert SECRET not in repr(harness.adapter)
    assert SECRET not in repr(harness.adapter.snapshot())
    assert not hasattr(harness.adapter.snapshot(), "worker_command")


def test_67_password_absent_from_all_adapter_events(harness):
    events = list(harness.spawn())
    harness.emit(make_status(kind=WorkerStatusKind.READY))
    events.extend(harness.adapter.poll(0.0))
    harness.process.returncode = 0
    events.extend(harness.adapter.poll(1.0))
    assert SECRET not in repr(events)


def test_68_password_absent_from_ordinary_errors(tmp_path):
    adapter = RtkProcessAdapter(
        [sys.executable, "child"], manager_lock_path=tmp_path / "manager.lock"
    )
    errors = []
    with pytest.raises(AdapterNotOpenError) as not_open:
        adapter.spawn(SpawnWorker("run-A"), make_config("run-A"))
    errors.append(not_open.value)
    adapter.open()
    with pytest.raises(ProcessAdapterError) as mismatch:
        adapter.spawn(SpawnWorker("run-B"), make_config("run-A"))
    errors.append(mismatch.value)
    adapter.close()
    assert SECRET not in repr(errors)


# ---------------------------------------------------------------------------
# 69-70. One-child invariants
# ---------------------------------------------------------------------------


def test_69_at_most_one_active_process(harness):
    harness.spawn("run-A")
    first = harness.adapter.process
    with pytest.raises(ActiveChildError):
        harness.adapter.close()
    with pytest.raises(ActiveChildError):
        harness.spawn("run-B")
    assert harness.adapter.process is first
    assert harness.adapter.active_run_id == "run-A"
    assert len(harness.popen.processes) == 1


def test_70_randomized_mocked_lifecycle_maintains_one_child(tmp_path):
    rng = random.Random(20260826)
    value = Harness(tmp_path, kill_exits=False, stop_grace_sec=0.25)
    now = 0.0
    run_number = 0
    try:
        for _ in range(300):
            now += rng.random() * 0.2
            if value.adapter.process is None:
                if rng.random() < 0.7:
                    run_number += 1
                    run_id = "run-%d" % run_number
                    value.spawn(run_id)
            else:
                process = value.process
                choice = rng.randrange(4)
                if choice == 0:
                    with pytest.raises(ActiveChildError):
                        value.adapter.spawn(
                            SpawnWorker("blocked"), make_config("blocked")
                        )
                elif choice == 1:
                    value.adapter.stop(
                        StopWorker(value.adapter.active_run_id), now
                    )
                elif choice == 2:
                    process.returncode = rng.choice((0, 20, 21, -15))
                value.adapter.poll(now)

            assert (value.adapter.process is None) == (
                value.adapter.active_run_id is None
            )
            assert int(value.adapter.process is not None) <= 1
    finally:
        value.cleanup()


def test_stop_oserror_preserves_supervision_until_kill(harness):
    harness.spawn()

    process = harness.process

    def failing_terminate():
        raise OSError(
            "synthetic SIGTERM failure"
        )

    process.terminate = failing_terminate

    assert (
        harness.adapter.stop(
            StopWorker("run-A"),
            0.0,
        )
        == ()
    )

    snapshot = harness.adapter.snapshot()

    assert snapshot.stop_requested is True
    assert snapshot.stop_deadline == pytest.approx(
        5.0
    )
    assert snapshot.active_run_id == "run-A"

    events = harness.adapter.poll(5.0)

    assert process.kill_calls == 1
    assert any(
        isinstance(
            event,
            ChildProcessExited,
        )
        for event in events
    )
    assert harness.adapter.active_run_id is None


def test_kill_oserror_is_retried_on_later_poll(harness):
    harness.spawn()

    process = harness.process

    harness.adapter.stop(
        StopWorker("run-A"),
        0.0,
    )

    original_kill = process.kill
    attempts = {"count": 0}

    def flaky_kill():
        attempts["count"] += 1

        if attempts["count"] == 1:
            raise OSError(
                "synthetic SIGKILL failure"
            )

        original_kill()

    process.kill = flaky_kill

    assert harness.adapter.poll(5.0) == ()

    snapshot = harness.adapter.snapshot()

    assert attempts["count"] == 1
    assert snapshot.kill_sent is False
    assert snapshot.active_run_id == "run-A"

    events = harness.adapter.poll(5.1)

    assert attempts["count"] == 2
    assert any(
        isinstance(
            event,
            ChildProcessExited,
        )
        for event in events
    )
    assert harness.adapter.active_run_id is None
