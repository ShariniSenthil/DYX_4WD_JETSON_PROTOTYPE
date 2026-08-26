"""Real-process integration tests for adapter -> production RTK bootstrap."""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import pytest

from rover_backend.rtk_manager_core import (
    SpawnWorker,
    StopWorker,
    WorkerExitReason,
)
from rover_backend.rtk_process_adapter import (
    ChildProcessExited,
    ChildStatusReceived,
    RtkProcessAdapter,
)
from rover_backend.rtk_process_protocol import (
    WORKER_CONFIG_SCHEMA_VERSION,
    AdvisoryFileLock,
    OwnershipConflictError,
    WorkerConfig,
    WorkerStatusKind,
)


SECRET = "SUPER_SECRET_RTK_PASSWORD_93a7"


def make_config(run_id: str = "run-A") -> WorkerConfig:
    return WorkerConfig(
        schema_version=WORKER_CONFIG_SCHEMA_VERSION,
        run_id=run_id,
        caster_host="caster.example.test",
        caster_port=2101,
        mountpoint="ROVER_RTCM3",
        username="rover",
        password=SECRET,
        rtcm_topic="/mavros/gps_rtk/send_rtcm",
        connect_timeout_sec=5.0,
        socket_timeout_sec=1.0,
        healthy_age_sec=3.0,
        stale_reconnect_sec=10.0,
        reconnect_delay_sec=2.0,
        first_data_timeout_sec=8.0,
        max_mavros_rtcm_frame_bytes=720,
    )


def install_fake_ros_runtime(
    tmp_path: Path,
    monkeypatch,
) -> Path:
    """Install child-only fake ROS/correction modules through PYTHONPATH."""

    fake_root = tmp_path / "fake-runtime"
    package = fake_root / "rtk_correction_bridge"
    package.mkdir(parents=True)

    (package / "__init__.py").write_text("")

    (fake_root / "rclpy.py").write_text(
        """
_running = True


def init(args=None):
    global _running
    _running = True


def ok():
    return _running


def shutdown():
    global _running
    _running = False
"""
    )

    (package / "ntrip_failures.py").write_text(
        """
class NtripAuthError(ConnectionError):
    pass


class NtripMountpointRejectedError(ConnectionError):
    pass
"""
    )

    (package / "ntrip_to_px4_node.py").write_text(
        """
import time


class NtripToPx4Node:
    def __init__(self, worker_config=None):
        if worker_config is None:
            raise RuntimeError("worker_config was not injected")
        self.worker_config = worker_config

    def run(self):
        while True:
            time.sleep(0.05)

    def destroy_node(self):
        pass
"""
    )

    rover_backend_src = (
        Path(__file__).resolve().parents[2]
    )

    previous = os.environ.get(
        "PYTHONPATH",
        "",
    )

    entries = [
        str(fake_root),
        str(rover_backend_src),
    ]

    if previous:
        entries.append(previous)

    monkeypatch.setenv(
        "PYTHONPATH",
        os.pathsep.join(entries),
    )

    return fake_root


def make_adapter(
    tmp_path: Path,
) -> tuple[RtkProcessAdapter, Path]:
    injection_lock = (
        tmp_path / "worker-injection.lock"
    )

    adapter = RtkProcessAdapter(
        [
            sys.executable,
            "-m",
            "rover_backend.rtk_worker_bootstrap",
            "--injection-lock-path",
            str(injection_lock),
        ],
        manager_lock_path=(
            tmp_path / "manager.lock"
        ),
        stop_grace_sec=0.5,
    )

    adapter.open()

    return adapter, injection_lock


def collect_until(
    adapter: RtkProcessAdapter,
    now: float,
    predicate,
    timeout_sec: float = 3.0,
):
    deadline = (
        time.monotonic()
        + timeout_sec
    )

    collected = []

    while time.monotonic() < deadline:
        events = adapter.poll(now)
        collected.extend(events)

        if predicate(collected):
            return collected, now

        now += 0.01
        time.sleep(0.01)

    raise AssertionError(
        "timed out waiting for RTK worker event"
    )


def wait_for_ready(
    adapter: RtkProcessAdapter,
):
    events, now = collect_until(
        adapter,
        0.0,
        lambda items: any(
            isinstance(
                item,
                ChildStatusReceived,
            )
            and (
                item.event.kind
                is WorkerStatusKind.READY
            )
            for item in items
        ),
    )

    statuses = [
        item.event
        for item in events
        if isinstance(
            item,
            ChildStatusReceived,
        )
    ]

    assert [
        status.kind
        for status in statuses
    ] == [
        WorkerStatusKind.STARTED,
        WorkerStatusKind.READY,
    ]

    assert all(
        status.run_id == "run-A"
        for status in statuses
    )

    assert SECRET not in repr(events)

    return now


def test_real_adapter_bootstrap_stop_lifecycle(
    tmp_path,
    monkeypatch,
):
    install_fake_ros_runtime(
        tmp_path,
        monkeypatch,
    )

    adapter, injection_path = (
        make_adapter(tmp_path)
    )

    try:
        started = adapter.spawn(
            SpawnWorker("run-A"),
            make_config("run-A"),
        )

        assert SECRET not in repr(started)
        assert SECRET not in repr(
            adapter.worker_command
        )

        now = wait_for_ready(adapter)

        contender = AdvisoryFileLock(
            injection_path
        )

        try:
            with pytest.raises(
                OwnershipConflictError
            ):
                contender.acquire_nonblocking()
        finally:
            contender.close()

        adapter.stop(
            StopWorker("run-A"),
            now + 0.01,
        )

        events, _ = collect_until(
            adapter,
            now + 0.02,
            lambda items: any(
                isinstance(
                    item,
                    ChildProcessExited,
                )
                for item in items
            ),
        )

        exit_event = next(
            item
            for item in events
            if isinstance(
                item,
                ChildProcessExited,
            )
        )

        assert (
            exit_event.reason
            is WorkerExitReason.RETRYABLE_FAILURE
        )

        assert adapter.process is None
        assert adapter.active_run_id is None
        assert (
            adapter.liveness_write_fd
            is None
        )
        assert (
            adapter.status_read_fd
            is None
        )

        replacement = AdvisoryFileLock(
            injection_path
        )

        try:
            replacement.acquire_nonblocking()
            assert replacement.locked is True
        finally:
            replacement.close()

    finally:
        if adapter.process is not None:
            adapter.stop(
                StopWorker(
                    adapter.active_run_id
                ),
                100.0,
            )

            collect_until(
                adapter,
                100.5,
                lambda items: any(
                    isinstance(
                        item,
                        ChildProcessExited,
                    )
                    for item in items
                ),
            )

        adapter.close()


def test_real_bootstrap_exits_on_parent_liveness_eof(
    tmp_path,
    monkeypatch,
):
    install_fake_ros_runtime(
        tmp_path,
        monkeypatch,
    )

    adapter, _ = make_adapter(tmp_path)

    try:
        adapter.spawn(
            SpawnWorker("run-A"),
            make_config("run-A"),
        )

        now = wait_for_ready(adapter)

        adapter._close_liveness_writer()

        events, _ = collect_until(
            adapter,
            now + 0.01,
            lambda items: any(
                isinstance(
                    item,
                    ChildProcessExited,
                )
                for item in items
            ),
        )

        exit_event = next(
            item
            for item in events
            if isinstance(
                item,
                ChildProcessExited,
            )
        )

        assert exit_event.returncode == 20

        assert (
            exit_event.reason
            is WorkerExitReason.RETRYABLE_FAILURE
        )

        assert adapter.process is None
        assert adapter.active_run_id is None

    finally:
        if adapter.process is not None:
            adapter.stop(
                StopWorker(
                    adapter.active_run_id
                ),
                100.0,
            )

            collect_until(
                adapter,
                100.5,
                lambda items: any(
                    isinstance(
                        item,
                        ChildProcessExited,
                    )
                    for item in items
                ),
            )

        adapter.close()


def test_real_bootstrap_injection_lock_conflict_is_terminal(
    tmp_path,
    monkeypatch,
):
    install_fake_ros_runtime(
        tmp_path,
        monkeypatch,
    )

    adapter, injection_path = (
        make_adapter(tmp_path)
    )

    existing = AdvisoryFileLock(
        injection_path
    )

    existing.acquire_nonblocking()

    try:
        adapter.spawn(
            SpawnWorker("run-A"),
            make_config("run-A"),
        )

        events, _ = collect_until(
            adapter,
            0.0,
            lambda items: any(
                isinstance(
                    item,
                    ChildProcessExited,
                )
                for item in items
            ),
        )

        terminal = [
            item.event
            for item in events
            if isinstance(
                item,
                ChildStatusReceived,
            )
            and (
                item.event.kind
                is WorkerStatusKind.TERMINAL_ERROR
            )
        ]

        assert len(terminal) == 1

        assert (
            terminal[0].detail_code
            == "OWNERSHIP_CONFLICT"
        )

        exit_event = next(
            item
            for item in events
            if isinstance(
                item,
                ChildProcessExited,
            )
        )

        assert exit_event.returncode == 22

        assert (
            exit_event.reason
            is WorkerExitReason.OWNERSHIP_CONFLICT
        )

        assert adapter.process is None
        assert adapter.active_run_id is None

        assert SECRET not in repr(events)

    finally:
        existing.close()

        if adapter.process is not None:
            adapter.stop(
                StopWorker(
                    adapter.active_run_id
                ),
                100.0,
            )

            collect_until(
                adapter,
                100.5,
                lambda items: any(
                    isinstance(
                        item,
                        ChildProcessExited,
                    )
                    for item in items
                ),
            )

        adapter.close()
