"""Tests for the production RTK worker bootstrap without ROS."""

from __future__ import annotations

import os
import sys
import threading
import time
import types

import pytest

from rover_backend.rtk_process_protocol import (
    WORKER_CONFIG_SCHEMA_VERSION,
    AdvisoryFileLock,
    WorkerConfig,
    WorkerExitCode,
    WorkerStatusKind,
    decode_worker_status,
    encode_worker_config,
    encode_worker_status,
)
from rover_backend.rtk_worker_bootstrap import (
    ParentLivenessGuard,
    _run_ros_runtime,
    run_worker,
)


SECRET = "SUPER_SECRET_RTK_PASSWORD_93a7"


def make_config(run_id="run-A"):
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
        gga_enabled=False,
        gga_interval_sec=10.0,
        gga_max_age_sec=5.0,
        max_mavros_rtcm_frame_bytes=720,
    )


def read_all(fd):
    chunks = []
    while True:
        chunk = os.read(fd, 4096)
        if not chunk:
            return b"".join(chunks)
        chunks.append(chunk)


def decode_frames(payload):
    frames = []
    start = 0

    while start < len(payload):
        end = payload.find(b"\n", start)
        assert end >= 0
        frame = payload[start:end + 1]
        frames.append(
            decode_worker_status(frame)
        )
        start = end + 1

    return frames


def make_worker_pipes(config_payload):
    config_r, config_w = os.pipe()
    live_r, live_w = os.pipe()
    status_r, status_w = os.pipe()

    os.write(config_w, config_payload)
    os.close(config_w)

    return (
        config_r,
        live_r,
        live_w,
        status_r,
        status_w,
    )


def test_started_then_ready_and_clean_exit(tmp_path):
    config = make_config()
    (
        config_r,
        live_r,
        live_w,
        status_r,
        status_w,
    ) = make_worker_pipes(
        encode_worker_config(config)
    )

    def runtime(received_config, ready):
        assert received_config == config
        ready()
        return int(WorkerExitCode.CLEAN)

    try:
        code = run_worker(
            config_fd=config_r,
            liveness_fd=live_r,
            status_fd=status_w,
            injection_lock_path=tmp_path / "injection.lock",
            runtime_entry=runtime,
            parent_eof_callback=lambda: None,
        )

        assert code == int(WorkerExitCode.CLEAN)

        payload = read_all(status_r)
        events = decode_frames(payload)

        assert [event.kind for event in events] == [
            WorkerStatusKind.STARTED,
            WorkerStatusKind.READY,
        ]

        assert all(
            event.run_id == "run-A"
            for event in events
        )

        assert SECRET.encode() not in payload

    finally:
        os.close(live_w)
        os.close(status_r)


def test_config_invalid_maps_to_config_invalid(tmp_path):
    (
        config_r,
        live_r,
        live_w,
        status_r,
        status_w,
    ) = make_worker_pipes(b"{}")

    try:
        code = run_worker(
            config_fd=config_r,
            liveness_fd=live_r,
            status_fd=status_w,
            injection_lock_path=tmp_path / "injection.lock",
            runtime_entry=lambda config, ready: 0,
            parent_eof_callback=lambda: None,
        )

        assert code == int(
            WorkerExitCode.CONFIG_INVALID
        )

        assert read_all(status_r) == b""

    finally:
        os.close(live_w)
        os.close(status_r)


def test_injection_lock_conflict_is_terminal(tmp_path):
    config = make_config()

    (
        config_r,
        live_r,
        live_w,
        status_r,
        status_w,
    ) = make_worker_pipes(
        encode_worker_config(config)
    )

    path = tmp_path / "injection.lock"

    first = AdvisoryFileLock(path)
    first.acquire_nonblocking()

    try:
        code = run_worker(
            config_fd=config_r,
            liveness_fd=live_r,
            status_fd=status_w,
            injection_lock_path=path,
            runtime_entry=lambda config, ready: 0,
            parent_eof_callback=lambda: None,
        )

        assert code == int(
            WorkerExitCode.OWNERSHIP_CONFLICT
        )

        events = decode_frames(
            read_all(status_r)
        )

        assert len(events) == 1
        assert (
            events[0].kind
            is WorkerStatusKind.TERMINAL_ERROR
        )
        assert (
            events[0].detail_code
            == "OWNERSHIP_CONFLICT"
        )

    finally:
        first.close()
        os.close(live_w)
        os.close(status_r)


def test_runtime_failure_maps_retryable(tmp_path):
    config = make_config()

    (
        config_r,
        live_r,
        live_w,
        status_r,
        status_w,
    ) = make_worker_pipes(
        encode_worker_config(config)
    )

    def runtime(config, ready):
        ready()
        raise RuntimeError("synthetic failure")

    try:
        code = run_worker(
            config_fd=config_r,
            liveness_fd=live_r,
            status_fd=status_w,
            injection_lock_path=tmp_path / "injection.lock",
            runtime_entry=runtime,
            parent_eof_callback=lambda: None,
        )

        assert code == int(
            WorkerExitCode.RETRYABLE_FAILURE
        )

        events = decode_frames(
            read_all(status_r)
        )

        assert events[-1].kind is (
            WorkerStatusKind.TERMINAL_ERROR
        )
        assert (
            events[-1].detail_code
            == "RUNTIME_FAILURE"
        )

    finally:
        os.close(live_w)
        os.close(status_r)


def test_injection_lock_released_after_worker_exit(tmp_path):
    config = make_config()

    (
        config_r,
        live_r,
        live_w,
        status_r,
        status_w,
    ) = make_worker_pipes(
        encode_worker_config(config)
    )

    path = tmp_path / "injection.lock"

    try:
        assert run_worker(
            config_fd=config_r,
            liveness_fd=live_r,
            status_fd=status_w,
            injection_lock_path=path,
            runtime_entry=lambda config, ready: 0,
            parent_eof_callback=lambda: None,
        ) == 0

        replacement = AdvisoryFileLock(path)
        try:
            replacement.acquire_nonblocking()
            assert replacement.locked is True
        finally:
            replacement.close()

    finally:
        os.close(live_w)
        os.close(status_r)


def test_parent_liveness_eof_callback():
    read_fd, write_fd = os.pipe()

    seen = threading.Event()

    guard = ParentLivenessGuard(
        read_fd,
        seen.set,
    )

    guard.start()

    os.close(write_fd)

    try:
        assert seen.wait(timeout=1.0)
    finally:
        guard.close()


def test_worker_owned_fds_close_after_run(tmp_path):
    config = make_config()

    (
        config_r,
        live_r,
        live_w,
        status_r,
        status_w,
    ) = make_worker_pipes(
        encode_worker_config(config)
    )

    assert run_worker(
        config_fd=config_r,
        liveness_fd=live_r,
        status_fd=status_w,
        injection_lock_path=tmp_path / "injection.lock",
        runtime_entry=lambda config, ready: 0,
        parent_eof_callback=lambda: None,
    ) == 0

    with pytest.raises(OSError):
        os.fstat(config_r)

    with pytest.raises(OSError):
        os.fstat(live_r)

    with pytest.raises(OSError):
        os.fstat(status_w)

    os.close(live_w)
    os.close(status_r)


@pytest.mark.parametrize(
    "exit_code, detail_code",
    [
        (
            WorkerExitCode.AUTH_FAILED,
            "AUTH_FAILED",
        ),
        (
            WorkerExitCode.MOUNTPOINT_REJECTED,
            "MOUNTPOINT_REJECTED",
        ),
    ],
)
def test_terminal_runtime_result_emits_terminal_status(
    tmp_path,
    exit_code,
    detail_code,
):
    config = make_config()

    (
        config_r,
        live_r,
        live_w,
        status_r,
        status_w,
    ) = make_worker_pipes(
        encode_worker_config(config)
    )

    def runtime(received_config, ready):
        assert received_config == config
        ready()
        return int(exit_code)

    try:
        code = run_worker(
            config_fd=config_r,
            liveness_fd=live_r,
            status_fd=status_w,
            injection_lock_path=(
                tmp_path / "injection.lock"
            ),
            runtime_entry=runtime,
            parent_eof_callback=lambda: None,
        )

        assert code == int(exit_code)

        events = decode_frames(
            read_all(status_r)
        )

        assert [
            event.kind
            for event in events
        ] == [
            WorkerStatusKind.STARTED,
            WorkerStatusKind.READY,
            WorkerStatusKind.TERMINAL_ERROR,
        ]

        assert (
            events[-1].detail_code
            == detail_code
        )

        assert SECRET.encode() not in (
            b"".join(
                encode_worker_status(event)
                for event in events
            )
        )

    finally:
        os.close(live_w)
        os.close(status_r)


@pytest.mark.parametrize(
    "failure_name, expected_code",
    [
        (
            "NtripAuthError",
            WorkerExitCode.AUTH_FAILED,
        ),
        (
            "NtripMountpointRejectedError",
            WorkerExitCode.MOUNTPOINT_REJECTED,
        ),
    ],
)
def test_ros_cleanup_cannot_mask_terminal_exit(
    monkeypatch,
    failure_name,
    expected_code,
):
    package = types.ModuleType(
        "rtk_correction_bridge"
    )
    package.__path__ = []

    failures = types.ModuleType(
        "rtk_correction_bridge.ntrip_failures"
    )

    class NtripAuthError(ConnectionError):
        pass

    class NtripMountpointRejectedError(
        ConnectionError
    ):
        pass

    failures.NtripAuthError = (
        NtripAuthError
    )
    failures.NtripMountpointRejectedError = (
        NtripMountpointRejectedError
    )

    node_module = types.ModuleType(
        "rtk_correction_bridge.ntrip_to_px4_node"
    )

    failure_class = getattr(
        failures,
        failure_name,
    )

    class FakeNode:
        def __init__(
            self,
            worker_config=None,
        ):
            self.worker_config = worker_config

        def run(self):
            raise failure_class(
                "synthetic terminal failure"
            )

        def destroy_node(self):
            raise RuntimeError(
                "synthetic destroy failure"
            )

    node_module.NtripToPx4Node = FakeNode

    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.init = lambda args=None: None
    fake_rclpy.ok = lambda: True

    def failing_shutdown():
        raise RuntimeError(
            "synthetic shutdown failure"
        )

    fake_rclpy.shutdown = failing_shutdown

    monkeypatch.setitem(
        sys.modules,
        "rclpy",
        fake_rclpy,
    )
    monkeypatch.setitem(
        sys.modules,
        "rtk_correction_bridge",
        package,
    )
    monkeypatch.setitem(
        sys.modules,
        "rtk_correction_bridge.ntrip_failures",
        failures,
    )
    monkeypatch.setitem(
        sys.modules,
        "rtk_correction_bridge.ntrip_to_px4_node",
        node_module,
    )

    result = _run_ros_runtime(
        make_config(),
        lambda: None,
    )

    assert result == int(expected_code)


def test_lock_filesystem_error_is_retryable(
    tmp_path,
    monkeypatch,
):
    config = make_config()

    (
        config_r,
        live_r,
        live_w,
        status_r,
        status_w,
    ) = make_worker_pipes(
        encode_worker_config(config)
    )

    def fail_lock(self):
        raise PermissionError(
            "synthetic permission failure"
        )

    monkeypatch.setattr(
        "rover_backend.rtk_worker_bootstrap."
        "AdvisoryFileLock.acquire_nonblocking",
        fail_lock,
    )

    try:
        code = run_worker(
            config_fd=config_r,
            liveness_fd=live_r,
            status_fd=status_w,
            injection_lock_path=(
                tmp_path / "injection.lock"
            ),
            runtime_entry=lambda config, ready: 0,
            parent_eof_callback=lambda: None,
        )

        assert code == int(
            WorkerExitCode.RETRYABLE_FAILURE
        )

        # Failure happened before STARTED and is not a
        # terminal configuration condition.
        assert read_all(status_r) == b""

    finally:
        os.close(live_w)
        os.close(status_r)
