"""Tests for the ROS-free RTK process-protocol primitives."""

from __future__ import annotations

import json
import os
from dataclasses import FrozenInstanceError, fields

import pytest

from rover_backend.rtk_manager_core import WorkerExitReason
from rover_backend.rtk_process_protocol import (
    DEFAULT_INJECTION_LOCK_PATH,
    DEFAULT_MANAGER_LOCK_PATH,
    MAX_WORKER_CONFIG_BYTES,
    AdvisoryFileLock,
    ConfigDecodeError,
    ConfigTooLargeError,
    ConfigValidationError,
    FileDescriptorIOError,
    OwnershipConflictError,
    StatusDecodeError,
    StatusValidationError,
    WORKER_CONFIG_SCHEMA_VERSION,
    WORKER_STATUS_SCHEMA_VERSION,
    WorkerConfig,
    WorkerExitCode,
    WorkerStatusEvent,
    WorkerStatusKind,
    decode_worker_config,
    decode_worker_status,
    encode_worker_config,
    encode_worker_status,
    read_bounded_fd,
    worker_exit_reason_from_code,
    write_all_fd,
)


SECRET = "SUPER_SECRET_RTK_PASSWORD_93a7"


def make_config(**overrides) -> WorkerConfig:
    values = {
        "schema_version": WORKER_CONFIG_SCHEMA_VERSION,
        "run_id": "run-001",
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
        "gga_enabled": False,
        "gga_interval_sec": 10.0,
        "gga_max_age_sec": 5.0,
        "max_mavros_rtcm_frame_bytes": 720,
    }
    values.update(overrides)
    return WorkerConfig(**values)


def status_payload(**overrides) -> bytes:
    value = {
        "schema_version": WORKER_STATUS_SCHEMA_VERSION,
        "run_id": "run-001",
        "kind": "STARTED",
        "detail_code": None,
    }
    value.update(overrides)
    return json.dumps(value, separators=(",", ":")).encode("utf-8") + b"\n"


# ---------------------------------------------------------------------------
# 1-18. Worker configuration
# ---------------------------------------------------------------------------


def test_01_valid_config_round_trip():
    config = make_config()
    encoded = encode_worker_config(config)
    assert decode_worker_config(encoded) == config
    decoded_json = json.loads(encoded)
    assert set(decoded_json) == {item.name for item in fields(WorkerConfig)}
    assert decoded_json["password"] == SECRET


def test_02_password_absent_from_repr_and_validation_errors():
    config_repr = repr(make_config())
    assert SECRET not in config_repr
    assert "password=<redacted>" in config_repr
    with pytest.raises(ConfigValidationError) as captured:
        make_config(caster_port=0)
    assert SECRET not in str(captured.value)
    assert SECRET not in repr(captured.value)


def test_03_malformed_json_rejected():
    with pytest.raises(ConfigDecodeError):
        decode_worker_config(b'{"schema_version":1')
    with pytest.raises(ConfigDecodeError):
        decode_worker_config(b'{"schema_version":1,"schema_version":1}')
    with pytest.raises(ConfigDecodeError):
        decode_worker_config(b"\xff")


def test_04_non_object_json_rejected():
    with pytest.raises(ConfigDecodeError):
        decode_worker_config(b"[]")


def test_05_missing_field_rejected():
    value = json.loads(encode_worker_config(make_config()))
    del value["run_id"]
    with pytest.raises(ConfigDecodeError, match="missing"):
        decode_worker_config(json.dumps(value).encode("utf-8"))


def test_06_unknown_field_rejected():
    value = json.loads(encode_worker_config(make_config()))
    value["future_field"] = "unsupported"
    with pytest.raises(ConfigDecodeError, match="unexpected"):
        decode_worker_config(json.dumps(value).encode("utf-8"))


def test_07_wrong_schema_version_rejected():
    value = json.loads(encode_worker_config(make_config()))
    value["schema_version"] = (
        WORKER_CONFIG_SCHEMA_VERSION
        + 1
    )
    with pytest.raises(ConfigDecodeError, match="unsupported"):
        decode_worker_config(json.dumps(value).encode("utf-8"))
    with pytest.raises(ConfigValidationError):
        make_config(schema_version=True)


def test_08_oversized_config_rejected_before_parse_and_on_encode():
    oversized_invalid_json = b"{" + b"x" * MAX_WORKER_CONFIG_BYTES
    with pytest.raises(ConfigTooLargeError):
        decode_worker_config(oversized_invalid_json)
    with pytest.raises(ConfigTooLargeError):
        encode_worker_config(make_config(run_id="r" * MAX_WORKER_CONFIG_BYTES))


def test_09_invalid_run_id_rejected():
    for value in ("", "   ", None, 42):
        with pytest.raises(ConfigValidationError):
            make_config(run_id=value)


def test_10_invalid_port_rejected():
    for value in (0, 65536, 2101.0, "2101"):
        with pytest.raises(ConfigValidationError):
            make_config(caster_port=value)


def test_11_bool_port_rejected():
    with pytest.raises(ConfigValidationError):
        make_config(caster_port=True)


def test_12_invalid_timeout_rejected():
    for value in (0, -0.1, "5", True):
        with pytest.raises(ConfigValidationError):
            make_config(connect_timeout_sec=value)


def test_13_nan_and_infinite_timeout_rejected():
    for value in (float("nan"), float("inf"), float("-inf")):
        with pytest.raises(ConfigValidationError):
            make_config(socket_timeout_sec=value)


def test_14_stale_reconnect_must_exceed_healthy_age():
    with pytest.raises(ConfigValidationError):
        make_config(healthy_age_sec=3.0, stale_reconnect_sec=3.0)
    with pytest.raises(ConfigValidationError):
        make_config(healthy_age_sec=3.0, stale_reconnect_sec=2.0)


def test_15_absolute_rtcm_topic_required():
    for value in ("mavros/rtcm/send", "", "   "):
        with pytest.raises(ConfigValidationError):
            make_config(rtcm_topic=value)
    assert make_config(rtcm_topic="/rtcm").rtcm_topic == "/rtcm"


def test_16_mavros_frame_720_accepted():
    assert make_config(max_mavros_rtcm_frame_bytes=720).max_mavros_rtcm_frame_bytes == 720


def test_17_mavros_frame_1029_accepted():
    config = make_config(max_mavros_rtcm_frame_bytes=1029)
    assert config.max_mavros_rtcm_frame_bytes == 1029


def test_18_mavros_frame_1030_rejected():
    for value in (0, 1030, True, 720.0):
        with pytest.raises(ConfigValidationError):
            make_config(max_mavros_rtcm_frame_bytes=value)


def test_password_significant_spaces_survive_worker_round_trip():
    secret = " secret with spaces "

    config = make_config(
        password=secret
    )

    encoded = encode_worker_config(
        config
    )

    decoded = decode_worker_config(
        encoded
    )

    assert decoded.password == secret


@pytest.mark.parametrize(
    "password",
    (
        "bad\nsecret",
        "bad\rsecret",
        "bad\tsecret",
        "bad\x00secret",
        "bad\x7fsecret",
    ),
)
def test_password_control_characters_rejected_at_worker_boundary(
    password,
):
    with pytest.raises(
        ConfigValidationError
    ):
        make_config(
            password=password
        )


@pytest.mark.parametrize(
    (
        "field_name",
        "field_value",
    ),
    (
        (
            "caster_host",
            "caster.test\r\nInjected: yes",
        ),
        (
            "caster_host",
            "caster test",
        ),
        (
            "mountpoint",
            "MOUNT\nInjected",
        ),
        (
            "mountpoint",
            "MOUNT POINT",
        ),
        (
            "rtcm_topic",
            "/rtcm\r\nInjected",
        ),
        (
            "rtcm_topic",
            "/rtcm topic",
        ),
    ),
)
def test_worker_protocol_tokens_reject_controls_and_whitespace(
    field_name,
    field_value,
):
    with pytest.raises(
        ConfigValidationError
    ):
        make_config(
            **{
                field_name: field_value,
            }
        )


def test_worker_username_control_characters_are_rejected():
    with pytest.raises(
        ConfigValidationError
    ):
        make_config(
            username="user\r\nInjected"
        )


# ---------------------------------------------------------------------------
# 19-28. Bounded file-descriptor I/O
# ---------------------------------------------------------------------------


def test_19_write_all_fd_full_write():
    read_fd, write_fd = os.pipe()
    try:
        write_all_fd(write_fd, b"rtcm-config")
        os.close(write_fd)
        write_fd = -1
        assert os.read(read_fd, 64) == b"rtcm-config"
    finally:
        os.close(read_fd)
        if write_fd >= 0:
            os.close(write_fd)


def test_20_partial_writes_handled(monkeypatch):
    calls = []

    def partial_write(fd, payload):
        calls.append((fd, bytes(payload)))
        return min(2, len(payload))

    monkeypatch.setattr(os, "write", partial_write)
    write_all_fd(17, memoryview(b"abcdef"))
    assert [payload for _, payload in calls] == [b"abcdef", b"cdef", b"ef"]


def test_21_interrupted_write_retried(monkeypatch):
    attempts = 0

    def interrupted_once(fd, payload):
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise InterruptedError
        return len(payload)

    monkeypatch.setattr(os, "write", interrupted_once)
    write_all_fd(19, bytearray(b"abc"))
    assert attempts == 2


def test_22_zero_byte_write_rejected(monkeypatch):
    monkeypatch.setattr(os, "write", lambda fd, payload: 0)
    with pytest.raises(FileDescriptorIOError):
        write_all_fd(21, b"abc")


def test_23_read_until_eof():
    read_fd, write_fd = os.pipe()
    try:
        os.write(write_fd, b"complete-payload")
        os.close(write_fd)
        write_fd = -1
        assert read_bounded_fd(read_fd, 64) == b"complete-payload"
    finally:
        os.close(read_fd)
        if write_fd >= 0:
            os.close(write_fd)


def test_24_partial_reads_handled(monkeypatch):
    chunks = iter((b"ab", b"cd", b"ef", b""))
    requests = []

    def partial_read(fd, count):
        requests.append((fd, count))
        return next(chunks)

    monkeypatch.setattr(os, "read", partial_read)
    assert read_bounded_fd(23, 10) == b"abcdef"
    assert len(requests) == 4


def test_25_interrupted_read_retried(monkeypatch):
    attempts = 0

    def interrupted_once(fd, count):
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise InterruptedError
        return b""

    monkeypatch.setattr(os, "read", interrupted_once)
    assert read_bounded_fd(25, 10) == b""
    assert attempts == 2


def test_26_exactly_max_bytes_accepted():
    read_fd, write_fd = os.pipe()
    try:
        os.write(write_fd, b"12345678")
        os.close(write_fd)
        write_fd = -1
        assert read_bounded_fd(read_fd, 8) == b"12345678"
    finally:
        os.close(read_fd)
        if write_fd >= 0:
            os.close(write_fd)


def test_27_max_plus_one_bytes_rejected():
    read_fd, write_fd = os.pipe()
    try:
        os.write(write_fd, b"123456789")
        os.close(write_fd)
        write_fd = -1
        with pytest.raises(ConfigTooLargeError):
            read_bounded_fd(read_fd, 8)
    finally:
        os.close(read_fd)
        if write_fd >= 0:
            os.close(write_fd)


def test_28_invalid_fd_and_limit_rejected():
    for fd in (True, 1.5, "3", -1):
        with pytest.raises((TypeError, ValueError)):
            write_all_fd(fd, b"")
        with pytest.raises((TypeError, ValueError)):
            read_bounded_fd(fd)
    for limit in (True, 1.5, 0, -1):
        with pytest.raises((TypeError, ValueError)):
            read_bounded_fd(3, limit)


# ---------------------------------------------------------------------------
# 29-36. Worker status protocol
# ---------------------------------------------------------------------------


def test_29_started_status_round_trip():
    event = WorkerStatusEvent(1, "run-001", WorkerStatusKind.STARTED, None)
    assert decode_worker_status(encode_worker_status(event)) == event


def test_30_ready_status_round_trip():
    event = WorkerStatusEvent(1, "run-001", WorkerStatusKind.READY, "RTCM_FLOWING")
    assert decode_worker_status(encode_worker_status(event)) == event


def test_31_terminal_error_status_round_trip():
    event = WorkerStatusEvent(
        1, "run-001", WorkerStatusKind.TERMINAL_ERROR, "AUTH_FAILED"
    )
    assert decode_worker_status(encode_worker_status(event)) == event


def test_32_missing_run_id_rejected():
    value = json.loads(status_payload())
    del value["run_id"]
    with pytest.raises(StatusDecodeError, match="missing"):
        decode_worker_status(json.dumps(value).encode("utf-8") + b"\n")
    with pytest.raises(StatusValidationError):
        WorkerStatusEvent(1, "   ", WorkerStatusKind.STARTED)


def test_33_unknown_status_kind_and_invalid_detail_rejected():
    with pytest.raises(StatusDecodeError, match="unknown"):
        decode_worker_status(status_payload(kind="FUTURE_KIND"))
    with pytest.raises(StatusValidationError):
        WorkerStatusEvent(1, "run-001", WorkerStatusKind.STARTED, "")
    with pytest.raises(StatusValidationError):
        WorkerStatusEvent(1, "run-001", WorkerStatusKind.STARTED, "X" * 65)


def test_34_unsupported_status_schema_rejected():
    with pytest.raises(StatusDecodeError, match="unsupported"):
        decode_worker_status(status_payload(schema_version=2))
    with pytest.raises(StatusValidationError):
        WorkerStatusEvent(True, "run-001", WorkerStatusKind.STARTED)


def test_35_status_has_no_password_or_config_field():
    event = WorkerStatusEvent(1, "run-001", WorkerStatusKind.READY, "FLOWING")
    encoded = encode_worker_status(event)
    assert SECRET not in repr(event)
    assert SECRET.encode("utf-8") not in encoded
    assert {item.name for item in fields(WorkerStatusEvent)} == {
        "schema_version",
        "run_id",
        "kind",
        "detail_code",
    }
    assert "password" not in json.loads(encoded)
    assert "config" not in json.loads(encoded)


def test_36_event_and_config_are_immutable():
    event = WorkerStatusEvent(1, "run-001", WorkerStatusKind.STARTED)
    with pytest.raises(FrozenInstanceError):
        event.run_id = "run-002"
    config = make_config()
    with pytest.raises(FrozenInstanceError):
        config.password = "replacement"


# ---------------------------------------------------------------------------
# 37-44. Stable worker exit codes
# ---------------------------------------------------------------------------


def test_37_exit_code_0_maps_clean():
    assert WorkerExitCode.CLEAN == 0
    assert worker_exit_reason_from_code(0) is WorkerExitReason.CLEAN


def test_38_exit_code_20_maps_retryable_failure():
    assert WorkerExitCode.RETRYABLE_FAILURE == 20
    assert worker_exit_reason_from_code(20) is WorkerExitReason.RETRYABLE_FAILURE


def test_39_exit_code_21_maps_config_invalid():
    assert WorkerExitCode.CONFIG_INVALID == 21
    assert worker_exit_reason_from_code(21) is WorkerExitReason.CONFIG_INVALID


def test_40_exit_code_22_maps_ownership_conflict():
    assert WorkerExitCode.OWNERSHIP_CONFLICT == 22
    assert worker_exit_reason_from_code(22) is WorkerExitReason.OWNERSHIP_CONFLICT


def test_41_exit_code_23_maps_auth_failed():
    assert WorkerExitCode.AUTH_FAILED == 23
    assert worker_exit_reason_from_code(23) is WorkerExitReason.AUTH_FAILED


def test_42_exit_code_24_maps_mountpoint_rejected():
    assert WorkerExitCode.MOUNTPOINT_REJECTED == 24
    assert worker_exit_reason_from_code(24) is WorkerExitReason.MOUNTPOINT_REJECTED


def test_43_unknown_nonzero_maps_retryable_failure():
    assert worker_exit_reason_from_code(1) is WorkerExitReason.RETRYABLE_FAILURE
    assert worker_exit_reason_from_code(255) is WorkerExitReason.RETRYABLE_FAILURE
    assert worker_exit_reason_from_code(-9) is WorkerExitReason.RETRYABLE_FAILURE


def test_44_bool_and_non_int_exit_codes_rejected():
    for value in (True, False, 20.0, "20", None):
        with pytest.raises(TypeError):
            worker_exit_reason_from_code(value)


# ---------------------------------------------------------------------------
# 45-54. FD-held advisory locks
# ---------------------------------------------------------------------------


def test_45_manager_lock_acquires(tmp_path):
    assert DEFAULT_MANAGER_LOCK_PATH == "/run/lock/rover-rtk-manager.lock"
    lock = AdvisoryFileLock(tmp_path / "rover-rtk-manager.lock")
    try:
        assert lock.locked is False
        assert lock.acquire_nonblocking() is lock
        assert lock.locked is True
    finally:
        lock.close()


def test_46_second_manager_lock_conflicts(tmp_path):
    path = tmp_path / "rover-rtk-manager.lock"
    first = AdvisoryFileLock(path)
    second = AdvisoryFileLock(path)
    try:
        first.acquire_nonblocking()
        with pytest.raises(OwnershipConflictError):
            second.acquire_nonblocking()
        assert first.locked is True
        assert second.locked is False
    finally:
        second.close()
        first.close()


def test_47_close_releases_lock(tmp_path):
    lock = AdvisoryFileLock(tmp_path / "manager.lock")
    lock.acquire_nonblocking()
    lock.close()
    assert lock.locked is False


def test_48_another_owner_acquires_after_release(tmp_path):
    path = tmp_path / "manager.lock"
    first = AdvisoryFileLock(path)
    second = AdvisoryFileLock(path)
    first.acquire_nonblocking()
    first.close()
    try:
        second.acquire_nonblocking()
        assert second.locked is True
    finally:
        second.close()


def test_49_injection_lock_has_same_semantics(tmp_path):
    assert DEFAULT_INJECTION_LOCK_PATH == "/run/lock/rover-rtk-injection.lock"
    path = tmp_path / "rover-rtk-injection.lock"
    first = AdvisoryFileLock(path)
    second = AdvisoryFileLock(path)
    try:
        first.acquire_nonblocking()
        with pytest.raises(OwnershipConflictError):
            second.acquire_nonblocking()
    finally:
        second.close()
        first.close()


def test_50_repeated_close_and_close_before_acquire_are_safe(tmp_path):
    lock = AdvisoryFileLock(tmp_path / "manager.lock")
    lock.close()
    lock.close()
    lock.acquire_nonblocking()
    lock.close()
    lock.close()
    assert lock.locked is False


def test_51_context_manager_acquires_and_releases(tmp_path):
    path = tmp_path / "manager.lock"
    lock = AdvisoryFileLock(path)
    with lock as entered:
        assert entered is lock
        assert lock.locked is True
    assert lock.locked is False
    with AdvisoryFileLock(path) as replacement:
        assert replacement.locked is True


def test_52_same_object_repeated_acquire_is_idempotent(tmp_path):
    lock = AdvisoryFileLock(tmp_path / "manager.lock")
    try:
        first_result = lock.acquire_nonblocking()
        second_result = lock.acquire_nonblocking()
        assert first_result is lock
        assert second_result is lock
        assert lock.locked is True
    finally:
        lock.close()


def test_53_create_parent_false_does_not_create_missing_parent(tmp_path):
    parent = tmp_path / "missing" / "nested"
    lock = AdvisoryFileLock(parent / "manager.lock")
    with pytest.raises(FileNotFoundError):
        lock.acquire_nonblocking()
    assert parent.exists() is False
    assert lock.locked is False


def test_54_create_parent_true_creates_parent_and_acquires(tmp_path):
    parent = tmp_path / "created" / "nested"
    lock = AdvisoryFileLock(parent / "manager.lock", create_parent=True)
    try:
        lock.acquire_nonblocking()
        assert parent.is_dir()
        assert lock.locked is True
    finally:
        lock.close()


# ---------------------------------------------------------------------------
# 55-61. Final status framing and symbolic detail-code hardening
# ---------------------------------------------------------------------------


def test_55_status_framing_round_trip():
    event = WorkerStatusEvent(1, "run-055", WorkerStatusKind.READY, "RTCM_FLOWING")
    encoded = encode_worker_status(event)
    assert encoded.endswith(b"\n")
    assert encoded.count(b"\n") == 1
    assert decode_worker_status(encoded) == event


def test_56_status_newline_is_required():
    encoded = encode_worker_status(
        WorkerStatusEvent(1, "run-056", WorkerStatusKind.STARTED)
    )
    with pytest.raises(StatusDecodeError, match="trailing newline"):
        decode_worker_status(encoded[:-1])


def test_57_concatenated_status_events_are_rejected():
    first = encode_worker_status(
        WorkerStatusEvent(1, "run-057", WorkerStatusKind.STARTED)
    )
    second = encode_worker_status(
        WorkerStatusEvent(1, "run-057", WorkerStatusKind.READY)
    )
    with pytest.raises(StatusDecodeError, match="multiple newlines"):
        decode_worker_status(first + second)


def test_58_status_trailing_garbage_is_rejected():
    encoded = encode_worker_status(
        WorkerStatusEvent(1, "run-058", WorkerStatusKind.STARTED)
    )
    with pytest.raises(StatusDecodeError):
        decode_worker_status(encoded + b"garbage")
    with pytest.raises(StatusDecodeError):
        decode_worker_status(encoded + b"garbage\n")


def test_59_symbolic_detail_code_is_accepted():
    for detail_code in ("A", "AUTH_FAILED", "RTCM3_STREAM_2", "A" + "0" * 63):
        event = WorkerStatusEvent(
            1, "run-059", WorkerStatusKind.TERMINAL_ERROR, detail_code
        )
        assert decode_worker_status(encode_worker_status(event)) == event


def test_60_free_text_detail_code_is_rejected():
    for detail_code in (
        "Authentication failed",
        "auth_failed",
        "2_AUTH_FAILED",
        "AUTH-FAILED",
        "A" * 65,
    ):
        with pytest.raises(StatusValidationError):
            WorkerStatusEvent(
                1, "run-060", WorkerStatusKind.TERMINAL_ERROR, detail_code
            )


def test_61_newline_url_and_email_like_detail_codes_are_rejected():
    for detail_code in (
        "AUTH_FAILED\nTRACEBACK",
        "HTTPS://CASTER.EXAMPLE",
        "USER@EXAMPLE.COM",
    ):
        with pytest.raises(StatusValidationError):
            WorkerStatusEvent(
                1, "run-061", WorkerStatusKind.TERMINAL_ERROR, detail_code
            )


@pytest.mark.parametrize(
    ("field_name", "field_value"),
    (
        ("caster_host", "cástér.test"),
        ("mountpoint", "MÖUNT"),
        ("rtcm_topic", "/mavros/rtçm"),
    ),
)
def test_worker_protocol_tokens_reject_non_ascii(
    field_name,
    field_value,
):
    with pytest.raises(
        ConfigValidationError,
        match="ASCII",
    ):
        make_config(
            **{
                field_name: field_value,
            }
        )
