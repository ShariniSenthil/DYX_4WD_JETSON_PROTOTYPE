"""Pure process-protocol primitives for the backend-owned RTK worker.

This module deliberately contains no process spawning, ROS imports, sockets, or
runtime manager integration.  It defines the validated data exchanged with a
future worker, bounded file-descriptor I/O, stable worker exit codes, and
FD-held POSIX advisory locks.
"""

from __future__ import annotations

import errno
import fcntl
import json
import math
import os
import re
from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Optional

from rover_backend.rtk_manager_core import WorkerExitReason


WORKER_CONFIG_SCHEMA_VERSION = 1
WORKER_STATUS_SCHEMA_VERSION = 1
MAX_WORKER_CONFIG_BYTES = 16 * 1024
MAX_WORKER_STATUS_BYTES = 4 * 1024
MAX_STATUS_DETAIL_CHARS = 64
_STATUS_DETAIL_CODE_PATTERN = re.compile(r"^[A-Z][A-Z0-9_]{0,63}$")

# RTCM3's largest wire frame is a 3-byte header, 1023-byte payload, and
# 3-byte CRC.  Keeping this bound local avoids coupling the backend package to
# the separately installed ROS correction-bridge package.
MAX_RTCM3_FRAME_BYTES = 1029

DEFAULT_MANAGER_LOCK_PATH = "/run/lock/rover-rtk-manager.lock"
DEFAULT_INJECTION_LOCK_PATH = "/run/lock/rover-rtk-injection.lock"

_READ_CHUNK_BYTES = 64 * 1024


class ProcessProtocolError(Exception):
    """Base class for explicit RTK process-protocol failures."""


class ConfigValidationError(ProcessProtocolError, ValueError):
    """A worker configuration violates the protocol contract."""


class ConfigDecodeError(ProcessProtocolError, ValueError):
    """A serialized worker configuration cannot be decoded safely."""


class ConfigTooLargeError(ConfigDecodeError):
    """A worker configuration or bounded read exceeded its byte limit."""


class StatusValidationError(ProcessProtocolError, ValueError):
    """A worker status event violates the protocol contract."""


class StatusDecodeError(ProcessProtocolError, ValueError):
    """A serialized worker status event cannot be decoded safely."""


class FileDescriptorIOError(ProcessProtocolError, OSError):
    """A bounded file-descriptor operation cannot make safe progress."""


class OwnershipConflictError(ProcessProtocolError):
    """Another owner currently holds the requested advisory file lock."""


def _require_nonempty_string(
    value: object,
    name: str,
    error_type: type[ValueError],
) -> str:
    if not isinstance(value, str):
        raise error_type("%s must be a string" % name)
    if not value.strip():
        raise error_type("%s must be non-empty" % name)
    return value


def _require_positive_finite(
    value: object,
    name: str,
    error_type: type[ValueError],
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise error_type("%s must be a finite number > 0" % name)
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise error_type("%s must be a finite number > 0" % name)
    return number


@dataclass(frozen=True, slots=True, repr=False)
class WorkerConfig:
    """Validated immutable configuration delivered to one RTK worker."""

    schema_version: int
    run_id: str
    caster_host: str
    caster_port: int
    mountpoint: str
    username: str
    password: str
    rtcm_topic: str
    connect_timeout_sec: float
    socket_timeout_sec: float
    healthy_age_sec: float
    stale_reconnect_sec: float
    reconnect_delay_sec: float
    first_data_timeout_sec: float
    max_mavros_rtcm_frame_bytes: int

    def __post_init__(self) -> None:
        if (
            isinstance(self.schema_version, bool)
            or not isinstance(self.schema_version, int)
            or self.schema_version != WORKER_CONFIG_SCHEMA_VERSION
        ):
            raise ConfigValidationError(
                "schema_version must be %d" % WORKER_CONFIG_SCHEMA_VERSION
            )

        _require_nonempty_string(self.run_id, "run_id", ConfigValidationError)
        _require_nonempty_string(
            self.caster_host, "caster_host", ConfigValidationError
        )
        if isinstance(self.caster_port, bool) or not isinstance(
            self.caster_port, int
        ):
            raise ConfigValidationError("caster_port must be an int in 1..65535")
        if not 1 <= self.caster_port <= 65535:
            raise ConfigValidationError("caster_port must be an int in 1..65535")
        _require_nonempty_string(
            self.mountpoint, "mountpoint", ConfigValidationError
        )
        _require_nonempty_string(self.username, "username", ConfigValidationError)
        _require_nonempty_string(self.password, "password", ConfigValidationError)
        topic = _require_nonempty_string(
            self.rtcm_topic, "rtcm_topic", ConfigValidationError
        )
        if not topic.startswith("/"):
            raise ConfigValidationError("rtcm_topic must be an absolute path")

        timeout_names = (
            "connect_timeout_sec",
            "socket_timeout_sec",
            "healthy_age_sec",
            "stale_reconnect_sec",
            "reconnect_delay_sec",
            "first_data_timeout_sec",
        )
        for name in timeout_names:
            value = _require_positive_finite(
                getattr(self, name), name, ConfigValidationError
            )
            object.__setattr__(self, name, value)
        if self.stale_reconnect_sec <= self.healthy_age_sec:
            raise ConfigValidationError(
                "stale_reconnect_sec must be greater than healthy_age_sec"
            )

        frame_bytes = self.max_mavros_rtcm_frame_bytes
        if isinstance(frame_bytes, bool) or not isinstance(frame_bytes, int):
            raise ConfigValidationError(
                "max_mavros_rtcm_frame_bytes must be an int in 1..%d"
                % MAX_RTCM3_FRAME_BYTES
            )
        if not 1 <= frame_bytes <= MAX_RTCM3_FRAME_BYTES:
            raise ConfigValidationError(
                "max_mavros_rtcm_frame_bytes must be an int in 1..%d"
                % MAX_RTCM3_FRAME_BYTES
            )

    def __repr__(self) -> str:
        """Return a diagnostic representation that never reveals the password."""
        return (
            "WorkerConfig("
            "schema_version=%r, run_id=%r, caster_host=%r, caster_port=%r, "
            "mountpoint=%r, username=%r, password=<redacted>, rtcm_topic=%r, "
            "connect_timeout_sec=%r, socket_timeout_sec=%r, healthy_age_sec=%r, "
            "stale_reconnect_sec=%r, reconnect_delay_sec=%r, "
            "first_data_timeout_sec=%r, max_mavros_rtcm_frame_bytes=%r)"
            % (
                self.schema_version,
                self.run_id,
                self.caster_host,
                self.caster_port,
                self.mountpoint,
                self.username,
                self.rtcm_topic,
                self.connect_timeout_sec,
                self.socket_timeout_sec,
                self.healthy_age_sec,
                self.stale_reconnect_sec,
                self.reconnect_delay_sec,
                self.first_data_timeout_sec,
                self.max_mavros_rtcm_frame_bytes,
            )
        )


_WORKER_CONFIG_FIELDS = frozenset(
    {
        "schema_version",
        "run_id",
        "caster_host",
        "caster_port",
        "mountpoint",
        "username",
        "password",
        "rtcm_topic",
        "connect_timeout_sec",
        "socket_timeout_sec",
        "healthy_age_sec",
        "stale_reconnect_sec",
        "reconnect_delay_sec",
        "first_data_timeout_sec",
        "max_mavros_rtcm_frame_bytes",
    }
)


def _reject_duplicate_object_keys(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON object field")
        result[key] = value
    return result


def _decode_json_object(data: object, *, max_bytes: int, label: str) -> dict:
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise TypeError("%s payload must be bytes-like" % label)
    raw = bytes(data)
    if len(raw) > max_bytes:
        if label == "worker config":
            raise ConfigTooLargeError(
                "worker config payload exceeds %d bytes" % max_bytes
            )
        raise StatusDecodeError("worker status payload exceeds %d bytes" % max_bytes)
    try:
        text = raw.decode("utf-8", errors="strict")
    except UnicodeDecodeError as error:
        decode_error = ConfigDecodeError if label == "worker config" else StatusDecodeError
        raise decode_error("%s payload is not valid UTF-8" % label) from error
    try:
        value = json.loads(
            text,
            object_pairs_hook=_reject_duplicate_object_keys,
            parse_constant=lambda _value: (_ for _ in ()).throw(
                ValueError("non-finite JSON number")
            ),
        )
    except (json.JSONDecodeError, ValueError) as error:
        decode_error = ConfigDecodeError if label == "worker config" else StatusDecodeError
        raise decode_error("%s payload is not valid JSON" % label) from error
    if not isinstance(value, dict):
        decode_error = ConfigDecodeError if label == "worker config" else StatusDecodeError
        raise decode_error("%s JSON root must be an object" % label)
    return value


def encode_worker_config(config: WorkerConfig) -> bytes:
    """Encode one validated worker configuration as bounded UTF-8 JSON."""
    if not isinstance(config, WorkerConfig):
        raise ConfigValidationError("config must be a WorkerConfig")
    payload = {
        "schema_version": config.schema_version,
        "run_id": config.run_id,
        "caster_host": config.caster_host,
        "caster_port": config.caster_port,
        "mountpoint": config.mountpoint,
        "username": config.username,
        "password": config.password,
        "rtcm_topic": config.rtcm_topic,
        "connect_timeout_sec": config.connect_timeout_sec,
        "socket_timeout_sec": config.socket_timeout_sec,
        "healthy_age_sec": config.healthy_age_sec,
        "stale_reconnect_sec": config.stale_reconnect_sec,
        "reconnect_delay_sec": config.reconnect_delay_sec,
        "first_data_timeout_sec": config.first_data_timeout_sec,
        "max_mavros_rtcm_frame_bytes": config.max_mavros_rtcm_frame_bytes,
    }
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    if len(encoded) > MAX_WORKER_CONFIG_BYTES:
        raise ConfigTooLargeError(
            "encoded worker config exceeds %d bytes" % MAX_WORKER_CONFIG_BYTES
        )
    return encoded


def decode_worker_config(data: bytes) -> WorkerConfig:
    """Decode exactly one strict, validated worker configuration object."""
    value = _decode_json_object(
        data, max_bytes=MAX_WORKER_CONFIG_BYTES, label="worker config"
    )
    fields = set(value)
    missing = _WORKER_CONFIG_FIELDS - fields
    if missing:
        raise ConfigDecodeError(
            "worker config is missing required field(s): %s"
            % ", ".join(sorted(missing))
        )
    unexpected = fields - _WORKER_CONFIG_FIELDS
    if unexpected:
        raise ConfigDecodeError(
            "worker config has unexpected field(s): %s"
            % ", ".join(sorted(unexpected))
        )
    schema_version = value["schema_version"]
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version != WORKER_CONFIG_SCHEMA_VERSION
    ):
        raise ConfigDecodeError("unsupported worker config schema_version")
    try:
        return WorkerConfig(**value)
    except ConfigValidationError as error:
        raise ConfigDecodeError("invalid worker config: %s" % error) from error


def _require_fd(fd: object) -> int:
    if isinstance(fd, bool) or not isinstance(fd, int):
        raise TypeError("fd must be a non-negative int")
    if fd < 0:
        raise ValueError("fd must be a non-negative int")
    return fd


def write_all_fd(fd: int, payload: bytes) -> None:
    """Write all bytes to ``fd`` without closing it.

    Partial writes are continued and interrupted system calls are retried.  A
    zero-byte write is treated as a hard error so the function cannot spin
    without making progress.
    """
    valid_fd = _require_fd(fd)
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError("payload must be bytes-like")
    data = bytes(payload)
    offset = 0
    while offset < len(data):
        try:
            written = os.write(valid_fd, data[offset:])
        except InterruptedError:
            continue
        if written <= 0:
            raise FileDescriptorIOError("os.write made no forward progress")
        offset += written


def read_bounded_fd(fd: int, max_bytes: int = MAX_WORKER_CONFIG_BYTES) -> bytes:
    """Read ``fd`` through EOF while retaining at most ``max_bytes + 1`` bytes."""
    valid_fd = _require_fd(fd)
    if isinstance(max_bytes, bool) or not isinstance(max_bytes, int):
        raise TypeError("max_bytes must be a positive int")
    if max_bytes <= 0:
        raise ValueError("max_bytes must be a positive int")

    result = bytearray()
    while True:
        detect_limit = max_bytes + 1
        remaining = detect_limit - len(result)
        try:
            chunk = os.read(valid_fd, min(_READ_CHUNK_BYTES, remaining))
        except InterruptedError:
            continue
        if not chunk:
            return bytes(result)
        if len(chunk) > remaining or len(result) + len(chunk) > max_bytes:
            raise ConfigTooLargeError("file descriptor payload exceeds %d bytes" % max_bytes)
        result.extend(chunk)


class WorkerStatusKind(Enum):
    """Small status vocabulary emitted by the future worker."""

    STARTED = "STARTED"
    READY = "READY"
    TERMINAL_ERROR = "TERMINAL_ERROR"


@dataclass(frozen=True, slots=True)
class WorkerStatusEvent:
    """Credential-free status event for one worker run.

    ``detail_code`` is either ``None`` or a short uppercase symbolic machine
    code.  It must never carry human-readable text, exception messages, URLs,
    credentials, email addresses, or traceback content.
    """

    schema_version: int
    run_id: str
    kind: WorkerStatusKind
    detail_code: Optional[str] = None

    def __post_init__(self) -> None:
        if (
            isinstance(self.schema_version, bool)
            or not isinstance(self.schema_version, int)
            or self.schema_version != WORKER_STATUS_SCHEMA_VERSION
        ):
            raise StatusValidationError(
                "schema_version must be %d" % WORKER_STATUS_SCHEMA_VERSION
            )
        _require_nonempty_string(self.run_id, "run_id", StatusValidationError)
        if not isinstance(self.kind, WorkerStatusKind):
            raise StatusValidationError("kind must be a WorkerStatusKind")
        if self.detail_code is not None:
            detail = _require_nonempty_string(
                self.detail_code, "detail_code", StatusValidationError
            )
            if (
                len(detail) > MAX_STATUS_DETAIL_CHARS
                or _STATUS_DETAIL_CODE_PATTERN.fullmatch(detail) is None
            ):
                raise StatusValidationError(
                    "detail_code must match ^[A-Z][A-Z0-9_]{0,63}$"
                )


_WORKER_STATUS_FIELDS = frozenset(
    {"schema_version", "run_id", "kind", "detail_code"}
)


def encode_worker_status(event: WorkerStatusEvent) -> bytes:
    """Encode one credential-free status event as newline-framed UTF-8 JSON."""
    if not isinstance(event, WorkerStatusEvent):
        raise StatusValidationError("event must be a WorkerStatusEvent")
    payload = {
        "schema_version": event.schema_version,
        "run_id": event.run_id,
        "kind": event.kind.value,
        "detail_code": event.detail_code,
    }
    encoded_json = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    encoded = encoded_json + b"\n"
    if len(encoded) > MAX_WORKER_STATUS_BYTES:
        raise StatusDecodeError(
            "encoded worker status exceeds %d bytes" % MAX_WORKER_STATUS_BYTES
        )
    return encoded


def decode_worker_status(data: bytes) -> WorkerStatusEvent:
    """Decode exactly one JSON event terminated by one physical newline."""
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise TypeError("worker status payload must be bytes-like")
    framed = bytes(data)
    if len(framed) > MAX_WORKER_STATUS_BYTES:
        raise StatusDecodeError(
            "worker status payload exceeds %d bytes" % MAX_WORKER_STATUS_BYTES
        )
    if not framed.endswith(b"\n"):
        raise StatusDecodeError(
            "worker status frame requires exactly one trailing newline"
        )
    if b"\n" in framed[:-1]:
        raise StatusDecodeError("worker status frame contains multiple newlines")

    value = _decode_json_object(
        framed[:-1],
        max_bytes=MAX_WORKER_STATUS_BYTES - 1,
        label="worker status",
    )
    fields = set(value)
    missing = _WORKER_STATUS_FIELDS - fields
    if missing:
        raise StatusDecodeError(
            "worker status is missing required field(s): %s"
            % ", ".join(sorted(missing))
        )
    unexpected = fields - _WORKER_STATUS_FIELDS
    if unexpected:
        raise StatusDecodeError(
            "worker status has unexpected field(s): %s"
            % ", ".join(sorted(unexpected))
        )
    schema_version = value["schema_version"]
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version != WORKER_STATUS_SCHEMA_VERSION
    ):
        raise StatusDecodeError("unsupported worker status schema_version")
    try:
        kind = WorkerStatusKind(value["kind"])
    except (TypeError, ValueError) as error:
        raise StatusDecodeError("unknown worker status kind") from error
    try:
        return WorkerStatusEvent(
            schema_version=schema_version,
            run_id=value["run_id"],
            kind=kind,
            detail_code=value["detail_code"],
        )
    except StatusValidationError as error:
        raise StatusDecodeError("invalid worker status: %s" % error) from error


class WorkerExitCode(IntEnum):
    """Stable numeric OS exit codes for RTK worker termination."""

    CLEAN = 0
    RETRYABLE_FAILURE = 20
    CONFIG_INVALID = 21
    OWNERSHIP_CONFLICT = 22
    AUTH_FAILED = 23
    MOUNTPOINT_REJECTED = 24


_WORKER_EXIT_REASON_BY_CODE = {
    WorkerExitCode.CLEAN: WorkerExitReason.CLEAN,
    WorkerExitCode.RETRYABLE_FAILURE: WorkerExitReason.RETRYABLE_FAILURE,
    WorkerExitCode.CONFIG_INVALID: WorkerExitReason.CONFIG_INVALID,
    WorkerExitCode.OWNERSHIP_CONFLICT: WorkerExitReason.OWNERSHIP_CONFLICT,
    WorkerExitCode.AUTH_FAILED: WorkerExitReason.AUTH_FAILED,
    WorkerExitCode.MOUNTPOINT_REJECTED: WorkerExitReason.MOUNTPOINT_REJECTED,
}


def worker_exit_reason_from_code(code: int) -> WorkerExitReason:
    """Map a stable OS exit code to the manager's semantic exit reason."""
    if isinstance(code, bool) or not isinstance(code, int):
        raise TypeError("code must be an int")
    try:
        exit_code = WorkerExitCode(code)
    except ValueError:
        return WorkerExitReason.RETRYABLE_FAILURE
    return _WORKER_EXIT_REASON_BY_CODE[exit_code]


class AdvisoryFileLock:
    """Exclusive nonblocking POSIX lock held for the lifetime of an open FD.

    ``__enter__`` acquires the lock. Repeated acquisition on the same locked
    object is idempotent. ``close`` is safe before acquisition and on repeated
    calls, and it releases ownership by unlocking and closing the held FD.
    Parent directories are created only when ``create_parent=True``.
    """

    def __init__(self, path: os.PathLike[str] | str, create_parent: bool = False):
        if isinstance(path, (bytes, bytearray)):
            raise TypeError("path must be a string or string-like path")
        try:
            resolved_path = os.fspath(path)
        except TypeError as error:
            raise TypeError("path must be a string or string-like path") from error
        if not isinstance(resolved_path, str) or not resolved_path:
            raise ValueError("path must be a non-empty string path")
        if not isinstance(create_parent, bool):
            raise TypeError("create_parent must be a bool")
        self.path = resolved_path
        self.create_parent = create_parent
        self._fd: Optional[int] = None

    @property
    def locked(self) -> bool:
        """Whether this object currently owns an open, locked FD."""
        return self._fd is not None

    def acquire_nonblocking(self) -> "AdvisoryFileLock":
        """Acquire exclusive ownership immediately or raise on contention."""
        if self._fd is not None:
            return self
        if self.create_parent:
            parent = os.path.dirname(os.path.abspath(self.path))
            os.makedirs(parent, mode=0o755, exist_ok=True)

        flags = os.O_RDWR | os.O_CREAT
        if hasattr(os, "O_CLOEXEC"):
            flags |= os.O_CLOEXEC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        fd = os.open(self.path, flags, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as error:
            os.close(fd)
            if error.errno in (errno.EACCES, errno.EAGAIN, errno.EWOULDBLOCK):
                raise OwnershipConflictError(
                    "advisory lock is already owned: %s" % self.path
                ) from error
            raise
        except BaseException:
            os.close(fd)
            raise
        self._fd = fd
        return self

    def close(self) -> None:
        """Release this object's lock and FD; safe to call repeatedly."""
        fd = self._fd
        if fd is None:
            return
        self._fd = None
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        finally:
            os.close(fd)

    def __enter__(self) -> "AdvisoryFileLock":
        return self.acquire_nonblocking()

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        self.close()
        return False
