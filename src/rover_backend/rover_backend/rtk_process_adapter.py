"""POSIX subprocess lifecycle adapter for the backend-owned RTK worker.

The adapter owns manager authority, one child process, and three independent
one-way pipes.  It is deliberately synchronous and nonblocking: a future
backend timer is responsible for calling :meth:`poll` with injected monotonic
time.
"""

from __future__ import annotations

import errno
import math
import os
import subprocess
from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Union

from rover_backend.rtk_manager_core import (
    SpawnWorker,
    StopWorker,
    WorkerExitReason,
)
from rover_backend.rtk_process_protocol import (
    DEFAULT_MANAGER_LOCK_PATH,
    MAX_WORKER_STATUS_BYTES,
    AdvisoryFileLock,
    ProcessProtocolError,
    WorkerConfig,
    WorkerStatusEvent,
    decode_worker_status,
    encode_worker_config,
    worker_exit_reason_from_code,
    write_all_fd,
)


class ProcessAdapterError(Exception):
    """Base class for deterministic process-adapter failures."""


class AdapterNotOpenError(ProcessAdapterError):
    """An operation requires manager authority owned by an open adapter."""


class ActiveChildError(ProcessAdapterError):
    """An operation would abandon or replace the currently owned child."""


class SpawnFailedError(ProcessAdapterError):
    """A worker could not be spawned or safely configured."""


class ProcessAdapterProtocolError(ProcessAdapterError):
    """The child emitted an invalid or oversized physical status frame."""


@dataclass(frozen=True, slots=True)
class ChildProcessStarted:
    """A child was created successfully; this is not a READY indication."""

    run_id: str
    pid: int


@dataclass(frozen=True, slots=True)
class ChildStatusReceived:
    """A validated, matching-run worker status event."""

    event: WorkerStatusEvent


@dataclass(frozen=True, slots=True)
class ChildProcessExited:
    """The child was reaped and its central exit-code mapping was applied."""

    run_id: str
    reason: WorkerExitReason
    returncode: int


AdapterEvent = Union[
    ChildProcessStarted,
    ChildStatusReceived,
    ChildProcessExited,
]


@dataclass(frozen=True, slots=True)
class RtkProcessAdapterSnapshot:
    """Credential-free diagnostic state for the process adapter."""

    is_open: bool
    active_run_id: Optional[str]
    pid: Optional[int]
    config_writer_open: bool
    liveness_writer_open: bool
    status_reader_open: bool
    status_buffer_bytes: int
    stop_requested: bool
    stop_deadline: Optional[float]
    kill_sent: bool
    exit_reported: bool


class RtkProcessAdapter:
    """Own at most one POSIX RTK worker subprocess and its lifecycle pipes."""

    def __init__(
        self,
        worker_command: Sequence[str],
        *,
        manager_lock_path: os.PathLike[str] | str = DEFAULT_MANAGER_LOCK_PATH,
        stop_grace_sec: float = 5.0,
        popen_factory: Optional[Callable[..., subprocess.Popen]] = None,
    ) -> None:
        if isinstance(worker_command, (str, bytes, bytearray)):
            raise TypeError("worker_command must be a sequence of arguments")
        try:
            command = tuple(worker_command)
        except TypeError as error:
            raise TypeError(
                "worker_command must be a sequence of arguments"
            ) from error
        if not command:
            raise ValueError("worker_command must not be empty")
        if any(not isinstance(item, str) or not item for item in command):
            raise ValueError("worker_command arguments must be non-empty strings")
        if isinstance(stop_grace_sec, bool) or not isinstance(
            stop_grace_sec, (int, float)
        ):
            raise TypeError("stop_grace_sec must be a finite number > 0")
        grace = float(stop_grace_sec)
        if not math.isfinite(grace) or grace <= 0.0:
            raise ValueError("stop_grace_sec must be a finite number > 0")
        if popen_factory is not None and not callable(popen_factory):
            raise TypeError("popen_factory must be callable")

        self.worker_command = command
        self.stop_grace_sec = grace
        self._manager_lock = AdvisoryFileLock(manager_lock_path)
        self._popen_factory = popen_factory

        self.active_run_id: Optional[str] = None
        self.process: Optional[subprocess.Popen] = None
        self.config_write_fd: Optional[int] = None
        self.liveness_write_fd: Optional[int] = None
        self.status_read_fd: Optional[int] = None
        self.status_buffer = bytearray()
        self.stop_requested = False
        self.stop_deadline: Optional[float] = None
        self.kill_sent = False
        self.exit_reported = False
        self._last_now_sec: Optional[float] = None

    def __repr__(self) -> str:
        snapshot = self.snapshot()
        return (
            "RtkProcessAdapter(is_open=%r, active_run_id=%r, pid=%r, "
            "stop_requested=%r, kill_sent=%r)"
            % (
                snapshot.is_open,
                snapshot.active_run_id,
                snapshot.pid,
                snapshot.stop_requested,
                snapshot.kill_sent,
            )
        )

    @property
    def is_open(self) -> bool:
        """Whether this adapter currently owns backend manager authority."""
        return self._manager_lock.locked

    def snapshot(self) -> RtkProcessAdapterSnapshot:
        """Return an immutable snapshot containing no command or credentials."""
        process = self.process
        pid = None if process is None else process.pid
        return RtkProcessAdapterSnapshot(
            is_open=self.is_open,
            active_run_id=self.active_run_id,
            pid=pid,
            config_writer_open=self.config_write_fd is not None,
            liveness_writer_open=self.liveness_write_fd is not None,
            status_reader_open=self.status_read_fd is not None,
            status_buffer_bytes=len(self.status_buffer),
            stop_requested=self.stop_requested,
            stop_deadline=self.stop_deadline,
            kill_sent=self.kill_sent,
            exit_reported=self.exit_reported,
        )

    def open(self) -> "RtkProcessAdapter":
        """Acquire backend manager authority; repeated calls are idempotent."""
        self._manager_lock.acquire_nonblocking()
        return self

    def close(self) -> None:
        """Release authority after all child resources have been reaped."""
        if self.process is not None:
            raise ActiveChildError("cannot close adapter while a child is active")
        self._close_all_parent_fds()
        self.status_buffer.clear()
        self._manager_lock.close()

    def __enter__(self) -> "RtkProcessAdapter":
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        self.close()
        return False

    def spawn(
        self,
        action: SpawnWorker,
        config: WorkerConfig,
    ) -> tuple[AdapterEvent, ...]:
        """Spawn one configured worker and return only a process-start event."""
        if not self.is_open:
            raise AdapterNotOpenError("adapter must be open before spawn")
        if not isinstance(action, SpawnWorker):
            raise TypeError("action must be a SpawnWorker")
        if not isinstance(config, WorkerConfig):
            raise TypeError("config must be a WorkerConfig")
        if action.run_id != config.run_id:
            raise ProcessAdapterError("spawn action and config run_id must match")
        if self.process is not None or self.active_run_id is not None:
            raise ActiveChildError("an RTK worker child is already active")
        if self._has_stale_child_resources():
            raise ProcessAdapterError("stale child resources prevent spawn")

        # Encoding is deterministic and may reject an otherwise-valid config
        # whose serialized form exceeds the protocol limit. Reject it before
        # creating pipes or a subprocess.
        try:
            encoded_config = encode_worker_config(config)
        except ProcessProtocolError:
            raise SpawnFailedError(
                "failed to encode RTK worker configuration"
            ) from None

        pipe_fds: list[int] = []
        try:
            config_read_fd, config_write_fd = os.pipe()
            pipe_fds.extend((config_read_fd, config_write_fd))
            liveness_read_fd, liveness_write_fd = os.pipe()
            pipe_fds.extend((liveness_read_fd, liveness_write_fd))
            status_read_fd, status_write_fd = os.pipe()
            pipe_fds.extend((status_read_fd, status_write_fd))
            os.set_blocking(status_read_fd, False)
        except BaseException as error:
            self._close_fds(pipe_fds)
            raise SpawnFailedError("failed to prepare RTK worker pipes") from error

        child_fds = (config_read_fd, liveness_read_fd, status_write_fd)
        argv = [
            *self.worker_command,
            "--config-fd",
            str(config_read_fd),
            "--liveness-fd",
            str(liveness_read_fd),
            "--status-fd",
            str(status_write_fd),
        ]
        popen = self._popen_factory or subprocess.Popen
        try:
            process = popen(
                argv,
                pass_fds=child_fds,
                close_fds=True,
                env=None,
            )
        except BaseException as error:
            self._close_fds(pipe_fds)
            raise SpawnFailedError("failed to spawn RTK worker") from error

        # Popen has duplicated the child ends.  The parent must discard its
        # copies before delivering config so config EOF is unambiguous.
        self._close_fds(child_fds)
        self.active_run_id = action.run_id
        self.process = process
        self.config_write_fd = config_write_fd
        self.liveness_write_fd = liveness_write_fd
        self.status_read_fd = status_read_fd
        self.status_buffer.clear()
        self.stop_requested = False
        self.stop_deadline = None
        self.kill_sent = False
        self.exit_reported = False

        try:
            write_all_fd(config_write_fd, encoded_config)
        except BaseException:
            self._close_config_writer()
            self._request_termination_without_clock()
            raise SpawnFailedError(
                "failed to deliver RTK worker configuration"
            ) from None
        finally:
            encoded_config = b""
            self._close_config_writer()

        return (ChildProcessStarted(action.run_id, process.pid),)

    def stop(
        self,
        action: StopWorker,
        now_sec: float,
    ) -> tuple[AdapterEvent, ...]:
        """Request cooperative termination once for the matching active run."""
        now = self._accept_now(now_sec)
        if not isinstance(action, StopWorker):
            raise TypeError("action must be a StopWorker")
        process = self.process
        if process is None or action.run_id != self.active_run_id:
            return ()
        if self.stop_requested:
            return ()

        self.stop_requested = True
        self.stop_deadline = now + self.stop_grace_sec
        try:
            process.terminate()
        except ProcessLookupError:
            pass
        return ()

    def poll(self, now_sec: float) -> tuple[AdapterEvent, ...]:
        """Drain statuses, escalate a due stop, and nonblockingly reap a child."""
        now = self._accept_now(now_sec)
        process = self.process
        if process is None:
            return ()

        accepted = self._drain_status_pipe()

        # A config-delivery failure requests SIGTERM before any clock exists.
        # The first valid poll starts its deterministic escalation deadline.
        if self.stop_requested and self.stop_deadline is None:
            self.stop_deadline = now + self.stop_grace_sec

        returncode = process.poll()
        if (
            returncode is None
            and self.stop_requested
            and self.stop_deadline is not None
            and now >= self.stop_deadline
            and not self.kill_sent
        ):
            self.kill_sent = True
            try:
                process.kill()
            except ProcessLookupError:
                pass
            returncode = process.poll()

        if returncode is None:
            return tuple(accepted)

        accepted.extend(self._drain_status_pipe())
        run_id = self.active_run_id
        if run_id is None:
            raise ProcessAdapterError("child ownership missing during reap")
        reason = worker_exit_reason_from_code(returncode)
        exit_event = ChildProcessExited(run_id, reason, returncode)
        self.exit_reported = True
        self._clear_reaped_child()
        accepted.append(exit_event)
        return tuple(accepted)

    def _accept_now(self, now_sec: float) -> float:
        if isinstance(now_sec, bool) or not isinstance(now_sec, (int, float)):
            raise TypeError("now_sec must be a finite number")
        now = float(now_sec)
        if not math.isfinite(now):
            raise ValueError("now_sec must be finite")
        previous = self._last_now_sec
        if previous is not None and now < previous:
            raise ValueError("now_sec must be nondecreasing")
        self._last_now_sec = now
        return now

    def _request_termination_without_clock(self) -> None:
        process = self.process
        if process is None or self.stop_requested:
            return
        self.stop_requested = True
        self.stop_deadline = None
        try:
            process.terminate()
        except OSError:
            pass

    def _drain_status_pipe(self) -> list[AdapterEvent]:
        accepted: list[AdapterEvent] = []
        fd = self.status_read_fd
        if fd is None:
            return accepted

        saw_eof = False
        while True:
            try:
                chunk = os.read(fd, 64 * 1024)
            except InterruptedError:
                continue
            except BlockingIOError:
                break
            except OSError as error:
                if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                raise
            if not chunk:
                saw_eof = True
                self._close_status_reader()
                break
            self.status_buffer.extend(chunk)
            accepted.extend(self._extract_status_frames())

        accepted.extend(self._extract_status_frames())

        if saw_eof and self.status_buffer:
            self.status_buffer.clear()
            raise ProcessAdapterProtocolError(
                "worker status pipe closed with a partial frame"
            )

        return accepted

    def _extract_status_frames(self) -> list[AdapterEvent]:
        accepted: list[AdapterEvent] = []
        while True:
            newline_index = self.status_buffer.find(b"\n")
            if newline_index < 0:
                if len(self.status_buffer) > MAX_WORKER_STATUS_BYTES:
                    self.status_buffer.clear()
                    raise ProcessAdapterProtocolError(
                        "unterminated worker status frame exceeds limit"
                    )
                return accepted
            frame_size = newline_index + 1
            if frame_size > MAX_WORKER_STATUS_BYTES:
                del self.status_buffer[:frame_size]
                raise ProcessAdapterProtocolError(
                    "worker status frame exceeds limit"
                )
            frame = bytes(self.status_buffer[:frame_size])
            del self.status_buffer[:frame_size]
            try:
                event = decode_worker_status(frame)
            except (ProcessProtocolError, TypeError, ValueError) as error:
                raise ProcessAdapterProtocolError(
                    "invalid worker status frame"
                ) from error
            if event.run_id == self.active_run_id:
                accepted.append(ChildStatusReceived(event))

    def _clear_reaped_child(self) -> None:
        self._close_all_parent_fds()
        self.status_buffer.clear()
        self.active_run_id = None
        self.process = None
        self.stop_requested = False
        self.stop_deadline = None
        self.kill_sent = False

    def _has_stale_child_resources(self) -> bool:
        return (
            self.config_write_fd is not None
            or self.liveness_write_fd is not None
            or self.status_read_fd is not None
            or bool(self.status_buffer)
            or self.stop_requested
            or self.stop_deadline is not None
            or self.kill_sent
        )

    def _close_config_writer(self) -> None:
        fd = self.config_write_fd
        self.config_write_fd = None
        if fd is not None:
            self._close_fds((fd,))

    def _close_liveness_writer(self) -> None:
        fd = self.liveness_write_fd
        self.liveness_write_fd = None
        if fd is not None:
            self._close_fds((fd,))

    def _close_status_reader(self) -> None:
        fd = self.status_read_fd
        self.status_read_fd = None
        if fd is not None:
            self._close_fds((fd,))

    def _close_all_parent_fds(self) -> None:
        self._close_config_writer()
        self._close_liveness_writer()
        self._close_status_reader()

    @staticmethod
    def _close_fds(fds) -> None:
        for fd in fds:
            try:
                os.close(fd)
            except OSError as error:
                if error.errno != errno.EBADF:
                    raise
