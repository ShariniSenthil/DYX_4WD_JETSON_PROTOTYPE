"""Secure subprocess bootstrap for the backend-owned RTK correction worker."""

from __future__ import annotations

import argparse
import errno
import os
import select
import threading
from typing import Callable, Optional

from rover_backend.rtk_process_protocol import (
    DEFAULT_INJECTION_LOCK_PATH,
    AdvisoryFileLock,
    ConfigDecodeError,
    ConfigTooLargeError,
    OwnershipConflictError,
    ProcessProtocolError,
    WORKER_STATUS_SCHEMA_VERSION,
    WorkerConfig,
    WorkerExitCode,
    WorkerStatusEvent,
    WorkerStatusKind,
    decode_worker_config,
    encode_worker_status,
    read_bounded_fd,
    write_all_fd,
)


def _close_fd(fd: Optional[int]) -> None:
    if fd is None:
        return
    try:
        os.close(fd)
    except OSError:
        # Cleanup must never replace the worker's already-selected
        # semantic exit reason.
        pass


class ParentLivenessGuard:
    """Watch an inherited parent-liveness FD and react immediately to EOF."""

    def __init__(
        self,
        fd: int,
        on_parent_eof: Callable[[], None],
    ) -> None:
        if isinstance(fd, bool) or not isinstance(fd, int) or fd < 0:
            raise ValueError("liveness fd must be a non-negative int")
        if not callable(on_parent_eof):
            raise TypeError("on_parent_eof must be callable")

        self._fd: Optional[int] = fd
        self._on_parent_eof = on_parent_eof
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None:
            return
        thread = threading.Thread(
            target=self._run,
            name="rtk-parent-liveness",
            daemon=True,
        )
        self._thread = thread
        thread.start()

    def close(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=0.5)
        self._thread = None

        fd = self._fd
        self._fd = None
        _close_fd(fd)

    def _run(self) -> None:
        while not self._stop.is_set():
            fd = self._fd
            if fd is None:
                return

            try:
                readable, _, _ = select.select(
                    [fd],
                    [],
                    [],
                    0.2,
                )
            except (OSError, ValueError):
                if self._stop.is_set():
                    return
                return

            if self._stop.is_set():
                return

            if fd not in readable:
                continue

            try:
                payload = os.read(fd, 1)
            except InterruptedError:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                return

            if payload:
                continue

            if not self._stop.is_set():
                self._on_parent_eof()
            return


def _emit_status(
    status_fd: int,
    run_id: str,
    kind: WorkerStatusKind,
    detail_code: Optional[str] = None,
) -> None:
    event = WorkerStatusEvent(
        schema_version=WORKER_STATUS_SCHEMA_VERSION,
        run_id=run_id,
        kind=kind,
        detail_code=detail_code,
    )
    write_all_fd(
        status_fd,
        encode_worker_status(event),
    )


def _try_emit_terminal(
    status_fd: int,
    run_id: str,
    detail_code: str,
) -> None:
    try:
        _emit_status(
            status_fd,
            run_id,
            WorkerStatusKind.TERMINAL_ERROR,
            detail_code,
        )
    except (OSError, ProcessProtocolError):
        pass


def _default_parent_eof() -> None:
    # Parent authority disappeared unexpectedly. Exit immediately so the OS
    # releases the injection lock and ROS/MAVROS publisher resources.
    os._exit(int(WorkerExitCode.RETRYABLE_FAILURE))


def _run_ros_runtime(
    config: WorkerConfig,
    ready_callback: Callable[[], None],
) -> int:
    # Lazy imports keep this bootstrap unit-testable without ROS.
    import rclpy

    from rtk_correction_bridge.ntrip_failures import (
        NtripAuthError,
        NtripMountpointRejectedError,
    )
    from rtk_correction_bridge.ntrip_to_px4_node import NtripToPx4Node

    node = None

    # Do not allow the bootstrap's private FD command-line arguments to be
    # interpreted as ROS arguments.
    rclpy.init(args=[])

    try:
        node = NtripToPx4Node(
            worker_config=config,
        )

        ready_callback()

        try:
            node.run()
        except NtripAuthError:
            return int(
                WorkerExitCode.AUTH_FAILED
            )
        except NtripMountpointRejectedError:
            return int(
                WorkerExitCode.MOUNTPOINT_REJECTED
            )

        return int(WorkerExitCode.CLEAN)

    finally:
        # Cleanup errors must never mask AUTH_FAILED,
        # MOUNTPOINT_REJECTED, or another already-selected result.
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def run_worker(
    *,
    config_fd: int,
    liveness_fd: int,
    status_fd: int,
    injection_lock_path: os.PathLike[str] | str = DEFAULT_INJECTION_LOCK_PATH,
    runtime_entry: Optional[
        Callable[[WorkerConfig, Callable[[], None]], int]
    ] = None,
    parent_eof_callback: Optional[Callable[[], None]] = None,
) -> int:
    """Run one secure RTK worker lifetime."""

    config: Optional[WorkerConfig] = None
    guard: Optional[ParentLivenessGuard] = None
    injection_lock = AdvisoryFileLock(
        injection_lock_path,
    )

    try:
        try:
            encoded_config = read_bounded_fd(
                config_fd,
            )
            config = decode_worker_config(
                encoded_config,
            )
        except (
            ConfigDecodeError,
            ConfigTooLargeError,
            OSError,
            TypeError,
            ValueError,
        ):
            return int(
                WorkerExitCode.CONFIG_INVALID
            )
        finally:
            _close_fd(config_fd)

        try:
            injection_lock.acquire_nonblocking()
        except OwnershipConflictError:
            _try_emit_terminal(
                status_fd,
                config.run_id,
                "OWNERSHIP_CONFLICT",
            )
            return int(
                WorkerExitCode.OWNERSHIP_CONFLICT
            )
        except OSError:
            # Filesystem/permission/lock-path failures are environmental
            # and retryable, not terminal configuration failures.
            return int(
                WorkerExitCode.RETRYABLE_FAILURE
            )

        try:
            _emit_status(
                status_fd,
                config.run_id,
                WorkerStatusKind.STARTED,
                "CONFIG_ACCEPTED",
            )
        except (OSError, ProcessProtocolError):
            return int(
                WorkerExitCode.RETRYABLE_FAILURE
            )

        guard = ParentLivenessGuard(
            liveness_fd,
            parent_eof_callback
            or _default_parent_eof,
        )
        guard.start()

        ready_sent = False

        def emit_ready() -> None:
            nonlocal ready_sent
            if ready_sent:
                return

            _emit_status(
                status_fd,
                config.run_id,
                WorkerStatusKind.READY,
                "WORKER_READY",
            )
            ready_sent = True

        runtime = (
            runtime_entry
            if runtime_entry is not None
            else _run_ros_runtime
        )

        try:
            result = runtime(
                config,
                emit_ready,
            )
        except KeyboardInterrupt:
            return int(
                WorkerExitCode.CLEAN
            )
        except Exception:
            _try_emit_terminal(
                status_fd,
                config.run_id,
                "RUNTIME_FAILURE",
            )
            return int(
                WorkerExitCode.RETRYABLE_FAILURE
            )

        if (
            isinstance(result, bool)
            or not isinstance(result, int)
        ):
            _try_emit_terminal(
                status_fd,
                config.run_id,
                "RUNTIME_FAILURE",
            )
            return int(
                WorkerExitCode.RETRYABLE_FAILURE
            )

        if result == int(
            WorkerExitCode.AUTH_FAILED
        ):
            _try_emit_terminal(
                status_fd,
                config.run_id,
                "AUTH_FAILED",
            )

        elif result == int(
            WorkerExitCode.MOUNTPOINT_REJECTED
        ):
            _try_emit_terminal(
                status_fd,
                config.run_id,
                "MOUNTPOINT_REJECTED",
            )

        return result

    finally:
        # Finalization is best-effort. It must not overwrite the semantic
        # worker exit code selected above.
        if guard is not None:
            try:
                guard.close()
            except Exception:
                pass
        else:
            _close_fd(liveness_fd)

        try:
            injection_lock.close()
        except Exception:
            pass

        _close_fd(status_fd)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--config-fd",
        required=True,
        type=int,
    )
    parser.add_argument(
        "--liveness-fd",
        required=True,
        type=int,
    )
    parser.add_argument(
        "--status-fd",
        required=True,
        type=int,
    )
    parser.add_argument(
        "--injection-lock-path",
        default=DEFAULT_INJECTION_LOCK_PATH,
    )

    return parser.parse_args()


def main() -> None:
    args = _arguments()

    exit_code = run_worker(
        config_fd=args.config_fd,
        liveness_fd=args.liveness_fd,
        status_fd=args.status_fd,
        injection_lock_path=args.injection_lock_path,
    )

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
