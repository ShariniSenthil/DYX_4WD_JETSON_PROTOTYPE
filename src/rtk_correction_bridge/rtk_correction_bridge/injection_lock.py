"""RTCM injection ownership lock for standalone correction-bridge launches."""

from __future__ import annotations

import errno
import fcntl
import os
from typing import Optional


# Must match rover_backend.rtk_process_protocol.DEFAULT_INJECTION_LOCK_PATH.
DEFAULT_INJECTION_LOCK_PATH = "/run/lock/rover-rtk-injection.lock"


class InjectionOwnershipConflictError(RuntimeError):
    """Another process already owns RTCM injection authority."""


class InjectionOwnershipLock:
    """Hold exclusive RTCM injection authority through an open file descriptor."""

    def __init__(
        self,
        path: os.PathLike[str] | str = DEFAULT_INJECTION_LOCK_PATH,
    ) -> None:
        self.path = os.fspath(path)
        self._fd: Optional[int] = None

    @property
    def locked(self) -> bool:
        return self._fd is not None

    def acquire_nonblocking(self) -> None:
        if self._fd is not None:
            return

        flags = os.O_RDWR | os.O_CREAT

        if hasattr(os, "O_CLOEXEC"):
            flags |= os.O_CLOEXEC

        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW

        fd = os.open(
            self.path,
            flags,
            0o640,
        )

        try:
            fcntl.flock(
                fd,
                fcntl.LOCK_EX | fcntl.LOCK_NB,
            )
        except OSError as error:
            os.close(fd)

            if error.errno in (
                errno.EACCES,
                errno.EAGAIN,
            ):
                raise InjectionOwnershipConflictError(
                    "RTCM injection authority is already owned"
                ) from None

            raise

        self._fd = fd

    def close(self) -> None:
        fd = self._fd
        self._fd = None

        if fd is None:
            return

        try:
            fcntl.flock(
                fd,
                fcntl.LOCK_UN,
            )
        finally:
            os.close(fd)

    def __enter__(self):
        self.acquire_nonblocking()
        return self

    def __exit__(
        self,
        exc_type,
        exc_value,
        traceback,
    ) -> bool:
        self.close()
        return False
