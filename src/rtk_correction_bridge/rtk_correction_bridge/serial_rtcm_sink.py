"""Byte-exact serial sink for direct RTCM correction injection."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Protocol


class SerialRtcmSinkError(RuntimeError):
    """Base direct-serial RTCM sink failure."""


class SerialRtcmSinkUnavailableError(SerialRtcmSinkError):
    """The configured serial endpoint cannot currently be opened."""


class SerialRtcmSinkWriteError(SerialRtcmSinkError):
    """A complete RTCM frame could not be written."""


class _SerialPort(Protocol):
    @property
    def is_open(self) -> bool: ...

    def write(
        self,
        data: bytes | bytearray | memoryview,
    ) -> int: ...

    def close(self) -> None: ...


SerialFactory = Callable[..., _SerialPort]


def _default_serial_factory(**kwargs) -> _SerialPort:
    """Open through PySerial only when direct mode needs it."""

    try:
        import serial
    except ImportError as error:
        raise RuntimeError(
            "PySerial is required for direct RTCM injection"
        ) from error

    return serial.Serial(**kwargs)


@dataclass(frozen=True, slots=True)
class SerialRtcmSinkSnapshot:
    """Credential-free direct-serial delivery diagnostics."""

    serial_open: bool
    open_attempts_total: int
    open_failures_total: int
    reopen_total: int
    frames_written_total: int
    bytes_written_total: int
    write_failures_total: int
    last_successful_write_monotonic: Optional[float]


class SerialRtcmSink:
    """Write already-validated complete RTCM frames to one endpoint.

    This class does not parse, modify, fragment, or route RTCM.

    A frame succeeds only after every byte is accepted by the serial API.
    Partial writes continue from the remaining byte. A hard failure closes
    the endpoint and fails that incomplete frame; it is never replayed from
    byte zero.
    """

    def __init__(
        self,
        device: str,
        *,
        baudrate: int = 230400,
        write_timeout_sec: float = 1.0,
        reopen_delay_sec: float = 1.0,
        clock: Callable[[], float] = time.monotonic,
        serial_factory: SerialFactory = _default_serial_factory,
    ) -> None:
        if (
            not isinstance(device, str)
            or not device
            or not device.startswith("/dev/")
            or any(
                ord(character) < 32
                or ord(character) == 127
                for character in device
            )
        ):
            raise ValueError(
                "device must be an absolute /dev path "
                "without control characters"
            )

        if (
            isinstance(baudrate, bool)
            or not isinstance(baudrate, int)
            or baudrate <= 0
        ):
            raise ValueError(
                "baudrate must be an int > 0"
            )

        self._require_finite_number(
            write_timeout_sec,
            "write_timeout_sec",
            minimum=0.0,
            strict=True,
        )

        self._require_finite_number(
            reopen_delay_sec,
            "reopen_delay_sec",
            minimum=0.0,
            strict=False,
        )

        if not callable(clock):
            raise TypeError(
                "clock must be callable"
            )

        if not callable(serial_factory):
            raise TypeError(
                "serial_factory must be callable"
            )

        self.device = device
        self.baudrate = baudrate
        self.write_timeout_sec = float(
            write_timeout_sec
        )
        self.reopen_delay_sec = float(
            reopen_delay_sec
        )

        self._clock = clock
        self._serial_factory = serial_factory
        self._lock = threading.RLock()

        self._serial: Optional[_SerialPort] = None
        self._next_open_at: Optional[float] = None

        self._open_attempts_total = 0
        self._open_failures_total = 0
        self._reopen_total = 0

        self._frames_written_total = 0
        self._bytes_written_total = 0
        self._write_failures_total = 0

        self._last_successful_write_monotonic: Optional[
            float
        ] = None

    @staticmethod
    def _require_finite_number(
        value: object,
        name: str,
        *,
        minimum: float,
        strict: bool,
    ) -> None:
        if (
            isinstance(value, bool)
            or not isinstance(
                value,
                (int, float),
            )
        ):
            relation = ">" if strict else ">="

            raise ValueError(
                f"{name} must be a finite number "
                f"{relation} {minimum:g}"
            )

        number = float(value)

        invalid = (
            not math.isfinite(number)
            or (
                number <= minimum
                if strict
                else number < minimum
            )
        )

        if invalid:
            relation = ">" if strict else ">="

            raise ValueError(
                f"{name} must be a finite number "
                f"{relation} {minimum:g}"
            )

    @property
    def snapshot(self) -> SerialRtcmSinkSnapshot:
        with self._lock:
            return SerialRtcmSinkSnapshot(
                serial_open=self._serial_is_open(),
                open_attempts_total=(
                    self._open_attempts_total
                ),
                open_failures_total=(
                    self._open_failures_total
                ),
                reopen_total=self._reopen_total,
                frames_written_total=(
                    self._frames_written_total
                ),
                bytes_written_total=(
                    self._bytes_written_total
                ),
                write_failures_total=(
                    self._write_failures_total
                ),
                last_successful_write_monotonic=(
                    self._last_successful_write_monotonic
                ),
            )

    def open(self) -> None:
        """Open the endpoint, respecting reconnect backoff."""

        with self._lock:
            self._ensure_open(
                self._now()
            )

    def close(self) -> None:
        """Close idempotently without scheduling a reopen."""

        with self._lock:
            self._close_serial_noexcept()
            self._next_open_at = None

    def write_frame(
        self,
        frame_bytes: bytes | bytearray | memoryview,
    ) -> None:
        """Write exactly one non-empty frame."""

        if not isinstance(
            frame_bytes,
            (bytes, bytearray, memoryview),
        ):
            raise TypeError(
                "frame_bytes must be bytes-like"
            )

        frame = bytes(
            frame_bytes
        )

        if not frame:
            raise ValueError(
                "frame_bytes must be non-empty"
            )

        with self._lock:
            now = self._now()

            self._ensure_open(
                now
            )

            port = self._serial

            if port is None:
                raise RuntimeError(
                    "serial sink open invariant violated"
                )

            offset = 0

            try:
                while offset < len(frame):
                    remaining = (
                        len(frame) - offset
                    )

                    written = port.write(
                        memoryview(frame)[offset:]
                    )

                    if (
                        isinstance(written, bool)
                        or not isinstance(
                            written,
                            int,
                        )
                        or written <= 0
                        or written > remaining
                    ):
                        raise OSError(
                            "serial write made invalid "
                            "forward progress"
                        )

                    offset += written

            except Exception as error:
                self._write_failures_total += 1

                self._close_serial_noexcept()

                self._next_open_at = (
                    now
                    + self.reopen_delay_sec
                )

                raise SerialRtcmSinkWriteError(
                    "unable to write complete RTCM frame"
                ) from error

            self._frames_written_total += 1
            self._bytes_written_total += len(
                frame
            )

            self._last_successful_write_monotonic = (
                now
            )

    def _ensure_open(
        self,
        now: float,
    ) -> None:
        if self._serial_is_open():
            return

        if self._serial is not None:
            self._close_serial_noexcept()

        if (
            self._next_open_at is not None
            and now < self._next_open_at
        ):
            raise SerialRtcmSinkUnavailableError(
                "direct RTCM serial reopen delay active"
            )

        self._open_attempts_total += 1

        port: Optional[_SerialPort] = None

        try:
            port = self._serial_factory(
                port=self.device,
                baudrate=self.baudrate,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=None,
                write_timeout=(
                    self.write_timeout_sec
                ),
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
                exclusive=True,
            )

            if (
                not callable(
                    getattr(
                        port,
                        "write",
                        None,
                    )
                )
                or not callable(
                    getattr(
                        port,
                        "close",
                        None,
                    )
                )
                or not bool(
                    getattr(
                        port,
                        "is_open",
                        False,
                    )
                )
            ):
                raise RuntimeError(
                    "serial factory returned "
                    "an unusable open port"
                )

        except Exception as error:
            if port is not None:
                try:
                    port.close()
                except Exception:
                    pass

            self._open_failures_total += 1

            self._next_open_at = (
                now
                + self.reopen_delay_sec
            )

            raise SerialRtcmSinkUnavailableError(
                "unable to open direct RTCM serial device"
            ) from error

        self._serial = port

        # Successful recovery after any previous open attempt.
        if self._open_attempts_total > 1:
            self._reopen_total += 1

        self._next_open_at = None

    def _serial_is_open(self) -> bool:
        if self._serial is None:
            return False

        try:
            return bool(
                self._serial.is_open
            )
        except Exception:
            return False

    def _close_serial_noexcept(self) -> None:
        port = self._serial
        self._serial = None

        if port is None:
            return

        try:
            port.close()
        except Exception:
            pass

    def _now(self) -> float:
        try:
            value = float(
                self._clock()
            )
        except Exception as error:
            raise RuntimeError(
                "serial sink clock failed"
            ) from error

        if not math.isfinite(value):
            raise RuntimeError(
                "serial sink clock must be finite"
            )

        return value
