"""ROS-free serial RTCM3 source for a LoRa radio link.

The radio passes the base station's RTCM3 bytes through unchanged; this class
only opens the port, reads raw bytes and closes it. Framing, CRC checking,
health and reconnect deadlines stay in RtcmWorkerTransport and the node, so a
LoRa stream is judged exactly like an NTRIP stream.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional, Protocol


class SerialRtcmSourceError(RuntimeError):
    """The LoRa serial port could not be opened or read."""


class _SerialPort(Protocol):
    @property
    def is_open(self) -> bool: ...

    def read(self, size: int = 1) -> bytes: ...

    def reset_input_buffer(self) -> None: ...

    def close(self) -> None: ...


def _default_serial_factory(**kwargs) -> _SerialPort:
    try:
        import serial
    except ImportError as error:  # pragma: no cover - deployment dependency
        raise SerialRtcmSourceError(
            "pyserial is required for the LoRa RTCM source"
        ) from error
    return serial.Serial(**kwargs)


@dataclass(frozen=True, slots=True)
class SerialRtcmSourceSnapshot:
    device: str
    baudrate: int
    serial_open: bool
    open_total: int
    open_failures_total: int
    read_errors_total: int
    bytes_read_total: int
    last_error: Optional[str]


class SerialRtcmSource:
    """Blocking reader with a bounded read timeout."""

    def __init__(
        self,
        device: str,
        *,
        baudrate: int,
        read_timeout_sec: float,
        serial_factory: Optional[Callable[..., _SerialPort]] = None,
    ) -> None:
        if not isinstance(device, str) or not device.startswith("/dev/"):
            raise ValueError("device must be an absolute /dev path")
        if isinstance(baudrate, bool) or not isinstance(baudrate, int) or baudrate <= 0:
            raise ValueError("baudrate must be an int > 0")
        timeout = float(read_timeout_sec)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("read_timeout_sec must be finite and > 0")
        self._device = device
        self._baudrate = baudrate
        self._read_timeout_sec = timeout
        self._factory = serial_factory or _default_serial_factory
        self._port: Optional[_SerialPort] = None
        self._open_total = 0
        self._open_failures_total = 0
        self._read_errors_total = 0
        self._bytes_read_total = 0
        self._last_error: Optional[str] = None

    @property
    def is_open(self) -> bool:
        return self._port is not None and bool(self._port.is_open)

    @property
    def snapshot(self) -> SerialRtcmSourceSnapshot:
        return SerialRtcmSourceSnapshot(
            device=self._device,
            baudrate=self._baudrate,
            serial_open=self.is_open,
            open_total=self._open_total,
            open_failures_total=self._open_failures_total,
            read_errors_total=self._read_errors_total,
            bytes_read_total=self._bytes_read_total,
            last_error=self._last_error,
        )

    def open(self) -> None:
        if self.is_open:
            return
        self.close()
        try:
            port = self._factory(
                port=self._device,
                baudrate=self._baudrate,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=self._read_timeout_sec,
                exclusive=True,
            )
            # Drop whatever the radio buffered while nobody was reading; the
            # parser would discard a partial frame anyway.
            port.reset_input_buffer()
        except Exception as error:
            self._open_failures_total += 1
            self._last_error = f"open failed: {type(error).__name__}"
            raise SerialRtcmSourceError(
                f"cannot open LoRa port {self._device}: {error}"
            ) from error
        self._port = port
        self._open_total += 1
        self._last_error = None

    def read(self, size: int = 4096) -> bytes:
        """Return available bytes, or b'' when the read timeout elapsed."""

        if self._port is None:
            raise SerialRtcmSourceError("LoRa port is not open")
        try:
            data = self._port.read(size)
        except Exception as error:
            self._read_errors_total += 1
            self._last_error = f"read failed: {type(error).__name__}"
            raise SerialRtcmSourceError(
                f"LoRa port {self._device} read failed: {error}"
            ) from error
        data = bytes(data or b"")
        self._bytes_read_total += len(data)
        return data

    def close(self) -> None:
        port = self._port
        self._port = None
        if port is None:
            return
        try:
            port.close()
        except Exception:
            pass
