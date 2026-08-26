"""ROS-free RTCM3 framing parser with CRC-24Q validation.

This module reconstructs complete RTCM3 frames from an incremental byte
stream. It does not publish frames, apply MAVROS transport size policy, or
depend on ROS, sockets, or logging.

RTCM3 frame layout:

* byte 0: preamble 0xD3
* bytes 1-2: 6 reserved bits (must be zero) + 10-bit payload length
* payload: 0..1023 bytes
* final 3 bytes: CRC-24Q over header + payload, big-endian

Protocol-valid total size is 6..1029 bytes. A 721-byte or 1029-byte frame is
framing-valid here even if a later worker refuses it for transport reasons.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

PREAMBLE = 0xD3
HEADER_LENGTH = 3
CRC_LENGTH = 3
MAX_PAYLOAD_LENGTH = 1023
MAX_FRAME_LENGTH = HEADER_LENGTH + MAX_PAYLOAD_LENGTH + CRC_LENGTH  # 1029
DEFAULT_BUFFER_LIMIT = 64 * 1024
DEFAULT_PARTIAL_TIMEOUT_SEC = 2.0

# Full CRC-24Q polynomial including the x^24 term.
CRC24Q_POLY = 0x1864CFB
CRC24Q_CHECK_ASCII = b'123456789'
CRC24Q_CHECK_VALUE = 0xCDE703


def _crc24q_table() -> tuple[int, ...]:
    """Return the 256-entry CRC-24Q table for the unreflected algorithm."""
    table = []
    for index in range(256):
        crc = index << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= CRC24Q_POLY
        table.append(crc & 0xFFFFFF)
    return tuple(table)


_CRC24Q_TABLE = _crc24q_table()


def _require_finite_now_sec(now_sec: float) -> float:
    """Return ``now_sec`` as float, rejecting non-numeric and non-finite values."""
    if isinstance(now_sec, bool) or not isinstance(now_sec, (int, float)):
        raise TypeError('now_sec must be a finite number')
    value = float(now_sec)
    if not math.isfinite(value):
        raise ValueError('now_sec must be finite')
    return value


def _require_positive_finite_timeout(partial_timeout_sec: float) -> float:
    """Return a finite timeout strictly greater than zero."""
    if isinstance(partial_timeout_sec, bool) or not isinstance(
        partial_timeout_sec, (int, float)
    ):
        raise TypeError('partial_timeout_sec must be a finite number')
    value = float(partial_timeout_sec)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError('partial_timeout_sec must be a finite value > 0')
    return value


def crc24q(data: bytes) -> int:
    """Return the CRC-24Q of ``data``.

    Parameters match RTCM 10403.x / QualComm CRC-24Q:

    * width 24
    * polynomial 0x1864CFB (effective 24-bit 0x864CFB)
    * initial value 0
    * no input or output reflection
    * final XOR 0

    The ASCII vector ``123456789`` produces ``0xCDE703``.
    """
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise TypeError('data must be bytes-like')
    crc = 0
    for byte in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ _CRC24Q_TABLE[(crc >> 16) ^ byte]
    return crc


def _message_type_from_payload(payload: bytes) -> Optional[int]:
    """Decode the 12-bit RTCM message number, or None if payload is too short."""
    if len(payload) < 2:
        return None
    return (payload[0] << 4) | (payload[1] >> 4)


@dataclass(frozen=True, slots=True)
class RTCM3Frame:
    """One CRC-valid RTCM3 frame.

    ``frame_bytes`` is the complete wire image: 3-byte header, payload, and
    3-byte CRC. ``message_type`` is the first 12 payload bits when the payload
    is at least two bytes; otherwise it is None. Unknown message numbers are
    still accepted.
    """

    frame_bytes: bytes
    payload_length: int
    total_length: int
    message_type: Optional[int]


@dataclass(frozen=True, slots=True)
class RTCM3ParserCounters:
    """Deterministic parser counters. Semantics:

    ``bytes_fed_total``
        Every byte passed to :meth:`RTCM3Parser.feed`, including noise.

    ``frames_valid_total``
        Complete candidates whose CRC-24Q matched.

    ``bytes_valid_total``
        Header + payload + CRC bytes of every CRC-valid protocol frame,
        including frames larger than any later MAVROS transport limit.

    ``frames_crc_invalid_total``
        Complete candidates rejected because CRC-24Q did not match.

    ``headers_invalid_total``
        Candidates rejected because reserved header bits were non-zero.

    ``resync_bytes_discarded_total``
        Bytes dropped while hunting for a valid frame: leading noise, the
        single preamble byte of an invalid-header or CRC-invalid candidate,
        the single preamble byte of a partial-timeout candidate, and any
        leftover bytes discarded because no preamble remains.

    ``partial_frame_timeouts_total``
        Times an incomplete candidate at buffer index zero exceeded the
        partial-frame timeout.

    This object does not include published, oversize, or ROS error counters.
    Those belong to a later worker/transport policy.
    """

    bytes_fed_total: int
    frames_valid_total: int
    bytes_valid_total: int
    frames_crc_invalid_total: int
    headers_invalid_total: int
    resync_bytes_discarded_total: int
    partial_frame_timeouts_total: int


class RTCM3Parser:
    """Incremental RTCM3 framer with CRC-24Q, resync, and injectable time.

    Input may contain one frame, many frames, split frames, noise, or a mix.
    The parser never clears the remaining buffer after a single bad candidate;
    it discards only that candidate's 0xD3 and rescans.

    Time is never read from the wall clock. ``feed`` and ``service`` both
    require a finite, nondecreasing ``now_sec``. The partial-candidate timer
    starts at the exact ``now_sec`` when a candidate 0xD3 first becomes
    ``buffer[0]``; later bytes and later timestamps do not restart that timer.
    """

    def __init__(
        self,
        *,
        buffer_limit: int = DEFAULT_BUFFER_LIMIT,
        partial_timeout_sec: float = DEFAULT_PARTIAL_TIMEOUT_SEC,
    ) -> None:
        if int(buffer_limit) < MAX_FRAME_LENGTH:
            raise ValueError(
                'buffer_limit must be >= %d' % MAX_FRAME_LENGTH
            )
        self._buffer_limit = int(buffer_limit)
        self._partial_timeout_sec = _require_positive_finite_timeout(
            partial_timeout_sec
        )
        self._buffer = bytearray()
        self._partial_started_at: Optional[float] = None
        self._last_now_sec: Optional[float] = None
        self._bytes_fed_total = 0
        self._frames_valid_total = 0
        self._bytes_valid_total = 0
        self._frames_crc_invalid_total = 0
        self._headers_invalid_total = 0
        self._resync_bytes_discarded_total = 0
        self._partial_frame_timeouts_total = 0

    @property
    def counters(self) -> RTCM3ParserCounters:
        """Return a snapshot of the deterministic parser counters."""
        return RTCM3ParserCounters(
            bytes_fed_total=self._bytes_fed_total,
            frames_valid_total=self._frames_valid_total,
            bytes_valid_total=self._bytes_valid_total,
            frames_crc_invalid_total=self._frames_crc_invalid_total,
            headers_invalid_total=self._headers_invalid_total,
            resync_bytes_discarded_total=self._resync_bytes_discarded_total,
            partial_frame_timeouts_total=self._partial_frame_timeouts_total,
        )

    @property
    def buffered_length(self) -> int:
        """Return the current residual buffer size in bytes."""
        return len(self._buffer)

    def feed(self, data: bytes, now_sec: float) -> list[RTCM3Frame]:
        """Append ``data`` and return every newly completed valid frame.

        ``now_sec`` is a required finite monotonic-style timestamp. It must be
        greater than or equal to every previously accepted ``now_sec``. New
        bytes do not restart a candidate timer that already started when that
        0xD3 became ``buffer[0]``.
        """
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError('data must be bytes-like')
        now = self._accept_now_sec(now_sec)
        self._last_now_sec = now
        self._bytes_fed_total += len(data)
        if len(data):
            self._buffer.extend(data)
        return self._drain(now)

    def service(self, now_sec: float) -> list[RTCM3Frame]:
        """Apply partial-frame timeout without consuming new stream bytes."""
        now = self._accept_now_sec(now_sec)
        self._last_now_sec = now
        return self._drain(now)

    def _accept_now_sec(self, now_sec: float) -> float:
        """Return ``now_sec`` if finite and nondecreasing; do not mutate state."""
        now = _require_finite_now_sec(now_sec)
        if self._last_now_sec is not None and now < self._last_now_sec:
            raise ValueError('now_sec must be nondecreasing')
        return now

    def _drain(self, now_sec: float) -> list[RTCM3Frame]:
        frames: list[RTCM3Frame] = []
        while self._buffer:
            preamble_at = self._buffer.find(PREAMBLE)
            if preamble_at < 0:
                self._discard_prefix(len(self._buffer))
                self._partial_started_at = None
                break
            if preamble_at > 0:
                self._discard_prefix(preamble_at)
                self._partial_started_at = None

            # Candidate 0xD3 is now at index zero.
            self._arm_partial_timer(now_sec)
            if self._partial_timed_out(now_sec):
                self._timeout_current_preamble()
                continue
            if len(self._buffer) < HEADER_LENGTH:
                break

            header1 = self._buffer[1]
            header2 = self._buffer[2]
            if (header1 & 0xFC) != 0:
                self._headers_invalid_total += 1
                self._discard_current_preamble()
                continue

            payload_length = ((header1 & 0x03) << 8) | header2
            total_length = HEADER_LENGTH + payload_length + CRC_LENGTH
            if total_length > MAX_FRAME_LENGTH:
                self._headers_invalid_total += 1
                self._discard_current_preamble()
                continue
            if len(self._buffer) < total_length:
                break

            covered = bytes(self._buffer[: HEADER_LENGTH + payload_length])
            crc_offset = HEADER_LENGTH + payload_length
            received_crc = (
                (self._buffer[crc_offset] << 16)
                | (self._buffer[crc_offset + 1] << 8)
                | self._buffer[crc_offset + 2]
            )
            if crc24q(covered) != received_crc:
                self._frames_crc_invalid_total += 1
                self._discard_current_preamble()
                continue

            frame_bytes = bytes(self._buffer[:total_length])
            del self._buffer[:total_length]
            self._partial_started_at = None
            payload = frame_bytes[HEADER_LENGTH:HEADER_LENGTH + payload_length]
            frames.append(
                RTCM3Frame(
                    frame_bytes=frame_bytes,
                    payload_length=payload_length,
                    total_length=total_length,
                    message_type=_message_type_from_payload(payload),
                )
            )
            self._frames_valid_total += 1
            self._bytes_valid_total += total_length

        self._trim_residual()
        return frames

    def _arm_partial_timer(self, now_sec: float) -> None:
        if self._partial_started_at is None and self._buffer:
            if self._buffer[0] == PREAMBLE:
                self._partial_started_at = now_sec

    def _partial_timed_out(self, now_sec: float) -> bool:
        if self._partial_started_at is None:
            return False
        return (now_sec - self._partial_started_at) >= self._partial_timeout_sec

    def _timeout_current_preamble(self) -> None:
        self._partial_frame_timeouts_total += 1
        self._discard_current_preamble()

    def _discard_current_preamble(self) -> None:
        if self._buffer:
            self._discard_prefix(1)
        self._partial_started_at = None

    def _discard_prefix(self, count: int) -> None:
        if count <= 0:
            return
        self._resync_bytes_discarded_total += count
        del self._buffer[:count]

    def _legitimate_partial_at_zero(self) -> bool:
        """Return True if buffer[0] is a protocol-legal incomplete candidate."""
        if not self._buffer or self._buffer[0] != PREAMBLE:
            return False
        if len(self._buffer) < HEADER_LENGTH:
            return True
        if (self._buffer[1] & 0xFC) != 0:
            return False
        payload_length = ((self._buffer[1] & 0x03) << 8) | self._buffer[2]
        total_length = HEADER_LENGTH + payload_length + CRC_LENGTH
        if total_length > MAX_FRAME_LENGTH:
            return False
        return len(self._buffer) < total_length

    def _trim_residual(self) -> None:
        """Bound residual garbage after complete candidates were parsed.

        Parse-before-trim: a complete candidate is validated before this
        runs. A legitimate incomplete candidate at index zero is preserved.
        """
        while len(self._buffer) > self._buffer_limit:
            if self._legitimate_partial_at_zero():
                return
            preamble_at = self._buffer.find(PREAMBLE)
            if preamble_at < 0:
                self._discard_prefix(len(self._buffer))
                self._partial_started_at = None
                return
            if preamble_at > 0:
                self._discard_prefix(preamble_at)
                self._partial_started_at = None
                continue
            self._discard_current_preamble()
