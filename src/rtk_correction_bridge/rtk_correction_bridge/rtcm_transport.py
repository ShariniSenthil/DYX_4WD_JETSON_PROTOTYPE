"""ROS-free RTCM worker transport: parser sessions and MAVROS size policy.

This module sits between ``RTCM3Parser`` and a later ROS publisher. It does
not import ROS, sockets, or logging.

RTCM3 protocol-valid frames may be 6..1029 bytes. The downstream MAVROS
development size gate is separate and defaults to 720 bytes. A CRC-valid
721..1029-byte frame remains protocol-valid: it updates valid-frame counters
and timestamps, increments the oversize counter, and is not published.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional

from rtk_correction_bridge.rtcm3_parser import (
    MAX_FRAME_LENGTH,
    RTCM3Frame,
    RTCM3Parser,
    RTCM3ParserCounters,
)

DEFAULT_MAX_MAVROS_RTCM_FRAME_BYTES = 720
MAX_MAVROS_RTCM_FRAME_BYTES_LIMIT = MAX_FRAME_LENGTH


def validate_max_mavros_rtcm_frame_bytes(value: object) -> int:
    """Return a MAVROS size-gate integer in ``1..1029``.

    ``720`` is the development default, not a protocol maximum. The protocol
    maximum is ``MAX_FRAME_LENGTH`` (1029) from the parser.
    """
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError('max_mavros_rtcm_frame_bytes must be an integer')
    if not 1 <= value <= MAX_MAVROS_RTCM_FRAME_BYTES_LIMIT:
        raise ValueError(
            'max_mavros_rtcm_frame_bytes must be 1..%d'
            % MAX_MAVROS_RTCM_FRAME_BYTES_LIMIT
        )
    return value


def _zero_parser_counters() -> RTCM3ParserCounters:
    return RTCM3ParserCounters(
        bytes_fed_total=0,
        frames_valid_total=0,
        bytes_valid_total=0,
        frames_crc_invalid_total=0,
        headers_invalid_total=0,
        resync_bytes_discarded_total=0,
        partial_frame_timeouts_total=0,
    )


@dataclass
class RtcmWorkerCounters:
    """Worker-lifetime counters. These persist across NTRIP reconnects."""

    socket_bytes_received_total: int = 0
    rtcm_frames_valid_total: int = 0
    rtcm_bytes_valid_total: int = 0
    rtcm_frames_crc_invalid_total: int = 0
    rtcm_headers_invalid_total: int = 0
    rtcm_resync_bytes_discarded_total: int = 0
    rtcm_partial_frame_timeouts_total: int = 0
    rtcm_frames_oversize_total: int = 0
    rtcm_frames_published_total: int = 0
    rtcm_publish_errors_total: int = 0


class RtcmWorkerTransport:
    """Per-socket parser sessions plus worker-lifetime transport policy.

    Every NTRIP TCP connection must call :meth:`new_parser_session`. Residual
    bytes from a previous connection are discarded with that parser. Worker
    counters are not reset.

    ``last_socket_byte_at``, ``last_valid_frame_at``, and
    ``last_published_frame_at`` are session-scoped. A new socket cannot
    inherit publication freshness, so legacy /healthy and correction-age
    topics stay false / -1.0 until this session publishes a supported frame.
    """

    def __init__(
        self,
        *,
        max_mavros_rtcm_frame_bytes: int = (
            DEFAULT_MAX_MAVROS_RTCM_FRAME_BYTES
        ),
        parser_factory: Optional[Callable[[], RTCM3Parser]] = None,
    ) -> None:
        self.max_mavros_rtcm_frame_bytes = (
            validate_max_mavros_rtcm_frame_bytes(
                max_mavros_rtcm_frame_bytes
            )
        )
        self._parser_factory = parser_factory or RTCM3Parser
        self.counters = RtcmWorkerCounters()
        self.last_socket_byte_at: Optional[float] = None
        self.last_valid_frame_at: Optional[float] = None
        self.last_published_frame_at: Optional[float] = None
        self._parser: Optional[RTCM3Parser] = None
        self._parser_snapshot = _zero_parser_counters()

    @property
    def parser(self) -> Optional[RTCM3Parser]:
        """Return the parser for the current socket session, if any."""
        return self._parser

    def new_parser_session(self) -> RTCM3Parser:
        """Create a new parser for one NTRIP TCP connection.

        Residual bytes from a previous session are discarded. Parser-counter
        snapshots restart so later feed/service deltas cannot double-count
        the old parser. All three correction timestamps reset to None.
        Worker-lifetime counters are left unchanged.
        """
        self._parser = self._parser_factory()
        self._parser_snapshot = _zero_parser_counters()
        self._clear_session_timestamps()
        return self._parser

    def discard_parser_session(self) -> None:
        """Drop the current socket parser, residual bytes, and timestamps."""
        self._parser = None
        self._parser_snapshot = _zero_parser_counters()
        self._clear_session_timestamps()

    def _clear_session_timestamps(self) -> None:
        self.last_socket_byte_at = None
        self.last_valid_frame_at = None
        self.last_published_frame_at = None

    def process_stream_bytes(
        self,
        data: bytes,
        now_sec: float,
    ) -> list[bytes]:
        """Feed NTRIP body bytes and return MAVROS-publishable frame images.

        Non-empty ``data`` updates ``socket_bytes_received_total`` and
        ``last_socket_byte_at``. Handshake leftovers must use this same path.
        """
        parser = self._require_parser()
        if data:
            self.counters.socket_bytes_received_total += len(data)
            self.last_socket_byte_at = now_sec
        frames = parser.feed(data, now_sec)
        self._sync_parser_counter_deltas(parser)
        return self._process_parsed_frames(frames, now_sec)

    def service_parser(self, now_sec: float) -> list[bytes]:
        """Advance partial-frame timeout with no new socket bytes.

        Call this on socket timeout before evaluating source staleness. A
        timed-out false candidate may expose a valid frame already buffered
        behind it.
        """
        parser = self._require_parser()
        frames = parser.service(now_sec)
        self._sync_parser_counter_deltas(parser)
        return self._process_parsed_frames(frames, now_sec)

    def attempt_publish(
        self,
        frame_bytes: bytes,
        now_sec: float,
        publisher: Callable[[bytes], object],
    ) -> bool:
        """Hand one size-gated frame to ``publisher``.

        ``publisher`` receives the complete RTCM3 image, including header and
        CRC. The published counter and timestamp update only if ``publisher``
        returns without exception.
        """
        try:
            publisher(frame_bytes)
        except Exception:
            self.counters.rtcm_publish_errors_total += 1
            return False
        self.counters.rtcm_frames_published_total += 1
        self.last_published_frame_at = now_sec
        return True

    def record_publish_success(self, now_sec: float) -> None:
        """Record one successful publication of a size-gated frame."""
        self.counters.rtcm_frames_published_total += 1
        self.last_published_frame_at = now_sec

    def record_publish_error(self) -> None:
        """Record a publication attempt that raised."""
        self.counters.rtcm_publish_errors_total += 1

    def published_age_sec(self, now_sec: float) -> float:
        """Return seconds since this session's last successful publish.

        ``inf`` means this session has not published a supported frame.
        """
        if self.last_published_frame_at is None:
            return math.inf
        return now_sec - self.last_published_frame_at

    def is_healthy(
        self,
        connected: bool,
        now_sec: float,
        healthy_age_sec: float,
    ) -> bool:
        """Return legacy /healthy semantics for a published-frame age."""
        if not connected or self.last_published_frame_at is None:
            return False
        return (
            now_sec - self.last_published_frame_at
        ) <= healthy_age_sec

    def first_valid_frame_timed_out(
        self,
        connection_start: float,
        now_sec: float,
        first_data_timeout_sec: float,
    ) -> bool:
        """Return True if this socket session has no CRC-valid RTCM yet."""
        if self.last_valid_frame_at is not None:
            return False
        return (now_sec - connection_start) > first_data_timeout_sec

    def source_is_stale(
        self,
        now_sec: float,
        stale_reconnect_sec: float,
    ) -> bool:
        """Return True when valid-frame age exceeds the reconnect limit.

        Oversize CRC-valid frames keep the source fresh. Published-frame age
        is not used here.
        """
        if self.last_valid_frame_at is None:
            return False
        return (
            now_sec - self.last_valid_frame_at
        ) > stale_reconnect_sec

    def source_deadline_elapsed(
        self,
        connection_start: float,
        now_sec: float,
        first_data_timeout_sec: float,
        stale_reconnect_sec: float,
    ) -> bool:
        """Return True if first-valid or stale-source deadline has elapsed.

        Socket-byte arrival, CRC-invalid traffic, and publishability do not
        postpone these deadlines. Only CRC-valid frames, including oversize
        frames, refresh ``last_valid_frame_at``.
        """
        return self.first_valid_frame_timed_out(
            connection_start,
            now_sec,
            first_data_timeout_sec,
        ) or self.source_is_stale(
            now_sec,
            stale_reconnect_sec,
        )

    def _require_parser(self) -> RTCM3Parser:
        if self._parser is None:
            raise RuntimeError(
                'no RTCM parser session is active; '
                'call new_parser_session() first'
            )
        return self._parser

    def _process_parsed_frames(
        self,
        frames: list[RTCM3Frame],
        now_sec: float,
    ) -> list[bytes]:
        publishable: list[bytes] = []
        for frame in frames:
            self.last_valid_frame_at = now_sec
            if frame.total_length > self.max_mavros_rtcm_frame_bytes:
                self.counters.rtcm_frames_oversize_total += 1
                continue
            publishable.append(frame.frame_bytes)
        return publishable

    def _sync_parser_counter_deltas(self, parser: RTCM3Parser) -> None:
        """Fold this session's parser-counter deltas into worker totals."""
        current = parser.counters
        prev = self._parser_snapshot
        self.counters.rtcm_frames_valid_total += (
            current.frames_valid_total - prev.frames_valid_total
        )
        self.counters.rtcm_bytes_valid_total += (
            current.bytes_valid_total - prev.bytes_valid_total
        )
        self.counters.rtcm_frames_crc_invalid_total += (
            current.frames_crc_invalid_total
            - prev.frames_crc_invalid_total
        )
        self.counters.rtcm_headers_invalid_total += (
            current.headers_invalid_total - prev.headers_invalid_total
        )
        self.counters.rtcm_resync_bytes_discarded_total += (
            current.resync_bytes_discarded_total
            - prev.resync_bytes_discarded_total
        )
        self.counters.rtcm_partial_frame_timeouts_total += (
            current.partial_frame_timeouts_total
            - prev.partial_frame_timeouts_total
        )
        self._parser_snapshot = current
