"""Phase-1B ROS-free tests for RTCM worker transport policy.

These tests cover the MAVROS size gate, parser-session isolation, counter
aggregation, and publication bookkeeping. They do not import ROS or sockets.
"""

from __future__ import annotations

import random

import pytest

from rtk_correction_bridge.rtcm3_parser import (
    DEFAULT_PARTIAL_TIMEOUT_SEC,
    MAX_FRAME_LENGTH,
    PREAMBLE,
    crc24q,
)
from rtk_correction_bridge.rtcm_transport import (
    DEFAULT_MAX_MAVROS_RTCM_FRAME_BYTES,
    MAX_MAVROS_RTCM_FRAME_BYTES_LIMIT,
    RtcmWorkerTransport,
    validate_max_mavros_rtcm_frame_bytes,
)


def make_rtcm3_frame(payload: bytes) -> bytes:
    """Build one protocol-valid RTCM3 frame around ``payload``."""
    length = len(payload)
    header = bytes((PREAMBLE, (length >> 8) & 0x03, length & 0xFF))
    covered = header + payload
    return covered + crc24q(covered).to_bytes(3, 'big')


def frame_of_total_length(total_length: int) -> bytes:
    """Return one CRC-valid RTCM3 frame with an exact total size."""
    payload_length = total_length - 6
    if payload_length < 0:
        raise ValueError('total_length must be >= 6')
    payload = bytes(index % 256 for index in range(payload_length))
    frame = make_rtcm3_frame(payload)
    assert len(frame) == total_length
    return frame


def corrupt_crc(frame: bytes) -> bytes:
    """Return a complete-looking candidate with a flipped CRC byte."""
    corrupt = bytearray(frame)
    corrupt[-1] ^= 0x01
    return bytes(corrupt)


class RecordingPublisher:
    """Collect complete frame images handed toward publication."""

    def __init__(self) -> None:
        self.frames: list[bytes] = []

    def __call__(self, frame_bytes: bytes) -> None:
        self.frames.append(frame_bytes)


class FailingPublisher:
    """Raise on every publication attempt."""

    def __init__(self, exc: Exception | None = None) -> None:
        self.calls = 0
        self.exc = exc or RuntimeError('publish failed')

    def __call__(self, frame_bytes: bytes) -> None:
        self.calls += 1
        raise self.exc


def new_transport() -> RtcmWorkerTransport:
    transport = RtcmWorkerTransport()
    transport.new_parser_session()
    return transport


def random_chunks(data: bytes, rng: random.Random) -> list[bytes]:
    chunks = []
    index = 0
    remaining = len(data)
    while remaining:
        size = rng.randint(1, remaining)
        chunks.append(data[index:index + size])
        index += size
        remaining -= size
    return chunks


# ---------------------------------------------------------------------------
# Parameter validation
# ---------------------------------------------------------------------------


def test_default_mavros_gate_is_720_not_protocol_max():
    assert DEFAULT_MAX_MAVROS_RTCM_FRAME_BYTES == 720
    assert MAX_MAVROS_RTCM_FRAME_BYTES_LIMIT == MAX_FRAME_LENGTH == 1029
    transport = RtcmWorkerTransport()
    assert transport.max_mavros_rtcm_frame_bytes == 720


def test_max_mavros_rtcm_frame_bytes_bounds():
    assert validate_max_mavros_rtcm_frame_bytes(1) == 1
    assert validate_max_mavros_rtcm_frame_bytes(720) == 720
    assert validate_max_mavros_rtcm_frame_bytes(1029) == 1029
    with pytest.raises(ValueError):
        validate_max_mavros_rtcm_frame_bytes(0)
    with pytest.raises(ValueError):
        validate_max_mavros_rtcm_frame_bytes(1030)
    with pytest.raises(TypeError):
        validate_max_mavros_rtcm_frame_bytes(True)
    with pytest.raises(TypeError):
        validate_max_mavros_rtcm_frame_bytes(720.0)


# ---------------------------------------------------------------------------
# 1-5. Supported complete frames are publishable unchanged
# ---------------------------------------------------------------------------


def test_1_six_byte_valid_frame_is_publishable():
    frame = frame_of_total_length(6)
    transport = new_transport()
    candidates = transport.process_stream_bytes(frame, 1.0)
    assert candidates == [frame]
    assert len(candidates[0]) == 6
    assert transport.counters.rtcm_frames_valid_total == 1
    assert transport.counters.rtcm_frames_oversize_total == 0


def test_2_180_byte_total_frame_is_publishable_unchanged():
    frame = frame_of_total_length(180)
    transport = new_transport()
    candidates = transport.process_stream_bytes(frame, 1.0)
    assert candidates == [frame]
    assert len(candidates[0]) == 180


def test_3_181_byte_total_frame_is_not_split():
    frame = frame_of_total_length(181)
    transport = new_transport()
    candidates = transport.process_stream_bytes(frame, 1.0)
    assert len(candidates) == 1
    assert candidates[0] == frame
    assert len(candidates[0]) == 181
    assert [len(item) for item in candidates] != [180, 1]


def test_4_719_byte_total_frame_is_publishable_unchanged():
    frame = frame_of_total_length(719)
    transport = new_transport()
    candidates = transport.process_stream_bytes(frame, 1.0)
    assert candidates == [frame]
    assert len(candidates[0]) == 719


def test_5_720_byte_total_frame_is_publishable_unchanged():
    frame = frame_of_total_length(720)
    transport = new_transport()
    candidates = transport.process_stream_bytes(frame, 1.0)
    assert candidates == [frame]
    assert len(candidates[0]) == 720
    assert transport.counters.rtcm_frames_oversize_total == 0
    assert transport.counters.rtcm_frames_valid_total == 1


# ---------------------------------------------------------------------------
# 6-7. Oversize protocol-valid frames are not publishable
# ---------------------------------------------------------------------------


def test_6_721_byte_valid_frame_is_oversize_not_publishable():
    frame = frame_of_total_length(721)
    transport = new_transport()
    candidates = transport.process_stream_bytes(frame, 2.0)
    assert candidates == []
    assert transport.counters.rtcm_frames_valid_total == 1
    assert transport.counters.rtcm_bytes_valid_total == 721
    assert transport.counters.rtcm_frames_oversize_total == 1
    assert transport.counters.rtcm_frames_published_total == 0
    assert transport.last_valid_frame_at == 2.0
    assert transport.last_published_frame_at is None


def test_7_1029_byte_valid_frame_is_oversize_not_publishable():
    frame = frame_of_total_length(1029)
    transport = new_transport()
    candidates = transport.process_stream_bytes(frame, 2.0)
    assert candidates == []
    assert transport.counters.rtcm_frames_valid_total == 1
    assert transport.counters.rtcm_bytes_valid_total == 1029
    assert transport.counters.rtcm_frames_oversize_total == 1
    assert transport.counters.rtcm_frames_published_total == 0
    assert transport.last_valid_frame_at == 2.0
    assert transport.last_published_frame_at is None


# ---------------------------------------------------------------------------
# 8. CRC-invalid is not publishable
# ---------------------------------------------------------------------------


def test_8_crc_invalid_frame_is_not_publishable():
    good = frame_of_total_length(80)
    transport = new_transport()
    candidates = transport.process_stream_bytes(corrupt_crc(good), 1.0)
    assert candidates == []
    assert transport.counters.rtcm_frames_valid_total == 0
    assert transport.counters.rtcm_frames_crc_invalid_total == 1
    assert transport.counters.rtcm_frames_published_total == 0
    assert transport.last_valid_frame_at is None
    assert transport.last_published_frame_at is None


# ---------------------------------------------------------------------------
# 9-12. Chunk boundaries and handshake leftovers
# ---------------------------------------------------------------------------


@pytest.mark.parametrize('seed', range(8))
def test_9_random_tcp_chunk_boundaries_reconstruct_one_frame(seed):
    frame = frame_of_total_length(247)
    rng = random.Random(seed)
    transport = new_transport()
    emitted: list[bytes] = []
    now = 0.0
    for chunk in random_chunks(frame, rng):
        now += 0.01
        emitted.extend(transport.process_stream_bytes(chunk, now))
    assert emitted == [frame]
    assert transport.counters.rtcm_frames_valid_total == 1
    assert transport.counters.socket_bytes_received_total == len(frame)


def test_10_multiple_frames_in_one_recv_are_separate_candidates():
    first = frame_of_total_length(40)
    second = frame_of_total_length(90)
    third = frame_of_total_length(12)
    transport = new_transport()
    candidates = transport.process_stream_bytes(
        first + second + third,
        4.0,
    )
    assert candidates == [first, second, third]
    assert transport.counters.rtcm_frames_valid_total == 3


def test_11_handshake_partial_plus_recv_remainder_reconstructs_one_frame():
    frame = frame_of_total_length(200)
    split = 17
    transport = new_transport()
    handshake = transport.process_stream_bytes(frame[:split], 1.0)
    assert handshake == []
    remainder = transport.process_stream_bytes(frame[split:], 2.0)
    assert remainder == [frame]
    assert transport.counters.rtcm_frames_valid_total == 1
    assert transport.counters.socket_bytes_received_total == len(frame)


def test_12_handshake_payload_containing_complete_frame_emits_it():
    frame = frame_of_total_length(64)
    extra = frame_of_total_length(18)
    transport = new_transport()
    handshake = transport.process_stream_bytes(frame + extra[:5], 1.0)
    assert handshake == [frame]
    recv = transport.process_stream_bytes(extra[5:], 2.0)
    assert recv == [extra]


# ---------------------------------------------------------------------------
# 13-14. service() timeout with no new bytes
# ---------------------------------------------------------------------------


def test_13_partial_candidate_times_out_via_service_without_new_bytes():
    transport = new_transport()
    assert transport.process_stream_bytes(
        bytes((PREAMBLE, 0x00)),
        0.0,
    ) == []
    assert transport.counters.rtcm_partial_frame_timeouts_total == 0
    assert transport.service_parser(
        DEFAULT_PARTIAL_TIMEOUT_SEC - 0.001
    ) == []
    assert transport.counters.rtcm_partial_frame_timeouts_total == 0
    assert transport.service_parser(DEFAULT_PARTIAL_TIMEOUT_SEC) == []
    assert transport.counters.rtcm_partial_frame_timeouts_total == 1
    assert transport.parser is not None
    assert transport.parser.buffered_length == 0


def test_14_service_timeout_can_expose_trailing_valid_frame():
    good = make_rtcm3_frame(b'\x12\x34')
    stalled = bytes((PREAMBLE, 0x00, 80)) + b'\x00' * 4
    transport = new_transport()
    assert transport.process_stream_bytes(stalled + good, 0.0) == []
    exposed = transport.service_parser(DEFAULT_PARTIAL_TIMEOUT_SEC)
    assert exposed == [good]
    assert transport.counters.rtcm_frames_valid_total == 1
    assert transport.counters.rtcm_partial_frame_timeouts_total == 1


# ---------------------------------------------------------------------------
# 15. New parser session discards residual bytes
# ---------------------------------------------------------------------------


def test_15_new_parser_session_does_not_inherit_residual_bytes():
    frame = frame_of_total_length(120)
    transport = new_transport()
    assert transport.process_stream_bytes(frame[:20], 1.0) == []
    assert transport.parser is not None
    assert transport.parser.buffered_length == 20

    transport.new_parser_session()
    assert transport.parser is not None
    assert transport.parser.buffered_length == 0
    assert transport.last_socket_byte_at is None
    assert transport.last_valid_frame_at is None
    assert transport.last_published_frame_at is None

    leftover = transport.process_stream_bytes(frame[20:], 2.0)
    assert leftover == []
    assert transport.counters.rtcm_frames_valid_total == 0

    rebuilt = transport.process_stream_bytes(frame, 3.0)
    assert rebuilt == [frame]
    assert transport.counters.rtcm_frames_valid_total == 1


# ---------------------------------------------------------------------------
# 16-17. Garbage and oversize accounting
# ---------------------------------------------------------------------------


def test_16_raw_garbage_updates_socket_bytes_but_not_valid_or_published():
    garbage = bytes(range(PREAMBLE))
    transport = new_transport()
    candidates = transport.process_stream_bytes(garbage, 4.0)
    assert candidates == []
    assert transport.counters.socket_bytes_received_total == len(garbage)
    assert transport.last_socket_byte_at == 4.0
    assert transport.counters.rtcm_frames_valid_total == 0
    assert transport.counters.rtcm_frames_published_total == 0
    assert transport.last_valid_frame_at is None
    assert transport.last_published_frame_at is None
    assert transport.is_healthy(True, 4.0, 5.0) is False


def test_17_crc_valid_oversize_stream_does_not_count_as_published():
    frame = frame_of_total_length(800)
    transport = new_transport()
    publisher = RecordingPublisher()
    candidates = transport.process_stream_bytes(frame * 3, 5.0)
    assert candidates == []
    for candidate in candidates:
        transport.attempt_publish(candidate, 5.0, publisher)
    assert publisher.frames == []
    assert transport.counters.rtcm_frames_valid_total == 3
    assert transport.counters.rtcm_frames_oversize_total == 3
    assert transport.counters.rtcm_frames_published_total == 0
    assert transport.last_valid_frame_at == 5.0
    assert transport.last_published_frame_at is None
    assert transport.is_healthy(True, 5.0, 5.0) is False
    assert transport.source_is_stale(5.0, 10.0) is False


# ---------------------------------------------------------------------------
# 18-19. Publication success and failure bookkeeping
# ---------------------------------------------------------------------------


def test_18_publication_success_updates_published_state_only_once():
    frame = frame_of_total_length(100)
    transport = new_transport()
    publisher = RecordingPublisher()
    candidates = transport.process_stream_bytes(frame, 7.0)
    assert candidates == [frame]
    assert transport.attempt_publish(candidates[0], 7.0, publisher) is True
    assert publisher.frames == [frame]
    assert transport.counters.rtcm_frames_published_total == 1
    assert transport.last_published_frame_at == 7.0
    assert transport.service_parser(8.0) == []
    assert transport.counters.rtcm_frames_published_total == 1
    assert transport.last_published_frame_at == 7.0
    assert transport.counters.rtcm_publish_errors_total == 0


def test_19_publication_failure_does_not_update_published_state():
    frame = frame_of_total_length(50)
    transport = new_transport()
    publisher = FailingPublisher()
    candidates = transport.process_stream_bytes(frame, 3.0)
    assert candidates == [frame]
    assert transport.last_published_frame_at is None
    assert transport.attempt_publish(candidates[0], 3.0, publisher) is False
    assert publisher.calls == 1
    assert transport.counters.rtcm_publish_errors_total == 1
    assert transport.counters.rtcm_frames_published_total == 0
    assert transport.last_published_frame_at is None
    assert transport.counters.rtcm_frames_valid_total == 1


# ---------------------------------------------------------------------------
# 20-21. Session aggregation and deterministic lifetime accounting
# ---------------------------------------------------------------------------


def test_20_parser_counters_aggregate_across_sessions_without_double_count():
    first = frame_of_total_length(40)
    second = frame_of_total_length(60)
    transport = RtcmWorkerTransport()

    transport.new_parser_session()
    assert transport.process_stream_bytes(first, 1.0) == [first]
    assert transport.service_parser(1.5) == []
    assert transport.counters.rtcm_frames_valid_total == 1
    assert transport.counters.rtcm_bytes_valid_total == len(first)
    first_resync = transport.counters.rtcm_resync_bytes_discarded_total

    transport.new_parser_session()
    garbage = b'\x00\x01\x02'
    assert transport.process_stream_bytes(garbage + second, 2.0) == [second]
    assert transport.service_parser(2.5) == []
    assert transport.service_parser(3.0) == []

    assert transport.counters.rtcm_frames_valid_total == 2
    assert transport.counters.rtcm_bytes_valid_total == (
        len(first) + len(second)
    )
    assert transport.counters.socket_bytes_received_total == (
        len(first) + len(garbage) + len(second)
    )
    assert transport.counters.rtcm_resync_bytes_discarded_total == (
        first_resync + len(garbage)
    )


def test_21_worker_lifetime_byte_frame_accounting_is_deterministic():
    small = frame_of_total_length(24)
    oversize = frame_of_total_length(721)
    invalid = corrupt_crc(frame_of_total_length(30))
    garbage = b'\x11\x22\x33'
    transport = RtcmWorkerTransport()

    transport.new_parser_session()
    transport.process_stream_bytes(garbage + small + invalid, 1.0)
    transport.process_stream_bytes(oversize, 2.0)
    publisher = RecordingPublisher()
    for candidate in transport.process_stream_bytes(small, 3.0):
        assert transport.attempt_publish(candidate, 3.0, publisher)

    transport.new_parser_session()
    transport.process_stream_bytes(small, 4.0)
    transport.service_parser(4.5)
    transport.service_parser(5.0)

    counters = transport.counters
    assert counters.socket_bytes_received_total == (
        len(garbage) + len(small) + len(invalid) + len(oversize) + len(small)
        + len(small)
    )
    assert counters.rtcm_frames_valid_total == 4
    assert counters.rtcm_bytes_valid_total == (
        len(small) + len(oversize) + len(small) + len(small)
    )
    assert counters.rtcm_frames_crc_invalid_total == 1
    assert counters.rtcm_frames_oversize_total == 1
    assert counters.rtcm_frames_published_total == 1
    assert counters.rtcm_publish_errors_total == 0
    assert publisher.frames == [small]


# ---------------------------------------------------------------------------
# Health / reconnect semantics used by the worker
# ---------------------------------------------------------------------------


def test_health_false_until_supported_frame_is_published():
    transport = new_transport()
    assert transport.is_healthy(True, 0.0, 5.0) is False
    assert transport.published_age_sec(0.0) == float('inf')
    transport.process_stream_bytes(b'\x00\x01', 1.0)
    assert transport.is_healthy(True, 1.0, 5.0) is False
    transport.process_stream_bytes(frame_of_total_length(721), 2.0)
    assert transport.is_healthy(True, 2.0, 5.0) is False
    frame = frame_of_total_length(20)
    candidates = transport.process_stream_bytes(frame, 3.0)
    transport.attempt_publish(candidates[0], 3.0, RecordingPublisher())
    assert transport.is_healthy(True, 3.0, 5.0) is True
    assert transport.is_healthy(False, 3.0, 5.0) is False
    assert transport.is_healthy(True, 8.01, 5.0) is False
    assert transport.published_age_sec(4.0) == pytest.approx(1.0)


def test_first_valid_frame_timeout_ignores_socket_garbage():
    transport = new_transport()
    transport.process_stream_bytes(b'\x00' * 40, 1.0)
    assert transport.last_socket_byte_at == 1.0
    assert transport.first_valid_frame_timed_out(0.0, 9.9, 10.0) is False
    assert transport.first_valid_frame_timed_out(0.0, 10.1, 10.0) is True
    transport.process_stream_bytes(frame_of_total_length(721), 11.0)
    assert transport.first_valid_frame_timed_out(0.0, 20.0, 10.0) is False


def test_source_staleness_uses_valid_frame_not_published_age():
    transport = new_transport()
    transport.process_stream_bytes(frame_of_total_length(800), 10.0)
    assert transport.last_published_frame_at is None
    assert transport.source_is_stale(19.9, 10.0) is False
    assert transport.source_is_stale(20.1, 10.0) is True
    transport.process_stream_bytes(frame_of_total_length(800), 21.0)
    assert transport.source_is_stale(30.0, 10.0) is False
    assert transport.is_healthy(True, 21.0, 5.0) is False


def test_new_session_does_not_inherit_previous_publication_freshness():
    healthy_age_sec = 5.0
    transport = RtcmWorkerTransport()
    publisher = RecordingPublisher()

    # 1. Session A publishes a supported frame at t=100.
    transport.new_parser_session()
    frame_a = frame_of_total_length(80)
    candidates = transport.process_stream_bytes(frame_a, 100.0)
    assert candidates == [frame_a]
    assert transport.attempt_publish(candidates[0], 100.0, publisher) is True
    assert transport.is_healthy(True, 100.0, healthy_age_sec) is True
    assert transport.last_published_frame_at == 100.0
    session_a_published = transport.counters.rtcm_frames_published_total
    session_a_valid = transport.counters.rtcm_frames_valid_total
    session_a_socket_bytes = transport.counters.socket_bytes_received_total
    assert session_a_published == 1
    assert session_a_valid == 1

    # 2-3. Session B starts at t=101 with no RTCM yet.
    # Session A published 1s ago, well inside healthy_age_sec.
    transport.new_parser_session()
    assert transport.last_socket_byte_at is None
    assert transport.last_valid_frame_at is None
    assert transport.last_published_frame_at is None
    assert transport.is_healthy(True, 101.0, healthy_age_sec) is False
    assert transport.published_age_sec(101.0) == float('inf')
    assert session_a_published == transport.counters.rtcm_frames_published_total
    assert session_a_valid == transport.counters.rtcm_frames_valid_total
    assert (
        session_a_socket_bytes
        == transport.counters.socket_bytes_received_total
    )

    # 4. Raw garbage updates socket time only.
    garbage = b'\x00\x01\x02'
    assert transport.process_stream_bytes(garbage, 102.0) == []
    assert transport.last_socket_byte_at == 102.0
    assert transport.last_valid_frame_at is None
    assert transport.last_published_frame_at is None
    assert transport.is_healthy(True, 102.0, healthy_age_sec) is False
    assert transport.published_age_sec(102.0) == float('inf')

    # 5. CRC-valid oversize updates valid time only.
    oversize = frame_of_total_length(721)
    assert transport.process_stream_bytes(oversize, 103.0) == []
    assert transport.last_valid_frame_at == 103.0
    assert transport.last_published_frame_at is None
    assert transport.is_healthy(True, 103.0, healthy_age_sec) is False
    assert transport.published_age_sec(103.0) == float('inf')

    # 6. Session B publishes a supported frame; session health may become true.
    frame_b = frame_of_total_length(40)
    candidates = transport.process_stream_bytes(frame_b, 104.0)
    assert candidates == [frame_b]
    assert transport.attempt_publish(candidates[0], 104.0, publisher) is True
    assert transport.last_published_frame_at == 104.0
    assert transport.is_healthy(True, 104.0, healthy_age_sec) is True
    assert transport.published_age_sec(104.0) == pytest.approx(0.0)

    # 7. Worker-lifetime counters from Session A remain present.
    assert transport.counters.rtcm_frames_published_total == 2
    assert transport.counters.rtcm_frames_valid_total == 3
    assert transport.counters.rtcm_frames_oversize_total == 1
    assert transport.counters.socket_bytes_received_total == (
        session_a_socket_bytes
        + len(garbage)
        + len(oversize)
        + len(frame_b)
    )


def _half_second_ticks(start_sec: float, end_sec: float) -> list[float]:
    ticks = []
    now = start_sec
    while now <= end_sec + 1e-9:
        ticks.append(round(now, 1))
        now += 0.5
    return ticks


def test_continuous_garbage_does_not_postpone_first_valid_deadline():
    transport = new_transport()
    connection_start = 0.0
    first_data_timeout = 10.0
    stale_reconnect = 10.0
    garbage = b'\x00\x01\x02\x04'
    deadline_at = None
    for now in _half_second_ticks(0.5, 10.5):
        assert transport.process_stream_bytes(garbage, now) == []
        elapsed = transport.source_deadline_elapsed(
            connection_start,
            now,
            first_data_timeout,
            stale_reconnect,
        )
        if now <= first_data_timeout:
            assert elapsed is False
        else:
            assert elapsed is True
            if deadline_at is None:
                deadline_at = now
    assert deadline_at == 10.5
    assert transport.last_valid_frame_at is None
    assert transport.last_published_frame_at is None
    assert transport.last_socket_byte_at == 10.5


def test_continuous_crc_invalid_does_not_postpone_first_valid_deadline():
    transport = new_transport()
    connection_start = 0.0
    invalid = corrupt_crc(frame_of_total_length(40))
    deadline_at = None
    for now in _half_second_ticks(0.5, 10.5):
        assert transport.process_stream_bytes(invalid, now) == []
        elapsed = transport.source_deadline_elapsed(
            connection_start,
            now,
            10.0,
            10.0,
        )
        if now <= 10.0:
            assert elapsed is False
        else:
            assert elapsed is True
            if deadline_at is None:
                deadline_at = now
    assert deadline_at == 10.5
    assert transport.last_valid_frame_at is None
    assert transport.counters.rtcm_frames_crc_invalid_total >= 1
    assert transport.last_published_frame_at is None


def test_continuous_invalid_traffic_does_not_postpone_stale_source_deadline():
    transport = new_transport()
    connection_start = 0.0
    supported = frame_of_total_length(50)
    candidates = transport.process_stream_bytes(supported, 1.0)
    assert candidates == [supported]
    assert transport.last_valid_frame_at == 1.0
    assert transport.source_deadline_elapsed(
        connection_start,
        1.0,
        10.0,
        10.0,
    ) is False

    garbage = b'\x00\x01\x02'
    invalid = corrupt_crc(frame_of_total_length(30))
    deadline_at = None
    for index, now in enumerate(_half_second_ticks(1.5, 11.5)):
        payload = garbage if index % 2 == 0 else invalid
        assert transport.process_stream_bytes(payload, now) == []
        elapsed = transport.source_deadline_elapsed(
            connection_start,
            now,
            10.0,
            10.0,
        )
        if now - 1.0 <= 10.0:
            assert elapsed is False
            assert transport.last_valid_frame_at == 1.0
        else:
            assert elapsed is True
            if deadline_at is None:
                deadline_at = now
    assert deadline_at == 11.5
    assert transport.last_valid_frame_at == 1.0
    assert transport.last_socket_byte_at == 11.5


def test_continuous_oversize_valid_frames_keep_source_fresh():
    transport = new_transport()
    connection_start = 0.0
    oversize = frame_of_total_length(721)
    for now in _half_second_ticks(0.5, 20.0):
        assert transport.process_stream_bytes(oversize, now) == []
        assert transport.last_valid_frame_at == now
        assert transport.source_deadline_elapsed(
            connection_start,
            now,
            10.0,
            10.0,
        ) is False
        assert transport.is_healthy(True, now, 5.0) is False
    assert transport.last_published_frame_at is None
    assert transport.counters.rtcm_frames_oversize_total == len(
        _half_second_ticks(0.5, 20.0)
    )


def test_supported_valid_frame_refreshes_source_valid_age():
    transport = new_transport()
    connection_start = 0.0
    supported = frame_of_total_length(80)
    publisher = RecordingPublisher()
    for now in _half_second_ticks(0.5, 20.0):
        candidates = transport.process_stream_bytes(supported, now)
        assert candidates == [supported]
        assert transport.attempt_publish(
            candidates[0],
            now,
            publisher,
        ) is True
        assert transport.last_valid_frame_at == now
        assert transport.last_published_frame_at == now
        assert transport.source_deadline_elapsed(
            connection_start,
            now,
            10.0,
            10.0,
        ) is False
        assert transport.source_is_stale(now, 10.0) is False
    assert transport.is_healthy(True, 20.0, 5.0) is True
