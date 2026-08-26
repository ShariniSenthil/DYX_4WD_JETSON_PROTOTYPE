"""Unit tests for the ROS-free RTCM3 parser.

These tests cover CRC-24Q, incremental framing, resynchronization, buffer
bounds, partial-candidate timeout, and deterministic counters. They do not
exercise NTRIP, MAVROS, or ROS publication policy.
"""

from __future__ import annotations

import math
import random

import pytest

from rtk_correction_bridge.rtcm3_parser import (
    CRC24Q_CHECK_ASCII,
    CRC24Q_CHECK_VALUE,
    DEFAULT_BUFFER_LIMIT,
    DEFAULT_PARTIAL_TIMEOUT_SEC,
    MAX_FRAME_LENGTH,
    MAX_PAYLOAD_LENGTH,
    PREAMBLE,
    RTCM3Frame,
    RTCM3Parser,
    crc24q,
)


def make_rtcm3_frame(payload: bytes) -> bytes:
    """Build one protocol-valid RTCM3 frame around ``payload``."""
    if len(payload) > MAX_PAYLOAD_LENGTH:
        raise ValueError('payload exceeds RTCM3 maximum')
    length = len(payload)
    header = bytes((PREAMBLE, (length >> 8) & 0x03, length & 0xFF))
    covered = header + payload
    return covered + crc24q(covered).to_bytes(3, 'big')


def payload_with_type(message_type: int, extra: bytes = b'') -> bytes:
    """Encode a 12-bit RTCM message number plus optional payload tail."""
    if not 0 <= message_type <= 0xFFF:
        raise ValueError('message_type must fit in 12 bits')
    first = (message_type >> 4) & 0xFF
    second = (message_type & 0x0F) << 4
    if extra:
        second = (second & 0xF0) | (extra[0] & 0x0F)
        return bytes((first, second)) + extra[1:]
    return bytes((first, second))


def noise_without_preamble(rng: random.Random, size: int) -> bytes:
    """Return ``size`` random bytes that never contain 0xD3."""
    return bytes(rng.choice(range(0, PREAMBLE)) for _ in range(size))


def frame_bytes_of(frames: list[RTCM3Frame]) -> list[bytes]:
    return [frame.frame_bytes for frame in frames]


def feed_chunks(
    parser: RTCM3Parser,
    chunks: list[bytes],
    now_sec: float,
) -> list[RTCM3Frame]:
    frames: list[RTCM3Frame] = []
    for chunk in chunks:
        frames.extend(parser.feed(chunk, now_sec))
    return frames


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
# 1. CRC-24Q
# ---------------------------------------------------------------------------


def test_crc24q_known_ascii_vector():
    assert crc24q(CRC24Q_CHECK_ASCII) == CRC24Q_CHECK_VALUE
    assert crc24q(b'123456789') == 0xCDE703


def test_crc24q_empty_is_zero():
    assert crc24q(b'') == 0


def test_crc24q_rejects_non_bytes():
    with pytest.raises(TypeError):
        crc24q('123456789')  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# 2-6. Framing: one feed, splits, byte-by-byte, multi, partial next
# ---------------------------------------------------------------------------


def test_one_valid_frame_in_one_feed():
    payload = payload_with_type(1077, b'\x00\x11\x22')
    frame = make_rtcm3_frame(payload)
    parser = RTCM3Parser()
    frames = parser.feed(frame, 0.0)
    assert len(frames) == 1
    assert frames[0].frame_bytes == frame
    assert frames[0].payload_length == len(payload)
    assert frames[0].total_length == len(frame)
    assert frames[0].message_type == 1077
    assert parser.buffered_length == 0


@pytest.mark.parametrize(
    'payload_length',
    [0, 1, 2, 10, 50, 715, 1023],
)
def test_frame_split_at_every_byte_boundary(payload_length):
    payload = bytes((index * 17) % 256 for index in range(payload_length))
    frame = make_rtcm3_frame(payload)
    assert len(frame) == 3 + payload_length + 3
    for split in range(1, len(frame)):
        parser = RTCM3Parser()
        first = parser.feed(frame[:split], 0.0)
        assert first == []
        second = parser.feed(frame[split:], 0.0)
        assert frame_bytes_of(second) == [frame]
        assert parser.buffered_length == 0


@pytest.mark.parametrize('payload_length', [0, 1, 2, 16, 715, 1023])
def test_byte_by_byte_feeding(payload_length):
    payload = bytes((index * 13) % 256 for index in range(payload_length))
    frame = make_rtcm3_frame(payload)
    parser = RTCM3Parser()
    emitted: list[RTCM3Frame] = []
    for index, byte in enumerate(frame):
        out = parser.feed(bytes((byte,)), 0.0)
        if index < len(frame) - 1:
            assert out == []
        emitted.extend(out)
    assert frame_bytes_of(emitted) == [frame]


def test_multiple_valid_frames_in_one_feed():
    frames_in = [
        make_rtcm3_frame(b''),
        make_rtcm3_frame(b'\xaa'),
        make_rtcm3_frame(payload_with_type(1005, b'\x01\x02\x03\x04')),
        make_rtcm3_frame(bytes(range(40))),
    ]
    parser = RTCM3Parser()
    out = parser.feed(b''.join(frames_in), 0.0)
    assert frame_bytes_of(out) == frames_in


def test_complete_frame_plus_partial_next():
    first = make_rtcm3_frame(payload_with_type(1074, b'\x10\x20'))
    second = make_rtcm3_frame(payload_with_type(1084, b'\x30\x40\x50'))
    parser = RTCM3Parser()
    split = 4
    out = parser.feed(first + second[:split], 0.0)
    assert frame_bytes_of(out) == [first]
    assert parser.buffered_length == split
    out = parser.feed(second[split:], 0.0)
    assert frame_bytes_of(out) == [second]
    assert parser.buffered_length == 0


# ---------------------------------------------------------------------------
# 7-12. Noise, false preambles, invalid header, CRC corruption, recovery
# ---------------------------------------------------------------------------


def test_leading_garbage_before_valid_frame():
    frame = make_rtcm3_frame(payload_with_type(1234, b'\x00\x01'))
    garbage = bytes(range(PREAMBLE))  # 0x00..0xD2, no preamble
    parser = RTCM3Parser()
    out = parser.feed(garbage + frame, 0.0)
    assert frame_bytes_of(out) == [frame]
    assert parser.counters.resync_bytes_discarded_total == len(garbage)


def test_repeated_false_d3_bytes():
    frame = make_rtcm3_frame(b'\x12\x30\x00')
    # 0xD3 0xD3 0xD3 is an invalid reserved-bit header (byte1 = 0xD3).
    false_preambles = bytes([PREAMBLE, PREAMBLE, PREAMBLE, PREAMBLE])
    parser = RTCM3Parser()
    out = parser.feed(false_preambles + frame, 0.0)
    assert frame_bytes_of(out) == [frame]
    assert parser.counters.headers_invalid_total >= 1
    assert parser.counters.frames_valid_total == 1


def test_invalid_reserved_bits():
    parser = RTCM3Parser()
    # Reserved bits in header[1] bits 7..2 must be zero. 0xFC is all reserved.
    candidate = bytes((PREAMBLE, 0xFC, 0x00, 0x11, 0x22, 0x33))
    out = parser.feed(candidate, 0.0)
    assert out == []
    assert parser.counters.headers_invalid_total == 1
    assert parser.counters.frames_valid_total == 0
    assert parser.counters.resync_bytes_discarded_total == len(candidate)


@pytest.mark.parametrize('corrupt_index_name', ['header', 'payload', 'crc'])
def test_crc_corruption_of_header_payload_and_crc(corrupt_index_name):
    # Odd payload length so a 1-bit length-low flip still yields a complete
    # candidate (length decreases) rather than waiting for extra bytes.
    payload = payload_with_type(1077, b'\x01\x02\x03\x04')
    good = make_rtcm3_frame(payload)
    assert len(payload) % 2 == 1
    if corrupt_index_name == 'header':
        index = 2
    elif corrupt_index_name == 'payload':
        index = 4
    else:
        index = len(good) - 1
    corrupt = bytearray(good)
    corrupt[index] ^= 0x01
    parser = RTCM3Parser()
    out = parser.feed(bytes(corrupt), 0.0)
    assert out == []
    if corrupt_index_name == 'header' and (corrupt[1] & 0xFC) != 0:
        assert parser.counters.headers_invalid_total >= 1
    else:
        assert parser.counters.frames_crc_invalid_total >= 1
    assert parser.counters.frames_valid_total == 0


def test_corrupt_candidate_immediately_followed_by_valid_frame():
    good = make_rtcm3_frame(payload_with_type(4001, b'\xaa\xbb\xcc'))
    corrupt = bytearray(make_rtcm3_frame(payload_with_type(4002, b'\x11\x22')))
    corrupt[-2] ^= 0xFF
    parser = RTCM3Parser()
    out = parser.feed(bytes(corrupt) + good, 0.0)
    assert frame_bytes_of(out) == [good]
    assert parser.counters.frames_crc_invalid_total >= 1
    assert parser.counters.frames_valid_total == 1


def test_parser_recovers_without_losing_next_frame():
    first = make_rtcm3_frame(payload_with_type(1005, b'\x01'))
    second = make_rtcm3_frame(payload_with_type(1033, b'\x02\x03'))
    third = make_rtcm3_frame(payload_with_type(1077, b'\x04\x05\x06'))
    corrupt = bytearray(second)
    corrupt[5] ^= 0x80
    noise = b'\x00\x01\x02\xff'
    parser = RTCM3Parser()
    stream = first + noise + bytes([PREAMBLE, PREAMBLE]) + bytes(corrupt) + third
    out = parser.feed(stream, 0.0)
    assert frame_bytes_of(out) == [first, third]
    assert parser.counters.frames_valid_total == 2


def test_false_preamble_inside_payload_does_not_split_valid_frame():
    payload = bytes([PREAMBLE, PREAMBLE, 0x00, 0x01, PREAMBLE, 0x02])
    frame = make_rtcm3_frame(payload)
    parser = RTCM3Parser()
    out = parser.feed(frame, 0.0)
    assert frame_bytes_of(out) == [frame]
    assert out[0].payload_length == len(payload)


# ---------------------------------------------------------------------------
# 13-16. Short payloads and message-type decoding
# ---------------------------------------------------------------------------


def test_zero_payload_valid_frame():
    frame = make_rtcm3_frame(b'')
    assert len(frame) == 6
    parser = RTCM3Parser()
    out = parser.feed(frame, 0.0)
    assert len(out) == 1
    assert out[0].payload_length == 0
    assert out[0].total_length == 6
    assert out[0].message_type is None
    assert out[0].frame_bytes == frame


def test_one_byte_payload():
    frame = make_rtcm3_frame(b'\xa5')
    parser = RTCM3Parser()
    out = parser.feed(frame, 0.0)
    assert len(out) == 1
    assert out[0].payload_length == 1
    assert out[0].message_type is None


def test_message_type_decoding():
    cases = {
        1005: payload_with_type(1005, b'\x00'),
        1074: payload_with_type(1074),
        1077: payload_with_type(1077, b'\x0f\x00'),
        1127: payload_with_type(1127, b'\x01\x02'),
        0: payload_with_type(0, b'\x00'),
        4095: payload_with_type(4095, b'\x00'),
    }
    parser = RTCM3Parser()
    for message_type, payload in cases.items():
        frame = make_rtcm3_frame(payload)
        out = parser.feed(frame, 0.0)
        assert len(out) == 1
        assert out[0].message_type == message_type


def test_unknown_message_type_still_accepted():
    payload = payload_with_type(3999, b'\x10\x20\x30')
    frame = make_rtcm3_frame(payload)
    parser = RTCM3Parser()
    out = parser.feed(frame, 0.0)
    assert len(out) == 1
    assert out[0].message_type == 3999
    assert out[0].frame_bytes == frame


# ---------------------------------------------------------------------------
# 17-18. Protocol maximum and MAVROS-sized 721-byte frame
# ---------------------------------------------------------------------------


def test_maximum_protocol_frame_1023_payload_1029_total():
    payload = bytes((index * 19) % 256 for index in range(MAX_PAYLOAD_LENGTH))
    frame = make_rtcm3_frame(payload)
    assert len(frame) == MAX_FRAME_LENGTH
    parser = RTCM3Parser()
    out = parser.feed(frame, 0.0)
    assert len(out) == 1
    assert out[0].payload_length == 1023
    assert out[0].total_length == 1029
    assert out[0].frame_bytes == frame
    assert parser.counters.bytes_valid_total == 1029


def test_721_byte_frame_is_rtcm_valid():
    payload = bytes((index * 23) % 256 for index in range(715))
    frame = make_rtcm3_frame(payload)
    assert len(frame) == 721
    parser = RTCM3Parser()
    out = parser.feed(frame, 0.0)
    assert len(out) == 1
    assert out[0].total_length == 721
    assert out[0].frame_bytes == frame
    # Parser must not apply the 720-byte MAVROS transport policy.
    assert parser.counters.frames_valid_total == 1
    assert parser.counters.bytes_valid_total == 721


# ---------------------------------------------------------------------------
# 19-20. Randomized socket chunking and noise
# ---------------------------------------------------------------------------


@pytest.mark.parametrize('seed', [1, 7, 13, 42, 99, 256, 1024, 2026])
def test_randomized_socket_chunking_of_known_multi_frame_stream(seed):
    rng = random.Random(seed)
    originals = []
    for _ in range(rng.randint(3, 12)):
        length = rng.randint(0, 80)
        originals.append(make_rtcm3_frame(rng.randbytes(length)))
    stream = b''.join(originals)
    parser = RTCM3Parser()
    out = feed_chunks(parser, random_chunks(stream, rng), 0.0)
    assert frame_bytes_of(out) == originals
    assert parser.buffered_length == 0
    assert parser.counters.frames_valid_total == len(originals)
    assert parser.counters.bytes_valid_total == len(stream)


@pytest.mark.parametrize('seed', [2, 8, 21, 77, 128, 999])
def test_random_leading_and_inter_frame_noise(seed):
    rng = random.Random(seed)
    originals = [
        make_rtcm3_frame(rng.randbytes(rng.randint(0, 40)))
        for _ in range(rng.randint(2, 8))
    ]
    parts = [noise_without_preamble(rng, rng.randint(0, 30))]
    for frame in originals:
        parts.append(frame)
        parts.append(noise_without_preamble(rng, rng.randint(0, 30)))
    stream = b''.join(parts)
    parser = RTCM3Parser()
    out = feed_chunks(parser, random_chunks(stream, rng), 0.0)
    assert frame_bytes_of(out) == originals
    noise_len = len(stream) - sum(len(frame) for frame in originals)
    assert parser.counters.resync_bytes_discarded_total == noise_len
    assert parser.counters.frames_valid_total == len(originals)


# ---------------------------------------------------------------------------
# 21. 64 KiB garbage / buffer bound
# ---------------------------------------------------------------------------


def test_64kib_garbage_is_discarded_and_buffer_stays_bounded():
    parser = RTCM3Parser()
    garbage = b'\x00' * DEFAULT_BUFFER_LIMIT
    out = parser.feed(garbage, 0.0)
    assert out == []
    assert parser.buffered_length == 0
    assert parser.counters.resync_bytes_discarded_total == DEFAULT_BUFFER_LIMIT
    assert parser.buffered_length <= DEFAULT_BUFFER_LIMIT


def test_parse_before_trim_keeps_frame_after_64kib_garbage():
    frame = make_rtcm3_frame(payload_with_type(1077, b'\x01\x02'))
    parser = RTCM3Parser()
    out = parser.feed(b'\x11' * DEFAULT_BUFFER_LIMIT + frame, 0.0)
    assert frame_bytes_of(out) == [frame]
    assert parser.buffered_length == 0


def test_double_buffer_limit_garbage_does_not_grow_unbounded():
    parser = RTCM3Parser()
    out = parser.feed(b'\x22' * (DEFAULT_BUFFER_LIMIT * 2), 0.0)
    assert out == []
    assert parser.buffered_length <= DEFAULT_BUFFER_LIMIT
    assert parser.counters.resync_bytes_discarded_total == DEFAULT_BUFFER_LIMIT * 2


def test_legitimate_partial_at_zero_is_preserved():
    parser = RTCM3Parser()
    header = bytes((PREAMBLE, 0x00, 100))
    body = b'\x05' * 10
    parser.feed(header + body, 0.0)
    assert parser.buffered_length == len(header) + len(body)
    assert parser.counters.frames_valid_total == 0
    remainder = make_rtcm3_frame(b'\x05' * 100)[len(header) + len(body):]
    out = parser.feed(remainder, 0.0)
    assert len(out) == 1
    assert out[0].payload_length == 100


# ---------------------------------------------------------------------------
# 22-24. Partial-candidate timeout
# ---------------------------------------------------------------------------


def test_partial_candidate_timeout():
    parser = RTCM3Parser()
    out = parser.feed(bytes((PREAMBLE, 0x00)), now_sec=0.0)
    assert out == []
    assert parser.counters.partial_frame_timeouts_total == 0
    out = parser.service(DEFAULT_PARTIAL_TIMEOUT_SEC - 0.001)
    assert out == []
    assert parser.counters.partial_frame_timeouts_total == 0
    out = parser.service(DEFAULT_PARTIAL_TIMEOUT_SEC)
    assert out == []
    assert parser.counters.partial_frame_timeouts_total == 1
    assert parser.buffered_length == 0


def test_new_bytes_do_not_reset_partial_timeout():
    parser = RTCM3Parser()
    # Payload length 50 requires 56 total bytes.
    parser.feed(bytes((PREAMBLE, 0x00, 50)), now_sec=0.0)
    parser.feed(b'\x01' * 10, now_sec=1.5)
    assert parser.counters.partial_frame_timeouts_total == 0
    assert parser.buffered_length == 13
    parser.service(2.0)
    assert parser.counters.partial_frame_timeouts_total == 1
    # If the 1.5s feed had reset the timer, 2.0s would not yet expire.


def test_multiple_false_partial_candidates_recover_sequentially():
    good = make_rtcm3_frame(payload_with_type(1077, b'\x99\x88\x77'))
    partial_a = bytes((PREAMBLE, 0x00, 200)) + b'\x11' * 8
    partial_b = bytes((PREAMBLE, 0x00, 180)) + b'\x22' * 8
    parser = RTCM3Parser()
    out = parser.feed(partial_a + partial_b + good, now_sec=0.0)
    assert out == []
    assert parser.counters.partial_frame_timeouts_total == 0

    out = parser.service(2.0)
    assert out == []
    assert parser.counters.partial_frame_timeouts_total == 1

    out = parser.service(4.0)
    assert frame_bytes_of(out) == [good]
    assert parser.counters.partial_frame_timeouts_total == 2
    assert parser.counters.frames_valid_total == 1


def test_service_without_candidate_is_noop():
    parser = RTCM3Parser()
    assert parser.service(10.0) == []
    assert parser.counters.partial_frame_timeouts_total == 0


def test_timeout_discards_only_preamble_and_rescans():
    good = make_rtcm3_frame(b'\x12\x34')
    parser = RTCM3Parser()
    parser.feed(bytes((PREAMBLE, 0x00, 80)) + b'\x00' * 4 + good, now_sec=0.0)
    out = parser.service(2.0)
    assert frame_bytes_of(out) == [good]


# ---------------------------------------------------------------------------
# 25. Counter semantics
# ---------------------------------------------------------------------------


def test_counters_match_scripted_sequence():
    parser = RTCM3Parser()
    garbage = b'\x00\x01\x02'
    zero = make_rtcm3_frame(b'')
    invalid_header = bytes((PREAMBLE, 0xFC, 0x00, 0x11, 0x22, 0x33))
    good = make_rtcm3_frame(payload_with_type(1005, b'\x00\x01'))
    corrupt = bytearray(make_rtcm3_frame(payload_with_type(1005, b'\x00\x02')))
    # Flip a CRC byte that is not 0xD3 after mutation, if possible.
    corrupt[-1] ^= 0x01
    if corrupt[-1] == PREAMBLE:
        corrupt[-1] ^= 0x02
    corrupt_bytes = bytes(corrupt)

    parser.feed(garbage, 0.0)
    parser.feed(zero, 0.0)
    parser.feed(invalid_header, 0.0)
    parser.feed(corrupt_bytes, 0.0)
    parser.feed(good, 0.0)
    parser.feed(bytes((PREAMBLE, 0x00)), now_sec=0.0)
    parser.service(2.0)

    counters = parser.counters
    assert counters.bytes_fed_total == (
        len(garbage)
        + len(zero)
        + len(invalid_header)
        + len(corrupt_bytes)
        + len(good)
        + 2
    )
    assert counters.frames_valid_total == 2
    assert counters.bytes_valid_total == len(zero) + len(good)
    assert counters.headers_invalid_total == 1
    assert counters.frames_crc_invalid_total == 1
    assert counters.partial_frame_timeouts_total == 1
    # Every fed byte was either counted as valid-frame bytes or discarded.
    assert (
        counters.bytes_valid_total + counters.resync_bytes_discarded_total
        == counters.bytes_fed_total
    )
    assert parser.buffered_length == 0
    assert counters.bytes_fed_total == (
        counters.bytes_valid_total + counters.resync_bytes_discarded_total
    )


def test_counters_snapshot_is_immutable():
    parser = RTCM3Parser()
    parser.feed(make_rtcm3_frame(b'\x00\x10'), 0.0)
    snapshot = parser.counters
    parser.feed(make_rtcm3_frame(b'\x00\x20'), 0.0)
    assert snapshot.frames_valid_total == 1
    assert parser.counters.frames_valid_total == 2


# ---------------------------------------------------------------------------
# Adversarial / property tests
# ---------------------------------------------------------------------------


@pytest.mark.parametrize('seed', list(range(30)))
def test_property_valid_frames_emerge_byte_identical(seed):
    rng = random.Random(1000 + seed)
    originals = []
    for _ in range(rng.randint(1, 15)):
        length = rng.choice([0, 1, 2, rng.randint(3, 120), 180, 715])
        if length > MAX_PAYLOAD_LENGTH:
            length = MAX_PAYLOAD_LENGTH
        originals.append(make_rtcm3_frame(rng.randbytes(length)))

    parts: list[bytes] = []
    for frame in originals:
        parts.append(noise_without_preamble(rng, rng.randint(0, 40)))
        parts.append(frame)
    parts.append(noise_without_preamble(rng, rng.randint(0, 40)))
    stream = b''.join(parts)

    parser = RTCM3Parser()
    out = feed_chunks(parser, random_chunks(stream, rng), now_sec=0.0)
    assert frame_bytes_of(out) == originals
    assert parser.buffered_length <= DEFAULT_BUFFER_LIMIT
    assert parser.counters.frames_valid_total == len(originals)
    assert parser.counters.bytes_valid_total == sum(
        len(frame) for frame in originals
    )


@pytest.mark.parametrize('seed', list(range(20)))
def test_property_crc_mutations_never_emit(seed):
    rng = random.Random(2000 + seed)
    good = make_rtcm3_frame(rng.randbytes(rng.randint(2, 60)))
    follower = make_rtcm3_frame(rng.randbytes(rng.randint(2, 60)))
    corrupt = bytearray(good)
    # Mutate payload or CRC only so declared length stays complete.
    index = rng.randrange(3, len(corrupt))
    corrupt[index] = corrupt[index] ^ rng.randint(1, 255)
    parser = RTCM3Parser()
    out = parser.feed(bytes(corrupt) + follower, 0.0)
    assert follower in frame_bytes_of(out)
    assert good not in frame_bytes_of(out)
    assert parser.counters.frames_valid_total >= 1
    # The mutated copy must never be emitted.
    for frame in out:
        assert frame.frame_bytes != bytes(corrupt)


@pytest.mark.parametrize('seed', list(range(10)))
def test_property_buffer_remains_bounded_under_noise(seed):
    rng = random.Random(3000 + seed)
    parser = RTCM3Parser()
    for _ in range(20):
        chunk = noise_without_preamble(rng, rng.randint(1, 8000))
        parser.feed(chunk, 0.0)
        assert parser.buffered_length <= DEFAULT_BUFFER_LIMIT
    parser.feed(b'\x00' * DEFAULT_BUFFER_LIMIT, 0.0)
    assert parser.buffered_length <= DEFAULT_BUFFER_LIMIT
    assert parser.counters.frames_valid_total == 0


def test_buffer_limit_rejects_values_below_protocol_max():
    with pytest.raises(ValueError):
        RTCM3Parser(buffer_limit=1028)


def test_feed_rejects_non_bytes():
    parser = RTCM3Parser()
    with pytest.raises(TypeError):
        parser.feed('d3', 0.0)  # type: ignore[arg-type]


def test_empty_feed_does_not_change_counters():
    parser = RTCM3Parser()
    parser.feed(b'', 0.0)
    counters = parser.counters
    assert counters.bytes_fed_total == 0
    assert counters.frames_valid_total == 0
    assert parser.buffered_length == 0


# ---------------------------------------------------------------------------
# Phase-1A review: required finite now_sec and internal-preamble recovery
# ---------------------------------------------------------------------------


def test_feed_requires_now_sec():
    parser = RTCM3Parser()
    with pytest.raises(TypeError):
        parser.feed(b'\xd3\x00')


@pytest.mark.parametrize('now_sec', [math.nan, math.inf, -math.inf])
def test_nonfinite_now_sec_rejected(now_sec):
    parser = RTCM3Parser()
    with pytest.raises(ValueError, match='finite'):
        parser.feed(b'\xd3', now_sec)
    with pytest.raises(ValueError, match='finite'):
        parser.service(now_sec)


def test_candidate_timer_starts_when_preamble_becomes_buffer_zero():
    parser = RTCM3Parser()
    # Incomplete candidate: payload length 50 requires 56 total bytes.
    out = parser.feed(bytes((PREAMBLE, 0x00, 50)), 10.0)
    assert out == []
    out = parser.feed(b'\x01' * 10, 11.0)
    assert out == []
    assert parser.counters.partial_frame_timeouts_total == 0
    out = parser.service(11.99)
    assert out == []
    assert parser.counters.partial_frame_timeouts_total == 0
    out = parser.service(12.0)
    assert out == []
    assert parser.counters.partial_frame_timeouts_total == 1
    # If the t=11.0 feed had reset the timer, t=12.0 would not yet expire.


def test_adversarial_internal_preamble_recovers_after_timeout():
    good = make_rtcm3_frame(payload_with_type(1077, b'\x01\x02'))
    # Complete outer frame whose payload starts with a protocol-valid but
    # incomplete inner candidate (declared payload 200, total 206).
    inner_false = bytes((PREAMBLE, 0x00, 200))
    filler = b'\x11' * 10
    outer = bytearray(make_rtcm3_frame(inner_false + filler))
    for crc_index in range(-3, 0):
        if outer[crc_index] != PREAMBLE:
            outer[crc_index] ^= 0x01
            if outer[crc_index] == PREAMBLE:
                outer[crc_index] ^= 0x02
            break
    else:
        raise AssertionError('could not corrupt CRC without introducing 0xD3')
    assert PREAMBLE not in bytes(outer[-3:])
    corrupt = bytes(outer)

    parser = RTCM3Parser()
    out = parser.feed(corrupt + good, 10.0)
    assert out == []
    assert parser.counters.frames_crc_invalid_total == 1
    assert parser.counters.frames_valid_total == 0
    assert parser.counters.partial_frame_timeouts_total == 0
    assert parser.buffered_length > 0

    out = parser.service(11.99)
    assert out == []
    assert parser.counters.partial_frame_timeouts_total == 0
    assert parser.counters.frames_valid_total == 0

    out = parser.service(12.0)
    assert frame_bytes_of(out) == [good]
    counters = parser.counters
    assert counters.frames_valid_total == 1
    assert counters.bytes_valid_total == len(good)
    assert counters.frames_crc_invalid_total == 1
    assert counters.headers_invalid_total == 0
    assert counters.partial_frame_timeouts_total == 1
    assert parser.buffered_length == 0
    assert counters.bytes_fed_total == len(corrupt) + len(good)
    assert (
        counters.bytes_valid_total + counters.resync_bytes_discarded_total
        == counters.bytes_fed_total
    )


# ---------------------------------------------------------------------------
# Phase-1A hardening: timeout config and nondecreasing injected time
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    'partial_timeout_sec',
    [math.nan, math.inf, -math.inf, 0.0, -1.0],
)
def test_partial_timeout_sec_must_be_finite_and_positive(partial_timeout_sec):
    with pytest.raises(ValueError):
        RTCM3Parser(partial_timeout_sec=partial_timeout_sec)


def test_equal_timestamps_accepted():
    frame = make_rtcm3_frame(payload_with_type(1077, b'\x01\x02'))
    parser = RTCM3Parser()
    assert parser.feed(frame[:5], 10.0) == []
    out = parser.feed(frame[5:], 10.0)
    assert frame_bytes_of(out) == [frame]
    assert parser.service(10.0) == []


def test_increasing_timestamps_accepted():
    frame = make_rtcm3_frame(payload_with_type(1077, b'\x01\x02'))
    parser = RTCM3Parser()
    assert parser.feed(frame[:4], 10.0) == []
    assert parser.feed(frame[4:6], 10.5) == []
    out = parser.feed(frame[6:], 11.0)
    assert frame_bytes_of(out) == [frame]


def test_decreasing_feed_timestamp_rejected():
    parser = RTCM3Parser()
    parser.feed(bytes((PREAMBLE, 0x00)), 10.0)
    parser.feed(b'\x01', 11.0)
    with pytest.raises(ValueError, match='nondecreasing'):
        parser.feed(b'\x02', 10.9)


def test_decreasing_service_timestamp_rejected():
    parser = RTCM3Parser()
    parser.feed(bytes((PREAMBLE, 0x00)), 10.0)
    parser.service(11.0)
    with pytest.raises(ValueError, match='nondecreasing'):
        parser.service(10.9)


def test_backwards_time_does_not_corrupt_parser_state():
    parser = RTCM3Parser()
    partial = bytes((PREAMBLE, 0x00, 50)) + b'\x01' * 10
    parser.feed(partial, 10.0)
    before_counters = parser.counters
    before_buffered = parser.buffered_length
    with pytest.raises(ValueError, match='nondecreasing'):
        parser.feed(b'\x02\x03', 9.5)
    with pytest.raises(ValueError, match='nondecreasing'):
        parser.service(9.5)
    assert parser.counters == before_counters
    assert parser.buffered_length == before_buffered
    # Candidate timer still starts at 10.0, not reset by the rejected calls.
    assert parser.service(11.99) == []
    assert parser.counters.partial_frame_timeouts_total == 0
    assert parser.buffered_length == before_buffered
    assert parser.service(12.0) == []
    assert parser.counters.partial_frame_timeouts_total == 1


def test_parser_continues_after_rejected_backwards_timestamp():
    frame = make_rtcm3_frame(payload_with_type(1005, b'\x00\x01'))
    parser = RTCM3Parser()
    assert parser.feed(frame[:4], 10.0) == []
    with pytest.raises(ValueError, match='nondecreasing'):
        parser.feed(frame[4:], 9.0)
    assert parser.counters.frames_valid_total == 0
    out = parser.feed(frame[4:], 10.0)
    assert frame_bytes_of(out) == [frame]
    assert parser.counters.frames_valid_total == 1
    assert parser.buffered_length == 0
