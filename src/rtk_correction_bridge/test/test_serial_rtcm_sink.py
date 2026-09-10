"""Tests for the direct RTCM serial sink."""

from __future__ import annotations

import pytest

from rtk_correction_bridge.serial_rtcm_sink import (
    SerialRtcmSink,
    SerialRtcmSinkUnavailableError,
    SerialRtcmSinkWriteError,
)


DEVICE = (
    "/dev/serial/by-id/"
    "usb-Septentrio_mosaic-H-test"
)


class FakeClock:
    def __init__(
        self,
        value: float = 100.0,
    ):
        self.value = value

    def __call__(self):
        return self.value

    def advance(
        self,
        seconds: float,
    ):
        self.value += seconds


class FakeSerial:
    def __init__(
        self,
        plan=None,
    ):
        self.plan = list(
            plan or []
        )

        self.is_open = True
        self.received = bytearray()
        self.write_calls = []
        self.close_calls = 0

    def write(
        self,
        data,
    ):
        chunk = bytes(
            data
        )

        self.write_calls.append(
            chunk
        )

        action = (
            self.plan.pop(0)
            if self.plan
            else len(chunk)
        )

        if isinstance(
            action,
            BaseException,
        ):
            raise action

        count = int(
            action
        )

        if count > 0:
            self.received.extend(
                chunk[:count]
            )

        return count

    def close(self):
        self.close_calls += 1
        self.is_open = False


class FakeFactory:
    def __init__(
        self,
        outcomes,
    ):
        self.outcomes = list(
            outcomes
        )

        self.calls = []

    def __call__(
        self,
        **kwargs,
    ):
        self.calls.append(
            kwargs
        )

        outcome = self.outcomes.pop(
            0
        )

        if isinstance(
            outcome,
            BaseException,
        ):
            raise outcome

        return outcome


def make_sink(
    factory,
    *,
    clock=None,
    **kwargs,
):
    return SerialRtcmSink(
        DEVICE,
        serial_factory=factory,
        clock=(
            clock
            if clock is not None
            else FakeClock()
        ),
        **kwargs,
    )


def test_exact_frame_and_exclusive_open():
    port = FakeSerial()
    factory = FakeFactory(
        [port]
    )

    sink = make_sink(
        factory,
        write_timeout_sec=0.5,
    )

    frame = (
        b"\xd3\x00\x03"
        b"ABC"
        b"\x01\x02\x03"
    )

    sink.write_frame(
        frame
    )

    assert bytes(
        port.received
    ) == frame

    call = factory.calls[0]

    assert call["port"] == DEVICE
    assert call["baudrate"] == 230400
    assert call["bytesize"] == 8
    assert call["parity"] == "N"
    assert call["stopbits"] == 1
    assert call["write_timeout"] == 0.5

    assert call["xonxoff"] is False
    assert call["rtscts"] is False
    assert call["dsrdtr"] is False

    assert call["exclusive"] is True

    snapshot = sink.snapshot

    assert snapshot.serial_open is True
    assert snapshot.open_attempts_total == 1
    assert snapshot.open_failures_total == 0
    assert snapshot.frames_written_total == 1
    assert (
        snapshot.bytes_written_total
        == len(frame)
    )
    assert snapshot.write_failures_total == 0


def test_partial_writes_complete_exact_frame():
    port = FakeSerial(
        [2, 2, 2]
    )

    sink = make_sink(
        FakeFactory(
            [port]
        )
    )

    sink.write_frame(
        b"abcdef"
    )

    assert bytes(
        port.received
    ) == b"abcdef"

    assert port.write_calls == [
        b"abcdef",
        b"cdef",
        b"ef",
    ]


def test_zero_byte_write_fails_and_closes():
    port = FakeSerial(
        [0]
    )

    sink = make_sink(
        FakeFactory(
            [port]
        )
    )

    with pytest.raises(
        SerialRtcmSinkWriteError
    ):
        sink.write_frame(
            b"abc"
        )

    snapshot = sink.snapshot

    assert snapshot.serial_open is False
    assert snapshot.write_failures_total == 1
    assert snapshot.frames_written_total == 0
    assert snapshot.bytes_written_total == 0

    assert port.close_calls == 1


def test_initial_open_failure_obeys_reopen_delay():
    clock = FakeClock()

    port = FakeSerial()

    factory = FakeFactory(
        [
            OSError("busy"),
            port,
        ]
    )

    sink = make_sink(
        factory,
        clock=clock,
        reopen_delay_sec=1.0,
    )

    with pytest.raises(
        SerialRtcmSinkUnavailableError
    ):
        sink.write_frame(
            b"abc"
        )

    assert len(
        factory.calls
    ) == 1

    snapshot = sink.snapshot

    assert snapshot.open_attempts_total == 1
    assert snapshot.open_failures_total == 1

    # Backoff must prevent hammering the USB endpoint.
    with pytest.raises(
        SerialRtcmSinkUnavailableError
    ):
        sink.write_frame(
            b"abc"
        )

    assert len(
        factory.calls
    ) == 1

    clock.advance(
        1.0
    )

    sink.write_frame(
        b"abc"
    )

    snapshot = sink.snapshot

    assert snapshot.open_attempts_total == 2
    assert snapshot.open_failures_total == 1
    assert snapshot.reopen_total == 1
    assert snapshot.frames_written_total == 1


def test_write_failure_reconnects_without_fallback():
    clock = FakeClock()

    first = FakeSerial(
        [
            OSError(
                "USB disconnected"
            )
        ]
    )

    second = FakeSerial()

    factory = FakeFactory(
        [
            first,
            second,
        ]
    )

    sink = make_sink(
        factory,
        clock=clock,
        reopen_delay_sec=0.5,
    )

    with pytest.raises(
        SerialRtcmSinkWriteError
    ):
        sink.write_frame(
            b"first"
        )

    assert first.close_calls == 1

    with pytest.raises(
        SerialRtcmSinkUnavailableError
    ):
        sink.write_frame(
            b"second"
        )

    # Reopen delay prevented another open attempt.
    assert len(
        factory.calls
    ) == 1

    clock.advance(
        0.5
    )

    sink.write_frame(
        b"second"
    )

    assert bytes(
        second.received
    ) == b"second"

    snapshot = sink.snapshot

    assert snapshot.write_failures_total == 1
    assert snapshot.reopen_total == 1
    assert snapshot.frames_written_total == 1


def test_partial_then_failure_never_replays_failed_frame():
    clock = FakeClock()

    first = FakeSerial(
        [
            2,
            OSError(
                "disconnect"
            ),
        ]
    )

    second = FakeSerial()

    factory = FakeFactory(
        [
            first,
            second,
        ]
    )

    sink = make_sink(
        factory,
        clock=clock,
        reopen_delay_sec=0.0,
    )

    with pytest.raises(
        SerialRtcmSinkWriteError
    ):
        sink.write_frame(
            b"abcdef"
        )

    # Two bytes reached the failed endpoint, therefore replaying
    # abcdef from byte zero would risk duplicate/corrupt input.
    assert bytes(
        first.received
    ) == b"ab"

    snapshot = sink.snapshot

    assert snapshot.frames_written_total == 0
    assert snapshot.bytes_written_total == 0

    # Only the next independent RTCM frame may use the new port.
    sink.write_frame(
        b"XYZ"
    )

    assert bytes(
        second.received
    ) == b"XYZ"


def test_empty_frame_rejected_without_opening_port():
    factory = FakeFactory(
        [
            FakeSerial()
        ]
    )

    sink = make_sink(
        factory
    )

    with pytest.raises(
        ValueError
    ):
        sink.write_frame(
            b""
        )

    assert factory.calls == []


def test_close_is_idempotent():
    port = FakeSerial()

    sink = make_sink(
        FakeFactory(
            [port]
        )
    )

    sink.open()

    sink.close()
    sink.close()

    assert port.close_calls == 1
    assert sink.snapshot.serial_open is False


@pytest.mark.parametrize(
    "value",
    (
        0,
        -1,
        True,
        230400.0,
        "230400",
    ),
)
def test_invalid_baud_rejected(
    value,
):
    with pytest.raises(
        ValueError
    ):
        SerialRtcmSink(
            DEVICE,
            baudrate=value,
            serial_factory=FakeFactory(
                [FakeSerial()]
            ),
        )
