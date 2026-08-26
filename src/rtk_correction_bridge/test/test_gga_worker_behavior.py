"""Behavior tests for the exact production GGA worker methods."""

import ast
import time

from pathlib import Path

import pytest

from rtk_correction_bridge.nmea_gga import (
    GgaSourceFix,
    format_gga_sentence,
)


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "rtk_correction_bridge"
    / "ntrip_to_px4_node.py"
)


METHOD_NAMES = {
    "_gga_source_age",
    "_gga_source_state",
    "_build_current_gga",
    "_maybe_send_gga",
    "_session_connection_start",
    "_check_source_deadlines",
}


def _harness_class():
    source = SOURCE.read_text()
    tree = ast.parse(source)

    methods = []

    for node in ast.walk(tree):
        if (
            isinstance(node, ast.FunctionDef)
            and node.name in METHOD_NAMES
        ):
            methods.append(node)

    assert {
        node.name
        for node in methods
    } == METHOD_NAMES

    klass = ast.ClassDef(
        name="Harness",
        bases=[],
        keywords=[],
        body=methods,
        decorator_list=[],
    )

    module = ast.Module(
        body=[klass],
        type_ignores=[],
    )

    ast.fix_missing_locations(
        module
    )

    namespace = {
        "time": time,
        "GgaSourceFix": GgaSourceFix,
        "format_gga_sentence": (
            format_gga_sentence
        ),
    }

    exec(
        compile(
            module,
            str(SOURCE),
            "exec",
        ),
        namespace,
    )

    return namespace["Harness"]


def _ready_worker():
    worker = _harness_class()()

    worker.gga_enabled = True
    worker.gga_interval_sec = 10.0
    worker.gga_max_age_sec = 5.0

    worker._gga_latitude_deg = 13.0827
    worker._gga_longitude_deg = 80.2707
    worker._gga_altitude_msl_m = 12.5

    worker._gga_fix_type = 3
    worker._gga_satellites_visible = 16
    worker._gga_hdop = 0.8

    worker._gga_position_at = 100.0
    worker._gga_gpsraw_at = 100.0

    worker._last_gga_sent_at = None
    worker._session_first_gga_sent_at = None

    worker.gga_sent_total = 0
    worker.gga_send_errors = 0

    return worker


class Socket:
    def __init__(
        self,
        *,
        fail=False,
    ):
        self.fail = fail
        self.writes = []

    def sendall(
        self,
        payload,
    ):
        if self.fail:
            raise OSError(
                "synthetic send failure"
            )

        self.writes.append(
            bytes(payload)
        )


def test_fresh_fix_sends_once_and_records_session_start():
    worker = _ready_worker()
    sock = Socket()

    assert worker._maybe_send_gga(
        sock,
        101.0,
        force=True,
    )

    assert len(sock.writes) == 1
    assert sock.writes[0].startswith(
        b"$GPGGA,"
    )

    assert (
        worker._session_first_gga_sent_at
        == 101.0
    )

    assert worker.gga_sent_total == 1


def test_interval_throttles_second_send():
    worker = _ready_worker()
    sock = Socket()

    assert worker._maybe_send_gga(
        sock,
        101.0,
        force=True,
    )

    assert not worker._maybe_send_gga(
        sock,
        105.0,
    )

    assert len(sock.writes) == 1


def test_stale_source_never_writes():
    worker = _ready_worker()
    sock = Socket()

    worker._gga_position_at = 80.0
    worker._gga_gpsraw_at = 80.0

    assert not worker._maybe_send_gga(
        sock,
        100.0,
        force=True,
    )

    assert sock.writes == []


def test_no_fix_never_writes():
    worker = _ready_worker()
    sock = Socket()

    worker._gga_fix_type = 1

    assert not worker._maybe_send_gga(
        sock,
        101.0,
        force=True,
    )

    assert sock.writes == []


def test_send_error_is_counted_and_propagated():
    worker = _ready_worker()
    sock = Socket(
        fail=True
    )

    with pytest.raises(
        OSError,
        match="synthetic send failure",
    ):
        worker._maybe_send_gga(
            sock,
            101.0,
            force=True,
        )

    assert worker.gga_send_errors == 1
    assert worker.gga_sent_total == 0


class FakeTransport:
    def __init__(self):
        self.last_valid_frame_at = None

        self.first_timeout_calls = []
        self.stale_calls = []

        self.first_timeout_result = False
        self.stale_result = False

    def first_valid_frame_timed_out(
        self,
        start,
        now,
        timeout,
    ):
        self.first_timeout_calls.append(
            (
                start,
                now,
                timeout,
            )
        )

        return self.first_timeout_result

    def source_is_stale(
        self,
        now,
        timeout,
    ):
        self.stale_calls.append(
            (
                now,
                timeout,
            )
        )

        return self.stale_result


def test_vrs_waiting_for_fix_does_not_trigger_false_rtcm_timeout():
    worker = _ready_worker()

    worker.connection_start = 0.0
    worker._session_first_gga_sent_at = None

    worker._gga_position_at = None
    worker._gga_gpsraw_at = None

    worker.first_data_timeout_sec = 10.0
    worker.stale_reconnect_sec = 20.0

    worker.transport = FakeTransport()

    worker._check_source_deadlines(
        100.0
    )

    assert (
        worker.transport.first_timeout_calls
        == []
    )

    assert worker.transport.stale_calls == []


def test_first_rtcm_timeout_starts_from_first_successful_gga():
    worker = _ready_worker()

    worker.connection_start = 0.0
    worker._session_first_gga_sent_at = 50.0

    worker.first_data_timeout_sec = 10.0
    worker.stale_reconnect_sec = 20.0

    worker.transport = FakeTransport()
    worker.transport.first_timeout_result = True

    with pytest.raises(
        TimeoutError,
        match="No first CRC-valid RTCM",
    ):
        worker._check_source_deadlines(
            61.0
        )

    assert (
        worker.transport.first_timeout_calls[0][0]
        == 50.0
    )


def test_stale_gga_does_not_cause_pointless_caster_reconnect():
    worker = _ready_worker()

    worker.connection_start = 0.0
    worker._session_first_gga_sent_at = 1.0

    worker._gga_position_at = 1.0
    worker._gga_gpsraw_at = 1.0

    worker.first_data_timeout_sec = 10.0
    worker.stale_reconnect_sec = 20.0

    worker.transport = FakeTransport()
    worker.transport.last_valid_frame_at = 2.0
    worker.transport.stale_result = True

    worker._check_source_deadlines(
        100.0
    )

    assert worker.transport.stale_calls == []


def test_stale_gga_resets_first_rtcm_deadline_without_reconnect():
    worker = _ready_worker()

    worker.connection_start = 0.0
    worker._session_first_gga_sent_at = 50.0

    # GGA that originally started the VRS wait has since gone stale.
    worker._gga_position_at = 50.0
    worker._gga_gpsraw_at = 50.0

    worker.first_data_timeout_sec = 10.0
    worker.stale_reconnect_sec = 20.0

    worker.transport = FakeTransport()
    worker.transport.first_timeout_result = True

    worker._check_source_deadlines(
        100.0
    )

    # Do not blame/reconnect the caster while rover GNSS itself is stale.
    assert worker.transport.first_timeout_calls == []
    assert worker.transport.stale_calls == []

    # The next successful fresh GGA must establish a new first-frame epoch.
    assert worker._session_first_gga_sent_at is None
