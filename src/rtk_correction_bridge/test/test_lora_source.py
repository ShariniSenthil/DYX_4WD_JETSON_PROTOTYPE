"""LoRa RTCM correction source (2026-09-26).

The radio passes the base's RTCM3 bytes through; the worker reads them from a
local serial port and feeds the same parser/deadline/health/sink path as
NTRIP. Tests cover the ROS-free port wrapper, the production _run_lora loop
(extracted from source and run against fakes) and the status payload.
"""

import ast
import time
from pathlib import Path

import pytest

from rtk_correction_bridge.rtcm_transport import RtcmWorkerCounters
from rtk_correction_bridge.serial_rtcm_source import (
    SerialRtcmSource,
    SerialRtcmSourceError,
)
from rtk_correction_bridge.status_snapshot import (
    build_correction_status_snapshot,
)

PORT = "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30A1B2C-if00-port0"
SOURCE = (
    Path(__file__).resolve().parents[1]
    / "rtk_correction_bridge"
    / "ntrip_to_px4_node.py"
)


class FakePort:
    def __init__(self, reads=(), fail_read=False):
        self.reads = list(reads)
        self.fail_read = fail_read
        self.is_open = True
        self.reset_calls = 0
        self.closed = 0

    def read(self, size=1):
        if self.fail_read:
            raise OSError("device reports readiness to read but returned no data")
        return self.reads.pop(0) if self.reads else b""

    def reset_input_buffer(self):
        self.reset_calls += 1

    def close(self):
        self.is_open = False
        self.closed += 1


# --------------------------------------------------------------------------
# SerialRtcmSource
# --------------------------------------------------------------------------


def test_open_uses_8n1_read_timeout_exclusive_and_flushes():
    opened = {}
    port = FakePort()

    def factory(**kwargs):
        opened.update(kwargs)
        return port

    source = SerialRtcmSource(PORT, baudrate=57600, read_timeout_sec=1.0,
                              serial_factory=factory)
    source.open()
    assert opened == {
        "port": PORT, "baudrate": 57600, "bytesize": 8, "parity": "N",
        "stopbits": 1, "timeout": 1.0, "exclusive": True,
    }
    assert port.reset_calls == 1
    assert source.is_open
    assert source.snapshot.open_total == 1


def test_read_returns_bytes_and_counts_and_timeout_is_empty():
    port = FakePort(reads=[b"\xd3\x00\x13", b""])
    source = SerialRtcmSource(PORT, baudrate=57600, read_timeout_sec=1.0,
                              serial_factory=lambda **_: port)
    source.open()
    assert source.read() == b"\xd3\x00\x13"
    assert source.read() == b""
    assert source.snapshot.bytes_read_total == 3


def test_open_failure_is_reported_and_counted():
    def factory(**_):
        raise OSError("No such file or directory")

    source = SerialRtcmSource(PORT, baudrate=57600, read_timeout_sec=1.0,
                              serial_factory=factory)
    with pytest.raises(SerialRtcmSourceError):
        source.open()
    snapshot = source.snapshot
    assert not snapshot.serial_open
    assert snapshot.open_failures_total == 1
    assert snapshot.last_error == "open failed: OSError"


def test_read_failure_is_reported_and_counted():
    port = FakePort(fail_read=True)
    source = SerialRtcmSource(PORT, baudrate=57600, read_timeout_sec=1.0,
                              serial_factory=lambda **_: port)
    source.open()
    with pytest.raises(SerialRtcmSourceError):
        source.read()
    assert source.snapshot.read_errors_total == 1


def test_read_before_open_is_an_error_and_close_is_idempotent():
    port = FakePort()
    source = SerialRtcmSource(PORT, baudrate=57600, read_timeout_sec=1.0,
                              serial_factory=lambda **_: port)
    with pytest.raises(SerialRtcmSourceError):
        source.read()
    source.open()
    source.close()
    source.close()
    assert port.closed == 1
    assert not source.is_open


@pytest.mark.parametrize(
    "kwargs",
    [
        {"device": "ttyUSB0", "baudrate": 57600, "read_timeout_sec": 1.0},
        {"device": PORT, "baudrate": 0, "read_timeout_sec": 1.0},
        {"device": PORT, "baudrate": True, "read_timeout_sec": 1.0},
        {"device": PORT, "baudrate": 57600, "read_timeout_sec": 0.0},
    ],
)
def test_invalid_construction_is_rejected(kwargs):
    with pytest.raises(ValueError):
        SerialRtcmSource(**kwargs)


# --------------------------------------------------------------------------
# Production _run_lora loop against fakes
# --------------------------------------------------------------------------


class FakeRclpy:
    def __init__(self, budget):
        self.budget = budget

    def ok(self):
        self.budget -= 1
        return self.budget >= 0

    def spin_once(self, *_args, **_kwargs):
        pass


class FakeLogger:
    def __init__(self, events):
        self.events = events

    def warn(self, message):
        self.events.append(("warn", message))

    def error(self, message):
        self.events.append(("error", message))


class FakeSource:
    def __init__(self, reads):
        self.reads = list(reads)
        self.events = None

    def open(self):
        self.events.append("open")

    def read(self, size):
        item = self.reads.pop(0) if self.reads else b""
        if isinstance(item, Exception):
            raise item
        return item

    def close(self):
        self.events.append("close")


def _run_lora(reads, budget, deadline_errors=()):
    tree = ast.parse(SOURCE.read_text())
    method = next(
        n for n in ast.walk(tree)
        if isinstance(n, ast.FunctionDef) and n.name == "_run_lora"
    )
    module = ast.Module(
        body=[ast.ClassDef(name="Harness", bases=[], keywords=[],
                           body=[method], decorator_list=[])],
        type_ignores=[],
    )
    ast.fix_missing_locations(module)
    rclpy = FakeRclpy(budget)
    namespace = {"rclpy": rclpy, "time": time}
    exec(compile(module, str(SOURCE), "exec"), namespace)

    events = []
    deadline_errors = list(deadline_errors)
    source = FakeSource(reads)
    source.events = events

    node = namespace["Harness"]()
    node._lora_source = source
    node.lora_serial_device = PORT
    node.lora_serial_baud = 57600
    node.reconnect_delay_sec = 5.0
    node.connected = False
    node.connection_start = None
    node.get_logger = lambda: FakeLogger(events)
    node._new_parser_session = lambda: events.append("new_session")
    node._publish_health = lambda force=False: (events.append("health"), (True, 0.1))[1]
    node._process_stream_bytes = lambda data, now: events.append(("bytes", data))
    node._service_parser = lambda now: events.append("idle")
    node._maybe_log_health = lambda: None
    node._maybe_log_source_status = lambda *a: None
    node._discard_parser_session = lambda: events.append("discard")
    node._set_disconnected = lambda: events.append("disconnected")
    node._sleep_with_ros = lambda sec: events.append(("sleep", sec))

    def check(now):
        if deadline_errors:
            error = deadline_errors.pop(0)
            if error is not None:
                raise error

    node._check_source_deadlines = check
    node._run_lora()
    return node, events


def test_loop_opens_feeds_bytes_and_idles_on_read_timeout():
    node, events = _run_lora([b"\xd3\x00", b""], budget=4)
    assert events[:3] == ["open", "new_session", "health"]
    assert ("bytes", b"\xd3\x00") in events
    assert "idle" in events
    assert node.connection_start is not None


def test_deadline_timeout_closes_reports_and_reopens_after_delay():
    _, events = _run_lora(
        [b"\xd3", b""],
        budget=8,
        deadline_errors=[TimeoutError("No first CRC-valid RTCM frame"), None],
    )
    first_close = events.index("close")
    assert events[first_close + 1] == "discard"
    assert events[first_close + 2] == "disconnected"
    assert ("sleep", 5.0) in events
    assert events.count("open") == 2
    assert any(
        kind == "error" and "No first CRC-valid RTCM frame" in message
        for kind, message in (e for e in events if isinstance(e, tuple) and len(e) == 2 and e[0] in ("warn", "error"))
    )


def test_read_error_reopens_port():
    _, events = _run_lora(
        [SerialRtcmSourceError("LoRa port read failed"), b""],
        budget=8,
    )
    assert events.count("open") == 2
    assert "disconnected" in events


def test_run_dispatches_lora_before_ntrip_socket_loop():
    tree = ast.parse(SOURCE.read_text())
    run = next(n for n in ast.walk(tree)
               if isinstance(n, ast.FunctionDef) and n.name == "run")
    first = run.body[0]
    assert isinstance(first, ast.If)
    assert ast.unparse(first.test) == "self.is_lora"


# --------------------------------------------------------------------------
# Status payload
# --------------------------------------------------------------------------


def _status(**kwargs):
    return build_correction_status_snapshot(
        connected=True, healthy=True, correction_age_sec=0.2,
        counters=RtcmWorkerCounters(), mavros_subscribers=1,
        max_mavros_rtcm_frame_bytes=720, **kwargs,
    )


def test_status_defaults_to_ntrip_without_lora_block():
    payload = _status()
    assert payload["correction_source"] == "NTRIP"
    assert payload["lora"] is None


def test_status_reports_lora_port_and_counters():
    port = FakePort(reads=[b"abc"])
    source = SerialRtcmSource(PORT, baudrate=57600, read_timeout_sec=1.0,
                              serial_factory=lambda **_: port)
    source.open()
    source.read()
    payload = _status(correction_source="LORA",
                      lora_source_snapshot=source.snapshot)
    assert payload["correction_source"] == "LORA"
    assert payload["lora"]["device"] == PORT
    assert payload["lora"]["baud"] == 57600
    assert payload["lora"]["open"] is True
    assert payload["lora"]["bytes_read_total"] == 3


def test_status_rejects_unknown_source():
    with pytest.raises(ValueError):
        _status(correction_source="RADIO")
