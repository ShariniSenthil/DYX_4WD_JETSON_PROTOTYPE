"""ROS-free contract tests for RTCM A/B sink routing."""

from __future__ import annotations

import ast
from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "rtk_correction_bridge"
    / "ntrip_to_px4_node.py"
)


def _source() -> str:
    return SOURCE.read_text()


def _routing_harness():
    tree = ast.parse(
        _source()
    )

    method = None

    for node in ast.walk(tree):
        if (
            isinstance(node, ast.FunctionDef)
            and node.name == "_process_parsed_frames"
        ):
            method = node
            break

    assert method is not None

    klass = ast.ClassDef(
        name="Harness",
        bases=[],
        keywords=[],
        body=[method],
        decorator_list=[],
    )

    module = ast.Module(
        body=[klass],
        type_ignores=[],
    )

    ast.fix_missing_locations(
        module
    )

    namespace = {}

    exec(
        compile(
            module,
            str(SOURCE),
            "exec",
        ),
        namespace,
    )

    return namespace["Harness"]


class FakeTransport:
    def __init__(self):
        self.calls = []

    def attempt_publish(
        self,
        frame_bytes,
        now,
        sink,
    ):
        self.calls.append(
            (
                bytes(frame_bytes),
                now,
                sink,
            )
        )

        try:
            sink(
                frame_bytes
            )
        except Exception:
            return False

        return True


class RecordingSink:
    def __init__(
        self,
        *,
        fail=False,
    ):
        self.fail = fail
        self.frames = []

    def __call__(
        self,
        frame_bytes,
    ):
        self.frames.append(
            bytes(frame_bytes)
        )

        if self.fail:
            raise RuntimeError(
                "synthetic sink failure"
            )


def _worker(
    active_sink,
):
    worker = _routing_harness()()

    worker.transport = FakeTransport()
    worker._active_sink = active_sink
    worker._pending_publish_errors = 0

    return worker


def test_each_candidate_reaches_only_active_sink():
    sink = RecordingSink()
    worker = _worker(
        sink
    )

    worker._process_parsed_frames(
        [
            b"frame-one",
            b"frame-two",
        ],
        10.0,
    )

    assert sink.frames == [
        b"frame-one",
        b"frame-two",
    ]

    assert len(
        worker.transport.calls
    ) == 2

    assert (
        worker._pending_publish_errors
        == 0
    )


def test_failed_direct_sink_has_no_mavros_fallback():
    direct = RecordingSink(
        fail=True
    )

    legacy = RecordingSink()

    worker = _worker(
        direct
    )

    # Existing legacy method may exist on the real node, but routing must
    # never consult it after _active_sink has selected direct injection.
    worker._publish_rtcm_frame = legacy

    worker._process_parsed_frames(
        [
            b"direct-only",
        ],
        20.0,
    )

    assert direct.frames == [
        b"direct-only",
    ]

    assert legacy.frames == []

    assert (
        worker._pending_publish_errors
        == 1
    )


def test_constructor_selects_one_sink_and_keeps_mavros_publisher():
    source = _source()

    assert (
        "self.rtcm_pub = self.create_publisher("
        in source
    )

    assert (
        "self._serial_sink = SerialRtcmSink("
        in source
    )

    assert (
        "self._serial_sink.write_frame"
        in source
    )

    assert (
        "else self._publish_rtcm_frame"
        in source
    )

    assert (
        "self._active_sink,"
        in source
    )


def test_direct_mode_uses_protocol_ceiling_legacy_keeps_configured_gate():
    source = _source()

    assert (
        "MAX_MAVROS_RTCM_FRAME_BYTES_LIMIT"
        in source
    )

    start = source.index(
        "self.effective_rtcm_frame_limit_bytes"
    )

    end = source.index(
        "self._password_source",
        start,
    )

    section = source[
        start:end
    ]

    assert (
        "if self.direct_inject"
        in section
    )

    assert (
        "else self.max_mavros_rtcm_frame_bytes"
        in section
    )


def test_destroy_node_closes_direct_sink():
    source = _source()

    start = source.index(
        "def destroy_node(self):"
    )

    end = source.index(
        "def _correction_age(",
        start,
    )

    method = source[
        start:end
    ]

    assert "serial_sink.close()" in method

    assert (
        "super().destroy_node()"
        in method
    )
