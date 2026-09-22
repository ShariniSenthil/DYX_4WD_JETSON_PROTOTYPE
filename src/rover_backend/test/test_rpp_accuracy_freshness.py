"""/rpp/accuracy freshness is tracked independently of its retained value.

Runs the production ros_bridge method bodies (AST-extracted; ROS is not
available on the workstation) against a fake clock.
"""

from __future__ import annotations

import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from helpers.realtime_fixtures import import_system_routes

SOURCE = Path(__file__).resolve().parents[1] / "rover_backend" / "ros_bridge.py"


class State:
    def __init__(self):
        self.sections = {"accuracy": {}}

    def update(self, section, **updates):
        self.sections.setdefault(section, {}).update(updates)

    def section(self, section):
        return dict(self.sections.get(section, {}))


def _methods(namespace, *names):
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == "RoverBackendRosNode")
    out = {}
    for name in names:
        fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
        exec(compile(ast.Module(body=[fn], type_ignores=[]), str(SOURCE), "exec"), namespace)
        out[name] = namespace[name]
    return out


@pytest.fixture
def bridge():
    clock = SimpleNamespace(now=1000.0)
    state = State()
    namespace = {
        "math": math,
        "time": SimpleNamespace(monotonic=lambda: clock.now),
        "rover_state": state,
        "_json_object": lambda data: json.loads(data),
        "_finite_float": lambda v, d=None: (
            float(v) if isinstance(v, (int, float)) and math.isfinite(v) else d
        ),
        "_safe_int": lambda v, d=0: int(v) if isinstance(v, int) else d,
        "String": object,
        "Any": object,
    }
    methods = _methods(namespace, "_accuracy_callback", "_rpp_debug_stale_monitor",
                       "_monotonic_age")
    cls = next(n for n in ast.parse(SOURCE.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == "RoverBackendRosNode")
    stale_sec = next(
        n.value.value for n in cls.body
        if isinstance(n, ast.Assign) and n.targets[0].id == "RPP_ACCURACY_STALE_SEC"
    )
    node = SimpleNamespace(
        _last_rpp_debug_monotonic=None,
        _last_rpp_accuracy_monotonic=None,
        _rpp_accuracy_last_receipt_monotonic=None,
        _rpp_debug_dropped_frames=0,
        _mark_ros_message=lambda: None,
        RPP_ACCURACY_STALE_SEC=stale_sec,
    )
    node._monotonic_age = methods["_monotonic_age"]  # @staticmethod in production
    return SimpleNamespace(
        node=node, clock=clock, state=state,
        callback=lambda payload: methods["_accuracy_callback"](
            node, SimpleNamespace(data=json.dumps(payload))),
        monitor=lambda: methods["_rpp_debug_stale_monitor"](node),
        stale_sec=stale_sec,
    )


SAMPLE = {"cross_track_error_mm": 4.0, "front_back_error_mm": 3.0,
          "radial_error_mm": 5.0, "goal_number": 2}


def test_threshold_is_five_20hz_control_cycles(bridge):
    assert bridge.stale_sec == pytest.approx(5 / 20.0)


def test_never_received_is_not_fresh(bridge):
    bridge.monitor()
    accuracy = bridge.state.section("accuracy")
    assert accuracy["rpp_accuracy_stream_fresh"] is False
    assert accuracy["rpp_accuracy_receive_age_ms"] is None


def _stream(bridge, samples=2):
    for _ in range(samples):
        bridge.callback(SAMPLE)
        bridge.clock.now += 0.05


def test_lone_retained_sample_is_never_fresh(bridge):
    # TRANSIENT_LOCAL redelivers the last sample on (re)subscription, e.g.
    # after a backend restart while RPP is silent.
    bridge.callback(SAMPLE)
    bridge.monitor()
    accuracy = bridge.state.section("accuracy")
    assert accuracy["available"] is True
    assert accuracy["rpp_accuracy_stream_fresh"] is False


def test_fresh_then_stale_while_retained_value_stays(bridge):
    _stream(bridge)
    bridge.monitor()
    accuracy = bridge.state.section("accuracy")
    assert accuracy["rpp_accuracy_stream_fresh"] is True
    assert accuracy["rpp_accuracy_receive_age_ms"] == pytest.approx(50.0)
    assert accuracy["available"] is True

    # RPP stops publishing: the retained value and available flag remain,
    # but freshness must drop.
    bridge.clock.now += 0.30
    bridge.monitor()
    accuracy = bridge.state.section("accuracy")
    assert accuracy["rpp_accuracy_stream_fresh"] is False
    assert accuracy["radial_error_mm"] == 5.0
    assert accuracy["available"] is True

    # A resumed stream (two samples in the window) revives it.
    _stream(bridge)
    bridge.monitor()
    assert bridge.state.section("accuracy")["rpp_accuracy_stream_fresh"] is True


def test_accuracy_freshness_is_independent_of_rpp_debug(bridge):
    bridge.node._last_rpp_debug_monotonic = bridge.clock.now
    bridge.monitor()
    accuracy = bridge.state.section("accuracy")
    assert accuracy["rpp_debug_stream_fresh"] is True
    assert accuracy["rpp_accuracy_stream_fresh"] is False


def test_telemetry_payload_carries_accuracy_freshness(monkeypatch):
    routes = import_system_routes(monkeypatch)
    from rover_backend.state import rover_state
    rover_state.update("accuracy", rpp_accuracy_stream_fresh=True,
                       rpp_accuracy_receive_age_ms=12.5)
    payload = routes.build_telemetry_payload()
    assert payload["rpp_accuracy_stream_fresh"] is True
    assert payload["rpp_accuracy_receive_age_ms"] == 12.5
    rover_state.update("accuracy", rpp_accuracy_stream_fresh=False)
    assert routes.build_telemetry_payload()["rpp_accuracy_stream_fresh"] is False
