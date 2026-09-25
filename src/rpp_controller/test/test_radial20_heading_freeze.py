"""radial20 final-approach heading freeze (2026-09-25).

The nozzle sits ~0.38 m behind the skid-steer turn centre, so a heading
correction at crawl speed swings it the wrong way. Inside the freeze distance
the controller holds the heading the rover already has. These tests execute
the real methods extracted from the node source.
"""

import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


RPP = (
    Path(__file__).resolve().parents[1]
    / "rpp_controller"
    / "rpp_controller_node.py"
)
LAUNCH = (
    Path(__file__).resolve().parents[2]
    / "rover_bringup/launch/rover.launch.py"
)
METHODS = (
    "_reset_radial20_heading_freeze",
    "radial20_heading_freeze",
    "course_compensated_yaw",
)
KEY = (1.0, 2.0)


def _class():
    return next(
        n
        for n in ast.parse(RPP.read_text()).body
        if isinstance(n, ast.ClassDef) and n.name == "RPPController"
    )


def _method(name):
    return next(
        n
        for n in _class().body
        if isinstance(n, ast.FunctionDef) and n.name == name
    )


def _node(*, enabled=True, yaw=0.0, bias=0.0):
    env = {"math": math}
    module = ast.Module(body=[_method(n) for n in METHODS], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), "<rpp>", "exec"), env)
    node = NS()
    for name in METHODS:
        setattr(node, name, env[name].__get__(node))
    node.normalize_angle = lambda v: math.atan2(math.sin(v), math.cos(v))
    node.get_logger = lambda: NS(info=lambda msg: None)
    node.radial20_heading_freeze_enabled = enabled
    node.radial20_heading_freeze_distance = 0.30
    node.radial20_heading_freeze_max_offset = math.radians(5.0)
    node.radial20_frozen_bearing = None
    node.radial20_frozen_goal_key = None
    node.terminal_bearing_frozen = False
    node.current_yaw = yaw
    node.moving_course_bias = bias
    return node


def test_disabled_passes_guidance_through():
    node = _node(enabled=False, yaw=math.radians(3.0))
    assert node.radial20_heading_freeze(0.123, 0.0, 0.10, KEY) == 0.123
    assert node.terminal_bearing_frozen is False
    assert node.radial20_frozen_bearing is None


def test_outside_freeze_distance_passes_guidance_through():
    node = _node(yaw=math.radians(3.0))
    assert node.radial20_heading_freeze(0.05, 0.0, 0.31, KEY) == 0.05
    assert node.terminal_bearing_frozen is False


def test_captures_current_heading_once_and_holds_it():
    node = _node(yaw=math.radians(2.4))
    first = node.radial20_heading_freeze(-0.01, 0.0, 0.29, KEY)
    assert math.degrees(first) == pytest.approx(2.4)
    assert node.terminal_bearing_frozen is True

    # Guidance, remaining distance and measured yaw all keep changing; the
    # published heading must not follow them.
    node.current_yaw = math.radians(-1.0)
    again = node.radial20_heading_freeze(math.radians(-3.7), 0.0, 0.05, KEY)
    assert again == first


def test_frozen_offset_is_bounded_to_the_line():
    node = _node(yaw=math.radians(9.0))
    out = node.radial20_heading_freeze(0.0, 0.0, 0.20, KEY)
    assert math.degrees(out) == pytest.approx(5.0)
    node = _node(yaw=math.radians(-9.0))
    out = node.radial20_heading_freeze(0.0, 0.0, 0.20, KEY)
    assert math.degrees(out) == pytest.approx(-5.0)


def test_offset_is_relative_to_the_line_across_pi():
    line = math.radians(179.0)
    node = _node(yaw=math.radians(-178.0))  # 3 deg left of the line
    out = node.radial20_heading_freeze(line, line, 0.20, KEY)
    assert math.degrees(node.normalize_angle(out - line)) == pytest.approx(3.0)


def test_published_yaw_is_the_captured_heading_after_course_bias():
    yaw = math.radians(2.0)
    bias = math.radians(0.8)
    node = _node(yaw=yaw, bias=bias)
    frozen = node.radial20_heading_freeze(0.0, 0.0, 0.25, KEY)
    assert node.course_compensated_yaw(frozen) == pytest.approx(yaw)


def test_goal_change_releases_the_freeze():
    node = _node(yaw=math.radians(2.0))
    node.radial20_heading_freeze(0.0, 0.0, 0.20, KEY)
    node.current_yaw = math.radians(-1.0)
    out = node.radial20_heading_freeze(0.0, 0.0, 0.50, (3.0, 4.0))
    assert out == 0.0
    assert node.radial20_frozen_bearing is None
    assert node.terminal_bearing_frozen is False


def test_reset_clears_the_freeze():
    node = _node(yaw=math.radians(2.0))
    node.radial20_heading_freeze(0.0, 0.0, 0.20, KEY)
    node._reset_radial20_heading_freeze()
    assert node.radial20_frozen_bearing is None
    assert node.radial20_frozen_goal_key is None


@pytest.mark.parametrize("along", [None, float("nan")])
def test_invalid_along_never_freezes(along):
    node = _node(yaw=math.radians(2.0))
    assert node.radial20_heading_freeze(0.07, 0.0, along, KEY) == 0.07
    assert node.radial20_frozen_bearing is None


@pytest.mark.parametrize("yaw", [None, float("nan")])
def test_invalid_yaw_never_freezes(yaw):
    node = _node(yaw=yaw)
    assert node.radial20_heading_freeze(0.07, 0.0, 0.10, KEY) == 0.07
    assert node.radial20_frozen_bearing is None


def test_literal_stop_releases_the_freeze():
    src = ast.get_source_segment(RPP.read_text(), _method("publish_stop"))
    assert "self._reset_radial20_heading_freeze()" in src


def test_node_default_off_and_launch_enables_it():
    declarations = {}
    for node in ast.walk(_method("__init__")):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "declare_parameter"
            and len(node.args) >= 2
            and isinstance(node.args[0], ast.Constant)
        ):
            declarations[node.args[0].value] = ast.literal_eval(node.args[1])
    assert declarations["radial20_heading_freeze_enabled"] is False
    assert declarations["radial20_heading_freeze_distance_m"] == 0.30
    assert declarations["radial20_heading_freeze_max_offset_deg"] == 5.0

    launch = LAUNCH.read_text()
    assert '"radial20_heading_freeze_enabled": True' in launch
    assert '"radial20_heading_freeze_distance_m": 0.30' in launch
    assert '"radial20_heading_freeze_max_offset_deg": 5.0' in launch
