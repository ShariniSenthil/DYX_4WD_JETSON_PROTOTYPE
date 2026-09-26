"""Approach slowdown to every stop goal (2026-09-26).

Operator requirement: brake over the last 1.0 m with speed proportional to
the distance to go -- 1.0 m -> 1.0 m/s, 0.8 -> 0.8, 0.4 -> 0.4, 0.2-0.3 ->
0.2-0.3, zero at the point. Before this the rover held 1.0 m/s to ~0.63 m
and then braked at ~0.9 m/s^2 (26_09 stage_2/3).
"""

import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[3]
RPP = ROOT / "src/rpp_controller/rpp_controller/rpp_controller_node.py"
LAUNCH = ROOT / "src/rover_bringup/launch/rover.launch.py"
SOURCE = RPP.read_text()


def _method(name):
    cls = next(n for n in ast.parse(SOURCE).body
               if isinstance(n, ast.ClassDef) and n.name == "RPPController")
    return next(n for n in cls.body
                if isinstance(n, ast.FunctionDef) and n.name == name)


def _node(enabled=True):
    env = {"math": math}
    module = ast.Module(body=[_method("approach_speed_cap")], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), "<rpp>", "exec"), env)
    node = NS(
        approach_slowdown_enabled=enabled,
        approach_slowdown_distance=1.0,
        approach_min_speed=0.10,
        cruise_speed=1.0,
    )
    node.approach_speed_cap = env["approach_speed_cap"].__get__(node)
    return node


def test_disabled_is_no_cap():
    node = _node(enabled=False)
    for d in (2.0, 0.5, 0.1, 0.0):
        assert node.approach_speed_cap(d) == math.inf


def test_no_cap_beyond_one_metre_and_cruise_at_one_metre():
    node = _node()
    assert node.approach_speed_cap(1.5) == math.inf
    assert node.approach_speed_cap(1.0) == math.inf
    assert node.approach_speed_cap(0.999) == pytest.approx(1.0, abs=0.01)


@pytest.mark.parametrize("distance,speed", [
    (1.0, 1.0), (0.8, 0.8), (0.7, 0.7), (0.4, 0.4), (0.3, 0.3), (0.2, 0.2),
])
def test_operator_table_speed_equals_distance(distance, speed):
    node = _node()
    assert min(1.0, node.approach_speed_cap(distance - 1e-9)) == \
        pytest.approx(speed, abs=1e-6)


def test_floor_so_the_rover_still_reaches_the_zero_latch():
    node = _node()
    for d in (0.10, 0.05, 0.02, 0.0, -0.01):
        assert node.approach_speed_cap(d) == pytest.approx(0.10)
    assert node.approach_speed_cap(0.15) == pytest.approx(0.15)


def test_never_speeds_up_while_getting_closer():
    node = _node()
    ds = [1.0 - i * 0.005 for i in range(201)]
    caps = [min(1.0, node.approach_speed_cap(d)) for d in ds]
    assert all(a >= b - 1e-12 for a, b in zip(caps, caps[1:]))


def test_below_the_measured_radial20_braking_from_one_metre():
    """radial20 alone kept 1.0 m/s to ~0.67 m: the cap is lower everywhere
    from 0.99 m down to 0.10 m."""
    node = _node()
    for d in (0.99, 0.8, 0.6, 0.4, 0.2, 0.1):
        assert node.approach_speed_cap(d) < min(
            1.0, math.sqrt(2 * 0.75 * (d - 0.003))
        ) + 1e-9


def test_non_finite_distance_is_no_cap():
    node = _node()
    assert node.approach_speed_cap(math.nan) == math.inf


def test_wiring():
    assert 'self.declare_parameter("approach_slowdown_enabled", False)' in SOURCE
    loop = ast.get_source_segment(SOURCE, _method("control_loop"))
    assert loop.index("self.approach_speed_cap_mps = math.inf") < \
        loop.index("self.approach_speed_cap(")
    publish = ast.get_source_segment(SOURCE, _method("publish_velocity_ned"))
    assert "approach_speed_cap_mps" in publish


def test_launch_enables_one_metre_proportional_with_010_floor():
    tree = ast.parse(LAUNCH.read_text())
    values = {}
    for n in ast.walk(tree):
        if isinstance(n, ast.Dict):
            for k, v in zip(n.keys, n.values):
                if isinstance(k, ast.Constant) and isinstance(v, ast.Constant):
                    values.setdefault(k.value, v.value)
    assert values["approach_slowdown_enabled"] is True
    assert values["approach_slowdown_distance_m"] == 1.00
    assert values["approach_min_speed_mps"] == 0.10
