"""Approach slowdown to every stop goal (2026-09-26).

Operator requirement: reduce speed over the last 1.0 m, gradually, and be
slow inside 200 mm. Before this the rover held 1.0 m/s to ~0.63 m and then
braked at ~0.9 m/s^2 (26_09 stage_2/3).
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
        approach_final_distance=0.20,
        approach_final_speed=0.20,
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


def test_slow_inside_200mm():
    node = _node()
    for d in (0.20, 0.15, 0.10, 0.05, 0.02, 0.0, -0.01):
        assert node.approach_speed_cap(d) == pytest.approx(0.20)


def test_constant_gentle_deceleration_between_one_metre_and_200mm():
    node = _node()
    decel = (1.0 - 0.04) / (2 * 0.8)            # 0.60 m/s^2
    for d in (0.9, 0.7, 0.5, 0.3, 0.21):
        v = node.approach_speed_cap(d)
        assert v * v == pytest.approx(0.04 + 2 * decel * (d - 0.20))
    # monotonic: never speeds up while getting closer
    ds = [1.0 - i * 0.01 for i in range(101)]
    caps = [min(1.0, node.approach_speed_cap(d)) for d in ds]
    assert all(a >= b - 1e-12 for a, b in zip(caps, caps[1:]))
    assert node.approach_speed_cap(0.5) == pytest.approx(math.sqrt(0.04 + 0.36))


def test_gentler_than_the_measured_radial20_braking():
    """radial20 alone: sqrt(2*0.75*(d-0.003)); the cap must be at or below it
    from 0.63 m down to 0.20 m so the rover starts slowing earlier."""
    node = _node()
    for d in (0.6, 0.5, 0.4, 0.3, 0.2):
        assert node.approach_speed_cap(d) <= math.sqrt(2 * 0.75 * (d - 0.003))


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


def test_launch_enables_one_metre_to_200mm_at_020():
    tree = ast.parse(LAUNCH.read_text())
    values = {}
    for n in ast.walk(tree):
        if isinstance(n, ast.Dict):
            for k, v in zip(n.keys, n.values):
                if isinstance(k, ast.Constant) and isinstance(v, ast.Constant):
                    values.setdefault(k.value, v.value)
    assert values["approach_slowdown_enabled"] is True
    assert values["approach_slowdown_distance_m"] == 1.00
    assert values["approach_final_distance_m"] == 0.20
    assert values["approach_final_speed_mps"] == 0.20
