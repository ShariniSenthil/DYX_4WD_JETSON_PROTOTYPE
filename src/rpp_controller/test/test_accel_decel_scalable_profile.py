"""Regression guards for V2 fixed 800 mm scalable cruise profiles."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RPP_PATH = PACKAGE_ROOT / "rpp_controller" / "rpp_controller_node.py"
LAUNCH_PATH = (
    Path(__file__).resolve().parents[2]
    / "rover_bringup"
    / "launch"
    / "rover.launch.py"
)


def _controller_method(name):
    tree = ast.parse(RPP_PATH.read_text(encoding="utf-8"))
    controller = next(
        item for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "RPPController"
    )
    return next(
        item for item in controller.body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )


def _method_source(name):
    source = RPP_PATH.read_text(encoding="utf-8")
    return ast.get_source_segment(source, _controller_method(name))


def _terminal_speed_method():
    method = _controller_method("terminal_speed_for_along_remaining")
    module = ast.Module(body=[method], type_ignores=[])
    namespace = {"math": math}
    exec(compile(ast.fix_missing_locations(module), str(RPP_PATH), "exec"), namespace)
    return namespace["terminal_speed_for_along_remaining"]


def test_launch_contract():
    launch = LAUNCH_PATH.read_text(encoding="utf-8")
    assert "CRUISE_SPEED_MIN_MPS = 0.60" in launch
    assert "CRUISE_SPEED_MAX_MPS = 1.00" in launch
    assert "LONGITUDINAL_PROFILE_DISTANCE_M = 0.80" in launch
    assert 'os.environ.get("DYX_CRUISE_SPEED_MPS", "1.00")' in launch
    assert '"acceleration_distance_m": LONGITUDINAL_PROFILE_DISTANCE_M' in launch
    assert '"deceleration_distance_m": LONGITUDINAL_PROFILE_DISTANCE_M' in launch
    assert '"approach_slowdown_enabled": False' in launch
    assert '"radial_stop_terminal_guidance_distance_m": 0.90' in launch
    assert '"post_pivot_reanchor_all_legs": False' in launch
    assert '"steering_control_point_ahead_m": 0.50' in launch


def test_radial20_receives_fixed_distance_cap():
    step = _method_source("_step_radial20_terminal_for_cycle")
    assert "self.deceleration_speed_limit(" in step
    assert "fixed_distance_profile_speed" in step
    assert "tracking_speed_command = min(" in step
    assert step.index("tracking_speed_command = min(") < step.index(
        "self.radial_stop_regulator.step(sample)"
    )


def test_radial20_rejects_competing_slowdown():
    validate = _method_source("validate_parameters")
    assert "self.radial20_active and self.approach_slowdown_enabled" in validate


def test_radial20_arms_from_along_distance():
    control = _method_source("control_loop")
    start = control.index("# radial20 terminal authority:")
    end = control.index(
        "if (\n            self.legacy_terminal_stop_active", start
    )
    block = control[start:end]
    assert "goal_along_remaining" in block


@pytest.mark.parametrize("cruise", [0.60, 0.70, 0.80, 0.90, 1.00])
def test_terminal_profile_endpoints(cruise):
    floor = 0.15
    start = 0.80
    tolerance = 0.020
    rate = (cruise**2 - floor**2) / (2.0 * (start - tolerance))
    node = SimpleNamespace(
        cruise_speed=cruise,
        deceleration_required=True,
        deceleration_distance=start,
        waypoint_tolerance=tolerance,
        deceleration_floor_speed=floor,
        deceleration_rate=rate,
    )
    terminal_speed = _terminal_speed_method().__get__(node)
    assert terminal_speed(start) == pytest.approx(cruise)
    assert terminal_speed(tolerance) == pytest.approx(floor)
    samples = [terminal_speed(d) for d in (0.80, 0.60, 0.40, 0.20, 0.020)]
    assert all(floor <= value <= cruise for value in samples)
    assert all(a >= b for a, b in zip(samples, samples[1:]))


def test_node_defaults_are_800mm():
    source = RPP_PATH.read_text(encoding="utf-8")
    assert 'declare_parameter("acceleration_distance_m", 0.80)' in source
    assert 'declare_parameter("deceleration_distance_m", 0.80)' in source
