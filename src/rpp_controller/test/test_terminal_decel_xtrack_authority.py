"""Regression guards for terminal-deceleration cross-track authority.

The 2026-09-01 18:42/18:44 field bags showed the former 3-degree near-goal
limit pinned while cross-track continued to grow.  These tests execute the
live controller's blend function without importing ROS and verify that the
production launch uses the same grounded 4.5-degree floor.
"""

from __future__ import annotations

import ast
import math
from pathlib import Path
import textwrap

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
NODE_PATH = PACKAGE_ROOT / "rpp_controller" / "rpp_controller_node.py"
LAUNCH_PATH = REPOSITORY_ROOT / "src" / "rover_bringup" / "launch" / "rover.launch.py"
NODE_SOURCE = NODE_PATH.read_text(encoding="utf-8")
LAUNCH_SOURCE = LAUNCH_PATH.read_text(encoding="utf-8")
NODE_TREE = ast.parse(NODE_SOURCE)


def _controller_class() -> ast.ClassDef:
    return next(
        item
        for item in NODE_TREE.body
        if isinstance(item, ast.ClassDef) and item.name == "RPPController"
    )


def _method_source(name: str) -> str:
    method = next(
        item
        for item in _controller_class().body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )
    source = ast.get_source_segment(NODE_SOURCE, method)
    assert source is not None
    return textwrap.dedent(source)


def _parameter_default(name: str):
    init = next(
        item
        for item in _controller_class().body
        if isinstance(item, ast.FunctionDef) and item.name == "__init__"
    )
    for node in ast.walk(init):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "declare_parameter"
            and len(node.args) >= 2
            and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == name
        ):
            continue
        return ast.literal_eval(node.args[1])
    raise AssertionError(f"parameter {name!r} was not declared")


def _production_launch_value(name: str):
    tree = ast.parse(LAUNCH_SOURCE)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        for key, value in zip(node.keys, node.values):
            if (
                isinstance(key, ast.Constant)
                and key.value == name
                and isinstance(value, ast.Constant)
            ):
                return ast.literal_eval(value)
    raise AssertionError(f"launch parameter {name!r} was not found")


def _blend_controller():
    namespace: dict[str, object] = {}
    exec(
        "class Controller:\n"
        + textwrap.indent(_method_source("smoothstep01"), "    ")
        + "\n"
        + textwrap.indent(
            _method_source("terminal_correction_limit_for_along"), "    "
        ),
        {"math": math},
        namespace,
    )
    controller_type = namespace["Controller"]
    # ast.get_source_segment starts at ``def`` and intentionally omits the
    # decorator line; restore the live method's static binding in this
    # ROS-independent execution harness.
    controller_type.smoothstep01 = staticmethod(controller_type.smoothstep01)
    controller = controller_type()
    controller.waypoint_tolerance = 0.03
    controller.terminal_near_correction_start_distance = 0.79
    controller.terminal_near_correction_limit = math.radians(4.5)
    controller.terminal_decel_correction_limit = math.radians(12.0)
    controller.terminal_bearing_frozen = True
    return controller


def test_live_and_launch_near_terminal_authority_are_4_5_degrees():
    assert _parameter_default("terminal_near_correction_limit_deg") == 4.5
    assert _production_launch_value("terminal_near_correction_limit_deg") == 4.5
    assert _production_launch_value("terminal_decel_correction_limit_deg") == 12.0


def test_terminal_authority_blends_monotonically_from_12_to_4_5_degrees():
    controller = _blend_controller()
    samples = [
        math.degrees(controller.terminal_correction_limit_for_along(along))
        for along in (0.79, 0.60, 0.40, 0.20, 0.10, 0.03, 0.00, -0.02)
    ]

    assert samples[0] == pytest.approx(12.0)
    assert samples[-1] == pytest.approx(4.5)
    assert all(left >= right for left, right in zip(samples, samples[1:]))
    assert all(4.5 - 1.0e-9 <= value <= 12.0 + 1.0e-9 for value in samples)
    assert controller.terminal_bearing_frozen is False


def test_terminal_authority_does_not_clip_demands_below_new_floor():
    controller = _blend_controller()
    near_limit = controller.terminal_correction_limit_for_along(0.03)
    unsaturated_demand = math.radians(3.0)

    applied = max(-near_limit, min(near_limit, unsaturated_demand))

    assert math.degrees(near_limit) == pytest.approx(4.5)
    assert applied == pytest.approx(unsaturated_demand)


def test_near_terminal_authority_stays_far_below_native_pivot_entry():
    near_limit_deg = _production_launch_value("terminal_near_correction_limit_deg")

    assert near_limit_deg < _production_launch_value(
        "terminal_decel_correction_limit_deg"
    )
    assert near_limit_deg < 45.0
