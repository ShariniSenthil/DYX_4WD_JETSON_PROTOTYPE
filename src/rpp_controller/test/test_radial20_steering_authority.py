"""ROS-free regression guards for V5 radial20 steering authority."""

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "rpp_controller" / "rpp_controller_node.py"
NODE_SOURCE = NODE_PATH.read_text(encoding="utf-8")
NODE_TREE = ast.parse(NODE_SOURCE)


def _control_loop_source():
    controller = next(
        item
        for item in NODE_TREE.body
        if isinstance(item, ast.ClassDef) and item.name == "RPPController"
    )
    method = next(
        item
        for item in controller.body
        if isinstance(item, ast.FunctionDef) and item.name == "control_loop"
    )
    source = ast.get_source_segment(NODE_SOURCE, method)
    assert source is not None
    return source


def _radial20_motion_block():
    source = _control_loop_source()
    start = source.index("# radial20 terminal authority:")
    end = source.index(
        "if (\n            self.legacy_terminal_stop_active",
        start,
    )
    return source[start:end]


def test_radial20_uses_predictive_terminal_xtrack_for_steering():
    block = _radial20_motion_block()

    assert "self.xtrack_priority_guidance(" in block
    assert "terminal_mode=True" in block
    assert "goal_x," in block
    assert "goal_y," in block
    assert "self.limit_moving_guidance_bearing(guidance_bearing)" in block

    assert "desired_goal_bearing = math.atan2(" not in block
    assert "self.terminal_bounded_guidance(" not in block


def test_radial20_keeps_speed_braking_authority_and_xtrack_only_caps_downward():
    block = _radial20_motion_block()

    radial_speed = block.index("speed = radial_result.forward_speed_command_mps")
    cap = block.index("speed = min(speed, self.xtrack_priority_speed)")
    publish = block.index("self.publish_velocity_ned(")

    assert radial_speed < cap < publish
    assert "self.update_xtrack_speed_cap_state(" in block
    assert "hard_speed_cap_mps=speed" in block
    assert "apply_deceleration=False" in block


def test_radial20_zero_directive_still_precedes_any_motion_guidance():
    block = _radial20_motion_block()

    zero = block.index(
        "if radial_result.motion_direction is RadialStopMotionDirection.ZERO:"
    )
    stop = block.index("self.publish_stop()", zero)
    predictive = block.index("self.xtrack_priority_guidance(")

    assert zero < stop < predictive
