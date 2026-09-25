"""ROS-free regression guards for the September 11 radial20 stop authority."""

import ast
import math
import textwrap
from types import SimpleNamespace

import pytest
from enum import Enum
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


def test_radial20_uses_bounded_goal_bearing_for_steering():
    block = _radial20_motion_block()

    assert "desired_goal_bearing = math.atan2(" in block
    assert "goal_y - self.current_y" in block
    assert "goal_x - self.current_x" in block
    assert "self.terminal_bounded_guidance(" in block
    assert "goal_along_remaining," in block

    assert "self.xtrack_priority_guidance(" not in block
    assert "self.update_xtrack_speed_cap_state(" not in block


def test_radial20_keeps_exclusive_speed_braking_authority():
    block = _radial20_motion_block()

    radial_speed = block.index("speed = radial_result.forward_speed_command_mps")
    publish = block.index("self.publish_velocity_ned(")

    assert radial_speed < publish
    assert "hard_speed_cap_mps=speed" in block
    assert "apply_deceleration=False" in block
    assert "self.xtrack_priority_speed" not in block


def test_radial20_zero_directive_precedes_any_motion_guidance():
    block = _radial20_motion_block()

    zero = block.index(
        "if radial_result.motion_direction is RadialStopMotionDirection.ZERO:"
    )
    stop = block.index("self.publish_stop()", zero)
    guidance = block.index("self.terminal_bounded_guidance(")

    assert zero < stop < guidance


class _MotionDirection(Enum):
    ZERO = "zero"
    FORWARD = "forward"


def _execute_radial20_branch(result):
    """Execute the production branch with observable guidance/publisher seams."""
    calls = []
    node = SimpleNamespace(
        radial20_active=True,
        radial_stop_request_armed=False,
        radial_stop_config=SimpleNamespace(terminal_guidance_distance_m=0.75),
        current_x=0.0,
        current_y=0.0,
        _step_radial20_terminal_for_cycle=lambda **kwargs: result,
        _publish_radial20_result_if_ready=lambda value: None,
        publish_stop=lambda: calls.append(("stop",)),
        log_waiting=lambda message: None,
        log_control=lambda *args: None,
        _record_published_translational_speed=lambda speed: None,
    )

    def guidance(path_bearing, desired_bearing, along_remaining):
        calls.append(("guidance", path_bearing, desired_bearing, along_remaining))
        return 0.1

    def publish(north, east, **kwargs):
        calls.append(("publish", north, east, kwargs))
        return north, east, math.hypot(north, east)

    def freeze(bearing, path_bearing, along_remaining, goal_key):
        calls.append(("freeze", bearing, path_bearing, along_remaining, goal_key))
        return bearing

    node.terminal_bounded_guidance = guidance
    node.radial20_heading_freeze = freeze
    node.publish_velocity_ned = publish
    source = (
        "def run(self):\n"
        "    goal_requires_precision_stop = True\n"
        "    goal_distance = 0.2\n"
        "    goal_along_remaining = 0.2\n"
        "    goal_signed_cross_track = 0.01\n"
        "    goal_x, goal_y = 0.2, 0.01\n"
        "    path_bearing, path_heading_error = 0.0, 0.0\n"
        "    mode_prefix = ''\n"
        + textwrap.indent(textwrap.dedent("        " + _radial20_motion_block()), "    ")
    )
    namespace = {"math": math, "RadialStopMotionDirection": _MotionDirection}
    exec(compile(source, str(NODE_PATH), "exec"), namespace)
    namespace["run"](node)
    return calls


@pytest.mark.parametrize("speed", [0.01, 0.07, 0.4])
def test_terminal_publisher_receives_regulator_speed_as_hard_cap(speed):
    result = SimpleNamespace(
        motion_direction=_MotionDirection.FORWARD,
        forward_speed_command_mps=speed,
        state=SimpleNamespace(value="brake_profile"),
    )
    calls = _execute_radial20_branch(result)

    assert [call[0] for call in calls] == ["guidance", "freeze", "publish"]
    assert calls[0][1:] == pytest.approx((0.0, math.atan2(0.01, 0.2), 0.2))
    assert calls[1][1:4] == pytest.approx((0.1, 0.0, 0.2))
    assert calls[1][4] == (0.2, 0.01)
    _, north, east, options = calls[2]
    assert math.hypot(north, east) == pytest.approx(speed)
    assert options == {
        "apply_acceleration": True,
        "apply_deceleration": False,
        "hard_speed_cap_mps": speed,
        "yaw_enu_rad": 0.1,
    }


@pytest.mark.parametrize("missing_result", [False, True])
def test_zero_or_missing_terminal_result_cannot_reach_motion_publisher(missing_result):
    result = None if missing_result else SimpleNamespace(
        motion_direction=_MotionDirection.ZERO,
        forward_speed_command_mps=0.4,
        state=SimpleNamespace(value="zero_latch"),
    )
    assert _execute_radial20_branch(result) == [("stop",)]
