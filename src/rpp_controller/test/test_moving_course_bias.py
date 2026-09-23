"""Course-bias compensation for moving steering (2026-09-23).

The rover's EKF body-frame velocity showed it travels 0.5-1.8 deg to the side
of its reported yaw.  A pure heading loop then settles at
xtrack ~= lookahead * tan(bias).  These tests exercise the real controller
methods (extracted from source) against that kinematics.
"""

import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


ROOT = Path(__file__).resolve().parents[3]
RPP = ROOT / "src/rpp_controller/rpp_controller/rpp_controller_node.py"
LAUNCH = ROOT / "src/rover_bringup/launch/rover.launch.py"
METHODS = (
    "_reset_moving_course_bias",
    "_update_moving_course_bias",
    "explicit_yaw_rate_command",
    "line_guidance",
    "_recovery_lookahead",
)


def _method(name):
    cls = next(n for n in ast.parse(RPP.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == "RPPController")
    return next(n for n in cls.body
                if isinstance(n, ast.FunctionDef) and n.name == name)


class _Time:
    def __init__(self, ns):
        self.nanoseconds = ns

    def __sub__(self, other):
        return NS(nanoseconds=self.nanoseconds - other.nanoseconds)


class _Clock:
    def __init__(self):
        self.ns = 0

    def now(self):
        return _Time(self.ns)


def _controller(*, enabled=True, deadband=(0.15, 0.30)):
    env = {"math": math}
    module = ast.Module(body=[_method(n) for n in METHODS], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), "<rpp>", "exec"), env)

    node = NS()
    for name in METHODS:
        setattr(node, name, env[name].__get__(node))
    clock = _Clock()
    node.get_clock = lambda: clock
    node.clock = clock
    node.normalize_angle = lambda v: math.atan2(math.sin(v), math.cos(v))
    node.CONTROL_HZ = 50.0
    node.MAXIMUM_MOVING_SPEED_MPS = 1.0
    node.deceleration_max_dt_sec = 0.10
    # Stationary pivot values (launch).
    node.maximum_yaw_rate = 0.45
    node.minimum_yaw_rate = 0.06
    node.pivot_yaw_kp = 1.80
    # Moving steering values (launch).
    node.moving_yaw_rate_max = 0.18
    node.moving_yaw_kp = 0.85
    node.moving_yaw_deadband_enter = math.radians(deadband[0])
    node.moving_yaw_deadband_exit = math.radians(deadband[1])
    node.moving_yaw_rate_slew = 0.60
    node.moving_yaw_damping_gain_min = 0.20
    node.moving_yaw_damping_gain_max = 0.32
    node.moving_yaw_rate_filter_alpha = 0.20
    node.moving_yaw_damping_limit = 0.08
    node.steering_reference_speed = 0.60
    node.moving_yaw_quiet = False
    node.moving_yaw_rate_output = 0.0
    node.moving_yaw_rate_last_time = None
    node.filtered_moving_yaw_rate = 0.0
    node.last_commanded_yaw_rate_radps = 0.0
    # Line guidance values (launch: 0.55 m @ 0.60 m/s reference, 0.35-0.90 m).
    node.line_tracking_lookahead_speed_gain = 0.55 / 0.60
    node.line_tracking_lookahead_xtrack_gain = 1.0
    node.line_tracking_lookahead_min = 0.35
    node.line_tracking_lookahead_max = 0.90
    node.line_tracking_xtrack_deadband = 0.005
    node.command_slew_speed = 1.0
    # Course bias.
    node.moving_course_bias_enabled = enabled
    node.moving_course_bias_time_constant = 2.0
    node.moving_course_bias_min_speed = 0.50
    node.moving_course_bias_max_yaw_rate = 0.05
    node.moving_course_bias_limit = math.radians(3.0)
    node._reset_moving_course_bias()
    # Pivot ramp and recovery lookahead off: isolate course-bias behaviour.
    node.pivot_yaw_rate_slew = 0.0
    node.pivot_yaw_rate_output = 0.0
    node.pivot_yaw_rate_last_time = None
    node.recovery_lookahead_max = 0.0
    node.recovery_lookahead_xtrack_start = 0.05
    node.recovery_lookahead_xtrack_gain = 1.0
    # Odometry state.
    node.current_x = node.current_y = 0.0
    node.current_yaw = 0.0
    node.current_yaw_rate_radps = 0.0
    node.current_body_velocity_forward_mps = 1.0
    node.current_body_velocity_left_mps = 0.0
    return node


def _set_crab(node, speed, beta):
    node.current_body_velocity_forward_mps = speed * math.cos(beta)
    node.current_body_velocity_left_mps = speed * math.sin(beta)


def _step(node, seconds=0.02):
    node.clock.ns += int(seconds * 1e9)


def test_disabled_estimator_returns_zero_and_leaves_command_unchanged():
    node = _controller(enabled=False)
    _set_crab(node, 1.0, math.radians(1.5))
    for _ in range(200):
        _step(node)
        assert node._update_moving_course_bias(1.0) == 0.0
    assert node.moving_course_bias_active is False


def test_estimate_converges_to_measured_crab_with_two_second_time_constant():
    node = _controller()
    beta = math.radians(1.5)
    _set_crab(node, 1.0, beta)
    for _ in range(100):          # 2.0 s = one time constant
        _step(node)
        node._update_moving_course_bias(1.0)
    assert math.degrees(node.moving_course_bias) == pytest.approx(
        1.5 * (1 - math.exp(-1)), abs=0.05)
    for _ in range(400):          # +8 s
        _step(node)
        node._update_moving_course_bias(1.0)
    assert math.degrees(node.moving_course_bias) == pytest.approx(1.5, abs=0.02)
    assert node.moving_course_bias_active is True


@pytest.mark.parametrize("case", ["slow_command", "slow_measured", "turning"])
def test_estimate_is_held_outside_steady_straight_driving(case):
    node = _controller()
    node.moving_course_bias = math.radians(0.8)
    command_speed = 1.0
    _set_crab(node, 1.0, math.radians(2.5))
    if case == "slow_command":
        command_speed = 0.3
    elif case == "slow_measured":
        _set_crab(node, 0.3, math.radians(2.5))
    else:
        node.current_yaw_rate_radps = 0.2
    for _ in range(100):
        _step(node)
        node._update_moving_course_bias(command_speed)
    assert math.degrees(node.moving_course_bias) == pytest.approx(0.8)
    assert node.moving_course_bias_active is False


def test_estimate_is_clamped_to_limit():
    node = _controller()
    _set_crab(node, 1.0, math.radians(8.0))
    for _ in range(1000):
        _step(node)
        node._update_moving_course_bias(1.0)
    # Input is clamped to the limit, so the filtered estimate approaches it
    # from below and can never exceed it.
    assert math.degrees(node.moving_course_bias) == pytest.approx(3.0, abs=0.01)
    assert node.moving_course_bias <= math.radians(3.0)


def test_non_finite_odometry_holds_estimate():
    node = _controller()
    node.moving_course_bias = math.radians(0.5)
    node.current_body_velocity_forward_mps = math.inf
    _step(node)
    node._update_moving_course_bias(1.0)
    assert math.degrees(node.moving_course_bias) == pytest.approx(0.5)
    assert node.moving_course_bias_active is False


def test_stationary_pivot_resets_estimate():
    node = _controller()
    node.moving_course_bias = math.radians(1.2)
    node.moving_course_bias_active = True
    node.explicit_yaw_rate_command(0.5, stationary_pivot=True)
    assert node.moving_course_bias == 0.0
    assert node.moving_course_bias_active is False
    assert node.moving_course_bias_last_time is None


def test_command_steers_course_not_heading():
    """Heading already on target, but travel is 1.5 deg left: steer right."""
    node = _controller()
    node.moving_course_bias = math.radians(1.5)
    _set_crab(node, 1.0, math.radians(1.5))
    command = 0.0
    for _ in range(50):
        _step(node)
        command = node.explicit_yaw_rate_command(0.0, translational_speed_mps=1.0)
    assert command < 0.0


def _simulate(node, beta_deg, seconds=20.0, dt=0.02):
    """Straight east line; rover travels at yaw + beta; perfect yaw-rate tracking."""
    beta = math.radians(beta_deg)
    node.current_y = 0.0
    node.current_yaw = 0.0
    xtrack = []
    for _ in range(int(seconds / dt)):
        _step(node, dt)
        _set_crab(node, 1.0, beta)
        bearing, _ = node.line_guidance(0.0, 0.0, 0.0, math.radians(18.0))
        rate = node.explicit_yaw_rate_command(bearing, translational_speed_mps=1.0)
        node.current_yaw_rate_radps = rate
        node.current_yaw += rate * dt
        node.current_x += math.cos(node.current_yaw + beta) * dt
        node.current_y += math.sin(node.current_yaw + beta) * dt
        xtrack.append(node.current_y)
    return xtrack


def test_closed_loop_offset_without_compensation_matches_lookahead_tan_bias():
    node = _controller(enabled=False)
    xtrack = _simulate(node, 1.5)
    settled = sum(xtrack[-250:]) / 250
    expected = 0.90 * math.tan(math.radians(1.5))    # ~23.6 mm
    assert settled == pytest.approx(expected, abs=0.012)


def test_closed_loop_offset_is_removed_with_compensation():
    node = _controller(enabled=True)
    xtrack = _simulate(node, 1.5)
    settled = [abs(v) for v in xtrack[-250:]]        # last 5 s
    assert max(settled) < 0.006


def test_launch_enables_compensation_and_tightens_deadband():
    tree = ast.parse(LAUNCH.read_text())
    values = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and isinstance(value, ast.Constant):
                    values.setdefault(key.value, value.value)
    assert values["moving_course_bias_enabled"] is True
    assert values["moving_course_bias_time_constant_sec"] == 2.0
    assert values["moving_course_bias_min_speed_mps"] == 0.50
    assert values["moving_course_bias_max_yaw_rate_radps"] == 0.05
    assert values["moving_course_bias_limit_deg"] == 3.0
    assert values["moving_yaw_deadband_enter_deg"] == 0.15
    assert values["moving_yaw_deadband_exit_deg"] == 0.30
