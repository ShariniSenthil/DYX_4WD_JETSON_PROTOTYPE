"""PX4-Mission-style corner for the RPP pivot (2026-09-23).

Compared against PX4 Mission mode on the same L-shape (ULogs 210/211 vs RPP
bag 190952), PX4 corners in ~2.7 s and rejoins the next leg without overshoot
because it (1) pivots toward a lookahead point on the new line re-aimed from
the current pose, (2) ramps to ~45 deg/s, (3) hands back to driving at 6 deg
while still turning, and (4) recovers with a ~1.5 m lookahead.  These tests
pin the RPP implementation of the same four behaviours; every feature
defaults off in code and is enabled in rover.launch.py.
"""

import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from rpp_controller.legacy_alignment import (
    LegacyAlignmentConfig,
    LegacyAlignmentDirective,
    LegacyAlignmentInput,
    LegacyAlignmentLifecycle,
    LegacyAlignmentPhase,
)


ROOT = Path(__file__).resolve().parents[3]
RPP = ROOT / "src/rpp_controller/rpp_controller/rpp_controller_node.py"
LAUNCH = ROOT / "src/rover_bringup/launch/rover.launch.py"
METHODS = (
    "_pivot_target_offset_limit",
    "_pivot_dynamic_target_bearing",
    "_recovery_lookahead",
    "terminal_native_pivot_command",
    "reset_terminal_native_pivot",
    "explicit_yaw_rate_command",
    "_reset_moving_course_bias",
    "_update_moving_course_bias",
    "line_guidance",
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


def _controller(*, dynamic=True, handover=True, slew=1.2, recovery_max=1.5,
                path_limit_deg=11.0):
    env = {
        "math": math,
        "LegacyAlignmentPhase": LegacyAlignmentPhase,
    }
    module = ast.Module(body=[_method(n) for n in METHODS], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), "<rpp>", "exec"), env)
    node = NS()
    for name in METHODS:
        setattr(node, name, env[name].__get__(node))
    clock = _Clock()
    node.clock = clock
    node.get_clock = lambda: clock
    node.get_logger = lambda: NS(warn=lambda *_: None, error=lambda *_: None)
    node.normalize_angle = lambda v: math.atan2(math.sin(v), math.cos(v))
    node.CONTROL_HZ = 50.0
    node.MAXIMUM_MOVING_SPEED_MPS = 1.0
    node.deceleration_max_dt_sec = 0.10
    node.rpp_explicit_yaw_enabled = True
    # Pivot (launch values).
    node.pivot_enter_angle = math.radians(15.0)
    node.pivot_exit_angle = math.radians(4.0)
    node.terminal_native_pivot_enter_error = math.radians(15.0)
    node.terminal_native_pivot_release_error = math.radians(1.5)
    node.terminal_native_pivot_request_error = math.radians(60.0)
    node.maximum_yaw_rate = 0.75
    node.minimum_yaw_rate = 0.06
    node.pivot_yaw_kp = 1.80
    node.pivot_dynamic_target_enabled = dynamic
    node.pivot_target_lookahead = 1.5
    node.pivot_moving_handover_enabled = handover
    node.pivot_handover_release_error = math.radians(6.0)
    node.pivot_yaw_rate_slew = slew
    node.pivot_yaw_rate_output = 0.0
    node.pivot_yaw_rate_last_time = None
    node.legacy_alignment = NS(phase=LegacyAlignmentPhase.ENTRY,
                               native_carrier_issued=False)
    node.reset_terminal_native_pivot()
    # Moving steering (launch values).
    node.moving_yaw_rate_max = 0.18
    node.moving_yaw_kp = 0.85
    node.moving_yaw_deadband_enter = math.radians(0.15)
    node.moving_yaw_deadband_exit = math.radians(0.30)
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
    node.moving_course_bias_enabled = False
    node._reset_moving_course_bias()
    # Line guidance (launch values) + recovery lookahead.
    node.line_tracking_lookahead_speed_gain = 0.55 / 0.60
    node.line_tracking_lookahead_xtrack_gain = 1.0
    node.line_tracking_lookahead_min = 0.35
    node.line_tracking_lookahead_max = 0.90
    node.line_tracking_xtrack_deadband = 0.005
    node.recovery_lookahead_max = recovery_max
    node.recovery_lookahead_xtrack_start = 0.05
    node.recovery_lookahead_xtrack_gain = 1.0
    node.path_correction_limit = math.radians(path_limit_deg)
    node.command_slew_speed = 1.0
    # Pose.
    node.current_x = node.current_y = 0.0
    node.current_yaw = 0.0
    node.current_yaw_rate_radps = 0.0
    node.current_body_velocity_forward_mps = 0.0
    node.current_body_velocity_left_mps = 0.0
    return node


def _step(node, seconds=0.02):
    node.clock.ns += int(seconds * 1e9)


# -- 1. dynamic pivot target -------------------------------------------------

def test_dynamic_target_disabled_returns_line_bearing():
    node = _controller(dynamic=False)
    node.current_y = 0.3
    assert node._pivot_dynamic_target_bearing(0.0, 0.0, 0.0) == 0.0


@pytest.mark.parametrize("offset", [0.05, 0.15, -0.10])
def test_dynamic_target_aims_back_at_the_line(offset):
    node = _controller()
    node.current_y = offset                      # +left of an east-going line
    bearing = node._pivot_dynamic_target_bearing(0.0, 0.0, 0.0)
    assert bearing == pytest.approx(-math.atan2(offset, 1.5))
    assert bearing * offset < 0.0                # turns toward the line


def test_dynamic_target_offset_is_clamped_below_reentry():
    node = _controller()
    node.current_y = 0.6                         # atan(0.6/1.5) = 21.8 deg
    bearing = node._pivot_dynamic_target_bearing(0.0, 0.0, 0.0)
    limit = node._pivot_target_offset_limit()
    assert math.degrees(limit) == pytest.approx(8.0)
    assert bearing == pytest.approx(-limit)
    # Handover leaves heading within 6 deg of the target -> path error <= 14.
    assert math.degrees(limit + node.pivot_handover_release_error) < 15.0


def test_keeper_reaims_every_cycle_when_dynamic():
    node = _controller()
    active, _, _ = node.terminal_native_pivot_command(math.radians(90.0), "T")
    assert active and node.terminal_native_pivot_true_bearing == pytest.approx(
        math.radians(90.0))
    node.terminal_native_pivot_command(math.radians(84.0), "T")
    assert node.terminal_native_pivot_true_bearing == pytest.approx(
        math.radians(84.0))


def test_keeper_keeps_latched_bearing_when_not_dynamic():
    node = _controller(dynamic=False)
    node.terminal_native_pivot_command(math.radians(90.0), "T")
    node.terminal_native_pivot_command(math.radians(84.0), "T")
    assert node.terminal_native_pivot_true_bearing == pytest.approx(
        math.radians(90.0))


# -- 3. release threshold ----------------------------------------------------

@pytest.mark.parametrize("handover,release_deg", [(True, 6.0), (False, 1.5)])
def test_keeper_release_threshold(handover, release_deg):
    node = _controller(handover=handover)
    node.terminal_native_pivot_command(math.radians(90.0), "T")
    node.current_yaw = math.radians(90.0 - release_deg - 0.3)
    active, _, _ = node.terminal_native_pivot_command(math.radians(90.0), "T")
    assert active is True
    node.current_yaw = math.radians(90.0 - release_deg + 0.3)
    active, _, _ = node.terminal_native_pivot_command(math.radians(90.0), "T")
    assert active is False


def _lifecycle(moving_handover):
    return LegacyAlignmentLifecycle(LegacyAlignmentConfig(
        native_release_heading_rad=math.radians(1.5),
        settle_reentry_heading_rad=math.radians(4.0),
        stop_speed_mps=0.06,
        stop_yaw_rate_radps=0.05,
        settle_sec=0.20,
        post_settle_hold_sec=0.20,
        non_pivot_release_xtrack_m=0.008,
        non_pivot_release_heading_rad=math.radians(1.5),
        non_pivot_hold_sec=0.20,
        fast_capture_max_cross_track_m=0.05,
        pivot_enter_rad=math.radians(15.0),
        pivot_keeper_timeout_sec=10.0,
        pre_pivot_timeout_sec=8.0,
        stationary_violation_debounce_sec=0.10,
        reanchor_all_legs=False,
        moving_handover=moving_handover,
    ))


def _sample(t, heading_deg, native, speed=0.0, yaw_rate=0.0):
    return LegacyAlignmentInput(
        now_sec=t, telemetry_fresh=True, measured_speed_mps=speed,
        measured_yaw_rate_radps=yaw_rate,
        path_heading_error_rad=math.radians(heading_deg),
        alignment_cross_track_m=0.0, native_pivot_active=native,
        first_approach=False,
    )


def _run_to_native_pivot(life):
    t = 0.0
    life.step(_sample(t, 90.0, True))                    # ENTRY -> PRE_PIVOT_STOP
    while life.phase is LegacyAlignmentPhase.PRE_PIVOT_STOP:
        t += 0.05
        life.step(_sample(t, 90.0, True))
    assert life.phase is LegacyAlignmentPhase.NATIVE_PIVOT
    t += 0.05
    r = life.step(_sample(t, 60.0, True, yaw_rate=0.7))
    assert r.directive is LegacyAlignmentDirective.NATIVE_CARRIER
    life.ack_native_carrier_published()
    return t


def test_lifecycle_moving_handover_skips_settle():
    life = _lifecycle(True)
    t = _run_to_native_pivot(life)
    # Still turning at release: no stationary requirement.
    r = life.step(_sample(t + 0.05, 5.0, False, yaw_rate=0.2))
    assert r.directive is LegacyAlignmentDirective.COMPLETE_MOVING_HANDOVER
    assert r.pivot_complete is True
    assert r.consumed is False
    assert life.phase is not LegacyAlignmentPhase.PIVOT_SETTLE


def test_lifecycle_without_handover_still_settles():
    life = _lifecycle(False)
    t = _run_to_native_pivot(life)
    r = life.step(_sample(t + 0.05, 1.0, False, yaw_rate=0.2))
    assert life.phase is LegacyAlignmentPhase.PIVOT_SETTLE
    assert r.directive is LegacyAlignmentDirective.HOLD_ZERO


# -- 2. pivot yaw-rate ramp --------------------------------------------------

def test_pivot_rate_ramps_up_at_slew_limit():
    node = _controller(slew=1.2)
    outputs = []
    for _ in range(40):
        _step(node)
        outputs.append(node.explicit_yaw_rate_command(
            math.radians(90.0), stationary_pivot=True))
    assert outputs[0] == pytest.approx(1.2 / 50.0)          # one 20 ms step
    steps = [b - a for a, b in zip(outputs, outputs[1:]) if b < 0.75]
    assert max(steps) <= 1.2 * 0.02 + 1e-9
    assert outputs[-1] == pytest.approx(0.75)               # reaches the cap


def test_pivot_rate_falls_immediately_near_target():
    node = _controller(slew=1.2)
    node.pivot_yaw_rate_output = 0.75
    node.pivot_yaw_rate_last_time = node.clock.now()
    _step(node)
    node.current_yaw = math.radians(88.0)                   # 2 deg to go
    out = node.explicit_yaw_rate_command(math.radians(90.0), stationary_pivot=True)
    assert out == pytest.approx(1.8 * math.radians(2.0))    # proportional, no ramp


def test_pivot_rate_step_when_slew_disabled():
    node = _controller(slew=0.0)
    _step(node)
    out = node.explicit_yaw_rate_command(math.radians(90.0), stationary_pivot=True)
    assert out == pytest.approx(0.75)


# -- 4. recovery lookahead ---------------------------------------------------

@pytest.mark.parametrize("xtrack", [0.0, 0.02, 0.05])
def test_recovery_lookahead_unchanged_near_the_line(xtrack):
    node = _controller()
    assert node._recovery_lookahead(0.9, xtrack) == pytest.approx(0.9)


def test_recovery_lookahead_grows_with_xtrack_to_cap():
    node = _controller()
    assert node._recovery_lookahead(0.9, 0.30) == pytest.approx(1.15)
    assert node._recovery_lookahead(0.9, 2.0) == pytest.approx(1.5)


def test_recovery_lookahead_disabled_is_identity():
    node = _controller(recovery_max=0.0)
    assert node._recovery_lookahead(0.9, 0.5) == pytest.approx(0.9)


def _recover(node, x0, seconds=4.0, dt=0.02):
    """Rover on an east line, x0 left of it, heading along the line, 1 m/s."""
    node.current_x, node.current_y, node.current_yaw = 0.0, x0, 0.0
    ys = []
    for _ in range(int(seconds / dt)):
        _step(node, dt)
        bearing, _ = node.line_guidance(0.0, 0.0, 0.0, node.path_correction_limit)
        rate = node.explicit_yaw_rate_command(bearing, translational_speed_mps=1.0)
        node.current_yaw_rate_radps = rate
        node.current_yaw += rate * dt
        node.current_x += math.cos(node.current_yaw) * dt
        node.current_y += math.sin(node.current_yaw) * dt
        ys.append(node.current_y)
    return ys


@pytest.mark.parametrize("x0,ratio", [(0.30, 0.90), (0.50, 0.60)])
def test_post_pivot_recovery_overshoot_is_reduced(x0, ratio):
    """190952 leg 3 started 0.27-0.33 m off; old settings overshot at 1 m/s.

    Idealised kinematics (perfect yaw-rate tracking): 0.30 m -> 77 vs 65 mm,
    0.50 m -> 113 vs 57 mm. The residual is the moving heading loop's unwind
    rate (moving_yaw_rate_max 0.18 rad/s), a separate tuning item.
    """
    old = _recover(_controller(recovery_max=0.0, path_limit_deg=18.0), x0)
    new = _recover(_controller(recovery_max=1.5, path_limit_deg=11.0), x0)
    assert -min(new) < ratio * -min(old)


def test_recovery_heading_stays_below_reentry():
    node = _controller(recovery_max=1.5, path_limit_deg=11.0)
    node.current_x, node.current_y, node.current_yaw = 0.0, 0.5, 0.0
    worst = 0.0
    for _ in range(250):
        _step(node)
        bearing, _ = node.line_guidance(0.0, 0.0, 0.0, node.path_correction_limit)
        rate = node.explicit_yaw_rate_command(bearing, translational_speed_mps=1.0)
        node.current_yaw += rate * 0.02
        node.current_x += math.cos(node.current_yaw) * 0.02
        node.current_y += math.sin(node.current_yaw) * 0.02
        worst = max(worst, abs(math.degrees(node.current_yaw)))
    assert worst < 15.0


# -- launch contract ---------------------------------------------------------

def _launch_values():
    values = {}
    for node in ast.walk(ast.parse(LAUNCH.read_text())):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if isinstance(value, ast.Constant):
                    if isinstance(key, ast.Constant):
                        values.setdefault(key.value, []).append(value.value)
                    elif isinstance(key, ast.BinOp) or isinstance(key, ast.JoinedStr):
                        pass
    return values


def _joined_key_value(name):
    """Bridge uses implicitly concatenated string keys ("a_" "b")."""
    for node in ast.walk(ast.parse(LAUNCH.read_text())):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and key.value == name:
                    yield value.value


def test_launch_enables_mission_style_corner():
    values = _launch_values()
    one = lambda k: values[k][0]
    assert one("pivot_dynamic_target_enabled") is True
    assert one("pivot_target_lookahead_m") == 1.5
    assert one("pivot_moving_handover_enabled") is True
    assert one("pivot_handover_release_error_deg") == 6.0
    assert one("pivot_yaw_rate_slew_radps2") == 1.2
    assert one("recovery_lookahead_max_m") == 1.5


def test_launch_moving_corrections_stay_below_pivot_reentry():
    values = _launch_values()
    enter = values["pivot_enter_angle_deg"][0]
    for key in (
        "path_correction_limit_deg",
        "segment_alignment_correction_limit_deg",
        "terminal_line_correction_limit_deg",
        "xtrack_priority_correction_limit_deg",
    ):
        # >= 3 deg margin for heading overshoot above the commanded bearing.
        assert values[key][0] <= enter - 3.0, key


def test_launch_yaw_rate_chain_is_consistent():
    rates = list(_joined_key_value("maximum_yaw_rate_radps"))
    # rpp_controller and cmd_vel_bridge both declare it; the bridge clamps
    # explicit yaw-rate to its value, so it must not be below RPP's.
    assert len(rates) == 2 and len(set(rates)) == 1
    # FCU RO_YAW_RATE_LIM is 45 deg/s; the firmware clamps above that.
    assert rates[0] <= math.radians(45.0)
