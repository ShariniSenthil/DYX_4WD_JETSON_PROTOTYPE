"""Steer the turning point, not the nozzle (2026-09-26).

26_09 stage_1 bags: the pose origin (GNSS antenna = spray nozzle) sits
~0.5 m behind the point the rover turns about, so turning toward the line
first swings the nozzle away from it.  Steering the nozzle gave a swing during
start-up and 58-170 mm overshoot after every pivot.  line_guidance() now steers
a point ``steering_control_point_ahead_m`` ahead along the heading, while the
cross-track it returns (telemetry, gates, stop) stays the nozzle's.

These tests run the real controller methods (extracted from source, same
harness as test_moving_course_bias) against a skid-steer model whose nozzle is
0.5 m behind its turning point.
"""

import ast
import math

import pytest

from test_moving_course_bias import LAUNCH, RPP, _controller, _method, _step

NOZZLE_BEHIND_M = 0.50


def _node(ahead):
    node = _controller(enabled=False)
    node.steering_control_point_ahead = ahead
    node.recovery_lookahead_max = 1.5          # launch value
    return node


def _drive(node, start_offset, start_heading_deg, seconds=10.0, dt=0.02,
           accel=0.5, heading_model="px4", yaw_tau=0.35):
    """East line y=0. Speed ramps 0 -> 1 m/s. Returns nozzle lateral offset.

    heading_model="px4": PX4 follows the published yaw (explicit-yaw
    contract): the published bearing is slewed at 10 deg/s and the heading
    follows it with a first-order lag (field: ~0.4-0.45 s).
    heading_model="rpp_rate": heading follows RPP's own yaw-rate command
    (0.18 rad/s cap, 0.85 kp) through a first-order lag -- slower than the
    field, kept as a pessimistic second plant.
    """
    yaw = math.radians(start_heading_deg)
    # turning point position so that the nozzle starts at (0, start_offset)
    cx = NOZZLE_BEHIND_M * math.cos(yaw)
    cy = start_offset + NOZZLE_BEHIND_M * math.sin(yaw)
    speed = 0.0
    yaw_rate = 0.0
    published = yaw
    slew = math.radians(10.0) * dt
    out = []
    for _ in range(int(seconds / dt)):
        _step(node, dt)
        speed = min(1.0, speed + accel * dt)
        node.command_slew_speed = speed
        node.current_yaw = yaw
        node.current_x = cx - NOZZLE_BEHIND_M * math.cos(yaw)
        node.current_y = cy - NOZZLE_BEHIND_M * math.sin(yaw)
        bearing, nozzle_xtrack = node.line_guidance(
            0.0, 0.0, 0.0, math.radians(11.0)
        )
        out.append(nozzle_xtrack)
        if heading_model == "px4":
            published += max(-slew, min(slew, bearing - published))
            yaw_rate = (published - yaw) / yaw_tau
        else:
            command = node.explicit_yaw_rate_command(
                bearing, translational_speed_mps=speed
            )
            yaw_rate += (command - yaw_rate) * dt / yaw_tau
        node.current_yaw_rate_radps = yaw_rate
        yaw += yaw_rate * dt
        cx += speed * math.cos(yaw) * dt
        cy += speed * math.sin(yaw) * dt
    return out


def _overshoot(trace):
    sign = 1.0 if trace[0] > 0 else -1.0
    return max(0.0, -min(sign * v for v in trace))


def test_zero_offset_is_the_previous_nozzle_law():
    node = _node(0.0)
    node.current_y = 0.10
    node.current_yaw = math.radians(8.0)
    bearing, xtrack = node.line_guidance(0.0, 0.0, 0.0, math.radians(11.0))
    lookahead = min(0.90, 0.55 / 0.60 * 1.0 + 0.10)
    lookahead = max(lookahead, min(1.5, lookahead + (0.10 - 0.05)))
    assert xtrack == pytest.approx(0.10)
    assert bearing == pytest.approx(-math.atan2(0.10 - 0.005, lookahead))


def test_returned_cross_track_is_always_the_nozzle():
    node = _node(0.5)
    node.current_y = -0.20
    node.current_yaw = math.radians(10.0)
    _bearing, xtrack = node.line_guidance(0.0, 0.0, 0.0, math.radians(11.0))
    assert xtrack == pytest.approx(-0.20)


def test_nozzle_on_line_but_nose_turned_left_steers_back_right():
    """The turning point is already left of the line, so do not keep turning
    left -- the nozzle-only law would see zero error here."""
    node = _node(0.5)
    node.current_yaw = math.radians(6.0)
    bearing, xtrack = node.line_guidance(0.0, 0.0, 0.0, math.radians(11.0))
    assert xtrack == pytest.approx(0.0)
    assert bearing < 0.0


def test_control_point_uses_heading_relative_to_the_line():
    node = _node(0.5)
    node.current_yaw = math.radians(90.0 + 6.0)
    bearing, _ = node.line_guidance(
        math.radians(90.0), 0.0, 0.0, math.radians(11.0)
    )
    assert node.normalize_angle(bearing - math.radians(90.0)) < 0.0


@pytest.mark.parametrize("offset,heading", [
    (0.517, 3.2),      # 113945 leg 2
    (0.30, 3.0),
    (-0.32, -3.0),     # 115438 leg 2
])
@pytest.mark.parametrize("yaw_tau", [0.30, 0.45])
def test_post_pivot_recovery_no_overshoot(offset, heading, yaw_tau):
    nozzle = _drive(_node(0.0), offset, heading, yaw_tau=yaw_tau)
    turning_point = _drive(_node(0.5), offset, heading, yaw_tau=yaw_tau)
    assert _overshoot(nozzle) > 0.04
    assert _overshoot(turning_point) < 0.015
    assert max(abs(v) for v in turning_point[-100:]) < 0.01


def test_post_pivot_recovery_improves_with_slow_heading_plant():
    nozzle = _drive(_node(0.0), 0.517, 3.2, heading_model="rpp_rate", yaw_tau=0.30)
    turning_point = _drive(_node(0.5), 0.517, 3.2, heading_model="rpp_rate", yaw_tau=0.30)
    assert _overshoot(turning_point) < 0.65 * _overshoot(nozzle)
    assert max(abs(v) for v in turning_point[-100:]) < \
        0.6 * max(abs(v) for v in nozzle[-100:])


def test_start_on_line_with_heading_error_does_not_swing_through():
    """Released on the line, 3 deg off (pivot release threshold)."""
    nozzle = _drive(_node(0.0), 0.0, 3.0)
    turning_point = _drive(_node(0.5), 0.0, 3.0)
    assert min(nozzle) < -0.015                  # nozzle law swings through
    assert min(turning_point) > -0.005           # turning point does not
    assert max(abs(v) for v in turning_point) < 0.05
    assert max(abs(v) for v in turning_point[-100:]) < 0.01


def test_launch_sets_turning_point_offset():
    tree = ast.parse(LAUNCH.read_text())
    values = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and isinstance(value, ast.Constant):
                    values.setdefault(key.value, value.value)
    assert values["steering_control_point_ahead_m"] == 0.50


def test_parameter_is_declared_read_and_validated():
    source = RPP.read_text()
    assert 'declare_parameter("steering_control_point_ahead_m", 0.0)' in source
    assert "steering_control_point_ahead_m must be finite and in [0, 1.0]" in source
    guidance = ast.get_source_segment(source, _method("line_guidance"))
    assert "steering_control_point_ahead" in guidance


# --------------------------------------------------------------------------
# xtrack_priority_guidance: the latch that owns the whole post-pivot recovery
# (26_09 stage_2: recovery commands matched this law to 0.05-0.18 deg).
# --------------------------------------------------------------------------

PRIORITY_METHODS = ("xtrack_priority_guidance", "_recovery_lookahead")


class _PClock:
    def __init__(self):
        self.ns = 0

    def now(self):
        from test_moving_course_bias import _Time
        return _Time(self.ns)


def _priority(ahead):
    env = {"math": math}
    module = ast.Module(body=[_method(n) for n in PRIORITY_METHODS], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), "<rpp>", "exec"), env)
    node = type("N", (), {})()
    for name in PRIORITY_METHODS:
        setattr(node, name, env[name].__get__(node))
    clock = _PClock()
    node.get_clock = lambda: clock
    node.clock = clock
    node.normalize_angle = lambda v: math.atan2(math.sin(v), math.cos(v))
    node.CONTROL_HZ = 50.0
    node.MAXIMUM_MOVING_SPEED_MPS = 1.0
    node.steering_reference_speed = 0.60
    node.command_slew_speed = 1.0
    # launch values
    node.xtrack_priority_lookahead = 0.55
    node.xtrack_priority_lookahead_max = 0.90
    node.xtrack_priority_correction_limit = math.radians(12.0)
    node.xtrack_prediction_time_sec = 0.25
    node.xtrack_rate_filter_alpha = 0.20
    node.xtrack_correction_slew_rate = math.radians(30.0)
    node.xtrack_neutral_crossing_band = 0.015
    node.recovery_lookahead_max = 1.5
    node.recovery_lookahead_xtrack_start = 0.05
    node.recovery_lookahead_xtrack_gain = 1.0
    # terminal profile (launch values; only used by terminal_mode tests)
    node.terminal_xtrack_prediction_time_sec = 0.35
    node.terminal_xtrack_neutral_crossing_band = 0.005
    node.terminal_xtrack_away_rate_threshold = 0.01
    node.terminal_xtrack_crossing_prediction_time_sec = 0.5
    node.terminal_xtrack_crossing_rate_threshold = 0.01
    node.terminal_xtrack_crossing_predicted_threshold = 0.005
    node.terminal_xtrack_away_lookahead = 0.35
    node.terminal_xtrack_away_correction_limit = math.radians(22.0)
    node.terminal_xtrack_crossing_lookahead = 0.35
    node.terminal_xtrack_crossing_correction_limit = math.radians(22.0)
    node.terminal_xtrack_lookahead = 0.50
    node.terminal_xtrack_correction_limit = math.radians(22.0)
    node.terminal_xtrack_correction_slew_rate = math.radians(60.0)
    node.terminal_xtrack_unwind_slew_rate = math.radians(90.0)
    # state
    node.last_xtrack_sample = None
    node.last_xtrack_sample_time = None
    node.filtered_xtrack_rate = 0.0
    node.last_xtrack_correction = 0.0
    node.last_xtrack_correction_time = None
    node.current_x = node.current_y = 0.0
    node.current_yaw = 0.0
    node.current_yaw_rate_radps = 0.0
    node.steering_control_point_ahead = ahead
    return node


def _settle(node, cycles=200, terminal=False):
    out = None
    for _ in range(cycles):
        node.clock.ns += 20_000_000
        out = node.xtrack_priority_guidance(0.0, 0.0, 0.0, terminal_mode=terminal)
    return out


def test_priority_zero_offset_matches_turning_point_when_heading_on_line():
    """Heading on the line and no yaw rate: both laws see the same point."""
    a, b = _priority(0.0), _priority(0.5)
    for n in (a, b):
        n.current_y = -0.20
    assert _settle(a)[0] == pytest.approx(_settle(b)[0])


def test_priority_nozzle_on_line_nose_left_steers_right():
    nozzle, turning = _priority(0.0), _priority(0.5)
    for n in (nozzle, turning):
        n.current_yaw = math.radians(6.0)
    assert _settle(nozzle)[0] == pytest.approx(0.0, abs=1e-9)
    # 0.5 * sin(6 deg) = 52 mm left of the line -> atan(0.052 / 0.90) = -3.3 deg
    assert _settle(turning)[0] < math.radians(-3.0)


def test_priority_returns_nozzle_cross_track_rate_and_prediction():
    """The latch, telemetry and terminal read these -- they stay the nozzle's."""
    nozzle, turning = _priority(0.0), _priority(0.5)
    for n in (nozzle, turning):
        n.current_y = -0.10
        n.current_yaw = math.radians(8.0)
        n.current_yaw_rate_radps = 0.2
    a, b = _settle(nozzle), _settle(turning)
    assert b[1] == pytest.approx(-0.10)
    assert (b[1], b[2], b[3]) == pytest.approx((a[1], a[2], a[3]))


def test_priority_terminal_profile_is_unchanged():
    nozzle, turning = _priority(0.0), _priority(0.5)
    for n in (nozzle, turning):
        n.current_y = 0.03
        n.current_yaw = math.radians(-5.0)
        n.current_yaw_rate_radps = -0.1
    assert _settle(turning, terminal=True) == pytest.approx(
        _settle(nozzle, terminal=True)
    )


def test_priority_unknown_yaw_rate_is_treated_as_zero():
    a, b = _priority(0.5), _priority(0.5)
    for n in (a, b):
        n.current_y = -0.2
        n.current_yaw = math.radians(4.0)
    a.current_yaw_rate_radps = math.inf
    assert _settle(a)[0] == pytest.approx(_settle(b)[0])


def _drive_priority(ahead, start_offset, start_heading_deg, seconds=10.0, dt=0.02,
                    yaw_tau=0.45, delay=0.12):
    """PX4 explicit-yaw plant fitted on 26_09 (delay 0-0.12 s + 0.40-0.50 s
    lag), published yaw slewed at 10 deg/s, nozzle 0.5 m behind the turning
    point, speed ramping 0 -> 1 m/s."""
    node = _priority(ahead)
    yaw = math.radians(start_heading_deg)
    cx = NOZZLE_BEHIND_M * math.cos(yaw)
    cy = start_offset + NOZZLE_BEHIND_M * math.sin(yaw)
    speed, published, history, out = 0.0, yaw, [], []
    slew = math.radians(10.0) * dt
    lag = int(round(delay / dt))
    for _ in range(int(seconds / dt)):
        node.clock.ns += int(dt * 1e9)
        speed = min(1.0, speed + 0.5 * dt)
        node.command_slew_speed = speed
        node.current_yaw = yaw
        node.current_x = cx - NOZZLE_BEHIND_M * math.cos(yaw)
        node.current_y = cy - NOZZLE_BEHIND_M * math.sin(yaw)
        bearing, nozzle_xtrack = node.xtrack_priority_guidance(0.0, 0.0, 0.0)[:2]
        out.append(nozzle_xtrack)
        published += max(-slew, min(slew, bearing - published))
        history.append(published)
        rate = (history[max(0, len(history) - 1 - lag)] - yaw) / yaw_tau
        node.current_yaw_rate_radps = rate
        yaw += rate * dt
        cx += speed * math.cos(yaw) * dt
        cy += speed * math.sin(yaw) * dt
    return out


@pytest.mark.parametrize("offset,heading", [
    (0.554, 2.9), (0.407, 4.6), (0.301, 3.1), (-0.322, -3.3), (-0.288, -3.0),
])
def test_priority_post_pivot_recovery_overshoot_drops(offset, heading):
    nozzle = _drive_priority(0.0, offset, heading)
    turning_point = _drive_priority(0.5, offset, heading)
    assert _overshoot(nozzle) > 0.03
    assert _overshoot(turning_point) < 0.015
    assert max(abs(v) for v in turning_point[-100:]) < 0.01
