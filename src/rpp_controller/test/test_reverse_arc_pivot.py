"""Reverse-arc pivot: planner unit tests, closed-loop plant tests, and node wiring."""

import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from rpp_controller.reverse_arc_pivot import (
    MODE_ARC,
    MODE_STRAIGHT,
    ReverseArcConfig,
    ReverseArcPivot,
)

ROOT = Path(__file__).resolve().parents[3]
RPP = ROOT / "src/rpp_controller/rpp_controller/rpp_controller_node.py"
LAUNCH = ROOT / "src/rover_bringup/launch/rover.launch.py"
BAG = ROOT / "scripts/bag_autorecord.py"
HALF_TRACK = 0.63 / 2.0          # RD_WHEEL_TRACK
ARC = ReverseArcConfig(mode=MODE_ARC)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# --------------------------------------------------------------- plant sim
def fly(turn_deg, true_ahead, true_left, cfg=None, *, forward_only=False,
        tau=0.40, dt=0.002, t_max=20.0):
    """Skid-steer plant: centre moves along heading; PX4 heading P + 45 dps
    limit; ~20 ms transport. After a straight-mode REVERSE_DONE the plant flies
    the caller's stationary pivot (target yaw, zero speed). Returns final
    nozzle cross/along to the new line and the per-side wheel speeds."""
    arc = ReverseArcPivot(cfg or ReverseArcConfig())
    line_bearing = math.radians(turn_deg)
    yaw = 0.0
    cx = true_ahead * math.cos(yaw) - true_left * math.sin(yaw)
    cy = true_ahead * math.sin(yaw) + true_left * math.cos(yaw)
    assert arc.start(0.0, yaw, line_bearing, 0.0, 0.0, line_bearing)
    t = rate = speed = 0.0
    history = []
    wheels = []
    command = None
    released = False
    pivoting = False
    tick = 0
    while t < t_max:
        nx = cx - true_ahead * math.cos(yaw) + true_left * math.sin(yaw)
        ny = cy - true_ahead * math.sin(yaw) - true_left * math.cos(yaw)
        if tick % 10 == 0:
            if not pivoting:
                command = arc.step(t, nx, ny, yaw, rate)
                if not command.active:
                    if command.fallback:
                        break
                    pivoting = True
            if pivoting:
                command = NS(yaw_enu_rad=line_bearing, speed_mps=0.0)
            if abs(wrap(line_bearing - yaw)) <= math.radians(3.5):
                released = True
                break
        history.append(command)
        c = history[-10] if len(history) >= 10 else history[0]
        rc = max(-math.radians(45), min(math.radians(45), wrap(c.yaw_enu_rad - yaw) / tau))
        rate += (rc - rate) * dt / 0.05
        target = abs(c.speed_mps) if forward_only else c.speed_mps
        speed += (target - speed) * dt / 0.05
        wheels.append((speed - rate * HALF_TRACK, speed + rate * HALF_TRACK))
        cx += speed * math.cos(yaw) * dt
        cy += speed * math.sin(yaw) * dt
        yaw += rate * dt
        t += dt
        tick += 1
    n = (-math.sin(line_bearing), math.cos(line_bearing))
    u = (math.cos(line_bearing), math.sin(line_bearing))
    return NS(cross=nx * n[0] + ny * n[1], along=nx * u[0] + ny * u[1],
              released=released, arc=arc, t=t, wheels=wheels)


def one_sided_fraction(wheels, moving=0.10, ratio=0.5):
    """Fraction of moving samples where one side runs at < ratio x the other
    (the stalled-inner-side condition that drags the chassis)."""
    moving_samples = [w for w in wheels if max(abs(w[0]), abs(w[1])) > moving]
    if not moving_samples:
        return 0.0
    bad = sum(1 for l, r in moving_samples
              if min(abs(l), abs(r)) < ratio * max(abs(l), abs(r)))
    return bad / len(moving_samples)


# ------------------------------------------------------------- unit tests
def test_config_rejects_bad_values():
    with pytest.raises(ValueError):
        ReverseArcConfig(max_speed_mps=0.0)
    with pytest.raises(ValueError):
        ReverseArcConfig(front_weight=1.0)
    with pytest.raises(ValueError):
        ReverseArcConfig(freeze_remaining_rad=math.radians(25.0))
    with pytest.raises(ValueError):
        ReverseArcConfig(centre_left_left_turn_m=math.nan)


def test_small_turn_and_nonfinite_input_are_not_started():
    arc = ReverseArcPivot(ReverseArcConfig())
    assert not arc.start(0.0, 0.0, math.radians(15.0), 0.0, 0.0, 0.2)
    assert not arc.active
    assert not arc.start(0.0, math.nan, 1.0, 0.0, 0.0, 1.0)
    assert arc.step(0.0, 0.0, 0.0, 0.0, 0.0).active is False


def test_direction_selects_calibrated_centre():
    cfg = ReverseArcConfig()
    arc = ReverseArcPivot(cfg)
    assert arc.start(0.0, 0.0, math.radians(90), 0, 0, math.radians(90))
    assert (arc.ahead, arc.left) == (cfg.centre_ahead_left_turn_m, cfg.centre_left_left_turn_m)
    assert arc.start(0.0, 0.0, math.radians(-90), 0, 0, math.radians(-90))
    assert (arc.ahead, arc.left) == (cfg.centre_ahead_right_turn_m, cfg.centre_left_right_turn_m)


def test_yaw_reference_is_smooth_and_bounded():
    cfg = ReverseArcConfig()
    arc = ReverseArcPivot(cfg)
    arc.start(10.0, 0.3, 0.3 + math.radians(120), 0, 0, 0.3 + math.radians(120))
    assert arc.yaw_reference(10.0) == (pytest.approx(0.3), 0.0)
    end_yaw, end_rate = arc.yaw_reference(10.0 + arc.duration_sec)
    assert end_yaw == pytest.approx(0.3 + math.radians(120))
    assert end_rate == 0.0
    peak = max(abs(arc.yaw_reference(10.0 + arc.duration_sec * k / 100)[1]) for k in range(101))
    assert peak <= cfg.peak_yaw_rate_radps + 1e-9
    assert arc.duration_sec >= cfg.min_duration_sec


def test_stationary_pivot_prediction_matches_geometry():
    # Pure rotation about C: heading east, nozzle at P, centre 0.49 m ahead.
    # After a 90 deg left turn the nozzle sits at (0.49, -0.49), i.e. 0.49 m to
    # the RIGHT of the new northbound line through P (left-positive cross).
    arc = ReverseArcPivot(ReverseArcConfig(centre_left_left_turn_m=0.0))
    arc.start(0.0, 0.0, math.radians(90), 0.0, 0.0, math.radians(90))
    assert arc.predict_final_cross(0.0, 0.0, 0.0, 0.0) == pytest.approx(-0.49, abs=1e-9)


def test_yawspeed_is_never_zero_while_active():
    arc = ReverseArcPivot(ARC)
    arc.start(0.0, 0.0, math.radians(-90), 0, 0, math.radians(-90))
    command = arc.step(0.0, 0.0, 0.0, 0.0, 0.0)
    assert command.active
    assert command.speed_mps == 0.0            # no rotation measured yet
    assert command.yaw_rate_radps < 0.0        # sign follows the turn
    assert abs(command.yaw_rate_radps) >= math.radians(5.0) - 1e-12


def test_speed_is_clamped_and_reverse_for_a_right_angle_turn():
    cfg = ARC
    arc = ReverseArcPivot(cfg)
    arc.start(0.0, 0.0, math.radians(90), 0, 0, math.radians(90))
    command = arc.step(1.0, 0.0, 0.0, math.radians(20), math.radians(40))
    assert command.speed_mps < 0.0
    assert abs(command.speed_mps) <= cfg.max_speed_mps + 1e-12


def test_final_turn_is_frozen_to_zero_speed():
    arc = ReverseArcPivot(ARC)
    arc.start(0.0, 0.0, math.radians(90), 0, 0, math.radians(90))
    command = arc.step(3.0, 0.0, -0.4, math.radians(85), math.radians(20))
    assert command.speed_mps == 0.0
    assert command.reason == "FROZEN_FINAL_TURN"


def test_timeout_and_reverse_cap_abort_to_fallback():
    arc = ReverseArcPivot(ARC)
    arc.start(0.0, 0.0, math.radians(90), 0, 0, math.radians(90))
    command = arc.step(100.0, 0.0, 0.0, 0.0, 0.0)
    assert (command.active, command.fallback, command.reason) == (False, True, "TIMEOUT")

    arc = ReverseArcPivot(ReverseArcConfig(mode=MODE_ARC, max_reverse_travel_m=0.05))
    arc.start(0.0, 0.0, math.radians(90), 0, 0, math.radians(90))
    arc.step(0.0, 0.0, 0.0, 0.0, 0.0)
    command = arc.step(0.1, -0.2, 0.0, 0.0, 0.0)   # centre moved 0.2 m back
    assert command.fallback and command.reason == "MAX_REVERSE_TRAVEL"


def test_nonfinite_measurement_aborts_to_fallback():
    arc = ReverseArcPivot(ReverseArcConfig())
    arc.start(0.0, 0.0, math.radians(90), 0, 0, math.radians(90))
    command = arc.step(0.1, 0.0, 0.0, 0.0, math.inf)
    assert command.fallback and command.reason == "NON_FINITE_INPUT"
    assert arc.step(0.2, 0.0, 0.0, 0.0, 0.0).fallback


# --------------------------------------------------------- closed-loop sim
LEFT_TRUE = ((0.49, 0.03), (0.38, 0.05), (0.60, 0.05))
RIGHT_TRUE = ((0.39, -0.044), (0.33, -0.08), (0.50, -0.02))


@pytest.mark.parametrize("turn", [90, 135, 160, -90, -135, -160])
def test_arc_large_turns_end_within_60mm_of_the_new_line(turn):
    for ahead, left in (LEFT_TRUE if turn > 0 else RIGHT_TRUE):
        result = fly(turn, ahead, left, ARC)
        assert result.released, (turn, ahead, left)
        assert abs(result.cross) <= 0.060, (turn, ahead, left, result.cross)
        assert result.along < 0.0          # nozzle behind the point, on the line


@pytest.mark.parametrize("turn", [25, 45, -25, -45])
def test_arc_small_turns_end_within_110mm_across_centre_spread(turn):
    for ahead, left in (LEFT_TRUE if turn > 0 else RIGHT_TRUE):
        result = fly(turn, ahead, left, ARC)
        assert result.released
        assert abs(result.cross) <= 0.110, (turn, ahead, left, result.cross)


@pytest.mark.parametrize("mode", [MODE_STRAIGHT, MODE_ARC])
@pytest.mark.parametrize("turn", [90, -90])
def test_calibrated_centre_beats_the_stationary_pivot_by_far(turn, mode):
    ahead, left = (0.49, 0.03) if turn > 0 else (0.39, -0.044)
    result = fly(turn, ahead, left, ReverseArcConfig(mode=mode))
    stationary_cross = math.hypot(ahead, left)   # ~ L * sin(90)
    assert abs(result.cross) < 0.15 * stationary_cross


@pytest.mark.parametrize("mode", [MODE_STRAIGHT, MODE_ARC])
@pytest.mark.parametrize("turn", [90, -90])
def test_forward_only_px4_is_detected_and_falls_back(turn, mode):
    ahead, left = (0.49, 0.03) if turn > 0 else (0.39, -0.044)
    result = fly(turn, ahead, left, ReverseArcConfig(mode=mode),
                 forward_only=True)
    assert result.arc.fallback
    assert result.arc.fallback_reason == "REVERSE_NOT_EXECUTED"
    assert result.t < 2.0


# ---------------------------------------------------- straight mode (default)
def test_default_mode_is_straight():
    assert ReverseArcConfig().mode == MODE_STRAIGHT
    with pytest.raises(ValueError):
        ReverseArcConfig(mode="sideways")


@pytest.mark.parametrize("turn", [45, 90, 135, -45, -90, -135])
def test_straight_mode_never_drives_one_side_only(turn):
    # Field 2026-09-25: the arc's turn radius crosses half the track, so one
    # side is commanded ~0 and is dragged. Straight mode reverses with both
    # sides equal, then pivots with both sides equal and opposite.
    for ahead, left in (LEFT_TRUE if turn > 0 else RIGHT_TRUE):
        result = fly(turn, ahead, left)
        assert result.released
        assert one_sided_fraction(result.wheels) < 0.05, (turn, ahead, left)


def test_arc_mode_is_one_sided_for_a_right_angle_turn():
    # The measured failure, kept as a regression anchor for the arc mode.
    result = fly(90, 0.49, 0.03, ARC)
    assert one_sided_fraction(result.wheels) > 0.3


@pytest.mark.parametrize("turn", [25, 45, 90, 135, 150, -25, -45, -90, -135, -150])
def test_straight_mode_ends_within_40mm_across_centre_spread(turn):
    for ahead, left in (LEFT_TRUE if turn > 0 else RIGHT_TRUE):
        result = fly(turn, ahead, left)
        assert result.released, (turn, ahead, left)
        assert not result.arc.fallback
        # Error left is the calibrated-vs-true centre mismatch only.
        assert abs(result.cross) <= 0.040 + 1.1 * abs(
            (0.49 if turn > 0 else 0.39) - ahead), (turn, ahead, left, result.cross)


def test_straight_reverse_is_about_the_centre_distance():
    result = fly(90, 0.49, 0.03)
    assert result.arc.done_reason == "REVERSE_DONE"
    assert 0.40 <= result.arc.reverse_travel_m <= 0.56


def test_straight_mode_holds_heading_and_starts_slow():
    cfg = ReverseArcConfig()
    arc = ReverseArcPivot(cfg)
    arc.start(0.0, 0.2, 0.2 + math.radians(90), 0.0, 0.0, 0.2 + math.radians(90))
    command = arc.step(0.0, 0.0, 0.0, 0.2, 0.0)
    assert command.active and command.reason == "REVERSING"
    assert command.yaw_enu_rad == pytest.approx(0.2)
    assert command.yaw_rate_radps == 0.0
    assert -cfg.straight_min_speed_mps - 1e-9 <= command.speed_mps < 0.0


def test_straight_mode_skips_reverse_for_a_near_u_turn():
    arc = ReverseArcPivot(ReverseArcConfig())
    arc.start(0.0, 0.0, math.radians(175), 0.0, 0.0, math.radians(175))
    command = arc.step(0.0, 0.0, 0.0, 0.0, 0.0)
    assert (command.active, command.fallback) == (False, False)
    assert command.reason == "TURN_OUT_OF_RANGE"
    assert arc.done


def test_straight_mode_is_done_when_already_on_target():
    # Nozzle already 0.49 m behind the corner: the pivot alone lands it.
    arc = ReverseArcPivot(ReverseArcConfig(centre_left_left_turn_m=0.0))
    arc.start(0.0, 0.0, math.radians(90), 0.0, 0.0, math.radians(90))
    command = arc.step(0.0, -0.49, 0.0, 0.0, 0.0)
    assert command.reason == "REVERSE_DONE" and arc.done


# ------------------------------------------------------------- node wiring
def _class(name="RPPController"):
    return next(n for n in ast.parse(RPP.read_text()).body
                if isinstance(n, ast.ClassDef) and n.name == name)


def _method(name):
    return next(n for n in _class().body
                if isinstance(n, ast.FunctionDef) and n.name == name)


def _declared_defaults():
    init = _method("__init__")
    out = {}
    for call in ast.walk(init):
        if (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                and call.func.attr == "declare_parameter"
                and isinstance(call.args[0], ast.Constant)
                and str(call.args[0].value).startswith("reverse_arc_")):
            out[call.args[0].value] = ast.literal_eval(call.args[1])
    return out


def _launch_values():
    values = {}
    for d in ast.walk(ast.parse(LAUNCH.read_text())):
        if isinstance(d, ast.Dict):
            for k, v in zip(d.keys, d.values):
                if isinstance(k, ast.Constant) and str(k.value).startswith("reverse_arc_"):
                    values[k.value] = ast.literal_eval(v)
    return values


def test_feature_is_off_in_node_and_launch_and_launch_lists_every_parameter():
    # 2026-09-26: disabled again in rover.launch.py (operator decision); the
    # rover flies the ordinary stationary pivot. The C->P1 entry stays off.
    defaults = _declared_defaults()
    launch = _launch_values()
    assert defaults["reverse_arc_pivot_enabled"] is False
    assert launch["reverse_arc_pivot_enabled"] is False
    assert launch["reverse_arc_first_approach_enabled"] is False
    assert set(launch) == set(defaults)
    for name, value in defaults.items():
        assert launch[name] == value, name


def test_node_defaults_build_a_valid_config():
    d = _declared_defaults()
    ReverseArcConfig(
        centre_ahead_left_turn_m=d["reverse_arc_centre_ahead_left_turn_m"],
        centre_left_left_turn_m=d["reverse_arc_centre_left_left_turn_m"],
        centre_ahead_right_turn_m=d["reverse_arc_centre_ahead_right_turn_m"],
        centre_left_right_turn_m=d["reverse_arc_centre_left_right_turn_m"],
        peak_yaw_rate_radps=math.radians(d["reverse_arc_peak_yaw_rate_degps"]),
        min_duration_sec=d["reverse_arc_min_duration_sec"],
        max_speed_mps=d["reverse_arc_max_speed_mps"],
        freeze_remaining_rad=math.radians(d["reverse_arc_freeze_remaining_deg"]),
        front_weight=d["reverse_arc_front_weight"],
        min_turn_rad=math.radians(d["reverse_arc_min_turn_deg"]),
        max_reverse_travel_m=d["reverse_arc_max_reverse_travel_m"],
        timeout_factor=d["reverse_arc_timeout_factor"],
        mode=d["reverse_arc_mode"],
        straight_accel_mps2=d["reverse_arc_straight_accel_mps2"],
        straight_min_speed_mps=d["reverse_arc_straight_min_speed_mps"],
        straight_done_tol_m=d["reverse_arc_straight_done_tol_m"],
    )
    assert d["reverse_arc_mode"] == MODE_STRAIGHT


def test_native_carrier_path_asks_reverse_arc_first():
    source = ast.unparse(_method("_run_legacy_segment_alignment"))
    owner = source.index("self._reverse_arc_owns_pivot(")
    arc = source.index("self._publish_reverse_arc_pivot(")
    legacy = source.index("self._publish_legacy_native_carrier(")
    assert owner < arc < legacy


def test_reverse_arc_resets_with_the_pivot_keeper():
    source = ast.unparse(_method("reset_terminal_native_pivot"))
    assert "reverse_arc.reset()" in source


def test_bag_recorder_captures_reverse_arc_topic():
    assert '"/rpp/reverse_arc"' in BAG.read_text()


# --------------------------- adapter behaviour through the real node methods
class _Publisher:
    def __init__(self):
        self.calls = []

    def publish_with_yaw(self, message, yaw, rate):
        self.calls.append(("moving", message.vector.x, message.vector.y, yaw, rate))
        return True

    def publish_zero_with_yaw(self, message, yaw, rate):
        self.calls.append(("zero", message.vector.x, message.vector.y, yaw, rate))
        return True


class _Debug:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(json.loads(message.data))


def _node(enabled=True, explicit=True, cfg=ARC):
    env = {"math": math, "json": json}

    class Vector:
        def __init__(self):
            self.header = NS(stamp=None, frame_id="")
            self.vector = NS(x=0.0, y=0.0, z=0.0)

    class String:
        data = ""

    env["Vector3Stamped"] = Vector
    env["String"] = String
    names = ("_reverse_arc_owns_pivot", "_publish_reverse_arc_pivot",
             "_publish_reverse_arc_debug")
    module = ast.Module(body=[_method(n) for n in names], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), "<rpp>", "exec"), env)
    clock = NS(t=0.0)
    node = NS(
        reverse_arc_enabled=enabled,
        rpp_explicit_yaw_enabled=explicit,
        reverse_arc_first_approach_enabled=False,
        reverse_arc=ReverseArcPivot(cfg),
        terminal_native_pivot_true_bearing=math.radians(90),
        current_x=0.0, current_y=0.0, current_yaw=0.0,
        current_yaw_rate_radps=0.0,
        velocity_pub=_Publisher(),
        reverse_arc_debug_pub=_Debug(),
        legacy_calls=[], logs=[],
        command_slew_speed=0.5, command_slew_last_time=1,
        explicit_yaw_slew_value=0.3, explicit_yaw_slew_last_time=1,
    )
    node._precision_now_sec = lambda: clock.t
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: None))
    node.get_logger = lambda: NS(warn=node.logs.append, error=node.logs.append)
    node.errors = []
    node.reset_speed_profiles = lambda: None
    node.publish_motion_profile_monitor = lambda speed: None
    node.log_control = lambda *a: None
    node.ground_xtrack = lambda x: x
    node._publish_legacy_native_carrier = lambda *a: node.legacy_calls.append(a)
    for n in names:
        setattr(node, n, env[n].__get__(node))
    return node, clock


def test_adapter_is_inert_when_disabled_or_not_explicit_yaw():
    for enabled, explicit in ((False, True), (True, False)):
        node, _ = _node(enabled, explicit)
        assert node._reverse_arc_owns_pivot(False, math.radians(90), 0.0, 0.0) is False
        assert not node.reverse_arc.active


def test_adapter_skips_first_approach_by_default():
    node, _ = _node()
    assert node._reverse_arc_owns_pivot(True, math.radians(90), 0.0, 0.0) is False


def test_adapter_publishes_reverse_as_velocity_opposite_the_yaw():
    node, clock = _node()
    assert node._reverse_arc_owns_pivot(False, math.radians(90), 0.0, 0.0)
    node._publish_reverse_arc_pivot(1.57, 1.57, 0.0, "", 1.0, 1.0)
    assert node.velocity_pub.calls[-1][0] == "zero"      # not rotating yet
    clock.t = 1.0
    node.current_yaw = math.radians(20)
    node.current_yaw_rate_radps = math.radians(40)
    node._publish_reverse_arc_pivot(1.57, 1.57, 0.0, "", 1.0, 1.0)
    kind, north, east, yaw, rate = node.velocity_pub.calls[-1]
    assert kind == "moving"
    along = east * math.cos(yaw) + north * math.sin(yaw)  # ENU: x=east, y=north
    assert along < 0.0
    assert math.hypot(north, east) == pytest.approx(abs(along))
    assert rate > 0.0
    assert node.explicit_yaw_slew_value is None
    assert node.command_slew_speed == 0.0
    assert node.reverse_arc_debug_pub.messages[-1]["active"] is True
    assert node.legacy_calls == []


def test_adapter_falls_back_to_stationary_pivot_on_abort():
    node, clock = _node()
    node._reverse_arc_owns_pivot(False, math.radians(90), 0.0, 0.0)
    clock.t = 100.0                                       # past the timeout
    node._publish_reverse_arc_pivot(1.57, 1.57, 0.0, "", 1.0, 1.0)
    assert len(node.legacy_calls) == 1
    assert node.reverse_arc_debug_pub.messages[-1]["fallback"] is True
    # The fallback latch holds for the rest of this pivot.
    assert node._reverse_arc_owns_pivot(False, math.radians(90), 0.0, 0.0) is False


def test_adapter_straight_reverse_then_hands_over_to_stationary_pivot():
    node, clock = _node(cfg=ReverseArcConfig())
    assert node._reverse_arc_owns_pivot(False, math.radians(90), 0.0, 0.0)
    node._publish_reverse_arc_pivot(1.57, 1.57, 0.0, "", 1.0, 1.0)
    kind, north, east, yaw, rate = node.velocity_pub.calls[-1]
    assert kind == "moving"
    assert east < 0.0 and north == pytest.approx(0.0, abs=1e-12)  # straight back
    assert rate == 0.0
    # Rover has reversed to the solved point: hand over to the plain pivot.
    clock.t = 2.0
    node.current_x = -0.52        # 0.49 ahead + 0.03 left offset, left turn
    node._publish_reverse_arc_pivot(1.57, 1.57, 0.0, "", 1.0, 1.0)
    assert len(node.legacy_calls) == 1
    assert node.legacy_calls[-1][-1] == "PX4 PIVOT KEEPER / NATIVE TURN AFTER REVERSE"
    assert node.reverse_arc_debug_pub.messages[-1]["done"] is True
    assert node.reverse_arc_debug_pub.messages[-1]["mode"] == MODE_STRAIGHT
    # And it never restarts within the same pivot.
    assert node._reverse_arc_owns_pivot(False, math.radians(90), 0.0, 0.0) is False
