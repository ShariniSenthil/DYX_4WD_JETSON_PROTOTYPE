"""Focused guards for the production native-pivot settle/recapture lifecycle."""

from __future__ import annotations

import ast
import hashlib
import math
from pathlib import Path

from rpp_controller.legacy_alignment import (
    LegacyAlignmentConfig,
    LegacyAlignmentDirective,
    LegacyAlignmentInput,
    LegacyAlignmentLifecycle,
    LegacyAlignmentPhase,
    compute_low_energy_realign_command,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
NODE_PATH = PACKAGE_ROOT / "rpp_controller" / "rpp_controller_node.py"
LAUNCH_PATH = REPOSITORY_ROOT / "src" / "rover_bringup" / "launch" / "rover.launch.py"
NODE_SOURCE = NODE_PATH.read_text(encoding="utf-8")
LAUNCH_SOURCE = LAUNCH_PATH.read_text(encoding="utf-8")
NODE_TREE = ast.parse(NODE_SOURCE)

DEG2 = math.radians(2.0)
DEG4 = math.radians(4.0)
DEG15 = math.radians(15.0)
DEG30 = math.radians(30.0)
YAW_30DPS = math.radians(30.0)
YAW_98DPS = math.radians(98.0)
NATIVE_HASH = "89de9ecbc275c72378c428de25e4aa19b9e72af531a65271949ce28d1cb0790f"


def _controller_method(name: str) -> ast.FunctionDef:
    controller = next(
        item
        for item in NODE_TREE.body
        if isinstance(item, ast.ClassDef) and item.name == "RPPController"
    )
    return next(
        item
        for item in controller.body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )


def _method_source(name: str) -> str:
    source = ast.get_source_segment(NODE_SOURCE, _controller_method(name))
    assert source is not None
    return source


def _declared_defaults() -> dict[str, object]:
    declarations = {}
    for node in ast.walk(_controller_method("__init__")):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "declare_parameter"
            and len(node.args) >= 2
            and isinstance(node.args[0], ast.Constant)
        ):
            declarations[str(node.args[0].value)] = ast.literal_eval(node.args[1])
    return declarations


def make_config(**overrides) -> LegacyAlignmentConfig:
    values = {
        "native_release_heading_rad": DEG4,
        "tight_heading_rad": DEG2,
        "stop_speed_mps": 0.010,
        "stop_yaw_rate_radps": 0.050,
        "settle_sec": 0.20,
        "post_settle_hold_sec": 1.00,
        "recapture_xtrack_m": 0.020,
        "recapture_heading_rad": DEG2,
        "recapture_settle_sec": 0.20,
        "non_pivot_release_xtrack_m": 0.008,
        "non_pivot_release_heading_rad": DEG4,
        "non_pivot_hold_sec": 0.20,
        "fast_capture_max_cross_track_m": 0.050,
        "pivot_enter_rad": math.radians(45.0),
        "pivot_keeper_timeout_sec": 10.0,
        "pre_pivot_timeout_sec": 8.0,
        "realign_grace_sec": 0.30,
        "realign_split_heading_rad": DEG15,
        "realign_near_speed_mps": 0.20,
        "realign_far_speed_mps": 0.12,
        "realign_bearing_cone_rad": DEG30,
        "realign_max_translation_m": 0.30,
        "realign_timeout_sec": 9.0,
    }
    values.update(overrides)
    return LegacyAlignmentConfig(**values)


class Driver:
    def __init__(self, heading_deg=90.0, **cfg):
        self.life = LegacyAlignmentLifecycle(make_config(**cfg))
        self.now = 0.0
        self.dt = 0.05
        self.heading_deg = heading_deg
        self.speed = 0.30
        self.yaw_rate = YAW_98DPS
        self.xtrack = 0.040
        self.fresh = True
        self.first_approach = True
        self.already_reanchored = False
        self.current_x = 0.0
        self.current_y = 0.0
        self._started = False

    def tick(self, **overrides):
        ack = bool(overrides.pop("ack", True))
        if "now" in overrides:
            self.now = float(overrides.pop("now"))
        else:
            step = overrides.pop("elapsed", None)
            if step is None:
                step = 0.0 if not self._started else self.dt
            self.now += float(step)
        self._started = True
        self.heading_deg = float(overrides.pop("heading_deg", self.heading_deg))
        self.speed = float(overrides.pop("speed", self.speed))
        self.yaw_rate = float(overrides.pop("yaw_rate", self.yaw_rate))
        self.xtrack = float(overrides.pop("xtrack", self.xtrack))
        self.fresh = bool(overrides.pop("fresh", self.fresh))
        self.first_approach = bool(
            overrides.pop("first_approach", self.first_approach)
        )
        self.already_reanchored = bool(
            overrides.pop("already_reanchored", self.already_reanchored)
        )
        if "current_x" in overrides:
            self.current_x = overrides.pop("current_x")
        if "current_y" in overrides:
            self.current_y = overrides.pop("current_y")
        heading = math.radians(self.heading_deg)
        native_active = False
        if self.life.needs_native_command:
            native_active = abs(heading) > DEG4
        result = self.life.step(
            LegacyAlignmentInput(
                now_sec=self.now,
                telemetry_fresh=self.fresh,
                measured_speed_mps=self.speed,
                measured_yaw_rate_radps=self.yaw_rate,
                path_heading_error_rad=heading,
                alignment_cross_track_m=self.xtrack,
                native_pivot_active=native_active,
                first_approach=self.first_approach,
                already_reanchored=self.already_reanchored,
                current_x=self.current_x,
                current_y=self.current_y,
            )
        )
        if result.directive is LegacyAlignmentDirective.NATIVE_CARRIER and ack:
            self.life.ack_native_carrier_published()
        if (
            self.life.native_carrier_issued
            and self.life.phase is LegacyAlignmentPhase.NON_PIVOT_CAPTURE
        ):
            raise AssertionError(
                "native_carrier_issued cannot coexist with NON_PIVOT_CAPTURE"
            )
        return result


def enter_pre(heading_deg=90.0):
    driver = Driver(heading_deg=heading_deg)
    result = driver.tick()
    assert result.phase is LegacyAlignmentPhase.PRE_PIVOT_STOP
    assert result.directive is LegacyAlignmentDirective.HOLD_ZERO
    assert result.native_carrier_issued is False
    return driver


def complete_pre_zero_transition(driver, heading_deg=90.0):
    driver.tick(heading_deg=heading_deg, speed=0.0, yaw_rate=0.0)
    result = driver.tick(
        elapsed=0.21,
        heading_deg=heading_deg,
        speed=0.0,
        yaw_rate=0.0,
    )
    assert result.phase is LegacyAlignmentPhase.NATIVE_PIVOT
    assert result.directive is LegacyAlignmentDirective.HOLD_ZERO
    assert result.native_carrier_issued is False
    return result


def publish_native(driver, heading_deg=90.0):
    result = driver.tick(heading_deg=heading_deg, speed=0.0, yaw_rate=0.20)
    assert result.directive is LegacyAlignmentDirective.NATIVE_CARRIER
    assert driver.life.native_carrier_issued is True
    return result


def release_to_settle(driver, heading_deg=3.5):
    result = driver.tick(
        heading_deg=heading_deg,
        speed=0.20,
        yaw_rate=YAW_98DPS,
    )
    assert result.phase is LegacyAlignmentPhase.PIVOT_SETTLE
    assert result.directive is LegacyAlignmentDirective.HOLD_ZERO
    assert not result.reanchor_requested
    return result


def finish_certificate_and_hold(driver, heading_deg=1.0):
    driver.tick(heading_deg=heading_deg, speed=0.0, yaw_rate=0.0)
    result = driver.tick(
        elapsed=0.21,
        heading_deg=heading_deg,
        speed=0.0,
        yaw_rate=0.0,
    )
    assert result.transition_reason == "POST_SETTLE_HOLD"
    return driver.tick(
        elapsed=1.01,
        heading_deg=heading_deg,
        speed=0.0,
        yaw_rate=0.0,
    )


def test_aligned_start_under_4deg_is_non_pivot():
    driver = Driver(heading_deg=3.0)
    driver.speed = 0.0
    driver.yaw_rate = 0.0
    driver.xtrack = 0.004
    first = driver.tick()
    assert first.phase is LegacyAlignmentPhase.NON_PIVOT_CAPTURE
    assert first.directive is LegacyAlignmentDirective.NON_PIVOT_CAPTURE
    assert first.native_carrier_issued is False
    done = driver.tick(elapsed=0.21, heading_deg=2.0, xtrack=0.004)
    assert done.directive is LegacyAlignmentDirective.COMPLETE_FALLTHROUGH


def test_heading_family_classification_unchanged():
    for heading in (5.0, 15.0, 30.0, 44.0, 45.0, 90.0, -90.0, 180.0, -180.0):
        result = Driver(heading_deg=heading).tick()
        assert result.phase is LegacyAlignmentPhase.PRE_PIVOT_STOP
        assert result.directive is LegacyAlignmentDirective.HOLD_ZERO
        assert result.directive is not LegacyAlignmentDirective.NATIVE_CARRIER
    aligned = Driver(heading_deg=4.0).tick()
    assert aligned.phase is LegacyAlignmentPhase.NON_PIVOT_CAPTURE


def test_moving_over_4deg_goes_pre_never_carrier():
    result = Driver(heading_deg=90.0).tick(speed=0.40, yaw_rate=YAW_98DPS)
    assert result.phase is LegacyAlignmentPhase.PRE_PIVOT_STOP
    assert result.directive is LegacyAlignmentDirective.HOLD_ZERO
    assert result.native_carrier_issued is False


def test_stationary_entry_still_waits_0_20s():
    driver = enter_pre()
    driver.tick(heading_deg=90.0, speed=0.0, yaw_rate=0.0)
    early = driver.tick(heading_deg=90.0, speed=0.0, yaw_rate=0.0, elapsed=0.19)
    assert early.phase is LegacyAlignmentPhase.PRE_PIVOT_STOP
    assert early.directive is LegacyAlignmentDirective.HOLD_ZERO
    done = driver.tick(heading_deg=90.0, speed=0.0, yaw_rate=0.0, elapsed=0.02)
    assert done.phase is LegacyAlignmentPhase.NATIVE_PIVOT
    assert done.directive is LegacyAlignmentDirective.HOLD_ZERO


def test_pre_speed_yaw_freshness_resets():
    driver = enter_pre()
    driver.tick(heading_deg=90.0, speed=0.0, yaw_rate=0.0)
    driver.tick(heading_deg=90.0, speed=0.05, yaw_rate=0.0)
    assert driver.life.pre_stop_inside_since is None
    driver.tick(heading_deg=90.0, speed=0.0, yaw_rate=0.0)
    driver.tick(heading_deg=90.0, speed=0.0, yaw_rate=0.20)
    assert driver.life.pre_stop_inside_since is None
    driver.tick(heading_deg=90.0, speed=0.0, yaw_rate=0.0)
    stale = driver.tick(heading_deg=90.0, speed=0.0, yaw_rate=0.0, fresh=False)
    assert stale.directive is LegacyAlignmentDirective.HOLD_ZERO
    assert driver.life.pre_stop_inside_since is None
    assert driver.life.phase is LegacyAlignmentPhase.PRE_PIVOT_STOP


def test_pre_799_continues_and_80_times_out():
    driver = enter_pre()
    start = driver.life.pre_started_at
    still = driver.tick(
        now=start + 7.99,
        heading_deg=90.0,
        speed=0.30,
        yaw_rate=YAW_30DPS,
    )
    assert still.phase is LegacyAlignmentPhase.PRE_PIVOT_STOP
    assert still.directive is LegacyAlignmentDirective.HOLD_ZERO
    timed = driver.tick(
        now=start + 8.00,
        heading_deg=90.0,
        speed=0.30,
        yaw_rate=YAW_30DPS,
    )
    assert timed.phase is LegacyAlignmentPhase.SAFETY_HOLD
    assert timed.directive is LegacyAlignmentDirective.SAFETY_HOLD
    assert timed.reset_native_carrier is True
    assert timed.native_carrier_issued is False


def test_pre_cancel_to_non_pivot_has_no_native_identity():
    driver = enter_pre()
    result = driver.tick(heading_deg=3.0, speed=0.0, yaw_rate=0.0)
    assert result.phase is LegacyAlignmentPhase.NON_PIVOT_CAPTURE
    assert result.directive is LegacyAlignmentDirective.HOLD_ZERO
    assert result.reset_native_carrier is True
    assert result.native_carrier_issued is False
    assert not result.reanchor_requested


def test_pre_completion_remains_zero_and_carrier_starts_next_cycle():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    first_native = publish_native(driver)
    assert first_native.previous_phase is LegacyAlignmentPhase.NATIVE_PIVOT
    assert first_native.directive is LegacyAlignmentDirective.NATIVE_CARRIER


def test_classification_without_ack_cannot_authorize_c_prime():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    result = driver.tick(heading_deg=3.0, speed=0.0, yaw_rate=0.0, ack=False)
    assert result.phase is LegacyAlignmentPhase.NON_PIVOT_CAPTURE
    assert result.directive is LegacyAlignmentDirective.HOLD_ZERO
    assert result.native_carrier_issued is False
    assert not result.reanchor_requested


def test_successful_publication_ack_can_authorize_c_prime():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=1.0)
    result = finish_certificate_and_hold(driver, heading_deg=1.0)
    assert result.reanchor_requested is True
    assert result.directive is LegacyAlignmentDirective.REANCHOR_ZERO
    assert driver.life.reanchor_complete is False
    driver.life.ack_reanchor_completed()
    second = driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0, xtrack=0.040)
    assert second.reanchor_requested is False
    assert driver.life.reanchor_complete is True


def test_publication_exception_cannot_ack():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    result = driver.tick(heading_deg=90.0, speed=0.0, yaw_rate=0.20, ack=False)
    assert result.directive is LegacyAlignmentDirective.NATIVE_CARRIER
    assert driver.life.native_carrier_issued is False
    adapter = _method_source("_run_legacy_segment_alignment")
    publish = adapter.index("self._publish_legacy_native_carrier(")
    ack = adapter.index("ack_native_carrier_published")
    failed = adapter.index("NATIVE CARRIER PUBLISH FAILED")
    assert publish < failed < ack


def test_native_release_with_30_to_98_dps_is_literal_zero_only():
    for yaw_rate in (YAW_30DPS, YAW_98DPS):
        driver = enter_pre()
        complete_pre_zero_transition(driver)
        publish_native(driver)
        result = driver.tick(
            heading_deg=3.0,
            speed=0.15,
            yaw_rate=yaw_rate,
        )
        assert result.phase is LegacyAlignmentPhase.PIVOT_SETTLE
        assert result.directive is LegacyAlignmentDirective.HOLD_ZERO
        assert not result.reanchor_requested


def test_native_over_10s_warns_once_and_continues():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    start = driver.life.keeper_started_at
    first = driver.tick(
        now=start + 10.01,
        heading_deg=90.0,
        speed=0.0,
        yaw_rate=0.20,
    )
    assert first.directive is LegacyAlignmentDirective.NATIVE_CARRIER
    assert first.warn_native_timeout is True
    assert first.phase is not LegacyAlignmentPhase.SAFETY_HOLD
    second = driver.tick(
        now=start + 10.20,
        heading_deg=90.0,
        speed=0.0,
        yaw_rate=0.20,
    )
    assert second.directive is LegacyAlignmentDirective.NATIVE_CARRIER
    assert second.warn_native_timeout is False


def test_realign_grace_is_0_30s():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=3.5)
    spinning = driver.tick(heading_deg=3.5, speed=0.10, yaw_rate=YAW_30DPS)
    assert spinning.phase is LegacyAlignmentPhase.PIVOT_SETTLE
    first = driver.tick(heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    assert first.transition_reason == "REALIGN_GRACE"
    early = driver.tick(elapsed=0.29, heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    assert early.phase is LegacyAlignmentPhase.PIVOT_SETTLE
    done = driver.tick(elapsed=0.02, heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    assert done.phase is LegacyAlignmentPhase.LOW_ENERGY_REALIGN
    assert done.directive is LegacyAlignmentDirective.HOLD_ZERO


def test_low_energy_near_band_is_0_20_and_far_band_is_0_12():
    near = compute_low_energy_realign_command(
        math.radians(10.0),
        0.0,
        split_heading_rad=DEG15,
        near_speed_mps=0.20,
        far_speed_mps=0.12,
        bearing_cone_rad=DEG30,
    )
    far = compute_low_energy_realign_command(
        math.radians(20.0),
        0.0,
        split_heading_rad=DEG15,
        near_speed_mps=0.20,
        far_speed_mps=0.12,
        bearing_cone_rad=DEG30,
    )
    assert near[1] == 0.20
    assert far[1] == 0.12
    assert math.isclose(math.hypot(near[2], near[3]), 0.20, abs_tol=1.0e-12)
    assert math.isclose(math.hypot(far[2], far[3]), 0.12, abs_tol=1.0e-12)


def test_low_energy_bearing_cone_is_plus_minus_30deg():
    bearing, speed, north, east = compute_low_energy_realign_command(
        math.radians(44.0),
        0.0,
        split_heading_rad=DEG15,
        near_speed_mps=0.20,
        far_speed_mps=0.12,
        bearing_cone_rad=DEG30,
    )
    command_error = abs(math.atan2(math.sin(bearing), math.cos(bearing)))
    assert command_error <= DEG30 + 1.0e-12
    assert speed == 0.12
    del north, east


def test_low_energy_adapter_disables_accel_decel_and_hard_caps():
    source = _method_source("_publish_legacy_low_energy_realign")
    assert "apply_acceleration=False" in source
    assert "apply_deceleration=False" in source
    assert "hard_speed_cap_mps=speed" in source
    assert "precision_pivot_carrier_command" not in source
    assert "terminal_native_pivot_command" not in source
    assert "segment_alignment_recovery_speed" not in source


def test_low_energy_displacement_and_timeout_and_nonfinite_are_local_safety():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=3.5)
    driver.tick(heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    driver.tick(elapsed=0.31, heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    assert driver.life.phase is LegacyAlignmentPhase.LOW_ENERGY_REALIGN
    moved = driver.tick(
        heading_deg=3.5,
        speed=0.0,
        yaw_rate=0.0,
        current_x=0.30,
        current_y=0.0,
    )
    assert moved.phase is LegacyAlignmentPhase.SAFETY_HOLD

    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=3.5)
    driver.tick(heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    driver.tick(elapsed=0.31, heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    start = driver.life.realign_started_at
    timed = driver.tick(
        now=start + 9.0,
        heading_deg=3.5,
        speed=0.0,
        yaw_rate=0.0,
    )
    assert timed.phase is LegacyAlignmentPhase.SAFETY_HOLD

    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=3.5)
    driver.tick(heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    driver.tick(elapsed=0.31, heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    bad = driver.tick(
        heading_deg=3.5,
        speed=0.0,
        yaw_rate=0.0,
        current_x=math.inf,
    )
    assert bad.phase is LegacyAlignmentPhase.SAFETY_HOLD


def test_stale_realign_does_not_refresh_watchdog_budget():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=3.5)
    driver.tick(heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    driver.tick(elapsed=0.31, heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    start = driver.life.realign_started_at
    stale = driver.tick(
        now=start + 5.0,
        heading_deg=3.5,
        speed=0.0,
        yaw_rate=0.0,
        fresh=False,
    )
    assert stale.phase is LegacyAlignmentPhase.LOW_ENERGY_REALIGN
    assert stale.directive is LegacyAlignmentDirective.HOLD_ZERO
    assert driver.life.realign_started_at == start
    late = driver.tick(
        now=start + 9.0,
        heading_deg=3.5,
        speed=0.0,
        yaw_rate=0.0,
        fresh=True,
    )
    assert late.phase is LegacyAlignmentPhase.SAFETY_HOLD


def test_600mm_xtrack_does_not_stop_lifecycle():
    control = _method_source("control_loop")
    assert "SEGMENT ALIGNMENT LARGE CROSS-TRACK / CONTINUING RECOVERY" in control
    assert "alignment_safety_hold" not in control
    block = control[
        control.index("segment_alignment_max_cross_track") : control.index(
            "if self.precision_pivot_enabled:"
        )
    ]
    assert "self.publish_stop()" not in block
    assert "return" not in block.split("if self.precision_pivot_enabled:")[0]


def test_no_alignment_safety_hold_symbol_in_production_source():
    assert "alignment_safety_hold" not in NODE_SOURCE
    assert "PIVOT_REALIGN" not in NODE_SOURCE
    assert "PIVOT_REALIGN" not in Path(
        PACKAGE_ROOT / "rpp_controller" / "legacy_alignment.py"
    ).read_text(encoding="utf-8")


def test_two_deg_realign_exit_reacquires_full_settle():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=3.5)
    driver.tick(heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    driver.tick(elapsed=0.31, heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    driver.tick(heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    back = driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0)
    assert back.phase is LegacyAlignmentPhase.PIVOT_SETTLE
    assert back.directive is LegacyAlignmentDirective.HOLD_ZERO
    assert driver.life.settle_inside_since is None


def test_ge45_realign_escalates_through_pre():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=3.5)
    driver.tick(heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    driver.tick(elapsed=0.31, heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    result = driver.tick(heading_deg=50.0, speed=0.0, yaw_rate=0.0)
    assert result.phase is LegacyAlignmentPhase.PRE_PIVOT_STOP
    assert result.directive is LegacyAlignmentDirective.HOLD_ZERO


def test_low_energy_alone_cannot_reanchor():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=3.5)
    driver.tick(heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    result = driver.tick(elapsed=0.31, heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    assert result.phase is LegacyAlignmentPhase.LOW_ENERGY_REALIGN
    assert result.reanchor_requested is False
    moving = driver.tick(heading_deg=3.5, speed=0.0, yaw_rate=0.0)
    assert moving.reanchor_requested is False
    assert moving.directive is LegacyAlignmentDirective.LOW_ENERGY_REALIGN


def test_actual_native_settle_hold_reanchors_once_and_later_geometry_unchanged():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=1.0)
    first = finish_certificate_and_hold(driver, heading_deg=1.0)
    assert first.reanchor_requested is True
    assert driver.life.reanchor_complete is False
    driver.life.ack_reanchor_completed()
    assert driver.life.reanchor_complete is True
    second = driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0, xtrack=0.010)
    assert second.reanchor_requested is False
    assert second.phase is LegacyAlignmentPhase.POST_PIVOT_RECAPTURE


def test_recapture_remains_0_20_and_zero_is_next_cycle_boundary():
    adapter = _method_source("_run_legacy_segment_alignment")
    recapture = adapter[
        adapter.index("LegacyAlignmentDirective.RECAPTURE") : adapter.index(
            "COMPLETE_ZERO"
        )
    ]
    assert "speed = self.post_pivot_capture_speed" in recapture
    assert "hard_speed_cap_mps=speed" in recapture
    assert "apply_acceleration=False" in recapture
    assert "apply_deceleration=False" in recapture
    assert "segment_alignment_recovery_speed" not in recapture
    completion = adapter[adapter.index("COMPLETE_ZERO") :]
    assert "self.reset_speed_profiles()" in completion
    assert "self.command_slew_speed = 0.0" in completion
    assert "self.command_slew_last_time = None" in completion
    assert "self.publish_stop()" in completion
    assert "return True" in completion


def test_recapture_cannot_complete_before_geometry_dwell_or_use_50mm_fallback():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=1.0)
    requested = finish_certificate_and_hold(driver, heading_deg=1.0)
    if requested.reanchor_requested:
        driver.life.ack_reanchor_completed()
    wide = driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0, xtrack=0.080)
    assert wide.directive is LegacyAlignmentDirective.RECAPTURE
    assert wide.directive is not LegacyAlignmentDirective.FALLBACK_GLOBAL_XTRACK
    early = driver.tick(
        heading_deg=1.0,
        speed=0.0,
        yaw_rate=0.0,
        xtrack=0.010,
        elapsed=0.19,
    )
    assert early.directive is LegacyAlignmentDirective.RECAPTURE
    done = driver.tick(
        heading_deg=1.0,
        speed=0.0,
        yaw_rate=0.0,
        xtrack=0.010,
        elapsed=0.21,
    )
    assert done.directive is LegacyAlignmentDirective.COMPLETE_ZERO


def test_legacy_param_and_precision_gate_remain_field_safe():
    defaults = _declared_defaults()
    assert defaults["precision_pivot_enabled"] is False
    assert defaults["legacy_pivot_post_settle_hold_sec"] == 1.00
    assert defaults["legacy_pivot_realign_grace_sec"] == 0.30
    assert defaults["legacy_pivot_realign_split_heading_deg"] == 15.0
    assert defaults["legacy_pivot_realign_near_speed_mps"] == 0.20
    assert defaults["legacy_pivot_realign_far_speed_mps"] == 0.12
    assert defaults["legacy_pivot_realign_bearing_cone_deg"] == 30.0
    assert defaults["legacy_pivot_realign_max_translation_m"] == 0.30
    assert defaults["legacy_pivot_realign_timeout_sec"] == 9.0
    assert defaults["post_pivot_capture_speed_mps"] == 0.20
    assert defaults["acceleration_distance_m"] == 0.20
    assert defaults["deceleration_distance_m"] == 0.50
    assert defaults["cruise_speed_mps"] == 1.00
    assert defaults["waypoint_tolerance_m"] == 0.03
    assert '"precision_pivot_enabled": False' in LAUNCH_SOURCE
    assert '"legacy_pivot_realign_grace_sec": 0.30' in LAUNCH_SOURCE
    assert "RD_MAX_THR_YAW_R" not in NODE_SOURCE
    assert "RO_YAW_RATE_TH" not in NODE_SOURCE
    assert "RO_YAW_RATE_LIM" not in NODE_SOURCE


def test_native_pivot_method_remains_byte_exact():
    digest = hashlib.sha256(
        _method_source("terminal_native_pivot_command").encode("utf-8")
    ).hexdigest()
    assert digest == NATIVE_HASH


def test_legacy_reset_coverage_is_explicit():
    for reason in (
        "INITIALIZE",
        "SEGMENT_GOAL_CHANGED",
        "MISSION_ENABLED",
        "MOTION_STATE_RESET",
        "EMERGENCY_STOP",
        "MARKING_HOLD",
        "POINT_COMPLETED",
        "GEOMETRY_INVALIDATED",
        "PATH_INSTALLED",
        "LOCALIZATION_JUMP",
        "MID_LEG_ALIGNMENT_REENTRY",
        "TERMINAL_MISS",
        "SHUTDOWN",
        "C_LINE_LOCKED",
    ):
        assert f'"{reason}"' in NODE_SOURCE
    reset = _method_source("_reset_legacy_alignment_lifecycle")
    assert "self.legacy_alignment.reset(reason)" in reset
    assert "self.reset_terminal_native_pivot()" in reset


def test_terminal_stop_contract_is_unchanged():
    latch = _method_source("latch_exact_marking_stop")
    assert "target_distance <= self.waypoint_tolerance" in latch
    defaults = _declared_defaults()
    assert defaults["waypoint_tolerance_m"] == 0.03


def test_control_loop_keeps_precision_pivot_behind_its_own_flag():
    control = _method_source("control_loop")
    precision = control.index("if self.precision_pivot_enabled:")
    legacy = control.index("self._run_legacy_segment_alignment(")
    assert precision < legacy
    assert "self._run_precision_pivot_alignment(" in control[precision:legacy]


def test_legacy_adapter_has_no_old_anchor_recenter_path():
    adapter = _method_source("_run_legacy_segment_alignment")
    assert "MotionDirective.RECENTER" not in adapter
    assert "_publish_precision_anchor_approach" not in adapter
    assert "anchor_radial_error_m" not in adapter
    assert "precision_pivot_carrier_command" not in adapter


def test_native_release_without_carrier_never_reaches_post_pivot_recapture():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    result = driver.tick(heading_deg=3.0, speed=0.0, yaw_rate=0.0, ack=False)
    assert result.phase is LegacyAlignmentPhase.NON_PIVOT_CAPTURE
    assert result.directive is LegacyAlignmentDirective.HOLD_ZERO
    assert result.native_carrier_issued is False
    phases = {result.phase}
    for _ in range(40):
        later = driver.tick(
            heading_deg=1.0,
            speed=0.0,
            yaw_rate=0.0,
            xtrack=0.004,
        )
        phases.add(later.phase)
        assert later.phase is not LegacyAlignmentPhase.POST_PIVOT_RECAPTURE
        assert later.phase is not LegacyAlignmentPhase.PIVOT_SETTLE
    assert LegacyAlignmentPhase.NON_PIVOT_CAPTURE in phases


def test_pre_cancel_after_real_pivot_reacquires_full_settle_hold():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=1.0)
    escalate = driver.tick(heading_deg=50.0, speed=0.0, yaw_rate=0.0)
    assert escalate.phase is LegacyAlignmentPhase.PRE_PIVOT_STOP
    cancel = driver.tick(heading_deg=3.0, speed=0.0, yaw_rate=0.0)
    assert cancel.phase is LegacyAlignmentPhase.PIVOT_SETTLE
    assert cancel.directive is LegacyAlignmentDirective.HOLD_ZERO
    assert driver.life.native_carrier_issued is True
    assert driver.life.settle_inside_since is None
    assert driver.life.post_settle_hold_since is None
    driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0)
    early = driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0, elapsed=0.19)
    assert early.phase is LegacyAlignmentPhase.PIVOT_SETTLE
    cert = driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0, elapsed=0.02)
    assert cert.transition_reason == "POST_SETTLE_HOLD"
    hold = driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0, elapsed=0.99)
    assert hold.phase is LegacyAlignmentPhase.PIVOT_SETTLE
    done = driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0, elapsed=0.02)
    assert done.phase is LegacyAlignmentPhase.POST_PIVOT_RECAPTURE


def test_native_carrier_issued_never_coexists_with_non_pivot_capture():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    assert driver.life.native_carrier_issued is True
    driver.tick(heading_deg=50.0, speed=0.0, yaw_rate=0.0)
    driver.tick(heading_deg=3.0, speed=0.0, yaw_rate=0.0)
    assert driver.life.native_carrier_issued is True
    assert driver.life.phase is not LegacyAlignmentPhase.NON_PIVOT_CAPTURE
    driver.life.reset("SEMANTIC_RESET")
    assert driver.life.native_carrier_issued is False
    assert driver.life.phase is LegacyAlignmentPhase.ENTRY


def test_reanchor_request_does_not_mark_complete_before_ack():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=1.0)
    result = finish_certificate_and_hold(driver, heading_deg=1.0)
    assert result.reanchor_requested is True
    assert driver.life.reanchor_complete is False


def test_successful_reanchor_ack_marks_complete_exactly_once():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=1.0)
    finish_certificate_and_hold(driver, heading_deg=1.0)
    driver.life.ack_reanchor_completed()
    assert driver.life.reanchor_complete is True
    driver.life.ack_reanchor_completed()
    assert driver.life.reanchor_complete is True
    later = driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0, xtrack=0.040)
    assert later.reanchor_requested is False


def test_failed_reanchor_cannot_reach_recapture_and_is_local_safety_hold():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=1.0)
    requested = finish_certificate_and_hold(driver, heading_deg=1.0)
    assert requested.reanchor_requested is True
    assert driver.life.reanchor_complete is False
    driver.life.enter_safety_hold("REANCHOR_FAILED")
    for _ in range(20):
        later = driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0, xtrack=0.004)
        assert later.phase is LegacyAlignmentPhase.SAFETY_HOLD
        assert later.directive is LegacyAlignmentDirective.SAFETY_HOLD
        assert later.directive is not LegacyAlignmentDirective.RECAPTURE
    adapter = _method_source("_run_legacy_segment_alignment")
    fail = adapter.index("REANCHOR_FAILED")
    recapture = adapter.index("LegacyAlignmentDirective.RECAPTURE")
    assert "enter_safety_hold" in adapter
    assert "ack_reanchor_completed" in adapter
    assert adapter.index("ack_reanchor_completed") < fail
    assert fail < recapture


def test_failed_reanchor_later_semantic_geometry_remains_unchanged():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=1.0)
    finish_certificate_and_hold(driver, heading_deg=1.0)
    driver.life.enter_safety_hold("REANCHOR_FAILED")
    driver.life.reset("SEGMENT_GOAL_CHANGED")
    assert driver.life.phase is LegacyAlignmentPhase.ENTRY
    assert driver.life.native_carrier_issued is False
    assert driver.life.reanchor_complete is False
