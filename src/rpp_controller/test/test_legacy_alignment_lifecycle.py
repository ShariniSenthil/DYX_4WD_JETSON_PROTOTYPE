"""Focused guards for the production native-pivot settle/reanchor/hold lifecycle."""

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
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
NODE_PATH = PACKAGE_ROOT / "rpp_controller" / "rpp_controller_node.py"
LAUNCH_PATH = REPOSITORY_ROOT / "src" / "rover_bringup" / "launch" / "rover.launch.py"
NODE_SOURCE = NODE_PATH.read_text(encoding="utf-8")
LAUNCH_SOURCE = LAUNCH_PATH.read_text(encoding="utf-8")
NODE_TREE = ast.parse(NODE_SOURCE)

DEG4 = math.radians(4.0)
YAW_30DPS = math.radians(30.0)
YAW_98DPS = math.radians(98.0)
NATIVE_HASH = "ab1a69086a10d69a3719dea04fdfd772887dfec02ee318020c47e93b3e0cea00"


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
        "stop_speed_mps": 0.010,
        "stop_yaw_rate_radps": 0.050,
        "settle_sec": 0.20,
        "post_settle_hold_sec": 1.00,
        "non_pivot_release_xtrack_m": 0.008,
        "non_pivot_release_heading_rad": DEG4,
        "non_pivot_hold_sec": 0.20,
        "fast_capture_max_cross_track_m": 0.050,
        "pivot_enter_rad": math.radians(45.0),
        "pivot_keeper_timeout_sec": 10.0,
        "pre_pivot_timeout_sec": 8.0,
        "stationary_violation_debounce_sec": 0.10,
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


def reach_settle_certificate(driver, heading_deg=1.0):
    """Drive the stationary settle_sec dwell to completion.

    Returns the tick where the certificate passes: REANCHOR_ZERO if a reanchor
    is owed on this first approach, otherwise the first POST_SETTLE_HOLD tick.
    """
    driver.tick(heading_deg=heading_deg, speed=0.0, yaw_rate=0.0)
    return driver.tick(
        elapsed=0.21,
        heading_deg=heading_deg,
        speed=0.0,
        yaw_rate=0.0,
    )


def finish_settle_and_hold_after_reanchor(driver, heading_deg=1.0):
    """Drive the post-reanchor 1.0s literal-zero hold to COMPLETE_ZERO.

    Assumes reach_settle_certificate has already run and, if it requested a
    reanchor, that reanchor has already been acked.
    """
    result = driver.tick(heading_deg=heading_deg, speed=0.0, yaw_rate=0.0)
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


def test_pre_speed_yaw_brief_violation_does_not_reset():
    """A lone speed/yaw-rate blip inside the debounce window (e.g.
    GPS-antenna lever-arm noise during residual yaw settling) must not
    discard already-earned pre-stop dwell progress."""
    driver = enter_pre()
    driver.tick(heading_deg=90.0, speed=0.0, yaw_rate=0.0)
    started_at = driver.life.pre_stop_inside_since
    assert started_at is not None
    blip = driver.tick(heading_deg=90.0, speed=0.05, yaw_rate=0.0, elapsed=0.05)
    assert blip.transition_reason == "PRE_PIVOT_CERTIFICATE_DWELL"
    assert driver.life.pre_stop_inside_since == started_at
    driver.tick(heading_deg=90.0, speed=0.0, yaw_rate=0.0, elapsed=0.02)
    assert driver.life.violation_started_at is None
    assert driver.life.pre_stop_inside_since == started_at

    yaw_blip = driver.tick(heading_deg=90.0, speed=0.0, yaw_rate=0.20, elapsed=0.05)
    assert yaw_blip.transition_reason == "PRE_PIVOT_CERTIFICATE_DWELL"
    assert driver.life.pre_stop_inside_since == started_at


def test_pre_speed_yaw_sustained_violation_still_resets():
    """A violation held past the debounce window is real motion, not
    noise, and must still reset the dwell timer."""
    driver = enter_pre()
    driver.tick(heading_deg=90.0, speed=0.0, yaw_rate=0.0)
    driver.tick(heading_deg=90.0, speed=0.05, yaw_rate=0.0, elapsed=0.05)
    driver.tick(heading_deg=90.0, speed=0.05, yaw_rate=0.0, elapsed=0.05)
    late = driver.tick(heading_deg=90.0, speed=0.05, yaw_rate=0.0, elapsed=0.05)
    assert late.transition_reason == "PRE_PIVOT_GATES_OPEN"
    assert driver.life.pre_stop_inside_since is None


def test_pre_stale_telemetry_still_resets_immediately():
    """Staleness is a separate, non-debounced gate: it resets on the first
    bad sample regardless of the stationary-violation debounce."""
    driver = enter_pre()
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
    result = reach_settle_certificate(driver, heading_deg=1.0)
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


def test_settle_brief_violation_does_not_reset():
    """Same debounce protection inside PIVOT_SETTLE, the phase responsible
    for the field-observed 8s dwell inflation from GPS-antenna lever-arm
    noise on an otherwise-stationary chassis."""
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=1.0)
    driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0)
    started_at = driver.life.settle_inside_since
    assert started_at is not None
    blip = driver.tick(heading_deg=1.0, speed=0.05, yaw_rate=0.0, elapsed=0.05)
    assert blip.transition_reason == "SETTLE_CERTIFICATE_DWELL"
    assert driver.life.settle_inside_since == started_at


def test_settle_sustained_violation_still_resets():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=1.0)
    driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0)
    driver.tick(heading_deg=1.0, speed=0.05, yaw_rate=0.0, elapsed=0.05)
    driver.tick(heading_deg=1.0, speed=0.05, yaw_rate=0.0, elapsed=0.05)
    late = driver.tick(heading_deg=1.0, speed=0.05, yaw_rate=0.0, elapsed=0.05)
    assert late.transition_reason == "SETTLE_GATES_OPEN"
    assert driver.life.settle_inside_since is None


def test_settle_escalates_through_pre_on_ge45_heading():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=3.5)
    result = driver.tick(heading_deg=50.0, speed=0.0, yaw_rate=0.0)
    assert result.phase is LegacyAlignmentPhase.PRE_PIVOT_STOP
    assert result.directive is LegacyAlignmentDirective.HOLD_ZERO


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


def test_actual_native_settle_hold_reanchors_before_hold_and_geometry_unchanged():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=1.0)
    first = reach_settle_certificate(driver, heading_deg=1.0)
    assert first.reanchor_requested is True
    assert first.directive is LegacyAlignmentDirective.REANCHOR_ZERO
    assert driver.life.reanchor_complete is False
    driver.life.ack_reanchor_completed()
    assert driver.life.reanchor_complete is True
    second = driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0, xtrack=0.010)
    assert second.reanchor_requested is False
    assert second.phase is LegacyAlignmentPhase.PIVOT_SETTLE
    assert second.transition_reason == "POST_SETTLE_HOLD"


def test_bag_114356_p1_reanchor_completes_without_a_heading_gate_or_crawl():
    """Regression for run 114356 P1.

    Field bag: heading certified at 2.55deg on the old line, then the
    C'->P1 reanchor bearing came out to 3.41deg. The old design required a
    tighter 2deg recapture certificate against the *new* line and crawled at
    0.20 m/s for ~33s / 6m because that gate never closed. The new design has
    no post-reanchor heading gate at all: once the chassis is reanchored and
    held stationary for post_settle_hold_sec, it releases straight to
    COMPLETE_ZERO regardless of the reanchored heading, as long as it is well
    under the 45deg re-pivot threshold.
    """
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=2.55)
    settled = reach_settle_certificate(driver, heading_deg=2.55)
    assert settled.reanchor_requested is True
    driver.life.ack_reanchor_completed()
    done = finish_settle_and_hold_after_reanchor(driver, heading_deg=3.41)
    assert done.directive is LegacyAlignmentDirective.COMPLETE_ZERO
    assert done.pivot_complete is True


def test_second_pivot_after_reanchor_requests_fresh_reanchor_not_stale_c_prime():
    """Regression: a second, genuine pivot must not complete using the first
    pivot's stale C'1 anchor.

    Sequence: first pivot -> reanchor C'1 -> heading blows back out to
    >=45deg during the post-reanchor hold -> second native pivot -> the
    second pivot must request its own fresh reanchor, and must not reach
    COMPLETE_ZERO while still carrying the first pivot's stale anchor state.
    """
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=1.0)
    first_settle = reach_settle_certificate(driver, heading_deg=1.0)
    assert first_settle.reanchor_requested is True
    driver.life.ack_reanchor_completed()
    # Mirrors the node: reanchor_c_to_p1_after_pivot() succeeded, so
    # c_line_reanchored_after_pivot (fed back as already_reanchored) is True.
    driver.already_reanchored = True

    # Heading blows back out to >=45deg while holding after the first reanchor.
    escalate = driver.tick(heading_deg=50.0, speed=0.0, yaw_rate=0.0)
    assert escalate.phase is LegacyAlignmentPhase.PRE_PIVOT_STOP
    assert driver.life.reanchor_complete is False

    # Second, genuine native pivot. native_carrier_issued deliberately stays
    # True from the first pivot (unrelated to this fix), so drive the
    # stationary certificate directly instead of via complete_pre_zero_transition.
    driver.tick(heading_deg=50.0, speed=0.0, yaw_rate=0.0)
    second_native_pivot = driver.tick(
        elapsed=0.21, heading_deg=50.0, speed=0.0, yaw_rate=0.0
    )
    assert second_native_pivot.phase is LegacyAlignmentPhase.NATIVE_PIVOT
    publish_native(driver, heading_deg=50.0)
    # Mirrors the node-side fix: the moment the new carrier actually
    # publishes during first_approach with a stale anchor, that stale
    # anchor is cleared (c_line_bearing itself is left untouched).
    driver.already_reanchored = False

    second_settle = release_to_settle(driver, heading_deg=1.0)
    assert second_settle.reanchor_requested is False
    second_certificate = reach_settle_certificate(driver, heading_deg=1.0)
    assert second_certificate.reanchor_requested is True
    assert second_certificate.directive is LegacyAlignmentDirective.REANCHOR_ZERO
    assert second_certificate.directive is not LegacyAlignmentDirective.COMPLETE_ZERO


def test_native_carrier_publish_clears_stale_reanchor_flag_not_bearing():
    adapter = _method_source("_run_legacy_segment_alignment")
    carrier = adapter[
        adapter.index(
            "if result.directive is LegacyAlignmentDirective.NATIVE_CARRIER:"
        ) : adapter.index("if result.directive is LegacyAlignmentDirective.HOLD_ZERO:")
    ]
    ack_index = carrier.index("ack_native_carrier_published")
    reset_index = carrier.index("self.c_line_reanchored_after_pivot = False")
    assert ack_index < reset_index
    guard = carrier[ack_index:reset_index]
    assert "first_approach" in guard
    assert "self.c_line_reanchored_after_pivot" in guard
    assert "self.c_line_bearing =" not in guard


def test_legacy_param_and_precision_gate_remain_field_safe():
    defaults = _declared_defaults()
    assert defaults["precision_pivot_enabled"] is False
    assert defaults["legacy_pivot_post_settle_hold_sec"] == 1.00
    assert defaults["legacy_pivot_stationary_violation_debounce_sec"] == 0.10
    assert defaults["post_pivot_capture_speed_mps"] == 0.20
    assert defaults["acceleration_distance_m"] == 0.20
    assert defaults["deceleration_distance_m"] == 0.50
    assert defaults["cruise_speed_mps"] == 1.00
    assert defaults["waypoint_tolerance_m"] == 0.03
    assert '"precision_pivot_enabled": False' in LAUNCH_SOURCE
    assert '"legacy_pivot_post_settle_hold_sec": 1.00' in LAUNCH_SOURCE
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
    assert "LegacyAlignmentDirective.RECAPTURE" not in adapter
    assert "LegacyAlignmentDirective.LOW_ENERGY_REALIGN" not in adapter


def test_native_release_without_carrier_never_reaches_pivot_settle():
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
    assert cert.reanchor_requested is True
    driver.life.ack_reanchor_completed()
    hold_start = driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0)
    assert hold_start.transition_reason == "POST_SETTLE_HOLD"
    hold = driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0, elapsed=0.99)
    assert hold.phase is LegacyAlignmentPhase.PIVOT_SETTLE
    done = driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0, elapsed=0.02)
    assert done.directive is LegacyAlignmentDirective.COMPLETE_ZERO


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
    result = reach_settle_certificate(driver, heading_deg=1.0)
    assert result.reanchor_requested is True
    assert driver.life.reanchor_complete is False


def test_successful_reanchor_ack_marks_complete_exactly_once():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=1.0)
    reach_settle_certificate(driver, heading_deg=1.0)
    driver.life.ack_reanchor_completed()
    assert driver.life.reanchor_complete is True
    driver.life.ack_reanchor_completed()
    assert driver.life.reanchor_complete is True
    later = driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0, xtrack=0.040)
    assert later.reanchor_requested is False


def test_failed_reanchor_is_local_safety_hold():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=1.0)
    requested = reach_settle_certificate(driver, heading_deg=1.0)
    assert requested.reanchor_requested is True
    assert driver.life.reanchor_complete is False
    driver.life.enter_safety_hold("REANCHOR_FAILED")
    for _ in range(20):
        later = driver.tick(heading_deg=1.0, speed=0.0, yaw_rate=0.0, xtrack=0.004)
        assert later.phase is LegacyAlignmentPhase.SAFETY_HOLD
        assert later.directive is LegacyAlignmentDirective.SAFETY_HOLD
    adapter = _method_source("_run_legacy_segment_alignment")
    fail = adapter.index("REANCHOR_FAILED")
    assert "enter_safety_hold" in adapter
    assert "ack_reanchor_completed" in adapter
    assert adapter.index("ack_reanchor_completed") < fail


def test_failed_reanchor_later_semantic_geometry_remains_unchanged():
    driver = enter_pre()
    complete_pre_zero_transition(driver)
    publish_native(driver)
    release_to_settle(driver, heading_deg=1.0)
    reach_settle_certificate(driver, heading_deg=1.0)
    driver.life.enter_safety_hold("REANCHOR_FAILED")
    driver.life.reset("SEGMENT_GOAL_CHANGED")
    assert driver.life.phase is LegacyAlignmentPhase.ENTRY
    assert driver.life.native_carrier_issued is False
    assert driver.life.reanchor_complete is False
