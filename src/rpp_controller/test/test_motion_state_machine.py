import math

import pytest

from rpp_controller.motion_state_machine import (
    MotionDirective,
    MotionState,
    PivotMotionConfig,
    PivotMotionInput,
    VerifiedPivotStateMachine,
)


class Driver:
    def __init__(self, fsm, dt=0.1):
        self.fsm = fsm
        self.now = 0.0
        self.dt = dt

    def tick(self, *, elapsed=None, **overrides):
        step = self.dt if elapsed is None else elapsed
        self.now += step
        values = {
            "monotonic_time_sec": self.now,
            "dt_sec": step,
            "anchor_radial_error_m": 0.10,
            "measured_linear_speed_mps": 0.0,
            "measured_yaw_rate_radps": 0.0,
            "heading_error_rad": math.radians(90.0),
            "telemetry_fresh": True,
            "pivot_requested": True,
            "brake_to_anchor_requested": True,
            "recapture_complete": False,
        }
        values.update(overrides)
        return self.fsm.step(PivotMotionInput(**values))


def compact_config(**overrides):
    values = {
        "pivot_anchor_tolerance_m": 0.020,
        "pivot_recenter_threshold_m": 0.020,
        "stop_speed_tolerance_mps": 0.030,
        "stop_yaw_rate_tolerance_radps": 0.050,
        "release_heading_tolerance_rad": math.radians(2.0),
        "stop_settle_sec": 0.20,
        "pivot_release_settle_sec": 0.20,
        "control_dt_max_sec": 0.10,
        "brake_timeout_sec": 2.0,
        "pivot_timeout_sec": 2.0,
        "recenter_timeout_sec": 0.30,
        "realign_timeout_sec": 2.0,
        "recapture_timeout_sec": 2.0,
        "max_recenter_attempts": 2,
    }
    values.update(overrides)
    return PivotMotionConfig(**values)


def enter_pivot(driver, *, initial_heading_deg=90.0):
    assert driver.tick().state is MotionState.CORNER_APPROACH
    assert driver.tick().state is MotionState.BRAKE_TO_ANCHOR
    assert driver.tick(
        anchor_radial_error_m=0.01,
        measured_linear_speed_mps=0.20,
        heading_error_rad=math.radians(initial_heading_deg),
    ).state is MotionState.STOP_SETTLE
    first = driver.tick(
        anchor_radial_error_m=0.01,
        heading_error_rad=math.radians(initial_heading_deg),
    )
    assert first.state is MotionState.STOP_SETTLE
    result = driver.tick(
        anchor_radial_error_m=0.01,
        heading_error_rad=math.radians(initial_heading_deg),
    )
    assert result.state is MotionState.PIVOT
    assert result.stop_certificate.valid
    return result


def settle_pivot_to_recapture(driver):
    result = driver.tick(
        anchor_radial_error_m=0.01,
        heading_error_rad=math.radians(1.0),
        measured_yaw_rate_radps=0.20,
    )
    assert result.state is MotionState.PIVOT_SETTLE
    assert not result.release_certificate.valid
    assert driver.tick(
        anchor_radial_error_m=0.01, heading_error_rad=0.0
    ).state is MotionState.PIVOT_SETTLE
    result = driver.tick(anchor_radial_error_m=0.01, heading_error_rad=0.0)
    assert result.state is MotionState.POSITION_CHECK
    assert result.release_certificate.valid
    result = driver.tick(anchor_radial_error_m=0.01, heading_error_rad=0.0)
    assert result.state is MotionState.RECAPTURE
    assert result.release_certificate.valid
    return result


@pytest.mark.parametrize("turn_deg", [45.0, 90.0, 120.0, 180.0])
def test_clean_pivot_angles_require_stop_and_release_certificates(turn_deg):
    driver = Driver(VerifiedPivotStateMachine(compact_config()))

    enter_pivot(driver, initial_heading_deg=turn_deg)
    result = settle_pivot_to_recapture(driver)

    assert result.directive is MotionDirective.RECAPTURE
    assert result.max_pivot_drift_m == pytest.approx(0.01)
    result = driver.tick(
        anchor_radial_error_m=0.03,
        heading_error_rad=0.0,
        recapture_complete=True,
    )
    assert result.state is MotionState.TRACK
    assert result.transition_reason == "next_leg_recaptured"


def test_stop_settle_requires_position_speed_yaw_rate_freshness_and_dwell():
    driver = Driver(VerifiedPivotStateMachine(compact_config()))
    driver.tick()
    driver.tick()
    result = driver.tick(
        anchor_radial_error_m=0.01,
        measured_linear_speed_mps=0.20,
        measured_yaw_rate_radps=0.20,
    )
    assert result.state is MotionState.STOP_SETTLE

    result = driver.tick(
        anchor_radial_error_m=0.01,
        measured_linear_speed_mps=0.0,
        measured_yaw_rate_radps=0.20,
    )
    assert not result.stop_certificate.yaw_rate_ok
    assert result.stop_certificate.held_sec == 0.0
    result = driver.tick(
        anchor_radial_error_m=0.01,
        measured_linear_speed_mps=0.0,
        measured_yaw_rate_radps=0.0,
    )
    assert result.state is MotionState.STOP_SETTLE
    assert driver.tick(
        anchor_radial_error_m=0.01,
        measured_linear_speed_mps=0.0,
        measured_yaw_rate_radps=0.0,
    ).state is MotionState.PIVOT


def test_stale_telemetry_resets_stop_dwell_without_spending_recenter_attempt():
    driver = Driver(VerifiedPivotStateMachine(compact_config()))
    driver.tick()
    driver.tick()
    driver.tick(anchor_radial_error_m=0.01)
    driver.tick(anchor_radial_error_m=0.01)

    stale = driver.tick(anchor_radial_error_m=0.01, telemetry_fresh=False)
    assert stale.state is MotionState.STOP_SETTLE
    assert stale.stop_certificate.held_sec == 0.0
    assert stale.recenter_attempts == 0
    assert driver.tick(anchor_radial_error_m=0.01).state is MotionState.STOP_SETTLE
    assert driver.tick(anchor_radial_error_m=0.01).state is MotionState.PIVOT


@pytest.mark.parametrize("drift_m", [0.030, 0.100])
def test_pivot_translation_never_releases_and_enters_recenter(drift_m):
    driver = Driver(VerifiedPivotStateMachine(compact_config()))
    enter_pivot(driver)

    result = driver.tick(
        anchor_radial_error_m=drift_m,
        heading_error_rad=0.0,
    )
    assert result.state is MotionState.RECENTER
    assert result.recenter_attempts == 1
    assert result.max_pivot_drift_m == pytest.approx(drift_m)
    assert not result.release_certificate.position_ok


def test_heading_good_but_position_bad_cannot_release():
    driver = Driver(VerifiedPivotStateMachine(compact_config()))
    enter_pivot(driver)

    result = driver.tick(anchor_radial_error_m=0.10, heading_error_rad=0.0)

    assert result.state is MotionState.RECENTER
    assert result.directive is MotionDirective.RECENTER


def test_speed_good_but_yaw_rate_bad_cannot_release():
    driver = Driver(VerifiedPivotStateMachine(compact_config()))
    enter_pivot(driver)
    driver.tick(
        anchor_radial_error_m=0.01,
        heading_error_rad=0.0,
        measured_yaw_rate_radps=0.20,
    )

    for _ in range(4):
        result = driver.tick(
            anchor_radial_error_m=0.01,
            heading_error_rad=0.0,
            measured_linear_speed_mps=0.0,
            measured_yaw_rate_radps=0.20,
        )

    assert result.state is MotionState.PIVOT_SETTLE
    assert result.release_certificate.linear_speed_ok
    assert not result.release_certificate.yaw_rate_ok
    assert not result.release_certificate.valid


def test_stale_pivot_telemetry_cannot_release_and_times_out():
    driver = Driver(
        VerifiedPivotStateMachine(compact_config(pivot_timeout_sec=0.25))
    )
    enter_pivot(driver)

    result = driver.tick(
        elapsed=0.30,
        anchor_radial_error_m=0.01,
        heading_error_rad=0.0,
        telemetry_fresh=False,
    )

    assert result.state is MotionState.HOLD_FAIL
    assert result.transition_reason == "pivot_timeout"
    assert not result.release_certificate.valid


def test_transient_stale_pivot_telemetry_forces_zero_hold():
    driver = Driver(VerifiedPivotStateMachine(compact_config()))
    enter_pivot(driver)

    stale = driver.tick(
        anchor_radial_error_m=0.01,
        heading_error_rad=math.radians(20.0),
        telemetry_fresh=False,
    )

    assert stale.state is MotionState.PIVOT
    assert stale.directive is MotionDirective.HOLD_ZERO
    assert not stale.release_certificate.valid


def test_stale_corner_approach_holds_zero_then_watchdog_fails():
    driver = Driver(
        VerifiedPivotStateMachine(compact_config(brake_timeout_sec=0.25))
    )
    assert driver.tick().state is MotionState.CORNER_APPROACH

    stale = driver.tick(telemetry_fresh=False)
    assert stale.state is MotionState.CORNER_APPROACH
    assert stale.directive is MotionDirective.HOLD_ZERO

    failed = driver.tick(elapsed=0.30, telemetry_fresh=False)
    assert failed.state is MotionState.HOLD_FAIL
    assert failed.directive is MotionDirective.HOLD_FAIL
    assert failed.transition_reason == "corner_approach_telemetry_timeout"


def test_transient_stale_recenter_telemetry_forces_zero_hold():
    driver = Driver(VerifiedPivotStateMachine(compact_config()))
    enter_pivot(driver)
    driver.tick(anchor_radial_error_m=0.10, heading_error_rad=0.0)

    stale = driver.tick(
        anchor_radial_error_m=0.10,
        heading_error_rad=0.0,
        telemetry_fresh=False,
    )

    assert stale.state is MotionState.RECENTER
    assert stale.directive is MotionDirective.HOLD_ZERO
    assert stale.recenter_attempts == 1


def test_transient_stale_realign_telemetry_forces_zero_and_resets_release():
    driver = Driver(VerifiedPivotStateMachine(compact_config()))
    enter_pivot(driver)
    driver.tick(anchor_radial_error_m=0.10, heading_error_rad=0.0)
    driver.tick(anchor_radial_error_m=0.01, heading_error_rad=0.0)
    driver.tick(anchor_radial_error_m=0.01, heading_error_rad=0.0)
    assert driver.tick(
        anchor_radial_error_m=0.01, heading_error_rad=0.0
    ).state is MotionState.REALIGN
    driver.tick(anchor_radial_error_m=0.01, heading_error_rad=0.0)

    stale = driver.tick(
        anchor_radial_error_m=0.01,
        heading_error_rad=0.0,
        telemetry_fresh=False,
    )

    assert stale.state is MotionState.REALIGN
    assert stale.directive is MotionDirective.HOLD_ZERO
    assert stale.release_certificate.held_sec == 0.0
    assert not stale.release_certificate.valid
    assert driver.tick(
        anchor_radial_error_m=0.01, heading_error_rad=0.0
    ).state is MotionState.REALIGN
    result = driver.tick(anchor_radial_error_m=0.01, heading_error_rad=0.0)
    assert result.state is MotionState.RECAPTURE


def test_recenter_success_stops_realigns_and_recaptures():
    driver = Driver(VerifiedPivotStateMachine(compact_config()))
    enter_pivot(driver)
    assert driver.tick(
        anchor_radial_error_m=0.10, heading_error_rad=0.0
    ).state is MotionState.RECENTER

    assert driver.tick(
        anchor_radial_error_m=0.01,
        measured_linear_speed_mps=0.10,
        heading_error_rad=math.radians(8.0),
    ).state is MotionState.STOP_SETTLE
    assert driver.tick(
        anchor_radial_error_m=0.01,
        heading_error_rad=math.radians(8.0),
    ).state is MotionState.STOP_SETTLE
    assert driver.tick(
        anchor_radial_error_m=0.01,
        heading_error_rad=math.radians(8.0),
    ).state is MotionState.REALIGN
    assert driver.tick(
        anchor_radial_error_m=0.01, heading_error_rad=0.0
    ).state is MotionState.REALIGN
    result = driver.tick(anchor_radial_error_m=0.01, heading_error_rad=0.0)

    assert result.state is MotionState.RECAPTURE
    assert result.release_certificate.valid
    assert result.recenter_attempts == 1
    assert result.max_pivot_drift_m == pytest.approx(0.10)


def test_recenter_attempts_are_bounded_and_failure_holds():
    driver = Driver(
        VerifiedPivotStateMachine(
            compact_config(recenter_timeout_sec=0.25, max_recenter_attempts=2)
        )
    )
    enter_pivot(driver)
    assert driver.tick(
        anchor_radial_error_m=0.10, heading_error_rad=0.0
    ).recenter_attempts == 1

    retry = driver.tick(elapsed=0.30, anchor_radial_error_m=0.10)
    assert retry.state is MotionState.RECENTER
    assert retry.recenter_attempts == 2
    failed = driver.tick(elapsed=0.30, anchor_radial_error_m=0.10)

    assert failed.state is MotionState.HOLD_FAIL
    assert failed.directive is MotionDirective.HOLD_FAIL
    assert failed.failed
    assert failed.transition_reason == "recenter_attempts_exhausted"


def test_position_drift_during_realign_reenters_recenter_not_recapture():
    driver = Driver(VerifiedPivotStateMachine(compact_config()))
    enter_pivot(driver)
    driver.tick(anchor_radial_error_m=0.10, heading_error_rad=0.0)
    driver.tick(anchor_radial_error_m=0.01, heading_error_rad=0.0)
    driver.tick(anchor_radial_error_m=0.01, heading_error_rad=0.0)
    assert driver.tick(
        anchor_radial_error_m=0.01, heading_error_rad=0.0
    ).state is MotionState.REALIGN

    result = driver.tick(anchor_radial_error_m=0.04, heading_error_rad=0.0)

    assert result.state is MotionState.RECENTER
    assert result.recenter_attempts == 2


def test_recapture_requires_explicit_completion_and_fresh_telemetry():
    driver = Driver(VerifiedPivotStateMachine(compact_config()))
    enter_pivot(driver)
    settle_pivot_to_recapture(driver)

    result = driver.tick(anchor_radial_error_m=0.01, heading_error_rad=0.0)
    assert result.state is MotionState.RECAPTURE
    failed = driver.tick(
        anchor_radial_error_m=0.01,
        heading_error_rad=0.0,
        telemetry_fresh=False,
        recapture_complete=True,
    )
    assert failed.state is MotionState.HOLD_FAIL


def test_large_and_negative_dt_are_bounded_for_dwell():
    driver = Driver(VerifiedPivotStateMachine(compact_config()))
    driver.tick()
    driver.tick()
    driver.tick(anchor_radial_error_m=0.01)

    large = driver.tick(elapsed=3.0, anchor_radial_error_m=0.01)

    assert large.bounded_dt_sec == pytest.approx(0.10)
    assert large.stop_certificate.held_sec == pytest.approx(0.10)
    # Use a fresh FSM because monotonic time may advance while a negative dt is
    # reported by a clock-domain conversion.
    fsm = VerifiedPivotStateMachine(compact_config())
    result = fsm.step(
        PivotMotionInput(
            monotonic_time_sec=0.1,
            dt_sec=-2.0,
            anchor_radial_error_m=0.0,
            measured_linear_speed_mps=0.0,
            measured_yaw_rate_radps=0.0,
            heading_error_rad=0.0,
            telemetry_fresh=True,
        )
    )
    assert result.bounded_dt_sec == 0.0


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("anchor_radial_error_m", math.nan),
        ("measured_linear_speed_mps", math.inf),
        ("heading_error_rad", -math.inf),
    ],
)
def test_nonfinite_input_is_rejected(field, value):
    values = {
        "monotonic_time_sec": 0.1,
        "dt_sec": 0.1,
        "anchor_radial_error_m": 0.0,
        "measured_linear_speed_mps": 0.0,
        "measured_yaw_rate_radps": 0.0,
        "heading_error_rad": 0.0,
        "telemetry_fresh": True,
    }
    values[field] = value

    with pytest.raises(ValueError, match="must be finite"):
        VerifiedPivotStateMachine().step(PivotMotionInput(**values))


def test_invalid_config_and_backward_time_are_rejected():
    with pytest.raises(ValueError):
        PivotMotionConfig(pivot_recenter_threshold_m=0.001)
    with pytest.raises(ValueError, match="must equal"):
        PivotMotionConfig(
            pivot_anchor_tolerance_m=0.020,
            pivot_recenter_threshold_m=0.050,
        )
    fsm = VerifiedPivotStateMachine()
    fsm.step(
        PivotMotionInput(0.2, 0.1, 0.0, 0.0, 0.0, 0.0, True)
    )
    with pytest.raises(ValueError, match="must not move backwards"):
        fsm.step(PivotMotionInput(0.1, 0.1, 0.0, 0.0, 0.0, 0.0, True))
