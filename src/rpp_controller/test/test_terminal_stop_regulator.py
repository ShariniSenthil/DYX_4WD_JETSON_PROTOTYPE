import math
from dataclasses import replace

import pytest

from rpp_controller.terminal_stop_regulator import (
    MotionDirection,
    PositionWindowStationaryDetector,
    RadialStopConfig,
    RadialStopFailure,
    RadialStopInput,
    RadialStopState,
    TerminalStopRegulator,
)


def compact_config(**overrides):
    values = {
        "radial_tolerance_m": 0.020,
        "terminal_guidance_distance_m": 0.75,
        "conservative_decel_mps2": 0.30,
        "brake_margin_m": 0.010,
        "stationary_window_sec": 0.50,
        "stationary_displacement_m": 0.005,
        "stationary_yaw_rate_radps": 0.050,
        "maximum_position_sample_gap_sec": 0.20,
        "terminal_timeout_sec": 3.0,
        "settle_timeout_sec": 1.5,
    }
    values.update(overrides)
    return RadialStopConfig(**values)


class Driver:
    def __init__(self, regulator=None, dt=0.10):
        self.regulator = regulator or TerminalStopRegulator(compact_config())
        self.now = 0.0
        self.dt = dt
        self.x_m = 0.0

    def tick(self, *, elapsed=None, **overrides):
        step = self.dt if elapsed is None else elapsed
        self.now += step
        values = {
            "monotonic_time_sec": self.now,
            "position_sample_time_sec": self.now,
            "active": True,
            "terminal_identity": "mark-17",
            "along_remaining_m": 0.50,
            "cross_error_m": 0.0,
            "position_x_m": self.x_m,
            "position_y_m": 0.0,
            "position_derived_speed_mps": 0.20,
            "measured_yaw_rate_radps": 0.0,
            "tracking_speed_command_mps": 0.40,
            "telemetry_fresh": True,
        }
        values.update(overrides)
        return self.regulator.step(RadialStopInput(**values))


def enter_zero(driver, *, along=0.012, cross=0.016, **overrides):
    values = {
        "along_remaining_m": along,
        "cross_error_m": cross,
        "position_derived_speed_mps": 0.05,
    }
    values.update(overrides)
    result = driver.tick(**values)
    assert result.state is RadialStopState.ZERO_LATCH
    assert result.hold_zero
    return result


def settle(driver, *, along=0.012, cross=0.016, x_m=0.0):
    result = None
    for _ in range(7):
        result = driver.tick(
            along_remaining_m=along,
            cross_error_m=cross,
            position_x_m=x_m,
            position_derived_speed_mps=0.0,
        )
        if result.state is RadialStopState.SETTLE:
            break
    assert result is not None
    assert result.state is RadialStopState.SETTLE
    return result


@pytest.mark.parametrize(
    "field,value",
    [
        ("radial_tolerance_m", 0.0),
        ("terminal_guidance_distance_m", math.nan),
        ("conservative_decel_mps2", -0.1),
        ("brake_margin_m", -0.001),
        ("stationary_window_sec", 0.0),
        ("stationary_displacement_m", math.inf),
        ("stationary_yaw_rate_radps", 0.0),
        ("terminal_timeout_sec", 1.0),
    ],
)
def test_config_rejects_invalid_values(field, value):
    values = compact_config().__dict__ if hasattr(compact_config(), "__dict__") else {
        name: getattr(compact_config(), name)
        for name in compact_config().__dataclass_fields__
    }
    values[field] = value
    with pytest.raises(ValueError):
        RadialStopConfig(**values)


def test_config_rejects_brake_margin_outside_radial_tolerance():
    with pytest.raises(ValueError, match="brake_margin_m"):
        compact_config(brake_margin_m=0.021)


def test_radial_circle_boundary_is_inclusive_and_not_a_square():
    on_circle = Driver()
    result = enter_zero(on_circle)
    assert result.radial_error_m == pytest.approx(0.020)

    outside_circle = Driver()
    result = outside_circle.tick(
        along_remaining_m=0.016,
        cross_error_m=0.016,
        position_derived_speed_mps=0.05,
    )
    assert result.state is RadialStopState.TERMINAL_GUIDANCE
    assert not result.hold_zero


def test_nominal_state_sequence_ends_in_persistent_zero_hold():
    driver = Driver()
    observed = []

    observed.append(driver.tick(
        along_remaining_m=0.50,
        position_derived_speed_mps=0.20,
    ).state)
    observed.append(driver.tick(
        along_remaining_m=0.20,
        position_derived_speed_mps=0.40,
    ).state)
    observed.append(enter_zero(driver).state)
    observed.append(settle(driver).state)
    observed.append(driver.tick(
        along_remaining_m=0.012,
        cross_error_m=0.016,
        position_derived_speed_mps=0.0,
    ).state)
    observed.append(driver.tick(active=False).state)

    assert observed == [
        RadialStopState.TERMINAL_GUIDANCE,
        RadialStopState.BRAKE_PROFILE,
        RadialStopState.ZERO_LATCH,
        RadialStopState.SETTLE,
        RadialStopState.CERTIFIED,
        RadialStopState.HOLD_ZERO,
    ]


def test_stopping_lead_uses_current_position_derived_speed():
    regulator = TerminalStopRegulator(compact_config())
    slow = regulator.stopping_distance_m(0.20)
    fast = regulator.stopping_distance_m(0.40)

    assert slow == pytest.approx(0.20 ** 2 / 0.60 + 0.010)
    assert fast == pytest.approx(0.40 ** 2 / 0.60 + 0.010)
    assert fast > slow


def test_dynamic_measured_speed_changes_brake_trigger_distance():
    slow = Driver()
    slow.tick(
        along_remaining_m=0.40,
        position_derived_speed_mps=0.20,
        tracking_speed_command_mps=0.20,
    )
    slow_result = slow.tick(
        along_remaining_m=0.20,
        position_derived_speed_mps=0.20,
        tracking_speed_command_mps=0.20,
    )
    assert slow_result.state is RadialStopState.TERMINAL_GUIDANCE

    fast = Driver()
    fast.tick(along_remaining_m=0.40, position_derived_speed_mps=0.40)
    fast_result = fast.tick(
        along_remaining_m=0.20,
        position_derived_speed_mps=0.40,
    )
    assert fast_result.state is RadialStopState.BRAKE_PROFILE
    assert fast_result.forward_speed_command_mps < 0.40


def test_stopping_lead_uses_commanded_speed_when_position_speed_underreads():
    driver = Driver()
    result = driver.tick(
        along_remaining_m=0.50,
        position_derived_speed_mps=0.05,
        tracking_speed_command_mps=0.40,
    )

    assert result.effective_braking_speed_mps == pytest.approx(0.40)
    assert result.stop_distance_m == pytest.approx(0.40 ** 2 / 0.60 + 0.010)


def test_brake_profile_targets_literal_zero_without_a_speed_floor():
    driver = Driver()
    driver.tick(along_remaining_m=0.40, position_derived_speed_mps=0.40)
    braking = driver.tick(
        along_remaining_m=0.050,
        cross_error_m=0.10,
        position_derived_speed_mps=0.40,
    )
    assert braking.state is RadialStopState.BRAKE_PROFILE
    assert braking.profile_speed_mps == pytest.approx(math.sqrt(0.6 * 0.04))

    zero = driver.tick(
        along_remaining_m=0.010,
        cross_error_m=0.10,
        position_derived_speed_mps=0.10,
    )
    assert zero.state is RadialStopState.ZERO_LATCH
    assert zero.forward_speed_command_mps == 0.0
    assert zero.profile_speed_mps == 0.0


@pytest.mark.parametrize("along", [0.0, -0.0001, -0.20])
def test_forward_is_forbidden_at_or_after_goal_plane(along):
    driver = Driver()
    result = driver.tick(
        along_remaining_m=along,
        cross_error_m=0.25,
        tracking_speed_command_mps=0.40,
    )

    assert result.state is RadialStopState.ZERO_LATCH
    assert result.motion_direction is MotionDirection.ZERO
    assert result.forward_speed_command_mps == 0.0


def test_zero_latch_never_releases_during_coast():
    driver = Driver()
    enter_zero(driver)

    for index in range(6):
        result = driver.tick(
            along_remaining_m=0.010 - index * 0.004,
            cross_error_m=0.0,
            position_x_m=index * 0.010,
            position_derived_speed_mps=0.08,
        )
        assert result.state is RadialStopState.ZERO_LATCH
        assert result.forward_speed_command_mps == 0.0
        assert not result.stationary


def test_position_window_resets_on_motion_yaw_and_sample_gap():
    detector = PositionWindowStationaryDetector(
        window_sec=0.50,
        displacement_limit_m=0.005,
        yaw_rate_limit_radps=0.05,
        maximum_sample_gap_sec=0.20,
    )

    detector.update(
        position_sample_time_sec=0.0,
        x_m=0.0,
        y_m=0.0,
        yaw_rate_radps=0.0,
    )
    held = detector.update(
        position_sample_time_sec=0.4,
        x_m=0.0,
        y_m=0.0,
        yaw_rate_radps=0.0,
    )
    assert held.held_sec == 0.0  # 0.4 second gap reset the window.

    detector.update(
        position_sample_time_sec=0.5,
        x_m=0.0,
        y_m=0.0,
        yaw_rate_radps=0.0,
    )
    moved = detector.update(
        position_sample_time_sec=0.6,
        x_m=0.006,
        y_m=0.0,
        yaw_rate_radps=0.0,
    )
    assert moved.held_sec == 0.0
    assert moved.sample_count == 1

    yawed = detector.update(
        position_sample_time_sec=0.7,
        x_m=0.006,
        y_m=0.0,
        yaw_rate_radps=0.051,
    )
    assert yawed.sample_count == 0
    assert not yawed.stationary

    result = None
    for index in range(1, 7):
        result = detector.update(
            position_sample_time_sec=0.7 + index * 0.1,
            x_m=0.006,
            y_m=0.0,
            yaw_rate_radps=0.0,
        )
    assert result is not None
    assert result.stationary
    assert result.held_sec == pytest.approx(0.5)


def test_duplicate_position_samples_cannot_accumulate_stationary_dwell():
    driver = Driver(dt=0.05)
    enter_zero(driver, position_sample_time_sec=10.0)

    for _ in range(20):
        result = driver.tick(
            elapsed=0.05,
            position_sample_time_sec=10.0,
            along_remaining_m=0.010,
            cross_error_m=0.0,
            position_derived_speed_mps=0.0,
        )

    assert result.state is RadialStopState.ZERO_LATCH
    assert result.stationary_window_sec == 0.0
    assert result.certificate is None


def test_distinct_ten_hz_position_samples_can_prove_stationarity():
    driver = Driver(dt=0.05)
    enter_zero(driver, position_sample_time_sec=20.0)

    result = None
    for index in range(1, 7):
        result = driver.tick(
            elapsed=0.10,
            position_sample_time_sec=20.0 + index * 0.10,
            along_remaining_m=0.010,
            cross_error_m=0.0,
            position_derived_speed_mps=0.0,
        )
        if result.state is RadialStopState.SETTLE:
            break

    assert result is not None
    assert result.state is RadialStopState.SETTLE
    assert result.stationary_window_sec == pytest.approx(0.50)


def test_position_window_memory_is_bounded_to_about_one_window():
    detector = PositionWindowStationaryDetector(
        window_sec=0.50,
        displacement_limit_m=0.005,
        yaw_rate_limit_radps=0.05,
        maximum_sample_gap_sec=0.20,
    )
    evidence = None
    for index in range(101):
        evidence = detector.update(
            position_sample_time_sec=index * 0.10,
            x_m=0.0,
            y_m=0.0,
            yaw_rate_radps=0.0,
        )

    assert evidence is not None
    assert evidence.stationary
    assert evidence.sample_count <= 7


def test_backward_position_sample_fails_regulator_closed():
    driver = Driver()
    enter_zero(driver, position_sample_time_sec=5.0)
    failed = driver.tick(
        position_sample_time_sec=4.9,
        along_remaining_m=0.010,
        cross_error_m=0.0,
        position_derived_speed_mps=0.0,
    )

    assert failed.state is RadialStopState.HOLD_FAIL
    assert failed.failure is RadialStopFailure.INVALID_MEASUREMENT
    assert failed.certificate is None


def test_first_entry_inside_then_settle_outside_fails_closed():
    driver = Driver()
    enter_zero(driver, along=0.015, cross=0.0)
    settle(driver, along=-0.025, cross=0.0, x_m=0.040)
    result = driver.tick(
        along_remaining_m=-0.025,
        cross_error_m=0.0,
        position_x_m=0.040,
        position_derived_speed_mps=0.0,
    )

    assert result.state is RadialStopState.HOLD_FAIL
    assert result.failure is RadialStopFailure.SETTLED_OUTSIDE_TOLERANCE
    assert result.certificate is None
    assert result.hold_zero


def test_stale_telemetry_fails_and_failure_latches_until_reset():
    driver = Driver()
    driver.tick()
    failed = driver.tick(telemetry_fresh=False)
    assert failed.state is RadialStopState.HOLD_FAIL
    assert failed.failure is RadialStopFailure.STALE_TELEMETRY
    assert failed.forward_speed_command_mps == 0.0

    still_failed = driver.tick(active=False, telemetry_fresh=True)
    assert still_failed.state is RadialStopState.HOLD_FAIL
    assert still_failed.failure is RadialStopFailure.STALE_TELEMETRY

    driver.regulator.reset()
    recovered = driver.tick(active=False)
    assert recovered.state is RadialStopState.TRACK


def test_terminal_request_loss_requires_explicit_reset_or_cancel():
    driver = Driver()
    driver.tick(along_remaining_m=0.50)
    failed = driver.tick(active=False)

    assert failed.state is RadialStopState.HOLD_FAIL
    assert failed.failure is RadialStopFailure.TERMINAL_REQUEST_LOST
    assert failed.hold_zero

    driver.regulator.cancel()
    tracking = driver.tick(active=False)
    assert tracking.state is RadialStopState.TRACK


def test_terminal_timeout_never_certifies_and_holds_zero():
    driver = Driver(
        TerminalStopRegulator(compact_config(terminal_timeout_sec=2.0))
    )
    driver.tick(along_remaining_m=0.50)
    result = driver.tick(elapsed=2.01, along_remaining_m=0.25)

    assert result.state is RadialStopState.HOLD_FAIL
    assert result.failure is RadialStopFailure.TERMINAL_TIMEOUT
    assert result.certificate is None
    assert result.hold_zero


def test_settle_timeout_never_certifies():
    driver = Driver()
    enter_zero(driver)
    result = driver.tick(
        elapsed=1.51,
        along_remaining_m=0.010,
        cross_error_m=0.0,
        position_x_m=0.10,
    )

    assert result.state is RadialStopState.HOLD_FAIL
    assert result.failure is RadialStopFailure.SETTLE_TIMEOUT
    assert result.certificate is None


def test_certificate_requires_valid_position_window_and_radial_settle():
    driver = Driver()
    enter_zero(driver)
    settling = settle(driver)
    assert settling.certificate is None

    certified = driver.tick(
        along_remaining_m=0.012,
        cross_error_m=0.016,
        position_derived_speed_mps=0.0,
    )
    assert certified.state is RadialStopState.CERTIFIED
    assert certified.certificate is not None
    assert certified.certificate.version == 2
    assert certified.certificate.terminal_identity == "mark-17"
    assert certified.certificate.radial_error_m == pytest.approx(0.020)
    assert certified.certificate.speed_source == "position_derived"
    assert certified.certificate.stationary_window_sec >= 0.50


def test_certified_transitions_to_persistent_hold_zero():
    driver = Driver()
    enter_zero(driver)
    settle(driver)
    certified = driver.tick(
        along_remaining_m=0.012,
        cross_error_m=0.016,
        position_derived_speed_mps=0.0,
    )
    certificate = certified.certificate

    held = driver.tick(
        active=False,
        terminal_identity="different",
        along_remaining_m=-1.0,
        cross_error_m=1.0,
        telemetry_fresh=False,
    )
    assert held.state is RadialStopState.HOLD_ZERO
    assert held.certificate is certificate
    assert held.forward_speed_command_mps == 0.0

    held_again = driver.tick(active=False)
    assert held_again.state is RadialStopState.HOLD_ZERO


def test_identity_change_fails_closed_without_certificate():
    driver = Driver()
    driver.tick(terminal_identity="mark-17")
    result = driver.tick(terminal_identity="mark-18")

    assert result.state is RadialStopState.HOLD_FAIL
    assert result.failure is RadialStopFailure.IDENTITY_CHANGED
    assert result.certificate is None


def test_output_contract_cannot_request_reverse_or_pivot():
    fields = set(TerminalStopRegulator(compact_config()).step(
        RadialStopInput(
            monotonic_time_sec=0.0,
            position_sample_time_sec=0.0,
            active=False,
            terminal_identity=None,
            along_remaining_m=1.0,
            cross_error_m=0.0,
            position_x_m=0.0,
            position_y_m=0.0,
            position_derived_speed_mps=0.0,
            measured_yaw_rate_radps=0.0,
            tracking_speed_command_mps=0.4,
            telemetry_fresh=True,
        )
    ).__dataclass_fields__)

    assert "yaw_rate_command_radps" not in fields
    assert "bearing_command_rad" not in fields
    assert "reverse_speed_command_mps" not in fields
    assert set(MotionDirection) == {
        MotionDirection.FORWARD,
        MotionDirection.ZERO,
    }


def test_nonfinite_measurement_fails_safe_and_nonmonotonic_time_raises():
    driver = Driver()
    failed = driver.tick(along_remaining_m=math.nan)
    assert failed.state is RadialStopState.HOLD_FAIL
    assert failed.failure is RadialStopFailure.INVALID_MEASUREMENT

    regulator = TerminalStopRegulator(compact_config())
    sample = RadialStopInput(
        monotonic_time_sec=1.0,
        position_sample_time_sec=1.0,
        active=False,
        terminal_identity=None,
        along_remaining_m=1.0,
        cross_error_m=0.0,
        position_x_m=0.0,
        position_y_m=0.0,
        position_derived_speed_mps=0.0,
        measured_yaw_rate_radps=0.0,
        tracking_speed_command_mps=0.4,
        telemetry_fresh=True,
    )
    regulator.step(sample)
    with pytest.raises(ValueError, match="must not move backwards"):
        regulator.step(replace(sample, monotonic_time_sec=0.9))
