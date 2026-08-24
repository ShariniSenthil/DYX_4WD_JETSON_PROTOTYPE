import math

import pytest

from rpp_controller.speed_regulator import (
    LongitudinalRegulator,
    LongitudinalRegulatorConfig,
    SpeedCapOwner,
    SpeedRegulatorInput,
    allowable_speed_for_distance,
    braking_distance,
)


def make_config(**overrides):
    values = {
        "hardware_speed_ceiling_mps": 1.0,
        "acceleration_mps2": 1.0,
        "deceleration_mps2": 1.0,
        "launch_speed_mps": 0.10,
        "control_dt_max_sec": 0.10,
        "heading_accel_full_error_rad": math.radians(2.0),
        "heading_recovery_start_rad": math.radians(4.0),
        "heading_recovery_full_rad": math.radians(15.0),
        "cross_track_accel_full_m": 0.01,
        "cross_track_recovery_start_m": 0.02,
        "cross_track_recovery_full_m": 0.10,
        "recovery_min_speed_mps": 0.20,
        "recovery_exit_dwell_sec": 0.30,
        "corner_angle_threshold_rad": math.radians(45.0),
        "corner_target_speed_mps": 0.12,
        "corner_accel_block_buffer_m": 0.10,
        "terminal_target_speed_mps": 0.0,
        "braking_latency_sec": 0.10,
        "braking_margin_m": 0.05,
        "curvature_enabled": False,
        "lateral_acceleration_max_mps2": 0.30,
    }
    values.update(overrides)
    return LongitudinalRegulatorConfig(**values)


def request(**overrides):
    values = {
        "mission_speed_ceiling_mps": 1.0,
        "measured_speed_mps": 0.0,
        "last_commanded_speed_mps": 0.0,
        "dt_sec": 0.10,
        "along_track_progress_m": 10.0,
        "heading_error_rad": 0.0,
        "cross_track_error_m": 0.0,
    }
    values.update(overrides)
    return SpeedRegulatorInput(**values)


def primed_regulator(config=None, *, speed=1.0, progress=0.0):
    regulator = LongitudinalRegulator(config or make_config())
    regulator.reset(along_track_progress_m=progress, initial_speed_mps=speed)
    return regulator


def test_terminal_target_override_defaults_to_configured_phase2_behavior():
    config = make_config(terminal_target_speed_mps=0.15)
    baseline = primed_regulator(config, speed=1.0).resolve(
        request(distance_to_terminal_m=0.05)
    )
    explicit_none = primed_regulator(config, speed=1.0).resolve(
        request(
            distance_to_terminal_m=0.05,
            terminal_target_speed_override_mps=None,
        )
    )

    assert baseline.caps.terminal_mps == pytest.approx(0.15)
    assert explicit_none.caps.terminal_mps == pytest.approx(
        baseline.caps.terminal_mps
    )


def test_precision_terminal_override_uses_minimum_actuatable_target():
    config = make_config(terminal_target_speed_mps=0.15)
    result = primed_regulator(config, speed=1.0).resolve(
        request(
            distance_to_terminal_m=0.05,
            terminal_target_speed_override_mps=0.04,
        )
    )

    assert result.caps.terminal_mps == pytest.approx(0.04)
    assert result.requested_speed_mps == pytest.approx(0.04)
    assert result.winning_cap_owner is SpeedCapOwner.TERMINAL


@pytest.mark.parametrize("override", [-0.01, math.nan, math.inf, 1.01])
def test_terminal_target_override_rejects_invalid_values(override):
    with pytest.raises(ValueError):
        primed_regulator(speed=1.0).resolve(
            request(
                distance_to_terminal_m=0.10,
                terminal_target_speed_override_mps=override,
            )
        )


def test_mission_and_hardware_ceilings_are_separate_caps():
    mission_owned = primed_regulator(speed=1.0).resolve(
        request(mission_speed_ceiling_mps=0.70)
    )
    hardware_owned = primed_regulator(speed=1.0).resolve(
        request(mission_speed_ceiling_mps=1.30)
    )

    assert mission_owned.requested_speed_mps == pytest.approx(0.70)
    assert mission_owned.winning_cap_owner is SpeedCapOwner.MISSION
    assert mission_owned.caps.hardware_mps == pytest.approx(1.0)
    assert hardware_owned.requested_speed_mps == pytest.approx(1.0)
    assert hardware_owned.winning_cap_owner is SpeedCapOwner.HARDWARE


def test_winning_cap_is_deterministic_when_caps_tie():
    result = primed_regulator(speed=1.0).resolve(request())

    assert result.requested_speed_mps == pytest.approx(1.0)
    assert result.winning_cap_owner is SpeedCapOwner.MISSION


def test_tracking_inputs_default_to_no_cap_and_acceleration_allowed():
    result = primed_regulator(speed=1.0).resolve(request())

    assert result.caps.tracking_mps is None
    assert result.caps.acceleration_mps == pytest.approx(1.0)


def test_tracking_cap_has_explicit_owner_and_immediate_precedence():
    result = primed_regulator(speed=1.0).resolve(
        request(tracking_speed_cap_mps=0.35)
    )

    assert result.caps.tracking_mps == pytest.approx(0.35)
    assert result.requested_speed_mps == pytest.approx(0.35)
    assert result.winning_cap_owner is SpeedCapOwner.TRACKING


def test_recovery_tie_precedence_is_stable_before_tracking_cap():
    result = primed_regulator(speed=0.20).resolve(
        request(
            last_commanded_speed_mps=0.20,
            cross_track_error_m=0.20,
            tracking_speed_cap_mps=0.20,
        )
    )

    assert result.requested_speed_mps == pytest.approx(0.20)
    assert result.winning_cap_owner is SpeedCapOwner.RECOVERY


def test_tracking_acceleration_permission_blocks_upward_speed_change():
    regulator = primed_regulator(speed=0.40)
    result = regulator.resolve(
        request(
            last_commanded_speed_mps=0.40,
            along_track_progress_m=20.0,
            tracking_acceleration_allowed=False,
        )
    )

    assert result.caps.acceleration_mps == pytest.approx(0.40)
    assert result.requested_speed_mps == pytest.approx(0.40)
    assert result.winning_cap_owner is SpeedCapOwner.ACCELERATION


@pytest.mark.parametrize("value", [math.nan, math.inf, -0.01])
def test_tracking_cap_rejects_nonfinite_or_negative_values(value):
    with pytest.raises(ValueError):
        primed_regulator(speed=1.0).resolve(
            request(tracking_speed_cap_mps=value)
        )


def test_tracking_acceleration_permission_requires_boolean():
    with pytest.raises(ValueError):
        primed_regulator(speed=1.0).resolve(
            request(tracking_acceleration_allowed=1)
        )


def test_acceleration_uses_geometric_progress_and_time_slew():
    regulator = LongitudinalRegulator(make_config())
    regulator.reset(along_track_progress_m=5.0)

    first = regulator.resolve(request(along_track_progress_m=5.0))
    second = regulator.resolve(request(along_track_progress_m=5.02))

    assert first.requested_speed_mps == pytest.approx(0.10)
    assert first.winning_cap_owner is SpeedCapOwner.ACCELERATION
    assert first.acceleration_progress_m == pytest.approx(0.0)
    assert second.requested_speed_mps == pytest.approx(0.20)
    assert second.acceleration_progress_m == pytest.approx(0.02)


def test_large_dt_is_bounded_and_cannot_create_a_speed_jump():
    regulator = LongitudinalRegulator(make_config())
    regulator.reset()

    result = regulator.resolve(request(dt_sec=4.0))

    assert result.bounded_dt_sec == pytest.approx(0.10)
    assert result.requested_speed_mps == pytest.approx(0.10)


def test_negative_dt_is_floored_and_cannot_create_a_reverse_slew():
    regulator = LongitudinalRegulator(make_config())
    regulator.reset()

    result = regulator.resolve(request(dt_sec=-0.5))

    assert result.bounded_dt_sec == 0.0
    assert result.requested_speed_mps == 0.0


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("heading_error_rad", math.radians(16.0)),
        ("cross_track_error_m", 0.20),
    ],
)
def test_large_tracking_error_selects_explicit_recovery_authority(field, value):
    regulator = primed_regulator(speed=0.50)
    cycle = request(last_commanded_speed_mps=0.50, **{field: value})

    result = regulator.resolve(cycle)

    assert result.acceleration_gate_scale == 0.0
    assert result.requested_speed_mps == pytest.approx(0.20)
    assert result.winning_cap_owner is SpeedCapOwner.RECOVERY
    assert result.recovery_active is True
    assert result.recovery_transition == "ENTERED"


def test_recovery_request_cannot_remain_trapped_at_adapter_floor():
    regulator = LongitudinalRegulator(make_config(recovery_min_speed_mps=0.15))
    regulator.reset()

    result = regulator.resolve(
        request(
            along_track_progress_m=0.0,
            heading_error_rad=math.radians(20.0),
            cross_track_error_m=0.20,
        )
    )

    assert result.caps.acceleration_mps == pytest.approx(0.0)
    assert result.requested_speed_mps == pytest.approx(0.15)
    assert result.requested_speed_mps > 0.04
    assert result.winning_cap_owner is SpeedCapOwner.RECOVERY


@pytest.mark.parametrize(
    ("cross_track", "heading_deg"),
    [
        (0.016, 0.0),
        (0.050, 5.0),
        (0.099, 0.0),
        (0.0, 14.9),
    ],
)
def test_subthreshold_tracking_error_does_not_select_full_recovery(
    cross_track, heading_deg
):
    regulator = primed_regulator(speed=0.60)

    result = regulator.resolve(
        request(
            last_commanded_speed_mps=0.60,
            cross_track_error_m=cross_track,
            heading_error_rad=math.radians(heading_deg),
        )
    )

    assert result.requested_speed_mps > 0.20
    assert result.requested_speed_mps < 1.0
    assert result.winning_cap_owner is not SpeedCapOwner.RECOVERY
    assert result.recovery_active is False


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("cross_track_error_m", 0.10),
        ("heading_error_rad", math.radians(15.0)),
    ],
)
def test_full_recovery_enters_at_existing_large_error_threshold(field, value):
    result = primed_regulator(speed=0.60).resolve(request(**{field: value}))

    assert result.recovery_active is True
    assert result.recovery_transition == "ENTERED"
    assert result.requested_speed_mps == pytest.approx(0.20)
    assert result.winning_cap_owner is SpeedCapOwner.RECOVERY


def test_recovery_hysteresis_requires_both_errors_inside_exit_thresholds():
    regulator = primed_regulator(speed=0.50)
    entered = regulator.resolve(
        request(cross_track_error_m=0.20, last_commanded_speed_mps=0.50)
    )
    held = regulator.resolve(
        request(
            cross_track_error_m=0.03,
            heading_error_rad=math.radians(5.0),
            last_commanded_speed_mps=entered.requested_speed_mps,
        )
    )
    inside_one = regulator.resolve(
        request(
            cross_track_error_m=0.01,
            heading_error_rad=math.radians(2.0),
            last_commanded_speed_mps=held.requested_speed_mps,
        )
    )
    inside_two = regulator.resolve(
        request(
            cross_track_error_m=0.01,
            heading_error_rad=math.radians(2.0),
            last_commanded_speed_mps=inside_one.requested_speed_mps,
        )
    )
    exited = regulator.resolve(
        request(
            cross_track_error_m=0.01,
            heading_error_rad=math.radians(2.0),
            last_commanded_speed_mps=inside_two.requested_speed_mps,
        )
    )

    assert entered.recovery_transition == "ENTERED"
    assert held.recovery_active is True
    assert held.recovery_transition == "ACTIVE"
    assert inside_one.recovery_active is True
    assert inside_one.recovery_exit_dwell_sec == pytest.approx(0.10)
    assert inside_two.recovery_active is True
    assert inside_two.recovery_exit_dwell_sec == pytest.approx(0.20)
    assert exited.recovery_active is False
    assert exited.recovery_transition == "EXITED"
    assert exited.recovery_exit_dwell_sec == pytest.approx(0.30)


def test_recovery_exit_dwell_resets_when_either_error_leaves_exit_band():
    regulator = primed_regulator(speed=0.50)
    regulator.resolve(request(cross_track_error_m=0.20))
    inside = regulator.resolve(
        request(
            cross_track_error_m=0.01,
            heading_error_rad=math.radians(2.0),
            last_commanded_speed_mps=0.20,
        )
    )
    interrupted = regulator.resolve(
        request(
            cross_track_error_m=0.021,
            heading_error_rad=math.radians(2.0),
            last_commanded_speed_mps=inside.requested_speed_mps,
        )
    )
    restarted = regulator.resolve(
        request(
            cross_track_error_m=0.01,
            heading_error_rad=math.radians(2.0),
            last_commanded_speed_mps=interrupted.requested_speed_mps,
        )
    )

    assert inside.recovery_exit_dwell_sec == pytest.approx(0.10)
    assert interrupted.recovery_active is True
    assert interrupted.recovery_exit_dwell_sec == 0.0
    assert restarted.recovery_active is True
    assert restarted.recovery_exit_dwell_sec == pytest.approx(0.10)


def test_recovery_exit_resumes_slew_without_one_cycle_speed_spike():
    regulator = LongitudinalRegulator(
        make_config(
            recovery_min_speed_mps=0.20,
            recovery_exit_dwell_sec=0.20,
        )
    )
    regulator.reset()
    recovery = regulator.resolve(
        request(
            along_track_progress_m=10.0,
            cross_track_error_m=0.20,
        )
    )
    settling = regulator.resolve(
        request(
            along_track_progress_m=10.0,
            last_commanded_speed_mps=recovery.requested_speed_mps,
            cross_track_error_m=0.0,
            heading_error_rad=0.0,
        )
    )
    resumed = regulator.resolve(
        request(
            along_track_progress_m=10.0,
            last_commanded_speed_mps=settling.requested_speed_mps,
            cross_track_error_m=0.0,
            heading_error_rad=0.0,
        )
    )

    assert recovery.requested_speed_mps == pytest.approx(0.20)
    assert settling.requested_speed_mps == pytest.approx(0.20)
    assert settling.recovery_active is True
    assert resumed.recovery_transition == "EXITED"
    assert resumed.requested_speed_mps == pytest.approx(0.30)
    assert resumed.requested_speed_mps < 1.0


def test_terminal_zero_and_hard_zero_remain_above_recovery_authority():
    terminal = primed_regulator(speed=0.50).resolve(
        request(
            cross_track_error_m=0.20,
            distance_to_terminal_m=0.0,
        )
    )
    hard_stop = primed_regulator(speed=0.50).resolve(
        request(
            cross_track_error_m=0.20,
            hard_zero=True,
        )
    )

    assert terminal.requested_speed_mps == 0.0
    assert terminal.winning_cap_owner is SpeedCapOwner.TERMINAL
    assert hard_stop.requested_speed_mps == 0.0
    assert hard_stop.winning_cap_owner is SpeedCapOwner.HARD_ZERO


def test_partial_heading_error_reduces_acceleration_rate():
    regulator = primed_regulator(speed=0.20)

    result = regulator.resolve(
        request(
            last_commanded_speed_mps=0.20,
            heading_error_rad=math.radians(6.0),
        )
    )

    assert 0.0 < result.acceleration_gate_scale < 1.0
    assert 0.20 < result.caps.acceleration_mps < 0.30


def test_braking_model_uses_maximum_measured_or_commanded_speed():
    regulator = primed_regulator(speed=1.0)
    result = regulator.resolve(
        request(
            measured_speed_mps=-0.80,
            last_commanded_speed_mps=0.30,
            distance_to_terminal_m=2.0,
        )
    )

    assert result.effective_speed_mps == pytest.approx(0.80)
    assert result.terminal_required_braking_distance_m == pytest.approx(
        braking_distance(0.80, 0.0, 1.0, 0.10, 0.05)
    )


def test_allowable_speed_inverts_braking_distance():
    speed = 0.73
    target = 0.12
    distance = braking_distance(speed, target, 0.75, 0.11, 0.04)

    recovered = allowable_speed_for_distance(distance, target, 0.75, 0.11, 0.04)

    assert recovered == pytest.approx(speed)


def test_hard_corner_preview_slows_before_corner_and_blocks_acceleration():
    regulator = primed_regulator(speed=1.0)
    result = regulator.resolve(
        request(
            measured_speed_mps=1.0,
            last_commanded_speed_mps=1.0,
            distance_to_corner_m=0.40,
            corner_angle_rad=math.radians(90.0),
        )
    )

    assert result.caps.corner_mps is not None
    assert result.caps.corner_mps < 1.0
    assert result.winning_cap_owner is SpeedCapOwner.CORNER
    assert result.acceleration_gate_scale == 0.0


def test_corner_below_semantic_threshold_does_not_create_a_cap():
    result = primed_regulator(speed=1.0).resolve(
        request(
            distance_to_corner_m=0.10,
            corner_angle_rad=math.radians(20.0),
        )
    )

    assert result.caps.corner_mps is None


def test_terminal_braking_cap_tightens_with_remaining_distance():
    far_regulator = primed_regulator(speed=1.0)
    near_regulator = primed_regulator(speed=1.0)

    far = far_regulator.resolve(request(distance_to_terminal_m=1.0))
    near = near_regulator.resolve(request(distance_to_terminal_m=0.20))

    assert near.caps.terminal_mps < far.caps.terminal_mps
    assert near.winning_cap_owner is SpeedCapOwner.TERMINAL


def test_hard_zero_overrides_every_cap_immediately():
    regulator = primed_regulator(speed=1.0)

    result = regulator.resolve(
        request(
            measured_speed_mps=1.0,
            last_commanded_speed_mps=1.0,
            hard_zero=True,
        )
    )

    assert result.requested_speed_mps == 0.0
    assert result.winning_cap_owner is SpeedCapOwner.HARD_ZERO


def test_reset_rearms_a_controlled_launch_at_new_along_track_origin():
    regulator = primed_regulator(speed=1.0)
    regulator.reset(along_track_progress_m=25.0)

    result = regulator.resolve(request(along_track_progress_m=25.0))

    assert result.acceleration_progress_m == 0.0
    assert result.requested_speed_mps == pytest.approx(0.10)
    assert result.winning_cap_owner is SpeedCapOwner.ACCELERATION


def test_optional_curvature_cap_uses_lateral_acceleration_law():
    enabled = primed_regulator(make_config(curvature_enabled=True), speed=1.0)
    disabled = primed_regulator(make_config(curvature_enabled=False), speed=1.0)

    enabled_result = enabled.resolve(request(curvature_inv_m=2.0))
    disabled_result = disabled.resolve(request(curvature_inv_m=2.0))

    assert enabled_result.caps.curvature_mps == pytest.approx(math.sqrt(0.30 / 2.0))
    assert enabled_result.winning_cap_owner is SpeedCapOwner.CURVATURE
    assert disabled_result.caps.curvature_mps is None


def test_all_active_caps_and_result_are_finite():
    result = primed_regulator(
        make_config(curvature_enabled=True), speed=0.8
    ).resolve(
        request(
            mission_speed_ceiling_mps=0.9,
            measured_speed_mps=0.7,
            last_commanded_speed_mps=0.8,
            heading_error_rad=0.1,
            cross_track_error_m=0.03,
            distance_to_corner_m=0.8,
            corner_angle_rad=math.pi / 2,
            distance_to_terminal_m=1.4,
            curvature_inv_m=0.5,
        )
    )

    assert math.isfinite(result.requested_speed_mps)
    assert math.isfinite(result.effective_speed_mps)
    assert all(math.isfinite(value) for _, value in result.caps.ordered_items())


@pytest.mark.parametrize(
    "overrides",
    [
        {"hardware_speed_ceiling_mps": 0.0},
        {"deceleration_mps2": 0.0},
        {"control_dt_max_sec": float("nan")},
        {"recovery_exit_dwell_sec": 0.0},
        {"heading_recovery_start_rad": 0.3, "heading_recovery_full_rad": 0.2},
        {"cross_track_recovery_start_m": 0.2, "cross_track_recovery_full_m": 0.1},
        {"recovery_min_speed_mps": 1.1},
        {"corner_angle_threshold_rad": math.pi + 0.01},
    ],
)
def test_invalid_configurations_are_rejected(overrides):
    with pytest.raises(ValueError):
        make_config(**overrides)


@pytest.mark.parametrize(
    "bad_request",
    [
        request(measured_speed_mps=float("nan")),
        request(last_commanded_speed_mps=float("inf")),
        request(dt_sec=float("nan")),
        request(along_track_progress_m=float("nan")),
        request(distance_to_terminal_m=-0.01),
    ],
)
def test_invalid_runtime_values_are_rejected_without_nan_output(bad_request):
    with pytest.raises(ValueError):
        LongitudinalRegulator(make_config()).resolve(bad_request)
