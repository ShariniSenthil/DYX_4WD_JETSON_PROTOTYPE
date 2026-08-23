import math

import pytest

from rpp_controller.tracking_control import (
    TrackingControlConfig,
    TrackingControlInput,
    TrackingControlState,
    TrackingMetricsAccumulator,
    TrackingStabilityController,
)


def sample(**overrides):
    values = dict(
        path_identity="path-a",
        projection_s_m=1.0,
        signed_cross_track_m=0.0,
        heading_error_rad=0.0,
        commanded_speed_mps=0.5,
        measured_speed_mps=0.5,
        bearing_clamped=False,
        telemetry_fresh=True,
        dt_sec=0.1,
    )
    values.update(overrides)
    return TrackingControlInput(**values)


def stable_controller(**config_overrides):
    config = TrackingControlConfig(
        stable_recapture_dwell_sec=0.2,
        **config_overrides,
    )
    controller = TrackingStabilityController(config)
    controller.step(sample())
    controller.step(sample())
    assert controller.state is TrackingControlState.TRACKING
    return controller


@pytest.mark.parametrize(
    "overrides",
    [
        {"signed_cross_track_m": 0.050},
        {"signed_cross_track_m": -0.050},
        {"heading_error_rad": math.radians(15.0)},
        {"heading_error_rad": math.radians(-15.0)},
        {"bearing_clamped": True},
    ],
)
def test_large_error_or_bearing_clamp_immediately_enters_recovery(overrides):
    controller = stable_controller()

    result = controller.step(sample(**overrides))

    assert result.state is TrackingControlState.RECOVERY
    assert result.acceleration_allowed is False
    assert result.recovery_speed_scale == pytest.approx(0.35)
    assert result.transition_reason.startswith("recovery_entered:")


def test_hysteresis_does_not_exit_recovery_between_enter_and_exit_thresholds():
    controller = stable_controller()
    controller.step(sample(signed_cross_track_m=0.06))

    result = controller.step(sample(signed_cross_track_m=0.03))

    assert result.state is TrackingControlState.RECOVERY
    assert result.stable_dwell_sec == 0.0
    assert result.transition_reason == "recapture_exit_gate_lost"


def test_tight_exit_gates_must_hold_for_full_recapture_dwell():
    controller = stable_controller()
    controller.step(sample(signed_cross_track_m=0.06))

    first = controller.step(sample(dt_sec=0.1))
    complete = controller.step(sample(dt_sec=0.1))

    assert first.state is TrackingControlState.RECAPTURE_STABLE
    assert first.acceleration_allowed is False
    assert first.recovery_speed_scale == pytest.approx(0.5)
    assert complete.state is TrackingControlState.TRACKING
    assert complete.acceleration_allowed is True
    assert complete.recovery_speed_scale == pytest.approx(1.0)


def test_lost_exit_gate_resets_recapture_dwell():
    config = TrackingControlConfig(stable_recapture_dwell_sec=0.3)
    controller = TrackingStabilityController(config)
    controller.step(sample(dt_sec=0.1))
    assert controller.state is TrackingControlState.RECAPTURE_STABLE

    lost = controller.step(sample(signed_cross_track_m=0.03, dt_sec=0.1))
    restarted = controller.step(sample(dt_sec=0.1))

    assert lost.state is TrackingControlState.RECOVERY
    assert lost.stable_dwell_sec == 0.0
    assert restarted.state is TrackingControlState.RECAPTURE_STABLE
    assert restarted.stable_dwell_sec == pytest.approx(0.1)


def test_dt_is_bounded_before_it_advances_dwell():
    config = TrackingControlConfig(
        stable_recapture_dwell_sec=0.2,
        control_dt_max_sec=0.05,
    )
    controller = TrackingStabilityController(config)

    result = controller.step(sample(dt_sec=10.0))

    assert result.bounded_dt_sec == pytest.approx(0.05)
    assert result.stable_dwell_sec == pytest.approx(0.05)
    assert result.state is TrackingControlState.RECAPTURE_STABLE


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        ({"telemetry_fresh": False}, "stale_telemetry"),
        ({"projection_s_m": math.nan}, "nonfinite_projection_s_m"),
        ({"signed_cross_track_m": math.inf}, "nonfinite_signed_cross_track_m"),
        ({"heading_error_rad": math.nan}, "nonfinite_heading_error_rad"),
        ({"dt_sec": -0.01}, "negative_dt_sec"),
    ],
)
def test_stale_or_invalid_input_fails_safe(overrides, reason):
    controller = stable_controller()

    result = controller.step(sample(**overrides))

    assert result.valid is False
    assert result.state is TrackingControlState.RECOVERY
    assert result.acceleration_allowed is False
    assert result.recovery_speed_scale == 0.0
    assert result.bounded_dt_sec == 0.0
    assert result.invalid_reason == reason


def test_stable_tracking_allows_acceleration_until_an_enter_gate_trips():
    controller = stable_controller()

    result = controller.step(
        sample(
            signed_cross_track_m=0.03,
            heading_error_rad=math.radians(8.0),
        )
    )

    assert result.state is TrackingControlState.TRACKING
    assert result.acceleration_allowed is True


def test_path_identity_change_forces_a_fresh_recapture_dwell():
    controller = stable_controller()

    result = controller.step(sample(path_identity="path-b", dt_sec=0.1))

    assert result.reset_applied is True
    assert result.state is TrackingControlState.RECAPTURE_STABLE
    assert result.acceleration_allowed is False
    assert result.transition_reason == "path_reset_recapture"


def test_metrics_exact_mean_rms_and_max_values():
    metrics = TrackingMetricsAccumulator(TrackingControlConfig())
    for index, xtrack in enumerate((0.01, -0.02, 0.03), start=1):
        assert metrics.add(
            sample(
                projection_s_m=float(index),
                signed_cross_track_m=xtrack,
                heading_error_rad=0.1 * index,
                commanded_speed_mps=0.5,
                measured_speed_mps=0.4,
            ),
            TrackingControlState.TRACKING,
            mission_identity="mission-a",
        )

    result = metrics.snapshot()
    assert result.sample_count == 3
    assert result.mean_abs_cross_track_m == pytest.approx(0.02)
    assert result.rms_cross_track_m == pytest.approx(
        math.sqrt((0.01**2 + 0.02**2 + 0.03**2) / 3.0)
    )
    assert result.max_abs_cross_track_m == pytest.approx(0.03)
    assert result.mean_abs_heading_error_rad == pytest.approx(0.2)
    assert result.max_abs_heading_error_rad == pytest.approx(0.3)
    assert result.mean_abs_speed_error_mps == pytest.approx(0.1)
    assert result.rms_speed_error_mps == pytest.approx(0.1)
    assert result.max_abs_speed_error_mps == pytest.approx(0.1)


def test_p95_uses_deterministic_nearest_rank_behavior():
    metrics = TrackingMetricsAccumulator(
        TrackingControlConfig(metrics_quantile_window_capacity=100)
    )
    for index in range(1, 101):
        metrics.add(
            sample(
                projection_s_m=float(index),
                signed_cross_track_m=index / 1000.0,
            ),
            TrackingControlState.TRACKING,
        )

    result = metrics.snapshot()
    assert result.p95_abs_cross_track_m == pytest.approx(0.096)
    assert result.trailing_p95_abs_cross_track_m == pytest.approx(0.095)
    assert result.p95_histogram_saturated is False
    assert result.histogram_overflow_count == 0


def test_quantile_storage_is_bounded_and_tracks_the_trailing_window():
    metrics = TrackingMetricsAccumulator(
        TrackingControlConfig(metrics_quantile_window_capacity=4)
    )
    for index in range(10):
        metrics.add(
            sample(
                projection_s_m=float(index),
                signed_cross_track_m=float(index),
            ),
            TrackingControlState.TRACKING,
        )

    result = metrics.snapshot()
    assert metrics.quantile_storage_size == 4
    assert result.quantile_sample_count == 4
    assert result.sample_count == 10
    assert result.trailing_p95_abs_cross_track_m == pytest.approx(9.0)
    assert result.p95_abs_cross_track_m == pytest.approx(9.0)
    assert result.p95_histogram_saturated is True
    assert result.histogram_overflow_count == 8


def test_whole_run_p95_retains_early_bad_samples_after_trailing_window_ages_out():
    config = TrackingControlConfig(
        metrics_quantile_window_capacity=10,
        metrics_histogram_bin_width_m=0.001,
        metrics_histogram_max_m=0.2,
    )
    metrics = TrackingMetricsAccumulator(config)
    for index in range(10):
        metrics.add(
            sample(projection_s_m=float(index), signed_cross_track_m=0.100),
            TrackingControlState.TRACKING,
        )
    for index in range(10, 100):
        metrics.add(
            sample(projection_s_m=float(index), signed_cross_track_m=0.001),
            TrackingControlState.TRACKING,
        )

    result = metrics.snapshot()
    assert result.quantile_sample_count == 10
    assert result.trailing_p95_abs_cross_track_m == pytest.approx(0.001)
    assert result.p95_abs_cross_track_m == pytest.approx(0.101)
    assert abs(result.p95_abs_cross_track_m - 0.100) <= (
        config.metrics_histogram_bin_width_m + 1.0e-12
    )


def test_histogram_overflow_and_saturation_are_explicit():
    metrics = TrackingMetricsAccumulator(
        TrackingControlConfig(
            metrics_histogram_bin_width_m=0.01,
            metrics_histogram_max_m=0.05,
        )
    )
    for index, xtrack in enumerate((0.01, 0.08, 0.09, 0.10), start=1):
        metrics.add(
            sample(projection_s_m=float(index), signed_cross_track_m=xtrack),
            TrackingControlState.RECOVERY,
        )

    result = metrics.snapshot()
    assert result.histogram_overflow_count == 3
    assert result.p95_histogram_saturated is True
    assert result.p95_abs_cross_track_m == pytest.approx(0.10)


def test_snap_back_violations_are_diagnostic_and_counted_against_max_s():
    metrics = TrackingMetricsAccumulator(TrackingControlConfig())
    for projection_s in (1.0, 2.0, 1.5, 1.75, 2.1):
        metrics.add(
            sample(projection_s_m=projection_s),
            TrackingControlState.TRACKING,
        )

    assert metrics.snapshot().monotonic_s_violation_count == 2


def test_recovery_recapture_and_cruise_time_are_separate_and_dt_bounded():
    config = TrackingControlConfig(
        control_dt_max_sec=0.1,
        metrics_cruise_speed_threshold_mps=0.8,
    )
    metrics = TrackingMetricsAccumulator(config)

    metrics.add(sample(dt_sec=2.0), TrackingControlState.RECOVERY)
    metrics.add(sample(dt_sec=2.0), TrackingControlState.RECAPTURE_STABLE)
    metrics.add(
        sample(commanded_speed_mps=0.9, dt_sec=2.0),
        TrackingControlState.TRACKING,
    )
    metrics.add(
        sample(commanded_speed_mps=0.7, dt_sec=2.0),
        TrackingControlState.TRACKING,
    )

    result = metrics.snapshot()
    assert result.elapsed_sec == pytest.approx(0.4)
    assert result.recovery_time_sec == pytest.approx(0.1)
    assert result.recapture_time_sec == pytest.approx(0.1)
    assert result.cruise_time_sec == pytest.approx(0.1)


def test_metrics_reset_deterministically_on_path_or_mission_identity_change():
    metrics = TrackingMetricsAccumulator(TrackingControlConfig())
    metrics.add(
        sample(path_identity="path-a", signed_cross_track_m=1.0),
        TrackingControlState.RECOVERY,
        mission_identity="mission-a",
    )
    metrics.add(
        sample(path_identity="path-b", signed_cross_track_m=0.01),
        TrackingControlState.TRACKING,
        mission_identity="mission-a",
    )

    result = metrics.snapshot()
    assert result.mission_identity == "mission-a"
    assert result.path_identity == "path-b"
    assert result.sample_count == 1
    assert result.max_abs_cross_track_m == pytest.approx(0.01)

    metrics.add(
        sample(path_identity="path-b", signed_cross_track_m=0.02),
        TrackingControlState.TRACKING,
        mission_identity="mission-b",
    )
    result = metrics.snapshot()
    assert result.mission_identity == "mission-b"
    assert result.sample_count == 1


def test_discontinuity_preserves_totals_but_starts_new_projection_epoch():
    metrics = TrackingMetricsAccumulator(TrackingControlConfig())
    for projection_s in (10.0, 11.0):
        metrics.add(
            sample(projection_s_m=projection_s, signed_cross_track_m=0.02),
            TrackingControlState.TRACKING,
            mission_identity="mission-a",
        )
    before = metrics.snapshot()

    metrics.note_discontinuity("ekf_jump")
    metrics.add(
        sample(projection_s_m=1.0, signed_cross_track_m=0.03),
        TrackingControlState.RECOVERY,
        mission_identity="mission-a",
    )
    after = metrics.snapshot()

    assert after.sample_count == before.sample_count + 1
    assert after.monotonic_s_violation_count == 0
    assert after.discontinuity_count == 1
    assert after.last_discontinuity_reason == "ekf_jump"
    assert after.valid_for_acceptance is False
    assert after.max_abs_cross_track_m == pytest.approx(0.03)


def test_discontinuity_reset_restores_acceptance_validity():
    metrics = TrackingMetricsAccumulator(TrackingControlConfig())
    metrics.add(sample(), TrackingControlState.TRACKING)
    metrics.note_discontinuity("path_reacquired")
    assert metrics.snapshot().valid_for_acceptance is False

    metrics.reset("mission-b", "path-b")
    result = metrics.snapshot()
    assert result.sample_count == 0
    assert result.discontinuity_count == 0
    assert result.last_discontinuity_reason is None
    assert result.valid_for_acceptance is True


def test_metrics_reject_invalid_samples_without_contaminating_aggregates():
    metrics = TrackingMetricsAccumulator(TrackingControlConfig())
    accepted = metrics.add(
        sample(signed_cross_track_m=math.nan),
        TrackingControlState.TRACKING,
    )

    result = metrics.snapshot()
    assert accepted is False
    assert result.sample_count == 0
    assert result.rejected_sample_count == 1
    assert result.mean_abs_cross_track_m == 0.0


@pytest.mark.parametrize(
    "overrides",
    [
        {"recovery_exit_cross_track_m": 0.05},
        {"recovery_exit_heading_error_rad": math.radians(15.0)},
        {"recovery_speed_scale": 0.6, "recapture_speed_scale": 0.5},
        {"metrics_quantile_window_capacity": 0},
        {"metrics_histogram_bin_width_m": 0.0},
        {"metrics_histogram_max_m": 0.0},
        {
            "metrics_histogram_bin_width_m": 1.0e-9,
            "metrics_histogram_max_m": 1.0,
        },
        {"control_dt_max_sec": 0.0},
        {"stable_recapture_dwell_sec": -1.0},
    ],
)
def test_invalid_config_is_rejected(overrides):
    with pytest.raises(ValueError):
        TrackingControlConfig(**overrides)
