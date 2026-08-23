import json
import math
from dataclasses import FrozenInstanceError, replace

import pytest

from rpp_controller.terminal_certificate import (
    ControllerPose,
    PrecisionTerminalCertificate,
    TerminalConfig,
    TerminalDirective,
    TerminalInput,
    TerminalState,
    TerminalStopStateMachine,
)


def compact_config(**overrides):
    values = {
        "terminal_radial_tolerance_m": 0.010,
        "capture_entry_tolerance_m": 0.010,
        "settle_radial_tolerance_m": 0.010,
        "stop_speed_tolerance_mps": 0.020,
        "stop_yaw_rate_tolerance_radps": 0.050,
        "settle_dwell_sec": 0.30,
        "approach_distance_m": 0.75,
        "brake_distance_m": 0.30,
        "terminal_timeout_sec": 3.0,
        "settle_timeout_sec": 1.5,
        "control_dt_max_sec": 0.10,
        "minimum_actuatable_speed_mps": 0.040,
    }
    values.update(overrides)
    return TerminalConfig(**values)


class Driver:
    def __init__(self, fsm, dt=0.10):
        self.fsm = fsm
        self.now = 0.0
        self.dt = dt

    def tick(self, *, elapsed=None, **overrides):
        step = self.dt if elapsed is None else elapsed
        self.now += step
        values = {
            "monotonic_time_sec": self.now,
            "dt_sec": step,
            "terminal_requested": True,
            "terminal_identity": "mark-17",
            "distance_to_terminal_m": 0.50,
            "radial_error_m": 0.20,
            "cross_track_error_m": 0.004,
            "along_track_error_m": 0.006,
            "measured_linear_speed_mps": 0.0,
            "measured_yaw_rate_radps": 0.0,
            "telemetry_fresh": True,
            "braking_required": False,
            "heading_error_deg": 1.25,
            "current_pose": ControllerPose(1.0, 2.0, 0.2),
        }
        values.update(overrides)
        return self.fsm.step(TerminalInput(**values))


def capture(driver, **overrides):
    values = {
        "distance_to_terminal_m": 0.010,
        "radial_error_m": 0.008,
        "cross_track_error_m": 0.003,
        "along_track_error_m": 0.007,
    }
    values.update(overrides)
    result = driver.tick(**values)
    assert result.state is TerminalState.CAPTURE
    assert result.directive is TerminalDirective.HOLD_ZERO
    assert result.zero_latched
    return result


def enter_settle(driver, **overrides):
    capture(driver, **overrides)
    assert driver.tick(radial_error_m=0.008).state is TerminalState.ZERO_LATCH
    result = driver.tick(radial_error_m=0.008)
    assert result.state is TerminalState.SETTLE
    return result


def certify(driver, **final_overrides):
    enter_settle(driver)
    values = {"radial_error_m": 0.008}
    values.update(final_overrides)
    result = driver.tick(elapsed=0.11, **values)
    assert result.state is TerminalState.CERTIFIED
    assert result.certificate is not None
    return result


def test_exact_approach_brake_capture_zero_settle_certified_timeline():
    driver = Driver(TerminalStopStateMachine(compact_config()))

    approach = driver.tick(distance_to_terminal_m=0.60)
    assert approach.previous_state is TerminalState.TRACK
    assert approach.state is TerminalState.APPROACH
    assert approach.directive is TerminalDirective.APPROACH

    brake = driver.tick(distance_to_terminal_m=0.20)
    assert brake.state is TerminalState.BRAKE
    assert brake.directive is TerminalDirective.BRAKE

    captured = capture(driver)
    assert captured.previous_state is TerminalState.BRAKE
    assert captured.settle_held_sec == 0.0

    latched = driver.tick(radial_error_m=0.008)
    assert latched.state is TerminalState.ZERO_LATCH
    assert latched.directive is TerminalDirective.HOLD_ZERO

    settling = driver.tick(radial_error_m=0.008)
    assert settling.state is TerminalState.SETTLE
    assert settling.certificate is None

    certified = driver.tick(elapsed=0.11, radial_error_m=0.008)
    assert certified.state is TerminalState.CERTIFIED
    assert certified.directive is TerminalDirective.HOLD_ZERO
    assert certified.certificate.precision_pass


def test_terminal_request_remains_track_until_geometric_approach_window():
    driver = Driver(TerminalStopStateMachine(compact_config()))

    armed = driver.tick(distance_to_terminal_m=1.20, radial_error_m=1.20)
    assert armed.state is TerminalState.TRACK
    assert armed.directive is TerminalDirective.TRACK

    approach = driver.tick(distance_to_terminal_m=0.75, radial_error_m=0.75)
    assert approach.state is TerminalState.APPROACH
    assert approach.directive is TerminalDirective.APPROACH


def test_ten_millimetre_boundary_is_inclusive_but_first_sample_cannot_pass():
    driver = Driver(TerminalStopStateMachine(compact_config()))

    result = capture(driver, radial_error_m=0.010)

    assert result.state is TerminalState.CAPTURE
    assert result.certificate is None
    assert result.settle_held_sec == 0.0


def test_eight_to_fourteen_mm_drift_never_certifies_and_watchdog_fails_zero():
    driver = Driver(
        TerminalStopStateMachine(compact_config(settle_timeout_sec=0.50))
    )
    capture(driver, radial_error_m=0.008)

    drifted = driver.tick(radial_error_m=0.014)
    assert drifted.state is TerminalState.ZERO_LATCH
    assert drifted.directive is TerminalDirective.HOLD_ZERO
    assert drifted.settle_held_sec == 0.0

    for _ in range(2):
        result = driver.tick(radial_error_m=0.006)
        assert result.certificate is None
        assert result.directive is TerminalDirective.HOLD_ZERO

    failed = driver.tick(elapsed=0.20, radial_error_m=0.006)
    assert failed.state is TerminalState.HOLD_FAIL
    assert failed.directive is TerminalDirective.HOLD_FAIL
    assert failed.zero_latched
    assert failed.certificate is None
    assert failed.transition_reason == "terminal_settle_timeout"


def test_stale_telemetry_cannot_capture_and_resets_settle_dwell():
    driver = Driver(TerminalStopStateMachine(compact_config()))

    stale_entry = driver.tick(radial_error_m=0.008, telemetry_fresh=False)
    assert stale_entry.state is TerminalState.APPROACH
    assert stale_entry.directive is TerminalDirective.HOLD_ZERO
    assert not stale_entry.motion_evidence_valid
    assert not stale_entry.zero_latched

    capture(driver, radial_error_m=0.008)
    stale = driver.tick(radial_error_m=0.008, telemetry_fresh=False)
    assert stale.directive is TerminalDirective.HOLD_ZERO
    assert stale.settle_held_sec == 0.0
    assert stale.certificate is None

    # Fresh evidence must begin a new dwell rather than inheriting stale time.
    driver.tick(radial_error_m=0.008)
    restarted = driver.tick(radial_error_m=0.008)
    assert restarted.settle_held_sec == pytest.approx(0.10)
    assert restarted.certificate is None


@pytest.mark.parametrize(
    ("bad_field", "bad_value"),
    [
        ("measured_linear_speed_mps", 0.021),
        ("measured_yaw_rate_radps", 0.051),
    ],
)
def test_speed_and_yaw_rate_each_reset_dwell_independently(
    bad_field, bad_value
):
    driver = Driver(TerminalStopStateMachine(compact_config()))
    settling = enter_settle(driver)
    assert settling.settle_held_sec > 0.0

    bad = driver.tick(radial_error_m=0.008, **{bad_field: bad_value})
    assert bad.state is TerminalState.SETTLE
    assert bad.settle_held_sec == 0.0
    assert bad.certificate is None

    first_good = driver.tick(radial_error_m=0.008)
    assert first_good.settle_held_sec == 0.0
    assert first_good.certificate is None


def test_zero_latch_never_regrows_motion_for_same_identity():
    driver = Driver(TerminalStopStateMachine(compact_config()))
    capture(driver)

    for values in (
        {"radial_error_m": 0.20, "distance_to_terminal_m": 0.60},
        {
            "radial_error_m": math.nan,
            "distance_to_terminal_m": math.nan,
            "braking_required": True,
        },
        {"radial_error_m": 0.008, "telemetry_fresh": False},
    ):
        result = driver.tick(**values)
        assert result.zero_latched
        assert result.directive in {
            TerminalDirective.HOLD_ZERO,
            TerminalDirective.HOLD_FAIL,
        }
        assert result.directive not in {
            TerminalDirective.APPROACH,
            TerminalDirective.BRAKE,
            TerminalDirective.TRACK,
        }


def test_cancel_and_identity_change_are_explicit_latch_boundaries():
    driver = Driver(TerminalStopStateMachine(compact_config()))
    capture(driver)

    cancelled = driver.tick(
        terminal_requested=False,
        terminal_identity=None,
        radial_error_m=math.nan,
    )
    assert cancelled.state is TerminalState.TRACK
    assert cancelled.directive is TerminalDirective.TRACK
    assert not cancelled.zero_latched
    assert cancelled.terminal_identity is None
    assert cancelled.transition_reason == "terminal_request_cancelled"

    capture(driver, terminal_identity="mark-17")
    changed = driver.tick(
        terminal_identity="mark-18",
        radial_error_m=0.20,
        distance_to_terminal_m=0.50,
    )
    assert changed.state is TerminalState.APPROACH
    assert changed.terminal_identity == "mark-18"
    assert not changed.zero_latched
    assert "terminal_identity_changed" in changed.transition_reason


def test_public_reset_requires_named_semantic_boundary():
    fsm = TerminalStopStateMachine(compact_config())
    driver = Driver(fsm)
    capture(driver)

    with pytest.raises(ValueError, match="semantic_boundary_reason"):
        fsm.reset(monotonic_time_sec=driver.now, semantic_boundary_reason="")

    fsm.reset(
        monotonic_time_sec=driver.now,
        semantic_boundary_reason="path_replaced",
    )
    assert fsm.state is TerminalState.TRACK
    assert not fsm.zero_latched
    assert fsm.last_reset_reason == "path_replaced"


def test_request_and_settle_watchdogs_fail_without_releasing_motion():
    approach_driver = Driver(
        TerminalStopStateMachine(compact_config(terminal_timeout_sec=0.25))
    )
    approach_driver.tick(radial_error_m=0.20)
    timed_out = approach_driver.tick(elapsed=0.26, radial_error_m=0.20)
    assert timed_out.state is TerminalState.HOLD_FAIL
    assert timed_out.directive is TerminalDirective.HOLD_FAIL
    assert not timed_out.zero_latched

    settle_driver = Driver(
        TerminalStopStateMachine(
            compact_config(settle_dwell_sec=0.10, settle_timeout_sec=0.25)
        )
    )
    capture(settle_driver)
    failed = settle_driver.tick(
        elapsed=0.26,
        radial_error_m=0.008,
        measured_linear_speed_mps=0.10,
    )
    assert failed.state is TerminalState.HOLD_FAIL
    assert failed.directive is TerminalDirective.HOLD_FAIL
    assert failed.zero_latched


def test_hold_fail_is_latched_for_same_identity_but_new_identity_can_start():
    driver = Driver(
        TerminalStopStateMachine(compact_config(terminal_timeout_sec=0.20))
    )
    driver.tick(radial_error_m=0.20)
    failed = driver.tick(elapsed=0.20, radial_error_m=0.20)
    assert failed.state is TerminalState.HOLD_FAIL

    same = driver.tick(radial_error_m=0.008)
    assert same.state is TerminalState.HOLD_FAIL
    assert same.directive is TerminalDirective.HOLD_FAIL

    replacement = driver.tick(
        terminal_identity="mark-18",
        radial_error_m=0.20,
    )
    assert replacement.state is TerminalState.APPROACH
    assert replacement.directive is TerminalDirective.APPROACH


def test_nonfinite_evidence_is_safe_before_and_after_capture():
    driver = Driver(
        TerminalStopStateMachine(compact_config(settle_timeout_sec=0.40))
    )
    before = driver.tick(
        distance_to_terminal_m=math.nan,
        radial_error_m=math.nan,
        cross_track_error_m=math.inf,
        along_track_error_m=-math.inf,
        measured_linear_speed_mps=math.nan,
        measured_yaw_rate_radps=math.inf,
        heading_error_deg=math.nan,
    )
    assert before.state is TerminalState.APPROACH
    assert before.directive is TerminalDirective.HOLD_ZERO
    assert not before.motion_evidence_valid
    assert not before.zero_latched
    assert before.certificate is None

    capture(driver)
    invalid = driver.tick(
        radial_error_m=math.nan,
        cross_track_error_m=math.nan,
        measured_linear_speed_mps=math.inf,
    )
    assert invalid.directive is TerminalDirective.HOLD_ZERO
    assert invalid.settle_held_sec == 0.0
    assert invalid.certificate is None

    failed = driver.tick(elapsed=0.40, radial_error_m=math.nan)
    assert failed.state is TerminalState.HOLD_FAIL
    assert failed.directive is TerminalDirective.HOLD_FAIL


def test_certificate_preserves_distinct_errors_poses_truth_and_json_safety():
    driver = Driver(TerminalStopStateMachine(compact_config()))
    capture_pose = ControllerPose(10.0, -4.0, 0.25)
    final_pose = ControllerPose(10.002, -3.999, 0.24)
    capture(
        driver,
        radial_error_m=0.009,
        current_pose=capture_pose,
    )
    driver.tick(radial_error_m=0.009)
    driver.tick(radial_error_m=0.009)
    result = driver.tick(
        elapsed=0.11,
        radial_error_m=0.0074,
        cross_track_error_m=-0.0042,
        along_track_error_m=0.0061,
        heading_error_deg=-1.5,
        measured_linear_speed_mps=0.004,
        measured_yaw_rate_radps=-0.012,
        current_pose=final_pose,
    )
    assert result.state is TerminalState.CERTIFIED
    assert result.currently_valid
    certificate = result.certificate
    assert certificate.version == 2
    assert certificate.radial_error_mm == pytest.approx(7.4)
    assert certificate.cross_error_mm == pytest.approx(-4.2)
    assert certificate.along_error_mm == pytest.approx(6.1)
    assert certificate.stop_spec_mm == pytest.approx(10.0)
    assert certificate.first_capture_pose == capture_pose
    assert certificate.final_settled_pose == final_pose
    assert certificate.max_radial_during_settle_mm == pytest.approx(9.0)

    payload = certificate.to_dict()
    encoded = json.dumps(payload, allow_nan=False, sort_keys=True)
    assert "controller_estimator_frame_only" in encoded
    assert payload["localization_accuracy_certified"] is False
    assert payload["physical_accuracy_certified"] is False
    assert not any(
        isinstance(item, (TerminalState, TerminalDirective))
        for item in payload.values()
    )


def test_certificate_and_supporting_dataclasses_are_immutable():
    pose = ControllerPose(1.0, 2.0)
    config = TerminalConfig()
    assert config.stop_speed_tolerance_mps == pytest.approx(0.010)
    with pytest.raises(FrozenInstanceError):
        pose.x_m = 3.0
    with pytest.raises(FrozenInstanceError):
        config.terminal_radial_tolerance_m = 0.02

    driver = Driver(TerminalStopStateMachine(compact_config()))
    certificate = certify(driver).certificate
    with pytest.raises(FrozenInstanceError):
        certificate.precision_pass = False


@pytest.mark.parametrize(
    "overrides",
    [
        {"terminal_radial_tolerance_m": math.nan},
        {"terminal_radial_tolerance_m": 0.0},
        {
            "terminal_radial_tolerance_m": 0.010,
            "capture_entry_tolerance_m": 0.011,
        },
        {
            "terminal_radial_tolerance_m": 0.010,
            "settle_radial_tolerance_m": 0.011,
        },
        {"approach_distance_m": 0.20, "brake_distance_m": 0.30},
        {"settle_dwell_sec": -0.1},
        {"settle_dwell_sec": 0.30, "settle_timeout_sec": 0.30},
        {"settle_dwell_sec": 0.30, "settle_timeout_sec": 0.20},
        {"minimum_actuatable_speed_mps": -0.01},
        {"terminal_timeout_sec": "soon"},
    ],
)
def test_config_validation_rejects_nonfinite_unordered_or_negative(overrides):
    with pytest.raises(ValueError):
        compact_config(**overrides)


def test_dt_is_bounded_for_reporting_but_dwell_uses_monotonic_elapsed_time():
    driver = Driver(TerminalStopStateMachine(compact_config()), dt=1.0)

    approach = driver.tick(radial_error_m=0.20)
    assert approach.bounded_dt_sec == pytest.approx(0.10)

    capture(driver, elapsed=0.01)
    result = driver.tick(
        elapsed=0.31,
        dt_sec=math.inf,
        radial_error_m=0.008,
    )
    assert result.bounded_dt_sec == 0.0
    # State staging remains explicit even though monotonic dwell already passed.
    assert result.state is TerminalState.ZERO_LATCH
    assert result.settle_held_sec == pytest.approx(0.31)


def test_fresh_valid_motion_evidence_resumes_pre_latch_approach():
    driver = Driver(TerminalStopStateMachine(compact_config()))
    stale = driver.tick(radial_error_m=0.20, telemetry_fresh=False)
    assert stale.directive is TerminalDirective.HOLD_ZERO
    assert not stale.motion_evidence_valid

    resumed = driver.tick(radial_error_m=0.20, telemetry_fresh=True)
    assert resumed.state is TerminalState.APPROACH
    assert resumed.directive is TerminalDirective.APPROACH
    assert resumed.motion_evidence_valid
    assert not resumed.zero_latched


def test_default_speed_gate_rejects_eleven_mm_per_second_during_settle():
    driver = Driver(TerminalStopStateMachine(TerminalConfig()))
    enter_settle(driver)

    too_fast = driver.tick(
        radial_error_m=0.008,
        measured_linear_speed_mps=0.011,
    )

    assert too_fast.state is TerminalState.SETTLE
    assert too_fast.settle_held_sec == 0.0
    assert not too_fast.currently_valid
    assert too_fast.certificate is None


def test_monotonic_time_and_terminal_identity_contracts_are_strict():
    fsm = TerminalStopStateMachine(compact_config())
    valid = TerminalInput(
        monotonic_time_sec=1.0,
        dt_sec=0.1,
        terminal_requested=False,
        terminal_identity=None,
        distance_to_terminal_m=math.nan,
        radial_error_m=math.nan,
        cross_track_error_m=math.nan,
        along_track_error_m=math.nan,
        measured_linear_speed_mps=math.nan,
        measured_yaw_rate_radps=math.nan,
        telemetry_fresh=False,
    )
    fsm.step(valid)
    with pytest.raises(ValueError, match="backwards"):
        fsm.step(replace(valid, monotonic_time_sec=0.5))

    with pytest.raises(ValueError, match="terminal_identity"):
        TerminalStopStateMachine(compact_config()).step(
            replace(
                valid,
                monotonic_time_sec=0.0,
                terminal_requested=True,
            )
        )

    with pytest.raises(ValueError, match="radial_error_m must be numeric"):
        TerminalStopStateMachine(compact_config()).step(
            replace(
                valid,
                monotonic_time_sec=0.0,
                radial_error_m="invalid",
            )
        )


def test_certified_live_validity_remains_true_while_all_gates_hold():
    driver = Driver(TerminalStopStateMachine(compact_config()))
    certified = certify(driver)
    historical = certified.certificate

    heartbeat = driver.tick(
        radial_error_m=0.009,
        cross_track_error_m=0.004,
        along_track_error_m=0.005,
        measured_linear_speed_mps=0.005,
        measured_yaw_rate_radps=0.010,
    )

    assert heartbeat.state is TerminalState.CERTIFIED
    assert heartbeat.directive is TerminalDirective.HOLD_ZERO
    assert heartbeat.currently_valid
    assert heartbeat.certificate is historical


def test_certified_drift_revokes_live_validity_but_preserves_history():
    driver = Driver(TerminalStopStateMachine(compact_config()))
    certified = certify(driver)
    historical = certified.certificate

    drifted = driver.tick(radial_error_m=0.014)

    assert drifted.state is TerminalState.HOLD_FAIL
    assert drifted.directive is TerminalDirective.HOLD_FAIL
    assert drifted.zero_latched
    assert not drifted.currently_valid
    assert drifted.transition_reason == "certified_position_outside_tolerance"
    assert drifted.certificate is historical
    assert drifted.certificate.radial_error_mm == pytest.approx(8.0)


def test_certified_stale_telemetry_revokes_live_validity_zero_safely():
    driver = Driver(TerminalStopStateMachine(compact_config()))
    certified = certify(driver)

    stale = driver.tick(radial_error_m=0.008, telemetry_fresh=False)

    assert stale.state is TerminalState.HOLD_FAIL
    assert stale.directive is TerminalDirective.HOLD_FAIL
    assert stale.zero_latched
    assert not stale.currently_valid
    assert stale.transition_reason == "certified_telemetry_stale"
    assert stale.certificate is certified.certificate


def test_certificate_to_dict_rejects_manually_constructed_nonfinite_data():
    certificate = PrecisionTerminalCertificate(
        version=2,
        terminal_identity="mark-1",
        precision_pass=True,
        radial_error_mm=math.nan,
        cross_error_mm=1.0,
        along_error_mm=2.0,
        heading_error_deg=None,
        measured_speed_mps=0.0,
        measured_yaw_rate_radps=0.0,
        stop_spec_mm=10.0,
        settle_sec=0.3,
        first_capture_pose=None,
        final_settled_pose=None,
        max_radial_during_settle_mm=9.0,
        capture_timestamp_sec=1.0,
        settle_started_timestamp_sec=1.0,
        certified_timestamp_sec=1.3,
    )
    with pytest.raises(ValueError, match="non-finite"):
        certificate.to_dict()
