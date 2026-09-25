"""Exercise the restored stop with the actual production launch parameters."""

import ast
import math
from pathlib import Path

import pytest

from rpp_controller.terminal_stop_regulator import (
    RadialStopConfig,
    RadialStopFailure,
    RadialStopInput,
    RadialStopState,
    TerminalStopRegulator,
)


@pytest.fixture
def launch_settings():
    launch = (
        Path(__file__).resolve().parents[2]
        / "rover_bringup/launch/rover.launch.py"
    )
    settings = {}
    for node in ast.walk(ast.parse(launch.read_text())):
        if not isinstance(node, ast.Dict):
            continue
        for key, value in zip(node.keys, node.values):
            if (
                isinstance(key, ast.Constant)
                and isinstance(key.value, str)
                and key.value.startswith("radial_stop_")
            ):
                assert key.value not in settings, "duplicate launch setting"
                settings[key.value] = ast.literal_eval(value)
    return settings


@pytest.fixture
def regulator(launch_settings):
    values = {
        key.removeprefix("radial_stop_"): value
        for key, value in launch_settings.items()
        if key != "radial_stop_telemetry_timeout_sec"
    }
    values["maximum_position_sample_gap_sec"] = values.pop(
        "max_position_sample_gap_sec"
    )
    return TerminalStopRegulator(RadialStopConfig(**values))


def sample(t, along, cross=0.0, speed=0.0):
    return RadialStopInput(
        monotonic_time_sec=t,
        position_sample_time_sec=t,
        active=True,
        terminal_identity="september11-mark",
        along_remaining_m=along,
        cross_error_m=cross,
        position_x_m=-along,
        position_y_m=cross,
        position_derived_speed_mps=speed,
        measured_yaw_rate_radps=0.0,
        tracking_speed_command_mps=1.0,
        telemetry_fresh=True,
    )


def test_production_launch_restores_september11_profile(launch_settings):
    assert launch_settings == {
        "radial_stop_radial_tolerance_m": 0.020,
        "radial_stop_terminal_guidance_distance_m": 0.75,
        "radial_stop_conservative_decel_mps2": 0.75,
        "radial_stop_brake_margin_m": 0.003,
        "radial_stop_stationary_window_sec": 0.50,
        "radial_stop_stationary_displacement_m": 0.005,
        "radial_stop_stationary_yaw_rate_radps": 0.050,
        "radial_stop_max_position_sample_gap_sec": 0.20,
        "radial_stop_terminal_timeout_sec": 15.0,
        "radial_stop_settle_timeout_sec": 5.0,
        "radial_stop_telemetry_timeout_sec": 0.25,
    }


def test_node_constructor_matches_restored_config_and_removed_params_are_absent():
    node_path = (
        Path(__file__).resolve().parents[1]
        / "rpp_controller/rpp_controller_node.py"
    )
    tree = ast.parse(node_path.read_text())
    constructors = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "RadialStopConfig"
    ]
    assert len(constructors) == 1
    assert {keyword.arg for keyword in constructors[0].keywords} == set(
        RadialStopConfig.__dataclass_fields__
    )
    removed_parameters = {
        "radial_stop_minimum_actuatable_speed_mps",
        "radial_stop_minimum_speed_stop_lead_m",
        "radial_stop_corrective_creep_speed_mps",
        "radial_stop_corrective_creep_pulse_sec",
        "radial_stop_corrective_creep_max_along_m",
        "radial_stop_corrective_creep_max_cross_m",
        "radial_stop_corrective_creep_max_attempts",
    }
    constants = {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }
    assert not constants.intersection(removed_parameters)


def test_approach_passes_old_leads_then_certifies_inside_circle(regulator):
    regulator.step(sample(0.0, 0.75, speed=1.0))
    for t, along in [(0.1, 0.035), (0.2, 0.022)]:
        output = regulator.step(sample(t, along, speed=0.20))
        assert output.state is RadialStopState.BRAKE_PROFILE
        assert not output.hold_zero
        assert output.forward_speed_command_mps == pytest.approx(
            math.sqrt(2 * 0.75 * (along - 0.003))
        )

    output = regulator.step(sample(0.3, 0.020))
    assert output.state is RadialStopState.ZERO_LATCH
    assert output.forward_speed_command_mps == 0.0
    states = []
    for tick in range(4, 13):
        output = regulator.step(sample(tick / 10, 0.020))
        states.append(output.state)
        assert output.hold_zero
        assert output.forward_speed_command_mps == 0.0
    assert RadialStopState.SETTLE in states
    assert RadialStopState.CERTIFIED in states
    assert output.certificate is not None
    assert output.certificate.radial_error_m == pytest.approx(0.020)


def test_profile_has_no_floor_before_goal_plane(regulator):
    regulator.step(sample(0.0, 0.75, cross=0.03, speed=1.0))
    output = regulator.step(sample(0.1, 0.0031, cross=0.03, speed=0.20))
    assert 0.0 < output.forward_speed_command_mps < 0.07
    assert not output.hold_zero
    output = regulator.step(sample(0.2, 0.0, cross=0.03))
    assert output.state is RadialStopState.ZERO_LATCH
    assert output.hold_zero


def test_settled_short_fails_and_never_retries(regulator):
    # Enter the circle, then settle 40 mm short after position correction.
    regulator.step(sample(0.0, 0.020))
    observed = []
    for tick in range(1, 31):
        output = regulator.step(sample(tick / 10, 0.040))
        observed.append(output.state)
        assert output.hold_zero
        assert output.forward_speed_command_mps == 0.0
        assert output.certificate is None
    assert RadialStopState.SETTLE in observed
    assert output.state is RadialStopState.HOLD_FAIL
    assert output.failure is RadialStopFailure.SETTLED_OUTSIDE_TOLERANCE
