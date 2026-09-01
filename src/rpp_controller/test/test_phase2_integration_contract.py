"""Static Phase-2 integration guards that do not require a ROS installation."""

from __future__ import annotations

import ast
import hashlib
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
NODE_PATH = PACKAGE_ROOT / "rpp_controller" / "rpp_controller_node.py"
LAUNCH_PATH = REPOSITORY_ROOT / "src" / "rover_bringup" / "launch" / "rover.launch.py"
NODE_SOURCE = NODE_PATH.read_text(encoding="utf-8")
LAUNCH_SOURCE = LAUNCH_PATH.read_text(encoding="utf-8")
NODE_TREE = ast.parse(NODE_SOURCE)


def _controller_class() -> ast.ClassDef:
    return next(
        item
        for item in NODE_TREE.body
        if isinstance(item, ast.ClassDef) and item.name == "RPPController"
    )


def _method(name: str) -> ast.FunctionDef:
    return next(
        item
        for item in _controller_class().body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )


def _method_source(name: str) -> str:
    source = ast.get_source_segment(NODE_SOURCE, _method(name))
    assert source is not None
    return source


def _called_attributes(method_name: str) -> set[str]:
    attributes = set()
    for node in ast.walk(_method(method_name)):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            attributes.add(node.func.attr)
    return attributes


def test_accepted_guidance_and_speed_control_default_on_while_tracking_stays_off():
    # precision_speed_control_enabled was promoted from the launch file's
    # own staged rollout (see the launch file's comment at this param):
    # guidance first, speed control next -- both are now live, tracking/
    # pivot/terminal remain off pending their own separate promotion.
    declarations = {}
    for node in ast.walk(_method("__init__")):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "declare_parameter"
            and len(node.args) >= 2
            and isinstance(node.args[0], ast.Constant)
        ):
            continue
        declarations[node.args[0].value] = ast.literal_eval(node.args[1])

    # Node-level defaults stay safe (off) regardless of what the launch
    # file overrides -- only the launch file's explicit values are promoted.
    assert declarations["precision_guidance_enabled"] is True
    assert declarations["precision_speed_control_enabled"] is False
    assert declarations["precision_lookahead_time_s"] == 0.90
    assert '"precision_guidance_enabled": True' in LAUNCH_SOURCE
    assert '"precision_speed_control_enabled": True' in LAUNCH_SOURCE
    assert '"precision_tracking_control_enabled": False' in LAUNCH_SOURCE
    assert '"precision_pivot_enabled": False' in LAUNCH_SOURCE
    assert '"precision_terminal_enabled": False' in LAUNCH_SOURCE
    assert '"precision_lookahead_time_s": 0.90' in LAUNCH_SOURCE


def test_every_phase2_parameter_is_explicitly_wired_in_launch():
    declared = set()
    for node in ast.walk(_method("__init__")):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "declare_parameter"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and str(node.args[0].value).startswith("precision_")
        ):
            declared.add(str(node.args[0].value))

    launch_keys = {
        str(node.value)
        for node in ast.walk(ast.parse(LAUNCH_SOURCE))
        if isinstance(node, ast.Constant)
        and isinstance(node.value, str)
        and node.value.startswith("precision_")
    }
    assert declared
    assert declared <= launch_keys


def test_precision_features_require_primary_geometry_tracking():
    validation = _method_source("validate_parameters")
    assert "precision_guidance_enabled" in validation
    assert "precision_speed_control_enabled" in validation
    assert "not self.geometry_tracking_enabled" in validation
    assert "require geometry_tracking_enabled=true" in validation


def test_phase2_cannot_create_an_early_terminal_zero():
    init_source = _method_source("__init__")
    publication = _method_source("publish_precision_velocity_ned")
    control = _method_source("control_loop")

    assert '"precision_terminal_target_speed_mps", 0.15' in init_source
    assert '"precision_minimum_moving_speed_mps", 0.04' in init_source
    assert '"waypoint_tolerance_m": 0.03' in LAUNCH_SOURCE
    assert "max(resolved_speed, self.precision_minimum_moving_speed)" in publication
    assert control.index("latch_exact_marking_stop") < control.index(
        "publish_precision_velocity_ned"
    )


def test_precision_publication_bypasses_competing_legacy_profiles():
    precision_calls = _called_attributes("publish_precision_velocity_ned")
    forbidden = {
        "acceleration_speed_limit",
        "deceleration_speed_limit",
        "command_speed_slew_limit",
        "publish_velocity_ned",
    }
    assert precision_calls.isdisjoint(forbidden)

    legacy_calls = _called_attributes("publish_velocity_ned")
    assert {
        "acceleration_speed_limit",
        "deceleration_speed_limit",
        "command_speed_slew_limit",
    }.issubset(legacy_calls)


def test_legacy_publisher_and_native_pivot_math_remain_byte_exact():
    expected_hashes = {
        "publish_velocity_ned": (
            "1f179fa785d59686aa3f449e0bec29722d75ea2a6999644bdfeb0b02318a6123"
        ),
        "terminal_native_pivot_command": (
            "ab1a69086a10d69a3719dea04fdfd772887dfec02ee318020c47e93b3e0cea00"
        ),
    }
    for method_name, expected in expected_hashes.items():
        digest = hashlib.sha256(
            _method_source(method_name).encode("utf-8")
        ).hexdigest()
        assert digest == expected


def test_native_pivot_carrier_remains_on_legacy_publication_path():
    control = _method_source("control_loop")
    adapter = _method_source("_run_legacy_segment_alignment")
    carrier = _method_source("_publish_legacy_native_carrier")

    assert "self._run_legacy_segment_alignment(" in control
    assert "LegacyAlignmentDirective.NATIVE_CARRIER" in adapter
    assert "self.publish_velocity_ned(" in carrier
    assert "apply_acceleration=False" in carrier
    assert "apply_deceleration=False" in carrier
    assert "publish_precision_velocity_ned" not in carrier
    assert "publish_precision_velocity_ned" not in adapter[
        adapter.index("NATIVE_CARRIER") : adapter.index("HOLD_ZERO")
    ]


def test_projection_and_regulator_results_are_current_cycle_scoped():
    begin = _method_source("_begin_precision_cycle")
    projection = _method_source("_current_cycle_projection")
    geometry = _method_source("_geometry_tracking_solution")
    resolver = _method_source("_resolve_precision_speed_for_cycle")

    assert "self.precision_cycle_token += 1" in begin
    assert "self.geometry_last_projection = None" in begin
    assert "self.geometry_last_projection_cycle_token = None" in begin
    assert "!= self.precision_cycle_token" in projection
    assert (
        "self.geometry_last_projection_cycle_token = self.precision_cycle_token"
        in geometry
    )
    assert "self.precision_speed_cycle_token == self.precision_cycle_token" in resolver
    assert "hard_zero=False" in resolver


def test_speed_debug_contains_every_cap_and_winner():
    debug = _method_source("_publish_speed_debug")
    for field in (
        "winning_cap_owner",
        "resolver_winning_cap_owner",
        "mission_mps",
        "hardware_mps",
        "recovery_mps",
        "acceleration_mps",
        "heading_mps",
        "cross_track_mps",
        "corner_mps",
        "terminal_mps",
        "curvature_mps",
    ):
        assert f'"{field}"' in debug


def test_guidance_debug_exposes_endpoint_extension_and_final_angles():
    debug = _method_source("_publish_guidance_debug")

    for field in (
        "requested_lookahead_m",
        "actual_steering_target_distance_m",
        "endpoint_extension_used",
        "endpoint_extension_distance_m",
        "target_behind_rover",
        "path_heading_error_rad",
        "lookahead_heading_error_rad",
        "final_command_correction_rad",
    ):
        assert f'"{field}"' in debug


def test_precision_reset_boundaries_are_explicit():
    for reason in (
        "LITERAL_STOP",
        "MISSION_ENABLED",
        "MOTION_STATE_RESET",
        "SEGMENT_GOAL_CHANGED",
        "PATH_INSTALLED",
        "GEOMETRY_INVALIDATED",
        "LOCALIZATION_JUMP",
        "PIVOT_COMPLETE_RECAPTURE_ARMED",
        "PIVOT_RECAPTURE_COMPLETE",
    ):
        assert reason in NODE_SOURCE
