"""Static integration guards for the default-off Phase-4 ROS adapter."""

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
NODE_PATH = PACKAGE_ROOT / "rpp_controller/rpp_controller_node.py"
LAUNCH_PATH = REPOSITORY_ROOT / "src/rover_bringup/launch/rover.launch.py"
NODE = NODE_PATH.read_text(encoding="utf-8")
LAUNCH = LAUNCH_PATH.read_text(encoding="utf-8")
TREE = ast.parse(NODE)


def function_source(name):
    node = next(
        item
        for item in ast.walk(TREE)
        if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
        and item.name == name
    )
    return ast.get_source_segment(NODE, node)


def test_flag_is_default_off_in_node_and_launch():
    assert 'declare_parameter("precision_tracking_control_enabled", False)' in NODE
    assert '"precision_tracking_control_enabled": False' in LAUNCH
    assert '"precision_tracking_histogram_bin_width_m", 0.001' in NODE
    assert '"precision_tracking_histogram_max_m", 1.0' in NODE


def test_tracking_gate_requires_all_precision_authorities():
    validate = function_source("validate_parameters")
    assert "precision_tracking_control_enabled" in validate
    assert "geometry_tracking_enabled" in validate
    assert "precision_guidance_enabled" in validate
    assert "precision_speed_control_enabled" in validate


def test_tracking_uses_current_cycle_projection_guidance_and_shared_dt():
    compute = function_source("_compute_precision_tracking_for_cycle")
    assert "_current_cycle_projection()" in compute
    assert "_compute_precision_guidance_for_cycle()" in compute
    assert "precision_cycle_dt_sec" in compute
    assert "precision_last_published_translational_speed_mps" in compute
    assert "geometry_installed_signature" in compute


def test_tracking_enabled_bypasses_legacy_xtrack_state_mutations():
    control = function_source("control_loop")
    assert (
        "self.precision_tracking_control_enabled and not first_approach"
        in control
    )
    assert "if precision_tracking_authority:" in control
    assert "else:\n            (\n                xtrack_guidance_bearing" in control
    assert "else:\n            (\n                xtrack_speed_cap_active" in control
    assert "not precision_tracking_authority" in control


def test_tracking_cap_and_acceleration_permission_enter_speed_resolver():
    resolver = function_source("_resolve_precision_speed_for_cycle")
    debug = function_source("_publish_speed_debug")
    assert "tracking_acceleration_allowed" in resolver
    assert "tracking_speed_cap_mps" in resolver
    assert '"tracking_mps": caps.tracking_mps' in debug


def test_metrics_use_actual_published_command_and_raw_projection_s():
    publish = function_source("publish_precision_velocity_ned")
    metrics = function_source("_record_precision_tracking_metrics")
    assert "_record_precision_tracking_metrics(output_speed)" in publish
    assert "projection_s_m=projection.projected_s" in metrics
    assert "commanded_speed_mps=float(published_speed_mps)" in metrics
    assert "precision_tracking_metrics.add" in metrics


def test_metrics_and_stability_have_required_reset_boundaries():
    mission = function_source("mission_enable_callback")
    install = function_source("_try_install_path_geometry")
    invalidate = function_source("_invalidate_installed_geometry")
    odom = function_source("odom_callback")
    pivot = function_source("_run_precision_pivot_alignment")
    for source in (mission, install, invalidate, odom, pivot):
        assert "_reset_precision_tracking" in source
    assert 'reset_metrics=False' in pivot
    assert '"LOCALIZATION_JUMP"' in odom
    assert 'reset_metrics=False' in odom
    assert 'reset_metrics=True' not in odom
    assert 'precision_tracking_metrics.note_discontinuity' in odom
    localization_tail = odom.split('"LOCALIZATION_JUMP"', 1)[1]
    assert "note_discontinuity" in localization_tail


def test_tracking_debug_is_guarded_and_bounded_metrics_are_configured():
    debug = function_source("_publish_tracking_debug")
    assert '"/rpp/tracking_debug"' in NODE
    assert "try:" in debug and "except Exception" in debug
    assert '"physical_ground_truth_certified": False' in debug
    assert '"quantile_window_capacity"' in debug
    assert '"raw_projection_monotonic_violations"' in debug
    assert '"whole_run_p95_abs_cross_track_mm"' in debug
    assert '"trailing_p95_abs_cross_track_mm"' in debug
    assert '"histogram_overflow_count"' in debug
    assert '"p95_histogram_saturated"' in debug
    assert '"discontinuity_count"' in debug
    assert '"last_discontinuity_reason"' in debug
    assert '"valid_for_acceptance"' in debug
    assert '"precision_tracking_metrics_capacity": 2048' in LAUNCH
    assert '"precision_tracking_histogram_bin_width_m": 0.001' in LAUNCH
    assert '"precision_tracking_histogram_max_m": 1.0' in LAUNCH


def test_tracking_cache_is_initialized_once_and_acceptance_is_diagnostic_only():
    initializer = function_source("__init__")
    debug = function_source("_publish_tracking_debug")
    control = function_source("control_loop")
    assert initializer.count("self.precision_tracking_cycle_token = None") == 1
    assert initializer.count("self.precision_tracking_output = None") == 1
    assert initializer.count("self.precision_tracking_input = None") == 1
    assert "valid_for_acceptance" in debug
    assert "valid_for_acceptance" not in control


def test_phase4_does_not_change_terminal_latch_or_native_pivot_publish_path():
    latch = function_source("latch_exact_marking_stop")
    pivot = function_source("_run_precision_pivot_alignment")
    assert "target_distance <= self.waypoint_tolerance" in latch
    assert "precision_terminal" not in latch
    assert "publish_precision_velocity_ned" not in pivot
    assert "publish_velocity_ned" in pivot
