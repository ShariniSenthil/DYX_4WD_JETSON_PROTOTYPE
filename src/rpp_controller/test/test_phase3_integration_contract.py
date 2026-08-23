"""Static Phase-3 ROS integration guards without requiring ROS imports."""

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


def _declared_defaults() -> dict[str, object]:
    declarations = {}
    for node in ast.walk(_method("__init__")):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "declare_parameter"
            and len(node.args) >= 2
            and isinstance(node.args[0], ast.Constant)
        ):
            declarations[str(node.args[0].value)] = ast.literal_eval(node.args[1])
    return declarations


def test_phase3_gate_is_default_off_in_node_and_launch():
    assert _declared_defaults()["precision_pivot_enabled"] is False
    assert '"precision_pivot_enabled": False' in LAUNCH_SOURCE
    validation = _method_source("validate_parameters")
    assert "self.precision_pivot_enabled" in validation
    assert "not self.geometry_tracking_enabled" in validation


def test_recenter_threshold_defaults_to_and_is_validated_equal_to_anchor():
    defaults = _declared_defaults()
    assert defaults["precision_pivot_anchor_tolerance_m"] == 0.030
    assert defaults["precision_pivot_recenter_threshold_m"] == 0.030
    assert '"precision_pivot_recenter_threshold_m": 0.030' in LAUNCH_SOURCE
    validation = _method_source("validate_parameters")
    assert "pivot_recenter_threshold_m" in validation
    assert "pivot_anchor_tolerance_m" in validation
    assert "math.isclose(" in validation


def test_every_phase3_adapter_parameter_is_explicit_in_launch():
    expected = {
        name
        for name in _declared_defaults()
        if name.startswith("precision_pivot_") or name == "post_pivot_capture_speed_mps"
    }
    launch_keys = {
        str(node.value)
        for node in ast.walk(ast.parse(LAUNCH_SOURCE))
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }
    assert expected <= launch_keys


def test_legacy_odom_acceptance_does_not_depend_on_angular_z_finiteness():
    callback = _method_source("odom_callback")
    assert "angular = msg.twist.twist.angular" in callback
    assert "yaw_rate = float(angular.z)" in callback
    unconditional_gate = callback[
        callback.index("if not all(") : callback.index("try:")
    ]
    assert "yaw_rate" not in unconditional_gate
    assert "yaw_rate if math.isfinite(yaw_rate) else math.inf" in callback
    assignment = callback.index("self.current_yaw_rate_radps = yaw_rate")
    timestamp = callback.index("self.last_odom_time = self.get_clock().now()")
    assert assignment < timestamp


def test_invalid_or_stale_yaw_rate_stops_before_fsm_or_motion_mapping():
    source = _method_source("_run_precision_pivot_alignment")
    freshness = source.index("and math.isfinite(self.current_yaw_rate_radps)")
    stale_gate = source.index("if not telemetry_fresh:", freshness)
    stop = source.index("self.publish_stop()", stale_gate)
    early_return = source.index("return True", stop)
    fsm_step = source.index("self.precision_pivot_fsm.step(")
    first_directive_mapping = source.index("MotionDirective.HOLD_FAIL")
    assert freshness < stale_gate < stop < early_return < fsm_step
    assert early_return < first_directive_mapping


def test_legacy_carrier_method_remains_byte_exact():
    digest = hashlib.sha256(
        _method_source("terminal_native_pivot_command").encode("utf-8")
    ).hexdigest()
    assert digest == "89de9ecbc275c72378c428de25e4aa19b9e72af531a65271949ce28d1cb0790f"


def test_precision_carrier_has_no_legacy_4deg_auto_release():
    source = _method_source("precision_pivot_carrier_command")
    assert "terminal_native_pivot_release_error" not in source
    assert "terminal_native_pivot_request_error" in source
    assert "self.current_yaw + carrier_error" in source
    assert "publish_precision_velocity_ned" not in source


def test_fsm_directives_own_persistent_stop_and_native_carrier_mapping():
    source = _method_source("_run_precision_pivot_alignment")
    assert "MotionDirective.HOLD_ZERO" in source
    assert "MotionDirective.HOLD_FAIL" in source
    assert source.count("self.publish_stop()") >= 5
    assert "self.precision_pivot_carrier_command(" in source
    assert "self.publish_velocity_ned(" in source
    assert "publish_precision_velocity_ned" not in source


def test_brake_and_recenter_share_bounded_forward_cone_anchor_approach():
    routing = _method_source("_run_precision_pivot_alignment")
    approach = _method_source("_publish_precision_anchor_approach")
    assert "MotionDirective.BRAKE_TO_ANCHOR" in routing
    assert "MotionDirective.RECENTER" in routing
    assert "self._publish_precision_anchor_approach()" in routing
    assert "precision_pivot_recenter_forward_cone" in approach
    assert "hard_speed_cap_mps=speed" in approach


def test_p1_reanchor_occurs_only_after_release_certificate():
    source = _method_source("_run_precision_pivot_alignment")
    certificate = source.index("result.release_certificate.valid")
    guard = source.index("if not self.precision_pivot_release_certified")
    reanchor = source.index("self.reanchor_c_to_p1_after_pivot()")
    assert certificate < guard < reanchor


def test_semantic_anchor_latch_and_midleg_exception_are_explicit():
    goal = _method_source("segment_goal_callback")
    control = _method_source("control_loop")
    ensure = _method_source("_ensure_precision_pivot_anchor")
    assert "self.segment_start_x" in goal
    assert "C_TO_P1_START" in goal
    assert "SEMANTIC_SEGMENT_START" in ensure
    assert "MID_LEG_REENTRY_CURRENT_POSE" in control


def test_phase3_reset_boundaries_are_explicit():
    for reason in (
        "GEOMETRY_INVALIDATED",
        "PATH_INSTALLED",
        "LOCALIZATION_JUMP",
        "SEGMENT_GOAL_CHANGED",
        "MISSION_ENABLED",
        "MOTION_STATE_RESET",
        "EMERGENCY_STOP",
        "MARKING_HOLD",
        "POINT_COMPLETED",
    ):
        assert f'"{reason}"' in NODE_SOURCE


def test_pivot_debug_is_guarded_and_exposes_measured_evidence():
    source = _method_source("_publish_pivot_debug")
    assert "try:" in source
    assert "except Exception" in source
    for field in (
        "anchor_identity",
        "anchor_radial_error_m",
        "measured_speed_mps",
        "measured_yaw_rate_radps",
        "telemetry_fresh",
        "stop_certificate_valid",
        "release_certificate_valid",
        "max_pivot_drift_m",
        "recenter_attempts",
    ):
        assert f'"{field}"' in source


def test_recapture_is_speed_bounded_and_resets_both_longitudinal_owners():
    source = _method_source("_run_precision_pivot_alignment")
    assert "speed = self.post_pivot_capture_speed" in source
    assert "hard_speed_cap_mps=speed" in source
    completion = source.index("PRECISION_PIVOT_RECAPTURE_COMPLETE")
    assert source.rfind("self.reset_speed_profiles()", 0, completion) >= 0
    assert source.rfind("self.command_slew_speed = 0.0", 0, completion) >= 0
    assert "self.publish_stop()" in source[completion:]
