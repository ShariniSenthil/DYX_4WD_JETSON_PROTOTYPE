"""Static Phase-5 ROS adapter guards; no ROS installation is required."""

from __future__ import annotations

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
        if isinstance(item, ast.FunctionDef) and item.name == name
    )
    return ast.get_source_segment(NODE, node)


def test_terminal_flag_defaults_off_and_preserves_legacy_30mm_contract():
    assert 'declare_parameter("precision_terminal_enabled", False)' in NODE
    assert '"precision_terminal_enabled": False' in LAUNCH
    assert '"precision_terminal_radial_tolerance_m", 0.010' in NODE
    assert '"waypoint_tolerance_m": 0.03' in LAUNCH
    assert "target_distance <= self.waypoint_tolerance" in function_source(
        "latch_exact_marking_stop"
    )


def test_terminal_gate_requires_complete_precision_stack_and_freshness_bound():
    validate = function_source("validate_parameters")
    for authority in (
        "geometry_tracking_enabled",
        "precision_guidance_enabled",
        "precision_speed_control_enabled",
        "precision_tracking_control_enabled",
        "precision_pivot_enabled",
    ):
        assert authority in validate
    assert "precision_terminal_telemetry_timeout_sec" in validate
    assert "<= self.odom_timeout_sec" in validate
    assert "precision_terminal_min_actuatable_speed_mps must equal" in validate
    assert (
        "if self.precision_terminal_enabled and not math.isclose(" in validate
    )
    ceiling_check = validate.index(
        "precision terminal minimum actuatable speed exceeds hardware ceiling"
    )
    assert "self.precision_terminal_enabled" in validate[
        max(0, ceiling_check - 300):ceiling_check
    ]
    assert "precision_tracking_control, and precision_pivot" in validate


def test_terminal_steps_once_with_current_projection_and_synchronized_identity():
    step = function_source("_step_precision_terminal_for_cycle")
    identity = function_source("_current_precision_terminal_identity")
    assert "precision_terminal_cycle_token == self.precision_cycle_token" in step
    assert "_current_cycle_projection()" in step
    assert "self.precision_guidance_result" in step
    assert "self.precision_terminal_fsm.step(sample)" in step
    assert "geometry_contract_synchronized" in identity
    for field in (
        "mission_run_id",
        "goal_instance_id",
        "path_signature",
        "raw_path_index",
        "active_goal_identity",
    ):
        assert field in identity


def test_10mm_capture_preempts_legacy_latch_and_every_moving_floor():
    control = function_source("control_loop")
    precision = control.index("_step_precision_terminal_for_cycle")
    legacy = control.index("latch_exact_marking_stop")
    first_move = control.index("publish_precision_velocity_ned")
    assert precision < legacy < first_move
    hold_branch = control[precision:legacy]
    assert "TerminalDirective.HOLD_ZERO" in hold_branch
    assert "self.publish_stop()" in hold_branch
    assert "return" in hold_branch
    # Legacy-latch mutual exclusion is keyed on self.legacy_terminal_stop_active
    # (terminal_stop_mode == "legacy") rather than the old bare
    # precision_terminal_enabled negation, since a third authority
    # (terminal_stop_mode == "radial20") must also be excluded from the
    # 30 mm latch -- see the plan review's mode-selector requirement (R1).
    assert control.count("self.legacy_terminal_stop_active") >= 2
    assert "and self.marking_stop_latched" in control
    assert "and self.latch_exact_marking_stop(" in control


def test_precision_mode_20mm_does_not_enter_legacy_30mm_latch():
    """The 10--30 mm annulus remains moving under precision authority."""

    control = function_source("control_loop")
    assert '"precision_terminal_radial_tolerance_m", 0.010' in NODE
    assert '"waypoint_tolerance_m": 0.03' in LAUNCH
    assert (
        "self.legacy_terminal_stop_active\n"
        "            and goal_requires_precision_stop\n"
        "            and self.latch_exact_marking_stop("
    ) in control


def test_radial20_mode_defaults_off_and_is_mutually_exclusive_with_every_other_authority():
    """terminal_stop_mode's own code-level default is legacy (a source
    property that must hold regardless of deployment config); the launch
    file's TERMINAL_STOP_MODE constant is a deliberate, operator-set
    deployment choice (currently radial20 for field testing) and is not
    asserted here -- only that both nodes reference the one shared constant,
    never a hardcoded per-node value that could drift apart (R1)."""

    assert 'declare_parameter("terminal_stop_mode", "legacy")' in NODE
    assert LAUNCH.count('"terminal_stop_mode": TERMINAL_STOP_MODE') == 2
    assert '"radial_stop_radial_tolerance_m", 0.020' in NODE

    control = function_source("control_loop")
    precision = control.index("_step_precision_terminal_for_cycle")
    radial20 = control.index("_step_radial20_terminal_for_cycle")
    legacy = control.index("latch_exact_marking_stop")
    # radial20 is evaluated after the Phase-5 FSM branch and before the
    # legacy 30 mm latch, matching the Phase-5 branch's own ordering
    # guarantee -- see test_10mm_capture_preempts_legacy_latch_and_every_moving_floor.
    assert precision < radial20 < legacy
    radial20_branch = control[radial20:legacy]
    assert "self.radial20_active" in control[:radial20]
    assert "RadialStopMotionDirection.ZERO" in radial20_branch
    assert "self.publish_stop()" in radial20_branch
    assert "return" in radial20_branch
    assert "self.terminal_bounded_guidance(" in radial20_branch
    assert "hard_speed_cap_mps=speed" in radial20_branch

    # The generic pre-Phase-5 deceleration zone must not also run for
    # radial20, or the legacy speed/guidance profile would briefly compete
    # with the new regulator in the terminal_goal_intercept_distance_m
    # (0.90 m) to radial_stop_terminal_guidance_distance_m (0.75 m) gap.
    terminal_active_source = control[
        control.index("terminal_active = (") : control.index(
            "terminal_active = ("
        )
        + 400
    ]
    assert "not self.radial20_active" in terminal_active_source

    result = function_source("publish_terminal_result")
    assert "if self.radial20_active:" in result
    assert "elif self.precision_terminal_enabled:" in result
    reset = function_source("_reset_precision_terminal")
    assert "self._reset_radial20_terminal(reason)" in reset


def test_terminal_override_is_additive_and_visible_in_speed_debug():
    resolver = function_source("_resolve_precision_speed_for_cycle")
    debug = function_source("_publish_speed_debug")
    publisher = function_source("publish_precision_velocity_ned")
    assert "terminal_target_speed_override_mps" in resolver
    assert '"effective_terminal_target_speed_mps"' in debug
    assert "max(resolved_speed, self.precision_minimum_moving_speed)" in publisher
    assert "TerminalDirective" not in publisher


def test_terminal_resets_are_semantic_not_transient_stale():
    stale = function_source("_step_precision_terminal_stale_cycle")
    assert "telemetry_fresh=False" in stale
    assert "_reset_precision_terminal" not in stale
    for reason in (
        "SEGMENT_GOAL_CHANGED",
        "SEGMENT_GOAL_IDENTITY_CHANGED",
        "MISSION_ENABLED",
        "MOTION_STATE_RESET",
        "EMERGENCY_STOP",
        "PATH_INSTALLED",
        "GEOMETRY_INVALIDATED",
        "LOCALIZATION_JUMP",
    ):
        assert reason in NODE


def test_heartbeat_is_guarded_and_has_live_and_immutable_evidence():
    heartbeat = function_source("_publish_precision_terminal_heartbeat")
    assert '"/rpp/terminal_certificate"' in NODE
    assert "try:" in heartbeat and "except Exception" in heartbeat
    assert "except Exception:\n                pass" in heartbeat
    for field in (
        "state",
        "directive",
        "zero_latched",
        "motion_evidence_valid",
        "currently_valid",
        "terminal_identity_components",
        "ros_time_ns",
        "certificate",
        "precision_certificate_version",
        "precision_pass",
        "telemetry_fresh",
    ):
        assert f'"{field}"' in heartbeat


def test_precision_result_keeps_legacy_fields_and_appends_v2_certificate():
    result = function_source("publish_terminal_result")
    for legacy in (
        "outcome",
        "radial_error_m",
        "cross_track_error_m",
        "along_track_remaining_m",
        "tolerance_m",
        "within_tolerance",
        "stop_commanded",
    ):
        assert f'"{legacy}"' in result
    for additive in (
        "controller_outcome",
        "precision_certificate_version",
        "terminal_identity",
        "mission_run_id",
        "goal_instance_id",
        "path_signature",
        "raw_path_index",
        "active_goal_identity",
        "precision_pass",
        "cross_error_mm",
        "along_error_mm",
        "stop_spec_mm",
        "precision_stop_spec_mm",
        "measured_yaw_rate_radps",
        "speed_at_release_mps",
        "yaw_rate_at_release_radps",
        "telemetry_fresh",
        "settle_sec",
        "max_radial_during_settle_mm",
        "first_capture_pose",
        "final_settled_pose",
        "truth_frame",
        "precision_certificate",
    ):
        assert f'"{additive}"' in result


def test_marking_and_extension_share_terminal_authority_and_pivot_is_untouched():
    control = function_source("control_loop")
    pivot = function_source("_run_precision_pivot_alignment")
    assert "goal_requires_precision_stop = True" in control
    assert "goal_is_extension = not goal_is_marking" in control
    assert "precision_terminal_enabled" in control
    assert "precision_terminal" not in pivot
