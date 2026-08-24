"""Static integration guards for Gate-2 control-authority ordering."""

from __future__ import annotations

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "rpp_controller" / "rpp_controller_node.py"
NODE_SOURCE = NODE_PATH.read_text(encoding="utf-8")
NODE_TREE = ast.parse(NODE_SOURCE)


def _method_source(name: str) -> str:
    controller = next(
        item
        for item in NODE_TREE.body
        if isinstance(item, ast.ClassDef) and item.name == "RPPController"
    )
    method = next(
        item
        for item in controller.body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )
    source = ast.get_source_segment(NODE_SOURCE, method)
    assert source is not None
    return source


def _control_region(start_marker: str, end_marker: str) -> str:
    control = _method_source("control_loop")
    start = control.index(start_marker)
    end = control.index(end_marker, start)
    return control[start:end]


def test_legacy_recovery_branch_does_not_force_precision_speed_recovery():
    recovery = _control_region(
        "if (\n            not self.precision_tracking_control_enabled",
        "# CONTINUOUS TWO-METRE TERMINAL APPROACH",
    )

    assert "and not terminal_active" in recovery
    assert "recovery_requested" not in recovery
    assert "_resolve_precision_speed_for_cycle()" in recovery
    assert "publish_precision_velocity_ned" in recovery


def test_longitudinal_regulator_is_the_only_precision_recovery_state_owner():
    resolver_path = PACKAGE_ROOT / "rpp_controller" / "speed_regulator.py"
    resolver = resolver_path.read_text(encoding="utf-8")
    node_resolver = _method_source("_resolve_precision_speed_for_cycle")

    assert "self._recovery_active" in resolver
    assert "heading_error >= config.heading_recovery_full_rad" in resolver
    assert "cross_track >= config.cross_track_recovery_full_m" in resolver
    assert "heading_error <= config.heading_recovery_start_rad" in resolver
    assert "cross_track <= config.cross_track_recovery_start_m" in resolver
    assert "recovery_requested" not in node_resolver


def test_recovery_contract_is_above_generic_moving_floor():
    validation = _method_source("validate_parameters")

    assert "precision_speed_config.recovery_min_speed_mps" in validation
    assert "<= self.precision_minimum_moving_speed" in validation
    assert (
        "precision_recovery_min_speed_mps must be greater than" in validation
    )


def test_terminal_bounded_bearing_cannot_be_overwritten_by_precision_lookahead():
    terminal = _control_region(
        "if terminal_active:",
        "# Normal pass-through and non-terminal movement.",
    )

    bounded = terminal.index("guidance_bearing = self.terminal_bounded_guidance")
    publication = terminal.index("self.publish_velocity_ned(", bounded)
    authority_region = terminal[bounded:publication]
    assert "precision_guidance.limited_command_bearing_rad" not in authority_region
    assert "precision_guidance.steering_target_point" not in terminal
    assert "endpoint_extension" not in terminal


def test_terminal_speed_cannot_be_overridden_by_generic_precision_resolver():
    terminal = _control_region(
        "if terminal_active:",
        "# Normal pass-through and non-terminal movement.",
    )

    assert "_resolve_precision_speed_for_cycle" not in terminal
    assert "publish_precision_velocity_ned" not in terminal
    assert "apply_deceleration=True" in terminal
    assert "hard_speed_cap_mps=terminal_speed_cap" in terminal


def test_gate2_terminal_authority_latches_after_endpoint_entry():
    control = _method_source("control_loop")
    terminal_selection = control[
        control.index("gate2_active =") : control.index(
            "# Preserve xtrack speed-cap state", control.index("gate2_active =")
        )
    ]

    assert "or (gate2_active and self.terminal_precision_armed)" in terminal_selection
    assert "goal_distance <= self.terminal_goal_intercept_distance" in (
        terminal_selection
    )


def test_zero_latches_precede_every_gate2_moving_authority():
    control = _method_source("control_loop")

    legacy_zero = control.index("self.latch_exact_marking_stop(")
    terminal_authority = control.index("if terminal_active:")
    normal_precision = control.rindex("self.publish_precision_velocity_ned(")
    assert legacy_zero < terminal_authority < normal_precision


def test_gate2_off_retains_legacy_guidance_and_terminal_publisher():
    normal = _control_region(
        "# Normal pass-through and non-terminal movement.",
        "self.log_control(\n            status,",
    )
    terminal = _control_region(
        "if terminal_active:",
        "# Normal pass-through and non-terminal movement.",
    )

    assert "self.line_guidance(" in normal
    assert "if self.precision_guidance_enabled:" in normal
    assert "if self.precision_speed_control_enabled:" in normal
    assert "self.publish_velocity_ned(" in terminal
