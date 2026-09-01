"""Focused guards for the fresh runtime C-to-P1 movement authority."""

from __future__ import annotations

import ast
import inspect
import math
from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "rpp_controller" / "rpp_controller_node.py"
NODE_SOURCE = NODE_PATH.read_text(encoding="utf-8")
NODE_TREE = ast.parse(NODE_SOURCE)
sys.path.insert(0, str(PACKAGE_ROOT))

from rpp_controller.runtime_entry import (  # noqa: E402
    select_runtime_entry_authority,
)


def _controller_method(name: str) -> ast.FunctionDef:
    controller = next(
        item
        for item in NODE_TREE.body
        if isinstance(item, ast.ClassDef) and item.name == "RPPController"
    )
    return next(
        item
        for item in controller.body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )


def _method_source(name: str) -> str:
    source = ast.get_source_segment(NODE_SOURCE, _controller_method(name))
    assert source is not None
    return source


def _load_methods(*names: str):
    namespace = {"math": math}
    module = ast.fix_missing_locations(
        ast.Module(body=[_controller_method(name) for name in names], type_ignores=[])
    )
    exec(compile(module, str(NODE_PATH), "exec"), namespace)
    return tuple(namespace[name] for name in names)


_lock_c_to_p1_line, _line_guidance = _load_methods(
    "lock_c_to_p1_line",
    "line_guidance",
)


class _Logger:
    def warn(self, _message):
        pass


class _Controller:
    lock_c_to_p1_line = _lock_c_to_p1_line
    line_guidance = _line_guidance

    def __init__(self):
        self.first_marking_completed = False
        self.marking_waypoints = [(10.0, 0.0)]
        self.current_x = 0.0
        self.current_y = 0.5
        self.c_line_start_x = None
        self.c_line_start_y = None
        self.c_line_bearing = None
        self.c_line_locked = False
        self.c_line_reanchored_after_pivot = False
        self.segment_alignment_active = False
        self.segment_alignment_pivot_complete = False
        self.segment_pivot_keeper_started_at = None
        self.alignment_inside_since = None
        self.line_tracking_lookahead = 0.55
        self.command_slew_speed = 1.0
        self.line_tracking_lookahead_speed_gain = 0.55
        self.line_tracking_lookahead_min = 0.35
        self.line_tracking_lookahead_max = 0.80
        self.line_tracking_lookahead_xtrack_gain = 1.0
        self.yaw_rate_feedforward_enabled = False
        self.yaw_rate_feedforward_heading_gain = 0.0
        self.terminal_native_pivot_active = False
        self.maximum_yaw_rate = 0.20
        self.last_yaw_rate_feedforward_radps = 0.0

    def _reset_legacy_alignment_lifecycle(self, _reason):
        self.segment_alignment_pivot_complete = False
        self.segment_pivot_keeper_started_at = None
        self.alignment_inside_since = None

    def get_logger(self):
        return _Logger()

    @staticmethod
    def normalize_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi


def test_start_after_lateral_ready_motion_uses_fresh_c_to_p1_line():
    controller = _Controller()
    stale_prepared_bearing = 0.0

    assert controller.lock_c_to_p1_line("test start")
    expected = math.atan2(-0.5, 10.0)
    assert math.isclose(controller.c_line_bearing, expected, abs_tol=1.0e-12)
    assert not math.isclose(controller.c_line_bearing, stale_prepared_bearing)

    stale_nav_solution = (0.55, 0.0, stale_prepared_bearing, 1, 11, 41)
    selected = select_runtime_entry_authority(
        stale_nav_solution,
        first_approach=True,
        p1_x=10.0,
        p1_y=0.0,
        c_to_p1_bearing=controller.c_line_bearing,
    )
    guidance, xtrack = controller.line_guidance(
        selected[2], selected[0], selected[1], math.radians(22.0)
    )

    assert selected[:3] == (10.0, 0.0, expected)
    assert math.isclose(xtrack, 0.0, abs_tol=1.0e-12)
    assert math.isclose(guidance, expected, abs_tol=1.0e-12)


def test_runtime_entry_selection_does_not_mutate_prepared_nav_path():
    controller = _Controller()
    prepared_nav_path = ((0.0, 0.0), (0.05, 0.0), (10.0, 0.0))
    nav_snapshot = tuple(prepared_nav_path)

    assert controller.lock_c_to_p1_line("test start")
    select_runtime_entry_authority(
        (0.55, 0.0, 0.0, 1, 2, 2),
        first_approach=True,
        p1_x=10.0,
        p1_y=0.0,
        c_to_p1_bearing=controller.c_line_bearing,
    )

    assert prepared_nav_path == nav_snapshot
    assert "nav_path_points" not in inspect.getsource(
        select_runtime_entry_authority
    )


def test_runtime_entry_selection_does_not_mutate_path_signature():
    controller = _Controller()
    path_signature = "sha256:prepared-static-path"

    assert controller.lock_c_to_p1_line("test start")
    select_runtime_entry_authority(
        (0.55, 0.0, 0.0, 1, 2, 2),
        first_approach=True,
        p1_x=10.0,
        p1_y=0.0,
        c_to_p1_bearing=controller.c_line_bearing,
    )

    assert path_signature == "sha256:prepared-static-path"
    selector = inspect.getsource(select_runtime_entry_authority)
    assert "path_signature" not in selector
    assert "geometry_installed_signature" not in selector


def test_nav_tangent_cannot_overwrite_first_approach_path_heading():
    control = _method_source("control_loop")
    selection = control.index("nav_solution = select_runtime_entry_authority(")
    unpack = control.index(") = nav_solution", selection)
    heading = control.index("path_heading_error =", unpack)
    pivot = control.index("abs(path_heading_error) >= self.pivot_enter_angle", heading)

    assert selection < unpack < heading < pivot


def test_precision_guidance_cannot_overwrite_first_approach_steering():
    control = _method_source("control_loop")

    assert (
        NODE_SOURCE.count("self.precision_guidance_enabled and not first_approach")
        >= 4
    )
    assert "self.precision_tracking_control_enabled and not first_approach" in control


def test_45_degree_pivot_decision_uses_fresh_runtime_heading():
    stale_nav_solution = (0.5, 0.0, 0.0, 1, 2, 2)
    fresh_bearing = math.atan2(-1.01, 1.0)
    selected = select_runtime_entry_authority(
        stale_nav_solution,
        first_approach=True,
        p1_x=1.0,
        p1_y=0.0,
        c_to_p1_bearing=fresh_bearing,
    )
    rover_yaw = 0.0
    pivot_enter = math.radians(45.0)

    assert abs(selected[2] - rover_yaw) >= pivot_enter
    assert abs(stale_nav_solution[2] - rover_yaw) < pivot_enter


def test_post_p1_nav_solution_is_returned_byte_for_byte_unchanged():
    p1_to_p2 = (1.55, 2.25, 0.375, 42, 52, 75)

    selected = select_runtime_entry_authority(
        p1_to_p2,
        first_approach=False,
        p1_x=999.0,
        p1_y=999.0,
        c_to_p1_bearing=-2.0,
    )

    assert selected is p1_to_p2


def test_runtime_entry_guidance_remains_forward_without_backtracking():
    controller = _Controller()
    assert controller.lock_c_to_p1_line("test start")
    path_bearing = controller.c_line_bearing

    for lateral_offset in (-2.0, -0.5, 0.0, 0.5, 2.0):
        controller.current_x = 2.0
        controller.current_y = lateral_offset
        guidance, _xtrack = controller.line_guidance(
            path_bearing,
            10.0,
            0.0,
            math.radians(22.0),
        )
        correction = controller.normalize_angle(guidance - path_bearing)
        assert abs(correction) <= math.radians(22.0) + 1.0e-12
        assert math.cos(correction) > 0.0
