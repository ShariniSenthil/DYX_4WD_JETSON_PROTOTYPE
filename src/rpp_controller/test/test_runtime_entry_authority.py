"""Regression tests for local-odom START->P1 runtime-entry authority."""

from __future__ import annotations

import ast
import json
import math
from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "rpp_controller" / "rpp_controller_node.py"
NODE_SOURCE = NODE_PATH.read_text(encoding="utf-8")
NODE_TREE = ast.parse(NODE_SOURCE)

sys.path.insert(0, str(PACKAGE_ROOT))

from rpp_controller.runtime_entry import (  # noqa: E402
    build_runtime_entry_path,
    track_runtime_entry_path,
)


def _controller_class() -> ast.ClassDef:
    return next(
        item
        for item in NODE_TREE.body
        if isinstance(item, ast.ClassDef) and item.name == "RPPController"
    )


def _controller_method(name: str) -> ast.FunctionDef:
    return next(
        item
        for item in _controller_class().body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )


def _method_source(name: str) -> str:
    source = ast.get_source_segment(NODE_SOURCE, _controller_method(name))
    assert source is not None
    return source


class _String:
    def __init__(self):
        self.data = ""


def _load_methods(*names: str):
    namespace = {"json": json, "math": math, "String": _String}
    module = ast.fix_missing_locations(
        ast.Module(
            body=[_controller_method(name) for name in names],
            type_ignores=[],
        )
    )
    exec(compile(module, str(NODE_PATH), "exec"), namespace)
    return tuple(namespace[name] for name in names)


(
    _finite_or_none,
    _publish_reanchor_debug,
    _lock_c_to_p1_line,
    _reanchor_c_to_p1_after_pivot,
    _reanchor_runtime_path_after_pivot,
) = _load_methods(
    "_finite_or_none",
    "_publish_reanchor_debug",
    "lock_c_to_p1_line",
    "reanchor_c_to_p1_after_pivot",
    "reanchor_runtime_path_after_pivot",
)


class _Logger:
    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class _Publisher:
    def __init__(self, fail=False):
        self.fail = fail
        self.messages = []

    def publish(self, message):
        if self.fail:
            raise RuntimeError("test publisher failure")
        self.messages.append(message)


class _Controller:
    _finite_or_none = _finite_or_none
    _publish_reanchor_debug = _publish_reanchor_debug
    lock_c_to_p1_line = _lock_c_to_p1_line
    reanchor_c_to_p1_after_pivot = _reanchor_c_to_p1_after_pivot
    reanchor_runtime_path_after_pivot = _reanchor_runtime_path_after_pivot

    RUNTIME_ENTRY_SPACING_M = 0.05

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
        self.runtime_entry_points = []
        self.runtime_entry_cursor_index = 0
        self.runtime_entry_lookahead_index = 0
        self.runtime_entry_goal_index = None
        self.segment_alignment_active = False
        self.xtrack_priority_active = False
        self.xtrack_priority_inside_since = None
        self.waypoint_tolerance = 0.03
        self.segment_runtime_reanchored = False
        self.segment_goal_number = 2
        self.pivot_debug_pub = _Publisher()
        self.legacy_reset_reasons = []
        self.xtrack_reset_count = 0

    def _install_runtime_entry_path(self, start_x, start_y, p1_x, p1_y, _reason):
        points = build_runtime_entry_path(
            start_x,
            start_y,
            p1_x,
            p1_y,
            spacing_m=self.RUNTIME_ENTRY_SPACING_M,
        )
        if len(points) < 2:
            return False
        self.runtime_entry_points = list(points)
        self.runtime_entry_cursor_index = 1
        self.runtime_entry_lookahead_index = 1
        self.runtime_entry_goal_index = len(points) - 1
        return True

    def _reset_legacy_alignment_lifecycle(self, reason):
        self.legacy_reset_reasons.append(reason)

    def reset_xtrack_damping_state(self):
        self.xtrack_reset_count += 1

    @staticmethod
    def ground_xtrack(value):
        return -float(value)

    def get_logger(self):
        return _Logger()


def _maximum_segment_length(points):
    return max(
        math.hypot(
            points[index + 1][0] - points[index][0],
            points[index + 1][1] - points[index][1],
        )
        for index in range(len(points) - 1)
    )


def test_runtime_entry_builder_has_exact_endpoints_and_max_50mm_spacing():
    start = (1.234, -2.345)
    p1 = (4.876, 3.210)
    points = build_runtime_entry_path(
        start[0], start[1], p1[0], p1[1], spacing_m=0.05
    )
    assert points[0] == start
    assert math.isclose(points[-1][0], p1[0], rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(points[-1][1], p1[1], rel_tol=0.0, abs_tol=1.0e-12)
    assert len(points) >= 2
    assert _maximum_segment_length(points) <= 0.05 + 1.0e-12


def test_start_lock_uses_current_local_odom_c_and_builds_runtime_path():
    controller = _Controller()
    controller.current_x = 2.25
    controller.current_y = -1.75
    controller.marking_waypoints = [(8.0, 4.0)]

    assert controller.lock_c_to_p1_line("test mission enable")
    assert controller.c_line_start_x == 2.25
    assert controller.c_line_start_y == -1.75
    assert controller.runtime_entry_points[0] == (2.25, -1.75)
    assert controller.runtime_entry_points[-1] == (8.0, 4.0)
    assert _maximum_segment_length(controller.runtime_entry_points) <= 0.05 + 1.0e-12
    expected_bearing = math.atan2(4.0 - (-1.75), 8.0 - 2.25)
    assert math.isclose(
        controller.c_line_bearing,
        expected_bearing,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    )
    assert controller.runtime_entry_cursor_index == 1
    assert controller.runtime_entry_goal_index == len(controller.runtime_entry_points) - 1


def test_retained_callbacks_cannot_move_locked_start_c_before_real_pivot_reanchor():
    controller = _Controller()
    controller.current_x = 1.0
    controller.current_y = 2.0
    assert controller.lock_c_to_p1_line("initial")
    original_start = (controller.c_line_start_x, controller.c_line_start_y)
    original_path = tuple(controller.runtime_entry_points)

    controller.current_x = 1.4
    controller.current_y = 2.3
    assert controller.lock_c_to_p1_line("retained callback")
    assert (controller.c_line_start_x, controller.c_line_start_y) == original_start
    assert tuple(controller.runtime_entry_points) == original_path


def test_post_pivot_reanchor_uses_fresh_local_odom_c_prime_and_resets_cursor():
    controller = _Controller()
    controller.current_x = 0.0
    controller.current_y = 0.0
    controller.marking_waypoints = [(6.0, 0.0)]
    assert controller.lock_c_to_p1_line("initial")
    initial_path = tuple(controller.runtime_entry_points)

    controller.current_x = 0.08
    controller.current_y = -0.06
    controller.runtime_entry_cursor_index = 25
    controller.runtime_entry_lookahead_index = 30
    assert controller.reanchor_c_to_p1_after_pivot()

    assert controller.c_line_reanchored_after_pivot is True
    assert controller.c_line_start_x == 0.08
    assert controller.c_line_start_y == -0.06
    assert controller.runtime_entry_points[0] == (0.08, -0.06)
    assert controller.runtime_entry_points[-1] == (6.0, 0.0)
    assert tuple(controller.runtime_entry_points) != initial_path
    assert controller.runtime_entry_cursor_index == 1
    assert controller.runtime_entry_lookahead_index == 1
    assert controller.runtime_entry_goal_index == len(controller.runtime_entry_points) - 1
    assert _maximum_segment_length(controller.runtime_entry_points) <= 0.05 + 1.0e-12


def test_runtime_reanchor_debug_outcome_reason_and_json_contract():
    expected_fields = {
        "schema_version",
        "source",
        "outcome",
        "reason",
        "goal_number",
        "anchor_x",
        "anchor_y",
        "goal_x",
        "goal_y",
        "bearing_rad",
        "bearing_deg",
        "cross_track_m",
        "cross_track_mm",
    }

    cases = (
        ("FIRED", "runtime_path_installed", lambda controller: None, True),
        (
            "DECLINED",
            "segment_runtime_reanchored",
            lambda controller: setattr(controller, "segment_runtime_reanchored", True),
            False,
        ),
        (
            "DECLINED",
            "distance_le_waypoint_tolerance",
            lambda controller: (
                setattr(controller, "current_x", 1.99),
                setattr(controller, "current_y", 0.0),
            ),
            False,
        ),
        (
            "DECLINED",
            "_install_runtime_entry_path_returned_false",
            lambda controller: setattr(
                controller, "_install_runtime_entry_path", lambda *_args: False
            ),
            False,
        ),
        (
            "FAILED",
            "missing_anchor_or_goal",
            lambda controller: setattr(controller, "current_x", None),
            False,
        ),
    )

    for outcome, reason, arrange, expected_return in cases:
        controller = _Controller()
        arrange(controller)

        assert controller.reanchor_runtime_path_after_pivot(2.0, 0.0) is expected_return
        assert len(controller.pivot_debug_pub.messages) == 1
        payload = json.loads(controller.pivot_debug_pub.messages[0].data)
        assert set(payload) == expected_fields
        assert payload["schema_version"] == 1
        assert payload["source"] == "RPP_POST_PIVOT_REANCHOR"
        assert (payload["outcome"], payload["reason"]) == (outcome, reason)

        if outcome == "FIRED":
            assert payload["cross_track_mm"] == 0.0


def test_runtime_reanchor_debug_publisher_failure_preserves_control_return():
    success = _Controller()
    success.pivot_debug_pub = _Publisher(fail=True)
    assert success.reanchor_runtime_path_after_pivot(2.0, 0.0) is True
    assert success.segment_runtime_reanchored is True

    declined = _Controller()
    declined.segment_runtime_reanchored = True
    declined.pivot_debug_pub = _Publisher(fail=True)
    assert declined.reanchor_runtime_path_after_pivot(2.0, 0.0) is False


def test_runtime_tracker_advances_without_intermediate_stops_or_backtracking():
    points = build_runtime_entry_path(0.0, 0.0, 1.0, 0.0, spacing_m=0.05)
    solution = track_runtime_entry_path(
        points,
        current_x=0.31,
        current_y=0.01,
        cursor_index=1,
        lookahead_m=0.20,
        point_reach_m=0.075,
        waypoint_epsilon_m=0.001,
    )
    assert solution is not None
    target_x, target_y, bearing, cursor, lookahead, goal = solution
    assert cursor >= 6
    assert lookahead >= cursor
    assert goal == len(points) - 1
    assert target_x >= points[cursor][0]
    assert math.isclose(target_y, 0.0, abs_tol=1.0e-12)
    assert math.isclose(bearing, 0.0, abs_tol=1.0e-12)


def test_rpp_no_longer_owns_gp_origin_or_fused_global_c_conversion():
    forbidden = (
        "GeoPointStamped",
        "NavSatFix",
        "prepared_path_gp_origin",
        "latest_gp_origin",
        "latest_fused_global_fix",
        "last_fused_global_monotonic",
        "_runtime_current_c_from_gp_origin",
        "project_geodetic_to_px4_enu",
        "/mavros/global_position/gp_origin",
        "/mavros/global_position/global",
    )
    for marker in forbidden:
        assert marker not in NODE_SOURCE

    assert '"/mavros/local_position/odom"' in NODE_SOURCE
    assert NODE_SOURCE.count("start_x = float(self.current_x)") >= 2


def test_first_approach_uses_runtime_sidecar_then_returns_to_fixed_nav_path():
    control = _method_source("control_loop")
    first_if = control.index("if first_approach:")
    runtime_call = control.index(
        "nav_solution = self.runtime_entry_tracking_solution(goal_x, goal_y)",
        first_if,
    )
    else_index = control.index("else:", runtime_call)
    fixed_nav_call = control.index(
        "nav_solution = self.nav_path_tracking_solution(goal_x, goal_y)",
        else_index,
    )
    assert first_if < runtime_call < else_index < fixed_nav_call


def test_runtime_entry_methods_do_not_mutate_fixed_nav_path_or_signature():
    lock_source = _method_source("lock_c_to_p1_line")
    reanchor_source = _method_source("reanchor_c_to_p1_after_pivot")
    for source in (lock_source, reanchor_source):
        assert "nav_path_points" not in source
        assert "geometry_installed_signature" not in source
        assert "prepared_path_signature" not in source
        assert "path_signature" not in source


def test_obsolete_selector_is_not_control_authority_anymore():
    control = _method_source("control_loop")
    assert "select_runtime_entry_authority" not in control
    assert "runtime_entry_tracking_solution" in control
