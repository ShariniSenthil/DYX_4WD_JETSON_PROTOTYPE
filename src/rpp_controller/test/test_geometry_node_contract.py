"""ROS-free guards for the Phase-1 node/launch integration contract."""

import ast
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
NODE_PATH = (
    REPOSITORY_ROOT
    / "src"
    / "rpp_controller"
    / "rpp_controller"
    / "rpp_controller_node.py"
)
LAUNCH_PATH = REPOSITORY_ROOT / "src" / "rover_bringup" / "launch" / "rover.launch.py"


def _class_node(path, class_name):
    module = ast.parse(path.read_text(encoding="utf-8"))
    return next(
        node
        for node in module.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )


def _method(class_node, name):
    return next(
        node
        for node in class_node.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def _executable_unbound_method(class_node, name):
    """Compile one ROS-independent method body as an unbound Python function."""

    method = _method(class_node, name)
    module = ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[]))
    namespace = {}
    exec(compile(module, str(NODE_PATH), "exec"), namespace)
    return namespace[name]


def _assigned_self_attributes(function):
    attributes = set()
    for node in ast.walk(function):
        if not isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)):
            continue
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        for target in targets:
            if (
                isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id == "self"
            ):
                attributes.add(target.attr)
    return attributes


def test_installed_authority_invalidation_preserves_all_pending_sources():
    controller = _class_node(NODE_PATH, "RPPController")
    invalidate = _method(controller, "_invalidate_installed_geometry")

    assigned = _assigned_self_attributes(invalidate)

    assert not any(name.startswith("geometry_pending_") for name in assigned)


def test_empty_source_callbacks_clear_only_their_own_pending_component():
    controller = _class_node(NODE_PATH, "RPPController")
    expected = {
        "path_types_callback": "geometry_pending_path_types",
        "marking_indices_callback": "geometry_pending_marking_indices",
        "path_signature_callback": "geometry_pending_path_signature",
        "nav_path_callback": "geometry_pending_nav_points",
        "marking_waypoints_callback": "geometry_pending_marking_waypoints",
    }

    for method_name, own_component in expected.items():
        assigned_pending = {
            name
            for name in _assigned_self_attributes(_method(controller, method_name))
            if name.startswith("geometry_pending_")
        }
        assert assigned_pending == {own_component}


def test_ready_invalidation_keeps_pending_buffers_and_signature_is_commit_gate():
    controller = _class_node(NODE_PATH, "RPPController")
    ready = _method(controller, "trajectory_ready_callback")
    installer = ast.unparse(_method(controller, "_try_install_path_geometry"))

    assert not any(
        name.startswith("geometry_pending_")
        for name in _assigned_self_attributes(ready)
    )
    comparison = "if calculated_signature != signature"
    build = "geometry = PathGeometryIndex.build"
    assert installer.index(comparison) < installer.index(build)


def test_legacy_solution_is_returned_when_tracking_is_disabled():
    controller = _class_node(NODE_PATH, "RPPController")
    method = _method(controller, "nav_path_tracking_solution")
    source = ast.unparse(method)

    legacy_assignment = source.index(
        "legacy_solution = self._legacy_nav_path_tracking_solution"
    )
    shadow_call = source.index("self._geometry_tracking_solution", legacy_assignment)
    legacy_return = source.index("return legacy_solution", shadow_call)

    assert legacy_assignment < shadow_call < legacy_return
    assert "if self.geometry_diagnostics_enabled" in source


def test_shadow_debug_and_failure_logger_cannot_escape_legacy_dispatch():
    controller = _class_node(NODE_PATH, "RPPController")
    dispatch = _executable_unbound_method(
        controller,
        "nav_path_tracking_solution",
    )
    sentinel = ("legacy", 1, 2, 3)

    class RaisingLogger:
        def error(self, _message):
            raise RuntimeError("logger unavailable")

    class FakeController:
        geometry_tracking_enabled = False
        geometry_diagnostics_enabled = True

        def _legacy_nav_path_tracking_solution(self, _goal_x, _goal_y):
            return sentinel

        def _publish_geometry_debug(self):
            raise RuntimeError("debug publisher unavailable")

        def _geometry_tracking_solution(self, _goal_x, _goal_y, *, shadow):
            assert shadow is True
            self._publish_geometry_debug()

        def get_logger(self):
            return RaisingLogger()

    result = dispatch(FakeController(), 4.0, 5.0)

    assert result is sentinel


def test_first_goal_binding_and_transitions_seed_progress_and_hint():
    controller = _class_node(NODE_PATH, "RPPController")
    source = ast.unparse(_method(controller, "_try_bind_geometry_goal"))

    assert "if previous_raw_index != binding.raw_path_index" in source
    assert "progress_s=binding.active_span.start_s" in source
    assert "hint_segment_index=hint" in source


def test_accepted_phase_one_defaults_and_jump_bounds_are_in_launch():
    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")
    node_source = NODE_PATH.read_text(encoding="utf-8")

    for declaration in (
        'declare_parameter("geometry_tracking_enabled", True)',
        'declare_parameter("geometry_diagnostics_enabled", False)',
    ):
        assert declaration in node_source
    for launch_setting in (
        '"precision_path_contract_enabled": True',
        '"geometry_tracking_enabled": True',
        '"geometry_diagnostics_enabled": False',
    ):
        assert launch_setting in launch_source
    assert '"geometry_max_backward_jump_m": 0.10' in launch_source
    assert '"geometry_max_forward_jump_m": 1.00' in launch_source


def test_geometry_debug_contract_has_bounded_status_and_progress_identity():
    controller = _class_node(NODE_PATH, "RPPController")
    debug_source = ast.unparse(_method(controller, "_publish_geometry_debug"))
    projection_source = ast.unparse(
        _method(controller, "_geometry_tracking_solution")
    )

    for field in (
        "ros_time_ns",
        "nearest_raw_index",
        "raw_segment_start_index",
        "raw_segment_end_index",
        "active_span_start_raw_index",
        "active_span_stop_raw_index",
        "geometry_reset_reason",
        "geometry_reset_count",
    ):
        assert field in debug_source
    assert "if status not in allowed_statuses" in debug_source
    assert "max_backward_jump_m=self.geometry_max_backward_jump" in projection_source
