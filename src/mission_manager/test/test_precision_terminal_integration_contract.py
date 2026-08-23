import ast
from pathlib import Path


SOURCE_PATH = (
    Path(__file__).parents[1]
    / "mission_manager"
    / "mission_manager_node.py"
)
SOURCE = SOURCE_PATH.read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)


def function_source(name):
    for node in ast.walk(TREE):
        if (
            isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name == name
        ):
            return ast.get_source_segment(SOURCE, node)
    raise AssertionError(f"missing function {name}")


def test_precision_terminal_is_default_off_and_requires_path_contract():
    init = function_source("__init__")
    assert 'declare_parameter("precision_terminal_enabled", False)' in init
    assert 'declare_parameter("precision_terminal_heartbeat_timeout_sec", 0.50)' in init
    assert "precision_terminal_enabled requires" in init
    assert '"/rpp/terminal_certificate"' in init


def test_goal_metadata_adds_stable_run_and_instance_only_for_precision_run():
    publish = function_source("_publish_goal")
    assert "self.precision_terminal_enabled and self._mission_run_id" in publish
    assert "make_goal_instance_id(" in publish
    assert "mission_run_id=mission_run_id" in publish
    assert "goal_instance_id=goal_instance_id" in publish


def test_marking_hold_revalidates_certificate_and_resets_on_any_loss():
    handler = function_source("_precision_marking_pre_spray")
    assert "self._precision_terminal_decision(now_monotonic_sec)" in handler
    assert "self._reset_precision_marking_hold()" in handler
    assert "self._marking_hold_elapsed_sec < self.marking_hold_sec" in handler
    assert handler.index("self._capture_accuracy_snapshot(") > handler.index(
        "self._marking_hold_elapsed_sec < self.marking_hold_sec"
    )


def test_dummy_precision_path_never_uses_legacy_radius_speed_gate():
    loop = function_source("_control_loop")
    precision_branch = loop.index("if self.precision_terminal_enabled:")
    decision = loop.index("self._precision_terminal_decision(now)", precision_branch)
    legacy_radius = loop.index("inside_extension_radius =", decision)
    assert decision < legacy_radius
    assert "Dummy/extension precision stop certified" in loop


def test_default_off_legacy_result_path_remains_present():
    loop = function_source("_control_loop")
    assert 'rpp_terminal_outcome == "MISSED"' in loop
    assert 'rpp_terminal_outcome != "CAPTURED"' in loop
    assert "inside_extension_radius and extension_stationary" in loop


def test_accuracy_snapshot_copies_precision_evidence_without_geometry():
    capture = function_source("_capture_accuracy_snapshot")
    assert "self._precision_terminal_decision(time.monotonic())" in capture
    assert '"precision_certificate": certificate' in capture
    assert '"precision_terminal_evidence": copy.deepcopy(heartbeat)' in capture
    assert '"terminal_result": copy.deepcopy(rpp)' in capture
    assert "_marking_error_components" not in capture
