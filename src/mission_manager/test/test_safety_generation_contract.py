"""Protect Mission Manager's monotonic hard-stop latch contract."""

import ast
from pathlib import Path


SOURCE_PATH = (
    Path(__file__).parents[1] / "mission_manager" / "mission_manager_node.py"
)
SOURCE = SOURCE_PATH.read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)


def function_source(name: str) -> str:
    for node in ast.walk(TREE):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            source = ast.get_source_segment(SOURCE, node)
            assert source is not None
            return source
    raise AssertionError(f"missing function {name}")


def test_only_explicit_release_clears_emergency_stop():
    assert "_set_safety" not in SOURCE
    assignments = [
        node
        for node in ast.walk(TREE)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Attribute)
            and target.attr == "_emergency_stop"
            for target in node.targets
        )
    ]
    clearing_lines = [
        node.lineno
        for node in assignments
        if isinstance(node.value, ast.Constant) and node.value.value is False
    ]
    release = next(
        node
        for node in ast.walk(TREE)
        if isinstance(node, ast.FunctionDef)
        and node.name == "_release_emergency_stop_latch"
    )
    assert len(clearing_lines) == 1
    assert release.lineno <= clearing_lines[0] <= release.end_lineno


def test_hard_stop_increments_generation_and_disables_motion():
    hard_stop = function_source("_assert_emergency_stop")
    assert "self._safety_generation += 1" in hard_stop
    assert "self._emergency_stop = True" in hard_stop
    assert "self._mission_enable = False" in hard_stop
    assert "self._publish_marking_active(False)" in hard_stop


def test_long_running_motion_commands_validate_generation_before_commit():
    first_commit_markers = {
        "_start_service": "self._point_status =",
        "_resume_service": 'self._state = "RUNNING"',
        "_next_point_service": 'self._state = "RUNNING"',
    }
    for name, first_commit in first_commit_markers.items():
        source = function_source(name)
        assert "safety_generation = self._safety_generation" in source
        assert "self._safety_generation != safety_generation" in source
        assert source.index("self._safety_generation != safety_generation") < source.index(
            first_commit
        )
        assert source.index("if self._emergency_stop:") < source.index(first_commit)
        assert source.index("self._enable_motion()") > source.index(first_commit)


def test_px4_settle_contract_is_unchanged():
    assert "OFFBOARD_STREAM_SETTLE_SEC = 0.60" in SOURCE
    assert "OFFBOARD_BEFORE_ARM_SETTLE_SEC = 0.50" in SOURCE


def test_release_validates_generation_before_clearing_estop_latch():
    release = function_source("_release_emergency_stop_service")

    request_read = release.index("request.expected_generation")
    generation_check = release.index(
        "expected_generation != current_generation"
    )
    latch_release = release.index(
        "self._release_emergency_stop_latch()"
    )

    assert request_read < generation_check < latch_release
    assert "response.success = False" in release
    assert "response.current_generation = current_generation" in release
