"""Pin Mission Manager as the owner of the RTK FIXED motion gate.

trajectory_generator places surveyed GPS missions without requiring RTK
FIXED, because placement only projects through PX4 gp_origin. That is safe
only while every motion-enabling path here still requires a fresh
fix_type == 6. These tests fail if that gate is weakened or bypassed.
"""

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


def test_rtk_motion_gate_requires_fresh_fixed_solution():
    gate = function_source("_rtk_motion_ok")
    assert "gps_age > self.GPS_FIX_STALE_SEC" in gate
    assert "self._gps_fix_type != 6" in gate


def test_motion_health_blocks_without_rtk_fixed():
    health = function_source("_motion_health_status")
    assert "self._rtk_motion_ok()" in health
    assert "if not rtk_ok:" in health
    assert "return False, f\"RTK FIXED required: {rtk_reason}\"" in health


def test_motion_health_requires_ready_prepared_path():
    health = function_source("_motion_health_status")
    assert "self._pending_prepared_path.ready" in health


def test_require_motion_health_raises_on_failure():
    require = function_source("_require_motion_health")
    assert "self._motion_health_status(" in require
    assert "raise RuntimeError(reason)" in require


def test_start_resume_next_require_motion_health():
    for name in ("_start_service", "_resume_service", "_next_point_service"):
        assert "self._require_motion_health(" in function_source(name), name


def test_auto_skip_continuation_requires_rtk_fixed():
    assert "rtk_ok, rtk_reason = self._rtk_motion_ok()" in function_source(
        "_skip_point_service"
    )


def test_trajectory_readiness_loss_invalidates_installed_path():
    callback = function_source("_trajectory_ready_callback")
    assert "self._invalidate_installed_path(" in callback
