"""The placed path's topology authority is the PX4-local geometry.

compile_source_topology() projects GPS through a P1-anchored frame for
deterministic validation. At an exact threshold that projection can differ
from the real gp_origin placement, so it must never decide dummy points.
"""

import ast
from pathlib import Path


SOURCE_PATH = (
    Path(__file__).parents[1]
    / "trajectory_generator"
    / "trajectory_generator_node.py"
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


def test_navigation_path_topology_comes_from_placed_marking_points():
    generate = function_source("_generate_navigation_path")
    assert "compile_metric_topology(" in generate
    assert "marking_points," in generate
    assert "source_topology" not in generate


def test_placement_never_reads_source_topology_geometry():
    place = function_source("_try_place_prepared_mission_locked")
    assert "self._source_topology_is_current()" in place
    assert "self.source_topology." not in place
    assert "metric_points" not in place


def test_no_placement_code_consumes_source_metric_points():
    assert "source_topology.metric_points" not in SOURCE
    assert ".metric_points" not in SOURCE


def test_no_backup_module_is_imported():
    for node in ast.walk(TREE):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            module = getattr(node, "module", None) or ""
            names = [alias.name for alias in node.names]
            for name in [module, *names]:
                assert ".before_" not in name and ".bak" not in name
