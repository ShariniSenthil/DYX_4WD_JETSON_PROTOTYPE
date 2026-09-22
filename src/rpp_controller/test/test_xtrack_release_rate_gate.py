"""Regression tests for V4.2 xtrack recovery release stabilization."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "rpp_controller" / "rpp_controller_node.py"
NODE_SOURCE = NODE_PATH.read_text(encoding="utf-8")
NODE_TREE = ast.parse(NODE_SOURCE)


class _Duration:
    def __init__(self, sec):
        self.nanoseconds = int(sec * 1e9)


class _Time:
    def __init__(self, sec):
        self.sec = float(sec)

    def __sub__(self, other):
        return _Duration(self.sec - other.sec)


class _Clock:
    def __init__(self, node):
        self.node = node

    def now(self):
        return _Time(self.node.now_sec)


class _Logger:
    def warn(self, _msg):
        pass


def _method(name):
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


def _load_update():
    namespace = {"math": math}
    module = ast.fix_missing_locations(
        ast.Module(
            body=[_method("update_xtrack_speed_cap_state")],
            type_ignores=[],
        )
    )
    exec(compile(module, str(NODE_PATH), "exec"), namespace)
    return namespace["update_xtrack_speed_cap_state"]


def _node():
    node = NS(
        xtrack_priority_active=True,
        xtrack_priority_inside_since=None,
        xtrack_priority_enter=0.015,
        xtrack_priority_exit=0.008,
        xtrack_priority_release_heading=math.radians(4.0),
        xtrack_priority_release_rate=0.010,
        xtrack_priority_hold_sec=0.30,
        xtrack_priority_speed=0.30,
        now_sec=0.0,
        ground_xtrack=lambda value: -float(value),
        get_logger=lambda: _Logger(),
    )
    node.get_clock = lambda: _Clock(node)
    return node


def test_v42_rate_gate_blocks_release_while_crossing_line_fast():
    update = _load_update()
    node = _node()

    for sec in (0.00, 0.10, 0.20, 0.40):
        node.now_sec = sec
        active, _, elapsed = update(
            node,
            0.004,
            0.005,
            math.radians(1.0),
            0.030,
        )
        assert active is True
        assert elapsed == 0.0
        assert node.xtrack_priority_inside_since is None


def test_v42_rate_gate_releases_only_after_rate_and_geometry_settle():
    update = _load_update()
    node = _node()

    node.now_sec = 0.00
    active, _, _ = update(
        node,
        0.006,
        0.007,
        math.radians(2.0),
        0.006,
    )
    assert active is True

    node.now_sec = 0.20
    active, _, elapsed = update(
        node,
        0.006,
        0.007,
        math.radians(2.0),
        0.006,
    )
    assert active is True
    assert 0.19 <= elapsed <= 0.21

    node.now_sec = 0.31
    active, _, elapsed = update(
        node,
        0.006,
        0.007,
        math.radians(2.0),
        0.006,
    )
    assert active is False
    assert elapsed >= 0.30


def test_v42_rate_violation_resets_earned_release_dwell():
    update = _load_update()
    node = _node()

    node.now_sec = 0.00
    update(node, 0.006, 0.007, math.radians(2.0), 0.006)

    node.now_sec = 0.20
    update(node, 0.006, 0.007, math.radians(2.0), 0.020)
    assert node.xtrack_priority_inside_since is None

    node.now_sec = 0.30
    active, _, elapsed = update(
        node,
        0.006,
        0.007,
        math.radians(2.0),
        0.006,
    )
    assert active is True
    assert elapsed == 0.0
