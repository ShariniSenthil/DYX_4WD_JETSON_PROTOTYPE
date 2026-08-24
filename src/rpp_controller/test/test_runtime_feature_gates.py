"""ROS-free tests for safe runtime precision feature-gate transactions."""

from __future__ import annotations

import ast
from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "rpp_controller" / "rpp_controller_node.py"
sys.path.insert(0, str(PACKAGE_ROOT))

from rpp_controller.feature_gates import (  # noqa: E402
    PRECISION_FEATURE_GATES,
    geometry_processing_requested,
    validate_precision_feature_gates,
)


class _SetParametersResult:
    def __init__(self, *, successful=False, reason=""):
        self.successful = successful
        self.reason = reason


class _ParameterApi:
    class Type:
        BOOL = 1


class _Parameter:
    def __init__(self, name, value, parameter_type=1):
        self.name = name
        self.value = value
        self.type_ = parameter_type


class _Logger:
    def __init__(self):
        self.messages = []

    def warn(self, message):
        self.messages.append(message)


def _controller_methods(*names):
    tree = ast.parse(NODE_PATH.read_text(encoding="utf-8"))
    controller = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "RPPController"
    )
    methods = [
        node
        for node in controller.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {
        "Parameter": _ParameterApi,
        "SetParametersResult": _SetParametersResult,
        "PRECISION_FEATURE_GATES": PRECISION_FEATURE_GATES,
        "geometry_processing_requested": geometry_processing_requested,
        "validate_precision_feature_gates": validate_precision_feature_gates,
    }
    module = ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[]))
    exec(compile(module, str(NODE_PATH), "exec"), namespace)
    return tuple(namespace[name] for name in names)


_gate_values, _set_gates = _controller_methods(
    "_precision_feature_gate_values",
    "_on_set_precision_feature_gates",
)


class _FakeController:
    _precision_feature_gate_values = _gate_values
    _on_set_precision_feature_gates = _set_gates

    def __init__(self):
        for name in PRECISION_FEATURE_GATES:
            setattr(self, name, False)
        self.geometry_processing_enabled = False
        self.mission_enabled = False
        self.emergency_stop = True
        self.geometry_active_span = None
        self.geometry_installed_signature = None
        self.events = []
        self.logger = _Logger()

    def _try_install_path_geometry(self):
        self.events.append("install")
        return True

    def _try_bind_geometry_goal(self, *, log_error):
        assert log_error is False
        self.events.append("bind")
        return True

    def _invalidate_installed_geometry(self, reason):
        self.events.append(("invalidate", reason))

    def _reset_precision_regulator(self, reason, *, progress_s):
        self.events.append(("regulator", reason, progress_s))

    def _reset_precision_tracking(
        self,
        reason,
        *,
        reset_metrics,
        path_identity,
    ):
        self.events.append(("tracking", reason, reset_metrics, path_identity))

    def _reset_precision_pivot(self, reason, *, clear_anchor):
        self.events.append(("pivot", reason, clear_anchor))

    def _reset_precision_terminal(self, reason):
        self.events.append(("terminal", reason))

    def reset_terminal_native_pivot(self):
        self.events.append("native_pivot_reset")

    def get_logger(self):
        return self.logger


def test_safe_activation_installs_and_binds_staged_geometry_immediately():
    controller = _FakeController()

    result = controller._on_set_precision_feature_gates(
        [_Parameter("geometry_tracking_enabled", True)]
    )

    assert result.successful
    assert controller.geometry_tracking_enabled is True
    assert controller.geometry_processing_enabled is True
    assert controller.events == ["install", "bind"]


def test_actual_gate_change_is_rejected_while_motion_can_be_active():
    for mission_enabled, emergency_stop in ((True, True), (False, False)):
        controller = _FakeController()
        controller.mission_enabled = mission_enabled
        controller.emergency_stop = emergency_stop

        result = controller._on_set_precision_feature_gates(
            [_Parameter("geometry_tracking_enabled", True)]
        )

        assert not result.successful
        assert controller.geometry_tracking_enabled is False
        assert controller.geometry_processing_enabled is False
        assert controller.events == []


def test_dependency_rejection_is_atomic():
    controller = _FakeController()

    result = controller._on_set_precision_feature_gates(
        [
            _Parameter("precision_guidance_enabled", True),
            _Parameter("precision_speed_control_enabled", True),
        ]
    )

    assert not result.successful
    assert all(
        getattr(controller, name) is False
        for name in PRECISION_FEATURE_GATES
    )
    assert controller.events == []


def test_multi_parameter_dependency_transaction_is_accepted_together():
    controller = _FakeController()

    result = controller._on_set_precision_feature_gates(
        [
            _Parameter("geometry_tracking_enabled", True),
            _Parameter("precision_guidance_enabled", True),
        ]
    )

    assert result.successful
    assert controller.geometry_tracking_enabled is True
    assert controller.precision_guidance_enabled is True
    assert controller.events == ["install", "bind"]


def test_disable_revokes_authority_and_reenable_reinstalls_staged_inputs():
    controller = _FakeController()
    assert controller._on_set_precision_feature_gates(
        [_Parameter("geometry_tracking_enabled", True)]
    ).successful
    controller.events.clear()

    disabled = controller._on_set_precision_feature_gates(
        [_Parameter("geometry_tracking_enabled", False)]
    )
    assert disabled.successful
    assert controller.geometry_processing_enabled is False
    assert controller.events == [("invalidate", "FEATURE_GATES_DISABLED")]

    controller.events.clear()
    enabled = controller._on_set_precision_feature_gates(
        [_Parameter("geometry_tracking_enabled", True)]
    )
    assert enabled.successful
    assert controller.events == ["install", "bind"]


def test_all_retained_geometry_callbacks_stage_while_processing_is_off():
    tree = ast.parse(NODE_PATH.read_text(encoding="utf-8"))
    controller = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "RPPController"
    )
    expected = {
        "nav_path_callback": "geometry_pending_nav_points",
        "marking_waypoints_callback": "geometry_pending_marking_waypoints",
        "path_types_callback": "geometry_pending_path_types",
        "marking_indices_callback": "geometry_pending_marking_indices",
        "path_signature_callback": "geometry_pending_path_signature",
        "trajectory_ready_callback": "geometry_trajectory_ready",
        "segment_goal_metadata_callback": "geometry_pending_goal_metadata",
    }

    for method_name, staged_attribute in expected.items():
        method = next(
            node
            for node in controller.body
            if isinstance(node, ast.FunctionDef) and node.name == method_name
        )
        source = ast.unparse(method)
        assert f"self.{staged_attribute} =" in source
        first_processing_check = source.find("if not self.geometry_processing_enabled")
        first_staging_write = source.find(f"self.{staged_attribute} =")
        assert first_processing_check == -1 or first_staging_write < first_processing_check


def test_pure_dependency_validation_and_processing_summary():
    gates = {name: False for name in PRECISION_FEATURE_GATES}
    assert validate_precision_feature_gates(gates) is None
    assert geometry_processing_requested(gates) is False

    gates["precision_terminal_enabled"] = True
    assert "precision_terminal_enabled requires" in (
        validate_precision_feature_gates(gates) or ""
    )

    gates = {name: True for name in PRECISION_FEATURE_GATES}
    assert validate_precision_feature_gates(gates) is None
    assert geometry_processing_requested(gates) is True
