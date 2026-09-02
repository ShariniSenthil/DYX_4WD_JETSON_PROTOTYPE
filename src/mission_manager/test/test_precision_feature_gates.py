import ast
from pathlib import Path

import pytest

from mission_manager.path_contract import make_path_signature
from mission_manager.precision_feature_gates import (
    decide_precision_feature_gate_update,
)


NAVIGATION_PATH = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
MISSION_WAYPOINTS = [(1.0, 0.0)]
PATH_TYPES = [0, 2, 0]
MARKING_INDICES = [-1, 0, -1]
PATH_SIGNATURE = make_path_signature(
    NAVIGATION_PATH,
    MISSION_WAYPOINTS,
    PATH_TYPES,
    MARKING_INDICES,
)


def test_accepted_path_contract_is_default_on_but_terminal_stays_off():
    """Persist Gate-1 acceptance without enabling terminal authority."""
    source_path = (
        Path(__file__).parents[1]
        / "mission_manager"
        / "mission_manager_node.py"
    )
    source = source_path.read_text(encoding="utf-8")
    assert (
        'declare_parameter("precision_path_contract_enabled", True)'
        in source
    )
    assert 'declare_parameter("terminal_stop_mode", "legacy")' in source
    assert 'declare_parameter("precision_terminal_enabled", False)' in source


def decide(updates, **overrides):
    arguments = {
        "current_path_contract_enabled": False,
        "current_terminal_enabled": False,
        "mission_state": "READY",
        "mission_enable": False,
        "emergency_stop": True,
        "navigation_path": NAVIGATION_PATH,
        "mission_waypoints": MISSION_WAYPOINTS,
        "path_types": PATH_TYPES,
        "marking_indices": MARKING_INDICES,
        "path_signature": PATH_SIGNATURE,
        "current_path_index": 1,
        "semantic_path_indices": [1],
        "pass_through_point_type": 0,
    }
    arguments.update(overrides)
    return decide_precision_feature_gate_update(updates, **arguments)


def test_ready_path_activation_is_atomic_and_requests_one_goal_republish():
    decision = decide([("precision_path_contract_enabled", True)])

    assert decision.accepted is True
    assert decision.changed is True
    assert decision.path_contract_enabled is True
    assert decision.terminal_enabled is False
    assert decision.republish_ready_goal is True


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        ({"path_signature": None}, "signature_required"),
        ({"path_signature": "0" * 64}, "signature_mismatch"),
        ({"navigation_path": []}, "installed navigation path"),
        ({"mission_waypoints": []}, "installed mission waypoints"),
        ({"path_types": PATH_TYPES[:-1]}, "metadata lengths"),
    ],
)
def test_ready_activation_rejects_invalid_or_missing_contract(overrides, reason):
    decision = decide(
        [("precision_path_contract_enabled", True)],
        **overrides,
    )

    assert decision.accepted is False
    assert reason in decision.reason
    assert decision.path_contract_enabled is False
    assert decision.terminal_enabled is False
    assert decision.changed is False
    assert decision.republish_ready_goal is False


@pytest.mark.parametrize("mission_state", ["RUNNING", "PAUSED", "WAITING_FOR_NEXT"])
def test_actual_changes_are_rejected_in_active_mission_states(mission_state):
    decision = decide(
        [("precision_path_contract_enabled", True)],
        mission_state=mission_state,
    )

    assert decision.accepted is False
    assert mission_state in decision.reason


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        ({"mission_enable": True}, "mission_enable"),
        ({"emergency_stop": False}, "emergency_stop"),
    ],
)
def test_actual_changes_require_stationary_safety_gates(overrides, reason):
    decision = decide(
        [("precision_path_contract_enabled", True)],
        **overrides,
    )

    assert decision.accepted is False
    assert reason in decision.reason


def test_idempotent_write_is_allowed_while_active():
    decision = decide(
        [("precision_path_contract_enabled", True)],
        current_path_contract_enabled=True,
        mission_state="RUNNING",
        mission_enable=True,
        emergency_stop=False,
    )

    assert decision.accepted is True
    assert decision.changed is False
    assert decision.path_contract_enabled is True


def test_terminal_requires_path_contract_in_same_atomic_result():
    rejected = decide([("precision_terminal_enabled", True)])
    accepted = decide(
        [
            ("precision_path_contract_enabled", True),
            ("precision_terminal_enabled", True),
        ]
    )

    assert rejected.accepted is False
    assert "requires precision_path_contract_enabled" in rejected.reason
    assert rejected.path_contract_enabled is False
    assert rejected.terminal_enabled is False
    assert accepted.accepted is True
    assert accepted.path_contract_enabled is True
    assert accepted.terminal_enabled is True
    assert accepted.republish_ready_goal is True


def test_invalid_batch_cannot_partially_mutate_resulting_authority():
    decision = decide(
        [
            ("precision_path_contract_enabled", True),
            ("precision_terminal_enabled", "true"),
        ]
    )

    assert decision.accepted is False
    assert decision.path_contract_enabled is False
    assert decision.terminal_enabled is False


def test_callback_keeps_terminal_authority_restart_only_and_updates_path_cache():
    source_path = (
        Path(__file__).parents[1]
        / "mission_manager"
        / "mission_manager_node.py"
    )
    source = source_path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    callback = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef)
        and node.name == "_on_precision_feature_parameters"
    )
    callback_source = ast.get_source_segment(source, callback)

    assert 'name in {"terminal_stop_mode", "precision_terminal_enabled"}' in callback_source
    assert "terminal authority cannot change live" in callback_source
    assert "self.precision_terminal_enabled =" not in callback_source

    path_assignment = callback_source.index("self.precision_path_contract_enabled =")
    republish = callback_source.index("self._publish_goal()")
    status = callback_source.index("self._publish_status(force=True)")
    assert path_assignment < republish
    assert republish < status

    status_function = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "_status_payload"
    )
    status_source = ast.get_source_segment(source, status_function)
    assert '"precision_path_contract_enabled"' in status_source
    assert '"precision_terminal_enabled"' in status_source
    assert '"terminal_stop_mode"' in status_source
    assert '"terminal_certificate_required"' in status_source
