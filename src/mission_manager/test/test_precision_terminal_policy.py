import copy

import pytest

from mission_manager.precision_terminal_policy import (
    PrecisionTerminalExpectation,
    validate_precision_terminal_heartbeat,
)


EXPECTATION = PrecisionTerminalExpectation(
    mission_run_id="run-7",
    path_signature="a" * 64,
    raw_path_index=42,
    active_goal_identity="P0008",
    goal_instance_id="b" * 64,
)


def valid_heartbeat():
    expected = {
        "mission_run_id": EXPECTATION.mission_run_id,
        "path_signature": EXPECTATION.path_signature,
        "raw_path_index": EXPECTATION.raw_path_index,
        "active_goal_identity": EXPECTATION.active_goal_identity,
        "goal_instance_id": EXPECTATION.goal_instance_id,
    }
    return {
        "schema_version": 2,
        "source": "RPP_PRECISION_TERMINAL_HEARTBEAT",
        "precision_terminal_enabled": True,
        "state": "certified",
        "zero_latched": True,
        "currently_valid": True,
        "precision_certificate_version": 2,
        "precision_pass": True,
        "terminal_identity": EXPECTATION.terminal_identity,
        "terminal_identity_components": copy.deepcopy(expected),
        **expected,
        "certificate": {
            "version": 2,
            "terminal_identity": EXPECTATION.terminal_identity,
            "precision_pass": True,
            "radial_error_mm": 7.4,
        },
    }


def decide(payload, *, received=10.0, now=10.1, timeout=0.5):
    return validate_precision_terminal_heartbeat(
        payload,
        received_monotonic_sec=received,
        now_monotonic_sec=now,
        timeout_sec=timeout,
        expectation=EXPECTATION,
    )


def test_current_matching_certificate_is_valid():
    decision = decide(valid_heartbeat())
    assert decision.valid
    assert decision.reason == "valid"
    assert decision.certificate["radial_error_mm"] == 7.4


@pytest.mark.parametrize(
    ("field", "value", "reason"),
    [
        ("schema_version", 1, "schema_version_mismatch"),
        ("state", "settle", "state_not_certified"),
        ("currently_valid", False, "certificate_not_currently_valid"),
        ("zero_latched", False, "zero_not_latched"),
        ("precision_certificate_version", 1, "certificate_version_mismatch"),
        ("precision_pass", False, "precision_pass_false"),
        ("mission_run_id", "old-run", "mission_run_id_mismatch"),
        ("path_signature", "c" * 64, "path_signature_mismatch"),
        ("raw_path_index", 41, "raw_path_index_mismatch"),
        ("active_goal_identity", "P0007", "active_goal_identity_mismatch"),
        ("goal_instance_id", "d" * 64, "goal_instance_id_mismatch"),
        ("terminal_identity", "old", "terminal_identity_mismatch"),
    ],
)
def test_wrong_state_or_identity_is_rejected(field, value, reason):
    payload = valid_heartbeat()
    payload[field] = value
    decision = decide(payload)
    assert not decision.valid
    assert decision.reason == reason


def test_nested_identity_and_certificate_are_independently_validated():
    payload = valid_heartbeat()
    payload["terminal_identity_components"]["goal_instance_id"] = "old"
    assert decide(payload).reason == "identity_component_goal_instance_id_mismatch"

    payload = valid_heartbeat()
    payload["certificate"]["terminal_identity"] = "old"
    assert decide(payload).reason == "nested_terminal_identity_mismatch"


def test_freshness_uses_local_receive_time_and_expires_strictly():
    payload = valid_heartbeat()
    payload["ros_time_ns"] = 1
    assert decide(payload, received=10.0, now=10.5).valid
    expired = decide(payload, received=10.0, now=10.500001)
    assert not expired.valid
    assert expired.reason == "heartbeat_expired"
