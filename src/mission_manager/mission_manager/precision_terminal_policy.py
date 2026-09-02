"""Pure Mission Manager policy for precision terminal heartbeats.

The policy deliberately validates controller evidence and identity only.  It
does not recalculate goal geometry, tolerance, or stopped state.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


PRECISION_TERMINAL_SCHEMA_VERSION = 2
PRECISION_TERMINAL_SOURCE = "RPP_PRECISION_TERMINAL_HEARTBEAT"


@dataclass(frozen=True, slots=True)
class PrecisionTerminalExpectation:
    """Identity expected for the currently authoritative semantic goal."""

    mission_run_id: str
    path_signature: str
    raw_path_index: int
    active_goal_identity: str
    goal_instance_id: str

    @property
    def terminal_identity(self) -> str:
        """Return the deterministic cross-node terminal identity."""

        return (
            f"RUN:{self.mission_run_id}|PATH:{self.path_signature}|"
            f"RAW:{self.raw_path_index}|GOAL:{self.active_goal_identity}|"
            f"INSTANCE:{self.goal_instance_id}"
        )


@dataclass(frozen=True, slots=True)
class PrecisionTerminalDecision:
    """Result of validating one locally timestamped heartbeat."""

    valid: bool
    reason: str
    certificate: dict[str, Any] | None = None


def validate_precision_terminal_heartbeat(
    payload: Mapping[str, Any] | None,
    *,
    received_monotonic_sec: float | None,
    now_monotonic_sec: float,
    timeout_sec: float,
    expectation: PrecisionTerminalExpectation,
) -> PrecisionTerminalDecision:
    """Validate freshness, certification state, and exact semantic identity."""

    if payload is None or received_monotonic_sec is None:
        return PrecisionTerminalDecision(False, "heartbeat_missing")
    if not all(
        math.isfinite(value)
        for value in (received_monotonic_sec, now_monotonic_sec, timeout_sec)
    ) or timeout_sec <= 0.0:
        return PrecisionTerminalDecision(False, "freshness_input_invalid")
    age_sec = max(0.0, now_monotonic_sec - received_monotonic_sec)
    if age_sec > timeout_sec:
        return PrecisionTerminalDecision(False, "heartbeat_expired")

    if payload.get("schema_version") != PRECISION_TERMINAL_SCHEMA_VERSION:
        return PrecisionTerminalDecision(False, "schema_version_mismatch")
    if str(payload.get("source") or "").strip() != PRECISION_TERMINAL_SOURCE:
        return PrecisionTerminalDecision(False, "source_mismatch")
    if payload.get("precision_terminal_enabled") is not True:
        return PrecisionTerminalDecision(False, "precision_terminal_disabled")
    # The dormant Phase-5 FSM has no distinct post-certification state: it
    # stays reporting "certified" forever once certified. radial20's state
    # machine (terminal_stop_regulator.py) splits that into a one-tick
    # "certified" event followed by a persistent "hold_zero" state -- by the
    # time this heartbeat is checked, radial20 has almost always already
    # moved past the single "certified" tick. Accept both: "currently_valid"/
    # "zero_latched"/"precision_pass" below are what actually gate hold/spray
    # authority, not the state label itself.
    if str(payload.get("state") or "").strip().lower() not in (
        "certified",
        "hold_zero",
    ):
        return PrecisionTerminalDecision(False, "state_not_certified")
    if payload.get("currently_valid") is not True:
        return PrecisionTerminalDecision(False, "certificate_not_currently_valid")
    if payload.get("zero_latched") is not True:
        return PrecisionTerminalDecision(False, "zero_not_latched")
    if payload.get("precision_certificate_version") != 2:
        return PrecisionTerminalDecision(False, "certificate_version_mismatch")
    if payload.get("precision_pass") is not True:
        return PrecisionTerminalDecision(False, "precision_pass_false")

    expected = {
        "mission_run_id": expectation.mission_run_id,
        "path_signature": expectation.path_signature,
        "raw_path_index": expectation.raw_path_index,
        "active_goal_identity": expectation.active_goal_identity,
        "goal_instance_id": expectation.goal_instance_id,
    }
    for key, value in expected.items():
        if payload.get(key) != value:
            return PrecisionTerminalDecision(False, f"{key}_mismatch")

    if payload.get("terminal_identity") != expectation.terminal_identity:
        return PrecisionTerminalDecision(False, "terminal_identity_mismatch")
    components = payload.get("terminal_identity_components")
    if not isinstance(components, Mapping):
        return PrecisionTerminalDecision(False, "identity_components_missing")
    for key, value in expected.items():
        if components.get(key) != value:
            return PrecisionTerminalDecision(
                False, f"identity_component_{key}_mismatch"
            )

    certificate = payload.get("certificate")
    if not isinstance(certificate, dict):
        return PrecisionTerminalDecision(False, "certificate_missing")
    if certificate.get("version") != 2:
        return PrecisionTerminalDecision(False, "nested_certificate_version_mismatch")
    if certificate.get("precision_pass") is not True:
        return PrecisionTerminalDecision(False, "nested_precision_pass_false")
    if certificate.get("terminal_identity") != expectation.terminal_identity:
        return PrecisionTerminalDecision(False, "nested_terminal_identity_mismatch")

    return PrecisionTerminalDecision(True, "valid", dict(certificate))
