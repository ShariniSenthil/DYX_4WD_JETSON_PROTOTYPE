"""Atomic policy for Mission Manager precision feature gates."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

from mission_manager.path_contract import resolve_path_signature


PRECISION_PATH_PARAMETER = "precision_path_contract_enabled"
PRECISION_TERMINAL_PARAMETER = "precision_terminal_enabled"
PRECISION_FEATURE_PARAMETERS = {
    PRECISION_PATH_PARAMETER,
    PRECISION_TERMINAL_PARAMETER,
}
ACTIVE_MISSION_STATES = {"RUNNING", "PAUSED", "WAITING_FOR_NEXT"}


@dataclass(frozen=True)
class PrecisionFeatureGateDecision:
    """One atomic decision for a proposed precision-gate parameter batch."""

    accepted: bool
    reason: str
    path_contract_enabled: bool
    terminal_enabled: bool
    changed: bool = False
    republish_ready_goal: bool = False


def _rejected(
    reason: str,
    *,
    path_contract_enabled: bool,
    terminal_enabled: bool,
) -> PrecisionFeatureGateDecision:
    return PrecisionFeatureGateDecision(
        accepted=False,
        reason=reason,
        path_contract_enabled=path_contract_enabled,
        terminal_enabled=terminal_enabled,
    )


def decide_precision_feature_gate_update(
    updates: Sequence[tuple[str, Any]],
    *,
    current_path_contract_enabled: bool,
    current_terminal_enabled: bool,
    mission_state: str,
    mission_enable: bool,
    emergency_stop: bool,
    navigation_path: Sequence[tuple[float, float]],
    mission_waypoints: Sequence[tuple[float, float]],
    path_types: Sequence[int],
    marking_indices: Sequence[int],
    path_signature: str | None,
    current_path_index: int,
    semantic_path_indices: Sequence[int],
    pass_through_point_type: int,
) -> PrecisionFeatureGateDecision:
    """Validate and resolve a precision-gate update without mutating state."""
    relevant_updates = [item for item in updates if item[0] in PRECISION_FEATURE_PARAMETERS]
    if not relevant_updates:
        return PrecisionFeatureGateDecision(
            accepted=True,
            reason="no precision feature-gate changes",
            path_contract_enabled=current_path_contract_enabled,
            terminal_enabled=current_terminal_enabled,
        )

    proposed: dict[str, bool] = {
        PRECISION_PATH_PARAMETER: current_path_contract_enabled,
        PRECISION_TERMINAL_PARAMETER: current_terminal_enabled,
    }
    seen: set[str] = set()
    for name, value in relevant_updates:
        if name in seen:
            return _rejected(
                f"duplicate precision parameter in atomic batch: {name}",
                path_contract_enabled=current_path_contract_enabled,
                terminal_enabled=current_terminal_enabled,
            )
        seen.add(name)
        if not isinstance(value, bool):
            return _rejected(
                f"{name} must be a bool",
                path_contract_enabled=current_path_contract_enabled,
                terminal_enabled=current_terminal_enabled,
            )
        proposed[name] = value

    proposed_path = proposed[PRECISION_PATH_PARAMETER]
    proposed_terminal = proposed[PRECISION_TERMINAL_PARAMETER]
    changed = (
        proposed_path != current_path_contract_enabled
        or proposed_terminal != current_terminal_enabled
    )
    if not changed:
        return PrecisionFeatureGateDecision(
            accepted=True,
            reason="precision feature gates unchanged",
            path_contract_enabled=proposed_path,
            terminal_enabled=proposed_terminal,
        )

    if mission_state in ACTIVE_MISSION_STATES:
        return _rejected(
            f"precision feature gates cannot change in state {mission_state}",
            path_contract_enabled=current_path_contract_enabled,
            terminal_enabled=current_terminal_enabled,
        )
    if mission_enable:
        return _rejected(
            "precision feature gates cannot change while mission_enable is true",
            path_contract_enabled=current_path_contract_enabled,
            terminal_enabled=current_terminal_enabled,
        )
    if not emergency_stop:
        return _rejected(
            "precision feature gates can change only while emergency_stop is true",
            path_contract_enabled=current_path_contract_enabled,
            terminal_enabled=current_terminal_enabled,
        )
    if proposed_terminal and not proposed_path:
        return _rejected(
            "precision_terminal_enabled requires precision_path_contract_enabled",
            path_contract_enabled=current_path_contract_enabled,
            terminal_enabled=current_terminal_enabled,
        )

    enabling_ready_path = (
        mission_state == "READY"
        and proposed_path
        and not current_path_contract_enabled
    )
    if enabling_ready_path:
        if not navigation_path:
            reason = "READY precision enable requires an installed navigation path"
        elif not mission_waypoints:
            reason = "READY precision enable requires installed mission waypoints"
        elif not (
            len(navigation_path) == len(path_types) == len(marking_indices)
        ):
            reason = "READY precision enable found inconsistent path metadata lengths"
        else:
            signature_decision = resolve_path_signature(
                navigation_path,
                mission_waypoints,
                path_types,
                marking_indices,
                path_signature,
                signature_required=True,
            )
            reason = (
                ""
                if signature_decision.can_install
                else "READY precision enable rejected path signature: "
                f"{signature_decision.reason}"
            )

        if not reason and not (0 <= current_path_index < len(navigation_path)):
            reason = "READY precision enable found an invalid active raw path index"
        if not reason and current_path_index not in semantic_path_indices:
            reason = "READY precision enable found no active semantic goal identity"
        if (
            not reason
            and path_types[current_path_index] == pass_through_point_type
        ):
            reason = "READY precision enable cannot republish a pass-through goal"
        if reason:
            return _rejected(
                reason,
                path_contract_enabled=current_path_contract_enabled,
                terminal_enabled=current_terminal_enabled,
            )

    return PrecisionFeatureGateDecision(
        accepted=True,
        reason="precision feature gates updated",
        path_contract_enabled=proposed_path,
        terminal_enabled=proposed_terminal,
        changed=True,
        republish_ready_goal=enabling_ready_path,
    )
