"""Pure validation helpers for runtime precision feature-gate changes."""

from __future__ import annotations

from collections.abc import Mapping


PRECISION_FEATURE_GATES = (
    "geometry_tracking_enabled",
    "geometry_diagnostics_enabled",
    "precision_guidance_enabled",
    "precision_speed_control_enabled",
    "precision_tracking_control_enabled",
    "precision_pivot_enabled",
    "precision_terminal_enabled",
)


def geometry_processing_requested(gates: Mapping[str, bool]) -> bool:
    """Return whether any precision consumer needs the geometry sidecar."""
    return any(bool(gates[name]) for name in PRECISION_FEATURE_GATES)


def validate_precision_feature_gates(gates: Mapping[str, bool]) -> str | None:
    """Return a rejection reason when the prospective gate set is invalid."""
    missing = [name for name in PRECISION_FEATURE_GATES if name not in gates]
    if missing:
        return "missing precision feature gates: " + ", ".join(missing)
    if any(type(gates[name]) is not bool for name in PRECISION_FEATURE_GATES):
        return "precision feature gates must be boolean"

    if (
        gates["precision_guidance_enabled"]
        or gates["precision_speed_control_enabled"]
        or gates["precision_tracking_control_enabled"]
        or gates["precision_pivot_enabled"]
    ) and not gates["geometry_tracking_enabled"]:
        return (
            "precision guidance, speed control, tracking control, and pivot "
            "require geometry_tracking_enabled=true"
        )

    if gates["precision_tracking_control_enabled"] and not (
        gates["geometry_tracking_enabled"]
        and gates["precision_guidance_enabled"]
        and gates["precision_speed_control_enabled"]
    ):
        return (
            "precision_tracking_control_enabled requires geometry tracking, "
            "precision guidance, and precision speed control"
        )

    if gates["precision_terminal_enabled"] and not (
        gates["geometry_tracking_enabled"]
        and gates["precision_guidance_enabled"]
        and gates["precision_speed_control_enabled"]
        and gates["precision_tracking_control_enabled"]
        and gates["precision_pivot_enabled"]
    ):
        return (
            "precision_terminal_enabled requires geometry tracking, precision "
            "guidance, precision speed control, precision tracking control, "
            "and precision pivot"
        )
    return None
