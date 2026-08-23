"""Pure helpers for prepared-path and semantic-goal synchronization."""

from __future__ import annotations

import hashlib
import struct
from dataclasses import dataclass
from typing import Any, Sequence


PATH_SIGNATURE_HEX_LENGTH = 64
SEGMENT_GOAL_METADATA_VERSION = 1


@dataclass
class PendingPreparedPath:
    """Separately retained inputs for one prepared-path candidate.

    DDS retained topics can be delivered in any cross-topic order. Clearing
    one source therefore clears only that source, while a readiness loss only
    closes the install gate. Neither operation destroys potentially newer
    values already received on the other topics.
    """

    navigation_path: list[tuple[float, float]] | None = None
    mission_waypoints: list[tuple[float, float]] | None = None
    point_types: list[int] | None = None
    marking_indices: list[int] | None = None
    path_signature: str | None = None
    ready: bool = False

    _SOURCE_FIELDS = {
        "navigation_path": "navigation_path",
        "mission_waypoints": "mission_waypoints",
        "point_types": "point_types",
        "marking_indices": "marking_indices",
        "path_signature": "path_signature",
    }

    def clear_source(self, source: str) -> None:
        """Clear exactly one retained source without disturbing the others."""
        field_name = self._SOURCE_FIELDS.get(source)
        if field_name is None:
            raise ValueError(f"unknown prepared-path source: {source}")
        setattr(self, field_name, None)

    def invalidate_readiness(self) -> None:
        """Close the commit gate while retaining all candidate components."""
        self.ready = False

    def clear_all(self) -> None:
        """Discard all pending inputs during an explicit local reset/stop."""
        for field_name in self._SOURCE_FIELDS.values():
            setattr(self, field_name, None)
        self.ready = False


@dataclass(frozen=True)
class PathSignatureDecision:
    """Result of applying the optional/required snapshot-signature gate."""

    can_install: bool
    synchronized_signature: str | None
    reason: str


def make_path_signature(
    navigation_points: Sequence[tuple[float, float]],
    marking_points: Sequence[tuple[float, float]],
    point_types: Sequence[int],
    marking_indices: Sequence[int],
) -> str:
    """Return the trajectory-generator v1 commit signature for one snapshot."""
    digest = hashlib.sha256()

    for label, points in (
        (b"NAVIGATION", navigation_points),
        (b"MARKINGS", marking_points),
    ):
        digest.update(label)
        for x, y in points:
            digest.update(struct.pack("!dd", float(x), float(y)))

    digest.update(b"TYPES")
    digest.update(bytes(int(value) for value in point_types))

    digest.update(b"INDICES")
    for value in marking_indices:
        digest.update(struct.pack("!i", int(value)))

    return digest.hexdigest()


def is_valid_path_signature(value: str) -> bool:
    """Return whether *value* is a canonical lowercase SHA-256 hex digest."""
    return (
        len(value) == PATH_SIGNATURE_HEX_LENGTH
        and value == value.lower()
        and all(character in "0123456789abcdef" for character in value)
    )


def resolve_path_signature(
    navigation_points: Sequence[tuple[float, float]],
    marking_points: Sequence[tuple[float, float]],
    point_types: Sequence[int],
    marking_indices: Sequence[int],
    received_signature: str | None,
    *,
    signature_required: bool,
) -> PathSignatureDecision:
    """Resolve the prepared snapshot's additive signature contract.

    Legacy mode accepts the original four-topic contract when no signature is
    available. If a signature is available, however, it is always treated as
    the atomic commit marker and must match. Precision mode additionally
    requires that marker before a snapshot can be installed.
    """
    if received_signature is None:
        if signature_required:
            return PathSignatureDecision(False, None, "signature_required")
        return PathSignatureDecision(True, None, "legacy_without_signature")

    if not is_valid_path_signature(received_signature):
        return PathSignatureDecision(False, None, "signature_invalid")

    calculated_signature = make_path_signature(
        navigation_points,
        marking_points,
        point_types,
        marking_indices,
    )
    if calculated_signature != received_signature:
        return PathSignatureDecision(False, None, "signature_mismatch")

    return PathSignatureDecision(True, received_signature, "signature_matched")


def build_segment_goal_metadata(
    *,
    path_signature: str,
    goal_sequence: int,
    raw_path_index: int,
    point_type: int,
    marking_index: int,
    marking_point_type: int,
    mission_run_id: str | None = None,
    goal_instance_id: str | None = None,
) -> dict[str, Any]:
    """Build the additive, versioned semantic-goal identity payload.

    ``goal_sequence`` and all indices are zero-based. Dummy identities are
    scoped to the immutable prepared-path signature; only real marking points
    receive the authoritative ``Pxxxx`` mission identity.
    """
    if not is_valid_path_signature(path_signature):
        raise ValueError("path_signature must be a lowercase SHA-256 digest")
    if goal_sequence < 0:
        raise ValueError("goal_sequence must be >= 0")
    if raw_path_index < 0:
        raise ValueError("raw_path_index must be >= 0")

    if point_type == marking_point_type:
        if marking_index < 0:
            raise ValueError("marking goal must have a non-negative marking_index")
        point_id: str | None = f"P{marking_index+1:04d}"
        active_goal_identity = point_id
    else:
        if marking_index != -1:
            raise ValueError("navigation-only goal must use marking_index=-1")
        point_id = None
        active_goal_identity = (
            f"PATH:{path_signature}:RAW:{raw_path_index}:TYPE:{point_type}"
        )

    payload = {
        "schema_version": SEGMENT_GOAL_METADATA_VERSION,
        "path_signature": path_signature,
        "goal_sequence": int(goal_sequence),
        "raw_path_index": int(raw_path_index),
        "point_type": int(point_type),
        "marking_index": int(marking_index),
        "point_id": point_id,
        "active_goal_identity": active_goal_identity,
    }
    if mission_run_id is None and goal_instance_id is None:
        return payload
    if not isinstance(mission_run_id, str) or not mission_run_id.strip():
        raise ValueError("mission_run_id must be a non-empty string")
    if not isinstance(goal_instance_id, str) or not goal_instance_id.strip():
        raise ValueError("goal_instance_id must be a non-empty string")
    payload["mission_run_id"] = mission_run_id.strip()
    payload["goal_instance_id"] = goal_instance_id.strip()
    return payload


def make_goal_instance_id(
    *,
    mission_run_id: str,
    path_signature: str,
    raw_path_index: int,
    goal_sequence: int,
) -> str:
    """Return a stable deterministic identity for one run-scoped goal."""

    if not isinstance(mission_run_id, str) or not mission_run_id.strip():
        raise ValueError("mission_run_id must be a non-empty string")
    if not is_valid_path_signature(path_signature):
        raise ValueError("path_signature must be a lowercase SHA-256 digest")
    if raw_path_index < 0 or goal_sequence < 0:
        raise ValueError("raw_path_index and goal_sequence must be >= 0")
    material = (
        f"{mission_run_id.strip()}|{path_signature}|"
        f"{int(raw_path_index)}|{int(goal_sequence)}"
    ).encode("utf-8")
    return hashlib.sha256(material).hexdigest()
