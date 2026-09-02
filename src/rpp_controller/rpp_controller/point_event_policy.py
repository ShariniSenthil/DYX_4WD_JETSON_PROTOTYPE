"""Pure point-event policy helpers for RPP semantic-goal handoff."""

from __future__ import annotations

import math


RESOLVED_POINT_EVENTS = frozenset(
    {
        "COMPLETED",
        "FAILED",
        "ACCURACY_FAILED",
        "SKIPPED",
    }
)


def should_release_first_marking(event: str, point_index: int) -> bool:
    """Return whether a terminal P1 event must release C-to-P1 guidance."""

    return str(event or "").strip().upper() in RESOLVED_POINT_EVENTS and point_index == 0


def first_marking_approach_is_active(
    *,
    first_marking_resolved: bool,
    c_line_locked: bool,
    c_line_bearing_available: bool,
    has_marking_waypoints: bool,
    segment_goal_number: int,
) -> bool:
    """Return whether the special C-to-P1 controller may own guidance."""

    return (
        not first_marking_resolved
        and segment_goal_number == 1
        and c_line_locked
        and c_line_bearing_available
        and has_marking_waypoints
    )


def latched_stop_terminal_outcome(
    *,
    target_distance: float,
    current_speed: float,
    waypoint_tolerance: float,
    stationary_speed_tolerance: float,
) -> str | None:
    """Resolve a latched precision stop when the rover is stationary.

    Entering the waypoint radius latches a zero command while the rover
    settles. Once stationary, the current radial error is authoritative:
    remaining inside is a capture, while settling outside is a miss.
    """

    if not math.isfinite(current_speed):
        return None
    if current_speed > stationary_speed_tolerance:
        return None
    if target_distance <= waypoint_tolerance:
        return "CAPTURED"
    return "MISSED"
 