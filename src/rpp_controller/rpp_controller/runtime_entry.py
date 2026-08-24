"""Pure authority selection for the runtime C-to-P1 entry line."""

from __future__ import annotations

import math
from typing import TypeAlias


NavTrackingSolution: TypeAlias = tuple[float, float, float, int, int, int]


def select_runtime_entry_authority(
    nav_solution: NavTrackingSolution,
    *,
    first_approach: bool,
    p1_x: float,
    p1_y: float,
    c_to_p1_bearing: float,
) -> NavTrackingSolution:
    """Keep nav indices diagnostic-only while fresh C-to-P1 owns movement.

    Outside the first approach, return the original tuple object unchanged so
    P1-to-P2 and every later semantic span retain their existing behavior.
    """

    if not first_approach:
        return nav_solution
    for name, value in (
        ("p1_x", p1_x),
        ("p1_y", p1_y),
        ("c_to_p1_bearing", c_to_p1_bearing),
    ):
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
    (
        _nav_target_x,
        _nav_target_y,
        _nav_path_bearing,
        nav_cursor_index,
        nav_lookahead_index,
        nav_goal_index,
    ) = nav_solution
    return (
        p1_x,
        p1_y,
        c_to_p1_bearing,
        nav_cursor_index,
        nav_lookahead_index,
        nav_goal_index,
    )
