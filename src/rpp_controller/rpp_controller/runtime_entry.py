"""Pure helpers for START-time C-to-P1 runtime trajectory."""

from __future__ import annotations

import math
from typing import Sequence, TypeAlias

Point2D: TypeAlias = tuple[float, float]
NavTrackingSolution: TypeAlias = tuple[float, float, float, int, int, int]


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def build_runtime_entry_path(
    start_x: float,
    start_y: float,
    p1_x: float,
    p1_y: float,
    spacing_m: float = 0.05,
) -> tuple[Point2D, ...]:
    """Build exact C->P1 endpoints with every segment <= spacing_m.

    C and P1 must already be in the same ROS/MAVROS map frame.
    No origin or frame conversion is done here.
    """
    start_x = _finite(start_x, "start_x")
    start_y = _finite(start_y, "start_y")
    p1_x = _finite(p1_x, "p1_x")
    p1_y = _finite(p1_y, "p1_y")
    spacing_m = _finite(spacing_m, "spacing_m")
    if spacing_m <= 0.0:
        raise ValueError("spacing_m must be > 0")

    dx = p1_x - start_x
    dy = p1_y - start_y
    distance = math.hypot(dx, dy)
    if distance <= 1.0e-9:
        return ((p1_x, p1_y),)

    divisions = max(1, int(math.ceil(distance / spacing_m)))
    return tuple(
        (
            start_x + dx * (index / divisions),
            start_y + dy * (index / divisions),
        )
        for index in range(divisions + 1)
    )


def track_runtime_entry_path(
    points: Sequence[Point2D],
    *,
    current_x: float,
    current_y: float,
    cursor_index: int,
    lookahead_m: float,
    point_reach_m: float,
    waypoint_epsilon_m: float = 0.001,
) -> NavTrackingSolution | None:
    """Track dense C->P1 interpolation as pass-through points."""
    current_x = _finite(current_x, "current_x")
    current_y = _finite(current_y, "current_y")
    lookahead_m = _finite(lookahead_m, "lookahead_m")
    point_reach_m = _finite(point_reach_m, "point_reach_m")
    waypoint_epsilon_m = _finite(waypoint_epsilon_m, "waypoint_epsilon_m")

    if lookahead_m <= 0.0:
        raise ValueError("lookahead_m must be > 0")
    if point_reach_m <= 0.0:
        raise ValueError("point_reach_m must be > 0")
    if waypoint_epsilon_m <= 0.0:
        raise ValueError("waypoint_epsilon_m must be > 0")
    if len(points) < 2:
        return None

    clean: list[Point2D] = []
    for index, point in enumerate(points):
        if len(point) != 2:
            raise ValueError(f"points[{index}] must contain x,y")
        clean.append(
            (
                _finite(point[0], f"points[{index}].x"),
                _finite(point[1], f"points[{index}].y"),
            )
        )

    goal_index = len(clean) - 1
    cursor = max(1, min(int(cursor_index), goal_index))

    while cursor < goal_index:
        px, py = clean[cursor]
        distance_to_point = math.hypot(px - current_x, py - current_y)
        ax, ay = clean[cursor - 1]
        sx = px - ax
        sy = py - ay
        segment_length = math.hypot(sx, sy)

        passed = False
        if segment_length > waypoint_epsilon_m:
            ux = sx / segment_length
            uy = sy / segment_length
            passed = ((current_x - px) * ux + (current_y - py) * uy) >= 0.0

        if distance_to_point <= point_reach_m or passed:
            cursor += 1
            continue
        break

    lookahead_index = cursor
    accumulated = math.hypot(
        clean[cursor][0] - current_x,
        clean[cursor][1] - current_y,
    )
    while lookahead_index < goal_index and accumulated < lookahead_m:
        x0, y0 = clean[lookahead_index]
        x1, y1 = clean[lookahead_index + 1]
        accumulated += math.hypot(x1 - x0, y1 - y0)
        lookahead_index += 1

    tangent_from = max(0, cursor - 1)
    tangent_to = min(goal_index, max(cursor, tangent_from + 1))
    ax, ay = clean[tangent_from]
    bx, by = clean[tangent_to]
    dx = bx - ax
    dy = by - ay
    if math.hypot(dx, dy) <= waypoint_epsilon_m:
        return None

    path_bearing = math.atan2(dy, dx)
    target_x, target_y = clean[lookahead_index]
    return (
        target_x,
        target_y,
        path_bearing,
        cursor,
        lookahead_index,
        goal_index,
    )


def select_runtime_entry_authority(
    nav_solution: NavTrackingSolution,
    *,
    first_approach: bool,
    p1_x: float,
    p1_y: float,
    c_to_p1_bearing: float,
) -> NavTrackingSolution:
    """Compatibility helper retained for existing tools/tests."""
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
