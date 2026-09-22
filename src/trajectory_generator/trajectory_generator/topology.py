"""Pure mission-topology compiler for the DYX 4WD marking rover.

No ROS, MAVROS, RTK, odometry, current rover pose, or live PX4 state
is required here.

For GPS missions, surveyed P1 is used only as a deterministic temporary
projection origin for static topology classification. Final runtime path
coordinates are still produced later in the actual PX4 local frame.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from trajectory_generator.localization_frame import GeographicOrigin
from trajectory_generator.localization_frame import project_geodetic_to_px4_enu


Point = tuple[float, float]


@dataclass(frozen=True)
class SegmentTopology:
    index: int
    distance_m: float
    use_dummy: bool
    incoming_direction: Point | None
    transfer_angle_deg: float | None
    reversal_angle_deg: float | None


@dataclass(frozen=True)
class MissionTopology:
    metric_points: tuple[Point, ...]
    segments: tuple[SegmentTopology, ...]


def _finite_point(
    point: Sequence[float],
    *,
    label: str,
) -> Point:
    if len(point) != 2:
        raise ValueError(f"{label} must contain exactly two values")

    first = float(point[0])
    second = float(point[1])

    if not (math.isfinite(first) and math.isfinite(second)):
        raise ValueError(f"{label} must contain finite values")

    return first, second


def _distance(first: Point, second: Point) -> float:
    return math.hypot(
        second[0] - first[0],
        second[1] - first[1],
    )


def _angle_between_vectors_deg(
    first: Point,
    second: Point,
) -> float:
    first_length = math.hypot(first[0], first[1])
    second_length = math.hypot(second[0], second[1])

    if first_length <= 0.0 or second_length <= 0.0:
        raise ValueError(
            "Cannot calculate extension angle from zero vector"
        )

    cosine = (
        first[0] * second[0]
        + first[1] * second[1]
    ) / (
        first_length * second_length
    )

    cosine = max(-1.0, min(1.0, cosine))

    return math.degrees(math.acos(cosine))


def _evaluate_row_transition(
    *,
    marking_points: Sequence[Point],
    index: int,
    row_transfer_min_angle_deg: float,
    row_transfer_max_angle_deg: float,
    row_reversal_min_angle_deg: float,
) -> tuple[
    bool,
    Point | None,
    float | None,
    float | None,
]:
    following_index = index + 2

    if following_index >= len(marking_points):
        return False, None, None, None

    current = marking_points[index]
    next_marking = marking_points[index + 1]
    following = marking_points[following_index]

    transfer = (
        next_marking[0] - current[0],
        next_marking[1] - current[1],
    )

    outgoing = (
        following[0] - next_marking[0],
        following[1] - next_marking[1],
    )

    if index > 0:
        previous = marking_points[index - 1]
        incoming = (
            current[0] - previous[0],
            current[1] - previous[1],
        )
    else:
        incoming = (
            -outgoing[0],
            -outgoing[1],
        )

    transfer_angle = _angle_between_vectors_deg(
        incoming,
        transfer,
    )

    reversal_angle = _angle_between_vectors_deg(
        incoming,
        outgoing,
    )

    use_dummy = (
        row_transfer_min_angle_deg
        <= transfer_angle
        <= row_transfer_max_angle_deg
        and reversal_angle
        >= row_reversal_min_angle_deg
    )

    return (
        use_dummy,
        incoming,
        transfer_angle,
        reversal_angle,
    )


def compile_metric_topology(
    marking_points: Sequence[Sequence[float]],
    *,
    extension_mode: str,
    row_transition_threshold_m: float | None,
    minimum_segment_length_m: float,
    row_transfer_min_angle_deg: float,
    row_transfer_max_angle_deg: float,
    row_reversal_min_angle_deg: float,
) -> MissionTopology:
    points = tuple(
        _finite_point(
            point,
            label=f"marking point {index + 1}",
        )
        for index, point in enumerate(marking_points)
    )

    if len(points) < 2:
        raise ValueError(
            "Mission requires at least two marking points"
        )

    mode = str(extension_mode).strip().upper()

    if mode not in {"ENABLE", "DISABLE"}:
        raise ValueError(
            f"Unsupported extension_mode: {extension_mode!r}"
        )

    segments: list[SegmentTopology] = []

    for index in range(len(points) - 1):
        current = points[index]
        next_marking = points[index + 1]

        transition_distance = _distance(
            current,
            next_marking,
        )

        if (
            not math.isfinite(transition_distance)
            or transition_distance
            < minimum_segment_length_m
        ):
            raise ValueError(
                "Consecutive marking points "
                f"{index + 1} and "
                f"{index + 2} are too close"
            )

        candidate = (
            mode == "ENABLE"
            and row_transition_threshold_m is not None
            and transition_distance
            < row_transition_threshold_m
        )

        use_dummy = False
        incoming_direction = None
        transfer_angle = None
        reversal_angle = None

        if candidate:
            (
                use_dummy,
                incoming_direction,
                transfer_angle,
                reversal_angle,
            ) = _evaluate_row_transition(
                marking_points=points,
                index=index,
                row_transfer_min_angle_deg=(
                    row_transfer_min_angle_deg
                ),
                row_transfer_max_angle_deg=(
                    row_transfer_max_angle_deg
                ),
                row_reversal_min_angle_deg=(
                    row_reversal_min_angle_deg
                ),
            )

        segments.append(
            SegmentTopology(
                index=index,
                distance_m=transition_distance,
                use_dummy=use_dummy,
                incoming_direction=incoming_direction,
                transfer_angle_deg=transfer_angle,
                reversal_angle_deg=reversal_angle,
            )
        )

    return MissionTopology(
        metric_points=points,
        segments=tuple(segments),
    )


def gps_points_to_survey_local(
    gps_points: Sequence[Sequence[float]],
) -> tuple[Point, ...]:
    points = tuple(
        _finite_point(
            point,
            label=f"GPS marking point {index + 1}",
        )
        for index, point in enumerate(gps_points)
    )

    if len(points) < 2:
        raise ValueError(
            "Mission requires at least two marking points"
        )

    anchor_latitude, anchor_longitude = points[0]

    origin = GeographicOrigin(
        latitude_deg=anchor_latitude,
        longitude_deg=anchor_longitude,
    )

    metric_points: list[Point] = []

    for latitude, longitude in points:
        projected = project_geodetic_to_px4_enu(
            origin,
            latitude,
            longitude,
        )

        metric_points.append(
            (
                projected.east_m,
                projected.north_m,
            )
        )

    return tuple(metric_points)


def compile_source_topology(
    *,
    coordinate_mode: str,
    raw_marking_points: Sequence[Sequence[float]],
    extension_mode: str,
    row_transition_threshold_m: float | None,
    minimum_segment_length_m: float,
    row_transfer_min_angle_deg: float,
    row_transfer_max_angle_deg: float,
    row_reversal_min_angle_deg: float,
) -> MissionTopology:
    mode = str(coordinate_mode).strip().lower()

    if mode == "gps":
        metric_points = gps_points_to_survey_local(
            raw_marking_points
        )

    elif mode == "local":
        metric_points = tuple(
            _finite_point(
                point,
                label=f"local marking point {index + 1}",
            )
            for index, point
            in enumerate(raw_marking_points)
        )

    else:
        raise ValueError(
            f"Unsupported coordinate mode: {coordinate_mode!r}"
        )

    return compile_metric_topology(
        metric_points,
        extension_mode=extension_mode,
        row_transition_threshold_m=(
            row_transition_threshold_m
        ),
        minimum_segment_length_m=(
            minimum_segment_length_m
        ),
        row_transfer_min_angle_deg=(
            row_transfer_min_angle_deg
        ),
        row_transfer_max_angle_deg=(
            row_transfer_max_angle_deg
        ),
        row_reversal_min_angle_deg=(
            row_reversal_min_angle_deg
        ),
    )
