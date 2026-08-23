"""Pure control-geometry primitives for the RPP controller.

This module deliberately has no ROS dependencies.  It builds a derived index
over the authoritative ``/nav_path`` arrays without changing, reordering, or
renumbering any raw point.  Coordinates use the rover's local ENU layout:
``x`` is East and ``y`` is North.  Headings are mathematical ENU angles
(``atan2(dy, dx)``), and signed cross-track is positive to the left of travel.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import bisect
import hashlib
import math
import struct
from typing import Iterable, Sequence


__all__ = [
    "ActiveSemanticSpan",
    "ArcLengthTarget",
    "GeometryProgressTracker",
    "GeometryResetReason",
    "GeometrySegment",
    "GoalMetadataBinding",
    "PathCorner",
    "PathGeometryIndex",
    "PathProjection",
    "Point2D",
    "RawPathPoint",
    "SegmentProjection",
    "SemanticAnchor",
    "POINT_TYPE_DUMMY_ALIGNMENT",
    "POINT_TYPE_MARKING",
    "POINT_TYPE_PASS_THROUGH",
    "project_onto_segment",
    "is_valid_path_signature",
    "make_path_signature",
    "validate_goal_metadata",
    "wrap_angle",
]


PATH_SIGNATURE_HEX_LENGTH = 64
SEGMENT_GOAL_METADATA_VERSION = 1


POINT_TYPE_PASS_THROUGH = 0
POINT_TYPE_DUMMY_ALIGNMENT = 1
POINT_TYPE_MARKING = 2
VALID_POINT_TYPES = frozenset(
    {
        POINT_TYPE_PASS_THROUGH,
        POINT_TYPE_DUMMY_ALIGNMENT,
        POINT_TYPE_MARKING,
    }
)


@dataclass(frozen=True, slots=True)
class Point2D:
    """A finite local-ENU point in metres."""

    x: float
    y: float


@dataclass(frozen=True, slots=True)
class RawPathPoint:
    """One authoritative nav-path sample and its parallel metadata."""

    raw_index: int
    point: Point2D
    point_type: int
    marking_index: int

    @property
    def semantic_identity(self) -> str | None:
        """Return the backend marking identity when one exists."""

        if self.point_type == POINT_TYPE_MARKING:
            return f"P{self.marking_index + 1:04d}"
        return None

    @property
    def is_semantic_anchor(self) -> bool:
        """Return whether this raw point must remain a control anchor."""

        return self.point_type != POINT_TYPE_PASS_THROUGH


@dataclass(frozen=True, slots=True)
class SemanticAnchor:
    """A marking or dummy point tied to its authoritative raw index."""

    raw_index: int
    point: Point2D
    point_type: int
    marking_index: int
    identity: str | None
    s: float
    incoming_heading_rad: float | None
    outgoing_heading_rad: float | None


@dataclass(frozen=True, slots=True)
class GeometrySegment:
    """One non-zero segment in the derived geometry index."""

    segment_index: int
    raw_start_index: int
    raw_end_index: int
    start: Point2D
    end: Point2D
    length: float
    s_start: float
    s_end: float
    heading_rad: float
    leg_id: int


@dataclass(frozen=True, slots=True)
class PathCorner:
    """A heading discontinuity between two derived segments."""

    corner_index: int
    raw_index: int
    point: Point2D
    s: float
    incoming_segment_index: int
    outgoing_segment_index: int
    incoming_heading_rad: float
    outgoing_heading_rad: float
    turn_angle_rad: float
    outgoing_leg_id: int

    @property
    def turn_angle_deg(self) -> float:
        """Signed ENU turn angle in degrees; positive means left."""

        return math.degrees(self.turn_angle_rad)

    @property
    def magnitude_deg(self) -> float:
        """Absolute corner angle in degrees."""

        return abs(self.turn_angle_deg)


@dataclass(frozen=True, slots=True)
class ActiveSemanticSpan:
    """The only raw/geometric range eligible for active-goal projection."""

    start_raw_index: int
    stop_raw_index: int
    start_s: float
    stop_s: float
    first_segment_index: int | None
    last_segment_index: int | None
    active_goal_identity: str | None


@dataclass(frozen=True, slots=True)
class SegmentProjection:
    """Exact closest-point projection onto one segment."""

    segment_index: int
    t: float
    point: Point2D
    signed_cross_track_m: float
    distance_m: float
    s: float


@dataclass(frozen=True, slots=True)
class PathProjection:
    """Projection result enriched with active-path measurements."""

    segment_index: int | None
    raw_start_index: int
    raw_end_index: int
    t: float
    point: Point2D
    signed_cross_track_m: float
    distance_m: float
    projected_s: float
    progress_s: float
    monotonic_clamped: bool
    remaining_to_active_stop_m: float
    remaining_path_m: float
    next_corner_distance_m: float | None
    next_corner_angle_rad: float | None
    next_leg_heading_rad: float | None
    used_full_reacquire: bool


@dataclass(frozen=True, slots=True)
class ArcLengthTarget:
    """A point and tangent selected by cumulative path distance."""

    segment_index: int | None
    t: float
    point: Point2D
    s: float
    heading_rad: float | None


@dataclass(frozen=True, slots=True)
class GoalMetadataBinding:
    """Validated Mission Manager identity bound to this geometry snapshot."""

    goal_sequence: int
    raw_path_index: int
    point_type: int
    marking_index: int
    point_id: str | None
    active_goal_identity: str
    active_span: ActiveSemanticSpan


class GeometryResetReason(str, Enum):
    """Explicit reasons accepted by :class:`GeometryProgressTracker`."""

    INITIAL_INSTALL = "INITIAL_INSTALL"
    PATH_REPLACED = "PATH_REPLACED"
    ACTIVE_GOAL_ADVANCED = "ACTIVE_GOAL_ADVANCED"
    LOCALIZATION_JUMP = "LOCALIZATION_JUMP"
    SOURCE_CLEARED = "SOURCE_CLEARED"
    MANUAL = "MANUAL"


def make_path_signature(
    navigation_points: Sequence[Point2D | Sequence[float]],
    marking_points: Sequence[Point2D | Sequence[float]],
    point_types: Sequence[int],
    marking_indices: Sequence[int],
) -> str:
    """Return the canonical trajectory-generator v1 snapshot signature.

    This intentionally lives in the RPP package so the control node does not
    depend on Mission Manager's Python package at runtime.
    """

    digest = hashlib.sha256()
    for label, values in (
        (b"NAVIGATION", navigation_points),
        (b"MARKINGS", marking_points),
    ):
        digest.update(label)
        for value in values:
            point = _as_point(value)
            digest.update(struct.pack("!dd", point.x, point.y))

    digest.update(b"TYPES")
    digest.update(bytes(int(value) for value in point_types))
    digest.update(b"INDICES")
    for value in marking_indices:
        digest.update(struct.pack("!i", int(value)))
    return digest.hexdigest()


def is_valid_path_signature(value: str) -> bool:
    """Return whether *value* is a canonical lowercase SHA-256 digest."""

    return (
        isinstance(value, str)
        and len(value) == PATH_SIGNATURE_HEX_LENGTH
        and value == value.lower()
        and all(character in "0123456789abcdef" for character in value)
    )


def _required_int(payload: dict[str, object], name: str) -> int:
    value = payload.get(name)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"goal metadata {name} must be an integer")
    return value


def validate_goal_metadata(
    payload: dict[str, object],
    *,
    expected_path_signature: str,
    geometry: "PathGeometryIndex",
    goal_point: Point2D | Sequence[float],
    coordinate_tolerance_m: float,
) -> GoalMetadataBinding:
    """Validate a versioned semantic-goal payload against current sources."""

    if not isinstance(payload, dict):
        raise ValueError("goal metadata must be a JSON object")
    if _required_int(payload, "schema_version") != SEGMENT_GOAL_METADATA_VERSION:
        raise ValueError("unsupported goal metadata schema_version")
    signature = payload.get("path_signature")
    if not is_valid_path_signature(signature):
        raise ValueError("goal metadata path_signature is invalid")
    if signature != expected_path_signature:
        raise ValueError("goal metadata path_signature does not match geometry")

    goal_sequence = _required_int(payload, "goal_sequence")
    raw_path_index = _required_int(payload, "raw_path_index")
    point_type = _required_int(payload, "point_type")
    marking_index = _required_int(payload, "marking_index")
    if goal_sequence < 0:
        raise ValueError("goal metadata goal_sequence must be non-negative")
    if not 0 <= raw_path_index < len(geometry.raw_points):
        raise ValueError("goal metadata raw_path_index is outside the path")

    raw = geometry.raw_points[raw_path_index]
    if not raw.is_semantic_anchor:
        raise ValueError("goal metadata raw_path_index is not a semantic anchor")
    anchors = geometry.semantic_anchors
    sequence_mismatch = (
        goal_sequence >= len(anchors)
        or anchors[goal_sequence].raw_index != raw_path_index
    )
    if sequence_mismatch:
        raise ValueError("goal metadata goal_sequence does not match semantic order")
    if point_type != raw.point_type or marking_index != raw.marking_index:
        raise ValueError("goal metadata semantic fields do not match path sidecars")

    expected_point_id = raw.semantic_identity
    expected_identity = expected_point_id or (
        f"PATH:{signature}:RAW:{raw_path_index}:TYPE:{point_type}"
    )
    point_id = payload.get("point_id")
    active_identity = payload.get("active_goal_identity")
    if point_id != expected_point_id or active_identity != expected_identity:
        raise ValueError("goal metadata identity does not match semantic point")

    goal = _as_point(goal_point)
    if not math.isfinite(coordinate_tolerance_m) or coordinate_tolerance_m < 0.0:
        raise ValueError("coordinate_tolerance_m must be finite and non-negative")
    if math.hypot(goal.x - raw.point.x, goal.y - raw.point.y) > coordinate_tolerance_m:
        raise ValueError("segment goal coordinate does not match metadata raw index")

    return GoalMetadataBinding(
        goal_sequence=goal_sequence,
        raw_path_index=raw_path_index,
        point_type=point_type,
        marking_index=marking_index,
        point_id=expected_point_id,
        active_goal_identity=expected_identity,
        active_span=geometry.active_span(stop_raw_index=raw_path_index),
    )


def _as_point(value: Point2D | Sequence[float]) -> Point2D:
    if isinstance(value, Point2D):
        point = value
    else:
        if len(value) != 2:
            raise ValueError("each raw point must contain exactly x and y")
        point = Point2D(float(value[0]), float(value[1]))
    if not (math.isfinite(point.x) and math.isfinite(point.y)):
        raise ValueError("raw path coordinates must be finite")
    return point


def wrap_angle(angle_rad: float) -> float:
    """Wrap a finite angle to ``[-pi, pi]`` deterministically."""

    if not math.isfinite(angle_rad):
        raise ValueError("angle must be finite")
    wrapped = (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if wrapped == -math.pi and angle_rad > 0.0 else wrapped


def project_onto_segment(
    point: Point2D | Sequence[float],
    segment: GeometrySegment,
) -> SegmentProjection:
    """Project a point exactly onto a non-zero geometry segment.

    Signed cross-track uses ``cross(segment_vector, point - start)``;
    therefore it is positive on the left side of travel in ENU.
    """

    query = _as_point(point)
    dx = segment.end.x - segment.start.x
    dy = segment.end.y - segment.start.y
    length_sq = dx * dx + dy * dy
    if length_sq <= 0.0:
        raise ValueError("cannot project onto a zero-length geometry segment")

    rel_x = query.x - segment.start.x
    rel_y = query.y - segment.start.y
    t = max(0.0, min(1.0, (rel_x * dx + rel_y * dy) / length_sq))
    foot = Point2D(segment.start.x + t * dx, segment.start.y + t * dy)
    error_x = query.x - foot.x
    error_y = query.y - foot.y
    distance = math.hypot(error_x, error_y)
    cross = dx * rel_y - dy * rel_x
    if cross > 0.0:
        signed_distance = distance
    elif cross < 0.0:
        signed_distance = -distance
    else:
        signed_distance = 0.0

    return SegmentProjection(
        segment_index=segment.segment_index,
        t=t,
        point=foot,
        signed_cross_track_m=signed_distance,
        distance_m=distance,
        s=segment.s_start + t * segment.length,
    )


class PathGeometryIndex:
    """Immutable derived geometry and semantic index for one raw nav path."""

    __slots__ = (
        "raw_points",
        "raw_s_by_index",
        "raw_segment_to_geometry",
        "segments",
        "semantic_anchors",
        "corners",
        "total_length",
        "corner_threshold_rad",
        "_segment_s_starts",
        "_segment_s_ends",
        "_segment_raw_starts",
        "_segment_raw_ends",
        "_corner_s",
    )

    def __init__(
        self,
        raw_points: tuple[RawPathPoint, ...],
        raw_s_by_index: tuple[float, ...],
        raw_segment_to_geometry: tuple[int | None, ...],
        segments: tuple[GeometrySegment, ...],
        semantic_anchors: tuple[SemanticAnchor, ...],
        corners: tuple[PathCorner, ...],
        corner_threshold_rad: float,
    ) -> None:
        object.__setattr__(self, "raw_points", raw_points)
        object.__setattr__(self, "raw_s_by_index", raw_s_by_index)
        object.__setattr__(self, "raw_segment_to_geometry", raw_segment_to_geometry)
        object.__setattr__(self, "segments", segments)
        object.__setattr__(self, "semantic_anchors", semantic_anchors)
        object.__setattr__(self, "corners", corners)
        object.__setattr__(
            self,
            "total_length",
            raw_s_by_index[-1] if raw_s_by_index else 0.0,
        )
        object.__setattr__(self, "corner_threshold_rad", corner_threshold_rad)
        object.__setattr__(
            self,
            "_segment_s_starts",
            tuple(segment.s_start for segment in segments),
        )
        object.__setattr__(
            self,
            "_segment_s_ends",
            tuple(segment.s_end for segment in segments),
        )
        object.__setattr__(
            self,
            "_segment_raw_starts",
            tuple(segment.raw_start_index for segment in segments),
        )
        object.__setattr__(
            self,
            "_segment_raw_ends",
            tuple(segment.raw_end_index for segment in segments),
        )
        object.__setattr__(self, "_corner_s", tuple(corner.s for corner in corners))

    def __setattr__(self, name: str, value: object) -> None:
        raise AttributeError("PathGeometryIndex is immutable")

    @classmethod
    def build(
        cls,
        raw_points: Iterable[Point2D | Sequence[float]],
        point_types: Sequence[int] | None = None,
        marking_indices: Sequence[int] | None = None,
        *,
        corner_threshold_rad: float = math.radians(45.0),
        zero_length_epsilon_m: float = 1.0e-12,
    ) -> "PathGeometryIndex":
        """Build an index while retaining every authoritative raw sample."""

        points = tuple(_as_point(point) for point in raw_points)
        count = len(points)
        if point_types is None:
            types = (POINT_TYPE_PASS_THROUGH,) * count
        else:
            types = tuple(int(value) for value in point_types)
        if marking_indices is None:
            indices = (-1,) * count
        else:
            indices = tuple(int(value) for value in marking_indices)

        if count == 0:
            raise ValueError("raw path must contain at least one point")
        if len(types) != count or len(indices) != count:
            raise ValueError("raw path and metadata lengths must match")
        if not math.isfinite(corner_threshold_rad) or not (
            0.0 <= corner_threshold_rad <= math.pi
        ):
            raise ValueError("corner_threshold_rad must be in [0, pi]")
        if not math.isfinite(zero_length_epsilon_m) or zero_length_epsilon_m < 0.0:
            raise ValueError("zero_length_epsilon_m must be finite and non-negative")

        raw: list[RawPathPoint] = []
        observed_markings: list[int] = []
        for raw_index, (point, point_type, marking_index) in enumerate(
            zip(points, types, indices)
        ):
            if point_type not in VALID_POINT_TYPES:
                raise ValueError(f"unsupported point type {point_type} at {raw_index}")
            if point_type == POINT_TYPE_MARKING:
                if marking_index < 0:
                    raise ValueError(
                        "marking points require a non-negative marking index"
                    )
                observed_markings.append(marking_index)
            elif marking_index != -1:
                raise ValueError("non-marking points must use marking index -1")
            raw.append(
                RawPathPoint(raw_index, point, point_type, marking_index)
            )
        if observed_markings != list(range(len(observed_markings))):
            raise ValueError(
                "marking indices must appear exactly once in zero-based order"
            )

        raw_lengths = [
            math.hypot(
                points[index + 1].x - points[index].x,
                points[index + 1].y - points[index].y,
            )
            for index in range(count - 1)
        ]
        raw_s = [0.0]
        cumulative_s = 0.0
        for length in raw_lengths:
            cumulative_s = math.fsum((cumulative_s, length))
            raw_s.append(cumulative_s)

        base_segments: list[GeometrySegment] = []
        raw_mapping: list[int | None] = [None] * max(0, count - 1)
        for raw_start, length in enumerate(raw_lengths):
            if length <= zero_length_epsilon_m:
                continue
            start = points[raw_start]
            end = points[raw_start + 1]
            segment_index = len(base_segments)
            segment = GeometrySegment(
                segment_index=segment_index,
                raw_start_index=raw_start,
                raw_end_index=raw_start + 1,
                start=start,
                end=end,
                length=length,
                s_start=raw_s[raw_start],
                s_end=raw_s[raw_start + 1],
                heading_rad=math.atan2(end.y - start.y, end.x - start.x),
                leg_id=0,
            )
            base_segments.append(segment)
            raw_mapping[raw_start] = segment_index

        corners: list[PathCorner] = []
        leg_id = 0
        segments: list[GeometrySegment] = []
        for segment_index, segment in enumerate(base_segments):
            if segment_index > 0:
                previous = base_segments[segment_index - 1]
                turn = wrap_angle(segment.heading_rad - previous.heading_rad)
                if (
                    abs(turn) > 1.0e-15
                    and abs(turn) + 1.0e-15 >= corner_threshold_rad
                ):
                    leg_id += 1
                    corner_raw = cls._corner_raw_index(raw, previous, segment)
                    corners.append(
                        PathCorner(
                            corner_index=len(corners),
                            raw_index=corner_raw,
                            point=raw[corner_raw].point,
                            s=raw_s[corner_raw],
                            incoming_segment_index=previous.segment_index,
                            outgoing_segment_index=segment.segment_index,
                            incoming_heading_rad=previous.heading_rad,
                            outgoing_heading_rad=segment.heading_rad,
                            turn_angle_rad=turn,
                            outgoing_leg_id=leg_id,
                        )
                    )
            segments.append(replace(segment, leg_id=leg_id))

        segment_raw_starts = [segment.raw_start_index for segment in segments]
        segment_raw_ends = [segment.raw_end_index for segment in segments]
        anchors_list: list[SemanticAnchor] = []
        for entry in raw:
            if not entry.is_semantic_anchor:
                continue
            incoming_position = bisect.bisect_right(
                segment_raw_ends, entry.raw_index
            ) - 1
            outgoing_position = bisect.bisect_left(
                segment_raw_starts, entry.raw_index
            )
            anchors_list.append(
                SemanticAnchor(
                    raw_index=entry.raw_index,
                    point=entry.point,
                    point_type=entry.point_type,
                    marking_index=entry.marking_index,
                    identity=entry.semantic_identity,
                    s=raw_s[entry.raw_index],
                    incoming_heading_rad=(
                        segments[incoming_position].heading_rad
                        if incoming_position >= 0
                        else None
                    ),
                    outgoing_heading_rad=(
                        segments[outgoing_position].heading_rad
                        if outgoing_position < len(segments)
                        else None
                    ),
                )
            )
        anchors = tuple(anchors_list)

        return cls(
            raw_points=tuple(raw),
            raw_s_by_index=tuple(raw_s),
            raw_segment_to_geometry=tuple(raw_mapping),
            segments=tuple(segments),
            semantic_anchors=anchors,
            corners=tuple(corners),
            corner_threshold_rad=corner_threshold_rad,
        )

    @staticmethod
    def _corner_raw_index(
        raw_points: Sequence[RawPathPoint],
        incoming: GeometrySegment,
        outgoing: GeometrySegment,
    ) -> int:
        for raw_index in range(incoming.raw_end_index, outgoing.raw_start_index + 1):
            if raw_points[raw_index].is_semantic_anchor:
                return raw_index
        return outgoing.raw_start_index

    def semantic_anchor_at(self, raw_index: int) -> SemanticAnchor | None:
        """Return semantic metadata for a raw index, if present."""

        self._validate_raw_index(raw_index)
        for anchor in self.semantic_anchors:
            if anchor.raw_index == raw_index:
                return anchor
        return None

    def active_span(
        self,
        *,
        stop_raw_index: int,
        start_raw_index: int | None = None,
    ) -> ActiveSemanticSpan:
        """Create an intersection-safe span ending at an active semantic goal."""

        self._validate_raw_index(stop_raw_index)
        if start_raw_index is None:
            prior = [
                anchor.raw_index
                for anchor in self.semantic_anchors
                if anchor.raw_index < stop_raw_index
            ]
            start_raw_index = prior[-1] if prior else 0
        self._validate_raw_index(start_raw_index)
        if start_raw_index > stop_raw_index:
            raise ValueError("active span start must not follow its stop")

        first_segment, last_segment = self._segment_bounds(
            start_raw_index, stop_raw_index
        )
        goal = self.semantic_anchor_at(stop_raw_index)
        if goal is None:
            raise ValueError("active span stop must be a semantic anchor")
        return ActiveSemanticSpan(
            start_raw_index=start_raw_index,
            stop_raw_index=stop_raw_index,
            start_s=self.raw_s_by_index[start_raw_index],
            stop_s=self.raw_s_by_index[stop_raw_index],
            first_segment_index=first_segment,
            last_segment_index=last_segment,
            active_goal_identity=goal.identity,
        )

    def _segment_bounds(
        self, start_raw_index: int, stop_raw_index: int
    ) -> tuple[int | None, int | None]:
        """Return exact eligible segment bounds for a raw-index span."""

        first = bisect.bisect_left(self._segment_raw_starts, start_raw_index)
        stop_exclusive = bisect.bisect_right(self._segment_raw_ends, stop_raw_index)
        last = stop_exclusive - 1
        if (
            first >= len(self.segments)
            or first > last
            or self.segments[first].raw_end_index > stop_raw_index
        ):
            return None, None
        return first, last

    def project(
        self,
        point: Point2D | Sequence[float],
        *,
        active_span: ActiveSemanticSpan,
        hint_segment_index: int | None = None,
        back_window_segments: int = 2,
        forward_window_segments: int = 4,
        reference_s: float | None = None,
        max_backward_jump_m: float = 0.10,
        max_forward_jump_m: float | None = None,
        full_reacquire_distance_m: float | None = None,
    ) -> PathProjection:
        """Project within an active semantic span and a local segment window.

        A full-span search occurs only when ``full_reacquire_distance_m`` is
        supplied and the best local result is farther than that threshold.
        Jump limits constrain both local and reacquisition candidates relative
        to ``reference_s``.  The bounded backward allowance admits ordinary
        estimator noise but prevents a full reacquisition from selecting an
        arbitrarily old branch of a self-intersecting path.
        """

        query = _as_point(point)
        self._validate_span(active_span)
        for name, value in (
            ("back_window_segments", back_window_segments),
            ("forward_window_segments", forward_window_segments),
        ):
            if isinstance(value, bool) or int(value) != value or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        if reference_s is not None and not math.isfinite(reference_s):
            raise ValueError("reference_s must be finite")
        if (
            isinstance(max_backward_jump_m, bool)
            or not math.isfinite(max_backward_jump_m)
            or max_backward_jump_m < 0.0
        ):
            raise ValueError(
                "max_backward_jump_m must be finite and non-negative"
            )
        if max_forward_jump_m is not None and (
            not math.isfinite(max_forward_jump_m) or max_forward_jump_m < 0.0
        ):
            raise ValueError("max_forward_jump_m must be finite and non-negative")
        if full_reacquire_distance_m is not None and (
            not math.isfinite(full_reacquire_distance_m)
            or full_reacquire_distance_m < 0.0
        ):
            raise ValueError(
                "full_reacquire_distance_m must be finite and non-negative"
            )

        if active_span.first_segment_index is None:
            anchor = self.raw_points[active_span.stop_raw_index]
            distance = math.hypot(query.x - anchor.point.x, query.y - anchor.point.y)
            return self._enrich_projection(
                segment_projection=None,
                raw_start=anchor.raw_index,
                raw_end=anchor.raw_index,
                projected_point=anchor.point,
                distance_m=distance,
                projected_s=active_span.stop_s,
                active_span=active_span,
                used_full_reacquire=False,
            )

        first = active_span.first_segment_index
        last = active_span.last_segment_index
        assert first is not None and last is not None
        if hint_segment_index is None or not (first <= hint_segment_index <= last):
            local_first, local_last = first, last
        else:
            local_first = max(first, hint_segment_index - back_window_segments)
            local_last = min(last, hint_segment_index + forward_window_segments)

        used_full = False
        local_error: ValueError | None = None
        try:
            local = self._best_projection(
                query,
                range(local_first, local_last + 1),
                reference_s=reference_s,
                max_backward_jump_m=max_backward_jump_m,
                max_forward_jump_m=max_forward_jump_m,
            )
        except ValueError as error:
            local_error = error

        local_is_partial = local_first != first or local_last != last
        should_reacquire = (
            full_reacquire_distance_m is not None
            and local_is_partial
            and (
                local_error is not None
                or local.distance_m > full_reacquire_distance_m
            )
        )
        if should_reacquire:
            local = self._best_projection(
                query,
                range(first, last + 1),
                reference_s=reference_s,
                max_backward_jump_m=max_backward_jump_m,
                max_forward_jump_m=max_forward_jump_m,
            )
            used_full = True
        elif local_error is not None:
            raise local_error

        segment = self.segments[local.segment_index]
        return self._enrich_projection(
            segment_projection=local,
            raw_start=segment.raw_start_index,
            raw_end=segment.raw_end_index,
            projected_point=local.point,
            distance_m=local.distance_m,
            projected_s=local.s,
            active_span=active_span,
            used_full_reacquire=used_full,
        )

    def _best_projection(
        self,
        query: Point2D,
        candidate_indices: Iterable[int],
        *,
        reference_s: float | None,
        max_backward_jump_m: float,
        max_forward_jump_m: float | None,
    ) -> SegmentProjection:
        candidates: list[SegmentProjection] = []
        for segment_index in candidate_indices:
            projection = project_onto_segment(query, self.segments[segment_index])
            if (
                reference_s is not None
                and projection.s < reference_s - max_backward_jump_m
            ):
                continue
            if (
                reference_s is not None
                and max_forward_jump_m is not None
                and projection.s > reference_s + max_forward_jump_m
            ):
                continue
            candidates.append(projection)
        if not candidates:
            raise ValueError("jump bounds exclude every projection candidate")
        reference = reference_s if reference_s is not None else candidates[0].s
        return min(
            candidates,
            key=lambda result: (
                result.distance_m,
                abs(result.s - reference),
                result.segment_index,
            ),
        )

    def _enrich_projection(
        self,
        *,
        segment_projection: SegmentProjection | None,
        raw_start: int,
        raw_end: int,
        projected_point: Point2D,
        distance_m: float,
        projected_s: float,
        active_span: ActiveSemanticSpan,
        used_full_reacquire: bool,
    ) -> PathProjection:
        next_corner = self.next_corner(projected_s)
        return PathProjection(
            segment_index=(
                segment_projection.segment_index if segment_projection else None
            ),
            raw_start_index=raw_start,
            raw_end_index=raw_end,
            t=segment_projection.t if segment_projection else 0.0,
            point=projected_point,
            signed_cross_track_m=(
                segment_projection.signed_cross_track_m
                if segment_projection
                else 0.0
            ),
            distance_m=distance_m,
            projected_s=projected_s,
            progress_s=projected_s,
            monotonic_clamped=False,
            remaining_to_active_stop_m=max(0.0, active_span.stop_s - projected_s),
            remaining_path_m=max(0.0, self.total_length - projected_s),
            next_corner_distance_m=(
                max(0.0, next_corner.s - projected_s) if next_corner else None
            ),
            next_corner_angle_rad=(
                next_corner.turn_angle_rad if next_corner else None
            ),
            next_leg_heading_rad=(
                next_corner.outgoing_heading_rad if next_corner else None
            ),
            used_full_reacquire=used_full_reacquire,
        )

    def point_at_s(
        self,
        s: float,
        *,
        active_span: ActiveSemanticSpan | None = None,
    ) -> ArcLengthTarget:
        """Return the exact interpolated target at cumulative distance ``s``."""

        if not math.isfinite(s):
            raise ValueError("s must be finite")
        lower = active_span.start_s if active_span else 0.0
        upper = active_span.stop_s if active_span else self.total_length
        if active_span:
            self._validate_span(active_span)
        target_s = max(lower, min(upper, s))
        if not self.segments or lower == upper:
            raw_index = (
                active_span.stop_raw_index
                if active_span
                else (0 if target_s <= 0.0 else len(self.raw_points) - 1)
            )
            return ArcLengthTarget(
                segment_index=None,
                t=0.0,
                point=self.raw_points[raw_index].point,
                s=target_s,
                heading_rad=None,
            )

        first_candidate = 0
        last_candidate = len(self.segments) - 1
        if active_span and active_span.first_segment_index is not None:
            assert active_span.last_segment_index is not None
            first_candidate = active_span.first_segment_index
            last_candidate = active_span.last_segment_index
        if first_candidate > last_candidate:
            return ArcLengthTarget(
                segment_index=None,
                t=0.0,
                point=self.raw_points[active_span.stop_raw_index].point,
                s=target_s,
                heading_rad=None,
            )
        selected_index = bisect.bisect_left(
            self._segment_s_ends,
            target_s - 1.0e-12,
            lo=first_candidate,
            hi=last_candidate + 1,
        )
        selected_index = min(last_candidate, max(first_candidate, selected_index))
        selected = self.segments[selected_index]
        t = max(
            0.0,
            min(1.0, (target_s - selected.s_start) / selected.length),
        )
        return ArcLengthTarget(
            segment_index=selected.segment_index,
            t=t,
            point=Point2D(
                selected.start.x + t * (selected.end.x - selected.start.x),
                selected.start.y + t * (selected.end.y - selected.start.y),
            ),
            s=target_s,
            heading_rad=selected.heading_rad,
        )

    def lookahead_target(
        self,
        progress_s: float,
        lookahead_m: float,
        *,
        active_span: ActiveSemanticSpan | None = None,
    ) -> ArcLengthTarget:
        """Return an arc-length target, clamped to the requested span."""

        if not math.isfinite(lookahead_m) or lookahead_m < 0.0:
            raise ValueError("lookahead_m must be finite and non-negative")
        return self.point_at_s(progress_s + lookahead_m, active_span=active_span)

    def next_corner(self, s: float) -> PathCorner | None:
        """Return the first corner strictly after cumulative distance ``s``."""

        if not math.isfinite(s):
            raise ValueError("s must be finite")
        index = bisect.bisect_right(self._corner_s, s)
        return self.corners[index] if index < len(self.corners) else None

    def _validate_raw_index(self, raw_index: int) -> None:
        if isinstance(raw_index, bool) or not isinstance(raw_index, int):
            raise ValueError("raw index must be an integer")
        if not (0 <= raw_index < len(self.raw_points)):
            raise ValueError("raw index is outside the path")

    def _validate_span(self, span: ActiveSemanticSpan) -> None:
        self._validate_raw_index(span.start_raw_index)
        self._validate_raw_index(span.stop_raw_index)
        if span.start_raw_index > span.stop_raw_index:
            raise ValueError("active span start must not follow its stop")
        if (
            span.start_s != self.raw_s_by_index[span.start_raw_index]
            or span.stop_s != self.raw_s_by_index[span.stop_raw_index]
        ):
            raise ValueError("active span does not belong to this geometry index")
        goal = self.semantic_anchor_at(span.stop_raw_index)
        if goal is None or span.active_goal_identity != goal.identity:
            raise ValueError("active span stop is not this path's semantic anchor")
        expected_first, expected_last = self._segment_bounds(
            span.start_raw_index, span.stop_raw_index
        )
        if (
            span.first_segment_index != expected_first
            or span.last_segment_index != expected_last
        ):
            raise ValueError("active span segment bounds do not match semantic limits")


class GeometryProgressTracker:
    """Stateful monotonic progress over one immutable geometry index."""

    __slots__ = (
        "geometry",
        "hint_segment_index",
        "progress_s",
        "last_reset_reason",
        "reset_count",
    )

    def __init__(self, geometry: PathGeometryIndex) -> None:
        self.geometry = geometry
        self.hint_segment_index: int | None = None
        self.progress_s: float | None = None
        self.last_reset_reason = GeometryResetReason.INITIAL_INSTALL
        self.reset_count = 1

    def reset(
        self,
        reason: GeometryResetReason,
        *,
        progress_s: float | None = None,
        hint_segment_index: int | None = None,
    ) -> None:
        """Explicitly release monotonic history for a logged reason."""

        if not isinstance(reason, GeometryResetReason):
            raise ValueError("reset reason must be a GeometryResetReason")
        if progress_s is not None and (
            not math.isfinite(progress_s)
            or not (0.0 <= progress_s <= self.geometry.total_length)
        ):
            raise ValueError("reset progress_s is outside the path")
        if hint_segment_index is not None and not (
            0 <= hint_segment_index < len(self.geometry.segments)
        ):
            raise ValueError("reset hint segment is outside the path")
        self.progress_s = progress_s
        self.hint_segment_index = hint_segment_index
        self.last_reset_reason = reason
        self.reset_count += 1

    def update(
        self,
        point: Point2D | Sequence[float],
        *,
        active_span: ActiveSemanticSpan,
        back_window_segments: int = 2,
        forward_window_segments: int = 4,
        max_backward_jump_m: float = 0.10,
        max_forward_jump_m: float | None = None,
        full_reacquire_distance_m: float | None = None,
    ) -> PathProjection:
        """Project and advance monotonic progress without backward snap."""

        reference_s = self.progress_s
        result = self.geometry.project(
            point,
            active_span=active_span,
            hint_segment_index=self.hint_segment_index,
            back_window_segments=back_window_segments,
            forward_window_segments=forward_window_segments,
            reference_s=reference_s,
            max_backward_jump_m=max_backward_jump_m,
            max_forward_jump_m=max_forward_jump_m,
            full_reacquire_distance_m=full_reacquire_distance_m,
        )
        lower_bound = active_span.start_s
        old_progress = (
            max(lower_bound, min(active_span.stop_s, reference_s))
            if reference_s is not None
            else lower_bound
        )
        progress = max(old_progress, result.projected_s)
        clamped = progress > result.projected_s + 1.0e-12
        self.progress_s = progress
        if result.segment_index is not None:
            self.hint_segment_index = result.segment_index
        next_corner = self.geometry.next_corner(progress)
        return replace(
            result,
            progress_s=progress,
            monotonic_clamped=clamped,
            remaining_to_active_stop_m=max(0.0, active_span.stop_s - progress),
            remaining_path_m=max(0.0, self.geometry.total_length - progress),
            next_corner_distance_m=(
                max(0.0, next_corner.s - progress) if next_corner else None
            ),
            next_corner_angle_rad=(
                next_corner.turn_angle_rad if next_corner else None
            ),
            next_leg_heading_rad=(
                next_corner.outgoing_heading_rad if next_corner else None
            ),
        )
