import math

import pytest

from rpp_controller.path_geometry import (
    GeometryProgressTracker,
    GeometryResetReason,
    PathGeometryIndex,
    Point2D,
    POINT_TYPE_DUMMY_ALIGNMENT,
    POINT_TYPE_MARKING,
    POINT_TYPE_PASS_THROUGH,
    is_valid_path_signature,
    make_path_signature,
    project_onto_segment,
    validate_goal_metadata,
)


def build_path(points, *, types=None, markings=None, corner_deg=45.0):
    return PathGeometryIndex.build(
        points,
        point_types=types,
        marking_indices=markings,
        corner_threshold_rad=math.radians(corner_deg),
    )


@pytest.mark.parametrize(
    ("points", "query", "expected_point", "expected_t", "expected_ct"),
    [
        ([(0, 0), (10, 0)], (4, 2), (4, 0), 0.4, 2.0),
        ([(0, 0), (0, 10)], (-2, 4), (0, 4), 0.4, 2.0),
        ([(0, 0), (10, 10)], (4, 6), (5, 5), 0.5, math.sqrt(2.0)),
    ],
)
def test_horizontal_vertical_and_diagonal_projection(
    points, query, expected_point, expected_t, expected_ct
):
    geometry = build_path(points)
    result = project_onto_segment(query, geometry.segments[0])

    assert result.point.x == pytest.approx(expected_point[0])
    assert result.point.y == pytest.approx(expected_point[1])
    assert result.t == pytest.approx(expected_t)
    assert result.signed_cross_track_m == pytest.approx(expected_ct)


@pytest.mark.parametrize(
    ("query", "expected_t", "expected_x"),
    [((-1, 3), 0.0, 0.0), ((4, 3), 0.4, 4.0), ((12, 3), 1.0, 10.0)],
)
def test_projection_clamps_before_inside_and_after_segment(
    query, expected_t, expected_x
):
    geometry = build_path([(0, 0), (10, 0)])

    result = project_onto_segment(query, geometry.segments[0])

    assert result.t == pytest.approx(expected_t)
    assert result.point.x == pytest.approx(expected_x)
    assert result.point.y == pytest.approx(0.0)


def test_signed_cross_track_is_positive_left_and_negative_right_in_enu():
    eastbound = build_path([(0, 0), (10, 0)]).segments[0]
    northbound = build_path([(0, 0), (0, 10)]).segments[0]

    assert project_onto_segment((5, 2), eastbound).signed_cross_track_m > 0
    assert project_onto_segment((5, -2), eastbound).signed_cross_track_m < 0
    assert project_onto_segment((-2, 5), northbound).signed_cross_track_m > 0
    assert project_onto_segment((2, 5), northbound).signed_cross_track_m < 0


def test_raw_points_and_metadata_are_preserved_without_mutating_inputs():
    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    types = [POINT_TYPE_PASS_THROUGH, POINT_TYPE_DUMMY_ALIGNMENT, POINT_TYPE_MARKING]
    markings = [-1, -1, 0]
    original = (list(points), list(types), list(markings))

    geometry = build_path(points, types=types, markings=markings)

    assert points == original[0]
    assert types == original[1]
    assert markings == original[2]
    assert [entry.point for entry in geometry.raw_points] == [
        Point2D(*point) for point in points
    ]
    assert [entry.point_type for entry in geometry.raw_points] == types
    assert [entry.marking_index for entry in geometry.raw_points] == markings
    assert [anchor.raw_index for anchor in geometry.semantic_anchors] == [1, 2]
    assert [anchor.identity for anchor in geometry.semantic_anchors] == [None, "P0001"]


def test_cumulative_distance_and_exact_raw_segment_mapping():
    geometry = build_path([(0, 0), (3, 4), (3, 8)])

    assert geometry.raw_s_by_index == pytest.approx((0.0, 5.0, 9.0))
    assert geometry.total_length == pytest.approx(9.0)
    assert geometry.raw_segment_to_geometry == (0, 1)
    assert [segment.raw_start_index for segment in geometry.segments] == [0, 1]
    assert [segment.raw_end_index for segment in geometry.segments] == [1, 2]


def test_zero_length_raw_segments_are_retained_but_excluded_from_geometry():
    geometry = build_path(
        [(0, 0), (0, 0), (1, 0)],
        types=[POINT_TYPE_PASS_THROUGH, POINT_TYPE_DUMMY_ALIGNMENT, POINT_TYPE_MARKING],
        markings=[-1, -1, 0],
    )

    assert len(geometry.raw_points) == 3
    assert geometry.raw_s_by_index == pytest.approx((0.0, 0.0, 1.0))
    assert geometry.raw_segment_to_geometry == (None, 0)
    assert len(geometry.segments) == 1
    assert [anchor.raw_index for anchor in geometry.semantic_anchors] == [1, 2]


def test_all_zero_length_path_projects_to_active_anchor_without_crashing():
    geometry = build_path(
        [(2, 3), (2, 3)],
        types=[POINT_TYPE_PASS_THROUGH, POINT_TYPE_MARKING],
        markings=[-1, 0],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=1)

    result = geometry.project((4, 3), active_span=span)

    assert result.segment_index is None
    assert result.point == Point2D(2.0, 3.0)
    assert result.distance_m == pytest.approx(2.0)
    assert result.projected_s == 0.0
    assert result.remaining_to_active_stop_m == 0.0


def test_corner_detection_records_signed_angle_next_heading_and_legs():
    geometry = build_path(
        [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)],
        types=[
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_MARKING,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_MARKING,
        ],
        markings=[-1, -1, 0, -1, 1],
    )

    assert len(geometry.corners) == 1
    corner = geometry.corners[0]
    assert corner.raw_index == 2
    assert corner.s == pytest.approx(2.0)
    assert corner.turn_angle_deg == pytest.approx(90.0)
    assert corner.outgoing_heading_rad == pytest.approx(math.pi / 2.0)
    assert [segment.leg_id for segment in geometry.segments] == [0, 0, 1, 1]


def test_corner_below_threshold_is_not_tagged():
    ten_degrees = math.radians(10.0)
    geometry = build_path(
        [(0, 0), (1, 0), (1 + math.cos(ten_degrees), math.sin(ten_degrees))],
        corner_deg=45.0,
    )

    assert geometry.corners == ()


def test_zero_corner_threshold_does_not_turn_collinear_fill_into_corners():
    geometry = build_path(
        [(0, 0), (1, 0), (2, 0), (3, 0)],
        corner_deg=0.0,
    )

    assert geometry.corners == ()


def test_semantic_anchor_carries_incoming_and_outgoing_headings():
    geometry = build_path(
        [(0, 0), (1, 0), (1, 1)],
        types=[POINT_TYPE_PASS_THROUGH, POINT_TYPE_MARKING, POINT_TYPE_MARKING],
        markings=[-1, 0, 1],
    )

    first = geometry.semantic_anchor_at(1)
    assert first is not None
    assert first.identity == "P0001"
    assert first.incoming_heading_rad == pytest.approx(0.0)
    assert first.outgoing_heading_rad == pytest.approx(math.pi / 2.0)


def test_active_span_defaults_to_previous_semantic_anchor():
    geometry = build_path(
        [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)],
        types=[
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_MARKING,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_MARKING,
        ],
        markings=[-1, 0, -1, -1, 1],
    )

    span = geometry.active_span(stop_raw_index=4)

    assert span.start_raw_index == 1
    assert span.stop_raw_index == 4
    assert span.start_s == pytest.approx(1.0)
    assert span.stop_s == pytest.approx(4.0)
    assert span.first_segment_index == 1
    assert span.last_segment_index == 3
    assert span.active_goal_identity == "P0002"


def test_active_span_requires_a_semantic_stop():
    geometry = build_path([(0, 0), (1, 0)])

    with pytest.raises(ValueError, match="semantic anchor"):
        geometry.active_span(start_raw_index=0, stop_raw_index=1)


def test_projection_reports_remaining_stop_full_path_and_corner_preview():
    geometry = build_path(
        [(0, 0), (2, 0), (2, 2), (4, 2)],
        types=[
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_MARKING,
            POINT_TYPE_MARKING,
            POINT_TYPE_MARKING,
        ],
        markings=[-1, 0, 1, 2],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=1)

    result = geometry.project((0.5, 0.2), active_span=span)

    assert result.projected_s == pytest.approx(0.5)
    assert result.remaining_to_active_stop_m == pytest.approx(1.5)
    assert result.remaining_path_m == pytest.approx(5.5)
    assert result.next_corner_distance_m == pytest.approx(1.5)
    assert result.next_corner_angle_rad == pytest.approx(math.pi / 2.0)
    assert result.next_leg_heading_rad == pytest.approx(math.pi / 2.0)


def test_arc_length_target_and_lookahead_cross_a_corner():
    geometry = build_path([(0, 0), (2, 0), (2, 2)])

    target = geometry.point_at_s(3.0)
    lookahead = geometry.lookahead_target(1.5, 1.5)

    assert target.point == Point2D(2.0, 1.0)
    assert target.heading_rad == pytest.approx(math.pi / 2.0)
    assert lookahead == target


def test_arc_target_is_clamped_to_active_semantic_stop():
    geometry = build_path(
        [(0, 0), (2, 0), (4, 0)],
        types=[POINT_TYPE_PASS_THROUGH, POINT_TYPE_MARKING, POINT_TYPE_MARKING],
        markings=[-1, 0, 1],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=1)

    target = geometry.lookahead_target(1.5, 10.0, active_span=span)

    assert target.s == pytest.approx(2.0)
    assert target.point == Point2D(2.0, 0.0)


def test_windowed_projection_stays_near_hint_at_self_intersection():
    # Both segment 0 and segment 4 pass through the origin.  The active span
    # permits both, so the local hint is the protection against future snap.
    geometry = build_path(
        [(-2, -2), (0, 0), (2, 2), (2, -2), (0, 0), (-2, 2)],
        types=[
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_MARKING,
        ],
        markings=[-1, -1, -1, -1, -1, 0],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=5)

    result = geometry.project(
        (0, 0),
        active_span=span,
        hint_segment_index=0,
        back_window_segments=0,
        forward_window_segments=1,
    )

    assert result.segment_index == 0
    assert result.projected_s < geometry.segments[4].s_start


def test_active_span_prevents_future_intersection_selection():
    geometry = build_path(
        [(-2, -2), (0, 0), (2, 2), (2, -2), (0, 0), (-2, 2)],
        types=[
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_MARKING,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_MARKING,
        ],
        markings=[-1, 0, -1, -1, -1, 1],
    )
    first_span = geometry.active_span(start_raw_index=0, stop_raw_index=1)

    result = geometry.project((0, 0), active_span=first_span)

    assert result.segment_index == 0
    assert result.raw_end_index == 1


def test_forward_jump_bound_rejects_future_intersection():
    geometry = build_path(
        [(-2, -2), (0, 0), (2, 2), (2, -2), (0, 0), (-2, 2)],
        types=[
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_MARKING,
        ],
        markings=[-1, -1, -1, -1, -1, 0],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=5)

    result = geometry.project(
        (0, 0),
        active_span=span,
        reference_s=0.5,
        max_forward_jump_m=3.0,
    )

    assert result.segment_index == 0


def test_full_reacquire_is_explicit_and_still_confined_to_active_span():
    geometry = build_path(
        [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0)],
        types=[
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_MARKING,
        ],
        markings=[-1, -1, -1, -1, -1, 0],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=5)

    result = geometry.project(
        (4.5, 0),
        active_span=span,
        hint_segment_index=0,
        back_window_segments=0,
        forward_window_segments=1,
        full_reacquire_distance_m=0.5,
    )

    assert result.used_full_reacquire
    assert result.segment_index == 4


def test_bounded_full_reacquire_runs_when_local_candidates_are_all_behind():
    geometry = build_path(
        [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0)],
        types=[POINT_TYPE_PASS_THROUGH] * 5 + [POINT_TYPE_MARKING],
        markings=[-1, -1, -1, -1, -1, 0],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=5)

    result = geometry.project(
        (4.3, 0),
        active_span=span,
        hint_segment_index=0,
        back_window_segments=0,
        forward_window_segments=1,
        reference_s=4.2,
        max_backward_jump_m=0.1,
        full_reacquire_distance_m=0.5,
    )

    assert result.used_full_reacquire
    assert result.segment_index == 4
    assert result.projected_s == pytest.approx(4.3)


def test_backward_bound_blocks_old_self_intersection_during_full_reacquire():
    geometry = build_path(
        [(-2, -2), (0, 0), (2, 2), (2, -2), (0, 0), (-2, 2)],
        types=[POINT_TYPE_PASS_THROUGH] * 5 + [POINT_TYPE_MARKING],
        markings=[-1, -1, -1, -1, -1, 0],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=5)
    tracker = GeometryProgressTracker(geometry)
    first_pose = (-0.1, 0.1)
    first = tracker.update(
        first_pose,
        active_span=span,
        max_backward_jump_m=0.5,
    )
    assert first.segment_index == 4

    second_pose = (0.1, 0.1)
    assert math.dist(first_pose, second_pose) < 0.5
    reacquired = tracker.update(
        second_pose,
        active_span=span,
        back_window_segments=0,
        forward_window_segments=0,
        max_backward_jump_m=0.5,
        full_reacquire_distance_m=0.05,
    )

    assert reacquired.used_full_reacquire
    # The exact origin is the shared endpoint of segments 3 and 4, so either
    # adjacent segment is a valid representation of current progress.  The old
    # segment-0 branch must never win the full-span distance tie.
    assert reacquired.segment_index in (3, 4)
    assert reacquired.segment_index != 0
    assert reacquired.projected_s >= first.progress_s - 0.5
    assert reacquired.progress_s == pytest.approx(first.progress_s)


@pytest.mark.parametrize("value", [-0.01, math.inf, math.nan, True])
def test_backward_jump_bound_must_be_finite_non_negative_number(value):
    geometry = build_path(
        [(0, 0), (1, 0)],
        types=[POINT_TYPE_PASS_THROUGH, POINT_TYPE_MARKING],
        markings=[-1, 0],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=1)

    with pytest.raises(ValueError, match="max_backward_jump_m"):
        geometry.project(
            (0.5, 0),
            active_span=span,
            max_backward_jump_m=value,
        )


def test_backward_bound_reports_when_no_local_or_full_candidate_is_valid():
    geometry = build_path(
        [(0, 0), (1, 0), (2, 0), (3, 0)],
        types=[POINT_TYPE_PASS_THROUGH] * 3 + [POINT_TYPE_MARKING],
        markings=[-1, -1, -1, 0],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=3)

    with pytest.raises(ValueError, match="jump bounds exclude every"):
        geometry.project(
            (0, 0),
            active_span=span,
            hint_segment_index=0,
            back_window_segments=0,
            forward_window_segments=0,
            reference_s=3.0,
            max_backward_jump_m=0.0,
            full_reacquire_distance_m=0.1,
        )


def test_small_backward_projection_noise_is_allowed_then_progress_is_clamped():
    geometry = build_path(
        [(0, 0), (1, 0), (2, 0)],
        types=[POINT_TYPE_PASS_THROUGH] * 2 + [POINT_TYPE_MARKING],
        markings=[-1, -1, 0],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=2)
    tracker = GeometryProgressTracker(geometry)
    tracker.reset(GeometryResetReason.MANUAL, progress_s=1.0, hint_segment_index=0)

    result = tracker.update(
        (0.95, 0),
        active_span=span,
        max_backward_jump_m=0.1,
    )

    assert result.projected_s == pytest.approx(0.95)
    assert result.progress_s == pytest.approx(1.0)
    assert result.monotonic_clamped


def test_next_corner_is_strictly_future_at_corner_boundary():
    geometry = build_path([(0, 0), (1, 0), (1, 1), (2, 1)])
    first_corner, second_corner = geometry.corners

    assert geometry.next_corner(first_corner.s - 1.0e-6) is first_corner
    assert geometry.next_corner(first_corner.s) is second_corner
    assert geometry.next_corner(first_corner.s + 1.0e-6) is second_corner


def test_progress_enrichment_skips_consumed_corner_after_active_goal_reset():
    geometry = build_path(
        [(0, 0), (1, 0), (1, 1), (2, 1)],
        types=[
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_MARKING,
            POINT_TYPE_PASS_THROUGH,
            POINT_TYPE_MARKING,
        ],
        markings=[-1, 0, -1, 1],
    )
    first_corner, second_corner = geometry.corners
    next_span = geometry.active_span(start_raw_index=1, stop_raw_index=3)
    tracker = GeometryProgressTracker(geometry)
    tracker.reset(
        GeometryResetReason.ACTIVE_GOAL_ADVANCED,
        progress_s=first_corner.s,
        hint_segment_index=first_corner.outgoing_segment_index,
    )

    result = tracker.update(
        first_corner.point,
        active_span=next_span,
        back_window_segments=0,
        forward_window_segments=1,
    )

    assert result.progress_s == pytest.approx(first_corner.s)
    assert result.next_corner_distance_m == pytest.approx(
        second_corner.s - first_corner.s
    )
    assert result.next_corner_angle_rad == pytest.approx(second_corner.turn_angle_rad)
    assert result.next_leg_heading_rad == pytest.approx(
        second_corner.outgoing_heading_rad
    )


def test_progress_tracker_never_snaps_backward_without_reset():
    geometry = build_path(
        [(0, 0), (1, 0), (2, 0), (3, 0)],
        types=[POINT_TYPE_PASS_THROUGH] * 3 + [POINT_TYPE_MARKING],
        markings=[-1, -1, -1, 0],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=3)
    tracker = GeometryProgressTracker(geometry)

    forward = tracker.update((2.4, 0), active_span=span)
    backward = tracker.update(
        (1.6, 0),
        active_span=span,
        max_backward_jump_m=1.0,
    )

    assert forward.progress_s == pytest.approx(2.4)
    assert backward.projected_s == pytest.approx(1.6)
    assert backward.progress_s == pytest.approx(2.4)
    assert backward.monotonic_clamped
    assert backward.remaining_to_active_stop_m == pytest.approx(0.6)


def test_explicit_localization_jump_reset_allows_reacquired_progress():
    geometry = build_path(
        [(0, 0), (1, 0), (2, 0), (3, 0)],
        types=[POINT_TYPE_PASS_THROUGH] * 3 + [POINT_TYPE_MARKING],
        markings=[-1, -1, -1, 0],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=3)
    tracker = GeometryProgressTracker(geometry)
    tracker.update((2.5, 0), active_span=span)

    tracker.reset(GeometryResetReason.LOCALIZATION_JUMP)
    reacquired = tracker.update((0.5, 0), active_span=span)

    assert tracker.last_reset_reason is GeometryResetReason.LOCALIZATION_JUMP
    assert tracker.reset_count == 2
    assert reacquired.projected_s == pytest.approx(0.5)
    assert reacquired.progress_s == pytest.approx(0.5)
    assert not reacquired.monotonic_clamped


@pytest.mark.parametrize(
    ("types", "markings", "message"),
    [
        ([0], [-1, -1], "lengths"),
        ([99, 2], [-1, 0], "unsupported point type"),
        ([0, 2], [-1, -1], "non-negative marking index"),
        ([0, 2, 2], [-1, 1, 0], "zero-based order"),
    ],
)
def test_invalid_semantic_contract_is_rejected(types, markings, message):
    points = [(float(index), 0.0) for index in range(len(types))]

    with pytest.raises(ValueError, match=message):
        build_path(points, types=types, markings=markings)


def test_nonfinite_input_is_rejected():
    with pytest.raises(ValueError, match="finite"):
        build_path([(0, 0), (math.nan, 1)])


def test_geometry_index_is_immutable():
    geometry = build_path([(0, 0), (1, 0)])

    with pytest.raises(AttributeError, match="immutable"):
        geometry.total_length = 99.0


def test_rpp_signature_helper_matches_canonical_trajectory_contract():
    signature = make_path_signature(
        [(0.0, 0.0), (1.25, -2.5), (3.0, 4.0)],
        [(1.25, -2.5), (3.0, 4.0)],
        [0, 2, 2],
        [-1, 0, 1],
    )

    assert signature == (
        "391db71b37accad9da8a436d041e8e4b"
        "d07b1f79a6b9bc4911f8316c9df5123d"
    )
    assert is_valid_path_signature(signature)


def test_goal_metadata_binds_identity_order_sidecars_and_coordinate():
    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    types = [POINT_TYPE_PASS_THROUGH, POINT_TYPE_DUMMY_ALIGNMENT, POINT_TYPE_MARKING]
    markings = [-1, -1, 0]
    signature = make_path_signature(points, [(2.0, 0.0)], types, markings)
    geometry = build_path(points, types=types, markings=markings)
    payload = {
        "schema_version": 1,
        "path_signature": signature,
        "goal_sequence": 1,
        "raw_path_index": 2,
        "point_type": POINT_TYPE_MARKING,
        "marking_index": 0,
        "point_id": "P0001",
        "active_goal_identity": "P0001",
    }

    binding = validate_goal_metadata(
        payload,
        expected_path_signature=signature,
        geometry=geometry,
        goal_point=(2.001, 0.0),
        coordinate_tolerance_m=0.002,
    )

    assert binding.raw_path_index == 2
    assert binding.active_span.start_raw_index == 1
    assert binding.active_span.stop_raw_index == 2


def test_dummy_goal_metadata_uses_path_scoped_identity_without_marking_id():
    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    types = [POINT_TYPE_PASS_THROUGH, POINT_TYPE_DUMMY_ALIGNMENT, POINT_TYPE_MARKING]
    markings = [-1, -1, 0]
    signature = make_path_signature(points, [(2.0, 0.0)], types, markings)
    geometry = build_path(points, types=types, markings=markings)
    identity = f"PATH:{signature}:RAW:1:TYPE:{POINT_TYPE_DUMMY_ALIGNMENT}"
    payload = {
        "schema_version": 1,
        "path_signature": signature,
        "goal_sequence": 0,
        "raw_path_index": 1,
        "point_type": POINT_TYPE_DUMMY_ALIGNMENT,
        "marking_index": -1,
        "point_id": None,
        "active_goal_identity": identity,
    }

    binding = validate_goal_metadata(
        payload,
        expected_path_signature=signature,
        geometry=geometry,
        goal_point=(1.0, 0.0),
        coordinate_tolerance_m=0.002,
    )

    assert binding.point_id is None
    assert binding.active_goal_identity == identity


@pytest.mark.parametrize(
    ("field", "replacement", "message"),
    [
        ("path_signature", "b" * 64, "signature does not match"),
        ("goal_sequence", 0, "semantic order"),
        ("raw_path_index", 1, "semantic order"),
        ("marking_index", 7, "semantic fields"),
        ("point_id", "P9999", "identity"),
    ],
)
def test_goal_metadata_rejects_crossed_contract_fields(field, replacement, message):
    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    types = [POINT_TYPE_PASS_THROUGH, POINT_TYPE_DUMMY_ALIGNMENT, POINT_TYPE_MARKING]
    markings = [-1, -1, 0]
    signature = make_path_signature(points, [(2.0, 0.0)], types, markings)
    geometry = build_path(points, types=types, markings=markings)
    payload = {
        "schema_version": 1,
        "path_signature": signature,
        "goal_sequence": 1,
        "raw_path_index": 2,
        "point_type": POINT_TYPE_MARKING,
        "marking_index": 0,
        "point_id": "P0001",
        "active_goal_identity": "P0001",
    }
    payload[field] = replacement

    with pytest.raises(ValueError, match=message):
        validate_goal_metadata(
            payload,
            expected_path_signature=signature,
            geometry=geometry,
            goal_point=(2.0, 0.0),
            coordinate_tolerance_m=0.002,
        )


def test_goal_metadata_rejects_pose_that_does_not_match_raw_index():
    points = [(0.0, 0.0), (1.0, 0.0)]
    types = [POINT_TYPE_PASS_THROUGH, POINT_TYPE_MARKING]
    markings = [-1, 0]
    signature = make_path_signature(points, [(1.0, 0.0)], types, markings)
    geometry = build_path(points, types=types, markings=markings)
    payload = {
        "schema_version": 1,
        "path_signature": signature,
        "goal_sequence": 0,
        "raw_path_index": 1,
        "point_type": POINT_TYPE_MARKING,
        "marking_index": 0,
        "point_id": "P0001",
        "active_goal_identity": "P0001",
    }

    with pytest.raises(ValueError, match="coordinate"):
        validate_goal_metadata(
            payload,
            expected_path_signature=signature,
            geometry=geometry,
            goal_point=(1.01, 0.0),
            coordinate_tolerance_m=0.002,
        )
