from dataclasses import FrozenInstanceError, replace
import math

import pytest

from rpp_controller.guidance import (
    GuidanceConfig,
    adaptive_lookahead_m,
    compute_precision_guidance,
    limit_bearing_to_moving_cone,
    wrap_heading_error,
)
from rpp_controller.path_geometry import (
    PathGeometryIndex,
    Point2D,
    POINT_TYPE_MARKING,
    POINT_TYPE_PASS_THROUGH,
)


def build_solution(
    points,
    *,
    rover_position,
    rover_yaw=0.0,
    speed=1.0,
    config=None,
    query=None,
):
    types = [POINT_TYPE_PASS_THROUGH] * len(points)
    types[-1] = POINT_TYPE_MARKING
    markings = [-1] * len(points)
    markings[-1] = 0
    geometry = PathGeometryIndex.build(
        points,
        point_types=types,
        marking_indices=markings,
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=len(points) - 1)
    projection = geometry.project(
        rover_position if query is None else query,
        active_span=span,
    )
    return compute_precision_guidance(
        config or GuidanceConfig(),
        geometry=geometry,
        projection=projection,
        active_span=span,
        rover_position=rover_position,
        rover_yaw_rad=rover_yaw,
        speed_mps=speed,
    )


def test_enu_left_and_right_turns_have_expected_bearing_polarity():
    config = GuidanceConfig(
        lookahead_min_m=1.5,
        lookahead_max_m=1.5,
        moving_bearing_cone_rad=math.pi,
    )

    left = build_solution(
        [(0, 0), (1, 0), (1, 2)],
        rover_position=(0.5, 0.0),
        config=config,
    )
    right = build_solution(
        [(0, 0), (1, 0), (1, -2)],
        rover_position=(0.5, 0.0),
        config=config,
    )

    assert left.desired_movement_bearing_rad > 0.0
    assert right.desired_movement_bearing_rad < 0.0
    assert left.local_path_heading_rad == pytest.approx(0.0)
    assert right.local_path_heading_rad == pytest.approx(0.0)


def test_wrapped_heading_error_uses_shortest_turn_across_pi():
    positive = wrap_heading_error(math.radians(-179), math.radians(179))
    negative = wrap_heading_error(math.radians(179), math.radians(-179))

    assert positive == pytest.approx(math.radians(2))
    assert negative == pytest.approx(math.radians(-2))


def test_speed_based_lookahead_obeys_minimum_and_maximum():
    config = GuidanceConfig(
        lookahead_min_m=0.5,
        lookahead_max_m=2.0,
        lookahead_time_s=1.0,
    )

    assert adaptive_lookahead_m(
        config, speed_mps=0.0, signed_cross_track_m=0.0
    ) == pytest.approx(0.5)
    assert adaptive_lookahead_m(
        config, speed_mps=0.75, signed_cross_track_m=0.0
    ) == pytest.approx(0.75)
    assert adaptive_lookahead_m(
        config, speed_mps=3.0, signed_cross_track_m=0.0
    ) == pytest.approx(2.0)


def test_cross_track_gain_uses_magnitude_and_is_bounded():
    config = GuidanceConfig(
        lookahead_min_m=0.2,
        lookahead_max_m=1.0,
        lookahead_time_s=0.5,
        xtrack_lookahead_gain=0.5,
    )

    positive = adaptive_lookahead_m(
        config, speed_mps=1.0, signed_cross_track_m=0.4
    )
    negative = adaptive_lookahead_m(
        config, speed_mps=1.0, signed_cross_track_m=-0.4
    )
    saturated = adaptive_lookahead_m(
        config, speed_mps=1.0, signed_cross_track_m=4.0
    )

    assert positive == pytest.approx(0.7)
    assert negative == pytest.approx(positive)
    assert saturated == pytest.approx(1.0)


def test_desired_bearing_targets_actual_lookahead_vector():
    config = GuidanceConfig(
        lookahead_min_m=2.0,
        lookahead_max_m=2.0,
        moving_bearing_cone_rad=math.pi,
    )
    result = build_solution(
        [(0, 0), (10, 0)],
        rover_position=(0, 1),
        query=(0, 1),
        config=config,
    )

    expected = math.atan2(-1.0, 2.0)
    assert result.lookahead_point == Point2D(2.0, 0.0)
    assert result.lookahead_bearing_rad == pytest.approx(expected)
    assert result.desired_movement_bearing_rad == pytest.approx(expected)
    assert result.heading_error_rad == pytest.approx(expected)
    assert result.signed_cross_track_m == pytest.approx(1.0)
    assert result.endpoint_extension_used is False
    assert result.steering_target_point == result.lookahead_point


def test_endpoint_overshoot_uses_forward_virtual_steering_target_only():
    geometry = PathGeometryIndex.build(
        [(0, 0), (10, 0)],
        point_types=[POINT_TYPE_PASS_THROUGH, POINT_TYPE_MARKING],
        marking_indices=[-1, 0],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=1)
    rover_position = (10.05, 0.02)
    projection = geometry.project(rover_position, active_span=span)
    result = compute_precision_guidance(
        GuidanceConfig(
            lookahead_min_m=0.55,
            lookahead_max_m=0.55,
            moving_bearing_cone_rad=math.pi,
        ),
        geometry=geometry,
        projection=projection,
        active_span=span,
        rover_position=rover_position,
        rover_yaw_rad=0.0,
        speed_mps=1.0,
    )

    assert result.endpoint_extension_used is True
    assert result.endpoint_extension_distance_m == pytest.approx(0.55)
    assert result.lookahead_point == Point2D(10.0, 0.0)
    assert result.steering_target_point == Point2D(10.55, 0.0)
    assert result.actual_steering_target_distance_m == pytest.approx(
        math.hypot(0.50, 0.02)
    )
    assert result.target_behind_rover is False
    assert abs(result.heading_error_rad) < math.radians(3.0)


def test_requested_lookahead_stays_forward_at_real_semantic_endpoint():
    geometry = PathGeometryIndex.build(
        [(0, 0), (10, 0)],
        point_types=[POINT_TYPE_PASS_THROUGH, POINT_TYPE_MARKING],
        marking_indices=[-1, 0],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=1)
    projection = geometry.project((10, 0), active_span=span)
    result = compute_precision_guidance(
        GuidanceConfig(
            lookahead_min_m=0.55,
            lookahead_max_m=0.55,
            moving_bearing_cone_rad=math.pi,
        ),
        geometry=geometry,
        projection=projection,
        active_span=span,
        rover_position=(10, 0),
        rover_yaw_rad=0.0,
        speed_mps=1.0,
    )

    assert result.lookahead_distance_m == pytest.approx(0.55)
    assert result.actual_steering_target_distance_m == pytest.approx(0.55)
    assert result.lookahead_bearing_rad == pytest.approx(0.0)
    assert result.target_behind_rover is False


def test_endpoint_extension_follows_incoming_diagonal_tangent():
    heading = math.pi / 4.0
    endpoint = Point2D(2.0, 2.0)
    geometry = PathGeometryIndex.build(
        [(0, 0), endpoint],
        point_types=[POINT_TYPE_PASS_THROUGH, POINT_TYPE_MARKING],
        marking_indices=[-1, 0],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=1)
    projection = geometry.project(endpoint, active_span=span)
    result = compute_precision_guidance(
        GuidanceConfig(
            lookahead_min_m=0.55,
            lookahead_max_m=0.55,
            moving_bearing_cone_rad=math.pi,
        ),
        geometry=geometry,
        projection=projection,
        active_span=span,
        rover_position=endpoint,
        rover_yaw_rad=heading,
        speed_mps=1.0,
    )

    offset = 0.55 / math.sqrt(2.0)
    assert result.steering_target_point.x == pytest.approx(endpoint.x + offset)
    assert result.steering_target_point.y == pytest.approx(endpoint.y + offset)
    assert result.lookahead_bearing_rad == pytest.approx(heading)
    assert result.path_heading_error_rad == pytest.approx(0.0)
    assert result.final_command_correction_rad == pytest.approx(0.0)


def test_virtual_endpoint_target_does_not_change_geometry_or_progress():
    geometry = PathGeometryIndex.build(
        [(0, 0), (10, 0)],
        point_types=[POINT_TYPE_PASS_THROUGH, POINT_TYPE_MARKING],
        marking_indices=[-1, 0],
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=1)
    semantic_endpoint = geometry.raw_points[span.stop_raw_index].point
    projection = geometry.project((10.05, 0), active_span=span)
    result = compute_precision_guidance(
        GuidanceConfig(
            lookahead_min_m=0.55,
            lookahead_max_m=0.55,
        ),
        geometry=geometry,
        projection=projection,
        active_span=span,
        rover_position=(10.05, 0),
        rover_yaw_rad=0.0,
        speed_mps=1.0,
    )

    assert geometry.raw_points[span.stop_raw_index].point == semantic_endpoint
    assert semantic_endpoint == Point2D(10.0, 0.0)
    assert span.stop_s == pytest.approx(10.0)
    assert projection.projected_s == pytest.approx(10.0)
    assert projection.progress_s == pytest.approx(10.0)
    assert projection.remaining_to_active_stop_m == pytest.approx(0.0)
    assert result.lookahead_target_s == pytest.approx(span.stop_s)
    assert result.lookahead_point == semantic_endpoint
    assert result.steering_target_point != semantic_endpoint


@pytest.mark.parametrize(
    ("desired_deg", "expected_deg"),
    [(90.0, 30.0), (-90.0, -30.0)],
)
def test_moving_bearing_cone_clamps_both_sides(desired_deg, expected_deg):
    limited, fired = limit_bearing_to_moving_cone(
        math.radians(desired_deg),
        0.0,
        math.radians(30.0),
    )

    assert math.degrees(limited) == pytest.approx(expected_deg)
    assert fired is True


def test_default_moving_bearing_cone_is_thirty_degrees():
    assert GuidanceConfig().moving_bearing_cone_rad == pytest.approx(
        math.radians(30.0)
    )


def test_zero_target_vector_falls_back_to_local_path_heading():
    config = GuidanceConfig(
        lookahead_min_m=0.0,
        lookahead_max_m=0.0,
        lookahead_time_s=0.0,
        moving_bearing_cone_rad=math.pi,
    )
    result = build_solution(
        [(0, 0), (0, 2)],
        rover_position=(0, 0),
        speed=0.0,
        config=config,
    )

    assert result.zero_vector_fallback_used is True
    assert result.local_path_heading_rad == pytest.approx(math.pi / 2.0)
    assert result.desired_movement_bearing_rad == pytest.approx(math.pi / 2.0)


def test_result_and_config_are_immutable():
    config = GuidanceConfig()
    result = build_solution(
        [(0, 0), (2, 0)],
        rover_position=(0, 0),
        config=config,
    )

    with pytest.raises(FrozenInstanceError):
        config.lookahead_min_m = 0.1
    with pytest.raises(FrozenInstanceError):
        result.heading_error_rad = 0.0


@pytest.mark.parametrize(
    "kwargs",
    [
        {"lookahead_min_m": math.nan},
        {"lookahead_max_m": math.inf},
        {"lookahead_min_m": 1.0, "lookahead_max_m": 0.5},
        {"lookahead_time_s": -0.1},
        {"xtrack_lookahead_gain": -0.1},
        {"moving_bearing_cone_rad": 0.0},
        {"moving_bearing_cone_rad": math.pi + 0.1},
    ],
)
def test_invalid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        GuidanceConfig(**kwargs)


@pytest.mark.parametrize(
    ("speed", "cross_track"),
    [
        (math.nan, 0.0),
        (math.inf, 0.0),
        (-0.1, 0.0),
        (0.0, math.nan),
        (0.0, math.inf),
    ],
)
def test_nonfinite_or_negative_lookahead_inputs_are_rejected(speed, cross_track):
    with pytest.raises(ValueError):
        adaptive_lookahead_m(
            GuidanceConfig(),
            speed_mps=speed,
            signed_cross_track_m=cross_track,
        )


def test_nonfinite_pose_yaw_and_projection_are_rejected():
    types = [POINT_TYPE_PASS_THROUGH, POINT_TYPE_MARKING]
    markings = [-1, 0]
    geometry = PathGeometryIndex.build(
        [(0, 0), (2, 0)],
        point_types=types,
        marking_indices=markings,
    )
    span = geometry.active_span(start_raw_index=0, stop_raw_index=1)
    projection = geometry.project((0, 0), active_span=span)

    common = dict(
        config=GuidanceConfig(),
        geometry=geometry,
        projection=projection,
        active_span=span,
        rover_position=(0, 0),
        rover_yaw_rad=0.0,
        speed_mps=0.0,
    )
    for replacement in (
        {"rover_position": (math.nan, 0)},
        {"rover_yaw_rad": math.inf},
        {"projection": replace(projection, progress_s=math.nan)},
        {
            "projection": replace(
                projection,
                signed_cross_track_m=math.inf,
            )
        },
    ):
        inputs = {**common, **replacement}
        with pytest.raises(ValueError):
            compute_precision_guidance(**inputs)
