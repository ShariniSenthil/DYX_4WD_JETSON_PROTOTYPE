"""Pure projection-based direction guidance for the RPP controller.

The module is deliberately ROS-free.  Coordinates and angles follow the
current rover convention used by :mod:`rpp_controller.path_geometry`: local
ENU coordinates (``x`` East, ``y`` North) and mathematical headings measured
counter-clockwise from East.

The adaptive lookahead law is intentionally explicit::

    Ld = clamp(
        lookahead_time_s * speed_mps
        + xtrack_lookahead_gain * abs(cross_track_m),
        lookahead_min_m,
        lookahead_max_m,
    )

The desired movement bearing always points from the current rover position to
the resulting arc-length target.  The final command bearing is limited to a
configurable cone about current yaw; the default retains the existing 30
degree moving-command restriction.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .path_geometry import (
    ActiveSemanticSpan,
    PathGeometryIndex,
    PathProjection,
    Point2D,
    wrap_angle,
)


__all__ = [
    "GuidanceConfig",
    "GuidanceResult",
    "adaptive_lookahead_m",
    "compute_precision_guidance",
    "limit_bearing_to_moving_cone",
    "wrap_heading_error",
]


_ZERO_VECTOR_EPSILON_M = 1.0e-12


@dataclass(frozen=True, slots=True)
class GuidanceConfig:
    """Validated parameters for projection-based direction guidance."""

    lookahead_min_m: float = 0.20
    lookahead_max_m: float = 1.00
    lookahead_time_s: float = 0.55
    xtrack_lookahead_gain: float = 0.0
    moving_bearing_cone_rad: float = math.radians(30.0)

    def __post_init__(self) -> None:
        values = (
            ("lookahead_min_m", self.lookahead_min_m),
            ("lookahead_max_m", self.lookahead_max_m),
            ("lookahead_time_s", self.lookahead_time_s),
            ("xtrack_lookahead_gain", self.xtrack_lookahead_gain),
            ("moving_bearing_cone_rad", self.moving_bearing_cone_rad),
        )
        for name, value in values:
            if isinstance(value, bool) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if self.lookahead_min_m < 0.0:
            raise ValueError("lookahead_min_m must be non-negative")
        if self.lookahead_max_m < self.lookahead_min_m:
            raise ValueError(
                "lookahead_max_m must be greater than or equal to "
                "lookahead_min_m"
            )
        if self.lookahead_time_s < 0.0:
            raise ValueError("lookahead_time_s must be non-negative")
        if self.xtrack_lookahead_gain < 0.0:
            raise ValueError("xtrack_lookahead_gain must be non-negative")
        if not 0.0 < self.moving_bearing_cone_rad <= math.pi:
            raise ValueError("moving_bearing_cone_rad must be in (0, pi]")


@dataclass(frozen=True, slots=True)
class GuidanceResult:
    """One immutable precision-direction solution in local ENU coordinates."""

    lookahead_distance_m: float
    lookahead_target_s: float
    lookahead_segment_index: int | None
    lookahead_point: Point2D
    steering_target_point: Point2D
    actual_steering_target_distance_m: float
    endpoint_extension_used: bool
    endpoint_extension_distance_m: float
    target_behind_rover: bool
    lookahead_bearing_rad: float
    local_path_heading_rad: float
    path_heading_error_rad: float
    heading_error_rad: float
    signed_cross_track_m: float
    desired_movement_bearing_rad: float
    limited_command_bearing_rad: float
    final_command_correction_rad: float
    bearing_clamp_fired: bool
    zero_vector_fallback_used: bool


def adaptive_lookahead_m(
    config: GuidanceConfig,
    *,
    speed_mps: float,
    signed_cross_track_m: float,
) -> float:
    """Evaluate the bounded adaptive lookahead law.

    ``speed_mps`` is a magnitude contract and therefore must be non-negative.
    Cross-track polarity does not affect distance; only its magnitude does.
    """

    _require_finite("speed_mps", speed_mps)
    _require_finite("signed_cross_track_m", signed_cross_track_m)
    if speed_mps < 0.0:
        raise ValueError("speed_mps must be non-negative")
    requested = (
        config.lookahead_time_s * speed_mps
        + config.xtrack_lookahead_gain * abs(signed_cross_track_m)
    )
    return max(config.lookahead_min_m, min(config.lookahead_max_m, requested))


def wrap_heading_error(target_bearing_rad: float, rover_yaw_rad: float) -> float:
    """Return the shortest signed ENU turn from rover yaw to target bearing."""

    _require_finite("target_bearing_rad", target_bearing_rad)
    _require_finite("rover_yaw_rad", rover_yaw_rad)
    return wrap_angle(target_bearing_rad - rover_yaw_rad)


def limit_bearing_to_moving_cone(
    desired_bearing_rad: float,
    rover_yaw_rad: float,
    cone_half_angle_rad: float,
) -> tuple[float, bool]:
    """Limit a desired bearing to a symmetric cone about current rover yaw."""

    _require_finite("desired_bearing_rad", desired_bearing_rad)
    _require_finite("rover_yaw_rad", rover_yaw_rad)
    _require_finite("cone_half_angle_rad", cone_half_angle_rad)
    if not 0.0 < cone_half_angle_rad <= math.pi:
        raise ValueError("cone_half_angle_rad must be in (0, pi]")

    error = wrap_heading_error(desired_bearing_rad, rover_yaw_rad)
    limited_error = max(-cone_half_angle_rad, min(cone_half_angle_rad, error))
    clamp_fired = not math.isclose(
        limited_error,
        error,
        rel_tol=0.0,
        abs_tol=1.0e-15,
    )
    return wrap_angle(rover_yaw_rad + limited_error), clamp_fired


def compute_precision_guidance(
    config: GuidanceConfig,
    *,
    geometry: PathGeometryIndex,
    projection: PathProjection,
    active_span: ActiveSemanticSpan,
    rover_position: Point2D | Sequence[float],
    rover_yaw_rad: float,
    speed_mps: float,
) -> GuidanceResult:
    """Compute one projection-based direction solution.

    Arc-length targeting starts at monotonic ``projection.progress_s`` and is
    clamped by ``active_span``.  If rover and target coincide, the desired
    bearing deterministically falls back to the local path heading (or current
    rover yaw for a path with no non-zero segment).
    """

    position = _as_finite_point(rover_position)
    _require_finite("rover_yaw_rad", rover_yaw_rad)
    _require_finite("projection.progress_s", projection.progress_s)
    _require_finite(
        "projection.signed_cross_track_m",
        projection.signed_cross_track_m,
    )

    lookahead_distance = adaptive_lookahead_m(
        config,
        speed_mps=speed_mps,
        signed_cross_track_m=projection.signed_cross_track_m,
    )
    target = geometry.lookahead_target(
        projection.progress_s,
        lookahead_distance,
        active_span=active_span,
    )

    local_path_heading = _local_path_heading(
        geometry,
        projection,
        target_heading_rad=target.heading_rad,
        rover_yaw_rad=rover_yaw_rad,
    )
    steering_target, extension_distance = _steering_target_with_endpoint_extension(
        geometry,
        active_span,
        target_point=target.point,
        requested_target_s=projection.progress_s + lookahead_distance,
    )
    extension_used = extension_distance > 0.0
    delta_x = steering_target.x - position.x
    delta_y = steering_target.y - position.y
    actual_target_distance = math.hypot(delta_x, delta_y)
    zero_vector = math.hypot(delta_x, delta_y) <= _ZERO_VECTOR_EPSILON_M
    desired_bearing = (
        local_path_heading if zero_vector else math.atan2(delta_y, delta_x)
    )
    desired_bearing = wrap_angle(desired_bearing)
    heading_error = wrap_heading_error(desired_bearing, rover_yaw_rad)
    path_heading_error = wrap_heading_error(local_path_heading, rover_yaw_rad)
    limited_bearing, clamp_fired = limit_bearing_to_moving_cone(
        desired_bearing,
        rover_yaw_rad,
        config.moving_bearing_cone_rad,
    )
    final_command_correction = wrap_angle(limited_bearing - local_path_heading)
    target_forward_component = (
        delta_x * math.cos(rover_yaw_rad) + delta_y * math.sin(rover_yaw_rad)
    )

    return GuidanceResult(
        lookahead_distance_m=lookahead_distance,
        lookahead_target_s=target.s,
        lookahead_segment_index=target.segment_index,
        lookahead_point=target.point,
        steering_target_point=steering_target,
        actual_steering_target_distance_m=actual_target_distance,
        endpoint_extension_used=extension_used,
        endpoint_extension_distance_m=extension_distance,
        target_behind_rover=target_forward_component < -_ZERO_VECTOR_EPSILON_M,
        lookahead_bearing_rad=desired_bearing,
        local_path_heading_rad=local_path_heading,
        path_heading_error_rad=path_heading_error,
        heading_error_rad=heading_error,
        signed_cross_track_m=projection.signed_cross_track_m,
        desired_movement_bearing_rad=desired_bearing,
        limited_command_bearing_rad=limited_bearing,
        final_command_correction_rad=final_command_correction,
        bearing_clamp_fired=clamp_fired,
        zero_vector_fallback_used=zero_vector,
    )


def _steering_target_with_endpoint_extension(
    geometry: PathGeometryIndex,
    active_span: ActiveSemanticSpan,
    *,
    target_point: Point2D,
    requested_target_s: float,
) -> tuple[Point2D, float]:
    """Extend only the steering target past an active semantic endpoint.

    Geometry lookup remains clamped to ``active_span.stop_s``.  If the
    requested arc-length target lies beyond that real endpoint, the missing
    distance is continued along the span's incoming tangent.  The returned
    virtual point is consumed only by the desired-bearing calculation; path
    progress, remaining distance, semantic identity and stop coordinates stay
    tied to the unmodified geometry objects.
    """

    _require_finite("requested_target_s", requested_target_s)
    missing_distance = requested_target_s - active_span.stop_s
    if missing_distance <= _ZERO_VECTOR_EPSILON_M:
        return target_point, 0.0
    segment_index = active_span.last_segment_index
    if segment_index is None:
        return target_point, 0.0
    if not 0 <= segment_index < len(geometry.segments):
        raise ValueError("active span last_segment_index is outside geometry")
    incoming_heading = geometry.segments[segment_index].heading_rad
    return (
        Point2D(
            target_point.x + missing_distance * math.cos(incoming_heading),
            target_point.y + missing_distance * math.sin(incoming_heading),
        ),
        missing_distance,
    )


def _local_path_heading(
    geometry: PathGeometryIndex,
    projection: PathProjection,
    *,
    target_heading_rad: float | None,
    rover_yaw_rad: float,
) -> float:
    if projection.segment_index is not None:
        index = projection.segment_index
        if not 0 <= index < len(geometry.segments):
            raise ValueError("projection segment_index is outside geometry")
        return geometry.segments[index].heading_rad
    if target_heading_rad is not None:
        _require_finite("target.heading_rad", target_heading_rad)
        return target_heading_rad
    return wrap_angle(rover_yaw_rad)


def _as_finite_point(value: Point2D | Sequence[float]) -> Point2D:
    if isinstance(value, Point2D):
        point = value
    else:
        if len(value) != 2:
            raise ValueError("rover_position must contain exactly x and y")
        point = Point2D(float(value[0]), float(value[1]))
    _require_finite("rover_position.x", point.x)
    _require_finite("rover_position.y", point.y)
    return point


def _require_finite(name: str, value: float) -> None:
    if isinstance(value, bool) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
