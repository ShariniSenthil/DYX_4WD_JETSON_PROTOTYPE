"""PX4 geographic-to-local horizontal frame conversion.

This module is deliberately ROS-free so its frame mathematics can be tested
without a running graph. PX4 local NED uses ``MapProjection`` with a spherical
Earth radius of 6,371,000 metres. ROS local coordinates use ENU axis order.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


PX4_EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True)
class GeographicOrigin:
    """Geographic origin declared by the PX4 estimator."""

    latitude_deg: float
    longitude_deg: float
    altitude_m: float = 0.0


@dataclass(frozen=True)
class LocalPointENU:
    """Horizontal ROS ENU point relative to the PX4 estimator origin."""

    east_m: float
    north_m: float


def _validate_coordinate(
    *,
    latitude_deg: float,
    longitude_deg: float,
    label: str,
) -> None:
    """Reject coordinates that cannot define a geographic position."""

    if not math.isfinite(latitude_deg) or not math.isfinite(longitude_deg):
        raise ValueError(f"{label} latitude/longitude must be finite")

    if abs(latitude_deg) > 90.0:
        raise ValueError(f"{label} latitude must be within [-90, 90]")

    if abs(longitude_deg) > 180.0:
        raise ValueError(f"{label} longitude must be within [-180, 180]")


def project_geodetic_to_px4_enu(
    origin: GeographicOrigin,
    latitude_deg: float,
    longitude_deg: float,
) -> LocalPointENU:
    """Project one geodetic coordinate into the PX4 local frame in ROS ENU.

    The calculation is an exact Python port of PX4
    ``MapProjection::project()``. PX4 returns North then East; this function
    returns East then North to match MAVROS/ROS ENU.
    """

    _validate_coordinate(
        latitude_deg=origin.latitude_deg,
        longitude_deg=origin.longitude_deg,
        label="Origin",
    )

    _validate_coordinate(
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        label="Target",
    )

    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    reference_latitude = math.radians(origin.latitude_deg)
    reference_longitude = math.radians(origin.longitude_deg)

    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_reference = math.sin(reference_latitude)
    cos_reference = math.cos(reference_latitude)
    delta_longitude = longitude - reference_longitude
    cos_delta_longitude = math.cos(delta_longitude)

    argument = (
        sin_reference * sin_latitude
        + cos_reference * cos_latitude * cos_delta_longitude
    )

    central_angle = math.acos(max(-1.0, min(1.0, argument)))

    scale = 1.0

    if abs(central_angle) > 0.0:
        scale = central_angle / math.sin(central_angle)

    north = (
        scale
        * (
            cos_reference * sin_latitude
            - sin_reference * cos_latitude * cos_delta_longitude
        )
        * PX4_EARTH_RADIUS_M
    )

    east = (
        scale
        * cos_latitude
        * math.sin(delta_longitude)
        * PX4_EARTH_RADIUS_M
    )

    return LocalPointENU(
        east_m=east,
        north_m=north,
    )
