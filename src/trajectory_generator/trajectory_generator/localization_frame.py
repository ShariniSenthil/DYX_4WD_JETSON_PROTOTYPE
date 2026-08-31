"""PX4 geographic projection and MAVROS NED/ENU conversion.

The stages are deliberately explicit:
  geodetic + PX4 gp_origin -> PX4 local NED
  PX4 local NED -> ROS/MAVROS ENU

PX4 MapProjection uses CONSTANTS_RADIUS_OF_EARTH = 6,371,000 m.
MAVROS FTF vector conversion is (N, E, D) -> (E, N, -D).
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
class LocalPointNED:
    """PX4 local NED point."""

    north_m: float
    east_m: float
    down_m: float = 0.0


@dataclass(frozen=True)
class LocalPointENU:
    """ROS/MAVROS local ENU point."""

    east_m: float
    north_m: float
    up_m: float = 0.0


def _validate_coordinate(*, latitude_deg: float, longitude_deg: float, label: str) -> None:
    """Reject invalid geographic coordinates."""

    if not math.isfinite(latitude_deg) or not math.isfinite(longitude_deg):
        raise ValueError(f"{label} latitude/longitude must be finite")
    if abs(latitude_deg) > 90.0:
        raise ValueError(f"{label} latitude must be within [-90, 90]")
    if abs(longitude_deg) > 180.0:
        raise ValueError(f"{label} longitude must be within [-180, 180]")


def project_geodetic_to_px4_ned(
    origin: GeographicOrigin,
    latitude_deg: float,
    longitude_deg: float,
) -> LocalPointNED:
    """Exact horizontal Python port of PX4 MapProjection::project()."""

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

    return LocalPointNED(north_m=north, east_m=east, down_m=0.0)


def transform_ned_to_enu(point: LocalPointNED) -> LocalPointENU:
    """Apply MAVROS FTF static vector rule: (N,E,D) -> (E,N,-D)."""

    values = (point.north_m, point.east_m, point.down_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("NED point must contain finite values")

    return LocalPointENU(
        east_m=point.east_m,
        north_m=point.north_m,
        up_m=-point.down_m,
    )


def project_geodetic_to_px4_enu(
    origin: GeographicOrigin,
    latitude_deg: float,
    longitude_deg: float,
) -> LocalPointENU:
    """Compatibility helper: geodetic -> PX4 NED -> MAVROS/ROS ENU."""

    return transform_ned_to_enu(
        project_geodetic_to_px4_ned(origin, latitude_deg, longitude_deg)
    )
