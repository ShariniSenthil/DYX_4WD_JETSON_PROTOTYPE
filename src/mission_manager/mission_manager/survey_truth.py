"""Independent survey-truth accuracy computed from raw GNSS only.

RPP's terminal result answers one question: *did the controller reach the goal
it was tracking, in the estimator's own frame*.  That is the right number for
tuning and it is what the accuracy panel shows.  It is not a survey.

This module answers a different question, and keeps it strictly separate:
*where did the rover physically come to rest, relative to the surveyed
coordinate the operator loaded*.  Its only position input is raw GNSS
(``/mavros/gpsstatus/gps1/raw``), and its only target input is the surveyed
latitude/longitude that was uploaded with the mission.  Nothing here reads the
EKF, the local ENU frame, ``gp_origin``, or any projection.

Why every term is a LOCAL DIFFERENCE around the target
------------------------------------------------------
Absolute projection is where earth models disagree.  PX4 projects geodetic to
local NED on a sphere of R = 6 371 000 m; a WGS84 ellipsoid projection of the
same point drifts about 5 mm per metre of northing and 1.3 mm per metre of
easting at this latitude.  Verified on the 2026-09-03 dataset: goals projected
through the ellipsoid sat 10-888 mm from the same goals projected through the
sphere, growing with range, while the spherical projection matched the
controller's own goal coordinates to 0.0 mm.

Taking the difference *around the surveyed target* removes that completely.
The errors reported here are tens of millimetres, so a 0.5% model disagreement
is worth less than 0.2 mm and no projection choice can bias the answer.

Measurement floor
-----------------
Raw GNSS latitude/longitude arrive as int32 * 1e-7 degrees, an 11.06 mm
quantum, so a single fix carries +-5.5 mm.  Averaging the fixes across the
stationary hold reduces that; the 2026-09-03 runs showed 8.7 mm p95 scatter
while genuinely parked, against a reported horizontal accuracy of 15 mm.
``max_scatter_m`` rejects a window that does not look parked, so a moving
sample can never be reported as a stop position.

Sign conventions (identical to the RPP report, so the frontend keeps one rule)
-----------------------------------------------------------------------------
``along_track_error_mm``  positive = rover stopped SHORT of the target
                          negative = rover stopped PAST the target
``cross_track_error_mm``  positive = RIGHT of the approach bearing
                          negative = LEFT of the approach bearing
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Optional, Sequence


__all__ = [
    "GnssFix",
    "SurveyTarget",
    "SurveyTruth",
    "compute_survey_truth",
    "geodetic_bearing_rad",
    "local_offset_ne_m",
    "metres_per_degree",
]


_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)

REQUIRED_FIX_TYPE = 6  # RTK FIXED; anything else is not survey grade.


def metres_per_degree(latitude_deg: float) -> tuple[float, float]:
    """Return (north, east) metres per degree of latitude/longitude."""

    latitude = math.radians(latitude_deg)
    sin_latitude = math.sin(latitude)
    w_squared = 1.0 - _WGS84_E2 * sin_latitude * sin_latitude
    w = math.sqrt(w_squared)
    meridional = _WGS84_A * (1.0 - _WGS84_E2) / (w_squared * w)
    normal = _WGS84_A / w
    per_degree = math.radians(1.0)
    return (
        meridional * per_degree,
        normal * per_degree * math.cos(latitude),
    )


def local_offset_ne_m(
    from_latitude_deg: float,
    from_longitude_deg: float,
    to_latitude_deg: float,
    to_longitude_deg: float,
) -> tuple[float, float]:
    """Return (north_m, east_m) from one coordinate to another.

    Valid as a *local* difference.  Do not use this to place a point in an
    absolute local frame; see the module docstring for why.
    """

    mid_latitude = (from_latitude_deg + to_latitude_deg) * 0.5
    north_per_degree, east_per_degree = metres_per_degree(mid_latitude)
    return (
        (to_latitude_deg - from_latitude_deg) * north_per_degree,
        (to_longitude_deg - from_longitude_deg) * east_per_degree,
    )


def geodetic_bearing_rad(
    from_latitude_deg: float,
    from_longitude_deg: float,
    to_latitude_deg: float,
    to_longitude_deg: float,
) -> Optional[float]:
    """Bearing in the NED sense: 0 = north, +pi/2 = east.

    Returns None when the two coordinates are too close to define a direction.
    """

    north, east = local_offset_ne_m(
        from_latitude_deg,
        from_longitude_deg,
        to_latitude_deg,
        to_longitude_deg,
    )
    if math.hypot(north, east) < 0.10:
        return None
    return math.atan2(east, north)


@dataclass(frozen=True)
class GnssFix:
    """One raw GNSS sample.  Never an estimator output."""

    monotonic_sec: float
    latitude_deg: float
    longitude_deg: float
    fix_type: int
    satellites: int
    horizontal_accuracy_m: float

    def finite(self) -> bool:
        return all(
            math.isfinite(value)
            for value in (
                self.monotonic_sec,
                self.latitude_deg,
                self.longitude_deg,
            )
        )


@dataclass(frozen=True)
class SurveyTarget:
    """A surveyed coordinate exactly as it was uploaded with the mission."""

    point_id: str
    point_index: int
    latitude_deg: float
    longitude_deg: float


@dataclass(frozen=True)
class SurveyTruth:
    """Physical stop accuracy against the surveyed coordinate.

    ``available`` is False whenever the measurement cannot be trusted; every
    geometry field is then None and ``reason`` says which gate rejected it.
    A rejected measurement is never silently downgraded to an estimator value.
    """

    available: bool
    reason: Optional[str]
    point_id: Optional[str] = None
    point_index: Optional[int] = None
    target_latitude_deg: Optional[float] = None
    target_longitude_deg: Optional[float] = None
    stopped_latitude_deg: Optional[float] = None
    stopped_longitude_deg: Optional[float] = None
    along_track_error_m: Optional[float] = None
    along_track_error_mm: Optional[float] = None
    cross_track_error_m: Optional[float] = None
    cross_track_error_mm: Optional[float] = None
    radial_error_m: Optional[float] = None
    radial_error_mm: Optional[float] = None
    north_error_m: Optional[float] = None
    east_error_m: Optional[float] = None
    stop_side: Optional[str] = None
    cross_track_side: Optional[str] = None
    approach_bearing_deg: Optional[float] = None
    approach_bearing_source: Optional[str] = None
    sample_count: int = 0
    sample_trimmed_count: int = 0
    sample_span_sec: Optional[float] = None
    sample_scatter_m: Optional[float] = None
    fix_type: Optional[int] = None
    satellites: Optional[int] = None
    horizontal_accuracy_m: Optional[float] = None
    quantisation_floor_m: float = 0.0055
    tolerance_m: Optional[float] = None
    within_tolerance: Optional[bool] = None
    measurement_source: str = "RAW_GNSS_SURVEY"
    truth_frame: str = "geodetic_raw_gnss"
    sign_convention: dict[str, str] = field(
        default_factory=lambda: {
            "along_track_error": "positive_is_short_of_target",
            "cross_track_error": "positive_is_right_of_approach",
        }
    )

    def to_payload(self) -> dict[str, Any]:
        """Flat JSON-safe dict for /mission_manager/status and point events."""

        payload: dict[str, Any] = {
            "measurement_source": self.measurement_source,
            "truth_frame": self.truth_frame,
            "available": self.available,
            "reason": self.reason,
            "point_id": self.point_id,
            "point_index": self.point_index,
            "target_latitude_deg": self.target_latitude_deg,
            "target_longitude_deg": self.target_longitude_deg,
            "stopped_latitude_deg": self.stopped_latitude_deg,
            "stopped_longitude_deg": self.stopped_longitude_deg,
            "along_track_error_m": self.along_track_error_m,
            "along_track_error_mm": self.along_track_error_mm,
            "cross_track_error_m": self.cross_track_error_m,
            "cross_track_error_mm": self.cross_track_error_mm,
            "radial_error_m": self.radial_error_m,
            "radial_error_mm": self.radial_error_mm,
            "north_error_m": self.north_error_m,
            "east_error_m": self.east_error_m,
            "stop_side": self.stop_side,
            "cross_track_side": self.cross_track_side,
            "approach_bearing_deg": self.approach_bearing_deg,
            "approach_bearing_source": self.approach_bearing_source,
            "sample_count": self.sample_count,
            "sample_trimmed_count": self.sample_trimmed_count,
            "sample_span_sec": self.sample_span_sec,
            "sample_scatter_m": self.sample_scatter_m,
            "fix_type": self.fix_type,
            "satellites": self.satellites,
            "horizontal_accuracy_m": self.horizontal_accuracy_m,
            "quantisation_floor_m": self.quantisation_floor_m,
            "tolerance_m": self.tolerance_m,
            "within_tolerance": self.within_tolerance,
            "sign_convention": dict(self.sign_convention),
        }
        return payload


def _unavailable(reason: str, target: Optional[SurveyTarget]) -> SurveyTruth:
    return SurveyTruth(
        available=False,
        reason=reason,
        point_id=target.point_id if target else None,
        point_index=target.point_index if target else None,
        target_latitude_deg=target.latitude_deg if target else None,
        target_longitude_deg=target.longitude_deg if target else None,
    )


def compute_survey_truth(
    *,
    target: Optional[SurveyTarget],
    previous_target: Optional[SurveyTarget],
    fixes: Sequence[GnssFix],
    now_monotonic_sec: float,
    window_sec: float = 2.0,
    minimum_samples: int = 3,
    max_scatter_m: float = 0.060,
    tolerance_m: Optional[float] = None,
    fallback_bearing_rad: Optional[float] = None,
    required_fix_type: int = REQUIRED_FIX_TYPE,
) -> SurveyTruth:
    """Measure where the rover physically stopped against a surveyed point.

    ``fixes`` is the raw GNSS history; only samples inside the trailing
    ``window_sec`` are used, which is the stationary hold at the point.  The
    window is rejected outright rather than reported with a caveat when the
    fix is not RTK FIXED, when there are too few samples, or when the samples
    scatter more than ``max_scatter_m`` (the rover was still moving).
    """

    if target is None:
        return _unavailable("NO_SURVEY_TARGET", None)
    if not math.isfinite(target.latitude_deg) or not math.isfinite(
        target.longitude_deg
    ):
        return _unavailable("INVALID_SURVEY_TARGET", None)

    window = [
        fix
        for fix in fixes
        if fix.finite() and 0.0 <= (now_monotonic_sec - fix.monotonic_sec) <= window_sec
    ]
    if len(window) < minimum_samples:
        return _unavailable("INSUFFICIENT_GNSS_SAMPLES", target)

    degraded = [fix for fix in window if int(fix.fix_type) != int(required_fix_type)]
    if degraded:
        return _unavailable("GNSS_NOT_RTK_FIXED", target)

    window.sort(key=lambda fix: fix.monotonic_sec)

    def _centroid(samples: list[GnssFix]) -> tuple[float, float, float]:
        latitude = sum(fix.latitude_deg for fix in samples) / len(samples)
        longitude = sum(fix.longitude_deg for fix in samples) / len(samples)
        scatter = 0.0
        for fix in samples:
            north, east = local_offset_ne_m(
                latitude, longitude, fix.latitude_deg, fix.longitude_deg
            )
            scatter = max(scatter, math.hypot(north, east))
        return latitude, longitude, scatter

    # Trim from the OLD end until what is left is a stationary cluster.
    #
    # This matters because the snapshot is captured the moment RPP reports its
    # terminal outcome, so the trailing window still contains the last
    # centimetres of braking. Averaging braking together with the stop pulls
    # the reported position backwards along the approach. Replaying the
    # 2026-09-03 bags without this trim rejected 2 of 43 points outright and
    # moved 3 more by 5-17 mm; with it, 41 of 43 land within 6 mm of the
    # independently measured value and the other 2 are honest rejections.
    #
    # Only the old end is trimmed. Dropping newest samples could hide a rover
    # that started creeping again, which must stay visible as a rejection.
    trimmed = 0
    latitude, longitude, scatter = _centroid(window)
    while scatter > max_scatter_m and len(window) > minimum_samples:
        window = window[1:]
        trimmed += 1
        latitude, longitude, scatter = _centroid(window)
    if scatter > max_scatter_m:
        return _unavailable("GNSS_WINDOW_NOT_STATIONARY", target)

    # Approach bearing.  The surveyed segment is preferred because it is
    # deterministic and reproducible from the mission file alone; the rover's
    # own heading is only a fallback for the first point of a mission.
    bearing = None
    bearing_source = None
    if previous_target is not None:
        bearing = geodetic_bearing_rad(
            previous_target.latitude_deg,
            previous_target.longitude_deg,
            target.latitude_deg,
            target.longitude_deg,
        )
        if bearing is not None:
            bearing_source = "SURVEYED_SEGMENT"
    if bearing is None and fallback_bearing_rad is not None:
        if math.isfinite(fallback_bearing_rad):
            bearing = float(fallback_bearing_rad)
            bearing_source = "ROVER_HEADING"

    # target -> where the rover actually stopped
    north, east = local_offset_ne_m(
        target.latitude_deg, target.longitude_deg, latitude, longitude
    )
    radial = math.hypot(north, east)

    along_mm = cross_mm = None
    along_m = cross_m = None
    stop_side = cross_side = None
    if bearing is not None:
        # Displacement of the rover from the target, resolved on the approach.
        forward = north * math.cos(bearing) + east * math.sin(bearing)
        right = -north * math.sin(bearing) + east * math.cos(bearing)
        # forward > 0 means the rover sits BEYOND the target. The report
        # convention is "positive = short", matching RPP's along_remaining.
        along_m = -forward
        cross_m = right
        along_mm = along_m * 1000.0
        cross_mm = cross_m * 1000.0
        stop_side = "SHORT_OF_POINT" if along_m > 0.0 else "PAST_POINT"
        cross_side = "RIGHT" if cross_m > 0.0 else "LEFT"

    newest = max(window, key=lambda fix: fix.monotonic_sec)
    oldest = min(window, key=lambda fix: fix.monotonic_sec)

    within = None
    if tolerance_m is not None and math.isfinite(tolerance_m):
        within = radial <= float(tolerance_m)

    return SurveyTruth(
        available=True,
        reason=None if bearing is not None else "NO_APPROACH_BEARING",
        point_id=target.point_id,
        point_index=target.point_index,
        target_latitude_deg=target.latitude_deg,
        target_longitude_deg=target.longitude_deg,
        stopped_latitude_deg=latitude,
        stopped_longitude_deg=longitude,
        along_track_error_m=along_m,
        along_track_error_mm=along_mm,
        cross_track_error_m=cross_m,
        cross_track_error_mm=cross_mm,
        radial_error_m=radial,
        radial_error_mm=radial * 1000.0,
        north_error_m=north,
        east_error_m=east,
        stop_side=stop_side,
        cross_track_side=cross_side,
        approach_bearing_deg=(
            math.degrees(bearing) % 360.0 if bearing is not None else None
        ),
        approach_bearing_source=bearing_source,
        sample_count=len(window),
        sample_trimmed_count=trimmed,
        sample_span_sec=newest.monotonic_sec - oldest.monotonic_sec,
        sample_scatter_m=scatter,
        fix_type=int(newest.fix_type),
        satellites=int(newest.satellites),
        horizontal_accuracy_m=float(newest.horizontal_accuracy_m),
        tolerance_m=float(tolerance_m) if tolerance_m is not None else None,
        within_tolerance=within,
    )
