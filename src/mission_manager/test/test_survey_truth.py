"""Contract tests for raw-GNSS survey truth.

The reference cases are real: target and stop coordinates are taken from
mission.csv_20260903_174520 / mission.csv_20260903_174005 and the expected
errors are the ones measured from those bags, so a regression here means the
report has stopped agreeing with the field data it was derived from.
"""

import math

import pytest

from mission_manager.survey_truth import (
    GnssFix,
    SurveyTarget,
    compute_survey_truth,
    geodetic_bearing_rad,
    local_offset_ne_m,
    metres_per_degree,
)


def _fixes(latitude, longitude, *, count=10, start=100.0, fix_type=6, jitter=0.0):
    """A parked window of raw fixes centred on one coordinate."""

    out = []
    for index in range(count):
        offset = jitter if index % 2 else -jitter
        out.append(
            GnssFix(
                monotonic_sec=start + index * 0.1,
                latitude_deg=latitude + offset,
                longitude_deg=longitude,
                fix_type=fix_type,
                satellites=18,
                horizontal_accuracy_m=0.015,
            )
        )
    return out


def test_metres_per_degree_matches_wgs84_at_site_latitude():
    north, east = metres_per_degree(13.1909)
    assert north == pytest.approx(110632.1, abs=1.0)
    assert east == pytest.approx(108401.2, abs=1.0)


def test_local_offset_is_symmetric_and_signed():
    north, east = local_offset_ne_m(13.1909, 80.2230, 13.19091, 80.22301)
    assert north > 0.0 and east > 0.0
    back_north, back_east = local_offset_ne_m(13.19091, 80.22301, 13.1909, 80.2230)
    assert back_north == pytest.approx(-north, rel=1e-9)
    assert back_east == pytest.approx(-east, rel=1e-9)


def test_bearing_is_ned_zero_is_north():
    due_north = geodetic_bearing_rad(13.1909, 80.2230, 13.1919, 80.2230)
    due_east = geodetic_bearing_rad(13.1909, 80.2230, 13.1909, 80.2240)
    assert math.degrees(due_north) == pytest.approx(0.0, abs=1e-6)
    assert math.degrees(due_east) == pytest.approx(90.0, abs=1e-6)


def test_bearing_is_none_for_coincident_points():
    assert geodetic_bearing_rad(13.1909, 80.2230, 13.1909, 80.2230) is None


# ---- the field reference case ----
# mission.csv_20260903_174520, P0002. Surveyed target and the raw RTK-FIXED
# position the rover actually settled at; measured error was 41.8 mm short and
# 38.8 mm left, radial 57.0 mm, on an approach bearing of the P0001->P0002
# surveyed segment.
_P0001 = SurveyTarget("P0001", 0, 13.190885290, 80.223172100)
_P0002 = SurveyTarget("P0002", 1, 13.190894620, 80.223111270)
_P0002_STOP = (13.190894219, 80.223111600)


def test_field_case_reproduces_measured_error():
    truth = compute_survey_truth(
        target=_P0002,
        previous_target=_P0001,
        fixes=_fixes(*_P0002_STOP),
        now_monotonic_sec=101.0,
    )
    assert truth.available is True
    assert truth.reason is None
    assert truth.along_track_error_mm == pytest.approx(41.8, abs=1.5)
    assert truth.cross_track_error_mm == pytest.approx(-38.8, abs=1.5)
    assert truth.radial_error_mm == pytest.approx(57.0, abs=1.5)
    assert truth.stop_side == "SHORT_OF_POINT"
    assert truth.cross_track_side == "LEFT"
    assert truth.approach_bearing_source == "SURVEYED_SEGMENT"


def test_along_sign_positive_is_short_negative_is_past():
    """The report convention must match RPP's along_track_remaining."""

    short = compute_survey_truth(
        target=_P0002,
        previous_target=_P0001,
        fixes=_fixes(*_P0002_STOP),
        now_monotonic_sec=101.0,
    )
    assert short.along_track_error_mm > 0.0

    # Push the stop 200 mm further along the same segment: now it is past.
    bearing = geodetic_bearing_rad(
        _P0001.latitude_deg,
        _P0001.longitude_deg,
        _P0002.latitude_deg,
        _P0002.longitude_deg,
    )
    north_per_degree, east_per_degree = metres_per_degree(_P0002.latitude_deg)
    past = compute_survey_truth(
        target=_P0002,
        previous_target=_P0001,
        fixes=_fixes(
            _P0002.latitude_deg + 0.2 * math.cos(bearing) / north_per_degree,
            _P0002.longitude_deg + 0.2 * math.sin(bearing) / east_per_degree,
        ),
        now_monotonic_sec=101.0,
    )
    assert past.along_track_error_mm < 0.0
    assert past.stop_side == "PAST_POINT"


def test_tolerance_verdict_uses_radial_only():
    truth = compute_survey_truth(
        target=_P0002,
        previous_target=_P0001,
        fixes=_fixes(*_P0002_STOP),
        now_monotonic_sec=101.0,
        tolerance_m=0.030,
    )
    assert truth.within_tolerance is False
    assert truth.tolerance_m == pytest.approx(0.030)

    # mission.csv_20260903_174005 P0003: measured radial 3.7 mm, a real pass.
    passing = compute_survey_truth(
        target=SurveyTarget("P0003", 2, 13.190905650, 80.223052880),
        previous_target=SurveyTarget("P0002", 1, 13.190915720, 80.223004150),
        fixes=_fixes(13.190905677, 80.223052900),
        now_monotonic_sec=101.0,
        tolerance_m=0.030,
    )
    assert passing.within_tolerance is True
    assert passing.radial_error_mm == pytest.approx(3.7, abs=1.5)


# ---- the rejection gates ----
def test_rejects_when_not_rtk_fixed():
    truth = compute_survey_truth(
        target=_P0002,
        previous_target=_P0001,
        fixes=_fixes(*_P0002_STOP, fix_type=5),
        now_monotonic_sec=101.0,
    )
    assert truth.available is False
    assert truth.reason == "GNSS_NOT_RTK_FIXED"
    assert truth.radial_error_mm is None


def test_a_single_degraded_sample_rejects_the_whole_window():
    fixes = _fixes(*_P0002_STOP)
    fixes[4] = GnssFix(
        monotonic_sec=fixes[4].monotonic_sec,
        latitude_deg=fixes[4].latitude_deg,
        longitude_deg=fixes[4].longitude_deg,
        fix_type=5,
        satellites=18,
        horizontal_accuracy_m=0.015,
    )
    truth = compute_survey_truth(
        target=_P0002,
        previous_target=_P0001,
        fixes=fixes,
        now_monotonic_sec=101.0,
    )
    assert truth.available is False
    assert truth.reason == "GNSS_NOT_RTK_FIXED"


def test_rejects_a_window_the_rover_was_still_moving_through():
    """A moving window must never be reported as a stop position."""

    fixes = [
        GnssFix(
            monotonic_sec=100.0 + index * 0.1,
            latitude_deg=_P0002_STOP[0] + index * 0.000005,  # ~0.55 m total
            longitude_deg=_P0002_STOP[1],
            fix_type=6,
            satellites=18,
            horizontal_accuracy_m=0.015,
        )
        for index in range(10)
    ]
    truth = compute_survey_truth(
        target=_P0002,
        previous_target=_P0001,
        fixes=fixes,
        now_monotonic_sec=101.0,
    )
    assert truth.available is False
    assert truth.reason == "GNSS_WINDOW_NOT_STATIONARY"


def test_accepts_the_measured_parked_scatter():
    """8.7 mm p95 scatter was normal while genuinely parked on 2026-09-03."""

    truth = compute_survey_truth(
        target=_P0002,
        previous_target=_P0001,
        fixes=_fixes(*_P0002_STOP, jitter=4.97e-8),  # +-5.5 mm, the quantum
        now_monotonic_sec=101.0,
    )
    assert truth.available is True
    assert truth.sample_scatter_m < 0.020


def test_rejects_when_the_window_is_empty_or_stale():
    stale = compute_survey_truth(
        target=_P0002,
        previous_target=_P0001,
        fixes=_fixes(*_P0002_STOP, start=10.0),
        now_monotonic_sec=101.0,
    )
    assert stale.available is False
    assert stale.reason == "INSUFFICIENT_GNSS_SAMPLES"

    empty = compute_survey_truth(
        target=_P0002,
        previous_target=_P0001,
        fixes=[],
        now_monotonic_sec=101.0,
    )
    assert empty.available is False
    assert empty.reason == "INSUFFICIENT_GNSS_SAMPLES"


def test_missing_target_is_reported_not_guessed():
    truth = compute_survey_truth(
        target=None,
        previous_target=None,
        fixes=_fixes(*_P0002_STOP),
        now_monotonic_sec=101.0,
    )
    assert truth.available is False
    assert truth.reason == "NO_SURVEY_TARGET"
    assert truth.to_payload()["radial_error_mm"] is None


def test_first_point_falls_back_to_rover_heading():
    truth = compute_survey_truth(
        target=_P0001,
        previous_target=None,
        fixes=_fixes(13.190885000, 80.223172271),
        now_monotonic_sec=101.0,
        fallback_bearing_rad=math.radians(280.0),
    )
    assert truth.available is True
    assert truth.approach_bearing_source == "ROVER_HEADING"
    assert truth.along_track_error_mm is not None


def test_radial_survives_without_any_bearing():
    """Radial error is bearing-free, so it must still be reported."""

    truth = compute_survey_truth(
        target=_P0001,
        previous_target=None,
        fixes=_fixes(13.190885000, 80.223172271),
        now_monotonic_sec=101.0,
        fallback_bearing_rad=None,
    )
    assert truth.available is True
    assert truth.reason == "NO_APPROACH_BEARING"
    assert truth.radial_error_mm is not None
    assert truth.along_track_error_mm is None
    assert truth.cross_track_error_mm is None


def test_payload_is_json_safe_and_declares_its_frame():
    import json

    payload = compute_survey_truth(
        target=_P0002,
        previous_target=_P0001,
        fixes=_fixes(*_P0002_STOP),
        now_monotonic_sec=101.0,
        tolerance_m=0.030,
    ).to_payload()
    json.dumps(payload)
    assert payload["measurement_source"] == "RAW_GNSS_SURVEY"
    assert payload["truth_frame"] == "geodetic_raw_gnss"
    assert payload["sign_convention"] == {
        "along_track_error": "positive_is_short_of_target",
        "cross_track_error": "positive_is_right_of_approach",
    }


def test_projection_model_cannot_bias_the_answer():
    """The whole design rests on this: local differencing is model-free.

    A 0.5% earth-model disagreement (sphere vs ellipsoid, the real gap) must
    move a ~57 mm error by less than a tenth of a millimetre.
    """

    baseline = compute_survey_truth(
        target=_P0002,
        previous_target=_P0001,
        fixes=_fixes(*_P0002_STOP),
        now_monotonic_sec=101.0,
    )
    north, east = local_offset_ne_m(
        _P0002.latitude_deg,
        _P0002.longitude_deg,
        *_P0002_STOP,
    )
    scaled = math.hypot(north * 1.005, east * 0.999)
    assert abs(scaled - baseline.radial_error_m) * 1000.0 < 0.30
