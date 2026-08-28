"""Regression tests for the PX4 geographic-to-local frame conversion."""

import math

import pytest

from trajectory_generator.localization_frame import GeographicOrigin
from trajectory_generator.localization_frame import project_geodetic_to_px4_enu


AUGUST_28_ORIGIN = GeographicOrigin(
    latitude_deg=13.1894576,
    longitude_deg=80.2220672,
)


@pytest.mark.parametrize(
    "latitude,longitude,expected_east,expected_north",
    [
        (13.18937434, 80.22219259, 13.574939798, -9.258086203),
        (13.18936988, 80.22223429, 18.089454758, -9.754012947),
        (13.18936523, 80.22227606, 22.611548222, -10.271065971),
        (13.18936129, 80.22231909, 27.270051583, -10.709169708),
        (13.18935658, 80.22236119, 31.827871768, -11.232892858),
    ],
)
def test_august_28_candidate_coordinates(
    latitude,
    longitude,
    expected_east,
    expected_north,
):
    """Reproduce the five audited targets from the recorded PX4 origin."""

    point = project_geodetic_to_px4_enu(
        AUGUST_28_ORIGIN,
        latitude,
        longitude,
    )

    assert point.east_m == pytest.approx(expected_east, abs=1e-9)
    assert point.north_m == pytest.approx(expected_north, abs=1e-9)


def test_origin_projects_to_zero():
    """Map the estimator origin onto local zero."""

    point = project_geodetic_to_px4_enu(
        AUGUST_28_ORIGIN,
        AUGUST_28_ORIGIN.latitude_deg,
        AUGUST_28_ORIGIN.longitude_deg,
    )

    assert point.east_m == pytest.approx(0.0, abs=1e-12)
    assert point.north_m == pytest.approx(0.0, abs=1e-12)


def test_projection_uses_ros_enu_axis_signs():
    """Return longitude motion as East and latitude motion as North."""

    north = project_geodetic_to_px4_enu(
        AUGUST_28_ORIGIN,
        AUGUST_28_ORIGIN.latitude_deg + 0.00001,
        AUGUST_28_ORIGIN.longitude_deg,
    )

    east = project_geodetic_to_px4_enu(
        AUGUST_28_ORIGIN,
        AUGUST_28_ORIGIN.latitude_deg,
        AUGUST_28_ORIGIN.longitude_deg + 0.00001,
    )

    assert north.north_m > 1.0
    assert abs(north.east_m) < 1e-6
    assert east.east_m > 1.0
    assert abs(east.north_m) < 1e-6


def test_projection_is_repeatable():
    """Produce identical output for identical origin and target inputs."""

    first = project_geodetic_to_px4_enu(
        AUGUST_28_ORIGIN,
        13.18937434,
        80.22219259,
    )

    second = project_geodetic_to_px4_enu(
        AUGUST_28_ORIGIN,
        13.18937434,
        80.22219259,
    )

    assert first == second


@pytest.mark.parametrize(
    "origin,latitude,longitude",
    [
        (GeographicOrigin(math.nan, 80.0), 13.0, 80.0),
        (GeographicOrigin(13.0, math.inf), 13.0, 80.0),
        (GeographicOrigin(91.0, 80.0), 13.0, 80.0),
        (GeographicOrigin(13.0, 181.0), 13.0, 80.0),
        (AUGUST_28_ORIGIN, math.nan, 80.0),
        (AUGUST_28_ORIGIN, 13.0, math.inf),
        (AUGUST_28_ORIGIN, -91.0, 80.0),
        (AUGUST_28_ORIGIN, 13.0, -181.0),
    ],
)
def test_invalid_coordinates_are_rejected(
    origin,
    latitude,
    longitude,
):
    """Reject non-finite and out-of-range origin or target coordinates."""

    with pytest.raises(ValueError):
        project_geodetic_to_px4_enu(
            origin,
            latitude,
            longitude,
        )
