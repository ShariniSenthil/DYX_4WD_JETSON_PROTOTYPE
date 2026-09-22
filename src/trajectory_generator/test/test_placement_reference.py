"""Placement-frame validation is separate from RTK motion policy."""

from __future__ import annotations

from types import SimpleNamespace

import pytest


pytest.importorskip("rclpy")

from test_prepare_lifecycle import _fake_generator  # noqa: E402
from test_prepare_lifecycle import _TestClock  # noqa: E402
from test_prepare_lifecycle import _TestTime  # noqa: E402
from trajectory_generator.localization_frame import (  # noqa: E402
    GeographicOrigin,
    project_geodetic_to_px4_enu,
)
from trajectory_generator.trajectory_generator_node import (  # noqa: E402
    TrajectoryGenerator,
)


NOW_NS = 50_000_000_000
ORIGIN = (13.0, 80.0)
FUSED = (13.0000450, 80.0000920)


def _odom(east, north):
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=east, y=north),
            ),
        ),
    )


def _install_frame(
    node,
    *,
    local_offset_m=(0.0, 0.0),
    fused_age_s=0.05,
    local_age_s=0.05,
):
    """Install a consistent PX4 origin/global/local sample set.

    RTK is deliberately NOT fixed: FLOAT, bridge unhealthy, no correction
    age. Placement must not depend on any of those.
    """

    node.get_clock = lambda: _TestClock(NOW_NS)

    node.latest_gp_origin = SimpleNamespace(
        position=SimpleNamespace(
            latitude=ORIGIN[0],
            longitude=ORIGIN[1],
            altitude=0.0,
        ),
    )
    node.latest_fused_global_fix = SimpleNamespace(
        latitude=FUSED[0],
        longitude=FUSED[1],
    )
    node.latest_fused_global_time = _TestTime(
        NOW_NS - fused_age_s * 1e9
    )

    enu = project_geodetic_to_px4_enu(
        GeographicOrigin(latitude_deg=ORIGIN[0], longitude_deg=ORIGIN[1]),
        FUSED[0],
        FUSED[1],
    )
    node.latest_local_odom = _odom(
        enu.east_m + local_offset_m[0],
        enu.north_m + local_offset_m[1],
    )
    node.latest_local_time = _TestTime(NOW_NS - local_age_s * 1e9)

    node.latest_gps_status = SimpleNamespace(fix_type=5)
    node.latest_gps_status_time = _TestTime(NOW_NS)
    node.rtk_healthy = False
    node.correction_age_sec = float("inf")


def _gps_generator():
    node = _fake_generator()
    _install_frame(node)
    return node


def test_valid_frame_places_even_when_rtk_motion_is_not_satisfied():
    node = _gps_generator()

    ready, reason = node._placement_reference_is_ready()
    assert ready is True, reason

    rtk_ok, rtk_reason = node._rtk_diagnostic_status()
    assert rtk_ok is False
    assert "fix_type=5" in rtk_reason


@pytest.mark.parametrize(
    "mutate",
    [
        lambda node: setattr(node, "latest_gps_status", None),
        lambda node: setattr(
            node,
            "latest_gps_status",
            SimpleNamespace(fix_type=0),
        ),
        lambda node: setattr(
            node,
            "latest_gps_status_time",
            _TestTime(0),
        ),
        lambda node: setattr(node, "correction_age_sec", 99.0),
    ],
    ids=[
        "no-gps-status",
        "no-fix",
        "stale-gps-status",
        "old-corrections",
    ],
)
def test_rtk_quality_never_blocks_placement(mutate):
    node = _gps_generator()
    mutate(node)

    ready, reason = node._placement_reference_is_ready()
    assert ready is True, reason


def test_control_loop_places_rtk_float_mission_as_placed_ready():
    node = _gps_generator()

    node._load_mission_source = lambda: (
        {
            "mission_id": "mission-float",
            "checksum_sha256": "e" * 64,
            "extension_mode": "DISABLE",
        },
        "gps",
        [
            (13.0000000, 80.0000000),
            (13.0000000, 80.0000100),
        ],
    )

    response = SimpleNamespace(success=False, message="")
    TrajectoryGenerator._prepare_callback(node, None, response)
    assert response.success is True

    node.max_target_distance_m = 500.0
    node.POINT_TYPE_MARKING = TrajectoryGenerator.POINT_TYPE_MARKING
    node.POINT_TYPE_PASS_THROUGH = (
        TrajectoryGenerator.POINT_TYPE_PASS_THROUGH
    )
    node._maybe_request_gp_origin = lambda: None
    node._convert_markings_to_local = lambda: (
        TrajectoryGenerator._convert_markings_px4_origin(node)
    )
    node._generate_navigation_path = lambda markings: (
        list(markings),
        [node.POINT_TYPE_MARKING for _ in markings],
        list(range(len(markings))),
        0,
    )
    node._build_path = lambda points, stamp: (tuple(points), stamp)
    node._make_signature = TrajectoryGenerator._make_signature
    node.mission_waypoints_pub = SimpleNamespace(publish=lambda _m: None)
    node.nav_path_pub = SimpleNamespace(publish=lambda _m: None)
    node._publish_path_metadata = lambda **_kwargs: None
    node._publish_path_signature = lambda _signature: None
    node._publish_survey_targets = lambda _signature, cleared=False: None

    statuses = []
    node._publish_status = lambda **kwargs: statuses.append(
        node._build_status_payload(**kwargs)
    )

    TrajectoryGenerator._control_loop(node)

    assert node.ready is True
    assert node._trajectory_phase() == "PLACED"

    ready_status = statuses[-1]
    assert ready_status["state"] == "READY"
    assert ready_status["placement_ready"] is True
    assert ready_status["rtk_diagnostic"]["ok"] is False

    # Surveyed P1 is projected through gp_origin; it is not moved to the
    # rover pose and equals the origin here.
    assert node.prepared_marking_points[0] == pytest.approx((0.0, 0.0))


def test_missing_gp_origin_blocks_placement():
    node = _gps_generator()
    node.latest_gp_origin = None

    ready, reason = node._placement_reference_is_ready()
    assert ready is False
    assert "gp_origin unavailable" in reason


def test_missing_fused_global_blocks_placement():
    node = _gps_generator()
    node.latest_fused_global_fix = None

    ready, reason = node._placement_reference_is_ready()
    assert ready is False
    assert "fused global position unavailable" in reason


def test_missing_local_odom_blocks_placement():
    node = _gps_generator()
    node.latest_local_odom = None

    ready, reason = node._placement_reference_is_ready()
    assert ready is False
    assert "local odometry unavailable" in reason


def test_stale_fused_global_blocks_placement():
    node = _fake_generator()
    _install_frame(node, fused_age_s=1.5, local_age_s=1.4)

    ready, reason = node._placement_reference_is_ready()
    assert ready is False
    assert "fused global position is stale" in reason


def test_stale_local_odom_blocks_placement():
    node = _fake_generator()
    _install_frame(node, fused_age_s=0.9, local_age_s=1.1)

    ready, reason = node._placement_reference_is_ready()
    assert ready is False
    assert "local odometry is stale" in reason


def test_excessive_receive_skew_blocks_placement():
    node = _fake_generator()
    _install_frame(node, fused_age_s=0.01, local_age_s=0.30)

    ready, reason = node._placement_reference_is_ready()
    assert ready is False
    assert "skew" in reason


def test_excessive_frame_residual_blocks_placement():
    node = _fake_generator()
    _install_frame(node, local_offset_m=(0.25, 0.25))

    ready, reason = node._placement_reference_is_ready()
    assert ready is False
    assert "inconsistent with live local frame" in reason


def test_residual_inside_limit_places():
    node = _fake_generator()
    _install_frame(node, local_offset_m=(0.20, 0.0))

    ready, reason = node._placement_reference_is_ready()
    assert ready is True, reason


def test_non_finite_frame_sample_blocks_placement():
    node = _gps_generator()
    node.latest_fused_global_fix = SimpleNamespace(
        latitude=float("nan"),
        longitude=FUSED[1],
    )

    ready, reason = node._placement_reference_is_ready()
    assert ready is False
    assert "non-finite" in reason


def test_status_reports_rtk_quality_as_diagnostic_only():
    node = _gps_generator()

    payload = node._build_status_payload(
        state="PREPARING",
        message="diagnostic",
    )

    quality = payload["rtk_diagnostic"]
    assert quality["ok"] is False
    assert quality["motion_gate_owner"] == "mission_manager"


def test_rtk_quality_passes_only_with_fresh_healthy_fixed_solution():
    node = _gps_generator()
    node.latest_gps_status = SimpleNamespace(fix_type=6)
    node.rtk_healthy = True
    node.correction_age_sec = 0.4

    assert node._rtk_diagnostic_status() == (True, "RTK FIXED")


def test_trajectory_generator_no_longer_owns_rtk_placement_gate():
    # Placement must not call the diagnostic RTK status, and the legacy
    # combined predicate must be gone so nothing can re-couple them.
    assert not hasattr(TrajectoryGenerator, "_reference_is_ready")
