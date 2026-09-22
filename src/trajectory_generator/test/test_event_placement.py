"""Event-driven placement: PREPARE and sensor callbacks, not the 5 Hz tick.

These tests run the real placement pipeline (PX4-origin projection,
metric topology, dummy points, 50 mm interpolation, signature) and record
every retained publication in order. Only ROS message construction and
the executor are replaced: the one-shot placement timer is fired by hand.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest


pytest.importorskip("rclpy")

from test_placement_reference import _install_frame  # noqa: E402
from test_prepare_lifecycle import _bind  # noqa: E402
from test_prepare_lifecycle import _fake_generator  # noqa: E402
from test_prepare_lifecycle import _TestClock  # noqa: E402
from test_prepare_lifecycle import _TestTime  # noqa: E402
from trajectory_generator.trajectory_generator_node import (  # noqa: E402
    TrajectoryGenerator,
)


GPS_SERPENTINE = [
    (13.0000000, 80.0000000),
    (13.0000000, 80.0000200),
    (12.9999900, 80.0000200),
    (12.9999900, 80.0000000),
]

LOCAL_LINE = [
    (0.0, 0.0),
    (1.0, 0.0),
    (1.0, 1.0),
]


def _placement_generator():
    node = _fake_generator()

    node.frame_id = "map"
    node.max_target_distance_m = 500.0
    node.maximum_navigation_points = 200_000
    node.minimum_dummy_clearance_m = 0.05
    node.gp_origin_request_attempts = 0

    for name in (
        "POINT_TYPE_PASS_THROUGH",
        "POINT_TYPE_DUMMY_ALIGNMENT",
        "POINT_TYPE_MARKING",
    ):
        setattr(node, name, getattr(TrajectoryGenerator, name))

    for name in (
        "_convert_markings_to_local",
        "_convert_markings_px4_origin",
        "_generate_navigation_path",
        "_append_point",
        "_append_interpolated_segment",
        "_calculate_dummy_point",
        "_publish_empty_outputs",
        "_fused_global_position_callback",
        "_local_odom_callback",
        "_gp_origin_callback",
    ):
        _bind(node, name)

    node._distance = TrajectoryGenerator._distance
    node._make_signature = TrajectoryGenerator._make_signature
    node.get_clock = lambda: _TestClock(1_000_000_000)
    node.last_wait_log_time = _TestTime(0)

    events = []
    node.events = events

    node._build_path = lambda points, stamp: tuple(points)
    node.mission_waypoints_pub = SimpleNamespace(
        publish=lambda path: events.append(("waypoints", path))
    )
    node.nav_path_pub = SimpleNamespace(
        publish=lambda path: events.append(("nav_path", path))
    )
    node._publish_path_metadata = lambda **kwargs: events.append(
        ("metadata", kwargs)
    )
    node._publish_path_signature = lambda signature: events.append(
        ("signature", signature)
    )
    node._publish_survey_targets = (
        lambda signature, cleared=False: events.append(
            ("survey", signature, cleared)
        )
    )
    node._publish_ready = lambda value: events.append(("ready", bool(value)))
    node._publish_status = lambda **kwargs: events.append(
        ("status", node._build_status_payload(**kwargs))
    )

    return node


def _mission(node, mission_id, mode, points, *, extension="ENABLE"):
    metadata = {
        "mission_id": mission_id,
        "checksum_sha256": mission_id[-1] * 64,
        "extension_mode": extension,
    }
    node._load_mission_source = lambda: (metadata, mode, list(points))


def _prepare(node):
    response = SimpleNamespace(success=False, message="")
    TrajectoryGenerator._prepare_callback(node, None, response)
    assert response.success is True, response.message
    return response


def _fire_kick(node):
    """Simulate the executor running the armed one-shot placement timer."""

    if node._placement_kick_timer.armed:
        TrajectoryGenerator._on_placement_kick(node)


def _names(node):
    return [event[0] for event in node.events]


def _signatures(node):
    return [
        event[1]
        for event in node.events
        if event[0] == "signature" and event[1]
    ]


def _control_tick_forbidden(node):
    node._maybe_request_gp_origin = lambda: pytest.fail(
        "the 5 Hz control tick must not be needed"
    )


def test_local_prepare_is_ready_without_the_control_timer():
    node = _placement_generator()
    _control_tick_forbidden(node)
    _mission(node, "local-a", "local", LOCAL_LINE, extension="DISABLE")

    _prepare(node)

    # PREPARE returns before generating; the one-shot kick is armed.
    assert node.ready is False
    assert node._placement_kick_timer.armed is True

    _fire_kick(node)

    assert node.ready is True
    assert node._trajectory_phase() == "PLACED"
    assert len(_signatures(node)) == 1
    assert node.prepared_marking_points[0] == LOCAL_LINE[0]


def test_gps_prepare_with_valid_reference_places_on_first_kick():
    node = _placement_generator()
    _install_frame(node)
    _control_tick_forbidden(node)
    _mission(node, "gps-a", "gps", GPS_SERPENTINE)

    _prepare(node)
    _fire_kick(node)

    assert node.ready is True
    assert node._trajectory_phase() == "PLACED"

    ready_status = [e[1] for e in node.events if e[0] == "status"][-1]
    assert ready_status["state"] == "READY"
    assert ready_status["placement_ready"] is True
    # Two row transitions on this serpentine; dummies still materialize.
    assert ready_status["dummy_point_count"] == 2


def test_gps_waits_then_places_on_first_completing_callback():
    node = _placement_generator()
    _install_frame(node)
    _control_tick_forbidden(node)
    _mission(node, "gps-a", "gps", GPS_SERPENTINE)

    good_odom = node.latest_local_odom
    node.latest_local_odom = None

    _prepare(node)
    _fire_kick(node)

    assert node.ready is False
    assert node.prepare_requested is True
    assert _signatures(node) == []

    # A fused-global sample alone does not complete the reference.
    TrajectoryGenerator._fused_global_position_callback(
        node, node.latest_fused_global_fix
    )
    _fire_kick(node)
    assert node.ready is False

    # The local odometry sample completes it; placement follows at once.
    TrajectoryGenerator._local_odom_callback(node, good_odom)
    assert node._placement_kick_timer.armed is True
    _fire_kick(node)

    assert node.ready is True
    assert len(_signatures(node)) == 1


def test_repeated_callbacks_never_republish_a_placed_trajectory():
    node = _placement_generator()
    _install_frame(node)
    _mission(node, "gps-a", "gps", GPS_SERPENTINE)

    _prepare(node)
    _fire_kick(node)
    assert node.ready is True

    published = len(node.events)
    kicks = node._placement_kick_timer.reset_count

    odom = node.latest_local_odom
    for _ in range(50):
        TrajectoryGenerator._local_odom_callback(node, odom)
        TrajectoryGenerator._fused_global_position_callback(
            node, node.latest_fused_global_fix
        )
        _fire_kick(node)

    node._maybe_request_gp_origin = lambda: None
    for _ in range(5):
        TrajectoryGenerator._control_loop(node)

    assert len(node.events) == published
    # Once placed, sensor callbacks do not even arm the kick.
    assert node._placement_kick_timer.reset_count == kicks
    assert len(_signatures(node)) == 1


def test_replacement_prepare_invalidates_previous_transaction():
    node = _placement_generator()
    _install_frame(node)
    _mission(node, "gps-a", "gps", GPS_SERPENTINE)
    _prepare(node)
    _fire_kick(node)
    first_signature = node.prepared_path_signature

    node.events.clear()
    _mission(node, "gps-b", "gps", GPS_SERPENTINE[:3])
    _prepare(node)

    # A's placed outputs are retracted before B is placed.
    assert ("signature", None) in node.events
    assert node.ready is False
    assert node.prepared_path_signature is None
    assert node.source_topology_mission_id == "gps-b"

    _fire_kick(node)

    assert node.ready is True
    assert node.mission_id == "gps-b"
    assert node.prepared_path_signature not in (None, first_signature)


def test_unbound_source_topology_cannot_be_placed():
    node = _placement_generator()
    _install_frame(node)
    _mission(node, "gps-a", "gps", GPS_SERPENTINE)
    _prepare(node)

    node.source_topology_checksum = "0" * 64
    _fire_kick(node)

    assert node.ready is False
    assert node.prepare_requested is False
    assert "Source topology does not belong" in node.last_error
    assert _signatures(node) == []


def test_placement_error_retracts_outputs_and_keeps_compiled_source():
    node = _placement_generator()
    _install_frame(node)
    _mission(node, "gps-a", "gps", GPS_SERPENTINE)
    node.max_target_distance_m = 0.5

    _prepare(node)
    node.events.clear()
    _fire_kick(node)

    assert node.ready is False
    assert node._trajectory_phase() == "COMPILED"
    assert "from the PX4 estimator origin" in node.last_error
    assert ("nav_path", ()) in node.events
    assert ("signature", None) in node.events
    assert ("survey", None, True) in node.events
    assert _signatures(node) == []


def test_signature_is_the_commit_marker_after_path_and_metadata():
    node = _placement_generator()
    _install_frame(node)
    _mission(node, "gps-a", "gps", GPS_SERPENTINE)

    _prepare(node)
    node.events.clear()
    _fire_kick(node)

    names = _names(node)
    signature = names.index("signature")

    assert names.index("waypoints") < signature
    assert names.index("nav_path") < signature
    assert names.index("metadata") < signature
    assert signature < names.index("ready")
    assert names.count("signature") == 1

    status = [e[1] for e in node.events if e[0] == "status"]
    assert [s["state"] for s in status] == ["READY"]


def test_new_gp_origin_callback_kicks_waiting_mission():
    node = _placement_generator()
    _install_frame(node)
    origin = node.latest_gp_origin
    node.latest_gp_origin = None
    _mission(node, "gps-a", "gps", GPS_SERPENTINE)

    _prepare(node)
    _fire_kick(node)
    assert node.ready is False

    TrajectoryGenerator._gp_origin_callback(node, origin)
    _fire_kick(node)

    assert node.ready is True


def test_surveyed_points_are_not_moved_to_rover_pose():
    node = _placement_generator()
    _install_frame(node)
    rover = node.latest_local_odom.pose.pose.position
    # The rover is metres away from P1 (the gp_origin here); placement
    # must not pull P1 or the path start toward the rover pose.
    assert abs(rover.x) + abs(rover.y) > 5.0
    _mission(node, "gps-a", "gps", GPS_SERPENTINE)

    _prepare(node)
    _fire_kick(node)

    assert node.prepared_marking_points[0] == pytest.approx((0.0, 0.0))
    assert node.prepared_navigation_points[0] == pytest.approx((0.0, 0.0))
