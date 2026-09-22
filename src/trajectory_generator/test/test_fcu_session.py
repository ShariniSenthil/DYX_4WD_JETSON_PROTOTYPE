"""Placement is bound to the current PX4/MAVROS session's gp_origin."""

from __future__ import annotations

from types import SimpleNamespace

import pytest


pytest.importorskip("rclpy")

from test_event_placement import _fire_kick  # noqa: E402
from test_event_placement import _mission  # noqa: E402
from test_event_placement import _placement_generator  # noqa: E402
from test_event_placement import _prepare  # noqa: E402
from test_event_placement import _signatures  # noqa: E402
from test_event_placement import GPS_SERPENTINE  # noqa: E402
from test_event_placement import LOCAL_LINE  # noqa: E402
from test_placement_reference import _install_frame  # noqa: E402
from trajectory_generator.trajectory_generator_node import (  # noqa: E402
    TrajectoryGenerator,
)


def _state(node, connected):
    TrajectoryGenerator._mavros_state_callback(
        node,
        SimpleNamespace(connected=connected),
    )


def _placed_gps_node():
    node = _placement_generator()
    _install_frame(node)
    _mission(node, "gps-a", "gps", GPS_SERPENTINE)
    _prepare(node)
    _fire_kick(node)
    assert node._trajectory_phase() == "PLACED"
    return node


def _retracted(events):
    return (
        ("signature", None) in events
        and ("nav_path", ()) in events
        and ("ready", False) in events
    )


def test_disconnect_after_ready_retracts_path_but_keeps_source():
    node = _placed_gps_node()
    topology = node.source_topology
    session = node.fcu_session_generation
    node.events.clear()

    _state(node, False)

    assert node.fcu_session_generation == session + 1
    assert node.latest_gp_origin is None
    assert node.ready is False
    assert node.prepared_path_signature is None
    assert _retracted(node.events)

    assert node.source_topology is topology
    assert node._source_topology_is_current() is True
    assert node._trajectory_phase() == "COMPILED"
    assert node.prepare_requested is True

    status = [e[1] for e in node.events if e[0] == "status"][-1]
    assert status["state"] == "PREPARING"
    assert status["trajectory_phase"] == "COMPILED"


def test_reconnect_without_new_origin_does_not_place():
    node = _placed_gps_node()
    _state(node, False)
    _state(node, True)

    assert node.mavros_connected is True
    _fire_kick(node)

    node._maybe_request_gp_origin = lambda: None
    TrajectoryGenerator._control_loop(node)

    ready, reason = node._placement_reference_is_ready()
    assert ready is False
    assert "gp_origin unavailable" in reason
    assert node.ready is False


def test_previous_session_origin_cannot_place():
    node = _placed_gps_node()
    stale_origin = node.latest_gp_origin
    stale_session = node.gp_origin_session_generation

    _state(node, False)
    _state(node, True)

    # Even if the old origin object were still present, it is bound to the
    # session it arrived in and is refused.
    node.latest_gp_origin = stale_origin
    node.gp_origin_session_generation = stale_session

    ready, reason = node._placement_reference_is_ready()
    assert ready is False
    assert "previous FCU session" in reason

    _fire_kick(node)
    node._maybe_request_gp_origin = lambda: None
    TrajectoryGenerator._control_loop(node)
    assert node.ready is False


def test_disconnected_fcu_cannot_place():
    node = _placed_gps_node()
    _state(node, False)

    ready, reason = node._placement_reference_is_ready()
    assert ready is False
    assert "not connected" in reason


def test_fresh_session_origin_restores_placement_exactly_once():
    node = _placed_gps_node()
    origin = node.latest_gp_origin
    first_signature = node.prepared_path_signature
    first_session = node.placed_session_generation

    _state(node, False)
    _state(node, True)
    node.events.clear()

    TrajectoryGenerator._gp_origin_callback(node, origin)
    _fire_kick(node)

    assert node._trajectory_phase() == "PLACED"
    assert node.placed_session_generation == node.fcu_session_generation
    assert node.placed_session_generation != first_session
    assert node.placement_timing["armed_reason"] == "fcu_session"
    assert node.placement_timing["armed_to_ready_ms"] >= 0.0
    # Same surveyed mission, same origin: identical geometry.
    assert node.prepared_path_signature == first_signature
    assert len(_signatures(node)) == 1

    # Further sensor traffic does not republish.
    for _ in range(10):
        TrajectoryGenerator._local_odom_callback(node, node.latest_local_odom)
        TrajectoryGenerator._gp_origin_callback(node, origin)
        _fire_kick(node)
    assert len(_signatures(node)) == 1


def test_placed_phase_requires_current_session_binding():
    node = _placed_gps_node()

    node.fcu_session_generation += 1

    assert node._trajectory_phase() == "COMPILED"


def test_disconnect_while_waiting_discards_origin_and_keeps_waiting():
    node = _placement_generator()
    _install_frame(node)
    node.latest_local_odom = None
    _mission(node, "gps-a", "gps", GPS_SERPENTINE)
    _prepare(node)
    _fire_kick(node)
    assert node.prepare_requested is True

    _state(node, False)

    assert node.latest_gp_origin is None
    assert node.prepare_requested is True
    assert node._trajectory_phase() == "COMPILED"


def test_first_state_report_keeps_origin_received_before_it():
    node = _placement_generator()
    _install_frame(node)
    origin = node.latest_gp_origin
    node.mavros_connected = False
    node._mavros_state_seen = False
    node.fcu_session_generation = 0
    node.latest_gp_origin = None
    node.gp_origin_session_generation = None

    TrajectoryGenerator._gp_origin_callback(node, origin)
    _state(node, True)

    assert node.latest_gp_origin is origin
    assert node.gp_origin_session_generation == node.fcu_session_generation


def test_local_mission_is_not_bound_to_gp_origin_session():
    node = _placement_generator()
    _mission(node, "local-a", "local", LOCAL_LINE, extension="DISABLE")
    _prepare(node)
    _fire_kick(node)
    signature = node.prepared_path_signature
    node.events.clear()

    _state(node, True)
    _state(node, False)

    assert node._trajectory_phase() == "PLACED"
    assert node.prepared_path_signature == signature
    assert node.events == []


def test_same_session_origin_change_still_errors():
    node = _placed_gps_node()
    moved = SimpleNamespace(
        position=SimpleNamespace(
            latitude=13.001,
            longitude=80.0,
            altitude=0.0,
        ),
    )

    TrajectoryGenerator._gp_origin_callback(node, moved)

    assert node.ready is False
    assert node.prepare_requested is False
    assert "gp_origin changed" in node.last_error
    assert node._source_topology_is_current() is True
