"""Realtime Socket.IO transport contract.

High-rate telemetry size is bounded independent of mission history.
"""

from __future__ import annotations


import pytest

from helpers.realtime_fixtures import import_system_routes
from helpers.realtime_fixtures import populate_mission
from rover_backend import realtime_contract as rc
from rover_backend.realtime_contract import payload_size_bytes

ADAPTER_MISSION_FIELDS = {
    # px4TelemetryAdapter.toRoverTelemetry reads these from telemetry.mission
    "active_point_index", "active_point_number", "active_point_state",
    "total_points", "state", "progress_percent", "completed_points",
    "skipped_points", "failed_points", "remaining_points",
    "navigation_point_count", "loaded", "ready", "marking_active",
}


@pytest.fixture
def routes(monkeypatch):
    return import_system_routes(monkeypatch)


@pytest.fixture
def state():
    from rover_backend.state import rover_state
    return rover_state


# --------------------------------------------------------------------- A


def test_telemetry_size_is_independent_of_mission_history(routes, state):
    sizes = {}
    for points in (1, 100, 1000):
        populate_mission(state, points)
        sizes[points] = payload_size_bytes(routes.build_telemetry_payload())
    # Only digit widths of counters may differ.
    assert max(sizes.values()) - min(sizes.values()) < 64, sizes
    assert sizes[1000] < 8_000, sizes


def test_telemetry_mission_is_projection_with_adapter_fields(routes, state):
    populate_mission(state, 50, completed=20)
    mission = routes.build_telemetry_payload()["mission"]
    assert ADAPTER_MISSION_FIELDS <= set(mission)
    for heavy in ("point_results", "point_status", "report", "last_point_event",
                  "waypoint_survey_snapshots", "active_waypoint"):
        assert heavy not in mission
    assert mission["completed_points"] == 20
    assert mission["active_point_index"] == 20


def test_telemetry_reuses_supplied_mission_payload(routes, state):
    populate_mission(state, 3)
    supplied = routes.build_mission_status_payload()
    supplied["state"] = "SENTINEL"
    assert routes.build_telemetry_payload(supplied)["mission"]["state"] == "SENTINEL"


def test_rest_mission_status_contract_is_unchanged(routes, state):
    populate_mission(state, 5)
    full = routes.build_mission_status_payload()
    for key in ("point_results", "point_status", "report", "last_point_event"):
        assert key in full
    assert len(full["point_results"]) == 5
