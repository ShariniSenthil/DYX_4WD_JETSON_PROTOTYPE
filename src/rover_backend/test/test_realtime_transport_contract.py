"""Realtime Socket.IO transport contract.

High-rate telemetry size is bounded independent of mission history, and
socket mission_status is change-driven.
"""

from __future__ import annotations


import pytest

from helpers.realtime_fixtures import import_system_routes
from helpers.realtime_fixtures import populate_mission
from helpers.realtime_fixtures import synthetic_point_result
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


# --------------------------------------------------------------------- B


def test_compact_lifecycle_has_no_history_and_is_bounded(routes, state):
    sizes = {}
    for points in (10, 1000):
        populate_mission(state, points)
        section = state.section("mission")
        compact = rc.build_mission_lifecycle_payload(
            routes.build_mission_status_payload(), section)
        for heavy in ("point_results", "report", "last_point_event",
                      "waypoint_survey_snapshots", "active_waypoint"):
            assert heavy not in compact
        assert compact["contract"] == "mission_lifecycle@1"
        assert "execution_mode" in compact and "safety_generation" in compact
        sizes[points] = payload_size_bytes(compact)
    # Grows only by the per-point state string (~12 B/point), not history.
    assert sizes[1000] - sizes[10] < 990 * 16, sizes


def _signature(routes, rover_state, **mission_updates):
    if mission_updates:
        rover_state.update("mission", **mission_updates)
    return rc.mission_lifecycle_signature(
        routes.build_mission_status_payload(), rover_state.section("mission"))


def test_signature_ignores_volatile_diagnostics(routes, state):
    populate_mission(state, 10, completed=3)
    base = _signature(routes, state)
    assert _signature(routes, state, rtk_correction_age_sec=1.7,
                      hold_elapsed_sec=0.4, arrival_settle_elapsed_sec=0.2,
                      gps_fix_status_age_sec=0.3) == base


@pytest.mark.parametrize("change", [
    {"emergency_stop": False},
    {"px4_armed": True},
    {"px4_mode": "HOLD"},
    {"state": "PAUSED"},
    {"resume_available": True},
    {"rtk_motion_ok": True},
    {"start_stage": "ARMING"},
    {"spray_controller_state": "FAULT"},
    {"active_point_index": 4},
    {"message": "new"},
    {"error": "boom"},
    {"execution_mode": "MANUAL"},
    {"safety_generation": 9},
    {"point_status": ["COMPLETED"] * 4 + ["PENDING"] * 6},
])
def test_signature_changes_on_lifecycle_change(routes, state, change):
    populate_mission(state, 10, completed=3)
    base = _signature(routes, state)
    assert _signature(routes, state, **change) != base


def test_signature_changes_on_new_point_result_without_serializing_it(routes, state):
    populate_mission(state, 10, completed=3)
    base = _signature(routes, state)
    results = state.section("mission")["point_results"]
    results["P0004"] = synthetic_point_result(3)
    event = dict(results["P0004"], received_at="2026-09-22T10:00:09+00:00")
    state.update("mission", point_results=results, last_point_event=event)
    changed = _signature(routes, state)
    assert changed != base
    assert "event_history" not in changed and "RAW_GNSS_SURVEY" not in changed


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def test_change_driven_emitter_heartbeat_and_immediate_change():
    clock = Clock()
    emitter = rc.ChangeDrivenEmitter(heartbeat_sec=1.0, clock=clock)
    assert emitter.should_emit("a") is True          # first packet
    emitted = 0
    for _ in range(49):                               # ~1 s at 50 Hz, unchanged
        clock.now += 0.02
        emitted += emitter.should_emit("a")
    assert emitted == 0
    clock.now += 0.02
    assert emitter.should_emit("a") is True          # heartbeat
    clock.now += 0.001
    assert emitter.should_emit("b") is True          # change: immediate
    emitter.reset()
    assert emitter.should_emit("b") is True          # new client after reset
