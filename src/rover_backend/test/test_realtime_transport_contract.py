"""Realtime Socket.IO transport contract (Phases 4-6, 8).

Telemetry size is bounded, mission_status is change-driven, and every point
event is delivered once, in order, carrying the exact stored point result.
"""

from __future__ import annotations

import ast
import asyncio
import collections
import copy
import json
import traceback
from pathlib import Path
from types import SimpleNamespace

import pytest

from helpers.realtime_fixtures import import_system_routes
from helpers.realtime_fixtures import populate_mission
from helpers.realtime_fixtures import synthetic_point_result
from rover_backend import realtime_contract as rc
from rover_backend.realtime_contract import payload_size_bytes

SOURCE = Path(__file__).resolve().parents[1] / "rover_backend"
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


def _emit_if_due(emitter, signature):
    if emitter.should_emit(signature):
        emitter.commit(signature)
        return True
    return False


def test_change_driven_emitter_heartbeat_and_immediate_change():
    clock = Clock()
    emitter = rc.ChangeDrivenEmitter(heartbeat_sec=1.0, clock=clock)
    assert _emit_if_due(emitter, "a") is True        # first packet
    emitted = 0
    for _ in range(49):                               # ~1 s at 50 Hz, unchanged
        clock.now += 0.02
        emitted += _emit_if_due(emitter, "a")
    assert emitted == 0
    clock.now += 0.02
    assert _emit_if_due(emitter, "a") is True        # heartbeat
    clock.now += 0.001
    assert _emit_if_due(emitter, "b") is True        # change: immediate
    emitter.reset()
    assert _emit_if_due(emitter, "b") is True        # new client after reset


def test_failed_emit_is_retried_on_the_next_iteration():
    clock = Clock()
    emitter = rc.ChangeDrivenEmitter(heartbeat_sec=60.0, clock=clock)
    _emit_if_due(emitter, "a")
    assert emitter.should_emit("estop") is True
    # emit raised -> commit() never called -> the change is still pending
    clock.now += 0.02
    assert emitter.should_emit("estop") is True


def test_default_heartbeat_is_slow_for_the_deployed_tablet():
    from rover_backend.config import settings
    assert settings.mission_status_heartbeat_sec == 5.0
    assert settings.socket_mission_status_compact is False


# --------------------------------------------------------------------- C


def test_point_result_event_copies_exact_stored_result():
    stored = synthetic_point_result(24)
    raw_event = {
        "event": "COMPLETED", "point_id": "P0025", "point_index": 24,
        "mission_run_id": "run-1", "received_at": "t",
        "accuracy": stored["accuracy"],
    }
    event = rc.build_point_result_event(raw_event, stored, "mission-1")
    assert event["contract"] == "point_result@1"
    assert event["mission_id"] == "mission-1"
    assert event["point_result"]["mission_id"] == "mission-1"
    assert "event_history" not in event["point_result"]
    accuracy = event["point_result"]["accuracy"]
    assert accuracy == stored["accuracy"]
    assert accuracy["measurement_source"] == "RPP_TERMINAL_RESULT"
    for key in ("along_track_error_mm", "cross_track_error_mm",
                "overall_accuracy_mm", "tolerance_mm", "within_tolerance",
                "timestamp_unix_ns"):
        assert accuracy[key] == stored["accuracy"][key]
    assert event["point_result"]["spray"] == stored["spray"]
    # Original event fields are preserved for existing consumers.
    assert event["event"] == "COMPLETED" and event["point_index"] == 24
    assert rc.point_event_socket_name(event) == "point_completed"
    assert rc.point_event_socket_name({"event": "ACCURACY_ACHIEVED"}) == "point_event"


def test_point_result_event_never_invents_a_result():
    event = rc.build_point_result_event({"event": "MISSION_TERMINATED"}, None, "m")
    assert event["point_result"] is None


# ------------------------------------------------------- broadcast loop


def _production(name, namespace):
    tree = ast.parse((SOURCE / "realtime.py").read_text())
    fn = next(n for n in tree.body
              if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name == name)
    exec(compile(ast.Module(body=[fn], type_ignores=[]), "realtime.py", "exec"), namespace)
    return namespace[name]


def test_loop_emits_every_point_event_in_order_and_mission_status_on_change(
    routes, state,
):
    populate_mission(state, 5, completed=2)
    state.update("mission", emergency_stop=False)
    emitted: list[tuple[str, dict]] = []
    failures: list[str] = []
    queue: collections.deque = collections.deque()

    async def run():
        stop = asyncio.Event()
        changed = asyncio.Event()
        iterations = {"n": 0}
        script = {
            # iteration -> action before records are read
            1: lambda: [queue.append({"event": "ACCURACY_ACHIEVED", "point_id": "P0003"}),
                        queue.append({"event": "COMPLETED", "point_id": "P0003"})],
            4: lambda: state.update("mission", emergency_stop=True),
        }

        async def records():
            iterations["n"] += 1
            action = script.get(iterations["n"])
            if action:
                action()
            if iterations["n"] >= 7:
                stop.set()
                changed.set()
            return [("sid", object())]

        async def fake_emit(event, payload, **_kw):
            emitted.append((event, copy.deepcopy(payload)))

        namespace = {
            "asyncio": asyncio,
            "_stop_event": stop,
            "_state_change_event": changed,
            "settings": SimpleNamespace(telemetry_broadcast_hz=200,
                                        mission_status_heartbeat_sec=60.0,
                                        socket_mission_status_compact=False,
                                        arrival_settle_seconds=0.3,
                                        marking_hold_seconds=3.0),
            "_all_socket_records": records,
            "trajectory_snapshot": SimpleNamespace(check_timeout=lambda: None),
            "_pending_point_events": queue,
            "_emit": fake_emit,
            "build_mission_status_payload": routes.build_mission_status_payload,
            "build_telemetry_payload": routes.build_telemetry_payload,
            "rover_state": state,
            "realtime_metrics": rc.RealtimeMetrics(enabled=False),
            "timed": rc.timed,
            "ChangeDrivenEmitter": rc.ChangeDrivenEmitter,
            "mission_lifecycle_signature": rc.mission_lifecycle_signature,
            "point_event_socket_name": rc.point_event_socket_name,
            "_revalidate_socket_sessions": lambda: asyncio.sleep(0),
            "LOGGER": SimpleNamespace(exception=lambda *a, **k: failures.append(
                traceback.format_exc())),
        }
        _production("_stable_signature", namespace.setdefault("json", json) and namespace)
        _production("_mission_progress_payload", namespace)
        _production("_socket_mission_payload", namespace)
        _production("_emit_pending_point_events", namespace)
        loop = _production("_broadcast_loop", namespace)
        await asyncio.wait_for(loop(), timeout=5)

    asyncio.run(run())
    assert failures == []
    names = [name for name, _ in emitted]
    # Both same-tick point events delivered, in order, before that tick's
    # lifecycle packets.
    point_events = [(n, p["event"]) for n, p in emitted if n.startswith("point_")]
    assert point_events == [("point_event", "ACCURACY_ACHIEVED"),
                            ("point_completed", "COMPLETED")]
    # Telemetry every iteration; mission_status only first + on e-stop change.
    assert names.count("telemetry") == 7
    mission_status = [p for n, p in emitted if n == "mission_status"]
    assert len(mission_status) == 2
    assert mission_status[1]["emergency_stop"] is True
    # Queued before iteration 1 reads state: delivered ahead of its telemetry.
    assert names.index("point_completed") < names.index("telemetry")


def test_ros_point_event_callback_publishes_exactly_the_stored_result(state):
    """Run the production _point_event_callback body (ROS-free)."""

    import math
    from helpers.realtime_fixtures import synthetic_accuracy

    tree = ast.parse((SOURCE / "ros_bridge.py").read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == "RoverBackendRosNode")
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
              and n.name == "_point_event_callback")
    published: list[dict] = []
    namespace = {
        "copy": copy, "Any": object, "String": object,
        "rover_state": state,
        "_json_object": json.loads,
        "utc_now_iso": lambda: "2026-09-22T10:00:00+00:00",
        "_safe_int": lambda v, d=0: int(v) if isinstance(v, int) else d,
        "_finite_float": lambda v, d=None: (
            float(v) if isinstance(v, (int, float)) and math.isfinite(v) else d),
        "_notify_authoritative_state_changed": lambda: None,
        "_publish_point_result_event": published.append,
        "build_point_result_event": rc.build_point_result_event,
    }
    exec(compile(ast.Module(body=[fn], type_ignores=[]), "ros_bridge.py", "exec"), namespace)
    node = SimpleNamespace(_mark_ros_message=lambda: None,
                           _schedule_live_report_checkpoint=lambda: None,
                           _schedule_terminal_mission_cleanup=lambda _e: None)
    populate_mission(state, 3, completed=0)
    accuracy = synthetic_accuracy(0)
    for event_name in ("ACCURACY_ACHIEVED", "COMPLETED"):
        message = SimpleNamespace(data=json.dumps({
            "event": event_name, "point_id": "P0001", "point_index": 0,
            "mission_run_id": "run-1", "state": "COMPLETED",
            "accuracy": accuracy,
            "spray": {"attempted": True, "outcome": "SUCCESS",
                      "reason": None, "elapsed_sec": 0.5},
        }))
        namespace["_point_event_callback"](node, message)

    assert [e["event"] for e in published] == ["ACCURACY_ACHIEVED", "COMPLETED"]
    stored = state.section("mission")["point_results"]["P0001"]
    last = published[-1]
    assert last["mission_id"] == "mission-1"
    assert last["mission_run_id"] == "run-1"
    expected = {k: v for k, v in stored.items() if k != "event_history"}
    expected["mission_id"] = "mission-1"
    assert last["point_result"] == expected
    for key in ("along_track_error_mm", "cross_track_error_mm",
                "overall_accuracy_mm", "timestamp_unix_ns"):
        assert last["point_result"]["accuracy"][key] == accuracy[key]
    assert last["point_result"]["accuracy"]["measurement_source"] == "RPP_TERMINAL_RESULT"
    # The published event is an independent copy of state.
    last["point_result"]["accuracy"]["overall_accuracy_mm"] = -1
    assert state.section("mission")["point_results"]["P0001"]["accuracy"][
        "overall_accuracy_mm"] == accuracy["overall_accuracy_mm"]
