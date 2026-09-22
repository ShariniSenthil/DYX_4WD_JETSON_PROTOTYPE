"""Execute production callback/route bodies with ROS and network I/O replaced.

ROS is unavailable on the workstation. AST extraction keeps these tests tied
to the actual integration code, following test_rtk_routes.py's monitor tests.
"""

from __future__ import annotations

import ast
import asyncio
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import threading
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

import pytest

from rover_backend.realtime_contract import ChangeDrivenEmitter, RealtimeMetrics, timed
from rover_backend.trajectory_push import (
    TrajectorySnapshotService,
    legacy_points,
    push_enabled,
)
from test_trajectory_push import Fixture, fake_projector


SOURCE = Path(__file__).resolve().parents[1] / "rover_backend"


def production_function(filename, name, namespace, *, node_method=False):
    tree = ast.parse((SOURCE / filename).read_text())
    if node_method:
        tree = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                    and n.name == "RoverBackendRosNode")
    function = next(n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
                    and n.name == name)
    function.decorator_list = []
    namespace.setdefault("Depends", lambda _: None)
    namespace.setdefault("require_auth", object())
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(SOURCE / filename), "exec"), namespace)
    return namespace[name]


class State:
    def __init__(self, **overrides):
        self.mission = {
            "mission_id": "m1", "loaded": True, "trajectory_ready": True,
            "navigation_path_preview": [], "navigation_point_count": 0,
            **overrides,
        }

    def section(self, _section):
        return dict(self.mission)

    def update(self, _section, **updates):
        self.mission.update(updates)

    def set_mission_state(self, state, **updates):
        self.update("mission", state=state, **updates)


@pytest.fixture
def context(monkeypatch):
    monkeypatch.setenv("DYX_TRAJECTORY_PUSH", "1")
    service = TrajectorySnapshotService(projector=fake_projector)
    for feed in Fixture().inputs().values():
        feed(service)
    state = State()
    sio = SimpleNamespace(emit=AsyncMock())
    return {
        "trajectory_snapshot": service, "rover_state": state, "sio": sio,
        "legacy_points": legacy_points, "push_enabled": push_enabled,
        "_mission_state": lambda: state.section("mission"),
    }


@pytest.mark.parametrize("updates", [
    {"mission_id": "m2"}, {"trajectory_ready": False}, {"loaded": False},
])
def test_loaded_path_and_reconnect_never_serve_a_revoked_mission(context, updates):
    context["rover_state"].update("mission", **updates)
    route = production_function("mission_routes.py", "loaded_path", context)
    replay = production_function("realtime.py", "_replay_trajectory", context)
    response = route()
    assert response["snapshot"] is None
    assert response["points"] == []
    asyncio.run(replay("client"))
    context["sio"].emit.assert_not_awaited()


def test_prepare_failure_drops_cache_before_service_call_and_blocks_delayed_feeds(context):
    context["rover_state"].update("mission", mission_id="m2")
    service = context["trajectory_snapshot"]

    def rejected(**_kwargs):
        assert service.live_payload() is None
        # Queued old retained callbacks during the failed RPC cannot revive it.
        for feed in Fixture().inputs().values():
            feed(service)
        return False, "ROS service unavailable"

    node = SimpleNamespace(
        _runtime_lock=threading.RLock(), _trajectory_prepare_client=object(),
        _call_trigger=rejected,
        _snapshot_feed=lambda method, *args: getattr(service, method)(*args),
    )
    prepare = production_function("ros_bridge.py", "prepare_trajectory", context, node_method=True)
    with pytest.raises(RuntimeError, match="unavailable"):
        prepare(node)
    assert context["rover_state"].mission["trajectory_ready"] is False
    assert service.live_payload() is None
    route = production_function("mission_routes.py", "loaded_path", context)
    replay = production_function("realtime.py", "_replay_trajectory", context)
    assert route()["snapshot"] is None
    assert route()["points"] == []
    asyncio.run(replay("client"))
    context["sio"].emit.assert_not_awaited()


def test_successful_prepare_resumes_only_after_acceptance(context):
    service = context["trajectory_snapshot"]
    fx = Fixture()
    service.feed_nav(fx.nav, (10, 0))

    def accepted(**_kwargs):
        for feed in fx.inputs().values():
            feed(service)
        service.feed_nav(fx.nav, (11, 0))
        assert service.live_payload() is None
        return True, "accepted"

    node = SimpleNamespace(
        _runtime_lock=threading.RLock(), _trajectory_prepare_client=object(),
        _call_trigger=accepted,
        _snapshot_feed=lambda method, *args: getattr(service, method)(*args),
    )
    prepare = production_function("ros_bridge.py", "prepare_trajectory", context, node_method=True)
    prepare(node)
    assert service.live_payload() is not None


def test_clear_invalidates_cache_before_any_ros_call_even_if_service_fails(context):
    service = context["trajectory_snapshot"]

    def unavailable(_command):
        assert service.live_payload() is None
        raise RuntimeError("manager unavailable")

    clear = production_function("ros_bridge.py", "clear_mission", context, node_method=True)
    with pytest.raises(RuntimeError, match="unavailable"):
        clear(SimpleNamespace(_manager_command=unavailable))
    assert service.live_payload() is None


def test_socket_replay_and_queued_emission_use_current_authority(context):
    service = context["trajectory_snapshot"]
    event = {"event": "trajectory_path", "payload": service.live_payload()}
    emit = production_function("realtime.py", "_emit_trajectory_event", context)
    replay = production_function("realtime.py", "_replay_trajectory", context)
    asyncio.run(replay("client"))
    context["sio"].emit.assert_awaited_once_with("trajectory_path", event["payload"], to="client")
    context["sio"].emit.reset_mock()
    service.invalidate("mission_replaced")
    asyncio.run(emit(event))
    asyncio.run(replay("client"))
    context["sio"].emit.assert_not_awaited()


@pytest.mark.parametrize("flag", ["0", "false", "off"])
def test_flag_off_skips_full_path_and_metadata_extraction(context, monkeypatch, flag):
    monkeypatch.setenv("DYX_TRAJECTORY_PUSH", flag)
    node = SimpleNamespace(
        _mark_ros_message=lambda: None,
        _path_points=Mock(side_effect=AssertionError("full path extracted")),
        _snapshot_feed=Mock(side_effect=AssertionError("snapshot fed")),
    )
    # Existing bounded-preview work is allowed; new extraction must never run.
    message = SimpleNamespace(poses=[], header=SimpleNamespace(frame_id="map"))
    for name in ("_nav_path_callback", "_mission_waypoints_callback"):
        callback = production_function("ros_bridge.py", name, context, node_method=True)
        callback(node, message)
    # No data attribute: even accessing metadata while disabled fails this test.
    for name in ("_path_types_callback", "_marking_indices_callback", "_path_signature_callback"):
        callback = production_function("ros_bridge.py", name, context, node_method=True)
        callback(node, object())
    node._path_points.assert_not_called()
    node._snapshot_feed.assert_not_called()


def test_http_and_socket_do_not_serve_cache_when_flag_disabled(context, monkeypatch):
    monkeypatch.setenv("DYX_TRAJECTORY_PUSH", "false")
    route = production_function("mission_routes.py", "loaded_path", context)
    replay = production_function("realtime.py", "_replay_trajectory", context)
    assert route()["snapshot"] is None
    asyncio.run(replay("client"))
    context["sio"].emit.assert_not_awaited()


def test_rejected_origin_cannot_reappear_through_legacy_rest_fallback(context, monkeypatch):
    legacy = [{"x": 1.0, "y": 2.0, "lat": 13.0, "lon": 80.0}]
    context["rover_state"].update(
        "mission", navigation_path_preview=legacy, navigation_point_count=180,
        navigation_path_preview_truncated=True, path_frame_id="map",
    )
    context["trajectory_snapshot"].feed_origin(13.001, 80.0)
    route = production_function("mission_routes.py", "loaded_path", context)
    response = route()
    assert response["points"] == []
    # The real count is reported (an empty list with count 0 reads as a failed
    # path); the state says WHY there are no points: re-prepare is required.
    assert response["navigation_point_count"] == 180
    assert response["snapshot_state"] == "invalid"
    assert response["snapshot_reason"] == "prepared_origin_changed"
    assert response["requires_reprepare"] is True
    assert response["assembling"] is False
    # Explicit rollback retains the original bounded-preview response.
    monkeypatch.setenv("DYX_TRAJECTORY_PUSH", "off")
    assert route()["points"] == legacy
    assert route()["preview_truncated"] is True


@pytest.mark.parametrize("operation", ["prepare_mission", "clear_mission"])
def test_route_revokes_preview_even_when_bridge_never_dispatches(context, operation):
    service = context["trajectory_snapshot"]

    async def unavailable(*_args):
        assert service.live_payload() is None
        raise RuntimeError("bridge unavailable")

    async def threadpool(function, *args):
        return function(*args)

    context.update({
        "_serialize_mission_mutation": object(),
        "_require_not_active": lambda **kwargs: None,
        "_run_ros_operation": unavailable,
        "run_in_threadpool": threadpool,
        "mission_store": SimpleNamespace(load_metadata=lambda: {"mission_id": "m1"}),
        "ros_bridge": SimpleNamespace(prepare_trajectory=Mock(), clear_mission=Mock()),
    })
    route = production_function("mission_routes.py", operation, context)
    with pytest.raises(RuntimeError, match="bridge unavailable"):
        asyncio.run(route())
    assert service.live_payload() is None


def test_realtime_loop_expires_pending_snapshot_without_clients_or_rest(context):
    service = context["trajectory_snapshot"]
    now = [100.0]
    service._clock = lambda: now[0]
    service.feed_types([0] * len(Fixture().types))
    events = []
    service.add_listener(events.append)
    now[0] += service.assembly_timeout_s + 1

    async def run_iteration():
        stop = asyncio.Event()
        changed = asyncio.Event()

        async def no_clients():
            stop.set()
            changed.set()
            return []

        context.update({
            "asyncio": asyncio,
            "_stop_event": stop,
            "_state_change_event": changed,
            "settings": SimpleNamespace(telemetry_broadcast_hz=1,
                                        mission_status_heartbeat_sec=1.0),
            "_all_socket_records": no_clients,
            "LOGGER": Mock(),
            "realtime_metrics": RealtimeMetrics(enabled=False),
            "timed": timed,
            "ChangeDrivenEmitter": ChangeDrivenEmitter,
            "_emit_pending_point_events": lambda deliver: asyncio.sleep(0),
        })
        broadcast = production_function("realtime.py", "_broadcast_loop", context)
        await asyncio.wait_for(broadcast(), timeout=1)
        context["LOGGER"].exception.assert_not_called()

    asyncio.run(run_iteration())
    assert len(events) == 1
    assert events[0]["payload"]["reason"] == "snapshot_assembly_timeout"
    assert events[0]["payload"]["requires_reprepare"] is True


def test_timeout_check_does_not_wait_for_projection_lock(context):
    service = context["trajectory_snapshot"]
    # Hold the lock in this thread while the periodic check runs elsewhere.
    with ThreadPoolExecutor(max_workers=1) as executor:
        with service._lock:
            future = executor.submit(service.check_timeout)
            future.result(timeout=1)


def test_pending_snapshot_reports_real_count_and_assembling_not_an_empty_path(context):
    """READY can reach the backend before its path: that is pending, not empty."""
    fresh = TrajectorySnapshotService(projector=fake_projector)
    fixture = Fixture()
    fixture.inputs()["origin"](fresh)
    fixture.inputs()["status"](fresh)  # READY status, no path components yet
    context["trajectory_snapshot"] = fresh
    context["rover_state"].update("mission", navigation_point_count=180)
    response = production_function("mission_routes.py", "loaded_path", context)()
    assert response["points"] == []
    assert response["navigation_point_count"] == 180  # never 0 while a path is expected
    assert response["snapshot_state"] == "assembling"
    assert response["assembling"] is True
    assert response["requires_reprepare"] is False
