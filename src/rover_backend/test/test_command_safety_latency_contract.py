"""Protect backend command ordering, timeout, and callback-I/O contracts."""

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BRIDGE_PATH = PACKAGE_ROOT / "rover_backend" / "ros_bridge.py"
REALTIME_PATH = PACKAGE_ROOT / "rover_backend" / "realtime.py"
BRIDGE_SOURCE = BRIDGE_PATH.read_text(encoding="utf-8")
BRIDGE_TREE = ast.parse(BRIDGE_SOURCE)


def function_source(name: str) -> str:
    matches = [
        node
        for node in ast.walk(BRIDGE_TREE)
        if isinstance(node, ast.FunctionDef) and node.name == name
    ]
    assert matches, f"missing function {name}"
    source = ast.get_source_segment(BRIDGE_SOURCE, matches[-1])
    assert source is not None
    return source


def test_estop_bypasses_ordinary_lock_and_release_is_generation_guarded():
    force = function_source("force_emergency_stop")
    release = function_source("release_emergency_stop")
    assert "self._operation_lock" not in force
    assert "self._safety_generation += 1" in force
    capture = release.index("requested_generation = self._safety_generation")
    ordinary_queue = release.index("with self._operation_lock:")
    validation = release.index("requested_generation != self._safety_generation")
    service_call = release.index("self.node.release_emergency_stop(")
    assert capture < ordinary_queue < validation < service_call
    assert "expected_manager_generation" in release
    assert "self.node.emergency_stop()" not in release


def test_manager_timeouts_cover_long_px4_operations_and_report_unknown():
    assert '"start": 30.0' in BRIDGE_SOURCE
    assert '"resume": 30.0' in BRIDGE_SOURCE
    assert '"next_point": 30.0' in BRIDGE_SOURCE
    assert '"stop": 20.0' in BRIDGE_SOURCE
    call_service = function_source("_call_service")
    assert "future.cancel()" in call_service
    assert "RosServiceOutcomeUnknownError" in call_service
    manager = function_source("_manager_command")
    assert "timeout_outcome_unknown=True" in manager


def test_point_callback_never_performs_durable_checkpoint_io():
    callback = function_source("_point_event_callback")
    worker = function_source("_report_checkpoint_worker")
    assert "checkpoint_live_report" not in callback
    assert "_schedule_live_report_checkpoint()" in callback
    assert "checkpoint_live_report" in worker
    assert "lifecycle_transaction" in worker


def test_realtime_has_event_driven_authoritative_state_wakeup():
    realtime = REALTIME_PATH.read_text(encoding="utf-8")
    assert "def notify_authoritative_state_changed" in realtime
    assert "state_change_event.wait()" in realtime
    assert "loop.call_soon_threadsafe(state_change_event.set)" in realtime


def test_command_response_waits_for_acknowledged_status_without_sleeping():
    proxy = function_source("_proxy_manager_operation")
    mark = proxy.index("self._manager_status_sync.mark(command)")
    dispatch = proxy.index("self._manager_command(command)")
    rejection = proxy.index("raise RuntimeError(service_message)")
    wait = proxy.index("self._manager_status_sync.wait_for_ack(")
    snapshot = proxy.index('return rover_state.section("mission")')
    assert mark < dispatch < rejection < wait < snapshot
    assert "time.sleep" not in proxy
    # A sync timeout is logged/recorded; only a manager rejection raises.
    assert proxy.count("raise ") == 1
    assert "last_command_status_sync" in proxy
    assert "MANAGER_STATUS_SYNC_TIMEOUT_SEC = 0.5" in BRIDGE_SOURCE


def test_status_ack_is_observed_only_after_state_commit():
    callback = function_source("_mission_status_callback")
    commit = callback.rindex('rover_state.update(\n            "mission",')
    observe = callback.index("self._manager_status_sync.observe(payload)")
    notify = callback.rindex("_notify_authoritative_state_changed()")
    assert commit < observe < notify


def test_sync_primitive_is_condition_based():
    sync_source = (PACKAGE_ROOT / "rover_backend" / "manager_status_sync.py").read_text(
        encoding="utf-8"
    )
    assert "threading.Condition()" in sync_source
    assert "wait_for(" in sync_source
    assert "time.sleep" not in sync_source
