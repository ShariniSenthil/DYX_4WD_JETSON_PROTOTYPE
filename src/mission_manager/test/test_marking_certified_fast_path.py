"""Certificate-backed radial20 marking: no duplicate Mission Manager dwell.

Drives the real ``_control_loop`` marking branch, the real RPP result /
certificate callbacks, the real certificate policy and the real accuracy
snapshot. Only ROS I/O (publishers, events, logging, clock) is recorded.
"""

from __future__ import annotations

import json
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

TEST_DIR = Path(__file__).resolve().parent
if str(TEST_DIR) not in sys.path:
    sys.path.insert(0, str(TEST_DIR))

# Installs ROS stubs when ROS is absent and imports the real node module.
import test_lifecycle_command_behavior as harness  # noqa: E402

node_module = harness.node_module
MissionManager = harness.MissionManager
from mission_manager.raw_gnss_spray_gate import (  # noqa: E402
    RawGnssSprayGateConfig,
)
from mission_manager.precision_terminal_policy import (  # noqa: E402
    PRECISION_TERMINAL_SCHEMA_VERSION,
    PRECISION_TERMINAL_SOURCE,
)

RUN_ID = "run-7f3a"
PATH_SIGNATURE = "a" * 64
GOALS = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]


class Clock:
    def __init__(self) -> None:
        self.t = 1000.0

    def monotonic(self) -> float:
        return self.t


@pytest.fixture
def clock(monkeypatch):
    fake = Clock()
    monkeypatch.setattr(node_module.time, "monotonic", fake.monotonic)
    return fake


def build(clock: Clock, *, mode: str = "radial20", execution: str = "AUTO"):
    m = object.__new__(MissionManager)
    events: list = []

    m._lock = threading.RLock()
    m.terminal_stop_mode = mode
    m.terminal_certificate_required = mode in {"radial20", "precision_fsm"}
    m.precision_fsm_active = mode == "precision_fsm"
    m.precision_terminal_heartbeat_timeout_sec = 0.50
    m.marking_hold_sec = 3.00
    m.spray_required = True
    m.spray_confirmation_timeout_sec = 5.0
    # Odometry freshness is not under test; the fake clock jumps seconds.
    m.odom_timeout_sec = 1.0e9
    m.dummy_arrival_tolerance_m = 0.05
    m.stationary_speed_tolerance_mps = 0.01

    m._state = "RUNNING"
    m._execution_mode = execution
    m._mission_enable = True
    m._emergency_stop = False
    m._mission_run_id = RUN_ID
    m._path_signature = PATH_SIGNATURE
    m._navigation_path = list(GOALS)
    m._path_types = [MissionManager.POINT_MARKING] * len(GOALS)
    m._marking_indices = list(range(len(GOALS)))
    m._semantic_path_indices = list(range(len(GOALS)))
    m._current_path_index = 0
    m._point_status = ["PENDING"] * len(GOALS)
    m._point_accuracy_snapshots = [None] * len(GOALS)
    m._point_results = [None] * len(GOALS)
    m._last_point_result = None
    m._spray_success_keys = set()
    m._spray_failure_keys = {}
    m._active_marking_number = None
    m._last_completed_marking_number = None
    m._auto_continue_until = None
    m._x, m._y = GOALS[0]
    m._last_odom_monotonic = clock.t
    m._last_message = ""
    m._marking_timing = {}
    m._last_marking_timing = {}
    m._arrival_settle_started = None
    m._arrival_settle_elapsed_sec = 0.0
    m._marking_hold_started = None
    m._marking_hold_elapsed_sec = 0.0
    m._spray_request_started = None
    m._rpp_terminal_result = None
    m._rpp_terminal_result_last_rx_monotonic = None
    m._rpp_terminal_certificate = None
    m._rpp_terminal_certificate_last_rx_monotonic = None
    m._pause_reason = None
    m._resume_available = False
    # Raw-GNSS spray gate: OFF here so these tests keep pinning the RPP-only
    # contract; test_raw_gnss_spray_gate.py covers the gate-enabled path.
    m.raw_gnss_spray_gate_enabled = False
    m.raw_gnss_spray_gate_config = RawGnssSprayGateConfig()
    m.raw_gnss_spray_evaluates_rpp_settled_miss = True
    m._raw_gnss_gate_started = None
    m._raw_gnss_gate_marking_number = None
    m._raw_gnss_gate_decision = None

    m._run_auto_stop_if_pending = lambda: False
    m._publish_status = lambda force=False: None
    m._monitor_runtime_rtk = lambda: None
    m._monitor_runtime_px4_control = lambda: None
    m._monitor_pause_recovery = lambda: None
    m._publish_goal = lambda: events.append(("goal", m._current_path_index))
    m._publish_runtime_path = lambda: None
    m._publish_marking_active = lambda value: events.append(("marking_active", value))
    m._disable_motion_preserve_estop = lambda: events.append("disable_motion")
    m._emit_system_event = lambda *a, **k: events.append(("system", a[0]))
    m._survey_truth_for_point = lambda number: {"available": False}
    m._complete_mission = lambda: events.append("mission_complete")

    def enter_error(message):
        events.append(("error", message))
        m._state = "ERROR"

    m._enter_error = enter_error

    def point_event(event, marking_number, path_index, **kw):
        events.append(("point_event", event, marking_number, kw.get("accuracy")))

    m._emit_point_event = point_event

    def missed(*, marking_number, failure_reason):
        events.append(("missed", marking_number, failure_reason))

    m._resolve_accuracy_failure = missed
    m.get_logger = lambda: SimpleNamespace(
        info=lambda msg: events.append(("log", msg)),
        warn=lambda msg: None,
        warning=lambda msg: None,
    )
    m.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(clock.t * 1e9))
    )
    return m, events


def expectation(m):
    # Legacy mode does not compute an expectation; build the same identity.
    required = m.terminal_certificate_required
    m.terminal_certificate_required = True
    try:
        exp = m._current_precision_terminal_expectation()
    finally:
        m.terminal_certificate_required = required
    assert exp is not None
    return exp


def certificate_payload(m, **override):
    exp = expectation(m)
    identity = {
        "mission_run_id": exp.mission_run_id,
        "path_signature": exp.path_signature,
        "raw_path_index": exp.raw_path_index,
        "active_goal_identity": exp.active_goal_identity,
        "goal_instance_id": exp.goal_instance_id,
    }
    payload = {
        "schema_version": PRECISION_TERMINAL_SCHEMA_VERSION,
        "source": PRECISION_TERMINAL_SOURCE,
        "precision_terminal_enabled": True,
        "state": "hold_zero",
        "currently_valid": True,
        "zero_latched": True,
        "precision_certificate_version": 2,
        "precision_pass": True,
        "telemetry_fresh": True,
        **identity,
        "terminal_identity": exp.terminal_identity,
        "terminal_identity_components": dict(identity),
        "certificate": {
            "version": 2,
            "precision_pass": True,
            "terminal_identity": exp.terminal_identity,
            "certified_timestamp_sec": 999.5,
            "stationary_window_sec": 0.50,
            "radial_error_mm": 9.0,
        },
    }
    payload.update(override)
    return payload


def result_payload(m, outcome="CAPTURED", **override):
    exp = expectation(m)
    number = m._marking_indices[m._current_path_index]
    goal = GOALS[m._current_path_index]
    payload = {
        "outcome": outcome,
        "reason": "RADIAL20_CERTIFIED" if outcome == "CAPTURED" else "RPP_MISSED",
        "goal_x": goal[0],
        "goal_y": goal[1],
        "marking_number": number + 1,
        "is_marking": True,
        "measurement_source": "RPP_TERMINAL_RESULT",
        "mission_run_id": exp.mission_run_id,
        "path_signature": exp.path_signature,
        "raw_path_index": exp.raw_path_index,
        "active_goal_identity": exp.active_goal_identity,
        "goal_instance_id": exp.goal_instance_id,
        "terminal_identity": exp.terminal_identity,
        "cross_track_error_mm": 4.0,
        "along_track_error_mm": -8.0,
        "overall_accuracy_mm": 8.9,
        "tolerance_mm": 20.0,
        "within_tolerance": True,
        "timestamp_unix_ns": 123,
    }
    payload.update(override)
    return payload


def deliver(m, *, result=None, certificate=None):
    if result is not None:
        m._rpp_terminal_result_callback(SimpleNamespace(data=json.dumps(result)))
    if certificate is not None:
        m._rpp_terminal_certificate_callback(
            SimpleNamespace(data=json.dumps(certificate))
        )


def sprayed(events):
    return ("marking_active", True) in events


def spray_success(m, point_id):
    m._spray_result_callback(
        SimpleNamespace(
            data=json.dumps(
                {"mission_run_id": RUN_ID, "point_id": point_id, "result": "SUCCESS"}
            )
        )
    )


# ---------------------------------------------------------------- fast path


def test_valid_radial20_capture_sprays_on_the_first_tick_without_hold(clock):
    m, events = build(clock)
    deliver(m, result=result_payload(m), certificate=certificate_payload(m))
    clock.t += 0.05  # one 20 Hz control cycle
    m._control_loop()

    assert sprayed(events)
    assert m._spray_request_started == pytest.approx(clock.t)
    assert m._marking_hold_elapsed_sec == 0.0
    names = [e[1] for e in events if e[0] == "point_event"]
    assert names == ["ACCURACY_ACHIEVED"]
    achieved = events.index(next(e for e in events if e[0] == "point_event"))
    assert achieved < events.index(("marking_active", True))
    timing = m._marking_timing
    assert timing["certificate_valid"] == timing["marking_active_true"] == clock.t
    assert timing["marking_active_true"] - timing["rpp_result_rx"] == pytest.approx(0.05)


def test_stored_accuracy_comes_only_from_rpp_terminal_result(clock):
    m, events = build(clock)
    deliver(m, result=result_payload(m), certificate=certificate_payload(m))
    m._control_loop()
    accuracy = next(e for e in events if e[0] == "point_event")[3]
    assert accuracy["measurement_source"] == "RPP_TERMINAL_RESULT"
    assert accuracy["overall_accuracy_mm"] == 8.9
    assert accuracy["cross_track_error_mm"] == 4.0
    assert accuracy["along_track_error_mm"] == -8.0
    assert accuracy["precision_pass"] is True
    assert accuracy["available"] is True
    assert m._point_accuracy_snapshots[0]["overall_accuracy_mm"] == 8.9


def test_marking_stays_active_through_spray_then_completes_auto(clock):
    m, events = build(clock, execution="AUTO")
    deliver(m, result=result_payload(m), certificate=certificate_payload(m))
    m._control_loop()
    assert sprayed(events)

    # Transaction in progress: no second ACCURACY_ACHIEVED, no completion.
    for _ in range(5):
        clock.t += 0.05
        m._control_loop()
    names = [e[1] for e in events if e[0] == "point_event"]
    assert names == ["ACCURACY_ACHIEVED"]
    assert m._point_status[0] == "ACTIVE"

    spray_success(m, "P0001")
    clock.t += 0.05
    m._control_loop()
    names = [e[1] for e in events if e[0] == "point_event"]
    assert names == ["ACCURACY_ACHIEVED", "COMPLETED"]
    assert m._point_status[0] == "COMPLETED"
    assert m._state == "RUNNING"
    assert m._auto_continue_until == pytest.approx(
        clock.t + MissionManager.AUTO_CONTINUE_DELAY_SEC
    )
    assert m._current_path_index == 1
    # The next point starts with no terminal evidence carried over.
    assert m._rpp_terminal_result is None and m._rpp_terminal_certificate is None
    completed = [e for e in events if e[0] == "point_event" and e[1] == "COMPLETED"]
    assert completed[0][3]["overall_accuracy_mm"] == 8.9
    assert m._last_marking_timing["point_id"] == "P0001"
    assert "spray_success_rx" in m._last_marking_timing

    # AUTO next-goal release still waits the unchanged 1.0 s.
    goals_before = [e for e in events if e[0] == "goal"]
    clock.t += 0.5
    m._control_loop()
    assert [e for e in events if e[0] == "goal"] == goals_before
    clock.t += 0.55
    m._control_loop()
    assert "next_goal_released" in m._last_marking_timing


def test_manual_completion_enters_waiting_for_next(clock):
    m, events = build(clock, execution="MANUAL")
    deliver(m, result=result_payload(m), certificate=certificate_payload(m))
    m._control_loop()
    spray_success(m, "P0001")
    clock.t += 0.05
    m._control_loop()
    assert m._point_status[0] == "COMPLETED"
    assert m._state == "WAITING_FOR_NEXT"
    assert "disable_motion" in events


def test_final_point_completion_completes_mission(clock):
    m, events = build(clock)
    m._point_status = ["COMPLETED", "COMPLETED", "PENDING"]
    m._current_path_index = 2
    m._x, m._y = GOALS[2]
    deliver(m, result=result_payload(m), certificate=certificate_payload(m))
    m._control_loop()
    spray_success(m, "P0003")
    m._control_loop()
    assert m._point_status[2] == "COMPLETED"
    assert "mission_complete" in events


def test_certificate_cannot_be_reused_for_the_next_point(clock):
    m, events = build(clock)
    first_cert = certificate_payload(m)
    first_result = result_payload(m)
    deliver(m, result=first_result, certificate=first_cert)
    m._control_loop()
    spray_success(m, "P0001")
    m._control_loop()
    assert m._current_path_index == 1
    events.clear()

    clock.t += 2.0
    m._auto_continue_until = None
    m._x, m._y = GOALS[1]
    # Replay P0001's certificate (fresh receive time) at P0002.
    deliver(m, certificate=first_cert)
    m._control_loop()
    assert not sprayed(events)
    assert m._point_status[1] == "ACTIVE"


# ------------------------------------------------------------ never sprays


def test_missed_never_sprays(clock):
    m, events = build(clock)
    deliver(
        m,
        result=result_payload(m, outcome="MISSED"),
        certificate=certificate_payload(m),
    )
    m._control_loop()
    assert not sprayed(events)
    assert any(e[0] == "missed" for e in events)


@pytest.mark.parametrize(
    "override",
    [
        {"precision_pass": False},
        {"currently_valid": False},
        {"zero_latched": False},
        {"state": "settle"},
        {"schema_version": 1},
    ],
)
def test_invalid_certificate_never_sprays(clock, override):
    m, events = build(clock)
    deliver(m, result=result_payload(m), certificate=certificate_payload(m, **override))
    m._control_loop()
    assert not sprayed(events)
    assert m._spray_request_started is None
    assert not [e for e in events if e[0] == "point_event"]


def test_stale_certificate_never_sprays(clock):
    m, events = build(clock)
    deliver(m, result=result_payload(m), certificate=certificate_payload(m))
    clock.t += 0.60  # beyond precision_terminal_heartbeat_timeout_sec
    m._control_loop()
    assert not sprayed(events)
    assert m._spray_request_started is None


@pytest.mark.parametrize(
    "field,value",
    [
        ("mission_run_id", "run-other"),
        ("path_signature", "b" * 64),
        ("raw_path_index", 1),
        ("active_goal_identity", "P0002"),
        ("goal_instance_id", "instance-other"),
        ("terminal_identity", "RUN:x|PATH:y"),
    ],
)
def test_wrong_certificate_identity_never_sprays(clock, field, value):
    m, events = build(clock)
    deliver(m, result=result_payload(m), certificate=certificate_payload(m, **{field: value}))
    m._control_loop()
    assert not sprayed(events)


@pytest.mark.parametrize(
    "field,value",
    [
        ("mission_run_id", "run-other"),
        ("path_signature", "b" * 64),
        ("raw_path_index", 2),
        ("goal_instance_id", "instance-other"),
        ("terminal_identity", "RUN:x|PATH:y"),
    ],
)
def test_wrong_result_identity_never_sprays(clock, field, value):
    m, events = build(clock)
    deliver(m, result=result_payload(m, **{field: value}), certificate=certificate_payload(m))
    m._control_loop()
    assert not sprayed(events)


def test_result_for_another_goal_position_is_ignored(clock):
    m, events = build(clock)
    deliver(
        m,
        result=result_payload(m, goal_x=GOALS[1][0], goal_y=GOALS[1][1]),
        certificate=certificate_payload(m),
    )
    m._control_loop()
    assert m._rpp_terminal_result is None
    assert not sprayed(events)


def test_incomplete_accuracy_never_sprays(clock):
    m, events = build(clock)
    deliver(
        m,
        result=result_payload(m, along_track_error_mm=None),
        certificate=certificate_payload(m),
    )
    m._control_loop()
    assert not sprayed(events)
    assert not [e for e in events if e[0] == "point_event"]


@pytest.mark.parametrize("estop,enable", [(True, False), (False, False)])
def test_estop_or_mission_disable_never_requests_spray(clock, estop, enable):
    m, events = build(clock)
    m._emergency_stop = estop
    m._mission_enable = enable
    deliver(m, result=result_payload(m), certificate=certificate_payload(m))
    m._control_loop()
    assert not sprayed(events)


def test_spray_controller_press_is_gated_on_estop_and_mission_enable():
    source = (
        Path(__file__).resolve().parents[2]
        / "spray_controller"
        / "spray_controller"
        / "spray_controller_node.py"
    ).read_text(encoding="utf-8")
    gate = source[source.index("def _spray_gate_ok"):]
    gate = gate[: gate.index("\n    def ")]
    assert "if not self.mission_enable or self.emergency_stop:" in gate
    press = source.index('kind="PRESS"')
    stable = source.rindex("if now - self.marking_active_since < self.pre_spray_stable_sec:", 0, press)
    gate_check = source.rindex("self._spray_gate_ok(now)", 0, stable)
    assert gate_check < stable < press


# ----------------------------------------------------- other modes unchanged


def test_precision_fsm_keeps_the_marking_hold(clock):
    m, events = build(clock, mode="precision_fsm")
    deliver(m, result=result_payload(m), certificate=certificate_payload(m))
    m._control_loop()
    assert not sprayed(events)
    assert "starting 3.0s pre-mark hold" in m._last_message
    for _ in range(58):  # 2.9 s, certificate kept fresh
        clock.t += 0.05
        deliver(m, certificate=certificate_payload(m))
        m._control_loop()
    assert not sprayed(events)
    clock.t += 0.15
    deliver(m, certificate=certificate_payload(m))
    m._control_loop()
    assert sprayed(events)


def test_legacy_mode_keeps_the_marking_hold(clock):
    m, events = build(clock, mode="legacy")
    deliver(m, result=result_payload(m))
    m._control_loop()
    assert not sprayed(events)
    assert m._marking_hold_started == clock.t
    clock.t += 2.9
    m._control_loop()
    assert not sprayed(events)
    clock.t += 0.15
    m._control_loop()
    assert sprayed(events)


def test_effective_hold_is_zero_only_for_radial20(clock):
    assert build(clock, mode="radial20")[0]._effective_marking_hold_sec() == 0.0
    assert build(clock, mode="precision_fsm")[0]._effective_marking_hold_sec() == 3.0
    assert build(clock, mode="legacy")[0]._effective_marking_hold_sec() == 3.0


# ------------------------------------------------------ unchanged constants


def test_patch_does_not_change_spray_stop_or_auto_constants():
    root = Path(__file__).resolve().parents[2]
    launch = (root / "rover_bringup" / "launch" / "rover.launch.py").read_text(
        encoding="utf-8"
    )
    assert '"spray_duration_sec": 0.50' in launch
    assert '"pre_spray_stable_sec": 0.25' in launch
    assert '"radial_stop_stationary_window_sec": 0.50' in launch
    assert '"marking_hold_sec": 3.00' in launch
    assert 'TERMINAL_STOP_MODE = "radial20"' in launch
    assert MissionManager.AUTO_CONTINUE_DELAY_SEC == 1.0
    spray = (
        root / "spray_controller" / "spray_controller" / "spray_controller_node.py"
    ).read_text(encoding="utf-8")
    assert 'self.declare_parameter("spray_duration_sec", 0.50)' in spray
    assert 'self.declare_parameter("pre_spray_stable_sec", 0.25)' in spray
    regulator = (
        root / "rpp_controller" / "rpp_controller" / "terminal_stop_regulator.py"
    ).read_text(encoding="utf-8")
    assert "stationary_window_sec: float = 0.50" in regulator
