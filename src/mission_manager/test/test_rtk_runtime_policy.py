"""Runtime RTK policy (operator-approved 2026-09-26).

FLOAT stops the rover and auto-resumes once FIXED holds 3 s; DGPS / 3D /
lower fixes, stale GPS status, or FLOAT lasting 120 s need a manual Resume;
a spray press already in progress finishes before the FLOAT pause.
"""

from __future__ import annotations

import sys
import threading
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from mission_manager.rtk_runtime_policy import (  # noqa: E402
    FloatPauseAction,
    RtkRunningAction,
    RtkRuntimeConfig,
    RtkRuntimePolicy,
)

# Reuse the lifecycle harness's ROS stubs and module import.
import test_lifecycle_command_behavior as harness  # noqa: E402

node_module = harness.node_module
MissionManager = harness.MissionManager


# ----------------------------------------------------------- pure policy
def policy():
    return RtkRuntimePolicy(RtkRuntimeConfig())


def test_config_validation():
    with pytest.raises(ValueError):
        RtkRuntimeConfig(fixed_hold_sec=0.0)
    with pytest.raises(ValueError):
        RtkRuntimeConfig(fixed_hold_sec=5.0, float_manual_after_sec=4.0)


@pytest.mark.parametrize(
    "fix,age,spray,expected",
    [
        (6, 0.1, False, RtkRunningAction.CONTINUE),
        (6, 0.1, True, RtkRunningAction.CONTINUE),
        (5, 0.1, False, RtkRunningAction.PAUSE_FLOAT),
        (5, 0.1, True, RtkRunningAction.DEFER_FLOAT_UNTIL_SPRAY_DONE),
        (4, 0.1, False, RtkRunningAction.PAUSE_LOST),   # DGPS
        (3, 0.1, False, RtkRunningAction.PAUSE_LOST),   # 3D
        (4, 0.1, True, RtkRunningAction.PAUSE_LOST),    # never deferred
        (0, 0.1, False, RtkRunningAction.PAUSE_LOST),
        (8, 0.1, False, RtkRunningAction.PAUSE_LOST),   # PPP is not FIXED
        (6, 11.0, False, RtkRunningAction.PAUSE_LOST),  # stale
        (5, float("inf"), False, RtkRunningAction.PAUSE_LOST),
    ],
)
def test_running_decisions(fix, age, spray, expected):
    assert policy().running(0.0, fix, age, spray) is expected


def test_fixed_must_hold_three_seconds_and_float_restarts_the_hold():
    p = policy()
    assert p.running(0.0, 5, 0.1, False) is RtkRunningAction.PAUSE_FLOAT
    assert p.float_paused(1.0, 6, 0.1) is FloatPauseAction.HOLD
    assert p.float_paused(3.9, 6, 0.1) is FloatPauseAction.HOLD
    assert p.float_paused(4.0, 5, 0.1) is FloatPauseAction.HOLD   # flicker
    assert p.fixed_held_sec(4.0) == 0.0
    assert p.float_paused(5.0, 6, 0.1) is FloatPauseAction.HOLD
    assert p.float_paused(7.9, 6, 0.1) is FloatPauseAction.HOLD
    assert p.float_paused(8.0, 6, 0.1) is FloatPauseAction.AUTO_RESUME


@pytest.mark.parametrize("fix,age", [(4, 0.1), (3, 0.1), (2, 0.1), (0, 0.1), (6, 11.0)])
def test_float_pause_escalates_on_worse_fix_or_stale(fix, age):
    p = policy()
    p.running(0.0, 5, 0.1, False)
    assert p.float_paused(1.0, fix, age) is FloatPauseAction.ESCALATE_LOST


def test_float_pause_times_out_to_manual_after_120_s():
    p = policy()
    p.running(0.0, 5, 0.1, False)
    assert p.float_paused(119.9, 5, 0.1) is FloatPauseAction.HOLD
    assert p.float_paused(120.0, 5, 0.1) is FloatPauseAction.ESCALATE_TIMEOUT


def test_fixed_hold_completing_at_the_timeout_still_resumes():
    p = policy()
    p.running(0.0, 5, 0.1, False)
    p.float_paused(117.0, 6, 0.1)
    assert p.float_paused(120.0, 6, 0.1) is FloatPauseAction.AUTO_RESUME


# ------------------------------------------------------------ node harness
class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock(monkeypatch):
    c = Clock()
    monkeypatch.setattr(node_module.time, "monotonic", c)
    return c


class Logger:
    def __init__(self, sink):
        self.sink = sink

    def warn(self, message):
        self.sink.append(("log", message))

    error = info = warn


def build(clock, *, state="RUNNING", fix=6, mode="OFFBOARD", armed=True):
    manager, events = harness.build_manager(state=state, mode=mode, armed=armed)
    manager._rtk_policy = RtkRuntimePolicy(
        RtkRuntimeConfig(
            fixed_hold_sec=MissionManager.RTK_FLOAT_FIXED_HOLD_SEC,
            float_manual_after_sec=MissionManager.RTK_FLOAT_MANUAL_AFTER_SEC,
            gps_stale_sec=MissionManager.GPS_FIX_STALE_SEC,
        )
    )
    manager._rtk_float_warning_active = False
    manager._gps_fix_type = fix
    manager._last_gps_fix_rx_monotonic = clock.t
    manager._last_fcu_rx_monotonic = clock.t
    manager._spray_request_started = None
    manager._rtk_healthy = True
    manager._last_rtk_health_rx_monotonic = clock.t
    manager._last_rtk_age_rx_monotonic = clock.t
    manager._rtk_correction_age_sec = 1.0
    manager._pause_reason = None if state == "RUNNING" else manager._pause_reason
    manager._resume_available = False if state == "RUNNING" else manager._resume_available
    manager._emit_system_event = lambda name, msg: events.append(("event", name, msg))
    manager.get_logger = lambda: Logger(events)
    return manager, events


def fix(manager, clock, fix_type, advance=0.0):
    clock.t += advance
    manager._gps_fix_type = fix_type
    manager._last_gps_fix_rx_monotonic = clock.t
    manager._last_fcu_rx_monotonic = clock.t


def tick(manager):
    MissionManager._monitor_runtime_rtk(manager)
    MissionManager._monitor_pause_recovery(manager)


def event_names(events):
    return [e[1] for e in events if isinstance(e, tuple) and e[0] == "event"]


def test_node_float_stops_the_rover_immediately(clock):
    m, events = build(clock)
    fix(m, clock, 5)
    tick(m)
    assert (m._state, m._pause_reason, m._resume_available) == ("PAUSED", "RTK_FLOAT", False)
    assert "disable_motion" in events and "reset_arrival" in events
    assert not m._mission_enable
    assert event_names(events) == ["RTK_FLOAT"]


def test_node_auto_resumes_after_three_seconds_of_fixed_without_px4_calls(clock):
    m, events = build(clock)
    fix(m, clock, 5)
    tick(m)
    fix(m, clock, 6, advance=0.5)
    tick(m)
    fix(m, clock, 6, advance=2.9)       # 2.9 s held
    tick(m)
    assert m._state == "PAUSED"
    fix(m, clock, 6, advance=0.1)       # 3.0 s held
    tick(m)
    assert (m._state, m._pause_reason) == ("RUNNING", None)
    assert m._mission_enable and "enable_motion" in events
    assert harness.px4_calls(events) == []
    assert event_names(events)[-1] == "RTK_RECOVERED"


def test_node_flicker_restarts_the_hold(clock):
    m, _ = build(clock)
    fix(m, clock, 5)
    tick(m)
    fix(m, clock, 6, advance=1.0)
    tick(m)
    fix(m, clock, 6, advance=2.5)
    tick(m)
    fix(m, clock, 5, advance=0.2)       # back to FLOAT before 3 s
    tick(m)
    fix(m, clock, 6, advance=0.2)
    tick(m)
    fix(m, clock, 6, advance=2.9)
    tick(m)
    assert m._state == "PAUSED"
    fix(m, clock, 6, advance=0.1)
    tick(m)
    assert m._state == "RUNNING"


@pytest.mark.parametrize("worse", [4, 3, 2, 0])
def test_node_worse_fix_while_running_needs_manual_resume(clock, worse):
    m, events = build(clock)
    fix(m, clock, worse)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "RTK_LOST")
    assert event_names(events) == ["RTK_PAUSED"]
    fix(m, clock, 6, advance=10.0)
    tick(m)
    assert m._state == "PAUSED"          # never auto-resumes
    assert "enable_motion" not in events


@pytest.mark.parametrize("worse", [4, 3])
def test_node_float_pause_escalates_to_manual_on_dgps_or_3d(clock, worse):
    m, events = build(clock)
    fix(m, clock, 5)
    tick(m)
    fix(m, clock, worse, advance=1.0)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "RTK_LOST")
    fix(m, clock, 6, advance=1.0)
    tick(m)
    fix(m, clock, 6, advance=10.0)
    tick(m)
    assert m._state == "PAUSED"
    assert "enable_motion" not in events
    # FIXED back: the existing manual path only OFFERS Resume.
    assert m._resume_available is True
    assert event_names(events) == ["RTK_FLOAT", "RTK_PAUSED", "RTK_RECOVERED"]
    assert "ready to resume" in [e for e in events if isinstance(e, tuple)
                                 and e[0] == "event"][-1][2]


def test_node_stale_gps_while_float_paused_escalates(clock):
    m, _ = build(clock)
    fix(m, clock, 5)
    tick(m)
    clock.t += 11.0                     # no GPSRAW for 11 s
    m._last_fcu_rx_monotonic = clock.t
    tick(m)
    assert m._pause_reason == "RTK_LOST"


def test_node_float_timeout_needs_manual_resume(clock):
    m, events = build(clock)
    fix(m, clock, 5)
    tick(m)
    fix(m, clock, 5, advance=120.0)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "RTK_FLOAT_TIMEOUT")
    fix(m, clock, 6, advance=1.0)
    tick(m)
    fix(m, clock, 6, advance=5.0)
    tick(m)
    assert m._state == "PAUSED" and "enable_motion" not in events


@pytest.mark.parametrize("mode,armed", [("MANUAL", True), ("OFFBOARD", False)])
def test_node_auto_resume_refused_if_px4_left_offboard(clock, mode, armed):
    m, events = build(clock)
    fix(m, clock, 5)
    tick(m)
    m._px4_mode, m._px4_armed = mode, armed
    fix(m, clock, 6, advance=1.0)
    tick(m)
    fix(m, clock, 6, advance=3.0)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "RTK_FLOAT_MANUAL")
    assert harness.px4_calls(events) == []
    assert "enable_motion" not in events


def test_node_auto_resume_refused_under_estop(clock):
    m, events = build(clock)
    fix(m, clock, 5)
    tick(m)
    m._emergency_stop = True
    fix(m, clock, 6, advance=1.0)
    tick(m)
    fix(m, clock, 6, advance=3.0)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "ESTOP")
    assert "enable_motion" not in events


def test_node_float_during_spray_finishes_the_press_then_pauses(clock):
    m, events = build(clock)
    m._spray_request_started = clock.t
    fix(m, clock, 5)
    tick(m)
    assert m._state == "RUNNING"         # press continues
    assert "disable_motion" not in events
    fix(m, clock, 5, advance=0.5)
    tick(m)
    assert m._state == "RUNNING"
    m._spray_request_started = None      # point completed
    fix(m, clock, 5, advance=0.05)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "RTK_FLOAT")


def test_node_float_clearing_during_spray_never_pauses(clock):
    m, events = build(clock)
    m._spray_request_started = clock.t
    fix(m, clock, 5)
    tick(m)
    fix(m, clock, 6, advance=0.3)
    tick(m)
    m._spray_request_started = None
    fix(m, clock, 6, advance=0.1)
    tick(m)
    assert m._state == "RUNNING" and "disable_motion" not in events


def test_node_dgps_during_spray_is_not_deferred(clock):
    m, _ = build(clock)
    m._spray_request_started = clock.t
    fix(m, clock, 4)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "RTK_LOST")


def test_node_float_pause_never_advertises_manual_resume(clock):
    m, _ = build(clock)
    fix(m, clock, 5)
    tick(m)
    fix(m, clock, 6, advance=1.0)
    tick(m)
    assert m._resume_available is False
