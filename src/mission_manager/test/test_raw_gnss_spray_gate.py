"""Raw-GNSS spray gate: raw RTK accuracy <= 30 mm is the ONLY spray/COMPLETE gate.

Unit tests for the pure gate, then the real ``_control_loop`` marking branch
driven through the certified-fast-path harness with the gate ENABLED and the
real survey-target / GNSS-history plumbing.
"""

from __future__ import annotations

import collections
import json
import math
import sys
from pathlib import Path

import pytest

TEST_DIR = Path(__file__).resolve().parent
if str(TEST_DIR) not in sys.path:
    sys.path.insert(0, str(TEST_DIR))

import test_marking_certified_fast_path as fast  # noqa: E402

from mission_manager.raw_gnss_spray_gate import (  # noqa: E402
    GATE_FAIL,
    GATE_PASS,
    GATE_WAITING,
    RawGnssSprayGateConfig,
    evaluate_raw_gnss_spray_gate,
)
from mission_manager.survey_truth import (  # noqa: E402
    GnssFix,
    SurveyTarget,
    metres_per_degree,
)

clock = fast.clock  # re-export the fixture

BASE_LAT = 13.0
BASE_LON = 80.2
# Surveyed targets 1 m apart heading east, matching fast.GOALS.
TARGETS = [
    SurveyTarget(
        point_id=f"P{i+1:04d}",
        point_index=i,
        latitude_deg=BASE_LAT,
        longitude_deg=BASE_LON + i / metres_per_degree(BASE_LAT)[1],
    )
    for i in range(len(fast.GOALS))
]


def offset(target: SurveyTarget, north_m: float, east_m: float) -> tuple[float, float]:
    per_n, per_e = metres_per_degree(target.latitude_deg)
    return target.latitude_deg + north_m / per_n, target.longitude_deg + east_m / per_e


def fix(t: float, target: SurveyTarget, north_m=0.0, east_m=0.0, fix_type=6) -> GnssFix:
    lat, lon = offset(target, north_m, east_m)
    return GnssFix(
        monotonic_sec=t,
        latitude_deg=lat,
        longitude_deg=lon,
        fix_type=fix_type,
        satellites=18,
        horizontal_accuracy_m=0.015,
    )


def fixes_at(target, start, end, north_m=0.0, east_m=0.0, fix_type=6, rate=10.0):
    out = []
    t = start
    while t <= end + 1e-9:
        out.append(fix(t, target, north_m, east_m, fix_type))
        t += 1.0 / rate
    return out


CONFIG = RawGnssSprayGateConfig()


def gate(fixes, *, now, stop=100.0, target=TARGETS[1], **kw):
    return evaluate_raw_gnss_spray_gate(
        target=target,
        previous_target=TARGETS[0],
        fixes=fixes,
        stop_monotonic_sec=stop,
        now_monotonic_sec=now,
        config=CONFIG,
        **kw,
    )


# ------------------------------------------------------------ pure gate


def test_defaults_are_the_30mm_contract():
    assert CONFIG.tolerance_m == 0.030


def test_waits_for_the_post_stop_window():
    d = gate(fixes_at(TARGETS[1], 100.0, 100.5), now=100.5)
    assert d.status == GATE_WAITING and not d.final


@pytest.mark.parametrize(
    "east_m,status",
    [(0.010, GATE_PASS), (0.029, GATE_PASS), (0.031, GATE_FAIL), (0.080, GATE_FAIL)],
)
def test_radial_decides_pass_or_fail(east_m, status):
    d = gate(fixes_at(TARGETS[1], 100.0, 101.0, east_m=east_m), now=101.0)
    assert d.status == status
    assert d.radial_error_mm == pytest.approx(east_m * 1000.0, abs=0.5)
    assert d.survey["available"] is True
    if status == GATE_FAIL:
        assert d.reason == "RAW_GNSS_RADIAL_EXCEEDS_TOLERANCE"


def test_radial_uses_both_axes():
    # 22 mm north + 22 mm east = 31.1 mm radial -> FAIL although each axis < 30.
    d = gate(fixes_at(TARGETS[1], 100.0, 101.0, north_m=0.022, east_m=0.022), now=101.0)
    assert d.status == GATE_FAIL


def test_pre_stop_braking_fixes_are_ignored():
    # 0.5 m away before the stop, perfect after it.
    history = fixes_at(TARGETS[1], 98.0, 99.9, east_m=-0.5) + fixes_at(
        TARGETS[1], 100.0, 101.0
    )
    assert gate(history, now=101.0).status == GATE_PASS


def test_float_fix_is_never_accepted_and_fails_at_timeout():
    history = fixes_at(TARGETS[1], 100.0, 103.0, fix_type=5)
    assert gate(history, now=101.0).status == GATE_WAITING
    d = gate(history, now=103.0)
    assert d.status == GATE_FAIL
    assert d.reason == "RAW_GNSS_UNAVAILABLE:GNSS_NOT_RTK_FIXED"


def test_no_gnss_fails_at_timeout():
    assert gate([], now=102.9).status == GATE_WAITING
    d = gate([], now=103.0)
    assert d.status == GATE_FAIL and d.reason.startswith("RAW_GNSS_UNAVAILABLE:")


def test_moving_cluster_is_not_a_stop():
    history = [fix(100.0 + 0.1 * i, TARGETS[1], east_m=0.01 * i) for i in range(11)]
    d = gate(history, now=101.0)
    assert d.status == GATE_WAITING
    assert d.survey["reason"] == "GNSS_WINDOW_NOT_STATIONARY"


def test_forced_unavailable_reason_fails_at_timeout():
    history = fixes_at(TARGETS[1], 100.0, 103.0)
    d = gate(history, now=103.0, unavailable_reason="SURVEY_TARGETS_NOT_FOR_CURRENT_MISSION")
    assert d.status == GATE_FAIL
    assert "SURVEY_TARGETS_NOT_FOR_CURRENT_MISSION" in d.reason


@pytest.mark.parametrize(
    "kw",
    [
        {"tolerance_m": 0.0},
        {"tolerance_m": math.nan},
        {"minimum_samples": 0},
        {"window_sec": 2.0, "timeout_sec": 1.0},
    ],
)
def test_invalid_config_is_rejected(kw):
    with pytest.raises(ValueError):
        RawGnssSprayGateConfig(**kw)


# ------------------------------------------------------ node integration


def build_gated(clock, **kw):
    m, events = fast.build(clock, **kw)
    m.raw_gnss_spray_gate_enabled = True
    m.survey_truth_enabled = True
    m.survey_truth_window_sec = 2.0
    m.survey_truth_minimum_samples = 3
    m.survey_truth_max_scatter_m = 0.060
    m.marking_tolerance_m = 0.02
    m._yaw = 0.0
    m._gnss_history = collections.deque(maxlen=400)
    m._survey_targets = list(TARGETS)
    m._survey_targets_signature = fast.PATH_SIGNATURE
    m._survey_targets_mode = "gps"
    m._point_accuracy_snapshots = [None] * len(fast.GOALS)
    # Use the real survey-truth path, not the harness stub.
    del m._survey_truth_for_point
    failures = []

    real_resolve = type(m)._resolve_accuracy_failure

    def resolve(*, marking_number, failure_reason, failure_source="RPP_MISSED"):
        failures.append((marking_number, failure_source, failure_reason))
        events.append(("missed", marking_number, failure_reason))

    m._resolve_accuracy_failure = resolve
    m._real_resolve = lambda **k: real_resolve(m, **k)
    return m, events, failures


def feed(m, clock, seconds, *, north_m=0.0, east_m=0.0, fix_type=6, cert=True, result=None):
    """Advance 20 Hz control ticks with 10 Hz GNSS and a fresh certificate."""
    target = TARGETS[m._marking_indices[m._current_path_index]]
    steps = int(round(seconds / 0.05))
    for i in range(steps):
        clock.t += 0.05
        if i % 2 == 0:
            m._gnss_history.append(fix(clock.t, target, north_m, east_m, fix_type))
        if cert:
            fast.deliver(m, certificate=fast.certificate_payload(m))
        m._control_loop()


def point_events(events):
    return [e for e in events if e[0] == "point_event"]


def test_rpp_capture_alone_no_longer_sprays(clock):
    m, events, _ = build_gated(clock)
    fast.deliver(m, result=fast.result_payload(m), certificate=fast.certificate_payload(m))
    m._control_loop()
    assert not fast.sprayed(events)
    assert "raw GNSS accuracy check" in m._last_message
    feed(m, clock, 0.9)
    assert not fast.sprayed(events)


def test_raw_under_30mm_sprays_and_completes(clock):
    m, events, failures = build_gated(clock)
    fast.deliver(m, result=fast.result_payload(m), certificate=fast.certificate_payload(m))
    m._control_loop()
    feed(m, clock, 1.1, east_m=0.018)
    assert fast.sprayed(events)
    assert not failures
    achieved = point_events(events)[0]
    assert achieved[1] == "ACCURACY_ACHIEVED"
    survey = achieved[3]["survey"]
    assert survey["measurement_source"] == "RAW_GNSS_SURVEY"
    assert survey["radial_error_mm"] == pytest.approx(18.0, abs=0.5)
    assert survey["spray_gate"]["pass"] is True
    assert survey["spray_gate"]["tolerance_mm"] == pytest.approx(30.0)
    # RPP numbers are still reported (debug), unchanged.
    assert achieved[3]["measurement_source"] == "RPP_TERMINAL_RESULT"
    assert achieved[3]["overall_accuracy_mm"] == 8.9

    status = m._raw_gnss_gate_status_payload()
    assert status["pass"] is True and status["point_id"] == "P0001"
    assert status["mission_run_id"] == fast.RUN_ID

    fast.spray_success(m, "P0001")
    clock.t += 0.05
    m._control_loop()
    assert m._point_status[0] == "COMPLETED"
    assert m._raw_gnss_gate_decision is None  # cleared for the next point


def test_raw_over_30mm_fails_without_spray_even_though_rpp_captured(clock):
    m, events, failures = build_gated(clock)
    fast.deliver(m, result=fast.result_payload(m), certificate=fast.certificate_payload(m))
    m._control_loop()
    feed(m, clock, 1.1, north_m=0.035)
    assert not fast.sprayed(events)
    assert failures and failures[0][1] == "RAW_GNSS"
    assert "RAW_GNSS_RADIAL_EXCEEDS_TOLERANCE" in failures[0][2]
    assert not [e for e in point_events(events) if e[1] == "ACCURACY_ACHIEVED"]


def test_rtk_float_fails_without_spray(clock):
    m, events, failures = build_gated(clock)
    fast.deliver(m, result=fast.result_payload(m), certificate=fast.certificate_payload(m))
    m._control_loop()
    feed(m, clock, 3.2, fix_type=5)
    assert not fast.sprayed(events)
    assert failures and "GNSS_NOT_RTK_FIXED" in failures[0][2]


def test_survey_targets_from_another_mission_never_spray(clock):
    m, events, failures = build_gated(clock)
    m._survey_targets_signature = "b" * 64
    fast.deliver(m, result=fast.result_payload(m), certificate=fast.certificate_payload(m))
    m._control_loop()
    feed(m, clock, 3.2)
    assert not fast.sprayed(events)
    assert "SURVEY_TARGETS_NOT_FOR_CURRENT_MISSION" in failures[0][2]


def test_local_coordinate_mission_never_sprays(clock):
    m, events, failures = build_gated(clock)
    m._survey_targets = []
    m._survey_targets_mode = "local"
    fast.deliver(m, result=fast.result_payload(m), certificate=fast.certificate_payload(m))
    m._control_loop()
    feed(m, clock, 3.2)
    assert not fast.sprayed(events)
    assert "MISSION_NOT_IN_GPS_COORDINATES" in failures[0][2]


def test_rpp_settled_miss_is_decided_by_raw_gnss(clock):
    m, events, failures = build_gated(clock)
    fast.deliver(
        m,
        result=fast.result_payload(
            m,
            outcome="MISSED",
            reason="RADIAL20_SETTLED_OUTSIDE_TOLERANCE",
            overall_accuracy_mm=24.0,
        ),
    )
    m._control_loop()
    assert not failures
    feed(m, clock, 1.1, east_m=0.012, cert=False)
    assert fast.sprayed(events)
    assert not failures
    achieved = point_events(events)[0][3]
    assert achieved["rpp_outcome"] == "MISSED"
    assert achieved["survey"]["spray_gate"]["pass"] is True

    fast.spray_success(m, "P0001")
    clock.t += 0.05
    m._control_loop()
    assert m._point_status[0] == "COMPLETED"


def test_rpp_settled_miss_with_bad_raw_fails(clock):
    m, events, failures = build_gated(clock)
    fast.deliver(
        m,
        result=fast.result_payload(
            m, outcome="MISSED", reason="RADIAL20_SETTLED_OUTSIDE_TOLERANCE"
        ),
    )
    m._control_loop()
    feed(m, clock, 1.1, east_m=0.045, cert=False)
    assert not fast.sprayed(events)
    assert failures[0][1] == "RAW_GNSS"


@pytest.mark.parametrize(
    "reason",
    ["RADIAL20_TERMINAL_TIMEOUT", "RADIAL20_STALE_TELEMETRY", "RADIAL20_SETTLE_TIMEOUT"],
)
def test_unsettled_rpp_miss_still_fails_directly(clock, reason):
    m, events, failures = build_gated(clock)
    fast.deliver(m, result=fast.result_payload(m, outcome="MISSED", reason=reason))
    m._control_loop()
    assert not fast.sprayed(events)
    assert failures == [(0, "RPP_MISSED", reason)]


def test_settled_miss_evaluation_can_be_disabled(clock):
    m, events, failures = build_gated(clock)
    m.raw_gnss_spray_evaluates_rpp_settled_miss = False
    fast.deliver(
        m,
        result=fast.result_payload(
            m, outcome="MISSED", reason="RADIAL20_SETTLED_OUTSIDE_TOLERANCE"
        ),
    )
    m._control_loop()
    assert failures and failures[0][1] == "RPP_MISSED"


def test_invalid_certificate_still_blocks_before_raw_gate(clock):
    m, events, failures = build_gated(clock)
    fast.deliver(
        m,
        result=fast.result_payload(m),
        certificate=fast.certificate_payload(m, precision_pass=False),
    )
    m._control_loop()
    feed(m, clock, 1.5, cert=False)
    assert not fast.sprayed(events)
    assert m._raw_gnss_gate_started is None


def test_certificate_loss_restarts_the_raw_window(clock):
    m, events, _ = build_gated(clock)
    fast.deliver(m, result=fast.result_payload(m), certificate=fast.certificate_payload(m))
    m._control_loop()
    first_start = m._raw_gnss_gate_started
    feed(m, clock, 0.6)
    clock.t += 0.6  # heartbeat goes stale -> hold and gate reset
    m._control_loop()
    assert m._raw_gnss_gate_started is None
    feed(m, clock, 0.5)
    assert m._raw_gnss_gate_started > first_start
    assert not fast.sprayed(events)
    feed(m, clock, 0.6)
    assert fast.sprayed(events)


def test_failure_report_carries_the_deciding_raw_measurement(clock):
    m, events, failures = build_gated(clock)
    # Use the real failure resolver this time.
    m._resolve_accuracy_failure = m._real_resolve
    m._store_point_result = lambda **kw: dict(kw)
    fast.deliver(m, result=fast.result_payload(m), certificate=fast.certificate_payload(m))
    m._control_loop()
    feed(m, clock, 1.1, east_m=0.050)
    failed = [e for e in point_events(events) if e[1] == "ACCURACY_FAILED"]
    assert len(failed) == 1
    survey = failed[0][3]["survey"]
    assert survey["spray_gate"]["status"] == "FAIL"
    assert survey["radial_error_mm"] == pytest.approx(50.0, abs=0.5)
    assert m._point_status[0] == "FAILED"
    assert not fast.sprayed(events)
    assert "FAILED from raw GNSS accuracy" in m._last_message


def test_status_payload_is_idle_between_points(clock):
    m, _, _ = build_gated(clock)
    status = m._raw_gnss_gate_status_payload()
    assert status["status"] == "IDLE" and status["pass"] is False
    json.dumps(status)


def test_launch_enables_the_raw_gate_at_30mm():
    launch = (
        Path(__file__).resolve().parents[2] / "rover_bringup" / "launch" / "rover.launch.py"
    ).read_text(encoding="utf-8")
    assert '"raw_gnss_spray_gate_enabled": True' in launch
    assert '"raw_gnss_spray_tolerance_m": 0.030' in launch
    assert '"require_raw_gnss_spray_gate": True' in launch
    assert '"raw_gnss_max_radial_mm": 30.0' in launch


def test_spray_controller_press_requires_the_raw_gate():
    source = (
        Path(__file__).resolve().parents[2]
        / "spray_controller"
        / "spray_controller"
        / "spray_controller_node.py"
    ).read_text(encoding="utf-8")
    gate_fn = source[source.index("def _spray_gate_ok"):]
    gate_fn = gate_fn[: gate_fn.index("\n    def ")]
    assert "self._raw_gnss_gate_ok()" in gate_fn


# ------------------------------------------- legacy terminal_stop_mode


def start_legacy_hold(m, clock):
    fast.deliver(m, result=fast.result_payload(m))
    m._control_loop()
    assert m._marking_hold_started == clock.t
    # The legacy 3.0 s hold runs before the raw gate starts its window.
    feed(m, clock, 3.05, cert=False)
    assert m._raw_gnss_gate_started is not None
    assert not fast.sprayed(m._test_events)


def build_legacy(clock):
    m, events, failures = build_gated(clock, mode="legacy")
    m._test_events = events
    return m, events, failures


def test_legacy_mode_raw_pass_sprays_with_the_deciding_measurement(clock):
    m, events, failures = build_legacy(clock)
    start_legacy_hold(m, clock)
    feed(m, clock, 1.1, east_m=0.015, cert=False)
    assert fast.sprayed(events)
    assert not failures
    achieved = [e for e in point_events(events) if e[1] == "ACCURACY_ACHIEVED"]
    assert len(achieved) == 1
    # Phase A cached a report-only survey block; the PASS must replace it.
    survey = achieved[0][3]["survey"]
    assert survey["spray_gate"]["pass"] is True
    assert survey["radial_error_mm"] == pytest.approx(15.0, abs=0.5)


def test_legacy_mode_raw_fail_never_sprays(clock):
    m, events, failures = build_legacy(clock)
    start_legacy_hold(m, clock)
    feed(m, clock, 1.1, north_m=0.040, cert=False)
    assert not fast.sprayed(events)
    assert failures and failures[0][1] == "RAW_GNSS"
    assert "RAW_GNSS_RADIAL_EXCEEDS_TOLERANCE" in failures[0][2]


def test_legacy_fail_report_carries_the_deciding_raw_measurement(clock):
    m, events, _ = build_legacy(clock)
    m._resolve_accuracy_failure = m._real_resolve
    m._store_point_result = lambda **kw: dict(kw)
    start_legacy_hold(m, clock)
    feed(m, clock, 1.1, east_m=0.050, cert=False)
    failed = [e for e in point_events(events) if e[1] == "ACCURACY_FAILED"]
    assert len(failed) == 1
    assert failed[0][3]["survey"]["spray_gate"]["status"] == "FAIL"
    assert m._point_status[0] == "FAILED"


def test_missing_mission_signature_never_sprays(clock):
    # Survey targets can only be matched to a mission with a known signature.
    m, events, failures = build_legacy(clock)
    fast.deliver(m, result=fast.result_payload(m))
    m._control_loop()
    # Signature lost after RPP CAPTURED; the matching targets must not count.
    m._survey_targets_signature = None
    m._path_signature = None
    feed(m, clock, 3.05 + 3.2, cert=False)
    assert not fast.sprayed(events)
    assert "SURVEY_TARGETS_NOT_FOR_CURRENT_MISSION" in failures[0][2]
