#!/usr/bin/env python3
"""Replay field bags through the patched marking-arrival gate.

Pass/fail is decided from events.jsonl + telemetry.csv without ROS.
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "mission_manager"))

from mission_manager.marking_arrival_policy import (  # noqa: E402
    FAIL_NOW,
    KEEP_APPROACHING,
    START_VERIFICATION,
    after_fail_mode_action,
    phase_a_decision,
)

GOAL_PLANE_M = 0.020
STATIONARY_MPS = 0.01
TOLERANCE_M = 0.03


def _f(value):
    try:
        if value is None or value == "":
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def replay_bundle(bundle: Path, execution_mode: str = "AUTO") -> dict:
    events_path = bundle / "events.jsonl"
    events = []
    with events_path.open() as handle:
        for line in handle:
            line = line.strip()
            if line:
                events.append(json.loads(line))

    telemetry = []
    tele_path = bundle / "telemetry.csv"
    if tele_path.is_file():
        with tele_path.open() as handle:
            telemetry = list(csv.DictReader(handle))

    started_hold = False
    fail_now_events = []
    verification_messages = []
    current_point = "P0001"
    rpp_outcome = ""

    for event in events:
        topic = event.get("topic")
        payload = event.get("payload") or {}
        t = float(event.get("elapsed_sec") or 0.0)
        if topic == "mission_manager/status":
            msg = str(payload.get("message") or "")
            if "verification hold" in msg or "starting 3.0s verification" in msg:
                verification_messages.append((t, msg))
            current_point = payload.get("current_point_id") or current_point
            radial = payload.get("marking_radial_error_m")
            along = payload.get("marking_along_error_m")
            speed = payload.get("speed_mps")
            if "RPP reported terminal MISSED" in msg:
                rpp_outcome = "MISSED"
            elif "RPP reported 30mm CAPTURED" in msg:
                rpp_outcome = "CAPTURED"
            inside = (
                radial is not None
                and math_isfinite(radial)
                and radial <= TOLERANCE_M
            )
            stationary = speed is not None and math_isfinite(speed) and speed <= STATIONARY_MPS
            past = (
                along is not None
                and math_isfinite(along)
                and along >= GOAL_PLANE_M
            )
            decision = phase_a_decision(
                inside_30mm=bool(inside),
                stationary=bool(stationary),
                rpp_outcome=rpp_outcome,
                past_goal_plane=bool(past),
            )
            if decision == START_VERIFICATION:
                started_hold = True
            elif decision == FAIL_NOW:
                fail_now_events.append(
                    {
                        "t": t,
                        "point": current_point,
                        "radial_mm": None if radial is None else radial * 1000.0,
                        "along_mm": None if along is None else along * 1000.0,
                        "rpp": rpp_outcome,
                        "action": after_fail_mode_action(execution_mode),
                    }
                )
                rpp_outcome = ""
                current_point = "NEXT"
        elif topic == "mission_manager/point_event":
            if payload.get("event") == "ACCURACY_FAILED":
                acc = payload.get("accuracy") or {}
                fail_now_events.append(
                    {
                        "t": t,
                        "point": payload.get("point_id"),
                        "bag_event": True,
                        "radial_mm": acc.get("radial_error_mm"),
                        "reason": acc.get("reason"),
                    }
                )

    # Drive the patched gate from telemetry as the live 20 Hz truth.
    live_fails = []
    handshake_by_time = []
    for event in events:
        msg = str((event.get("payload") or {}).get("message") or "")
        et = float(event.get("elapsed_sec") or 0.0)
        if "RPP reported terminal MISSED" in msg:
            handshake_by_time.append((et, "MISSED"))
        elif "RPP reported 30mm CAPTURED" in msg:
            handshake_by_time.append((et, "CAPTURED"))

    def rpp_at(t: float) -> str:
        outcome = ""
        for et, value in handshake_by_time:
            if et <= t:
                outcome = value
            else:
                break
        return outcome

    hold_started = False
    for row in telemetry:
        t = _f(row.get("elapsed_sec")) or 0.0
        radial = _f(row.get("marking_radial_error_m"))
        along = _f(row.get("marking_along_error_m"))
        speed = _f(row.get("actual_speed_mps"))
        point = row.get("current_point_id") or ""
        rpp_from_events = rpp_at(t)
        inside = radial is not None and radial <= TOLERANCE_M
        stationary = speed is not None and speed <= STATIONARY_MPS
        past = along is not None and along >= GOAL_PLANE_M
        if not hold_started:
            decision = phase_a_decision(
                inside_30mm=bool(inside),
                stationary=bool(stationary),
                rpp_outcome=rpp_from_events if point in {"P0001", ""} else "",
                past_goal_plane=bool(past),
            )
            if decision == START_VERIFICATION:
                hold_started = True
            elif decision == FAIL_NOW and point in {"P0001", "P0002", "P0003", "P0004", ""}:
                live_fails.append(
                    {
                        "t": t,
                        "point": point or "P0001",
                        "radial_mm": None if radial is None else radial * 1000.0,
                        "action": after_fail_mode_action(execution_mode),
                    }
                )
                break

    original_started_3s_on_miss = any(
        "RPP reported terminal MISSED" in msg
        or "RPP reported 30mm CAPTURED" in msg
        for _, msg in verification_messages
    ) or any("starting 3.0s verification" in msg for _, msg in verification_messages)

    return {
        "bundle": bundle.name,
        "original_verification_messages": verification_messages[:8],
        "original_started_3s_on_handshake": original_started_3s_on_miss,
        "patched_p1_started_hold": hold_started,
        "patched_p1_fail_now": live_fails,
        "patched_action": live_fails[0]["action"] if live_fails else None,
        "keep_approaching_if_no_fail": not live_fails and not hold_started,
    }


def math_isfinite(value) -> bool:
    try:
        return float(value) == float(value) and value != float("inf") and value != float("-inf")
    except (TypeError, ValueError):
        return False


def main() -> int:
    bags_root = ROOT / "bags" / "Mission Log"
    targets = [
        bags_root / "mission.csv_20260821_113612",
        bags_root / "mission.csv_20260821_104832",
        bags_root / "mission.csv_20260821_104948",
        bags_root / "mission.csv_20260821_104655",
    ]
    failed = 0
    print("=== marking-gate bag replay (patched policy) ===")
    for bundle in targets:
        if not bundle.is_dir():
            print(f"SKIP missing {bundle}")
            continue
        result = replay_bundle(bundle)
        print(f"\n{result['bundle']}")
        print(f"  original 3s-on-handshake: {result['original_started_3s_on_handshake']}")
        if result["original_verification_messages"]:
            t0, msg0 = result["original_verification_messages"][0]
            print(f"  original first verify msg t={t0:.3f}s: {msg0[:120]}")
        print(f"  patched P1 started 3s hold: {result['patched_p1_started_hold']}")
        print(f"  patched P1 fail-now: {result['patched_p1_fail_now'][:1]}")
        print(f"  patched AUTO/MANUAL action: {result['patched_action']}")

        # These field bags all sat outside 30 mm. Patched gate must not
        # start 3 s; it must FAIL immediately and AUTO-continue.
        if result["patched_p1_started_hold"]:
            print("  FAIL: patched gate started 3 s hold on a miss bag")
            failed += 1
        elif not result["patched_p1_fail_now"]:
            print("  FAIL: patched gate never failed P1 on a known miss bag")
            failed += 1
        elif result["patched_action"] != "AUTO_CONTINUE":
            print("  FAIL: AUTO miss did not continue")
            failed += 1
        else:
            fail = result["patched_p1_fail_now"][0]
            if fail.get("radial_mm") is not None and fail["radial_mm"] <= 30.0:
                print("  FAIL: fail-now fired inside 30 mm")
                failed += 1
            else:
                print("  PASS")

    # Synthetic pass path.
    assert (
        phase_a_decision(inside_30mm=True, stationary=True)
        == START_VERIFICATION
    )
    assert after_fail_mode_action("MANUAL") == "WAITING_FOR_NEXT"
    print("\nSynthetic PASS: 30mm+stationary starts 3s")
    print("Synthetic PASS: MANUAL miss waits for NEXT")
    print(f"\nReplay failures: {failed}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
