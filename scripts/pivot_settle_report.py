#!/usr/bin/env python3
"""Report the pivot-settle dwell breakdown for one or more mission bundles.

Answers the only question that matters when tuning the legacy pivot-settle
gate: how long after the chassis PHYSICALLY stopped did the lifecycle keep
holding, and what was it waiting on?

For each ALIGNMENT window it prints
  - when rotation actually ceased (last |yaw_rate| > 0.10 rad/s, from the
    odometry twist angular.z, which on this stack is the raw IMU gyro --
    verified: /mavros/imu/data angular_velocity.z and
    /mavros/local_position/odom twist.angular.z agree exactly),
  - when the gate signal (hypot(twist.linear.x, twist.linear.y), the same
    value rpp_controller_node.py:3465 feeds to
    LegacyAlignmentLifecycle._chassis_stationary) first drops below the
    threshold and stays there for the 1.20 s dwell budget,
  - the gap between the two: time spent waiting on the estimator, not the
    rover. This is the number a stop_speed_mps change should shrink.

It also prints the stationary noise floor of the gate signal, measured only
over samples where position moved <10 mm/s -- use this to check that the
configured threshold still sits above the floor's p99 after any tuning.

Usage:
    python3 scripts/pivot_settle_report.py <bundle-dir> [<bundle-dir> ...]

Pass --threshold to evaluate a value other than the 0.060 currently set in
rover.launch.py (e.g. to re-check the old 0.030 against a new bag).
"""

from __future__ import annotations

import argparse
import bisect
import importlib.util
import json
import math
import os
import sqlite3
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

# Reuse the verified CDR reader/parsers rather than re-deriving them. Hand
# rolling the offsets is how you get plausible-looking garbage: an unaligned
# read of Odometry yields finite-but-absurd velocities that still plot.
_spec = importlib.util.spec_from_file_location(
    "analyze_mission", os.path.join(HERE, "analyze_mission.py")
)
_am = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_am)

DWELL_BUDGET_SEC = 1.20  # settle_sec 0.20 + post_settle_hold_sec 1.00
ROTATION_STOPPED_RADPS = 0.10
STATIONARY_POS_RATE_MPS = 0.010


def _find_db(bundle: str) -> str:
    for cand in (
        os.path.join(bundle, "bag", "bag_0.db3"),
        os.path.join(bundle, "bag_0.db3"),
    ):
        if os.path.isfile(cand):
            return cand
    raise SystemExit(f"no bag_0.db3 under {bundle}")


def _load(db_path: str):
    conn = sqlite3.connect(db_path)
    topics = {name: tid for tid, name in conn.execute("select id,name from topics")}

    def rows(topic):
        if topic not in topics:
            return []
        return list(
            conn.execute(
                "select timestamp,data from messages where topic_id=? order by timestamp",
                (topics[topic],),
            )
        )

    odom = []
    for stamp, blob in rows("/mavros/local_position/odom"):
        m = _am._p_odometry(blob)
        odom.append(
            {
                "t": stamp / 1e9,
                "x": m["x"],
                "y": m["y"],
                "v": math.hypot(m["lx"], m["ly"]),
                "wz": m["az"],
            }
        )

    debug = []
    for stamp, blob in rows("/rpp/debug"):
        payload = _am._p_string(blob).get("json")
        if payload:
            debug.append((stamp / 1e9, payload))

    if not odom or not debug:
        raise SystemExit(f"{db_path}: need both /mavros/local_position/odom and /rpp/debug")
    return odom, debug


def _alignment_windows(debug):
    windows = []
    current = None
    for t, d in debug:
        if d.get("control_mode") == "ALIGNMENT":
            if current is None:
                current = [t, t, d.get("goal_number")]
            else:
                current[1] = t
        elif current is not None:
            windows.append(current)
            current = None
    if current is not None:
        windows.append(current)
    return [w for w in windows if w[1] - w[0] > 0.5]


def _stationary_floor(odom):
    """Gate-signal distribution over samples the rover was genuinely still for.

    'Genuinely still' is judged from POSITION, not from the velocity estimate
    being tested -- otherwise the measurement assumes its own conclusion.
    """
    times = [o["t"] for o in odom]
    vals = []
    for o in odom:
        lo = bisect.bisect_left(times, o["t"] - 0.5)
        hi = bisect.bisect_right(times, o["t"] + 0.5) - 1
        if hi <= lo:
            continue
        span = times[hi] - times[lo]
        if span <= 0:
            continue
        moved = math.hypot(odom[hi]["x"] - odom[lo]["x"], odom[hi]["y"] - odom[lo]["y"])
        if moved / span < STATIONARY_POS_RATE_MPS and abs(o["wz"]) < 0.01:
            vals.append(o["v"])
    vals.sort()
    return vals


def _pct(sorted_vals, q):
    if not sorted_vals:
        return float("nan")
    return sorted_vals[int(q * (len(sorted_vals) - 1))]


def report(bundle: str, threshold: float) -> None:
    odom, debug = _load(_find_db(bundle))
    t0 = debug[0][0]

    print("=" * 78)
    print(os.path.basename(bundle.rstrip("/")))

    floor = _stationary_floor(odom)
    if floor:
        over = sum(1 for v in floor if v > threshold) / len(floor) * 100.0
        print(
            f"  gate-signal floor while genuinely stationary (n={len(floor)}): "
            f"median {_pct(floor,.5):.4f}  p90 {_pct(floor,.9):.4f}  "
            f"p99 {_pct(floor,.99):.4f}  max {floor[-1]:.4f} m/s"
        )
        print(
            f"  at threshold {threshold:.3f}: {over:.1f}% of stationary samples "
            f"still read as MOVING"
        )

    total_waste = 0.0
    for start, end, goal in _alignment_windows(debug):
        seg = [o for o in odom if start - 0.3 <= o["t"] <= end + 0.2]
        if len(seg) < 5:
            continue

        stopped = None
        for o in seg:
            if abs(o["wz"]) > ROTATION_STOPPED_RADPS:
                stopped = o["t"]
        if stopped is None:
            stopped = seg[0]["t"]
        after = [o for o in seg if o["t"] >= stopped]

        crossing = None
        for i, o in enumerate(after):
            if o["v"] > threshold:
                continue
            held = [x for x in after[i:] if x["t"] <= o["t"] + DWELL_BUDGET_SEC + 0.05]
            if all(x["v"] <= threshold for x in held):
                crossing = o["t"]
                break

        post = [o for o in after if o["t"] >= stopped + 0.8]
        peak_v = max((o["v"] for o in post), default=float("nan"))
        peak_w = max((abs(o["wz"]) for o in post), default=float("nan"))
        box_x = (max(o["x"] for o in post) - min(o["x"] for o in post)) if post else 0.0
        box_y = (max(o["y"] for o in post) - min(o["y"] for o in post)) if post else 0.0
        waste = (crossing - stopped) if crossing else float("nan")
        if math.isfinite(waste):
            total_waste += waste

        print(
            f"  GOAL {goal}: alignment {end - start:5.2f}s | rotation stops "
            f"t+{stopped - start:5.2f} | gate clears t+"
            f"{(crossing - start) if crossing else float('nan'):5.2f} | "
            f"release t+{end - start:5.2f}"
        )
        print(
            f"          WAITING ON ESTIMATOR: {waste:5.2f}s   "
            f"(post-stop peak |v|={peak_v:.4f} m/s, peak |yaw_rate|="
            f"{peak_w:.4f} rad/s, position box {box_x*1000:.0f}x{box_y*1000:.0f} mm)"
        )
    print(f"  TOTAL time waiting on the estimator this mission: {total_waste:.2f}s")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bundles", nargs="+")
    ap.add_argument(
        "--threshold",
        type=float,
        default=0.060,
        help="stop_speed_mps to evaluate (default 0.060, matching rover.launch.py)",
    )
    args = ap.parse_args()
    for bundle in args.bundles:
        report(bundle, args.threshold)


if __name__ == "__main__":
    main()
