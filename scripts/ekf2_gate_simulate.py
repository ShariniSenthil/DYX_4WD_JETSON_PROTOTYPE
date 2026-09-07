#!/usr/bin/env python3
"""Replay PX4 1.16.2 runGnssChecks() over recorded ULogs under candidate params.

Mirrors src/modules/ekf2/EKF/aid_sources/gnss/gps_checks.cpp at PX4 commit
54f0455ffcd755534539a7cf33a09a20bf71d29d (the flashed 4WD baseline), so a
proposed EKF2_GPS_CHECK / EKF2_REQ_* change can be scored against real recorded
GNSS behaviour before any param is touched or firmware flashed.

Reports, per candidate: percentage of samples rejected, the longest continuous
rejection run, and whether that run exceeds EKF2_NOAID_TOUT -- which is the
threshold at which gps_control.cpp calls stopGnssFusion() and therefore forces
a horizontal position reset on recovery.

The drift/speed checks (HDRIFT/VDRIFT/HSPD/VSPD) are omitted: they only evaluate
when (!in_air && vehicle_at_rest), and estimator_gps_status shows they never
failed in the 2026-09-07 session. Verify that assumption with
scripts/ekf2_gnss_gate_report.py before trusting this on a new dataset.

Usage:
  python3 scripts/ekf2_gate_simulate.py <dir-of-ulogs> [--noaid-tout 5.0]
"""
import argparse
import glob
import os

import numpy as np
from pyulog import ULog

# EKF2_GPS_CHECK bit positions, gps_checks.cpp:47-57 (+ bit 10 from the backport)
M_NSATS, M_PDOP, M_HACC, M_VACC, M_SACC, M_SPOOF, M_FIX = 1, 2, 4, 8, 16, 512, 1024

BASE = dict(pdop=3.0, epv=5.0, nsats=6, sacc=0.5)
CONFIGS = [
    ("A as-run today", dict(BASE, mask=831, eph=3.0, fix=3, fix_masked=False)),
    ("B0 mask only", dict(BASE, mask=829, eph=3.0, fix=3, fix_masked=False)),
    ("B mask+EPH2.0", dict(BASE, mask=829, eph=2.0, fix=3, fix_masked=False)),
    ("C mask+EPH1.0", dict(BASE, mask=829, eph=1.0, fix=3, fix_masked=False)),
    ("D +REQ_FIX=5", dict(BASE, mask=1853, eph=1.0, fix=5, fix_masked=True)),
    ("E +REQ_FIX=6", dict(BASE, mask=1853, eph=1.0, fix=6, fix_masked=True)),
]


def longest_run(mask, t):
    """Longest contiguous duration (s) where mask is True."""
    best, start = 0.0, None
    for i, m in enumerate(mask):
        if m:
            if start is None:
                start = t[i]
            best = max(best, t[i] - start)
        else:
            start = None
    return best


def evaluate(d, c):
    """Return per-sample rejection mask, mirroring runGnssChecks()."""
    pdop = np.sqrt(d["hdop"] ** 2 + d["vdop"] ** 2)
    m = c["mask"]
    f_fix = d["fix_type"].astype(int) < c["fix"]
    # At baseline the fix flag is unconditional; the backport makes it maskable.
    fail = (f_fix & bool(m & M_FIX)) if c["fix_masked"] else f_fix
    fail = fail | ((d["satellites_used"] < c["nsats"]) & bool(m & M_NSATS))
    fail = fail | ((pdop > c["pdop"]) & bool(m & M_PDOP))
    fail = fail | ((d["eph"] > c["eph"]) & bool(m & M_HACC))
    fail = fail | ((d["epv"] > c["epv"]) & bool(m & M_VACC))
    fail = fail | ((d["s_variance_m_s"] > c["sacc"]) & bool(m & M_SACC))
    fail = fail | ((d["spoofing_state"] == 3) & bool(m & M_SPOOF))
    return fail


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logdir")
    ap.add_argument("--noaid-tout", type=float, default=5.0)
    args = ap.parse_args()

    logs = sorted(glob.glob(os.path.join(os.path.expanduser(args.logdir), "*.ulg")))
    data = []
    for p in logs:
        try:
            u = ULog(p, ["vehicle_gps_position"])
        except Exception as exc:  # noqa: BLE001 - report and continue over a log set
            print(f"SKIP {os.path.basename(p)}: {exc}")
            continue
        ds = [x for x in u.data_list if x.name == "vehicle_gps_position"]
        if ds:
            data.append((os.path.basename(p).split("_202")[0], ds[0].data))
    if not data:
        raise SystemExit(f"no usable ULogs under {args.logdir}")

    print(f"{'config':17s}{'GPS_CHECK':>10s}{'REQ_EPH':>9s}{'REQ_FIX':>9s}"
          f"{'%fail':>8s}{'worst_run_s':>13s}{'>NOAID':>9s}  offenders")
    print("-" * 100)
    for name, c in CONFIGS:
        tot = fl = 0
        worst = 0.0
        offenders = []
        for lname, d in data:
            t = d["timestamp"] / 1e6
            fail = evaluate(d, c)
            tot += len(fail)
            fl += int(fail.sum())
            if fail.any():
                r = longest_run(fail, t)
                worst = max(worst, r)
                if r > args.noaid_tout:
                    offenders.append(f"{lname}:{r:.1f}s")
        print(f"{name:17s}{c['mask']:>10d}{c['eph']:>9.1f}{c['fix']:>9d}"
              f"{100 * fl / tot:7.2f}%{worst:13.2f}"
              f"{('YES' if offenders else 'no'):>9s}  "
              f"{', '.join(offenders) if offenders else '-'}")

    print("\nA run exceeding NOAID_TOUT means stopGnssFusion() and a horizontal\n"
          "position reset on recovery. 'no' in that column is the goal.")


if __name__ == "__main__":
    main()
