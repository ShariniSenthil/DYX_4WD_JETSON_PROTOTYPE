#!/usr/bin/env python3
"""EKF2 GNSS gate report: which check fails, why, and what it costs.

Answers, from a directory of ULogs:
  - which of the EKF2_GPS_CHECK gates actually fail (estimator_gps_status)
  - whether PDOP failures are HDOP- or VDOP-driven
  - EPH separation between RTK and non-RTK fixes (sizes EKF2_REQ_EPH)
  - longest continuous run below each fix type (sizes EKF2_REQ_FIX)
  - whether a fail run exceeds reset_timeout_max (7 s), and the resulting resets

⚠ The check_fail_* flags are computed UNCONDITIONALLY (gps_checks.cpp:63-76)
and logged regardless of EKF2_GPS_CHECK. A masked-off check still reports its
flag here; the mask only controls whether it makes runGnssChecks() return false.
So a row appearing below does not by itself mean samples were rejected.

Usage:
  python3 scripts/ekf2_gnss_gate_report.py <dir-of-ulogs> [--reset-timeout 7.0]
"""
import argparse
import glob
import os

import numpy as np
from pyulog import ULog

PARAMS = ["EKF2_GPS_CHECK", "EKF2_REQ_PDOP", "EKF2_REQ_EPH", "EKF2_REQ_EPV",
          "EKF2_REQ_NSATS", "EKF2_REQ_SACC", "EKF2_NOAID_TOUT", "EKF2_REQ_GPS_H",
          "EKF2_GPS_CTRL", "EKF2_HGT_REF"]


def _topic(ulog, name):
    for ds in ulog.data_list:
        if ds.name == name:
            return ds
    return None


def _longest_run(mask, t):
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logdir")
    ap.add_argument("--reset-timeout", type=float, default=7.0,
                    help="reset_timeout_max in seconds (common.h:478, compile-time "
                         "const 7.0; NOT EKF2_NOAID_TOUT)")
    args = ap.parse_args()

    logs = sorted(glob.glob(os.path.join(os.path.expanduser(args.logdir), "*.ulg")))
    if not logs:
        raise SystemExit(f"no .ulg files under {args.logdir}")

    pset, rows, fail_rows = {}, [], []
    fix_all, eph_all, hdop_all, vdop_all = [], [], [], []
    eph_by_fix = {}

    for path in logs:
        name = os.path.basename(path).split("_202")[0]
        try:
            u = ULog(path, ["vehicle_gps_position", "estimator_gps_status",
                            "estimator_status"])
        except Exception as exc:  # noqa: BLE001 - report and continue over a log set
            print(f"SKIP {name}: {exc}")
            continue

        for k in PARAMS:
            if k in u.initial_parameters:
                pset.setdefault(k, set()).add(u.initial_parameters[k])

        gps = _topic(u, "vehicle_gps_position")
        if gps is not None:
            t = gps.data["timestamp"] / 1e6
            fix = gps.data["fix_type"].astype(int)
            eph, hdop, vdop = gps.data["eph"], gps.data["hdop"], gps.data["vdop"]
            fix_all.append(fix)
            eph_all.append(eph)
            hdop_all.append(hdop)
            vdop_all.append(vdop)
            for ft in np.unique(fix):
                eph_by_fix.setdefault(int(ft), []).append(eph[fix == ft])
            rows.append((name, t[-1] - t[0], 100 * np.mean(fix == 6),
                         _longest_run(fix < 6, t), _longest_run(fix < 5, t),
                         np.nanmax(hdop), np.nanmax(vdop)))

        egs = _topic(u, "estimator_gps_status")
        if egs is not None:
            t = egs.data["timestamp"] / 1e6
            for key in sorted(k for k in egs.data if k.startswith("check_fail")):
                v = egs.data[key] != 0
                if not v.any():
                    continue
                run = _longest_run(v, t)
                resets = ""
                est = _topic(u, "estimator_status")
                if est is not None and "reset_count_pos_ne" in est.data:
                    rc = est.data["reset_count_pos_ne"]
                    resets = f"{int(rc[0])}->{int(rc[-1])}"
                fail_rows.append((name, key.replace("check_fail_", ""),
                                  100 * np.mean(v), run,
                                  run > args.reset_timeout, resets))

    print("=== LIVE PARAMS (from ULogs; authoritative over any QGC export) ===")
    for k in PARAMS:
        if k in pset:
            v = sorted(pset[k])
            print(f"  {k:16s} = {v if len(v) > 1 else v[0]}")

    print("\n=== FIX TYPE / DOP PER LOG ===")
    hdr = f"{'log':9s}{'dur_s':>7s}{'%fix6':>7s}{'wrst<6':>8s}{'wrst<5':>8s}{'hdopmx':>8s}{'vdopmx':>8s}"
    print(hdr, "\n" + "-" * len(hdr), sep="")
    for name, dur, p6, w6, w5, hx, vx in rows:
        print(f"{name:9s}{dur:7.0f}{p6:7.1f}{w6:8.1f}{w5:8.1f}{hx:8.2f}{vx:8.2f}")

    print("\n=== CHECK FAILURES (only non-zero shown) ===")
    if not fail_rows:
        print("  none")
    else:
        hdr = f"{'log':9s}{'check':16s}{'%fail':>7s}{'longest_s':>11s}{'>7s':>6s}  resets"
        print(hdr, "\n" + "-" * (len(hdr) + 4), sep="")
        for name, key, pct, run, over, resets in fail_rows:
            print(f"{name:9s}{key:16s}{pct:7.2f}{run:11.2f}{'YES' if over else '-':>8s}  {resets}")

    if fix_all:
        F = np.concatenate(fix_all)
        H, V = np.concatenate(hdop_all), np.concatenate(vdop_all)
        P = np.sqrt(H ** 2 + V ** 2)
        print(f"\n=== AGGREGATE  n={len(F)} ===")
        print("  fix dist: " + ", ".join(
            f"{v}:{100 * np.mean(F == v):.2f}%" for v in sorted(set(F.tolist()))))
        print(f"  HDOP max {H.max():.2f}   VDOP max {V.max():.2f}   PDOP max {P.max():.2f}")
        n_pdop = int(np.sum(P > 3.0))
        if n_pdop:
            n_vdop_driven = int(np.sum((P > 3.0) & (H <= 3.0)))
            print(f"  PDOP>3.0 samples: {n_pdop}, of which HDOP alone would pass: "
                  f"{n_vdop_driven} ({100 * n_vdop_driven / n_pdop:.1f}% VDOP-driven)")

        print("\n  EPH range by fix_type (sizes EKF2_REQ_EPH):")
        for ft in sorted(eph_by_fix):
            a = np.concatenate(eph_by_fix[ft])
            print(f"    fix={ft} n={len(a):6d}  min={a.min():.4f}  max={a.max():.4f}")
        rtk = [k for k in eph_by_fix if k >= 5]
        non = [k for k in eph_by_fix if k < 5]
        if rtk and non:
            worst = max(np.concatenate(eph_by_fix[k]).max() for k in rtk)
            best = min(np.concatenate(eph_by_fix[k]).min() for k in non)
            verdict = (f"any threshold in ({worst:.3f}, {best:.3f}) separates cleanly"
                       if worst < best else "OVERLAP - EPH cannot separate RTK from non-RTK")
            print(f"\n  worst RTK EPH {worst:.4f} | best non-RTK EPH {best:.4f} -> {verdict}")


if __name__ == "__main__":
    main()
