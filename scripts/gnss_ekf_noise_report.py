#!/usr/bin/env python3
"""Task 2.1 — measure GNSS/EKF noise from recorded PX4 ulogs, before any tuning.

Reports, over mechanically-stationary windows:

    horizontal raw GNSS sigma      vertical raw GNSS sigma
    GNSS velocity sigma            GNSS yaw sigma
    EKF local-position sigma       EKF yaw sigma
    innovation / test-ratio distributions

Design notes (these are the reason the numbers can be trusted):

* Stationary windows are detected from ACTUATOR OUTPUT + GYRO only
  (`actuator_motors.control[0..1]` at neutral, `vehicle_angular_velocity.xyz[2]`
  below a yaw-rate gate). Neither GNSS nor the EKF is consulted, so measuring
  GNSS/EKF scatter inside those windows is not circular. CLAUDE.md's rule
  "never judge stopped-ness from the EKF velocity" is honoured structurally.
* Each window is trimmed at the head (`--settle`) because the EKF rings for
  seconds after a pivot, and rotation-in-place produces near-zero net
  translation — both known traps in this project.
* Sigma is reported at several averaging times. RTK error is strongly
  time-correlated, so the 0.1 s number is HIGH-FREQUENCY JITTER ONLY and is
  NOT the value to hand to EKF2_GPS_P_NOISE. See the tau table.
* Geodetic -> NED uses the PX4 sphere (R = 6371000 m), matching
  trajectory_generator/localization_frame.py. Residuals are taken about each
  window's own mean, so the earth model cancels anyway.
"""

import argparse
import glob
import math
import os
import sys

import numpy as np

try:
    from pyulog import ULog
except ImportError:
    sys.exit("ERROR: pyulog not installed — pip3 install pyulog")

EARTH_RADIUS_M = 6371000.0  # PX4's sphere, NOT the WGS84 ellipsoid


# ── helpers ────────────────────────────────────────────────────────────────
def topic(ulog, name, multi_id=0):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == multi_id:
            return d.data
    return None


def secs(data):
    return np.asarray(data["timestamp"], dtype=np.float64) * 1e-6


def sphere_ne(lat, lon, lat0, lon0):
    """Geodetic -> local North/East metres on PX4's sphere."""
    north = np.radians(lat - lat0) * EARTH_RADIUS_M
    east = np.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    return north, east


def wrap_pi(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def sigma_at_tau(resid, dt, taus):
    """std of block-averaged residuals, per averaging time.

    White noise gives sigma(tau) = sigma(dt) / sqrt(tau/dt). Anything flatter
    than that is time-correlated error, which is what actually limits absolute
    RTK accuracy. Blocks are only formed when the window holds >= 4 of them,
    because the residuals are already mean-removed per window and short
    windows would report an artificially small long-tau sigma.
    """
    out = {}
    for tau in taus:
        n = max(1, int(round(tau / dt)))
        vals = []
        for r in resid:
            nb = len(r) // n
            if nb < 3:
                continue
            blocks = r[: nb * n].reshape(nb, n).mean(axis=1)
            vals.append(blocks - blocks.mean())
        out[tau] = (float(np.std(np.concatenate(vals))), sum(len(v) for v in vals)) \
            if vals else (float("nan"), 0)
    return out


def pooled(resid):
    if not resid:
        return float("nan"), 0
    r = np.concatenate(resid)
    return float(np.std(r)), int(r.size)


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


# ── stationary detection ───────────────────────────────────────────────────
def stationary_intervals(ulog, args):
    """[(t0, t1)] where the drivetrain is commanded neutral and the rover is
    not rotating. Uses actuator + gyro only — never GNSS, never the EKF."""
    am = topic(ulog, "actuator_motors")
    av = topic(ulog, "vehicle_angular_velocity")
    if am is None or av is None:
        return []

    t_m = secs(am)
    drive = np.maximum(np.abs(np.nan_to_num(am["control[0]"])),
                       np.abs(np.nan_to_num(am["control[1]"])))
    t_g = secs(av)
    wz = np.abs(np.asarray(av["xyz[2]"], dtype=np.float64))

    t0, t1 = max(t_m[0], t_g[0]), min(t_m[-1], t_g[-1])
    if t1 - t0 <= 0:
        return []
    grid = np.arange(t0, t1, 0.02)  # 50 Hz decision grid
    quiet = (np.interp(grid, t_m, drive) < args.motor_gate) & \
            (np.interp(grid, t_g, wz) < args.yaw_rate_gate)

    intervals, start = [], None
    for i, q in enumerate(quiet):
        if q and start is None:
            start = grid[i]
        elif not q and start is not None:
            intervals.append((start, grid[i]))
            start = None
    if start is not None:
        intervals.append((start, grid[-1]))

    out, raw = [], []
    for a, b in intervals:
        raw.append((a, b))
        a2 = a + args.settle  # let the estimator stop ringing after a pivot
        if b - a2 >= args.min_window:
            out.append((a2, b))
    return out, raw


def slice_window(data, t, a, b):
    m = (t >= a) & (t <= b)
    return m if m.sum() >= 5 else None


# ── per-run measurement ────────────────────────────────────────────────────
def measure_run(path, args):
    ulog = ULog(path)
    wins, raw_wins = stationary_intervals(ulog, args)
    gps = topic(ulog, "vehicle_gps_position")
    lpos = topic(ulog, "vehicle_local_position")
    if gps is None or lpos is None:
        return None

    t_gps, t_lp = secs(gps), secs(lpos)
    lat = np.asarray(gps["latitude_deg"], float)
    lon = np.asarray(gps["longitude_deg"], float)

    r = {k: [] for k in ("gN", "gE", "gU_msl", "gU_ell", "gvN", "gvE", "gvD",
                         "gyaw", "eX", "eY", "eZ", "evX", "evY", "evZ", "eyaw")}
    reported = {k: [] for k in ("eph", "epv", "s_var", "hdg_acc", "sats",
                                "lp_eph", "lp_epv", "lp_evh", "lp_evv", "lp_hdg_sig")}
    window_means = []
    dt_gps = float(np.median(np.diff(t_gps))) if len(t_gps) > 2 else 0.1
    dt_lp = float(np.median(np.diff(t_lp))) if len(t_lp) > 2 else 0.1

    dropped = []
    kept = []
    for a, b in wins:
        mg = slice_window(gps, t_gps, a, b)
        if mg is not None and args.max_drift > 0:
            la, lo = lat[mg], lon[mg]
            n, e = sphere_ne(la, lo, la.mean(), lo.mean())
            tt = t_gps[mg] - t_gps[mg][0]
            ramp = math.hypot(*(np.polyfit(tt, v, 1)[0] * (tt[-1] - tt[0])
                                for v in (n, e)))
            if ramp > args.max_drift:
                dropped.append((a, b, ramp))
                continue
        kept.append((a, b))
        if mg is not None:
            la, lo = lat[mg], lon[mg]
            n, e = sphere_ne(la, lo, la.mean(), lo.mean())
            r["gN"].append(n - n.mean())
            r["gE"].append(e - e.mean())
            for key, field in (("gU_msl", "altitude_msl_m"),
                               ("gU_ell", "altitude_ellipsoid_m")):
                v = np.asarray(gps[field], float)[mg]
                r[key].append(v - v.mean())
            # parked: the TRUE velocity is zero, so residual = the value itself
            for key, field in (("gvN", "vel_n_m_s"), ("gvE", "vel_e_m_s"),
                               ("gvD", "vel_d_m_s")):
                r[key].append(np.asarray(gps[field], float)[mg])
            hd = np.asarray(gps["heading"], float)[mg]
            hd = hd[np.isfinite(hd)]
            if hd.size >= 5:
                r["gyaw"].append(wrap_pi(hd - np.arctan2(np.sin(hd).mean(),
                                                         np.cos(hd).mean())))
            for key, field in (("eph", "eph"), ("epv", "epv"),
                               ("s_var", "s_variance_m_s"),
                               ("hdg_acc", "heading_accuracy"),
                               ("sats", "satellites_used")):
                reported[key].append(np.asarray(gps[field], float)[mg])
            window_means.append((la.mean(), lo.mean(),
                                 float(np.asarray(gps["altitude_msl_m"], float)[mg].mean())))

        ml = slice_window(lpos, t_lp, a, b)
        if ml is not None:
            for key, field in (("eX", "x"), ("eY", "y"), ("eZ", "z")):
                v = np.asarray(lpos[field], float)[ml]
                r[key].append(v - v.mean())
            for key, field in (("evX", "vx"), ("evY", "vy"), ("evZ", "vz")):
                r[key].append(np.asarray(lpos[field], float)[ml])
            hd = np.asarray(lpos["heading"], float)[ml]
            hd = hd[np.isfinite(hd)]
            if hd.size >= 5:
                r["eyaw"].append(wrap_pi(hd - np.arctan2(np.sin(hd).mean(),
                                                         np.cos(hd).mean())))
            for key, field in (("lp_eph", "eph"), ("lp_epv", "epv"),
                               ("lp_evh", "evh"), ("lp_evv", "evv")):
                reported[key].append(np.asarray(lpos[field], float)[ml])
            hv = np.asarray(lpos["heading_var"], float)[ml]
            reported["lp_hdg_sig"].append(np.sqrt(np.clip(hv, 0, None)))

    wins = kept

    # ── post-stop settling: how far does the reported position keep moving
    #    AFTER the drivetrain goes neutral, and for how long? ───────────────
    settling = []
    for a, b in raw_wins:
        if b - a < 3.0:
            continue
        for src, t_src, getxy in (
                ("gnss", t_gps, lambda m: sphere_ne(lat[m], lon[m], lat[m].mean(),
                                                    lon[m].mean())),
                ("ekf", t_lp, lambda m: (np.asarray(lpos["x"], float)[m],
                                         np.asarray(lpos["y"], float)[m]))):
            m = (t_src >= a) & (t_src <= b)
            if m.sum() < 10:
                continue
            x, y = getxy(m)
            ts = t_src[m]
            tail = ts >= (ts[-1] - 1.0)
            x0, y0 = x[tail].mean(), y[tail].mean()
            d = np.hypot(x - x0, y - y0)
            settle_t = float("nan")
            for i in range(len(d)):
                if np.all(d[i:] < 0.010):
                    settle_t = ts[i] - a
                    break
            settling.append((src, float(d[0]), float(d.max()), settle_t))

    # ── innovations / test ratios, split stationary vs moving ──────────────
    aid = {}
    for short, name in (("pos", "estimator_aid_src_gnss_pos"),
                        ("vel", "estimator_aid_src_gnss_vel"),
                        ("hgt", "estimator_aid_src_gnss_hgt"),
                        ("yaw", "estimator_aid_src_gnss_yaw")):
        d = topic(ulog, name)
        if d is None:
            continue
        t = secs(d)
        stat = np.zeros(len(t), bool)
        for a, b in wins:
            stat |= (t >= a) & (t <= b)
        keys = [k for k in d.keys() if k.startswith("test_ratio[")] or ["test_ratio"]
        tr = np.max(np.vstack([np.asarray(d[k], float) for k in keys]), axis=0) \
            if keys[0].startswith("test_ratio[") else np.asarray(d["test_ratio"], float)
        inn_keys = [k for k in d.keys() if k.startswith("innovation[")]
        inn = np.vstack([np.asarray(d[k], float) for k in inn_keys]) if inn_keys \
            else np.asarray(d["innovation"], float)[None, :]
        aid[short] = {
            "t_ratio": tr, "stationary": stat,
            "innov_norm": np.sqrt((inn ** 2).sum(axis=0)),
            "rejected": np.asarray(d["innovation_rejected"], float).astype(bool),
            "fused": np.asarray(d["innovation_fused"], float).astype(bool)
            if "innovation_fused" in d else np.asarray(d["fused"], float).astype(bool),
        }

    est = topic(ulog, "estimator_status")
    gps_chk = topic(ulog, "estimator_gps_status")
    return {
        "name": os.path.basename(path),
        "duration_s": float(t_gps[-1] - t_gps[0]),
        "windows": wins, "dropped": dropped, "settling": settling,
        "stationary_s": float(sum(b - a for a, b in wins)),
        "resid": r, "reported": reported, "window_means": window_means,
        "dt_gps": dt_gps, "dt_lp": dt_lp, "aid": aid,
        "fix_type": np.asarray(gps["fix_type"], float),
        "resets": {k: int(np.asarray(est[k], float).max() - np.asarray(est[k], float).min())
                   for k in ("reset_count_pos_ne", "reset_count_vel_ne",
                             "reset_count_quat", "reset_count_pod_d")} if est else {},
        "gps_check_fail": int(np.asarray(gps_chk["checks_passed"], float).size -
                              np.asarray(gps_chk["checks_passed"], float).sum())
        if gps_chk else -1,
        "gps_check_n": int(np.asarray(gps_chk["checks_passed"], float).size) if gps_chk else 0,
    }


# ── reporting ──────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help=".ulg files or directories containing them")
    ap.add_argument("--motor-gate", type=float, default=0.01,
                    help="max |actuator_motors.control| counted as neutral (default 0.01)")
    ap.add_argument("--yaw-rate-gate", type=float, default=0.02,
                    help="max |gyro yaw rate| rad/s counted as not rotating (default 0.02)")
    ap.add_argument("--settle", type=float, default=2.0,
                    help="seconds trimmed off the head of each window (default 2.0)")
    ap.add_argument("--min-window", type=float, default=2.0,
                    help="minimum retained window length in seconds (default 2.0). "
                         "The marking hold is only 3.0s, so a longer minimum "
                         "silently discards every in-mission stop.")
    ap.add_argument("--max-drift", type=float, default=0.020,
                    help="reject a window whose horizontal residual holds a linear "
                         "ramp bigger than this, in metres (default 0.020). A ramp "
                         "is leftover motion, not receiver noise. 0 disables.")
    ap.add_argument("--cluster-radius", type=float, default=0.30,
                    help="two stationary windows within this many metres are "
                         "treated as the same physical place (default 0.30)")
    ap.add_argument("--tau-long", type=float, default=2.0,
                    help="longest averaging time in the sigma-vs-tau table (default 2.0)")
    args = ap.parse_args()

    files = []
    for p in args.paths:
        files.extend(sorted(glob.glob(os.path.join(p, "*.ulg")))
                     if os.path.isdir(p) else [p])
    if not files:
        sys.exit("ERROR: no .ulg files found")

    runs = [r for r in (measure_run(f, args) for f in files) if r]
    if not runs:
        sys.exit("ERROR: no run yielded usable data")

    print("=" * 78)
    print("TASK 2.1 — GNSS / EKF NOISE, MEASURED FROM STATIONARY WINDOWS")
    print("=" * 78)
    print(f"gates: |motor| < {args.motor_gate}, |yaw rate| < {args.yaw_rate_gate} rad/s, "
          f"settle {args.settle}s, min window {args.min_window}s\n")

    print(f"{'run':32s} {'dur':>7s} {'stat':>7s} {'wins':>5s} {'fix6':>6s} "
          f"{'sHz':>6s} {'sVt':>6s} {'resets':>9s}  window lengths (s)")
    for r in runs:
        rs = r["resets"]
        rstr = "/".join(str(rs.get(k, 0)) for k in
                        ("reset_count_pos_ne", "reset_count_vel_ne",
                         "reset_count_quat", "reset_count_pod_d")) if rs else "-"
        n = [w for w in r["resid"]["gN"] if len(w) >= 5]
        e = [w for w in r["resid"]["gE"] if len(w) >= 5]
        v = [w for w in r["resid"]["gU_msl"] if len(w) >= 5]
        sh = float(np.sqrt(np.mean(np.hypot(np.concatenate(n), np.concatenate(e)) ** 2))) \
            * 1000 if n else float("nan")
        sv = pooled(v)[0] * 1000 if v else float("nan")
        lens = " ".join(f"{b-a:.1f}" for a, b in r["windows"])
        if r["dropped"]:
            lens += "  [dropped " + ", ".join(f"{b-a:.1f}s/{d*1000:.0f}mm-ramp"
                                              for a, b, d in r["dropped"]) + "]"
        print(f"{r['name']:32s} {r['duration_s']:6.1f}s {r['stationary_s']:6.1f}s "
              f"{len(r['windows']):5d} {100*np.mean(r['fix_type']==6):5.1f}% "
              f"{sh:5.1f}m {sv:5.1f}m {rstr:>9s}  {lens}")

    def gather(key):
        return [w for r in runs for w in r["resid"][key] if len(w) >= 5]

    def rep(key):
        v = [w for r in runs for w in r["reported"][key] if len(w)]
        return np.concatenate(v) if v else np.array([])

    dt = float(np.median([r["dt_gps"] for r in runs]))
    dt_lp = float(np.median([r["dt_lp"] for r in runs]))

    print("\n" + "-" * 78)
    print("1) MEASURED SIGMA (residual about each window's own mean; parked)")
    print("-" * 78)
    rows = [
        ("raw GNSS north",      "gN",     "m",     dt),
        ("raw GNSS east",       "gE",     "m",     dt),
        ("raw GNSS horiz (2D)", None,     "m",     dt),
        ("raw GNSS alt MSL",    "gU_msl", "m",     dt),
        ("raw GNSS alt ellips", "gU_ell", "m",     dt),
        ("raw GNSS vel N",      "gvN",    "m/s",   dt),
        ("raw GNSS vel E",      "gvE",    "m/s",   dt),
        ("raw GNSS vel D",      "gvD",    "m/s",   dt),
        ("raw GNSS yaw",        "gyaw",   "rad",   dt),
        ("EKF local x",         "eX",     "m",     dt_lp),
        ("EKF local y",         "eY",     "m",     dt_lp),
        ("EKF local z",         "eZ",     "m",     dt_lp),
        ("EKF vel x",           "evX",    "m/s",   dt_lp),
        ("EKF vel y",           "evY",    "m/s",   dt_lp),
        ("EKF vel z",           "evZ",    "m/s",   dt_lp),
        ("EKF yaw",             "eyaw",   "rad",   dt_lp),
    ]
    print(f"{'quantity':22s} {'sigma':>12s} {'p95|.|':>10s} {'max|.|':>10s} {'n':>7s}")
    for label, key, unit, _ in rows:
        if key is None:
            n, e = gather("gN"), gather("gE")
            if not n:
                continue
            v = np.hypot(np.concatenate(n), np.concatenate(e))
            s = float(np.sqrt(np.mean(v ** 2)))
        else:
            w = gather(key)
            if not w:
                continue
            v = np.abs(np.concatenate(w))
            s = pooled(w)[0]
        scale, u = (1000.0, "mm") if unit == "m" else \
                   (1000.0, "mm/s") if unit == "m/s" else (math.degrees(1.0), "deg")
        print(f"{label:22s} {s*scale:9.1f} {u:3s} {pct(v,95)*scale:9.1f} "
              f"{v.max()*scale:9.1f} {v.size:7d}")

    print("\n" + "-" * 78)
    print("2) SIGMA vs AVERAGING TIME  (white noise would fall as 1/sqrt(tau))")
    print("-" * 78)
    tau_long = args.tau_long
    print(f"{'quantity':22s} {'unit':>6s} " +
          " ".join(f"{'t=' + format(t, '.3g') + 's':>10s}"
                   for t in (0.1, 0.5, tau_long)) +
          f" {'white@' + format(tau_long, '.3g') + 's':>11s}")
    for label, key, unit, base_dt in [r for r in rows if r[1]]:
        w = gather(key)
        if not w:
            continue
        tt = [base_dt, 0.5, tau_long]
        res = sigma_at_tau(w, base_dt, tt)
        scale, u = (1000.0, "mm") if unit == "m" else \
                   (1000.0, "mm/s") if unit == "m/s" else (math.degrees(1.0), "deg")
        s0 = res[tt[0]][0]
        cells = " ".join(f"{res[t][0]*scale:10.2f}" if res[t][1]
                         else f"{'--':>10s}" for t in tt)
        white = s0 * math.sqrt(base_dt / tau_long) * scale
        print(f"{label:22s} {u:>6s} {cells} {white:11.2f}")

    print("\n" + "-" * 78)
    print("3) RECEIVER / ESTIMATOR SELF-REPORTED ACCURACY (same windows)")
    print("-" * 78)
    print(f"{'field':28s} {'p50':>10s} {'p95':>10s} {'max':>10s}")
    for label, key, scale, u in (
            ("GNSS eph", "eph", 1000, "mm"), ("GNSS epv", "epv", 1000, "mm"),
            ("GNSS s_variance_m_s", "s_var", 1000, "mm/s"),
            ("GNSS heading_accuracy", "hdg_acc", math.degrees(1.0), "deg"),
            ("GNSS satellites_used", "sats", 1, ""),
            ("EKF eph", "lp_eph", 1000, "mm"), ("EKF epv", "lp_epv", 1000, "mm"),
            ("EKF evh", "lp_evh", 1000, "mm/s"), ("EKF evv", "lp_evv", 1000, "mm/s"),
            ("EKF heading sigma", "lp_hdg_sig", math.degrees(1.0), "deg")):
        v = rep(key)
        v = v[np.isfinite(v)]
        if not v.size:
            continue
        print(f"{label:28s} {pct(v,50)*scale:9.1f} {pct(v,95)*scale:9.1f} "
              f"{v.max()*scale:9.1f} {u}")

    print("\n" + "-" * 78)
    print("4) INNOVATION TEST RATIOS  (>1.0 = rejected by the gate)")
    print("-" * 78)
    print(f"{'aid source':16s} {'set':11s} {'n':>6s} {'p50':>8s} {'p95':>8s} "
          f"{'p99':>8s} {'max':>8s} {'rej%':>7s} {'fused%':>7s}")
    for short in ("pos", "vel", "hgt", "yaw"):
        for setname in ("stationary", "moving", "all"):
            n = tr = None
            trs, rej, fus = [], [], []
            for r in runs:
                a = r["aid"].get(short)
                if a is None:
                    continue
                m = a["stationary"] if setname == "stationary" else \
                    ~a["stationary"] if setname == "moving" else \
                    np.ones_like(a["stationary"])
                trs.append(a["t_ratio"][m]); rej.append(a["rejected"][m])
                fus.append(a["fused"][m])
            if not trs:
                continue
            tr = np.concatenate(trs); rj = np.concatenate(rej); fu = np.concatenate(fus)
            if not tr.size:
                continue
            print(f"{('gnss_'+short) if setname=='all' else '':16s} {setname:11s} "
                  f"{tr.size:6d} {pct(tr,50):8.3f} {pct(tr,95):8.3f} {pct(tr,99):8.3f} "
                  f"{tr.max():8.3f} {100*rj.mean():6.2f}% {100*fu.mean():6.2f}%")

    print("\n" + "-" * 78)
    print("5) LOW-FREQUENCY SCATTER — spread of the WINDOW MEANS")
    print("-" * 78)
    print("Within-window sigma above measures jitter only. These are the "
          "run-to-run/stop-to-stop\nmeans, which is the error scale that "
          "actually reaches a marking point.")
    allm = [(r["name"], m) for r in runs for m in r["window_means"]]
    if len(allm) >= 2:
        lat0 = float(np.median([m[0] for _, m in allm]))
        lon0 = float(np.median([m[1] for _, m in allm]))
        pts = []
        for name, (la, lo, al) in allm:
            nn, ee = sphere_ne(np.array([la]), np.array([lo]), lat0, lon0)
            pts.append((name, float(nn[0]), float(ee[0]), al))

        # cluster windows that sit at the same physical place across runs — the
        # only EXTERNAL reference these logs contain. CLAUDE.md's rule applies:
        # raw-vs-fused agreement proves nothing, revisiting a point does.
        clusters = []
        for pt in pts:
            for c in clusters:
                if math.hypot(pt[1] - c[0][1], pt[2] - c[0][2]) < args.cluster_radius:
                    c.append(pt)
                    break
            else:
                clusters.append([pt])
        clusters.sort(key=len, reverse=True)

        print(f"\nrevisited locations (windows within {args.cluster_radius:.2f} m "
              f"of each other):")
        any_multi = False
        for c in clusters:
            if len(c) < 2:
                continue
            any_multi = True
            n = np.array([p[1] for p in c]); e = np.array([p[2] for p in c])
            a = np.array([p[3] for p in c])
            print(f"\n  cluster at N{n.mean():+.2f} E{e.mean():+.2f}  ({len(c)} windows)")
            print(f"    {'run':32s} {'dN(mm)':>9s} {'dE(mm)':>9s} {'dAlt(mm)':>10s}")
            for name, nn, ee, al in c:
                print(f"    {name:32s} {(nn-n.mean())*1000:9.1f} "
                      f"{(ee-e.mean())*1000:9.1f} {(al-np.median(a))*1000:10.1f}")
            print(f"    spread: horiz {np.hypot(n-n.mean(), e-e.mean()).max()*1000:.1f} mm max, "
                  f"vert {(a.max()-a.min())*1000:.1f} mm range")
        if not any_multi:
            print("  none — every stationary window sat at a distinct place.")
        print("\nNOTE: a cluster mixes true RTK repeatability with the rover not "
              "parking in\n      exactly the same spot, so horizontal spread is an "
              "UPPER bound. Vertical\n      spread at one spot is the cleaner signal.")

    nfail = sum(r["gps_check_fail"] for r in runs if r["gps_check_n"])
    ntot = sum(r["gps_check_n"] for r in runs)
    if ntot:
        print(f"\nestimator_gps_status: {nfail}/{ntot} samples with a failing GPS check "
              f"({100*nfail/ntot:.2f}%)")
    print("\nNOTE: the t=0.1s sigma is high-frequency jitter. RTK error is "
          "time-correlated —\n      read the tau table before choosing "
          "EKF2_GPS_P_NOISE (Task 2.2).")


if __name__ == "__main__":
    main()
