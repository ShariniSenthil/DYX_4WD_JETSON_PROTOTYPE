#!/usr/bin/env python3
"""Split a ULog's parked windows on RTK ambiguity re-resolution boundaries.

Why this exists: the reported position of this rover is stable while one integer
ambiguity set is held and steps by 100-500 mm when the receiver re-resolves, with
no change in `eph`, satellite count or DOP (2026-09-09: +192 mm in log_15,
-210 mm in log_18).  Averaging or differencing across a re-fix therefore mixes two
different solutions and manufactures a "drift" or a "bias" that is neither.

    python3 scripts/rtk_refix_report.py <log.ulg> [--ref LAT,LON] [--min-window 8]

Height is the column to read.  Nothing in the rover controls it, whereas the
horizontal is closed-loop -- the rover drives until its *reported* position reaches
the surveyed target, so a horizontal bias is absorbed into where it physically
parks (CLAUDE.md hard rule).

Stationarity is gated on GNSS Doppler velocity AND yaw rate, so a pivot in place
cannot be mistaken for parked, and never on EKF velocity, which rings after a pivot.
"""

from __future__ import annotations

import argparse
import math

import numpy as np
from pyulog import ULog

WGS84_A, WGS84_F = 6378137.0, 1 / 298.257223563


def metres_per_degree(lat_deg: float):
    e2 = WGS84_F * (2 - WGS84_F)
    s = math.sin(math.radians(lat_deg))
    w = math.sqrt(1 - e2 * s * s)
    return (math.radians(1) * WGS84_A * (1 - e2) / w ** 3,
            math.radians(1) * WGS84_A / w * math.cos(math.radians(lat_deg)))


def yaw_rate(att, t):
    q0, q1, q2, q3 = att['q[0]'], att['q[1]'], att['q[2]'], att['q[3]']
    yaw = np.unwrap(np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 ** 2 + q3 ** 2)))
    ta = att['timestamp'] / 1e6
    return np.interp(t, ta, np.abs(np.gradient(yaw, ta)))


def report(path, ref, min_window, vmax=0.03, yawmax=0.05, refix_s=5.0):
    u = ULog(path, ['vehicle_gps_position', 'vehicle_attitude'])
    gps = [d for d in u.data_list if d.name == 'vehicle_gps_position']
    att = [d for d in u.data_list if d.name == 'vehicle_attitude']
    if not gps:
        print(f'{path}: no vehicle_gps_position'); return
    d = gps[0].data
    t = d['timestamp'] / 1e6
    fix = d['fix_type'].astype(int)
    speed = np.hypot(d['vel_n_m_s'], d['vel_e_m_s'])
    yr = yaw_rate(att[0].data, t) if att else np.zeros_like(t)
    parked = (speed < vmax) & (yr < yawmax) & (fix >= 6)

    mlat, mlon = metres_per_degree(ref[0] if ref else float(np.median(d['latitude_deg'])))
    origin = ref if ref else (float(np.median(d['latitude_deg'])),
                              float(np.median(d['longitude_deg'])))

    windows, i, n = [], 0, len(t)
    while i < n:
        if not parked[i]:
            i += 1; continue
        j = i
        while j < n and parked[j]:
            j += 1
        if t[j - 1] - t[i] >= min_window:
            windows.append((i, j))
        i = j

    print(f'{path.split("/")[-1]}   {t[-1]-t[0]:.0f} s, RTK fixed {100*(fix>=6).mean():.1f}% '
          f'of epochs, {len(windows)} parked windows >= {min_window:g} s')
    print(f'  reference {origin[0]:.8f}, {origin[1]:.8f}'
          f'{"  (surveyed)" if ref else "  (log median -- pass --ref for a surveyed point)"}')
    hdr = (f"  {'t(s)':>7s} {'dur':>6s} {'height(m)':>10s} {'step':>7s} {'dN':>8s} {'dE':>8s} "
           f"{'sats':>4s} {'vdop':>5s} {'eph':>5s}   gap since previous window")
    print(hdr); print('  ' + '-' * (len(hdr) - 2))
    prev_h = prev_j = None
    for (i, j) in windows:
        sl = slice(i, j)
        h = float(np.median(d['altitude_ellipsoid_m'][sl]))
        dn = (float(np.median(d['latitude_deg'][sl])) - origin[0]) * mlat * 1000
        de = (float(np.median(d['longitude_deg'][sl])) - origin[1]) * mlon * 1000
        gap, step = '', '      -'
        if prev_j is not None:
            seg = fix[prev_j:i]
            # sample period from the log itself, not assumed
            dt = float(np.median(np.diff(t[prev_j:i]))) if i - prev_j > 2 else 0.1
            lost = float((seg < 6).sum()) * dt
            gap = (f'{lost:.1f} s not fixed of {t[i]-t[prev_j-1]:.0f} s'
                   + ('   <<< RE-FIX' if lost >= refix_s else ''))
            step = f'{(h - prev_h)*1000:+7.0f}'
        print(f"  {t[i]:7.0f} {t[j-1]-t[i]:6.1f} {h:+10.3f} {step} {dn:+8.1f} {de:+8.1f} "
              f"{np.median(d['satellites_used'][sl]):4.0f} {np.median(d['vdop'][sl]):5.2f} "
              f"{np.median(d['eph'][sl])*1000:5.0f}   {gap}")
        prev_h, prev_j = h, j
    print('\n  A step across a RE-FIX line is a different ambiguity solution, not motion '
          'and not drift.\n  Steps between windows with no re-fix are the same solution.')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ulog')
    ap.add_argument('--ref', help='surveyed reference as LAT,LON (decimal degrees)')
    ap.add_argument('--min-window', type=float, default=8.0,
                    help='shortest parked window to report, seconds (default 8)')
    a = ap.parse_args()
    ref = tuple(float(x) for x in a.ref.split(',')) if a.ref else None
    report(a.ulog, ref, a.min_window)


if __name__ == '__main__':
    main()
