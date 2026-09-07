# EKF2 GNSS gate — production fix, traced to source and sized from data

**Date:** 2026-09-07. **Status:** evidence complete, decision proposed, **nothing
applied**. **Supersedes the candidate** sketched in the PDOP handoff
(`PDOP off + EKF2_REQ_FIX = 6`) — see §4, that value is unsafe on this firmware.

**Reads on:** [2026-09-07_EKF2_PDOP_Gate_Root_Cause.md](2026-09-07_EKF2_PDOP_Gate_Root_Cause.md)
(root cause), [2026-09-07_Handoff_EKF2_PDOP.md](2026-09-07_Handoff_EKF2_PDOP.md) (open items).

All firmware citations are `git show 54f0455ffcd755534539a7cf33a09a20bf71d29d:<path>`
in `/Users/dyx_a1/Vetri/Way_to_Mark/PX4-Autopilot-4WD-Prod-Baseline`.
⚠ **That checkout's HEAD is `2f8ce71`, 964 commits ahead of the flashed baseline,
and the file has been renamed `gps_checks.cpp` → `gnss_checks.cpp` upstream.**
Reading the working tree gives you the wrong architecture. Always `git show` at
the baseline commit.

All data is the 21 ULogs in `~/Documents/QGroundControl Daily/Logs/4WD/Missions/`
(`log_51`–`log_71`), 9,488 `vehicle_gps_position` samples.

---

## 1. The full failure chain, now closed end to end

The root-cause doc established VDOP → PDOP rejection. This closes the remaining
half: how a rejection becomes a multi-metre reset.

```
VDOP degrades (HDOP stays fine)
   │  EKF2.cpp:2443   pdop = sqrt(hdop² + vdop²)
   ▼
pdop > EKF2_REQ_PDOP (3.0)          gps_checks.cpp:69
   │
   ▼
runGnssChecks() returns false       gps_checks.cpp:171-185
   │
   ▼
_gps_data_ready = false             gps_control.cpp:79-80
   │  ← the sample is DISCARDED, not merely "not used to start fusion"
   ▼
failures persist > EKF2_NOAID_TOUT (5 s)
   │
   ▼
stopGnssFusion()  "GNSS quality poor - stopping use"   gps_control.cpp:82-84
   │  ← rover now dead-reckons on IMU
   ▼
checks recover → starting_conditions_passing
   │
   ▼
resetHorizontalPositionToGnss()     gps_control.cpp:200
   │
   ▼
**multi-metre position reset**
```

**Measured confirmation (`log_52`):** two continuous PDOP-fail runs of **8.50 s
and 8.00 s**, both past the 5.0 s timeout, and `estimator_status.reset_count_pos_ne`
goes **7 → 9** — exactly two resets, matching the two the operator saw.

Near-miss worth noting: `log_71`'s longest run was **4.00 s**, just under the
timeout. No reset. The margin between "brief glitch" and "position jump" is
about one second of VDOP.

## 2. Which checks actually fail — the decisive measurement

`estimator_gps_status.check_fail_*`, all 21 logs:

| check | logs failing | worst |
|---|---|---|
| `max_pdop` | log_52 (40.8%), log_71 (13.0%), log_63 (3.2%), log_62 (3.2%) | 40.8% |
| **every other check** | **none** | **0.00%** |

No nsats, hacc, vacc, sacc, drift, speed or spoofing failure occurred anywhere
in the session. **PDOP is the sole failing gate**, so clearing its mask bit
would have produced a session with zero GNSS check failures.

Attribution is unambiguous: of the 331 samples with `PDOP > 3.0`, **331/331 had
`HDOP ≤ 3.0`**. HDOP never exceeded **2.63** anywhere; VDOP reached **4.96**.
A purely vertical geometry problem was rejecting horizontally-excellent RTK.

## 3. What the gates actually discriminate — EPH vs fix_type

EPH by fix type, full range over all 9,488 samples:

| fix_type | n | EPH min | EPH max |
|---|---|---|---|
| 3 (3D) | 41 | 1.115 | 1.600 |
| 4 (code differential) | 538 | 2.425 | 2.925 |
| 5 (RTK float) | 919 | 0.020 | **0.970** |
| 6 (RTK fixed) | 7,990 | 0.015 | 0.065 |

**The separation is clean and gap-free: worst RTK EPH 0.970 m, best non-RTK EPH
1.115 m.** Any threshold in `(0.970, 1.115)` separates RTK from non-RTK with
zero error on this dataset. So EPH can do the discrimination that `EKF2_REQ_FIX`
was being considered for — using a parameter that already exists.

This also explains `log_51`: it ran **100% at fix_type 4** with EPH ≈ 2.7 m, and
`estimator_gps_status` shows **no check failure at all**. `EKF2_REQ_EPH = 3.0`
accepted a 2.9 m fix, and the fix check (`fix_type < 3`) passed because
differential is 4. That is the loose-gate hole, and it is an EPH problem, not a
PDOP problem.

## 4. ⚠ Why `EKF2_REQ_FIX = 6` must NOT be used

The handoff's candidate was `PDOP off + EKF2_REQ_FIX = 6`. The source trace shows
this backfires on this firmware.

Upstream (PR #25215, commit `4812311c6c`) splits checks into
`runInitialFixChecks` (strict, uses `ekf2_req_fix`) and `runSimplifiedChecks`
(which **hardcodes `fix_type < 3`**, `gnss_checks.cpp:84` at HEAD). The split is
gated on `_initial_checks_passed` — **a one-way latch, not `in_air`**. The
earlier reasoning that the rover reaches the relaxed path *because arming sets
`in_air`* is wrong about the mechanism; the latch sets roughly `EKF2_REQ_GPS_H`
(1.0 s) after GNSS first goes healthy, long before arming.

**The baseline `54f0455` has no such split at all** — `runGnssChecks` runs the
full strict check on *every* sample, continuously, forever. So on this firmware
`EKF2_REQ_FIX = 6` means: every RTK-FLOAT sample is discarded, for as long as
FLOAT lasts.

Sized from the logs — longest continuous run below each fix type:

| log | longest run `fix < 6` | longest run `fix < 5` |
|---|---|---|
| log_63 | **95.4 s** | 4.0 s |
| log_51 | 53.7 s (boot) | 53.7 s (boot) |
| all others | ≤ 0.4 s | 0.0 s |

`log_63` spent **95.4 continuous seconds below RTK FIXED** (83.2% of it at RTK
FLOAT, EPH ≈ 0.02–0.97 m — perfectly usable). With `EKF2_REQ_FIX = 6` that
becomes a 95-second total GNSS rejection: `stopGnssFusion` at t+5 s, 90 s of IMU
dead reckoning, then a position reset on recovery. **It converts a benign RTK
degradation into a guaranteed instance of the exact fault we are removing.**

The underlying category error: "is this fix good enough to fuse without
corrupting the state" and "is this fix good enough to paint a 30 mm mark" are
different questions for different layers. The second one already has a home —
`trajectory_generator` enforces `required_gps_fix_type: 6` and
`rtk_stable_sec: 3.0`. Requiring RTK FIXED *inside the estimator* removes its
only absolute reference instead of pausing the mission.

## 5. Proposed fix

### Phase 1 — parameters only, no firmware, no flash

| param | now | proposed | why |
|---|---|---|---|
| `EKF2_GPS_CHECK` | `831` | **`829`** | clears PDOP bit 1. Removes 100% of the session's check failures and both resets. |
| `EKF2_REQ_EPH` | `3.0` | **`2.0`** | rejects the 2.4–2.9 m code-differential fixes `log_51` accepted; accepts all RTK. `2.0` is the declared metadata minimum, so it stays in-range. |

`829 = 831 − 2`. Bits remaining: nsats, EPH, EPV, SACC, HDRIFT, VSPD, SPOOFED.

Replayed against this session, Phase 1 yields **zero GNSS check failures and
zero PDOP-driven resets**, while `log_51`'s 2.7 m origin fix is rejected.

`EKF2_REQ_PDOP` is deliberately **not** raised instead: its declared max is
`5.0` and `log_52` peaked at `5.61`, so raising it cannot cover the observed
range and would still fail. Turning the bit off is the only complete option.

### Phase 2 — optional firmware backport, defense in depth

Phase 1 leans entirely on receiver-self-reported EPH, and the 2026-09-04
vertical anomaly proved this receiver can be *confidently wrong*. A structural
fix-type gate is worth having as a second, independent axis.

**Patch:** [docs/patches/0001-ekf2-backport-EKF2_REQ_FIX-onto-54f0455.patch](patches/0001-ekf2-backport-EKF2_REQ_FIX-onto-54f0455.patch)
— 5 files, 121 lines, **verified to apply cleanly** to `54f0455` with
`git apply --check`. It is a minimal adaptation of upstream `4812311c6c` to the
baseline's single-function architecture (upstream touches 7 files because of the
`GnssChecks` class and `estimator_interface.h` constructor, neither of which
exists here).

What it does:
- adds `MASK_GPS_FIX (1<<10)` and makes the fix check maskable — at baseline the
  fix flag is **unconditional** (`gps_checks.cpp:172`), so without this a new
  `req_fix` could never be turned off;
- `flags.fix = (gps.fix_type < _params.req_fix)` instead of `< 3`;
- `common.h` default `gps_check_mask` `21 → 1045` and yaml default `1023 → 2047`,
  preserving today's "fix check always on" semantics by default;
- lowers `EKF2_REQ_EPH` metadata `min: 2 → 0.5`, which is what makes an
  in-range `EKF2_REQ_EPH = 1.0` legitimate;
- documents the `>5` hazard from §4 directly in the parameter's `long` text.

Params after flashing:

| param | value | why |
|---|---|---|
| `EKF2_REQ_FIX` | **`5`** | rejects 3D and code-differential; accepts RTK float and fixed. **Never 6** — §4. |
| `EKF2_GPS_CHECK` | **`1853`** | `829 + 1024`, enabling bit 10. |
| `EKF2_REQ_EPH` | **`1.0`** | sits inside the measured `(0.970, 1.115)` gap. |

Residual risk of `EKF2_REQ_FIX = 5`: `log_63`'s longest sub-FLOAT run was
**4.0 s** against the 5.0 s `EKF2_NOAID_TOUT` — a 1-second margin. If Phase 2
proceeds, consider raising `EKF2_NOAID_TOUT` alongside it, or treat `5` as
gated on a longer observation window than one session.

## 6. Verification before any flash

Phase 2 changes estimator gating and must not be flashed on reasoning alone.
The `px4-firmware-verification` skill exists for exactly this: replay these
ULogs through pre-patch and post-patch builds and diff the estimator output.
The specific questions to answer:

1. With `EKF2_GPS_CHECK = 1853, EKF2_REQ_FIX = 5`, does `log_52` still produce
   `reset_count_pos_ne 7 → 9`, or 7 → 7?
2. Does `log_63` (95.4 s below RTK FIXED) survive with **no** `stopGnssFusion`?
3. Does `log_51` fail its checks at boot, i.e. is the 2.7 m origin rejected?
4. Do the 16 clean logs replay bit-identical to baseline?

Phase 1 needs no replay — it only clears a mask bit and tightens an existing
threshold, both of which are exactly computable from the recorded
`check_fail_*` fields, and both are trivially reversible via `param set`.

## 7. Still open (unchanged by this work)

- **Why VDOP degrades at these times** — geometry, obstruction, multipath or
  receiver-internal, all still open. Needs the raw Septentrio SBF stream;
  `SEP_DUMP_COMM = 0` today, so it is not being logged. Phase 1 stops VDOP from
  *causing resets*; it does not explain the VDOP.
- The `EKF2_GPS_DELAY` 40–60 ms timing-shift claim is still relayed, not
  re-derived.
- The yaw/height fusion-percentage denominator asymmetry (§3.5 of the
  root-cause doc) was not chased down.

## 8. Reproduce

Every number in this document comes from one tool:

```bash
python3 scripts/ekf2_gnss_gate_report.py ~/Documents/QGroundControl\ Daily/Logs/4WD/Missions
```

It prints live params, per-log fix/DOP behaviour, which checks failed, whether a
fail run exceeded `EKF2_NOAID_TOUT` and the resulting `reset_count_pos_ne`, the
HDOP/VDOP attribution, and the EPH separation gap. Point it at any ULog
directory to re-run this analysis on a later session.
