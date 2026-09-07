# EKF2 GNSS gate — production fix, traced to source and sized from data

**Date:** 2026-09-07. **Status:** source-traced and replay-verified against 21
ULogs; **approved for controlled field validation, NOT field-verified** — the
rover has never yet run with the change. **Nothing applied: no param set, no
firmware flashed.**

> **Verdict (§5): set `EKF2_GPS_CHECK` from `831` to `829`. That is the entire
> fix.** One parameter, no firmware, instantly reversible. Replayed over all
> 9,488 GNSS samples it takes the worst continuous rejection run from 8.90 s to
> **0.00 s** and eliminates both position resets. Every other candidate —
> including tightening `EKF2_REQ_EPH` — is strictly worse on this data and
> belongs in a separate decision.

**Corrections applied 2026-09-07 after independent review** (verdict unchanged,
supporting statements were wrong):
- The stop-fusion threshold is **`reset_timeout_max` = 7 s**, a compile-time
  constant — **not** `EKF2_NOAID_TOUT` (5 s), which does something else entirely
  and cannot widen it. See §1.
- **`check_fail_max_pdop` will still be logged** after applying `829`; the fail
  flag is computed unconditionally and the mask only gates its effect. The
  earlier "expect no `max_pdop` rows" validation criterion was wrong. See §6.
- The `EKF2_REQ_FIX` patch is **source-review plausible, not verified** — never
  built, replayed, flashed or field-tested.

**Supersedes the candidate** sketched in the PDOP handoff
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
failures persist > reset_timeout_max (7 s, hardcoded)
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
and 8.00 s**, both past the 7 s threshold, and `estimator_status.reset_count_pos_ne`
goes **7 → 9** — exactly two resets, matching the two the operator saw.

Near-miss worth noting: `log_71`'s longest run was **4.00 s**, under the
threshold. No reset.

### ⚠ Three different timeouts, and only one of them stops fusion

An earlier revision of this document, and both scripts, wrongly named
`EKF2_NOAID_TOUT` as the stop-fusion trigger. Corrected from source:

| constant | value | settable? | what it actually does |
|---|---|---|---|
| `no_aid_timeout_max` | 1 s | no (`const`) | declares the sensor no longer contributing to aiding (`common.h:479`) |
| `valid_timeout_max` | 5 s | **yes** — this is `EKF2_NOAID_TOUT` (`EKF2.cpp:146`) | reports the *state estimate* invalid after dead reckoning (`ekf_helper.cpp:880`) |
| `reset_timeout_max` | **7 s** | **no** — `const unsigned` (`common.h:478`) | **calls `stopGnssFusion()`** (`gps_control.cpp:82`) |

Two consequences. First, the threshold that matters here is **7 s, not 5 s**.
Second, **it cannot be tuned** — no parameter binds to `reset_timeout_max`, so
any plan that proposes "raise the timeout to buy margin" is not available on this
firmware. `EKF2_NOAID_TOUT` is a different knob for a different purpose.

The correction does not change the verdict: `log_52`'s two runs were 8.50 s and
8.00 s (8.90 s measured off `vehicle_gps_position` timestamps rather than
`estimator_gps_status`), all comfortably past 7 s, and the recommended
configuration has no sustained run at all.

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
becomes a 95-second total GNSS rejection: `stopGnssFusion` at t+7 s, 88 s of IMU
dead reckoning, then a position reset on recovery. **It converts a benign RTK
degradation into a guaranteed instance of the exact fault we are removing.**

The underlying category error: "is this fix good enough to fuse without
corrupting the state" and "is this fix good enough to paint a 30 mm mark" are
different questions for different layers. The second one already has a home —
`trajectory_generator` enforces `required_gps_fix_type: 6` and
`rtk_stable_sec: 3.0`. Requiring RTK FIXED *inside the estimator* removes its
only absolute reference instead of pausing the mission.

## 5. Verdict — the fix is one parameter

`runGnssChecks` was replayed sample-by-sample over all 9,488 samples under each
candidate (`scripts/ekf2_gate_simulate.py`). This is what decides the verdict;
everything above is why the candidates were chosen.

| config | `GPS_CHECK` | `REQ_EPH` | `REQ_FIX` | %fail | worst fail run | stops fusion (>7 s)? |
|---|---|---|---|---|---|---|
| A — as-run today | 831 | 3.0 | 3 | 3.50% | 8.90 s | **YES** `log_52` |
| **B0 — mask only** | **829** | **3.0** | 3 | **0.01%** | **0.00 s** | **no** |
| B — mask + EPH 2.0 | 829 | 2.0 | 3 | 5.68% | 53.68 s | YES `log_51` |
| C — mask + EPH 1.0 | 829 | 1.0 | 3 | 6.10% | 53.68 s | YES `log_51` |
| D — + `REQ_FIX=5` | 1853 | 1.0 | 5 | 6.10% | 53.68 s | YES `log_51` |
| E — + `REQ_FIX=6` | 1853 | 1.0 | 6 | 15.79% | 95.39 s | YES `log_51`, `log_63` |

### ✅ Apply this, and only this

| param | now | set to |
|---|---|---|
| `EKF2_GPS_CHECK` | `831` | **`829`** |

`829 = 831 − 2` (clears PDOP bit 1). Remaining bits: nsats, EPH, EPV, SACC,
HDRIFT, VSPD, SPOOFED.

Replayed over the whole session this leaves **0.01% failing samples, a worst
continuous fail run of 0.00 s, and no run anywhere exceeding `reset_timeout_max`** —
i.e. **zero `stopGnssFusion` events and zero position resets**, against 8.90 s
and two resets today. `log_63`'s RTK-FLOAT stretch also goes to 0.00 s. Nothing
else in the session's behaviour changes.

It is one `param set`, instantly reversible, and needs no build or flash.

`EKF2_REQ_PDOP` is deliberately **not** raised instead: its declared max is `5.0`
and `log_52` peaked at `5.61`, so raising it cannot cover the observed range.
Clearing the bit is the only complete option.

### ⚠ Do NOT bundle the EPH change with it

An earlier revision of this document proposed `EKF2_REQ_EPH 3.0 → 2.0` alongside
the mask change. **The replay shows that is wrong to bundle**, for a reason the
static analysis missed: `log_51` was **armed, with GNSS position fusion active
100% of the log and `xy_global` true**. It was not a harmless pre-origin warmup.
Tightening EPH there does not "block a bad origin" — it removes the vehicle's
only global position for 53.7 s **while armed**, which is a capability change,
not a bug fix. And it fixes a condition that produced **no reset**
(`log_51` `reset_count_pos_ne` 6 → 6).

The 2.7 m origin fix is still a genuine weakness, and `EKF2_REQ_EPH = 2.0` is
still the right lever for it (fix-4 EPH 2.425–2.925 m occurs *only* in `log_51`,
so it costs nothing in any other log). But it is a **separate decision about what
the rover should do when RTK is unavailable** — refuse to provide a position, or
provide a degraded one — and it belongs in the task sheet with operator sign-off,
not smuggled in behind a reset fix.

### Phase 2 — optional firmware backport, NOT required for this fault

The mask change fully removes the observed fault, so this section is **not part
of the fix**. It exists because the remaining weakness — accepting a metre-scale
fix, as in `log_51` — is currently gated only by receiver-self-reported EPH, and
the 2026-09-04 vertical anomaly proved this receiver can be *confidently wrong*.
A structural fix-type gate is a second, independent axis for that.

Note the replay verdict: **D (`REQ_FIX=5`) is not better than B0 on any log in
this session** — it is strictly more restrictive, and its only benefit is against
a failure mode this session did not exhibit. Treat it as insurance to be argued
on its own merits, not as an improvement on the fix.

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
**4.0 s** against the 7 s `reset_timeout_max` — a 3-second margin, wider than
the 1 s an earlier revision claimed, because that revision used the wrong
constant. **That margin cannot be widened**: `reset_timeout_max` is a
compile-time `const` with no parameter binding, so the only way to buy room is
to make rejections rarer, not the timeout longer. Treat `5` as gated on a longer
observation window than one session.

**This patch is not production-verified.** It has not been built, replayed,
flashed or field-tested. "Applies cleanly and is source-review plausible" is the
whole of its current status.

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

The `EKF2_GPS_CHECK = 829` fix needs no firmware replay: it only clears a mask
bit, its effect is exactly computable from the recorded samples, and
`scripts/ekf2_gate_simulate.py` has already scored it over the full session
(§5). It is reversible with a single `param set`.

But **it has never actually been run on the rover**, so its status is *approved
for controlled field validation*, not *field-verified*.

### ⚠ What to expect in the validation logs — `max_pdop` will still appear

An earlier revision said to expect **no `max_pdop` rows** after applying `829`.
That is wrong. `flags.pdop` is computed **unconditionally**
(`gps_checks.cpp:69`) and published straight to
`estimator_gps_status.check_fail_max_pdop` (`EKF2.cpp:1241`). The mask only
controls whether that flag makes `runGnssChecks()` return false. So whenever
VDOP pushes PDOP past 3.0, the flag will still be logged.

Judge the validation on consequences, not on the flag:

| signal | expected with `829` |
|---|---|
| `check_fail_max_pdop` | **may still be true** — not a failure |
| `"GNSS quality poor - stopping use"` | **absent** |
| `reset_count_pos_ne` | **constant** across the run |
| GNSS position fusion | **stays active** throughout |
| sustained PDOP-driven dead reckoning | **none** |

`scripts/ekf2_gnss_gate_report.py` reports the raw flags and now says so in its
own header; read its `>7s` column and the `resets` column, not the presence of a
row.

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
fail run exceeded `reset_timeout_max` and the resulting `reset_count_pos_ne`, the
HDOP/VDOP attribution, and the EPH separation gap. Point it at any ULog
directory to re-run this analysis on a later session.
