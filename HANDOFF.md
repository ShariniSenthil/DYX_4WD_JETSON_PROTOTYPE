# Handoff — 2026-09-04 session

Analysis of the **2026-09-03 evening dataset** (8 runs, 43 waypoints, 29 straight
legs) and the three patches that came out of it. No field run was possible this
session, so **every patch below is unverified in the field** — that is stated
per patch, with what to watch for.

Dataset: `~/Documents/QGroundControl Daily/Logs/4WD/Madhavaram/Sep_03/evening_run_2/`
(`Bags/` and `Ulogs/` side by side, all 8 bundles and all 8 ulogs present).

| bag | ulog | role |
|---|---|---|
| `mission.csv_20260903_165719` | `log_77` | pivot / bidirectional |
| `mission.csv_20260903_165926` | `log_79` | pivot / bidirectional |
| `mission.csv_20260903_171227` | `log_82` | ABORTED, EKF false-position suspect |
| `mission.csv_20260903_171757` | `log_87` | mixed, 10 points |
| `mission.csv_20260903_172119` | `log_89` | mixed, 10 points |
| `mission.csv_20260903_174005` | `log_98` | straight-line |
| `mission.csv_20260903_174203` | `log_100` | straight-line |
| `mission.csv_20260903_174520` | `log_103` | straight-line |

---

## 1. Task list and status

| # | Task | Status |
|---|---|---|
| T0 | Build raw-RTK survey truth; separate it from EKF control truth | **DONE** |
| T1 | Stop accuracy per waypoint, both truths, find the cause | **DONE** |
| — | Patch A: stop profile | **DONE, committed, unverified in field** |
| — | Patch B: survey truth in mission_manager | **DONE, committed, unverified in field** |
| T3 | Straight-line tracking / the 3–5 cm weave | **DONE** |
| — | Patch C: cross-track recovery speed | **DONE, committed, unverified in field** |
| T2 | log_82 EKF false-position investigation | **NOT STARTED** |
| T4 | Pivot → settle → reanchor → drive lifecycle | **NOT STARTED** |
| T5 | Accuracy-panel vs report truth separation (spec) | **Superseded** by Patch B |
| T6 | Ranked bug verdict table | **NOT STARTED** — needs T2 and T4 |

Reports published this session:

- T0 + T1 — https://claude.ai/code/artifact/606c5224-b35a-432d-b1e4-db60cfb14e36
- T3 — https://claude.ai/code/artifact/93ca9eba-1f9f-486b-a169-251e26eba3fb

---

## 2. What was found

### T0 — survey truth, and two tooling traps

Raw GNSS was **RTK FIXED for 100% of every run**, 16–18 satellites, h_acc 15 mm.
Raw lat/lon are int32 × 1e-7 deg = an **11.06 mm quantum**, so single-fix survey
truth carries ±5.5 mm. Parked scatter was 8.7 mm p95.

Two apparent errors were **rejected** — both are measurement artefacts, not rover
faults, and both will bite again if not remembered:

1. **`scripts/analyze_mission.py` projects on the WGS84 ellipsoid; PX4 uses a
   sphere of R = 6 371 000 m.** Range-proportional: +0.51% north, −0.13% east.
   This manufactured an 847–888 mm "local-frame offset" on the runs whose
   `gp_origin` sat 165 m from the work area. Re-projecting spherically matched
   the controller's own `goal_x`/`goal_y` to **0.0 mm on all 43 goals**.
   *Rule: never compare an absolute local coordinate against an ellipsoid
   projection of a lat/lon. Difference around the target instead — at 40 mm
   error scale a 0.5% model gap is 0.2 mm.*

2. **The 22–49 mm EKF-vs-raw separation is transport latency, not bias.** Split
   by speed it is 45–65 mm above 0.30 m/s but only **11–15 mm at rest**, at the
   measurement floor. A constant −20 to −60 ms shift collapses the median to
   15 mm in every run. Parked and marking, the EKF agrees with raw RTK.

`/mavros/gpsstatus/gps1/raw` and `/mavros/global_position/raw/fix` are
byte-identical in position (650/650 samples), so GPSRAW is preferred — it
carries position and fix quality in one atomic message.

### T1 — the stop is short, and it is the controller

**35 of 43 waypoints stopped SHORT** of the surveyed point on raw RTK truth,
median 18.3 mm, sign test p = 4.2e-5. The settled EKF agreed with raw RTK to a
median of **1.1 mm** — so the short stop is the profile, not the estimator.

`TerminalStopRegulator._braking_output` commands
`sqrt(2·conservative_decel·(along_remaining − brake_margin))`, which crosses the
measured 0.143 m/s motor breakaway at `brake_margin + 0.143²/(2·decel)` —
**28.6 mm** at the as-run 0.015 / 0.75. Inside that distance the command is below
what the drivetrain can turn the wheels with. Median advance after the last
non-zero command: **−0.2 mm**. The rover stalls and never drives the remainder.

A second, separate effect: during hard braking the EKF along-track estimate runs
**transiently optimistic by ~25 mm** and self-corrects over ~1.5 s
(174520/P0001 read `along_remaining` = −0.9 mm, then relaxed to the true
+24.5 mm). The zero latch is one-shot, so the stop decision is taken on that
transient. The *settled* estimate is accurate; the *decision-instant* one is not.

Judged on physical truth, **18/43 made the 30 mm latch**. RPP's own verdict
agreed on 37 of 43 — **3 false accepts, 3 false rejects**.

### T3 — the 3–5 cm is not a weave

29 straight legs, all at 1.03 m/s.

- **|cross-track| at cruise start 22.6 mm, at cruise end 22.6 mm. Median
  reduction over a whole leg: +0.0 mm.**
- **22 of 29 legs cross the line** — start on one side, finish on the other.
  That is one half-cycle of an under-damped correction.
- The correction cycle is **3.59 s = 3.71 m**; legs are 4.5 m (p50), 3.5 m
  shortest. Only six complete swings exist across all 29 legs, because a leg is
  barely long enough to hold one.

Two different cross-track numbers, and the gap is the finding:

| | p50 | p95 | p99 |
|---|---|---|---|
| deviation from the rover's own fitted line (path smoothness) | 8.1 mm | 31.0 mm | 47.9 mm |
| RPP's cross-track vs the line it should track | 21.4 mm | 66.0 mm | 100.6 mm |

The rover drives a clean straight line, in the wrong place. Heading error p50
0.91°, p95 3.06°.

**Target check** — ≤20 mm cross-track *and* ≤1° heading, settled under 1 m:
**met on 1 of 29 legs.** Cross-track alone reaches 20 mm on 12 legs at a median
of 3.24 m; heading alone on 13 legs at 0.93 m.

**The 1 s command→heading lag is the designed preview, not a defect.** Measured
1.00 s against EKF attitude and 1.20 s against course-over-ground from raw GNSS
alone (correlation 0.25 at zero lag rising to 0.92 at ~1.0 s). It matches the
lookahead: measured 0.936 m = `precision_lookahead_time_s: 0.9` × 1.03 m/s.

Rejected: **software delay.** The loop runs at 20.00 Hz with **0 deadline misses
in 6 240 cycles**, compute 4.6 ms p50, odom age 15 ms p50.

---

## 3. Patches applied

### Patch A — stop profile · `4bfa04e`

`radial_stop_brake_margin_m: 0.015 → 0.003`
(`rover.launch.py` and the matching default in `rpp_controller_node.py`)

Moves the un-executable zone from **28.6 mm → 16.6 mm**. Expected result:
**3–17 mm short instead of 15–29 mm.**

`radial_stop_conservative_decel_mps2` left at **0.75 on purpose** — raising it
shrinks the zone too, but pulls BRAKE_PROFILE entry closer to the goal, which is
what caused the 2026-09-02 35–50 mm coast overshoot. One term at a time keeps
the next run attributable.

**Hard limit, do not chase it with tuning:** ramping to zero lands ~18 mm short
(2026-09-03); holding the 0.15 m/s floor lands 35–50 mm past (2026-09-02). The
rover cannot be commanded below ~0.15 m/s and cannot stop in under ~35 mm from
it, so **the point is outside the reachable set of any static profile.** Landing
on it needs settle → re-measure → bounded creep retry above breakaway, which is
a controller change. That option was considered and deliberately deferred.

### Patch B — survey truth in the report · `1ab82f3`

Report only. **The 30 mm latch, spray gating and the point verdict are unchanged
and still run on RPP.**

- New pure module `src/mission_manager/mission_manager/survey_truth.py`
  (ROS-free, no EKF input, 17 tests seeded with real Sep-03 coordinates).
- `trajectory_generator` publishes the uploaded lat/lon on
  **`/trajectory_generator/survey_targets`** (latched String/JSON,
  `dyx4wd/survey_targets@1`). Only place un-projected mission coordinates leave
  that node. A cleared mission retracts the targets.
- `mission_manager` buffers raw GPSRAW and attaches `accuracy["survey"]` to every
  point: surveyed target lat/lon, physical stop lat/lon, along/cross/radial, fix
  type, sats, h_acc, sample scatter, and how many samples were trimmed.
- Surfaces on `/mission_manager/point_event`, in `point_accuracy_snapshots` in
  the mission summary, and as `point_survey_snapshots` + `survey_truth_*` health
  on `/mission_manager/status`.
- Sign convention matches the RPP report: `along_track_error_mm` positive =
  SHORT of the target; `cross_track_error_mm` positive = RIGHT of approach.
- Survey truth may report `available: false` (fix not RTK FIXED, window not
  stationary, local-coordinate mission) without affecting any gate.

**Validated by replaying all 8 bags through the shipped module:** 40 of 43
waypoints within 6 mm of the independently measured value, most within 1 mm. The
replay caught a real defect on the way — captured at the latch, the trailing
window still held the last centimetres of braking, which rejected 2 points and
shifted 3 more by 5–17 mm. The window is now trimmed from the old end until what
remains is a stationary cluster (old end only, so a rover that starts creeping
again stays visible as a rejection). The 3 points still differing by 7–17 mm are
the ones where the rover kept moving 15–66 mm *after* the latch — different
instants, not a disagreement.

### Patch C — cross-track recovery speed · this session, see git log

`xtrack_priority_speed_mps: CRUISE_SPEED_MPS (1.00) → XTRACK_RECOVERY_SPEED_MPS (0.60)`

The latch was **engaged on 89.6% of cruise cycles** (median cross-track 22.6 mm
against a 15 mm engage threshold) and capped speed at 1.000 m/s — exactly the
cruise speed. It did nothing. `update_xtrack_speed_cap_state`'s own docstring
still calls it "the hardened 0.15 m/s xtrack speed-cap latch"; the cap had been
widened to cruise speed and lost its function.

Why this rather than shortening the lookahead: the correction cycle *time*
(3.59 s) is set by `precision_lookahead_time_s` and does not change with speed,
but the *distance* it consumes does. At 0.60 m/s the cycle is **2.15 m instead of
3.71 m**, roughly half a leg, so it can complete and settle. It also shortens the
lookahead to 0.54 m while correcting (from 0.936 m) through the existing
`clamp(time_s · speed, 0.2, 1.0)` formula, tightening the geometry without
touching a gain. This path already exists, already engages, is scoped to exactly
the off-line condition, and releases at 8 mm.

Why not lower than 0.60: motor breakaway is 0.143–0.219 m/s, so anything near the
old 0.15 m/s would stall. 0.60 is a demonstrated operating point (staged bring-up
0.4 → 0.6 → 0.8 → 1.0).

---

## 4. Deploy

A new module file was added in Patch B, so a plain restart is not enough:

```bash
git pull && colcon build --symlink-install --packages-select mission_manager trajectory_generator
```

Then restart the stack via `src/rover_bringup/launch/rover.launch.py`.
`rpp_controller` picks up Patches A and C from the launch file — restart only,
no build. **Do not use `scripts/start_rover.sh`; it is broken.**

`CLAUDE.md` is gitignored in this repo, so the notes written there this session
stay on the machine they were written on and will **not** appear on the Jetson.

---

## 5. What to watch on the next run

In priority order, because all three patches land at once:

1. **Terminal error sign and size.** Patch A should give 3–17 mm short. If any
   point *overshoots*, `radial_stop_brake_margin_m` went too small — that is the
   2026-09-02 failure mode returning, and the value to move back is this one, not
   `conservative_decel_mps2`.
2. **Cross-track convergence within a leg.** Patch C should make |xtrack| at
   cruise end clearly smaller than at cruise start, and should stop most legs
   crossing the line. `/rpp/xtrack_speed_cap_mps` should now read 0.600, not
   1.000, whenever `/rpp/xtrack_speed_cap_active` is true.
3. **Mission duration.** Patch C slows the rover whenever it is more than 15 mm
   off-line, which on the Sep-03 data was 89.6% of cruise time. If the latch does
   not release more often once tracking improves, missions get materially slower.
   That is the main way this patch could be wrong.
4. **`accuracy["survey"]` present and `available: true`** on completed points,
   with `sample_count` ≥ 3 and `fix_type` 6. If it reports
   `GNSS_WINDOW_NOT_STATIONARY` often, `survey_truth_window_sec` (2.0) is
   straddling motion for that capture site.

---

## 6. Open work

### T2 — log_82 EKF false-position (`mission.csv_20260903_171227`)
Not started. The bundle carries an `INCOMPLETE` marker and
`fcu_params.captured: false`. Needed: EKF↔raw horizontal separation P50/P95/max
over the run, timestamp of divergence onset, RTK quality at that moment, EKF
resets and innovations from the ulog, and the exact abort trigger from
`events.jsonl` + `/mission_manager/status`. The question to answer directly: did
RPP believe it had reached the coordinates while raw RTK showed the rover
physically displaced?

Note T0 does **not** clear this. T0 shows no steady bias at rest across the
dataset; a gross transient divergence is a different failure and log_82 is the
run to test it on. That run's `sep@rest` max was 105 mm, the joint highest.

### T4 — pivot → settle → reanchor → drive
Not started, and now the higher-value one. T3 showed the lateral loop does not
converge within a leg, so whatever cross-track the pivot and reanchor hand it
largely survives to the stop (median entry error 22.6 mm, worst 93.4 mm).
Reducing the entry error is the other half of the stop problem.

Per pivot: entry bearing, exit heading error, physical pivot walk (RTK) vs odom
walk, reanchor bearing, xtrack and HE at reanchor, xtrack/HE at translation
release, post-settle creep, hold duration.

⚠ **Evidence gap:** `/rpp/pivot_debug`, `/rpp/speed_debug`,
`/rpp/tracking_debug`, `/rpp/terminal_bearing_frozen`,
`/rpp/terminal_precision_armed` and `/rpp/terminal_correction_deg` all recorded
**0 messages** in these bags. Pivot lifecycle has to be reconstructed from
`/rpp/debug` + `/rpp/geometry_debug` + odom timestamps. If a run is ever possible
again, getting those topics publishing would make T4 much cheaper.

### T6 — ranked bug verdict table
Blocked on T2 and T4.

### Carried over, not addressed this session
- **`RO_MAX_THR_SPEED = 1.9` is unverified** — no log contains throttle above
  0.277, and the local slope is 1.25 m/s per unit throttle. Needs a MANUAL-mode
  bench sweep before cruise goes above ~0.6.
- **`RD_TRANS_TRN_DRV = 0.10472 rad = 6°` on the FCU vs `pivot_exit_angle_deg =
  12.0`** in `rpp_controller.yaml`. Live mismatch, predates this work.
- **`PWM_AUX_FUNC5 = 0`** in the live capture vs the code comment claiming 301.
- **`rpp_controller` param dump timing out** on some runs, costing bag provenance.

### Found this session, not fixed
- **`line_tracking_lookahead_m: 0.55` does not govern.** `precision_guidance_enabled`
  is true, so `guidance.py` uses
  `clamp(lookahead_time_s · speed + xtrack_lookahead_gain · |xtrack|, 0.2, 1.0)`.
  CLAUDE.md's note about 0.55 / 0.35–0.8 adaptive describes an inactive path and
  should be corrected there.
- **`xtrack_priority_lookahead_m: 0.55` appears unreachable** while precision
  guidance is on — measured lookahead stayed 0.936 m throughout. Worth confirming
  in code whether that parameter can ever apply.
- **`precision_xtrack_lookahead_gain: 0.0`.** Note the term would *lengthen* the
  lookahead as error grows, so raising it is not a fix for the T3 finding.

---

## 7. Method notes for whoever picks this up

- **Never treat EKF coordinates as physical truth.** Keep RPP/EKF control truth
  and raw RTK survey truth separate in every number reported.
- **Never judge "is it stopped / how fast" from EKF twist.** Differentiate raw
  GNSS position over a ~0.5 s window and say which source each number came from.
- **Do not hand-roll CDR parsing.** Import `_CDR` and the parsers from
  `scripts/analyze_mission.py`.
- **The ulog is authoritative for FCU params**, above any QGC export.
- Analysis scripts from this session were written to the session scratchpad and
  are not preserved in the repo. Everything they produced is in the two published
  reports and in `CLAUDE.md`. If T2/T4 are picked up, they will need to be
  rewritten — budget for that.
- One caution from experience this session: a topic missing from a decoder's
  subscription list returns "0 of 0" and looks exactly like a real finding. It
  happened once here (`xtrack_speed_cap_active`) and would have inverted a
  conclusion. Check message counts against the bag's own topic table first.
