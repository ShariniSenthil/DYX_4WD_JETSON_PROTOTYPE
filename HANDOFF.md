# Handoff — 2026-09-04 session

Analysis of the **2026-09-03 evening dataset** (8 runs, 43 waypoints, 29 straight
legs) and the patches that came out of it. Three landed, one was reverted. No
field run was possible this session, so **everything applied here is unverified
in the field** — that is stated per patch, with what to watch for.

Datasets:

- `~/Documents/QGroundControl Daily/Logs/4WD/Madhavaram/Sep_03/evening_run_2/`
  (`Bags/` and `Ulogs/` side by side, all 8 bundles and all 8 ulogs present)
- `~/Documents/QGroundControl Daily/Logs/4WD/Madhavaram/Sep_04/Run_01/` — five
  bundles pulled from the Jetson this session: `mission.csv_20260904_{141428,
  141904,142609,142908,143312}`, 38 waypoints, 37 straight legs, 6 real pivots.
  **These ran with Patch A already live**, so they verify it.

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
| — | Patch D: backend passthrough for survey truth | **DONE, committed.** Frontend is the operator's |
| T3 | Straight-line tracking / the 3–5 cm weave | **DONE** |
| — | Patch C: cross-track recovery speed | **REVERTED — not applied.** Finding stands, see §6 |
| T2 | log_82 EKF false-position investigation | **NOT STARTED** |
| T4 | Pivot → settle → reanchor → drive lifecycle | **INSTRUMENTED, FIELD VERIFICATION PENDING** |
| T5 | Accuracy-panel vs report truth separation (spec) | **Superseded** by Patch B |
| — | Sep-04 verification of Patch A + weakest-link ranking | **DONE** |
| — | Tracking-architecture review vs the 3WD `PX4_DXP` stack | **DONE** |
| — | Straight-line tracking fix plan | **STEP 0 APPLIED (`60d4149`); STEPS 1/2 NOT APPLIED** |
| T6 | Ranked bug verdict table | **NOT STARTED** — needs T2 and T4 |

Reports published this session:

- T0 + T1 — https://claude.ai/code/artifact/606c5224-b35a-432d-b1e4-db60cfb14e36
- T3 — https://claude.ai/code/artifact/93ca9eba-1f9f-486b-a169-251e26eba3fb
- Sep-04 / what blocks 1 cm — https://claude.ai/code/artifact/c37fa012-47aa-4526-9487-4f90c060fb71

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

### Sep-04 Run_01 — Patch A verified, and cross-track is now the binding term

38 waypoints, RTK FIXED throughout, `radial_stop_brake_margin_m: 0.003` live.

**Patch A worked.** Median along-track went **+18.3 mm short → +6.7 mm short**;
the command now runs to 8.8 mm remaining instead of 18.0 mm. The systematic
short bias is gone.

**What replaced it as the limit is cross-track**, which carries **66% of the
radial error energy** (variance 1122 vs 566 mm²): |cross| p50 22.9 mm / p95
63.3 mm against |along| p50 14.3 mm. Zero the cross term and 24 of 33 stops are
already inside 20 mm; zero along and only 13 of 33 are. Radial p50 is 41.2 mm
and 7 of 38 stops made 20 mm.

The cross error is **inherited whole from the leg**:
`corr(cross-track at end of cruise → physical cross error at the stop) =
+0.858`, and the terminal 0.75 m shifts it by only −3.2 mm (p50).

Along-track bias is solved but the **spread is not**: 24/33 within ±20 mm,
6 short by >20 mm, and 3 that overshoot by 38–43 mm having coasted 34–58 mm
after the last command — while 24 of 38 last commands were still below the
0.143 m/s breakaway. Both failure modes now appear in one dataset, which is the
reachability limit showing up directly.

**Longer legs did not help.** These legs are 6.0 m against Sep-03's 4.5 m and
the loop still does not converge: |xtrack| 24.1 mm entering the cruise, 19.2 mm
leaving it, improved on 19 of 37 legs, **27 of 37 crossing the line**. The
Sep-03 reading that a 4.5 m leg was simply too short to contain one 3.7 m
correction cycle was incomplete — length was not the constraint.

### Pivots (Sep-04, 6 real pivots) — the pivot is not the problem

Real pivots are ALIGNMENT blocks entered above 20° with under 1 m of walk; the
other 20 ALIGNMENT blocks are alignment *drives* (1.9° median entry, up to 3.8 m
travel) and pooling them corrupts every statistic.

| | p50 | p95 | max |
|---|---|---|---|
| turn angle | 92° | 153° | 173° |
| pivot walk | 575 mm | 636 | 651 |
| **\|HE\| at exit** | **1.43°** | 6.61° | 7.57° |
| **\|X\| at exit** | **21.0 mm** | 28.0 | 29.6 |
| heading swing, first 2 m | 2.54° | 13.2° | 15.3° |
| xtrack swing as reported, first 2 m | 63.5 mm | 126 | 142 |
| physical curvature, same window | 6.1 mm | 80.7 | 124.5 |

The pivot delivers good heading — **5 of 6 exit under 4°**. It does not deliver
zero cross-track: **21.0 mm p50**, only 2 of 6 inside 20 mm.

⚠ Roughly half the *reported* post-pivot swing is the reference line being
rebuilt, not the rover moving (physical curvature 6.1 mm p50). The **172.7° row
reversal** is the genuine outlier on every axis: exit HE 7.57°, heading swing
15.3°, and 124.5 mm of real physical curvature.

### Why straight-line tracking does not converge — the structural cause

```
lookahead time  = 0.9 s   (precision_lookahead_time_s x speed = 0.936 m @ 1.03 m/s)
command→heading = 1.0 s   (EKF yaw), 1.2 s (course over ground, raw GNSS only)
```

**The actuation delay equals the lookahead time.** Pure pursuit is stable only
when lookahead time comfortably exceeds loop delay; here the rover reaches the
aim point about when it finishes turning toward it. That is the marginal
stability condition, and it produces exactly the measured 3.59 s correction
cycle with no net convergence.

The delay is structural: `cmd_vel_bridge` sends `type_mask = 3527`
(`IGNORE_YAW | IGNORE_YAW_RATE`, `yaw = 0.0`, `yaw_rate = 0.0`), so **the
controller has no heading authority at all**. It steers by rotating the
velocity vector and waiting for PX4 to infer a turn from `atan2(vE, vN)`.

⚠ **`line_tracking_lookahead_m: 0.55` is not the lookahead that runs.**
`precision_guidance_enabled` is true, so `guidance.py:11` uses
`clamp(precision_lookahead_time_s x speed + xtrack_gain x |xtrack|, 0.2, 1.0)`
with the gain at 0.0. Measured live: 0.936 m. Tune the time constant.

### The 3WD `PX4_DXP` stack, for comparison

Same output topic (`/mavros/setpoint_raw/local`, PositionTarget, 50 Hz) and the
same controller/bridge split. The node is named `twist_to_setpoint` and the
3WD CLAUDE.md mentions `setpoint_velocity/cmd_vel` (TwistStamped) — both are
misleading; production publishes PositionTarget.

The difference is the mask. `PX4_DXP/src/twist_to_setpoint_node.py` sends
**2503** (velocity + **explicit yaw**, `msg.yaw = atan2(v_n, v_e)` recomputed
every cycle) and **455** when a yaw-rate feedforward is live, switching
dynamically. Its own comment gives the reason: PX4's derived-yaw path "lags on
turns". Steering law is otherwise the same pure-pursuit core, with
`l_d = clamp(1.6·v_smoothed + 0.05·|xtrack|, 0.52, 1.0)` and a
`1.5·theta_e` yaw-rate feedforward clamped to ±0.45 rad/s.

⚠ Do not port mask 455 on the strength of that tree alone: its own CLAUDE.md
states velocity OFFBOARD *discards* `trajectory_setpoint.yawspeed`, so explicit
**yaw** is the demonstrated half and the yaw-**rate** is not. (The operator
states the 3WD achieves cm tracking at 1 m/s and that repo's speed notes are
stale — do not repeat the "only at 0.35 m/s" caveat from its docs.)

### FIX vs GUARD — the layering that matters

| layer | what it does | current state |
|---|---|---|
| **Fix** | makes the lateral loop converge | broken (delay ≈ lookahead time) |
| **Guard** | bounds damage when tracking is already wrong | **neutral** |

Guards are `xtrack_priority_speed_mps` (= cruise, inert),
`xtrack_priority_correction_limit_deg: 22`, and
`terminal_goal_intercept_bearing_limit_deg: 22`. `bearing_clamp_fired` was
**0 cycles** across the Sep-03 straight legs, so the clamps are not binding
either — **the whole guard layer is currently inert, which is a clean baseline
for measuring a fix.**

⚠ **The cross-track speed cap was repeatedly mis-filed in this session as a
defect to fix** (both reports, an earlier revision of this file, and commit
`96c35e6` which set it to 0.60 and was reverted). **That framing was wrong and
the revert was right.** Lookahead is `time_s × speed`, so capping 1.03 → 0.60
shortens it 0.936 → 0.540 m and **step-changes pure-pursuit steering gain by
~1.7× (gain ∝ 1/L)**, discontinuously, at the 15 mm engage threshold, with
hysteresis on release — in a loop already marginally stable. That is a swing
generator, and the operator observed exactly that in the field. Guards stay
neutral while the fix is measured.

## 3. Patches

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

### Patch D — backend passthrough for survey truth · `2093b26`

`mission_manager` was producing `accuracy["survey"]` and the backend was
dropping it in two places. Both are passthrough only — no recalculation, no
normalisation, and no effect on gating or the point verdict.

- `MissionReportStore._canonical_point()` rebuilds `accuracy` from a fixed
  allowlist, so the nested `survey` object never reached
  `GET /api/mission/report`. Added outside the `if source_is_rpp:` block on
  purpose: a point RPP could not measure may still have a good physical
  measurement, and folding it into that branch would couple the two. A test
  locks the placement in.
- `survey_truth_enabled` / `_ready` / `_targets_loaded` / `_gnss_samples` /
  `_coordinate_mode` now flow `ros_bridge` → `state` →
  `build_mission_status_payload()`, which serves both
  `GET /api/system/status` and the `mission_status` socket event. This is the
  half the report cannot cover: the report says *afterwards* that survey truth
  was not recorded; `survey_truth_ready` says *before you start* that targets
  did not load or GNSS is not streaming, while it can still be fixed.

Deliberately NOT added to `_mission_progress_payload` — that payload is
change-gated by a signature over its whole contents and
`survey_truth_gnss_samples` is a constantly-moving counter, so including it
would make `mission_progress` fire every tick instead of on real progress.

Six tests in `test_precision_mission_report.py`. Frontend work (types, remark,
export columns) is the operator's and lives in the separate
`DYX_GCS_Frontend` repo.

### Patch E — post-pivot reanchor observability · `60d4149`

Diagnostics only. **No rover control behaviour, return value, fallback, speed,
lookahead, brake profile, pivot logic, or PX4 command was changed.**

`rpp_controller.reanchor_runtime_path_after_pivot()` now publishes one
best-effort structured event on `/rpp/pivot_debug` for every decision:

- `FIRED / runtime_path_installed`
- `DECLINED / segment_runtime_reanchored`
- `DECLINED / distance_le_waypoint_tolerance`
- `DECLINED / _install_runtime_entry_path_returned_false`
- `FAILED / missing_anchor_or_goal`

The payload carries `source=RPP_POST_PIVOT_REANCHOR`, schema version, goal
number, post-pivot anchor x/y, goal x/y, resulting bearing, and immediate
cross-track. On `FIRED`, immediate cross-track is `0.0` by construction because
the regenerated C'→goal line starts at the rover's current post-pivot position.

Publication is exception-contained and is never control authority. A failed
debug publisher therefore cannot change the original success/failure result of
the reanchor function.

Regression coverage was committed with the controller change. Verification
before commit/push:

- focused reanchor telemetry tests: **11 passed**
- functional `rpp_controller` suite: **436 passed**
- Python syntax / compile checks: passed
- `git diff --check`: passed
- simulated publisher failure preserved the original controller result
- all five decision paths emitted exactly one correctly classified event

ROS ament lint wrappers were not available on the local machine because their
Python modules are not installed.

Commit `60d4149 feat(rpp): instrument post-pivot reanchor decisions` was pushed
to `origin/feat/rtk-injection-v2` on 2026-09-04.

**Not deployed and not field-verified.** A rover test is not currently possible,
so Step 0 is complete at source/test/repository level only. Do not claim that a
real pivot has emitted this event until a future bag verifies it.

### Patch C — cross-track recovery speed · **REVERTED, NOT IN THE BUILD**

Proposed as `xtrack_priority_speed_mps: 1.00 → 0.60`, committed, then reverted
on the operator's instruction before any field use. The parameter is back at
`CRUISE_SPEED_MPS` (1.00).

**The measurement below is unaffected by the revert and still stands** — it is
kept because the finding is what matters, not the patch that was tried. The
open item is carried in §6.

The latch was **engaged on 89.6% of cruise cycles** (median cross-track 22.6 mm
against a 15 mm engage threshold) and capped speed at 1.000 m/s — exactly the
cruise speed. It did nothing. `update_xtrack_speed_cap_state`'s own docstring
still calls it "the hardened 0.15 m/s xtrack speed-cap latch"; the cap had been
widened to cruise speed and lost its function.

The reasoning for the reverted value: the correction cycle *time* (3.59 s) is set
by `precision_lookahead_time_s` and does not change with speed, but the *distance*
it consumes does, so 0.60 m/s would have put the cycle at 2.15 m instead of
3.71 m. It was never run, so that is a hypothesis, not a result.

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

In priority order. Only the stop profile changes rover behaviour; the survey-truth
work is report-only and the cross-track patch was reverted.

1. **Terminal error sign and size.** Patch A should give 3–17 mm short. If any
   point *overshoots*, `radial_stop_brake_margin_m` went too small — that is the
   2026-09-02 failure mode returning, and the value to move back is this one, not
   `conservative_decel_mps2`.
2. **Cross-track is unchanged and will stay unchanged.** Patch C was reverted, so
   expect the same behaviour as 2026-09-03: |xtrack| roughly equal at the start
   and end of each leg, and most legs still crossing the line.
   `/rpp/xtrack_speed_cap_mps` will still read 1.000 — equal to cruise speed,
   capping nothing — whenever `/rpp/xtrack_speed_cap_active` is true. Worth
   confirming rather than assuming, since it is the evidence behind §6.
3. **`accuracy["survey"]` present and `available: true`** on completed points,
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
**Partly answered by the Sep-04 dataset (see §2): the pivot itself is not the
primary culprit.** It exits at 1.43° median heading error with 5 of 6 under 4°.
The 172.7° row reversal remains the genuine outlier, with 124.5 mm of physical
curvature.

The previous observability blocker is now fixed in source by Patch E
(`60d4149`). Future runs can directly show whether
`reanchor_runtime_path_after_pivot()` fired, declined, or failed and what C'→goal
geometry it created.

**However, no rover test is currently possible.** Therefore T4 is still open at
the field-evidence level. Existing Sep-04 bags predate Patch E and cannot prove
the new events work on the rover.

Still unmeasured in a field run: reanchor bearing from the new event, xtrack/HE
at translation release, post-settle creep, and hold duration.

Per future pivot: entry bearing, exit heading error, physical pivot walk (RTK)
vs odom walk, reanchor outcome/reason, anchor x/y, goal x/y, reanchor bearing,
xtrack/HE at translation release, post-settle creep, and hold duration.

Historical evidence gap: `/rpp/pivot_debug`, `/rpp/speed_debug`,
`/rpp/tracking_debug`, `/rpp/terminal_bearing_frozen`,
`/rpp/terminal_precision_armed` and `/rpp/terminal_correction_deg` all recorded
0 messages in the old bags. Those old runs still require reconstruction from
`/rpp/debug` + `/rpp/geometry_debug` + odom timestamps.

### THE MAIN OPEN ITEM — straight-line tracking does not converge

Cause and evidence are in §2. Nothing is applied. The designed fix, in order:

**Step 0 — observability. DONE in `60d4149`, not field-verified.**
`rpp_controller` now publishes the post-pivot runtime-reanchor decision on
`/rpp/pivot_debug` with source `RPP_POST_PIVOT_REANCHOR`: outcome
(`FIRED` / `DECLINED` / `FAILED`), exact reason, anchor x/y, goal x/y, resulting
bearing, and immediate cross-track.

Controller + regression tests were committed together and pushed. This is
diagnostics-only and preserves the original control result even if publication
fails.

Because a rover test is not currently possible, there is still no field bag
containing these new events. Treat Step 0 as **source/test complete, field
verification pending**.

This observability remains important because two earlier pivot/reanchor
conclusions were drawn from the wrong observable and had to be retracted.

**Step 1 — explicit yaw. The structural fix.**
In `cmd_vel_bridge`: drop `IGNORE_YAW` (mask `3527 -> 2503`) and set
`msg.yaw = atan2(v_n, v_e)` in ENU, holding the last value below ~0.01 m/s to
avoid `atan2(0,0)`. This removes PX4's heading-inference step, which is where
the 1.0 s delay lives. The 3WD runs exactly this in production.

⚠ **Main risk, needs a deliberate test.** 4WD pivots are executed by PX4's own
differential logic reacting to the heading error implied by the velocity vector
(`RD_TRANS_DRV_TRN = 45 deg`). Commanding yaw explicitly changes what PX4 is
given as the target during that transition. Do **not** deploy straight to a
marking mission: one straight-line run, then one run with a single 90 deg pivot,
watching `/mavros/state` mode and the pivot walk.

**Step 2 — `precision_lookahead_time_s`, only after Step 1 is measured.**
With the delay reduced a shorter lookahead raises gain usefully instead of
destabilising. Tuning it now, against a 1.0 s delay, trades oscillation for
sluggishness. No run exists at any lookahead other than 0.936 m.

**Guards stay neutral throughout** — see the FIX vs GUARD table in §2. If the
loop converges the xtrack cap never engages and its value stops mattering.

### Reanchor — correct in source, unverifiable in the field

Two claims made this session were **wrong and are retracted**; recorded so they
are not re-derived:

1. ⛔ *"The runtime line is anchored before the pivot walk, so the pivot throws
   the rover off the line it just built."* **False.**
   `/runtime_nav_path` is published by **mission_manager**
   (`mission_manager_node.py:420`) and republished as goals advance — its point
   count falls monotonically through a run (1120 -> 995 -> 882 -> 758 -> 626 ->
   496 on `mission.csv_20260904_141428`). It has nothing to do with the
   reanchor, and timing it against pivots proves nothing.

2. ⛔ *"The cross-track speed cap being a no-op is a defect to fix."* **False,
   see §2** — it is a guard held neutral, and lowering it step-changes loop gain.

What the source actually does, verified by reading:
`legacy_alignment._settle_certificate_and_hold` emits `REANCHOR_ZERO` only after
`_chassis_stationary_debounced` passes and the `settle_sec` dwell elapses — i.e.
**after** the pivot settles. `reanchor_runtime_path_after_pivot` anchors at
`self.current_x/current_y` (post-pivot position) and never moves the goal. When
it succeeds, `path_bearing` follows the reanchored line
(`rpp_controller_node.py:9370-9382`). **CLAUDE.md's description is accurate.**

Also verified, because it would have invalidated the whole tracking analysis:
`/rpp/debug.cross_track_error_mm` comes from `goal_signed_cross_track`
(`rpp_controller_node.py:9442`), computed against `path_bearing` — which IS the
reanchored line. So the leg-convergence numbers are genuine tracking error, not
an artefact of measuring against `/nav_path`.

**Still open in field evidence:** cross-track at pivot exit was 21.0 mm p50
in the pre-instrumentation Sep-04 bags, while a successful reanchor creates zero
cross-track at C' by construction. Patch E now exposes whether the reanchor
fires, declines, or fails, but no rover run is currently possible. The next
available pivot bag must use the new event before deciding whether the reference
moves afterwards or the reanchor is being declined.

### T6 — ranked bug verdict table
Blocked on T2 and the remaining field-verification portion of T4.

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
  CLAUDE.md's note about 0.55 / 0.35–0.8 adaptive describes an inactive path.
  **Corrected in CLAUDE.md on 2026-09-04** — but CLAUDE.md is gitignored in this
  repo, so that correction is local to the machine it was written on and this
  file is the only copy that travels.
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
- **Verify which node publishes a topic before reading anything into its
  timing.** Two conclusions this session were drawn from `/runtime_nav_path`
  and had to be retracted; it belongs to `mission_manager`, not the reanchor.
  `grep -rn "<topic>" src/ --include="*.py"` first, every time.
- One caution from experience this session: a topic missing from a decoder's
  subscription list returns "0 of 0" and looks exactly like a real finding. It
  happened once here (`xtrack_speed_cap_active`) and would have inverted a
  conclusion. Check message counts against the bag's own topic table first.
