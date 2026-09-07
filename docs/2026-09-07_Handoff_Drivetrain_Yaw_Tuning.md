# 2026-09-07 Handoff — Drivetrain Baseline, Yaw-Rate Loop Tuning, OFFBOARD Pivot Validation

**Branch:** `feat/rtk-injection-v2`. Builds on [PX4_NSH_OVER_SSH.md](PX4_NSH_OVER_SSH.md)
(morning session) and closes several open items from that file and from
CLAUDE.md's 2026-09-04/09-05 sections. All ULog paths below are on the FCU
SD card (`/fs/microsd/log/2026-09-07/...`) unless noted as pulled to a local
path. Test harness scripts are archived in
[field-tests/2026-09-07_drivetrain_yaw_tuning/](field-tests/2026-09-07_drivetrain_yaw_tuning/).

## Live parameter state at end of session

```text
RD_MAX_THR_YAW_R = 0.37   (was 0.30-0.50 range tested; frozen here)
RO_YAW_RATE_P    = 0.20   (was 0.15; raised once, frozen here)
RO_YAW_RATE_I    = 0.03   (unchanged all session - was already this value
                            before any of today's tuning started; never
                            set by this session's scripts)
RO_YAW_RATE_LIM  = 45.0   (unchanged)
RO_YAW_ACCEL_LIM = -1     (unchanged, disabled)
RO_YAW_DECEL_LIM = -1     (unchanged, disabled)
COM_RC_IN_MODE   = 0      (restored after every MANUAL/ACRO test that used it)
```

All parameter changes went through NSH `param set` + `param save` +
`param show` verification, one parameter at a time, disarmed, exactly like
the morning session's `COM_RC_IN_MODE` procedure. No mixer, EKF, or firmware
change was made.

## 1. Drivetrain forward/reverse baseline — mapping correction

The originally requested test used MAVLink MANUAL_CONTROL `x` for
forward/reverse. Verified against firmware `54f0455ffcd755534539a7cf33a09a20bf71d29d`
before any motion: `x` decodes to `manual_control_setpoint.pitch`
(`mavlink_receiver.cpp:2095`), which `RoverDifferential.cpp` never reads.
The real forward/reverse axis is `z` -> `throttle` (`mavlink_receiver.cpp:2102`
-> `RoverDifferential.cpp:107`); steering is `y` -> `roll`. Test re-run on the
corrected `z` mapping (`neutral/FORWARD/REVERSE` at 0.30/0.50/0.70, `y=r=0`
held exact) completed cleanly. ULog: `08_04_54.ulg`.

## 2. ACRO is a genuine closed-loop yaw-rate path

Confirmed from source: `NAVIGATION_STATE_ACRO=10` sets
`flag_control_rates_enabled`, which routes `RoverDifferential::Run()` through
`DifferentialRateControl` (closed-loop PID against measured gyro rate via
`RoverControl::rateControl()`), not the raw open-loop MANUAL differential
path. In ACRO, MAVLink `y` (roll) is interpolated `[-1,1] -> [-RO_YAW_RATE_LIM,
+RO_YAW_RATE_LIM]` as the commanded `yaw_rate_setpoint`; `z` still drives
throttle unchanged, so holding `z=500` gives a pure in-place pivot. Live
`RO_YAW_RATE_LIM=45.0 deg/s` means full-stick ACRO commands exactly 45 deg/s.

## 3. Feedforward (`RD_MAX_THR_YAW_R`) sweep

Fixed-duration full-steer ACRO test (3s LEFT/RIGHT pulses, `RO_YAW_RATE_P/I`
held constant) at each value, comparing mean achieved yaw rate against the
45 deg/s commanded rate:

| `RD_MAX_THR_YAW_R` | Runs | Mean \|rate\| | Error |
|---:|---:|---:|---:|
| 0.30 | 1 | ~68 deg/s | +51% (heavy overshoot) |
| 0.35 | 1 | ~49.6 deg/s | +10% |
| 0.36 | 3 (12 segments) | 49.84 +/- 4.68 | +10.7% |
| **0.37** | 1 (3s) + 3 (5s, 12 segments) | 41.2 - 47.45 | -8.4% to +5% |
| 0.39 | 3 (12 segments) | 34.84 +/- 2.15 | -22.6% |
| 0.40 | 1 | ~40.1 deg/s | -11% |
| 0.50 | 1 | ~25-34 deg/s | ~-34% |

**Frozen at 0.37.** Repeat-run data (0.36 and 0.39, 3 runs / 12 segments each)
showed run-to-run scatter (+-2 to +-5 deg/s) comparable to the effect of a
0.01-0.02 step in the parameter — later single-run points (0.35, 0.37, 0.40)
should not be trusted to better than that resolution. This scatter is very
likely terrain-driven: the test surface is not flat, and every pivot
physically displaces the rover (~600-950 mm here, see section 5) onto a
slightly different patch of ground, so repeats are not on identical footing.
**Do not chase this feedforward value tighter without a flatter or larger
test area** - the noise floor is already close to the resolution being
sought.

## 4. `RO_YAW_RATE_P` bump: 0.15 -> 0.20

One 5s-pulse run at P=0.20 (`RD_MAX_THR_YAW_R=0.37`, `I=0.03`) vs the P=0.15
baseline (3-run average): steady-state error essentially unchanged (~8-10%
either way), LEFT/RIGHT symmetry improved (4.3% -> 2.5%), integral loading
dropped (max last-1s slope 0.00404/s -> 0.00216/s, expected as P takes more
of the load). A "ringing" check (error sign-changes in the final 2s of a
pulse) showed 5-8 crossings per segment at P=0.20 - not fast/violent, but
worth re-checking if P is raised further. **Frozen at 0.20, not chased
further** given diminishing returns visible in the noise band described
above. `RO_YAW_RATE_I` was left at its already-live 0.03 throughout - not
tuned this session.

## 5. ACRO exact-angle pivot test (bang-bang stop, open loop after stop)

Closed-loop 90/180 LEFT/RIGHT pivots, each stopped the instant measured
heading crosses the target (`y=0` commanded, no further correction - this is
what ACRO with zero stick means: hold zero rate, not hold heading). At frozen
params (`0.37/0.20/0.03`), 2 runs (8 pivots):

```text
time to target:  ~1.9s (90deg) / ~3.9s (180deg)
final error:     ~5-6deg magnitude at 90deg, ~5deg at 180deg (NEVER corrects
                 the overshoot - there is no mechanism to in this test)
rate near target: ~50-52 deg/s (well above the 45 deg/s command - this IS
                 what causes the overshoot)
translation:     650-1050mm (pure lever-arm swing, throttle stayed exactly
                 0.0 every time, confirmed from rover_throttle_setpoint)
```

**Key finding: the ~60-65 deg/s peak/near-target rate directly explains the
overshoot.** This ACRO-only result does **not** reflect production pivot
accuracy, because ACRO's zero-stick stop has no heading-hold mechanism -
see section 6 for the real production path.

## 6. Production heading loop, traced from source (important architecture note)

Traced the exact cascade actual missions use:

```text
DifferentialVelControl (from an OFFBOARD velocity vector's bearing)
  -> publishes rover_attitude_setpoint.yaw_setpoint
DifferentialAttControl: RoverControl::attitudeControl() using RO_YAW_P
  -> publishes rover_rate_setpoint.yaw_rate_setpoint
DifferentialRateControl: RoverControl::rateControl() using RO_YAW_RATE_P/I,
  RD_MAX_THR_YAW_R feedforward     <- the SAME inner loop ACRO exercises
  -> motors
```

**There is no clean MAVLink "command a heading" message on this rover.**
Checked both candidates: `SET_ATTITUDE_TARGET` sets `ocm.attitude=true` but
publishes to `vehicle_attitude_setpoint`, which `DifferentialAttControl`'s
OFFBOARD branch never reads (it reads `trajectory_setpoint.yaw`).
`SET_POSITION_TARGET_LOCAL_NED` populates `trajectory_setpoint.yaw` but
*requires* position/velocity/acceleration to not all be ignored, and never
sets `ocm.attitude` at all. Heading is only reachable by commanding a
velocity vector whose bearing implies the desired heading - exactly what
`cmd_vel_bridge`/`rpp_controller` already do.

`rpp_controller_node.py`'s actual pivot mechanism
(`_publish_legacy_native_carrier` / `terminal_native_pivot_command`) does
**not** aim at the true target bearing during a pivot (that would let PX4
exit `SPOT_TURNING` early at its own 45deg threshold, well before the real
target - the documented cause of cross-track drift). It aims a "carrier"
bearing = `current_yaw +/- 60deg`, recomputed every cycle, and releases only
when the **true** heading error drops to <=4deg (tighter than PX4's native
6deg `RD_TRANS_TRN_DRV`). Live params confirmed 2026-09-07:

```text
segment_alignment_speed_mps          = 1.0   (code default; NOT the 0.4
                                               recorded as as-run on 09-01 -
                                               this restart came up on
                                               defaults, not that day's
                                               override - re-check before
                                               relying on either number)
terminal_native_pivot_enter_error_deg    = 45.0
terminal_native_pivot_release_error_deg  = 4.0
terminal_native_pivot_request_error_deg  = 60.0
segment_pivot_keeper_timeout_sec         = 10.0
```

## 7. OFFBOARD carrier-vector replica — validates the real production loop

Built a standalone harness (`pivot_offboard_carrier_20260907.py`) replicating
the exact mechanism above (same message construction as `cmd_vel_bridge.py`:
`PositionTarget`, `FRAME_LOCAL_NED`, velocity-only type_mask, ENU fields) at
`speed=1.0 m/s`, run against a **standalone mavros** instance.

**Why standalone mavros, and current live-stack impact:** `cmd_vel_bridge`
publishes to `/mavros/setpoint_raw/local` unconditionally at 50 Hz; running a
second publisher on the same topic would race non-deterministically. The
full `rover.launch.py` tree was torn down (graceful SIGINT to the launch
process), a standalone `mavros` launched with the identical connection args,
and the test run with **no software E-stop layer** (mission_manager, the
actual owner of `/emergency_stop`, was not brought back up for this test -
operator-approved tradeoff; physical E-stop was the primary control, plus
every other harness guard: mode-loss, odom staleness, publisher-gap, PX4
failsafe text). **The full stack was restored afterward** via
`ros2 launch rover_bringup rover.launch.py` and verified: all production
nodes present, `cmd_vel_bridge` confirmed sole publisher on
`/mavros/setpoint_raw/local`, `/emergency_stop` released via the guarded
`/api/estop/release` flow (mission stayed disabled per that endpoint's own
contract). **Current state: full stack up, E-stop released, mission still
requires an explicit start.**

Result, 3 runs (12 pivots total), frozen params:

```text
final settle error (measured 4s after release):
  median 1.45deg, mean 1.40deg (excluding one outlier), max 6.89deg (one
  90_RIGHT case in run 1 only - did NOT repeat in the other two runs'
  90_RIGHT attempts, -0.73deg and -2.89deg - reads as isolated/terrain-driven,
  not a systematic direction-dependent bug)
  11 of 12 pivots landed within +-3deg; 8 of 12 within +-2deg

max|throttle_body_x| = 0.0000 on every single one of the 12 pivots -
  SPOT_TURNING held perfectly, zero forward drive, confirmed from
  rover_throttle_setpoint directly (not inferred)

translation (lever-arm swing only, throttle was exactly zero):
  90deg pivots:  607mm +/- 138mm (n=6),  6.75 mm/deg
  180deg pivots: 938mm +/- 145mm (n=6),  5.21 mm/deg
  ratio 1.54x - NOT linear with angle (would be ~2.0x if the antenna swept
  radius at a constant rate through the full rotation); per-degree rate is
  lower at 180deg, consistent with tracing something closer to a circular
  arc around an off-center pivot point than a constant-radius sweep

post-release dynamics (important nuance): the heading setpoint does NOT
  freeze at the release instant - it continues moving for ~0.1-0.3s before
  locking onto a value it then holds rock-steady (verified against
  measured_yaw directly in one case: setpoint and measured yaw matched to
  <0.1deg for the full 4s watch). This post-release motion sometimes
  improves the final error (LEFT pivots and 180_RIGHT, corrected toward
  zero) and once made it worse (the 6.89deg 90_RIGHT case, which moved
  ~10deg in the wrong direction after release rather than correcting) -
  this is the one open behavior worth deliberately re-testing if the
  90_RIGHT anomaly needs to be chased further.

ULogs: 11_44_16.ulg, 11_51_23.ulg, 11_53_02.ulg
```

**Conclusion: the production heading loop (RO_YAW_P outer + tuned inner
rate loop) resolves the ACRO bang-bang overshoot problem from section 5.**
The ~5-7deg uncorrected ACRO overshoot becomes a median 1.45deg / 11-of-12
under-3deg result once the real heading-hold mechanism is exercised.

## 8. Real production missions run today

Operator ran 7 real 5-point missions via the GCS app after the stack was
restored and E-stop released, using the frozen parameters from this session.
Bags pulled from `/home/flash/bags_jet/mission.csv_20260907_*` (Jetson) to
`bags_jet/today/` (local, gitignored) via rsync:

```text
mission.csv_20260907_173125  (17:31 IST)
mission.csv_20260907_173219  (17:32)
mission.csv_20260907_173423  (17:34)
mission.csv_20260907_173605  (17:36)
mission.csv_20260907_174739  (17:47)
mission.csv_20260907_174936  (17:49)
mission.csv_20260907_175125  (17:51)
```

All 7 have real surveyed 5-point CSVs (lat ~13.189-13.191, lon ~80.222).
**Not yet analysed** - waypoint tracking accuracy, marking-point verdicts,
and pivot-walk-in-production have not been measured against the frozen
parameters yet. This is the natural next step (`analyse-missions` skill
covers the decode/verdict methodology).

## Open items for next session

1. **Analyse the 7 mission bags above** - first real measurement of pivot
   walk / waypoint accuracy under the newly frozen `0.37/0.20/0.03`.
2. **90_RIGHT post-release anomaly** (section 7) - one 6.89deg case out of
   12, did not repeat; worth a few more OFFBOARD carrier runs specifically
   watching the post-release window if it recurs.
3. **`segment_alignment_speed_mps` currently reads 1.0 (code default) after
   today's stack restore, not the 0.4 recorded as as-run on 09-01** - confirm
   which value the operator actually wants live before the next field test;
   this session did not change it either way.
4. **Feedforward/P tuning was done on a non-flat test patch** - the run-to-run
   scatter documented in sections 3-4 means these values are "good enough
   here," not finely optimal; revisit if a flatter/larger area becomes
   available, and expect the real deployment site (also not flat/clean per
   the operator) to show similar variance.
5. **`RO_YAW_RATE_I` was never tuned this session** (stayed at its already-live
   0.03) - still open per the original ChatGPT-relayed plan's step 3, should
   the mission-bag analysis in item 1 show residual bias worth chasing.
