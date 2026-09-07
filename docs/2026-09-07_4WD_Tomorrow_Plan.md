# 4WD Rover — Tomorrow Plan
**Date:** 2026-09-07

**Branch:** `feat/rtk-injection-v2`

## Current handoff state

- Production task sheet remains the authority.
- Task 2.1 GNSS/EKF noise measurement is complete.
- No EKF2 tuning parameter has been changed yet.
- `radial20` remains the proven stop authority.
- Runtime path is allowed only for `C → P1`; all later legs remain fixed mission geometry.
- Current firmware Offboard velocity mode still derives heading from the velocity vector and ignores explicit yaw.
- Current native pivot is entered when heading error exceeds `RD_TRANS_DRV_TRN`; it exits when error falls below `RD_TRANS_TRN_DRV`.
- Current mixer is:
  ```text
  Motor1 = throttle - speed_diff
  Motor2 = throttle + speed_diff
  ```
- Physical Motor1/Motor2 side mapping still needs bench confirmation.

## Geometry decided today

Mission/reference point:

```text
Master GNSS antenna = nozzle = desired marking position
```

Measured horizontal geometry:

```text
Master/nozzle → IMU/FCU = +0.200 m forward
Master/nozzle → physical pivot centre ≈ +0.410 m forward
Wheel track ≈ 0.640 m
Y offsets ≈ 0
```

Candidate EKF horizontal geometry to validate:

```text
EKF2_GPS_POS_X = 0.000
EKF2_GPS_POS_Y = 0.000

EKF2_IMU_POS_X = +0.200
EKF2_IMU_POS_Y = 0.000
```

Do **not** put the 0.410 m physical pivot offset into `EKF2_GPS_POS_X`.

Z offsets are still unverified and must be measured physically.

---

# Tomorrow execution order

## T1 — Finish physical geometry
Measure and record:

```text
EKF2_IMU_POS_Z
EKF2_GPS_POS_Z
secondary GNSS antenna XYZ relative to nozzle/master reference
exact master↔secondary antenna baseline
actual wheel track
```

**Pass:** one signed XYZ drawing using PX4 axes: `+X forward, +Y right, +Z down`.

---

## T2 — Apply only lever-arm candidate
Change only:

```text
EKF2_IMU_POS_X = +0.200
EKF2_IMU_POS_Y = 0.000
EKF2_GPS_POS_X = 0.000
EKF2_GPS_POS_Y = 0.000
```

Set Z only after T1 measurement.

Save pre-change and post-change parameter files.

**Do not change** `EKF2_GPS_P_NOISE`, `EKF2_GPS_DELAY`, TAU, yaw gains, or pivot gains in this step.

---

## T3 — Static estimator test
**2 runs × 10 minutes**

### S1
- warm rover
- RTK FIXED
- untouched for 10 min

### S2
- complete PX4 + GNSS power cycle
- reacquire RTK FIXED
- untouched for 10 min

Record at minimum:

```text
vehicle_gps_position
vehicle_local_position
vehicle_global_position
vehicle_attitude
vehicle_angular_velocity
estimator_status
estimator_status_flags
estimator_gps_status
estimator_aid_src_gnss_pos
estimator_aid_src_gnss_vel
estimator_aid_src_gnss_yaw
estimator_aid_src_gnss_hgt
estimator_states
```

Calculate:

```text
GNSS horizontal/vertical sigma
EKF position sigma
GNSS yaw sigma
innovation/test ratios
0.1/0.5/1/2/5/10/20/30/60 s averaging behaviour
power-cycle position repeatability
```

**Decision:** only after this can `EKF2_GPS_P_NOISE=0.01` be judged.

---

## T4 — Prove the 1.72 s EKF post-stop movement
Use existing ULogs first; no new rover test required if fields exist.

Compare for every stop:

```text
raw GNSS position
EKF fused/local position
EKF velocity
output_tracking_error velocity
output_tracking_error position
motor-neutral timestamp
gyro yaw-rate
```

Verify whether the ~1.72 s median settling follows the EKF output predictor.

Current parameters to audit:

```text
EKF2_TAU_POS = 0.25
EKF2_TAU_VEL = 0.25
EKF2_GPS_DELAY = 50 ms
```

**Pass:** identify the source of the post-stop transient before changing TAU or delay.

---

## T5 — Pivot geometry validation
Run **6 pivot tests**:

```text
90° CW  × 2
90° CCW × 2
180° CW × 1
180° CCW × 1
```

Start each from a clearly marked fixed location.

Log:

```text
raw master-GNSS path
vehicle_local_position
yaw
yaw rate
rover attitude setpoint
rover rate setpoint
steering speed_diff
actuator_motors
```

Measure separately:

```text
physical nozzle arc
estimated nozzle/reference movement
heading overshoot
yaw-rate settling
final raw-GNSS position
```

Important: the nozzle is ~0.410 m behind the physical pivot centre, so nozzle motion during a spot-turn is real mechanical motion. Do not label that entire arc as estimator drift.

---

## T6 — Bench motor-side verification
With wheels safely unloaded / rover secured, prove:

```text
logical Motor1 → physical left or right side
logical Motor2 → physical left or right side
PWM1 destination
PWM2 destination
PWM3 destination
PWM4 destination
positive yaw command → actual yaw direction
neutral = 1500 us
min/max and reversible direction
```

No mixer sign patch until this table is proven.

---

# Do not do tomorrow before T1–T6 evidence

Do not:

```text
change EKF2_GPS_P_NOISE
change EKF2_TAU_POS / TAU_VEL
change EKF2_GPS_DELAY
retune yaw-rate controller
enable disabled precision-pivot states
add reanchor after P1
change motor mixer signs
```

---

# End-of-day deliverables

1. Final signed rover geometry sheet.
2. Pre/post EKF parameter files.
3. Two 10-minute static ULogs.
4. EKF post-stop predictor verdict.
5. Six pivot-test ULogs + summary table.
6. Motor1/Motor2/PWM physical mapping table.
7. Decision sheet:
   ```text
   EKF lever arm: PASS / FAIL
   GPS_P_NOISE: READY / HOLD
   GPS delay: READY / HOLD
   TAU_POS/VEL: READY / HOLD
   pivot geometry: PASS / FAIL
   motor mapping: PASS / FAIL
   ```

## Tomorrow success condition

Tomorrow is successful if we finish with the estimator geometry and pivot physics **proven**, not merely tuned:

```text
Nozzle/master = mission reference
IMU lever arm verified
static RTK correlation measured
1.72 s EKF settling source identified
pivot motion separated into real nozzle arc vs estimator error
motor mapping proven
```

Only after that should the next firmware work begin: **velocity + explicit-yaw Offboard contract**, then production pivot lifecycle and fixed-leg reacquisition.
