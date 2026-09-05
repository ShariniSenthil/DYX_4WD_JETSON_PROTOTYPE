# 4WD CM Tracking — Production Task Sheet

**Goal:** Close centimetre-level localization, straight tracking, pivot accuracy, and final stop accuracy without relying on unproven disabled control states or runtime-path workarounds.

---

# P0 — Rules

- Use only source-verified behavior.
- Do not enable disabled precision states just because they exist.
- Do not create duplicate controllers.
- `radial20` stop logic is proven and remains the terminal stop authority.
- Runtime path is allowed only for **C → P1 entry**.
- After P1, every `P* → P*` leg must use its own fixed mission geometry.
- Reanchor is diagnostic/recovery evidence only, **not** the production solution for pivot walk.
- Final physical accuracy authority remains raw RTK vs surveyed target.

---

# P1 — EKF2 Localization / Geometry

## Task 1.1 — Mechanical geometry survey

Measure from one common rover body reference, preferably the true pivot/rotation center:

```text
IMU XYZ
GNSS master antenna XYZ
GNSS secondary antenna XYZ
effective wheel track
```

PX4 body convention:

```text
+X forward
+Y right
+Z down
```

## Task 1.2 — Configure lever arms

Current horizontal lever arms are effectively zero.

Review and set from measured geometry:

```text
EKF2_IMU_POS_X/Y/Z
EKF2_GPS_POS_X/Y/Z
```

Do not copy old offsets such as `-0.1 m` without measuring this chassis.

## Task 1.3 — Verify estimator output point

Confirm that local position used by control represents the intended body reference during:

```text
stationary
straight drive
90° pivot
180° pivot
```

Pass condition:

```text
pure rotation does not create false body-reference translation beyond expected mechanical/RTK noise
```

## Task 1.4 — Review localization source quality / open items

Verify:

```text
GNSS position aid
GNSS velocity aid
GNSS yaw aid
height reference
sensor-delay handling
lever-arm application
reset handling
innovation rejection
yaw-align state
```

Fix only source-proven defects.

---

# P2 — EKF2 Noise Measurement + Parameter Tune

## Task 2.1 — Measure before tuning

Collect stationary RTK FIXED data and calculate:

```text
horizontal raw GNSS sigma
vertical raw GNSS sigma
velocity sigma
GNSS yaw sigma
EKF local-position sigma
yaw sigma
innovation/test-ratio distributions
```

## Task 2.2 — Tune from measured noise

Review:

```text
EKF2_GPS_P_NOISE
EKF2_GPS_P_GATE
EKF2_GPS_DELAY
EKF2_REQ_GPS_H
EKF2_REQ_EPV
GNSS yaw noise/gate settings
```

Do not tighten noise below measured receiver behavior.

## Task 2.3 — Production acceptance

Require:

```text
no repeated GNSS-yaw rejection
no unexplained yaw-align drop
no position jump during stationary pivot
stable raw/fused relationship
```

---

# P3 — Current EKF2 Unresolved Bugs / Risks

Keep these open until closed by evidence:

```text
1. Power-cycle-cleared raw GNSS vertical offset (~125–152 mm).
2. Stable bad GNSS solution can still appear RTK FIXED and healthy.
3. GNSS is height reference, so a wrong but stable datum can become estimator authority.
4. EKF2_REQ_GPS_H = 1 s is aggressive.
5. Horizontal IMU/GNSS lever-arm geometry is not yet configured.
6. GNSS-yaw rejection behavior on the current receiver is not yet fully audited.
7. Receiver-native ambiguity/base/correction state is not sufficiently logged.
```

Do not hide these with controller tuning.

---

# P4 — PX4 ↔ ROS2 OFFBOARD Contract

Use one explicit command contract.

## Straight tracking

ROS2 sends:

```text
velocity + explicit yaw
```

PX4 receives:

```text
trajectory_setpoint.velocity
trajectory_setpoint.yaw
```

Firmware contract:

```text
speed authority   <- velocity magnitude
heading authority <- explicit yaw
```

Fallback:

```text
if yaw invalid:
    derive bearing from velocity vector
```

## Pivot

ROS2 sends:

```text
zero or near-zero translation
explicit target yaw
```

PX4 must rotate under yaw control without requiring a fake moving vector.

## Return to straight tracking

After pivot settle:

```text
restore translational velocity
keep explicit straight-line yaw authority
```

### MAVROS type-mask contract

Required modes:

```text
STRAIGHT:
velocity + yaw

PIVOT:
yaw authority with translation held zero

STOP:
zero velocity with held yaw
```

Do not add yaw-rate feedforward yet unless a true geometric curvature feedforward is later required.

---

# P5 — ROS2 Tracking Weakness + Upgrade

Current weakness:

```text
guidance active
but production tracking authority is mostly generic pure-pursuit
and the stronger recovery/pivot states are disabled/unproven
```

Do not simply enable them.

## Required production states

Use a small proven FSM:

```text
ALIGN
TRACK
RECOVERY
APPROACH
STOP
SETTLE
HOLD
```

### TRACK

Inputs:

```text
fixed leg geometry
projection
signed cross-track
path heading
current yaw
distance remaining
```

Outputs:

```text
speed_cmd
explicit_yaw_cmd
```

### RECOVERY

Enter only when real xtrack/heading thresholds are exceeded.

Goal:

```text
return to the same fixed leg geometry
```

No runtime replacement line.

### APPROACH

Preserve lateral correction while longitudinal speed decelerates.

### STOP

Use the proven `radial20` stopping logic.

---

# P6 — Pivot Mechanism — Highest Control Priority

Pivot is a first-class maneuver, not a side effect of velocity-vector heading.

## Required pivot sequence

```text
BRAKE_TO_POINT
→ VERIFY_STOP
→ PIVOT
→ VERIFY_HEADING
→ VERIFY_YAW_RATE
→ VERIFY_POSITION
→ RELEASE_TO_NEXT_FIXED_LEG
```

## Pivot release certificate

Require all:

```text
position within allowed radius
linear speed below threshold
yaw rate below threshold
heading within tolerance
held continuously for settle time
```

## Pivot-rate control

Review/tune:

```text
RO_YAW_P
RO_YAW_RATE_P
RO_YAW_RATE_I
RO_YAW_RATE_LIM
RD_MAX_THR_YAW_R
RD_WHEEL_TRACK
```

Current high pivot rate must be reduced from measured command/response evidence, not arbitrary motor scaling.

## Pivot geometry pass condition

After lever-arm correction:

```text
body reference remains near the pivot point
heading converges without overshoot
yaw rate settles before forward release
```

---

# P7 — Remove Reanchor as Production Dependency

Reanchor helped diagnose pivot walk but is not the final architecture.

Production rule:

```text
C → P1:
runtime entry path allowed

P1 → P2 → P3 → ...:
use original fixed mission-leg geometry only
```

After pivot:

```text
controller must reacquire and track the existing next leg
```

Do not generate:

```text
current_pose → next_point replacement line
```

as normal behavior.

If pivot drift exceeds tolerance:

```text
fail / recover position
then return to the original mission leg
```

---

# P8 — Motor Mixing + Command Verification

Current logical mixer:

```text
motor_A = throttle - speed_diff
motor_B = throttle + speed_diff
```

Verify physical mapping before any sign patch.

## Bench verification

Confirm:

```text
Motor1 physical side
Motor2 physical side
PWM1/2/3/4 physical destination
left/right Sabertooth duplication
positive yaw command physical direction
neutral PWM
min/max PWM
reversible flags
left/right full-scale symmetry
```

## Command chain verification

Log and compare:

```text
ROS2 speed_cmd
ROS2 yaw_cmd
PX4 trajectory_setpoint
differential velocity setpoint
rover attitude setpoint
rover rate setpoint
steering speed difference
actuator motor outputs
actual yaw
actual yaw rate
actual speed
```

Pass condition:

```text
sign and magnitude are consistent end-to-end
```

---

# P9 — Stop Accuracy

Keep proven authority:

```text
radial20
```

FSM terminal sequence:

```text
APPROACH
→ radial20 deceleration
→ exact radial capture
→ zero command
→ stationary settle
→ HOLD
```

Keep the proven braking-margin behavior unless new data disproves it.

Final physical accuracy:

```text
raw RTK vs surveyed target
```

Required reported values:

```text
along error
cross error
radial error
```

---

# Final Implementation Order

```text
1. Measure rover geometry.
2. Configure EKF lever arms.
3. Measure EKF/GNSS noise.
4. Tune EKF parameters from data.
5. Close open EKF/GNSS integrity issues.
6. Implement velocity + explicit-yaw OFFBOARD contract.
7. Build minimal straight-line tracking FSM on fixed leg geometry.
8. Build/verify proper pivot lifecycle.
9. Remove reanchor/runtime-line dependency after P1.
10. Verify motor mixing and signs.
11. Integrate proven radial20 stop into the FSM.
12. Field validate straight → pivot → straight → radial20 stop.
```

---

# Production Acceptance

The rover is ready only when one continuous mission proves:

```text
C → P1 runtime entry
P1 → P2 fixed geometry
pivot at P2
P2 → P3 fixed geometry
...
```

with:

```text
stable estimator
correct lever-arm geometry
bounded pivot walk
explicit yaw tracking
cross-track convergence
correct motor signs
radial20 final stop
raw-RTK cm-level physical accuracy
```
