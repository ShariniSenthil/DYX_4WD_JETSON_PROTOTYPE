# 4WD EKF / GNSS / Hardware Audit — 2026-09-04

## Scope

This note records the firmware, estimator, GNSS, board, and drivetrain findings from the 2026-09-04 audit session for the 4WD marking rover.

Primary firmware repository:

- `ShariniSenthil/DYX_4WD_PX4_FIRMWARE_16.2`

3WD reference repository:

- `Vetri2425/PX4-Autopilot`

Target problem investigated:

- A pre-power-cycle GNSS altitude solution was consistently about 125–152 mm lower than later post-power-cycle runs while the GNSS receiver still reported RTK FIXED.
- The horizontal false-position hypothesis for `log_82` was separately rejected; `log_82` horizontal error is ordinary controller/tracking/stop error.

---

# 1. Firmware identity

## log_82 firmware

`log_82_2026-9-3-17-13-56.ulg` used:

```text
54f0455ffcd755534539a7cf33a09a20bf71d29d
```

Commit:

```text
fix(ekf2): break unbounded recursion
```

The recursion fix addresses EKF covariance-state robustness and is not a direct explanation for the stable reboot-cleared GNSS altitude offset.

## Current 4WD firmware

Current repo HEAD checked during this session:

```text
c7ccc74d310d94a125e89b8e7621dd7d02252f77
```

The current HEAD is one production commit ahead of the `log_82` firmware. That extra commit does not change EKF2, so the EKF2 implementation relevant to the anomaly is effectively unchanged.

---

# 2. Verified log_82 / power-cycle anomaly

The verified vertical anomaly is:

- Before power cycle:
  - approximately `-131 mm`
  - approximately `-152 mm`
  - approximately `-125 mm`
- After power cycle:
  - approximately `-2 mm` to `+52 mm`

At four physical piles shared between `log_82` and later runs, `log_82` was lower by:

```text
139 mm
154 mm
155 mm
160 mm
```

GNSS receiver status remained apparently healthy:

- `fix_type = 6`
- RTK FIXED
- ~17 satellites
- `h_acc ≈ 15 mm`
- no spoofing/jamming indication
- no receiver health failure reported

This proves a persistent, confidently reported absolute vertical error existed before reboot.

It does **not** prove EKF2 generated that error because the raw GNSS solution itself was shifted.

---

# 3. Horizontal false-position result

For `log_82`, the horizontal transient/local-frame false-position hypothesis was rejected.

Terminal raw target radial errors:

```text
P1 56.7 mm
P2 82.1 mm
P3 24.4 mm
P4 16.3 mm
```

RPP/EKF terminal radial errors:

```text
P1 53.9 mm
P2 84.7 mm
P3 36.6 mm
P4 15.4 mm
```

No terminal point satisfied:

```text
RPP/EKF <= 20 mm
AND
raw surveyed-target error > 50 mm
```

Full-mission settled search also found zero false-reach samples.

Conclusion:

```text
log_82 horizontal issue = ordinary controller/tracking/stop error
```

The vertical reboot-cleared GNSS anomaly remains a separate receiver-integrity investigation.

---

# 4. Exact EKF2 architecture found in firmware

Relevant source paths:

```text
src/modules/ekf2/EKF/aid_sources/gnss/gps_checks.cpp
src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp
src/modules/ekf2/EKF/aid_sources/gnss/gnss_height_control.cpp
src/modules/ekf2/EKF/height_control.cpp
src/modules/ekf2/EKF/bias_estimator/height_bias_estimator.hpp
```

## 4.1 GNSS quality checks

`runGnssChecks()` checks:

- fix type
- satellite count
- PDOP
- receiver-reported horizontal accuracy
- receiver-reported vertical accuracy
- speed accuracy
- horizontal drift
- vertical drift
- horizontal speed offset
- vertical speed offset
- spoofing flag

Important limitation:

A stable absolute offset is not detectable if the receiver itself reports the solution as healthy.

Example:

```text
true altitude      100.000 m
GNSS altitude       99.850 m
error              -0.150 m

fix_type              6
vacc                   good
vertical drift         ~0
```

The vertical drift check only detects change between consecutive GNSS altitude samples. A constant 150 mm bias can therefore remain invisible.

## 4.2 GNSS height-reference initialization

When GNSS height fusion starts and GNSS is the configured height reference, EKF2 follows the path:

```text
_height_sensor_ref = GNSS
reset_hgt_to_gps = true
initialiseAltitudeTo(GNSS measurement)
GNSS height bias estimator reset
```

This means a bad-but-stable GNSS altitude present during initialization can become the EKF altitude datum.

EKF2 does not create the original receiver error in this scenario; it accepts it as the reference.

## 4.3 Height-bias estimator behavior

The height bias estimator only adapts a sensor when that sensor is **not** the current height reference.

Therefore:

```text
if height reference = GNSS
```

GNSS does not estimate away its own constant height bias.

Other height sources can instead be biased relative to the GNSS-defined datum.

This makes a wrong GNSS reference internally self-consistent once accepted.

## 4.4 Height reference fallback

`checkHeightSensorRefFallback()` returns immediately when `_height_sensor_ref != UNKNOWN`.

Therefore a stable but wrong GNSS source that continues passing its checks can remain the active height reference; fallback does not continuously search for a “better” source.

---

# 5. Exact as-run EKF2 parameters from uploaded parameter file

Critical parameters found:

```text
EKF2_GPS_CTRL      = 15
EKF2_HGT_REF       = 1
EKF2_BARO_CTRL     = 1
EKF2_RNG_CTRL      = 0
EKF2_GPS_CHECK     = 831
EKF2_REQ_EPV       = 5.0
EKF2_REQ_GPS_H     = 1.0
EKF2_GPS_P_NOISE   = 0.01
EKF2_GPS_P_GATE    = 5.0
EKF2_GPS_DELAY     = 50 ms
EKF2_GPS_POS_Z     = -0.30 m
```

## 5.1 `EKF2_GPS_CTRL = 15`

Enabled:

- horizontal GNSS position
- GNSS altitude
- GNSS 3D velocity
- dual-antenna GNSS heading

GNSS vertical fusion was definitely enabled.

## 5.2 `EKF2_HGT_REF = 1`

Configured height reference:

```text
GNSS
```

Therefore GNSS was intended to be the long-term vertical authority.

## 5.3 `EKF2_GPS_CHECK = 831`

Enabled bits:

```text
0,1,2,3,4,5,8,9
```

Disabled:

```text
bit 6 = vertical position drift
bit 7 = horizontal speed offset
```

The disabled vertical-drift check is notable, but even if enabled it would not catch a constant 150 mm vertical bias.

## 5.4 `EKF2_REQ_GPS_H = 1 s`

Only one second of continuously healthy GNSS was required before acceptance.

Firmware default is 10 s.

This is an aggressive production configuration.

## 5.5 `EKF2_GPS_P_NOISE = 0.01 m`

GNSS position noise floor was set to 1 cm.

GNSS height observation noise is computed using:

```text
max(receiver vacc, 1.5 * EKF2_GPS_P_NOISE)
```

So the configured floor contributes only:

```text
15 mm
```

If the receiver reports approximately 15–20 mm accuracy while the actual absolute error is approximately 150 mm, EKF receives a highly confident but wrong measurement.

If that bad measurement defines the initial datum, the innovation can remain near zero.

---

# 6. EKF2 audit verdict

The current evidence supports a two-layer failure model.

## Layer 1 — upstream GNSS / RTK root-source fault

The raw GNSS solution itself moved by approximately 150 mm across the power cycle.

Therefore the original absolute vertical error existed upstream of EKF2.

Primary suspects include:

- Septentrio receiver navigation state
- RTK ambiguity state
- correction/reference/base state
- receiver-side PVT solution state
- NTRIP/base-reference changes

## Layer 2 — EKF2 protection / integrity weakness

The EKF2 configuration and source allow a bad-but-stable GNSS solution to become the authoritative height datum because:

- GNSS altitude fusion is enabled
- GNSS is the configured height reference
- only 1 s of healthy GNSS is required
- GNSS self-reported accuracy is trusted
- there is no independent absolute-height check
- the reference sensor does not estimate its own height bias
- stable absolute bias does not trigger drift checks

Verdict:

```text
Receiver likely generated the bad altitude.
EKF2 then had no independent mechanism to reject that stable, confident GNSS datum.
```

---

# 7. Board identity correction

The active flight controller is:

```text
Pixhawk 6X
PX4 target family: px4/fmu-v6x
```

The firmware repo contains:

```text
boards/px4/fmu-v6x/
```

including:

```text
default.px4board
differential.px4board
```

Do not use Cube Orange Plus assumptions for this rover.

The uploaded `.params` file by itself does not encode a human-readable board model name, so board identity should be verified from ULog system metadata / build target when needed.

---

# 8. Septentrio mosaic-H

The production receiver is:

```text
Septentrio mosaic-H
```

The Pixhawk 6X default build enables:

```text
CONFIG_DRIVERS_GNSS_SEPTENTRIO=y
```

The dedicated driver lives at:

```text
src/drivers/gnss/septentrio/
```

Important driver components include:

```text
module.yaml
rtcm.cpp
rtcm.h
sbf/
```

This is the correct upstream GNSS path to audit, not the u-blox F9P/UBX driver.

## 8.1 Relevant Septentrio parameters

Observed / relevant parameters:

```text
SEP_PORT1_CFG
SEP_PORT2_CFG
SEP_STREAM_MAIN
SEP_STREAM_LOG
SEP_OUTP_HZ
SEP_YAW_OFFS
SEP_PITCH_OFFS
SEP_SAT_INFO
SEP_DUMP_COMM
SEP_AUTO_CONFIG
SEP_CONST_USAGE
SEP_LOG_HZ
SEP_LOG_LEVEL
SEP_LOG_FORCE
SEP_HARDW_SETUP
```

Current important settings observed:

```text
SEP_PORT1_CFG    = 102
SEP_PORT2_CFG    = 0
SEP_AUTO_CONFIG  = 1
SEP_OUTP_HZ      = 1
SEP_STREAM_MAIN  = 1
SEP_STREAM_LOG   = 2
SEP_CONST_USAGE  = 0
SEP_HARDW_SETUP  = 0
SEP_SAT_INFO     = 0
SEP_DUMP_COMM    = 0
```

`SEP_OUTP_HZ = 1` corresponds to 10 Hz primary SBF PVT output.

`SEP_DUMP_COMM = 0` means raw Septentrio receiver communications were not logged.

That limits our ability to reconstruct receiver-native ambiguity/base/reference state from old ULogs.

---

# 9. Drivetrain / motor-controller architecture

Physical drivetrain clarified during this session:

```text
4 motors
2 dual-channel Sabertooth motor controllers
```

Layout:

```text
LEFT SIDE
Sabertooth #1
  CH1 -> Left Front motor
  CH2 -> Left Rear motor

RIGHT SIDE
Sabertooth #2
  CH1 -> Right Front motor
  CH2 -> Right Rear motor
```

Therefore:

```text
2 controllers x 2 channels = 4 motors
```

## 9.1 No dedicated Sabertooth PX4 driver found

No native module or board configuration named `sabertooth` was found in the current 4WD firmware repo.

The Sabertooth controllers are used through standard PX4 actuator/PWM output mapping.

## 9.2 Current 4WD motor-function mapping

Observed mapping:

```text
PWM_MAIN_FUNC1 = 101
PWM_MAIN_FUNC2 = 102
PWM_MAIN_FUNC3 = 101
PWM_MAIN_FUNC4 = 102
```

Logical interpretation:

```text
PWM1 -> Motor 1 / left command
PWM2 -> Motor 2 / right command
PWM3 -> Motor 1 / left command
PWM4 -> Motor 2 / right command
```

So one left command is duplicated to both left-side motor channels and one right command is duplicated to both right-side motor channels.

## 9.3 RoboClaw build flag

The current `fmu-v6x/differential.px4board` contains:

```text
CONFIG_DRIVERS_ROBOCLAW=y
```

This means the RoboClaw driver is compiled into the firmware.

It does **not** mean RoboClaw is the physical production motor controller.

For the current 4WD rover, RoboClaw should be treated as unrelated compiled baggage unless runtime configuration proves it is started.

---

# 10. 3WD reference — Vetri2425/PX4-Autopilot

The 3WD reference repo contains explicit Sabertooth-oriented comments inside:

```text
src/modules/rover_differential/RoverDifferential.cpp
```

The source defines:

```text
control[0] = left motor
control[1] = right motor
```

and comments:

```text
Sabertooth CH1 = left
Sabertooth CH2 = right
```

The inverse kinematics are:

```text
left  = throttle + speed_diff
right = throttle - speed_diff
```

This confirms the 3WD implementation also used standard rover actuator outputs for Sabertooth control rather than a dedicated Sabertooth PX4 driver.

The 3WD code also contains a custom manual tank-mode path where motor outputs are driven directly from throttle/steering inputs.

---

# 11. Corrected production hardware context

Use this context going forward:

```text
Flight controller:
Pixhawk 6X / px4_fmu-v6x

GNSS:
Septentrio mosaic-H
Dedicated PX4 Septentrio driver
SBF protocol
10 Hz main PVT output
automatic receiver configuration enabled

Estimator:
EKF2
GNSS horizontal + vertical + velocity + dual-antenna heading enabled
GNSS configured as height reference

Drivetrain:
4WD differential rover
4 motors
2 dual-channel Sabertooth controllers
left controller drives both left motors
right controller drives both right motors
Sabertooth controlled through PX4 PWM actuator outputs
no native Sabertooth PX4 driver found
```

---

# 12. Open investigations after this session

## A. Septentrio receiver-native root cause

Audit the exact mosaic-H SBF path:

- which SBF PVT block provides:
  - latitude
  - longitude
  - altitude
  - ellipsoid altitude
  - covariance / accuracy
  - RTK state
  - solution mode
  - correction age
  - base/reference identifiers
  - ambiguity/fix metadata
- which fields PX4 publishes into `sensor_gps`
- which fields are discarded
- which receiver states are cleared by:
  - PX4 reboot only
  - receiver reboot
  - full power cycle

Goal:

```text
explain how a stable wrong absolute vertical solution can remain RTK FIXED
and then disappear after power cycle
```

## B. EKF integrity hardening

Do not patch yet.

Audit potential production safeguards around:

```text
EKF2_HGT_REF
EKF2_REQ_GPS_H
EKF2_REQ_EPV
EKF2_GPS_P_NOISE
GNSS-vs-baro consistency
startup GNSS acceptance
height-reference observability
```

## C. 4WD actuator path

Verify exact current 4WD path:

```text
rover_differential
-> actuator_motors[0/1]
-> PWM output mapping
-> duplicated left/right channels
-> two Sabertooth controllers
-> four motors
```

Confirm:

- inversion
- deadband
- trim
- scaling
- min/max PWM
- reversible flags
- left/right symmetry
- whether unused RoboClaw code can be removed from production build

---

# Final session verdict

### Horizontal log_82 issue

```text
ORDINARY CONTROLLER / TRACKING / STOP ERROR
```

### Vertical reboot-cleared anomaly

```text
REAL RECEIVER-NATIVE ABSOLUTE GNSS ANOMALY
ROOT CAUSE NOT YET PROVEN
```

### EKF2 role

```text
EKF2 DID NOT CREATE THE ORIGINAL RAW GNSS OFFSET,
BUT IT WAS CONFIGURED TO TRUST GNSS AS THE HEIGHT REFERENCE
AND HAD NO INDEPENDENT PROTECTION AGAINST A STABLE,
CONFIDENTLY REPORTED ABSOLUTE GNSS BIAS.
```

### Current hardware baseline

```text
Pixhawk 6X
Septentrio mosaic-H
2 x dual-channel Sabertooth
4 motors
PX4 differential rover
PWM actuator-output Sabertooth control
```
