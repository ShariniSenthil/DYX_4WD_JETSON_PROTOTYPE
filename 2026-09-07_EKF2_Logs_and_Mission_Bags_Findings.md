# 4WD EKF2 Estimator Findings — Logs + Mission Bags

**Date:** 2026-09-07  
**Scope:** PX4 EKF2 horizontal-position behavior, GNSS raw vs fused position, power-cycle comparison, and estimator health during the 7 production missions.

---

## 1. Executive Verdict

There are **two different EKF2 situations** in today's data:

1. **Before mission execution**, EKF2 did show a real failure/recovery sequence:
   - GNSS position aiding was interrupted.
   - EKF2 entered dead-reckoning.
   - The fused position moved about **1.7–2.3 m away from raw RTK GNSS**.
   - When GNSS aiding recovered, EKF2 corrected/reset its horizontal position back toward GNSS.

2. **During the actual completed mission runs**, EKF2 was healthy:
   - Absolute horizontal position stayed valid.
   - No GPS-glitch estimator flag was recorded.
   - No metre-scale local-position jump/reset occurred.
   - Raw RTK vs fused position was typically about **11 mm apart while stopped**.
   - The larger 25–55 mm differences occurred mainly while the rover was moving and behaved like normal estimator/filter lag, not accumulating drift.

**Important:** The power cycle reset EKF2 state and counters, but it did **not permanently remove the underlying intermittent GNSS-aiding problem**. A similar GNSS-quality/fusion interruption occurred again after reboot before the final three good missions.

---

# 2. Timeline

## Before Power Cycle

### ~17:30 — EKF2 failure/recovery before Mission 1

Two major horizontal EKF position corrections were recorded immediately before the first mission.

| Time | EKF event | Horizontal correction | Raw RTK ↔ fused difference before correction |
|---|---|---:|---:|
| **17:30:28.799** | Position reset count **7 → 8** | **1.783 m** | ~**1.81 m** |
| **17:30:51.009** | Position reset count **8 → 9** | **1.631 m** | ~**1.72 m** |

Reset vectors:

```text
17:30:28.799
delta_xy ≈ (+1.478 m, -0.998 m)

17:30:51.009
delta_xy ≈ (+0.821 m, +1.409 m)
```

During this pre-mission period:

- GNSS position fusion was not continuously active.
- EKF2 entered inertial/dead-reckoning mode.
- Horizontal GNSS innovation became very large.
- GNSS position innovation test ratio reached approximately **27.2**.
- Horizontal position reset counter reached **9**.
- After GNSS fusion recovered, raw-vs-fused separation collapsed from metre-level error back toward centimetres.

### Interpretation

This is a **real EKF2 estimator drift/reset event**.

The RTK raw GNSS did not move 1–2 m away.  
The **fused EKF position** drifted away while GNSS aiding was unavailable/rejected, then corrected when GNSS was accepted again.

---

# 3. Mission Sequence

Seven production mission bags were recorded:

| Mission | Mission bag time | Power state | Mission result |
|---|---|---|---|
| M1 | **17:31:25** | Before power cycle | Paused / aborted |
| M2 | **17:32:19** | Before power cycle | Complete |
| M3 | **17:34:23** | Before power cycle | Complete |
| M4 | **17:36:05** | Before power cycle | Complete |
| — | **~17:40** | **PX4 power cycle** | EKF2 restarted |
| M5 | **17:47:39** | After power cycle | Complete |
| M6 | **17:49:36** | After power cycle | Complete |
| M7 | **17:51:25** | After power cycle | Complete |

---

# 4. EKF2 During Each Mission Bag

The mission bags contain:

- `/mavros/gpsstatus/gps1/raw`
- `/mavros/global_position/raw/fix`
- `/mavros/global_position/global`
- `/mavros/local_position/odom`
- `/mavros/estimator_status`

Important finding:

`/mavros/gpsstatus/gps1/raw` and `/mavros/global_position/raw/fix` had the **same latitude/longitude sample-for-sample** in these bags.

Therefore the meaningful position comparison is:

```text
Raw RTK GNSS
    vs
PX4 EKF2 fused global position
```

---

## Mission-by-Mission Raw vs Fused Position

| Mission | Power state | Stationary median | Stationary P95 | Moving median | Overall max |
|---|---|---:|---:|---:|---:|
| M1 17:31:25 | Before | **24.3 mm** | 44.2 mm | 32.5 mm | 98.2 mm |
| M2 17:32:19 | Before | **10.8 mm** | 32.5 mm | 43.4 mm | 87.4 mm |
| M3 17:34:23 | Before | **11.1 mm** | 24.3 mm | 43.4 mm | 123.6 mm |
| M4 17:36:05 | Before | **11.1 mm** | 32.5 mm | 24.3 mm | 84.0 mm |
| M5 17:47:39 | After | **10.8 mm** | 30.1 mm | 44.7 mm | 97.6 mm |
| M6 17:49:36 | After | **11.1 mm** | 21.7 mm | 55.3 mm | 89.5 mm |
| M7 17:51:25 | After | **11.1 mm** | 32.5 mm | 34.4 mm | 79.0 mm |

### Key pattern

While stopped:

```text
Raw RTK ↔ fused EKF2 ≈ 11 mm typical
```

While moving:

```text
Raw RTK ↔ fused EKF2 ≈ 25–55 mm typical
```

The moving difference does **not** keep increasing toward mission end.  
It reduces again when the rover stops.

That behavior is consistent with **dynamic estimator/filter lag**, not a continuously growing EKF drift during the mission.

---

# 5. Mission 1 — 17:31:25

Mission 1 is different from the others.

- It ran only about **9.1 s** before entering `PAUSED`.
- Stationary raw-vs-fused difference was about **24.3 mm median**.
- This is worse than the ~11 mm seen in the normal completed missions.
- However, the mission bag itself still showed:
  - valid absolute horizontal estimator state,
  - no GPS-glitch estimator flag,
  - no metre-scale local-position jump,
  - no obvious EKF horizontal reset during the recorded mission window.

The dangerous metre-scale EKF reset behavior happened **before this mission**, in the support ULog period around 17:30.

---

# 6. Missions 2–4 — Before Power Cycle

### M2 — 17:32:19

During the recorded mission:

- EKF absolute horizontal position: healthy.
- No estimator GPS-glitch flag.
- No local-position reset/jump.
- Static raw-vs-fused median: **10.8 mm**.

### M3 — 17:34:23

During the recorded mission:

- EKF estimator stayed healthy.
- Largest raw-vs-fused difference: about **123.6 mm**.
- That maximum occurred while the rover was moving.
- When stopped:
  - median: **11.1 mm**
  - P95: **24.3 mm**

The large moving difference therefore did not behave like accumulating estimator drift.

### M4 — 17:36:05

During the recorded mission:

- EKF estimator stayed healthy.
- Static raw-vs-fused median: **11.1 mm**.
- No metre-scale reset or position discontinuity.

### Verdict for M2–M4

These three missions were **already EKF-healthy during execution**.

Therefore, if M2–M4 had poorer path/marking behavior than M5–M7, the cause cannot be explained simply as an active EKF position drift occurring during those mission windows.

---

# 7. Power Cycle — ~17:40

The PX4 power cycle is visible because the estimator restarts and the local/global origin changes.

## Before reboot origin

```text
Lat: 13.1894406
Lon: 80.2220889
```

Used by:

- M1
- M2
- M3
- M4

## After reboot origin

```text
Lat: 13.1893416
Lon: 80.2223843
```

Used by:

- M5
- M6
- M7

Distance between the two origins:

```text
~33.843 m
```

This is **not a 33.8 m EKF drift**.

The rover was physically at a different location when the estimator initialized after reboot, so PX4 created a new local map origin.

Within each boot session, the origin remained stable.

---

# 8. EKF Reset Counters Across the Power Cycle

Before reboot:

```text
reset_count_pos_ne = 9
```

After reboot:

```text
reset_count_pos_ne restarted and later stabilized at 4
```

The reset counter staying constant during M5–M7 is important.

There was no new horizontal EKF position reset during the three final good mission windows.

---

# 9. Post-Reboot EKF Problem Before the Good Missions

The power cycle did **not permanently fix** the underlying estimator/GNSS problem.

At approximately **17:45**, before M5:

- GNSS PDOP quality failure appeared repeatedly.
- GNSS position fusion was interrupted for roughly **5.6 s**.
- EKF2 entered inertial/dead-reckoning mode.
- GNSS position innovation grew above approximately **2.2 m**.
- Raw RTK ↔ fused position reached approximately **2.31 m**.

The estimator recovered before M5 began.

This is important because it proves:

> Rebooting cleared the current estimator state, but the condition capable of causing GNSS-fusion interruption still existed afterward.

---

# 10. Final Three Missions After Power Cycle

## M5 — 17:47:39

- GNSS position fusion healthy through the recorded mission.
- No dead-reckoning period.
- No EKF position reset.
- Static raw-vs-fused median: **10.8 mm**.

## M6 — 17:49:36

- GNSS position fusion healthy.
- No estimator reset.
- Static raw-vs-fused median: **11.1 mm**.

## M7 — 17:51:25

- GNSS position fusion healthy.
- No estimator reset.
- Static raw-vs-fused median: **11.1 mm**.

### Final-three verdict

The final three missions ran after EKF2 had recovered and stabilized.

During those mission windows:

```text
GNSS position fusion: healthy
dead reckoning:       not active
horizontal resets:    none
static raw/fused:     ~11 mm
```

---

# 11. Local-Position Continuity During Missions

`/mavros/local_position/odom` was checked sample-to-sample.

Largest consecutive horizontal step observed:

| Mission | Largest normal XY step |
|---|---:|
| M1 | 42.4 mm |
| M2 | 43.7 mm |
| M3 | 44.2 mm |
| M4 | 44.1 mm |
| M5 | 44.7 mm |
| M6 | 44.0 mm |
| M7 | 44.4 mm |

There is no:

```text
0.5 m jump
1.0 m jump
2.0 m jump
```

inside any mission bag.

Those ~42–45 mm steps are consistent with normal rover movement at the odometry publication rate.

---

# 12. Estimator Status During Mission Execution

Across all seven mission bags, the recorded MAVROS estimator-status stream showed:

```text
pos_horiz_abs_status_flag       = TRUE
pos_horiz_rel_status_flag       = TRUE
velocity_horiz_status_flag      = TRUE
velocity_vert_status_flag       = TRUE
pred_pos_horiz_abs_status_flag  = TRUE
pred_pos_horiz_rel_status_flag  = TRUE

gps_glitch_status_flag          = FALSE
const_pos_mode_status_flag      = FALSE
accel_error_status_flag         = FALSE
```

for the recorded mission windows.

This supports the conclusion that the metre-scale estimator problem was **not active during the completed mission execution windows**.

---

# 13. Likely EKF2 Failure Mechanism Found in the Support ULogs

The live parameters included:

```text
EKF2_GPS_CHECK   = 831
EKF2_REQ_PDOP    = 3.0
EKF2_GPS_P_GATE  = 5
EKF2_GPS_P_NOISE = 0.01
EKF2_NOAID_TOUT  = 5.0 s
```

The logs showed `check_fail_max_pdop` becoming active during the bad estimator periods.

The observed sequence is:

```text
GNSS quality check fails
        ↓
some GNSS samples are not accepted for fusion
        ↓
GNSS horizontal aiding is interrupted
        ↓
EKF2 relies on inertial propagation
        ↓
position estimate drifts away from raw RTK
        ↓
GNSS becomes usable again
        ↓
EKF2 corrects / resets position back toward GNSS
```

Measured examples:

```text
~1.81 m error → reset
~1.72 m error → reset

post-reboot:
~2.31 m raw/fused separation during another aiding interruption
```

---

# 14. Important Recording Limitation

The mission bags do **not** contain the complete PX4 EKF2 internal uORB dataset.

The mission recorder provides MAVROS-level topics such as:

- estimator health flags,
- global raw GNSS,
- global fused position,
- local odometry.

The detailed EKF2 information such as:

- GNSS position innovation,
- GNSS innovation test ratio,
- horizontal reset counters,
- EKF dead-reckoning flags,
- EKF control-status internals,

came from the supporting PX4 ULogs.

There is also a short bag startup delay of approximately **2.5–5.6 s** before all ROS subscriptions are fully recording.

Therefore the mission bags prove estimator behavior from the **first available recorded EKF/GNSS sample to mission end**, not every millisecond immediately before Start was pressed.

---

# 15. Final Conclusion

## Confirmed EKF2 issue

A real estimator problem exists outside/around mission startup:

```text
GNSS aiding interruption
        ↓
EKF2 dead reckoning
        ↓
1–2+ m horizontal estimator drift
        ↓
GNSS recovery
        ↓
EKF position reset / snap back
```

This happened before the first mission and a similar episode happened again after the power cycle.

## During normal completed missions

M2–M7 themselves show:

```text
healthy estimator
stable local origin
no metre-scale position reset
no accumulating fused-position drift
~11 mm raw-vs-fused difference while stopped
```

Therefore:

> **M2–M4 being worse than M5–M7 cannot be explained only by EKF2 drift during mission execution.**

The EKF2 pre-mission robustness problem remains open, but the next mission-quality comparison should separately inspect:

```text
pivot
→ post-pivot re-anchor
→ path tracking
→ braking
→ terminal stop
→ final raw-RTK position error
```

while treating the EKF as healthy during the six completed mission windows.
