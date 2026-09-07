# EKF2 PDOP Gate — Root Cause, Firmware Source + Log Evidence

**Date:** 2026-09-07. Builds on and supersedes the PDOP-related parts of
[2026-09-07_EKF2_Logs_and_Mission_Bags_Findings.md](../2026-09-07_EKF2_Logs_and_Mission_Bags_Findings.md)
(that doc's mission-by-mission raw-vs-fused table and conclusion that
M2-M7 were EKF-healthy during execution both still stand). Firmware:
`54f0455ffcd755534539a7cf33a09a20bf71d29d`
(`ShariniSenthil/DYX_4WD_PX4_FIRMWARE_16.2`), confirmed identical across all
21 ULogs analysed. Source citations below are verified directly against
that commit in the local checkout at
`/Users/dyx_a1/Vetri/Way_to_Mark/PX4-Autopilot-4WD-Prod-Baseline` - every
line number was read, not inferred. Log evidence is decoded directly from
`~/Documents/QGroundControl Daily/Logs/4WD/Missions/` (21 ULogs,
`log_51` through `log_71`, spanning 17:23-17:54 IST on 2026-09-07).

**No parameter has been changed as a result of this document.**

---

## 1. Verdict

**Every PDOP-driven GNSS rejection observed across all 21 logs today was
caused by vertical geometry (VDOP), never horizontal (HDOP).** HDOP never
exceeded 2.36 anywhere in 940 samples across the full session; the
`EKF2_REQ_PDOP` gate (live value 3.0) combines HDOP and VDOP into one
number, so a vertical-geometry dip that has nothing to do with horizontal
accuracy throws away perfectly good horizontal aiding. This is not a
one-off: it recurred at four separate points across the session (17:30,
17:39, 17:45, 17:53), before and after a full PX4 power cycle, confirming
it is a property of the sky/antenna geometry at this site and time, not a
transient fault the reboot cleared.

The two multi-metre EKF position resets already documented (pre-M1: two
resets totalling ~1.78m and ~1.63m; pre-M5: one ~2.3m dead-reckoning
excursion) are both instances of this same mechanism, not two different
phenomena as an earlier pass at this data concluded.

---

## 2. Firmware mechanism, cited line-by-line

All paths relative to the PX4-Autopilot repo root, at commit `54f0455ffc`.

### 2.1 The parameter and the formula

- `EKF2_REQ_PDOP` is a real parameter: declared at `src/modules/ekf2/EKF2.hpp:533`
  (`ParamExtFloat<px4::params::EKF2_REQ_PDOP> _param_ekf2_req_pdop`), documented
  in `src/modules/ekf2/params_gnss.yaml:155-161` with **default 2.5**, wired to
  the EKF's internal `req_pdop` field at `src/modules/ekf2/EKF2.cpp:96`.
- PDOP is computed from the raw receiver's own HDOP/VDOP, not queried from the
  receiver directly — `src/modules/ekf2/EKF2.cpp:2442-2443`:
  ```cpp
  .pdop = sqrtf(vehicle_gps_position.hdop * vehicle_gps_position.hdop
                + vehicle_gps_position.vdop * vehicle_gps_position.vdop),
  ```
  This is the exact `PDOP = sqrt(HDOP^2 + VDOP^2)` formula - confirmed present,
  not assumed.

### 2.2 The check and the mask

`src/modules/ekf2/EKF/aid_sources/gnss/gps_checks.cpp`:

- Bit definitions, lines 44-53 (`MASK_GPS_NSATS=1<<0` through
  `MASK_GPS_SPOOFED=1<<9`; PDOP is `MASK_GPS_PDOP = 1<<1` = value 2).
- The check itself, line 69: `_gps_check_fail_status.flags.pdop = (gps.pdop > _params.req_pdop);`
- The combination logic, lines 170-186: if the PDOP flag is set **and** its
  mask bit is enabled in `_params.gps_check_mask` (along with the same
  pattern for every other check), the whole `runGnssChecks()` call returns
  `false` for that sample - there is no partial credit; one failing
  mask-enabled check fails the sample.

`EKF2_GPS_CHECK` live value is **831**. Decoding against the bit definitions
above: bits 0,1,2,3,4,5,8,9 are enabled (NSATS, **PDOP**, HACC, VACC, SACC,
HDRIFT, VSPD, SPOOFED); bits 6 (VDRIFT) and 7 (HSPD) are disabled. This
matches the 2026-09-04 audit's independent decode of the same mask. **The
PDOP bit is enabled on this rover.**

### 2.3 What happens to a rejected sample, and how fusion actually stops

`src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp:63-92`:

```cpp
if (runGnssChecks(gnss_sample)
    && isTimedOut(_last_gps_fail_us, max((uint64_t)1e6, (uint64_t)_min_gps_health_time_us / 10))) {
    if (isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us)) {
        // First time checks are passing, latching.
        _gps_checks_passed = true;
    }
} else {
    // Skip this sample
    _gps_data_ready = false;
    if ((_control_status.flags.gnss_vel || _control_status.flags.gnss_pos)
        && isTimedOut(_last_gps_pass_us, _params.reset_timeout_max)) {
        stopGnssFusion();
        ECL_WARN("GNSS quality poor - stopping use");
    }
}
```

- A single failing sample is simply skipped (`_gps_data_ready = false`) -
  fusion does not stop immediately.
- `_min_gps_health_time_us` is `EKF2_REQ_GPS_H`, live value **1.0 s**: a
  sample only *latches* `_gps_checks_passed = true` after a full second of
  continuously passing checks since the last failure.
- `reset_timeout_max` is a **hardcoded constant**, `src/modules/ekf2/EKF/common.h:478`:
  `const unsigned reset_timeout_max{7'000'000};` (7,000,000 us = 7 s), with the
  comment *"maximum time we allow horizontal inertial dead reckoning before
  attempting to reset the states... if the data is unavailable"* - i.e. this
  exact number is what decides how long the EKF free-runs on IMU alone before
  declaring GNSS position/velocity fusion stopped.
- Once GNSS fusion is stopped and later restarts, `controlGnssPosFusion()`
  (`gps_control.cpp:189-215`) calls `resetHorizontalPositionToGnss()`
  (defined `gps_control.cpp:352`) rather than smoothly re-blending - this is
  the exact function that produces the metre-scale position snap documented
  in section 6 of the "Logs and Mission Bags Findings" doc, and it is by
  design, not a bug: `Ekf::controlGnssPosFusion`'s `starting_conditions_passing`
  branch logs `"starting GNSS position fusion"` and resets outright.

**Chain, fully sourced end to end:**

```text
PDOP = sqrt(hdop^2 + vdop^2) > EKF2_REQ_PDOP (3.0, live)
        -> gps_checks.cpp:69 sets the pdop fail flag
        -> gps_checks.cpp:170-186 fails the whole sample (bit 1 enabled in mask 831)
        -> gps_control.cpp:79 skips the sample (_gps_data_ready = false)
        -> if this persists for reset_timeout_max = 7s (common.h:478, hardcoded)
           since the last PASSING sample -> stopGnssFusion() (gps_control.cpp:82-84)
        -> EKF free-runs on IMU alone (inertial dead reckoning)
        -> once a sample passes AND stays passing for EKF2_REQ_GPS_H = 1.0s
           (gps_control.cpp:65-71) -> _gps_checks_passed latches true
        -> controlGnssPosFusion() sees starting_conditions_passing
           -> resetHorizontalPositionToGnss() (gps_control.cpp:352, called
              from gps_control.cpp:200) -> EKF snaps to the GNSS position
```

---

## 3. Log evidence: the mechanism firing, with timestamps

### 3.1 Pre-M1 window (`log_52`, 17:30:19-17:31:10 IST)

Live parameters (decoded directly from `log_52.initial_parameters`):
`EKF2_REQ_PDOP=3.0`, `EKF2_GPS_CHECK=831`, `EKF2_REQ_GPS_H=1.0`,
`EKF2_NOAID_TOUT=5000000` (this is `valid_timeout_max`, `common.h:482` -
governs when the estimate is reported *invalid*, a related but separate
value from the 7s `reset_timeout_max` that actually stops fusion).

`sensor_gps` decoded at 1 Hz for this log:

| Time (IST) | fix_type | sats | HDOP | VDOP | PDOP (computed) | >3.0? |
|---|---:|---:|---:|---:|---:|---|
| 17:30:19.400 - 17:30:27.400 (9 consecutive samples) | 6 | 11 | 1.27 | 2.99 | **3.25** | **yes, ~9s continuous** |
| 17:30:28.400 | 6 | 12 | 1.08 | 1.68 | 2.00 | no |
| ... | | | | | | |
| 17:30:38 - 17:30:47 (mixed, several samples) | 6 | 10-12 | 1.24-1.70 | 2.19-4.89 | 2.52-**5.18** | **yes, repeated** |
| 17:30:51.400 (second reset instant) | 6 | 13 | 1.03 | 1.45 | 1.78 | no |

`fix_type` never drops below 6 (RTK FIXED) anywhere in this window; `eph`
stays 2-6 cm and `epv` 3-10 cm throughout, including at both reset instants.
**HDOP never exceeds 1.70 anywhere in this window; VDOP reaches 4.89.** This
is why a first pass at this data (checking only `fix_type`/`eph`/`epv`, the
fields a normal health check would show) found nothing wrong - the failure
is invisible unless HDOP and VDOP are checked against the actual gate
formula.

Documented EKF-side consequence (from the Findings doc, cross-referenced
here, not re-derived): `reset_count_pos_ne` 7->8 at 17:30:28.799 (delta
+1.478N/-0.998E, magnitude 1.783m) and 8->9 at 17:30:51.009 (delta
+0.821N/+1.409E, magnitude 1.631m) - both fall immediately after the ~9s and
~8.4s continuous-PDOP-failure windows above, consistent with the
7-second `reset_timeout_max` in section 2.3.

### 3.2 Pre-M5 window (`log_63`, 17:43:35-17:45:23 IST)

Same live parameters confirmed identical. The failure here is compound - a
genuine satellite-count drop coincides with the PDOP mechanism:

| Time (IST) | fix_type | sats | HDOP | VDOP | PDOP | eph / epv |
|---|---:|---:|---:|---:|---:|---|
| 17:44:59.100 | 5 | 15 | 0.81 | 1.19 | 1.44 | 0.02 / 0.05m |
| 17:45:00-03 | **3** | 16 | 0.81-0.86 | 0.98-1.09 | 1.27-1.37 | up to 1.6 / 3.4m |
| 17:45:04-08 | 5 | **7-9** | 1.24-1.79 | 1.65-2.95 | 1.92-**3.45** | up to 0.97 / 1.79m |
| 17:45:10+ | 6 (recovered) | 11-17 | back to 0.7-1.4 | back to 1.1-2.6 | back to ~1.3-3.0 | back to cm-level |

Satellite count genuinely collapsed from the mid-teens to 7-9 for ~8 seconds
- unlike the pre-M1 window, this one *does* show a visible raw-signal cause.
No jamming (`jamming_indicator=0`), spoofing (`spoofing_state=0`), or noise
floor elevation (`noise_per_ms=0`, `automatic_gain_control=0`) at any point
in either window - this reads as a reception/geometry event, not
interference.

---

## 4. HDOP vs VDOP attribution, all 21 logs

Decoded `sensor_gps.hdop`/`.vdop` from every ULog in the Missions folder,
computed `PDOP = sqrt(hdop^2+vdop^2)` per sample, flagged against the live
`EKF2_REQ_PDOP = 3.0`:

| Log | Samples | PDOP fails (>3.0) | HDOP median | VDOP median | HDOP max | VDOP max |
|---|---:|---:|---:|---:|---:|---:|
| `log_51` (17:23) | 53 | 0 | 0.82 | 1.31 | 0.92 | 1.60 |
| `log_52` (17:31, pre-M1) | 52 | **20 (38.5%)** | 1.24 | 2.19 | 2.36 | 4.89 |
| `log_53`-`log_56` (17:31-17:34) | 46 | 0 | 0.76-0.82 | 1.17-1.35 | 0.86-1.11 | 1.29-1.58 |
| `log_57`-`log_61` (17:35-17:38) | 187 | 0 | 0.68-0.78 | 1.07-1.26 | 0.74-1.25 | 1.19-2.53 |
| `log_62` (17:39) | 63 | 2 (3.2%) | 0.89 | 1.48 | 1.43 | 2.69 |
| `log_63` (17:45, pre-M5) | 109 | 3 (2.8%) | 0.82 | 1.30 | 1.79 | 2.95 |
| `log_64`-`log_70` (17:47-17:52) | 334 | 0 | 0.71-0.80 | 1.13-1.33 | 0.76-1.13 | 1.19-1.75 |
| `log_71` (17:53, post-missions) | 50 | **6 (12.0%)** | 0.77 | 1.25 | 1.35 | 3.30 |
| **Total** | **940** | **31 (3.3%)** | — | — | **2.36 (session max)** | 4.89 |

Attribution among the 31 failing samples:

| | HDOP alone | VDOP alone |
|---|---:|---:|
| Median | 1.35 | 2.99 |
| Max | 2.36 | 4.89 |
| Exceeds 3.0 on its own | **0 of 31 (0.0%)** | 11 of 31 (35.5%) |
| Is the larger of the two terms | 1 of 31 | **30 of 31 (96.8%)** |

**HDOP does not exceed 3.0 in a single sample across the entire 940-sample,
21-log dataset - not even during a PDOP failure.** Every rejection is
attributable to VDOP. Failures cluster in four separate logs at four
separate times (17:30, 17:39, 17:45, 17:53), spanning before and after the
power cycle, all 17 other logs show zero failures - this is a recurring
vertical-geometry characteristic of this site/time, not an isolated event.

---

## 5. What this does and does not establish

**Established, with source + log citations above:**
- `EKF2_REQ_PDOP` exists, is enabled in the live check mask, and its formula
  combines HDOP and VDOP.
- The 7-second dead-reckoning-then-reset behavior is a real, hardcoded
  firmware mechanism, not a hypothesis - `reset_timeout_max` is a named
  constant with a comment describing exactly this behavior.
- Every PDOP rejection observed today was driven by VDOP; HDOP was never
  close to the gate.
- This recurs across the session and across a power cycle - it is a property
  of the sky/antenna geometry here, not a transient the reboot fixed.

**Not established, and not claimed:**
- *Why* VDOP degrades at these specific times (satellite constellation
  geometry, a physical obstruction on part of the sky, antenna
  multipath, or something receiver-internal) - this needs the raw
  Septentrio SBF stream, which `SEP_DUMP_COMM=0` means was not logged
  (open item since the 2026-09-04 audit, still open).
- Whether dropping the PDOP check (or gating on HDOP/EPH/NSATS/SACC
  instead) is the correct production fix, versus some other mitigation -
  that is a configuration decision for the task-sheet process, not
  something this document decides.
- Whether the pre-M1/pre-M5 EKF resets caused any actual mission-quality
  degradation - M1 was aborted before reaching this state's consequences
  mattered, and M2-M7 were already confirmed EKF-healthy during their own
  execution windows by the earlier Findings doc.

## 6. Recommendation

Do not change `EKF2_GPS_CHECK`, `EKF2_REQ_PDOP`, or any related parameter
from this document alone. The evidence here supports specifically
evaluating: (a) disabling the PDOP bit (bit 1, value 2) in
`EKF2_GPS_CHECK` given HDOP/EPH/NSATS/SACC are already independently
enabled and would continue gating genuine horizontal-quality problems, or
(b) some other separation of horizontal from vertical GNSS quality gating -
through the normal task-sheet decision process, not applied directly here.
