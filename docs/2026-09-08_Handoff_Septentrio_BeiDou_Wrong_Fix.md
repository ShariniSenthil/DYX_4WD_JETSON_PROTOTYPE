# Handoff — the ~150 mm RTK bias is a receiver-side constellation change, not antenna ARP/PCV

**Date:** 2026-09-08 · **Branch:** `feat/rtk-injection-v2` · **Continues:**
[2026-09-08_Handoff_EKF2_Validation_and_Base_Reference.md](2026-09-08_Handoff_EKF2_Validation_and_Base_Reference.md)
and the "150 mm Septentrio RTK position bias" handoff that ranked antenna
ARP/phase-centre handling as root-cause candidate #1.

**Dataset:** `Estimator_Compare/` — `A/` = 11 good ULogs (17:06–17:31),
`B/` = `log_47`, `log_48` (19:21–19:25), `mission_P1_ref.csv` (surveyed piles 2–6).

**New tool:** [`scripts/gps_dump_sbf_report.py`](../scripts/gps_dump_sbf_report.py).
`SEP_DUMP_COMM=3` puts every byte of the FCU↔mosaic-H serial link into `gps_dump`.
The tool splits it back into the two directions and decodes both — CRC-checked SBF
from the receiver, CRC-checked RTCM3 into it. Everything below is reproducible with:

```bash
python3 scripts/gps_dump_sbf_report.py --reference /path/to/mission_P1_ref.csv /path/to/*.ulg
```

Struct layouts come from PX4 1.16.2's own `sbf/messages.h`, ArduPilot's
`AP_GPS_SBF.h`, and the mosaic-H Firmware v4.14.10 Reference Guide; every layout is
size-checked against the receiver's own `Length`/`SBLength` at decode time. (One
correction: ArduPilot declares `VectorInfoGeod.ReferenceID` as `u1`; the spec and the
receiver both say `u2`. The `u1` layout silently shifts `SignalInfo` by one byte.)

---

## 1. Headline

Between the good group and the bad group **the receiver was power-cycled, and it
came back not using BeiDou at all** — while the base kept sending BeiDou corrections
the whole time. That is the only verified state change between the two groups. Every
antenna / ARP / phase-centre flag is bit-identical across them.

| | A (good) | B (bad) |
|---|---|---|
| receiver uptime at log start | 2245 → 3780 s, continuous | **2678 s — restarted between the groups** |
| `SEP_AUTO_CONFIG` | 1 | 0 |
| BeiDou used in the PVT | 88–100 % of epochs (19 %/55 % in the first two logs) | **0.4 % / 3.7 %** |
| longest continuous BeiDou absence | ≤ 8 s | **365 s / 140 s** |
| `NrSV` (median) | 20 | 15 |
| HDOP / VDOP | 0.65 / 1.00 | 0.75 / 1.64 |
| RTCM 1124 (BeiDou MSM4) injected | yes, 8 sv | **yes, 8–9 sv, same rate** |
| receiver says base corrections available for BeiDou | — (block not output) | **no** |
| height at pile 2 (WGS84 ellipsoidal) | −87.288 … −87.308 m | **−87.404 / −87.431 m** |
| horizontal offset from surveyed pile 2 | see §5 caveat | **+143 … +151 mm North, static** |

## 2. Antenna ARP / phase-centre is ruled out as the A-vs-B cause

Three independent lines, all verifiable without the receiver:

- **PX4 never configures antennas.** `SeptentrioDriver::configure()`
  (`septentrio.cpp:804-1010`) sends exactly `scs`, `sso`, `sdio`, `sto`, `srd`, `sga`
  and — only when `SEP_CONST_USAGE != 0` — `ssu`. There is no `setAntennaOffset`,
  no antenna type, no PCO/PCV command anywhere in the driver. Antenna configuration
  is receiver NVRAM, untouched by PX4 in **both** `SEP_AUTO_CONFIG` states.
- **The receiver reports the same compensation state in A and B.**
  `PVTGeodetic.Misc` is `0x50`/`0x60` in every log of both groups, in the same
  proportion. Decoded against the mosaic-H guide: bit 0 = 0 (baseline does not point
  to the base ARP / unknown), bit 1 = 0 (rover phase-centre offset not compensated /
  unknown), bits 6–7 = 1 (**ARP-to-marker offset is zero**, i.e. no
  `setAntennaOffset` deltas are configured). Bits 4–5 are proprietary and toggle
  identically in both groups.
- **The missing RTCM antenna descriptors are missing in both groups.** 1005, 1074,
  1084, 1094, 1124, 1230 present; no 1006/1007/1008/1033 anywhere, A or B.

`BaseVectorGeod.Misc` in B is `0x10`/`0x20` — bits 0 and 1 both clear, so the spec's
"accurate ARP-to-ARP baseline is guaranteed only if both bits 0 and 1 are set" is
indeed not satisfied. **That is real and worth fixing, but it cannot explain A vs B**:
the identical `PVTGeodetic.Misc` shows the same state held during the good runs, which
landed within a few cm. Whatever this contributes is a constant of at most a few cm,
present in every run this project has ever recorded.

**Verdict: demote antenna ARP/PCV from candidate #1 to a standing, non-discriminating
configuration gap.** It is worth closing, but it is not this defect.

## 3. What actually changed: BeiDou left the solution

`PVTGeodetic.SignalInfo` is a bitmask of the signals used in the PVT (mosaic-H guide
§4.1.10):

```
A: 0x30220909 = GPS L1CA + GPS L2C + GLO L1CA + GLO L2CA + GAL E1 + GAL E5b + BDS B1I + BDS B2I
B: 0x00220909 = GPS L1CA + GPS L2C + GLO L1CA + GLO L2CA + GAL E1 + GAL E5b
```

Two BeiDou signals, ~5 satellites, gone — matching `NrSV` 20 → 15 and VDOP 1.00 → 1.64
exactly.

**It is not the correction stream.** The RTCM 1124 messages injected during B carry
8–9 BeiDou satellites in their DF394 satellite mask, at the same rate as in A
(CRC-24Q checked, decoded from the to-device stream). The rover received BeiDou
corrections and did not use them. `BaseVectorGeod.SignalInfo` — "signals for which
differential corrections are available from the base" — reads `0x00220909`, i.e. the
receiver itself reports *no BeiDou corrections available* while they are arriving on
its serial port.

**It is not propagation.** BDS B2I is 1207.14 MHz — the *same* frequency as Galileo
E5b, which was retained throughout. BDS B1I at 1561.098 MHz was dropped while GPS
L1CA at 1575.42 MHz was retained. Scintillation, ionospheric disturbance and
interference are frequency-selective, not constellation-selective; nothing
propagation-related drops one constellation and keeps another on the same carrier.
(Chennai at 19:20 local, ~1 h after sunset, is a textbook equatorial-scintillation
window, so this deserved ruling out explicitly. It is ruled out.)

**It is not PX4.** `SEP_CONST_USAGE = 0` in every log, and the driver skips `ssu`
entirely at that value ("when this is 0, the constellation usage isn't changed",
`module.yaml:127-144`). PX4 has never commanded the constellation set on this
vehicle, in either `SEP_AUTO_CONFIG` state. The constellation set is purely receiver
NVRAM — and the receiver restarted between A and B.

So: **BeiDou was excluded receiver-side, by configuration or licence, across a
power cycle.** Whether it is tracking, usage, or a permissions/licence gate is not
determinable from these logs — the receiver was not configured to output
`ChannelStatus`, `SatVisibility` or `ReceiverSetup`.

## 4. The B solution is a wrong ambiguity fix, and the vertical channel proves it

- **B is static.** `log_47` spans 0.19 m N × 0.36 m E over 470 s, `log_48` 0.10 × 0.08 m
  over 184 s, p50 speed 7 mm/s. These are placement tests, not mission stops, so the
  horizontal offset is a measurement bias and not a control error.
- **Confidently wrong.** `PosCovGeodetic` reports σ_North ≈ 9 mm against a 143–151 mm
  actual North error — wrong by ~16σ, while holding `Mode = 4` (RTK fixed) with
  `Error = 0`. That combination is the defining signature of an incorrect integer
  ambiguity resolution; it is not what a DOP-scaled noise process or a static antenna
  offset produces.
- **It drifts the way a wrong fix drifts.** Over 11 minutes parked, dN goes
  +123 → +153 mm and dE −21 → +22 mm, smoothly, still RTK FIXED. A wrong integer set
  maps through a slowly rotating geometry matrix and produces exactly this slow drift.
  An antenna/ARP error would be constant; noise would be random.
- **The height moves 100–140 mm and nothing in the rover can absorb it.** At pile 2,
  A reads −87.288…−87.308 m and B reads −87.404/−87.431 m, on flat ground. Height is
  not closed-loop controlled, so this is a clean, control-free confirmation that B's
  fix is wrong. It also matches the still-open 2026-09-03 anomaly in `CLAUDE.md`
  (raw GNSS altitude 125–152 mm *low* in three runs, cleared by a power cycle) in
  sign, magnitude and in being cleared by a restart — **these are very likely the
  same failure mode**, which would make this defect recurrent rather than a one-off.
- **DOP does not explain it within B.** Across B's stationary windows, `NrSV` ranges
  12–17 and HDOP 0.68–0.93 while the North error stays 121–158 mm with no ordering.
  The `corr(|North error|, VDOP) ≈ 0.90` from the previous handoff is a between-group
  artefact of two extreme points, not a within-group relationship.

## 5. ⚠ Correction to how the A-group numbers should be read

The A logs are **mission** logs. The rover drives to the surveyed target using this
same GNSS and stops when its *reported* position reaches the target, so a GNSS bias in
A is absorbed into the physical parking position and does not appear in the reported
error. **A's 3–42 mm horizontal numbers therefore measure controller convergence, not
GNSS accuracy** — they cannot be used to certify that A's fix was correct.

The honest A-vs-B accuracy comparison is the **height** channel (§4), which is
open-loop in both groups. It gives the same answer, so the conclusion stands — but the
reasoning in the incoming handoff's A/B table needs this caveat attached.

## 6. Root-cause ranking, revised

1. **Wrong integer ambiguity resolution** (was #3). Directly evidenced: 16σ error,
   RTK-FIXED with Error 0, slow geometry-driven drift, 3D (not planar) displacement,
   restart-clearable, matching the open Sep-03 vertical anomaly.
2. **Loss of BeiDou from the RTK solution as the trigger** (was #2, "geometry"). Now a
   specific, verified, receiver-side state change rather than "the sky moved": 5
   satellites and a whole constellation removed from ambiguity resolution and
   validation, VDOP up 64 %, on a 13.68 km single-base baseline. Causality is still
   *not* proven — it is the only verified difference, plus a plausible mechanism.
3. **Antenna ARP / phase-centre handling** (was #1). Real but non-discriminating; see
   §2. Fix it for correctness, not to close this defect.

## 7. Next moves

**No field data needed for 1–3.**

1. **Interrogate the receiver.** Over the web UI or the NSH/SSH route in
   [`docs/PX4_NSH_OVER_SSH.md`](PX4_NSH_OVER_SSH.md), capture and record:
   `lif,Permissions` (is BeiDou licensed on this mosaic-H?), `gst`
   (getSatelliteTracking), `gsu` (getSatelliteUsage), `gao` (getAntennaOffset),
   `lif,AntennaInfo`, `grd` (getReceiverDynamics), `gpm` (getPVTMode). Whatever is
   found, save it to NVRAM so a power cycle cannot change it again.
2. **Stop depending on receiver NVRAM for the constellation set.** Set
   `SEP_CONST_USAGE = 31` (GPS+GLONASS+Galileo+SBAS+BeiDou) and `SEP_AUTO_CONFIG = 1`.
   PX4 then issues `ssu` on every boot and the constellation set becomes deterministic
   and logged. ⚠ `ssu` sets *usage*, not *tracking* — if tracking is what was disabled,
   this alone will not restore BeiDou; check both in step 1.
3. **Make the receiver state observable in every log.** The B configuration already
   proves this works: add `ReceiverSetup` (5902 — antenna type, serial, ARP deltas),
   `ChannelStatus` (4013) and `SatVisibility` (4012) to the receiver's own SBF stream
   and `gps_dump` will capture them. `SEP_SAT_INFO` does **not** do this — the driver
   only copies the satellite count into `satellite_info` (`septentrio.cpp:1126-1131`).
   Without those blocks the per-satellite azimuth/elevation/CN0 needed to settle the
   sky-sector question does not exist in any log this project has.
4. **Then, and only then, re-test in the field:** repeat the static placement test at
   pile 2 with BeiDou confirmed in `SignalInfo`, in the same post-sunset window, and
   see whether the 150 mm returns. That is the experiment that turns association into
   causation.
5. **Separately, close the ARP gap** (§2): configure rover antenna type/ARP on the
   receiver, and ask the caster operator for 1006 or 1007/1008/1033 so
   `BaseVectorGeod.Misc` bits 0 and 1 can both be set. Expect a few cm, not 150 mm.

## 8. Incidental findings

- `ReceiverStatus.RxError` bit 3 (`SOFTWARE`: "set upon detection of a software
  warning or error") is set in **all 13 logs**, A and B. Constant, so not this defect,
  but it has never been explained — and the spec says it is cleared by `lif, error`,
  which also *reports* what the warning was. Run that in step 1 above.
- **`log_20` is the closest thing to a natural experiment and it cuts the other way.**
  It ran at `NrSV` 14 / HDOP 1.12 / BeiDou in only 19 % of epochs — a constellation
  about as degraded as B's — and the receiver stayed **RTK float for all 383 epochs**,
  i.e. it declined to fix. `log_21` was 46 % float and consolidated to fixed as BeiDou
  came in; every later A log is 100 % fixed. B, on a similarly thin constellation,
  declared **fixed** and was wrong by 150 mm. Why the receiver was willing to fix in B
  and not in `log_20` is unexplained and is probably the sharpest single question to
  put to the receiver's ambiguity/integrity diagnostics.
- Dual-antenna separation is a stable 0.7280–0.7282 m with `AmbiguityType = 0` (fixed)
  and ΔUp ≈ +1…+9 mm. Worth confirming against the physical mounting distance.
- `srd,high,UAV` (setReceiverDynamics) is sent only when `SEP_AUTO_CONFIG = 1`, so B
  ran on whatever dynamics model was persisted — another uncontrolled A/B difference,
  and another reason to keep auto-config enabled.
- The B group's richer SBF output (`BaseVectorGeod`, `PosCovGeodetic`, `PosLocal`,
  `AuxAntPositions`, `PVTSupport`) exists precisely *because* auto-config was off:
  with it on, PX4 clears the stream and sets exactly
  `PVTGeodetic+VelCovGeodetic+DOP+AttEuler+AttCovEuler+EndOfPVT+ReceiverStatus`. The
  SBF block census is a reliable fingerprint of whether auto-config ran. Use a
  *second* receiver stream for diagnostics so auto-config can stay on.
