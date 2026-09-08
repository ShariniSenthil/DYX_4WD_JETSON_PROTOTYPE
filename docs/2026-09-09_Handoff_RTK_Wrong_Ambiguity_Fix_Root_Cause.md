# Handoff — ROOT CAUSE: the position error is a wrong integer ambiguity fix, and it changes only when the receiver re-fixes

**Date:** 2026-09-09 · **Branch:** `feat/rtk-injection-v2`
**Supersedes the lead in:** [2026-09-08_Handoff_Septentrio_BeiDou_Wrong_Fix.md](2026-09-08_Handoff_Septentrio_BeiDou_Wrong_Fix.md)
(left in place unedited; §3 below records exactly what in it does not survive).
**Corpus:** every September 2026 ULog on the analysis machine — 164 found, **142
unique** after de-duping by name+size, 119 with >10 s of GNSS.
**Tool:** [`scripts/gps_dump_sbf_report.py`](../scripts/gps_dump_sbf_report.py).

---

## 1. The result

**Inside a single 98-minute log, at the same physical point, with nothing
changed — no reboot, no parameter, no configuration, no constellation change —
the reported position steps by 192 mm, and it steps exactly on the one RTK
ambiguity re-resolution in the log.**

`log_15_2026-9-8-14-10-54`, parked fixes within 0.3 m of P0001, split on its only
long RTK outage (1093 s, t = 2828 → 3921 s):

| | parked fixes | height p50 | dN p50 | sats | vdop | **eph** | BeiDou |
|---|---|---|---|---|---|---|---|
| **before** the re-fix | 10 064 | **−87.487 m** | +91.8 mm | 18 | 1.30 | **15 mm** | 99.7 % |
| **after** the re-fix | 9 509 | **−87.295 m** | +102.4 mm | 19 | 1.14 | **15 mm** | 99.7 % |

Within each band the height wanders only tens of mm across many separate parked
windows (−87.457 → −87.530 over 28 min before; −87.268 → −87.352 after). The
reported position is **stable while a fix is held and steps by decimetres when the
fix is re-resolved**. The receiver reported the same 15 mm `eph` in both states.

That is the defining signature of an incorrect integer ambiguity resolution, and
it is now demonstrated within one session rather than inferred across sessions.

### Corroboration and a control

- **`log_18` corroborates.** At one spot (dN ≈ +175.75 m, dE ≈ +75.17 m relative to
  P0001) the height reads −87.800 / −87.810, then after a 170 s fix loss reads
  **−88.018 / −88.037** — a **−210 mm** step across the re-fix.
- **`log_47` is the control.** RTK fixed 99.5 % of epochs, no outage longer than
  1.6 s, and across 8 parked windows over 4 minutes the height holds −87.415 to
  −87.444 — a 30 mm range. One fix, held for the whole session: **wrong, but held**,
  which is exactly what the hypothesis predicts and why that session looked like a
  stable "bias".

### Where truth sits

`log_12_2026-9-8-11-06-54` is an independently verified good fix (§2). It reads
**−87.277 m** at P0001, and the 2026-09-07 day median at the same point is
**−87.261 m**, from a different day and session.

| state | height at P0001 | vs truth |
|---|---|---|
| log_12 / Sep-07 | −87.26 … −87.28 | reference |
| log_15 **after** its re-fix | −87.295 | −20 … −35 mm ✓ |
| log_15 **before** its re-fix | −87.487 | **−210 mm** |
| log_47 / log_48 | −87.38 … −87.44 | **−110 … −165 mm** |

## 2. The verified-good reference log

`log_12`, parked 74.9 s (750 fixes, gated on GNSS Doppler <30 mm/s **and** yaw rate
<0.05 rad/s so a pivot cannot masquerade as parked), RTK fixed throughout,
20 satellites, HDOP 0.62 / VDOP 1.06, RTCM 6 Hz, `eph` 15 mm:

- **raw vs surveyed P0001:** dN −5.4, dE +8.5, radial **10.1 mm**
- **fused vs surveyed P0001:** dN +0.7, dE +5.1, radial **5.2 mm**
- **fused − raw:** North +6.1 mm, East −3.3 mm, Up **−300.0 mm** — exactly the
  configured `EKF2_GPS_POS_Z = −0.30`, so the vertical lever arm is applied
  correctly to the millimetre.
- scatter 1σ raw N/E/U 6.8 / 5.0 / 12.5 mm, fused 6.8 / 5.5 / 12.9 mm — while
  parked the EKF passes the raw fix through rather than smoothing or fighting it.
- EKF acceptance: position test ratios p50 0.0003 (max 0.012), height p50 0.0005
  (max 0.015), **0 rejections, 150/150 fused**; over the whole log
  `gps_check_fail_flags` nonzero **0/974** and `filter_fault_flags` **0/974**.

⚠ Two caveats that matter for how this log is quoted:
- **Raw-vs-fused agreement proves internal consistency, not correctness.**
  `vehicle_global_position` is derived from the raw fix; under a common-mode GNSS
  error both are wrong together and agree perfectly.
- The horizontal 10 mm is partly closed-loop: the rover drove itself there on this
  same GNSS and stops inside the 30 mm latch. **The height is the channel that
  carries evidence**, because nothing in the rover controls it — and it agrees with
  an independent day (Sep-07) to 16 mm.

## 3. What does not survive from the 2026-09-08 handoff

That document is left unedited. These four claims in it are retracted:

1. **"The receiver was power-cycled between the groups and came back not using
   BeiDou"** — framed as the mechanism. There were **five** receiver boots on
   Sep-08 (reconstructed from GPS week/TOW plus `ReceiverStatus.UpTime`):
   99.7 %, 91.9 %, 93.9 %, 19→100 %, and 0.4/3.7 % BeiDou. Four fresh starts came
   back with BeiDou; one did not. A reboot merely forces a re-fix.
2. **"Height 100–140 mm below the good group is control-free proof the bad fix is
   wrong."** The conclusion is right but the evidence was not: `log_15`, with
   BeiDou at 99.7 %, swings 230 mm vertically at P0001 within one log and reaches
   −87.53, *below* log_47. The A-vs-B height comparison proves nothing on its own.
   §1 replaces it with a within-log comparison that does hold.
3. **"BeiDou loss is the trigger" (rank #2).** `log_15` sat on a 210 mm-wrong fix
   with BeiDou at 99.7 %. BeiDou is **not necessary** for the failure. Its absence
   in boot #5 remains a real, unexplained, receiver-side anomaly (the base was
   sending RTCM 1124 with 6–10 satellites in every log all day) and a plausible
   aggravator of ambiguity resolution — but it is not the mechanism.
4. **"+148 mm North is uniquely large."** `log_15`'s long static holds sit at
   dN +96 … +108 mm at P0001 with BeiDou present. And more fundamentally: **nothing
   in these logs measures where the rover physically was**, so "parked 10 cm north"
   and "reported 10 cm north" cannot be separated for any stop, log_47/48 included.

What **does** survive: antenna ARP / phase-centre handling is bit-identical in the
good and bad logs (`PVTGeodetic.Misc` = `0x50`/`0x60` in all 13 A/B logs, bit 0 = 0,
bit 1 = 0, ARP-to-marker offset zero) and is therefore not the discriminator; the
error lives in the receiver's own differential baseline, not in EKF2; and the
receiver cannot self-assess — it reports ~9–15 mm confidence in both states.

## 4. September-wide sweep — what it cleared

- **RTCM injection is healthy everywhere.** 112 of 119 logs sit at exactly 6.0 Hz
  median. The 7 exceptions are 6 complete outages (0 Hz, 0 % RTK: `log_83`, `log_84`
  on 9-3; `log_49`/`50`/`51` on 9-7; `log_31` on 9-8) plus `log_1` at 4.8 Hz — all
  obvious, none of them a position-error log. **Reduced correction rate is not a
  factor in any of the bad-position logs.**
- **Constellation content is only measurable on 17 logs, all 2026-09-08**, because
  `SEP_DUMP_COMM` was 0 before that. Nothing on Sep-01/03/04/07 can be checked for
  which signals the solution used.
- **2026-09-08 is the anomalous day, not the evening.** Within-day vertical spread
  at P0001: Sep-03 p50 10.0 / max **41.9 mm**; Sep-07 p50 12.2 / max **43.8 mm**;
  Sep-08 p50 77.8 / max **416.4 mm** — starting in the 10:30 session.
- **No parameter explains it.** Across representative logs per day the only
  GNSS-relevant differences are `EKF2_GPS_CHECK` 831→829, `EKF2_GPS_YAW_OFF` 0→180
  (log_47 only), `SEP_AUTO_CONFIG` and `SEP_DUMP_COMM`. EKF2 parameters cannot move
  the raw receiver solution, and every offset here is measured on raw GNSS.

## 5. What to do

### 5.1 The guard the rover can have today

`eph` is 15 mm in both the correct and the 210 mm-wrong state, so no downstream
gate — EKF innovation, DOP, `fix_type`, satellite count — can tell them apart. That
is settled and it is not fixable by tightening anything.

**But the transition is observable:** `fix_type` dropping below 6 and returning.
After any RTK re-acquisition the position may have stepped by decimetres.
Recommended contract in `mission_manager`: treat an RTK re-fix as invalidating
prior marking references — refuse to continue marking until the rover has
re-verified against a known surveyed point. This is a real code change, not a
parameter, and it is the only defence that matches the measured failure.

### 5.2 The experiment that quantifies the exposure

Park at P0001 and force repeated re-fixes — cut corrections for ~60 s, restore,
repeat ~10 times — logging each settled position. If the fixes land in discrete
clusters separated by ~100–200 mm, that confirms the mechanism and measures how far
wrong a bad fix can be at this 13.68 km baseline. No mission, no bench, one hour.

### 5.3 Reduce how often it fixes wrong

A wrong fix at 13.68 km single-base is driven by ionospheric decorrelation and weak
geometry. Concretely:
- Restore BeiDou determinism (it costs ~5 satellites and 60 % of VDOP when absent):
  check `lif, Permissions`, `gst`, `gsu` on the receiver, `sst, all` + `eccf,
  Current, Boot` if tracking is off, and set `SEP_CONST_USAGE = 31` with
  `SEP_AUTO_CONFIG = 1` so PX4 commands `ssu` on every boot. ⚠ `ssu` sets *usage*,
  not *tracking*.
- Ask whether a nearer base or a network (VRS/MAC) mountpoint is available. 13.68 km
  is on the long side for reliable single-base integer fixing.
- Close the antenna gap (`sao` with the antenna type; base 1006/1007/1008/1033) —
  worth a few cm and it removes a known unmodelled term from the fix.

### 5.4 Observability, unchanged from the previous handoff

Add `ReceiverSetup` (5902), `ChannelStatus` (4013) and `SatVisibility` (4012) to a
**second** receiver SBF stream so `gps_dump` captures them with auto-config left on.
`SEP_SAT_INFO` does not do this — the driver only copies the satellite count into
`satellite_info` (`septentrio.cpp:1126-1131`). Without those blocks no log in this
project contains per-satellite azimuth/elevation/CN0 or the receiver's own
ambiguity/integrity diagnostics.

## 6. Two questions still open for the operator

1. Did anything physical change on 2026-09-08 — antenna mount, mast, rover, or the
   P0001 marker — particularly between the ~10:30 session and the afternoon? The
   416 mm within-day vertical spread is far outside Sep-03 and Sep-07 behaviour.
2. During `log_15`'s long holds (315 s / 788 s / 500 s), was the rover placed *on*
   P0001 or roughly 10 cm north of it? It decides whether the +96 … +108 mm dN in
   that log is placement or bias.
