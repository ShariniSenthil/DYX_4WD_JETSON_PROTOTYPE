# Handoff — EKF2 PDOP / GNSS gate

**Last updated:** 2026-09-07 evening, for the **2026-09-08 morning session**.
**Rover was not available** for the second half of 2026-09-07, so nothing was
applied to the vehicle.

**Read first:** [2026-09-07_EKF2_GNSS_Gate_Production_Fix.md](2026-09-07_EKF2_GNSS_Gate_Production_Fix.md)
— that is now the authoritative doc for this thread; it carries the verdict, the
replay evidence and the source citations.
[2026-09-07_EKF2_PDOP_Gate_Root_Cause.md](2026-09-07_EKF2_PDOP_Gate_Root_Cause.md)
remains valid for the original VDOP attribution.
Broader vehicle work for the morning is in
[2026-09-07_4WD_Tomorrow_Plan.md](2026-09-07_4WD_Tomorrow_Plan.md) (lever arms,
mixer side mapping) — unrelated to this thread, don't conflate them.

---

## ▶ First action in the morning

**Set one parameter, then fly a normal session and look at the logs.**

```
param set EKF2_GPS_CHECK 829
param save
param show EKF2_GPS_CHECK
```

`829 = 831 − 2`, clearing the PDOP bit. **Change nothing else.** Do not touch
`EKF2_REQ_EPH`, do not touch `EKF2_REQ_PDOP`, do not flash firmware.

NSH access is over the MAVROS `SERIAL_CONTROL` route — see
[PX4_NSH_OVER_SSH.md](PX4_NSH_OVER_SSH.md). Record the pre-change value so the
revert is one command.

## Where this stands

**Decision made and evidence-complete; field validation is the only thing left.**

The full chain is now closed end to end: high VDOP → `PDOP = sqrt(HDOP²+VDOP²)`
exceeds `EKF2_REQ_PDOP` → `runGnssChecks()` fails → **the sample is discarded**
(`gps_control.cpp:79`) → a rejection run longer than **`reset_timeout_max`
(7 s)** calls `stopGnssFusion()` → recovery calls
`resetHorizontalPositionToGnss()` → **multi-metre reset**. Measured in `log_52`:
two runs of 8.5 s / 8.0 s and `reset_count_pos_ne` **7 → 9**, exactly the two
resets the operator saw.

`max_pdop` was the **only** check that failed anywhere in 21 logs, and **331/331**
PDOP failures had `HDOP ≤ 3.0`. Replaying `runGnssChecks()` over all 9,488
samples, `EKF2_GPS_CHECK = 829` takes the worst continuous rejection run from
**8.90 s to 0.00 s** and eliminates both resets, with nothing else changed.

**Status: approved for controlled field validation — NOT field-verified.** The
rover has never run with `829`.

### ⚠ This supersedes the previous version of this note

The earlier handoff said: *"The real open question is a combined RTK-fix-state +
tightened-EPH horizontal gate… not a one-bit removal."* **The data contradicts
that.** Every candidate that adds an EPH tightening or a fix-type requirement is
**strictly worse** on these logs than the one-bit removal. Do not restart from
that framing.

## What to expect in the validation logs

**`check_fail_max_pdop` will still appear.** `flags.pdop` is computed
unconditionally (`gps_checks.cpp:69`) and published to `estimator_gps_status`
(`EKF2.cpp:1241`); the mask only gates whether it makes the check fail. Judge on
consequences:

| signal | expected with `829` |
|---|---|
| `check_fail_max_pdop` | may still be true — **not** a failure |
| `"GNSS quality poor - stopping use"` | absent |
| `reset_count_pos_ne` | constant across the run |
| GNSS position fusion | active throughout |
| sustained PDOP-driven dead reckoning | none |

```bash
python3 scripts/ekf2_gnss_gate_report.py <new-ulog-dir>   # read the >7s and resets columns
python3 scripts/ekf2_gate_simulate.py   <new-ulog-dir>    # re-score candidates on new data
```

If a reset still occurs with `829`, the premise is wrong and the whole chain
needs re-deriving — say so rather than reaching for the next parameter.

## Do NOT do these

- **`EKF2_REQ_FIX = 6`.** `log_63` spent **95.4 continuous seconds** below RTK
  FIXED at 2–97 cm accuracy. This baseline checks every sample continuously
  (no relaxed in-flight path), so `6` turns that into a 95 s GNSS blackout and a
  guaranteed reset — it manufactures the fault being removed.
- **Bundle `EKF2_REQ_EPH 3.0 → 2.0` with the mask change.** `log_51` was
  **armed with GNSS fusion active 100% and `xy_global` true**, so this doesn't
  block a bad origin — it removes the vehicle's only global position for 53.7 s
  while armed, to fix a condition that caused **no** reset. It's a legitimate
  hardening but it is a separate capability decision needing operator sign-off.
- **Try to raise the stop-fusion timeout.** `reset_timeout_max` is a
  `const unsigned` (`common.h:478`) with **no parameter binding**. It cannot be
  changed without a firmware edit. `EKF2_NOAID_TOUT` is a *different* knob
  (`valid_timeout_max`, 5 s) that only marks the estimate invalid.
- **Add an estimator-level RTK requirement to pause the mission.** That already
  exists at the right layer: `mission_manager_node.py:1587` blocks motion unless
  `fix_type == 6`, and `:1634` transitions RUNNING → PAUSED with
  `_pause_reason = "RTK_LOST"`, no auto-resume.

## Corrections carried forward (don't re-litigate)

From the original investigation:
1. A `log_52`-only pass checking `fix_type`/`eph`/`epv` concluded "no GNSS
   quality problem" — wrong; the failure only shows against the PDOP gate.
2. The claimed GNSS yaw/height fusion drop to ~33.9% does not hold; yaw/height
   stayed 100% fused, only position/velocity dropped (to 51.5%). Don't cite it.
3. `EKF2_GPS_P_NOISE=0.01` and `EKF2_GPS_DELAY=50 ms` are cleared as
   contributors to these episodes specifically — not a general endorsement.

From the 2026-09-07 evening trace and its independent review:
4. **The receiver is a Septentrio mosaic-H, not a u-blox F9P.** `fix_type` comes
   from `PVTGeodetic.mode_type` (`septentrio.cpp:1089`), `eph` from
   `h_accuracy/200`. Any trace citing `carr_soln` or `NAV-HPPOSLLH` is describing
   a different vehicle.
5. **Upstream's strict/simplified check split is gated on `_initial_checks_passed`,
   not `in_air`** — a one-way latch that sets ~1 s after GNSS health, while still
   disarmed. The `RoverLandDetector → in_air` reasoning is irrelevant to it. And
   the flashed baseline `54f0455` has **no split at all**.
6. **The stop-fusion threshold is 7 s (`reset_timeout_max`), not 5 s
   (`EKF2_NOAID_TOUT`).** Three separate timeouts exist; see the fix doc §1.
7. **`check_fail_max_pdop` is logged regardless of the mask** — see the table
   above.

## Open items, in priority order

1. **Field-validate `EKF2_GPS_CHECK = 829`** — the morning's job. Until a
   session runs with it, the fix is analytically verified only.
2. **Why VDOP degrades at these specific times** — still unexplained. Geometry,
   sky obstruction, multipath or a receiver-internal effect all still open.
   Needs the raw Septentrio SBF stream; `SEP_DUMP_COMM = 0` today so it is not
   being logged. Same open item as the 2026-09-04 audit, now with a second
   recurrence as evidence it repeats. **The `829` fix stops VDOP from causing
   resets; it does not explain the VDOP.**
3. **`EKF2_REQ_EPH` decision** — whether the rover should refuse to provide a
   position with no RTK, or provide a degraded one. Data says `2.0` is the right
   value *if* the answer is "refuse" (fix-4 EPH 2.425–2.925 m occurs only in
   `log_51`, so it costs nothing elsewhere). Needs operator sign-off, not a
   quiet ride-along with item 1.
4. **`EKF2_REQ_FIX` backport** — patch exists at
   `docs/patches/0001-ekf2-backport-EKF2_REQ_FIX-onto-54f0455.patch`, applies
   cleanly, **but is not built, replayed, flashed or field-tested**. Only
   worthwhile as defense against the `log_82`-class confidently-wrong fix, and
   only at value **5**. Run it through the `px4-firmware-verification` skill
   before any flash.
5. **The yaw/height logging-rate asymmetry** (root-cause doc §3.5) — unchased.
6. **The `EKF2_GPS_DELAY` 40–60 ms timing-shift claim** — relayed, never
   independently re-derived.

## Repo state

Branch `feat/rtk-injection-v2`, pushed, at **`61abeb8`**. Three commits from this
thread:

| commit | what |
|---|---|
| `c4a4c45` | source trace, backport patch, `ekf2_gnss_gate_report.py` |
| `05b1e0a` | replay verdict — `829` alone; unbundled the EPH change |
| `61abeb8` | corrected 7 s `reset_timeout_max` and the `max_pdop` criterion |

Note `c4a4c45` proposed `829 + EPH 2.0`; **`05b1e0a` supersedes it.** Read the
current doc, not that commit's version.

## Data locations

- ULogs analysed: `~/Documents/QGroundControl Daily/Logs/4WD/Missions/`
  (`log_51`–`log_71`, all firmware `54f0455ffc`), 9,488 `vehicle_gps_position`
  samples.
- Firmware for all source citations:
  `/Users/dyx_a1/Vetri/Way_to_Mark/PX4-Autopilot-4WD-Prod-Baseline` at
  `54f0455ffcd755534539a7cf33a09a20bf71d29d`. ⚠ **That checkout's HEAD is
  `2f8ce71`, 964 commits ahead, and the file is renamed
  `gps_checks.cpp` → `gnss_checks.cpp` there.** Always `git show` at the baseline
  commit; reading the working tree gives the wrong architecture.
- The 7 production mission bags (`M1`–`M7`) are in `bags_jet/today/` and
  `bags_jet/archive_20260907.tar.gz` (both gitignored) — separate from the
  Missions-folder ULogs used for this trace.
