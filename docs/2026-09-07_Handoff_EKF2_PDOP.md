# Handoff — EKF2 PDOP Gate Investigation

**Date:** 2026-09-07. **For:** whoever (Opus or otherwise) picks this thread
up next. **Read first:** [2026-09-07_EKF2_PDOP_Gate_Root_Cause.md](2026-09-07_EKF2_PDOP_Gate_Root_Cause.md)
- this note is a pointer/summary, that doc has the full source citations and
data. Also relevant: [2026-09-07_EKF2_Logs_and_Mission_Bags_Findings.md](../2026-09-07_EKF2_Logs_and_Mission_Bags_Findings.md)
(the original mission-bag pass this grew out of) and the 2026-09-04/09-05
EKF/GNSS docs (the pre-existing vertical-anomaly investigation this connects
to).

## Where this stands

**Root cause of today's two multi-metre EKF position resets (pre-M1,
pre-M5) is confirmed, sourced end-to-end from firmware, and cross-checked
against 21 ULogs:** `EKF2_REQ_PDOP` (live `3.0`) gates on
`PDOP = sqrt(HDOP^2+VDOP^2)`, and on this rover today **every single PDOP
rejection was caused by VDOP** - HDOP never exceeded 2.36 anywhere in 940
samples across the whole session, while VDOP was the larger term in 30 of
31 failing samples. The strongest single piece of evidence: at boot
(`log_51`) the estimator fused a 2.7 m-accuracy fix because PDOP was fine,
then later (`log_52`) rejected a 2.5 cm RTK-FIXED sample solely because PDOP
exceeded 3.0. **No parameter has been changed.**

**What this is not:** a simple "just disable the PDOP check" conclusion.
`EKF2_REQ_EPH=3.0 m` is the only other accuracy gate that would stand in
PDOP's place, and it's proven (by the same `log_51` fix) to be far too loose
- it would accept the 2.7 m fix too. The real open question is a
**combined** RTK-fix-state + tightened-EPH horizontal gate, decoupled from
VDOP, not a one-bit removal.

## What got corrected along the way (worth knowing so it isn't re-litigated)

1. An early pass at `log_52` alone (checking only `fix_type`/`eph`/`epv`)
   concluded "no GNSS quality problem visible" for the pre-M1 window. Wrong
   - the failure only shows up when HDOP/VDOP are checked against the actual
   `EKF2_REQ_PDOP` gate. Corrected in the root-cause doc.
2. A claim that GNSS yaw/height fusion also dropped to ~33.9% during the bad
   window was checked directly and does not hold - yaw/height stayed 100%
   fused; only position/velocity dropped (to 51.5%, not 33.9% either). Do
   not cite the yaw/height drop going forward; the doc's §3.5 has the
   correction and the likely (unconfirmed) explanation - a logging-rate
   asymmetry between how those topics get published on rejection.
3. `EKF2_GPS_P_NOISE=0.01` and `EKF2_GPS_DELAY=50 ms` are both cleared as
   contributors to today's episodes specifically (not a general endorsement
   of either value on other grounds - see the existing 2026-09-05 note
   about `GPS_P_NOISE` being far below the *correlated* GNSS error even
   though it's fine against raw jitter).

## Open items, in priority order

1. **The actual parameter/gate decision** (drop PDOP entirely vs. replace it
   with fix_type + tightened EPH vs. something else) needs to go through the
   task-sheet process - this thread has produced the evidence, not the
   decision.
2. **Why VDOP degrades at these specific times** is still unexplained -
   satellite geometry, a physical sky obstruction, antenna multipath, or a
   receiver-internal effect are all still open. Needs the raw Septentrio SBF
   stream (`SEP_DUMP_COMM=0` currently, so it isn't being logged) - this is
   the same open item the 2026-09-04 audit already flagged and it is still
   open today, now with a second independent recurrence as evidence it's
   real and repeating.
3. **The yaw/height logging-rate asymmetry** noted in §3.5 was not chased
   down - worth a quick look if anyone wants to use per-aid-source fusion
   percentages as evidence again, so the denominator question doesn't
   resurface unexplained.
4. **The `EKF2_GPS_DELAY` timing-shift claim** (40-60 ms) was relayed, not
   independently re-derived in the root-cause doc - if it matters for a
   decision, re-derive it directly the way the HDOP/VDOP attribution was
   done, rather than citing it as-is.

## Data locations

- ULogs analysed: `~/Documents/QGroundControl Daily/Logs/4WD/Missions/`
  (`log_51` through `log_71`, all firmware `54f0455ffc`).
- Firmware checkout used for all source citations:
  `/Users/dyx_a1/Vetri/Way_to_Mark/PX4-Autopilot-4WD-Prod-Baseline`,
  commit `54f0455ffcd755534539a7cf33a09a20bf71d29d` (`git show <path>` at
  that commit for any line number cited in the root-cause doc).
- The 7 production mission bags (`M1`-`M7`, same session) are pulled locally
  to `bags_jet/today/` and `bags_jet/archive_20260907.tar.gz` (both
  gitignored, not in the repo) - separate from the Missions-folder ULogs
  used for this EKF2 trace, which came from QGC's own log pull.
