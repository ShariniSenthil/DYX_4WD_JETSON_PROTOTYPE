# Handoff — EKF2 field validation, base-reference investigation, tooling

**Date:** 2026-09-08. **For:** whoever picks up the tuning-analysis thread
next. **Read first:** [2026-09-07_Handoff_EKF2_PDOP.md](2026-09-07_Handoff_EKF2_PDOP.md)
for the `EKF2_GPS_CHECK=829` background this session validated in the field.

## 1. `EKF2_GPS_CHECK = 829` — field-validated, holds

Set live via NSH this morning (`831 → 829`, saved, confirmed). Verified
against 12 ULogs (`EKF2_RUN_01/Ulogs`, ~630s total) and independently against
the full 97-minute `log_15`/`06_59_35.ulg` session:

- `reset_count_pos_ne`/`vel_ne`/`pos_d`/`vxy`/`vz`/`xy`/`z`: **zero movement**
  in every log.
- `dead_reckoning` fraction: **0.0%** everywhere — GNSS fusion never dropped.
- `estimator_status.gps_check_fail_flags` (the masked, decision-relevant
  flag): **0.0% nonzero** everywhere.
- `estimator_gps_status.check_fail_max_pdop` (the raw, unconditional
  diagnostic) still fires in some windows (37.5%/17.65% of two logs) — this
  is the **expected, not-a-failure** signal the handoff predicted.

**Verdict: the fix does exactly what it was designed to do.** It does not
touch the still-unexplained VDOP root cause (item below).

## 2. Base station reference — ruled out as the source of the position variability

This was today's big open question and it's now closed, cleanly, with two
independent lines of evidence:

- **Live RTCM 1005 decode**, direct from the wire (`scripts/rtcm_base_monitor.py`,
  new this session — bypasses PX4 entirely, reads `/mavros/gps_rtk/send_rtcm`
  and decodes the base ARP itself): **zero spread** in the base's lat/lon/height
  across a continuous 10-minute window (459 samples) and a later 15s recheck.
  `lat=13.072046300, lon=80.261925000, h=-78.2699m, station_id=0`, mountpoint
  `MP23960`.
- **Matches an independent RS3 (Emlid Reach) survey exactly**: `temp/mroad19_08_26.csv`
  and `temp/minside030926.csv` (both uncommitted, left in place) record the
  same base coordinate (`80.26192500 / 13.07204630 / -78.270`) from a survey
  session on **2026-08-19**, over two weeks earlier. Also confirms one of
  RS3's own surveyed points is an exact match to this repo's `P0001`
  (`13.18937434, 80.22219259`) — the mission target coordinates are
  legitimately surveyed, not arbitrary.

**Conclusion: the base's declared coordinate is rock-stable across weeks and
independently corroborated.** The variable position offsets observed today
(ranging ~5mm to >100mm from target, direction flipping between sessions —
left, then right) are **not** explained by a bad/drifting base reference. A
wrong-but-stable base would produce a wrong-but-stable rover position, not
one that changes session to session. The remaining mystery is downstream of
the base: in this rover's own reception, ambiguity resolution, or antenna
placement — not the correction source.

One more supporting fact, from directly comparing a ~90-100mm-off static
window against a ~16mm-off static window in `log_15`: **every internal
receiver quality metric was statistically identical between the two**
(`eph` 0.015m both, same HDOP/VDOP, same sat count, continuous RTCM use, zero
CRC failures in either). The receiver's own confidence signal cannot tell
"genuinely accurate" from "confidently wrong" — expected if the true source
of variability is something the receiver has no visibility into (antenna
placement precision, or a session-dependent ambiguity/multipath state).

⚠ Baseline length is ~13.7km (base to today's work site) — on the long side
for single-base RTK. Not proven relevant, but worth keeping in mind.

## 3. Reanchor / pivot investigation — clean toggle confirmed, dormant precise-recapture FSM found

- `post_pivot_reanchor_all_legs` (default `true`, overridden explicitly in
  `rover.launch.py:790` — **that override wins over any yaml**, so disabling
  it means editing the launch file, not just a param/yaml) is a clean,
  isolated single-flag toggle. Confirmed by tracing every read site: with it
  `false`, every leg except `C→P1` reverts to tracking only `/nav_path`
  (`following_runtime_line` false, `goal_signed_cross_track` driven purely
  by `/nav_path`). Three existing tests already pin this contract. No code
  change needed to get the production-sheet-P7 split (entry-leg reanchor
  allowed, every later leg fixed-geometry-only).
- Once reanchor is off, reconvergence after a pivot is **currently just
  ambient precision-guidance pure pursuit** — no dedicated post-pivot
  reacquisition step exists on the live path.
- **`VerifiedPivotStateMachine`** (`motion_state_machine.py:196+`) is fully
  built and wired (`rpp_controller_node.py:5042-5061`) but **disabled**
  (`precision_pivot_enabled=False`). States: `POSITION_CHECK → RECENTER →
  REALIGN → RECAPTURE → TRACK`, gated on measured xtrack/heading tolerances
  before resuming tracking. This is the closest existing match to what P7
  asks for ("reacquire and track the existing next leg" / "fail-recover-return"
  after pivot drift) — never field-validated, tolerances never checked
  against the measured 300-600mm pivot-walk magnitude.

## 4. Today's mission-accuracy spot check — the P2-terminal-swing pattern recurs

Two 10-point missions run back-to-back (`EKF2_run2/Bags`, same 10 physical
piles, second run reversed): **3/10 and 5/10 points** passed the 30mm survey
latch. Both missions are two 5-point rows joined by one row-transition pivot
at the same sequence position. In **both** runs, the point right after the
transition is fine, but points **2nd-4th after it** blow out (43-116mm)
before recovering by the last point — same shape, different physical piles,
opposite travel direction. This matches the still-open **"P2 terminal
swing"** finding from the 2026-09-01 session (`CLAUDE.md`) — not a new
defect, a second confirmation of an old open one.

**Four more missions collected this session, not yet analyzed**:
`Velocity_tuning/ekf_run_03/Bags/` (`mission.csv_20260908_172350` through
`_172843`) + `Ulogs/` (`log_20` through `log_30`, 17:06-17:31 IST). Next
session should run the same survey-vs-target check on these.

## 5. Infrastructure fixed and added this session

- **`mission_manager` was NOT actually symlink-installed**, contrary to
  this repo's own CLAUDE.md claim. `install/mission_manager/.../survey_truth.py`
  was a stale physical copy dated **Sep 4**, while `src/` had today's pulled
  patch (`522aa62`, `11a7d57`). Fixed with
  `colcon build --packages-select mission_manager --symlink-install`, which
  restored the editable/egg-link install (`build/mission_manager/mission_manager/`
  now resolves live to `src/`, confirmed byte-identical after every future
  edit). **Worth checking whether other packages have quietly lost their
  symlink the same way** — this one clearly did at some point without anyone
  noticing.
- That rebuild caused a **transient crash-loop** in `mission_manager_node`
  (`PackageNotFoundError: No package metadata was found for mission-manager`,
  5 respawn attempts, all failed) — confirmed to be a timing/race artifact
  from restarting mid-build, not a persistent break: re-tested the exact
  same metadata resolution moments later and it worked cleanly, and manually
  starting the node then succeeded immediately. If this happens again,
  retry the start rather than assuming the build broke something.
- **Three new permanent Jetson-side tools**, replacing one-off `/tmp` scripts
  written fresh each session:
  - `scripts/nsh_bridge.py` — reusable NSH-over-SERIAL_CONTROL CLI (committed, `2fa244b`).
  - `scripts/mavros_ftp_pull.py` — pulls files off the FCU SD card via
    MAVROS's own FTP plugin over the direct USB link (committed, `546bde7`).
    ⚠ Found to be **unreliable under heavy link load** — `SEP_DUMP_COMM=3` +
    continuous `SDLOG_MODE=2` logging can make even `FTP open` time out
    (`errno=110`, matches a previously-documented issue). Prefer the normal
    QGC download path for a large actively-growing log; use this tool for
    smaller/closed files or a quiet link.
  - `scripts/rtcm_base_monitor.py` — decodes RTCM 1005/1006 live off
    `/mavros/gps_rtk/send_rtcm` for the base's own ECEF/lat-lon-height,
    independent of PX4. **Not yet committed** (left as-is per operator
    instruction — commit when ready).
  - pymavlink deps for the NSH bridge now live permanently at
    `~/.local/share/nsh_bridge/deps` on the Jetson (survives reboots,
    replacing the old `/tmp`-based install that had to be redone every
    session).

## 6. Live FCU param state at end of session

| param | value | note |
|---|---|---|
| `EKF2_GPS_CHECK` | `829` | validated, see §1 |
| `SDLOG_MODE` | `0` | reverted back to default (armed-until-disarm) after the convergence testing was done; confirmed saved |
| `SEP_DUMP_COMM` | `3` | **left at "both directions"** from this morning's convergence-test setup — not reverted. Decide whether to leave it (useful for future raw-SBF debugging) or set back to `0` to reduce log volume/link load, given the FTP-reliability note above |

## 7. Open items, carried forward

1. **What actually causes the session-to-session position variability**,
   now that the base is cleared. Candidates not yet tested: antenna
   placement repeatability, ambiguity-resolution/multipath state specific to
   each cold start, the still-hardcoded Septentrio receiver dynamics
   (`srd,high,UAV` — confirmed still not configurable even in the newest
   available PX4 source, see 2026-09-08 driver investigation earlier this
   thread).
2. **Analyze the 4 new `ekf_run_03` missions** the same way as `EKF2_run2`.
3. **`EKF2_REQ_EPH` decision** — still needs operator sign-off, not just data
   (carried from 2026-09-07).
4. **VDOP degradation root cause** — still completely unexplained (carried
   from 2026-09-04/07).
5. **P2 terminal swing** — now seen a second time, still unsolved.
6. Whether to enable/tune `VerifiedPivotStateMachine` as the real P7 answer,
   now that it's confirmed built and dormant (§3).
