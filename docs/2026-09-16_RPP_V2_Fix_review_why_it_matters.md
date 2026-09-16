# Why `8d7c763` ("RPP_V2_Fix") needs follow-up before it's the baseline

| Field | Value |
|---|---|
| Date | 2026-09-16 |
| Commit reviewed | `8d7c763` "RPP_V2_Fix" (dyxsatheesh), applied via tooled patch against baseline `75299d9c` |
| Files | `src/rpp_controller/rpp_controller/rpp_controller_node.py`, `src/rpp_controller/rpp_controller/legacy_alignment.py`, `src/rover_bringup/launch/rover.launch.py` |
| Status | Code-reviewed, not yet field-verified |

This is not a rejection of the commit. The core mechanism it adds (a tighter re-entry threshold for heading rebounds during a pivot's settle phase) is a real, correctly-implemented improvement. This note is about the parts that need attention *before* this becomes what everyone builds on top of — because two of them are exactly the failure pattern this codebase has already been burned by more than once.

---

## CRITICAL — verify before trusting in the field

### `legacy_pivot_post_settle_hold_sec`: 1.00s → 0.20s

**Where:** `rover.launch.py:512` (was `1.00`, now `0.20`).

**Why this specific number is not a free parameter to tune from a desk.** This exact value was the subject of a dedicated field investigation on 2026-09-01, already written up in this repo's own history:

> "Release always landed at exactly (first sustained gate crossing) + 1.20s — the `settle_sec 0.20 + post_settle_hold_sec 1.00` budget was never wrong."

That session measured the pivot-settle timing across 26 real pivots, found the *gate* (when rotation is detected as stopped) was the actual bottleneck — not the hold — and left `post_settle_hold_sec` at `1.00` deliberately, having confirmed it wasn't the problem. This commit reduces it 5x without a comparable measurement.

**Why the new 6° rebound watchdog isn't a substitute.** `legacy_alignment.py:382` now re-triggers a pivot if heading rebounds ≥6° during settle. That's a real safety net for one failure mode: the rover physically swings back after finishing a turn. It is not a safety net for the *other* documented failure mode this repo has already characterized in detail — EKF/estimator ringing after a pivot, where the reported velocity or attitude keeps moving for up to several seconds without the actual heading being wrong. A shorter hold releases sooner into whatever the estimator is doing at that instant, regardless of whether it's genuinely settled. The 6° gate cannot see that failure mode because it's watching heading error, not settle confidence.

**What "essential to fix" means here:** not necessarily "revert to 1.00s." It means: re-run the same class of measurement the 2026-09-01 session did — real pivots, real bags, checking whether `0.20s` produces the same clean release behavior or reintroduces the exact problem that session was measuring in the first place — before this ships as the default everyone flies with.

---

## HIGH — fix before this becomes the reference point everyone reasons from

### `pivot_exit_angle_deg` silently stopped being cosmetic

**Where:** declared at `rpp_controller_node.py:365`; now wired into `LegacyAlignmentConfig.settle_reentry_heading_rad` (`legacy_alignment.py:70`).

**Why this is worse than a normal parameter change.** Before this commit, `pivot_exit_angle_deg` was *decorative* — used only in two log-message strings and one startup sanity check, never in a control decision. That was verified directly, by reading every call site, earlier in this same review session. This commit repurposes it into a real, functionally load-bearing threshold: the heading-rebound re-entry gate during pivot settle. Nothing marks that the parameter's job changed. The name didn't change. No comment at the declaration flags it. The two log lines that already described this value in a *different*, still-unenforced context (`rpp_controller_node.py:2228`, `2232`, "Forward-heading correction" / "Forward marking gate") are untouched and now describe neither the old meaning nor the new one accurately.

**Why this specific kind of change is dangerous in this codebase.** This repo has an extensive, self-documented history of exactly this failure pattern — multiple named thresholds that *look* like they should be the same conceptual value but silently disagree (the well-known case: `RD_TRANS_TRN_DRV` on the FCU at 6°, `pivot_exit_angle_deg` in config, and a code comment claiming 4°, all three different, "predates this session, unexplained"). Every one of those confusions cost real debugging time to untangle. This commit adds a fourth version of the same failure shape: a parameter whose behavior changed without its identity changing. The fix is small — rename it (e.g. `settle_reentry_heading_deg`) or put an unmissable comment at the `declare_parameter` call — and it closes off a class of bug this team has already paid for more than once.

---

## MEDIUM — worth doing, not blocking

### The rebound fix depends on an invariant nothing protects

**Where:** `rpp_controller_node.py:5666` (`reentering_from_settle` check), depends on `legacy_alignment.py`'s `native_carrier_issued` flag surviving a transition that a *different*, similarly-named flag (`terminal_native_pivot_active`, on the controller node itself) does not survive.

Traced end-to-end: this is correct as written today. `_enter_pre_pivot_stop()` resets the controller-level flag (via `reset_native_carrier=True` in its result) but deliberately does not call the lifecycle's full `reset()`, so `native_carrier_issued` persists across the rebound transition — which is exactly what the new logic needs to tell "this is a settle rebound" apart from "this is a brand-new pivot." Why it matters anyway: two flags with near-identical names and different owners, where correctness depends on one resetting and the other not, is the textbook setup for a future "cleanup" to break silently. A one-line comment at `native_carrier_issued`'s declaration explaining why it must *not* be reset by `_enter_pre_pivot_stop` costs nothing and prevents a regression nobody would think to test for.

### The new startup-time validation is an undocumented new failure mode

**Where:** `legacy_alignment.py:104` — `native_release_heading_rad < settle_reentry_heading_rad < pivot_enter_rad`, enforced in `__post_init__`.

This is good practice (fail fast on an inconsistent config) and I'd keep it. But before this commit, any relationship between these three angle parameters was merely "maybe poorly tuned." After it, a bad combination — including the exact `6.0`/`6.0` pairing this session's own earlier fix would have produced — makes `rpp_controller` refuse to start at all. That's a legitimate improvement, but it's a new way the node can fail in the field, and nothing in the commit calls it out. Worth a line in whatever runbook covers these three parameters.

---

## Not a bug, but say it out loud so it isn't mistaken for one

This commit does **not** fix the originally reported issue (heading error surviving a Pause/Resume with no correction). It fixes a narrower, related problem: rebound *during a pivot's own settle phase*, before release to cruise. The pre-existing 45° `MID_LEG_REENTRY` gate that governs re-correction *after* release to cruise (or across Pause/Resume) is untouched. If this commit gets referenced later as "the pivot-heading fix," that's the correction that needs making at the same time — otherwise the original report will look closed when it isn't.

---

## Not urgent

Three full-file backup snapshots (`*.pre_abs_yaw_v2_20260916_163027`, ~12,150 combined lines) were committed into git history. This repo already has an open item flagging ~104 dead `.before_*`/`.bak_*` files for deletion for exactly this reason. Not a bug, just don't let the pile grow — delete them from the branch when convenient.
