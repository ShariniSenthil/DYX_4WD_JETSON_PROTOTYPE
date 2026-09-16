# Mission Manager

**Scope:** `src/mission_manager/mission_manager/mission_manager_node.py` — orchestration, service execution (start/stop/pause/resume/skip/next), safety-generation e-stop contract, arm/OFFBOARD sequencing.
**Method:** static code review, verified against current `HEAD` (not the 2026-09-15 baseline). Two real commits landed since then: `7ba87cc` (resume-stage tracking) and `111fb44` (OFFBOARD/ARM confirmed from service ack, not a heartbeat poll).
**Grade at last full review (2026-09-15):** B−. No new P0s found in this pass.

---

## P1 — the OFFBOARD/ARM ack-trust change (`111fb44`) needs closing

Context: `_start_service`/`_resume_service`/`_next_point_service` used to poll `/mavros/state` (~1Hz PX4 heartbeat) to independently confirm a mode/arm change took effect, costing ~0.5-1s per confirmation. The new code trusts the SetMode/CommandBool service call's own `COMMAND_ACK` directly, setting `self._px4_mode`/`self._px4_armed` optimistically. This was almost certainly aimed at the field-reported "rover pauses itself frequently, pause-then-resume fixes it" symptom.

- **The ARM leg has zero recheck window at all.** OFFBOARD at least gets `OFFBOARD_BEFORE_ARM_SETTLE_SEC = 0.50s` (`:89`) of sleep before a recheck (`:2153-2156`). Nothing sleeps or rereads `self._px4_armed` between `_request_arm(True)` and `FINAL_CHECK`. If `CommandBool.success` were ever true without PX4 actually latching armed, the mission goes RUNNING and `_enable_motion()` fires with zero telemetry corroboration until the next `_control_loop` tick's `_monitor_runtime_px4_control()` call.
  **Fix:** add a short confirmation of `_px4_armed` before `FINAL_CHECK`, or at minimum assert it under the `FINAL_CHECK` lock (`:2164-2181` / `:2299-2316`) — it currently isn't checked there either.
- **`FINAL_CHECK` never actually re-verifies `_px4_mode`/`_px4_armed`.** Confirmed by reading `_require_motion_health`'s own checklist (`:1729-1768`) — it checks prepared-path/state/e-stop/RTK, not vehicle mode/arm state.
- **Needs field/library verification, not assertable from source alone:** whether MAVROS's `mode_sent` for `SET_MODE` is backed by a genuine PX4 `COMMAND_ACK` (like arming's `CommandBool.success` is) or is transport-level-only cannot be settled from this repo — `mavros_msgs` is an external dependency. If it's transport-level only, the 0.5s settle-then-recheck on the OFFBOARD leg is the *only* protection that exists anywhere in this sequence, and its reliability depends on a real heartbeat landing inside that 0.5s window. **Verify against the installed MAVROS version before trusting this in the field.**

**Net assessment:** this change likely *helps* the reported symptom (shortens the window a stream/telemetry gap could look like a spurious state change) but is **orthogonal to its actual root cause** — the `PX4_CONTROL_LOST`/`RTK_LOST` auto-pause watchdogs below are completely untouched by it and still never auto-recover. It also trades a small latency win for a real, currently-unquantified reduction in pre-RUNNING confirmation rigor on the ARM leg specifically. Needs a field test to fully close (deliberately induce a late/rejected arm after a good ack, if reproducible).

**`_resume_stage` bug history — confirmed fully fixed, correct version in place.** The "not paused" early-reject now does `self._resume_stage = "IDLE"; response.success = False; ...; return response` — a plain return, not a raise. Neither the original stuck-at-`PRECHECK` bug nor the intermediate regression (raising into the except handler, which called `_disable_motion_preserve_estop()` and killed motion on an actively-RUNNING mission for a merely-rejected Resume) is present. No further action needed here.

---

## P1 — still open, unchanged since 2026-09-15

- **An uncaught exception mid-point-completion can silently strand the mission in an infinite retry loop.**
  `mission_manager_node.py:3057` (`snapshot["survey"] = self._survey_truth_for_point(marking_number)`) chains unguarded into `compute_survey_truth` in `survey_truth.py:89,192`, which explicitly raises `ValueError` on non-finite coordinates or WGS84 geodesic non-convergence. `_capture_accuracy_snapshot` is called from `_control_loop:4150`, a bare timer callback with no top-level try/except anywhere in its ~480-line body. `self._point_status[marking_number] = "COMPLETED"` (`:4185`) sits *after* the vulnerable call, so a raise leaves point status un-advanced; the timer re-enters the same code path next tick and raises again, silently, forever. The rover sits stopped, looking exactly like normal in-progress marking — no operator-visible signal anywhere.
  **Fix:** wrap the completion tail in try/except, route into a visible ERROR state.

- **`_spray_ready_for_mission_start()` is still dead code.**
  `mission_manager_node.py:1559` — zero call sites in the file. A latched spray fault still neither blocks START nor pauses a running mission; it can reach COMPLETED having sprayed nothing for every point after the fault, traceable only via a per-point `spray_outcome=TIMEOUT`.
  **Fix:** wire it into the START precheck and add a mid-mission fault check in the marking phase, or delete the misleading dead code.

- **Zero persisted mission state.**
  No journal/checkpoint/sqlite/pickle write of any kind anywhere in the file. The new `_resume_stage` field is an in-memory RPC diagnostic, not persistence. A crash mid-mission still loses all progress with no resume path and no operator-facing "mission lost" signal distinct from a normal pause.

- **`terminal_stop_mode`'s class-declared default (`"legacy"`) still has no RPP liveness/staleness protection.**
  `:103,148,225-231` — only an explicit launch-file override to `"radial20"` (present in `rover.launch.py`, not this file) gets certificate protection. Any standalone/bench launch with default params silently loses this protection.

---

## What's already solid (don't touch)

- The e-stop generation contract — capture-then-revalidate under lock, compensating disarm on a superseded operation.
- The classic ROS2 service-call-blocks-its-own-callback deadlock is correctly avoided by design.
- `_monitor_runtime_px4_control()` genuinely never auto-recovers, leaves a clean resumable `PAUSED` state.
- The RTK-float-warning-only change (`ce25a4a`) — evaluated and confirmed safe, not a regression; entry gates for START/RESUME/NEXT still hard-require FIXED.
