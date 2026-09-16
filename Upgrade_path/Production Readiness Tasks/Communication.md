# Communication — Frontend ↔ Backend

**Scope:** `rover_backend` (FastAPI + Socket.IO) and its consumption contract with the DYX GCS frontend — upload/load/prepare handoff, mission-state serialization, REST/socket error contracts.
**Method:** static code review, verified against current `HEAD` (not the 2026-09-15 baseline) — every finding below re-checked against live source, not carried over blind.
**Grade at last full review (2026-09-15):** B+. No P0/P1s found in the changes since then — this layer has stayed the most solid part of the stack.

---

## P2 — Fix soon, no safety impact

- **Shallow-copy `section()`/`snapshot()` still leak live nested mutable objects by reference for `mission`/`position`.**
  `state.py:619-651, 653-684`. The `a36ca64` revert (MappingProxyType → plain `dict()`, correctly done to fix a real FastAPI/pydantic serialization crash) reopened a narrower version of the original bug: a shallow copy protects the top-level dict but not nested fields.
  `system_routes.py:415-421` (`build_mission_status_payload`, used by every REST mission-status response and the `mission_status`/`mission_progress` Socket.IO broadcasts) and `mission_routes.py:430-433` (`GET /loaded-path`) embed `point_results`, `point_status`, `navigation_path_preview` **directly by reference**.
  Verified: every current write-path caller (`mission_routes.py`, `mission_report.py`, `system_routes.py`, `ros_bridge.py`, `realtime.py`) consistently rebuilds a fresh dict/list before mutating (e.g. `ros_bridge.py:2908` `point_results = dict(mission.get("point_results") or {})`), so nothing is broken *today*. But it's unguarded — a future caller doing an in-place nested mutation (`section["point_results"]["5"] = x`) would silently corrupt shared state with no lock.
  **Fix:** deep-copy just these three known-mutable nested fields on the way out of `section()`/`snapshot()`, or add a debug-mode assertion that flags in-place mutation.

- **Non-atomic `accepted_for_start` reset on upload.**
  `mission_routes.py:255-258` resets `accepted_for_start=False` in a separate `rover_state.update()` call, executed *after* `mission_report_store.save_new_mission()` has already committed the new `mission_id`/`state=LOADED`. Since FastAPI runs sync `def` routes (e.g. `GET /api/mission/status`) in a thread-pool thread, a concurrent status read landing in the gap could observe the new `mission_id` paired with the *old* mission's `accepted_for_start=True`.
  Not practically exploitable (`/start` re-reads state fresh at call time; network latency dwarfs the gap) but a real non-atomicity.
  **Fix:** fold the reset into `RoverState.load_mission()` (`state.py:876-948`) so mission-id assignment and un-staging happen under one lock acquisition.

- **EMPTY→LOADED/COMPLETED remap can't distinguish a Stop/Complete blip from mission_manager losing its trajectory.**
  `ros_bridge.py:2471-2481`. The remap masks a transient EMPTY that trajectory_generator publishes mid-regeneration — but the same condition (`state_name == "EMPTY" and retained_mission_id and retained_staged`) matches equally if mission_manager restarts with no committed path. The backend would keep reporting `LOADED`/`staged: true` with no signal that a fresh `/prepare` is actually required.
  Real safety gate (mission_manager's own `start` service) still rejects correctly — this is an operator-diagnostics gap, not a safety one.

- **Post-Stop, `/start`'s backend-side precondition only checks `accepted_for_start`, not `trajectory_ready`.**
  `state.py:1160-1161` (`clear_mission_runtime`) resets `trajectory_ready=False` on Stop but retains `accepted_for_start`. `mission_routes.py:554-560` gates `/start` on `accepted_for_start` only. An immediate post-Stop `/start` before re-`/prepare` passes the backend's own check and only fails downstream via mission_manager, producing a generic 409/504 instead of a locally-generated "mission not ready, re-prepare first" message.

- **`staged`/`staged_id` derivation logic is duplicated.**
  Independently implemented in `mission_routes.py:88-90` (`_mission_state()`) and `system_routes.py:266-273` (`build_mission_status_payload()`), both with a "keep in sync" comment. Extract to one shared helper.

---

## Still open, frontend-only (unchanged since 2026-09-15)

- **E-stop release "outcome unknown" (504) collapses into a generic error on the frontend.** Backend contract (`mission_routes.py:149-165`) is correct — `outcome: "UNKNOWN"|"FAILED"`, `retry_safe` — the frontend's `ApiError` never parses it out. No backend change needed.
- **No live operator surface for RPP/mission_manager/spray internals on the dashboard.** `rpp_debug_*`, `spray_controller_state`, `spray_fault_reason` all still flow over the socket unchanged — nothing renders them.

---

## What's already solid (don't touch)

- Telemetry/state separation off the hot path — `rover_state` reads never block on ROS I/O.
- Every ROS service call is bounded with a real three-way outcome (success / clean failure / outcome-unknown).
- The safety-generation race handling in `release_emergency_stop` is genuinely careful, and the 2026-08-31 wrong-section bug fix has held with no recurrence found.
- Frontend reconnection (hand-rolled backoff, generation guard, full snapshot on reconnect) — no changes needed.
- The new `accepted_for_start`/`/load` split is a real, working safeguard — it closes the gap where upload used to auto-prepare *and* implicitly allow an immediate Start with no explicit operator confirmation step.
