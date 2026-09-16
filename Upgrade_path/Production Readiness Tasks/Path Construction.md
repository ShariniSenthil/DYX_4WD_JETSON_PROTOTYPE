# Path Construction — trajectory_generator

**Scope:** `src/trajectory_generator/trajectory_generator/trajectory_generator_node.py` — CSV/upload ingestion, coordinate projection, interpolation, row-transition/dummy-point geometry, prepare/control-loop state machine.
**Method:** static code review, verified against current `HEAD` (not the 2026-09-15 baseline). `git diff ce25a4a HEAD` on this file is +153 lines, confined to the row-transition/dummy-point geometry logic (`:118-131, 903-912, 1908-2154`) — genuinely re-read fresh, not just diffed.
**Grade at last full review (2026-09-15):** B — sound geometry (validated to 0.0mm against real data), one real crash path.

---

## P0 — still open, untouched by the recent changes

- **An unhandled `ValueError` from a corrupted-but-finite GPS sample can still crash the node mid-mission.**
  `_reference_is_ready()` (`:1326-1438`) is still called at `:2493`, **outside** the `try/except ValueError` block that starts at `:2522`. Inside it, `project_geodetic_to_px4_ned(...)` (`:1393-1401`) runs on `self.latest_fused_global_fix.latitude/longitude` with no surrounding guard. `localization_frame.py:46-54` raises a bare `ValueError` on any finite-but-out-of-range lat/lon. `_fused_global_position_callback` (`:839-845`) still stores every incoming `/mavros/global_position/global` message with **zero validation** — unlike its sibling `_gp_origin_callback` (`:709-727`), which explicitly checks finiteness/range/not-(0,0).
  A single corrupted-but-finite fused GPS sample (e.g. `latitude=999.0`) crashes the 5Hz control-loop timer callback — i.e. crashes the node — during a live mission. `respawn=True/2s` recovers the process but silently aborts any in-progress mission (mission_manager treats empty/missing `/nav_path` as invalidated).
  **Fix:** add the same finiteness+range validation used in `_gp_origin_callback` to `_fused_global_position_callback`, and move the `_reference_is_ready()` call inside the guarding try/except as defense in depth.

## P1 — new this pass: operator config is silently ignored

- **`dummy_point_distance_m` and `row_transition_threshold_m` from the frontend upload form are now dead on arrival.**
  `mission_routes.py:193,208` still accepts a frontend-supplied `dummy_point_distance_m` per upload, `mission_store.py:245-272` still validates and persists it into `mission_metadata.json`. But `trajectory_generator_node.py:900-912` (part of the +153 diff) now **hardcodes** `self.dummy_point_distance_m = EXTENSION_DISTANCE_M` (4.0m) and `row_transition_threshold_m = EXTENSION_TRIGGER_DISTANCE_M` (3.0m) whenever extension mode is enabled, ignoring the metadata values entirely. Compounding this: the backend's threshold is *still* env-overridable (`config.py:295-297,441-443`) — an operator changing that env var, or typing a custom distance in the upload form, sees it validated and stored, but trajectory_generator silently keeps using its own constants. No error, warning, or log indicates the override was dropped.
  **Fix:** either make trajectory_generator read these from metadata again (class constants as *defaults*, not silent overrides), or — if the hardening is deliberate — remove the now-dead frontend form field and backend validation path so it stops misleading operators.

- **The metadata validation for those same two fields is now pure dead code.** `:1078-1109` still parses, type-checks, and range-checks them (can reject an otherwise-valid mission on a bad value) — the resulting locals are never stored or used. Fix alongside the item above.

## P2 — new this pass

- **New `assert`-based None-narrowing is a latent non-`ValueError` crash path.** `:2089-2092` — `assert incoming_direction is not None` etc. If Python assertions are ever stripped (`-O`/`PYTHONOPTIMIZE`), a `None` reaches `_calculate_dummy_point` and raises `TypeError`, not `ValueError` — not caught by the control loop's `except ValueError` at `:2617`, crashing the node the same way the P0 above does. Same failure class, new trigger.
  **Fix:** replace the asserts with an explicit guard or explicit `ValueError`.

- **A rejected row-end candidate is now silently dropped, no log.** When `_extension_transition_geometry` (`:1928-1986`) determines a short transition isn't a genuine U-turn, it silently falls through to a plain straight segment — no diagnostic trail for an operator expecting a dummy point that didn't get generated, unlike the success case which logs a detailed "ROW TRANSITION" line (`:2129-2139`).

## P1/P2 — still open, unchanged

- **[P1]** Local-mode missions bypass every frame-consistency/GPS-quality/origin check. `_convert_markings_to_local()` (`:1802-1816`) just returns raw points with no equivalent of the GPS-mode origin-residual check.
- **[P1]** The GPS-quality gate is one-shot at PREPARE, not continuously enforced — `self.ready = True` after which the control loop returns immediately every cycle without re-checking fix quality.
- **[P1]** Zero dedicated tests for the node itself — only the low-level projection module (`test_localization_frame.py`) is tested.
- **[P2]** Dead WGS84-ellipsoid projection code (`_geodetic_to_ecef`, `_geodetic_delta_to_enu`, `_convert_markings_legacy`) still sits unreferenced — the same flawed method already proven wrong elsewhere in this project. Newly noticed: a second dead function, `_log_localization_shadow` (`:1665-1723`), zero call sites.
- **[P2]** Stale backup file siblings remain (`.backup_20260710_122727`, `.before_50mm_20260716_114241`).

---

## Confirmed unaffected by the upload/load endpoint split

`/upload` still unconditionally auto-triggers `ros_bridge.prepare_trajectory()` non-blocking (unchanged docstring: "return immediately... trajectory_generator owns the asynchronous RTK wait"). The new `/load` endpoint doesn't call `prepare_trajectory` at all — it only flips `accepted_for_start=True`, gated on `trajectory_ready` already being reported. `/prepare` remains available for explicit re-generation. trajectory_generator's own timing/staleness contract is unchanged.

---

## What's already solid (don't touch)

- Closed-form per-point interpolation, no accumulated error with path length — unchanged, still matches PX4's spherical projection exactly.
- Degenerate-input rejection at two layers (backend upload + generator).
- The `frame_id` contract with mission_manager is actively enforced, not just conventional.
- The new row-extension angle-check logic (transfer/reversal angle test replacing a cruder distance-only heuristic) is a genuine, positive improvement over the baseline.
