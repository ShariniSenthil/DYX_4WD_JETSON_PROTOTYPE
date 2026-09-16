# Systemd, Watchdog & Process Supervision

**Scope:** boot-time autostart, process respawn/backoff policy, node-liveness (hang, not crash) detection, `systemd/bag-autorecord.service`.
**Method:** static code review, verified against current `HEAD`. `git diff ce25a4a HEAD -- systemd/` is **empty** — confirmed nothing in this category has changed since the 2026-09-15 baseline review, including the RTK lifecycle files (`rtk_manager_core.py`, `rtk_runtime_orchestrator.py`, `rtk_process_adapter.py`, `rtk_runtime_service.py`) which also show zero diff.
**Grade at last full review (2026-09-15):** D+ — the weakest category in the whole stack, and still is.

---

## P0 — still open, unchanged since 2026-09-15

- **MAVROS has zero auto-restart.**
  `rover.launch.py:89-111` — a plain `ExecuteProcess` with no `respawn` kwarg at all (the only action type in this file that doesn't support one). A serial hiccup or USB device renumbering kills MAVROS and nothing brings the FCU link back short of a full manual stack restart.
  **Fix:** add an `OnProcessExit` handler, or move MAVROS under its own systemd unit with `Restart=on-failure`.

- **`rover_backend` is `respawn=False`, by explicit deliberate design comment.**
  `rover.launch.py:113-122` — "the operator can inspect the error without a bind-failure respawn loop." Any unhandled exception kills the one process hosting the API, e-stop release, and RTK control, with no watcher — the app goes permanently blank.
  **Fix:** flip to `respawn=True, respawn_delay=2.0`, matching its five sibling nodes. The debuggability trade-off made sense during active development; it's backwards for an unattended demo/field posture.

## P1 — still open, unchanged since 2026-09-15

- **No boot-time auto-start for the ROS2 stack anywhere in the repo.** Confirmed absent by fresh, repo-wide grep — no `@reboot`, no `WantedBy=`/enable path for the core stack anywhere in `src/`, `systemd/`, or `scripts/`. A Jetson power cycle brings up nothing but the OS (and the bag recorder, if installed). **The single highest-leverage fix in this whole review.**
  **Fix:** one systemd unit (`dyx-rover.service`) running `ros2 launch rover_bringup rover.launch.py` as user `flash`, `WantedBy=multi-user.target`, bounded `Restart=on-failure`.

- **`systemd/bag-autorecord.service` has no in-repo install/enable script.** Nothing copies the unit to `/etc/systemd/system/`, runs `daemon-reload`/`enable --now`, or creates its `EnvironmentFile` — only a `.example` is checked in. If the SD card is ever reflashed, the repo looks complete while the live system silently isn't reproducible from it.

- **Flat 2.0s respawn delay on all five protected nodes, no backoff, no crash-loop ceiling.** `cmd_vel_bridge`, `trajectory_generator`, `spray_controller`, `mission_manager`, `rpp_controller` — all still `respawn=True, respawn_delay=2.0` verbatim (`rover.launch.py:142-143, 169-170, 223-224, 271-272, 323-324`). No max-retry counter or backoff multiplier exists anywhere, unlike `bag-autorecord.service`'s own `StartLimitBurst`, or the RTK manager's real exponential backoff. A persistently crash-looping node is invisible to anyone not tailing a terminal.

- **No general node-liveness (hang, not crash) watchdog exists anywhere.** Confirmed fresh — no new mechanism added. The only "heartbeat"-named constructs found are application-level data-staleness checks (`cmd_vel_bridge`↔backend heartbeat, mission_manager's RPP-certificate freshness timeout) — neither restarts or flags a hung *process*.

---

## Already documented as a proposal, not yet implemented

`docs/PRODUCTION_READINESS_REVIEW.md` (added 2026-09-15, the "Gemini report") independently re-derives this exact P0/P1 list and proposes concrete `dyx-rover.service`/`dyx-backend.service` systemd units with `Restart=always`/`RestartSec=3`, plus flipping `rover_backend_node` to `respawn=True` and wrapping MAVROS in a respawn handler. **This is a ready-to-execute punch list — no code or systemd file has actually been added or changed.** Treat that document's §7 as the starting draft for this category's fixes rather than writing new unit files from scratch.

---

## What's already solid (don't touch)

- `RtkManagerCore` — real exponential backoff, tracked failure/restart counters, a documented terminal `ERROR` state that deliberately does not retry forever and is surfaced through the API. The one genuinely well-built supervision pattern in this repo; it just hasn't been applied anywhere else.
- `respawn=True` on the five mission-logic nodes is real, working crash recovery, even if unbounded.
- `bags_jet/` disk retention (preflight floor + oldest-first eviction) is a real, working disk-exhaustion guard.
- `bag-autorecord.service` itself, as a unit definition, is well-written — bounded restart, sandboxed, resource-capped, deliberately decoupled from the core stack.
