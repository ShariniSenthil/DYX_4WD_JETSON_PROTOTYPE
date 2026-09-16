# Multi-Node Health & Launch File

**Scope:** `src/rover_bringup/launch/rover.launch.py` — node launch/respawn configuration, cross-node failure propagation, what happens when one node dies.
**Method:** static code review, verified against current `HEAD` (not the 2026-09-15 baseline). `git diff ce25a4a HEAD` on this file is +36/-11, confirmed to be entirely RPP-controller tuning parameters (pivot angles, yaw rate limit) — no respawn/process-lifecycle line was touched.
**Grade at last full review (2026-09-15):** C+.

---

## P0 — still open, unchanged

- **mission_manager crash leaves the rover briefly ungated on the C→P1 entry leg.**
  `/mission_enable`/`/emergency_stop` are still `RELIABLE`/**`VOLATILE`** QoS (`cmd_vel_bridge.py:140-144`), stored as raw booleans with no last-received timestamp anywhere (`cmd_vel_bridge.py:180-188,372,375`; `rpp_controller_node.py:9521-9528`) — no staleness check exists on either topic in either consumer. On the C→P1 branch specifically, when `/active_waypoint` goes stale, `rpp_controller_node.py:9606-9624` does **not** stop — it falls through to a "P1 FALLBACK TARGET" and keeps driving toward the locally-cached goal, bypassing the one freshness check (`waypoint_timeout_sec`) that would otherwise catch this. If mission_manager dies mid-C→P1-leg while `rover_backend` keeps its own heartbeat alive, `cmd_vel_bridge`'s heartbeat backstop never trips either — it only watches the *backend* heartbeat, not a mission_manager-specific liveness signal (none exists).
  **Fix:** add staleness gating on `/mission_enable`/`/emergency_stop` reception, or drop the C→P1 fallback's exemption from freshness checking.

- **mission_manager still has zero persisted state.**
  See `Mission Manager.md`. A crash mid-mission loses all progress with no resume path.

## P1 — still open, unchanged

- **cmd_vel_bridge death has no ROS-level backstop.** Only `respawn=True, respawn_delay=2.0` at the launch level (`rover.launch.py:142-143`). No node subscribes to any cmd_vel_bridge-liveness signal; mission_manager only consumes `/cmd_vel_bridge/backend_heartbeat_healthy`, which reports *backend* heartbeat health, not cmd_vel_bridge's own liveness.
- **MAVROS liveness is only watched by mission_manager, and only while RUNNING.** `_monitor_runtime_px4_control()` opens with `if self._state != "RUNNING": return` (`mission_manager_node.py:1700-1701`). MAVROS dying while idle goes fully undetected.
- **No launch-level or ROS-native health aggregation exists.** Confirmed by fresh grep across `src/` for `/diagnostics|DiagnosticArray|diagnostic_updater` — zero real hits.

## P2 — new this pass

- **Duplicate dict key in `rover.launch.py` silently drops a value.**
  `legacy_pivot_post_settle_hold_sec` is defined **twice** in the same `rpp_controller` parameters dict: line 803 (`1.00`) and line 906 (`0.20`). Python dict literals silently let the later key win with no error or warning — the effective runtime value is `0.20`, not `1.00`, regardless of which was intended. This is a launch-file correctness bug independent of the RPP tuning question itself.
  **Fix:** remove the stale duplicate, confirm which value was actually meant to survive.

---

## Cross-check performed

Does the `111fb44` OFFBOARD/ARM ack-trust change (see `Mission Manager.md`) affect the C→P1 exposure window above? It **shortens** `_start_service`/`_resume_service` duration (removes ~1-2s of polling), which marginally *shrinks* the narrow window during which a crash specifically mid-arm/mid-OFFBOARD-transition could land. It does not change the P0 mechanism itself — that's about a crash at any point while RUNNING, most relevantly mid-C→P1-leg long after Start/Resume have already returned, where the VOLATILE-with-no-staleness-check gap is unbounded regardless of Start/Resume speed.

---

## What's already solid (don't touch)

- `RPP_EXPLICIT_YAW_ENABLED` and `TERMINAL_STOP_MODE` are each a single shared launch constant feeding both producer and consumer — can't disagree by construction.
- Static mission geometry (`/nav_path` etc.) is `TRANSIENT_LOCAL` — a respawned `rpp_controller` gets the full path back instantly, no republish race.
- `spray_controller` has its own independent staleness protection against a mission_manager outage — better protected than the drive path.
- Fail-safe-by-default is consistent everywhere checked — every node's uninitialized safety state defaults to blocked.
