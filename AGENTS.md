# DYX 4WD Marking Rover — Jetson Companion

Scope: ROS2 Humble colcon workspace on Jetson `192.168.3.101`, PX4 OFFBOARD
control of a 4-wheel skid-steer marking rover. Modeled on the sibling 3WD
project's AGENTS.md (`PX4_DXP/AGENTS.md`) — **do not assume any fact from
that file applies here.** The two rovers share almost no infrastructure:
different repo, different Jetson, different FCU, different motor
controllers, different subnet. Every fact below was verified against this
repo's own source/params/bags on 2026-08-31 (agent research + direct bag
decoding); citations are file:line where available. **A full field session on
2026-09-01 added §"2026-09-01 field session" below and corrected several
facts — read that section before trusting anything speed- or pivot-related
elsewhere in this file.**

> **This file is new (2026-08-31).** It has survived exactly one field
> session's worth of corrections (2026-09-01), not the many that
> `PX4_DXP/AGENTS.md` has — treat anything that later turns out wrong as an
> invitation to fix this file, not as a reason to distrust the whole thing.

## PX4 NSH through SSH — verified 2026-09-07

**Read [the NSH/SSH runbook](docs/PX4_NSH_OVER_SSH.md) before using PX4
`param`, `listener`, or `logger` remotely, or repeating the ground symmetry
test.** It contains the exact transport code, successful test harness,
execution result, and observed failure cases. This method does not require
a stack restart or another connection to the FCU serial device.

- SSH reaches **Jetson Linux**, not PX4 NSH. The verified route is
  `flash@192.168.3.101` → ROS2 Humble → publish
  `mavros_msgs/msg/Mavlink` on `/uas1/mavlink_sink` → existing MAVROS serial
  connection → MAVLink `SERIAL_CONTROL` → PX4 NSH. Replies arrive on
  `/uas1/mavlink_source`. Do not open `/dev/ttyACM0` alongside MAVROS.
- Exact shell settings: MAVLink 2, message ID `126`, shell device `10`,
  RESPOND flag `2`, timeout/baudrate fields `0/0`, encoder
  `srcSystem=1, srcComponent=191`. Newline-terminate commands, split at
  70 bytes and zero-pad; use pymavlink plus `mavros.mavlink` conversion
  helpers. Successful raw receive QoS: best effort / volatile, depth 1000.
- Source `/opt/ros/humble/setup.bash`; temporary dependencies were installed
  with `python3 -m pip install --target /tmp/ground_symmetry_deps pymavlink`.
  Set `MAVLINK20=1` before imports, then add that directory with
  `sys.path.insert`; do not overwrite the ROS `PYTHONPATH`.
- Foreground test SSH options were
  `ssh -tt -o BatchMode=yes -o ServerAliveInterval=2 -o ServerAliveCountMax=3`,
  with a local PTY and `python3 -u`. Wait for the echoed NSH command and a
  subsequent `nsh>`; send shell `\x03` to stop a listener, then an empty
  SERIAL_CONTROL message with flags 0 when closing the shell.
- NSH `ver all` confirmed PX4 1.16.2 at
  `54f0455ffcd755534539a7cf33a09a20bf71d29d`. Use that exact commit for
  receiver/mixer inspection, not a newer checkout's working-tree files.
- For the **authorized ground test only**, temporarily set/save
  `COM_RC_IN_MODE=1` while **disarmed**, then start
  `/mavros/manual_control/send` (`mavros_msgs/msg/ManualControl`) at 50 Hz
  with `x=0, y=0, z=500, r=0`. Steering is **y**, in raw MAVLink units:
  LEFT = -400/-550/-700, RIGHT = +400/+550/+700. **z=0 is full reverse**.
  `/mavros/actuator_control` is unsupported by this PX4 receiver; the existing
  `scripts/acro_yaw_manual_control_test.py` has the wrong mapping for this test.
- Verify fresh, valid PX4 `manual_control_setpoint` with
  `listener manual_control_setpoint -n 100000 -r 5` (5 Hz observation,
  separate from 50 Hz publication). Then enter MANUAL **while disarmed**,
  confirm 3 seconds stable with neutral/no reported failsafe, have the operator
  arm manually, and hold neutral another 3 seconds before steering.
  Never recover an armed HOLD attempt by forcing MANUAL and continuing.
- Keep neutral publishing on completion/abort/Ctrl-C, disarm, check the RC
  arm switch is DISARM, then `param set COM_RC_IN_MODE 0` → `param save` →
  `param show COM_RC_IN_MODE`; verify saved 0 and disarmed again. An earlier
  attempt **rearmed from the RC switch when input authority was restored**.
- E-stop release uses authenticated `POST /api/estop/release` and the manager's
  generation guard. The operator's live DYX app had **no RELEASE button**,
  despite one existing in local frontend source. Do not assume deployed UI
  parity. Do not publish `/emergency_stop=false` or clear a new stop automatically.
- The complete successful ULog is
  `/fs/microsd/log/2026-09-07/07_34_22.ulg`. No failsafe occurred during its
  timed sequence; PX4 printed “Preflight Fail: No CPU and RAM load information”
  **after disarming**. That message remains unexplained. Earlier FTP/parameter
  pull timeouts were observed, but their causal role in failsafe was not proven.
  Use NSH and logger announcements during the test; do bulk reads beforehand.

## Hardware

| Item | Value | Source |
|---|---|---|
| Jetson IP | `192.168.3.101` (user `flash`) | `src/rover_backend/config/backend.env` |
| GCS UDP target | `192.168.3.105:14550` | `src/rover_bringup/launch/rover.launch.py:50` |
| OS | ROS2 Humble (EOL May 2027) | `REFACTOR_TO_CPP_AUDIT.md` |
| FCU | **Holybro Pixhawk 6X** — NOT a CubeOrangePlus | `src/spray_controller/spray_controller/spray_controller_node.py:6`, `rover.launch.py:174` |
| FCU connection | `/dev/ttyACM0` @ 921600, `tgt_system=1 tgt_component=1 fcu_protocol=v2.0` | `rover.launch.py:47-59` |
| Drive | 4-wheel, **dual Sabertooth 2X32** motor controllers via PX4 PWM main outputs (`PWM_MAIN_FUNC1..4`), `MAV_TYPE=10` (ground rover) | `px4-params/*.params` (untracked local exports) |
| RoboClaw | Present in firmware, **disabled** (`RBCLW_SER_CFG=0`) — this rover does NOT drive RoboClaw despite `WORKING_STATUS_24_06_2026.txt` (stale, 2026-06-24) claiming "PX4 OFFBOARD → RoboClaw → wheels" | `px4-params/*.params` |
| Wheel track | `RD_WHEEL_TRACK = 0.63 m` | `px4-params/*.params` |
| | Full NTRIP client (`rtk_correction_bridge`), managed as a supervised child process by `rover_backend`, confirmed FIXED and live 2026-08-31 evening | see RTK section below |

## Control architecture — verified against live source, NOT the stale status doc

```
rpp_controller  →  /rpp/velocity_ned (Vector3Stamped, m/s, North/East)
                →  cmd_vel_bridge (jetson_4wd_control pkg)
                →  /mavros/setpoint_raw/local (PositionTarget, FRAME_LOCAL_NED, velocity-only mask)
                →  PX4 OFFBOARD
                →  Sabertooth 2x32 (via PX4 PWM main outputs)
```
(`cmd_vel_bridge.py:127-132, 158-162`)

Since 2026-09-01 the line `rpp_controller` tracks is not always `/nav_path`:
after any pivot certifies, it rebuilds a straight runtime line from where the
pivot actually left the rover to the current segment goal, and follows that
for the rest of the leg (`post_pivot_reanchor_all_legs`, default true). The
goal is never moved and `/nav_path` is never modified — see the 2026-09-01
session section for why this is safe and what it fixed.

**`WORKING_STATUS_24_06_2026.txt`'s architecture is stale** — it describes a
`corner_controller` node that no longer exists anywhere in `src/`
(superseded by `rpp_controller`) and a `/cmd_vel`→`/mavros/setpoint_velocity/cmd_vel`
path that isn't what's wired today. **`REFACTOR_TO_CPP_AUDIT.md`'s own ASCII
diagram is also simplified/inaccurate** on this point — trust the source
above over either document.

Key behaviors of `cmd_vel_bridge` (`src/jetson_4wd_control/jetson_4wd_control/cmd_vel_bridge.py`):
- Streams at **50 Hz unconditionally** (line 46, 185-188) — publishes zero or
  the live command, never stops. This is how the "≥2 Hz before OFFBOARD"
  rule below is satisfied structurally, not by discipline.
- **No yaw is commanded at all.** Confirmed 2026-09-01 by decoding every
  `/mavros/setpoint_raw/local` message in seven bags: `type_mask = 3527
  (0xdc7)` sets **both `IGNORE_YAW` and `IGNORE_YAW_RATE`**, and the `yaw` /
  `yaw_rate` fields are identically zero. The contract is VX/VY only, and PX4
  derives heading from the velocity vector. Don't go looking for a yaw-rate
  command topic — there isn't one, and adding one would not help: PX4
  velocity OFFBOARD discards `trajectory_setpoint.yawspeed`.
  Pivoting is handled entirely by PX4's own differential-rover logic
  (`RD_TRANS_DRV_TRN` = 45°, matching `rpp_controller`'s
  `pivot_enter_angle_deg: 45.0`). ⚠ **The exit angles no longer match**:
  `RD_TRANS_TRN_DRV` is `0.10472 rad = 6°` on the live FCU while
  `pivot_exit_angle_deg` is still `12.0` in `rpp_controller.yaml`. Older
  `px4-params/` exports say 12° — they are stale on this param.
- MAVROS fields are ENU (`x`=East, `y`=North), converted explicitly at
  lines 297-300. Confirmed independently by decoding
  `/mavros/local_position/odom` in a real bag (§2 of the `analyse-missions`
  skill) — same convention, cross-checked two ways.
- Hard ceiling `ABSOLUTE_MAXIMUM_SPEED_MPS = 1.00` (line 47), **not**
  parameter-overridable (lines 86-89).
- Non-zero velocity requires, in order: `emergency_stop` clear →
  `mission_enabled` → backend heartbeat healthy → `mavros/state.connected` →
  `armed` → `mode == "OFFBOARD"` → RPP command fresher than
  `command_timeout_sec` (0.25s). Any failure → literal zero, never a
  stale/cached command (`_control_loop`, lines 313-343).

Arm/OFFBOARD sequencing is owned by `mission_manager`, not `cmd_vel_bridge`
(`mission_manager_node.py:13`): stream settle 0.60s → request OFFBOARD →
confirm OFFBOARD **while still disarmed** → settle 0.50s → confirm OFFBOARD
again → then ARM (`mission_manager_node.py:1853-1879`). A background monitor
(lines 1414-1429) stops motion if PX4 unexpectedly leaves OFFBOARD or
disarms, but explicitly does **not** auto-recover ("NEVER forces PX4 back to
OFFBOARD and NEVER requests" — line 1416-1417 docstring); that's an operator
action.

## ⚠ `scripts/start_rover.sh` is BROKEN — do not tell anyone to run it

It launches `ros2 run offboard_controller cmd_vel_bridge` — package
**`offboard_controller` does not exist** anywhere in `src/` (only
`jetson_4wd_control` provides a `cmd_vel_bridge` executable, per
`src/jetson_4wd_control/setup.py:21`). It also uses the wrong FCU baud
(115200 vs the production launch's 921600) and never starts `rover_backend`
or `spray_controller`. Git history shows it was only ever touched in the
initial commit and never updated as the architecture moved on.

**The authoritative way to bring the stack up is
`src/rover_bringup/launch/rover.launch.py`**, not `start_rover.sh`. If asked
to restart the rover stack, point at that launch file (or ask the operator
how they actually run it at the rover — this file doesn't yet document a
verified one-liner for launching it end to end).

`scripts/stop_rover.sh`'s disarm sequence is still broadly sound
(disable mission → e-stop → zero velocity → disarm → MANUAL) even though it
publishes to a topic (`/mavros/setpoint_velocity/cmd_vel`) the current
architecture doesn't read — the disarm/mode-change service calls it makes
are independent of which velocity topic is wired up, so the sequence still
achieves a real stop.

## Critical impl rules

- **E-stop is owned exclusively by `mission_manager`**, with a monotonic
  **safety generation counter** (`_safety_generation`,
  `mission_manager_node.py:530`), incremented every time the latch is
  asserted (`_assert_emergency_stop`, line 1572). Release requires the
  matching generation via the `ReleaseEmergencyStop` service
  (`src/mission_manager_interfaces/srv/ReleaseEmergencyStop.srv`) — a plain
  `/emergency_stop=false` publish is not how release works.
- **2026-08-31 (today, commit `7ac712b`, current HEAD) fixed a real bug**:
  `rover_backend`'s release call was reading the safety generation from the
  wrong `rover_state` section (`"safety"` instead of `"mission"`), so
  `expected_manager_generation` was always `-1` and every release was
  unconditionally rejected — mission start was silently blocked
  (`src/rover_backend/rover_backend/ros_bridge.py:3356`, one-line fix). If a
  mission won't start and the estop won't release, this class of bug is
  worth checking for again.
- **`rover_backend` defaults safe on process start**: `/emergency_stop=true`,
  `/mission_enable=false`, no mission auto-prepared
  (`src/rover_backend/rover_backend/main.py:13, 420`).
- **Spray interlocks** (`src/spray_controller/spray_controller/spray_controller_node.py`):
  requires `require_px4_armed` + `require_px4_offboard`; any e-stop,
  mission-disable, OFFBOARD loss, disarm, stale control input, or
  marking-hold loss during a spray forces an immediate release-recovery path
  (docstring lines 27-28, callback line 537-540); faults **latch** and need
  an explicit `/spray/reset_fault`; a persistent journal prevents re-spray
  after a mid-spray crash. Independent hard watchdog `hard_press_timeout_sec
  = 5.0`.
- **Spray point-gating lives in `mission_manager`**: 30mm radial tolerance
  (`marking_tolerance_m: 0.03`), rover must sit under
  `stationary_speed_tolerance_mps: 0.01` continuously for
  `arrival_settle_sec: 0.30` before spray is requested, then held for
  `marking_hold_sec: 3.00` total — all confirmed live in the same-day param
  capture.
- **Symlink-install is confirmed**: the local Mac `install/` tree's
  egg-links point at the Jetson's own build path
  (`/home/flash/rover_ws/build/<pkg>`), for `jetson_4wd_control`,
  `mission_manager`, `rpp_controller` at least. Editing a `.py` under
  `<pkg>/<pkg>/*.py` for any of the 9 `ament_python` packages takes effect
  on the **next node restart, no `colcon build` needed** — unless
  `setup.py` (entry points) or `package.xml` (deps) changed.
- **`mission_manager_interfaces` is the one `ament_cmake` package**
  (generates `ReleaseEmergencyStop.srv`). A change to that `.srv` genuinely
  needs `colcon build --packages-select mission_manager_interfaces`, plus a
  rebuild/restart of every consumer (`mission_manager`, `rover_backend`) —
  this is the one real exception to the symlink-install shortcut above.

## OFFBOARD / safety-net differences from the 3WD sibling — read before assuming parity

- **`NAV_RCL_ACT=1`, `NAV_DLL_ACT=0`** in both local param exports — this
  rover does **not** disarm on RC-loss or datalink-loss the way the 3WD
  rover does (`NAV_RCL_ACT`/`NAV_DLL_ACT`=6/Disarm there). No code in `src/`
  compensates for this. **Do not assume a dropped link auto-disarms — it
  doesn't, on the params captured so far.** If this matters for a task,
  flag it rather than assuming the 3WD failsafe posture.
- The "≥2 Hz before OFFBOARD" and "gap → failsafe" rules from the 3WD
  project still apply in spirit (`cmd_vel_bridge` streams 50 Hz
  unconditionally, and RPP commands older than 0.25s are dropped to zero —
  see above) but the actual PX4-side failsafe response to a stream gap
  hasn't been separately confirmed here.

## RTK — confirmed live and working, but the manager code has open bugs

- Architecture: `rtk_correction_bridge`'s `ntrip_to_px4_node.py` is a full
  NTRIP client (RTCM3 CRC-24Q parsing, TLS-capable socket, NMEA GGA
  round-trip) — richer than the 3WD sibling's standalone script. Caster
  host/port/mountpoint/credentials are injected out-of-band by
  `rover_backend` (`ntrip_to_px4_node.py:83-101`), never plain ROS params —
  don't go looking for them in `ntrip_to_px4_node.yaml`, that file only has
  `health_heartbeat_sec` / `health_log_period_sec`.
- Lifecycle is owned by `rover_backend` (`RtkRuntimeOrchestrator` /
  `RtkManagerCore` / `RtkProcessAdapter` in `src/rover_backend/rover_backend/`).
  **`audit_report.md` (dated today, 2026-08-31) scores this "NEEDS FIX",
  88/100, and blocks Phase 3B/4/7 work on it** until two P1s are fixed: broad
  exceptions in `_execute_spawn`/`_execute_stop`
  (`rtk_runtime_orchestrator.py` ~lines 160, 174) can strand the FSM
  permanently, requiring a full backend restart to recover. One P2: the
  MAVROS-ready gate checks only `state.connected`, not that the RTCM
  injection topic actually has a subscriber — the audit recommends
  `count_subscribers('/mavros/gps_rtk/send_rtcm') > 0` instead.
- **Confirmed live and FIXED tonight** (2026-08-31, 19:16-19:18 IST,
  `bags_jet/evening_session_20260831/mission.csv_20260831_191653/telemetry.csv`):
  `gps_fix_type=6`, 26-33 satellites, `rtk_state=FIXED`, correction age near
  0. Independently re-confirmed by direct bag decode on a different bundle
  the same evening (`GPSRAW.fix_type=6`, 15 sats, `h_acc=2cm` — see the
  `analyse-missions` skill §2/§3). **Do not assume RTK needs setup here** —
  an old memory note about a project called `4WD_SERVER` said NTRIP wasn't
  configured; that note is about a *different, unrelated repo* (see
  "Not this project" below), not this one.
- `trajectory_generator` requires `required_gps_fix_type: 6` and
  `rtk_stable_sec: 3.0` before treating the fix as mission-ready.

## Live-captured params (2026-08-31 evening, matches HEAD `7ac712b`)

> ⚠ **Several of these moved on 2026-09-01.** For `cruise_speed_mps`,
> `deceleration_distance_m`, `acceleration_startup_ceiling_mps`,
> `precision_pivot_stop_speed_tolerance_mps` and the new
> `post_pivot_reanchor_all_legs`, use the table in §"2026-09-01 field
> session" instead. Everything else below still held as of that session.

Pulled from `bags_jet/evening_session_20260831/mission.csv_20260831_191653/params/*.yaml`
— **prefer this over any static `.params` export in `px4-params/`**, which
is untracked/gitignored and has at least one internal contradiction (an
"Aug-27 VETRI" export has `EKF2_GPS_YAW_OFF=0.0`, a "Jul-31 Satheesh" export
has `180.0` — the live capture matches the Satheesh export, treat that as
the one currently governing behavior).

| File | Key values |
|---|---|
| `rpp_controller.yaml` | `cruise_speed_mps: 1.0`, `acceleration_distance_m: 0.5`, `deceleration_distance_m: 0.5` / `deceleration_floor_speed_mps: 0.15`, `pivot_enter_angle_deg: 45.0` / `pivot_exit_angle_deg: 12.0`, `pivot_yaw_kp: 1.0`, `maximum_yaw_rate_radps: 0.2` / `minimum_yaw_rate_radps: 0.06`, `waypoint_tolerance_m: 0.03`, `line_tracking_lookahead_m: 0.55` (0.35-0.8 adaptive — landed **today**, commit `d38155d`), `xtrack_priority_enter_m: 0.015` / `exit_m: 0.008`. Most `precision_*` gates are OFF by default (`precision_speed_control_enabled`, `precision_terminal_enabled`, `precision_pivot_enabled`, `precision_tracking_control_enabled`, `precision_curvature_enabled` all `false`) — only `precision_guidance_enabled` and `geometry_tracking_enabled` are on, deliberately preserving "the production 30mm latch" (`rover.launch.py:490` comment). |
| `mission_manager.yaml` | `marking_tolerance_m: 0.03`, `arrival_settle_sec: 0.3`, `marking_hold_sec: 3.0`, `stationary_speed_tolerance_mps: 0.01`, `spray_required: true`, `spray_confirmation_timeout_sec: 7.0`, `waypoint_match_tolerance_m: 0.002` |
| `spray_controller.yaml` | `press_value: 1.0`, `release_value: 0.0`, `spray_duration_sec: 0.5`, `pre_spray_stable_sec: 0.25`, `hard_press_timeout_sec: 5.0`, `require_px4_armed: true`, `require_px4_offboard: true` |
| `trajectory_generator.yaml` | `required_gps_fix_type: 6`, `rtk_stable_sec: 3.0`, `max_correction_age_sec: 2.0`, `interpolation_spacing_m: 0.05`, `localization_mode: shadow`, `frame_id: map` |
| `cmd_vel_bridge.yaml` | `command_timeout_sec: 0.25`, `backend_heartbeat_timeout_sec: 1.5`, `maximum_speed_mps: 1.0` |

FCU (live capture, same bundle): `EKF2_GPS_P_NOISE=0.5`, `EKF2_GPS_V_NOISE=0.3`,
`EKF2_GPS_CTRL=15`, `GPS_YAW_OFFSET=180.0`, `COM_RC_IN_MODE=0`. ⚠
**`PWM_AUX_FUNC5=0` in the live capture, but `spray_controller_node.py:7`
comments claim `PWM_AUX_FUNC5=301`** — this is an open, unresolved
discrepancy between code comment and live FCU state, not yet root-caused.
Don't trust either source alone for the spray AUX function until it's
reconciled.

## 2026-09-01 field session — speed contract + pivot walk

Nine missions across the afternoon, on branch `feat/rtk-injection-v2`.
Everything here is measured from that session's bags and from PX4 ulogs, not
inferred. Bags in `bags_jet/Sep_01/P1_logs/`; ulogs in
`~/Documents/QGroundControl Daily/Logs/4WD/speed_0.4/`.

### The OFFBOARD contract is velocity-only — there is no yaw command

`/mavros/setpoint_raw/local` carries `type_mask = 3527 (0xdc7)`, which sets
**both `IGNORE_YAW` and `IGNORE_YAW_RATE`**; the `yaw` and `yaw_rate` fields
are identically zero in every message of every bag. The entire contract is
VX/VY in `FRAME_LOCAL_NED`. **PX4 derives heading from the velocity vector
itself.** Do not go looking for a yaw command, and do not add one — PX4
velocity OFFBOARD discards `trajectory_setpoint.yawspeed` anyway (the 3WD
sibling hit the same structural limit and documented it).

### Speed shortfall — ROOT CAUSE: proportional-only droop, PROVEN from ulog

Commanded 0.40 m/s, achieved ~0.235 m/s. PX4 (v1.16.2, git
`54f0455ffcd755534539a7cf33a09a20bf71d29d`) internally regulates the **full
0.400 m/s setpoint — no clamp** — and outputs 0.264 normalized throttle,
which reconstructs its own control law to floating point:

```
throttle 0.264 = FF 0.211 (= 0.4 / RO_MAX_THR_SPEED 1.9)
               +  P 0.053 (= RO_SPEED_P 0.3 x error)
               +  I 0.000 (RO_SPEED_I was 0)      residual 3e-8
```

`RO_SPEED_I = 0` is a proportional-only loop, which **cannot** remove
steady-state error. **Saturation was ruled out**: max motor command 0.705
normalized including pivots, 0% of samples at any limit, PWM 1150-1850us
against 1000/2000 bounds, mapping `1500 + 500*control` with no downscaling.

**Fix applied same session: `RO_SPEED_I = 0` -> `0.2`. Achieved went
0.235 -> 0.37 m/s against a 0.40 command (59% -> 92%).**

Earlier in the session `RO_SPEED_LIM` was `0.40` and DID bind: a 1.00 m/s
mission command produced only 0.22-0.30 m/s, which only fits a 0.4 clamp
(without it the loop would have reached ~0.75 throttle). It was raised to
`1.20` before the 17:34 runs. **The ulog is authoritative for FCU params —
a QGC export read 0.4 while the ulog for the same runs read 1.2.**

### Drivetrain nonlinearity — RO_MAX_THR_SPEED is UNRESOLVED

Measured throttle -> speed, position-derived, non-pivot samples only:

| what | value |
|---|---|
| deadband ends | ~0.10-0.15 normalized throttle |
| local slope, 0.20-0.30 throttle | **1.25 m/s per unit throttle** |
| local affine fit | `speed ~= 1.248 * throttle - 0.103` |
| motor breakaway (command) | **0.143-0.219 m/s** |

`RO_MAX_THR_SPEED = 1.9` and the operator states full-throttle speed really
is ~1.9 m/s — but no log contains throttle above **0.277**, so that is
unverified from data, and the local slope is only 1.25. Both can be true if
the plant is strongly nonlinear, which is the worst case for a single linear
feedforward constant: one value cannot serve both 0.4 and 1.0 m/s.
**Settle it with a MANUAL-mode bench sweep** (`RO_SPEED_P=0`,
`RO_SPEED_I=0`, held throttle 0.10->1.00, logging position, motor outputs
and actual traction-battery current) **before pushing cruise above ~0.6.**
The ~0.32 A current channel in the ulog does NOT measure the Sabertooth
traction supply, so these logs cannot assess voltage sag.

### `RD_MAX_THR_YAW_R` is m/s, NOT rad/s — the yaw axis is fine

It is a **wheel-speed difference in m/s**. PX4 computes
`normalized_diff = (yaw_rate * RD_WHEEL_TRACK / 2) / RD_MAX_THR_YAW_R`, so
`0.30` yields a full-differential yaw rate of `2*0.30/0.63 = 0.952 rad/s`.
The rover sustains 0.66 rad/s (37.8 deg/s) at only ~0.60-0.70 differential
throttle, against `RO_YAW_RATE_LIM = 40 deg/s`. **The yaw feedforward is
correctly sized and does not saturate.** An earlier claim in this session
that it was ~2.5x oversized was a unit misreading and is retracted.

### Pivot settle — the stationary gate was reading a ringing estimator

`_chassis_stationary()` (`legacy_alignment.py:519`) gates on
`current_speed_mps = hypot(twist.linear.x, twist.linear.y)` off
`/mavros/local_position/odom` (`rpp_controller_node.py:3465`). That EKF
velocity has a **stationary noise floor of median 0.0233 / p99 0.0535 m/s**
(1221 samples where the rover genuinely moved <10 mm/s) and **rings through
zero for seconds after a pivot** — `hypot()` rectifies the negative lobe into
an apparent "rebound". At the old 0.030 gate, **33.4% of genuinely
stationary samples read as MOVING**.

Release always landed at exactly `(first sustained gate crossing) + 1.20 s`
— the `settle_sec 0.20 + post_settle_hold_sec 1.00` budget was never wrong.
Raising the gate `0.030 -> 0.060` cut estimator wait from 1.4-6.6 s to a
uniform ~1.2 s per pivot, **~10.4 s off a 4-point mission**. The yaw-rate
half of the gate never binds: post-stop `|yaw_rate|` is <=0.0085 rad/s
against a 0.050 threshold.

### Pivot walk — every pivot displaces the rover 300-600 mm

Measured net displacement during ALIGNMENT, 26 pivots: **363-513 mm when the
rover pivots, 37 mm when it does not** (one leg in run 162110 never pivoted —
a clean natural experiment). Much of it is the GPS antenna swinging through
its lever-arm arc; `EKF2_GPS_POS_X/Y = 0` so EKF2 treats the antenna as the
vehicle origin, and the antenna doubles as the spray-nozzle reference, so the
lever arm cannot be compensated without moving spray targeting.

Only the C->P1 entry leg used to rebuild its line from the post-pivot
position, and it was the only leg landing inside the 30 mm marking latch:

```
    bag   gate  reanchor |      P1       P2       P3       P4 | completed
 154640   0.03        no |   -72.9   -190.6   -165.8    -89.7 | 0/4
 162110   0.06        no |    +4.4   -178.6   -171.7   -133.7 | 1/4
 165849   0.06       YES |    -6.0   -183.7    -15.9    +18.5 | 3/4
 170044   0.06       YES |    -9.6   -145.8    -10.7    +10.5 | 2/4
```

Reanchoring every leg is safe because **nothing is painted between marking
points**: `marking_indices` fire only at the surveyed nav_path indices
(0/44/92/133 on this mission) and `/marking_active` went true exactly once,
for 0.84 s. The inter-point line decides how the rover ARRIVES, not what it
marks; the goal is never moved and `/nav_path` is never modified.

⚠ **Precision guidance derives its correction from the `/nav_path` geometry
projection, not from the path being tracked.** Leaving it with bearing
authority on a reanchored leg steers the rover back onto the surveyed line
and silently undoes the reanchor. Authority is now keyed on
`following_runtime_line` (true for the entry leg AND any reanchored leg), not
on `first_approach`. There is a test that fails if any guidance-authority
site goes back to keying off the leg index.

### As-run params, 2026-09-01 18:04 (run `mission.csv_20260901_180445`)

| where | values |
|---|---|
| `rpp_controller.yaml` | `cruise_speed_mps: 0.4` (staged bring-up 0.4->0.6->0.8->1.0), `acceleration_distance_m: 0.5`, `acceleration_startup_ceiling_mps: 0.25`, `deceleration_distance_m: 1.0`, `deceleration_floor_speed_mps: 0.15`, `precision_pivot_stop_speed_tolerance_mps: 0.06`, `post_pivot_reanchor_all_legs: true`, `segment_alignment_speed_mps: 0.4` |
| FCU (from ulog) | `RO_SPEED_LIM: 1.2`, `RO_SPEED_P: 0.3`, **`RO_SPEED_I: 0.2`** (was 0), `RO_SPEED_TH: 0.02`, `RO_MAX_THR_SPEED: 1.9`, `RO_ACCEL_LIM/RO_DECEL_LIM/RO_JERK_LIM: -1` (all disabled), `RO_YAW_P: 1.0`, `RO_YAW_RATE_P: 0.5`, `RO_YAW_RATE_I: 0.05`, `RO_YAW_RATE_LIM: 40 deg/s`, `RD_MAX_THR_YAW_R: 0.30 m/s`, `RD_WHEEL_TRACK: 0.63`, `THR_MDL_FAC: 0` |

### Commits landed this session

`b350d7a` pivot-settle gate 0.030->0.060 · `aa1f3fc` bag decoder +
pivot-settle report tooling · `4b8fe1b` reanchor every leg · `e638471`
acceleration bootstrap ceiling 0.08->0.25 · `19ebc7c` decel window
0.50->1.00 m · `943c17c` bounded cruise speed, staged at 0.40.

⚠ A teammate's commit (`d9cc15f`, SHARINI, `rover_backend` only) landed
mid-session and rebased two of these, so hashes moved once.

### Open, unresolved

- **P2 terminal swing.** With the reanchor in, every leg now STARTS on its
  line (-2 to -28 mm) but P1->P2 still arrives 146-184 mm off. Its terminal
  swing is -220/-237 mm vs 17-65 mm on every other leg, and it is the only
  leg reaching STOP at `distance_to_goal` 0.15-0.19 m rather than 0.02-0.03.
  `along_remaining` went NEGATIVE while cross-track was 184 mm, i.e. it
  crossed the goal plane beside the point. Confirmed real motion, not a
  reference-line change. The 3WD project has hit and solved two things that
  look like this — the 1/L bearing-gain spike as the aim point closes, and a
  bounded along-track arrival predicate — see `PX4_DXP/AGENTS.md` and
  `src/rpp_controller_node.py` there.
- **`RO_MAX_THR_SPEED`** — see the nonlinearity section above.
- **`RD_TRANS_TRN_DRV = 0.10472 rad = 6 deg`** on the FCU while
  `pivot_exit_angle_deg = 12.0` in `rpp_controller.yaml`. Live mismatch,
  predates this session, unexplained. (`RD_TRANS_DRV_TRN` = 45 deg still
  matches `pivot_enter_angle_deg`.)
- **`rpp_controller` param dump timing out** (`timeout_after_15s` in
  `manifest.json` -> `as_run_config.ros_params.errors`) on 3 of 9 runs,
  costing bag provenance. Not root-caused.

## Repo status (2026-08-31)

- **`WORKING_STATUS_24_06_2026.txt`** — stale (2026-06-24), architecture it
  describes no longer exists. Historical context only.
- **`audit_report.md`** — current (dated today). RTK code review "NEEDS
  FIX" (§ above); separately confirms the primary consumer of
  `rover_backend`'s API is the verified React Native rover app in the GCS
  section below.
- **`REFACTOR_TO_CPP_AUDIT.md`** — current (dated today, branch
  `feat/rtk-injection-v2`). 9 `ament_python` packages, zero C++, ~50,650
  lines across 55 production files. **104 dead `.before_*`/`.bak_*` backup
  files (~5.1MB, mostly in `rpp_controller`)** flagged for deletion
  regardless of any refactor decision. `rpp_controller_node.py` is a single
  **9,281-line class**, rated hardest to port (9.5/10) and 55-65% of a
  hypothetical C++ port's cost. Overall system scored **5.5/10 —
  "CONDITIONAL PRODUCTION: safe only with an operator in the loop"**;
  weakest categories control-loop determinism (GIL, 4/10) and code hygiene
  (dead files, 3/10).
- Stray zero-byte files at repo root (`0.0`, `=`, `Function`, `P1`, `P2`,
  `lower`, `self.closest_marking_distance`, and similar) look like accidental
  shell-redirection artifacts, not meaningful content — safe to ignore, low
  priority to clean up.
- `.before_*_<timestamp>` / `.bak_*` files litter `src/` — **always read the
  live file without the suffix**; a stale sibling next to the real node
  source is the normal state in this repo, not a sign something's wrong.

## GCS frontend — verified

The verified 4WD GCS frontend is
`/Users/dyx_a1/Vetri/temp/DYX_GCS_Frontend` (package `dyx-gcs-mobile`,
Expo/React Native). It is configured for this rover at
`192.168.3.101:5001`; `src/config.ts` uses
`http://192.168.3.101:5001` and `ws://192.168.3.101:5001` as the default
HTTP and Socket.IO backend URLs. The backend is this `rover_ws` workspace,
under `src/rover_backend` (the backend package under `src/`), not the
separate `/Users/dyx_a1/Vetri/4WD_SERVER` repository. Treat this frontend,
this rover IP, and this workspace backend as the verified 4WD stack.

## Hard rules

- Do not run `scripts/start_rover.sh` as a restart mechanism — it's broken
  (see above). Use `rover.launch.py`, or ask the operator how they actually
  bring the stack up.
- Do not assume RC-loss / datalink-loss disarms the rover — `NAV_RCL_ACT`/
  `NAV_DLL_ACT` are not set to Disarm here, unlike the 3WD sibling.
- Do not confuse this backend with `/Users/dyx_a1/Vetri/4WD_SERVER`; that is
  a different repo. The verified frontend is
  `/Users/dyx_a1/Vetri/temp/DYX_GCS_Frontend`.
- Do not trust a static `.params` export in `px4-params/` over a
  same-session live param capture from a bag bundle — the exports disagree
  with each other on at least one FCU param.
- ArduRover is abandoned — do not propose ArduRover solutions (carried over
  from the 3WD project's rule, applies equally here: this is a PX4 rover).
- **The PX4 ulog is authoritative for FCU params, above any QGC export.** On
  2026-09-01 an export read `RO_SPEED_LIM=0.4` while the ulog for the same
  runs read `1.2`. ulogs embed the full param set plus firmware version and
  git hash — read them with `pyulog`.
- **PX4 ulog FILENAME times are unreliable.** Reconstruct wall time from the
  embedded GNSS UTC and confirm the bag<->ulog pairing by cross-correlating a
  shared signal (yaw rate works; expect the NED/FRD <-> ENU/FLU sign flip).
- **Never judge "is the rover stopped/how fast is it" from the EKF velocity**
  (`twist.linear` on `/mavros/local_position/odom`). It has a ~0.023 m/s
  stationary noise floor and rings through zero for seconds after a pivot.
  Differentiate POSITION over a ~0.5 s window instead, and say which one you
  used for every number you report.
- **`RD_MAX_THR_YAW_R` is a wheel-speed difference in m/s, not a yaw rate.**
  Full-differential yaw rate is `2 * RD_MAX_THR_YAW_R / RD_WHEEL_TRACK`.
- **Do not hand-roll CDR parsing for bags.** Import the verified `_CDR` class
  and parsers from `scripts/analyze_mission.py`. A wrong-alignment parse
  yields finite-but-absurd values that still plot — this already happened
  once here.

## Quick reference

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/odom
ros2 topic echo /rpp/accuracy --once
journalctl -u bag-autorecord.service -f      # the only systemd unit on this Jetson
```

- Bag bundles: `bags_jet/<mission.csv_timestamp>/` (sometimes nested under
  `bags_jet/<DD_MM_YYYY>/Patch_NN/`) — see the `analyse-missions` skill for
  full anatomy and a working decoder (`scripts/analyze_mission.py`).
- `python3 scripts/pivot_settle_report.py <bundle-dir> [--threshold 0.060]`
  — per-pivot dwell breakdown: when rotation physically stopped vs when the
  stationary gate cleared, i.e. how long was spent waiting on the estimator
  rather than the rover. Also prints the gate signal's stationary noise floor,
  judged from position so the measurement does not assume its own conclusion.
- Backend: `http://192.168.3.101:5001` (FastAPI, port from
  `src/rover_backend/config/backend.env`).
- Deploy/review workflow: see the `rover-ship` skill — this repo has no
  systemd-managed core services, so "restart" mostly means "ask the
  operator to restart the stack at the rover", not a scriptable SSH command.
