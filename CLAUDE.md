# DYX 4WD Marking Rover — Jetson Companion

Scope: ROS2 Humble colcon workspace on Jetson `192.168.3.101`, PX4 OFFBOARD
control of a 4-wheel skid-steer marking rover. Modeled on the sibling 3WD
project's CLAUDE.md (`PX4_DXP/CLAUDE.md`) — **do not assume any fact from
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
> `PX4_DXP/CLAUDE.md` has — treat anything that later turns out wrong as an
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
| GNSS receiver | **Septentrio mosaic-H**, dual-antenna, SBF protocol, 10 Hz PVT, dedicated PX4 driver `src/drivers/gnss/septentrio/` — **not** a u-blox F9P; audit the Septentrio path, not UBX | `docs/2026-09-04_EKF_GNSS_Hardware_Audit.md` §8 |
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
| `rpp_controller.yaml` | `cruise_speed_mps: 1.0`, `acceleration_distance_m: 0.5`, `deceleration_distance_m: 0.5` / `deceleration_floor_speed_mps: 0.15`, `pivot_enter_angle_deg: 45.0` / `pivot_exit_angle_deg: 12.0`, `pivot_yaw_kp: 1.0`, `maximum_yaw_rate_radps: 0.2` / `minimum_yaw_rate_radps: 0.06`, `waypoint_tolerance_m: 0.03`, `line_tracking_lookahead_m: 0.55` (0.35-0.8 adaptive — landed **today**, commit `d38155d`; ⚠ **this is NOT the lookahead that runs** — see the 2026-09-04 control-path section), `xtrack_priority_enter_m: 0.015` / `exit_m: 0.008`. Most `precision_*` gates are OFF by default (`precision_speed_control_enabled`, `precision_terminal_enabled`, `precision_pivot_enabled`, `precision_tracking_control_enabled`, `precision_curvature_enabled` all `false`) — only `precision_guidance_enabled` and `geometry_tracking_enabled` are on, deliberately preserving "the production 30mm latch" (`rover.launch.py:490` comment). |
| `mission_manager.yaml` | `marking_tolerance_m: 0.03`, `arrival_settle_sec: 0.3`, `marking_hold_sec: 3.0`, `stationary_speed_tolerance_mps: 0.01`, `spray_required: true`, `spray_confirmation_timeout_sec: 7.0`, `waypoint_match_tolerance_m: 0.002` |
| `spray_controller.yaml` | `press_value: 1.0`, `release_value: 0.0`, `spray_duration_sec: 0.5`, `pre_spray_stable_sec: 0.25`, `hard_press_timeout_sec: 5.0`, `require_px4_armed: true`, `require_px4_offboard: true` |
| `trajectory_generator.yaml` | `required_gps_fix_type: 6`, `rtk_stable_sec: 3.0`, `max_correction_age_sec: 2.0`, `interpolation_spacing_m: 0.05`, `localization_mode: shadow`, `frame_id: map` |
| `cmd_vel_bridge.yaml` | `command_timeout_sec: 0.25`, `backend_heartbeat_timeout_sec: 1.5`, `maximum_speed_mps: 1.0` |

FCU (live capture, same bundle): ⚠ `EKF2_GPS_P_NOISE=0.5` — **STALE, this is
0.01 on the vehicle**; confirmed in the live bag capture of two separate
2026-09-04 bundles and in the 2026-09-04 audit. See the 2026-09-05 section.
`EKF2_GPS_V_NOISE=0.3`,
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
  bounded along-track arrival predicate — see `PX4_DXP/CLAUDE.md` and
  `src/rpp_controller_node.py` there.
- **`RO_MAX_THR_SPEED`** — see the nonlinearity section above.
- **`RD_TRANS_TRN_DRV = 0.10472 rad = 6 deg`** on the FCU while
  `pivot_exit_angle_deg = 12.0` in `rpp_controller.yaml`. Live mismatch,
  predates this session, unexplained. (`RD_TRANS_DRV_TRN` = 45 deg still
  matches `pivot_enter_angle_deg`.)
- **`rpp_controller` param dump timing out** (`timeout_after_15s` in
  `manifest.json` -> `as_run_config.ros_params.errors`) on 3 of 9 runs,
  costing bag provenance. Not root-caused.

## 2026-09-03 evening dataset — stop accuracy, and two tooling traps

Eight runs (`log_77/79/82/87/89/98/100/103` paired with
`mission.csv_20260903_{165719,165926,171227,171757,172119,174005,174203,174520}`),
43 waypoints, under
`~/Documents/QGroundControl Daily/Logs/4WD/Madhavaram/Sep_03/evening_run_2/`
(`Bags/` and `Ulogs/` side by side). Raw GNSS was **RTK FIXED for 100% of
every run**, 16-18 sats, h_acc 15 mm.

### ⚠ `scripts/analyze_mission.py` projects on the WGS84 ELLIPSOID; PX4 uses a SPHERE

`_geodesic_ne_m` / `_metres_per_degree` are ellipsoidal. PX4 (and
`src/trajectory_generator/trajectory_generator/localization_frame.py`) project
geodetic to local NED on a sphere of `R = 6 371 000 m`. The disagreement is
**range-proportional: +0.51% north, -0.13% east** at this site.

This produced a convincing 847-888 mm "local-frame offset" on the runs whose
`gp_origin` sat 165 m from the work area, and 10-52 mm on the runs where it
was close. Re-projecting spherically matched the controller's own `goal_x`/
`goal_y` to **0.0 mm on all 43 goals** — there was no rover bug.

**Rule: never compare an absolute local-frame coordinate against an
ellipsoid projection of a lat/lon.** Either project spherically, or (better,
and what `survey_truth.py` does) take the difference *around* the target so
the earth model cancels — at 40 mm error scale a 0.5% model gap is 0.2 mm.

### ⚠ EKF-vs-raw-GNSS separation is transport latency, not estimator bias

Naively differencing `/mavros/global_position/global` against
`/mavros/global_position/raw/fix` gives p50 22-49 mm and looks like an
estimator error. Split by speed it is **45-65 mm above 0.30 m/s and only
11-15 mm at rest** — at the measurement floor. A constant **-20 to -60 ms**
shift of the raw stream collapses the median to 15 mm in every run. Parked
and marking, the EKF agrees with raw RTK. Always split by speed before
calling a position difference a bias.

(Raw lat/lon are int32 * 1e-7 deg = an **11.06 mm quantum**, so single-fix
survey truth carries ±5.5 mm. `/mavros/gpsstatus/gps1/raw` and
`/mavros/global_position/raw/fix` are byte-identical in position — verified
650/650 samples — so GPSRAW is preferred: it carries position and fix quality
in one atomic message.)

### Why the rover stops short — measured, and only partly fixable by tuning

**35 of 43 waypoints stopped SHORT of the surveyed point** (raw RTK truth),
median 18.3 mm, sign test p = 4.2e-5. The settled EKF agreed with raw RTK to
a median of 1.1 mm, so **the short stop is the controller, not the
estimator**.

`TerminalStopRegulator._braking_output` commands
`sqrt(2 * conservative_decel * (along_remaining - brake_margin))`. It crosses
the measured 0.143 m/s motor breakaway at
`brake_margin + 0.143^2/(2*decel)` — **28.6 mm** at the as-run 0.015/0.75.
Inside that distance the command is below what the drivetrain can turn the
wheels with, so the rover stalls: median advance after the last non-zero
command was **-0.2 mm**.

There is a second, separate effect worth knowing: during hard braking the EKF
along-track estimate runs **transiently optimistic by ~25 mm** and
self-corrects over ~1.5 s (174520/P0001: `along_remaining` read -0.9 mm, then
relaxed back to the true +24.5 mm). The zero latch is one-shot, so the stop
decision is taken on that transient. The *settled* estimate is accurate; the
*decision-instant* estimate is not.

**2026-09-04: `radial_stop_brake_margin_m` 0.015 -> 0.003** (d_break 28.6 ->
16.6 mm). `radial_stop_conservative_decel_mps2` deliberately left at 0.75 —
raising it also shrinks d_break but brings BRAKE_PROFILE entry closer to the
goal, which is what caused the 2026-09-02 35-50 mm coast overshoot. Staged
next step if the run lands short with no overshoot: 0.75 -> 1.00.

⚠ **Do not expect tuning to reach zero.** The measured pair bounds it: ramp to
zero -> ~18 mm short (2026-09-03); hold the 0.15 m/s floor -> 35-50 mm past
(2026-09-02). The rover cannot be commanded below ~0.15 m/s and cannot stop in
under ~35 mm from it, so **the point is outside the reachable set of any
static profile**. Landing on it needs settle -> re-measure -> bounded creep
retry above breakaway, which is a controller change, not a parameter.

### Survey truth is now computed and reported (report only — RPP still gates)

- `trajectory_generator` publishes the uploaded lat/lon on
  **`/trajectory_generator/survey_targets`** (latched String/JSON,
  `dyx4wd/survey_targets@1`). Only place un-projected mission coordinates
  leave that node; nothing steers from it.
- `mission_manager` buffers raw GPSRAW fixes and computes physical stop
  accuracy in **`src/mission_manager/mission_manager/survey_truth.py`**
  (pure, ROS-free, 17 tests seeded with real Sep-03 coordinates).
- It appears as `accuracy["survey"]` in `/mission_manager/point_event`, in
  `point_accuracy_snapshots` in the mission summary, and as
  `point_survey_snapshots` + `survey_truth_*` health in
  `/mission_manager/status`.
- **The 30 mm latch, spray gating and the point verdict are unchanged** and
  still run on RPP. Survey truth is a second, independent measurement — it is
  allowed to be `available: false` (degraded fix, window not stationary,
  local-coordinate mission) without affecting anything.
- Sign convention matches the RPP report: `along_track_error_mm` positive =
  SHORT of the target; `cross_track_error_mm` positive = RIGHT of approach.
- Validated by replaying all 8 bags through the shipped module: 40/43 within
  6 mm of the independently measured value. The 3 that differ by 7-17 mm are
  the points where the rover kept moving 15-66 mm *after* the latch, so the
  two numbers are different instants, not a disagreement.

Judged on physical truth, 18/43 made the 30 mm latch. RPP's own verdict
agreed on 37 of 43 — **3 false accepts and 3 false rejects**.

## 2026-09-04 — verified control-path facts (read before touching tracking)

Established by reading the live source and cross-checking against the Sep-03
and Sep-04 bags. Each item says how it was verified, because two plausible
readings were **wrong** and are recorded here so they are not re-derived.

### The lookahead that actually governs is time-based, not `line_tracking_lookahead_m`

`precision_guidance_enabled: true`, so `guidance.py:11` computes

```
lookahead = clamp(lookahead_time_s * speed + xtrack_lookahead_gain * |xtrack|, min, max)
```

with `precision_lookahead_time_s: 0.9`, `precision_xtrack_lookahead_gain: 0.0`,
`precision_lookahead_min/max_m: 0.2 / 1.0`. Measured live: **0.936 m p50** at
1.03 m/s — exactly `0.9 x 1.03`. `line_tracking_lookahead_m: 0.55` and
`xtrack_priority_lookahead_m: 0.55` describe a path that does not run while
precision guidance is on. Tune `precision_lookahead_time_s`, not either 0.55.

Note the gain term would *lengthen* the lookahead as error grows, so raising
`precision_xtrack_lookahead_gain` is not a fix for poor convergence.

### The OFFBOARD contract has no heading authority — this is the structural one

`cmd_vel_bridge` sends `type_mask = 3527` (`IGNORE_YAW | IGNORE_YAW_RATE`) with
`msg.yaw = 0.0` and `msg.yaw_rate = 0.0` (`cmd_vel_bridge.py:59, 308`). PX4
derives heading from `atan2(vE, vN)` on its own. The controller can only steer
by rotating the velocity vector and waiting for PX4 to infer the turn.

Measured consequence: command-bearing to actual-heading lag is **1.00 s**
against EKF attitude and **1.20 s** against course-over-ground derived from raw
GNSS alone (correlation 0.25 at zero lag rising to 0.92 at ~1.0 s, so the peak
is real). That delay is **equal to the 0.9 s lookahead time** — the classic
marginal-stability condition for pure pursuit, and it matches the observed
non-convergent cross-track (Sep-04: 24.1 mm entering the cruise, 19.2 mm
leaving it, 27 of 37 legs crossing the line).

The 3WD sibling does not do this. `PX4_DXP/src/twist_to_setpoint_node.py`
publishes `type_mask = 2503` with an explicit `msg.yaw = atan2(v_n, v_e)` every
cycle, and 455 when a yaw-rate feedforward is live. Its own comment states the
reason: PX4's derived-yaw path "lags on turns". ⚠ That repo's CLAUDE.md
separately claims velocity OFFBOARD *discards* `trajectory_setpoint.yawspeed`,
so the explicit **yaw** (2503) is the demonstrated part and the **yaw_rate**
(455) is not — do not port 455 on the strength of the 3WD tree alone.

### The post-pivot reanchor is CORRECT in source, and UNOBSERVABLE in bags

The ordering documented above is right — this was checked because a bag-timing
reading suggested otherwise, and that reading was wrong:

- `legacy_alignment._settle_certificate_and_hold` emits `REANCHOR_ZERO` only
  after `_chassis_stationary_debounced` passes and the `settle_sec` dwell has
  elapsed, i.e. **after** the pivot settles.
- `reanchor_runtime_path_after_pivot` anchors at `self.current_x/current_y`,
  the post-pivot position, and never moves the goal.
- When it succeeds, `segment_runtime_reanchored` is set and
  `nav_solution = runtime_entry_tracking_solution(...)` with
  `path_label = "LEG_PATH"`, so `path_bearing` follows the reanchored line
  (`rpp_controller_node.py:9370-9382`).

⚠ **`/runtime_nav_path` is NOT the reanchor.** It is published by
`mission_manager` (`mission_manager_node.py:420`) and republished as goals
advance — its point count decreases monotonically through a mission
(1120 -> 995 -> 882 -> 758 -> 626 -> 496 on `mission.csv_20260904_141428`).
Timing it against pivots proves nothing about reanchoring. `rpp_controller`
publishes **nothing at all** when it reanchors, and `/rpp/pivot_debug` records
zero messages, so whether a given pivot actually reanchored cannot currently be
read from a bag. Fixing that observability is a prerequisite for any pivot or
reanchor work.

### What `/rpp/debug.cross_track_error_mm` is measured against

`goal_signed_cross_track` (`rpp_controller_node.py:9442`) is the perpendicular
offset from the line through the **goal** with bearing `path_bearing`, and
`path_bearing` follows the reanchored runtime line when one is installed. So
the reported cross-track IS against the line being tracked, not always against
`/nav_path` — the Sep-03/Sep-04 leg-convergence numbers are valid as tracking
error. (Precision guidance's own `signed_cross_track_m`, which *is* derived
from the `/nav_path` projection, is only substituted when
`not following_runtime_line` — `rpp_controller_node.py:10096-10102`.)

### Measured pivot behaviour (Sep-04, 6 real pivots)

Turns of 87-173 deg, walk 521-651 mm. Exit `|HE|` p50 **1.43 deg**, 5 of 6
under 4 deg — the pivot delivers good heading. Exit `|X|` p50 **21.0 mm**, only
2 of 6 inside 20 mm; if the reanchor had fired this should be ~0 by
construction, which is the open question the missing observability blocks.
The single 172.7 deg row reversal is the outlier on every axis: exit HE
7.57 deg, heading swing 15.3 deg, and 124.5 mm of genuine physical curvature
(raw GNSS, not a reference change).

⚠ Roughly half the *reported* post-pivot cross-track swing (63.5 mm p50) is the
reference line being rebuilt, not the rover moving — physical curvature from
raw GNSS is 6.1 mm p50. Do not read the reported swing as motion.

### `xtrack_priority_speed_mps` is a GUARD held neutral -- leave it that way

Set to `CRUISE_SPEED_MPS`, so the latch caps speed at exactly the speed it is
capping. Engaged on 62-90% of cruise cycles across both datasets and limits
nothing.

⚠ **This was repeatedly mis-filed as a defect to fix (2026-09-04 reports,
HANDOFF.md, and commit `96c35e6`, which set it to 0.60 and was reverted).
That framing is wrong and the revert was right.** Lowering the cap does not
merely slow the mission: lookahead is `precision_lookahead_time_s x speed`, so
capping 1.03 -> 0.60 m/s shortens the lookahead 0.936 -> 0.540 m and
**step-changes the pure-pursuit steering gain by ~1.7x** (gain goes as 1/L), as
a discontinuity, at the 15 mm engage threshold, in a loop that is already
marginally stable because the actuation delay equals the lookahead time. It
engages and releases with hysteresis, so the gain steps both ways mid-leg.
That is a swing generator, and the operator observed exactly that in the field.

Layering rule that follows: speed caps and xtrack priority are the **guard**
layer -- they bound damage when tracking is already wrong. The **fix** layer is
whatever makes the lateral loop converge (heading authority, then lookahead).
Guards must stay neutral while the fix is being measured, or they confound the
measurement and inject their own dynamics. Once tracking settles this guard is
not needed at all.

## 2026-09-05 — production task sheet is now the plan; EKF2/GNSS baseline

**`docs/4WD_CM_TRACKING_PRODUCTION_TASKSHEET_FINAL.md` is the current plan**
(committed `5686e21`), with `docs/2026-09-04_EKF_GNSS_Hardware_Audit.md`
(`b8aa859`) as its evidence base. It supersedes the ad-hoc "Step 0/1/2"
sequence still described in `HANDOFF.md`. Read the task sheet's P0 rules before
proposing anything here.

Two of those rules change how earlier work in this file should be read:

- **Runtime path is allowed only for the C -> P1 entry leg.** Every later
  `P* -> P*` leg must track its own fixed mission geometry. The post-pivot
  reanchor described in the "Control architecture" section above is therefore
  **diagnostic/recovery evidence only, not the production answer to pivot
  walk** — P7 removes it as a dependency. The instrumentation added in
  `60d4149` is still worth having under that reading.
- **Do not enable disabled precision states just because they exist**, and do
  not tighten estimator noise below measured receiver behaviour.

### Corrected as-run EKF2 configuration

⚠ **`EKF2_GPS_P_NOISE = 0.01`, not 0.5.** Confirmed in the live bag capture of
`mission.csv_20260904_141428` and `_143312`, and independently in the audit's
parameter file. The 0.5 in the 2026-08-31 table above is stale. This matters:
GNSS height observation noise is `max(receiver vacc, 1.5 * EKF2_GPS_P_NOISE)`,
so a 1 cm floor contributes only 15 mm — a confidently-wrong measurement is
accepted with a near-zero innovation.

| param | value | note |
|---|---|---|
| `EKF2_GPS_P_NOISE` | **0.01** | live-capture confirmed; was documented as 0.5 |
| `EKF2_GPS_V_NOISE` | 0.30 | live-capture confirmed |
| `EKF2_GPS_CTRL` | 15 | horizontal + altitude + 3D velocity + dual-antenna heading |
| `EKF2_HGT_REF` | **1 = GNSS** | GNSS is the long-term vertical authority |
| `EKF2_REQ_GPS_H` | **1.0 s** | firmware default is 10 s — aggressive |
| `EKF2_GPS_CHECK` | 831 | vertical-drift and horizontal-speed-offset checks DISABLED |
| `EKF2_GPS_DELAY` | 50 ms | |
| `EKF2_GPS_POS_Z` | -0.30 m | vertical lever arm set |
| `EKF2_GPS_POS_X/Y`, `EKF2_IMU_POS_*` | **effectively zero** | horizontal lever arms NOT configured — task sheet P1 |

⚠ The lever-arm params are **not** in the bag `fcu_params` capture — they came
from an uploaded `.params` file, which this file's own hard rules rank below a
live capture. Re-read them off the FCU before acting on them.

### The log_82 questions, resolved and unresolved

**Horizontal: RESOLVED — ordinary tracking/stop error, not a false position.**
Two independent methods agree. The audit searched every sample for
`RPP <= 20 mm AND raw > 50 mm` and found **zero**; its per-point raw radials
(56.7 / 82.1 / 24.4 / 16.3 mm) match the survey-truth module's
(54.9 / 83.6 / 27.2 / 15.6 mm) to a few mm. The error the operator saw is the
cross-track problem already characterised above.

**Vertical: OPEN — real receiver-native anomaly, root cause unproven.**
⚠ Likely the same failure mode as the 2026-09-08 B group (§2026-09-08): same
sign, same 100-150 mm magnitude, same "cleared by a power cycle". Read that
section before re-opening this one.
Raw GNSS altitude in the three runs **before** a rover power-off (`165719`,
`165926`, `171227`) sits 125-152 mm below the eight-run median; all five runs
**after** it sit within -2 to +52 mm. At the four piles `171227` shares with
later runs it reads 139/154/155/160 mm lower — 4 of 4 stops, same sign and
magnitude — while every other run agrees with every other at the same pile to
±36 mm. Throughout, the receiver reported `fix_type 6`, 17 satellites and
`h_acc 15 mm`. **A power cycle cleared it.** EKF2 did not create the offset but
was configured to trust GNSS as height reference with no independent protection
against a stable, confidently-reported absolute bias.

### ⚠ Comparing raw GNSS against fused global proves NOTHING about correctness

`/mavros/global_position/global` is derived from the raw fix. Under a
common-mode GNSS error both are wrong together and agree perfectly, so their
agreement measures internal consistency only. `fix_type` and `h_acc` are the
receiver's self-assessment and are exactly what is wrong in this failure mode.
Detecting a bad fix needs an **external reference** — revisiting the same
physical point across runs is the one available from these bags, and it is what
found the vertical offset. The 2026-09-03 "EKF-vs-raw separation is transport
latency" conclusion above is correct **and insufficient** for this reason.

### ⚠ Two traps in position-derived speed (both hit on 2026-09-04)

- **It cannot distinguish parked from pivoting in place.** Rotation produces
  near-zero net translation over a ~0.5 s window, so pivot samples land in an
  "at rest" bucket and carry the largest errors — this contaminated the
  2026-09-03 at-rest split (conservatively: true at-rest agreement is *better*
  than the 11-15 mm reported). Gate on yaw rate as well; the controller already
  does via `radial_stop_stationary_yaw_rate_radps`, the analysis scripts did not.
- **It uses bag RECEIVE time, which compresses under bursty delivery.** Three
  fixes arriving 1.0 ms apart while representing 0.3 s of real motion produced a
  spurious 2.61 m/s. `NavSatFix` carries a header stamp that `_p_navsatfix`
  reads and discards — use it instead.

## 2026-09-05 — Task 2.1 measured: GNSS/EKF noise (no param changed)

`docs/2026-09-05_Task_2.1_GNSS_EKF_Noise_Measurement.md`, tool
`scripts/gnss_ekf_noise_report.py`, raw output under `docs/measurements/`.
Stationary windows are detected from **actuator output + gyro only**, never from
GNSS or the EKF, so the scatter measured inside them is not circular.

| quantity (parked, RTK FIXED) | measured sigma |
|---|---|
| raw GNSS horizontal (2D) | **2.6 mm** |
| raw GNSS vertical | **4.8 mm** |
| raw GNSS velocity N/E/D | **7.9 / 8.3 / 10.6 mm/s** |
| raw GNSS yaw (dual-antenna) | **0.128°** |
| EKF local position x/y/z | **2.1 / 3.1 / 5.7 mm** |
| EKF local velocity x/y/z | **4.3 / 4.7 / 4.7 mm/s** |
| EKF yaw | **0.0095°** |

Four things worth carrying forward:

- **RTK error is time-correlated — averaging does not remove it.** Against a
  1/sqrt(tau) white-noise expectation at tau = 2 s, GNSS east is 3.2x high,
  GNSS altitude 3.5x, EKF y 5.2x, EKF z 5.1x (EKF y and z do not fall with tau
  at all). Repeatability at one revisited point across four runs is **42 mm
  vertical** against 4.8 mm within-stop jitter. **Never size a noise parameter
  off the 0.1 s jitter.**
- **Innovation gates have no detection authority here.** `estimator_aid_src_gnss_*`
  test ratios: p50 ~0.000-0.002, worst single sample anywhere **0.143**,
  **zero rejections** across 1390 samples x 4 aid sources, moving and stationary,
  100% fused. Zero EKF resets, 0/1390 failing `estimator_gps_status` checks. The
  `log_82`-class fault (stable, confidently-wrong fix) arrives with a near-zero
  innovation — catching it needs an external reference, not a tighter gate.
- **`EKF2_GPS_V_NOISE = 0.30` must not be lowered.** The receiver reports
  `s_variance_m_s` = 0.4 mm/s against 8-11 mm/s measured — it under-reports
  velocity accuracy ~20x, and PX4 takes `max(reported, EKF2_GPS_V_NOISE)`.
- **The EKF keeps moving ~1.7 s after the drivetrain goes neutral** (36 stops:
  p50 16 mm / p95 111 mm of further reported motion, p50 1.72 s to settle within
  10 mm), while the raw receiver settles in p50 0.09 s. `arrival_settle_sec` is
  0.30 s and the zero latch is one-shot, so the 30 mm marking verdict is taken
  mid-transient. Consistent with the 6/43 RPP-vs-truth disagreements on Sep-03.

⚠ **The Sep-04 mission bags have NO paired ulog** — the only Sep-04 ulog
(`log_25`) is 5.6 s. Every estimator-internal number above is Sep-03 only.
⚠ **Prefer the ulog over `GPSRAW` for GNSS position noise**: bag lat/lon carry
an 11.06 mm quantum (sigma 3.19 mm) that dominates a millimetre-scale
measurement; `vehicle_gps_position.latitude_deg` in the ulog is a double.

Still wanted before Task 2.2 closes: **a 20-30 min static log** (parked, armed,
RTK FIXED, full logging rate). It needs no bench and no mission, and it would
measure the correlation time directly instead of the tau <= 2 s floor above.

## 2026-09-08 — the ~150 mm RTK bias is receiver-side, and BeiDou is the lead

> ⚠ **SUPERSEDED on 2026-09-09 — read §2026-09-09 first.** The BeiDou lead, the
> reboot framing and the A-vs-B height argument in this section are retracted;
> `log_15` sits on a 210 mm-wrong fix with BeiDou at 99.7 %. What still holds:
> antenna ARP/PCV is not the discriminator, the error is in the receiver's own
> baseline, and the receiver cannot self-assess. The tool below is unchanged.

Full writeup: `docs/2026-09-08_Handoff_Septentrio_BeiDou_Wrong_Fix.md`. Dataset
`Estimator_Compare/` (A = 11 good ULogs 17:06-17:31, B = `log_47`/`log_48`
19:21-19:25). Everything below was decoded from the raw serial traffic PX4
captures in `gps_dump` (`SEP_DUMP_COMM=3`), not from `sensor_gps`.

**Tool: `python3 scripts/gps_dump_sbf_report.py [--reference targets.csv] <*.ulg>`**
— splits `gps_dump` into the two directions and decodes both: CRC-checked SBF
from the receiver, CRC-checked RTCM3 into it. This is the only way to see which
GNSS signals the RTK solution actually used, and the receiver's own ARP /
phase-centre flags. Layouts come from PX4's `sbf/messages.h`, ArduPilot's
`AP_GPS_SBF.h` and the mosaic-H v4.14.10 reference guide, each size-checked
against the receiver's `Length`/`SBLength`. ⚠ ArduPilot's `VectorInfoGeod`
declares `ReferenceID` as `u1`; spec and receiver both say `u2`, and the `u1`
layout silently shifts `SignalInfo` by one byte.

### Antenna ARP / phase-centre is NOT the cause of the good-vs-bad split

- **PX4 never configures the antenna.** `SeptentrioDriver::configure()` sends
  only `scs`, `sso`, `sdio`, `sto`, `srd`, `sga`, and `ssu` when
  `SEP_CONST_USAGE != 0`. No `setAntennaOffset`, no antenna type, no PCO/PCV,
  in **either** `SEP_AUTO_CONFIG` state. Antenna config is receiver NVRAM.
- `PVTGeodetic.Misc` is `0x50`/`0x60` in all 13 logs, good and bad, in the same
  proportion: bit 0 = 0 (baseline not known to point to base ARP), bit 1 = 0
  (rover PCO not compensated), bits 6-7 = 1 (ARP-to-marker offset is zero).
  Missing RTCM 1007/1008/1033 likewise identical in both groups.
- `BaseVectorGeod.Misc` bits 0 and 1 are both clear, so the spec's "accurate
  ARP-to-ARP baseline guaranteed only if both are set" is genuinely unmet —
  real, worth closing, worth a few cm, **not this defect**.

### What actually changed

The receiver was **power-cycled** between the groups (uptime 3780 s -> 2678 s)
and came back not using BeiDou. `PVTGeodetic.SignalInfo`:
`0x30220909` (with BDS B1I+B2I) in A, `0x00220909` in B. BeiDou used in 88-100%
of A epochs, **0.4% / 3.7%** of B epochs; `NrSV` 20 -> 15, VDOP 1.00 -> 1.64.

- **Not the corrections.** RTCM 1124 carried 8-9 BeiDou satellites in B at the
  same rate as A, yet `BaseVectorGeod.SignalInfo` reports no BeiDou corrections
  available. The receiver discarded them.
- **Not propagation.** BDS B2I (1207.14 MHz) was dropped while Galileo E5b at
  the *same* frequency was kept; BDS B1I (1561 MHz) dropped while GPS L1CA
  (1575 MHz) kept. Nothing frequency-selective does that.
- **Not PX4.** `SEP_CONST_USAGE = 0` means "constellation usage isn't changed"
  and the driver skips `ssu` entirely — the constellation set is receiver NVRAM
  in both auto-config states.

### The bad fix is a wrong integer ambiguity resolution

B is static (0.10-0.36 m span over 184-470 s), so 148 mm is measurement, not
control. `PosCovGeodetic` reports sigma_North ~9 mm against a 143-151 mm error
while holding `Mode = 4` / `Error = 0` — wrong by ~16 sigma. It drifts smoothly
+123 -> +153 mm over 11 parked minutes (a fixed wrong integer set through a
rotating geometry matrix; an antenna offset would be constant, noise random).
**Height reads 100-140 mm below the good group at the same pile** — matching the
still-open 2026-09-03 vertical anomaly (§2026-09-05) in sign, magnitude and in
being cleared by a power cycle. Probably the same recurrent failure mode.

Revised ranking: **#1 wrong ambiguity fix** (was #3), **#2 BeiDou loss as the
trigger** (was "geometry"), **#3 antenna ARP/PCV** (was #1).

### Sharpest open question

`log_20` ran on a constellation about as thin as B's (`NrSV` 14, HDOP 1.12,
BeiDou 19%) and stayed **RTK float for all 383 epochs** — it declined to fix.
B, similarly thin, declared FIXED and was wrong by 150 mm. Unexplained.

### Next moves (no field data needed for the first three)

1. Interrogate the receiver via NSH/web UI and record: `lif,Permissions` (is
   BeiDou licensed?), `gst`, `gsu`, `gao`, `lif,AntennaInfo`, `grd`, `gpm`, and
   `lif,error` (every log has `ReceiverStatus.RxError` bit 3 `SOFTWARE` set, in
   both groups, never explained; `lif,error` reports and clears it). Save to
   NVRAM so a power cycle cannot change it again.
2. Set `SEP_CONST_USAGE = 31` and `SEP_AUTO_CONFIG = 1` so PX4 commands `ssu`
   every boot and the constellation set stops depending on NVRAM. ⚠ `ssu` sets
   *usage*, not *tracking* — check both in step 1.
3. Add `ReceiverSetup` (5902), `ChannelStatus` (4013) and `SatVisibility` (4012)
   to a **second** receiver SBF stream (so auto-config can stay on) and
   `gps_dump` will capture them. `SEP_SAT_INFO` does **not** do this — the driver
   only copies the satellite count into `satellite_info`. Without those blocks no
   log in this project contains per-satellite azimuth/elevation/CN0.
4. Only then re-test: static placement at P0001, BeiDou confirmed in
   `SignalInfo`, same post-sunset window, and see whether 150 mm returns.

## 2026-09-09 — ROOT CAUSE: wrong integer ambiguity fix; it only changes on a re-fix

Full writeup: `docs/2026-09-09_Handoff_RTK_Wrong_Ambiguity_Fix_Root_Cause.md`.
Established over **every September ULog** (164 found, 142 unique, 119 with >20 s
of GNSS), not one A/B bundle.

**Inside one 98-minute log, at the same physical point, with nothing changed —
no reboot, no parameter, no configuration, no constellation change — the reported
position steps 192 mm, exactly on the only RTK ambiguity re-resolution in the log.**
`log_15_2026-9-8-14-10-54`, parked fixes within 0.3 m of P0001, split on its one
1093 s RTK outage (t = 2828 -> 3921 s):

| | fixes | height p50 | sats | vdop | `eph` | BeiDou |
|---|---|---|---|---|---|---|
| before the re-fix | 10 064 | **-87.487 m** | 18 | 1.30 | **15 mm** | 99.7 % |
| after the re-fix | 9 509 | **-87.295 m** | 19 | 1.14 | **15 mm** | 99.7 % |

Within each band the height wanders only tens of mm across many parked windows.
**Reported position is stable while a fix is held and steps by decimetres when the
fix is re-resolved.** `log_18` corroborates (-210 mm across a 170 s re-fix at one
spot). `log_47` is the control: 99.5 % fixed, no outage over 1.6 s, height held to
30 mm across 4 minutes — one fix, held, wrong.

Truth anchor: `log_12_2026-9-8-11-06-54` is verified good — parked 74.9 s, RTK
fixed, raw **10.1 mm** and fused **5.2 mm** from surveyed P0001, `fused - raw` =
N +6.1 / E -3.3 / **U -300.0 mm** (exactly `EKF2_GPS_POS_Z`), 0 innovation
rejections, 0/974 `gps_check_fail_flags`. It reads **-87.277 m** at P0001 and the
Sep-07 day median is **-87.261 m**. So log_15-before is ~210 mm low and log_47/48
~110-165 mm low, while log_15-after is within 35 mm.

### Retracted from the 2026-09-08 section

- The **reboot** is not the mechanism — five receiver boots on Sep-08, four came
  back with BeiDou (99.7/91.9/93.9/19->100 %), one did not (0.4/3.7 %). A reboot
  just forces a re-fix.
- **BeiDou is not necessary** — log_15 was 210 mm wrong with BeiDou at 99.7 %. Its
  absence in boot #5 stays a real, unexplained receiver-side anomaly (the base sent
  RTCM 1124 with 6-10 sv in every log all day) and a plausible aggravator, not the
  cause.
- The **A-vs-B height argument** was not evidence: log_15 swings 230 mm vertically
  at P0001 within one log and goes *below* log_47.
- **+148 mm North is not uniquely large** — log_15's long holds sit at dN +96..+108
  at P0001. And nothing in these logs measures where the rover physically was.

### Cleared by the September-wide sweep

- **RTCM injection is healthy everywhere**: 112/119 logs at exactly 6.0 Hz median.
  The 7 exceptions are 6 total outages (0 Hz, 0 % RTK: `log_83`/`log_84` 9-3,
  `log_49`/`50`/`51` 9-7, `log_31` 9-8) plus `log_1` at 4.8 Hz. Reduced correction
  rate is not a factor in any bad-position log.
- **Constellation content is measurable only on 17 logs, all 2026-09-08**
  (`SEP_DUMP_COMM` was 0 before that). Sep-01/03/04/07 cannot be checked at all.
- **2026-09-08 is the anomalous day, not the evening.** Within-day vertical spread
  at P0001: Sep-03 max **41.9 mm**, Sep-07 max **43.8 mm**, Sep-08 max **416.4 mm**,
  starting in the 10:30 session. Unexplained; no parameter can do it (the only
  GNSS-relevant diffs are `EKF2_GPS_CHECK` 831->829, `EKF2_GPS_YAW_OFF` 0->180 on
  log_47, `SEP_AUTO_CONFIG`, `SEP_DUMP_COMM` — and EKF2 params cannot move raw GNSS).

### What to do

1. **The only defence that matches the failure.** `eph` is 15 mm in both the correct
   and the 210 mm-wrong state, so no innovation/DOP/fix_type/sat-count gate can
   separate them — settled, not fixable by tightening anything. But the *transition*
   is observable: `fix_type` dropping below 6 and returning. `mission_manager` should
   treat an RTK re-fix as invalidating prior marking references and refuse to keep
   marking until the rover re-verifies against a known surveyed point.
2. **Quantify the exposure**: park at P0001, force ~10 re-fixes (cut corrections
   60 s, restore, repeat), log each settled position. Discrete clusters 100-200 mm
   apart confirm the mechanism. One hour, no bench, no mission.
3. **Fix less often wrongly**: restore BeiDou determinism (`lif, Permissions`, `gst`,
   `gsu`; `sst, all` + `eccf, Current, Boot` if tracking is off; `SEP_CONST_USAGE=31`
   with `SEP_AUTO_CONFIG=1` — ⚠ `ssu` sets usage, not tracking); ask for a nearer
   base or a VRS/MAC mountpoint (13.68 km is long for single-base integer fixing);
   close the antenna gap (`sao` with antenna type, base 1006/1007/1008/1033).

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
  (`twist.linear` on `/mavros/local_position/odom`). It rings through zero for
  seconds after a pivot. Differentiate POSITION over a ~0.5 s window instead,
  **and gate on yaw rate too** (position-derived speed cannot tell parked from
  pivoting in place), and say which one you used for every number you report.
  ⚠ The "~0.023 m/s stationary noise floor" quoted elsewhere in this file is
  the post-pivot RINGING, not the floor: gated on actuator output + gyro with
  a settle trim, the genuinely parked floor is **6.3 mm/s 2D RMS** (2026-09-05,
  §Task 2.1). The 0.030 -> 0.060 pivot-settle gate was sized against the
  ringing and stays correct.
- **`RD_MAX_THR_YAW_R` is a wheel-speed difference in m/s, not a yaw rate.**
  Full-differential yaw rate is `2 * RD_MAX_THR_YAW_R / RD_WHEEL_TRACK`.
- **Do not hand-roll CDR parsing for bags.** Import the verified `_CDR` class
  and parsers from `scripts/analyze_mission.py`. A wrong-alignment parse
  yields finite-but-absurd values that still plot — this already happened
  once here.
- **Never compare positions across an RTK re-fix.** A `fix_type` drop below 6 and
  return can move the reported position by 100-500 mm with no change in `eph`,
  satellite count or DOP (2026-09-09: 192 mm in `log_15`, -210 mm in `log_18`).
  Split every dataset on re-fix boundaries before averaging, differencing, or
  calling anything a bias.
- **Never read a mission-log stop error as GNSS accuracy.** The rover closes the
  loop on this same GNSS and stops when its *reported* position reaches the
  surveyed target, so a receiver bias is absorbed into the physical parking
  position and does not appear in the reported error. Mission-stop numbers
  measure controller convergence. Use an open-loop channel instead — **height**
  is the one the rover never controls — or a static placement test.
- **Do not hand-roll SBF or RTCM parsing either.** Use
  `scripts/gps_dump_sbf_report.py`; its layouts are size-checked against the
  receiver's own `Length`/`SBLength` at decode time. ArduPilot's
  `VectorInfoGeod.ReferenceID` is `u1` and is wrong — spec and receiver say `u2`.
- **PX4 does not configure the Septentrio antenna or its constellation set.**
  No antenna/ARP/PCV command exists in the driver, and `SEP_CONST_USAGE = 0`
  suppresses `ssu`. Both are receiver NVRAM and survive — or change across — a
  power cycle, in either `SEP_AUTO_CONFIG` state. Do not infer receiver
  configuration from PX4 parameters.

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
- `python3 scripts/gps_dump_sbf_report.py [--reference targets.csv] <*.ulg>`
  — decodes the raw Septentrio SBF and injected RTCM3 out of `gps_dump`: which
  signals/constellations the RTK solution used, ARP/phase-centre flags, base
  correction contents, receiver uptime, and stationary error vs surveyed truth.
- Backend: `http://192.168.3.101:5001` (FastAPI, port from
  `src/rover_backend/config/backend.env`).
- Deploy/review workflow: see the `rover-ship` skill — this repo has no
  systemd-managed core services, so "restart" mostly means "ask the
  operator to restart the stack at the rover", not a scriptable SSH command.
