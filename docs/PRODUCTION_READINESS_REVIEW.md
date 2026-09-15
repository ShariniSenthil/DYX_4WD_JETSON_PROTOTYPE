# DYX 4WD Rover — Production Readiness Review & Architecture Audit
**Document ID:** `DYX-AUDIT-2026-PROD-01`  
**Date:** 2026-09-15  
**Target Hardware:** Jetson Orin Nano / Holybro Pixhawk 6X / Dual Sabertooth 2X32 / Septentrio Mosaic-H RTK  
**Target Software:** ROS 2 Humble / PX4 v1.16.2 / FastAPI + ASGI Socket.IO / React Native Frontend  
**Scope:** Complete End-to-End Subsystem Review for Transition from Field Prototype to Commercial Demo Product  

---

## Subsystem Scorecard Summary

| Subsystem | Component / Package | Production Score | Status | Primary Production Blocker |
| :--- | :--- | :---: | :--- | :--- |
| **1. Frontend & Comm Link** | `rover_backend` (REST/WS/Bridge) | **78 / 100** | Needs Hardening | 5.0s service timeout trap; `deepcopy` under RLock at 10 Hz; unrecoverable 504 on normal settle times. |
| **2. Path Engine** | `trajectory_generator` | **91 / 100** | Near Production | Linear-only interpolation; lacks smooth continuous curvature for row entry/exit. |
| **3. Motion Controller** | `rpp_controller` + `cmd_vel_bridge` | **82 / 100** | Needs Tuning | Overshoot lockup without auto-reverse recovery; >10.6k lines monolith complexity. |
| **4. Stack Orchestrator** | `mission_manager` | **84 / 100** | Production Capable | Mid-mission crash loses state (no persistent checkpointing); PX4 remains armed on node death. |
| **5. Spray Actuator** | `spray_controller` | **88 / 100** | Production Capable | `PWM_AUX_FUNC5` FCU parameter discrepancy (FCU reads 0 vs code expects 301). |
| **6. Multi-Node Health** | `rover_bringup` (Launch & QoS) | **65 / 100** | **High Risk** | `mavros` launched with no respawn; `rover_backend` launched with `respawn=False`. |
| **7. OS & Lifecycle** | `systemd`, Watchdogs, Supervisor | **52 / 100** | **Prototype Level** | Manual terminal launch; no systemd service for stack; no hardware/software watchdog. |
| **OVERALL STACK** | **End-to-End System** | **77 / 100** | **Pre-Production** | **Demo-viable only after applying Critical Phase 1 fixes.** |

---

## 1. Communication Architecture & Frontend Integration

### 1.1 End-to-End Communication Topology

```
+-----------------------------------------------------------------------------------+
|                        FRONTEND (Tablet React Native GCS)                         |
+-----------------------------------------------------------------------------------+
          │ REST (Auth, Start, Pause, Stop, Upload)    ▲ Socket.IO (5-10 Hz JSON)
          ▼                                            │ (telemetry, mission_status,
+──────────────────────────────────────────────────────┼────────────────────────────+
| ROVER BACKEND (`rover_backend` on Jetson Linux)      │  point_completed, safety)  |
|                                                      │                            |
|  [FastAPI REST Router] ──► [starlette run_in_tp] ──► [RosBridge Node Client]      |
|                                                      │ (thread-safe service calls)|
|  [AsyncIO Event Loop]  ◄─────────────────────────── [RoverState (RLock Store)]    |
+──────────────────────────────────────────────────────▲────────────────────────────+
                                                       │
                                        rclpy MultiThreadedExecutor(4)
                                                       │
+──────────────────────────────────────────────────────┴────────────────────────────+
| ROS 2 HUMBLE MIDDLEWARE BUS                                                       |
|                                                                                   |
|  /mavros/local_position/odom  ──► rpp_controller ──► /rpp/command                 |
|                                        ▲                     │                    |
|                                        │                     ▼                    |
|  /nav_path, /segment_goal     ──► mission_manager ──► cmd_vel_bridge (50 Hz)      |
|                                        │                     │                    |
|                                        ▼                     ▼                    |
|                                 spray_controller     /mavros/setpoint_raw/local   |
+───────────────────────────────────────────────────────────────────────────────────+
                                                               │ MAVLink 2 @ 921600
                                                               ▼
                                                    Holybro Pixhawk 6X (PX4)
```

### 1.2 Operator Visibility Without Hangs
The operator must have continuous, low-latency awareness of what the rover is executing:
- **Streaming Pipeline** (`rover_backend/realtime.py`):
  - A background async task `_broadcast_loop()` runs at `telemetry_broadcast_hz` (default 5.0 Hz).
  - Emits `telemetry` (vehicle battery, heading, raw GNSS HRMS/VRMS, fix type, speed, local coordinates).
  - Emits `mission_status` (mission lifecycle state, active point ID, index, total/completed/skipped/failed counts, progress percentage, spray confirmation).
  - Emits event-driven discrete payloads on transitions: `point_completed`, `point_failed`, `safety_state`, `mission_state`.
- **Latency Optimization**:
  - State changes in ROS callbacks call `notify_authoritative_state_changed()`, which invokes `loop.call_soon_threadsafe(state_change_event.set)`.
  - This wakes `_broadcast_loop()` out-of-schedule, delivering state transitions to the frontend in **< 30 ms**.

### 1.3 Failure Modes, Deadlocks, & Crash Hazards

#### Hazard 1.1: The 5.0s Service Timeout vs 5.6s Settle Race (`RosServiceOutcomeUnknownError`)
- **Location**: `rover_backend/ros_bridge.py:3475-3535` and `mission_manager/mission_manager_node.py:2130-2165`.
- **Mechanism**:
  - When the operator clicks "Start", FastAPI calls `ros_bridge.start_mission()`, which invokes the ROS service `/mission_manager/start`.
  - `ros_bridge` has `SERVICE_RESPONSE_TIMEOUT_SEC = 5.0`.
  - Inside `mission_manager`, the start sequence physically settles the rover before driving:
    1. 0.60s stream settle (`OFFBOARD_STREAM_SETTLE_SEC`)
    2. Switch to OFFBOARD mode and wait for confirmation (`VEHICLE_STATE_CONFIRM_TIMEOUT_SEC = 3.0`)
    3. 0.50s post-offboard settle (`OFFBOARD_BEFORE_ARM_SETTLE_SEC`)
    4. Send ARM command and wait for confirmation (up to 3.0s)
  - If PX4 takes 2.5s to confirm mode and 2.0s to confirm arming:
    $$\text{Total Time} = 0.60 + 2.50 + 0.50 + 2.00 = 5.60\text{ s}$$
  - At $t = 5.0\text{ s}$, `ros_bridge` times out, raises `RosServiceOutcomeUnknownError`, and returns HTTP `504 Gateway Timeout` to the frontend.
- **Production Impact**:
  - The tablet UI displays "Start Failed: Execution outcome unknown".
  - Meanwhile, on the field, the rover **successfully armed, switched to OFFBOARD, and started driving**!
  - The operator panics, clicks Start again (which fails with 409 Conflict because the mission is already RUNNING), or clicks E-Stop.
- **Production Fix**:
  Increase `SERVICE_RESPONSE_TIMEOUT_SEC` in `ros_bridge.py` from `5.0` to `10.0` for arming/mode-change services, and provide multi-stage progress reporting to the frontend during startup.

#### Hazard 1.2: `copy.deepcopy` Under RLock Freezing Event Loop
- **Location**: `rover_backend/state.py:628`.
- **Mechanism**:
  - `RoverState.section()` executes `with self._lock: return copy.deepcopy(self._state[section_name])`.
  - `build_mission_status_payload()` and `build_telemetry_payload()` call `section()` multiple times per tick at 5–10 Hz.
  - While `navigation_path_preview` is bounded to 500 points, deepcopying large nested dictionaries in Python consumes significant CPU and holds the lock across GIL boundaries.
  - When ROS bridge threads attempt to write incoming 50 Hz `/mavros/local_position/odom` data, they block on `self._lock`.
- **Production Fix**:
  Use atomic shallow dictionary copies or immutable dataclass snapshots (`copy.copy` of top-level containers) rather than recursive deep copies.

---

## 2. Path Engine & Geometry Fidelity (`trajectory_generator`)

### 2.1 Coordinate Conversion Pipeline: WGS-84 to Local ENU

```
Surveyed CSV (Lat, Lon)
   │
   ▼
[trajectory_generator/localization_frame.py]
PX4 Azimuthal Equidistant MapProjection (Radius = 6,371,000 m)
Referenced to PX4 EKF2 `gp_origin` (/mavros/global_position/gp_origin)
   │
   ▼
Local NED Frame (North, East, Down)
   │
   ▼
[MAVROS FTF Transform: x_enu = East, y_enu = North, z_enu = -Down]
Local ENU Frame (`map`)
   │
   ▼
Discrete Interpolation (50 mm Spacing) + Turnaround Dummy Geometry
```

### 2.2 Mathematical Algorithm for Millimeter Stability
1. **Origin Synchronization**:
   - `trajectory_generator` does not invent an arbitrary local origin. It subscribes to `/mavros/global_position/gp_origin` and actively requests `GPS_GLOBAL_ORIGIN` (MAVLink message `#49`) from PX4 via `/mavros/cmd/command`.
   - By anchoring the origin to the exact point chosen by PX4's EKF2 estimator, coordinate divergence between the path plan and the real-time position estimate is mathematically $0.000\text{ mm}$.
2. **Azimuthal Equidistant Projection**:
   $$\cos c = \sin \phi_0 \sin \phi + \cos \phi_0 \cos \phi \cos(\lambda - \lambda_0)$$
   $$k' = \frac{c}{\sin c}$$
   $$x_{\text{North}} = R \cdot k' \cdot (\cos \phi_0 \sin \phi - \sin \phi_0 \cos \phi \cos(\lambda - \lambda_0))$$
   $$y_{\text{East}} = R \cdot k' \cdot (\cos \phi \sin(\lambda - \lambda_0))$$
   This is an exact Python port of PX4's `MapProjection::project()`.
3. **Discrete 50 mm Spacing**:
   Every linear segment is interpolated with spacing `interpolation_spacing_m = 0.05` (50 mm). This ensures the pure-pursuit lookahead algorithm never encounters geometric discontinuities or point starvation.

### 2.3 Row Transition & Dummy Alignment Logic (Extension Mode)
When `extension_mode == "ENABLE"`:
- Evaluates consecutive marking point transitions where distance $< 3.0\text{ m}$ (`EXTENSION_TRIGGER_DISTANCE_M`).
- Validates geometric angles:
  - Row transfer angle: $45^\circ \le \theta_{\text{transfer}} \le 135^\circ$ (sideways).
  - Row reversal angle: $\theta_{\text{reversal}} \ge 135^\circ$ (serpentine return).
- Computes an external dummy point projected **4.0 m outwards** (`dummy_point_distance_m`) along the incoming row heading vector.
- **Why this is critical**: Prevents the 4WD skid-steer from pivoting directly on top of the marked boundary point. The rover drives 4 meters into the headland, pivots cleanly without turf tearing over the surveyed spot, and enters the next row already aligned straight.

### 2.4 Production Weaknesses
- **Piecewise Linear Sharp Vertices**: The path consists of straight lines meeting at abrupt corners. While skid-steers can pivot on spot, moving turns suffer from centrifugal lateral error.
- **Recommendation**: Implement cubic spline interpolation or clothoid transition fillets for turns to allow smooth continuous driving at speed.

---

## 3. Motion Controller (`rpp_controller` + `cmd_vel_bridge`)

### 3.1 Tracking Architecture & Pose Ingestion
- **Pose Ingestion**: `/mavros/local_position/odom` at 50 Hz.
- **Path Ingestion**: `/nav_path` (`nav_msgs/Path`) with `TRANSIENT_LOCAL` QoS.
- **Active Goal**: `/segment_goal` (`geometry_msgs/PoseStamped`) and `/active_waypoint`.
- **Command Output**: `/rpp/command` (`RppCommand`) containing North/East velocity setpoints and explicit ENU yaw. Forwarded by `cmd_vel_bridge` to `/mavros/setpoint_raw/local` at 50 Hz.

### 3.2 Pure Pursuit & Motion Profiling
1. **Adaptive Lookahead**:
   $$L_a = \text{clamp}(0.55 \cdot v,\, 0.35\text{ m},\, 0.80\text{ m})$$
2. **Cross-Track Priority Zone**:
   - If cross-track error $|e_{xt}| > 15\text{ mm}$, enters priority mode: linear speed is capped and steering gain is maximized.
   - Exits priority mode once $|e_{xt}| < 8\text{ mm}$.
3. **Motion Envelopes**:
   - Acceleration: 0 to 1.0 m/s over 1.0 m ($a = 0.5\text{ m/s}^2$).
   - Deceleration: Begins 1.0 m before semantic goal, braking from 1.0 m/s down to `TERMINAL_FLOOR_SPEED_MPS = 0.15 m/s`.
   - Startup Ceiling: `acceleration_startup_ceiling_mps = 0.25 m/s` to overcome motor breakaway torque.
4. **Terminal Stop Authority (`radial20`)**:
   Within 0.75 m of goal, `TerminalStopRegulator` engages. Asserts literal zero velocity when radial error $r \le 20\text{ mm}$ and vehicle speed $< 0.01\text{ m/s}$.

### 3.3 Critical Production Failure Modes

```
                                 [Rover Approaches Waypoint Pn]
                                                │
                                                ▼
                              Is Radial Error <= 20 mm?
                                     │               │
                                    YES              NO
                                     │               │
                                     ▼               ▼
                          [Assert Stop (Zero)]   Did it cross goal plane?
                                                       │          │
                                                      YES         NO
                                                       │          │
                                                       │          ▼
                                                       │     [Brake toward 0.15 m/s]
                                                       ▼
                                         [LATCH ZERO & FREEZE HOLD]
                                         [NO AUTO-REVERSE RECOVERY]
                                                       │
                                                       ▼
                                         ❌ STALLED POINT IN DEMO
```

1. **Overshoot Lockup (No Auto-Reverse)**:
   - In `rpp_controller_node.py:121`, if ground slip causes the rover to cross the waypoint plane at $r = 24\text{ mm}$ (missing the 20 mm inner circle), the controller enforces: *"stop in safe hold; do not reverse automatically"*.
   - **Demo Impact**: The rover halts 24 mm past the point, fails the arrival settle timer, and sits frozen forever.
   - **Fix**: Add an automated low-speed ($0.08\text{ m/s}$) reverse crawl if overshoot is $< 80\text{ mm}$.
2. **Codebase Monolith (>10,600 Lines)**:
   - `rpp_controller_node.py` is 10,607 lines in a single file. It mixes math, geometry, ROS callbacks, parameter handlers, and diagnostic loggers.
   - High risk of regression during minor tuning; high cognitive overhead for field engineers.

---

## 4. Stack Orchestrator (`mission_manager`)

### 4.1 State Machine Lifecycle
```
 [EMPTY] ──► [LOADED] ──► [PREPARING] ──► [READY] ──► [RUNNING] ──► [COMPLETED]
                                                          │   ▲
                                                          ▼   │
                                                        [PAUSED]
                                                          │
                                                          ▼
                                                   [WAITING_FOR_NEXT] (Manual mode)
```

### 4.2 Concurrency & Worker Architecture
- Spins on `rclpy.executors.MultiThreadedExecutor(num_threads=4)`.
- Callback groups:
  - `_io_group = ReentrantCallbackGroup()`: 20 Hz control loop, 5 Hz status publisher, sensor subscriptions.
  - `_mission_service_group = MutuallyExclusiveCallbackGroup()`: Serializes `START`, `PAUSE`, `RESUME`, `STOP`, `CLEAR`.
  - `self._lock = threading.RLock()`: Guards state updates.

### 4.3 Safety Latches & E-Stop Security
- Every E-stop assertion increments `self._safety_generation`.
- Releasing E-stop requires calling `/mission_manager/release_emergency_stop` with the exact matching generation counter.
- A rogue or delayed `/emergency_stop = false` publication cannot release the rover.

### 4.4 Critical Failure Modes
1. **Crash Disconnect Leaves PX4 Armed in OFFBOARD**:
   If `mission_manager` crashes, `cmd_vel_bridge` streams zero velocity, but **PX4 remains armed in OFFBOARD mode**. If an operator approaches the rover, the drivetrain remains energized.
2. **Zero Mission Checkpointing**:
   If `mission_manager` restarts, it boots with `_state = "EMPTY"`. Progress on a 150-point mission is completely lost, requiring a full re-upload.
   - **Fix**: Persist `active_point_index` and completed points to disk (`~/.ros/dyx_mission_checkpoint.json`) after each spray.

---

## 5. Spray Control System (`spray_controller`)

### 5.1 Actuator Interface & PX4 Contract
- **Hardware Output**: Holybro Pixhawk 6X AUX5 PWM servo.
- **Command Route**: MAVLink `MAV_CMD_DO_SET_ACTUATOR` (`187`) via `/mavros/cmd/command`:
  - `param1 = 1.0`: Press servo (Spray ON).
  - `param1 = 0.0`: Release servo (Spray OFF).
  - Actuator Set 1 addressing AUX outputs.

### 5.2 Production Fail-Safes & Journaling
1. **Durable Journaling (`dyx_spray_controller_journal.json`)**:
   State transitions (`PRESS_COMMAND_SENT`, `PRESSED`, `RELEASE_UNCONFIRMED`, `COMPLETED`) are written to disk before command dispatch. On reboot after power loss, it detects unconfirmed sprays and avoids double-spraying.
2. **Hard Press Watchdog (`hard_press_timeout_sec = 5.0s`)**:
   Regardless of ROS health, the actuator cannot remain pressed for $> 5.0\text{ s}$.
3. **Arrival Verification**:
   Requires rover radial error $\le 30\text{ mm}$ and speed $\le 0.01\text{ m/s}$ continuously for $0.25\text{ s}$ (`pre_spray_stable_sec`) before triggering.

### 5.3 Production Blocker: FCU Parameter Mismatch
- `spray_controller_node.py` expects `PWM_AUX_FUNC5 = 301` (Actuator Set 1).
- Live parameters captured from the Pixhawk show `PWM_AUX_FUNC5 = 0`.
- **Impact**: If `PWM_AUX_FUNC5` is 0, PX4 ignores DO_SET_ACTUATOR and no paint is dispensed.
- **Fix**: Set and save `PWM_AUX_FUNC5 = 301` on the Pixhawk via NSH before live testing.

---

## 6. Multi-Node Health & Fault Isolation

### 6.1 Launch Configuration Audit (`rover.launch.py`)

| Node / Process | Launch Method | Respawn Policy | Delay | Production Assessment |
| :--- | :--- | :---: | :---: | :--- |
| `mavros` | `ExecuteProcess("ros2 launch...")` | **NONE (False)** | 0.0s | **CRITICAL FLAW**: Serial glitch permanently kills entire FCU link. |
| `rover_backend` | `launch_ros.actions.Node` | **False** | 4.0s | **CRITICAL FLAW**: Backend exit blinds operator and stops rover. |
| `cmd_vel_bridge` | `launch_ros.actions.Node` | True | 2.0s | Acceptable; bridge recovers fast. |
| `trajectory_generator` | `launch_ros.actions.Node` | True | 2.0s | Acceptable; path is retained with Transient Local QoS. |
| `spray_controller` | `launch_ros.actions.Node` | True | 2.0s | Acceptable; state recovered via journal. |
| `mission_manager` | `launch_ros.actions.Node` | True | 2.0s | Unsafe respawn: wipes mission state to EMPTY. |
| `rpp_controller` | `launch_ros.actions.Node` | True | 2.0s | Unsafe respawn: loses active tracking target. |

### 6.2 Cascade Failure Matrix

```
+─────────────────────+─────────────────────────────────────────────────────────────+
| Component Failed    | System Impact & Cascade Effect                              |
+─────────────────────+─────────────────────────────────────────────────────────────+
| MAVROS Exits        | Total stack paralysis. Position and odometry freeze.        |
|                     | Bridge enters timeout and commands zero. Stack unrecoverable |
|                     | without full restart.                                       |
+─────────────────────+─────────────────────────────────────────────────────────────+
| Backend Exits       | Heartbeat to `cmd_vel_bridge` drops within 1.5s.             |
|                     | Bridge zeros velocity. Operator loses all UI telemetry.     |
+─────────────────────+─────────────────────────────────────────────────────────────+
| Mission Manager Dies| Respawns with `_state=EMPTY` and `_emergency_stop=True`.    |
|                     | Rover stops, but PX4 stays armed in OFFBOARD.               |
+─────────────────────+─────────────────────────────────────────────────────────────+
| Septentrio RTK Drop | Fix drops from 6 (FIXED) to 5 (FLOAT).                       |
|                     | Mission Manager raises warning; RPP stops if tolerance lost.|
+─────────────────────+─────────────────────────────────────────────────────────────+
```

---

## 7. Systemd Services, Watchdogs, & Production Lifecycle

### 7.1 Current State: Prototype
- The only systemd unit is `systemd/bag-autorecord.service`.
- The rover stack (`rover.launch.py`) is started manually over SSH. An SSH dropout or terminal close risks terminating the stack.

### 7.2 Target Production Architecture

Create two monitored systemd services on Jetson:

#### 1. `dyx-rover.service` (`/etc/systemd/system/dyx-rover.service`)
```ini
[Unit]
Description=DYX 4WD Rover Core ROS 2 Stack
After=network-online.target nvargus-daemon.service
Wants=network-online.target

[Service]
Type=exec
User=flash
Group=flash
WorkingDirectory=/home/flash/rover_ws
Environment=ROS_DOMAIN_ID=0
Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ExecStart=/bin/bash -lc "source /opt/ros/humble/setup.bash && source /home/flash/rover_ws/install/setup.bash && ros2 launch rover_bringup rover.launch.py"
Restart=always
RestartSec=3
KillMode=mixed
KillSignal=SIGINT
TimeoutStopSec=10
LimitNOFILE=65536

[Install]
WantedBy=multi-user.target
```

#### 2. `dyx-backend.service` (`/etc/systemd/system/dyx-backend.service`)
```ini
[Unit]
Description=DYX 4WD Rover Backend REST & Socket.IO
After=dyx-rover.service
Requires=dyx-rover.service

[Service]
Type=exec
User=flash
Group=flash
WorkingDirectory=/home/flash/rover_ws
EnvironmentFile=/home/flash/rover_ws/src/rover_backend/config/backend.env
ExecStart=/bin/bash -lc "source /opt/ros/humble/setup.bash && source /home/flash/rover_ws/install/setup.bash && python3 -m rover_backend.main"
Restart=always
RestartSec=3
KillMode=mixed
TimeoutStopSec=5

[Install]
WantedBy=multi-user.target
```

---

## Action Plan for Commercial Demo Readiness

### Priority 1: Mandatory Pre-Demo Fixes (Immediate)
1. **Fix Launch Respawns**:
   - In `rover.launch.py`, change `rover_backend_node` to `respawn=True`.
   - Wrap `mavros` in a respawning handler so USB glitches do not permanently kill telemetry.
2. **Reconcile Spray Servo Param**:
   - Run `param show PWM_AUX_FUNC5` via PX4 NSH. If 0, execute `param set PWM_AUX_FUNC5 301` and `param save`.
3. **Extend Service Discovery/Response Timeouts**:
   - In `ros_bridge.py`, change `SERVICE_RESPONSE_TIMEOUT_SEC` to `10.0` for arming and starting commands.
4. **Implement Overshoot Auto-Reverse Recovery in RPP**:
   - If goal plane is crossed and $r \le 80\text{ mm}$, command reverse crawl at $0.08\text{ m/s}$ rather than freezing.

### Priority 2: System Hardening (Demo Ready)
1. **Install and Enable Systemd Services**: Deploy `dyx-rover.service` and `dyx-backend.service` so the stack boots automatically with the vehicle.
2. **Add Mission State Journaling**: Save completed point indices to disk in `mission_manager` so mid-mission hiccups can be resumed without restarting the job.
3. **Optimize State Locks**: Replace `copy.deepcopy` in `state.py` with shallow copy snapshots to reduce ASGI event loop latency.
