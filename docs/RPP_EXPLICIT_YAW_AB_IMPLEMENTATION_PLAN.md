# A/B RPP Explicit Yaw — Production Implementation Plan (v1)

> ⚠ **SUPERSEDED by
> [`RPP_EXPLICIT_YAW_AB_IMPLEMENTATION_PLAN_v2.md`](RPP_EXPLICIT_YAW_AB_IMPLEMENTATION_PLAN_v2.md).**
> Independent verification (2026-09-10) found this v1 firmware baseline (`67b4e5c`,
> "stationary-only") went stale within about an hour of being written: the firmware branch had
> already advanced to `0ad2fc56` ("enable absolute yaw steering at any speed"), which implements
> the exact moving-speed explicit-yaw contract this document lists as the B-side blocking gate. v2
> re-baselines the firmware section to `0ad2fc56` / `7f27e7a1` and adds a topic-name
> disambiguation note (`/rpp/command` vs. the existing `/rpp/command_speed_mps`). Everything else in
> this v1 document — the ROS-side design, invariants, phase plan, and test strategy — was
> independently verified accurate against live source and is unchanged in v2. Read v2 for current
> status; this file is kept for history.

**Repository:** `ShariniSenthil/DYX_4WD_JETSON_PROTOTYPE`  
**Source branch at review start:** `feat/rtcm-direct-gnss-uart`  
**Verified source HEAD:** `1fc7bf328453b46b50354a2f1b011769de53bb44`  
**Verified source tree:** `17b2b4cc01975a9c16a98a274d34b6b842c36bee`  
**Verified RPP source blob:** `af086856082b485bc062c7ac27c95d4afec04236`  
**Verified bridge source blob:** `59e0a3158eea9764dd4c723e0d4b5cb3240b9f75`  
**Proposed implementation branch:** `feat/rpp-explicit-yaw-ab` — **not created by this document**  
**Firmware dependency branch:** `ShariniSenthil/DYX_4WD_PX4_FIRMWARE_16.2/fix/rover-land-state`  
**Firmware reviewed commit:** `67b4e5cc8e44ba489e89fa2934870ef4362eca73`  
**Document status:** **PLAN ONLY — NO RPP / BRIDGE IMPLEMENTATION APPLIED**  
**Date:** 2026-09-10

**Document purpose:** Define a production-safe A/B migration from the current RPP velocity-vector heading
contract to an atomic **velocity + explicit yaw** contract. The current behavior remains available as
the A-side rollback path. The B-side must not be field-enabled until the PX4 differential-rover
firmware accepts finite `trajectory_setpoint.yaw` as heading authority at **all translational speeds**.

Every “PROPOSED — NOT APPLIED” excerpt below is design guidance. It is not a claim that the source
already contains that code.

---

# STATUS — 2026-09-10: PLAN ONLY / IMPLEMENTATION NOT STARTED

**This section is authoritative for current state.**

## Current verified software state

The current ROS2 production control path is still the A-side:

```text
RPP
    final guidance / command bearing
        │
        ├─ converted into North/East velocity components
        ▼
/rpp/velocity_ned
    geometry_msgs/Vector3Stamped
        │
        ▼
cmd_vel_bridge
    mavros_msgs/PositionTarget
    velocity only
    IGNORE_YAW
    IGNORE_YAW_RATE
    type_mask = 3527
        │
        ▼
MAVROS
    ROS ENU → MAVLink/PX4 NED transforms
        │
        ▼
PX4 DifferentialVelControl
    bearing derived from velocity vector
        │
        ▼
Rover attitude / yaw-rate cascade
        │
        ▼
motors
```

The current production RPP pivot path still uses a **carrier velocity vector** to keep PX4 in its
native differential-rover spot-turn state until the tighter RPP heading criterion is satisfied.

No RPP explicit-yaw command message exists in the reviewed baseline.

No B-side `/rpp/command` publisher/subscriber exists in the reviewed baseline.

The current bridge does not forward a valid yaw setpoint.

## Current firmware dependency state

Firmware commit `67b4e5c` correctly added **stationary absolute-yaw pivot support**:

```text
speed < 0.01 m/s + finite yaw
    → speed = 0
    → bearing = explicit yaw

speed < 0.01 m/s + yaw ignored / NaN
    → speed = 0
    → bearing = current vehicle yaw
```

However, at moving speed the reviewed firmware still derives bearing from the velocity vector and
ignores explicit yaw.

Therefore:

```text
Firmware 67b4e5c:
    stationary explicit yaw = supported
    moving explicit yaw     = NOT YET supported
```

**B-side moving field testing is blocked until that firmware contract is completed.**

The required firmware behavior is:

```text
finite trajectory_setpoint.yaw
    → heading authority = explicit yaw
    → valid at both moving and zero speed

yaw invalid / NaN + moving velocity
    → backward-compatible bearing from velocity vector

yaw invalid / NaN + zero velocity
    → hold current vehicle yaw
```

The ROS2 A-side can be implemented and regression-tested before that firmware update because Mode A
continues to send `IGNORE_YAW`.

## Phase completion status

| Phase | Scope | Status |
|---|---|---|
| 0 | baseline/source/firmware contract lock | **Review complete; implementation not started** |
| 1 | atomic `rpp_interfaces/RppCommand` transport | **Not started** |
| 2 | default-off A/B selection and Mode-A preservation | **Not started** |
| 3 | RPP explicit-yaw propagation for moving control | **Not started** |
| 4 | bridge explicit-yaw forwarding + hard-safety suppression | **Not started** |
| 5 | B-side zero-translation pivot + intentional HOLD semantics | **Not started** |
| 6 | automated integration/safety validation | **Not started** |
| 7 | Jetson/PX4 A-side regression field test | **Not performed** |
| 8 | B-side straight tracking field test | **Blocked by full firmware support** |
| 9 | B-side pivot/transition field test | **Blocked by full firmware support** |
| 10 | repeated A/B mission comparison | **Not performed** |
| 11 | production-default decision | **Not reached** |

---

# 1. Executive decision

Implement explicit yaw as an **A/B selectable RPP-to-PX4 command contract**.

Do **not** delete, replace, or rewrite away the current velocity-vector path.

The single authority switch is:

```text
rpp_explicit_yaw_enabled = false
    → A-SIDE / CURRENT behavior
    → RPP velocity vector
    → bridge velocity-only mask 3527
    → PX4 derives heading from vector
    → current carrier-vector pivot remains active

rpp_explicit_yaw_enabled = true
    → B-SIDE / NEW behavior
    → atomic RPP velocity + explicit ENU yaw command
    → bridge velocity+yaw mask 2503
    → PX4 speed authority = velocity magnitude
    → PX4 heading authority = explicit yaw
    → pivot = zero translation + explicit target yaw
```

The default for the first implementation and deployment is:

```text
rpp_explicit_yaw_enabled = false
```

The switch is **restart-only**.

Do not hot-switch A/B while a mission is active.

Do not dynamically choose the route based on whichever topic was most recently received.

---

## 1.1 Non-negotiable invariants

1. `rpp_explicit_yaw_enabled` defaults to **false**.
2. `false` must preserve current production RPP and bridge behavior.
3. Mode A keeps `/rpp/velocity_ned` and bridge type mask `3527`.
4. Mode A keeps the current carrier-vector pivot behavior.
5. Mode B uses one atomic command containing velocity and yaw from the **same RPP control cycle**.
6. Mode B normal control uses explicit yaw as heading authority.
7. Mode B must never silently fall back to Mode A when the B command is malformed, stale, or missing.
8. A bridge-level hard safety stop must always suppress yaw authority.
9. E-stop, mission disable, heartbeat timeout, PX4 disconnect, disarm, wrong mode, RPP timeout, or invalid
   B command must never rotate the rover because of stale yaw.
10. Mode B pivot uses zero translational velocity plus the real target/path yaw.
11. Mode B must not use the ±carrier-bearing actuation workaround.
12. The existing RPP guidance formulas are not changed in this patch.
13. The existing RPP speed/deceleration formulas are not tuned in this patch.
14. Existing path geometry, terminal accuracy certificates, mission sequencing, marking logic, and RTK
    logic stay unchanged.
15. No yaw-rate feed-forward is introduced.
16. `maximum_yaw_rate_radps` in the bridge remains compatibility-only unless a later independent phase
    explicitly changes that.
17. RPP yaw remains **ROS ENU yaw**; the bridge must not manually convert yaw to NED.
18. MAVROS remains responsible for the ROS ENU ↔ MAVLink/PX4 NED frame transform.
19. The current A-side stays available for immediate rollback after B-side deployment.
20. RPP/bridge mode mismatch must fail closed, not cross-fallback.
21. A/B field evidence must be recorded separately; do not infer B quality from implementation alone.
22. Full B-side field activation is blocked until PX4 supports explicit yaw at all speeds.
23. The firmware's invalid-yaw velocity-vector fallback is a firmware compatibility feature, **not**
    permission for the ROS B-side to silently degrade into A.
24. Absolute marking truth and RPP tracking error remain separate measurement domains.

---

# 2. Current verified production path

## 2.1 Current RPP command path

Current command publisher:

```text
src/rpp_controller/rpp_controller/rpp_controller_node.py
```

Current output topic:

```text
/rpp/velocity_ned
```

Current message:

```text
geometry_msgs/Vector3Stamped

vector.x = North velocity [m/s]
vector.y = East velocity  [m/s]
vector.z = 0
```

Current RPP local geometry convention is:

```text
map x = East
map y = North

bearing = atan2(delta_north, delta_east)
```

Therefore current RPP bearing values use ROS ENU yaw convention:

```text
0 rad       = East
+pi/2 rad   = North
±pi rad     = West
-pi/2 rad   = South
```

Current moving command construction is conceptually:

```python
north = speed * math.sin(command_bearing)
east  = speed * math.cos(command_bearing)
```

The desired heading already exists inside RPP before it is reduced to a velocity vector.

---

## 2.2 Current bridge path

Current bridge:

```text
src/jetson_4wd_control/jetson_4wd_control/cmd_vel_bridge.py
```

Current input:

```text
/rpp/velocity_ned
```

Current output:

```text
/mavros/setpoint_raw/local
mavros_msgs/PositionTarget
```

Current type mask is the velocity-only contract:

```python
TYPE_MASK_VELOCITY_ONLY = (
    PositionTarget.IGNORE_PX
    | PositionTarget.IGNORE_PY
    | PositionTarget.IGNORE_PZ
    | PositionTarget.IGNORE_AFX
    | PositionTarget.IGNORE_AFY
    | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW
    | PositionTarget.IGNORE_YAW_RATE
)
```

Numeric value:

```text
3527
```

Current bridge ROS fields are ENU even though the MAVLink coordinate frame is local NED:

```text
message.velocity.x = East
message.velocity.y = North
message.velocity.z = 0

message.yaw      = 0       # ignored by mask
message.yaw_rate = 0       # ignored by mask
```

MAVROS performs the frame transformations.

---

## 2.3 Current bridge safety gates

The bridge already fail-stops on conditions including:

```text
E-stop active
mission disabled
backend heartbeat stale
PX4 disconnected
PX4 disarmed
PX4 not in OFFBOARD
RPP command stale / timed out
```

Those gates must remain authoritative.

The B-side changes **what a normal valid control command contains**.

It must not weaken any existing bridge-level safety gate.

---

# 3. Critical current-code findings

## 3.1 The desired yaw already exists in RPP

The B-side does not need a new path-heading formula.

During normal tracking, RPP already computes a final guidance / command bearing and then converts that
bearing into North/East components.

Correct B-side design:

```text
existing final RPP command bearing
    ├─ continues to produce North/East velocity
    └─ is also published explicitly as yaw_enu_rad
```

Do not reconstruct B-side yaw downstream from the velocity vector.

Doing that would recreate the current coupling and defeat the purpose of the explicit-yaw contract.

---

## 3.2 The current generic velocity publisher does not know yaw

Current `publish_velocity_ned(...)` receives North/East velocity values and applies existing
speed-profile / limiting behavior before publishing.

Many RPP call sites reach this helper.

A careless B-side implementation such as:

```python
# WRONG FOR THE B-SIDE
yaw = math.atan2(north, east)
```

inside the final publisher would hide any call site that forgot to propagate the real RPP heading
authority.

The correct B-side must pass the final heading decision explicitly from the control logic that owns it.

---

## 3.3 `publish_stop()` is semantically overloaded

Current code uses zero-velocity publication for multiple reasons.

Those reasons are **not equivalent** once yaw becomes independently active.

They must be classified into two categories.

### Category 1 — hard safety / fail-closed zero

Examples include:

```text
E-stop
mission disabled
stale odometry
stale active waypoint
invalid guidance data
invalid command
missing required mission metadata
tracking calculation failure
command timeout
other fail-closed guard
```

Required B-side behavior:

```text
velocity = 0
yaw_valid = false
```

At the bridge:

```text
velocity = 0
IGNORE_YAW
type_mask = 3527
```

Result:

```text
NO commanded rotation
```

### Category 2 — intentional control HOLD

Examples can include:

```text
pivot itself
post-pivot heading hold
post-pivot settle / recenter hold
terminal verified zero hold
marking hold
deliberate stationary controller state
```

Required B-side behavior when a finite intended heading is known:

```text
velocity = 0
yaw_valid = true
yaw = intended held heading
```

At the bridge:

```text
velocity = 0
explicit yaw active
type_mask = 2503
```

Result:

```text
stationary heading authority remains active
```

**Do not globally turn every `publish_stop()` call into a yaw-hold.**

That would make safety paths capable of rotating the rover.

---

## 3.4 Current pivot carrier is a transport workaround, not the target architecture

Current legacy alignment/pivot logic generates a carrier bearing because current PX4 derives heading
from a nonzero velocity vector.

Conceptually:

```text
true target heading
    │
    ├─ if large heading error
    ▼
dynamic carrier bearing ≈ current yaw ± 60°
    │
    ▼
nonzero velocity vector
    │
    ▼
PX4 sees large heading error
    │
    ▼
native SPOT_TURNING
```

This is valid A-side behavior and must be retained for rollback.

It is not valid B-side actuation.

B-side pivot becomes:

```text
true target/path yaw
    │
    ▼
RppCommand
velocity_north = 0
velocity_east  = 0
yaw_enu        = true target/path yaw
yaw_valid      = true
    │
    ▼
bridge mask 2503
    │
    ▼
PX4 explicit-yaw heading authority
    │
    ▼
SPOT_TURNING / yaw cascade
```

The existing RPP pivot lifecycle and certificates should remain.

Only the actuation transport changes.

---

# 4. Target architecture

```text
                                   RPP CONTROL LOOP
                                          │
                                          │ existing guidance math
                                          ▼
                              ┌─────────────────────────┐
                              │ final speed / N-E vector│
                              │ final ENU command yaw   │
                              └────────────┬────────────┘
                                           │
                              A/B startup selection
                                           │
                 ┌─────────────────────────┴─────────────────────────┐
                 │                                                   │
                 │ A: explicit_yaw=false                             │ B: explicit_yaw=true
                 ▼                                                   ▼
      ┌────────────────────────┐                         ┌──────────────────────────┐
      │ /rpp/velocity_ned      │                         │ /rpp/command             │
      │ Vector3Stamped         │                         │ rpp_interfaces/RppCommand│
      │ N/E velocity only      │                         │ N/E velocity + ENU yaw   │
      └────────────┬───────────┘                         └─────────────┬────────────┘
                   │                                                   │
                   ▼                                                   ▼
          cmd_vel_bridge                                      cmd_vel_bridge
                   │                                                   │
                   │ mask 3527                                         │ normal: mask 2503
                   │ yaw ignored                                       │ yaw valid
                   ▼                                                   ▼
             MAVROS ENU→NED                                      MAVROS ENU→NED
                   │                                                   │
                   └─────────────────────────┬─────────────────────────┘
                                             ▼
                                    PX4 trajectory_setpoint
                                             │
                    ┌────────────────────────┴────────────────────────┐
                    │                                                 │
          yaw invalid / NaN                                  yaw finite
                    │                                                 │
                    ▼                                                 ▼
         velocity-vector fallback                         explicit heading authority
                    │                                                 │
                    └────────────────────────┬────────────────────────┘
                                             ▼
                                  DifferentialVelControl
                                  speed = |velocity|
                                  bearing = selected yaw/bearing
                                             │
                                             ▼
                                  DifferentialAttControl
                                             │
                                             ▼
                                  DifferentialRateControl
                                             │
                                             ▼
                                            motors
```

Only one ROS command input path is active in the bridge per startup mode.

---

# 5. A/B command-state matrix

| RPP state | A-side command | B-side command | B yaw validity |
|---|---|---|---|
| TRACK | N/E velocity | N/E velocity + final guidance yaw | valid |
| APPROACH | N/E velocity | N/E velocity + final approach yaw | valid |
| BRAKE while still moving | N/E velocity | N/E velocity + final command yaw | valid |
| PIVOT | current carrier vector | zero velocity + true pivot yaw | valid |
| intentional heading HOLD | zero vector | zero velocity + held intended yaw | valid |
| post-pivot settle | current zero/carrier lifecycle behavior | zero velocity + pivot/leg yaw | valid |
| terminal deliberate zero | zero vector | zero + held finite yaw where semantically required | valid |
| E-stop | zero vector | zero | **invalid** |
| mission disabled | zero vector | zero | **invalid** |
| stale RPP prerequisites | zero vector | zero | **invalid** |
| invalid/nonfinite guidance | zero vector | zero | **invalid** |
| bridge heartbeat timeout | bridge zero | bridge zero + yaw ignored | **suppressed** |
| bridge RPP timeout | bridge zero | bridge zero + yaw ignored | **suppressed** |
| PX4 disconnected/disarmed/not OFFBOARD | bridge zero | bridge zero + yaw ignored | **suppressed** |

This table is a design requirement.

Any zero-command call site that cannot be confidently classified must default to the **hard-safety**
semantics until proven otherwise.

---

# 6. File-change matrix

| File | State | Planned role |
|---|---|---|
| `src/rpp_interfaces/msg/RppCommand.msg` | **NEW** | atomic velocity + explicit yaw command |
| `src/rpp_interfaces/CMakeLists.txt` | **NEW** | ROS interface generation |
| `src/rpp_interfaces/package.xml` | **NEW** | interface package dependencies/export |
| `src/rpp_controller/package.xml` | **MODIFY** | depend on `rpp_interfaces` |
| `src/rpp_controller/rpp_controller/rpp_controller_node.py` | **MODIFY** | default-off A/B publisher, yaw propagation, B pivot output, safety/HOLD distinction, diagnostics |
| `src/rpp_controller/rpp_controller/legacy_alignment.py` | **REVIEW / MINIMAL MODIFY ONLY IF REQUIRED** | keep lifecycle; remove carrier dependency only from B actuation without changing A behavior |
| `src/rpp_controller/test/...` | **MODIFY / NEW TESTS** | A regression + B command authority + pivot/safety coverage |
| `src/jetson_4wd_control/package.xml` | **MODIFY** | depend on `rpp_interfaces` |
| `src/jetson_4wd_control/jetson_4wd_control/cmd_vel_bridge.py` | **MODIFY** | startup A/B subscription, mask 2503, yaw validation, hard-safety mask 3527 |
| `src/jetson_4wd_control/test/test_cmd_vel_bridge_contract.py` | **NEW** | behavioral bridge tests |
| `src/rover_bringup/launch/rover.launch.py` | **MODIFY** | one default-false source of truth passed to RPP + bridge |
| `docs/4WD_CM_TRACKING_PRODUCTION_TASKSHEET_FINAL.md` | **NO DESIGN CHANGE** | already defines desired P4/P5/P6 contract; update status only after validation |
| PX4 firmware repo | **SEPARATE DEPENDENCY** | full moving+stationary explicit-yaw support required before B field activation |
| MAVROS source | **NO CHANGE** | retain existing frame transforms |
| mission manager | **NO CHANGE EXPECTED** | mission authority stays unchanged |
| frontend | **NO CHANGE** | no operator hot-switch UI in first implementation |
| RTK correction code | **NO CHANGE** | unrelated transport path |

---

# 7. New interface package — `rpp_interfaces`

## 7.1 Why a dedicated interface package

Do not overload:

```text
Vector3Stamped.vector.z
```

with yaw.

`vector.z` currently means zero vertical velocity. Reusing it as yaw is a semantic contract violation
and makes frame validation ambiguous.

Do not use a separate topic such as:

```text
/rpp/velocity_ned
/rpp/yaw
```

for B-side authority.

Two topics can be received from different RPP cycles.

For a 20 Hz controller, even one-cycle skew can pair:

```text
velocity(k)
with
yaw(k-1)
```

and contaminate A/B evidence.

The B-side needs one atomic message.

---

## 7.2 Proposed message

New file:

```text
src/rpp_interfaces/msg/RppCommand.msg
```

**PROPOSED — NOT APPLIED**

```text
# One atomic command produced by one RPP control cycle.
#
# velocity_north_mps / velocity_east_mps:
#   local tangent-plane velocity components in m/s.
#
# yaw_enu_rad:
#   ROS ENU yaw.
#   0 = East, +pi/2 = North, +/-pi = West, -pi/2 = South.
#
# yaw_valid:
#   true  = yaw_enu_rad is authoritative for normal B-side control.
#   false = no yaw authority; downstream must fail closed to a no-rotation
#           safety stop rather than silently reverting to vector steering.

std_msgs/Header header
float64 velocity_north_mps
float64 velocity_east_mps
float64 yaw_enu_rad
bool yaw_valid
```

No yaw-rate field is added.

No path ID, waypoint ID, controller state, or debugging metadata is added to this control message in
the first patch.

Keep the control interface narrow.

---

## 7.3 Proposed topic

```text
/rpp/command
```

QoS should match the current RPP command transport:

```text
KEEP_LAST
depth = 1
RELIABLE
VOLATILE
```

Do not use a deep command queue.

The bridge must act on the latest control authority, not replay stale historical commands.

---

## 7.4 Proposed `rpp_interfaces/CMakeLists.txt`

**PROPOSED — NOT APPLIED**

```cmake
cmake_minimum_required(VERSION 3.8)
project(rpp_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RppCommand.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

---

## 7.5 Proposed `rpp_interfaces/package.xml`

Follow the existing workspace ROS interface-package pattern.

Required intent:

```text
ament_cmake
rosidl_default_generators
std_msgs
rosidl_default_runtime
member_of_group rosidl_interface_packages
```

Do not make `rpp_controller` or `jetson_4wd_control` generate the interface locally.

One interface package must own the message schema.

---

# 8. Single A/B configuration authority

## 8.1 Proposed launch constant

In:

```text
src/rover_bringup/launch/rover.launch.py
```

add one initial source of truth:

**PROPOSED — NOT APPLIED**

```python
RPP_EXPLICIT_YAW_ENABLED = False
```

Pass the same value to:

```text
rpp_controller
cmd_vel_bridge
```

The initial production deployment must remain:

```text
False
```

---

## 8.2 Restart-only rule

The command transport must not be hot-switched during a mission.

Correct sequence:

```text
mission stopped / rover safe
    ↓
change A/B launch/config value
    ↓
restart affected ROS stack
    ↓
both RPP and bridge start in same mode
    ↓
verify startup log
    ↓
begin A or B test
```

Forbidden:

```text
live parameter toggle while commands are flowing
```

RPP already has a restart-only parameter model for non-approved runtime settings. The new transport
mode should follow that pattern.

The bridge must also treat the selected input contract as startup configuration.

---

## 8.3 Mode mismatch must fail closed

Recommended structural design:

```text
RPP A:
    publishes only /rpp/velocity_ned

Bridge A:
    subscribes only /rpp/velocity_ned

RPP B:
    publishes only /rpp/command

Bridge B:
    subscribes only /rpp/command
```

Therefore accidental mismatch produces:

```text
no valid command received
    ↓
existing RPP command timeout
    ↓
bridge safety stop
```

Do **not** make the bridge subscribe to both topics and accept whichever arrived last.

That would create an implicit fallback path and destroy A/B isolation.

---

# 9. RPP publisher architecture

## 9.1 Preserve current speed-control math

The first explicit-yaw patch must not re-implement acceleration, deceleration, speed limiting, terminal
speed behavior, or precision speed regulation.

The existing pipeline continues to determine final North/East velocity.

Only the final transport is extended with an explicit heading field.

---

## 9.2 Proposed selected output seam

The clean seam is immediately after existing speed-limiting logic has produced the final N/E command.

Conceptual design:

```text
existing control calculation
    ↓
final North/East command
final exact RPP command bearing
    ↓
selected output emitter
    ├─ A → current Vector3Stamped
    └─ B → RppCommand
```

**PROPOSED — NOT APPLIED**

```python
def _emit_selected_command(
    self,
    *,
    north_mps: float,
    east_mps: float,
    yaw_enu_rad: float,
    yaw_valid: bool,
) -> None:
    if not self.rpp_explicit_yaw_enabled:
        self._publish_legacy_velocity_vector(
            north_mps=north_mps,
            east_mps=east_mps,
        )
        return

    self._publish_explicit_yaw_command(
        north_mps=north_mps,
        east_mps=east_mps,
        yaw_enu_rad=yaw_enu_rad,
        yaw_valid=yaw_valid,
    )
```

Names are illustrative; implementation should follow the existing local naming style.

---

## 9.3 Do not reconstruct missing B yaw

This is forbidden:

```python
if yaw_enu_rad is None:
    yaw_enu_rad = math.atan2(north_mps, east_mps)
```

Reason:

A missing explicit yaw is a wiring defect in Mode B.

Automatically reconstructing it makes the defect invisible and restores the exact coupling the B-side
is intended to remove.

Correct behavior for a missing/nonfinite required yaw in B:

```text
detect invalid B command
    ↓
publish fail-closed zero with yaw_valid=false
    ↓
diagnostic/log reason
```

No automatic Mode-A steering fallback.

---

# 10. Moving TRACK / APPROACH contract

## 10.1 Existing authority

Normal RPP tracking already owns a final guidance bearing.

That same value currently generates the N/E vector.

Mode B must publish that exact final bearing.

Conceptually:

```python
north = speed * math.sin(guidance_bearing)
east  = speed * math.cos(guidance_bearing)
```

B-side output:

```text
velocity_north_mps = north
velocity_east_mps  = east
yaw_enu_rad        = guidance_bearing
yaw_valid          = true
```

If a later stage limits/modifies the command bearing, publish the **final post-limiter bearing** that
actually owns the velocity command.

Do not publish a pre-limiter path bearing beside a post-limiter velocity command.

---

## 10.2 Precision-guidance path

`publish_precision_velocity_ned(command_bearing, result)` already receives the explicit command
bearing directly.

This is the easiest B-side integration point.

Required invariant:

```text
RppCommand.yaw_enu_rad == exact final command_bearing used by the precision control cycle
```

not:

```text
raw path heading
current vehicle yaw
atan2(reconstructed output vector)
previous-cycle heading
```

---

## 10.3 Generic moving paths

Every moving call site that currently reaches:

```text
publish_velocity_ned(north, east, ...)
```

must be audited.

For B-side moving output, each call site must provide the exact heading authority that generated that
motion decision.

A test should make it impossible to add a new moving B-side call path without a finite explicit yaw.

---

# 11. Hard safety stop contract

## 11.1 RPP safety output

Keep `publish_stop()` semantically safe.

Recommended B-side meaning:

```text
publish_stop()
    → N = 0
    → E = 0
    → yaw_valid = false
```

Do not latch a stale heading into this generic safety helper.

If an intentional control state needs heading hold, use a **different helper**.

---

## 11.2 Proposed explicit heading-hold helper

**PROPOSED — NOT APPLIED**

```python
def publish_heading_hold(self, yaw_enu_rad: float) -> None:
    if not math.isfinite(yaw_enu_rad):
        self.publish_stop()
        return

    # B: zero translation + active heading authority
    # A: preserve the currently intended legacy zero behavior
    ...
```

This helper is only for known intentional controller states.

It must not be used for generic fail-closed conditions.

---

## 11.3 Last valid heading latch

A last-valid-yaw latch can be useful for deliberate HOLD states.

Suggested state:

```text
last_explicit_yaw_cmd_rad
```

Update it only when:

```text
Mode B
AND yaw_valid=true
AND yaw is finite
AND command was accepted for publication
```

Do not update it from:

```text
invalid command
safety stop
NaN
stale data
reconstructed velocity direction
```

If a HOLD requests a latched yaw but no valid latch exists:

```text
fail closed → yaw_valid=false
```

Do not invent a heading.

---

# 12. Pivot contract

## 12.1 A-side — unchanged

Mode A must continue to use the current carrier-vector mechanism.

No refactor that changes the A-side pivot timing, threshold, speed, or release logic belongs in this
patch.

A-side:

```text
true path bearing
    ↓
carrier logic
    ↓
nonzero velocity carrier
    ↓
PX4 vector-derived heading
    ↓
native SPOT_TURNING
```

---

## 12.2 B-side — zero-translation explicit yaw

Mode B:

```text
target_yaw = existing true latched path / leg bearing

velocity_north_mps = 0.0
velocity_east_mps  = 0.0
yaw_enu_rad        = target_yaw
yaw_valid          = true
```

No fake 1.0 m/s translational vector.

No ±60° carrier offset.

No yaw-rate feed-forward.

---

## 12.3 Preserve pivot lifecycle

The transport change must not delete the existing state/certificate logic.

Preserve, as applicable to the current production lifecycle:

```text
BRAKE / stop before pivot
verify stationary condition
issue pivot authority
verify heading convergence
verify yaw-rate settle
verify position / drift condition
release to fixed next leg
post-pivot settle / reanchor behavior currently required
```

The goal is:

```text
same decision lifecycle
different actuation contract
```

not:

```text
new actuation contract
+ rewritten pivot state machine
```

---

## 12.4 Legacy directive naming

The existing legacy alignment state machine may contain transport-specific names such as
`NATIVE_CARRIER`.

Do not broaden the first patch merely to rename the entire state machine.

Acceptable first implementation:

```text
legacy directive remains internally named NATIVE_CARRIER
A mapping → current carrier vector
B mapping → zero translation + explicit true target yaw
```

But B-side diagnostics must not falsely state that a carrier vector was physically issued.

If a minimal generic acknowledgement is needed, add it narrowly and preserve A compatibility.

---

# 13. Post-pivot transition

The most important B-side handoff is:

```text
PIVOT
    velocity = 0
    yaw = fixed-leg heading
        ↓
heading / yaw-rate / position certificates pass
        ↓
TRACK
    translational velocity restored
    explicit yaw remains the same fixed-leg / guidance authority
```

Required properties:

1. No one-cycle loss of yaw authority.
2. No zero-yaw default injection.
3. No use of previous carrier bearing.
4. No sudden Mode-A vector fallback.
5. No unnecessary translation during the pivot state.
6. Existing fixed-leg geometry remains authoritative.
7. Existing reanchor behavior is not silently expanded or removed in the transport patch.

This transition gets its own deterministic test and field plot.

---

# 14. Bridge B-side input contract

## 14.1 Mode A subscription

Mode A:

```text
subscribe: /rpp/velocity_ned
type:      geometry_msgs/Vector3Stamped
```

Current validation and speed clamp stay unchanged.

---

## 14.2 Mode B subscription

Mode B:

```text
subscribe: /rpp/command
type:      rpp_interfaces/RppCommand
```

Required validation before accepting a normal command:

```text
North finite
East finite
yaw_valid == true
yaw finite
message semantically fresh under existing timeout model
speed within absolute policy after current clamp/validation behavior
```

If any required B control field is invalid:

```text
do not cache as a normal command
do not reconstruct yaw
do not route through Mode A
publish / remain in hard safety stop
```

---

## 14.3 B-side horizontal speed clamp

If the current bridge speed clamp rescales N/E velocity magnitude, retain it.

Example:

```text
input magnitude = 1.10 m/s
max             = 1.00 m/s
```

The bridge may rescale N/E under the current policy.

It must **not** alter the explicit yaw simply because speed was clamped.

Heading authority and longitudinal speed authority are separate under the new contract.

---

# 15. Bridge MAVROS output contract

## 15.1 Mode A mask

Retain:

```text
3527
```

Equivalent:

```python
TYPE_MASK_VELOCITY_ONLY = (
    PositionTarget.IGNORE_PX
    | PositionTarget.IGNORE_PY
    | PositionTarget.IGNORE_PZ
    | PositionTarget.IGNORE_AFX
    | PositionTarget.IGNORE_AFY
    | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW
    | PositionTarget.IGNORE_YAW_RATE
)
```

---

## 15.2 Mode B normal-control mask

Add:

**PROPOSED — NOT APPLIED**

```python
TYPE_MASK_VELOCITY_YAW = (
    PositionTarget.IGNORE_PX
    | PositionTarget.IGNORE_PY
    | PositionTarget.IGNORE_PZ
    | PositionTarget.IGNORE_AFX
    | PositionTarget.IGNORE_AFY
    | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW_RATE
)
```

Numeric value:

```text
2503
```

Normal B publication:

```python
message.coordinate_frame = self.FRAME_LOCAL_NED
message.type_mask = self.TYPE_MASK_VELOCITY_YAW

# ROS/MAVROS message fields are ENU here.
message.velocity.x = float(east)
message.velocity.y = float(north)
message.velocity.z = 0.0

message.yaw = float(yaw_enu_rad)
message.yaw_rate = 0.0
```

---

## 15.3 Hard-safety output mask

Every bridge-level safety inhibit must use:

```text
velocity = 0
yaw ignored
mask = 3527
```

Conceptually:

**PROPOSED — NOT APPLIED**

```python
def _publish_safety_stop(self) -> None:
    message = PositionTarget()
    ...
    message.type_mask = self.TYPE_MASK_VELOCITY_ONLY
    message.velocity.x = 0.0
    message.velocity.y = 0.0
    message.velocity.z = 0.0
    message.yaw = 0.0       # value irrelevant because yaw is ignored
    message.yaw_rate = 0.0
    self.publisher.publish(message)
```

Never reuse a generic B normal-publish helper with a cached yaw for bridge hard safety.

---

# 16. Frame contract

## 16.1 RPP side

```text
yaw_enu_rad:
0°    East
+90°  North
±180° West
-90°  South
```

Velocity fields in `RppCommand` are explicitly named:

```text
velocity_north_mps
velocity_east_mps
```

No ambiguity from generic x/y labels.

---

## 16.2 Bridge side

For `mavros_msgs/PositionTarget`, the ROS-side values are supplied using the MAVROS ENU convention:

```text
message.velocity.x = East
message.velocity.y = North
message.yaw        = ENU yaw
```

Do not manually do:

```text
ENU yaw → NED yaw
```

inside `cmd_vel_bridge`.

MAVROS already owns the transform.

Double-conversion would rotate the command incorrectly.

---

## 16.3 Required cardinal-direction tests

At minimum:

| RPP ENU yaw | Meaning | Expected ROS bridge yaw |
|---:|---|---:|
| `0` | East | `0` |
| `+pi/2` | North | `+pi/2` |
| `pi` or `-pi` | West | normalized equivalent |
| `-pi/2` | South | `-pi/2` |

Also verify matching velocity components for the current RPP convention.

These are ROS-side contract tests.

PX4-side transformed values should be checked separately using the received/logged
`trajectory_setpoint`.

---

# 17. Firmware contract required by the B-side

## 17.1 Required differential-rover logic

The full firmware must resolve speed and heading independently:

```cpp
const float travel_speed = velocity_in_local_frame.norm();
constexpr float stationary_yaw_speed_threshold = 0.01f;

differential_velocity_setpoint.speed =
    travel_speed < stationary_yaw_speed_threshold
        ? 0.f
        : travel_speed;

if (PX4_ISFINITE(trajectory_setpoint.yaw)) {
    differential_velocity_setpoint.bearing =
        matrix::wrap_pi(trajectory_setpoint.yaw);

} else if (travel_speed < stationary_yaw_speed_threshold) {
    differential_velocity_setpoint.bearing = _vehicle_yaw;

} else {
    differential_velocity_setpoint.bearing =
        atan2f(
            velocity_in_local_frame(1),
            velocity_in_local_frame(0)
        );
}
```

Exact implementation details remain in the firmware repo.

The ROS plan depends on the resulting behavior, not on a particular whitespace/layout.

---

## 17.2 Required firmware truth table

| Velocity | Yaw | Firmware heading authority |
|---|---|---|
| moving | finite | **explicit yaw** |
| moving | invalid/NaN | velocity-vector bearing |
| zero/near-zero | finite | **explicit yaw** |
| zero/near-zero | invalid/NaN | current vehicle yaw |

B-side activation requires all four cases to be verified.

---

## 17.3 No firmware tuning in this migration

Do not change in the same A/B interface patch:

```text
RO_YAW_P
RO_YAW_RATE_P
RO_YAW_RATE_I
RO_YAW_RATE_LIM
RD_MAX_THR_YAW_R
RD_TRANS_DRV_TRN
RD_TRANS_TRN_DRV
```

First prove command authority and transport behavior.

Then measure.

Only then tune in a separate controlled phase if needed.

---

# 18. RPP debug / observability

## 18.1 Keep existing debug fields

Current `/rpp/debug` already contains useful controller/tracking evidence.

Do not rename existing public debug fields just to support B.

---

## 18.2 Add explicit-yaw observability

Recommended additive fields:

```text
command_transport_mode
    "velocity_vector"
    "velocity_yaw"

explicit_yaw_cmd_rad
explicit_yaw_cmd_deg
explicit_yaw_valid

command_kind
    "TRACK"
    "APPROACH"
    "PIVOT"
    "HOLD"
    "SAFETY_STOP"

pivot_actuation_mode
    "legacy_carrier"
    "explicit_yaw"
```

These are diagnostics.

They do not belong in the minimal `RppCommand` wire contract.

---

## 18.3 Do not overstate telemetry truth

RPP debug can prove:

```text
what RPP calculated
what RPP attempted to publish
```

Bridge logs can prove:

```text
what bridge accepted
what PositionTarget contract bridge emitted
```

PX4 ULog can prove:

```text
what PX4 received as trajectory_setpoint
what yaw setpoint/controller states were produced
what the vehicle estimated
```

GNSS surveyed-truth analysis is separate.

Do not label RPP cross-track as absolute surveyed marking error.

---

# 19. Startup logging requirements

At startup, both RPP and bridge must emit one unambiguous mode line.

Mode A:

```text
RPP command mode=velocity_vector explicit_yaw=false topic=/rpp/velocity_ned
```

```text
BRIDGE command mode=velocity_vector explicit_yaw=false input=/rpp/velocity_ned yaw_mask=3527
```

Mode B:

```text
RPP command mode=velocity_yaw explicit_yaw=true topic=/rpp/command
```

```text
BRIDGE command mode=velocity_yaw explicit_yaw=true input=/rpp/command normal_mask=2503 safety_mask=3527
```

Do not log every 20/50 Hz command at INFO.

Use debug/telemetry for high-rate evidence.

---

# 20. Failure semantics

## 20.1 B yaw invalid

```text
RppCommand.yaw_valid = false
```

Bridge response:

```text
hard safety stop
mask 3527
no vector-heading fallback
```

---

## 20.2 B yaw NaN with `yaw_valid=true`

Treat as malformed.

Bridge response:

```text
hard safety stop
diagnostic/log
no fallback
```

---

## 20.3 B velocity nonfinite

Treat as malformed.

Bridge response:

```text
hard safety stop
```

---

## 20.4 B command stale

Existing command timeout remains.

Bridge response:

```text
hard safety stop
mask 3527
```

---

## 20.5 RPP/bridge mode mismatch

Because subscriptions are disjoint:

```text
bridge receives no matching command
    ↓
timeout
    ↓
hard safety stop
```

No automatic cross-subscription.

---

## 20.6 Firmware mismatch

If B ROS code is enabled against stationary-only firmware `67b4e5c`:

```text
PIVOT may work
TRACK moving explicit yaw is ignored
```

That is an invalid B test configuration.

Do not field-enable B until full firmware capability is confirmed.

---

# 21. Shutdown requirements

On ROS shutdown, exception, or node teardown:

```text
bridge must attempt a no-yaw hard safety zero
```

not:

```text
zero + cached explicit yaw
```

The existing shutdown behavior should be preserved where possible, with the mask explicitly checked.

RPP shutdown should not invent a final active-yaw command.

---

# 22. Test strategy — interface package

## 22.1 Build test

Acceptance:

```text
rpp_interfaces builds cleanly
RppCommand imports from Python
rpp_controller resolves dependency
jetson_4wd_control resolves dependency
```

---

## 22.2 Message schema test

Verify:

```text
velocity_north_mps
velocity_east_mps
yaw_enu_rad
yaw_valid
```

No accidental yaw-rate field.

No overloaded `Vector3.z`.

---

# 23. Test strategy — RPP

## 23.1 A-side regression

With:

```text
rpp_explicit_yaw_enabled=false
```

verify:

```text
/rpp/velocity_ned remains the active command topic
message type remains Vector3Stamped
vector.x remains North
vector.y remains East
vector.z remains 0
existing speed limiting unchanged
existing carrier-vector pivot unchanged
existing RPP tests pass
no /rpp/command authority is used
```

The A-side is the regression baseline.

---

## 23.2 B normal TRACK test

Given a deterministic guidance solution:

```text
speed = S
bearing = B
```

assert:

```text
north = S * sin(B)
east  = S * cos(B)

RppCommand.velocity_north_mps == final north
RppCommand.velocity_east_mps  == final east
RppCommand.yaw_enu_rad         == exact final B
RppCommand.yaw_valid           == true
```

Do not compare yaw to an independently reconstructed `atan2()` unless the test is specifically
checking geometry consistency.

The primary assertion is that the source-of-truth bearing was propagated.

---

## 23.3 B invalid-guidance test

Inject nonfinite/missing heading input.

Expected:

```text
no normal moving command with yaw_valid=true
fail-closed zero
yaw_valid=false
```

---

## 23.4 B safety-stop test

For each RPP fail-closed branch reasonably testable at node/unit level:

```text
E-stop
mission false
stale odometry
stale active waypoint
invalid tracking result
```

assert zero command and invalid yaw authority.

---

## 23.5 B intentional-HOLD test

When a deliberate HOLD has a finite intended heading:

```text
N = 0
E = 0
yaw = held heading
yaw_valid = true
```

If no finite held heading exists:

```text
yaw_valid = false
```

---

## 23.6 B pivot test

For a known target bearing:

```text
A:
    existing carrier behavior unchanged

B:
    N = 0
    E = 0
    yaw = exact target/path bearing
    yaw_valid = true
```

Explicitly assert:

```text
B pivot does NOT output the legacy 1.0 m/s carrier vector
```

---

# 24. Test strategy — bridge

Create a real behavioral test file; the current bridge test directory is primarily lint coverage.

Suggested:

```text
src/jetson_4wd_control/test/test_cmd_vel_bridge_contract.py
```

## 24.1 A normal command

Expected:

```text
mask = 3527
velocity fields correct
yaw ignored
```

---

## 24.2 B normal moving command

Expected:

```text
mask = 2503
velocity.x = East
velocity.y = North
yaw = exact ENU yaw received
yaw_rate ignored
```

---

## 24.3 B stationary pivot command

Input:

```text
N = 0
E = 0
yaw_valid = true
yaw = target
```

Expected:

```text
mask = 2503
velocity = 0
yaw = target
```

---

## 24.4 Hard safety tests

For every bridge safety gate:

```text
E-stop
mission disabled
heartbeat timeout
PX4 disconnected
PX4 disarmed
wrong mode
RPP timeout
invalid B command
```

expected output:

```text
mask = 3527
velocity = 0
yaw ignored
```

This is a release-blocking test set.

---

## 24.5 Invalid B yaw

Cases:

```text
yaw_valid=false
yaw=NaN
yaw=+Inf
yaw=-Inf
```

Expected:

```text
no normal mask 2503 publication
hard safety output only
no A fallback
```

---

## 24.6 Cardinal-frame tests

Verify East/North/West/South ENU yaw passes untouched into the ROS `PositionTarget.yaw` field.

Do not assert PX4 NED value in this unit test; that transform is MAVROS responsibility.

---

## 24.7 Mode-mismatch test

Where practical in integration test:

```text
RPP A + bridge B
RPP B + bridge A
```

Expected:

```text
no matching normal command
timeout
hard safety stop
```

---

# 25. End-to-end integration test matrix

| Test | RPP mode | Bridge mode | Firmware | Expected |
|---|---|---|---|---|
| A baseline | A | A | old/new compatible | current vector behavior |
| mismatch 1 | A | B | any | fail closed |
| mismatch 2 | B | A | any | fail closed |
| B straight | B | B | **full yaw** | moving explicit yaw reaches PX4 |
| B pivot | B | B | **full yaw** | zero translation + yaw pivot |
| B hard stop | B | B | **full yaw** | zero + yaw ignored |
| B stale command | B | B | **full yaw** | zero + yaw ignored |
| B→A rollback | A | A | full yaw | current vector behavior restored |

Do not interpret a B straight test against stationary-only firmware as valid evidence.

---

# 26. PX4 bench / ULog verification

Before field motion, confirm the complete chain.

## 26.1 B moving command

Inject:

```text
nonzero velocity
finite explicit yaw intentionally known
```

Verify in PX4 evidence:

```text
trajectory_setpoint.velocity received
trajectory_setpoint.yaw finite
DifferentialVelControl resolved heading follows explicit yaw
speed follows velocity magnitude
```

For a decisive bench/SITL test, choose a finite yaw deliberately different from the vector direction.

Example concept:

```text
velocity vector points East
explicit yaw points North
```

Expected with full firmware:

```text
speed authority = |East velocity|
heading authority = North yaw
```

This proves precedence instead of merely observing two numerically equal inputs.

Do not use this deliberately inconsistent command in a moving field run.

---

## 26.2 B pivot command

Inject:

```text
velocity = 0
explicit yaw differs from current yaw
```

Verify:

```text
trajectory_setpoint velocity = 0
trajectory_setpoint yaw = target
rover yaw setpoint follows target
forward translational setpoint remains zero during pivot state
```

---

## 26.3 Hard safety command

Verify that a bridge hard stop yields:

```text
trajectory_setpoint velocity = 0
trajectory_setpoint yaw = NaN / ignored downstream
```

and no new target-yaw rotation is commanded.

---

# 27. Implementation phases

## Phase 0 — baseline lock and firmware gate

Before editing:

```text
source branch = feat/rtcm-direct-gnss-uart
source HEAD   = 1fc7bf328453b46b50354a2f1b011769de53bb44
RPP blob      = af086856082b485bc062c7ac27c95d4afec04236
bridge blob   = 59e0a3158eea9764dd4c723e0d4b5cb3240b9f75
```

Record:

```text
git status
current RPP tests
current bridge tests
colcon build baseline
```

Confirm no unreviewed source change has invalidated the excerpts in this plan.

Firmware gate:

```text
67b4e5c stationary-yaw support alone is not enough for B TRACK.
```

The ROS work can proceed, but B field activation remains blocked until full firmware support is
verified.

**Acceptance:**

```text
baseline tests recorded
tree state understood
A-side current behavior documented
full firmware dependency explicitly tracked
```

---

## Phase 1 — add atomic `RppCommand` interface only

Create:

```text
src/rpp_interfaces/msg/RppCommand.msg
src/rpp_interfaces/CMakeLists.txt
src/rpp_interfaces/package.xml
```

Modify package dependencies only as required for build.

Do not change runtime command routing yet.

**Acceptance:**

```text
interface generates
Python import succeeds
workspace builds
all existing tests pass
runtime remains A-side only
```

---

## Phase 2 — add default-off A/B configuration and transport plumbing

Modify:

```text
rover.launch.py
rpp_controller_node.py
cmd_vel_bridge.py
package.xml files
tests
```

Add:

```text
RPP_EXPLICIT_YAW_ENABLED = False
```

RPP:

```text
A → legacy publisher
B → RppCommand publisher
```

Bridge:

```text
A → legacy subscriber
B → RppCommand subscriber
```

Do not yet remove/replace pivot carrier behavior.

**Acceptance:**

```text
default is false
Mode A output is unchanged
Mode mismatch fails closed
startup logs state selected mode
no subscriber race / no both-topic authority
existing tests pass
```

This is the first deployment-safe milestone.

---

## Phase 3 — thread explicit yaw through all moving RPP control paths

Audit every moving output call site.

For each Mode-B moving command, pass the exact final RPP heading authority.

Add B diagnostics.

Do not reconstruct missing yaw from the velocity vector.

**Acceptance:**

```text
TRACK publishes exact final guidance yaw
APPROACH publishes exact final command yaw
BRAKE while moving retains finite intended yaw
no moving B command can be published with missing/nonfinite yaw
A output remains unchanged
```

No field B activation yet if firmware gate remains open.

---

## Phase 4 — bridge explicit-yaw forwarding and hard-safety split

Add:

```text
TYPE_MASK_VELOCITY_YAW = 2503
```

Retain:

```text
TYPE_MASK_VELOCITY_ONLY = 3527
```

Normal B:

```text
velocity + yaw
mask 2503
```

Hard safety:

```text
zero velocity
yaw ignored
mask 3527
```

**Acceptance:**

```text
normal B yaw passes unchanged in ROS ENU
all safety gates use 3527
invalid B yaw fails closed
no automatic B→A fallback
cardinal tests pass
A mask remains 3527
```

---

## Phase 5 — replace B carrier actuation with zero-translation yaw

Mode A:

```text
unchanged current carrier
```

Mode B:

```text
N=0
E=0
yaw=true target/path bearing
yaw_valid=true
```

Classify zero-command sites into:

```text
hard safety stop
intentional heading hold
```

Preserve existing pivot lifecycle/certificates.

**Acceptance:**

```text
A carrier behavior unchanged
B pivot carrier translational speed = 0
B pivot target is true path/leg yaw, not ±60° carrier
post-pivot HOLD keeps intentional yaw
safety zero does not keep yaw
existing release thresholds unchanged
```

---

## Phase 6 — automated full-contract validation

Run:

```text
RPP tests
bridge tests
interface tests
focused legacy-alignment tests
colcon build
py_compile where applicable
git diff --check
```

Add explicit checks for:

```text
A exact contract
B TRACK
B PIVOT
B HOLD
B safety
mode mismatch
cardinal frame convention
nonfinite handling
no yaw-rate output
```

**Acceptance:**

```text
all focused tests pass
all pre-existing relevant tests pass
build clean
diff check clean
no known silent fallback path
```

---

## Phase 7 — deploy A-side to Jetson first

Deploy new ROS code with:

```text
rpp_explicit_yaw_enabled=false
```

Use the current production firmware or completed full-yaw firmware; either should preserve the A-side
because yaw remains ignored.

Verify:

```text
/rpp/velocity_ned still active
/rpp/command not used for authority
bridge mask 3527
normal straight mission behavior unchanged
carrier pivot still works
E-stop/timeout behavior unchanged
no B-only errors
```

This is the deployment regression gate.

Do not enable B until this passes.

---

## Phase 8 — B-side straight tracking test

Prerequisite:

```text
full moving+stationary explicit-yaw firmware flashed and verified
```

Set:

```text
rpp_explicit_yaw_enabled=true
```

Start with a controlled low-risk straight leg.

Verify end-to-end:

```text
RPP publishes /rpp/command
yaw_valid=true
bridge publishes mask 2503
PositionTarget yaw matches RPP ENU yaw on ROS side
PX4 trajectory_setpoint yaw is finite
PX4 heading setpoint follows explicit yaw
velocity magnitude owns speed
no carrier logic involved in straight tracking
```

Record:

```text
RPP command timestamp
RPP explicit yaw
current yaw
heading error
yaw rate
cross-track error
speed
PX4 trajectory yaw
PX4 resolved yaw setpoint
```

Do not tune during this test.

---

## Phase 9 — B-side pivot and transition test

Use one controlled ~90° pivot geometry.

Verify:

```text
pre-pivot translational stop
pivot command N/E = 0
explicit yaw = true target heading
no carrier velocity
PX4 forward speed setpoint remains zero during pivot
heading converges
yaw-rate settle passes
position/drift certificate passes
TRACK resumes with translation restored
explicit yaw remains continuous through release
```

Deliberately exercise hard safety while stationary in a safe setup:

```text
trigger E-stop / inhibit
    → bridge mask 3527
    → zero velocity
    → no stale target-yaw rotation
```

This is mandatory before full mission B testing.

---

## Phase 10 — repeated A/B mission comparison

Use the same as closely as practical:

```text
mission geometry
waypoint order
speed settings
RPP parameters
PX4 yaw parameters
firmware
GNSS receiver configuration
RTK state requirements
ground surface
payload
battery state
```

Run repeated A and B missions.

Do not compare only one A vs one B run.

Collect enough repeated legs to separate normal field variation from a real controller effect.

---

## Phase 11 — production-default decision

Do not make B default merely because it functions.

B becomes a production-default candidate only after:

```text
transport contract proven
safety contract proven
pivot contract proven
A/B tracking evidence reviewed
terminal accuracy shows no regression
rollback tested
```

Changing the default from:

```text
False → True
```

must be a separate small commit / release decision.

Keep Mode A available after B acceptance unless a later deprecation plan is separately approved.

---

# 28. Field A/B measurement plan

## 28.1 What this migration is allowed to claim

The A/B test is designed to measure whether independent yaw authority changes:

```text
command-to-heading response
heading-error evolution
cross-track convergence / oscillation
pivot translation/drift
pivot-to-track continuity
terminal behavior
```

Do not state improvement before the measurements exist.

---

## 28.2 Straight-leg comparison metrics

For each eligible straight tracking interval:

```text
cross-track RMS
cross-track P95
cross-track peak-to-peak
heading-error RMS
heading-error P95
yaw-rate RMS / oscillation characteristics
speed mean / RMS
command-yaw → measured-yaw response delay
```

Use comparable intervals and exclude explicitly documented pivot/terminal states from straight-cruise
statistics.

---

## 28.3 Pivot metrics

Measure:

```text
initial heading error
pivot duration
peak yaw rate
settled heading error
translation during pivot
position displacement during pivot
post-pivot settle duration
heading discontinuity at release
cross-track behavior immediately after release
```

Expected architectural difference:

```text
A → carrier command creates a synthetic motion vector to request turn authority
B → zero translational command with direct heading authority
```

But the actual drift reduction is a field result, not a guaranteed claim.

---

## 28.4 Terminal metrics

Compare:

```text
distance-to-goal profile
speed profile
terminal stop position
settle behavior
along/cross values at controller level
marking outcome
```

Do not change braking parameters in the same A/B dataset.

---

## 28.5 Absolute accuracy boundary

RPP cross-track and along-track metrics describe controller tracking relative to the path/target in
the estimator/control frame.

They do **not** independently prove surveyed absolute marking truth.

If evaluating true marking accuracy:

```text
surveyed target truth
vs
independent/raw GNSS truth channel as defined by the accuracy program
```

must remain separate from:

```text
RPP path-tracking error
```

Do not add the two errors together blindly without a validated common-frame error model.

---

# 29. Bag / ULog evidence requirements

For each A/B field run, retain enough evidence to reconstruct the authority chain.

## RPP / ROS evidence

Record as available:

```text
/rpp/debug
/rpp/legacy_alignment_debug
/rpp/velocity_ned       # A
/rpp/command            # B
/mavros/local_position/odom
/mavros/state
mission/controller state topics required to segment TRACK/PIVOT/HOLD
```

If `/mavros/setpoint_raw/local` is observable in the current stack, record it; otherwise use bridge
instrumentation and PX4 ULog for the downstream setpoint truth.

---

## PX4 ULog evidence

At minimum inspect topics/messages sufficient to establish:

```text
trajectory_setpoint
rover attitude/yaw setpoint or equivalent logged controller status
vehicle attitude / yaw
angular velocity / yaw rate
rover velocity status
local/global position as needed for segment alignment
actuator/motor evidence when diagnosing pivot behavior
```

Use the actual available ULog topic names from the firmware build; do not fabricate missing topics.

---

# 30. A/B acceptance criteria

The implementation is ready for B-side production evaluation only when all are true.

## Software / architecture

- [ ] `rpp_explicit_yaw_enabled` exists and defaults to `false`.
- [ ] Mode A retains `/rpp/velocity_ned`.
- [ ] Mode A bridge mask remains `3527`.
- [ ] Mode A carrier pivot remains functionally unchanged.
- [ ] B uses one atomic `RppCommand`.
- [ ] B does not use separate unsynchronized velocity/yaw topics.
- [ ] B does not overload `Vector3Stamped.z`.
- [ ] B moving command carries the exact final RPP yaw.
- [ ] B does not reconstruct a missing yaw from N/E velocity.
- [ ] B invalid yaw fails closed.
- [ ] RPP/bridge mismatch fails closed.
- [ ] No automatic B→A fallback exists.
- [ ] No simultaneous A+B authority exists.

## Safety

- [ ] Bridge E-stop uses zero velocity + yaw ignored.
- [ ] Mission-disabled stop uses zero velocity + yaw ignored.
- [ ] Backend-heartbeat timeout uses zero velocity + yaw ignored.
- [ ] RPP timeout uses zero velocity + yaw ignored.
- [ ] PX4 disconnected stop uses zero velocity + yaw ignored.
- [ ] PX4 disarmed stop uses zero velocity + yaw ignored.
- [ ] wrong-mode stop uses zero velocity + yaw ignored.
- [ ] malformed B command uses zero velocity + yaw ignored.
- [ ] shutdown does not publish stale active yaw.
- [ ] safety behavior is tested, not only code-reviewed.

## Frame contract

- [ ] RPP yaw documented as ROS ENU.
- [ ] bridge passes ENU yaw directly to MAVROS ROS message.
- [ ] no manual ENU→NED yaw conversion exists in bridge.
- [ ] East cardinal test passes.
- [ ] North cardinal test passes.
- [ ] West cardinal test passes.
- [ ] South cardinal test passes.

## Pivot / HOLD

- [ ] B pivot translation command is exactly zero.
- [ ] B pivot yaw is true target/path yaw.
- [ ] B pivot does not issue ±60° carrier actuation.
- [ ] intentional HOLD and hard safety STOP use separate semantics.
- [ ] post-pivot transition retains continuous yaw authority.
- [ ] existing heading/yaw-rate/position release certificates remain intact.

## Firmware

- [ ] finite moving yaw has precedence over vector bearing.
- [ ] finite stationary yaw commands pivot/hold.
- [ ] moving invalid yaw falls back to vector bearing.
- [ ] stationary invalid yaw holds current yaw.
- [ ] firmware build/bench verification exists.
- [ ] no new yaw-rate feed-forward added.

## Deployment

- [ ] new ROS stack deployed once in Mode A first.
- [ ] A-side field regression passes.
- [ ] B straight test passes.
- [ ] B pivot test passes.
- [ ] hard-safety no-rotation field/bench check passes.
- [ ] rollback to A has been exercised.
- [ ] repeated A/B mission evidence reviewed.
- [ ] terminal accuracy does not regress.

---

# 31. Rollback contract

Software rollback during field testing:

```text
rpp_explicit_yaw_enabled = false
```

Then restart the relevant rover ROS stack in a safe state.

Expected restored path:

```text
RPP
    → /rpp/velocity_ned
    → Vector3Stamped N/E
    → cmd_vel_bridge
    → PositionTarget mask 3527
    → yaw ignored
    → MAVROS
    → PX4
    → velocity-vector bearing fallback
```

Expected pivot:

```text
current carrier-vector mechanism
```

With the completed full-yaw firmware, no firmware rollback should be required for A-side operation
because `IGNORE_YAW` produces invalid/NaN yaw and the firmware retains vector-bearing fallback.

A-side rollback must not depend on deleting the B code.

---

# 32. Recommended commit sequence

Keep commits independently reviewable.

```text
Commit 1
docs: add RPP explicit-yaw A/B production plan

Commit 2
rpp_interfaces: add atomic velocity-yaw command

Commit 3
rpp: add default-off explicit-yaw command transport

Commit 4
bridge: forward explicit yaw with hard-safety yaw suppression

Commit 5
rpp: use zero-translation explicit yaw for B-side pivot and holds

Commit 6
tests: cover RPP explicit-yaw A/B authority and rollback
```

If package/launch plumbing naturally belongs with Commit 2 or 3, keep the diff coherent rather than
forcing an artificial split.

Do not change the default to B in these commits.

The eventual:

```text
rpp_explicit_yaw_enabled = true
```

production-default change is a separate post-field-acceptance decision.

---

# 33. Reviewer checklist

The reviewer should specifically challenge these questions:

1. Can both `/rpp/velocity_ned` and `/rpp/command` control the bridge in one startup mode?
2. Can a B command ever silently fall back to vector-derived heading inside ROS?
3. Does `rpp_explicit_yaw_enabled=false` preserve the current production command bytes/fields and pivot
   behavior as closely as practical?
4. Does every moving B command carry the exact final RPP command bearing?
5. Is any B yaw reconstructed with `atan2(N,E)` merely because a call site omitted yaw?
6. Can E-stop cause a target-yaw pivot?
7. Can backend heartbeat timeout cause a target-yaw pivot?
8. Can RPP timeout cause a target-yaw pivot?
9. Can shutdown publish zero velocity with stale valid yaw?
10. Is `yaw_valid=false` handled as a hard safety condition by the bridge?
11. Does a nonfinite yaw get rejected even if `yaw_valid=true`?
12. Does bridge speed clamping leave explicit yaw unchanged?
13. Is yaw passed to MAVROS in ROS ENU convention without a second frame conversion?
14. Are cardinal-direction tests present?
15. Does B pivot output exactly zero translation?
16. Is the real pivot target the fixed/path bearing rather than the current carrier bearing?
17. Does A still use the old carrier when selected?
18. Are intentional HOLD and hard safety STOP separate?
19. Is mode selection restart-only?
20. Does mode mismatch fail closed?
21. Is full moving-yaw firmware support verified before B field activation?
22. Are yaw gains/lookahead/speeds unchanged in this interface migration?
23. Is yaw-rate feed-forward still absent?
24. Are old RPP tests still passing?
25. Are real bridge behavioral tests added beyond lint?
26. Can ULog prove the finite explicit yaw reached `trajectory_setpoint`?
27. Can field evidence distinguish RPP command latency from PX4/vehicle response latency?
28. Is terminal accuracy compared without conflating RPP tracking error with surveyed truth?
29. Has A rollback been exercised after B?
30. Is any proposal in the plan being claimed as already implemented without evidence?

---

# 34. Explicitly out of scope

Do not combine any of the following into the first RPP explicit-yaw A/B patch:

```text
yaw-rate feed-forward
RO_YAW_* tuning
RD_* tuning
precision lookahead tuning
speed increase/decrease
braking retune
new path geometry
fixed-leg geometry redesign
terminal tolerance change
GNSS/EKF tuning
RTCM injection changes
Septentrio configuration changes
DDS migration
MAVROS removal
Ethernet transport migration
frontend controls
mission endpoint changes
accuracy formula redesign
automatic A/B switching
automatic B→A fallback
removal of legacy A-side
```

Each can be evaluated later as its own hypothesis.

---

# 35. Production implementation rule

The migration order is:

```text
1. Lock verified baseline
2. Add atomic interface
3. Add A/B plumbing with default A
4. Thread exact RPP yaw
5. Add bridge yaw + safety split
6. Replace only B pivot actuation
7. Run automated regression/safety tests
8. Deploy A and prove no regression
9. Complete + flash full explicit-yaw PX4 firmware
10. Bench-prove firmware precedence
11. Enable B for straight test
12. Enable B for pivot/transition test
13. Run repeated A/B missions
14. Review evidence
15. Decide separately whether B becomes default
```

At no point does the implementation require deleting the legacy route.

---

# 36. Final target contract

After the B-side is fully implemented and the matching firmware is present:

```text
STRAIGHT / TRACK
----------------
RPP:
    final speed
    final N/E velocity
    final explicit ENU yaw

Bridge:
    PositionTarget velocity + yaw
    type_mask = 2503
    yaw_rate ignored

PX4:
    speed   <- norm(velocity)
    heading <- explicit finite yaw


PIVOT
-----
RPP:
    N = 0
    E = 0
    yaw = true target/path yaw
    yaw_valid = true

Bridge:
    zero velocity + yaw
    type_mask = 2503

PX4:
    translational speed = 0
    heading authority = explicit yaw


INTENTIONAL HOLD
----------------
RPP:
    zero velocity
    finite held yaw
    yaw_valid = true

Bridge:
    zero velocity + yaw
    type_mask = 2503


HARD SAFETY
-----------
RPP or bridge safety authority:
    zero velocity
    no active yaw authority

Bridge:
    type_mask = 3527
    IGNORE_YAW

PX4:
    no new target-yaw command


BACKWARD-COMPATIBLE A-SIDE
--------------------------
RPP:
    /rpp/velocity_ned

Bridge:
    type_mask = 3527

PX4:
    yaw invalid / ignored
    moving → derive bearing from velocity vector
    zero   → hold current yaw
```

This is the production contract the implementation must prove.

---

# 37. Stop condition before writing each phase

Before applying each implementation phase:

```text
review current HEAD
confirm prior phase committed/clean
confirm tests from prior phase
inspect exact target snippets
prepare one reviewable patch
apply
run focused tests
run regression tests
git diff --check
review resulting diff
commit only after pass
```

If current source has moved since the verified baseline in a way that touches the planned files:

```text
STOP
re-read the changed snippets
update this plan or the phase patch
do not apply stale exact-block edits
```

This document defines the contract and review gates.

It does not override newer verified source.
