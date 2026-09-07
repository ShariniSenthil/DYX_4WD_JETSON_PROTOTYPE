# PX4 NSH over Jetson SSH — verified 2026-09-07

This is the method that completed the controlled ground drivetrain test in
`/fs/microsd/log/2026-09-07/07_34_22.ulg`. SSH connects to the **Jetson Linux
shell**; Python on the Jetson transports NSH commands through the **existing
MAVROS connection** to the Pixhawk. `param`, `listener`, `ver`, and `logger`
below are **PX4 NSH commands**, not Jetson shell executables.

This record documents execution and transport, not drivetrain analysis or
tuning. It is specific to the verified firmware and live ROS graph; inspect
them again before a future test. Reading NSH does not require arming, MANUAL
mode, or changing `COM_RC_IN_MODE`.

## Connection and exact transport options

```text
Mac → SSH flash@192.168.3.101 → Jetson Python / ROS2 Humble
    → /uas1/mavlink_sink (mavros_msgs/msg/Mavlink)
    → MAVROS router → existing /dev/ttyACM0:921600 connection
    → MAVLink SERIAL_CONTROL → PX4 NSH
    ← /uas1/mavlink_source (mavros_msgs/msg/Mavlink)
```

MAVROS already owns `/dev/ttyACM0`. Do not open a second serial connection,
stop MAVROS, run the broken `scripts/start_rover.sh`, or assume an NSH binary
exists on Linux to perform this procedure.

| Item | Exact value used |
|---|---|
| SSH user / host | `flash@192.168.3.101` |
| SSH options for the running test | `-tt -o BatchMode=yes -o ServerAliveInterval=2 -o ServerAliveCountMax=3` |
| Local command runner | PTY enabled (`tty=true`); SSH remained foreground |
| ROS environment | `source /opt/ros/humble/setup.bash` |
| MAVROS package | `2.14.0-1jammy.20260608.191037` |
| PX4 | Holybro Pixhawk 6X / `PX4_FMU_V6X`, release `1.16.2` |
| PX4 hash, confirmed by NSH `ver all` | `54f0455ffcd755534539a7cf33a09a20bf71d29d` |
| Wire message | `SERIAL_CONTROL`, MAVLink message ID `126`, MAVLink 2 |
| Shell device | `SERIAL_CONTROL_DEV_SHELL = 10` |
| Request flags | `SERIAL_CONTROL_FLAG_RESPOND = 2` |
| Message timeout / baudrate fields | `0` / `0`; these do not replace MAVROS's FCU connection baud |
| Python encoder source | `srcSystem=1`, `srcComponent=191` |
| Payload | UTF-8 command, newline terminated; chunks of at most 70 bytes, zero padded to 70 |
| Outbound topic | `/uas1/mavlink_sink` — publish here |
| Inbound topic | `/uas1/mavlink_source` — subscribe here |
| Successful harness outbound QoS | Depth `10`, default reliable / volatile |
| Successful harness inbound QoS | Depth `1000`, best effort / volatile |
| Packet helpers | `mavros.mavlink.convert_to_rosmsg`, `convert_to_bytes`; pymavlink common v2 dialect |
| Shell completion | Wait for the command echo **and a later** `nsh>` prompt |
| Interrupt a running `listener` | Send `\x03` through SERIAL_CONTROL, retain RESPOND |
| Release the shell at the end | Empty SERIAL_CONTROL payload with `flags=0` |

Inspect the actual graph before using these names:

```bash
ssh -o BatchMode=yes -o ConnectTimeout=8 flash@192.168.3.101
source /opt/ros/humble/setup.bash
ros2 topic info /uas1/mavlink_sink -v
ros2 topic info /uas1/mavlink_source -v
ros2 node info /mavros/manual_control
ros2 interface show mavros_msgs/msg/ManualControl
```

The router subscribes to `mavlink_sink` and publishes `mavlink_source`.
Both router endpoints offered best-effort QoS. The larger receive queue in
the successful harness avoids treating a small default sensor-data queue
as suitable for bursts of NSH text mixed with all other FCU telemetry.

## Python dependencies and reusable NSH example

pymavlink was initially absent on the Jetson. It was installed **only into a
temporary directory**, without replacing ROS or system packages:

```bash
# On the Jetson:
python3 -m pip install --target /tmp/ground_symmetry_deps pymavlink
```

The resolved versions were `pymavlink==2.4.49`, `fastcrc==0.3.6`, and
`lxml==6.1.3`. `/tmp` contents are temporary and may disappear after reboot.
Set `MAVLINK20` before importing pymavlink or `mavros.mavlink`, and add the
temporary directory to `sys.path`. **Do not replace `PYTHONPATH` with just
that directory**: doing so removed the sourced ROS Python paths in this
session and produced `ModuleNotFoundError: No module named 'mavros'`.

The following is the transport core from the successful harness. It assumes
`rclpy.init()` and a running executor for `node`; it does not arm or publish
motor commands:

```python
import os
import sys

os.environ["MAVLINK20"] = "1"
sys.path.insert(0, "/tmp/ground_symmetry_deps")

from pymavlink.dialects.v20 import common as mav
from mavros.mavlink import convert_to_rosmsg, convert_to_bytes
from mavros_msgs.msg import Mavlink
from rclpy.qos import QoSProfile, ReliabilityPolicy

encoder = mav.MAVLink(None, srcSystem=1, srcComponent=191)
decoder = mav.MAVLink(None)
publisher = node.create_publisher(Mavlink, "/uas1/mavlink_sink", 10)

def receive(message):
    if message.msgid != 126:
        return
    for packet in decoder.parse_buffer(convert_to_bytes(message)) or []:
        if packet.get_type() == "SERIAL_CONTROL":
            text = bytes(packet.data[:packet.count]).decode(errors="replace")
            print(text, end="", flush=True)

subscription = node.create_subscription(
    Mavlink, "/uas1/mavlink_source", receive,
    QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT),
)

def send_shell(text, flags=mav.SERIAL_CONTROL_FLAG_RESPOND):
    raw = text.encode()
    chunks = [raw[i:i + 70] for i in range(0, len(raw), 70)] or [b""]
    for chunk in chunks:
        packet = encoder.serial_control_encode(
            mav.SERIAL_CONTROL_DEV_SHELL, flags, 0, 0, len(chunk),
            list(chunk) + [0] * (70 - len(chunk)),
        )
        packet.pack(encoder)
        encoder.seq = (encoder.seq + 1) % 256
        publisher.publish(convert_to_rosmsg(packet))

# Wait for ROS discovery, send one command, then wait for its echoed command
# and subsequent nsh> prompt before sending the next command.
send_shell("ver all\n")
# After all commands/listeners finish:
# send_shell("", flags=0)
```

The initial read-only probe is archived as
[symmetry_px4_shell_probe.py.txt](field-tests/2026-09-07_drivetrain_symmetry/symmetry_px4_shell_probe.py.txt).
It sent `ver all` and two setpoint samples, then closed the shell. Its
default sensor-data receive queue was smaller than the successful harness's
queue; use the successful harness settings above for sustained monitoring.

### Useful NSH commands used in this session

```text
ver all
param show COM_RC_IN_MODE
logger status
listener vehicle_status -n 1
listener manual_control_setpoint -n 2 -r 5
listener manual_control_setpoint -n 100000 -r 5
```

`-n` is the sample count; `-r` is the listener's observation rate in Hz.
The successful test **published commands at 50 Hz and observed NSH at 5 Hz**.
Do not confuse these rates. Interrupt the long-running listener with
SERIAL_CONTROL `\x03` before sending another shell command.

NSH output arrives in fragments and includes ANSI escape sequences. Assemble
complete topic records before parsing fields. The successful parser used the
`TOPIC: manual_control_setpoint` marker and the final `sticks_moving` line;
this is a firmware-specific text layout, not a stable typed telemetry API.
Do not interpret a previously buffered `nsh>` prompt as completion of a new
command. `param show` includes indices, e.g.:

```text
x + COM_RC_IN_MODE [179,509] : 0
```

`+` means saved and `*` means unsaved. Match the parameter name and numeric
value while allowing the bracketed indices between its name and colon.

## Verified MANUAL_CONTROL mapping

This is separate from NSH transport: motion commands use the existing
`/mavros/manual_control/send` plugin, **not raw SERIAL_CONTROL**.

| Field / setting | Value and meaning |
|---|---|
| ROS message | `mavros_msgs/msg/ManualControl` |
| MAVLink message | `MANUAL_CONTROL` |
| Publish rate | 50 Hz (`create_timer(0.02, ...)`) |
| Publisher QoS | Depth 1, reliable / volatile |
| `x` | `0` |
| `y` | Steering: `0`, `-400/+400`, `-550/+550`, `-700/+700` |
| `z` | **`500` for zero forward/reverse throttle** |
| `r` | `0`; this firmware's full-MANUAL rover steering consumes roll, not yaw |
| Buttons, extensions, auxiliaries | All zero |
| Required temporary input selection | `COM_RC_IN_MODE=1` |
| Normal restored selection | `COM_RC_IN_MODE=0` (RC only) |

MAVROS 2.14.0's **send** callback copies the ROS fields directly into MAVLink;
it does not multiply normalized floats by 1000. Thus `y=-400` requests
`roll=-0.4`. PX4 converts `z` using `throttle = z / 500 - 1`, so **`z=0`
requests full reverse**, not neutral. Its receiver maps `y / 1000` to roll,
and `RoverDifferential::generateSteeringAndThrottleSetpoint()` uses that roll
as normalized differential steering. At zero throttle the mixer computes:

```text
Motor1 = -steering
Motor2 = +steering
LEFT  y < 0 → Motor1 positive, Motor2 negative
RIGHT y > 0 → Motor1 negative, Motor2 positive
```

Live PWM functions were `101, 102, 101, 102` on MAIN outputs 1–4. The neutral
check in this run expected 1500 µs on those four outputs; re-check actual
mapping/calibration before reusing that assumption on different hardware.

`/mavros/actuator_control` exists but transmits `SET_ACTUATOR_CONTROL_TARGET`,
which the receiver at this firmware hash does not handle. It was not used.
**Do not use `scripts/acro_yaw_manual_control_test.py` for this procedure**:
that existing script commands `r`, sets `z=0`, and runs at 20 Hz; it does not
implement the tested mapping or sequence. It was not modified in this task.

Source references checked at the exact versions:

- [MAVROS 2.14.0 manual-control plugin](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/manual_control.cpp#L82)
- [PX4 MANUAL_CONTROL receiver](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/mavlink/mavlink_receiver.cpp#L2083)
- [PX4 input-source selection and timeout](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/manual_control/ManualControlSelector.cpp#L66)
- [PX4 rover manual mapping and mixer](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/rover_differential/RoverDifferential.cpp#L96)

The local source checkout used for exact-commit inspection was
`/Users/dyx_a1/Vetri/Way_to_Mark/PX4-Autopilot-4WD-Prod-Baseline`.
Use `git show 54f0455ffc:<path>` there: its working tree contained newer
paths/classes that did not exist at the live firmware's commit.

## Successful ground-test order

This is a motion procedure, not a side effect of using NSH. Use only within
an operator-authorized test, with the rover on the ground and a physical
E-stop available. The successful run used manual arming in QGroundControl;
the script never sent an arm-true request. No production nodes were edited
or restarted, and no firmware, mixer, EKF, or other parameter was changed.

1. Confirm connected and **disarmed** using `/mavros/state`; if necessary,
   request `/mavros/cmd/arming` (`mavros_msgs/srv/CommandBool`, `value=false`)
   and verify the subsequent state. Read any bulk parameters before streaming.
2. Through **PX4 NSH**, issue in order:

   ```text
   param set COM_RC_IN_MODE 1
   param save
   param show COM_RC_IN_MODE
   ```

   Confirm the live value and saved marker. Only this parameter is authorized
   to change for this test.
3. Start neutral MANUAL_CONTROL at 50 Hz: `x=0, y=0, z=500, r=0`. Check there
   is exactly one publisher on `/mavros/manual_control/send`.
4. Start `listener manual_control_setpoint -n 100000 -r 5` through NSH.
   Verify repeated `valid: True`, advancing `timestamp`/`timestamp_sample`,
   MAVLink `data_source`, and `roll: 0`, `throttle: 0`. Successful preflight
   observed source `3`, print age 0.001562 s, and sample lag 6 µs. Source `3`
   is this session's MAVLink instance, not a universal constant. Before the
   clean startup, NSH had shown an old **invalid** RC-sourced setpoint;
   topic existence alone did not prove input health.
5. While **still disarmed**, request `/mavros/set_mode`
   (`mavros_msgs/srv/SetMode`, `custom_mode="MANUAL"`). Confirm MANUAL remains
   stable for at least 3 seconds with valid neutral input and no reported
   failsafe. `AUTO.LOITER` is HOLD and does not satisfy this gate.
6. Only now have the operator arm manually. Leave the transmitter arm switch
   in DISARM while arming through QGroundControl. Keep neutral streaming.
7. Confirm PX4's SD logger start announcement and its full `.ulg` path from
   `/mavros/statustext/recv`. Then run the 3-second armed-neutral step and
   the steering sequence below. The brief wait for the logger announcement
   is additional neutral before the timed sequence.
8. After the final 5-second neutral step, continue neutral through cleanup;
   the successful harness waited another 3 seconds, interrupted the NSH
   listener, and requested disarm. Verify disarmed and `logger status`
   reports `Not logging`.
9. Confirm the transmitter arm switch reads DISARM before changing input
   authority back. Then run:

   ```text
   param set COM_RC_IN_MODE 0
   param save
   param show COM_RC_IN_MODE
   ```

   Verify saved `0` and re-check that PX4 remains disarmed before closing
   the shell and ending the neutral publisher.

**Do not start from armed HOLD and change input authority.** If MANUAL is
lost, stop the attempt; do not force MANUAL back while armed to continue it.

### Exact timed sequence

All rows keep `x=0, z=500, r=0`; only `y` changes.

| Step | `y` | Duration |
|---|---:|---:|
| neutral after arming | 0 | 3 s |
| LEFT | -400 | 3 s |
| neutral | 0 | 3 s |
| RIGHT | +400 | 3 s |
| neutral | 0 | 3 s |
| LEFT | -550 | 3 s |
| neutral | 0 | 3 s |
| RIGHT | +550 | 3 s |
| neutral | 0 | 3 s |
| LEFT | -700 | 3 s |
| neutral | 0 | 3 s |
| RIGHT | +700 | 3 s |
| neutral | 0 | 5 s |

Total scheduled sequence: 41 seconds. Actual step durations from the harness
were within 3 ms of their targets. Neutral was also streamed before arming,
while waiting for the logger, and throughout cleanup. No command exceeded
`abs(y)=700` / normalized steering 0.70.

## Abort handling and exact observation limits

The successful harness used a background `SingleThreadedExecutor` and a
20 ms timer. `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`
kept ROS alive for custom `SIGINT`, `SIGTERM`, and `SIGHUP` handlers. They
latched abort, immediately published neutral, and let the timer keep
publishing neutral through disarm and parameter restoration. Ctrl-C was
implemented this way; **the successful run ended normally**, so do not claim
an in-motion Ctrl-C or physical E-stop fault-injection test was performed.

The actual checks were:

- Preflight accumulated 10 fresh, valid setpoint records; during monitoring,
  invalid input, non-MAVLink source (outside 2–7), non-advancing timestamps,
  print age or sample lag ≥0.20 s, or nonzero PX4 throttle latched abort.
- A complete parsed setpoint record missing for >0.65 s latched abort.
  The parser was specific to this firmware's NSH text; observation was 5 Hz,
  not a hardware-synchronous 50 Hz validity channel.
- After MANUAL was confirmed, leaving MANUAL or heartbeat system status
  `5`/`6` latched abort. During the active test, disconnect, disarm, or state
  telemetry older than 1.5 s also latched abort.
- Received PX4 text containing `failsafe` and `activated`, `triggered`, or
  `enabled` latched abort once the MANUAL guard was enabled. This was a
  heartbeat/STATUSTEXT check; the harness did not continuously subscribe to
  typed PX4 `vehicle_status.failsafe`. The preflight NSH snapshot showed
  `failsafe: False`.
- During armed neutral, stale PWM telemetry (>0.65 s) or MAIN 1–4 outside
  `1500 ± 1 µs` latched abort after a 0.25 s command-transition allowance.
- A publisher scheduling gap >0.10 s during the active test, or an asserted
  `/emergency_stop`, latched abort. A stalled process cannot run its own
  timer/handler until it resumes; process kill/power loss is not covered by
  Python signal cleanup. Retain the physical stop path.

These are **temporary harness thresholds**, not new PX4 parameter values or
a claim of hard real-time guarantees. Live `COM_RC_LOSS_T=0.5`,
`NAV_RCL_ACT=1` (Hold), `COM_FAIL_ACT_T=5`, and `NAV_DLL_ACT=0` were read but
not changed. The active neutral path did not depend on those failsafes.

### RC-switch restoration trap

An earlier aborted attempt restored `COM_RC_IN_MODE=0` and PX4 immediately
reported **“Armed by RC switch”**. A subsequent disarm was sent and verified.
Therefore, disarm and verify the RC arm switch **before** restoring RC input,
then verify disarmed again afterward. The successful harness read
`/mavros/rc/in`, `RC_MAP_ARM_SW`, `RC_ARMSWITCH_TH`, and that channel's
MIN/MAX/TRIM/DZ/REV to check the switch, rather than assuming a raw midpoint
always means DISARM. This session's arm channel was 8 and threshold 0.75.

## Backend E-stop release when the live app has no RELEASE button

The local frontend source contained a header RELEASE button, but the operator
confirmed that the **live DYX app did not expose it**. Do not direct the user
to a button solely because it exists in a local source checkout.

The verified alternative, explicitly authorized by the operator, was:

1. SSH to the Jetson and source ROS plus
   `/home/flash/rover_ws/install/setup.bash`.
2. Authenticate with `POST http://127.0.0.1:5001/api/auth/login`, JSON keys
   `username` and `password`, using the configured backend credentials.
   In this session Python loaded `rover_backend.config.settings` and used
   `settings.static_username` / `settings.static_password` in memory only.
3. Use the returned token as `Authorization: Bearer <token>` for
   `POST /api/estop/release` with `{}`. The backend resolves the current
   safety generation and calls the guarded manager release service.
4. Verify the response and live ROS latch: `emergency_stop=false`,
   `mission_enable=false`. Releasing must not start or resume a mission.
5. End the temporary session with `POST /api/auth/logout` using the same token.

No credentials or tokens are stored in this runbook or archives. Do not
publish `/emergency_stop=false` to bypass the guarded release, and do not
automatically clear a newly asserted stop during a test. A request outcome
timeout requires state verification before any retry. Relevant local code:
[release route](../src/rover_backend/rover_backend/system_routes.py),
[generation-aware bridge](../src/rover_backend/rover_backend/ros_bridge.py).

## Logging and earlier execution problems

- The live `SDLOG_MODE=0` started the SD ULog on arming. `logger status`
  reported `Not logging` before arming and again after the successful
  disarm. The filename was taken from PX4's `[logger]` STATUSTEXT; no FTP
  traffic was needed during the successful motion sequence.
- `MAV_CMD_LOGGING_START` at this hash starts **MAVLink log streaming**;
  do not substitute it for evidence that an SD-card ULog has opened.
- Earlier attempts saw full parameter-pull failures, an FTP list timeout
  (`errno=110`), and a PX4 failsafe that left MANUAL. Their causal relationship
  was **not established**. The successful run used NSH set/show/save and
  logger announcements instead, with bulk parameter reads before streaming.
- A transient-local E-stop subscription missed the live volatile publisher;
  a reliable STATUSTEXT subscriber was incompatible with its best-effort
  publisher. The successful subscribers used compatible best-effort,
  volatile QoS. Check endpoint QoS with `ros2 topic info ... -v`.
- `rclpy.node.Node.clients` is a reserved read-only attribute. The first
  temporary script failed when assigning to it; the working helper used
  `svc_clients`. That attempt never started motion.
- After the successful sequence and disarm, PX4 printed
  **“Preflight Fail: No CPU and RAM load information”**. No failsafe or abort
  occurred during the completed sequence. This post-disarm message remains
  unexplained; no parameter or firmware fix was attempted.

## Archived evidence and exact files

The successful execution was launched from the Mac with:

```bash
ssh -tt -o BatchMode=yes -o ServerAliveInterval=2 -o ServerAliveCountMax=3 \
  flash@192.168.3.101 \
  'source /opt/ros/humble/setup.bash; python3 -u /tmp/ground_symmetry_clean_20260907.py'
```

The files below are exact copies retrieved from the Jetson after completion.
Python is archived as **`.py.txt`**, deliberately not installed or registered
as a ROS node. They are historical field-test evidence, not an automatically
approved motion launcher. The clean harness imports the base helper; the
base file's standalone `main()` belongs to an earlier attempt and **must not
be used as the successful procedure**. Future reuse must re-check the live
graph, firmware, neutral mapping, operator readiness, and the stated guards.

- [Successful clean harness](field-tests/2026-09-07_drivetrain_symmetry/ground_symmetry_clean_20260907.py.txt)
- [Imported base helper](field-tests/2026-09-07_drivetrain_symmetry/ground_symmetry_20260907.py.txt)
- [Initial read-only NSH probe](field-tests/2026-09-07_drivetrain_symmetry/symmetry_px4_shell_probe.py.txt)
- [Successful execution result, including measured step durations](field-tests/2026-09-07_drivetrain_symmetry/result.json)
- [Original paths and SHA-256 checksums](field-tests/2026-09-07_drivetrain_symmetry/manifest.json)

Other temporary Jetson outputs were
`/tmp/ground_symmetry_clean_px4_monitor.txt` and
`/tmp/ground_symmetry_20260907_commands.csv`. They were not a ROS bag or a
downloaded ULog. The full ULog remains on the FCU at
`/fs/microsd/log/2026-09-07/07_34_22.ulg`; no drivetrain analysis was performed.

Successful final state: disarmed in MANUAL; `COM_RC_IN_MODE=0`, with `param
save` completed and `param show` reporting the saved marker. The root
`AGENTS.md` and `CLAUDE.md` were initially Git-ignored; the operator requested
that both be explicitly tracked in the documentation commit. Existing ignore
patterns do not suppress changes to already tracked files. This shared
runbook and evidence directory are included alongside them.
