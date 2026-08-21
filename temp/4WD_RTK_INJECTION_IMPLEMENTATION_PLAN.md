# 4WD RTK Injection Architecture — Detailed Implementation Plan

**Document status:** Detailed design and implementation plan; production release is gated by the Phase 0 deployment checks explicitly identified below  
**System:** 4WD rover, ROS 2 Humble, MAVROS, PX4, external GNSS, FastAPI backend, React Native/Expo frontend  
**Primary objective:** Replace the manually started, unsupervised RTK correction process with a backend-owned, persistent, observable, and recoverable architecture while preserving the proven MAVROS/PX4/GNSS data path.

---

## 1. Executive Summary

The current 4WD RTK correction path can deliver NTRIP correction bytes to MAVROS, but it is not managed as part of the rover application. An operator must launch `start_rtk.sh`, enter the NTRIP password interactively, and keep track of whether the standalone ROS process is still alive. The backend can display limited RTK status, but it cannot configure, start, stop, reconnect, or supervise injection. Frontend controls call routes that are not implemented by the current backend. Multiple manual launches can also create multiple publishers on the same RTCM injection topic.

The target architecture makes the backend the only RTK control authority. One long-lived `RTKManager` will own desired state, active profile, child-process lifecycle, restart policy, single-source enforcement, and correction-stream health. NTRIP profiles and credentials will be stored on the rover, not treated as tablet-owned configuration. When the backend starts, it will load the saved settings and, if autostart is enabled and a valid default profile exists, start correction injection automatically.

The correction worker will parse the TCP byte stream as RTCM3 rather than blindly slicing socket reads. It will locate the `0xD3` preamble, decode the 10-bit payload length, buffer incomplete frames, validate CRC-24Q, discard invalid frames, resynchronize after corruption, and publish only complete validated frames to:

```text
/mavros/gps_rtk/send_rtcm
```

using:

```text
mavros_msgs/msg/RTCM
```

MAVROS remains responsible for MAVLink `GPS_RTCM_DATA` transport and fragmentation. PX4 and the connected GNSS receiver remain responsible for consuming corrections and producing the GNSS solution.

The backend and frontend will report two independent health domains:

1. **Correction-stream health:** whether the worker is running, connected, receiving valid RTCM, publishing frames, reconnecting, stale, or failing.
2. **GNSS solution quality:** whether the receiver reports no fix, 3D fix, RTK float, or RTK fixed, together with satellites and accuracy.

This distinction prevents a misleading single “RTK healthy” indicator. A correction stream may be healthy while the GNSS has not reached RTK fixed, and the GNSS may temporarily retain an RTK solution after the stream becomes stale.

---

## 2. Goals

The implementation must:

- Establish one backend-owned authority for RTK injection.
- Enforce no more than one active correction source and no more than one managed injection child.
- Persist NTRIP profiles, default profile selection, and autostart policy on the rover. Runtime desired mode is intentionally not persisted.
- Treat the NTRIP password as write-only through the API and never return it to the frontend.
- Start RTK automatically when the backend starts, when policy and configuration permit.
- Supervise the correction child and recover from unexpected exits.
- Preserve deliberate operator Stop across supervisor cycles.
- Reconnect after network or caster interruptions with bounded exponential backoff and jitter.
- Parse RTCM3 frames and validate CRC-24Q before publishing.
- Resynchronize correctly after malformed or corrupted stream data.
- Publish complete validated RTCM frames through the existing MAVROS ROS topic.
- Expose useful counters, timestamps, state, and last-error information without exposing credentials.
- Keep correction-stream health separate from GNSS RTK solution quality.
- Replace non-functional frontend control calls with a defined REST contract.
- Remove the operational dependency on `start_rtk.sh` after controlled migration.
- Shut the child down cleanly when the backend exits.

---

## 3. Current 4WD System

### 3.1 Current correction data flow

```text
[ NTRIP caster: TCP port 2101 + HTTP Basic authentication ]
                              |
                              v
[ rtk_correction_bridge / ntrip_to_px4_node ]
  - opens the NTRIP socket
  - reads a TCP byte stream
  - slices received data into chunks of at most 180 bytes
  - does not reconstruct RTCM3 frames
  - does not validate CRC-24Q
                              |
                              | mavros_msgs/msg/RTCM
                              v
/mavros/gps_rtk/send_rtcm
                              |
                              v
[ MAVROS gps_rtk plugin ]
                              |
                              | MAVLink GPS_RTCM_DATA
                              v
[ PX4 flight controller ]
                              |
                              v
[ External GNSS receiver ]
                              |
                              | GNSS solution feedback
                              v
/mavros/gpsstatus/gps1/raw
                              |
                              v
[ rover_backend status state ]
                              |
                              | GET /api/rtk/status
                              v
[ Frontend status UI ]
```

The ROS checkout confirms the correction path into MAVROS. The final PX4-to-GNSS serial routing is parameter- and hardware-dependent and must be verified on the deployed rover during integration testing.

### 3.2 Current launch flow

```text
Rover/Jetson starts
    |
    | no rover-stack or RTK boot service in the audited system
    v
Operator launches rover_bringup manually
    |
    | starts MAVROS, backend, and the remaining rover nodes
    | does not start rtk_correction_bridge
    v
Operator separately runs start_rtk.sh
    |
    | script asks for the NTRIP password interactively
    v
ntrip_to_px4_node starts
```

Backend-start recovery and full Jetson-boot recovery are not the same capability. This plan implements backend-start recovery: once `rover_backend` starts, it owns and restores the configured RTK desired state. Full recovery from Jetson power-on still depends on an outer mechanism starting the rover bringup process; that outer boot service is intentionally outside the initial implementation scope.

### 3.3 Current backend behavior

The backend currently:

- subscribes to correction bridge health and correction-age topics;
- subscribes to GNSS status from `/mavros/gpsstatus/gps1/raw`;
- exposes read-only `GET /api/rtk/status` data;
- does not own the NTRIP child;
- does not persist authoritative rover-side profiles;
- does not start, stop, configure, reconnect, or restart the child;
- does not enforce a single active correction source.

### 3.4 Current frontend behavior

The working frontend responsibility is status display. Configuration and lifecycle controls call backend routes that do not exist in the audited backend. Profiles and passwords are kept in tablet storage, so the tablet is currently treated as a configuration authority even though it cannot complete the control flow.

### 3.5 Current limitations

| Priority | Limitation | Operational effect |
|---|---|---|
| P0 | No single RTK owner or exclusivity guard | Two manual starts can create multiple publishers and conflicting correction injection. |
| P1 | Raw TCP reads are sliced without RTCM3 framing or CRC validation | Corrupt data and arbitrary frame boundaries cannot be diagnosed or rejected at the rover bridge. |
| P1 | RTK is outside bringup and backend lifecycle | RTK does not return automatically after a backend start, process crash, or rover-stack restart. |
| P1 | No child supervisor | An unexpected child exit requires operator intervention. |
| P1 | Frontend control routes do not match backend capabilities | Start, configure, and reconnect actions fail at the HTTP layer. |
| P1 | Stream health and GNSS solution are not modeled cleanly as separate domains | Field diagnosis is ambiguous. |
| P2 | Credentials are tablet-side and the shell requires interactive entry | Autostart is impossible and secret ownership is fragmented. |
| P2 | No frame-level metrics | The system cannot report valid frames, CRC failures, resynchronization, or RTCM message types. |
| P2 | Manual script remains a competing start path | Even after adding APIs, bypassing the manager would remain possible unless the script is retired or blocked. |

---

## 4. Target Architecture

### 4.1 Ownership model

```text
Linux / outer process manager
    |
    +-- owns rover bringup only
            |
            +-- starts rover_backend
                    |
                    +-- owns one RTKManager
                            |
                            +-- owns profile/settings store
                            +-- owns desired RTK state
                            +-- owns one correction child
                            +-- owns restart supervision
                            +-- enforces single source
```

No separate production RTK service will independently own the worker. No frontend action will spawn a ROS process directly. No API handler will create a child outside the manager.

### 4.2 Control plane and data plane

```text
CONTROL PLANE

[ Frontend ]
    |
    | REST: profiles, settings, start, stop, reconnect, status
    v
[ FastAPI RTK routes ]
    |
    v
[ RTKManager: single owner ] <----> [ Rover-side profile/settings store ]
    |
    | supervised child lifecycle
    v
[ NTRIP correction worker ]


DATA PLANE

[ NTRIP caster ]
    |
    | TCP byte stream
    v
[ RTCM3 parser ]
    | 0xD3 framing
    | 10-bit length decode
    | CRC-24Q validation
    | resynchronization
    v
[ Complete valid RTCM3 frame ]
    |
    | mavros_msgs/msg/RTCM
    v
/mavros/gps_rtk/send_rtcm
    |
    v
[ MAVROS ] -- GPS_RTCM_DATA --> [ PX4 ] --> [ GNSS ]
                                              |
                                              | GPS solution
                                              v
                              /mavros/gpsstatus/gps1/raw
                                              |
                                              v
                                    [ Backend telemetry ]
                                              |
                                              v
                                         [ Frontend ]
```

### 4.3 Core invariant

For all supported production start paths:

```text
active_managed_correction_children <= 1
active_correction_sources <= 1
```

All lifecycle mutations must be serialized through `RTKManager`. A start request is idempotent when the requested source/profile already matches the desired and active configuration. An unsupported publisher that ignores the lock cannot be prevented through ROS alone; it is detected and handled as an ownership fault.

---

## 5. Component Responsibilities

### 5.1 `RTKManager`

`RTKManager` is a single backend-lifetime instance created during the FastAPI lifespan startup phase. It is the only component authorized to change correction desired state or create and terminate a correction child.

Responsibilities:

- load profiles and global RTK settings;
- track `desired_mode`, `desired_profile_id`, and runtime state separately;
- validate a profile before starting;
- serialize start, stop, profile switch, reconnect, and shutdown operations;
- wait for the worker to confirm acquisition of the inter-process injection ownership lock before declaring the child active;
- prevent duplicate child creation from simultaneous API requests;
- start the installed correction-worker executable directly, without a shell or `ros2 run` wrapper;
- pass the complete one-time child configuration, including the password, through an inherited anonymous read pipe rather than arguments or environment;
- launch the worker in a dedicated process group and maintain a parent-liveness pipe whose EOF tells the worker to terminate after a hard backend exit;
- monitor child PID, exit code, health heartbeat, and last valid frame age;
- restart unexpected exits when desired mode remains active;
- avoid restarting after an operator stop or backend shutdown;
- apply restart backoff, jitter, a restart budget, and crash-loop state;
- expose immutable status snapshots to API handlers;
- perform graceful termination followed by bounded forced termination if necessary;
- redact secrets from errors and logs.

The manager must use one async lock or equivalent serialized command queue for every mutating operation. API handlers call manager methods; they do not manipulate process state directly.

Production deployment must run one backend process/Uvicorn worker. A separate backend-instance lock prevents a second backend process from creating another manager. Failure to acquire that lock keeps the duplicate backend's RTK control plane unavailable and reports `MANAGER_OWNERSHIP_CONFLICT` rather than creating a child.

### 5.2 NTRIP correction worker

The existing correction node remains a narrow data-plane worker, enhanced to:

- read all per-run configuration from the manager's one-time anonymous configuration pipe;
- connect and authenticate to the selected NTRIP caster;
- distinguish HTTP/NTRIP handshake errors from stream errors;
- maintain the TCP reconnect loop;
- optionally send NMEA GGA when the selected caster requires it;
- feed received bytes into a persistent RTCM3 parser buffer;
- publish only complete CRC-valid frames;
- publish structured health and parser metrics;
- respond cleanly to termination signals;
- acquire and hold the correction-injection lock for its entire publishing lifetime;
- watch the parent-liveness pipe and exit when the backend side closes;
- never persist profiles or implement frontend policy.

Socket reconnection belongs inside the worker because the socket and parser are its data-plane resources. Child-process restart belongs to `RTKManager` because it owns the lifecycle boundary.

Worker health is published at 1 Hz on `/rtk_correction_bridge/status` using a typed `rtk_correction_bridge_msgs/msg/CorrectionStatus` message with reliable, volatile QoS and depth 5. `run_id` is generated by the manager for every child start; the manager ignores status whose `run_id` does not match the current child. Loss of three consecutive heartbeats marks worker health stale even if the PID remains alive. The manager remains `STARTING` until a matching status reports `ownership_lock_acquired=true`.

The configuration pipe uses a 4-byte network-order length followed by UTF-8 JSON, capped at 16 KiB. It contains `schema_version`, `run_id`, non-secret profile fields, password, ROS topic, verified total-frame limit, timeouts, and health topic. The worker must receive the entire payload within 5 seconds, reject unknown schema versions or trailing data, close the FD after parsing, and exit with sanitized `CHILD_CONFIG_INVALID` on short/malformed input. The configuration pipe and parent-liveness pipe are separate descriptors. No per-run field is inherited through environment variables.

`CorrectionStatus` has these frozen v1 fields: `builtin_interfaces/Time observed_at`, `string run_id`, `string profile_id`, `uint8 connection_state`, `bool ownership_lock_acquired`, all parser counters from Section 10 as `uint64`, `float32 last_socket_byte_age_sec`, `float32 last_valid_frame_age_sec`, `float32 last_published_frame_age_sec`, `uint32 reconnect_attempt`, `float32 reconnect_delay_sec`, `string error_code`, and `string error_message`. Unknown/unavailable age values use `-1.0` in ROS and are converted to `null` by the REST adapter. Connection-state enum values are `STARTING=0`, `CONNECTING=1`, `STREAMING=2`, `RECONNECTING=3`, `STALE=4`, and `ERROR=5`.

### 5.3 Profile and settings store

The store is rover-authoritative. It contains:

- schema version;
- profile ID and operator-visible name;
- caster host and port;
- mountpoint;
- username;
- password secret value;
- optional TLS mode and certificate policy;
- optional GGA policy;
- default profile ID;
- autostart enabled/disabled;
- monotonically increasing configuration revision used for concurrent-update protection.

Selected initial storage implementation:

```text
/var/lib/rover_backend/rtk/rtk.sqlite3
```

SQLite is selected so profile metadata, settings, password replacement, and revision changes commit in one transaction. Enable foreign keys, use a supported journal mode for the target filesystem, and call an integrity check during controlled startup validation. The path must be configurable for development and tests. The production directory must be owned by the backend service account with mode `0700`; the database and backup files must be mode `0600`. Backups are created only after a successful integrity check and are rotated as a small bounded set. A failed write, `fsync`, transaction commit, permission check, disk-full condition, or read-only filesystem condition leaves the previous committed generation active and returns `STORE_WRITE_FAILED`.

Profile IDs are server-generated UUIDs and never used as unchecked path fragments. Display names need not be unique. Host, mountpoint, username, and display-name lengths are bounded by the API schema; ports are limited to `1..65535`. The store rejects a schema version newer than the running software and runs explicit forward migrations for older supported versions. Downgrade compatibility is handled by the release rollback runbook and database backup, not by silently rewriting a newer schema.

If the deployed Jetson has no hardware-backed or OS credential service, a root-capable user can still read the password in the protected database. File permissions protect against accidental exposure and unprivileged users; they are not equivalent to hardware-backed encryption. This limitation must be documented honestly.

### 5.4 Backend RTK API

The API validates input, authorizes the operation through the backend's production access-control middleware, invokes `RTKManager`, and returns a status snapshot. It must not directly spawn or kill processes. Production deployment is blocked until mutation routes require authentication and frontend-to-backend credentials are protected in transit. Development may explicitly allow loopback-only unauthenticated access.

### 5.5 Frontend

The frontend becomes an operator console, not a correction processor or credential authority. It:

- creates and edits rover-side profiles through REST;
- treats the password as a write-only field;
- selects the default profile and autostart policy;
- requests start, stop, or reconnect;
- displays correction-stream status independently from GNSS solution quality;
- displays actionable backend errors;
- does not start a second process when opening a screen, polling, reconnecting, or resuming the app;
- does not depend on the tablet remaining open for injection to continue;
- removes or hides unsupported correction-source controls until corresponding backend implementations exist.

### 5.6 MAVROS, PX4, and GNSS

These remain outside backend lifecycle ownership:

- the worker publishes validated frames as `mavros_msgs/msg/RTCM`;
- MAVROS converts and, where required by the deployed plugin, fragments frames into MAVLink `GPS_RTCM_DATA` messages;
- PX4 routes the messages to the configured GNSS interface;
- the GNSS computes its solution;
- PX4/MAVROS return solution information through GPS status topics.

Before field enablement, confirm the maximum `mavros_msgs/msg/RTCM.data` vector accepted by the exact deployed MAVROS version, measured in total RTCM frame bytes including header and CRC. The worker must not silently publish frames above that supported limit. It must count and report them as `publish_rejected_oversize` with the observed total size, without logging correction payload bytes.

---

## 6. Single-Source Enforcement

Single-source protection must work at more than one layer.

### 6.1 In-process serialization

- One `RTKManager` instance is stored in backend application state.
- Every mutation uses one manager lock/command queue.
- `start()` checks desired and actual state before creating a child.
- Concurrent identical starts collapse into one operation.
- A request for a different profile performs a controlled stop-then-start transition.

### 6.2 Inter-process ownership lock

The correction worker acquires an exclusive runtime lock before it creates or activates the RTCM ROS publisher, for example:

```text
/run/lock/rover-rtk-injection.lock
```

The production path uses an OS-held advisory lock whose lifetime follows the worker process, not a PID file. The worker writes its `run_id`, PID, and profile ID into the already-locked file for diagnostics. The lock path and permissions must work for the backend service account.

All supported correction workers must honor this lock. Failure to acquire it must result in a clear `OWNERSHIP_CONFLICT` state and no ROS publishing.

### 6.3 ROS graph safety check

Before starting, and periodically while active, inspect publishers on `/mavros/gps_rtk/send_rtcm`:

- expected publisher count and identity: one managed worker;
- unknown or additional publisher before start: refuse the new start;
- unknown or additional publisher appearing while streaming: set desired state unchanged, stop the managed child, latch `OWNERSHIP_CONFLICT`, and require the unknown publisher to disappear plus an explicit Retry before resuming;
- default safety policy during migration: refuse a new start if an unknown publisher already exists;
- after cutover: treat any additional publisher as a deployment defect.

ROS graph checks improve diagnostics but do not replace the ownership lock because discovery is asynchronous and can race. The expected publisher identity is the current `run_id`-specific worker node name and topic type `mavros_msgs/msg/RTCM`; checks run before start and every second while active.

### 6.4 Legacy bypass removal

After migration, `start_rtk.sh` must no longer remain an independent production start path. The direct-launch script is deleted/disabled in the deployed package; if the filename is retained for operator compatibility, its only allowed behavior is calling the backend API. It must not invoke the worker, `ros2 run`, or any alternate publisher directly.

---

## 7. NTRIP Profile Persistence and Credential Handling

### 7.1 Profile model

```json
{
  "id": "field-base-01",
  "name": "Field Base 01",
  "host": "caster.example.net",
  "port": 2101,
  "mountpoint": "MOUNTPOINT",
  "username": "configured-user",
  "password_configured": true,
  "tls_mode": "REQUIRED",
  "send_gga": false,
  "enabled": true
}
```

The password value is never present in profile read responses.

Initial validation contract:

| Field | Rule |
|---|---|
| `id` | Server-generated UUID; immutable. |
| `name` | 1–64 Unicode characters after trimming. |
| `host` | 1–253 characters; DNS name or IP literal only, with no scheme, path, userinfo, or control characters. |
| `port` | Integer `1..65535`. |
| `mountpoint` | 1–128 characters; normalize one optional leading `/`, reject query/fragment/control characters. |
| `username` | 1–128 characters for the initial authenticated-NTRIP release. |
| `password` | 1–256 bytes when UTF-8 encoded; write-only. |
| `tls_mode` | `REQUIRED` or `DISABLED`. |
| `send_gga` | Boolean; default `false`. Enabling it is blocked until the rover GGA source/interval is configured and tested. |
| `enabled` | Boolean. Disabled profiles cannot be default or started. |

### 7.2 Write semantics

- Creating a profile with NTRIP Basic authentication requires a password.
- Updating a profile without a password preserves the existing secret.
- Updating with a non-empty password replaces the secret atomically.
- Clearing a password requires an explicit action such as `clear_password: true`; an omitted or blank UI field must not accidentally erase it.
- Deleting the default profile is rejected until settings select another default, regardless of runtime state. Deleting the active profile also requires RTK to be stopped.
- Profile IDs are stable and are not derived only from display names.
- Updating or clearing the secret of the active profile returns `409 PROFILE_ACTIVE`; the operator must stop, update, and start explicitly.
- The store uses one global configuration revision. Every profile/settings mutation, including create and delete, requires `If-Match: "<revision>"`; stale writes return `409 REVISION_CONFLICT` with the latest non-secret revision. Request bodies do not duplicate the revision.

### 7.3 Secret handling rules

- Never include the password in `GET` responses.
- Never include it in process arguments, exception messages, status objects, telemetry, analytics, or logs.
- Deliver the one-time child configuration through an inherited anonymous pipe. The worker reads it once before connection, closes the descriptor, and performs best-effort in-memory cleanup. The secret is never placed in arguments or environment.
- Redact userinfo from URLs before logging.
- Do not log the HTTP `Authorization` header.
- Frontend local storage may cache non-secret profile metadata only.
- `tls_mode` is either `REQUIRED` or explicit `DISABLED`; there is no silent “preferred” downgrade. `REQUIRED` validates certificate chain and hostname using the configured system/custom CA bundle. If the caster supports only plaintext TCP, `DISABLED` must be explicitly selected and the UI must warn that Basic credentials and corrections are not protected in transit.
- Avoid returning raw child stdout/stderr to the frontend; map it to sanitized error codes and messages.

### 7.4 Store validation and recovery

At backend startup:

1. Read and validate schema version.
2. Validate server-generated profile IDs and required fields.
3. Verify password presence without reading it into logs.
4. Validate `default_profile_id` refers to an enabled profile.
5. Run database integrity/schema checks. On corruption, move the primary database to a timestamped quarantine path, copy the latest verified backup into place through an atomic rename, rerun integrity checks, and open the restored database. Set `STORE_RECOVERED_FROM_BACKUP`, disable autostart for that backend run, and require an operator to review status and Start explicitly. If no verified backup restores cleanly, expose read-only diagnostic status and `CONFIG_INVALID`; do not create a blank store over recoverable evidence.
6. Disable autostart when no valid default profile or required password is available.
7. Expose the reason through status without exposing secret material.

---

## 8. Backend Startup Autostart

### 8.1 Startup sequence

```text
FastAPI lifespan starts
    |
    v
Create profile/settings store
    |
    v
Load and validate persisted configuration
    |
    v
Create exactly one RTKManager
    |
    v
Start manager monitor/supervisor task
    |
    v
Is autostart enabled?
    | no ------------------------------> remain STOPPED
    |
    | yes
    v
Is a valid default profile available?
    | no ------------------------------> ERROR / CONFIG_INVALID; backend stays alive
    |
    | yes
    v
Set desired_mode=NTRIP and desired_profile_id=<default>
    |
    v
Wait/retry for required ROS/MAVROS readiness within policy
    |
    v
Start supervised child
    |
    v
CONNECTING -> STREAMING when valid RTCM frames are published
```

Autostart failure must not crash the entire backend. The backend remains available so the frontend can display the problem and correct the configuration.

### 8.2 Desired-state persistence policy

Recommended initial policy:

- `autostart_enabled` is an explicit rover setting.
- `default_profile_id` identifies what autostart uses.
- Runtime `desired_mode` and `desired_profile_id` are not persisted.
- A manual Start of profile B does not change default profile A. Changing the next-start default requires an explicit settings update.
- Operator Stop changes runtime desired mode to `DISABLED` for the current backend lifetime.
- Stop never changes persistent autostart policy. Only `PUT /api/rtk/settings` changes whether the next backend start autostarts.

This avoids ambiguity between “stop now” and “do not start next time.”

### 8.3 Shutdown sequence

On backend shutdown:

1. Mark manager as shutting down so no restart is scheduled.
2. Stop accepting lifecycle mutations.
3. Send graceful termination to the child.
4. Wait for a configured timeout.
5. Force termination only if the child does not exit.
6. Await and cancel supervisor tasks.
7. Release locks and close resources.

Backend shutdown is not an operator Stop and must not silently rewrite persistent autostart settings.

---

## 9. Child Supervision and Restart Policy

### 9.1 Responsibilities by failure boundary

| Failure | Recovery owner |
|---|---|
| Socket timeout, caster disconnect, transient DNS/network failure | NTRIP worker reconnect loop |
| RTCM stream stalls while socket remains open | Worker detects staleness and reconnects; manager observes health |
| Worker process exits or crashes | `RTKManager` restarts child when desired mode remains active |
| Backend exits | Outer rover bringup/process manager restarts backend if configured |
| Rover/Jetson reboots | Outer boot service, when later added, starts rover bringup |

### 9.2 Restart algorithm

- Record exit timestamp, exit code, signal, runtime duration, and a sanitized last error.
- If desired mode is disabled or shutdown is in progress, do not restart.
- Otherwise schedule restart with exponential backoff and jitter.
- Reset backoff only after a stable streaming period, not immediately after process creation.
- Maintain a sliding-window restart budget, for example a configurable maximum within a configurable number of minutes.
- When the budget is exhausted, enter `CRASH_LOOP`, stop automatic process restarts, retain desired mode, and require operator retry or a cooldown policy.
- `POST /api/rtk/reconnect` resets the socket/child attempt according to current state; an explicit `retry_crash_loop` flag or dedicated action is required to clear a crash loop.

Selected initial production defaults, all centrally configurable:

| Setting | Default |
|---|---:|
| NTRIP TCP connect timeout | 10 s |
| NTRIP handshake timeout | 10 s |
| Socket-byte stale warning | 5 s |
| Valid-frame stale/reconnect threshold | 15 s |
| Partial-frame candidate timeout | 2 s |
| Parser residual-buffer cap | 64 KiB |
| Worker reconnect delay | 1 s base, 30 s cap, ±20% jitter |
| Manager child-restart delay | 1 s base, 60 s cap, ±20% jitter |
| Manager restart budget | 5 unexpected exits per 5 min |
| Stable streaming period that resets restart backoff | 60 s |
| Worker health heartbeat / stale threshold | 1 Hz / 3 s |
| Graceful stop / forced-kill wait | 5 s / 2 s |
| ROS graph readiness/publisher check | every 1 s |

Authentication or mountpoint rejection latches `ERROR` after the first confirmed rejection and is not retried automatically. A profile update or explicit Retry is required. Transient DNS, timeout, EOF, HTTP 5xx, and network failures use worker reconnect backoff. All numeric timings remain central configuration values rather than scattered constants.

### 9.3 Liveness versus readiness

- **Process alive:** child PID exists and has not exited.
- **Connected:** NTRIP handshake succeeded.
- **Receiving:** socket bytes arrived recently.
- **Validated:** a CRC-valid RTCM frame arrived recently.
- **Publishing:** a validated frame was successfully published recently.
- **Ready/streaming:** the system has recent successful validated publication.

A live child is not automatically healthy.

### 9.4 Process containment

The manager launches the installed worker executable directly in a new process group, drains stdout/stderr continuously with redaction and rate limits, and records the actual child handle rather than rediscovering it by process name. Graceful stop sends `SIGTERM` to the process group, waits 5 seconds, then sends `SIGKILL` and waits 2 seconds. A replacement cannot start until the previous group is confirmed gone and the worker-held injection lock is available. The inherited parent-liveness pipe closes automatically if the backend is killed. A dedicated watchdog thread or event-loop FD watcher, independent of DNS, TLS, socket, ROS, and health loops, blocks on that pipe and triggers process-wide shutdown/socket close within 1 second of EOF. Deployment tests must also confirm the outer rover process cgroup does not allow descendants to survive a backend/stack stop.

---

## 10. RTCM3 Parsing and CRC-24Q Validation

### 10.1 Frame format

An RTCM3 frame is handled as:

```text
Byte 0       : preamble 0xD3
Bytes 1..2   : reserved bits + 10-bit payload length
Payload      : 0..1023 bytes
Final 3 bytes: CRC-24Q
```

Length decode:

```text
payload_length = ((header[1] & 0x03) << 8) | header[2]
total_frame_length = 3 + payload_length + 3
```

CRC-24Q parameters:

```text
width: 24 bits
full polynomial representation: 0x1864CFB
effective 24-bit polynomial: 0x864CFB
initial value: 0x000000
reflect input/output: false / false
final XOR: 0x000000
input covered: 3-byte header + payload
received CRC: final three bytes, network byte order
check vector: ASCII "123456789" -> 0xCDE703
```

The implementation masks the running state to 24 bits. Unit tests must include the stated check vector and captured RTCM frames.

### 10.2 NTRIP body boundary

The RTCM parser receives only correction-body bytes after the NTRIP/HTTP response has been decoded. The NTRIP client must:

- accept the confirmed successful response dialects used by the deployed caster, including `HTTP 200` and `ICY 200` when observed;
- reject sourcetable and authentication/error responses as RTCM input;
- cap response headers at 16 KiB;
- preserve body bytes received in the same socket read as the header terminator;
- decode HTTP chunked transfer framing before feeding body bytes to the RTCM parser when the server selects chunked transfer;
- reject redirects in the initial release instead of forwarding credentials to an unapproved host;
- clear the parser buffer whenever a socket session ends, so bytes from two connections can never form one frame.

### 10.3 Incremental parser algorithm

The parser owns a persistent byte buffer across socket reads.

```text
append newly received bytes

while buffer contains data:
    find the next 0xD3 preamble

    if no preamble exists:
        count all buffered bytes as discarded noise
        clear the buffer
        stop parsing until more bytes arrive

    discard and count bytes before the preamble

    if fewer than 3 header bytes exist:
        wait for the next socket read

    if (header[1] & 0xFC) != 0:
        increment invalid-header counter
        discard/count only the candidate 0xD3 byte
        rescan the remaining bytes

    decode the 10-bit payload length
    compute total frame length

    if the complete frame is not buffered:
        wait for the next socket read

    compute CRC-24Q over header + payload

    if CRC matches:
        remove the complete frame from the buffer
        increment valid-frame and valid-byte counters

        if total frame byte length exceeds configured deployed MAVROS input limit:
            increment oversize counter
            do not publish
        else:
            publish exactly one complete frame
            increment published counter only after the local ROS publish call succeeds
    else:
        increment CRC-invalid counter
        discard only the current candidate preamble byte
        rescan the remaining bytes for the next 0xD3
```

Discarding only the failed candidate preamble allows recovery when a new valid frame begins inside bytes that followed a false preamble. Clearing the entire buffer after a CRC failure would lose recoverable frames.

CRC-valid oversize frames increment both the valid and oversize counters but not the published counter. `rtcm_bytes_valid_total` includes the complete frame bytes: header, payload, and CRC. Every invalid header or CRC candidate causes its discarded preamble byte to increment the resynchronization-discard count. A zero-payload frame is framing-valid and may publish if its CRC and downstream size checks pass; it has no decoded RTCM message number.

### 10.4 Parser safety limits

- Protocol payload length cannot exceed 1023 bytes.
- The residual accumulation buffer cap is 64 KiB and therefore exceeds the largest protocol frame of 1029 total bytes.
- After each socket read, parse all complete candidates before applying the residual cap. If the residual still exceeds the cap, repeatedly discard/count bytes before the next preamble. Preserve a suffix beginning at `0xD3` only when it is a 1- or 2-byte possible header, or when its complete header declares a protocol-valid incomplete frame of at most 1029 bytes. Otherwise discard that candidate preamble and continue. This rule never trims a complete valid frame before attempting validation.
- A partial-candidate timer starts with a monotonic timestamp when a candidate `0xD3` first becomes buffer position zero; new bytes do not reset it. An independent 100 ms parser timer services timeouts even when the socket produces no new bytes. At 2 seconds, increment the partial-timeout counter, discard/count that candidate preamble, and rescan buffered bytes. The bound is 2 seconds per false candidate; the test suite measures cumulative recovery with multiple false candidates.
- Socket receive/feed chunks are capped at 16 KiB. Combined with parse-before-trim and the 64 KiB residual cap, this gives a measurable peak parser-buffer bound of less than 80 KiB plus fixed parser overhead.
- Clear residual parser bytes and partial timers at every socket-session boundary.
- Phase 0 must determine the deployed MAVROS maximum accepted `mavros_msgs/msg/RTCM.data` vector length in **total frame bytes**, including RTCM header and CRC. Startup is `CONFIG_INVALID` if this value is absent; it is never guessed from payload length.
- Frames that are protocol-valid but exceed the deployed MAVROS limit are not partially sliced by the worker; they are rejected with a dedicated counter and error reason until a verified transport policy is implemented.

### 10.5 Required counters and timestamps

At minimum:

| Field | Meaning |
|---|---|
| `socket_bytes_received_total` | All correction-stream bytes read from the socket. |
| `rtcm_frames_valid_total` | Complete frames with valid CRC-24Q. |
| `rtcm_bytes_valid_total` | Header + payload + CRC bytes in CRC-valid complete frames, including oversize frames. |
| `rtcm_frames_crc_invalid_total` | Complete candidates rejected for CRC mismatch. |
| `rtcm_headers_invalid_total` | Candidates rejected for invalid header/reserved bits. |
| `rtcm_resync_bytes_discarded_total` | Noise/corrupt bytes discarded while locating a valid frame. |
| `rtcm_partial_frame_timeouts_total` | Buffered partial frames that exceeded timeout. |
| `rtcm_frames_oversize_total` | Protocol-valid frames above the verified downstream limit. |
| `rtcm_frames_published_total` | Supported-size frames for which the local ROS publish call returned successfully. |
| `rtcm_publish_errors_total` | ROS publication failures. |
| `last_socket_byte_at` | Last received socket byte time. |
| `last_valid_frame_at` | Last CRC-valid frame time. |
| `last_published_frame_at` | Last successful ROS publish time. |

Optionally decode the RTCM message number from the first 12 payload bits only when payload length is at least two bytes, and expose a bounded per-type counter. Unknown or unavailable message types are still published when the frame is valid.

All counters above are per worker `run_id` and reset to zero when a new child starts. The manager separately reports restart count since the current backend started. Counter reset semantics are part of the API schema.

### 10.6 Publication contract

For every valid and supported frame:

```text
one RTCM3 frame
    -> one mavros_msgs/msg/RTCM message
    -> /mavros/gps_rtk/send_rtcm
```

The worker must not split frames merely at 180-byte boundaries. MAVROS owns MAVLink `GPS_RTCM_DATA` fragmentation. This boundary must be verified against the exact deployed MAVROS build using total frame vectors below, at, and above 180 bytes, at the confirmed maximum, and at maximum + 1. Release acceptance requires a MAVLink capture/test harness or equivalent instrument that compares fragment flags, sequence, and reassembled bytes exactly; observing only that a ROS publish call returned is insufficient.

---

## 11. Health Model: Corrections versus GNSS Solution

### 11.1 Correction-stream health

This answers: “Is the worker receiving valid corrections and handing them to the ROS topic while the expected MAVROS subscriber is discoverable?”

ROS 2 publication has no application-level acknowledgement from MAVROS. Therefore `STREAMING` means recent CRC-valid frames were accepted by the local ROS publisher while the expected MAVROS subscription was present; it does not by itself prove MAVROS processed the message or the GNSS consumed it. End-to-end evidence comes from MAVLink/PX4 inspection during validation and from the separate GNSS solution domain during operation.

Required fields:

- desired mode and profile;
- manager/runtime state;
- child PID and restart count;
- caster connection state;
- last socket-byte age;
- last valid-frame age;
- last published-frame age;
- valid, invalid, resync, oversize, and publish counters;
- current reconnect/backoff attempt;
- ownership conflict or unknown publisher error;
- sanitized last error and timestamp.

Suggested stream health classification:

| Classification | Meaning |
|---|---|
| `STOPPED` | Desired mode is disabled. |
| `STARTING` | Manager is creating or awaiting the child. |
| `CONNECTING` | Child is alive but NTRIP handshake/stream is not ready. |
| `STREAMING` | Recent valid RTCM was successfully published. |
| `STALE` | Child may be connected, but no valid frame was published within threshold. |
| `RECONNECTING` | Worker is backing off/re-establishing the source. |
| `DEGRADED` | Data flows but CRC, oversize, or publish-error thresholds are exceeded. Ownership conflict is always a latched error, never degraded. |
| `ERROR` | Configuration, authentication, ownership, or unrecoverable runtime error. |
| `CRASH_LOOP` | Process restart budget is exhausted. |

Deterministic health thresholds and clearing rules:

- `STALE`: no supported-size valid publication for 15 seconds; clear on the next supported-size valid publication.
- `DEGRADED_CRC`: CRC-invalid candidates exceed 5% of at least 20 complete candidates in a rolling 60-second window; clear after 60 consecutive seconds below 1%.
- `DEGRADED_PUBLISH`: three local ROS publish errors occur within 30 seconds; clear after 60 seconds with no publish error and at least one successful publication.
- `DEGRADED_OVERSIZE`: any oversize frame occurs; clear after 10 minutes without another oversize frame.
- `RECONNECTING`: current matching worker status reports `RECONNECTING`; clear when it reports `CONNECTING` or a supported-size valid publication moves state to `STREAMING`.
- A matching supported-size valid publication recovers `STARTING`, `CONNECTING`, `STALE`, or `RECONNECTING` to `STREAMING` unless a higher-precedence ownership/error/crash-loop condition is latched.
- If the expected MAVROS subscription disappears for three consecutive one-second graph checks while active, enter `STALE: MAVROS_NOT_READY`. Keep the worker connected, parsing, and publishing locally for diagnostics, but do not report `STREAMING`. When the expected subscription returns, the next supported-size valid local publication restores `STREAMING`. MAVROS absence alone does not restart the NTRIP worker.

### 11.2 GNSS solution quality

This answers: “What navigation solution is the receiver producing?”

Source remains PX4/MAVROS GPS telemetry, including:

- `fix_type` mapped to operator labels;
- satellites visible/used when available;
- horizontal and vertical accuracy from the available message fields;
- GNSS observation timestamp and age;
- RTK baseline/receiver data when confirmed and useful.

Suggested labels:

```text
NO_FIX
2D_FIX
3D_FIX
DGPS
RTK_FLOAT
RTK_FIXED
UNKNOWN
```

The mapping must follow the actual MAVLink/MAVROS message semantics used by the deployed version. Unknown values must remain unknown, not be guessed as fixed.

### 11.3 Diagnostic combinations

| Correction stream | GNSS solution | Interpretation |
|---|---|---|
| `STREAMING` | `RTK_FIXED` | End-to-end system is currently producing the desired solution. |
| `STREAMING` | `3D_FIX` | Corrections arrive, but receiver/configuration/sky/base compatibility may prevent RTK. |
| `STALE` or `RECONNECTING` | `RTK_FIXED` | Receiver may be holding a recent solution; correction recovery is still required. |
| `STREAMING` | telemetry stale | Correction path is healthy, but GNSS feedback path is unavailable or stale. |
| `ERROR` | `3D_FIX` | GNSS works autonomously, but corrections are unavailable. |

No overall green status may be derived solely from child liveness or NTRIP TCP connection.

---

## 12. REST API Surface

All responses use stable machine-readable error codes and sanitized human-readable messages. JSON property names use `snake_case`, enum values use uppercase, timestamps use RFC 3339 UTC, and absent measurements use `null`. Lifecycle mutations synchronously record the new desired state and initiate/serialize the transition, then return `200` with the resulting status snapshot; they do not wait for `STREAMING` and do not use asynchronous operation IDs.

### 12.1 Profile and settings routes

| Method | Route | Purpose |
|---|---|---|
| `GET` | `/api/rtk/profiles` | List non-secret profile metadata. |
| `POST` | `/api/rtk/profiles` | Create a rover-side NTRIP profile; password is write-only. |
| `GET` | `/api/rtk/profiles/{profile_id}` | Read one profile without its password. |
| `PUT` | `/api/rtk/profiles/{profile_id}` | Replace all non-secret editable fields; omitted password alone preserves the existing secret. |
| `PATCH` | `/api/rtk/profiles/{profile_id}` | Update selected fields; omitted password preserves existing secret. |
| `DELETE` | `/api/rtk/profiles/{profile_id}` | Delete a non-active, non-required profile. |
| `GET` | `/api/rtk/settings` | Read default profile, autostart, and non-secret policies. |
| `PUT` | `/api/rtk/settings` | Set default profile and autostart policy. |

### 12.2 Lifecycle and status routes

| Method | Route | Purpose | Required semantics |
|---|---|---|---|
| `GET` | `/api/rtk/status` | Return correction and GNSS domains in one snapshot. | Read-only and safe to poll. |
| `POST` | `/api/rtk/start` | Set desired mode to NTRIP and start/select a profile. | Idempotent for the same profile; serialized for all calls. |
| `POST` | `/api/rtk/stop` | Set current desired mode to disabled and stop the child. | Must prevent supervisor restart. |
| `POST` | `/api/rtk/reconnect` | Reconnect the current desired source. | Must not create a second child. |
| `POST` | `/api/rtk/retry` | Clear recoverable error/crash-loop state and retry desired source. | Explicit operator action. |

Required start body:

```json
{
  "source": "NTRIP",
  "profile_id": "field-base-01"
}
```

Required status shape, with additional counters allowed compatibly:

```json
{
  "rtk_api_version": "1.0",
  "desired": {
    "mode": "NTRIP",
    "profile_id": "field-base-01",
    "autostart_enabled": true
  },
  "correction_stream": {
    "state": "STREAMING",
    "source": "NTRIP",
    "profile_id": "field-base-01",
    "child_running": true,
    "connection_state": "connected",
    "last_valid_frame_age_sec": 0.3,
    "last_published_frame_age_sec": 0.3,
    "valid_frames_total": 18429,
    "crc_invalid_total": 2,
    "oversize_total": 0,
    "restart_count": 1,
    "last_error": null
  },
  "gnss_solution": {
    "status": "RTK_FIXED",
    "fix_type": 6,
    "satellites_visible": 26,
    "horizontal_accuracy_m": 0.014,
    "telemetry_age_sec": 0.1
  }
}
```

The `fix_type` example is illustrative; the implementation must use the deployed telemetry mapping as the source of truth.

### 12.3 HTTP behavior

- `200`: completed idempotent read/mutation.
- `201`: profile created.
- `400`: malformed or inconsistent request.
- `404`: unknown profile.
- `409`: ownership conflict, active-profile deletion, invalid transition, or conflicting operation.
- `422`: profile/settings validation failure.
- `503`: manager unavailable during shutdown or required subsystem unavailable.

Repeated polling must have no lifecycle side effects. A frontend retry of the same start request must not create another child.

Error envelope:

```json
{
  "error": {
    "code": "PROFILE_ACTIVE",
    "message": "Stop RTK before modifying the active profile.",
    "retryable": false,
    "details": {}
  },
  "status": {}
}
```

Lifecycle edge rules:

| Request/current state | Result |
|---|---|
| Start same profile while `STARTING`, `CONNECTING`, `STREAMING`, `STALE`, `RECONNECTING`, or `DEGRADED` | Idempotent `200`; no new child. |
| Start different profile while active | Controlled stop, confirmed exit/lock release, then start requested profile; never overlap. |
| Start while `STOPPING` | `409 OPERATION_IN_PROGRESS`; caller polls and retries. |
| Start while `ERROR` or `CRASH_LOOP` | `409 RETRY_REQUIRED`; use Retry or fix configuration. |
| Stop while stopped | Idempotent `200`. |
| Stop in any active/error/crash-loop state | Set desired disabled, cancel pending restart/backoff, terminate child if present, return status. |
| Reconnect while active | Controlled recycle of the same profile: terminate, await exit and lock release, then start a new `run_id`. |
| Reconnect while stopped, stopping, error, or crash loop | `409 INVALID_TRANSITION`. |
| Retry while `ERROR` or `CRASH_LOOP` with valid configuration and no ownership conflict | Clear latch/budget and start desired/default requested profile. |
| Retry in any other state | `409 INVALID_TRANSITION`. |

Profile-list, individual-profile, and settings **responses** include the global `revision` and `ETag: "<revision>"`. All corresponding create, update, and delete requests send that ETag in `If-Match`; request bodies do not carry a second revision. Successful mutations return the new revision and ETag. Lifecycle commands are serialized in arrival order, and the returned status is authoritative for multiple clients.

Production API controls:

- require authenticated operator role for profile, settings, and lifecycle mutations;
- allow status/profile metadata reads only according to the rover backend's existing read policy;
- protect frontend-to-backend traffic with HTTPS or an equivalently protected rover-local transport before accepting passwords;
- configure CORS to the approved frontend origins and apply CSRF protection when cookie authentication is used;
- rate-limit profile writes and lifecycle mutations without rate-limiting health polling into unavailability;
- disable request-body logging for RTK profile routes in the application server, reverse proxy, telemetry, and crash middleware.

---

## 13. Lifecycle and State Machine

### 13.1 Desired state

```text
DISABLED
NTRIP(profile_id)
```

Future correction sources may extend this model, but only one desired mode can exist at once.

### 13.2 Runtime states

```text
STOPPED
STARTING
CONNECTING
STREAMING
STALE
RECONNECTING
DEGRADED
STOPPING
ERROR
CRASH_LOOP
```

### 13.3 Principal transitions

| From | Event | To | Action |
|---|---|---|---|
| `STOPPED` | Start/autostart with valid profile | `STARTING` | Acquire ownership and create child. |
| `STARTING` | Matching child health confirms ownership lock | `CONNECTING` | Await NTRIP handshake and valid frames. A live PID alone is insufficient. |
| `CONNECTING` | Valid frame published | `STREAMING` | Mark ready and reset stable-runtime timers. |
| `CONNECTING` | Transient source failure | `RECONNECTING` | Worker applies reconnect policy. |
| `STREAMING` | Valid-frame age exceeds threshold | `STALE` | Mark unhealthy and trigger worker reconnect policy. |
| `STREAMING` | Error thresholds exceeded but data continues | `DEGRADED` | Continue with alert and counters. |
| Any active state | Operator Stop | `STOPPING` | Set desired disabled before terminating child. |
| `STOPPING` | Child exited | `STOPPED` | Release ownership. |
| Any active state | Unexpected child exit | `STARTING` | Restart with backoff if budget allows. |
| Any active state | Restart budget exhausted | `CRASH_LOOP` | Stop automatic child restarts. |
| `ERROR`/`CRASH_LOOP` | Explicit retry with valid desired state | `STARTING` | Clear eligible latch and retry. |
| Any state | Backend shutdown | `STOPPING` | Stop without changing persistent autostart policy. |

### 13.4 Transition rules

- Change desired state before sending a stop signal, eliminating the race where an intentional exit is restarted.
- Never report `STREAMING` until a valid frame has been successfully published.
- Profile switching is stop-old, confirm exit/release, then start-new. No overlap is permitted.
- Authentication or mountpoint rejection latches `ERROR` without automatic retry.
- Updating the active profile is rejected with `409 PROFILE_ACTIVE`; it never silently mutates or restarts the child.
- Child spawn failure enters `ERROR: SPAWN_FAILED` and counts toward the manager restart budget only when the failure is transient. Missing executable is non-retryable until deployment is fixed.
- Lock conflict enters latched `ERROR: OWNERSHIP_CONFLICT`.
- MAVROS readiness delay remains `STARTING` with reason `MAVROS_NOT_READY`; backend startup continues and readiness checks repeat every second.
- Missing three worker heartbeats moves an active state to `STALE`; if the PID is alive, the manager terminates/restarts the child after the 15-second valid-frame threshold.
- A forced-kill failure enters `ERROR: CHILD_TERMINATION_FAILED` and blocks a replacement child so overlap cannot occur.

When multiple conditions exist, status precedence is: `STOPPING` > `OWNERSHIP_CONFLICT`/`ERROR` > `CRASH_LOOP` > `STALE` > `DEGRADED` > `RECONNECTING` > `STREAMING` > `CONNECTING` > `STARTING` > `STOPPED`. Desired state remains a separate field and is never inferred from runtime state.

---

## 14. Failure Handling

| Failure | Detection | Required behavior | User-visible result |
|---|---|---|---|
| DNS failure | Connection exception | Worker retries with backoff/jitter. | `RECONNECTING`, sanitized DNS error. |
| Caster unreachable | Connect timeout/refusal | Retry without exiting where practical. | `RECONNECTING`, attempt/backoff. |
| Authentication rejected | Handshake status | Latch immediately; do not retry until credentials change or the operator explicitly retries. | `ERROR: AUTH_FAILED`. |
| Mountpoint rejected | Handshake/source response | Treat as configuration error. | `ERROR: MOUNTPOINT_REJECTED`. |
| Socket drops | EOF/exception | Close socket, reset parser connection context, reconnect. | Temporary `RECONNECTING`. |
| Bytes stop on live socket | Last-byte timeout | Close and reconnect. | `STALE` then `RECONNECTING`. |
| Bytes arrive but no valid frame | Valid-frame timeout and invalid counters | Mark stale/degraded; reconnect after policy threshold. | CRC/resync counters visible. |
| CRC mismatch | CRC-24Q comparison | Drop candidate, increment counter, resynchronize; never publish invalid frame. | `DEGRADED` only if threshold exceeded. |
| Garbage or lost framing | Preamble scan/buffer limit | Discard counted bytes and resynchronize. | Resync counter visible. |
| Frame above downstream limit | Size guard | Do not split blindly; drop/count/report. | Oversize diagnostic. |
| ROS publisher failure | Publish exception/status | Count, report, and retry/recreate according to ROS policy. | `DEGRADED` or `ERROR`. |
| MAVROS not ready | Expected ROS graph subscription unavailable | Remain `STARTING`, retry every second, and do not create/declare an active publishing child. | `STARTING: MAVROS_NOT_READY`. |
| Unknown second publisher | ROS graph inspection | Refuse start; if already active, stop the managed child, latch the error, and require conflict removal plus explicit Retry. | `ERROR: OWNERSHIP_CONFLICT`. |
| Expected MAVROS subscriber disappears | Three consecutive graph checks fail | Keep worker alive for diagnostics, enter stale state, and recover after subscriber returns plus next valid local publication. | `STALE: MAVROS_NOT_READY`. |
| Worker crash | Child monitor | Restart with manager backoff if desired state remains active. | Restart count/state visible. |
| Repeated worker crash | Restart budget | Enter crash loop; await explicit retry/cooldown. | `CRASH_LOOP`. |
| Backend restarts | FastAPI lifespan | Reload store; autostart when enabled/default valid. | Automatic recovery from backend start. |
| Frontend closes/restarts | No backend failure | No effect on RTK child. | UI resynchronizes from status. |
| Rover/Jetson reboots | Outer stack not automatically started in initial scope | Backend autostart works only after rover bringup starts. | Documented deployment limitation. |
| Corrupt store | Integrity/schema failure | Quarantine primary, atomically restore latest verified backup, disable autostart for this run, and require explicit Start; if recovery fails, keep diagnostic API alive. | `STORE_RECOVERED_FROM_BACKUP` or `CONFIG_INVALID`. |
| Missing secret | Secret reference check | Do not start. | `PASSWORD_NOT_CONFIGURED`. |

Errors must be rate-limited in logs, time-stamped, and represented by stable codes. Raw secrets, authorization headers, RTCM payloads, and full credential-bearing commands must never appear in logs.

---

## 15. Launch Integration

### 15.1 Initial launch boundary

`rover.launch.py` continues to start `rover_backend`. The correction worker is not added as an independently owned launch node because doing so would split lifecycle authority between ROS launch and `RTKManager`.

The backend startup path becomes:

```text
ros2 launch rover_bringup rover.launch.py
    |
    v
rover_backend
    |
    v
FastAPI lifespan -> RTKManager -> optional autostart -> correction worker
```

### 15.2 Configuration passed by launch/environment

Launch/deployment configuration should provide non-secret system policy only:

- profile/settings directory;
- runtime lock path;
- RTCM ROS topic;
- autostart default for first-run migration, if needed;
- stale and readiness thresholds;
- restart/backoff limits;
- log level;
- MAVROS readiness policy.

Caster passwords must not be embedded in launch files, source code, ROS command arguments, or committed environment files.

### 15.3 Backend readiness ordering

Because ROS processes may become ready in different orders, backend autostart must tolerate MAVROS starting later. Readiness requires a discovered subscription on `/mavros/gps_rtk/send_rtcm` with type `mavros_msgs/msg/RTCM`, configured full node identity defaulting to `/mavros`, and QoS compatible with the worker's reliable publisher. Until all checks pass, the manager exposes `STARTING: MAVROS_NOT_READY` and retries every second rather than treating startup ordering as fatal.

### 15.4 Future outer boot service

A later deployment phase may add one service that owns the whole rover bringup:

```text
Jetson boot -> rover-stack service -> rover.launch.py -> backend -> RTKManager
```

It should not add a separate independently managed RTK service.

---

## 16. Proposed Code Organization

Exact paths should follow existing repository conventions, but responsibilities should remain separated as follows.

### Backend

```text
rover_backend/
  rtk/
    manager.py          # desired state, lifecycle serialization, supervisor
    profile_store.py    # atomic persistence and secret references
    models.py           # request/response and internal state models
    process.py          # controlled child creation/termination/redaction
    health.py           # correction health aggregation and thresholds
    errors.py           # stable error codes
  api/
    rtk.py              # REST routes calling RTKManager only
```

### ROS correction package

```text
rtk_correction_bridge/
  ntrip_to_px4_node.py  # NTRIP connection and ROS publication
  rtcm3_parser.py       # incremental framing and CRC-24Q
  health_models.py      # worker counters/state payloads if needed
```

### Frontend

```text
services/rtkService     # exact backend REST contract
types/rtk               # correction and GNSS domain types
adapters/...            # backend-to-UI mapping without lifecycle side effects
screens/...             # profile/configuration and status controls
```

The parser must be independently unit-testable without ROS or network access. The manager must be testable with a fake process adapter and fake profile store.

---

## 17. Frontend Implementation Plan

### 17.1 Service contract

- Replace legacy or non-existent route usage with the REST surface in this document.
- Centralize RTK routes in one service module.
- Generate or define response types that keep `correction_stream` and `gnss_solution` separate.
- Treat all lifecycle operations as idempotent user actions with loading and final-state refresh.
- Polling `GET /api/rtk/status` must never invoke start/reconnect implicitly.

### 17.2 Profile UI

- List profiles from the rover.
- Allow create/edit/delete subject to backend constraints.
- Display `password_configured`, never a recovered password.
- Show a blank password field when editing; blank means “unchanged.”
- Provide a separate explicit “replace password” or “clear password” action.
- Allow selecting a default profile.
- Allow configuring backend-start autostart with a clear explanation.

### 17.3 Status UI

Display two cards or sections:

**Correction source**

- desired source/profile;
- state: off, connecting, streaming, stale, reconnecting, degraded, error;
- correction age;
- valid and invalid frame counts;
- reconnect attempt/restart count;
- actionable last error.

**GNSS solution**

- no fix/2D/3D/DGPS/RTK float/RTK fixed;
- satellites;
- horizontal accuracy;
- telemetry age.

### 17.4 Controls

- Start: one call with selected profile.
- Stop now: runtime stop only, with a separate option if the user also wants to disable future autostart.
- Reconnect: reconnect current desired profile without spawning a parallel child.
- Retry: visible only for latched error/crash loop.
- Disable unsupported source buttons until backend capability exists.

### 17.5 Multi-client behavior

Two tablets or two screens may issue calls concurrently. Backend serialization is authoritative. Each client refreshes status after mutations and accepts the backend snapshot instead of assuming its local action won.

---

## 18. Migration from `start_rtk.sh`

Migration must prevent an overlap between the manual worker and the managed worker.

### Stage A — Inventory and compatibility

- Record all current script parameters and their backend profile equivalents.
- Confirm no cron, service, operator alias, or launch include invokes the script.
- Capture a representative RTCM stream for parser tests without storing credentials.
- Verify deployed MAVROS message-size behavior and PX4/GNSS routing.

### Stage B — Add worker framing and metrics

- Implement and test the standalone RTCM3 parser.
- Integrate it into the current worker behind a development flag if rollback is needed.
- Confirm validated complete-frame publication with the deployed MAVROS version.
- Add health metrics and sanitize logging.

### Stage C — Add backend control plane with autostart disabled

- Add store, models, `RTKManager`, process adapter, supervisor, and REST routes.
- Keep autostart disabled by default during bench validation.
- Import or create the default rover-side profile through the API.
- Validate start/stop/reconnect and duplicate protection.

### Stage D — Frontend cutover

- Update the frontend service contract.
- Move profile authority to the rover.
- Require the operator to re-enter each retained password into the rover-side API; do not silently upload old plaintext credentials.
- After the backend confirms each new rover profile, delete the known legacy RTK profile/password AsyncStorage keys and verify they are absent. Document that historical device backups or external crash logs require the organization's normal retention/purge controls.
- Stop writing NTRIP passwords to tablet storage.
- Replace dead controls and separate health displays.

Deployment order is backend first, then the updated frontend, then legacy script retirement, and finally autostart enablement. For one migration release, the backend preserves every existing top-level `GET /api/rtk/status` field unchanged and adds the new nested domains plus `rtk_api_version: "1.0"`; the next breaking removal requires an API major-version change. The frontend accepts major version `1`, tolerates newer minor versions, and disables mutation controls with “backend update required” when the version is absent or has a different major version.

### Stage E — Operational cutover

1. Stop any manually running correction node.
2. Verify zero publishers on `/mavros/gps_rtk/send_rtcm` before managed start.
3. Start through the backend.
4. Verify exactly one expected publisher.
5. Delete/disable the direct launcher in the deployed package. If `start_rtk.sh` remains, verify from its installed contents and an execution test that it is an API-only client and cannot launch a correction worker.
6. Update operator documentation and troubleshooting steps.

### Stage F — Enable backend-start autostart

- Set a validated default profile.
- Enable autostart.
- Restart the backend and verify automatic recovery.
- Restart the rover bringup and verify the same behavior.
- Keep full Jetson power-on expectations explicitly separate until an outer boot service exists.

### Rollback

Maintain a versioned software rollback package and configuration backup. Do not keep two simultaneous production start paths as the rollback mechanism. If rollback is required, disable manager autostart, stop the managed child, verify the topic has zero publishers, deploy the previous version, and then use the prior documented procedure.

---

## 19. Phased Implementation

### Phase 0 — Baseline and contracts

Deliverables:

- confirmed deployed MAVROS/PX4/GNSS constraints;
- status and REST schemas;
- state/error enums;
- current performance baseline: connection time, frames/bytes, correction age, time to float/fixed;
- sanitized RTCM fixtures.

Exit gate: contracts and downstream frame-size behavior are agreed and testable.

### Phase 1 — RTCM3 parser and worker observability

Deliverables:

- incremental parser;
- CRC-24Q implementation;
- resynchronization and buffer bounds;
- parser metrics;
- complete-frame ROS publication;
- worker health output.

Exit gate: invalid frames never publish; fragmented and coalesced CRC-valid supported-size streams reproduce exact original frames; CRC-valid oversize frames are consumed and counted once but never published.

### Phase 2 — Rover-side profiles and credential handling

Deliverables:

- versioned models;
- atomic profile/settings store;
- protected secret handling;
- profile validation and recovery;
- profile CRUD tests.

Exit gate: secrets are absent from read APIs, logs, command lines, and frontend persistence.

### Phase 3 — `RTKManager` and supervision

Deliverables:

- singleton lifecycle owner;
- serialized commands;
- inter-process lock;
- process adapter;
- desired/actual state model;
- child monitor, backoff, restart budget, crash-loop behavior;
- graceful shutdown.

Exit gate: concurrent starts create one child; unexpected exit restarts; intentional stop does not restart.

### Phase 4 — REST API and backend startup integration

Deliverables:

- routes and stable errors;
- combined status snapshot with separated domains;
- FastAPI lifespan setup/teardown;
- backend-start autostart;
- MAVROS readiness handling.

Exit gate: backend restart restores RTK when configured and remains available when autostart fails.

### Phase 5 — Frontend cutover

Deliverables:

- updated service calls/types;
- rover-side profile screens;
- write-only password UX;
- separate correction/GNSS status;
- start, stop, reconnect, and retry controls;
- removal/hiding of dead paths.

Exit gate: every displayed control reaches an implemented route and has tested error behavior.

### Phase 6 — System and field validation

Deliverables:

- failure-injection results;
- end-to-end rover test evidence;
- migration and rollback runbooks;
- operator guide;
- accepted production configuration.

Exit gate: all acceptance criteria pass on the target rover.

### Phase 7 — Optional full boot recovery

Deliverable: one outer rover-stack boot service, added only after backend-owned RTK is stable.

Exit gate: Jetson power cycle starts bringup, backend, manager, and configured RTK without creating a second RTK owner.

---

## 20. Test and Validation Plan

### 20.1 RTCM3 parser unit tests

Test fixtures must include:

- one complete valid frame in one read;
- a valid frame split at every possible byte boundary;
- multiple valid frames in one read;
- one complete frame plus a partial next frame;
- leading noise before `0xD3`;
- repeated false `0xD3` bytes;
- CRC corruption in header/payload/CRC bytes;
- valid frame immediately after a corrupt frame;
- zero-length payload frame with valid CRC, asserting publication when under the downstream limit and no message-type decode;
- maximum 1023-byte payload frame;
- protocol-valid frame above deployed MAVROS limit;
- invalid reserved header bits;
- stalled partial frame;
- buffer-limit attack/garbage stream;
- randomized chunking of a known stream;
- randomized corruption where each mutated candidate is first confirmed to fail CRC, with the invariant that those CRC-invalid candidates never emit;
- CRC-24Q known-answer vectors.

Assertions:

- emitted bytes equal the original valid frame exactly;
- output ordering is preserved;
- each CRC-valid supported-size frame emits once; each CRC-valid oversize frame is consumed/counted once and does not emit;
- invalid candidates never emit;
- counters match expected values;
- buffer remains bounded;
- resynchronization recovers the next valid frame.

### 20.2 Worker tests

- NTRIP request formatting and redaction.
- Successful and rejected handshake.
- Authentication and mountpoint error mapping.
- Socket timeout, EOF, DNS failure, reconnect, and backoff.
- Parser reset/recovery across socket sessions.
- Stale-byte and stale-valid-frame detection.
- ROS publication of one message per CRC-valid supported-size frame and none for oversize frames.
- Frame-size guard using deployed MAVROS constraint.
- Graceful signal handling and bounded exit.

Use a local fake NTRIP caster; tests must not depend on production credentials or internet availability.

### 20.3 Profile/store tests

- create/read/update/delete;
- omitted password preserves secret;
- explicit replace and clear semantics;
- read responses never contain password;
- permissions on production-like filesystem;
- concurrent writes and atomicity;
- malformed/truncated primary file;
- last-known-good recovery;
- missing secret/default profile;
- schema migration;
- secret redaction in every exception path.

### 20.4 Manager unit tests

Use fake child and clock adapters to test deterministically:

- single start;
- repeated identical start;
- concurrent identical starts;
- concurrent different-profile starts;
- stop during start;
- reconnect during connecting/streaming;
- profile switch with confirmed non-overlap;
- unexpected child exit and restart;
- operator stop and no restart;
- backend shutdown and no restart;
- exponential backoff/jitter bounds;
- stable-runtime backoff reset;
- restart-budget exhaustion;
- retry from crash loop;
- lock acquisition failure;
- unknown ROS publisher detection;
- status snapshot consistency during transitions.

### 20.5 API tests

- route validation and status codes;
- idempotent lifecycle calls;
- stable error codes;
- active/default profile deletion conflict;
- write-only password behavior;
- simultaneous requests from multiple clients;
- shutdown/unavailable behavior;
- response schema compatibility with frontend;
- no lifecycle side effect from status polling.

### 20.6 Frontend tests

- profile create/edit with hidden password;
- blank edit password preserves configured secret;
- start/stop/reconnect/retry calls;
- loading, error, stale, and crash-loop displays;
- separate correction and GNSS cards;
- application close/resume without start side effects;
- two-screen or two-client refresh consistency;
- unsupported controls hidden/disabled;
- no password in AsyncStorage, logs, telemetry, or crash reports.

### 20.7 Integration tests with ROS/MAVROS

- Start fake caster -> worker -> `/mavros/gps_rtk/send_rtcm` subscriber and compare exact frames.
- Verify one ROS RTCM message per CRC-valid supported-size frame and none for CRC-valid oversize frames.
- Verify frames larger than 180 bytes are transported correctly by the deployed MAVROS plugin.
- Verify behavior at the deployed plugin maximum.
- Verify MAVLink `GPS_RTCM_DATA` reaches PX4 using telemetry/log evidence.
- Verify the configured PX4 port forwards corrections to the intended GNSS receiver using PX4 parameters plus receiver/PX4 correction-reception or correction-age evidence where exposed. RTK fix alone is supporting evidence, not sole proof of routing.
- Verify GNSS `fix_type`, satellites, and accuracy return through backend status.

### 20.8 Failure-injection tests

While streaming:

- disable network;
- terminate caster connection;
- return authentication failure;
- stream valid bytes, then silence;
- inject CRC-corrupt frames and random noise;
- kill the worker process;
- kill/restart backend;
- start two API calls simultaneously;
- attempt a manual competing publisher;
- close/restart frontend;
- restart rover bringup.

For each test, record state transitions, recovery time, child count, publisher count, correction age, restart count, and GNSS solution behavior.

### 20.9 Field validation

On the target rover:

1. Begin with open-sky GNSS conditions and known-good caster access.
2. Confirm backend autostart creates exactly one child and one topic publisher.
3. Confirm correction state reaches `STREAMING` based on valid publication.
4. Confirm GNSS independently progresses to expected solution quality.
5. Drive/operate for at least 60 minutes or one complete normal mission, whichever is longer.
6. Introduce controlled network loss and verify recovery.
7. Restart the child and backend separately.
8. Confirm no password or authorization header appears in logs/process listings.
9. Collect CPU, memory, reconnect, CRC, oversize, and correction-age observations.
10. Validate operator workflow without SSH or `start_rtk.sh`.

The secret-exposure check scans the backend application logs, system journal for the rover stack, child stdout/stderr capture, process arguments/environment visible to the service account, API access logs, frontend AsyncStorage export, and configured crash-report payloads using a unique test credential that is rotated immediately after validation.

---

## 21. Acceptance Criteria

### Ownership and lifecycle

- [ ] Exactly one `RTKManager` exists per backend process.
- [ ] Simultaneous Start requests result in at most one correction child.
- [ ] Exactly one expected publisher exists on `/mavros/gps_rtk/send_rtcm` during normal operation.
- [ ] A competing supported worker cannot acquire the injection lock.
- [ ] Operator Stop terminates the child and the supervisor does not restart it.
- [ ] An unexpected child exit restarts automatically within configured policy.
- [ ] Repeated crashes enter a visible crash-loop state rather than restarting forever at high frequency.
- [ ] Backend shutdown terminates the child cleanly.

### Autostart and persistence

- [ ] With autostart enabled and a valid default profile, starting the backend starts RTK without shell interaction.
- [ ] With autostart disabled, starting the backend does not start RTK.
- [ ] Invalid/missing profile or secret prevents autostart but does not prevent the backend API from starting.
- [ ] Backend restart restores configured RTK behavior.
- [ ] Documentation explicitly states that full Jetson-boot recovery needs the outer rover stack to start.

### RTCM correctness

- [ ] TCP chunk boundaries do not affect reconstructed frames.
- [ ] Every published ROS message contains exactly one complete RTCM3 frame.
- [ ] CRC-24Q-invalid frames are never published.
- [ ] The parser recovers and publishes a valid frame following corrupt/noisy bytes.
- [ ] Parser memory is bounded.
- [ ] Valid, invalid, discarded, partial-timeout, oversize, and published counters are accurate.
- [ ] The deployed MAVROS maximum frame behavior is measured and enforced.

### Credentials

- [ ] NTRIP profiles and default selection are rover-authoritative.
- [ ] Passwords are write-only through the API.
- [ ] Passwords do not appear in profile reads, status, logs, process arguments, frontend storage, or crash reports.
- [ ] Secret files and directories use documented restrictive ownership and permissions.
- [ ] Profile writes are atomic and recoverable from interruption.

### Health and user experience

- [ ] `GET /api/rtk/status` separately reports correction-stream health and GNSS solution quality.
- [ ] `STREAMING` requires recent successful publication of a CRC-valid frame.
- [ ] `STREAMING` is documented and tested as local ROS handoff with expected subscriber discovery, not as a MAVROS delivery acknowledgement.
- [ ] GNSS `RTK_FIXED` is derived from GNSS/PX4/MAVROS telemetry, not inferred from NTRIP connection.
- [ ] Every frontend RTK control maps to an implemented backend route.
- [ ] Closing or restarting the frontend has no effect on active injection.
- [ ] A second frontend client cannot create a second correction process.
- [ ] Operators can configure, start, stop, reconnect, diagnose, and recover RTK without SSH.

### End-to-end

- [ ] Valid RTCM travels through worker -> MAVROS -> PX4 -> configured GNSS receiver.
- [ ] GNSS solution feedback returns through MAVROS -> backend -> frontend.
- [ ] Controlled network loss, worker crash, and backend restart produce the documented state transitions and recovery.
- [ ] Migration leaves no independent production `start_rtk.sh` process-launch path.

---

## 22. Non-Goals

The initial implementation does not include:

- redesigning MAVROS, PX4, or GNSS firmware;
- changing the physical GNSS wiring or PX4 serial parameters, except verifying them;
- implementing a new RTCM transport protocol inside MAVROS;
- implementing a second correction source such as LoRa before single-source interfaces are stable;
- using GNSS RTK fixed as proof that the correction stream is currently healthy;
- automatic mission gating or vehicle-control policy based on RTK quality;
- cloud synchronization of NTRIP credentials;
- exposing NTRIP passwords back to the frontend;
- keeping the tablet application alive to sustain corrections;
- a separate system service that owns RTK independently of the backend;
- full Jetson power-on autostart in the first delivery phase;
- changing caster subscriptions, base-station configuration, or commercial credentials;
- guaranteeing secrecy from a local root/physical attacker without a hardware-backed secret store.

---

## 23. Operational Observability

Logs and status should answer these questions without shell archaeology:

- What correction source/profile is desired?
- Is the child alive?
- Is the caster connected?
- Are socket bytes arriving?
- Are CRC-valid RTCM frames arriving?
- Are valid frames being published to MAVROS?
- How old is the last successful publication?
- Are frames being rejected for CRC, framing, or size?
- Did the child restart, and why?
- Is another publisher competing for the topic?
- What solution does the GNSS currently report?

Structured logs should include manager generation/run ID, profile ID, child PID, state transition, error code, reconnect attempt, and counters where relevant. They must not include secrets or raw RTCM payloads. High-frequency errors require rate limiting and periodic summaries.

Recommended operational alarms:

- desired NTRIP but no valid publication beyond stale threshold;
- repeated CRC-invalid ratio above a configured window threshold;
- oversize frame observed;
- child restart rate above threshold;
- ownership conflict/additional ROS publisher;
- GNSS telemetry stale;
- correction stream healthy for a sustained period while GNSS remains below the expected solution level.

---

## 24. Definition of Done

The work is complete when the target rover can be operated as follows:

```text
Start rover bringup
    |
    v
Backend starts
    |
    v
RTKManager loads the rover-side default profile
    |
    v
Configured autostart starts one supervised correction worker
    |
    v
Worker reconstructs and CRC-validates RTCM3 frames
    |
    v
Validated frames publish to MAVROS
    |
    v
PX4/GNSS consume corrections
    |
    v
Frontend independently shows correction health and GNSS solution
```

An operator must no longer need to run `start_rtk.sh`, enter a password in a Jetson shell, keep the tablet open, or guess whether a green indicator represents the correction connection or the actual GNSS solution. The backend is the single RTK authority, failure behavior is deterministic and observable, invalid RTCM is rejected before MAVROS publication, and recovery from backend start is automatic when explicitly configured.
