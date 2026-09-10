# A/B RTCM Direct Injection — Production Implementation Plan

**Repository:** `ShariniSenthil/DYX_4WD_JETSON_PROTOTYPE`  
**Working branch:** `feat/rtcm-direct-gnss-uart`  
**Verified branch HEAD:** `0a5fac60a56d95ae2da8a8f5938fa539caaa7274`  
**Parent:** `97cd7e4f6b1f7b0963eb9605e017be985ce63421`  
**Source branch at review start:** `feat/rtk-injection-v2` — same HEAD as the new branch  
**Document purpose:** Originally a code-review plan only, with no implementation applied. **That is
no longer current — see the STATUS section immediately below.** The phase-by-phase design content
(§§1–33) is retained unedited as the implementation record; a "PROPOSED — NOT APPLIED" label on a
code excerpt below describes the state of the plan at authoring time, not the current repository.
Phase and acceptance-criteria sections carry inline status annotations pointing back to STATUS.  
**Revised 2026-09-10 (pre-implementation pass):** every "current code" excerpt below was re-derived
from source at the verified HEAD. Earlier revisions quoted a `_publish_candidates()` function that
does not exist and several wrong API names; those are corrected in place. Two deploy-breaking
omissions (profile-store schema-version gate, MAVROS readiness gate in direct mode) are now
addressed in §10.3 and §6.8.  
**Revised again 2026-09-10 (post field-validation pass):** Phases 1–6 have been implemented and
field-validated. See STATUS below for the final verified architecture, hardware identity, persisted
production profile state, the root cause of an initial false-negative field result, B-side PASS
evidence, and what remains explicitly open (Phase 7 / accuracy).

---

# STATUS — 2026-09-10: Phases 1–6 implemented and field-validated (PASS)

**This section is authoritative for current state.** The phase-by-phase content that follows (§§1–33)
is retained unedited as the implementation design record; where it says "PROPOSED — NOT APPLIED" that
describes the plan at authoring time, not the repository today.

## Final verified architecture

Two correction routes exist side by side, exactly as designed in §1/§4:

```text
Legacy A-side (already closed, see below):
    NTRIP → Jetson → MAVROS/PX4 → mosaic-H

Production B-side (this session's subject — PASS):
    NTRIP → Jetson → direct Mosaic-H native USB2
```

## Hardware

- Mosaic-H receiver serial number: `3804732`.
- USB interface mapping: `if02 → /dev/ttyACM1 → USB1` (diagnostics, still available) and
  `if04 → /dev/ttyACM2 → USB2` (production correction path).
- **Production stable device path:**
  `/dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device_3804732-if04`.
- `/dev/ttyACM0` is the Pixhawk FCU and must never be treated as the Mosaic-H.
- Receiver USB2 persistent config was verified as `DataInOut, USB2, RTCMv3, none, (on)` and saved
  Current → Boot.

⚠ **§18 below describes a discrete wired UART to a second mosaic-H COM. That is not what was built or
verified.** What was verified is the receiver's own **native USB2 CDC-ACM interface** (`if04`) — a
second logical serial port the receiver already exposes over the same USB cable, not a separate
physical wire. The "separate COM, don't combine PX4 TX and Jetson TX" safety intent in §18 is
satisfied (USB2 is logically and electrically independent of the port PX4's driver uses); read "COM"
there as "the receiver's USB2 logical port," not a wired UART.

## Persisted rover profile state (production)

Active profile 1 now owns direct injection as its production startup path:

- `direct_inject = true`
- `direct_serial_device = /dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device_3804732-if04`
- direct serial API baud = `230400`
- desired lifecycle state after successful explicit START = `RUNNING`
- latest observed persisted runtime revision = `8`

The software/schema **factory default for a brand-new profile remains `direct_inject = false`**
(§10.2's `DEFAULT_DIRECT_INJECT = False` is unchanged and correct) — a fresh database still
initializes to `STOPPED`. But this rover's one active, persisted profile has been switched to direct
USB2, so direct USB2 is now this rover's effective production correction path, not a still-optional
alternative.

## Startup/reconcile behavior — verified from source

On backend startup: `rtk_backend_lifecycle.start()` → `runtime.start()` →
`control.reconcile_runtime()`. `reconcile_runtime()` reads the persisted `desired_state`; if
`RUNNING`, it calls `runtime_service.request_start()`. Normal backend shutdown physically stops/reaps
the RTK runtime but does **not** change persisted operator intent. So after RTK has been explicitly
STARTed once with `desired_state` persisted as `RUNNING`, a normal
`ros2 launch rover_bringup rover.launch.py` restores RTK automatically — no repeat operator START
needed. A fresh database still initializes to `STOPPED`.

One accepted architectural detail, unchanged by this session (already documented at §6.8/§10.3/Phase
6 below): `RtkManagerCore._maybe_launch()` still gates worker spawning on MAVROS readiness even in
`direct_inject=true` mode. This does not route RTCM through MAVROS — it only means the direct-USB
worker waits for MAVROS to become ready. Accepted as non-blocking because MAVROS is part of the same
production launch and normally ready within seconds. **No patch was requested or made for this.**

## Why the first B-side attempt appeared broken — root cause, not a defect

The first stationary direct-mode test showed no `/mavros/gps_rtk/send_rtcm` traffic (expected — direct
mode doesn't use that topic) but the receiver also stayed at 3D lock, never reaching RTK. This was
**not a USB2 problem.** The profile's transport-field change had correctly forced persisted
`desired_state = STOPPED`, per the safety lifecycle contract in §22 — RTK had simply never been
explicitly STARTed again after the profile edit. Confirmed at the time: `rtk_runtime_state.desired_state
= STOPPED`, revision `7`, no RTK worker node running, nothing holding `/dev/ttyACM2`, USB permissions
correct, `flash` in `dialout`. After issuing the normal backend/GCS RTK START (`desired_state →
RUNNING`), the direct worker spawned and the B-side test proceeded successfully. **Recorded here so
this incident is not later misdiagnosed as a USB/serial failure** — it is §22's stop/start contract
working as designed, field-confirmed.

## B-side field validation — PASS

RTK FIXED was achieved and held entirely through `Jetson NTRIP → parser → SerialRtcmSink → USB2 →
Mosaic-H`, observed approximately 113 s after `desired_state` became `RUNNING`:

| Time | Fix | Sats | Horizontal accuracy |
|---|---|---:|---:|
| 17:12:39 | `6 / RTK FIXED` | 9 | 24 mm |
| 17:12:54 | `6 / RTK FIXED` | 11 | 15 mm |

(The legacy `eph` field showed large stale values during this test and must not be used as the
position-accuracy authority here — see the accuracy boundary below.)

Transport counters at the later sample: `frames_written_total = 853`, `bytes_written_total = 80524`,
`write_failures_total = 0`, `crc_failures = 0`, `invalid_headers = 0`. Roughly 395 additional RTCM
frames arrived over ~15 s (~26 frames/s) with no transport errors.

Validated checklist (all PASS):

1. NTRIP/parser valid RTCM
2. worker in `direct_serial` mode
3. `SerialRtcmSink` opens USB2
4. written frame/byte counters increase
5. write/delivery errors zero
6. permissions/no competing process
7. Mosaic-H consumes corrections, transitions 3D → RTK FIXED
8. end-to-end B-side transport/acquisition

`/mavros/gps_rtk/send_rtcm` remained silent throughout direct mode, proving exclusive routing with no
dual RTCM path.

**Not exercised in this session** — Phase 6's own "also deliberately test" fault-injection list
(§27): live unplug/reconnect of the direct serial cable while running, and flipping `direct_inject` on
an already-running worker and observing old-sink-release-before-new-sink-open. The stop/start contract
above was confirmed via the profile-edit path (revision 7→8), which is a different trigger than an
in-place fault. Treat those specific Phase 6 sub-tests as still open if anyone needs them signed off.

## A-side — already closed (unchanged this session)

Legacy MAVROS/PX4 A-side had already passed: `state = HEALTHY`, `injection_mode = mavros_px4`,
`direct_inject = false`, effective frame limit `720`, `valid_frames = 1341`, `delivery_frames = 1341`,
`published_frames = 1341`, delivery/publish errors `0`, oversize drops `0`, MAVROS subscribers `1`.
Both transport paths are now individually validated.

## Accuracy boundary — read before citing this session as an accuracy result

**This closes correction transport and RTK acquisition only.** Do not claim, and do not read this
document as claiming:

- RTK FIXED proves surveyed absolute accuracy;
- 15 mm receiver `h_acc` proves 15 mm truth error;
- direct USB solves wrong ambiguity selection;
- direct USB solves the previously observed ~150–210 mm wrong-but-RTK-FIXED position behavior
  documented in this repo's `CLAUDE.md` (2026-09-08/09 sections).

This matches invariant §1.9 and Phase 7 (§27) below, and is unchanged — direct injection was never
proposed as a fix for that failure mode, only for correction-transport robustness.

**There is no Phase 7 result in this document.** A surveyed/revisited-point accuracy comparison
between A-side and B-side has not been performed. Do not read Phase 6's PASS as a Phase 7 PASS.

## Frontend status

No frontend patch is required for the current production profile. The live `DYX_GCS_Frontend` editor
predates the direct fields, and its PATCH builder only sends fields actually changed — editing
caster/mountpoint/password/etc. does not overwrite the backend's persisted direct-USB fields. Known
limitations, not blockers for the existing production profile: the frontend cannot currently display
or change `direct_inject`, cannot select a USB device, and creating a brand-new profile without direct
fields still gets the backend default `direct_inject=false`. Future UI enhancement.

## Phase completion status

| Phase | Scope | Status |
|---|---|---|
| 1 | config/persistence/API | **Complete** |
| 2 | `SerialRtcmSink` | **Complete** |
| 3 | exclusive routing | **Complete** |
| 4 | health/status/hardening | **Complete** |
| 5 | A-side field validation | **PASS / closed** |
| 6 | B-side direct USB2 field validation | **PASS / closed** |
| 7 | A/B field accuracy comparison (survey truth) | **Not performed** — see accuracy boundary above |

Current production correction route: `NTRIP → Jetson → Mosaic-H USB2`. Persisted `RUNNING` means a
normal rover backend restart restores it automatically once the existing MAVROS-readiness gate clears
(see above).

---

## 1. Executive decision

Implement the new direct mosaic-H correction path as an **A/B selectable sink**. Do not delete, replace, or refactor away the current MAVROS/PX4 correction path.

The single authority switch is:

```text
direct_inject = false
    → CURRENT behavior
    → NTRIP → RTCM3 parser → MAVROS → MAVLink → PX4 → mosaic-H

direct_inject = true
    → NEW behavior
    → NTRIP → RTCM3 parser → direct Jetson serial → mosaic-H
```

### Non-negotiable invariants

1. `direct_inject` defaults to **false**.
2. `false` must preserve current production behavior.
3. `true` must send corrections only through the new direct serial sink.
4. The same RTCM frame must **never** be sent to both sinks.
5. Direct serial failure must **not** silently fall back to MAVROS.
6. Current NTRIP, GGA, TLS, RTCM3 CRC validation, persistence, process supervision, and injection-authority locking stay in place.
7. Current MAVROS/PX4 injection code remains available for immediate A/B rollback.
8. No PX4 firmware source deletion or MAVROS plugin deletion is part of this branch.
9. The new direct path does **not** claim to solve the previously observed wrong-but-RTK-FIXED ambiguity problem. This project changes correction transport robustness only.

---

# 2. Current verified production path

## 2.1 End-to-end current correction path

```text
NTRIP caster
    │
    │ TCP/TLS
    ▼
Jetson
src/rtk_correction_bridge/rtk_correction_bridge/ntrip_to_px4_node.py
    │
    │ recv(4096)
    ▼
src/rtk_correction_bridge/rtk_correction_bridge/rtcm_transport.py
    │
    │ RTCM3 parser + CRC validation
    │ current MAVROS-size policy
    ▼
mavros_msgs/RTCM
    │
    ▼
/mavros/gps_rtk/send_rtcm
    │
    ▼
MAVROS GpsRtkPlugin
    │
    │ MAVLink GPS_RTCM_DATA
    │ max 4 fragments × 180 bytes
    ▼
PX4
    │
    ▼
gps_inject_data
    │
    ▼
PX4 Septentrio GNSS driver
    │
    ▼
mosaic-H correction input
```

The current Jetson/PX4 MAVROS link remains a separate concern and is not removed by this work.

---

# 3. Critical current-code finding

The important implementation seam is **not merely the publish call site**.

Today, `RtcmWorkerTransport` applies the MAVROS frame-size gate **before** `NtripToPx4Node` receives the candidate frame list.

Current file:

```text
src/rtk_correction_bridge/rtk_correction_bridge/rtcm_transport.py
```

Current constants:

```python
DEFAULT_MAX_MAVROS_RTCM_FRAME_BYTES = 720
MAX_MAVROS_RTCM_FRAME_BYTES_LIMIT = MAX_FRAME_LENGTH
```

Current filtering behavior, verbatim from `RtcmWorkerTransport._process_parsed_frames`:

```python
for frame in frames:
    self.last_valid_frame_at = now_sec

    if frame.total_length > self.max_mavros_rtcm_frame_bytes:
        self.counters.rtcm_frames_oversize_total += 1
        continue

    publishable.append(frame.frame_bytes)
```

Therefore this would be **wrong**:

```python
# WRONG DESIGN
# The >720 B frame has already been discarded before this branch.
if direct_inject:
    serial_sink.write(frame)
else:
    self.rtcm_pub.publish(msg)
```

The A/B implementation must make the transport's accepted-frame ceiling depend on the active sink:

```text
direct_inject=false → retain current configured MAVROS limit, normally 720 B
direct_inject=true  → accept complete protocol-valid RTCM3 frame, up to 1029 B
```

This preserves legacy behavior while removing the MAVROS-only limitation from direct mode.

---

# 4. Target architecture

```text
                           NTRIP CASTER
                                │
                           TCP / TLS / GGA
                                │
                                ▼
                 ┌────────────────────────────┐
                 │ NtripToPx4Node             │
                 │                            │
                 │ existing socket handling   │
                 │ existing reconnect logic   │
                 │ existing GGA logic         │
                 └─────────────┬──────────────┘
                               │ bytes
                               ▼
                 ┌────────────────────────────┐
                 │ RtcmWorkerTransport        │
                 │ RTCM3 parser + CRC         │
                 └─────────────┬──────────────┘
                               │ complete valid frame
                               ▼
                     A/B ACTIVE-SINK ROUTER
                               │
             ┌─────────────────┴──────────────────┐
             │                                    │
 direct_inject=false                    direct_inject=true
             │                                    │
             ▼                                    ▼
 CURRENT MAVROS SINK                       NEW SERIAL SINK
 mavros_msgs/RTCM                         SerialRtcmSink
 /mavros/gps_rtk/send_rtcm                     │
             │                                  │ exact RTCM3 bytes
             ▼                                  ▼
          MAVROS                         dedicated Jetson UART
             │                                  │
             ▼                                  ▼
      GPS_RTCM_DATA                         mosaic-H
             │                           correction-only COM
             ▼
            PX4
             │
             ▼
         mosaic-H
```

Only the selected branch is permitted to execute.

---

# 5. File-change matrix

| File | State | Planned role |
|---|---|---|
| `src/rtk_correction_bridge/rtk_correction_bridge/ntrip_to_px4_node.py` | **MODIFY** | Create active sink, select frame-size policy, route each validated frame to exactly one sink, keep health topics |
| `src/rtk_correction_bridge/rtk_correction_bridge/rtcm_transport.py` | **NO CODE CHANGE** | It already exports `MAX_MAVROS_RTCM_FRAME_BYTES_LIMIT` (= `MAX_FRAME_LENGTH` = 1029) and `validate_max_mavros_rtcm_frame_bytes()` already accepts up to 1029. Only the module docstring needs a direct-mode sentence — see §8 |
| `src/rtk_correction_bridge/rtk_correction_bridge/serial_rtcm_sink.py` | **NEW** | Dedicated production serial writer for complete RTCM3 frames |
| `src/rtk_correction_bridge/rtk_correction_bridge/status_snapshot.py` | **MODIFY** | Add active injection mode and direct-serial observability |
| `src/rtk_correction_bridge/package.xml` | **MODIFY** | Add runtime serial library dependency if PySerial is selected |
| `src/rtk_correction_bridge/setup.py` | **REVIEW / likely no structural change** | New Python module is package-discovered automatically; dependency policy to be decided with ROS image packaging |
| `src/rover_backend/rover_backend/rtk_process_protocol.py` | **MODIFY** | Worker config schema bump and direct-mode fields |
| `src/rover_backend/rover_backend/rtk_profile_store.py` | **MODIFY** | Persist A/B flag and serial settings; additive DB migration |
| `src/rover_backend/rover_backend/rtk_routes.py` | **MODIFY** | Expose create/update/read fields through authenticated RTK API |
| `src/rover_backend/rover_backend/rtk_worker_bootstrap.py` | **NO FUNCTIONAL CHANGE EXPECTED** | Preserve secure worker config FD, parent liveness, and injection lock |
| `src/rover_backend/rover_backend/rtk_manager_core.py` | **NO CODE CHANGE — but read §6.8** | `_maybe_launch()` will not spawn the worker unless `_mavros_ready`. Direct mode inherits that gate deliberately |
| `src/rover_backend/rover_backend/rtk_mavros_readiness.py` | **NO CODE CHANGE — but read §6.8** | Readiness requires a live subscriber on the MAVROS RTCM topic, in both modes |
| `src/rover_backend/rover_backend/rtk_control_service.py` | **MODIFY** | `update_profile()` does not restart a running worker on a field-only edit; a `direct_inject` change must force a controlled stop/start — see §22 |
| PX4 firmware | **NO CODE CHANGE in this patch** | Continue GNSS output/fusion; receiver persistent configuration handled separately |
| MAVROS | **NO CODE CHANGE / NO REMOVAL** | Remains legacy A-side transport |

---

# 6. File 1 — `ntrip_to_px4_node.py`

## 6.1 Current verified responsibilities

Path:

```text
src/rtk_correction_bridge/rtk_correction_bridge/ntrip_to_px4_node.py
```

⚠ **Earlier revisions of this document described a `_publish_candidates()` method. No such method
exists anywhere in `src/`.** The real chain, verified at HEAD, is:

```text
_process_stream_bytes(data, now)
    → self.transport.process_stream_bytes(data, now)   # returns list[bytes]
    → self._process_parsed_frames(candidates, now)
        → self.transport.attempt_publish(frame_bytes, now, self._publish_rtcm_frame)
            → self._publish_rtcm_frame(frame_bytes)     # builds RTCM msg, publishes
```

The node stores its own size gate and builds the transport from the worker config:

```python
self.max_mavros_rtcm_frame_bytes = (
    validate_max_mavros_rtcm_frame_bytes(
        worker_config.max_mavros_rtcm_frame_bytes
    )
)

self.transport = RtcmWorkerTransport(
    max_mavros_rtcm_frame_bytes=(
        self.max_mavros_rtcm_frame_bytes
    )
)
```

There is no `self.config` attribute and no `self._transport` attribute; the node uses
`self.transport`.

Current per-frame delivery:

```python
def _process_parsed_frames(self, candidates, now):
    for frame_bytes in candidates:

        published = self.transport.attempt_publish(
            frame_bytes,
            now,
            self._publish_rtcm_frame,
        )

        if not published:
            self._pending_publish_errors += 1


def _publish_rtcm_frame(self, frame_bytes):
    msg = RTCM()
    msg.data = list(frame_bytes)
    self.rtcm_pub.publish(msg)
```

### The active-sink seam already exists

`RtcmWorkerTransport.attempt_publish(frame_bytes, now_sec, publisher)` **takes the sink as a
callable** and already owns the try/except, the published counter, and the published timestamp:

```python
def attempt_publish(self, frame_bytes, now_sec, publisher):
    try:
        publisher(frame_bytes)
    except Exception:
        self.counters.rtcm_publish_errors_total += 1
        return False
    self.counters.rtcm_frames_published_total += 1
    self.last_published_frame_at = now_sec
    return True
```

A/B routing is therefore **selecting which callable is passed**, not writing a new delivery
function. See §6.6.

---

## 6.2 Proposed imports

**PROPOSED — NOT APPLIED**

```python
from rtk_correction_bridge.rtcm_transport import (
    MAX_MAVROS_RTCM_FRAME_BYTES_LIMIT,
)
from rtk_correction_bridge.serial_rtcm_sink import (
    SerialRtcmSink,
    SerialRtcmSinkError,
)
```

`MAX_MAVROS_RTCM_FRAME_BYTES_LIMIT` is already defined in `rtcm_transport.py` as
`MAX_FRAME_LENGTH` (1029) and is already the upper bound accepted by
`validate_max_mavros_rtcm_frame_bytes()`. **No new constant is needed.**

---

## 6.3 Proposed mode constants

**PROPOSED — NOT APPLIED**

```python
INJECTION_MODE_MAVROS_PX4 = "mavros_px4"
INJECTION_MODE_DIRECT_SERIAL = "direct_serial"
```

Constructor:

```python
self._injection_mode = (
    INJECTION_MODE_DIRECT_SERIAL
    if worker_config.direct_inject
    else INJECTION_MODE_MAVROS_PX4
)
```

There is no `self.config` on this node — see §6.1.

---

## 6.4 Proposed mode-specific frame ceiling

**PROPOSED — NOT APPLIED**

```python
self.max_mavros_rtcm_frame_bytes = (
    validate_max_mavros_rtcm_frame_bytes(
        worker_config.max_mavros_rtcm_frame_bytes
    )
)

effective_max_frame_bytes = (
    MAX_MAVROS_RTCM_FRAME_BYTES_LIMIT
    if worker_config.direct_inject
    else self.max_mavros_rtcm_frame_bytes
)

self.transport = RtcmWorkerTransport(
    max_mavros_rtcm_frame_bytes=effective_max_frame_bytes
)
```

Keep `self.max_mavros_rtcm_frame_bytes` as the **configured legacy gate** for logging and status
reporting; `effective_max_frame_bytes` is only what the transport enforces.

### Why this is deliberately minimal

The transport parameter still has the old MAVROS-specific name. Renaming it during the first A/B patch would broaden the diff and make false-mode regression review harder.

For the initial A/B branch:

```text
false → same value and same behavior as current code
true  → use RTCM3 protocol ceiling instead
```

A future cleanup can rename internals after A/B field acceptance.

---

## 6.5 Proposed sink construction

**PROPOSED — NOT APPLIED**

```python
self._serial_sink = None

if worker_config.direct_inject:
    self._serial_sink = SerialRtcmSink(
        device=worker_config.direct_serial_device,
        baudrate=worker_config.direct_serial_baud,
        write_timeout_sec=(
            worker_config.direct_serial_write_timeout_sec
        ),
        reopen_delay_sec=(
            worker_config.direct_serial_reopen_sec
        ),
    )
```

### Important

When `direct_inject=false`:

- do not require direct serial hardware;
- do not open the serial device;
- do not alter current MAVROS publication;
- do not alter current RTCM topic;
- do not alter current 720 B behavior.

When `direct_inject=true`, **`self.rtcm_pub` is still created and still advertises the RTCM topic.**
It simply never publishes. This is required, not incidental: the backend's MAVROS-readiness gate
counts subscribers on that topic and refuses to launch the worker without one — see §6.8. Creating
the publisher costs nothing and keeps that gate satisfiable in both modes.

---

## 6.6 Proposed single routing function

Do not scatter `if direct_inject` across socket/parser code.

Use one active-sink function.

**PROPOSED — NOT APPLIED**

Do **not** write a new delivery function. `attempt_publish()` already is one, and it already owns
the try/except, the published counter, and the published timestamp (§6.1). Re-implementing that in
the node would create a second definition of "delivered" and duplicate exception-safe accounting the
transport owns.

Select the sink callable once in the constructor:

```python
self._active_sink = (
    self._write_rtcm_frame_serial
    if worker_config.direct_inject
    else self._publish_rtcm_frame
)
```

with the new writer alongside the existing publisher:

```python
def _write_rtcm_frame_serial(self, frame_bytes):
    if self._serial_sink is None:
        raise RuntimeError(
            'direct RTCM injection selected without serial sink'
        )

    self._serial_sink.write_frame(frame_bytes)
```

`_process_parsed_frames` then changes by exactly one identifier:

```python
def _process_parsed_frames(self, candidates, now):
    for frame_bytes in candidates:

        published = self.transport.attempt_publish(
            frame_bytes,
            now,
            self._active_sink,        # was self._publish_rtcm_frame
        )

        if not published:
            self._pending_publish_errors += 1
```

### Why this is the whole change

- `attempt_publish` swallows the sink exception and returns `False`, so a serial write failure
  already increments `rtcm_publish_errors_total` and already leaves `last_published_frame_at`
  untouched. Correction age therefore goes stale on its own in direct mode — no new health plumbing.
- Nothing calls `record_publish_success()` / `record_publish_error()` from the node today, so their
  MAVROS-flavoured names never appear on the direct path. **The naming question raised in earlier
  revisions of this section does not arise.** For reference, the real signatures are
  `record_publish_success(now_sec)` and `record_publish_error()` — not `record_publish_failure()`,
  and `record_publish_success` takes no `frame_length`.
- One `if` at construction, one identifier at the call site. Mutual exclusion is structural: there
  is exactly one `_active_sink`.

---

## 6.7 No automatic fallback

**PROPOSED REQUIRED BEHAVIOR**

This is forbidden:

```python
try:
    serial_sink.write_frame(frame)
except Exception:
    mavros_pub.publish(frame)  # DO NOT DO THIS
```

Correct direct-mode behavior:

```text
serial failure
    → mark delivery unhealthy
    → report explicit direct-serial error
    → close/reopen serial according to policy
    → NO MAVROS fallback
```

Reason:

- A/B evidence must remain unambiguous.
- One correction authority must exist at a time.
- Silent fallback hides hardware faults.
- A serial recovery followed by a still-active legacy path could create dual delivery.

Rollback is an operator/configuration action:

```text
direct_inject=false
```

followed by worker reconciliation/restart.

---

## 6.8 ⚠ Direct mode still requires MAVROS to be alive — decided, not overlooked

The RTK worker is launched by `RtkManagerCore`, and `_maybe_launch()` refuses to spawn it unless
`_mavros_ready` is true, parking in `WAITING_FOR_MAVROS` instead. The same gate re-applies after any
unexpected worker exit. `evaluate_mavros_rtcm_readiness()` in `rtk_mavros_readiness.py` requires all
three of:

```text
MAVROS reports an FCU connection
/mavros/state observation is fresh
rtcm_subscriber_count >= 1 on the MAVROS RTCM injection topic
```

So with `direct_inject=true` the worker will not start unless MAVROS is up and a subscriber exists on
the very topic the direct path bypasses.

**Decision for this branch: leave the gate exactly as it is.** Reasons:

- The MAVROS link to PX4 exists in every configuration this rover runs; it carries GNSS telemetry,
  arming, and OFFBOARD. It is not an optional component that direct injection makes removable.
- Changing the launch gate would touch the manager FSM, which is the most safety-relevant code in
  the RTK stack and is out of scope for a transport experiment.
- Keeping one launch precondition for both sides keeps the A/B comparison honest.

**Two consequences must be stated rather than discovered:**

1. `self.rtcm_pub` must still be created in direct mode (§6.5), or the subscriber count is zero and
   the worker never launches.
2. **This work does not make correction delivery survive a MAVROS outage.** It removes the MAVLink
   fragmentation and the 720 B ceiling from the correction path; it does not remove MAVROS as a
   liveness dependency. Do not describe the B-side as "no longer depends on MAVROS."

If a future phase genuinely needs correction delivery without MAVROS, that is a separate change to
`rtk_mavros_readiness.py` and `rtk_manager_core.py`, made deliberately and tested on its own.

---

# 7. File 2 — NEW `serial_rtcm_sink.py`

## 7.1 New file path

```text
src/rtk_correction_bridge/rtk_correction_bridge/serial_rtcm_sink.py
```

This file does not exist in the reviewed baseline.

Its scope must remain narrow:

```text
input: one already CRC-validated complete RTCM3 frame
output: exactly the same bytes on the configured serial device
```

It must **not**:

- parse RTCM again;
- alter RTCM bytes;
- fragment RTCM;
- build MAVLink;
- publish ROS messages;
- send Septentrio configuration commands;
- configure mosaic-H;
- automatically select another injection path.

---

## 7.2 Proposed class API

**PROPOSED — NOT APPLIED**

```python
class SerialRtcmSinkError(RuntimeError):
    pass


class SerialRtcmSink:
    def __init__(
        self,
        *,
        device: str,
        baudrate: int,
        write_timeout_sec: float,
        reopen_delay_sec: float,
    ) -> None:
        ...

    def write_frame(
        self,
        frame_bytes: bytes,
    ) -> None:
        ...

    def close(self) -> None:
        ...

    def snapshot(self) -> dict:
        ...
```

---

## 7.3 Proposed serial settings

Initial production baseline:

```text
device       = /dev/serial/by-id/<actual-stable-device-id>
baud         = 230400
data bits    = 8
parity       = none
stop bits    = 1
flow control = none
```

The exact `/dev/serial/by-id/...` path must come from the actual Jetson hardware. It must **not** be invented or hardcoded from a transient `/dev/ttyUSB0` assumption.

`230400` is the initial engineering default, not a hard protocol requirement. It remains configurable.

---

## 7.4 Proposed open behavior

Using the mature Python serial library is preferred over writing custom termios state handling.

**PROPOSED — NOT APPLIED**

```python
import serial


def _open(self) -> None:
    self._serial = serial.Serial(
        port=self.device,
        baudrate=self.baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0,
        write_timeout=self.write_timeout_sec,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
        exclusive=True,
    )
```

**Open exclusively — this is a requirement, not an option.** PySerial supports `exclusive=True` on
POSIX; pass it.

The existing `InjectionOwnershipLock` is an `flock` on `/run/lock/rover-rtk-injection.lock`. It
excludes a second **rover_ws** injection process and nothing else. It does not stop any other process
on the Jetson from holding or writing the same TTY, and a second writer on that UART is precisely the
new risk this design introduces. `exclusive=True` and the injection lock cover different failure
modes; use both.

---

## 7.5 Complete-frame write requirement

No application-level chunking.

**PROPOSED — NOT APPLIED**

```python
def write_frame(
    self,
    frame_bytes: bytes,
) -> None:
    if not frame_bytes:
        raise SerialRtcmSinkError(
            "empty RTCM frame"
        )

    self._ensure_open()

    view = memoryview(frame_bytes)
    offset = 0

    try:
        while offset < len(view):
            written = self._serial.write(
                view[offset:]
            )

            if written is None or written <= 0:
                raise SerialRtcmSinkError(
                    "serial write made no progress"
                )

            offset += written

    except (
        serial.SerialException,
        serial.SerialTimeoutException,
        OSError,
    ) as exc:
        self._record_write_failure()
        self._close_for_reconnect()

        raise SerialRtcmSinkError(
            f"direct RTCM serial write failed: {exc}"
        ) from exc

    self._record_write_success(
        frame_length=len(frame_bytes)
    )
```

Even if the serial library normally completes a blocking write, the loop makes the contract explicit: success means all bytes of the selected complete RTCM frame were accepted by the serial API.

---

## 7.6 Proposed reconnect behavior

```text
serial device unavailable
    ↓
open attempt fails
    ↓
record failure
    ↓
active sink unhealthy
    ↓
do not publish to MAVROS
    ↓
respect direct_serial_reopen_sec
    ↓
retry open on later frame
```

Avoid a tight open-failure loop.

No NTRIP reconnect should be forced solely because the local serial device is unavailable unless later testing proves that coupling is desirable. NTRIP source health and serial sink health are separate failure domains.

---

## 7.7 Proposed serial sink counters

Suggested local metrics:

```text
open_attempts_total
open_failures_total
reopen_total
frames_written_total
bytes_written_total
write_failures_total
last_successful_write_monotonic
serial_open
```

These supplement, not replace, existing parser/transport counters.

---

# 8. File 3 — `rtcm_transport.py`

Path:

```text
src/rtk_correction_bridge/rtk_correction_bridge/rtcm_transport.py
```

## 8.1 Current behavior to retain

```python
DEFAULT_MAX_MAVROS_RTCM_FRAME_BYTES = 720
```

Do not remove this constant.

Legacy mode still needs this limit because its downstream MAVROS/MAVLink transport has a smaller maximum than RTCM3 itself.

---

## 8.2 No code change is required in this file

The constant the direct path needs **already exists and is already exported**:

```python
DEFAULT_MAX_MAVROS_RTCM_FRAME_BYTES = 720
MAX_MAVROS_RTCM_FRAME_BYTES_LIMIT = MAX_FRAME_LENGTH   # 1029
```

and `validate_max_mavros_rtcm_frame_bytes()` already accepts `1..1029`. Earlier revisions proposed
adding a `MAX_RTCM3_FRAME_BYTES` alias — that is redundant. `NtripToPx4Node` simply chooses:

```python
frame_limit = (
    MAX_MAVROS_RTCM_FRAME_BYTES_LIMIT
    if worker_config.direct_inject
    else worker_config.max_mavros_rtcm_frame_bytes
)
```

**The only edit this file needs is documentation.** Its module docstring currently states that 720 is
"the downstream MAVROS development size gate" as though it were the only gate. Add one sentence: in
direct-serial mode the node constructs the transport with the protocol ceiling instead. Leave the
code alone.

### Result

```text
A / false:
    same constructor limit as today
    same >720 oversize-drop behavior

B / true:
    protocol-valid RTCM3 frames pass up to 1029 bytes
    no MAVROS-only 720 B restriction
```

---

## 8.3 What must not happen

Do **not** globally change:

```python
DEFAULT_MAX_MAVROS_RTCM_FRAME_BYTES = 1029
```

That would silently change legacy A-side behavior and allow frames the existing MAVROS route cannot carry.

---

# 9. File 4 — `rtk_process_protocol.py`

Path:

```text
src/rover_backend/rover_backend/rtk_process_protocol.py
```

## 9.1 Current verified protocol

Current worker config schema (note the exact constant name):

```python
WORKER_CONFIG_SCHEMA_VERSION = 3
MAX_RTCM3_FRAME_BYTES = 1029
```

Current `WorkerConfig` already carries NTRIP, GGA, TLS, RTCM topic, timeouts, and:

```python
max_mavros_rtcm_frame_bytes: int
```

Current decoding is strict about expected keys/schema.

---

## 9.2 Proposed schema bump

**PROPOSED — NOT APPLIED**

```python
WORKER_CONFIG_SCHEMA_VERSION = 4
```

⚠ **Field ordering matters.** `tls_mode: str = "REQUIRED"` is currently the only defaulted field and
it is last. Appending non-default fields after it raises
`TypeError: non-default argument follows default argument` at import time. Give the new fields
defaults so they sit after `tls_mode`:

```python
@dataclass(frozen=True)
class WorkerConfig:
    # ... current fields unchanged ...

    max_mavros_rtcm_frame_bytes: int

    tls_mode: str = "REQUIRED"

    direct_inject: bool = False
    direct_serial_device: Optional[str] = None
    direct_serial_baud: int = 230400
    direct_serial_write_timeout_sec: float = 1.0
    direct_serial_reopen_sec: float = 1.0
```

Defaulting them here is also the second layer of the safe-default guarantee: a config that omits them
decodes as legacy mode.

The same fields must be added to **four** places, not two — there are no `to_dict`/`from_dict`
methods on this dataclass:

```text
encode_worker_config()          # the payload dict
decode_worker_config()          # the read-back and per-field validation
_WORKER_CONFIG_FIELDS           # the frozenset driving strict exact-key validation
__hash__ / repr field tuple     # the tuple listing every field
```

---

## 9.3 Validation rules

There is no `WorkerProtocolError` in this module. Use the existing classes:
`ConfigValidationError` in `__post_init__`, `ConfigDecodeError` in `decode_worker_config()`.

```python
if not isinstance(direct_inject, bool):
    raise ConfigValidationError(
        "direct_inject must be a bool"
    )

if direct_inject:
    if not direct_serial_device:
        raise ConfigValidationError(
            "direct_serial_device is required "
            "when direct_inject=true"
        )

    if not direct_serial_device.startswith("/dev/"):
        raise ConfigValidationError(
            "direct_serial_device must be "
            "an absolute /dev path"
        )
```

Follow the existing house style for the numeric checks: reject `bool` explicitly before
`isinstance(..., int)`, since `bool` is an `int` in Python and every current field in this file
guards against it.

Suggested numerical validation:

```text
direct_serial_baud > 0
direct_serial_write_timeout_sec > 0
direct_serial_reopen_sec >= 0
```

False mode must accept:

```json
{
  "direct_inject": false,
  "direct_serial_device": null
}
```

so a normal legacy deployment has zero direct-serial hardware requirement.

---

# 10. File 5 — `rtk_profile_store.py`

Path:

```text
src/rover_backend/rover_backend/rtk_profile_store.py
```

## 10.1 Current verified persistence design

The current RTK profile is persisted in SQLite and already supports additive schema migration.

Current profile includes, among its other fields:

```text
rtcm_topic
connect/socket/health/reconnect settings
GGA settings
TLS settings
max_mavros_rtcm_frame_bytes
enabled
revision/timestamps
```

The current default MAVROS frame size remains:

```text
720 bytes
```

---

## 10.2 Proposed new defaults

**PROPOSED — NOT APPLIED**

```python
DEFAULT_DIRECT_INJECT = False

DEFAULT_DIRECT_SERIAL_DEVICE = None

DEFAULT_DIRECT_SERIAL_BAUD = 230400

DEFAULT_DIRECT_SERIAL_WRITE_TIMEOUT_SEC = 1.0

DEFAULT_DIRECT_SERIAL_REOPEN_SEC = 1.0
```

---

## 10.3 Proposed additive database columns

**PROPOSED — NOT APPLIED**

```sql
direct_inject INTEGER NOT NULL DEFAULT 0
    CHECK(direct_inject IN (0, 1));

direct_serial_device TEXT NULL;

direct_serial_baud
    INTEGER NOT NULL DEFAULT 230400;

direct_serial_write_timeout_sec
    REAL NOT NULL DEFAULT 1.0;

direct_serial_reopen_sec
    REAL NOT NULL DEFAULT 1.0;
```

Migration must be additive only, guarded on `existing_version` exactly the way the v1→v2 GGA columns
and the v1/v2→v3 `tls_mode` column already are.

Existing production rows must automatically become:

```text
direct_inject = false
```

This is the database-level guarantee that deploying the branch does not activate the new transport.

### ⚠ The schema-version gate must be widened, or the backend will not open an existing rover DB

Two things move together and one of them is easy to miss:

```python
RTK_PROFILE_SCHEMA_VERSION = 3        # → 4
```

and the accepted-version check in the store opener:

```python
if existing_version not in {
    0,
    1,
    2,
    RTK_PROFILE_SCHEMA_VERSION,
}:
    raise RtkProfileStoreError(
        f"unsupported RTK profile schema version {existing_version}"
    )
```

Bumping the constant to `4` without adding `3` to that set means every rover already running today
raises `unsupported RTK profile schema version 3` on backend start. The RTK profile store fails to
open and the backend does not come up.

Required change:

```python
RTK_PROFILE_SCHEMA_VERSION = 4

if existing_version not in {
    0,
    1,
    2,
    3,
    RTK_PROFILE_SCHEMA_VERSION,
}:
    ...
```

plus a `if existing_version in {1, 2, 3}:` branch adding the five columns, and the existing
`PRAGMA user_version = RTK_PROFILE_SCHEMA_VERSION` write at the end of migration.

**Test this against a real copy of the Jetson's current `.db` file, not only a freshly created one.**
A fresh DB is created at the new version and never exercises the migration branch, so an
upgrade-path bug passes the unit tests and bricks the rover.

---

## 10.4 Proposed profile fields

Add to persisted profile/snapshot:

```python
direct_inject: bool

direct_serial_device: Optional[str]

direct_serial_baud: int

direct_serial_write_timeout_sec: float

direct_serial_reopen_sec: float
```

Add to profile:

```text
create
update
row decoding
validation
credential-free snapshot
worker_config_from_profile()
```

Example mapping:

```python
WorkerConfig(
    # ... existing fields ...

    max_mavros_rtcm_frame_bytes=(
        profile.max_mavros_rtcm_frame_bytes
    ),

    direct_inject=profile.direct_inject,

    direct_serial_device=(
        profile.direct_serial_device
    ),

    direct_serial_baud=(
        profile.direct_serial_baud
    ),

    direct_serial_write_timeout_sec=(
        profile.direct_serial_write_timeout_sec
    ),

    direct_serial_reopen_sec=(
        profile.direct_serial_reopen_sec
    ),
)
```

---

# 11. File 6 — `rtk_routes.py`

Path:

```text
src/rover_backend/rover_backend/rtk_routes.py
```

## 11.1 Current verified API model

Current create request exposes:

```python
rtcm_topic: Any = "/mavros/gps_rtk/send_rtcm"
...
max_mavros_rtcm_frame_bytes: Any = 720
enabled: Any = True
```

Current update request exposes the same setting as optional:

```python
max_mavros_rtcm_frame_bytes: Any = None
```

Current response projection also returns:

```python
"rtcm_topic": profile.rtcm_topic,
...
"max_mavros_rtcm_frame_bytes": (
    profile.max_mavros_rtcm_frame_bytes
),
```

No direct-serial selection exists yet.

---

## 11.2 Proposed create request

**PROPOSED — NOT APPLIED**

```python
class RtkProfileCreateRequest(BaseModel):
    # ... current fields untouched ...

    direct_inject: Any = False

    direct_serial_device: Any = None

    direct_serial_baud: Any = 230400

    direct_serial_write_timeout_sec: Any = 1.0

    direct_serial_reopen_sec: Any = 1.0

    model_config = ConfigDict(
        extra="forbid",
    )
```

---

## 11.3 Proposed update request

```python
class RtkProfileUpdateRequest(BaseModel):
    # ... current fields untouched ...

    direct_inject: Any = None

    direct_serial_device: Any = None

    direct_serial_baud: Any = None

    direct_serial_write_timeout_sec: Any = None

    direct_serial_reopen_sec: Any = None
```

### Nullable-field review — RESOLVED, no new mechanism needed

Earlier revisions flagged this as an open checkpoint. It is already handled by existing code.

`_update_values()` uses:

```python
values = body.model_dump(exclude_unset=True)
```

so an **omitted** field never reaches the store, while an **explicit JSON `null`** does. That is
exactly the desired semantics:

- omitted field → preserve current value;
- JSON `null` → explicitly clear `direct_serial_device`.

`password` is the one field with a deliberate carve-out — explicit null is popped so it means
"preserve", because clearing a password by accident is unsafe. `direct_serial_device` needs **no**
such carve-out: clearing it is a legitimate, reversible operator action, and profile validation
already rejects `direct_inject=true` with a null device.

No focused normalization path is required. Do not add one.

---

## 11.4 Proposed profile response

```python
"direct_inject": profile.direct_inject,

"direct_serial_device": (
    profile.direct_serial_device
),

"direct_serial_baud": (
    profile.direct_serial_baud
),

"direct_serial_write_timeout_sec": (
    profile.direct_serial_write_timeout_sec
),

"direct_serial_reopen_sec": (
    profile.direct_serial_reopen_sec
),
```

The serial device path is not a secret.

NTRIP password handling must remain exactly as currently designed: write-only and never included in public profile/status payloads.

---

# 12. File 7 — `status_snapshot.py`

Path:

```text
src/rtk_correction_bridge/rtk_correction_bridge/status_snapshot.py
```

## 12.1 Goal

Existing health topic names remain stable so downstream rover logic does not need an A/B-specific integration rewrite.

Keep:

```text
/rtk_correction_bridge/healthy
/rtk_correction_bridge/correction_age_sec
/rtk_correction_bridge/status
```

But expose which active sink generated that health.

---

## 12.2 Proposed status fields

**PROPOSED — NOT APPLIED**

```json
{
  "injection_mode": "mavros_px4",
  "direct_inject": false
}
```

or:

```json
{
  "injection_mode": "direct_serial",
  "direct_inject": true,
  "direct_serial": {
    "device": "/dev/serial/by-id/...",
    "baud": 230400,
    "open": true,
    "frames_written_total": 12345,
    "bytes_written_total": 2048123,
    "write_failures_total": 0
  }
}
```

Suggested function extension — note the real name is `build_correction_status_snapshot`:

```python
def build_correction_status_snapshot(
    *,
    # current arguments unchanged...
    injection_mode: str,
    direct_serial: Optional[dict] = None,
) -> dict:
    ...
```

⚠ `mavros_subscribers` and `max_mavros_rtcm_frame_bytes` are **required** keyword arguments of this
function today, fed from `self.rtcm_pub.get_subscription_count()` and the configured gate.

**Keep reporting their real values in direct mode.** Do not zero them and do not make them optional.
The subscriber count stays operationally meaningful in both modes because the worker launch gate
depends on it (§6.8), and `max_mavros_rtcm_frame_bytes` still describes the configured legacy gate
that a rollback would re-apply. `injection_mode` is what tells a reader which sink the health refers
to.

---

# 13. Health semantics

## 13.1 Legacy A-side

With:

```text
direct_inject=false
```

retain current health semantics as closely as possible:

```text
success = complete accepted RTCM frame published to MAVROS ROS topic
age     = age since last successful legacy delivery
```

This is required for a clean regression baseline.

---

## 13.2 Direct B-side

With:

```text
direct_inject=true
```

the transport success point changes to:

```text
success = complete RTCM frame fully written through the serial sink API
age     = age since last successful complete serial-frame write
```

A direct serial open/write error must latch/produce unhealthy state through the same high-level health interface.

### Precision of the claim

A successful serial write proves that the Jetson userspace/kernel serial path accepted the complete bytes.

It does **not**, by itself, prove:

- mosaic-H parsed that RTCM frame;
- the RTK engine used it;
- the resulting ambiguity solution is correct.

Receiver solution/fix telemetry remains a separate truth.

---

# 14. File 8 — `package.xml`

Current path:

```text
src/rtk_correction_bridge/package.xml
```

Current runtime dependencies are:

```xml
<depend>rclpy</depend>
<depend>mavros_msgs</depend>
<depend>sensor_msgs</depend>
<depend>std_msgs</depend>
```

If PySerial is selected, proposed addition:

```xml
<exec_depend>python3-serial</exec_depend>
```

Deployment/CI must install the same runtime dependency deterministically.

Do not introduce a serial package in application code without making its deployment dependency explicit.

⚠ **Nothing in `src/` or `scripts/` imports `serial` today — this is a genuinely new dependency on
the Jetson.** And because this workspace is symlink-installed, a missing dependency does **not**
surface at build time; it surfaces as an `ImportError` when the worker process starts, which the
manager will then treat as a repeated worker failure.

Verify on the Jetson before Phase 3, and record the result:

```text
python3 -c "import serial; print(serial.__version__)"
id flash                      # dialout group, or an equivalent udev rule
ls -l /dev/serial/by-id/
```

---

# 15. File 9 — `setup.py`

Path:

```text
src/rtk_correction_bridge/setup.py
```

Current package uses:

```python
packages=find_packages(
    exclude=["test"]
)
```

so the new module:

```text
rtk_correction_bridge/serial_rtcm_sink.py
```

will be included automatically.

No new console script is needed.

Current entry point remains:

```text
ntrip_to_px4
    = rtk_correction_bridge.ntrip_to_px4_node:main
```

Dependency policy should remain consistent with the rover build image. Preferred ROS deployment route is to declare the system dependency in `package.xml`/rosdep rather than create ad-hoc runtime `pip install` behavior.

---

# 16. File 10 — `rtk_worker_bootstrap.py`

Path:

```text
src/rover_backend/rover_backend/rtk_worker_bootstrap.py
```

## Planned status: no functional change

Preserve:

```text
secure worker config FD
parent-liveness guard
worker process supervision
exclusive RTK injection authority lock
ROS node startup
```

The same single worker receives the mode through `WorkerConfig`.

Do **not** spawn:

```text
one MAVROS worker
+
one direct worker
```

That would violate single correction authority.

Correct design:

```text
ONE worker
   ↓
ONE config
   ↓
ONE active sink
```

---

# 17. Configuration example

## 17.1 Safe production default after deployment

```json
{
  "direct_inject": false,
  "direct_serial_device": null,
  "direct_serial_baud": 230400,
  "direct_serial_write_timeout_sec": 1.0,
  "direct_serial_reopen_sec": 1.0
}
```

Result:

```text
EXACT LEGACY MODE INTENT
NTRIP → parser → MAVROS → PX4 → mosaic-H
```

---

## 17.2 Direct test configuration

**Filled in with the actual verified Jetson device identity (2026-09-10) — no longer a placeholder.**
See STATUS above for how it was determined (Mosaic-H serial `3804732`, USB2 interface `if04`).

```json
{
  "direct_inject": true,
  "direct_serial_device": "/dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device_3804732-if04",
  "direct_serial_baud": 230400,
  "direct_serial_write_timeout_sec": 1.0,
  "direct_serial_reopen_sec": 1.0
}
```

This is now the production profile's persisted configuration, not a test-only example — see STATUS
above.

Result:

```text
NTRIP → parser → direct serial → mosaic-H
```

No RTCM should be published through the legacy injection topic in this mode.

---

# 18. mosaic-H / physical-port contract

⚠ **As verified 2026-09-10 (see STATUS at the top of this document), the "SEPARATE mosaic-H correction
COM" below is the receiver's own native USB2 CDC-ACM interface (`if04`, `/dev/ttyACM2`) — a second
logical serial port the mosaic-H already exposes over its single USB cable — not a discrete wired
UART as the port-contract language below was originally written to describe.** The safety intent
(never combine PX4's input with the Jetson's direct-injection output on one receiver RX) is still what
was verified and is still satisfied; only the physical mechanism differs from what this section
originally assumed.

The software patch assumes the following hardware contract.

```text
mosaic-H GNSS/PVT output COM
    → PX4 Septentrio receiver input
    → existing GNSS data path

SEPARATE mosaic-H correction COM
    ← Jetson direct serial RTCM
```

Do not electrically combine PX4 TX and Jetson TX into one receiver RX.

The direct correction COM must be configured persistently in mosaic-H, separately from runtime correction transport.

Expected receiver-side intent:

```text
correction COM:
    RTCMv3 input (or explicitly approved auto input)
    8N1
    matching baud
    no flow control initially
    persistent/save configuration
```

PX4 receiver ownership intent:

```text
SEP_AUTO_CONFIG = 0
```

so PX4 consumes the configured receiver output instead of being the owner of receiver runtime configuration.

Exact mosaic-H persistent command script should be reviewed/applied as a separate receiver-configuration artifact; this Jetson A/B patch must not send those commands itself.

---

# 19. Do not modify these behaviors in Phase 1

The following are intentionally out of scope:

```text
NTRIP authentication
TLS policy
caster selection
mountpoint selection
NTRIP TCP framing
GGA/VRS policy
RTCM3 CRC algorithm
RTCM message decoding
trajectory/origin logic
GNSS fusion
EKF parameters
PX4 Septentrio parsing
MAVROS GNSS telemetry source
frontend mission logic
```

This keeps the experiment about one variable:

```text
How validated correction frames reach mosaic-H.
```

---

# 20. Tests

## 20.1 New test file

Create:

```text
src/rtk_correction_bridge/test/test_serial_rtcm_sink.py
```

Minimum test cases:

### T1 — full successful write

Input:

```text
one valid byte buffer
```

Assert:

```text
exact bytes written
frames_written_total += 1
bytes_written_total += len(frame)
no mutation
```

### T2 — partial serial write

Fake serial returns, for example:

```text
20 B
30 B
remaining B
```

Assert the sink keeps writing until the complete frame is delivered.

### T3 — write timeout/failure

Assert:

```text
failure counter increments
port closes for recovery
exception propagates
no success timestamp update
```

### T4 — initial open failure

Assert:

```text
open failure recorded
no frame reported successful
reopen delay respected
```

### T5 — reconnect

First open/write fails, later attempt succeeds.

Assert full frame is sent only on the successful call.

### T6 — empty frame

Reject locally. Parser should normally prevent this, but sink contract remains defensive.

---

## 20.2 Existing `test_rtcm_transport.py`

Add/retain explicit A/B-boundary tests.

### Legacy 720 test

```text
frame length <=720
    → candidate returned

valid frame length 721..1029
    → valid counter increments
    → oversize counter increments
    → no candidate returned
```

This proves A-side behavior did not change.

### Direct ceiling test

Instantiate transport with:

```text
max frame = 1029
```

Then:

```text
valid 721..1029 frame
    → candidate returned
```

No parser bypass is allowed.

---

## 20.3 Node routing tests

Add tests around `NtripToPx4Node` active sink.

### A-side

```text
direct_inject=false

valid candidate
    → MAVROS publish exactly once
    → SerialRtcmSink never called
```

### B-side

```text
direct_inject=true

valid candidate
    → serial write exactly once
    → MAVROS RTCM publish never called
```

### B-side failure

```text
serial write fails
    → record failure
    → healthy becomes false/latched per policy
    → MAVROS publish count remains ZERO
```

This last assertion is essential.

---

## 20.4 Backend protocol tests

Modify:

```text
src/rover_backend/test/test_rtk_process_protocol.py
```

Test:

```text
schema 4 roundtrip
direct false + device null valid
direct true + valid /dev path valid
direct true + null device rejected
invalid baud rejected
invalid timeout rejected
extra/unexpected keys still rejected
```

---

## 20.5 Profile-store tests

Modify:

```text
src/rover_backend/test/test_rtk_profile_store.py
```

Test:

```text
old DB migrates additively
old profile gets direct_inject=false
create defaults false
update false→true
update true→false
serial settings persist
revision behavior remains correct
password never leaks
```

---

## 20.6 Route tests

Modify:

```text
src/rover_backend/test/test_rtk_routes.py
```

Test:

```text
create request default false
explicit true accepted only with valid persisted settings
profile response exposes mode + non-secret serial settings
partial update works
unknown fields still forbidden
password remains write-only
```

---

## 20.7 Status tests

Modify:

```text
src/rtk_correction_bridge/test/test_status_snapshot.py
```

Test exact mode reporting:

```json
{
  "injection_mode": "mavros_px4",
  "direct_inject": false
}
```

and:

```json
{
  "injection_mode": "direct_serial",
  "direct_inject": true
}
```

---

## 20.8 Lock the deliberate MAVROS-gate behavior

Not blocking, but worth having. §6.8 decides that direct mode keeps the MAVROS readiness gate. That
is a deliberate choice, and an unlabelled deliberate choice eventually gets "fixed" by someone who
reads it as an accidental regression.

Add to:

```text
src/rover_backend/test/test_rtk_manager_core.py
```

Assert explicitly:

```text
direct_inject=true + mavros_ready=false
    → RtkManagerCore stays WAITING_FOR_MAVROS
    → no spawn action is emitted
```

The test name and docstring should say this is intended, and point at §6.8. If a later phase
deliberately decouples direct mode from MAVROS readiness, this test is the thing that must be
changed on purpose — which is exactly the point.

---

# 21. A/B state table

| `direct_inject` | Parser | Frame ceiling after validation | MAVROS RTCM publish | Direct serial | Automatic fallback |
|---|---|---:|---|---|---|
| `false` | existing RTCM3 parser | current legacy configured limit, normally 720 B | **YES** | **NO** | N/A |
| `true` | same existing RTCM3 parser | RTCM3 protocol max, 1029 B | **NO** | **YES** | **NO** |

---

# 22. Mode-switch behavior

A mode change must establish a clean worker/sink lifetime boundary.

The existing backend already provides the required fail-closed behavior.
`RtkProfileStore.update_profile()` compares worker-affecting runtime values.
When an active RUNNING profile changes, it persists:

```text
desired_state = STOPPED
```

`RtkControlService.update_profile()` then observes the RUNNING -> STOPPED
transition and forwards the normal runtime STOP.
For the five direct-injection fields:

```text
direct_inject
direct_serial_device
direct_serial_baud
direct_serial_write_timeout_sec
direct_serial_reopen_sec
```

they must remain included in `old_runtime_values`, so changing any of them
while RUNNING follows this lifecycle:

```text
profile PATCH
    |
new profile values persist
    |
RtkProfileStore detects runtime_changed
    |
persisted desired_state becomes STOPPED
    |
RtkControlService forwards STOP
    |
current worker teardown completes
    |
system remains STOPPED
    |
operator explicitly issues START
    |
new worker receives the new WorkerConfig
    |
exactly one selected injection sink exists
```

There is deliberately no automatic stop-then-start after a profile edit.
This matches the existing safety convention: edited credentials or transport
settings must never silently hot-swap under a running worker. During A/B field
testing, changing injection mode therefore interrupts correction delivery until
the operator explicitly starts RTK again.

Do not implement live sink switching inside a running worker.

---

# 23. Shutdown requirements

Direct mode must close the serial device when:

```text
worker stops
backend parent disappears
ROS shutdown occurs
mode is changed by worker restart
fatal worker exception occurs
```

Proposed node cleanup:

```python
try:
    rclpy.spin(node)
finally:
    if node._serial_sink is not None:
        node._serial_sink.close()

    node.destroy_node()
```

Prefer an explicit node method instead of reaching into a private attribute from `main()`:

```python
def close(self) -> None:
    if self._serial_sink is not None:
        self._serial_sink.close()
```

---

# 24. Logging requirements

At startup, log one unambiguous line:

Legacy:

```text
RTCM injection mode=mavros_px4
topic=/mavros/gps_rtk/send_rtcm
max_frame_bytes=720
```

Direct:

```text
RTCM injection mode=direct_serial
device=/dev/serial/by-id/...
baud=230400
max_frame_bytes=1029
```

Never log NTRIP passwords.

On serial failure:

```text
DIRECT_RTCM_SERIAL_WRITE_FAILED
```

On serial open failure:

```text
DIRECT_RTCM_SERIAL_OPEN_FAILED
```

On recovery:

```text
DIRECT_RTCM_SERIAL_RECOVERED
```

Use stable machine-readable error codes in addition to human-readable log text.

---

# 25. Telemetry naming review

Some current fields are MAVROS-specific, for example:

```text
published_frames
publish_errors
max_mavros_rtcm_frame_bytes
mavros_rtcm_subscribers
```

Do **not** rename all existing fields in the initial A/B patch.

For compatibility:

- retain existing fields;
- add `injection_mode`;
- add a `direct_serial` section;
- document that `published_frames` remains legacy-specific if that is how downstream code currently consumes it.

If a generic delivery count is needed, add a new field:

```text
delivered_frames
delivery_errors
```

rather than silently changing the meaning of an existing public API field.

**State what the retained MAVROS-specific fields read in direct mode**, so no implementer has to
guess and no downstream alert misreads them:

| field | value when `direct_inject=true` |
|---|---|
| `published_frames` / `publish_errors` | count **direct-serial** deliveries, because `attempt_publish()` is the single delivery accountant in both modes (§6.6) |
| `mavros_rtcm_subscribers` | the real subscriber count on the still-advertised RTCM topic — not zero (§6.8) |
| `max_mavros_rtcm_frame_bytes` | the configured legacy gate, unchanged; the transport's effective ceiling is 1029 |

Anything consuming `published_frames` as "MAVROS publications specifically" must read
`injection_mode` alongside it.

---

# 26. Backend telemetry projection review

Current backend RTK telemetry projects correction-stream information separately from GNSS solution information.

Preserve that separation.

Recommended future payload shape:

```json
{
  "correction_stream": {
    "injection_mode": "direct_serial",
    "connected": true,
    "healthy": true,
    "correction_age_sec": 0.18,
    "valid_frames": 1234,
    "delivery_frames": 1234,
    "delivery_errors": 0,
    "direct_serial": {
      "open": true,
      "baud": 230400
    }
  },
  "gnss_solution": {
    "fix_type": 6,
    "rtk_fixed": true
  }
}
```

Do not infer:

```text
correction stream healthy == RTK position definitely correct
```

They remain two different truths.

---

# 27. Implementation phases

## Phase 0 — baseline lock **and hardware verification**

Before editing:

```text
working branch        = feat/rtcm-direct-gnss-uart
working branch HEAD   = 2c5ba33
source-code baseline  = 0a5fac60a56d95ae2da8a8f5938fa539caaa7274
```

The branch has moved past the verified source baseline because **this document is itself committed
on it**. The only intervening change is documentation; no file under `src/` differs between
`0a5fac6` and the current HEAD. Every source excerpt in this plan remains valid.

Confirm that before Phase 1, and re-verify the excerpts if it ever stops being true:

```text
git diff --stat 0a5fac6..HEAD -- src/     # must be empty
```

Record test baseline.

No production behavior change.

### ⚠ Verify the hardware first — it gates every later phase

The port contract in §18 is a prerequisite for Phase 6, but it must be **confirmed here**, before any
code is written. If the second UART is not physically wired, Phases 1–5 are wasted work. Confirm and
record:

```text
a mosaic-H COM port that is free (not the one feeding PX4)
a Jetson UART physically wired to that COM, TX and GND at minimum
the stable device identity: ls -l /dev/serial/by-id/
the receiver accepts RTCMv3 input on that COM, saved to NVRAM
python3-serial present on the Jetson, and device permissions (§14)
```

Paste the real `/dev/serial/by-id/...` string into §17.2 at this point. Do not proceed to Phase 1
with a placeholder.

---

## Phase 1 — schema/persistence only

Modify:

```text
rtk_process_protocol.py
rtk_profile_store.py
rtk_routes.py
tests
```

Add direct fields with:

```text
direct_inject default=false
```

Do not route serial yet.

Acceptance:

```text
all existing RTK tests pass
old DB migration works, tested against a COPY OF THE JETSON'S REAL .db
    (a freshly created DB never exercises the migration branch)
schema-version gate accepts both the old and new user_version
legacy runtime config still generated
changing direct_inject on a running profile forces STOPPED; an explicit START is required
```

---

## Phase 2 — standalone `SerialRtcmSink`

Create:

```text
serial_rtcm_sink.py
test_serial_rtcm_sink.py
```

The sink is not yet selected in default production configuration.

Acceptance:

```text
partial writes
timeouts
disconnect
reopen
full-byte identity
counters
cleanup
```

all tested without hardware using a fake serial object.

---

## Phase 3 — A/B routing

Modify:

```text
ntrip_to_px4_node.py
rtcm_transport.py only as minimally required
node/transport tests
```

Acceptance:

```text
false → legacy only
true  → serial only
never both
no automatic fallback
```

---

## Phase 4 — health/status

Modify:

```text
status_snapshot.py
backend telemetry projection if required
tests
```

Acceptance:

```text
active mode visible
active-sink failures visible
health age based on active delivery point
GNSS solution remains separate
```

---

## Phase 5 — Jetson hardware dry run, A-side first

**STATUS: COMPLETE — PASS (2026-09-10). See STATUS section at the top of this document for the
verified counters.**

Deploy new code with:

```text
direct_inject=false
```

Verify:

```text
current RTCM topic still publishes
current frame counters behave normally
RTK FIX behavior unchanged
serial device is NOT opened
no new direct-serial errors
```

This is the deployment regression gate.

Do not enable direct mode until this passes.

---

## Phase 6 — B-side stationary receiver test

**STATUS: COMPLETE — PASS (2026-09-10). See the STATUS section at the top of this document for the
verified architecture, hardware identity, persisted profile state, and evidence (fix/sats/accuracy
table, transport counters). The "also deliberately test" fault-injection block below (unplug/reconnect,
flip-while-running) was NOT exercised this session — see STATUS for exactly what was and wasn't
covered. Do not read this Phase 6 PASS as also covering Phase 7 (accuracy) — it does not; see the
accuracy boundary in STATUS.**

Configure mosaic-H dedicated correction COM persistently.

Set:

```text
direct_inject=true
```

Verify:

```text
MAVROS correction publication stops
direct serial opens
valid RTCM frames are fully written
serial byte/frame counts advance
correction age remains healthy
mosaic-H RTK FLOAT/FIX behavior occurs
```

Also deliberately test:

```text
unplug direct serial
    → unhealthy
    → NO MAVROS fallback

reconnect
    → direct serial recovers
    → still no dual sink

flip direct_inject while the worker is running
    → worker actually stops and restarts
    → old sink released before the new one opens
    → status injection_mode reflects the new sink
```

And record, rather than test as a defect, that with MAVROS down the worker does **not** start in
direct mode either (§6.8). That is the accepted behavior of this branch, not a bug — but it should
be observed once so nobody rediscovers it during a field session.

---

## Phase 7 — A/B field comparison

**STATUS: NOT PERFORMED.** Phases 5 and 6 (transport/acquisition) are PASS; this phase (accuracy) has
not been run. Do not infer a Phase 7 result from the Phase 6 PASS — see the accuracy boundary in the
STATUS section at the top of this document.

Use the same:

```text
receiver
antennas
surveyed location
caster
mountpoint
network
time window as closely as practical
```

Compare A vs B on **transport** metrics — these are what this change actually affects:

```text
socket bytes
valid RTCM frames
RTCM type counts
oversize drops          ← expected to be the visible difference
delivery frames
delivery errors
correction age
```

### ⚠ Do not judge this change on fix quality metrics

`RTK FLOAT/FIX transitions`, `fix retention`, and `GNSS reported accuracy` were listed here in
earlier revisions. They cannot validate anything about correctness. Per the 2026-09-09 root-cause
work, the receiver reports `eph` = 15 mm and `fix_type` = 6 in both the correct state and the
210 mm-wrong state — the receiver's self-assessment is exactly what fails in this rover's known
failure mode. A B-side run can look perfect on all three and be decimetres wrong.

This is the same point as invariant §9: the direct path is not a fix for the wrong-ambiguity problem,
so it must not be graded as if it were.

If a position check is wanted anyway, use the one **open-loop** channel available: **height at a
revisited surveyed point**, which the rover never controls. And split every dataset on RTK re-fix
boundaries before averaging or differencing anything — a `fix_type` drop below 6 and return can move
the reported position 100–500 mm with no change in `eph`, satellite count, or DOP.

Do not compare absolute coordinates between runs without accounting for normal GNSS/environmental changes.

---

# 28. Acceptance criteria

The A/B implementation is ready for production field evaluation only when all are true.

**Status key (added 2026-09-10):** `[x]` = confirmed by the 2026-09-10 field session (see STATUS at
the top of this document) or a direct, unambiguous consequence of its results; `[ ]` retained with an
explicit reason = implemented but not independently re-exercised against live hardware this session —
do not read an unchecked item as failing, only as not yet field-evidenced.

- [x] `direct_inject` defaults to `false`. — confirmed: factory default for a new profile is unchanged (STATUS).
- [ ] Existing profiles migrate to `false` automatically. — not exercised this session.
- [x] **A copy of the Jetson's real RTK database opens on the new schema version** — the accepted
      `user_version` set includes the old version (§10.3). — confirmed: the live Jetson profile's
      persisted revision advanced 7→8 during this session on the real production database (STATUS).
- [x] **Changing `direct_inject` on a running profile forces a controlled worker stop/start** (§22). —
      field-confirmed; see STATUS, "why the first B-side attempt appeared broken."
- [x] `false` does not open the direct serial device. — A-side PASS (STATUS).
- [x] `false` retains the current MAVROS RTCM topic. — A-side PASS (STATUS).
- [x] `false` retains the current legacy max-frame behavior. — A-side PASS, effective 720 B limit
      confirmed (STATUS).
- [x] `true` never publishes correction data through MAVROS. — B-side PASS: MAVROS RTCM topic
      remained silent throughout direct mode (STATUS).
- [ ] `true` accepts complete valid RTCM3 frames up to 1029 B. — not independently re-measured against
      live hardware this session (Phase 2/3 unit-test coverage only).
- [ ] `true` writes exact frame bytes, in order, without mutation. — not independently re-verified at
      the byte level this session (unit-test coverage only).
- [ ] Partial serial writes are completed correctly. — unit-test coverage only, not field-exercised.
- [ ] Serial timeout/disconnect cannot cause a hidden MAVROS fallback. — exclusivity confirmed under
      normal operation this session (no MAVROS traffic during a successful direct-mode run); the
      disconnect/reconnect fault injection itself was not exercised — see STATUS.
- [x] Only one RTK worker/injection authority exists. — confirmed: no dual routing observed during the
      B-side PASS run (STATUS).
- [ ] Direct serial resources close on worker shutdown. — not exercised this session.
- [ ] Health identifies the active injection mode. — implemented (Phase 4); status-payload content was
      not specifically queried in this field session.
- [ ] Direct serial delivery failure makes the correction path unhealthy. — not exercised; zero write
      failures occurred during this session's PASS run.
- [x] NTRIP/GGA/TLS behavior is regression-tested. — both the A-side and B-side PASS runs exercised the
      same live NTRIP/GGA/TLS client upstream of the sink split.
- [ ] Receiver RTK solution remains reported independently from correction transport health. — design
      invariant per §13.2/§26; not specifically re-queried this session.
- [ ] A-side rollback requires only configuration + controlled worker restart/reconcile. — not
      exercised this session (the production profile was not rolled back to A-side after the B-side
      PASS).
- [x] No current MAVROS/PX4 correction code has been deleted. — confirmed: A-side is unchanged and
      still independently PASS (STATUS).
- [x] `python3-serial` is installed on the Jetson and the device is accessible to the worker user
      (§14). — confirmed: the worker opened `/dev/ttyACM2` and wrote frames successfully (STATUS).
- [ ] The serial device is opened exclusively (§7.4). — not fault-tested this session (no attempt was
      made to open a second competing writer against the same device).
- [x] Documentation and status output do **not** claim the B-side removes the MAVROS dependency (§6.8).
      — confirmed by this revision's own accuracy-boundary and MAVROS-readiness-gate framing above.

---

# 29. Rollback contract

Software rollback during field testing:

```text
direct_inject = false
```

Then perform a controlled worker restart/reconciliation.

Expected restored path:

```text
NTRIP
 → existing RTCM3 parser
 → existing <=720 B legacy policy
 → mavros_msgs/RTCM
 → /mavros/gps_rtk/send_rtcm
 → MAVROS
 → PX4
 → mosaic-H
```

No code revert should be necessary to return to A-side behavior.

---

# 30. Proposed first implementation diff scope

For the first commit, keep scope small.

### New

```text
src/rtk_correction_bridge/rtk_correction_bridge/serial_rtcm_sink.py

src/rtk_correction_bridge/test/test_serial_rtcm_sink.py
```

### Modify

```text
src/rtk_correction_bridge/rtk_correction_bridge/ntrip_to_px4_node.py

src/rtk_correction_bridge/rtk_correction_bridge/status_snapshot.py

src/rtk_correction_bridge/package.xml

src/rover_backend/rover_backend/rtk_process_protocol.py

src/rover_backend/rover_backend/rtk_profile_store.py

src/rover_backend/rover_backend/rtk_routes.py

relevant existing tests
```

The existing `rtk_control_service.py` requires no behavioral change for this
feature. The store owns the worker-affecting comparison and persists STOPPED;
the control service already forwards that forced RUNNING -> STOPPED transition.
See §22.

### Documentation-only change

```text
src/rtk_correction_bridge/rtk_correction_bridge/rtcm_transport.py
```

**No functional code change** — see §5 and §8. Only the module docstring's description of the 720 B
gate is edited. It is listed separately here so it is not counted as a behavioural diff during
false-mode regression review.

### Do not modify unless a verified dependency appears

```text
PX4 firmware
MAVROS source
mission/RPP code
trajectory code
frontend code
rtk_worker_bootstrap.py
NTRIP parser algorithm
```

---

# 31. Recommended commit sequence

Keep reviewable commits rather than one large patch.

```text
Commit 1
rtk: add persisted direct-injection A/B configuration

Commit 2
rtk: add production serial RTCM sink

Commit 3
rtk: route validated RTCM frames through selected injection sink

Commit 4
rtk: expose active injection mode and serial delivery health

Commit 5
rtk: add A/B regression and fault-injection coverage
```

Do not delete the legacy path after B-side testing in this branch.

---

# 32. Reviewer checklist

The reviewer should specifically challenge these questions:

1. Can any execution path deliver one RTCM frame to both MAVROS and direct serial?
2. Can `direct_inject=true` ever automatically fall back to MAVROS?
3. Does deploying the new DB schema activate direct mode on an existing rover?
4. Does a 721–1029 B valid RTCM frame remain rejected in legacy mode but accepted in direct mode?
5. Is every byte of a direct frame written before success is recorded?
6. Does serial failure update correction health without falsely claiming receiver acceptance?
7. Does mode switching close the old sink before creating the new worker lifetime?
8. Is the actual serial device stable across reboot (`/dev/serial/by-id/...`)?
9. Is the selected mosaic-H correction COM physically separate from PX4's GNSS output COM?
10. Is the receiver configured persistently so this Jetson code sends correction bytes only?
11. Are NTRIP credentials still protected exactly as before?
12. Are current public API/status fields preserved rather than silently redefined?
13. Can an operator restore current behavior by setting only `direct_inject=false` and restarting/reconciling the worker?
14. Are no PX4/MAVROS correction capabilities removed?
15. Does an existing rover's RTK database still open after the schema bump, tested on a real copy?
16. Does flipping `direct_inject` on a *running* worker actually change the sink, or only the row?
17. Is the RTCM publisher still created in direct mode, so the MAVROS-readiness launch gate can be satisfied?
18. Is delivery still accounted for in exactly one place — `attempt_publish()` — rather than duplicated in the node?
19. Does any A/B conclusion rest on `fix_type`, `eph`, or DOP? It must not (§27, Phase 7).

---

# 33. Final proposed contract

```text
                    SHARED, UNCHANGED CORE
                    ----------------------
                    backend RTK authority
                    persisted RTK profile
                    NTRIP client
                    TLS
                    GGA
                    RTCM3 parser
                    CRC validation
                    process supervision
                    injection lock
                             │
                             ▼
                   COMPLETE VALID RTCM3
                             │
                             ▼
                    ONE A/B DECISION
                             │
             ┌───────────────┴───────────────┐
             │                               │
     direct_inject=false             direct_inject=true
             │                               │
             ▼                               ▼
       CURRENT SYSTEM                    NEW SYSTEM
       MAVROS → PX4                  SerialRtcmSink
          → mosaic-H                    → mosaic-H
             │                               │
             └──────────── NEVER BOTH ───────┘
```

**Production principle:** add the direct path beside the current path, prove it, compare it, and retain immediate A-side rollback. Do not remove the existing RTCM injection path during this work.
