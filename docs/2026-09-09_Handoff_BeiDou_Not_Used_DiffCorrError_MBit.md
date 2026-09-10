# Handoff — BeiDou is licensed, tracked, and correctly delivered, but the receiver flags a live `DiffCorrError` ("0 MBit anomaly") and never uses it

**Date:** 2026-09-09 · **Branch:** `feat/rtk-injection-v2`
**Continues:** [2026-09-08_Handoff_Septentrio_BeiDou_Wrong_Fix.md](2026-09-08_Handoff_Septentrio_BeiDou_Wrong_Fix.md),
[2026-09-09_Handoff_RTK_Wrong_Ambiguity_Fix_Root_Cause.md](2026-09-09_Handoff_RTK_Wrong_Ambiguity_Fix_Root_Cause.md).
Both prior docs treated "why does BeiDou sometimes go unused" as a standing
open question. This session traced it end to end, live, in one sitting — base
→ Jetson → PX4 → receiver → PVT — rather than reconstructing it after the
fact from logs, and found the actual mechanism.

**Context**: same session also fixed two unrelated pre-existing receiver
misconfigurations (`spm` stuck in `Static` instead of `Rover`; `Stream1`
bound only to the diagnostic USB port, never to the FCU-facing `COM3`) that
were causing "PX4 never receives position" — those are real bugs, now fixed
and persisted to `Boot`/`User1`, but they are **not** the cause of what
follows. This finding held before and after both fixes.

## 1. The result

With RTK genuinely fixed (`Mode=4`, `fix_type=6`, `satellites_used=10`) and
using only GPS+GLONASS+Galileo, a 5-minute live capture on this session's
manual receiver configuration showed:

```
PVT epochs: 304/304 RTKfixed
BeiDou tracked: 30 distinct satellites, elevations up to 73 deg
BeiDou used in PVT: 0.0% -- every signal (B1I/B1C/B2I/B2a/B2b/B3I), every
                     satellite, every epoch sits at PVTStatus code 0
                     ("not-used"). Never code 1 (waiting for ephemeris),
                     never code 3 (rejected).
```

Every configuration-level gate that could plausibly explain this was checked
live and found open:

| Gate | Live result |
|---|---|
| `lif, Permissions` (BeiDou license) | `bdsb1/bdsb2/bdsb3 = 1, Permitted` |
| Satellite-level usage (`gsu`) | `C01...C51` (BeiDou PRNs) present in the usage list |
| Signal-level tracking (`gnt`) | `BDSB1I+BDSB2I` present (B3I absent — not tracked, but irrelevant since B1I alone should be usable) |
| Signal-level usage (`gnu`) | `BDSB1I+BDSB2I+BDSB3I` present in both the PVT and NavData usage lists |
| RF tracking | Working — 30 satellites, real elevation/azimuth via `SatVisibility`/`ChannelStatus` |

None of that is the blocker. The blocker is upstream of all of it, in the
receiver's own correction-availability accounting.

## 2. The base is correctly sending BeiDou corrections — verified live, not inferred

Tapped `/mavros/gps_rtk/send_rtcm` directly on the Jetson (the actual
publisher `ntrip_to_px4_node` uses to hand RTCM to MAVROS→PX4→receiver) for
15 s:

```
82 messages, 0 CRC failures
1230 x14  1005 x14  1074 x14 (GPS,12sv)  1084 x14 (GLONASS,6sv)
1094 x13 (Galileo,6sv)  1124 x13 (BeiDou MSM4, 6sv)
```

BeiDou (1124) arrives at the same ~1 Hz cadence as every other constellation,
every message CRC-valid, 6 satellites populated. This rules out the base,
the NTRIP bridge, the Jetson, PX4, and the cabling — the correct bytes are
provably arriving at the receiver's door.

## 3. The receiver's own accounting still says BeiDou isn't available

`BaseVectorGeod.SignalInfo` (4028) — *"signals for which differential
corrections are available from the base"* — decoded live, same time window:

```
SignalInfo = 0x00220909 = GPS L1CA+L2C + GLONASS L1CA+L2CA + Galileo E1+E5b
BeiDou bits: NONE set
Mode = 4 (RTK fixed)
```

Identical mask to every prior capture in this project's history
(2026-09-08 doc, 2026-09-09 doc, and the 2026-09-09 `log_56` re-analysis
earlier this session). This is the first time it's been captured
**simultaneously** with proof that the corresponding RTCM message was
correctly delivered in the same window — closing the gap between "arrived"
and "counted as available."

## 4. The mechanism: a live, recurring `DiffCorrError`

`lif, DiffCorrError` — *"Last detected anomalies in the incoming
differential correction streams"* (the receiver's own official diagnostic
for exactly this class of problem, per the Reference Guide's
`ReceiverStatus.ExtError` bit 1 description) — returned, live, at the same
session:

```
DiffCorr-error 1: T=1472992438  0 MBit anomaly
DiffCorr-error 1: T=1472992437  0 MBit anomaly
DiffCorr-error 1: T=1472992438  0 MBit anomaly
  (repeated, recent, ongoing)
```

⚠ **The Reference Guide documents that this internal file exists and what
category of problem it reports, but does not define an error-code table for
individual entries** — "0 MBit anomaly" is not decoded anywhere in the
manual. The interpretation below is a well-grounded hypothesis from RTCM3
protocol knowledge, not a manual-confirmed fact, and should be labeled as
such in any further write-up:

RTCM3 MSM messages (1074/1084/1094/1124) carry a **Multiple Message Bit**
(M-bit) in their header, used to chain several MSM messages covering the
*same* observation epoch (one per constellation) together, so a receiver
knows where one epoch's group of messages ends. In this base's message
sequence, BeiDou (1124) is consistently the **last** MSM4 message of the
group (`1230, 1005, 1074, 1084, 1094, 1124`). If the base's RTCM encoder
sets the M-bit sequence inconsistently across that chain — most plausibly on
the final (1124) message, or on the messages that are supposed to signal
"more follow" before it — a receiver enforcing correct M-bit chaining could
flag the group as anomalous and decline to credit part or all of it as valid
differential correction, **without** that showing up as a per-message CRC
failure (each 1124 message is individually well-formed and CRC-valid; the
anomaly is in the cross-message epoch-assembly logic, not the message
itself). That would produce exactly what's observed: CRC-clean 1124 arriving
on schedule, tracking and usage gates all open, and yet
`BaseVectorGeod.SignalInfo` never crediting it.

## 5. What this rules out and what it doesn't

**Ruled out, with live evidence, this session:**
- BeiDou licensing (`lif,Permissions` — fully permitted)
- Satellite/signal-level tracking or usage configuration (all open)
- RF signal quality / antenna (BeiDou tracks fine, elevations to 73°)
- The base not sending BeiDou corrections (it is, CRC-valid, on schedule)
- Anything in this project's own stack — Jetson, NTRIP bridge, PX4, cabling,
  `SEP_*` parameters (all confirmed correct or provably inert to this)

**Not yet proven:**
- The exact meaning of "0 MBit anomaly" (manual doesn't define it — would
  need Septentrio support or `lif,Debug`/deeper internal diagnostics to
  confirm)
- Whether this is a caster/base RTCM encoder bug, a base-software version
  issue, or something specific to how this particular base/mountpoint
  packages multi-constellation epochs
- Whether the same base exhibits this for every session or only some (this
  session's capture is a single ~5 minute window)

## 6. What to do

### 6.1 Firmware upgrade — the most concrete, officially-documented candidate fix

**This receiver runs `4.14.0` live** (confirmed via NSH `ver all` and the
receiver's own `SEPT MOSAIC-H` SBF identification string). Septentrio's own
Release Notes for firmware **4.14.10** (`mosaic-H Firmware Package v4.14.10.1
Release Notes`, digikey-hosted PDF, `mm.digikey.com/.../410352_Installation_Guide.pdf`)
list, under "Improvements in version 4.14.10":

> *"Fixed interoperability issue with some NTRIP casters when receiving
> burst of data."*

This is not confirmed to be identically the "0 MBit anomaly" — the release
notes don't name `DiffCorrError` or the M-bit specifically — but it is the
closest, most concrete, officially-documented fix in the exact version gap
between this unit's live firmware and the reference guide's version, and it
sits squarely in the same problem area (an anomaly in processing a *burst*
of caster-delivered correction data, not a per-message CRC failure).
**Recommended first step: upgrade this receiver to at least `4.14.10` (or
current `4.14.10.1`), then repeat this exact live test** (`lif,
DiffCorrError` + `BaseVectorGeod` + the `/mavros/gps_rtk/send_rtcm` tap) —
if the error stops recurring and BeiDou starts being credited, that
confirms this was the mechanism, without needing to involve the base
operator at all.

### 6.2 If the firmware upgrade doesn't resolve it

Then the cause is more likely base/caster-side, not receiver firmware:

1. Ask the caster/base operator whether their RTCM3 encoder correctly sets
   the Multiple Message Bit across a multi-constellation epoch group, and
   whether other rovers connected to the same mountpoint see BeiDou used.
2. If a second base or a different mountpoint is available, repeat this
   exact test against it — if `DiffCorrError` stops firing and BeiDou starts
   being credited, that confirms the base as the cause instead.

### 6.3 Regardless of outcome

3. Keep `SEP_DUMP_COMM=3` and the diagnostic streams (`Stream2`/`Stream3`/
   `Stream4` on `USB1`, added this session — `ChannelStatus`,
   `SatVisibility`, the `PVTGeodetic` family, `BaseVectorGeod`) in place for
   future sessions; they're what made this traceable in one sitting instead
   of across three days of log archaeology.
4. If Septentrio support is engaged, hand them the exact `lif,DiffCorrError`
   output above plus the RTCM byte capture — that is precisely the pairing
   their own diagnostic file is meant to support.

## 7. Tooling note

`scripts/gps_dump_sbf_report.py` was extended this session with verified,
size-checked decoders for `ChannelStatus` (4013, two-level sub-blocks) and
`SatVisibility` (4012), sourced from the mosaic-H Reference Guide v4.14.10
section 4.1.9/4.1.10 and cross-checked against receiver-declared
`SBLength`/`SB1Length`/`SB2Length` at decode time, per this repo's standing
rule never to hand-roll SBF offsets. `BaseVectorGeod` (4028) decoding already
existed. These are now reusable for any future live or logged capture.
