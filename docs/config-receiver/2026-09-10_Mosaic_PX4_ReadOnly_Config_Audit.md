# 2026-09-10 — Read-only live configuration audit: Septentrio Mosaic-H + PX4

Evidence-gathering pass only. **No changes were made to the Mosaic-H or to
PX4.** Every command sent to either device below is a read/list command
(`get*`, `lst*`, `help`, `ver`, `ps`, `gps status`, `septentrio status`,
`listener`, `param show`). No `set*`, `param set`, `param save`,
`CopyConfigFile`, reset, reboot, or restart of any production node was
issued at any point. USB2 (the live production RTCM injection path) was
never opened.

Access path: SSH `flash@192.168.3.101` → `/home/flash/rover_ws`. Mosaic-H
queried over **USB1 only** (`/dev/ttyACM1`,
`usb-Septentrio_Septentrio_USB_Device_3804732-if02`) via a read-only pyserial
session. PX4 queried through the existing NSH-over-SSH transport documented
in [`docs/PX4_NSH_OVER_SSH.md`](../PX4_NSH_OVER_SSH.md) (MAVROS
`SERIAL_CONTROL` over `/uas1/mavlink_sink` / `/uas1/mavlink_source`, PX4
NSH read commands only).

## Port mapping and ownership check (before touching anything)

```
command: readlink -f .../if02 , readlink -f .../if04 , fuser -v /dev/ttyACM1 /dev/ttyACM2
exact response: if02 -> /dev/ttyACM1 (USB1), if04 -> /dev/ttyACM2 (USB2)
                fuser: /dev/ttyACM2: flash 8805 F.... python3   (ttyACM1: no holder)
meaning: USB2 is actively held by the live RTCM sink (pid 8805) exactly as expected. USB1 was free, so it was used for every query below. USB2 was never opened.
```

---

## A. Mosaic-H live configuration (via USB1, receiver serial `3804732`)

### Current vs Boot configuration

```
command: lcf, Current
exact response:
  # Configuration File "Current"
  # Different from RxDefault:
  setDataInOut, USB2, RTCMv3
  setDataInOut, USB2, , none
  setSBFOutput, Stream1, COM3
  setSBFOutput, Stream1, , AuxAntPositions+AttEuler+BaseVectorGeod+PVTGeodetic+PosCovGeodetic+VelCovGeodetic+DOP+EndOfPVT+ReceiverStatus+AttCovEuler+EndOfAtt+PVTSupport+PosLocal+PVTSupportA
  setSBFOutput, Stream1, , , msec100
  setCOMSettings, COM3, baud921600
  setGNSSAttitude, , Float+Fixed

command: lcf, Boot
exact response: byte-for-byte identical to the Current listing above.
meaning: Current and Boot (persistent) configuration are IDENTICAL. Only 6 settings differ from receiver factory default anywhere on this receiver, and none of them have drifted between what's running now and what will load on the next power cycle. No uncommitted/at-risk config exists.
```

### 1–2. Constellations/signals and input configuration

```
command: getSignalUsage
exact response: SignalUsage, GPSL1CA+GPSL2PY+GPSL2C+GLOL1CA+GLOL2CA+GALL1BC+GALE5b+GEOL1+BDSB1I+BDSB2I+BDSB3I+QZSL1CA+QZSL2C+QZSL1CB, GPSL1CA+GLOL1CA+GLOL2CA+GALL1BC+GALE5b+GEOL1+BDSB1I+BDSB2I+BDSB3I+QZSL1CA+QZSL1CB
meaning: first list = signals tracked, second = signals actually used in PVT. GPS, GLONASS, Galileo, SBAS, BeiDou (B1I+B2I+B3I), QZSS are all enabled and in active use right now. BeiDou usage is NOT degraded at the moment of this query — all 3 BeiDou signals present.

command: getSatelliteUsage
exact response: 32 GPS PRNs + 24 GLONASS + 36 Galileo + 63 BeiDou + 7 QZSS all listed as usable (full constellation, no per-satellite exclusions).
meaning: no satellite is blacklisted.

command: lif, Permissions
exact response (relevant lines): gpsl1py=0(Not permitted), all other GPS/GLONASS/Galileo/BeiDou bands=1(Permitted) except gale6a/gale6bc/gale5a=0, res1/irnl5/irn_s/qzsl6=0; nb_ant=2; rtkrover=1; movingbase=1; datarate=100Hz; lband=0
meaning: this is the license file, receiver serial 3804732. BeiDou B1/B2/B3 are licensed (not the earlier session's concern). L-Band augmentation and INS are NOT licensed (irrelevant to current RTK-only usage).
```

### 3–4. RTK/RTCM input and USB1/USB2 I/O

```
command: getDataInOut, USB2
exact response: DataInOut, USB2, RTCMv3, none, (on)
meaning: matches the expected production config exactly — USB2 accepts RTCMv3 input, outputs nothing, port is on. Re-verified live, not assumed.

command: getDataInOut, USB1
exact response: DataInOut, USB1, auto, SBF+NMEA, (on)
meaning: USB1 auto-bauds, outputs SBF+NMEA (this is the diagnostics port we used).

command: getDataInOut, COM1 / COM2 / COM3
exact response: all three: auto, SBF+NMEA, (on)
meaning: COM3 is the physical link to the Pixhawk (confirmed baud921600 in the Current/Boot dump above, and independently from the PX4 side below). COM1/COM2 are unused spare physical serial ports, same generic default.

command: getRTCMv3Usage
exact response: all RTCM3 message types 1001-1230 listed as usable, no restriction.
meaning: receiver accepts the full RTCM3.x message set from whatever's injected on USB2.

command: getDiffCorrUsage
exact response: DiffCorrUsage, LowLatency, 3600.0, auto, 0, off
meaning: max correction age 3600s (receiver-side ceiling, very permissive — the real age gate is enforced upstream by rover software, e.g. trajectory_generator's max_correction_age_sec: 2.0), auto-detect correction type, no explicit DGPS timeout override.
```

### 5. SBF/NMEA output streams

```
command: getSBFOutput, all
exact response: Stream1 -> COM3, {AuxAntPositions+AttEuler+BaseVectorGeod+PVTGeodetic+PosCovGeodetic+VelCovGeodetic+DOP+EndOfPVT+ReceiverStatus+AttCovEuler+EndOfAtt+PVTSupport+PosLocal+PVTSupportA}, msec100 (10 Hz). Streams 2-10 and Res1-4: none/off.
meaning: only ONE SBF output stream exists, and it goes to COM3 (the Pixhawk link) at 10 Hz. Nothing is streamed to USB1 or USB2 (USB1's SBF+NMEA "on" from getDataInOut is the port's I/O capability flag, not an active configured stream — getSBFOutput shows no stream is actually routed there). This matches the CLAUDE.md architecture note (dedicated PX4 SBF driver, 10 Hz PVT).

command: getNMEAOutput, all
exact response: all 10 streams: none/off
meaning: no NMEA output is configured anywhere on the receiver.
```

### 6. Receiver dynamics / navigation mode

```
command: getReceiverDynamics
exact response: ReceiverDynamics, Moderate, Automotive
meaning: receiver's own motion model is "Automotive" — this is receiver-internal navigation-filter tuning, separate from PX4's EKF2. Not something PX4 configures (confirmed below — no antenna/dynamics `s*` command exists in the PX4 driver source).

command: getPVTMode
exact response: PVTMode, Rover, StandAlone+SBAS+DGNSS+RTKFloat+RTKFixed, auto
meaning: receiver operates as an RTK Rover, all fix-type fallbacks enabled, mode auto-selected by best available.

command: getGNSSAttitude
exact response: GNSSAttitude, MultiAntenna, Float+Fixed
meaning: dual-antenna heading enabled, accepts both float and fixed ambiguity solutions for attitude (matches AttEuler/AttCovEuler in the SBF output stream).
```

### 7. Antenna configuration/offsets

```
command: getAntennaOffset
exact response: AntennaOffset, Main, 0.0000, 0.0000, 0.0000, "Unknown", "Unknown"
                AntennaOffset, Aux1, 0.0000, 0.0000, 0.0000, "Unknown", "Unknown"
meaning: receiver-side antenna lever arm is zero/unset for both antennas, and antenna type is "Unknown" (no ARP/PCV model loaded) — this matches the 2026-09-08 finding that PX4 never configures antenna offset/type and it's receiver NVRAM. Consistent with prior evidence.

command: getAntennaLocation
exact response: AntennaLocation, Aux1, auto, 0,0,0 / AntennaLocation, Base, auto, 0,0,0
meaning: no manual antenna-to-marker geometry set either.

command: lai, Main / lai, Aux1
exact response: <Antenna ID="UNKNOWN"/>, all phase-center-variation table values 0.0 for every elevation, L1 and L2
meaning: no antenna calibration (PCV) profile is loaded for either antenna — confirms "Unknown" antenna type is not just a label, there is genuinely no PCV correction model active.
```

### 8. Ambiguity/fix-related and other settings

```
command: getElevationMask
exact response: ElevationMask, Tracking, 0 / ElevationMask, PVT, 0
meaning: no elevation cutoff — every satellite above the horizon is tracked and eligible for PVT.

command: getGeodeticDatum
exact response: GeodeticDatum, Default
meaning: WGS84/default datum, no custom transform.
```

### 9. Identification

```
command: getReceiverCapabilities
exact response: ReceiverCapabilities, Main+Aux1, GPSL1CA+GPSL2PY+GPSL2C+GLOL1CA+GLOL2CA+GALL1BC+GALE5b+GEOL1+BDSB1I+BDSB2I+BDSB3I+QZSL1CA+QZSL2C+QZSL1CB, DSK1+COM1..4+USB1+USB2+IP10-17+NTR1-3+..., SBAS+DGNSSRover+DGNSSBase+RTKRover+RTKBase+RTCMv23+RTCMv3x+...+MovingBase+..., 10, 10
meaning: hardware capability inventory — dual-antenna, all major GNSS bands supported, RTK rover+base capable, moving-base capable.

command: getReceiverInterface
exact response: ReceiverInterface, RxName, "mosaic"; SNMPVersion, "20230404"
meaning: this is the closest ASCII-queryable identity string — receiver model family "mosaic" (matches mosaic-H). No dedicated firmware-version ASCII getter exists on this firmware — `getReceiverSetup`, `getVersion`, `getFirmwareVersion`, `getReceiverType`, `getRxID` all returned "Invalid command!" (tried and ruled out, not guessed). Exact firmware build string was not obtained via USB1 ASCII in this pass; it would require either the receiver web UI or a dedicated `lif,` request we did not have the correct filename for (`lif, AntennaInfo` errored "Argument 'File' is invalid" — `lif` needs an exact internal filename, and the valid name for firmware version wasn't discovered in this pass).
```

### 10. Disk/logging (read-only)

```
command: getNtripSettings
exact response: NTR1/NTR2/NTR3 all: off
meaning: the receiver's own onboard NTRIP client is OFF on all 3 slots — confirms corrections arrive only via the injected USB2 RTCMv3 stream from the Jetson, never from a receiver-side NTRIP client. No redundant/competing correction path exists on the receiver itself.

command: ldi, DSK1
exact response: <Disk name="DSK1" total="31706841088" free="27411546112"><File name="log.sbf" size="4294967295"/></Disk>
meaning: ~31.7 GB internal disk, ~27.4 GB free, one active/rolling log.sbf file present (size shown is the max/reported cap, not necessarily actual bytes written).
```

---

## B. PX4 live GNSS configuration (via NSH-over-SSH)

```
command: ver all
exact response: PX4 git-hash 54f0455ffcd755534539a7cf33a09a20bf71d29d, Release 1.16.2, HW PX4_FMU_V6X, Build Jul 25 2026
meaning: confirms firmware identity matches all prior documentation — no firmware drift.
```

### 1. GPS driver / port ownership

```
command: ps
exact response (relevant line): 567 567 205 FIFO Task ... Waiting Semaphore ... septentrio start -d /dev/ttyS4 -b p:SER_TEL2_BAUD

command: septentrio status
exact response:
  INFO  [septentrio] Main GPS
  INFO  [septentrio] health: OK, port: /dev/ttyS4, baud rate: 921600
  INFO  [septentrio] controller -> receiver data rate: 0 B/s
  INFO  [septentrio] receiver -> controller data rate: 6142 B/s
  INFO  [septentrio] sat info: disabled
  INFO  [septentrio] rate RTCM injection:   0.00 Hz

command: param show SER_TEL2_BAUD
exact response: SER_TEL2_BAUD : 921600

command: gps status  (the GENERIC gps module, for comparison)
exact response: INFO [gps] not running
meaning: this rover runs the dedicated `septentrio` driver module, not the generic PX4 `gps` module — `gps status` correctly reports nothing, that's expected and not an error. The septentrio driver owns /dev/ttyS4 (the FMU's TELEM2 UART) at 921600 baud, which is wired to the receiver's physical COM3 port (getDataInOut/COM3 = auto/SBF+NMEA/on, Current/Boot config shows `setCOMSettings, COM3, baud921600` — the two sides' baud independently match). Health OK, 10 satellites used, fix_type 6. "controller -> receiver: 0 B/s" proves PX4 sends nothing to the receiver over this link — no configuration commands, no RTCM — consistent with SEP_AUTO_CONFIG=0 and the direct-USB2 injection architecture. "rate RTCM injection: 0.00 Hz" on THIS port confirms corrections do not flow through the FCU/MAVROS link at all; they only arrive via USB2 from the Jetson directly, matching the 2026-09-10 architecture note that `/mavros/gps_rtk/send_rtcm` stays silent.
```

### 2. Automatic receiver configuration — the main question

```
command: param show SEP*
exact response:
  x + SEP_AUTO_CONFIG [714,1176] : 0
  x   SEP_CONST_USAGE [715,1177] : 0
  x + SEP_DUMP_COMM   [716,1178] : 3
  x   SEP_HARDW_SETUP [717,1179] : 0
  x   SEP_LOG_FORCE   [718,1180] : 0
  x   SEP_LOG_HZ      [719,1181] : 0
  x   SEP_LOG_LEVEL   [720,1182] : 2
  x   SEP_OUTP_HZ     [721,1183] : 1
  x   SEP_PITCH_OFFS  [722,1184] : 0.0000
  x + SEP_PORT1_CFG   [723,1185] : 102
  x   SEP_PORT2_CFG   [724,1186] : 0
  x   SEP_SAT_INFO    [725,1187] : 0
  x   SEP_STREAM_LOG  [726,1188] : 2
  x   SEP_STREAM_MAIN [727,1189] : 1
  x   SEP_YAW_OFFS    [728,1190] : 0.0000
meaning: SEP_AUTO_CONFIG = 0 — auto-configuration is DISABLED on this firmware, read directly, not from memory/history. This is the exact production-target state described in the task ("PX4 does NOT automatically reconfigure the receiver"). SEP_CONST_USAGE = 0 means PX4 also never sends the `ssu` (setSatelliteUsage) command even if auto-config were on. SEP_DUMP_COMM = 3 confirms the raw SBF/RTCM capture used by `gps_dump_sbf_report.py` is still active. SEP_SAT_INFO = 0 matches the driver's own reported "sat info: disabled". SEP_PORT2_CFG = 0 confirms no second Septentrio port/instance is configured (single-receiver setup, no blending).
```

### 3. GNSS-related PX4 parameters

```
command: param show GPS_1*
exact response: GPS_1_CONFIG [333,687] : 0

command: param show GPS_2*
exact response: GPS_2_CONFIG [334,690] : 0
meaning: GPS_1_CONFIG/GPS_2_CONFIG (PX4's generic GPS-port-mapping parameter, used by the generic `gps` module) are both 0/disabled — expected, because this rover's GNSS input is owned entirely by the dedicated `septentrio` module started via its own SEP_PORT1_CFG=102, not through the generic GPS port-selection mechanism.

command: param show SENS_GPS*
exact response: SENS_GPS_MASK [704,1155] : 7 ; SENS_GPS_PRIME : 0 ; SENS_GPS_TAU : 10.0000
meaning: SENS_GPS_MASK=7 is PX4's generic multi-GPS output-blending mask (bitmask, all 3 bits set = default "use whatever GPS instances are present"), but with only one GPS driver instance running (septentrio) there is nothing to blend against — this is a default value, not evidence of active blending.

command: param show EKF2_GPS*
exact response:
  EKF2_GPS_CHECK  : 829
  EKF2_GPS_CTRL   : 15
  EKF2_GPS_DELAY  : 50.0000
  EKF2_GPS_POS_X  : 0.0000
  EKF2_GPS_POS_Y  : 0.0000
  EKF2_GPS_POS_Z  : -0.3000
  EKF2_GPS_P_GATE : 5.0000
  EKF2_GPS_P_NOISE: 0.0100
  EKF2_GPS_V_GATE : 5.0000
  EKF2_GPS_V_NOISE: 0.3000
  EKF2_GPS_YAW_OFF: 0.0000
meaning: matches the 2026-09-05 corrected baseline exactly (P_NOISE=0.01, not the stale 0.5) — re-verified live, not from memory. EKF2_GPS_POS_X/Y/Z = 0/0/-0.3 confirmed live (matches the 2026-09-09 NSH re-confirmation already on file).
  ⚠ EKF2_GPS_YAW_OFF = 0.0000 right now. This contradicts the CLAUDE.md 2026-08-31 live-capture table, which recorded "GPS_YAW_OFFSET=180.0". Either that value has changed since 2026-08-31, or "GPS_YAW_OFFSET" in that table referred to something else — there is no PX4 parameter literally named GPS_YAW_OFFSET (`param show GPS_YAW*` returned zero matches, checked directly). This is a genuine open discrepancy worth flagging, not resolved here per the read-only scope.

command: param show EKF2_HGT_REF
exact response: EKF2_HGT_REF : 1
meaning: 1 = GNSS is the height reference, confirmed live, matches documentation.

command: param show EKF2_REQ_GPS_H
exact response: EKF2_REQ_GPS_H : 1.0000
meaning: 1.0 s required GPS health hold time before use, confirmed live.

command: param show EKF2_AID_MASK
exact response: (no matches)
meaning: this legacy PX4 parameter name does not exist on 1.16.2 — superseded by EKF2_GPS_CTRL. Confirms not to search for it on this firmware.
```

### 4. Running driver state (live snapshot)

```
command: listener sensor_gps -n 1
exact response (abbreviated): fix_type: 6, satellites_used: 10, eph: 0.015, epv: 0.030, hdop: 1.50, vdop: 2.83,
  heading: 0.122 rad, heading_accuracy: 0.00727 rad, vel_ned_valid: True,
  rtcm_msg_used: 2, selected_rtcm_instance: 0, rtcm_crc_failed: False, rtcm_injection_rate: 0.0
meaning: RTK FIXED (fix_type 6), 10 sats in the current live solution, 15 mm reported horizontal accuracy, dual-antenna heading valid. rtcm_injection_rate=0.0 at the topic level again confirms RTCM does not pass through PX4's own accounting — it enters the receiver purely via USB2, outside PX4's view.
```

---

## C. Ownership verdict

| Setting | Mosaic-H currently owns it? | PX4 currently configures it? | Conflict? |
|---|---|---|---|
| RTCMv3 input (USB2) | Yes — `DataInOut, USB2, RTCMv3, none, (on)`, persisted in Boot config | No — `controller→receiver: 0 B/s` on the PX4-side COM3 link; PX4 never touches USB2 | No |
| USB1 I/O | Yes — receiver default (`auto, SBF+NMEA, on`), unused in production | No | No |
| USB2 I/O | Yes — persisted Boot config | No | No |
| SBF output (COM3 stream) | Yes — `SBFOutput Stream1 → COM3`, persisted in Boot | PX4 only *consumes* this stream; doesn't set it (no `sso` in driver source) | No |
| Constellation/signal selection | Yes — receiver NVRAM (`getSignalUsage`/`getSatelliteUsage`), independent of PX4 | No — `SEP_CONST_USAGE=0` means driver never sends `ssu` | No |
| Update/output rates (10 Hz PVT) | Yes — `msec100` in SBFOutput, persisted | No — driver doesn't set `sso` | No |
| Dynamic model (Automotive) | Yes — `ReceiverDynamics, Moderate, Automotive` | No — no `srd` command exists in the driver | No |
| Antenna offsets/PCV | Yes — receiver-side (currently zero/Unknown, by design per the 2026-09-05 note: antenna=nozzle=EKF origin) | No — no `sao` command in driver source | No |
| Autoconfiguration | Receiver applies whatever is in its own NVRAM at boot | **`SEP_AUTO_CONFIG=0`, confirmed live** — PX4 does not reconfigure the receiver at all | No |
| GNSS driver/protocol on PX4 side | N/A | Yes — dedicated `septentrio` module, `/dev/ttyS4` @ 921600 baud, `SEP_PORT1_CFG=102` | N/A |
| GNSS data consumption | N/A | Yes — `sensor_gps` published, EKF2 fused via `EKF2_GPS_CTRL=15` | N/A |
| Receiver-side NTRIP client | Off on all 3 slots (`getNtripSettings`) | N/A — corrections come from Jetson→USB2 only | No (no competing path) |

## D. Final read-only verdict

```
1. Already matches desired production ownership
```

Every setting checked shows Mosaic-H owning its own persistent configuration (Current == Boot, byte-for-byte) and PX4 (`SEP_AUTO_CONFIG=0`, `SEP_CONST_USAGE=0`, `controller→receiver: 0 B/s`) doing nothing but consuming the resulting `sensor_gps` stream. No conflicting or overlapping ownership was found on any of the 11 settings audited.

**One open item, not a conflict but worth resolving before deciding on parameter changes:** `EKF2_GPS_YAW_OFF` read live as `0.0000`, not the `180.0` previously recorded in CLAUDE.md from 2026-08-31. No parameter literally named `GPS_YAW_OFFSET` exists on this firmware (checked directly, zero matches). This is either a value that changed since 2026-08-31 or a mis-transcription in the earlier doc — flagged as evidence, not resolved here, per the read-only scope of this pass.

**Evidence gaps (couldn't get read-only within this pass):** exact Mosaic-H firmware/build-string identifier (every ASCII getter tried for it — `getVersion`, `getFirmwareVersion`, `getReceiverType`, `getRxID`, `getReceiverSetup` — returned "Invalid command!"; the receiver model string "mosaic" was obtained via `getReceiverInterface`, but not a firmware build number). That would need either the receiver's web UI or the correct `lif,` internal filename, neither of which was guessed at under the read-only mandate.

No commands were run that write, save, reset, restart, or reconfigure anything on either the Mosaic-H or PX4. USB2 was never opened.
