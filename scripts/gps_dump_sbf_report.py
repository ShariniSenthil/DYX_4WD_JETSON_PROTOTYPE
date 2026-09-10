#!/usr/bin/env python3
"""Decode the raw Septentrio serial traffic that PX4 captures in `gps_dump`.

`SEP_DUMP_COMM=3` makes PX4 log every byte in both directions between the Jetson's
FCU and the mosaic-H.  This tool splits that back into the two streams and decodes
them, which reaches receiver-internal state that `sensor_gps` / `vehicle_gps_position`
throw away -- most importantly *which GNSS signals the RTK solution actually used*
and the receiver's own ARP / phase-centre compensation flags.

    python3 scripts/gps_dump_sbf_report.py <log.ulg> [more.ulg ...]
    python3 scripts/gps_dump_sbf_report.py --keep-streams <outdir> <log.ulg> ...

Struct layouts are taken from verified parsers and cross-checked against the
receiver's own Length/SBLength fields at decode time -- never hand-rolled offsets
(CLAUDE.md hard rule).  Sources:
  * PX4 1.16.2 src/drivers/gnss/septentrio/sbf/messages.h  (PVTGeodetic, DOP,
    ReceiverStatus, VelCovGeodetic, AttEuler, AttCovEuler)
  * ArduPilot libraries/AP_GPS/AP_GPS_SBF.h  (BaseVectorGeod, AuxAntPositions)
  * mosaic-H Firmware v4.14.10 Reference Guide, section 4.1.10 (signal numbers)
    and the PVTGeodetic / BaseVectorGeod block definitions (Misc bit fields).
    ArduPilot's VectorInfoGeod has ReferenceID as u1; the spec says u2 and the
    receiver agrees (SBLength 52) -- the u2 layout is the one used here.
"""

from __future__ import annotations

import argparse
import collections
import csv
import math
import os
import struct
import sys

# --------------------------------------------------------------------------- #
# SBF framing
# --------------------------------------------------------------------------- #

SBF_SYNC = b'$@'


def crc16_ccitt(buf: bytes) -> int:
    """CRC-CCITT, poly 0x1021, init 0 -- the SBF block CRC."""
    crc = 0
    for b in buf:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def iter_sbf(data: bytes):
    """Yield (block_number, revision, length, body) for every CRC-valid SBF block."""
    i, n = 0, len(data)
    while True:
        j = data.find(SBF_SYNC, i)
        if j < 0 or j + 8 > n:
            return
        crc, ident, length = struct.unpack_from('<HHH', data, j + 2)
        if length < 8 or length % 4 or j + length > n:
            i = j + 2
            continue
        if crc16_ccitt(data[j + 4:j + length]) != crc:
            i = j + 2
            continue
        yield ident & 0x1FFF, ident >> 13, length, data[j + 8:j + length]
        i = j + length


# --------------------------------------------------------------------------- #
# SBF block layouts (everything after the common TOW u4 / WNc u2 header)
# --------------------------------------------------------------------------- #

BLOCK_NAMES = {
    4001: 'DOP', 4007: 'PVTGeodetic', 4012: 'SatVisibility', 4013: 'ChannelStatus',
    4014: 'ReceiverStatus', 4028: 'BaseVectorGeod',
    4052: 'PosLocal', 4076: 'PVTSupport', 4079: 'PVTSupportA', 5906: 'PosCovGeodetic',
    5908: 'VelCovGeodetic', 5921: 'EndOfPVT', 5938: 'AttEuler', 5939: 'AttCovEuler',
    5942: 'AuxAntPositions', 5943: 'EndOfAtt',
}

# The exact block list PX4 1.16.2 asks for with `sso` when SEP_AUTO_CONFIG != 0
# (septentrio.cpp, k_command_sbf_output_pvt):
#     sso,Stream<N>,COM<n>,PVTGeodetic+VelCovGeodetic+DOP+AttEuler+AttCovEuler
#                          +EndOfPVT+ReceiverStatus,<rate>
# PX4 clears only its own stream (`sso,Stream<N>,COM<n>,none,off`) and can never
# ADD a block outside this set, so anything else arriving on the FCU link came
# from a stream the receiver owns in its own configuration.
PX4_SSO_BLOCKS = frozenset({4007, 5908, 4001, 5938, 5939, 5921, 4014})

FMT = {
    4007: ('<BBdddfffffdfBBBBHHIBBHHHHB',
           'Mode Error Lat Lon Height Undulation Vn Ve Vu COG RxClkBias RxClkDrift '
           'TimeSystem Datum NrSV WACorrInfo ReferenceID MeanCorrAge SignalInfo '
           'AlertFlag NrBases PPPInfo Latency HAccuracy VAccuracy Misc'.split()),
    4001: ('<BBHHHHff', 'NrSV Reserved PDOP TDOP HDOP VDOP HPL VPL'.split()),
    4014: ('<BBIII', 'CPULoad ExtError UpTime RxState RxError'.split()),
    5906: ('<BBffffffffff',
           'Mode Error CovLatLat CovLonLon CovHgtHgt CovBB CovLatLon CovLatHgt '
           'CovLatB CovLonHgt CovLonB CovHgtB'.split()),
    5938: ('<BBHHffffff',
           'NrSV Error Mode Reserved Heading Pitch Roll PitchDot RollDot HeadingDot'.split()),
}
SUB_FMT = {
    4028: ('<BBBBdddfffHhHHI',
           'NrSV Error Mode Misc DeltaEast DeltaNorth DeltaUp DeltaVe DeltaVn DeltaVu '
           'Azimuth Elevation ReferenceID CorrAge SignalInfo'.split()),
    5942: ('<BBBBdddddd',
           'NrSV Error AmbiguityType AuxAntID DeltaEast DeltaNorth DeltaUp '
           'EastVel NorthVel UpVel'.split()),
    # mosaic-H Reference Guide v4.14.10, "SatVisibility Number: 4012" (p.365):
    # SatInfo sub-block SVID/FreqNr/Azimuth(0.01deg)/Elevation(0.01deg)/RiseSet/SatelliteInfo.
    4012: ('<BBHhBB', 'SVID FreqNr Azimuth Elevation RiseSet SatelliteInfo'.split()),
}

# mosaic-H Reference Guide v4.14.10, "ChannelStatus Number: 4013" (p.358-360).
# Two-level sub-block: N ChannelSatInfo entries (SB1Length bytes each, fixed layout
# below + receiver-defined padding we never assume), each followed by N2
# ChannelStateInfo entries (SB2Length bytes each). Both lengths are read from the
# block itself and used to advance -- never computed/guessed (CLAUDE.md hard rule).
CHANNEL_SAT_FMT = ('<BB2xHHb', 'SVID FreqNr AzimuthRiseSet HealthStatus Elevation'.split())
CHANNEL_STATE_FMT = ('<BxHHH', 'Antenna TrackingStatus PVTStatus PVTInfo'.split())


def decode_channel_status(body: bytes):
    """4013 ChannelStatus -> list of {SVID, Elevation, states:[{Antenna,TrackingStatus,PVTStatus}]}."""
    if len(body) < 12:
        return []
    n, sb1_len, sb2_len = struct.unpack_from('<BBB', body, 6)
    fmt1, names1 = CHANNEL_SAT_FMT
    fmt2, names2 = CHANNEL_STATE_FMT
    need1, need2 = struct.calcsize(fmt1), struct.calcsize(fmt2)
    if sb1_len < need1 or sb2_len < need2:  # layout disagrees with the receiver -- refuse to guess
        return []
    out = []
    off = 12
    for _ in range(n):
        if off + sb1_len > len(body):
            break
        rec = dict(zip(names1, struct.unpack_from(fmt1, body, off)))
        n2 = body[off + 9]  # N2 field: byte offset 9 within ChannelSatInfo (SVID0 FreqNr1 Reserved1[2] Az/RiseSet[2] HealthStatus[2] Elevation1 N2@9)
        off += sb1_len
        states = []
        for _ in range(n2):
            if off + sb2_len > len(body):
                break
            states.append(dict(zip(names2, struct.unpack_from(fmt2, body, off))))
            off += sb2_len
        rec['states'] = states
        out.append(rec)
    return out

# mosaic-H Reference Guide section 4.1.10, bit index -> signal
SIGNALS = {
    0: 'GPS L1CA', 1: 'GPS L1P', 2: 'GPS L2P', 3: 'GPS L2C', 4: 'GPS L5', 5: 'GPS L1C',
    6: 'QZSS L1CA', 7: 'QZSS L2C', 8: 'GLO L1CA', 9: 'GLO L1P', 10: 'GLO L2P',
    11: 'GLO L2CA', 12: 'GLO L3', 13: 'BDS B1C', 14: 'BDS B2a', 15: 'NavIC L5',
    17: 'GAL E1', 19: 'GAL E6', 20: 'GAL E5a', 21: 'GAL E5b', 22: 'GAL E5 AltBOC',
    23: 'MSS LBand', 24: 'SBAS L1CA', 25: 'SBAS L5', 26: 'QZSS L5', 27: 'QZSS L6',
    28: 'BDS B1I', 29: 'BDS B2I', 30: 'BDS B3I', 32: 'QZSS L1C', 33: 'QZSS L1S',
    34: 'BDS B2b', 38: 'QZSS L1CB', 39: 'QZSS L5S',
}
BEIDOU_BITS = (13, 14, 28, 29, 30, 34)

PVT_MODE = {0: 'NoPVT', 1: 'StandAlone', 2: 'Differential', 3: 'FixedLoc',
            4: 'RTKfixed', 5: 'RTKfloat', 6: 'SBAS', 7: 'MovBaseFixed',
            8: 'MovBaseFloat', 10: 'PPP'}
ARP_TO_MARKER = {0: 'unknown', 1: 'zero', 2: 'non-zero'}


def signal_names(mask: int) -> str:
    return '+'.join(SIGNALS.get(b, f'bit{b}') for b in range(40) if mask >> b & 1) or 'none'


# mosaic-H Reference Guide v4.14.10, "ChannelStatus Number: 4013" (p.358), the
# per-constellation 2-bit-slot tables under "Health, tracking and PVT status
# fields". Each entry maps (bit_hi, bit_lo) -> signal name. Slots not listed are
# 'Reserved' in the guide and are skipped.
CHANNEL_SIGNAL_SLOTS = {
    'GPS':     {(11, 10): 'L1C', (9, 8): 'L5', (7, 6): 'L2C', (5, 4): 'P2(Y)', (3, 2): 'P1(Y)', (1, 0): 'L1CA'},
    'GLONASS': {(9, 8): 'L3', (7, 6): 'L2CA', (5, 4): 'L2P', (3, 2): 'L1P', (1, 0): 'L1CA'},
    'GALILEO': {(13, 12): 'E5ab', (11, 10): 'E5b', (9, 8): 'E5a', (7, 6): 'E6BC', (3, 2): 'L1BC'},
    'SBAS':    {(3, 2): 'L5', (1, 0): 'L1'},
    'BEIDOU':  {(11, 10): 'B2b', (9, 8): 'B2a', (7, 6): 'B1C', (5, 4): 'B3I', (3, 2): 'B2I', (1, 0): 'B1I'},
}

STATUS_2BIT = {0: 'idle/notused', 1: 'search/waitEph', 2: 'sync/used', 3: 'tracking/rejected'}


def svid_constellation(svid: int) -> str:
    """mosaic-H Reference Guide v4.14.10, section 4.1.9 (p.238) SVID ranges."""
    if 1 <= svid <= 37:
        return 'GPS'
    if 38 <= svid <= 68:
        return 'GLONASS'
    if 71 <= svid <= 106:
        return 'GALILEO'
    if 120 <= svid <= 140 or 198 <= svid <= 215:
        return 'SBAS'
    if 141 <= svid <= 180 or 223 <= svid <= 245:
        return 'BEIDOU'
    if 181 <= svid <= 187:
        return 'QZSS'
    if 191 <= svid <= 197 or 216 <= svid <= 222:
        return 'NAVIC'
    return 'UNKNOWN'


def decode_2bit_field(value: int, constellation: str, kind: str):
    """kind: 'tracking' -> STATUS_2BIT{0..3} 0/1/2/3; 'pvt' -> not-used/waitEph/used/rejected."""
    slots = CHANNEL_SIGNAL_SLOTS.get(constellation, {})
    out = {}
    for (hi, lo), name in slots.items():
        code = (value >> lo) & 0b11
        out[name] = code
    return out


def decode_sbf(data: bytes, want=None):
    """Decode SBF blocks into {block_number: [dict, ...]}."""
    out = collections.defaultdict(list)
    for num, rev, length, body in iter_sbf(data):
        if want and num not in want:
            continue
        if len(body) < 6:
            continue
        tow, wnc = struct.unpack_from('<IH', body, 0)
        if tow == 0xFFFFFFFF:
            continue
        rec = {'tow': tow / 1000.0, 'wnc': wnc, '_rev': rev, '_len': length}
        if num in FMT:
            fmt, names = FMT[num]
            if len(body) - 6 < struct.calcsize(fmt):
                continue
            rec.update(dict(zip(names, struct.unpack_from(fmt, body, 6))))
        elif num in SUB_FMT:
            if len(body) - 6 < 2:
                continue
            n, sb_len = struct.unpack_from('<BB', body, 6)
            fmt, names = SUB_FMT[num]
            need = struct.calcsize(fmt)
            if sb_len < need:          # layout disagrees with the receiver -- refuse to guess
                continue
            subs = []
            for k in range(n):
                off = 8 + k * sb_len
                if off + need > len(body):
                    break
                subs.append(dict(zip(names, struct.unpack_from(fmt, body, off))))
            rec['subs'] = subs
        elif num == 4013:
            rec['sats'] = decode_channel_status(body)
        else:
            continue
        out[num].append(rec)
    return out


# --------------------------------------------------------------------------- #
# RTCM3 framing (the PX4 -> receiver correction stream)
# --------------------------------------------------------------------------- #

def crc24q(buf: bytes) -> int:
    crc = 0
    for b in buf:
        crc ^= b << 16
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1864CFB) & 0xFFFFFF if crc & 0x800000 else (crc << 1) & 0xFFFFFF
    return crc


def _bits(payload: bytes, start: int, n: int) -> int:
    v = 0
    for i in range(start, start + n):
        v = (v << 1) | ((payload[i >> 3] >> (7 - (i & 7))) & 1)
    return v


MSM4 = {1074: 'GPS', 1084: 'GLONASS', 1094: 'Galileo', 1104: 'SBAS',
        1114: 'QZSS', 1124: 'BeiDou'}
RTCM_NAMES = {**MSM4, 1005: 'base ARP', 1006: 'base ARP + height',
              1007: 'antenna descriptor', 1008: 'antenna descriptor+serial',
              1033: 'receiver+antenna descriptors',
              1230: 'GLONASS code-phase biases'}


def decode_rtcm(data: bytes):
    """Return (Counter of message type, {msm type: [nsat]}, n_ok, n_crc_fail)."""
    counts = collections.Counter()
    sats = collections.defaultdict(list)
    ok = bad = 0
    i, n = 0, len(data)
    while True:
        j = data.find(b'\xd3', i)
        if j < 0 or j + 6 > n:
            break
        ln = ((data[j + 1] & 0x03) << 8) | data[j + 2]
        end = j + 3 + ln + 3
        if ln == 0 or end > n:
            i = j + 1
            continue
        want = (data[end - 3] << 16) | (data[end - 2] << 8) | data[end - 1]
        if crc24q(data[j:end - 3]) != want:
            bad += 1
            i = j + 1
            continue
        ok += 1
        payload = data[j + 3:end - 3]
        mtype = _bits(payload, 0, 12)
        counts[mtype] += 1
        # MSM header is 73 bits, then the 64-bit DF394 satellite mask
        if mtype in MSM4 and len(payload) >= 18:
            sats[mtype].append(bin(_bits(payload, 73, 64)).count('1'))
        i = end
    return counts, sats, ok, bad


# --------------------------------------------------------------------------- #
# gps_dump extraction
# --------------------------------------------------------------------------- #

def split_gps_dump(ulg_path: str):
    """Return (from_device_bytes, to_device_bytes).

    PX4 GpsDump.msg: the MSB of `len` is set for messages sent TO the receiver.
    """
    from pyulog import ULog
    import numpy as np
    u = ULog(ulg_path, ['gps_dump'])
    if not u.data_list:
        return b'', b''
    d = u.data_list[0]
    cols = np.vstack([d.data['data[%d]' % i] for i in range(79)]).astype(np.uint8)
    lens = d.data['len'].astype(int)
    frm, to = bytearray(), bytearray()
    for i, raw in enumerate(lens):
        chunk = cols[:raw & 0x7F, i].tobytes()
        (to if raw & 0x80 else frm).extend(chunk)
    return bytes(frm), bytes(to)


# --------------------------------------------------------------------------- #
# reporting
# --------------------------------------------------------------------------- #

WGS84_A, WGS84_F = 6378137.0, 1 / 298.257223563


def metres_per_degree(lat_deg: float):
    e2 = WGS84_F * (2 - WGS84_F)
    s = math.sin(math.radians(lat_deg))
    w = math.sqrt(1 - e2 * s * s)
    return (math.radians(1) * WGS84_A * (1 - e2) / w ** 3,
            math.radians(1) * WGS84_A / w * math.cos(math.radians(lat_deg)))


def load_reference(path):
    ref = []
    with open(path) as f:
        for row in csv.DictReader(f):
            ref.append((float(row['latitude']), float(row['longitude']),
                        row.get('pile') or row.get('name') or '?'))
    return ref


def median(seq):
    s = sorted(seq)
    return s[len(s) // 2] if s else float('nan')


def report(ulg_path, ref, keep_dir=None, vmax=0.03, min_epochs=20):
    name = os.path.basename(ulg_path).replace('.ulg', '')
    frm, to = split_gps_dump(ulg_path)
    if keep_dir:
        os.makedirs(keep_dir, exist_ok=True)
        open(os.path.join(keep_dir, name + '.sbf'), 'wb').write(frm)
        open(os.path.join(keep_dir, name + '.rtcm'), 'wb').write(to)
    if not frm:
        print(f'{name}: no gps_dump (is SEP_DUMP_COMM set?)')
        return

    sbf = decode_sbf(frm, want={4001, 4007, 4014, 4028, 5942})
    pvt = [p for p in sbf[4007] if p['Lat'] > -1e9]
    print(f'\n===== {name} =====')

    # --- whose configuration is this receiver actually running? --------------
    emitted = {b for b, _rev, _len, _body in iter_sbf(frm)}
    extra = sorted(emitted - PX4_SSO_BLOCKS)
    missing = sorted(PX4_SSO_BLOCKS - emitted)
    if extra or missing:
        print(f'    config fingerprint: {len(emitted)} SBF blocks -- NOT PX4\'s sso set'
              + (f'; receiver-owned: '
                 + ' '.join(BLOCK_NAMES.get(b, str(b)) for b in extra) if extra else '')
              + (f'; absent: ' + ' '.join(BLOCK_NAMES.get(b, str(b)) for b in missing)
                 if missing else ''))
    else:
        print('    config fingerprint: 7 SBF blocks == PX4 sso set exactly '
              '(no receiver-owned stream on this port)')
    if not pvt:
        print('  no usable PVTGeodetic')
        return

    modes = collections.Counter(PVT_MODE.get(p['Mode'] & 0xF, p['Mode'] & 0xF) for p in pvt)
    dop = sbf[4001]
    print(f'  PVT epochs {len(pvt)}   modes {dict(modes)}   '
          f'NrSV med {median([p["NrSV"] for p in pvt if p["NrSV"] != 255]):.0f}   '
          f'HDOP {median([d["HDOP"] / 100 for d in dop if d["HDOP"] not in (0, 65535)]):.2f}  '
          f'VDOP {median([d["VDOP"] / 100 for d in dop if d["VDOP"] not in (0, 65535)]):.2f}')

    # --- which signals the PVT actually used ---------------------------------
    for mask, k in collections.Counter(p['SignalInfo'] for p in pvt).most_common(3):
        print(f'    SignalInfo 0x{mask:08x} x{k:<6d} {signal_names(mask)}')
    bds = [any(p['SignalInfo'] >> b & 1 for b in BEIDOU_BITS) for p in pvt]
    gap = cur = 0
    for u in bds:
        cur = 0 if u else cur + 1
        gap = max(gap, cur)
    rate = len(pvt) / max(pvt[-1]['tow'] - pvt[0]['tow'], 1e-6)
    print(f'    BeiDou used in {100 * sum(bds) / len(bds):.1f}% of epochs, '
          f'longest continuous absence {gap / rate:.1f} s')

    # --- ARP / phase-centre compensation state -------------------------------
    for misc, k in collections.Counter(p['Misc'] for p in pvt).most_common(3):
        print(f'    PVT Misc 0x{misc:02x} x{k:<6d} baseline->base ARP={misc & 1}  '
              f'rover PCO compensated={misc >> 1 & 1}  '
              f'ARP-to-marker offset {ARP_TO_MARKER.get(misc >> 6, "?")}')

    rs = sbf[4014]
    if rs:
        print(f'    receiver uptime {rs[0]["UpTime"]}s -> {rs[-1]["UpTime"]}s   '
              f'RxError 0x{rs[0]["RxError"]:x}  ExtError '
              f'{dict(collections.Counter(r["ExtError"] for r in rs))}')

    # --- differential baseline, when the receiver is configured to output it --
    bv = [b['subs'][0] for b in sbf[4028] if b.get('subs')]
    if bv:
        for misc, k in collections.Counter(b['Misc'] for b in bv).most_common(2):
            print(f'    BaseVector Misc 0x{misc:02x} x{k:<6d} '
                  f'points to base ARP={misc & 1}  rover PCO compensated={misc >> 1 & 1}'
                  f'   <-- ARP-to-ARP guaranteed only when both are 1')
        for mask, k in collections.Counter(b['SignalInfo'] for b in bv).most_common(2):
            print(f'    base corrections available for 0x{mask:08x} x{k:<6d} {signal_names(mask)}')
        lens = [math.hypot(math.hypot(b['DeltaEast'], b['DeltaNorth']), b['DeltaUp'])
                for b in bv if abs(b['DeltaEast']) < 1e9]
        print(f'    baseline length med {median(lens) / 1000:.3f} km')
    aux = [a['subs'][0] for a in sbf[5942] if a.get('subs')]
    if aux:
        lens = [math.hypot(math.hypot(a['DeltaEast'], a['DeltaNorth']), a['DeltaUp'])
                for a in aux if abs(a['DeltaEast']) < 1e9]
        print(f'    dual-antenna separation med {median(lens):.4f} m   '
              f'ambiguities {dict(collections.Counter(a["AmbiguityType"] for a in aux))} '
              f'(0=fixed)')

    # --- injected corrections ------------------------------------------------
    counts, sats, ok, bad = decode_rtcm(to)
    if ok:
        parts = []
        for t in sorted(counts):
            s = f'{t}'
            if t in sats and sats[t]:
                s += f'({median(sats[t]):.0f}sv)'
            parts.append(s)
        print(f'    RTCM in: {ok} msgs, {bad} CRC-fail | ' + ' '.join(parts))
        for t in sorted(counts):
            if t in (1007, 1008, 1033):
                break
        else:
            print('    RTCM in: no 1007/1008/1033 -- base antenna descriptors absent')

    # --- stationary position error against surveyed truth --------------------
    if not ref:
        return
    runs, cur = [], []
    for p in pvt:
        if math.hypot(p['Vn'], p['Ve']) < vmax and (p['Mode'] & 0xF) in (2, 4, 5):
            cur.append(p)
        else:
            if len(cur) >= min_epochs:
                runs.append(cur)
            cur = []
    if len(cur) >= min_epochs:
        runs.append(cur)
    for r in runs:
        lat = median([x['Lat'] for x in r]) * 180 / math.pi
        lon = median([x['Lon'] for x in r]) * 180 / math.pi
        tgt = min(ref, key=lambda t: (t[0] - lat) ** 2 + (t[1] - lon) ** 2)
        mlat, mlon = metres_per_degree(tgt[0])
        d_n, d_e = (lat - tgt[0]) * mlat * 1000, (lon - tgt[1]) * mlon * 1000
        if math.hypot(d_n, d_e) > 2000:
            continue
        mask = collections.Counter(x['SignalInfo'] for x in r).most_common(1)[0][0]
        print(f'    stop @ {tgt[2]:>3s}  n={len(r):5d}  dN {d_n:+8.1f}  dE {d_e:+8.1f}  '
              f'|r| {math.hypot(d_n, d_e):7.1f} mm  h {median([x["Height"] for x in r]):+9.3f} m  '
              f'{"BDS" if any(mask >> b & 1 for b in BEIDOU_BITS) else "no-BDS"}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ulogs', nargs='+')
    ap.add_argument('--keep-streams', metavar='DIR',
                    help='also write the raw .sbf / .rtcm streams here')
    ap.add_argument('--reference', metavar='CSV',
                    help='surveyed targets (columns latitude,longitude[,pile]) to '
                         'report stationary position error against')
    args = ap.parse_args()
    ref = load_reference(args.reference) if args.reference else []
    for p in args.ulogs:
        report(p, ref, keep_dir=args.keep_streams)


if __name__ == '__main__':
    main()
