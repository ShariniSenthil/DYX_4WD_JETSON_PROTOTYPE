#!/usr/bin/env python3
"""Decode the base station's own declared coordinate straight out of the RTCM
stream, independent of anything PX4/the Septentrio driver does with it.

Subscribes to /mavros/gps_rtk/send_rtcm (mavros_msgs/msg/RTCM), which per
ntrip_to_px4_node carries exactly one complete CRC-valid RTCM3 frame per
message. Decodes RTCM message types 1005/1006 (Stationary RTK Reference
Station ARP, with/without antenna height) -- these carry the base's own
ECEF X/Y/Z directly from the source, before PX4 ever sees it. Also tallies
every RTCM message type seen (MSM observables, GLONASS biases, etc.) as
context.

This exists because the flashed PX4 Septentrio driver's SBF autoconfig does
not request any base-identity block from the receiver (confirmed 2026-09-08
source read) -- there is no way to see the base's coordinate from the FCU
side. This is the only avenue: read it directly out of the correction
stream before it leaves the Jetson.

Usage:
    source /opt/ros/humble/setup.bash
    python3 scripts/rtcm_base_monitor.py --duration 600
"""
import argparse
import math
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import RTCM

WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)


class BitReader:
    """MSB-first bit reader over a bytes object, matching RTCM3's packing."""

    def __init__(self, data: bytes):
        self.data = data
        self.pos = 0  # bit position

    def u(self, nbits):
        val = 0
        for _ in range(nbits):
            byte_i = self.pos // 8
            bit_i = 7 - (self.pos % 8)
            bit = (self.data[byte_i] >> bit_i) & 1
            val = (val << 1) | bit
            self.pos += 1
        return val

    def s(self, nbits):
        val = self.u(nbits)
        if val & (1 << (nbits - 1)):
            val -= 1 << nbits
        return val


def ecef_to_llh(x, y, z):
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - WGS84_E2))
    for _ in range(6):
        sin_lat = math.sin(lat)
        n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1 - WGS84_E2 * n / (n + h)))
    sin_lat = math.sin(lat)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)
    h = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), h


def decode_1005_1006(payload: bytes):
    """payload = RTCM message content (after the 3-byte header, before CRC).
    Returns dict with msg_type, station_id, ecef xyz (m), lat/lon/h, and
    antenna height (1006 only), or None if not decodable."""
    br = BitReader(payload)
    msg_type = br.u(12)
    if msg_type not in (1005, 1006):
        return None
    station_id = br.u(12)
    br.u(6)   # ITRF realization year
    br.u(1)   # GPS indicator
    br.u(1)   # GLONASS indicator
    br.u(1)   # Galileo/reserved indicator
    br.u(1)   # reference station indicator
    x = br.s(38) * 0.0001
    br.u(1)   # single receiver oscillator indicator
    br.u(1)   # reserved
    y = br.s(38) * 0.0001
    br.u(2)   # quarter cycle indicator
    z = br.s(38) * 0.0001
    ant_height = None
    if msg_type == 1006:
        ant_height = br.u(16) * 0.0001
    lat, lon, h = ecef_to_llh(x, y, z)
    return {
        "msg_type": msg_type, "station_id": station_id,
        "ecef_x": x, "ecef_y": y, "ecef_z": z,
        "lat": lat, "lon": lon, "height_m": h,
        "antenna_height_m": ant_height,
    }


def parse_rtcm_frame(data: bytes):
    """data is one complete RTCM3 frame: 0xD3, 6 reserved bits + 10-bit
    length, then that many payload bytes, then 3-byte CRC24Q. Returns the
    payload bytes, or None if the framing doesn't check out."""
    if len(data) < 6 or data[0] != 0xD3:
        return None
    length = ((data[1] & 0x03) << 8) | data[2]
    if len(data) < 3 + length + 3:
        return None
    return data[3:3 + length]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=600.0, help="seconds to monitor (default 600 = 10 min)")
    args = ap.parse_args()

    rclpy.init()
    node = Node("rtcm_base_monitor")

    type_counts = {}
    base_fixes = []  # list of decoded 1005/1006 dicts with elapsed time
    n_frames = 0
    t0 = time.monotonic()

    def cb(msg: RTCM):
        nonlocal n_frames
        n_frames += 1
        payload = parse_rtcm_frame(bytes(msg.data))
        if payload is None or len(payload) < 2:
            type_counts["UNPARSEABLE"] = type_counts.get("UNPARSEABLE", 0) + 1
            return
        msg_type = (payload[0] << 4) | (payload[1] >> 4)
        type_counts[msg_type] = type_counts.get(msg_type, 0) + 1
        if msg_type in (1005, 1006):
            decoded = decode_1005_1006(payload)
            if decoded:
                decoded["elapsed_s"] = time.monotonic() - t0
                base_fixes.append(decoded)
                print(f"[{decoded['elapsed_s']:7.1f}s] type={decoded['msg_type']} station_id={decoded['station_id']} "
                      f"lat={decoded['lat']:.9f} lon={decoded['lon']:.9f} h={decoded['height_m']:.4f}m "
                      f"ant_h={decoded['antenna_height_m']}", flush=True)

    q = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(RTCM, "/mavros/gps_rtk/send_rtcm", cb, q)

    print(f"monitoring /mavros/gps_rtk/send_rtcm for {args.duration:.0f}s ...", flush=True)
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.5)

    print(f"\n=== done: {n_frames} RTCM frames over {args.duration:.0f}s ===")
    print("message type histogram:")
    for t in sorted(type_counts, key=lambda k: (isinstance(k, str), k)):
        print(f"  {t}: {type_counts[t]}")

    if base_fixes:
        lats = [f["lat"] for f in base_fixes]
        lons = [f["lon"] for f in base_fixes]
        hs = [f["height_m"] for f in base_fixes]
        print(f"\nbase station 1005/1006 fixes: n={len(base_fixes)}")
        print(f"  lat: min={min(lats):.9f} max={max(lats):.9f} spread_mm={(max(lats)-min(lats))*111320000:.2f}")
        print(f"  lon: min={min(lons):.9f} max={max(lons):.9f} spread_mm={(max(lons)-min(lons))*111320000:.2f}")
        print(f"  height: min={min(hs):.4f} max={max(hs):.4f} spread_mm={(max(hs)-min(hs))*1000:.2f}")
        print(f"  station_id(s) seen: {sorted(set(f['station_id'] for f in base_fixes))}")
    else:
        print("\nNo 1005/1006 base-position messages seen in this window -- "
              "the caster/mountpoint may not be sending base ARP at all.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
