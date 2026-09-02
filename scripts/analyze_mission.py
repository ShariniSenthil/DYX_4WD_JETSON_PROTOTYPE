#!/usr/bin/env python3
"""Post-mission behaviour analyser for the DYX 4WD rover — reads ONE finalised
bag/bundle and emits `analysis.json` + a human-readable `report.txt`.

Ported from the 3WD `PX4_DXP/tools/analyze_mission.py` tool. The engine
(sqlite3+CDR bag reader, geodesic math, stats helpers) is reused verbatim —
it is message-type-agnostic. Everything topic-specific below was rebuilt from
scratch and verified against a real bundle
(`bags_jet/mission.csv_20260825_155833/`) on 2026-08-31, because this stack's
topic set and message types differ substantially from 3WD's (see
`.claude/skills/analyse-missions/SKILL.md` §2 for the full inventory this was
built from). Referenced by `scripts/bag_autorecord.py` as
`_ANALYZER` when `BAG_AUTO_ANALYZE=1`.

Design (same as 3WD):
  * PURE READER — never touches the robot.
  * Dependency-light — stdlib-only sqlite3 + CDR reader, runs OFFLINE on any
    `.db3` bag, on the Mac GCS as well as the Jetson (no ROS install needed).
  * Tolerant of missing topics — a missing topic is a WARN, never a crash.
  * No shape assumptions — works for line / L / square / arbitrary mission.

Usage:
    python3 scripts/analyze_mission.py <bundle-or-bag-dir> [-o OUTDIR] [--json-only]

<bundle-or-bag-dir> may be either a recorder bundle (dir with `bag/` +
`manifest.json`) or a bare rosbag2 directory (dir with `metadata.yaml`).

KNOWN GAPS — read before trusting a number this tool prints:
  * The debug-JSON topics (/rpp/geometry_debug, /rpp/guidance_debug,
    /rpp/accuracy, /mission_manager/status, ...) carry many more fields than
    this tool extracts. It pulls the cross-track fields and leaves the rest
    in `Series.debug_raw[topic]` as parsed dicts for ad-hoc inspection.
  * /mission_waypoints vs /nav_path vs /runtime_nav_path semantics (§2 of the
    skill) were inferred from ONE bundle's data, not from reading
    trajectory_generator/rpp_controller source line by line. They matched
    every cross-check available (endpoint coincidence, manifest
    navigation_point_count, monotonic shrink of runtime_nav_path) but treat
    that as "strongly supported", not "verified against source".
  * No surveyed ground-truth source is wired up (§4.3 step 1 in the skill) —
    analyze_absolute() only compares EKF origin stability run-to-run, it
    cannot check absolute placement against survey without that source.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sqlite3
import struct
import sys
from datetime import datetime, timezone

# ── thresholds — UNCALIBRATED for this vehicle, treat as starting points ────
# 3WD's thresholds (XTRACK_PROD_CM=2.0 etc) came from months of field data on
# a different drivetrain and controller. Do not import them by number. These
# are placeholders until the first batch of clean 4WD bundles are analysed
# (see the skill's §5 "Known-good reference — NOT YET ESTABLISHED").
COVERAGE_RADIUS_M = 0.25     # a path point counts as "reached" within this distance
COVERAGE_COMPLETE = 0.98
COVERAGE_PARTIAL = 0.75
WAYPOINT_FIDELITY_WARN_CM = 5.0   # a mission_waypoint sitting this far from the
                                  # densified nav_path is worth flagging
TRUNC_START_SPEED_MPS = 0.10      # plausible "genuinely at rest" ceiling


_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)


def _metres_per_degree(lat_deg: float) -> tuple[float, float]:
    """(north, east) metres per degree on the WGS84 ellipsoid."""
    lat = math.radians(lat_deg)
    sn = math.sin(lat)
    w2 = 1.0 - _WGS84_E2 * sn * sn
    w = math.sqrt(w2)
    m_merid = _WGS84_A * (1.0 - _WGS84_E2) / (w2 * w)
    n_prime = _WGS84_A / w
    per = math.radians(1.0)
    return (m_merid * per, n_prime * per * math.cos(lat))


def _geodesic_ne_m(lat1, lon1, lat2, lon2) -> tuple[float, float]:
    mn, me = _metres_per_degree((lat1 + lat2) * 0.5)
    return ((lat2 - lat1) * mn, (lon2 - lon1) * me)


def _geodesic_m(lat1, lon1, lat2, lon2) -> float:
    dn, de = _geodesic_ne_m(lat1, lon1, lat2, lon2)
    return math.hypot(dn, de)


def _xtrack_to_polyline(p, poly) -> float:
    """Shortest distance from p=(x,y) to a polyline [(x,y), ...] (metres)."""
    best = float("inf")
    for j in range(len(poly) - 1):
        a, b = poly[j], poly[j + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        s2 = dx * dx + dy * dy
        t = 0.0 if s2 < 1e-12 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / s2))
        d = math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))
        if d < best:
            best = d
    return best


# ═══════════════════════════════════════════════════════════════════════════
# CDR reader — classic little-endian XCDR1 with proper member alignment.
# Ported verbatim from the 3WD tool: this class is generic to any ROS2 CDR
# message, not specific to either vehicle.
# ═══════════════════════════════════════════════════════════════════════════
class _CDR:
    __slots__ = ("d", "o")

    def __init__(self, data: bytes):
        self.d = data
        self.o = 4  # skip 4-byte encapsulation header (representation id + options)

    def _align(self, n: int) -> None:
        rel = self.o - 4
        self.o += (-rel) % n

    def u8(self):
        v = self.d[self.o]; self.o += 1; return v

    def i8(self):
        v = struct.unpack_from("<b", self.d, self.o)[0]; self.o += 1; return v

    def u16(self):
        self._align(2); v = struct.unpack_from("<H", self.d, self.o)[0]; self.o += 2; return v

    def u32(self):
        self._align(4); v = struct.unpack_from("<I", self.d, self.o)[0]; self.o += 4; return v

    def i32(self):
        self._align(4); v = struct.unpack_from("<i", self.d, self.o)[0]; self.o += 4; return v

    def u64(self):
        self._align(8); v = struct.unpack_from("<Q", self.d, self.o)[0]; self.o += 8; return v

    def f32(self):
        self._align(4); v = struct.unpack_from("<f", self.d, self.o)[0]; self.o += 4; return v

    def f64(self):
        self._align(8); v = struct.unpack_from("<d", self.d, self.o)[0]; self.o += 8; return v

    def boolean(self):
        return bool(self.u8())

    def string(self):
        n = self.u32()  # CDR string length includes the null terminator
        raw = self.d[self.o:self.o + n]; self.o += n
        return raw.split(b"\x00", 1)[0].decode("utf-8", "replace")

    def header(self):
        sec = self.i32(); nsec = self.u32(); fid = self.string()
        return (sec + nsec * 1e-9, fid)


# ── per-type parsers → normalized dicts. All verified 2026-08-31 against
# bags_jet/mission.csv_20260825_155833/bag/bag_0.db3 (see the inline notes
# per parser for what was checked and how). ──────────────────────────────────
def _p_navsatfix(d):
    """sensor_msgs/msg/NavSatFix. NavSatStatus is int8 status + uint16 service
    (NOT uint8 — the 3WD tool's hard-won bug, ported forward as a comment,
    not re-triggered: reading service as uint8 shifts every f64 after it by a
    byte and yields a garbage lat/lon). Verified: decoded lat/lon on
    /mavros/global_position/global and /raw/fix both land at ~13.1894N
    80.2221E, matching /mavros/global_position/gp_origin — not near (0, lat).
    """
    r = _CDR(d); r.header()
    r.i8()
    r.u16()
    lat = r.f64(); lon = r.f64(); alt = r.f64()
    return {"lat": lat, "lon": lon, "alt": alt}


def _p_geopoint(d):
    """geographic_msgs/msg/GeoPointStamped (gp_origin — EKF local-frame datum)."""
    r = _CDR(d); r.header()
    lat = r.f64(); lon = r.f64(); alt = r.f64()
    return {"lat": lat, "lon": lon, "alt": alt}


def _p_gpsraw(d):
    """mavros_msgs/msg/GPSRAW. Verified: fix_type=6 (RTK FIXED), sats=15,
    h_acc=0.02m on the sample bundle — plausible RTK-locked values, not
    garbage. Deliberately stops after h_acc/v_acc (unlike the 3WD parser,
    which also decodes a dual-antenna yaw/hdg_acc tail) — that tail's layout
    has not been verified on this mavros build; add it back only after
    checking it the same way NavSatFix was checked above.
    """
    r = _CDR(d); r.header()
    fix = r.u8()
    lat = r.i32(); lon = r.i32(); alt = r.i32()
    eph = r.u16(); epv = r.u16(); vel = r.u16(); cog = r.u16()
    sats = r.u8()
    r.i32()
    h_acc = r.u32(); v_acc = r.u32()
    return {"fix_type": fix, "lat": lat * 1e-7, "lon": lon * 1e-7, "alt": alt * 1e-3,
            "eph": eph, "epv": epv, "sats": sats,
            "h_acc": h_acc * 1e-3, "v_acc": v_acc * 1e-3}


def _p_state(d):
    """mavros_msgs/msg/State. Verified: this build's bags carry a std_msgs
    header (with_header=True decoded 'MANUAL' cleanly); try headerless too,
    same defensive pattern as the 3WD tool, in case an older/newer mavros
    build on this Jetson drops the header again.
    """
    last_err = None
    for with_header in (False, True):
        try:
            r = _CDR(d)
            if with_header:
                r.header()
            connected = r.boolean(); armed = r.boolean(); guided = r.boolean()
            manual = r.boolean(); mode = r.string(); system_status = r.u8()
            if len(mode) <= 32 and all(32 <= ord(c) < 127 for c in mode):
                return {"connected": connected, "armed": armed, "guided": guided,
                        "manual_input": manual, "mode": mode,
                        "system_status": system_status}
        except Exception as e:
            last_err = e
    raise ValueError(f"State CDR layout unrecognised: {last_err}")


def _p_bool(d):
    return {"data": _CDR(d).boolean()}


def _p_float64(d):
    """std_msgs/msg/Float64 — no header. Verified against /rpp/xtrack_mm
    (values ~27-44, plausible mm cross-track) and /rpp/command_speed_mps
    (1.0, plausible m/s)."""
    return {"data": _CDR(d).f64()}


def _p_float32(d):
    """std_msgs/msg/Float32 — no header. Verified against
    /rtk_correction_bridge/correction_age_sec (~0.0004-0.0005 s, a fresh
    correction — plausible)."""
    return {"data": _CDR(d).f32()}


def _p_uint64(d):
    """std_msgs/msg/UInt64 — no header. Verified against
    /rover_backend/heartbeat: monotonically increasing small integers
    (1206, 1207, ...), consistent with a tick counter."""
    return {"data": _CDR(d).u64()}


def _p_string(d):
    """std_msgs/msg/String — no header. Most String topics on this stack
    (/rpp/*_debug, /rpp/accuracy, /mission_manager/status,
    /mission_manager/segment_goal_metadata, /trajectory_generator/*) carry a
    JSON payload; parsed generically here and left raw if it isn't JSON."""
    text = _CDR(d).string()
    try:
        return {"data": text, "json": json.loads(text) if text else None}
    except (json.JSONDecodeError, ValueError):
        return {"data": text, "json": None}


def _p_pose_stamped(d):
    """geometry_msgs/msg/PoseStamped — used for /active_waypoint,
    /segment_goal. Verified: both decode to the same (x, y) as the first
    point of /runtime_nav_path and the first point of /mission_waypoints for
    the same instant — consistent with "current target point"."""
    r = _CDR(d); r.header()
    px, py, pz = r.f64(), r.f64(), r.f64()
    ox, oy, oz, ow = r.f64(), r.f64(), r.f64(), r.f64()
    return {"x": px, "y": py, "z": pz, "qx": ox, "qy": oy, "qz": oz, "qw": ow}


def _p_path(d):
    """nav_msgs/msg/Path — shared ROS2 core type, same layout as 3WD.
    Verified against /mission_waypoints (2 sparse points), /nav_path (~400
    densified points), /runtime_nav_path (same densified points, shrinking
    over successive messages) — all decode to plausible local-frame (x, y)
    in metres, with matching endpoints across the three topics."""
    r = _CDR(d); r.header()
    n = r.u32()
    poses = []
    for _ in range(n):
        r.header()
        px, py, pz = r.f64(), r.f64(), r.f64()
        r.f64(); r.f64(); r.f64(); r.f64()  # orientation, unused
        poses.append((px, py, pz))
    return {"poses": poses}


def _p_vec3stamped(d):
    """geometry_msgs/msg/Vector3Stamped — used for /rpp/velocity_ned."""
    r = _CDR(d); r.header()
    return {"x": r.f64(), "y": r.f64(), "z": r.f64()}


def _p_odometry(d):
    """nav_msgs/msg/Odometry — /mavros/local_position/odom. This is the pose
    source on this stack (3WD used a bare geometry_msgs/PoseStamped on
    /mavros/local_position/pose; that topic does not exist here). Layout:
    header, child_frame_id (string), PoseWithCovariance (Pose + float64[36]),
    TwistWithCovariance (Twist + float64[36]). Verified: rawlen 716 bytes
    exactly matches header+child_frame_id+7 f64 pose+36 f64 pose-cov+6 f64
    twist+36 f64 twist-cov, with 288 bytes (=36 f64, the twist covariance)
    correctly left over as `remaining` when that tail isn't parsed."""
    r = _CDR(d); r.header()
    r.string()  # child_frame_id, unused
    px, py, pz = r.f64(), r.f64(), r.f64()
    ox, oy, oz, ow = r.f64(), r.f64(), r.f64(), r.f64()
    for _ in range(36):
        r.f64()  # pose covariance, unused
    lx, ly, lz = r.f64(), r.f64(), r.f64()
    ax, ay, az = r.f64(), r.f64(), r.f64()
    return {"x": px, "y": py, "z": pz, "qx": ox, "qy": oy, "qz": oz, "qw": ow,
            "lx": lx, "ly": ly, "lz": lz, "ax": ax, "ay": ay, "az": az}


def _p_i32_multiarray(d):
    """std_msgs/msg/Int32MultiArray — /trajectory_generator/marking_indices.
    MultiArrayLayout (dim[]: {string label; uint32 size; uint32 stride;},
    uint32 data_offset) + int32[] data. Verified: decodes to a -1-filled
    sentinel array matching the manifest's navigation_point_count, with 0/1
    markers near the end on a spray-tail mission — consistent with
    'index into the path where marking starts/stops', but exact semantics
    not confirmed against trajectory_generator source."""
    r = _CDR(d)
    dim_len = r.u32()
    for _ in range(dim_len):
        r.string(); r.u32(); r.u32()
    r.u32()  # data_offset
    n = r.u32()
    return {"data": [r.i32() for _ in range(n)]}


def _p_u8_multiarray(d):
    """std_msgs/msg/UInt8MultiArray — /trajectory_generator/path_types.
    Verified: array length (447) matches manifest.identity.navigation_point_count
    exactly for the same bundle — this is a per-nav_path-point type code."""
    r = _CDR(d)
    dim_len = r.u32()
    for _ in range(dim_len):
        r.string(); r.u32(); r.u32()
    r.u32()
    n = r.u32()
    return {"data": [r.u8() for _ in range(n)]}


PARSERS = {
    "sensor_msgs/msg/NavSatFix": _p_navsatfix,
    "geographic_msgs/msg/GeoPointStamped": _p_geopoint,
    "mavros_msgs/msg/GPSRAW": _p_gpsraw,
    "mavros_msgs/msg/State": _p_state,
    "std_msgs/msg/Bool": _p_bool,
    "std_msgs/msg/Float64": _p_float64,
    "std_msgs/msg/Float32": _p_float32,
    "std_msgs/msg/UInt64": _p_uint64,
    "std_msgs/msg/String": _p_string,
    "geometry_msgs/msg/PoseStamped": _p_pose_stamped,
    "nav_msgs/msg/Path": _p_path,
    "geometry_msgs/msg/Vector3Stamped": _p_vec3stamped,
    "nav_msgs/msg/Odometry": _p_odometry,
    "std_msgs/msg/Int32MultiArray": _p_i32_multiarray,
    "std_msgs/msg/UInt8MultiArray": _p_u8_multiarray,
}


# ═══════════════════════════════════════════════════════════════════════════
# Bag reading — identical engine to 3WD, message-type-agnostic.
# ═══════════════════════════════════════════════════════════════════════════
def _find_bag_dir(root: str):
    """Return (bag_dir, manifest_or_None). Accepts a bundle or a bare bag dir."""
    manifest = None
    mpath = os.path.join(root, "manifest.json")
    if os.path.isfile(mpath):
        try:
            manifest = json.load(open(mpath))
        except Exception:
            manifest = None
    if os.path.isdir(os.path.join(root, "bag")) and os.path.isfile(
        os.path.join(root, "bag", "metadata.yaml")
    ):
        return os.path.join(root, "bag"), manifest
    if os.path.isfile(os.path.join(root, "metadata.yaml")) or glob.glob(
        os.path.join(root, "*.db3")
    ):
        return root, manifest
    for db in glob.glob(os.path.join(root, "**", "*.db3"), recursive=True):
        return os.path.dirname(db), manifest
    return None, manifest


def read_bag(bag_dir: str):
    """Yield (topic_name, msg_dict, t_seconds) in time order."""
    dbs = sorted(glob.glob(os.path.join(bag_dir, "*.db3")))
    if not dbs:
        sys.exit(f"ERROR: no .db3 file under {bag_dir} — this reader is "
                  f"stdlib sqlite3-only, it does not read mcap bags.")
    for db in dbs:
        conn = sqlite3.connect(db)
        conn.row_factory = sqlite3.Row
        try:
            topics = {r["id"]: (r["name"], r["type"])
                      for r in conn.execute("SELECT id,name,type FROM topics")}
            cur = conn.execute(
                "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp ASC"
            )
            for row in cur:
                name, typ = topics.get(row["topic_id"], (None, None))
                parser = PARSERS.get(typ)
                if name is None or parser is None:
                    continue
                try:
                    msg = parser(bytes(row["data"]))
                except Exception:
                    continue
                yield name, msg, row["timestamp"] * 1e-9
        finally:
            conn.close()


# ═══════════════════════════════════════════════════════════════════════════
# math + stats helpers — identical to 3WD.
# ═══════════════════════════════════════════════════════════════════════════
def _yaw_ned_from_quat(qx, qy, qz, qw):
    yaw_enu = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return math.pi / 2.0 - yaw_enu


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _pctl(sorted_vals, q):
    if not sorted_vals:
        return None
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    idx = q * (len(sorted_vals) - 1)
    lo = int(math.floor(idx)); hi = int(math.ceil(idx))
    if lo == hi:
        return sorted_vals[lo]
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (idx - lo)


def _stat_block(vals):
    """RMS / median / p95 / max / mean over |vals| in cm, plus signed bias."""
    if not vals:
        return None
    absv = sorted(abs(v) for v in vals)
    rms = math.sqrt(sum(v * v for v in vals) / len(vals))
    return {
        "n": len(vals),
        "rms_cm": round(rms * 100, 2),
        "median_cm": round(_pctl(absv, 0.5) * 100, 2),
        "p95_cm": round(_pctl(absv, 0.95) * 100, 2),
        "max_cm": round(absv[-1] * 100, 2),
        "mean_signed_cm": round((sum(vals) / len(vals)) * 100, 2),
    }


def _nearest(series, t):
    """series = sorted [(t, payload...)]; return the row nearest time t (or None)."""
    if not series:
        return None
    lo, hi = 0, len(series) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    best = series[lo]
    if lo > 0 and abs(series[lo - 1][0] - t) < abs(best[0] - t):
        best = series[lo - 1]
    return best


# ═══════════════════════════════════════════════════════════════════════════
# collection
# ═══════════════════════════════════════════════════════════════════════════
class Series:
    def __init__(self):
        self.pose = []              # (t, x, y, yaw_ned) — local ENU-ish frame from odom
        self.vel_meas = []          # (t, speed) from odom twist
        self.state = []             # (t, mode, armed, connected)
        self.mission_waypoints = [] # (t, [(x,y)]) sparse staged points, latest non-empty kept
        self.nav_path = []          # (t, [(x,y)]) EVERY message, longest one is "the plan"
        self.runtime_nav_path = []  # (t, [(x,y)]) EVERY message — shrinks as mission advances
        self.active_waypoint = []   # (t, x, y) current target point
        self.marking_indices = []   # (t, [int]) trajectory_generator marking-index array
        self.path_types = []        # (t, [int]) trajectory_generator per-point type array
        self.rpp_scalars = {}       # topic -> [(t, value)] for every /rpp/*_mps,_mm,_m,_deg,etc scalar
        self.rpp_bools = {}         # topic -> [(t, bool)] for /rpp/*_active etc
        self.debug_raw = {}         # topic -> [(t, parsed_json_or_None, raw_text)]
        self.spray_active = []      # (t, bool)
        self.marking_active = []    # (t, bool)
        self.emergency_stop = []    # (t, bool)
        self.rtk_healthy = []       # (t, bool)
        self.rtk_correction_age = []  # (t, seconds)
        self.backend_heartbeat_healthy = []  # (t, bool)
        self.rover_backend_heartbeat = []    # (t, int) monotonic counter
        self.gps_raw = []           # (t, lat, lon, sats, h_acc) mavros_msgs/GPSRAW — receiver's own fix
        self.raw_fix = []           # (t, lat, lon, alt) NavSatFix on raw/fix
        self.global_fix = []        # (t, lat, lon, alt) NavSatFix on global — EKF-composited
        self.ekf_origin = None      # (lat, lon) gp_origin, latched
        self.topics_seen = {}       # name -> count

    def _longest(self, lst):
        best = None
        for _t, pts in lst:
            if pts and (best is None or len(pts) > len(best)):
                best = pts
        return best


def collect(bag_dir: str) -> Series:
    s = Series()
    RPP_SCALAR_TOPICS = {
        "/rpp/xtrack_mm", "/rpp/goal_distance_mm", "/rpp/along_track_remaining_mm",
        "/rpp/closest_goal_distance_mm", "/rpp/command_speed_mps",
        "/rpp/terminal_correction_deg", "/rpp/xtrack_speed_cap_mps",
        "/rpp/acceleration_progress_m", "/rpp/deceleration_progress_m",
        "/rpp/deceleration_remaining_m",
    }
    RPP_BOOL_TOPICS = {
        "/rpp/acceleration_active", "/rpp/deceleration_active",
        "/rpp/xtrack_speed_cap_active", "/rpp/terminal_precision_armed",
        "/rpp/terminal_bearing_frozen",
    }
    DEBUG_JSON_TOPICS = {
        "/rpp/geometry_debug", "/rpp/guidance_debug", "/rpp/speed_debug",
        "/rpp/tracking_debug", "/rpp/pivot_debug", "/rpp/accuracy",
        "/rpp/terminal_result", "/rpp/terminal_certificate",
        "/rpp/legacy_alignment_debug",
        "/mission_manager/status",
        "/mission_manager/segment_goal_metadata", "/mission_manager/point_event",
        "/mission_manager/execution_mode", "/trajectory_generator/status",
        "/trajectory_generator/path_signature",
    }
    for topic, m, t in read_bag(bag_dir):
        s.topics_seen[topic] = s.topics_seen.get(topic, 0) + 1
        if topic == "/mavros/local_position/odom":
            s.pose.append((t, m["x"], m["y"], _yaw_ned_from_quat(m["qx"], m["qy"], m["qz"], m["qw"])))
            s.vel_meas.append((t, math.hypot(m["lx"], m["ly"])))
        elif topic == "/mavros/state":
            s.state.append((t, m["mode"], m.get("armed"), m.get("connected")))
        elif topic == "/mission_waypoints":
            if m["poses"]:
                s.mission_waypoints.append((t, [(p[0], p[1]) for p in m["poses"]]))
        elif topic == "/nav_path":
            if m["poses"]:
                s.nav_path.append((t, [(p[0], p[1]) for p in m["poses"]]))
        elif topic == "/runtime_nav_path":
            if m["poses"]:
                s.runtime_nav_path.append((t, [(p[0], p[1]) for p in m["poses"]]))
        elif topic == "/active_waypoint":
            s.active_waypoint.append((t, m["x"], m["y"]))
        elif topic == "/trajectory_generator/marking_indices":
            s.marking_indices.append((t, m["data"]))
        elif topic == "/trajectory_generator/path_types":
            s.path_types.append((t, m["data"]))
        elif topic in RPP_SCALAR_TOPICS:
            s.rpp_scalars.setdefault(topic, []).append((t, m["data"]))
        elif topic in RPP_BOOL_TOPICS:
            s.rpp_bools.setdefault(topic, []).append((t, m["data"]))
        elif topic in DEBUG_JSON_TOPICS:
            s.debug_raw.setdefault(topic, []).append((t, m.get("json"), m.get("data")))
        elif topic == "/spray/active":
            s.spray_active.append((t, m["data"]))
        elif topic == "/marking_active":
            s.marking_active.append((t, m["data"]))
        elif topic == "/emergency_stop":
            s.emergency_stop.append((t, m["data"]))
        elif topic == "/rtk_correction_bridge/healthy":
            s.rtk_healthy.append((t, m["data"]))
        elif topic == "/rtk_correction_bridge/correction_age_sec":
            s.rtk_correction_age.append((t, m["data"]))
        elif topic == "/cmd_vel_bridge/backend_heartbeat_healthy":
            s.backend_heartbeat_healthy.append((t, m["data"]))
        elif topic == "/rover_backend/heartbeat":
            s.rover_backend_heartbeat.append((t, m["data"]))
        elif topic == "/mavros/gpsstatus/gps1/raw":
            if abs(m["lat"]) <= 90.0 and not (m["lat"] == 0.0 and m["lon"] == 0.0):
                s.gps_raw.append((t, m["lat"], m["lon"], m["sats"], m["h_acc"]))
        elif topic == "/mavros/global_position/raw/fix":
            if m["lat"] == m["lat"] and abs(m["lat"]) <= 90.0 \
                    and not (m["lat"] == 0.0 and m["lon"] == 0.0):
                s.raw_fix.append((t, m["lat"], m["lon"], m["alt"]))
        elif topic == "/mavros/global_position/global":
            if m["lat"] == m["lat"] and abs(m["lat"]) <= 90.0:
                s.global_fix.append((t, m["lat"], m["lon"], m["alt"]))
        elif topic == "/mavros/global_position/gp_origin":
            if (m["lat"] == m["lat"] and abs(m["lat"]) <= 90.0
                    and not (m["lat"] == 0.0 and m["lon"] == 0.0)):
                s.ekf_origin = (m["lat"], m["lon"])

    for lst in (s.pose, s.vel_meas, s.state, s.active_waypoint, s.spray_active,
                s.marking_active, s.emergency_stop, s.rtk_healthy,
                s.rtk_correction_age, s.backend_heartbeat_healthy,
                s.rover_backend_heartbeat, s.gps_raw, s.raw_fix, s.global_fix):
        lst.sort(key=lambda r: r[0])
    return s


# ═══════════════════════════════════════════════════════════════════════════
# analysis sections
# ═══════════════════════════════════════════════════════════════════════════
def analyze_tracking(s: Series) -> dict:
    """Budget 1: pose vs the plan.

    Two independent numbers, deliberately kept apart (see the skill's §2/§6
    'never quote a single controller debug scalar' rule):
      - `pose_vs_nav_path`: pose distance to /nav_path (the densified plan),
        computed HERE from raw geometry — this is the honest number.
      - `self_graded_accuracy`: whatever /rpp/accuracy claims for
        cross_track_error_mm — the controller grading itself against
        whatever it internally considers the active goal/span.
    Large divergence between the two is itself a finding — it means the
    controller's internal path differs from /nav_path (a densification or
    span-tracking bug), not that either number alone is "the accuracy".
    """
    out = {}
    plan = s._longest(s.nav_path)
    if plan and s.pose:
        vals = [_xtrack_to_polyline((x, y), plan) for _, x, y, _ in s.pose]
        out["pose_vs_nav_path"] = _stat_block(vals)
    else:
        out["pose_vs_nav_path"] = None
        out["pose_vs_nav_path_warn"] = "missing /nav_path or /mavros/local_position/odom"

    acc = s.debug_raw.get("/rpp/accuracy", [])
    self_vals = [j["cross_track_error_mm"] / 10.0 for _, j, _ in acc
                 if j and "cross_track_error_mm" in j]  # mm -> cm
    if self_vals:
        out["self_graded_accuracy_cm"] = {
            "n": len(self_vals),
            "mean_cm": round(sum(self_vals) / len(self_vals), 2),
            "max_cm": round(max(self_vals), 2),
        }
    else:
        out["self_graded_accuracy_cm"] = None

    if out.get("pose_vs_nav_path") and out.get("self_graded_accuracy_cm"):
        d = abs(out["pose_vs_nav_path"]["rms_cm"] - out["self_graded_accuracy_cm"]["mean_cm"])
        out["divergence_cm"] = round(d, 2)
        out["divergence_warn"] = (
            "pose-vs-nav_path and /rpp/accuracy disagree by more than 1cm — "
            "investigate before trusting either number" if d > 1.0 else None
        )
    return out


def analyze_traversal(s: Series, radius_m: float = COVERAGE_RADIUS_M) -> dict:
    """Coverage: fraction of /nav_path the pose actually got within radius_m of.
    manifest.identity.state == RUNNING/COMPLETE is not proof of full
    traversal — verify geometrically, same reasoning as the 3WD skill."""
    plan = s._longest(s.nav_path)
    if not plan or not s.pose:
        return {"coverage": None, "warn": "missing /nav_path or pose"}
    pose_xy = [(x, y) for _, x, y, _ in s.pose]
    near = 0
    for q in plan:
        if any(math.hypot(q[0] - p[0], q[1] - p[1]) < radius_m for p in pose_xy):
            near += 1
    frac = near / len(plan)
    verdict = ("COMPLETE" if frac >= COVERAGE_COMPLETE else
               "PARTIAL" if frac >= COVERAGE_PARTIAL else "SEVERE_GAP")
    return {"coverage": round(frac, 4), "reached": near, "total": len(plan), "verdict": verdict}


def analyze_geometry_fidelity(s: Series, warn_cm: float = WAYPOINT_FIDELITY_WARN_CM) -> dict:
    """Budget 2: does the densified /nav_path still pass close to every
    /mission_waypoints corner, or did trajectory_generator's densification
    drift away from a staged point? (Analogue of the 3WD tool's
    plan-vs-conditioned-path fidelity check, but comparing the SPARSE staged
    input against the DENSE generated plan, since that's the pair this stack
    actually exposes distinctly on the bus.)"""
    sparse = s._longest(s.mission_waypoints)
    dense = s._longest(s.nav_path)
    if not sparse or not dense:
        return {"warn": "missing /mission_waypoints or /nav_path"}
    misses = []
    for i, p in enumerate(sparse):
        d = _xtrack_to_polyline(p, dense) * 100.0  # cm
        if d > warn_cm:
            misses.append({"waypoint_index": i, "point": p, "distance_cm": round(d, 2)})
    return {
        "sparse_points": len(sparse), "dense_points": len(dense),
        "waypoints_off_by_gt_warn_cm": misses,
        "warn_threshold_cm": warn_cm,
    }


def analyze_absolute(s: Series) -> dict:
    """Budget 3 (partial): derive the local->WGS84 transform from the bag's
    own pose/global_fix pairs, independent of manifest metadata. Reports EKF
    origin + derived anchor only — no surveyed source is wired into this
    bundle format yet, so this cannot check absolute placement error, only
    anchor stability. See the skill's §4.3 for the full cross-mission method
    once a survey source is available."""
    out = {"ekf_origin": s.ekf_origin}
    if not s.pose or not s.global_fix:
        out["warn"] = "missing pose or /mavros/global_position/global"
        return out
    pairs = []
    gi = 0
    for t, x, y, _ in s.pose:
        while gi + 1 < len(s.global_fix) and s.global_fix[gi + 1][0] <= t:
            gi += 1
        gt, glat, glon, _ = s.global_fix[gi]
        if abs(gt - t) < 0.2:
            pairs.append((glat, glon, x, y))
    if len(pairs) < 5:
        out["warn"] = f"only {len(pairs)} pose/global_fix pairs within 0.2s — too few to anchor"
        return out
    mlat = sum(p[0] for p in pairs) / len(pairs)
    mn, me = _metres_per_degree(mlat)
    # /mavros/local_position/odom is ENU: pose x=East, y=North (verified
    # empirically 2026-08-31 — dx/d(east) and dy/d(north) both ratio ~1.0
    # over a 766-sample bundle; do not swap these without re-checking).
    lat0 = sum(glat - y / mn for glat, glon, x, y in pairs) / len(pairs)
    lon0 = sum(glon - x / me for glat, glon, x, y in pairs) / len(pairs)
    out["derived_anchor"] = {"lat": round(lat0, 7), "lon": round(lon0, 7), "n_pairs": len(pairs)}
    if s.ekf_origin:
        d = _geodesic_m(s.ekf_origin[0], s.ekf_origin[1], lat0, lon0)
        out["anchor_vs_gp_origin_cm"] = round(d * 100, 1)
    return out


def analyze_spray(s: Series) -> dict:
    """Edges of /spray/active and /marking_active: on/off durations."""
    def edges(series):
        out = []
        prev = None
        for t, v in series:
            if prev is not None and v != prev:
                out.append((t, v))
            prev = v
        return out

    return {
        "spray_active_edges": len(edges(s.spray_active)),
        "marking_active_edges": len(edges(s.marking_active)),
        "spray_active_total_on_s": _sum_true_duration(s.spray_active),
        "marking_active_total_on_s": _sum_true_duration(s.marking_active),
    }


def _sum_true_duration(series):
    if len(series) < 2:
        return 0.0
    total = 0.0
    for i in range(len(series) - 1):
        t0, v0 = series[i]
        t1, _ = series[i + 1]
        if v0:
            total += (t1 - t0)
    return round(total, 2)


def analyze_health(s: Series) -> dict:
    out = {}
    modes = sorted(set(m for _, m, _, _ in s.state))
    armed_true = sum(1 for _, _, a, _ in s.state if a)
    out["modes_seen"] = modes
    out["armed_samples"] = armed_true
    out["state_samples"] = len(s.state)
    out["emergency_stop_activations"] = sum(1 for _, v in s.emergency_stop if v)
    if s.rtk_correction_age:
        ages = [a for _, a in s.rtk_correction_age]
        out["rtk_correction_age_s"] = {"max": round(max(ages), 3), "mean": round(sum(ages) / len(ages), 3)}
    if s.rtk_healthy:
        unhealthy = sum(1 for _, v in s.rtk_healthy if not v)
        out["rtk_unhealthy_samples"] = unhealthy
    if s.backend_heartbeat_healthy:
        unhealthy = sum(1 for _, v in s.backend_heartbeat_healthy if not v)
        out["backend_heartbeat_unhealthy_samples"] = unhealthy
    if len(s.rover_backend_heartbeat) >= 2:
        gaps = []
        for i in range(1, len(s.rover_backend_heartbeat)):
            t0, v0 = s.rover_backend_heartbeat[i - 1]
            t1, v1 = s.rover_backend_heartbeat[i]
            if v1 > v0 + 1:
                gaps.append({"t": round(t1, 2), "missed": v1 - v0 - 1})
        out["heartbeat_gaps"] = gaps
    return out


def _p_first_pose_speed(s: Series):
    return s.vel_meas[0][1] if s.vel_meas else None


def analyze(root: str, out_dir: str | None = None) -> dict:
    bag_dir, manifest = _find_bag_dir(root)
    if bag_dir is None:
        sys.exit(f"ERROR: no bag found under {root}")
    s = collect(bag_dir)

    a = {
        "schema": "dyx4wd_analyze_mission/analysis@1",
        "bundle": os.path.basename(root.rstrip("/")),
        "bag_dir": bag_dir,
        "topics_seen": s.topics_seen,
        "manifest_identity": (manifest or {}).get("identity"),
        "tracking": analyze_tracking(s),
        "traversal": analyze_traversal(s),
        "geometry_fidelity": analyze_geometry_fidelity(s),
        "absolute": analyze_absolute(s),
        "spray": analyze_spray(s),
        "health": analyze_health(s),
    }
    first_speed = _p_first_pose_speed(s)
    if first_speed is not None and first_speed > TRUNC_START_SPEED_MPS:
        a["truncation_warning"] = (
            f"first recorded speed {first_speed:.2f} m/s > {TRUNC_START_SPEED_MPS} "
            f"— this recording may have started mid-mission, not at rest"
        )

    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, "analysis.json"), "w") as f:
            json.dump(a, f, indent=2, default=str)
        with open(os.path.join(out_dir, "report.txt"), "w") as f:
            f.write(_fmt_report(a))
    return a


def _fmt_report(a: dict) -> str:
    lines = []
    lines.append(f"DYX 4WD mission analysis — {a['bundle']}")
    lines.append(f"generated {datetime.now(timezone.utc).isoformat()}")
    lines.append("")
    if a.get("truncation_warning"):
        lines.append(f"⚠ {a['truncation_warning']}")
        lines.append("")

    lines.append("§1 TRACKING (pose vs plan — see divergence_warn before trusting either number)")
    t = a["tracking"]
    if t.get("pose_vs_nav_path"):
        pv = t["pose_vs_nav_path"]
        lines.append(f"  pose vs /nav_path:      RMS {pv['rms_cm']} cm  p95 {pv['p95_cm']} cm  max {pv['max_cm']} cm  (n={pv['n']})")
    else:
        lines.append(f"  pose vs /nav_path:      unavailable — {t.get('pose_vs_nav_path_warn')}")
    if t.get("self_graded_accuracy_cm"):
        sg = t["self_graded_accuracy_cm"]
        lines.append(f"  self-graded /rpp/accuracy: mean {sg['mean_cm']} cm  max {sg['max_cm']} cm  (n={sg['n']})")
    if t.get("divergence_warn"):
        lines.append(f"  ⚠ {t['divergence_warn']} (Δ={t['divergence_cm']} cm)")
    lines.append("")

    lines.append("§2 TRAVERSAL (coverage of /nav_path)")
    tr = a["traversal"]
    if tr.get("coverage") is not None:
        lines.append(f"  {tr['reached']}/{tr['total']} points reached within {COVERAGE_RADIUS_M}m  →  {tr['coverage']*100:.1f}%  [{tr['verdict']}]")
    else:
        lines.append(f"  unavailable — {tr.get('warn')}")
    lines.append("")

    lines.append("§3 GEOMETRY FIDELITY (sparse /mission_waypoints vs dense /nav_path)")
    g = a["geometry_fidelity"]
    if g.get("warn"):
        lines.append(f"  unavailable — {g['warn']}")
    else:
        lines.append(f"  {g['sparse_points']} staged points, {g['dense_points']} densified points")
        if g["waypoints_off_by_gt_warn_cm"]:
            lines.append(f"  ⚠ {len(g['waypoints_off_by_gt_warn_cm'])} staged point(s) sit >{g['warn_threshold_cm']}cm from the densified path:")
            for miss in g["waypoints_off_by_gt_warn_cm"]:
                lines.append(f"     waypoint[{miss['waypoint_index']}] {miss['distance_cm']} cm off")
        else:
            lines.append(f"  all staged points within {g['warn_threshold_cm']}cm of the densified path")
    lines.append("")

    lines.append("§4 ABSOLUTE (anchor stability only — no survey source wired in)")
    ab = a["absolute"]
    if ab.get("warn"):
        lines.append(f"  {ab['warn']}")
    if ab.get("derived_anchor"):
        da = ab["derived_anchor"]
        lines.append(f"  derived anchor: {da['lat']}, {da['lon']}  (n={da['n_pairs']} pairs)")
    if ab.get("ekf_origin"):
        lines.append(f"  gp_origin:      {ab['ekf_origin'][0]}, {ab['ekf_origin'][1]}")
    if ab.get("anchor_vs_gp_origin_cm") is not None:
        lines.append(f"  anchor vs gp_origin: {ab['anchor_vs_gp_origin_cm']} cm")
    lines.append("")

    lines.append("§5 SPRAY")
    sp = a["spray"]
    lines.append(f"  /spray/active:    {sp['spray_active_edges']} transitions, {sp['spray_active_total_on_s']}s ON")
    lines.append(f"  /marking_active:  {sp['marking_active_edges']} transitions, {sp['marking_active_total_on_s']}s ON")
    lines.append("")

    lines.append("§6 HEALTH")
    h = a["health"]
    lines.append(f"  modes seen: {h.get('modes_seen')}  ({h.get('armed_samples')}/{h.get('state_samples')} samples armed)")
    lines.append(f"  emergency_stop activations: {h.get('emergency_stop_activations')}")
    if "rtk_correction_age_s" in h:
        lines.append(f"  RTK correction age: mean {h['rtk_correction_age_s']['mean']}s  max {h['rtk_correction_age_s']['max']}s")
    if h.get("rtk_unhealthy_samples"):
        lines.append(f"  ⚠ RTK unhealthy for {h['rtk_unhealthy_samples']} sample(s)")
    if h.get("heartbeat_gaps"):
        lines.append(f"  ⚠ {len(h['heartbeat_gaps'])} rover_backend heartbeat gap(s)")
    lines.append("")

    lines.append("Topics seen (name: count):")
    for name in sorted(a["topics_seen"]):
        lines.append(f"  {name}: {a['topics_seen'][name]}")

    return "\n".join(lines) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bundle", help="bundle dir (with bag/+manifest.json) or bare rosbag2 dir")
    ap.add_argument("-o", "--out", default=None, help="output dir for analysis.json/report.txt (default: <bundle>)")
    ap.add_argument("--json-only", action="store_true", help="print analysis.json to stdout, skip report.txt")
    ap.add_argument("--quiet", action="store_true", help="only write files, no stdout report")
    args = ap.parse_args()

    out_dir = args.out or args.bundle
    a = analyze(args.bundle, out_dir=None if args.json_only else out_dir)

    if args.json_only:
        print(json.dumps(a, indent=2, default=str))
    elif not args.quiet:
        print(_fmt_report(a))
    return 0


if __name__ == "__main__":
    sys.exit(main())
