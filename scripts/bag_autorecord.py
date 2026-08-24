#!/usr/bin/env python3
"""Auto rosbag recorder for the DYX 4WD marking rover.

Pure observer. Polls GET /api/mission/status and runs `ros2 bag record` while
a mission is starting or active. Never blocks Start/Stop. Never talks to PX4.

Env:
  ROVER_API_BASE              default http://127.0.0.1:5001
  ROVER_USERNAME / ROVER_PASSWORD
  ROVER_MACHINE_TOKEN         optional pre-issued session token
  ROVER_MACHINE_TOKEN_FILE    optional token file
  ROVER_AUTH_DISABLED         1 = no auth header (status will 401 in production)
  BAGS_DIR                    default ~/bags_jet
  BAG_RECORD_ALL              1 = ros2 bag record -a
  BAG_POLL_S                  default 0.2
  BAG_MAX_S                   default 7200
  BAG_API_GRACE_S             default 8
  BAG_QOS_OVERRIDES           default <repo>/config/rosbag_qos_overrides.yaml
  BAG_MISSION_FILE            default ~/rover_ws/missions/mission.csv
  BAG_MISSION_METADATA        default ~/.local/share/dyx_rover/runtime/mission_metadata.json
  BAG_MIN_FREE_BYTES          default 5 GiB
  BAG_LOW_FREE_BYTES          default 2 GiB
  BAG_MAX_TOTAL_BYTES         default 50 GiB
  BAG_FCU_PARAMS              default 1
  BAG_PARAM_DUMP_TIMEOUT_S    default 15
  BAG_AUTO_ANALYZE            default 0
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from datetime import datetime, timedelta, timezone

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

API_BASE = os.environ.get("ROVER_API_BASE", "http://127.0.0.1:5001").rstrip("/")
STATUS_URL = f"{API_BASE}/api/mission/status"
LOGIN_URL = f"{API_BASE}/api/auth/login"

TOKEN_FILE = os.environ.get(
    "ROVER_MACHINE_TOKEN_FILE",
    os.path.join(_REPO_ROOT, "config", "bag_autorecord.token"),
)
AUTH_OFF = os.environ.get("ROVER_AUTH_DISABLED", "").lower() in {
    "1",
    "true",
    "yes",
}
BAGS_DIR = os.environ.get("BAGS_DIR", os.path.expanduser("~/bags_jet"))
RECORD_ALL = os.environ.get("BAG_RECORD_ALL", "0") == "1"
POLL_S = float(os.environ.get("BAG_POLL_S", "0.2"))
MAX_S = float(os.environ.get("BAG_MAX_S", "7200"))
CSV_LOGGER = os.environ.get("BAG_ENABLE_CSV_LOGGER", "1") == "1"
_FIELD_LOGGER = os.path.join(_REPO_ROOT, "scripts", "field_test_logger.py")
API_GRACE_S = float(os.environ.get("BAG_API_GRACE_S", "8"))

IST = timezone(timedelta(hours=5, minutes=30), name="IST")
CAPTURE_FCU_PARAMS = os.environ.get("BAG_FCU_PARAMS", "1") == "1"
PARAM_DUMP_TIMEOUT_S = float(os.environ.get("BAG_PARAM_DUMP_TIMEOUT_S", "15"))
AUTO_ANALYZE = os.environ.get("BAG_AUTO_ANALYZE", "0") == "1"
_ANALYZER = os.path.join(_REPO_ROOT, "scripts", "analyze_mission.py")

MISSION_FILE = os.environ.get(
    "BAG_MISSION_FILE",
    os.path.expanduser("~/rover_ws/missions/mission.csv"),
)
MISSION_METADATA = os.environ.get(
    "BAG_MISSION_METADATA",
    os.path.expanduser("~/.local/share/dyx_rover/runtime/mission_metadata.json"),
)

# Motion-active mission_manager states. PREPARING is excluded (RTK wait).
ACTIVE_STATES = {"RUNNING", "PAUSED", "WAITING_FOR_NEXT"}
# Start service publishes these on /mission_manager/status before state=RUNNING.
START_STAGES = {
    "PRECHECK",
    "ZERO_SETPOINT_SETTLE",
    "SWITCHING_OFFBOARD",
    "ARMING",
    "FINAL_CHECK",
}

# PX4 6X marking rover. Missing names are listed, never silently invented.
FCU_PARAM_NAMES = [
    "COM_OF_LOSS_T",
    "COM_OBL_ACT",
    "COM_RC_IN_MODE",
    "PWM_AUX_FUNC5",
    "PWM_AUX_MIN5",
    "PWM_AUX_MAX5",
    "PWM_AUX_DIS5",
    "EKF2_GPS_P_NOISE",
    "EKF2_GPS_V_NOISE",
    "EKF2_HGT_REF",
    "EKF2_GPS_CTRL",
    "EKF2_GPS_YAW_OFF",
    "GND_SPEED_MAX",
    "RA_MAX",
    "RD_MAX",
]

PARAM_NODES = [
    "/mission_manager",
    "/trajectory_generator",
    "/rpp_controller",
    "/cmd_vel_bridge",
    "/spray_controller",
    "/ntrip_to_px4_node",
]

WATCH_SERVICES = ["bag-autorecord"]

TOPICS = [
    "/mavros/local_position/odom",
    "/mavros/setpoint_raw/local",
    "/rpp/velocity_ned",
    "/rpp/command_speed_mps",
    "/mavros/state",
    "/mavros/extended_state",
    "/mavros/estimator_status",
    "/mavros/sys_status",
    "/mavros/battery",
    "/mavros/statustext/recv",
    "/mavros/rc/in",
    "/mavros/manual_control/control",
    "/mavros/home_position/home",
    "/mavros/global_position/compass_hdg",
    "/mavros/imu/data",
    "/mavros/imu/mag",
    "/nav_path",
    "/mission_waypoints",
    "/trajectory_generator/path_types",
    "/trajectory_generator/marking_indices",
    "/trajectory_generator/path_signature",
    "/trajectory_generator/ready",
    "/trajectory_generator/status",
    "/runtime_nav_path",
    "/segment_goal",
    "/mission_manager/segment_goal_metadata",
    "/active_waypoint",
    "/mission_manager/status",
    "/mission_manager/point_event",
    "/mission_manager/execution_mode",
    "/mission_enable",
    "/emergency_stop",
    "/marking_active",
    "/mission_complete",
    "/rover_backend/heartbeat",
    "/cmd_vel_bridge/backend_heartbeat_healthy",
    "/rpp/acceleration_active",
    "/rpp/acceleration_progress_m",
    "/rpp/deceleration_active",
    "/rpp/deceleration_progress_m",
    "/rpp/deceleration_remaining_m",
    "/rpp/xtrack_speed_cap_active",
    "/rpp/xtrack_speed_cap_mps",
    "/rpp/terminal_precision_armed",
    "/rpp/terminal_bearing_frozen",
    "/rpp/terminal_correction_deg",
    "/rpp/xtrack_mm",
    "/rpp/goal_distance_mm",
    "/rpp/along_track_remaining_mm",
    "/rpp/closest_goal_distance_mm",
    "/rpp/accuracy",
    "/rpp/geometry_debug",
    "/rpp/guidance_debug",
    "/rpp/speed_debug",
    "/rpp/tracking_debug",
    "/rpp/pivot_debug",
    "/rpp/terminal_certificate",
    "/rpp/terminal_result",
    "/spray/status",
    "/spray/result",
    "/spray/active",
    "/spray/complete",
    "/spray/config",
    "/mavros/gpsstatus/gps1/raw",
    "/mavros/global_position/raw/fix",
    "/mavros/global_position/global",
    "/mavros/global_position/gp_origin",
    "/rtk_correction_bridge/healthy",
    "/rtk_correction_bridge/correction_age_sec",
]

_DEFAULT_QOS_OVERRIDES = os.path.join(
    _REPO_ROOT, "config", "rosbag_qos_overrides.yaml"
)
QOS_OVERRIDES = os.environ.get("BAG_QOS_OVERRIDES", _DEFAULT_QOS_OVERRIDES)

_GiB = 1024**3
MIN_FREE_BYTES = int(float(os.environ.get("BAG_MIN_FREE_BYTES", str(5 * _GiB))))
LOW_FREE_BYTES = int(float(os.environ.get("BAG_LOW_FREE_BYTES", str(2 * _GiB))))
MAX_TOTAL_BYTES = int(float(os.environ.get("BAG_MAX_TOTAL_BYTES", str(50 * _GiB))))

MANIFEST_NAME = "manifest.json"
INCOMPLETE_SENTINEL = "INCOMPLETE"
_FINALISING: set[str] = set()
_FINALISING_LOCK = threading.Lock()

_SECRET_KV = re.compile(
    r"(?i)([a-z0-9_.\-]*(?:token|password|passwd|secret|api[_-]?key|authorization)[a-z0-9_.\-]*)"
    r"\s*[:=]\s*['\"]?([^\s'\",;}]+)"
)
_URL_CREDS = re.compile(
    r"(?i)\b([a-z][a-z0-9+.\-]*://)([^/\s:@]+):([^/\s@]+)@"
)

_session_token: str | None = None
_session_lock = threading.Lock()


def log(msg: str) -> None:
    print(
        f"[bag_autorecord] {datetime.now().isoformat(timespec='seconds')} {msg}",
        flush=True,
    )


def _redact(value):
    if not isinstance(value, str) or not value:
        return value
    value = _URL_CREDS.sub(r"\1\2:***@", value)
    return _SECRET_KV.sub(lambda match: f"{match.group(1)}=***", value)


def _redact_obj(obj):
    if isinstance(obj, dict):
        return {key: _redact_obj(value) for key, value in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_redact_obj(value) for value in obj]
    return _redact(obj)


def _now_utc() -> datetime:
    return datetime.now(timezone.utc)


def _stamp(moment: datetime) -> dict:
    return {
        "utc": moment.astimezone(timezone.utc).isoformat(timespec="seconds"),
        "ist": moment.astimezone(IST).strftime("%Y-%m-%d %H:%M:%S %Z"),
        "epoch": round(moment.timestamp(), 3),
    }


def _safe_name(name: str | None) -> str:
    base = re.sub(r"[^A-Za-z0-9._-]+", "_", (name or "mission").strip()) or "mission"
    return base[:60]


def _run(cmd: list[str], timeout: float = 5.0) -> str | None:
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        if result.returncode != 0:
            return None
        return result.stdout
    except Exception:
        return None


def _sha256_file(path: str) -> str | None:
    try:
        digest = hashlib.sha256()
        with open(path, "rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()
    except OSError:
        return None


def _dir_bytes(path: str) -> int:
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass
    return total


def _bundle_integrity(bundle_dir: str, exclude: set[str]) -> dict:
    files: dict[str, dict] = {}
    for root, _dirs, names in os.walk(bundle_dir):
        for name in names:
            full = os.path.join(root, name)
            relative = os.path.relpath(full, bundle_dir)
            if relative in exclude:
                continue
            files[relative] = {
                "bytes": os.path.getsize(full) if os.path.exists(full) else None,
                "sha256": _sha256_file(full),
            }
    return {"file_count": len(files), "files": files}


def _env_or_file_token() -> str | None:
    env_token = os.environ.get("ROVER_MACHINE_TOKEN", "").strip()
    if env_token:
        return env_token
    try:
        with open(TOKEN_FILE, encoding="utf-8") as handle:
            value = handle.read().strip()
        return value or None
    except OSError:
        return None


def _login() -> str | None:
    username = os.environ.get("ROVER_USERNAME", "").strip()
    password = os.environ.get("ROVER_PASSWORD", "")
    if not username or not password:
        return None
    payload = json.dumps({"username": username, "password": password}).encode()
    request = urllib.request.Request(
        LOGIN_URL,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            body = json.loads(response.read().decode())
    except Exception as error:
        log(f"login failed: {type(error).__name__}")
        return None
    token = str(body.get("token") or body.get("access_token") or "").strip()
    if not token:
        log("login returned no token")
        return None
    return token


def _current_token(*, force_login: bool = False) -> str | None:
    global _session_token
    with _session_lock:
        if AUTH_OFF:
            return None
        if force_login:
            _session_token = None
        if _session_token:
            return _session_token
        _session_token = _env_or_file_token() or _login()
        return _session_token


def _auth_headers(*, force_login: bool = False) -> dict[str, str]:
    token = _current_token(force_login=force_login)
    if not token:
        return {}
    return {
        "Authorization": f"Bearer {token}",
        "X-Rover-Token": token,
    }


def _mission_should_record(mission: dict) -> bool:
    state = str(mission.get("state") or "").strip().upper()
    stage = str(mission.get("start_stage") or "IDLE").strip().upper()
    return state in ACTIVE_STATES or stage in START_STAGES


def poll_status() -> tuple[bool, dict | None, str | None]:
    """Return (ok, mission_dict, http_error). ok=False on transport/parse failure."""

    def _open(force_login: bool) -> tuple[int, dict | None]:
        request = urllib.request.Request(STATUS_URL, headers=_auth_headers(force_login=force_login))
        try:
            with urllib.request.urlopen(request, timeout=1.5) as response:
                body = json.loads(response.read().decode())
            return 200, body
        except urllib.error.HTTPError as error:
            return error.code, None
        except Exception:
            return 0, None

    status, body = _open(False)
    if status in {401, 403}:
        status, body = _open(True)
    if status != 200 or not isinstance(body, dict):
        return False, None, str(status or "transport")
    mission = body.get("mission")
    if not isinstance(mission, dict):
        return False, None, "missing_mission"
    return True, mission, None


def _git_sha() -> str | None:
    output = _run(
        ["git", "-C", _REPO_ROOT, "rev-parse", "--short=12", "HEAD"],
        timeout=3.0,
    )
    return output.strip() if output else None


def _service_states() -> dict:
    states = {}
    for service in WATCH_SERVICES:
        output = _run(["systemctl", "is-active", service], timeout=3.0)
        states[service] = output.strip() if output else "unknown"
    return states


def _environment() -> dict:
    return {
        "git_sha": _git_sha(),
        "services": _service_states(),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "hostname": socket.gethostname(),
        "recorder_pid": os.getpid(),
        "workspace": _REPO_ROOT,
    }


def _parse_ros2_param_value(output: str | None):
    if not output:
        return None
    token = output.strip().splitlines()[-1].strip()
    if not token or "not set" in output.lower() or "error" in output.lower():
        return None
    lower = token.lower()
    if lower in {"true", "false"}:
        return lower == "true"
    try:
        return int(token)
    except ValueError:
        try:
            return float(token)
        except ValueError:
            return token.strip("'\"")


def _fcu_params() -> dict:
    if not CAPTURE_FCU_PARAMS:
        return {"captured": False, "reason": "disabled", "values": {}, "missing": []}
    values: dict = {}
    missing: list[str] = []
    method = None
    for name in FCU_PARAM_NAMES:
        value = _parse_ros2_param_value(
            _run(
                ["ros2", "param", "get", "--hide-type", "/mavros/param", name],
                timeout=4.0,
            )
        )
        if value is not None:
            method = method or "ros2_param"
        else:
            missing.append(name)
        values[name] = value
    return {
        "captured": bool(values) and len(missing) < len(FCU_PARAM_NAMES),
        "method": method,
        "missing": missing,
        "values": values,
    }


def _identity_from_mission(mission: dict) -> dict:
    keep = (
        "mission_id",
        "filename",
        "checksum_sha256",
        "coordinate_mode",
        "extension_mode",
        "dummy_point_distance_m",
        "row_transition_threshold_m",
        "execution_mode",
        "state",
        "start_stage",
        "total_points",
        "navigation_point_count",
        "dummy_point_count",
    )
    identity = {key: mission.get(key) for key in keep}
    identity["available"] = True
    return identity


def _write_manifest(bundle_dir: str, manifest: dict) -> None:
    safe = _redact_obj(manifest)
    temporary = os.path.join(bundle_dir, MANIFEST_NAME + ".tmp")
    final = os.path.join(bundle_dir, MANIFEST_NAME)
    try:
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(safe, handle, indent=2, sort_keys=False)
        os.replace(temporary, final)
    except OSError as error:
        log(f"  manifest write failed: {error}")


def _read_manifest(bundle_dir: str) -> dict | None:
    try:
        with open(os.path.join(bundle_dir, MANIFEST_NAME), encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, json.JSONDecodeError):
        return None


def _free_bytes(path: str) -> int:
    try:
        return shutil.disk_usage(path).free
    except OSError:
        return 1 << 62


def _list_bundles(bags_dir: str) -> list[str]:
    found: list[str] = []
    try:
        for name in os.listdir(bags_dir):
            full = os.path.join(bags_dir, name)
            if os.path.isdir(full) and os.path.exists(os.path.join(full, MANIFEST_NAME)):
                found.append(full)
    except OSError:
        return []
    found.sort(key=lambda path: os.path.getmtime(path) if os.path.exists(path) else 0)
    return found


def _preflight_free_space(bags_dir: str) -> bool:
    free = _free_bytes(bags_dir)
    if free < MIN_FREE_BYTES:
        log(
            f"WARN low disk: {free / _GiB:.1f} GiB free < floor "
            f"{MIN_FREE_BYTES / _GiB:.1f} GiB — refusing capture"
        )
        return False
    return True


def _enforce_retention(bags_dir: str, protect: set[str] | None = None) -> None:
    try:
        with _FINALISING_LOCK:
            shielded = set(_FINALISING) | (protect or set())
        bundles = _list_bundles(bags_dir)
        if not bundles:
            return
        total = sum(_dir_bytes(bundle) for bundle in bundles)
        candidates = [bundle for bundle in bundles[:-1] if bundle not in shielded]
        while candidates and (
            total > MAX_TOTAL_BYTES or _free_bytes(bags_dir) < LOW_FREE_BYTES
        ):
            victim = candidates.pop(0)
            size = _dir_bytes(victim)
            try:
                shutil.rmtree(victim)
                total -= size
                log(
                    f"  retention: removed {os.path.basename(victim)} "
                    f"({size / _GiB:.2f} GiB)"
                )
            except OSError as error:
                log(f"  retention: could not remove {victim}: {error}")
                break
    except Exception as error:
        log(f"  retention error (ignored): {error}")


def reconcile_incomplete(bags_dir: str) -> None:
    for bundle in _list_bundles(bags_dir):
        manifest = _read_manifest(bundle)
        if manifest is None:
            continue
        outcome = manifest.get("outcome") or {}
        if outcome.get("recorder_end"):
            continue
        log(f"reconcile: {os.path.basename(bundle)} → INCOMPLETE")
        try:
            with open(os.path.join(bundle, INCOMPLETE_SENTINEL), "w", encoding="utf-8") as handle:
                handle.write(_now_utc().isoformat(timespec="seconds") + "\n")
        except OSError:
            pass
        outcome["status"] = "INCOMPLETE"
        outcome["recorder_end"] = _stamp(_now_utc())
        outcome["note"] = "finalised by crash reconciliation on daemon start"
        outcome["integrity"] = _bundle_integrity(
            bundle, exclude={MANIFEST_NAME, MANIFEST_NAME + ".tmp"}
        )
        manifest["outcome"] = outcome
        _write_manifest(bundle, manifest)


def _spawn_analyzer(bundle_dir: str) -> None:
    if not AUTO_ANALYZE or not os.path.isfile(_ANALYZER):
        return
    try:
        log_path = os.path.join(bundle_dir, "analyze.log")
        handle = open(log_path, "w", encoding="utf-8")
        subprocess.Popen(
            ["python3", _ANALYZER, bundle_dir, "--quiet"],
            start_new_session=True,
            stdout=handle,
            stderr=subprocess.STDOUT,
        )
        log("  analyser spawned")
    except Exception as error:
        log(f"  analyser spawn failed (ignored): {error}")


def _copy_if_exists(source: str, destination_dir: str, name: str) -> str | None:
    if not source or not os.path.isfile(source):
        return None
    target = os.path.join(destination_dir, name)
    try:
        shutil.copy2(source, target)
        return name
    except OSError as error:
        log(f"  snapshot {name} skipped: {error}")
        return None


def _dump_node_params(bundle_dir: str) -> dict:
    params_dir = os.path.join(bundle_dir, "params")
    os.makedirs(params_dir, exist_ok=True)
    dumped: dict[str, str] = {}
    missing: list[str] = []
    errors: dict[str, str] = {}
    for node in PARAM_NODES:
        safe = node.strip("/").replace("/", "_")
        target = os.path.join(params_dir, f"{safe}.yaml")
        try:
            result = subprocess.run(
                ["ros2", "param", "dump", node],
                capture_output=True,
                text=True,
                timeout=PARAM_DUMP_TIMEOUT_S,
            )
        except subprocess.TimeoutExpired:
            missing.append(node)
            errors[node] = f"timeout_after_{PARAM_DUMP_TIMEOUT_S:g}s"
            log(
                "WARN ROS parameter snapshot timed out | "
                f"node={node} timeout_s={PARAM_DUMP_TIMEOUT_S:g}"
            )
            continue
        except Exception as error:
            missing.append(node)
            errors[node] = type(error).__name__
            log(
                "WARN ROS parameter snapshot failed | "
                f"node={node} error={type(error).__name__}"
            )
            continue
        if result.returncode != 0 or not result.stdout.strip():
            missing.append(node)
            errors[node] = (
                f"returncode_{result.returncode}"
                if result.returncode != 0
                else "empty_output"
            )
            log(
                "WARN ROS parameter snapshot rejected | "
                f"node={node} reason={errors[node]}"
            )
            continue
        try:
            with open(target, "w", encoding="utf-8") as handle:
                handle.write(result.stdout)
        except OSError as error:
            missing.append(node)
            errors[node] = type(error).__name__
            log(
                "WARN ROS parameter snapshot write failed | "
                f"node={node} error={type(error).__name__}"
            )
            continue
        dumped[node] = f"params/{safe}.yaml"
    return {"dumped": dumped, "missing": missing, "errors": errors}


class Recorder:
    def __init__(self) -> None:
        self.proc: subprocess.Popen | None = None
        self.logger_proc: subprocess.Popen | None = None
        self.bundle_dir: str | None = None
        self.bag_dir: str | None = None
        self.manifest: dict | None = None
        self.start_t: float = 0.0
        self._last_refuse_log: float = 0.0
        self._last_start_fail: float = 0.0
        self._finalise_threads: list[threading.Thread] = []

    @property
    def active(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    @property
    def session_open(self) -> bool:
        return self.proc is not None

    def start(self, mission: dict) -> None:
        os.makedirs(BAGS_DIR, exist_ok=True)
        if time.time() - self._last_start_fail < 5.0:
            return
        if not _preflight_free_space(BAGS_DIR):
            now = time.time()
            if now - self._last_refuse_log > 30.0:
                self._last_refuse_log = now
            return

        _enforce_retention(BAGS_DIR)

        started = _now_utc()
        stamp = started.astimezone(IST).strftime("%Y%m%d_%H%M%S")
        label = mission.get("filename") or mission.get("mission_id") or "mission"
        name = f"{_safe_name(str(label))}_{stamp}"
        self.bundle_dir = os.path.join(BAGS_DIR, name)
        os.makedirs(self.bundle_dir, exist_ok=True)
        self.bag_dir = os.path.join(self.bundle_dir, "bag")

        command = ["ros2", "bag", "record", "-o", self.bag_dir]
        if not RECORD_ALL and QOS_OVERRIDES and os.path.isfile(QOS_OVERRIDES):
            command += ["--qos-profile-overrides-path", QOS_OVERRIDES]
        elif not RECORD_ALL:
            log(f"WARN qos overrides missing ({QOS_OVERRIDES}) — latched /nav_path may be empty")
        command += ["-a"] if RECORD_ALL else TOPICS
        log(
            f"START {self.bundle_dir} "
            f"({('ALL topics' if RECORD_ALL else f'{len(TOPICS)} topics')})"
        )
        bag_log = open(
            os.path.join(self.bundle_dir, "rosbag_console.log"),
            "w",
            encoding="utf-8",
        )
        self.proc = subprocess.Popen(
            command,
            start_new_session=True,
            stdout=bag_log,
            stderr=subprocess.STDOUT,
        )
        time.sleep(0.4)
        exit_code = self.proc.poll()
        if exit_code is not None:
            self._last_start_fail = time.time()
            log(f"ERROR ros2 bag exited immediately rc={exit_code}")
            self.stop("rosbag_start_failed")
            return
        self.start_t = time.time()
        self._start_csv_logger(self.bundle_dir)

        copied = [
            name
            for name in (
                _copy_if_exists(MISSION_FILE, self.bundle_dir, "mission.csv"),
                _copy_if_exists(
                    MISSION_METADATA, self.bundle_dir, "mission_metadata.json"
                ),
            )
            if name
        ]
        self.manifest = {
            "schema": "dyx4wd_bag_autorecord/manifest@1",
            "bundle": name,
            "identity": _identity_from_mission(mission),
            "timestamps": {
                "recorder_start": _stamp(started),
                "mission_start_observed": _stamp(started),
            },
            "as_run_config": {
                "ros_params": {"status": "pending"},
                "fcu_params": {"captured": False, "values": {}},
                "recorder": {
                    "topics": ("ALL" if RECORD_ALL else TOPICS),
                    "qos_overrides": (
                        QOS_OVERRIDES if os.path.isfile(QOS_OVERRIDES or "") else None
                    ),
                    "copied_files": copied,
                },
            },
            "environment": _environment(),
            "outcome": {"status": "RECORDING", "recorder_end": None},
        }
        _write_manifest(self.bundle_dir, self.manifest)
        thread = threading.Thread(
            target=self._capture_ros_params,
            args=(self.bundle_dir,),
            name="bag-param-dump",
            daemon=True,
        )
        thread.start()

    def _start_csv_logger(self, bundle_dir: str) -> None:
        if not CSV_LOGGER or not os.path.isfile(_FIELD_LOGGER):
            return
        try:
            env = os.environ.copy()
            env["ROVER_FIELD_LOG_DIR"] = bundle_dir
            handle = open(
                os.path.join(bundle_dir, "field_test_logger_console.log"),
                "w",
                encoding="utf-8",
            )
            self.logger_proc = subprocess.Popen(
                ["python3", _FIELD_LOGGER],
                env=env,
                start_new_session=True,
                stdout=handle,
                stderr=subprocess.STDOUT,
            )
            log("  field_test_logger started → telemetry.csv")
        except Exception as error:
            self.logger_proc = None
            log(f"  field_test_logger skipped: {error}")

    def _stop_csv_logger(self) -> None:
        proc = self.logger_proc
        self.logger_proc = None
        if proc is None:
            return
        try:
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass

    def _capture_ros_params(self, bundle_dir: str) -> None:
        try:
            dumped = _dump_node_params(bundle_dir)
            if self.manifest is not None and self.bundle_dir == bundle_dir:
                self.manifest["as_run_config"]["ros_params"] = dumped
                _write_manifest(bundle_dir, self.manifest)
        except Exception as error:
            log(f"  ros param dump failed (ignored): {error}")

    def stop(self, reason: str) -> None:
        if self.proc is None:
            return
        proc, bundle, manifest = self.proc, self.bundle_dir, self.manifest
        self.proc = None
        self.bundle_dir = None
        self.bag_dir = None
        self.manifest = None
        self._stop_csv_logger()
        log(f"STOP ({reason}) → {os.path.basename(bundle or '')}")
        with _FINALISING_LOCK:
            _FINALISING.add(bundle)
        thread = threading.Thread(
            target=self._finalise,
            args=(proc, bundle, manifest, reason),
            name=f"finalise-{os.path.basename(bundle or '')}",
            daemon=False,
        )
        self._finalise_threads = [item for item in self._finalise_threads if item.is_alive()]
        self._finalise_threads.append(thread)
        thread.start()

    def _finalise(self, proc, bundle, manifest, reason: str) -> None:
        try:
            self._finalise_inner(proc, bundle, manifest, reason)
        except Exception as error:
            log(f"  finalise thread error (bag is safe): {error}")
        finally:
            with _FINALISING_LOCK:
                _FINALISING.discard(bundle)

    def _finalise_inner(self, proc, bundle, manifest, reason: str) -> None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            log("  finalise slow — SIGTERM")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=10)
            except Exception:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception as error:
            log(f"  stop error: {error}")

        if not bundle or manifest is None:
            return
        try:
            ended = _now_utc()
            manifest["as_run_config"]["fcu_params"] = _fcu_params()
            manifest["timestamps"]["recorder_end"] = _stamp(ended)
            manifest["timestamps"]["mission_end_observed"] = _stamp(ended)
            manifest["outcome"] = {
                "status": "COMPLETE",
                "means": "recorder finalised cleanly; see bag coverage for mission length",
                "mission_end_reason": reason,
                "recorder_end": _stamp(ended),
                "integrity": _bundle_integrity(
                    bundle, exclude={MANIFEST_NAME, MANIFEST_NAME + ".tmp"}
                ),
            }
            _write_manifest(bundle, manifest)
            log(f"  saved: {bundle}")
        except Exception as error:
            log(f"  finalise-manifest error (bag is safe): {error}")
        _spawn_analyzer(bundle)
        _enforce_retention(
            BAGS_DIR,
            protect={self.bundle_dir} if self.bundle_dir else None,
        )


def main() -> int:
    recorder = Recorder()
    stop_flag = {"v": False}

    def _handle_signal(_signum, _frame):
        stop_flag["v"] = True

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    os.makedirs(BAGS_DIR, exist_ok=True)
    qos_ok = os.path.isfile(QOS_OVERRIDES)
    token = _current_token()
    log(
        f"watching {STATUS_URL}  bags→{BAGS_DIR}  "
        f"auth={'off' if AUTH_OFF else ('ok' if token else 'MISSING')}  "
        f"topics={len(TOPICS)}  max_s={MAX_S:.0f}  "
        f"qos={'yes' if qos_ok else 'MISSING'}"
    )
    if not qos_ok:
        log(f"WARN qos file missing: {QOS_OVERRIDES}")
    if not AUTH_OFF and not token:
        log("WARN no API token — login failed; status polls will 401 until credentials work")
    try:
        reconcile_incomplete(BAGS_DIR)
    except Exception as error:
        log(f"reconcile error (ignored): {error}")

    api_fail_since: float | None = None

    while not stop_flag["v"]:
        if recorder.session_open and not recorder.active:
            recorder.stop("rosbag_exited")

        ok, mission, error = poll_status()
        if ok and mission is not None:
            api_fail_since = None
            active = _mission_should_record(mission)
            if active and not recorder.session_open:
                recorder.start(mission)
            elif (not active) and recorder.session_open:
                state = str(mission.get("state") or "terminal")
                recorder.stop(f"mission {state}")
        elif recorder.session_open:
            api_fail_since = api_fail_since or time.time()
            if time.time() - api_fail_since > API_GRACE_S:
                recorder.stop(f"api_unreachable:{error}")
                api_fail_since = None

        if recorder.active and (time.time() - recorder.start_t) > MAX_S:
            recorder.stop("max_duration_cap")

        time.sleep(POLL_S)

    if recorder.active:
        recorder.stop("service_shutdown")
    for thread in list(recorder._finalise_threads):
        thread.join(timeout=60)
        if thread.is_alive():
            log(f"WARN {thread.name} still running — bundle will reconcile INCOMPLETE")
    log("exiting")
    return 0


if __name__ == "__main__":
    sys.exit(main())
