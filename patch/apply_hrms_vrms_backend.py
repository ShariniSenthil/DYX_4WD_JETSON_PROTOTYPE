#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
from datetime import datetime
import py_compile
import re
import shutil
import sys

ROOT = Path.home() / "rover_ws"
PKG = ROOT / "src" / "rover_backend" / "rover_backend"

ROS_BRIDGE = PKG / "ros_bridge.py"
STATE = PKG / "state.py"
SYSTEM_ROUTES = PKG / "system_routes.py"

BACKUP_ROOT = Path.home() / "rover_backups"
STAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
BACKUP = BACKUP_ROOT / f"before_hrms_vrms_{STAMP}"

CALLBACK = '''
    def _mavlink_estimator_status_callback(
        self,
        message: Mavlink,
    ) -> None:
        # Decode PX4 MAVLink ESTIMATOR_STATUS (#230).

        if int(message.msgid) != 230:
            return

        self._mark_ros_message()

        try:
            words = [int(value) for value in message.payload64]

            payload = struct.pack(
                "<" + ("Q" * len(words)),
                *words,
            )

            payload_length = int(message.len)
            if payload_length > 0:
                payload = payload[:payload_length]

            if len(payload) < 42:
                payload = payload.ljust(42, b"\\x00")

            (
                _time_usec,
                vel_ratio,
                pos_horiz_ratio,
                pos_vert_ratio,
                mag_ratio,
                hagl_ratio,
                tas_ratio,
                pos_horiz_accuracy,
                pos_vert_accuracy,
                flags,
            ) = struct.unpack(
                "<Q8fH",
                payload[:42],
            )

        except (
            struct.error,
            TypeError,
            ValueError,
            OverflowError,
        ):
            LOGGER.exception(
                "Failed to decode MAVLink ESTIMATOR_STATUS"
            )
            return

        horizontal_accuracy_m = _finite_float(
            pos_horiz_accuracy
        )
        vertical_accuracy_m = _finite_float(
            pos_vert_accuracy
        )

        if (
            horizontal_accuracy_m is not None
            and horizontal_accuracy_m <= 0.0
        ):
            horizontal_accuracy_m = None

        if (
            vertical_accuracy_m is not None
            and vertical_accuracy_m <= 0.0
        ):
            vertical_accuracy_m = None

        estimator_flags = int(flags)

        absolute_horizontal_valid = bool(
            estimator_flags & 16
        )
        absolute_vertical_valid = bool(
            estimator_flags & 32
        )
        gps_glitch = bool(
            estimator_flags & 1024
        )
        accel_error = bool(
            estimator_flags & 2048
        )

        rover_state.update(
            "estimator",
            available=(
                horizontal_accuracy_m is not None
                and vertical_accuracy_m is not None
            ),
            source="MAVLINK_ESTIMATOR_STATUS_230",
            horizontal_accuracy_m=horizontal_accuracy_m,
            horizontal_accuracy_mm=(
                horizontal_accuracy_m * 1000.0
                if horizontal_accuracy_m is not None
                else None
            ),
            vertical_accuracy_m=vertical_accuracy_m,
            vertical_accuracy_mm=(
                vertical_accuracy_m * 1000.0
                if vertical_accuracy_m is not None
                else None
            ),
            vel_ratio=_finite_float(
                vel_ratio
            ),
            pos_horiz_ratio=_finite_float(
                pos_horiz_ratio
            ),
            pos_vert_ratio=_finite_float(
                pos_vert_ratio
            ),
            mag_ratio=_finite_float(
                mag_ratio
            ),
            hagl_ratio=_finite_float(
                hagl_ratio
            ),
            tas_ratio=_finite_float(
                tas_ratio
            ),
            flags=estimator_flags,
            absolute_horizontal_valid=absolute_horizontal_valid,
            absolute_vertical_valid=absolute_vertical_valid,
            gps_glitch=gps_glitch,
            accel_error=accel_error,
            healthy=(
                absolute_horizontal_valid
                and absolute_vertical_valid
                and not gps_glitch
                and not accel_error
            ),
        )

'''

ESTIMATOR_STATE = '''            "estimator": {
                "available": False,
                "source": "MAVLINK_ESTIMATOR_STATUS_230",
                "horizontal_accuracy_m": None,
                "horizontal_accuracy_mm": None,
                "vertical_accuracy_m": None,
                "vertical_accuracy_mm": None,
                "vel_ratio": None,
                "pos_horiz_ratio": None,
                "pos_vert_ratio": None,
                "mag_ratio": None,
                "hagl_ratio": None,
                "tas_ratio": None,
                "flags": 0,
                "absolute_horizontal_valid": False,
                "absolute_vertical_valid": False,
                "gps_glitch": False,
                "accel_error": False,
                "healthy": False,
                "updated_at": now,
            },
'''

SUBSCRIPTION = '''        self.create_subscription(
            Mavlink,
            "/uas1/mavlink_source",
            self._mavlink_estimator_status_callback,
            sensor_qos,
        )

'''


def backup_files() -> None:
    BACKUP_ROOT.mkdir(parents=True, exist_ok=True)
    BACKUP.mkdir(parents=True, exist_ok=False)

    for path in (ROS_BRIDGE, STATE, SYSTEM_ROUTES):
        if not path.is_file():
            raise RuntimeError(f"Missing backend file: {path}")
        shutil.copy2(path, BACKUP / path.name)

    print(f"Backup created: {BACKUP}")


def patch_ros_bridge() -> None:
    text = ROS_BRIDGE.read_text()

    if "import struct\n" not in text:
        if "import os\n" not in text:
            raise RuntimeError("ros_bridge.py: import os anchor missing")
        text = text.replace(
            "import os\n",
            "import os\nimport struct\n",
            1,
        )

    if "from mavros_msgs.msg import Mavlink\n" not in text:
        if "from mavros_msgs.msg import GPSRAW\n" not in text:
            raise RuntimeError("ros_bridge.py: GPSRAW import anchor missing")
        text = text.replace(
            "from mavros_msgs.msg import GPSRAW\n",
            "from mavros_msgs.msg import GPSRAW\n"
            "from mavros_msgs.msg import Mavlink\n",
            1,
        )

    if '"/uas1/mavlink_source"' not in text:
        gps_sub = '''        self.create_subscription(
            GPSRAW,
            "/mavros/gpsstatus/gps1/raw",
            self._gps_status_callback,
            sensor_qos,
        )

'''
        if gps_sub not in text:
            raise RuntimeError(
                "ros_bridge.py: exact GPSRAW subscription anchor missing"
            )
        text = text.replace(
            gps_sub,
            gps_sub + SUBSCRIPTION,
            1,
        )

    if "def _mavlink_estimator_status_callback(" not in text:
        anchor = "    def _heading_callback(\n"
        if anchor not in text:
            raise RuntimeError(
                "ros_bridge.py: _heading_callback anchor missing"
            )
        text = text.replace(
            anchor,
            CALLBACK + anchor,
            1,
        )

    ROS_BRIDGE.write_text(text)
    print("Patched ros_bridge.py")


def patch_state() -> None:
    text = STATE.read_text()

    if '"estimator": {' not in text:
        gps_pos = text.find('"gps": {')
        if gps_pos < 0:
            raise RuntimeError("state.py: gps state block missing")

        rtk_anchor = '\n            "rtk": {'
        rtk_pos = text.find(rtk_anchor, gps_pos)
        if rtk_pos < 0:
            raise RuntimeError(
                "state.py: rtk state block after gps missing"
            )

        text = (
            text[:rtk_pos + 1]
            + ESTIMATOR_STATE
            + text[rtk_pos + 1:]
        )

    STATE.write_text(text)
    print("Patched state.py")


def patch_system_routes() -> None:
    text = SYSTEM_ROUTES.read_text()

    if '"estimator": estimator,' in text:
        print("system_routes.py already exposes estimator")
        return

    snapshot_match = re.search(
        r'(?m)^[ \t]*(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*=\s*'
        r'rover_state\.(?:snapshot|get_snapshot)\s*\(',
        text,
    )

    if snapshot_match is None:
        raise RuntimeError(
            "system_routes.py: could not identify rover_state snapshot variable"
        )

    snapshot_name = snapshot_match.group("name")

    rtk_match = re.search(
        r'(?m)^(?P<indent>[ \t]*)rtk\s*=.*$',
        text,
    )

    if rtk_match is None:
        raise RuntimeError(
            "system_routes.py: could not find rtk local assignment"
        )

    indent = rtk_match.group("indent")
    estimator_assignment = (
        f'{indent}estimator = dict('
        f'{snapshot_name}.get("estimator", {{}}))\n'
    )

    text = (
        text[:rtk_match.start()]
        + estimator_assignment
        + text[rtk_match.start():]
    )

    gps_return = '"gps": gps,\n'
    if gps_return not in text:
        raise RuntimeError(
            'system_routes.py: telemetry return field "gps": gps not found'
        )

    line_start = text.rfind("\n", 0, text.find(gps_return)) + 1
    field_indent = text[line_start:text.find(gps_return)]

    text = text.replace(
        gps_return,
        gps_return + f'{field_indent}"estimator": estimator,\n',
        1,
    )

    SYSTEM_ROUTES.write_text(text)
    print("Patched system_routes.py")


def check() -> None:
    for path in (ROS_BRIDGE, STATE, SYSTEM_ROUTES):
        py_compile.compile(
            str(path),
            doraise=True,
        )
    print("Python syntax check: PASS")


def main() -> int:
    try:
        backup_files()
        patch_ros_bridge()
        patch_state()
        patch_system_routes()
        check()

        print()
        print("HRMS/VRMS BACKEND PATCH COMPLETE")
        print()
        print("Build:")
        print("  cd ~/rover_ws")
        print("  source /opt/ros/humble/setup.bash")
        print("  colcon build --symlink-install --packages-select rover_backend")
        print("  source ~/rover_ws/install/setup.bash")
        print()
        print("Restart rover.launch.py after building.")
        return 0

    except Exception as exc:
        print(f"PATCH FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
