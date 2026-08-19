#!/usr/bin/env python3
from pathlib import Path
from datetime import datetime
import py_compile
import shutil
import sys

path = Path.home() / "rover_ws/src/rover_backend/rover_backend/ros_bridge.py"
backup_dir = Path.home() / "rover_backups"
backup_dir.mkdir(parents=True, exist_ok=True)
backup = backup_dir / f"ros_bridge_before_hrms_vrms_{datetime.now().strftime('%Y%m%d_%H%M%S')}.py"

CALLBACK = '    def _mavlink_estimator_status_callback(\n        self,\n        message: Mavlink,\n    ) -> None:\n        # Decode PX4 MAVLink ESTIMATOR_STATUS (#230).\n\n        if int(message.msgid) != 230:\n            return\n\n        self._mark_ros_message()\n\n        try:\n            words = [int(value) for value in message.payload64]\n\n            payload = struct.pack(\n                "<" + ("Q" * len(words)),\n                *words,\n            )\n\n            payload_length = int(message.len)\n\n            if payload_length > 0:\n                payload = payload[:payload_length]\n\n            payload = payload.ljust(\n                42,\n                b"\\x00",\n            )\n\n            (\n                _time_usec,\n                vel_ratio,\n                pos_horiz_ratio,\n                pos_vert_ratio,\n                mag_ratio,\n                hagl_ratio,\n                tas_ratio,\n                pos_horiz_accuracy,\n                pos_vert_accuracy,\n                flags,\n            ) = struct.unpack(\n                "<Q8fH",\n                payload[:42],\n            )\n\n        except (\n            struct.error,\n            TypeError,\n            ValueError,\n            OverflowError,\n        ):\n            LOGGER.exception(\n                "Failed to decode MAVLink ESTIMATOR_STATUS"\n            )\n            return\n\n        horizontal_accuracy_m = _finite_float(\n            pos_horiz_accuracy\n        )\n        vertical_accuracy_m = _finite_float(\n            pos_vert_accuracy\n        )\n\n        if (\n            horizontal_accuracy_m is not None\n            and horizontal_accuracy_m <= 0.0\n        ):\n            horizontal_accuracy_m = None\n\n        if (\n            vertical_accuracy_m is not None\n            and vertical_accuracy_m <= 0.0\n        ):\n            vertical_accuracy_m = None\n\n        estimator_flags = int(flags)\n\n        rover_state.update(\n            "estimator",\n            available=(\n                horizontal_accuracy_m is not None\n                and vertical_accuracy_m is not None\n            ),\n            source="MAVLINK_ESTIMATOR_STATUS_230",\n            horizontal_accuracy_m=horizontal_accuracy_m,\n            horizontal_accuracy_mm=(\n                horizontal_accuracy_m * 1000.0\n                if horizontal_accuracy_m is not None\n                else None\n            ),\n            vertical_accuracy_m=vertical_accuracy_m,\n            vertical_accuracy_mm=(\n                vertical_accuracy_m * 1000.0\n                if vertical_accuracy_m is not None\n                else None\n            ),\n            vel_ratio=_finite_float(vel_ratio),\n            pos_horiz_ratio=_finite_float(pos_horiz_ratio),\n            pos_vert_ratio=_finite_float(pos_vert_ratio),\n            mag_ratio=_finite_float(mag_ratio),\n            hagl_ratio=_finite_float(hagl_ratio),\n            tas_ratio=_finite_float(tas_ratio),\n            flags=estimator_flags,\n            absolute_horizontal_valid=bool(estimator_flags & 16),\n            absolute_vertical_valid=bool(estimator_flags & 32),\n            gps_glitch=bool(estimator_flags & 1024),\n            accel_error=bool(estimator_flags & 2048),\n            healthy=(\n                bool(estimator_flags & 16)\n                and bool(estimator_flags & 32)\n                and not bool(estimator_flags & 1024)\n                and not bool(estimator_flags & 2048)\n            ),\n        )\n\n'
SUBSCRIPTION = '        self.create_subscription(\n            Mavlink,\n            "/uas1/mavlink_source",\n            self._mavlink_estimator_status_callback,\n            sensor_qos,\n        )\n\n'

try:
    if not path.is_file():
        raise RuntimeError(f"File not found: {path}")

    shutil.copy2(path, backup)
    text = path.read_text()

    if "import struct\n" not in text:
        anchor = "import os\n"
        if anchor not in text:
            raise RuntimeError("Could not find import os")
        text = text.replace(anchor, anchor + "import struct\n", 1)

    if "from mavros_msgs.msg import Mavlink\n" not in text:
        anchor = "from mavros_msgs.msg import GPSRAW\n"
        if anchor not in text:
            raise RuntimeError("Could not find GPSRAW import")
        text = text.replace(
            anchor,
            anchor + "from mavros_msgs.msg import Mavlink\n",
            1,
        )

    if "def _mavlink_estimator_status_callback(" not in text:
        if '"/uas1/mavlink_source"' not in text:
            gps_sub = """        self.create_subscription(
            GPSRAW,
            "/mavros/gpsstatus/gps1/raw",
            self._gps_status_callback,
            sensor_qos,
        )

"""
            if gps_sub not in text:
                raise RuntimeError("Could not find GPSRAW subscription block")
            text = text.replace(
                gps_sub,
                gps_sub + SUBSCRIPTION,
                1,
            )
        else:
            text = text.replace(
                "self._mavlink_callback,",
                "self._mavlink_estimator_status_callback,",
                1,
            )

        anchor = "    def _heading_callback(\n"
        if anchor not in text:
            raise RuntimeError("Could not find _heading_callback")
        text = text.replace(anchor, CALLBACK + anchor, 1)

    path.write_text(text)
    py_compile.compile(str(path), doraise=True)

    print("ros_bridge.py HRMS/VRMS replacement: PASS")
    print("Backup:", backup)

except Exception as exc:
    print("FAILED:", exc, file=sys.stderr)
    sys.exit(1)
