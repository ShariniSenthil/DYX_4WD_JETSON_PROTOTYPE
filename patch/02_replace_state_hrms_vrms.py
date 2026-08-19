#!/usr/bin/env python3
from pathlib import Path
from datetime import datetime
import py_compile
import shutil
import sys

path = Path.home() / "rover_ws/src/rover_backend/rover_backend/state.py"
backup_dir = Path.home() / "rover_backups"
backup_dir.mkdir(parents=True, exist_ok=True)
backup = backup_dir / f"state_before_hrms_vrms_{datetime.now().strftime('%Y%m%d_%H%M%S')}.py"

BLOCK = '            "estimator": {\n                "available": False,\n                "source": "MAVLINK_ESTIMATOR_STATUS_230",\n                "horizontal_accuracy_m": None,\n                "horizontal_accuracy_mm": None,\n                "vertical_accuracy_m": None,\n                "vertical_accuracy_mm": None,\n                "vel_ratio": None,\n                "pos_horiz_ratio": None,\n                "pos_vert_ratio": None,\n                "mag_ratio": None,\n                "hagl_ratio": None,\n                "tas_ratio": None,\n                "flags": 0,\n                "absolute_horizontal_valid": False,\n                "absolute_vertical_valid": False,\n                "gps_glitch": False,\n                "accel_error": False,\n                "healthy": False,\n                "updated_at": now,\n            },\n'

try:
    if not path.is_file():
        raise RuntimeError(f"File not found: {path}")

    shutil.copy2(path, backup)
    text = path.read_text()

    if '"estimator": {' not in text:
        gps_pos = text.find('"gps": {')
        if gps_pos < 0:
            raise RuntimeError("Could not find gps state block")

        rtk_anchor = '\n            "rtk": {'
        rtk_pos = text.find(rtk_anchor, gps_pos)

        if rtk_pos < 0:
            raise RuntimeError("Could not find rtk block after gps")

        text = text[:rtk_pos + 1] + BLOCK + text[rtk_pos + 1:]

    path.write_text(text)
    py_compile.compile(str(path), doraise=True)

    print("state.py HRMS/VRMS replacement: PASS")
    print("Backup:", backup)

except Exception as exc:
    print("FAILED:", exc, file=sys.stderr)
    sys.exit(1)
