#!/usr/bin/env python3
from pathlib import Path
from datetime import datetime
import py_compile
import shutil
import sys

path = Path.home() / "rover_ws/src/rover_backend/rover_backend/ros_bridge.py"
backup_root = Path.home() / "rover_backups"
backup_root.mkdir(parents=True, exist_ok=True)
backup = backup_root / f"ros_bridge_before_hrms_vrms_api_fix_{datetime.now().strftime('%Y%m%d_%H%M%S')}.py"

GPS_UPDATE = '        # Mirror PX4 estimator accuracy into the existing GPS state.\n        # This guarantees /api/telemetry/latest exposes the values even\n        # if an older backend snapshot implementation omits new sections.\n        rover_state.update(\n            "gps",\n            px4_hrms_source="MAVLINK_ESTIMATOR_STATUS_230",\n            px4_hrms_m=horizontal_accuracy_m,\n            px4_hrms_mm=(\n                horizontal_accuracy_m * 1000.0\n                if horizontal_accuracy_m is not None\n                else None\n            ),\n            px4_vrms_m=vertical_accuracy_m,\n            px4_vrms_mm=(\n                vertical_accuracy_m * 1000.0\n                if vertical_accuracy_m is not None\n                else None\n            ),\n            px4_estimator_flags=estimator_flags,\n            px4_estimator_healthy=(\n                bool(estimator_flags & 16)\n                and bool(estimator_flags & 32)\n                and not bool(estimator_flags & 1024)\n                and not bool(estimator_flags & 2048)\n            ),\n        )\n\n'

try:
    if not path.is_file():
        raise RuntimeError(f"Missing file: {path}")

    text = path.read_text()

    if "def _mavlink_estimator_status_callback(" not in text:
        raise RuntimeError(
            "Estimator callback is not present. Apply the previous HRMS/VRMS patch first."
        )

    if "px4_hrms_mm=" in text:
        print("HRMS/VRMS GPS API mirror is already installed.")
    else:
        anchor = """        rover_state.update(
            "estimator",
"""
        if anchor not in text:
            raise RuntimeError(
                "Could not locate estimator state update inside callback."
            )

        shutil.copy2(path, backup)

        text = text.replace(
            anchor,
            GPS_UPDATE + anchor,
            1,
        )

        path.write_text(text)

    py_compile.compile(str(path), doraise=True)

    print("HRMS/VRMS API FIX: PASS")
    print()
    print("API fields will be exposed under telemetry.gps:")
    print("  px4_hrms_m")
    print("  px4_hrms_mm")
    print("  px4_vrms_m")
    print("  px4_vrms_mm")
    print("  px4_estimator_flags")
    print("  px4_estimator_healthy")
    if backup.exists():
        print("Backup:", backup)

except Exception as exc:
    print(f"PATCH FAILED: {exc}", file=sys.stderr)
    sys.exit(1)
