#!/usr/bin/env python3
from pathlib import Path
from datetime import datetime
import py_compile
import re
import shutil
import sys

path = Path.home() / "rover_ws/src/rover_backend/rover_backend/system_routes.py"
backup_dir = Path.home() / "rover_backups"
backup_dir.mkdir(parents=True, exist_ok=True)
backup = backup_dir / f"system_routes_before_hrms_vrms_{datetime.now().strftime('%Y%m%d_%H%M%S')}.py"

try:
    if not path.is_file():
        raise RuntimeError(f"File not found: {path}")

    shutil.copy2(path, backup)
    text = path.read_text()

    if '"estimator": estimator,' not in text:
        gps_return_pos = text.find('"gps": gps,')
        if gps_return_pos < 0:
            raise RuntimeError('Could not find telemetry field "gps": gps')

        prefix = text[:gps_return_pos]

        matches = list(re.finditer(
            r'(?m)^[ \t]*(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*=\s*'
            r'rover_state\.(?:snapshot|get_snapshot)\s*\(',
            prefix,
        ))

        if not matches:
            raise RuntimeError("Could not identify rover_state snapshot variable")

        snapshot_name = matches[-1].group("name")

        rtk_matches = list(re.finditer(
            r'(?m)^(?P<indent>[ \t]*)rtk\s*=.*$',
            prefix,
        ))

        if not rtk_matches:
            raise RuntimeError("Could not find local rtk assignment")

        rtk_match = rtk_matches[-1]
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

        gps_return_pos = text.find('"gps": gps,')
        line_start = text.rfind("\n", 0, gps_return_pos) + 1
        indent_return = text[line_start:gps_return_pos]
        line_end = text.find("\n", gps_return_pos)
        insert_at = line_end + 1

        text = (
            text[:insert_at]
            + f'{indent_return}"estimator": estimator,\n'
            + text[insert_at:]
        )

    path.write_text(text)
    py_compile.compile(str(path), doraise=True)

    print("system_routes.py HRMS/VRMS replacement: PASS")
    print("Backup:", backup)

except Exception as exc:
    print("FAILED:", exc, file=sys.stderr)
    sys.exit(1)
