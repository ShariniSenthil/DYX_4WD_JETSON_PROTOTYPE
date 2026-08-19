#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
from datetime import datetime
import py_compile
import shutil
import sys

ROOT = Path.home() / "rover_ws"
RPP = ROOT / "src/rpp_controller/rpp_controller/rpp_controller_node.py"
BACKUP_ROOT = Path.home() / "rover_backups"

STAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
BACKUP = BACKUP_ROOT / f"rpp_15mm_startup_fix_v3_{STAMP}"

OLD_CONDITION = (
    "if not (0.0 < self.marking_stop_xtrack_limit < self.waypoint_tolerance):"
)
NEW_CONDITION = (
    "if not (0.0 < self.marking_stop_xtrack_limit <= self.waypoint_tolerance):"
)

OLD_ERROR = (
    'raise ValueError("marking_stop_xtrack_limit_m must be below waypoint_tolerance_m")'
)
NEW_ERROR = (
    'raise ValueError("marking_stop_xtrack_limit_m must be positive and not exceed waypoint_tolerance_m")'
)


def main() -> int:
    try:
        if not RPP.is_file():
            raise RuntimeError(f"Missing file: {RPP}")

        text = RPP.read_text()

        if OLD_CONDITION not in text:
            if NEW_CONDITION in text:
                print("Validator is already patched.")
            else:
                raise RuntimeError(
                    "Exact validator line not found. No changes made."
                )
        else:
            BACKUP_ROOT.mkdir(parents=True, exist_ok=True)
            BACKUP.mkdir(parents=True, exist_ok=False)
            shutil.copy2(RPP, BACKUP / RPP.name)

            text = text.replace(
                OLD_CONDITION,
                NEW_CONDITION,
                1,
            )

            if OLD_ERROR in text:
                text = text.replace(
                    OLD_ERROR,
                    NEW_ERROR,
                    1,
                )

            RPP.write_text(text)

        py_compile.compile(
            str(RPP),
            doraise=True,
        )

        print("RPP 15 mm startup validator fix: PASS")
        print()
        print("Validator now allows:")
        print("  marking_stop_xtrack_limit_m <= waypoint_tolerance_m")
        print()
        print("Expected live values:")
        print("  waypoint_tolerance_m        = 0.015")
        print("  marking_stop_xtrack_limit_m = 0.015")
        print()
        print("Python syntax check: PASS")
        if BACKUP.exists():
            print(f"Backup: {BACKUP}")
        return 0

    except Exception as exc:
        print(f"PATCH FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
