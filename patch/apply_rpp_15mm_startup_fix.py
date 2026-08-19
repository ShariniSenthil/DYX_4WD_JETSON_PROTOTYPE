#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
from datetime import datetime
import re
import shutil
import py_compile
import sys

ROOT = Path.home() / "rover_ws"
RPP = ROOT / "src/rpp_controller/rpp_controller/rpp_controller_node.py"
LAUNCH = ROOT / "src/rover_bringup/launch/rover.launch.py"

STAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
BACKUP = ROOT.parent / "rover_backups" / f"rpp_15mm_startup_fix_{STAMP}"

TARGET = "0.015"


def replace_parameter(text: str, name: str, value: str) -> tuple[str, int]:
    patterns = [
        (
            rf'self\.declare_parameter\(\s*"{re.escape(name)}"\s*,\s*[0-9.]+\s*\)',
            f'self.declare_parameter("{name}", {value})',
        ),
        (
            rf'("{re.escape(name)}"\s*:\s*)[0-9.]+',
            rf'\g<1>{value}',
        ),
        (
            rf'("{re.escape(name)}"\s*\)\s*:\s*)[0-9.]+',
            rf'\g<1>{value}',
        ),
    ]

    total = 0
    for pattern, repl in patterns:
        text, count = re.subn(
            pattern,
            repl,
            text,
            flags=re.MULTILINE | re.DOTALL,
        )
        total += count
    return text, total


def main() -> int:
    try:
        if not RPP.is_file():
            raise RuntimeError(f"Missing {RPP}")
        if not LAUNCH.is_file():
            raise RuntimeError(f"Missing {LAUNCH}")

        BACKUP.mkdir(parents=True, exist_ok=False)
        shutil.copy2(RPP, BACKUP / "rpp_controller_node.py")
        shutil.copy2(LAUNCH, BACKUP / "rover.launch.py")

        rpp = RPP.read_text()

        # Ensure both real marking gates use the same 15 mm radius.
        rpp, wp_count = replace_parameter(
            rpp,
            "waypoint_tolerance_m",
            TARGET,
        )
        rpp, xt_count = replace_parameter(
            rpp,
            "marking_stop_xtrack_limit_m",
            TARGET,
        )

        # Change strict validation:
        # old: marking_stop_xtrack_limit_m >= waypoint_tolerance_m -> error
        # new: only > is invalid; equality is allowed.
        old_patterns = [
            r'if\s+self\.marking_stop_xtrack_limit_m\s*>=\s*self\.waypoint_tolerance_m\s*:',
            r'if\s+marking_stop_xtrack_limit_m\s*>=\s*waypoint_tolerance_m\s*:',
        ]

        validation_changed = 0
        for pat in old_patterns:
            repl = (
                'if self.marking_stop_xtrack_limit_m > self.waypoint_tolerance_m:'
                if 'self\\.' in pat
                else 'if marking_stop_xtrack_limit_m > waypoint_tolerance_m:'
            )
            rpp, count = re.subn(pat, repl, rpp, count=1)
            validation_changed += count
            if count:
                break

        # Fallback: exact source line may be formatted differently.
        if validation_changed == 0:
            rpp, count = re.subn(
                r'(marking_stop_xtrack_limit_m\s*)>=(\s*.*waypoint_tolerance_m)',
                r'\1>\2',
                rpp,
                count=1,
            )
            validation_changed += count

        rpp = rpp.replace(
            "marking_stop_xtrack_limit_m must be below waypoint_tolerance_m",
            "marking_stop_xtrack_limit_m must not exceed waypoint_tolerance_m",
        )

        if wp_count == 0:
            raise RuntimeError("Could not find waypoint_tolerance_m")
        if xt_count == 0:
            raise RuntimeError("Could not find marking_stop_xtrack_limit_m")
        if validation_changed == 0:
            raise RuntimeError(
                "Could not locate the strict marking_stop_xtrack_limit_m validation"
            )

        RPP.write_text(rpp)

        launch = LAUNCH.read_text()
        launch, launch_xt = replace_parameter(
            launch,
            "marking_stop_xtrack_limit_m",
            TARGET,
        )
        launch, _ = replace_parameter(
            launch,
            "waypoint_tolerance_m",
            TARGET,
        )
        LAUNCH.write_text(launch)

        py_compile.compile(str(RPP), doraise=True)
        py_compile.compile(str(LAUNCH), doraise=True)

        print("RPP 15 mm startup fix applied.")
        print(f"waypoint_tolerance_m        = {TARGET} m")
        print(f"marking_stop_xtrack_limit_m = {TARGET} m")
        print("Validator now allows equality.")
        print("Python syntax check: PASS")
        print(f"Backup: {BACKUP}")
        print()
        print("Build:")
        print("  cd ~/rover_ws")
        print("  source /opt/ros/humble/setup.bash")
        print("  colcon build --symlink-install --packages-select rpp_controller rover_bringup")
        print("  source install/setup.bash")
        return 0

    except Exception as exc:
        print(f"PATCH FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
