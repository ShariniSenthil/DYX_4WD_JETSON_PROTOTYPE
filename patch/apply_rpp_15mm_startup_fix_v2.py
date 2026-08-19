#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
from datetime import datetime
import py_compile
import re
import shutil
import sys

ROOT = Path.home() / "rover_ws"
RPP = ROOT / "src/rpp_controller/rpp_controller/rpp_controller_node.py"
LAUNCH = ROOT / "src/rover_bringup/launch/rover.launch.py"
BACKUP_ROOT = Path.home() / "rover_backups"

STAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
BACKUP = BACKUP_ROOT / f"rpp_15mm_startup_fix_v2_{STAMP}"

TARGET = "0.015"
ERROR_TEXT = "marking_stop_xtrack_limit_m must be below waypoint_tolerance_m"


def set_parameter(text: str, name: str, value: str) -> tuple[str, int]:
    total = 0

    # Python declare_parameter("name", number)
    pattern = (
        r'(self\.declare_parameter\(\s*'
        + re.escape(f'"{name}"')
        + r'\s*,\s*)[0-9.]+'
    )
    text, count = re.subn(
        pattern,
        rf'\g<1>{value}',
        text,
        count=1,
        flags=re.MULTILINE | re.DOTALL,
    )
    total += count

    # launch dict: "name": number
    pattern = rf'("{re.escape(name)}"\s*:\s*)[0-9.]+'
    text, count = re.subn(
        pattern,
        rf'\g<1>{value}',
        text,
        flags=re.MULTILINE,
    )
    total += count

    # LaunchConfiguration("name"): number
    pattern = rf'("{re.escape(name)}"\s*\)\s*:\s*)[0-9.]+'
    text, count = re.subn(
        pattern,
        rf'\g<1>{value}',
        text,
        flags=re.MULTILINE,
    )
    total += count

    return text, total


def patch_validator(text: str) -> tuple[str, str]:
    error_pos = text.find(ERROR_TEXT)
    if error_pos < 0:
        raise RuntimeError(
            f'Could not find exact validation error text: "{ERROR_TEXT}"'
        )

    # Search a bounded region before the raise. This prevents changing unrelated >= comparisons.
    window_start = max(0, error_pos - 1800)
    before = text[window_start:error_pos]

    # Find the nearest preceding "if ...:" block mentioning both parameters.
    candidates = list(
        re.finditer(
            r'(?ms)^[ \t]*if\s+(?P<condition>.*?marking_stop_xtrack_limit_m'
            r'.*?waypoint_tolerance_m.*?)\s*:\s*$',
            before,
        )
    )

    if not candidates:
        # Reverse parameter order fallback.
        candidates = list(
            re.finditer(
                r'(?ms)^[ \t]*if\s+(?P<condition>.*?waypoint_tolerance_m'
                r'.*?marking_stop_xtrack_limit_m.*?)\s*:\s*$',
                before,
            )
        )

    if not candidates:
        raise RuntimeError(
            "Found the ValueError text, but could not identify its preceding if-condition."
        )

    match = candidates[-1]
    condition = match.group("condition")

    if ">=" not in condition:
        raise RuntimeError(
            "The validation condition was found but it does not contain >=.\n"
            f"Condition was: {condition!r}"
        )

    new_condition = condition.replace(">=", ">", 1)

    absolute_start = window_start + match.start("condition")
    absolute_end = window_start + match.end("condition")

    patched = (
        text[:absolute_start]
        + new_condition
        + text[absolute_end:]
    )

    patched = patched.replace(
        ERROR_TEXT,
        "marking_stop_xtrack_limit_m must not exceed waypoint_tolerance_m",
        1,
    )

    return patched, condition


def main() -> int:
    try:
        if not RPP.is_file():
            raise RuntimeError(f"Missing source file: {RPP}")
        if not LAUNCH.is_file():
            raise RuntimeError(f"Missing launch file: {LAUNCH}")

        BACKUP_ROOT.mkdir(parents=True, exist_ok=True)
        BACKUP.mkdir(parents=True, exist_ok=False)

        shutil.copy2(RPP, BACKUP / "rpp_controller_node.py")
        shutil.copy2(LAUNCH, BACKUP / "rover.launch.py")

        rpp = RPP.read_text()

        rpp, wp_count = set_parameter(
            rpp,
            "waypoint_tolerance_m",
            TARGET,
        )
        rpp, xt_count = set_parameter(
            rpp,
            "marking_stop_xtrack_limit_m",
            TARGET,
        )

        if wp_count == 0:
            raise RuntimeError("waypoint_tolerance_m was not found in RPP source")
        if xt_count == 0:
            raise RuntimeError(
                "marking_stop_xtrack_limit_m was not found in RPP source"
            )

        rpp, old_condition = patch_validator(rpp)

        RPP.write_text(rpp)

        launch = LAUNCH.read_text()
        launch, launch_wp = set_parameter(
            launch,
            "waypoint_tolerance_m",
            TARGET,
        )
        launch, launch_xt = set_parameter(
            launch,
            "marking_stop_xtrack_limit_m",
            TARGET,
        )
        LAUNCH.write_text(launch)

        py_compile.compile(str(RPP), doraise=True)
        py_compile.compile(str(LAUNCH), doraise=True)

        print("RPP 15 mm V2 startup fix: APPLIED")
        print()
        print("Old validation condition:")
        print(" ", " ".join(old_condition.split()))
        print()
        print("Final required values:")
        print("  waypoint_tolerance_m        = 0.015")
        print("  marking_stop_xtrack_limit_m = 0.015")
        print("  equality is now accepted")
        print()
        print("Python syntax check: PASS")
        print(f"Backup: {BACKUP}")
        print()
        print("BUILD:")
        print("  cd ~/rover_ws")
        print("  source /opt/ros/humble/setup.bash")
        print("  colcon build --symlink-install --packages-select rpp_controller rover_bringup")
        print("  source ~/rover_ws/install/setup.bash")
        return 0

    except Exception as exc:
        print(f"PATCH FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
