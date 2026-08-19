#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
from datetime import datetime
import py_compile
import re
import shutil
import sys

ROOT = Path.home() / "rover_ws"

FILES = {
    "rpp": ROOT / "src/rpp_controller/rpp_controller/rpp_controller_node.py",
    "mission": ROOT / "src/mission_manager/mission_manager/mission_manager_node.py",
    "launch": ROOT / "src/rover_bringup/launch/rover.launch.py",
    "backend_state": ROOT / "src/rover_backend/rover_backend/state.py",
    "backend_bridge": ROOT / "src/rover_backend/rover_backend/ros_bridge.py",
}

SPRAY_DIR = ROOT / "src/spray_controller"

STAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
BACKUP = ROOT / f"backup_before_15mm_marking_{STAMP}"

TARGET_M = 0.015
TARGET_MM = 15.0


def require(path: Path) -> None:
    if not path.is_file():
        raise RuntimeError(f"Required file not found: {path}")


def replace_declared_parameter(text: str, name: str, value: str) -> tuple[str, int]:
    pattern = (
        r'self\.declare_parameter\(\s*"'
        + re.escape(name)
        + r'"\s*,\s*[0-9.]+\s*\)'
    )
    return re.subn(
        pattern,
        f'self.declare_parameter("{name}", {value})',
        text,
        flags=re.MULTILINE | re.DOTALL,
    )


def backup_files() -> None:
    BACKUP.mkdir(parents=True, exist_ok=False)

    for key, path in FILES.items():
        shutil.copy2(path, BACKUP / f"{key}__{path.name}")

    if SPRAY_DIR.is_dir():
        spray_backup = BACKUP / "spray_controller"
        spray_backup.mkdir()
        for path in SPRAY_DIR.rglob("*.py"):
            rel = path.relative_to(SPRAY_DIR)
            dest = spray_backup / rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, dest)

    print(f"Backup created: {BACKUP}")


def patch_rpp(path: Path) -> None:
    text = path.read_text()

    text, count = replace_declared_parameter(
        text,
        "waypoint_tolerance_m",
        "0.015",
    )
    if count == 0:
        raise RuntimeError(
            "RPP: waypoint_tolerance_m declaration not found"
        )

    text, _ = replace_declared_parameter(
        text,
        "telemetry_accuracy_target_m",
        "0.015",
    )

    path.write_text(text)
    print("RPP waypoint radius -> 0.015 m")


def patch_mission(path: Path) -> None:
    text = path.read_text()

    text, count = replace_declared_parameter(
        text,
        "marking_tolerance_m",
        "0.015",
    )
    if count == 0:
        # Handle multiline declaration with comments/formatting.
        pattern = (
            r'self\.declare_parameter\(\s*'
            r'"marking_tolerance_m"\s*,\s*'
            r'[0-9.]+\s*,?\s*\)'
        )
        text, count = re.subn(
            pattern,
            'self.declare_parameter(\n'
            '            "marking_tolerance_m",\n'
            '            0.015,\n'
            '        )',
            text,
            count=1,
            flags=re.MULTILINE | re.DOTALL,
        )

    if count == 0:
        raise RuntimeError(
            "Mission manager: marking_tolerance_m declaration not found"
        )

    text, _ = replace_declared_parameter(
        text,
        "accuracy_target_m",
        "0.015",
    )

    path.write_text(text)
    print("Mission marking radius -> 0.015 m")


def patch_launch(path: Path) -> None:
    text = path.read_text()

    keys = (
        "waypoint_tolerance_m",
        "marking_tolerance_m",
        "accuracy_target_m",
        "telemetry_accuracy_target_m",
    )

    total = 0

    for key in keys:
        patterns = [
            (
                rf'("{re.escape(key)}"\s*:\s*)[0-9.]+',
                rf'\g<1>0.015',
            ),
            (
                rf'("{re.escape(key)}"\s*\)\s*:\s*)[0-9.]+',
                rf'\g<1>0.015',
            ),
        ]

        for pattern, repl in patterns:
            text, count = re.subn(pattern, repl, text)
            total += count

    if total == 0:
        raise RuntimeError(
            "Launch: no waypoint/marking accuracy target found"
        )

    # Comments only.
    text = text.replace("radial <=30 mm", "radial <=15 mm")
    text = text.replace("radial <= 30 mm", "radial <= 15 mm")
    text = text.replace("30 mm radius", "15 mm radius")

    path.write_text(text)
    print(f"Launch runtime targets -> 0.015 m ({total} replacements)")


def patch_backend_state(path: Path) -> None:
    text = path.read_text()

    # Default API target for the waypoint/marking radius.
    text = re.sub(
        r'("accuracy_target_m"\s*:\s*)[0-9.]+',
        r'\g<1>0.015',
        text,
        count=1,
    )

    text = re.sub(
        r'("accuracy_target_mm"\s*:\s*)[0-9.]+',
        r'\g<1>15.0',
        text,
        count=1,
    )

    path.write_text(text)
    print("Backend API accuracy target -> 15 mm radius")


def patch_backend_bridge(path: Path) -> None:
    text = path.read_text()

    # Only patch fallback values associated with accuracy_target_mm.
    pattern = (
        r'(payload\.get\("accuracy_target_mm"\)\s*,\s*)'
        r'[0-9.]+'
    )
    text, count1 = re.subn(
        pattern,
        r'\g<1>15.0',
        text,
        count=1,
        flags=re.MULTILINE,
    )

    # Current code revisions may use "... or 30.0".
    target_pos = text.find('payload.get("accuracy_target_mm")')
    if target_pos >= 0:
        window_end = min(len(text), target_pos + 500)
        window = text[target_pos:window_end]
        new_window, count2 = re.subn(
            r'\bor\s+[0-9.]+',
            'or 15.0',
            window,
            count=1,
        )
        if count2:
            text = (
                text[:target_pos]
                + new_window
                + text[window_end:]
            )

    path.write_text(text)
    print("Backend fallback target -> 15 mm radius")


def patch_spray_controller() -> None:
    if not SPRAY_DIR.is_dir():
        print("Spray controller directory not found; no spray file patched.")
        return

    accepted_names = (
        "spray_tolerance_m",
        "marking_tolerance_m",
        "waypoint_tolerance_m",
        "accuracy_target_m",
    )

    total = 0

    for path in SPRAY_DIR.rglob("*.py"):
        text = path.read_text()
        original = text

        for name in accepted_names:
            text, count = replace_declared_parameter(
                text,
                name,
                "0.015",
            )
            total += count

            text, count = re.subn(
                rf'("{re.escape(name)}"\s*:\s*)[0-9.]+',
                rf'\g<1>0.015',
                text,
            )
            total += count

        if text != original:
            path.write_text(text)
            print("Spray tolerance patched:", path)

    if total == 0:
        print(
            "Spray controller has no independent positional tolerance parameter. "
            "This is OK if spray is triggered by mission_manager after the "
            "15 mm marking gate."
        )
    else:
        print(f"Spray positional tolerance replacements: {total}")


def syntax_check() -> None:
    check_files = list(FILES.values())

    if SPRAY_DIR.is_dir():
        check_files.extend(SPRAY_DIR.rglob("*.py"))

    for path in check_files:
        py_compile.compile(
            str(path),
            doraise=True,
        )

    print("Python syntax check: PASS")


def main() -> int:
    try:
        for path in FILES.values():
            require(path)

        backup_files()

        patch_rpp(FILES["rpp"])
        patch_mission(FILES["mission"])
        patch_launch(FILES["launch"])
        patch_backend_state(FILES["backend_state"])
        patch_backend_bridge(FILES["backend_bridge"])
        patch_spray_controller()

        syntax_check()

        print()
        print("15 MM MARKING PATCH COMPLETE")
        print()
        print("Final real marking/spray acceptance:")
        print("  radius   = 0.015 m = 15 mm")
        print("  diameter = 0.030 m = 30 mm")
        print()
        print("Use 0.015 m in radial-distance comparisons.")
        print("Do NOT use 0.030 m as the tolerance radius.")
        print()
        print("Build:")
        print("  cd ~/rover_ws")
        print("  source /opt/ros/humble/setup.bash")
        print("  colcon build --symlink-install --packages-select \\")
        print("    rpp_controller mission_manager rover_bringup rover_backend spray_controller")
        print("  source install/setup.bash")
        print()
        print("Restart your normal rover launch.")
        print()
        print(f"Backup: {BACKUP}")
        return 0

    except Exception as exc:
        print(f"PATCH FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
