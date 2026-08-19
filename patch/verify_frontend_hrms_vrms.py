#!/usr/bin/env python3
from pathlib import Path
import sys


def find_repo_root(start: Path) -> Path:
    start = start.resolve()
    script_dir = Path(__file__).resolve().parent

    candidates = [
        start,
        *start.parents,
        script_dir,
        *script_dir.parents,
    ]

    seen = set()

    for candidate in candidates:
        if candidate in seen:
            continue
        seen.add(candidate)

        if (
            (candidate / "src/services/rtkService.ts").is_file()
            and
            (candidate / "src/components/missionreport/RTKInjectionScreen.tsx").is_file()
        ):
            return candidate

    raise RuntimeError("Frontend repository root not found.")


try:
    root = find_repo_root(Path.cwd())

    service = (
        root / "src/services/rtkService.ts"
    ).read_text(encoding="utf-8")

    screen = (
        root / "src/components/missionreport/RTKInjectionScreen.tsx"
    ).read_text(encoding="utf-8")

    checks = {
        "PX4_TELEMETRY imported":
            "PX4_RTK, PX4_TELEMETRY" in service,

        "getRtkEstimatorAccuracy exists":
            "export async function getRtkEstimatorAccuracy()" in service,

        "px4_hrms_mm mapped":
            "px4_hrms_mm" in service,

        "px4_vrms_mm mapped":
            "px4_vrms_mm" in service,

        "RTK screen estimator state":
            "setEstimatorAccuracy" in screen,

        "HRMS UI":
            "HRMS (PX4 EKF 1σ)" in screen,

        "VRMS UI":
            "VRMS (PX4 EKF 1σ)" in screen,

        "PX4 Estimator UI":
            "PX4 Estimator" in screen,
    }

    failed = False

    for label, ok in checks.items():
        status = "PASS" if ok else "FAIL"
        print(f"{status:4}  {label}")
        failed = failed or not ok

    if failed:
        sys.exit(1)

    print()
    print("FRONTEND SOURCE VERIFICATION: PASS")
    print()
    print("Next:")
    print("  npx tsc --noEmit")
    print("or run your normal Expo dev-client workflow.")

except Exception as exc:
    print(f"VERIFY FAILED: {exc}", file=sys.stderr)
    sys.exit(1)
