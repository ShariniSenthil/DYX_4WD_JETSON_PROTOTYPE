#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
from datetime import datetime
import shutil
import sys


def find_repo_root(start: Path) -> Path:
    start = start.resolve()

    candidates = [start, *start.parents]

    # Also handle running the script directly from an extracted patch folder.
    script_dir = Path(__file__).resolve().parent
    candidates.extend([script_dir, *script_dir.parents])

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

    raise RuntimeError(
        "Could not find frontend repo root. "
        "Run this script from DYX_GCS_V-refactor or place the patch folder inside that repo."
    )


def replace_once(
    text: str,
    old: str,
    new: str,
    label: str,
) -> str:
    if new in text:
        print(f"{label}: already patched")
        return text

    if old not in text:
        raise RuntimeError(
            f"{label}: expected source anchor was not found. "
            "Your local frontend may differ from the checked GitHub version."
        )

    print(f"{label}: patched")
    return text.replace(old, new, 1)


def main() -> int:
    try:
        root = find_repo_root(Path.cwd())

        rtk_service = root / "src/services/rtkService.ts"
        screen = root / "src/components/missionreport/RTKInjectionScreen.tsx"

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_root = Path.home() / "DYX_frontend_backups" / f"hrms_vrms_{stamp}"
        backup_root.mkdir(parents=True, exist_ok=False)

        shutil.copy2(
            rtk_service,
            backup_root / "rtkService.ts",
        )
        shutil.copy2(
            screen,
            backup_root / "RTKInjectionScreen.tsx",
        )

        # ------------------------------------------------------------------
        # rtkService.ts
        # ------------------------------------------------------------------
        service = rtk_service.read_text(encoding="utf-8")

        service = replace_once(
            service,
            "import { PX4_RTK } from '../config/px4Endpoints';",
            "import { PX4_RTK, PX4_TELEMETRY } from '../config/px4Endpoints';",
            "rtkService endpoint import",
        )

        types_anchor = """export interface RtkStatusResponse extends BackendRtkStatusResponse {
"""

        types_block = """export interface RtkEstimatorAccuracy {
  available: boolean;
  source: string | null;

  hrms_m: number | null;
  hrms_mm: number | null;

  vrms_m: number | null;
  vrms_mm: number | null;

  flags: number | null;
  healthy: boolean;
}

interface RtkTelemetryGpsSection {
  px4_hrms_source?: string | null;
  px4_hrms_m?: number | null;
  px4_hrms_mm?: number | null;
  px4_vrms_m?: number | null;
  px4_vrms_mm?: number | null;
  px4_estimator_flags?: number | null;
  px4_estimator_healthy?: boolean;
}

interface RtkTelemetryLatestResponse {
  gps?: RtkTelemetryGpsSection | null;
}

"""

        if "export interface RtkEstimatorAccuracy {" not in service:
            if types_anchor not in service:
                raise RuntimeError(
                    "rtkService types: RtkStatusResponse anchor not found"
                )

            service = service.replace(
                types_anchor,
                types_block + types_anchor,
                1,
            )
            print("rtkService estimator types: patched")
        else:
            print("rtkService estimator types: already patched")

        function_anchor = """export async function getRtkConfiguration(): Promise<GetRtkConfigurationResponse> {
"""

        function_block = """export async function getRtkEstimatorAccuracy(): Promise<RtkEstimatorAccuracy> {
  const telemetry =
    await apiGet<RtkTelemetryLatestResponse>(PX4_TELEMETRY.LATEST);

  const gps = telemetry.gps ?? null;

  const hrmsM =
    typeof gps?.px4_hrms_m === 'number' &&
    Number.isFinite(gps.px4_hrms_m)
      ? gps.px4_hrms_m
      : null;

  const hrmsMm =
    typeof gps?.px4_hrms_mm === 'number' &&
    Number.isFinite(gps.px4_hrms_mm)
      ? gps.px4_hrms_mm
      : null;

  const vrmsM =
    typeof gps?.px4_vrms_m === 'number' &&
    Number.isFinite(gps.px4_vrms_m)
      ? gps.px4_vrms_m
      : null;

  const vrmsMm =
    typeof gps?.px4_vrms_mm === 'number' &&
    Number.isFinite(gps.px4_vrms_mm)
      ? gps.px4_vrms_mm
      : null;

  const flags =
    typeof gps?.px4_estimator_flags === 'number' &&
    Number.isFinite(gps.px4_estimator_flags)
      ? gps.px4_estimator_flags
      : null;

  return {
    available:
      hrmsMm !== null &&
      vrmsMm !== null,

    source:
      typeof gps?.px4_hrms_source === 'string'
        ? gps.px4_hrms_source
        : null,

    hrms_m: hrmsM,
    hrms_mm: hrmsMm,

    vrms_m: vrmsM,
    vrms_mm: vrmsMm,

    flags,
    healthy:
      gps?.px4_estimator_healthy === true,
  };
}

"""

        if "export async function getRtkEstimatorAccuracy()" not in service:
            if function_anchor not in service:
                raise RuntimeError(
                    "rtkService function: getRtkConfiguration anchor not found"
                )

            service = service.replace(
                function_anchor,
                function_block + function_anchor,
                1,
            )
            print("rtkService estimator fetch: patched")
        else:
            print("rtkService estimator fetch: already patched")

        default_anchor = """  getRtkConfiguration,
  updateRtkConfiguration,
"""

        default_new = """  getRtkConfiguration,
  getRtkEstimatorAccuracy,
  updateRtkConfiguration,
"""

        service = replace_once(
            service,
            default_anchor,
            default_new,
            "rtkService default export",
        )

        rtk_service.write_text(
            service,
            encoding="utf-8",
        )

        # ------------------------------------------------------------------
        # RTKInjectionScreen.tsx
        # ------------------------------------------------------------------
        ui = screen.read_text(encoding="utf-8")

        import_old = """  getRtkConfiguration,
  getRtkStatus,
  reconnectRtk,
  type ActiveRtkConfiguration,
  type RtkStatusResponse,
"""

        import_new = """  getRtkConfiguration,
  getRtkEstimatorAccuracy,
  getRtkStatus,
  reconnectRtk,
  type ActiveRtkConfiguration,
  type RtkEstimatorAccuracy,
  type RtkStatusResponse,
"""

        ui = replace_once(
            ui,
            import_old,
            import_new,
            "RTK screen service import",
        )

        state_old = """  const [status, setStatus] = useState<RtkStatusResponse | null>(null);
  const [isLoading, setIsLoading] = useState(false);
"""

        state_new = """  const [status, setStatus] = useState<RtkStatusResponse | null>(null);
  const [estimatorAccuracy, setEstimatorAccuracy] =
    useState<RtkEstimatorAccuracy | null>(null);
  const [isLoading, setIsLoading] = useState(false);
"""

        ui = replace_once(
            ui,
            state_old,
            state_new,
            "RTK screen estimator state",
        )

        disconnected_old = """    if (!visible || !isConnected) {
      setStatus(null);
      return;
    }

    try {
      const nextStatus = await getRtkStatus();
      setStatus(nextStatus);
"""

        disconnected_new = """    if (!visible || !isConnected) {
      setStatus(null);
      setEstimatorAccuracy(null);
      return;
    }

    try {
      const [nextStatus, nextEstimatorAccuracy] = await Promise.all([
        getRtkStatus(),
        getRtkEstimatorAccuracy().catch((accuracyError) => {
          console.warn(
            "[RTKInjection] Failed to read PX4 estimator accuracy:",
            accuracyError,
          );
          return null;
        }),
      ]);

      setStatus(nextStatus);
      setEstimatorAccuracy(nextEstimatorAccuracy);
"""

        ui = replace_once(
            ui,
            disconnected_old,
            disconnected_new,
            "RTK screen status polling",
        )

        load_disconnected_old = """    if (!visible || !isConnected) {
      setStatus(null);
      return;
    }

    setIsLoading(true);
"""

        load_disconnected_new = """    if (!visible || !isConnected) {
      setStatus(null);
      setEstimatorAccuracy(null);
      return;
    }

    setIsLoading(true);
"""

        ui = replace_once(
            ui,
            load_disconnected_old,
            load_disconnected_new,
            "RTK screen load reset",
        )

        computed_anchor = """  const rtkDotColor = isHealthy
    ? colors.success
    : isErrorState || !isConnected
      ? colors.danger
      : colors.accent;

  return (
"""

        computed_new = """  const rtkDotColor = isHealthy
    ? colors.success
    : isErrorState || !isConnected
      ? colors.danger
      : colors.accent;

  const hrmsMm =
    estimatorAccuracy?.hrms_mm ?? null;

  const vrmsMm =
    estimatorAccuracy?.vrms_mm ?? null;

  const estimatorHealthy =
    estimatorAccuracy?.healthy === true;

  return (
"""

        ui = replace_once(
            ui,
            computed_anchor,
            computed_new,
            "RTK screen computed accuracy",
        )

        grid_old = """              <View style={styles.summaryItem}>
                <Text style={styles.summaryLabel}>Satellites</Text>
                <Text style={styles.summaryValue}>
                  {status ? status.satellites_visible : "—"}
                </Text>
              </View>
              <View style={styles.summaryItem}>
                <Text style={styles.summaryLabel}>Correction</Text>
"""

        grid_new = """              <View style={styles.summaryItem}>
                <Text style={styles.summaryLabel}>Satellites</Text>
                <Text style={styles.summaryValue}>
                  {status ? status.satellites_visible : "—"}
                </Text>
              </View>

              <View style={styles.summaryItem}>
                <Text style={styles.summaryLabel}>HRMS (PX4 EKF 1σ)</Text>
                <Text style={styles.summaryValue}>
                  {hrmsMm == null
                    ? "—"
                    : `${hrmsMm.toFixed(1)} mm`}
                </Text>
              </View>

              <View style={styles.summaryItem}>
                <Text style={styles.summaryLabel}>VRMS (PX4 EKF 1σ)</Text>
                <Text style={styles.summaryValue}>
                  {vrmsMm == null
                    ? "—"
                    : `${vrmsMm.toFixed(1)} mm`}
                </Text>
              </View>

              <View style={styles.summaryItem}>
                <Text style={styles.summaryLabel}>PX4 Estimator</Text>
                <Text style={styles.summaryValue}>
                  {estimatorAccuracy?.available
                    ? estimatorHealthy
                      ? "Healthy"
                      : "Check"
                    : "—"}
                </Text>
              </View>

              <View style={styles.summaryItem}>
                <Text style={styles.summaryLabel}>Correction</Text>
"""

        ui = replace_once(
            ui,
            grid_old,
            grid_new,
            "RTK screen HRMS/VRMS UI",
        )

        screen.write_text(
            ui,
            encoding="utf-8",
        )

        print()
        print("FRONTEND HRMS/VRMS PATCH: PASS")
        print("Repo:", root)
        print("Backup:", backup_root)
        print()
        print("Modified:")
        print("  src/services/rtkService.ts")
        print("  src/components/missionreport/RTKInjectionScreen.tsx")
        print()
        print("UI fields:")
        print("  HRMS (PX4 EKF 1σ)")
        print("  VRMS (PX4 EKF 1σ)")
        print("  PX4 Estimator")
        print()
        print("RTK Fixed/Float status remains controlled by /api/rtk/status.")
        print("Estimator health does NOT turn a 3D_FIX into RTK_FIXED.")
        return 0

    except Exception as exc:
        print(f"PATCH FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
