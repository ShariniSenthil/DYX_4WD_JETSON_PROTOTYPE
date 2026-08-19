// Add this to your rover telemetry types.

export type Px4EstimatorAccuracy = {
  available: boolean;
  source: string;

  horizontal_accuracy_m: number | null;
  horizontal_accuracy_mm: number | null;
  vertical_accuracy_m: number | null;
  vertical_accuracy_mm: number | null;

  vel_ratio: number | null;
  pos_horiz_ratio: number | null;
  pos_vert_ratio: number | null;
  mag_ratio: number | null;
  hagl_ratio: number | null;
  tas_ratio: number | null;

  flags: number;
  absolute_horizontal_valid: boolean;
  absolute_vertical_valid: boolean;
  gps_glitch: boolean;
  accel_error: boolean;
  healthy: boolean;
  updated_at?: string | null;
};

// In your main telemetry interface/type add:
//
// estimator?: Px4EstimatorAccuracy;

export function getPx4HrmsMm(
  telemetry: { estimator?: Px4EstimatorAccuracy } | null | undefined,
): number | null {
  const value = telemetry?.estimator?.horizontal_accuracy_mm;

  return typeof value === "number" && Number.isFinite(value)
    ? value
    : null;
}

export function getPx4VrmsMm(
  telemetry: { estimator?: Px4EstimatorAccuracy } | null | undefined,
): number | null {
  const value = telemetry?.estimator?.vertical_accuracy_mm;

  return typeof value === "number" && Number.isFinite(value)
    ? value
    : null;
}
