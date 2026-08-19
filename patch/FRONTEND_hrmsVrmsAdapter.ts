export type HrmsVrmsView = {
  hrmsMm: number | null;
  vrmsMm: number | null;
  estimatorHealthy: boolean;
};

export function selectHrmsVrms(
  telemetry: any,
): HrmsVrmsView {
  const estimator = telemetry?.estimator;

  const h = estimator?.horizontal_accuracy_mm;
  const v = estimator?.vertical_accuracy_mm;

  return {
    hrmsMm:
      typeof h === "number" && Number.isFinite(h)
        ? h
        : null,

    vrmsMm:
      typeof v === "number" && Number.isFinite(v)
        ? v
        : null,

    estimatorHealthy:
      estimator?.healthy === true,
  };
}

export function formatMm(
  value: number | null,
): string {
  return value == null
    ? "--"
    : `${value.toFixed(1)} mm`;
}
