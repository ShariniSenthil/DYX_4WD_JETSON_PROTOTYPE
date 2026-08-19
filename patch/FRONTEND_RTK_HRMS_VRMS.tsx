// Put this inside the RTK screen/card where `telemetry` is available.

const estimator = telemetry?.estimator;

const hrmsMm =
  typeof estimator?.horizontal_accuracy_mm === "number"
    ? estimator.horizontal_accuracy_mm
    : null;

const vrmsMm =
  typeof estimator?.vertical_accuracy_mm === "number"
    ? estimator.vertical_accuracy_mm
    : null;

const estimatorHealthy =
  estimator?.healthy === true;


// Render using your existing StatusRow/card:
//
// <StatusRow
//   label="HRMS (PX4 EKF 1σ)"
//   value={hrmsMm == null ? "--" : `${hrmsMm.toFixed(1)} mm`}
// />
//
// <StatusRow
//   label="VRMS (PX4 EKF 1σ)"
//   value={vrmsMm == null ? "--" : `${vrmsMm.toFixed(1)} mm`}
// />
//
// <StatusRow
//   label="Estimator"
//   value={estimatorHealthy ? "HEALTHY" : "CHECK"}
// />
//
// Keep RTK FIX, satellites and HDOP.
//
// IMPORTANT:
// HRMS/VRMS display is estimator uncertainty.
// Spray/mark completion still uses radial waypoint error <= 15 mm.
