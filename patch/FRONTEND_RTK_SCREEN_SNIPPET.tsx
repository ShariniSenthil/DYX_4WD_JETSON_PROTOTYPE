// Example inside RTKInjectionScreen.tsx or your RTK status card.
//
// Keep these values separate from /rpp/accuracy.

const hrmsMm =
  typeof telemetry?.estimator?.horizontal_accuracy_mm === "number"
    ? telemetry.estimator.horizontal_accuracy_mm
    : null;

const vrmsMm =
  typeof telemetry?.estimator?.vertical_accuracy_mm === "number"
    ? telemetry.estimator.vertical_accuracy_mm
    : null;

const estimatorHealthy =
  telemetry?.estimator?.healthy === true;


// Render with your existing row/card component:
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
// Existing values should remain visible:
// RTK FIXED
// Satellites
// HDOP
//
// Marking acceptance remains:
// accuracy.radial_error_mm <= 15 mm
