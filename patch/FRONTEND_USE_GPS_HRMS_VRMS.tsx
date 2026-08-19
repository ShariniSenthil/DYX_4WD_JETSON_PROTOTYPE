// Final frontend data path.
//
// Use the already-existing telemetry.gps object.
// Raw GNSS accuracy remains separate and may be null.

const hrmsMm =
  typeof telemetry?.gps?.px4_hrms_mm === "number"
    ? telemetry.gps.px4_hrms_mm
    : null;

const vrmsMm =
  typeof telemetry?.gps?.px4_vrms_mm === "number"
    ? telemetry.gps.px4_vrms_mm
    : null;

const estimatorHealthy =
  telemetry?.gps?.px4_estimator_healthy === true;


// Add these optional fields to your existing GPS telemetry type:
//
// px4_hrms_source?: string | null;
// px4_hrms_m?: number | null;
// px4_hrms_mm?: number | null;
// px4_vrms_m?: number | null;
// px4_vrms_mm?: number | null;
// px4_estimator_flags?: number | null;
// px4_estimator_healthy?: boolean;


// Example:
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
// HRMS/VRMS are estimator uncertainty.
// Mark/spray completion remains radial waypoint error <= 15 mm.
