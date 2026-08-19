HRMS / VRMS FRONTEND INTEGRATION
================================

BACKEND DATA SOURCE
-------------------
PX4 MAVLink ESTIMATOR_STATUS message #230:
  pos_horiz_accuracy
  pos_vert_accuracy

These are 1-sigma estimator accuracies in metres.

Recommended UI labels:
  HRMS (PX4 EKF 1σ)
  VRMS (PX4 EKF 1σ)

Do not call gps.horizontal_accuracy_m / vertical_accuracy_m the same thing.
Your current NMEA GPS_RAW path leaves those raw receiver fields unavailable.

JETSON
------
Copy apply_hrms_vrms_backend.py to:
  ~/rover_ws/patch

Run:
  cd ~/rover_ws/patch
  python3 apply_hrms_vrms_backend.py

Build:
  cd ~/rover_ws
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --packages-select rover_backend
  source ~/rover_ws/install/setup.bash

Restart rover.launch.py.

Verify:
  bash ~/rover_ws/patch/verify_hrms_vrms_backend.sh

Expected API:
  telemetry.estimator.horizontal_accuracy_mm
  telemetry.estimator.vertical_accuracy_mm

FRONTEND
--------
Use FRONTEND_TELEMETRY_TYPE.ts to extend your telemetry type.
Use FRONTEND_RTK_SCREEN_SNIPPET.tsx to render the values.

IMPORTANT
---------
Do NOT use HRMS/VRMS as the spray trigger.

Your spray/marking completion stays:
  radial waypoint error <= 15 mm

HRMS/VRMS are estimator confidence/uncertainty values.
