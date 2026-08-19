JETSON 15 MM FINAL MARKING / SPRAY TOLERANCE
=============================================

FINAL GEOMETRY
--------------
Waypoint center = target coordinate.

Acceptance radius:
    0.015 m = 15 mm

Acceptance diameter:
    0.030 m = 30 mm

For P1, P2, P3, P4, ...:
    ACCEPT / STOP / HOLD / SPRAY / MARK
only when radial waypoint error <= 0.015 m.

IMPORTANT
---------
Do not configure 0.030 m as waypoint_tolerance_m.
0.030 m is the complete diameter, not the allowed radial error.

PATCHED AREAS
-------------
- RPP waypoint_tolerance_m -> 0.015
- mission_manager marking_tolerance_m -> 0.015
- accuracy_target_m -> 0.015 where present
- rover.launch.py runtime overrides -> 0.015
- backend displayed accuracy target -> 15 mm
- spray_controller positional tolerance -> 0.015 only if an independent
  positional tolerance parameter exists

UNCHANGED
---------
- EKF2_GPS_P_NOISE (PX4-side)
- PID gains
- accel/decel
- speed
- yaw/pivot tuning
- dummy/navigation-only point tolerances unless they explicitly share
  the real marking tolerance
- 50 mm diagnostic/test band unless you separately decide to change it

BUILD
-----
cd ~/rover_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select   rpp_controller mission_manager rover_bringup rover_backend spray_controller
source install/setup.bash

Restart rover launch.

VERIFY
------
bash verify_15mm_marking_tolerance.sh
