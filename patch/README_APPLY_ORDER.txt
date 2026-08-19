HRMS / VRMS BACKEND + FRONTEND PATCH
=====================================

UPLOAD / EXTRACT
----------------
Extract all files into:

  /home/flash/rover_ws/patch

APPLY BACKEND
-------------
cd ~/rover_ws/patch

python3 01_replace_ros_bridge_hrms_vrms.py
python3 02_replace_state_hrms_vrms.py
python3 03_replace_system_routes_hrms_vrms.py

BUILD
-----
Stop rover.launch.py first.

cd ~/rover_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select rover_backend

source ~/rover_ws/install/setup.bash

RESTART
-------
ros2 launch rover_bringup rover.launch.py

VERIFY
------
Open another terminal:

source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash

bash ~/rover_ws/patch/04_verify_hrms_vrms.sh

Expected raw PX4 data should show live values similar to:

  HRMS-style PX4 EKF 1σ: 13.xx mm
  VRMS-style PX4 EKF 1σ: 11.xx mm

Then the API estimator section should no longer be null.

API CHECK
---------
curl -sS \
-H "Authorization: Bearer $TOKEN" \
http://127.0.0.1:5001/api/telemetry/latest \
| python3 -c '
import json,sys
d=json.load(sys.stdin)
print(json.dumps(d.get("estimator"), indent=2))
'

FRONTEND FILES INCLUDED
-----------------------
FRONTEND_Px4EstimatorAccuracy.ts
FRONTEND_hrmsVrmsAdapter.ts
FRONTEND_RTK_HRMS_VRMS.tsx

Recommended UI labels:

  HRMS (PX4 EKF 1σ)
  VRMS (PX4 EKF 1σ)

IMPORTANT
---------
HRMS/VRMS are PX4 EKF position uncertainty estimates.

Do not use them as the spray trigger.

Final spray/mark completion remains based on the actual waypoint
radial error being <= 0.015 m (15 mm radius).
