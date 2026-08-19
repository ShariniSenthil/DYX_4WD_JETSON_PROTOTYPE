#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash

DIR="$1"

if [ "$DIR" = "right" ]; then
    LX="0.08"
    AZ="-0.25"
    NAME="DIRECT PX4 PIVOT RIGHT"
elif [ "$DIR" = "left" ]; then
    LX="0.08"
    AZ="0.25"
    NAME="DIRECT PX4 PIVOT LEFT"
else
    echo "Usage:"
    echo "~/rover_ws/scripts/test_direct_px4_pivot.sh right"
    echo "~/rover_ws/scripts/test_direct_px4_pivot.sh left"
    exit 1
fi

echo "===== SAFETY ====="
echo "WHEELS LIFTED. EMERGENCY STOP READY."
echo ""

echo "===== CHECK ONLY MAVROS ====="
ros2 node list | grep -E "cmd_vel_bridge|rpp|corner|mavros$" || true

echo "===== SET MAVROS FRAME BODY_NED ====="
ros2 param set /mavros/setpoint_velocity mav_frame BODY_NED
ros2 param get /mavros/setpoint_velocity mav_frame

echo "===== START DIRECT MAVROS SETPOINT STREAM ====="
echo "$NAME"
echo "linear.x=$LX angular.z=$AZ"

timeout 35s ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped "
header:
  frame_id: base_link
twist:
  linear:
    x: $LX
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: $AZ
" -r 20 > /tmp/direct_pivot_cmd.log 2>&1 &

sleep 3

echo "===== SET OFFBOARD ====="
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}"

sleep 1

echo "===== ARM ====="
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

sleep 5

echo "===== MAVROS SETPOINT OUTPUT ====="
ros2 topic echo /mavros/setpoint_velocity/cmd_vel --once

echo "===== PX4 STATE ====="
ros2 topic echo /mavros/state --once

echo ""
echo "OBSERVE WHEELS NOW FOR 10 SECONDS..."
sleep 10

echo "===== STOP ====="
~/rover_ws/scripts/stop_rover.sh || true

echo "===== TEST COMPLETE ====="
