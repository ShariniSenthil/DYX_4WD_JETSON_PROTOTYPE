#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash

DIR="$1"

if [ "$DIR" = "right" ]; then
    LX="0.05"
    AZ="-0.30"
    NAME="RIGHT TURN"
elif [ "$DIR" = "left" ]; then
    LX="0.05"
    AZ="0.30"
    NAME="LEFT TURN"
else
    echo "Usage:"
    echo "~/rover_ws/scripts/test_one_turn.sh right"
    echo "~/rover_ws/scripts/test_one_turn.sh left"
    exit 1
fi

echo "===== SAFETY ====="
echo "WHEELS LIFTED. EMERGENCY STOP READY."
echo ""

echo "===== CLEAN OLD PUBLISHERS ====="
pkill -9 -f "ros2 topic pub /cmd_vel" || true
pkill -9 -f rpp_controller || true
pkill -9 -f corner_controller || true
pkill -9 -f corner_controller_node || true
sleep 2

echo "===== CHECK NODES ====="
ros2 node list | grep -E "cmd_vel_bridge|rpp|corner|mavros$" || true

echo "===== SET BODY_NED ====="
ros2 param set /mavros/setpoint_velocity mav_frame BODY_NED
ros2 param get /mavros/setpoint_velocity mav_frame

echo "===== CLEAR EMERGENCY ====="
timeout 5s ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: false}" -r 10 > /tmp/turn_estop.log 2>&1 || true

echo "===== ENABLE MISSION ====="
timeout 60s ros2 topic pub /mission_enable std_msgs/msg/Bool "{data: true}" -r 10 > /tmp/turn_enable.log 2>&1 &

sleep 2

echo "===== START $NAME COMMAND ====="
echo "Command: linear.x=$LX angular.z=$AZ"

timeout 25s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: $LX
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: $AZ
" -r 20 > /tmp/turn_cmd.log 2>&1 &

echo "Waiting for OFFBOARD + ARM..."
sleep 12

echo "===== CMD VEL INPUT ====="
ros2 topic echo /cmd_vel --once

echo "===== MAVROS SETPOINT OUTPUT ====="
ros2 topic echo /mavros/setpoint_velocity/cmd_vel --once

echo "===== PX4 STATE ====="
ros2 topic echo /mavros/state --once

echo ""
echo "OBSERVE WHEELS NOW FOR 8 SECONDS..."
sleep 8

echo "===== STOP ROVER ====="
~/rover_ws/scripts/stop_rover.sh || true

echo "===== TEST COMPLETE ====="
