#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash

echo "===== SAFETY ====="
echo "WHEELS MUST BE LIFTED. EMERGENCY STOP READY."
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
timeout 5s ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: false}" -r 10 > /tmp/lr_estop.log 2>&1 || true

echo "===== ENABLE MISSION ====="
timeout 120s ros2 topic pub /mission_enable std_msgs/msg/Bool "{data: true}" -r 10 > /tmp/lr_enable.log 2>&1 &

sleep 2

run_test () {
    NAME="$1"
    AZ="$2"

    echo ""
    echo "===================================="
    echo "TEST: $NAME"
    echo "Command: linear.x=0.08 angular.z=$AZ"
    echo "===================================="

    pkill -9 -f "ros2 topic pub /cmd_vel" || true
    sleep 1

    timeout 15s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.08
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: $AZ
" -r 20 > /tmp/lr_cmd.log 2>&1 &

    sleep 8

    echo "===== CMD VEL INPUT ====="
    ros2 topic echo /cmd_vel --once

    echo "===== MAVROS SETPOINT OUTPUT ====="
    ros2 topic echo /mavros/setpoint_velocity/cmd_vel --once

    echo "===== PX4 STATE ====="
    ros2 topic echo /mavros/state --once

    echo ""
    echo "OBSERVE WHEELS NOW."
    read -p "Press ENTER after observing $NAME..."

    pkill -9 -f "ros2 topic pub /cmd_vel" || true

    timeout 2s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
" -r 20 > /tmp/lr_zero.log 2>&1 || true

    sleep 2
}

run_test "RIGHT TURN WITH 0.08 THROTTLE" "-0.30"
run_test "LEFT TURN WITH 0.08 THROTTLE" "0.30"

echo "===== STOP ROVER ====="
~/rover_ws/scripts/stop_rover.sh || true

echo "===== LEFT / RIGHT LINEAR 0.08 TEST COMPLETE ====="
