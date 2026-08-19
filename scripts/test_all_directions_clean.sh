#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash

echo "===== SAFETY ====="
echo "WHEELS MUST BE LIFTED. EMERGENCY STOP READY."
echo "REMOTE ON. STICKS NEUTRAL."
echo ""

echo "===== KILL OLD CMD_VEL PUBLISHERS ====="
pkill -9 -f "ros2 topic pub /cmd_vel" || true
pkill -9 -f rpp_controller || true
pkill -9 -f corner_controller || true
pkill -9 -f corner_controller_node || true
sleep 2

echo "===== CHECK REQUIRED NODES ====="
ros2 node list | grep -E "cmd_vel_bridge|rpp|corner|mavros$" || true

echo ""
echo "Expected:"
echo "/mavros"
echo "/cmd_vel_bridge"
echo ""

if ! ros2 node list | grep -q "/mavros"; then
    echo "ERROR: MAVROS is not running."
    exit 1
fi

if ! ros2 node list | grep -q "/cmd_vel_bridge"; then
    echo "ERROR: cmd_vel_bridge is not running."
    exit 1
fi

echo "===== SET MAVROS BODY_NED ====="
ros2 param set /mavros/setpoint_velocity mav_frame BODY_NED
ros2 param get /mavros/setpoint_velocity mav_frame

echo "===== CLEAR EMERGENCY ====="
timeout 4s ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: false}" -r 10 > /tmp/estop.log 2>&1 || true

echo "===== ENABLE MISSION ====="
timeout 300s ros2 topic pub /mission_enable std_msgs/msg/Bool "{data: true}" -r 10 > /tmp/enable.log 2>&1 &

sleep 2

run_test () {
    NAME="$1"
    LX="$2"
    AZ="$3"

    echo ""
    echo "===================================="
    echo "TEST: $NAME"
    echo "linear.x=$LX angular.z=$AZ"
    echo "===================================="

    pkill -9 -f "ros2 topic pub /cmd_vel" || true
    sleep 1

    timeout 20s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: $LX
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: $AZ
" -r 20 > /tmp/test_cmd.log 2>&1 &

    sleep 5

    echo "===== CMD VEL INPUT ====="
    timeout 3s ros2 topic echo /cmd_vel --once || true

    echo "===== MAVROS SETPOINT OUTPUT ====="
    timeout 3s ros2 topic echo /mavros/setpoint_velocity/cmd_vel --once || true

    echo "===== PX4 STATE ====="
    timeout 3s ros2 topic echo /mavros/state --once || true

    echo ""
    echo "OBSERVE WHEEL DIRECTION NOW."
    read -p "Press ENTER for next test..."

    pkill -9 -f "ros2 topic pub /cmd_vel" || true

    timeout 3s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
" -r 20 > /tmp/zero.log 2>&1 || true

    sleep 2
}

run_test "FORWARD" "0.08" "0.0"
run_test "REVERSE" "-0.08" "0.0"
run_test "RIGHT TURN MOVING" "0.08" "-0.20"
run_test "LEFT TURN MOVING" "0.08" "0.20"
run_test "PIVOT RIGHT THROUGH BRIDGE" "0.0" "-0.20"
run_test "PIVOT LEFT THROUGH BRIDGE" "0.0" "0.20"

echo "===== STOP ROVER ====="
~/rover_ws/scripts/stop_rover.sh || true

echo "===== ALL DIRECTION TEST COMPLETE ====="
