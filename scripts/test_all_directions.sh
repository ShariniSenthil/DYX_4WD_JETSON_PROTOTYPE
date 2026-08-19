#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash

echo "===== SAFETY ====="
echo "WHEELS LIFTED. EMERGENCY STOP READY."
echo ""

BRIDGE_COUNT=$(ros2 node list | grep -x "/cmd_vel_bridge" | wc -l)
MAVROS_COUNT=$(ros2 node list | grep -x "/mavros" | wc -l)
CORNER_COUNT=$(ros2 node list | grep -x "/corner_controller" | wc -l)

echo "Bridge count: $BRIDGE_COUNT"
echo "MAVROS count : $MAVROS_COUNT"
echo "Corner count : $CORNER_COUNT"

if [ "$BRIDGE_COUNT" -ne 1 ]; then
    echo "ERROR: Need exactly one /cmd_vel_bridge"
    exit 1
fi

if [ "$MAVROS_COUNT" -ne 1 ]; then
    echo "ERROR: Need exactly one /mavros"
    exit 1
fi

if [ "$CORNER_COUNT" -ne 0 ]; then
    echo "ERROR: /corner_controller still running"
    exit 1
fi

pkill -9 -f "ros2 topic pub /cmd_vel" || true
pkill -9 -f "ros2 topic pub /mission_enable" || true
pkill -9 -f "ros2 topic pub /emergency_stop" || true

echo "===== CLEAR EMERGENCY ====="
timeout 4s ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: false}" -r 10 > /tmp/all_dir_estop.log 2>&1 || true

echo "===== ENABLE MISSION ====="
timeout 150s ros2 topic pub /mission_enable std_msgs/msg/Bool "{data: true}" -r 10 > /tmp/all_dir_enable.log 2>&1 &

sleep 2

run_test () {
    NAME="$1"
    LX="$2"
    AZ="$3"

    echo ""
    echo "======================================"
    echo "TEST: $NAME"
    echo "INPUT SHOULD BE: linear.x=$LX angular.z=$AZ"
    echo "======================================"

    pkill -9 -f "ros2 topic pub /cmd_vel" || true
    sleep 1

    timeout 10s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: $LX
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: $AZ
" -r 20 > /tmp/all_dir_cmd.log 2>&1 &

    sleep 3

    echo "PX4 STATE:"
    ros2 topic echo /mavros/state --once

    echo "CMD VEL INPUT:"
    ros2 topic echo /cmd_vel --once

    echo "MAVROS SETPOINT OUTPUT:"
    timeout 5s ros2 topic echo /mavros/setpoint_velocity/cmd_vel --once || echo "NO SETPOINT OUTPUT"

    echo "STOP BETWEEN TESTS..."
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
" -r 20 > /tmp/all_dir_zero.log 2>&1 || true

    sleep 2
}

run_test "FORWARD" "0.08" "0.0"
run_test "REVERSE" "-0.08" "0.0"
run_test "RIGHT TURN MOVING" "0.08" "-0.20"
run_test "LEFT TURN MOVING" "0.08" "0.20"
run_test "PIVOT RIGHT MIN THROTTLE" "0.0" "-0.20"
run_test "PIVOT LEFT MIN THROTTLE" "0.0" "0.20"

echo ""
echo "===== STOP ROVER ====="
~/rover_ws/scripts/stop_rover.sh || true

echo "===== ALL DIRECTION TEST COMPLETE ====="
