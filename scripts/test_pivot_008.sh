#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash

echo "===== SAFETY ====="
echo "WHEELS MUST BE LIFTED. EMERGENCY STOP READY."

echo "===== CHECK REQUIRED NODES ====="
ros2 node list | grep -x "/cmd_vel_bridge" || { echo "ERROR: /cmd_vel_bridge not running"; exit 1; }
ros2 node list | grep -x "/mavros" || { echo "ERROR: /mavros not running"; exit 1; }

echo "===== KILL CORNER CONTROLLER ONLY ====="
pkill -9 -f corner_controller_node || true
pkill -9 -f corner_controller || true

echo "===== KILL OLD TOPIC PUBLISHERS ====="
pkill -9 -f "ros2 topic pub /cmd_vel" || true
pkill -9 -f "ros2 topic pub /mission_enable" || true
pkill -9 -f "ros2 topic pub /emergency_stop" || true

sleep 2

echo "===== CHECK NO CORNER CONTROLLER ====="
ros2 node list | grep -x "/corner_controller" && echo "WARNING: corner_controller still alive" || echo "OK: no corner_controller"

echo "===== START PIVOT INPUT COMMAND ====="
timeout 45s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.20
" -r 20 > /tmp/pivot_cmd.log 2>&1 &

sleep 2

echo "===== CLEAR EMERGENCY ====="
timeout 4s ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: false}" -r 10 > /tmp/pivot_estop.log 2>&1 || true

echo "===== ENABLE MISSION ====="
timeout 30s ros2 topic pub /mission_enable std_msgs/msg/Bool "{data: true}" -r 10 > /tmp/pivot_enable.log 2>&1 &

echo "===== WAIT FOR OFFBOARD + ARM ====="
sleep 12

echo "===== PX4 STATE ====="
ros2 topic echo /mavros/state --once

echo "===== CMD VEL INPUT ====="
ros2 topic echo /cmd_vel --once

echo "===== MAVROS SETPOINT OUTPUT ====="
timeout 6s ros2 topic echo /mavros/setpoint_velocity/cmd_vel --once || echo "NO MAVROS SETPOINT OUTPUT"

echo "===== STOP NOW ====="
~/rover_ws/scripts/stop_rover.sh || true

echo "===== TEST COMPLETE ====="
