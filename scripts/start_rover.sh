#!/bin/bash

set -e

echo "========================================"
echo "Starting Rover Stack"
echo "========================================"

# -----------------------------
# ROS Setup
# -----------------------------
source /opt/ros/humble/setup.bash
source /home/flash/rover_ws/install/setup.bash

# -----------------------------
# Kill old ROS processes
# -----------------------------
pkill -f mavros || true
pkill -f mission_manager || true
pkill -f trajectory_generator || true
pkill -f rpp_controller || true
pkill -f cmd_vel_bridge || true

sleep 2

# -----------------------------
# Start MAVROS
# -----------------------------
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyACM0:115200
exec bash
"

sleep 8

# -----------------------------
# Wait until PX4 connects
# -----------------------------
until ros2 topic echo /mavros/state --once | grep "connected: true"; do
    echo "Waiting for PX4..."
    sleep 2
done

echo "PX4 Connected"

# -----------------------------
# Start Trajectory Generator
# -----------------------------
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash
source /home/flash/rover_ws/install/setup.bash
ros2 run trajectory_generator trajectory_generator_node
exec bash
"

sleep 2

# -----------------------------
# Start Mission Manager
# -----------------------------
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash
source /home/flash/rover_ws/install/setup.bash
ros2 run mission_manager mission_manager_node
exec bash
"

sleep 2

# -----------------------------
# Start RPP Controller
# -----------------------------
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash
source /home/flash/rover_ws/install/setup.bash
ros2 run rpp_controller rpp_controller_node
exec bash
"

sleep 2

# -----------------------------
# Start cmd_vel bridge
# -----------------------------
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash
source /home/flash/rover_ws/install/setup.bash
ros2 run offboard_controller cmd_vel_bridge
exec bash
"

sleep 5

# -----------------------------
# Switch PX4 to OFFBOARD
# -----------------------------
ros2 service call /mavros/set_mode \
mavros_msgs/srv/SetMode \
"{custom_mode: 'OFFBOARD'}"

sleep 3

echo "========================================"
echo "ROVER READY"
echo "========================================"

# ==========================================================
# IMPORTANT:
# Keep this script alive so systemd DOES NOT restart it.
# ==========================================================

while true
do
    sleep 60
done