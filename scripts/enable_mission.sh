#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash

echo "Clearing emergency stop..."
timeout 10s ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: false}" -r 10

echo "Enabling mission..."
timeout 10s ros2 topic pub /mission_enable std_msgs/msg/Bool "{data: true}" -r 10

echo "Mission enable command sent."
