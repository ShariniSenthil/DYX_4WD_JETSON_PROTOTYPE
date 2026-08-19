#!/bin/bash

source /opt/ros/humble/setup.bash

pkill -9 -f mavros_node || true
pkill -9 -f mavros_router || true
pkill -9 -f cmd_vel_bridge || true
pkill -9 -f rpp_controller_node || true
pkill -9 -f mission_manager_node || true
pkill -9 -f trajectory_generator_node || true
pkill -9 -f "ros2 launch" || true
pkill -9 -f "ros2 topic pub" || true

ros2 daemon stop
ros2 daemon start

echo "All rover ROS nodes killed."
