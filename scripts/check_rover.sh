#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash

echo "========== MAVROS FRAME =========="
ros2 param get /mavros/setpoint_velocity mav_frame

echo "========== CMD VEL =========="
ros2 topic echo /cmd_vel --once

echo "========== BRIDGE OUTPUT =========="
ros2 topic echo /mavros/setpoint_velocity/cmd_vel --once

echo "========== PX4 STATE =========="
ros2 topic echo /mavros/state --once
