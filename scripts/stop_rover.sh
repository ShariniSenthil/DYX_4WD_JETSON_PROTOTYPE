#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash

echo "Disabling mission..."
timeout 3s ros2 topic pub /mission_enable std_msgs/msg/Bool "{data: false}" -r 5

echo "Activating emergency stop..."
timeout 3s ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}" -r 5

echo "Sending zero velocity..."
timeout 3s ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped "
header:
  frame_id: base_link
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
" -r 20

echo "Disarming PX4..."
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}" || true

echo "Requesting MANUAL mode..."
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'MANUAL'}" || true

echo "Stop command completed."
