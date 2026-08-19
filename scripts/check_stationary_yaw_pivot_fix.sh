#!/usr/bin/env bash
set -euo pipefail

WS="${HOME}/rover_ws"
RPP="$WS/src/rpp_controller/rpp_controller/rpp_controller_node.py"
BRIDGE="$WS/src/jetson_4wd_control/jetson_4wd_control/cmd_vel_bridge.py"
LAUNCH="$WS/src/rover_bringup/launch/rover.launch.py"

echo "===== SOURCE CHECK ====="
python3 -m py_compile "$RPP" "$BRIDGE" "$LAUNCH"

grep -q 'stationary_yaw_setpoint_enu' "$RPP"
grep -q 'stationary_yaw_setpoint_enu' "$BRIDGE"
grep -q '/mavros/setpoint_raw/attitude' "$BRIDGE"
grep -q 'AttitudeTarget' "$BRIDGE"

if grep -q 'TYPE_MASK_VELOCITY_AND_YAW_RATE' "$BRIDGE"; then
    echo "FAIL: old unsupported local yaw-rate pivot mask still exists"
    exit 1
fi
if grep -q '0.40MPS FIXED VECTOR' "$RPP"; then
    echo "FAIL: old translating fake-pivot vector still exists"
    exit 1
fi

echo "PASS: source uses PX4 rover absolute-yaw attitude pivot"

echo "===== LIVE CHECK ====="
if ros2 node list 2>/dev/null | grep -qx '/cmd_vel_bridge'; then
    TYPE="$(ros2 topic type /mavros/setpoint_raw/attitude 2>/dev/null || true)"
    if [[ "$TYPE" != "mavros_msgs/msg/AttitudeTarget" ]]; then
        echo "FAIL: attitude setpoint topic missing or wrong type: ${TYPE:-none}"
        exit 1
    fi
    ros2 param get /cmd_vel_bridge maximum_speed_mps
    echo "PASS: live attitude setpoint topic exists"
else
    echo "INFO: rover stack is not running; live checks skipped"
fi

echo "ABSOLUTE YAW PIVOT FIX VERIFIED"