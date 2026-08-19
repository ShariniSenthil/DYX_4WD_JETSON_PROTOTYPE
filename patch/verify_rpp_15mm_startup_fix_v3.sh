#!/usr/bin/env bash
set -euo pipefail

RPP="$HOME/rover_ws/src/rpp_controller/rpp_controller/rpp_controller_node.py"

echo "===== SOURCE VALIDATOR ====="
grep -n -B 2 -A 2 \
  'marking_stop_xtrack_limit_m must be positive and not exceed waypoint_tolerance_m' \
  "$RPP" || true

echo
echo "===== LIVE NODE ====="
ros2 node list | grep -E '^/rpp_controller$' || true

echo
echo "===== LIVE PARAMETERS ====="
ros2 param get /rpp_controller waypoint_tolerance_m || true
ros2 param get /rpp_controller marking_stop_xtrack_limit_m || true

echo
echo "Expected:"
echo "  waypoint_tolerance_m        = 0.015"
echo "  marking_stop_xtrack_limit_m = 0.015"
