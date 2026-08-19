#!/usr/bin/env bash
set -euo pipefail

echo "===== SOURCE CHECK ====="
grep -nE 'waypoint_tolerance_m|marking_stop_xtrack_limit_m|must not exceed' \
  "$HOME/rover_ws/src/rpp_controller/rpp_controller/rpp_controller_node.py" \
  | head -40

echo
echo "===== LIVE CHECK ====="
ros2 node list | grep -E '^/rpp_controller$' || true
ros2 param get /rpp_controller waypoint_tolerance_m || true
ros2 param get /rpp_controller marking_stop_xtrack_limit_m || true

echo
echo "Expected:"
echo "  waypoint_tolerance_m        = 0.015"
echo "  marking_stop_xtrack_limit_m = 0.015"
