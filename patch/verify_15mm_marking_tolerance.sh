#!/usr/bin/env bash
set -euo pipefail

echo "===== LIVE 15 MM TARGET CHECK ====="

ros2 param get /rpp_controller waypoint_tolerance_m || true
ros2 param get /mission_manager marking_tolerance_m || true
ros2 param get /mission_manager accuracy_target_m || true

echo
echo "Expected real waypoint/spray acceptance:"
echo "  radius   = 0.015 m = 15 mm"
echo "  diameter = 0.030 m = 30 mm"
echo
echo "RPP radial comparison must use <= 0.015 m."
