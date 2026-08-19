#!/usr/bin/env bash
set -euo pipefail

RPP="$HOME/rover_ws/src/rpp_controller/rpp_controller/rpp_controller_node.py"

grep -n -B 15 -A 10 \
  'marking_stop_xtrack_limit_m must be below waypoint_tolerance_m' \
  "$RPP" || true

echo
grep -n 'marking_stop_xtrack_limit_m' "$RPP" | head -40
