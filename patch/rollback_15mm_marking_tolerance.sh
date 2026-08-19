#!/usr/bin/env bash
set -euo pipefail

ROOT="$HOME/rover_ws"

B="$(
  find "$ROOT" -maxdepth 1 -type d \
    -name 'backup_before_15mm_marking_*' \
    -printf '%T@ %p\n' 2>/dev/null \
  | sort -nr \
  | head -1 \
  | cut -d' ' -f2-
)"

if [[ -z "$B" ]]; then
  echo "No backup_before_15mm_marking_* directory found."
  exit 1
fi

echo "Restoring: $B"

cp "$B/rpp__rpp_controller_node.py" \
  "$ROOT/src/rpp_controller/rpp_controller/rpp_controller_node.py"

cp "$B/mission__mission_manager_node.py" \
  "$ROOT/src/mission_manager/mission_manager/mission_manager_node.py"

cp "$B/launch__rover.launch.py" \
  "$ROOT/src/rover_bringup/launch/rover.launch.py"

cp "$B/backend_state__state.py" \
  "$ROOT/src/rover_backend/rover_backend/state.py"

cp "$B/backend_bridge__ros_bridge.py" \
  "$ROOT/src/rover_backend/rover_backend/ros_bridge.py"

if [[ -d "$B/spray_controller" ]]; then
  cp -a "$B/spray_controller/." \
    "$ROOT/src/spray_controller/"
fi

echo "Rollback complete."
