#!/usr/bin/env bash
set -euo pipefail

echo "===== ROS BACKEND SUBSCRIPTION ====="
ros2 node info /rover_backend \
  | grep -A2 -B2 '/uas1/mavlink_source' || true

echo
echo "===== SOURCE GPS MIRROR ====="
grep -nE 'px4_hrms|px4_vrms|px4_estimator' \
  "$HOME/rover_ws/src/rover_backend/rover_backend/ros_bridge.py" || true

echo
echo "===== API GPS ====="

if [[ -z "${TOKEN:-}" ]]; then
  echo "TOKEN is not exported."
  echo "Export TOKEN and run this script again."
  exit 0
fi

curl -sS \
  -H "Authorization: Bearer $TOKEN" \
  http://127.0.0.1:5001/api/telemetry/latest \
  | python3 -c '
import json
import sys

d = json.load(sys.stdin)
gps = d.get("gps") or {}

print(json.dumps({
    "fix_type": gps.get("fix_type"),
    "fix_name": gps.get("fix_name"),
    "rtk_fixed": gps.get("rtk_fixed"),
    "satellites_visible": gps.get("satellites_visible"),
    "hdop": gps.get("hdop"),
    "raw_gnss_horizontal_accuracy_m": gps.get("horizontal_accuracy_m"),
    "raw_gnss_vertical_accuracy_m": gps.get("vertical_accuracy_m"),
    "px4_hrms_m": gps.get("px4_hrms_m"),
    "px4_hrms_mm": gps.get("px4_hrms_mm"),
    "px4_vrms_m": gps.get("px4_vrms_m"),
    "px4_vrms_mm": gps.get("px4_vrms_mm"),
    "px4_estimator_flags": gps.get("px4_estimator_flags"),
    "px4_estimator_healthy": gps.get("px4_estimator_healthy"),
}, indent=2))
'
