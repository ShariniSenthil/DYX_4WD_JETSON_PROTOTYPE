#!/usr/bin/env bash
set -euo pipefail

echo "===== SOURCE CHECK ====="
grep -nE 'Mavlink|mavlink_source|mavlink_estimator_status|horizontal_accuracy_mm|vertical_accuracy_mm|estimator' \
  "$HOME/rover_ws/src/rover_backend/rover_backend/ros_bridge.py" \
  "$HOME/rover_ws/src/rover_backend/rover_backend/state.py" \
  "$HOME/rover_ws/src/rover_backend/rover_backend/system_routes.py" \
  | head -120

echo
echo "===== RAW PX4 ESTIMATOR ====="

timeout 8 python3 - <<'PY' || true
import struct
import rclpy
from mavros_msgs.msg import Mavlink
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class Check(Node):
    def __init__(self):
        super().__init__("hrms_vrms_raw_check")
        self.create_subscription(
            Mavlink,
            "/uas1/mavlink_source",
            self.cb,
            qos_profile_sensor_data,
        )

    def cb(self, msg):
        if int(msg.msgid) != 230:
            return
        words = [int(v) for v in msg.payload64]
        payload = struct.pack(
            "<" + ("Q" * len(words)),
            *words,
        )[:int(msg.len)].ljust(42, b"\x00")

        vals = struct.unpack("<Q8fH", payload[:42])
        h = vals[7]
        v = vals[8]
        flags = vals[9]

        print(f"HRMS-style PX4 EKF 1σ: {h*1000:.2f} mm")
        print(f"VRMS-style PX4 EKF 1σ: {v*1000:.2f} mm")
        print(f"flags: {flags}")
        rclpy.shutdown()

rclpy.init()
n = Check()
rclpy.spin(n)
PY

echo
echo "===== API ESTIMATOR ====="

if [[ -z "${TOKEN:-}" ]]; then
  echo "TOKEN not exported; skipping API check."
  exit 0
fi

curl -sS \
  -H "Authorization: Bearer $TOKEN" \
  http://127.0.0.1:5001/api/telemetry/latest \
  | python3 -c '
import json,sys
d=json.load(sys.stdin)
print(json.dumps(d.get("estimator"), indent=2))
'
