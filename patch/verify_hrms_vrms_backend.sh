#!/usr/bin/env bash
set -euo pipefail

echo "===== RAW MAVLINK ESTIMATOR_STATUS #230 ====="

timeout 8 python3 - <<'PY' || true
import struct
import rclpy

from mavros_msgs.msg import Mavlink
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class Check(Node):
    def __init__(self):
        super().__init__("check_px4_estimator_accuracy")
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
        )[:int(msg.len)]

        payload = payload.ljust(42, b"\x00")

        values = struct.unpack(
            "<Q8fH",
            payload[:42],
        )

        h = values[7]
        v = values[8]
        flags = values[9]

        print(f"Horizontal EKF 1-sigma: {h:.5f} m = {h*1000:.2f} mm")
        print(f"Vertical EKF 1-sigma  : {v:.5f} m = {v*1000:.2f} mm")
        print(f"Estimator flags       : {flags}")

        rclpy.shutdown()


rclpy.init()
node = Check()
rclpy.spin(node)
PY

echo
echo "===== BACKEND API ====="

if [[ -z "${TOKEN:-}" ]]; then
  echo "TOKEN not exported."
  echo "Raw MAVLink verification above is still valid."
  exit 0
fi

curl -sS   -H "Authorization: Bearer $TOKEN"   http://127.0.0.1:5001/api/telemetry/latest   | python3 -c '
import json
import sys

d = json.load(sys.stdin)

print(json.dumps({
    "gps": d.get("gps"),
    "estimator": d.get("estimator"),
    "accuracy": d.get("accuracy"),
}, indent=2))
'
