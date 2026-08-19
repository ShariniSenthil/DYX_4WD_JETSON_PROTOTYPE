#!/usr/bin/env python3

"""Print the C->P1->P2 test marking errors in millimetres."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class MarkingErrorMonitor(Node):
    def __init__(self) -> None:
        super().__init__("marking_error_mm_monitor")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            String,
            "/test/marking_error_mm",
            self._callback,
            qos,
        )
        print(
            "POINT | XTRACK(mm) | ALONG(mm) | RADIAL(mm) | "
            "RADIUS<=30 | SPEED(mm/s) | HOLD_VALID | HOLD(s) | L1-INFO(mm)",
            flush=True,
        )

    def _callback(self, message: String) -> None:
        try:
            data = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            print(message.data, flush=True)
            return

        print(
            f"{str(data.get('point', '-')):>5} | "
            f"{float(data.get('xtrack_mm', 0.0)):+10.1f} | "
            f"{float(data.get('along_error_mm', 0.0)):+9.1f} | "
            f"{float(data.get('radial_mm', 0.0)):10.1f} | "
            f"{'YES' if float(data.get('radial_mm', 0.0)) <= float(data.get('radius_limit_mm', 30.0)) else 'NO ':>10} | "
            f"{float(data.get('speed_mmps', 0.0)):11.1f} | "
            f"{'YES' if data.get('valid_for_hold') else 'NO ':>10} | "
            f"{float(data.get('manager_hold_sec', 0.0)):6.2f} | "
            f"{float(data.get('combined_mm', 0.0)):11.1f}",
            flush=True,
        )


def main() -> None:
    rclpy.init()
    node = MarkingErrorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()