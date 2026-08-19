#!/usr/bin/env python3

import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool


STATE_FILE = Path.home() / "rover_ws/.rover_gate_state"


def read_state():
    mission = False
    estop = True

    try:
        values = STATE_FILE.read_text().strip().lower().split()

        if len(values) == 2:
            mission = values[0] == "true"
            estop = values[1] == "true"
    except Exception:
        pass

    return mission, estop


def main():
    rclpy.init()
    node = Node("rover_gate_keeper")

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    mission_pub = node.create_publisher(
        Bool,
        "/mission_enable",
        qos,
    )

    estop_pub = node.create_publisher(
        Bool,
        "/emergency_stop",
        qos,
    )

    previous_state = None

    try:
        while rclpy.ok():
            mission, estop = read_state()

            mission_msg = Bool()
            mission_msg.data = mission

            estop_msg = Bool()
            estop_msg.data = estop

            mission_pub.publish(mission_msg)
            estop_pub.publish(estop_msg)

            current_state = (mission, estop)

            if current_state != previous_state:
                node.get_logger().info(
                    f"mission={mission} estop={estop}"
                )
                previous_state = current_state

            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.04)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
