#!/usr/bin/env python3

import threading
import time

import rclpy
from mavros_msgs.msg import ManualControl
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class ManualYawPublisher(Node):
    def __init__(self) -> None:
        super().__init__("acro_yaw_manual_control_test")

        self.publisher = self.create_publisher(
            ManualControl,
            "/mavros/manual_control/send",
            10,
        )

        self.r_command = 0.0

        # Publish at 20 Hz.
        self.timer = self.create_timer(0.05, self.publish_command)

    def set_yaw(self, command: float) -> None:
        self.r_command = float(command)

    def publish_command(self) -> None:
        message = ManualControl()

        message.header.stamp = self.get_clock().now().to_msg()

        # Keep forward, lateral and throttle commands centred.
        message.x = 0.0
        message.y = 0.0
        message.z = 0.0

        # MAVROS ROS 2 /send expects MAVLink-scale values:
        # -1000 to +1000.
        message.r = self.r_command

        message.buttons = 0
        message.buttons2 = 0
        message.enabled_extensions = 0

        message.s = 0.0
        message.t = 0.0
        message.aux1 = 0.0
        message.aux2 = 0.0
        message.aux3 = 0.0
        message.aux4 = 0.0
        message.aux5 = 0.0
        message.aux6 = 0.0

        self.publisher.publish(message)


def perform_step(
    node: ManualYawPublisher,
    label: str,
    yaw_command: float,
    duration: float,
) -> None:
    node.set_yaw(yaw_command)

    print()
    print("=" * 55)
    print(label)
    print(f"Command: {yaw_command:+.0f} / 1000")
    print("=" * 55)

    end_time = time.monotonic() + duration

    while time.monotonic() < end_time:
        remaining = end_time - time.monotonic()
        print(f"\rRemaining: {remaining:4.1f} seconds", end="", flush=True)
        time.sleep(0.1)

    print()


def main() -> None:
    rclpy.init()

    node = ManualYawPublisher()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    executor_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )
    executor_thread.start()

    try:
        node.set_yaw(0.0)

        print()
        print("PX4 ACRO FEED-FORWARD YAW TEST")
        print("--------------------------------")
        print("A centred MANUAL_CONTROL stream is now active.")
        print()
        print("Using QGroundControl:")
        print("  1. Select ACRO mode.")
        print("  2. Arm PX4.")
        print("  3. Keep the physical E-stop ready.")
        print()

        input("After PX4 is armed, press ENTER to start...")

        sequence = [
            ("CENTRE", 0.0, 4.0),

            ("RIGHT YAW 25%", 250.0, 5.0),
            ("CENTRE", 0.0, 4.0),
            ("LEFT YAW 25%", -250.0, 5.0),
            ("CENTRE", 0.0, 4.0),

            ("RIGHT YAW 50%", 500.0, 5.0),
            ("CENTRE", 0.0, 4.0),
            ("LEFT YAW 50%", -500.0, 5.0),
            ("CENTRE", 0.0, 4.0),

            ("RIGHT YAW 75%", 750.0, 5.0),
            ("CENTRE", 0.0, 4.0),
            ("LEFT YAW 75%", -750.0, 5.0),
            ("CENTRE", 0.0, 4.0),
        ]

        for label, command, duration in sequence:
            perform_step(node, label, command, duration)

        node.set_yaw(0.0)

        print()
        print("TEST SEQUENCE FINISHED")
        print("Centred commands will continue to be transmitted.")
        print()
        input("DISARM PX4 in QGroundControl, then press ENTER...")

    except KeyboardInterrupt:
        print("\nTest interrupted. Sending centred command.")

    finally:
        # Continue sending centre briefly before shutting down.
        node.set_yaw(0.0)
        time.sleep(2.0)

        executor.shutdown()
        executor_thread.join(timeout=2.0)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
