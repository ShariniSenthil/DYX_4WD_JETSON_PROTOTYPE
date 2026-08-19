#!/usr/bin/env python3

import math
import statistics
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class FullThrottleSpeedMeter(Node):
    START_DELAY_SEC = 5.0
    SAMPLE_DURATION_SEC = 8.0
    MIN_VALID_SPEED = 0.10

    def __init__(self) -> None:
        super().__init__("full_throttle_speed_meter")

        self.start_time = time.monotonic()
        self.samples: list[float] = []
        self.finished = False
        self.last_countdown = None

        self.create_subscription(
            TwistStamped,
            "/mavros/local_position/velocity_local",
            self.velocity_callback,
            20,
        )

        self.create_timer(0.1, self.timer_callback)

        print()
        print("FULL-THROTTLE SPEED TEST")
        print("------------------------")
        print("1. Select PX4 MANUAL mode.")
        print("2. Point the rover along a clear straight path.")
        print("3. Arm PX4.")
        print("4. Start applying full forward throttle when prompted.")
        print()
        print("Starting in 5 seconds...")

    def velocity_callback(self, message: TwistStamped) -> None:
        elapsed = time.monotonic() - self.start_time

        if elapsed < self.START_DELAY_SEC:
            return

        if elapsed > self.START_DELAY_SEC + self.SAMPLE_DURATION_SEC:
            return

        vx = float(message.twist.linear.x)
        vy = float(message.twist.linear.y)
        speed = math.hypot(vx, vy)

        if math.isfinite(speed) and speed >= self.MIN_VALID_SPEED:
            self.samples.append(speed)

    def timer_callback(self) -> None:
        elapsed = time.monotonic() - self.start_time

        if elapsed < self.START_DELAY_SEC:
            remaining = math.ceil(self.START_DELAY_SEC - elapsed)

            if remaining != self.last_countdown:
                self.last_countdown = remaining
                print(f"{remaining}...")
            return

        test_elapsed = elapsed - self.START_DELAY_SEC

        if test_elapsed < self.SAMPLE_DURATION_SEC:
            if self.last_countdown != "sampling":
                self.last_countdown = "sampling"
                print()
                print("GO — HOLD FULL FORWARD THROTTLE STRAIGHT")
            return

        if self.finished:
            return

        self.finished = True
        print()
        print("STOP THROTTLE AND DISARM PX4")
        print()

        if len(self.samples) < 10:
            print("ERROR: Not enough valid velocity samples.")
            print("Check that MAVROS velocity_local is publishing.")
            rclpy.shutdown()
            return

        ordered = sorted(self.samples)
        p95_index = round(0.95 * (len(ordered) - 1))

        average = statistics.fmean(self.samples)
        median = statistics.median(self.samples)
        p95 = ordered[p95_index]
        maximum = max(self.samples)

        print(f"Valid samples : {len(self.samples)}")
        print(f"Average speed : {average:.3f} m/s")
        print(f"Median speed  : {median:.3f} m/s")
        print(f"95% speed     : {p95:.3f} m/s")
        print(f"Maximum speed : {maximum:.3f} m/s")
        print()
        print("Use the stable median/P95 value, not a single maximum spike.")

        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = FullThrottleSpeedMeter()

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
