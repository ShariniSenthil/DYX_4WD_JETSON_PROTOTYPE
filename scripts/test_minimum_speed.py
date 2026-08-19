#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MinimumSpeedTester(Node):
    def __init__(self):
        super().__init__("minimum_speed_tester")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.position_east = None
        self.position_north = None
        self.yaw_enu = None
        self.last_odom_monotonic = None

        self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self.odom_callback,
            qos,
        )

        self.command_pub = self.create_publisher(
            Vector3Stamped,
            "/rpp/velocity_ned",
            qos,
        )

    def odom_callback(self, msg):
        self.position_east = float(msg.pose.pose.position.x)
        self.position_north = float(msg.pose.pose.position.y)
        self.yaw_enu = yaw_from_quaternion(
            msg.pose.pose.orientation
        )
        self.last_odom_monotonic = time.monotonic()

    def publish_command(self, speed, yaw_enu):
        velocity_east = speed * math.cos(yaw_enu)
        velocity_north = speed * math.sin(yaw_enu)

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map_ned"

        # Controller interface:
        # vector.x = North velocity
        # vector.y = East velocity
        msg.vector.x = velocity_north
        msg.vector.y = velocity_east
        msg.vector.z = 0.0

        self.command_pub.publish(msg)

    def publish_stop_for(self, duration):
        finish = time.monotonic() + duration

        while rclpy.ok() and time.monotonic() < finish:
            yaw = self.yaw_enu if self.yaw_enu is not None else 0.0
            self.publish_command(0.0, yaw)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.04)

    def wait_for_odom(self, timeout=10.0):
        finish = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < finish:
            rclpy.spin_once(self, timeout_sec=0.1)

            if (
                self.position_east is not None
                and self.position_north is not None
                and self.yaw_enu is not None
            ):
                return True

        return False

    def run_trial(self, speed, duration, trial_number):
        self.publish_stop_for(1.5)

        if (
            self.last_odom_monotonic is None
            or time.monotonic() - self.last_odom_monotonic > 1.0
        ):
            raise RuntimeError("Odometry is stale.")

        start_east = self.position_east
        start_north = self.position_north
        start_yaw = self.yaw_enu

        print(
            f"\nTRIAL {trial_number} START | "
            f"command={speed:.3f} m/s | "
            f"duration={duration:.1f} s"
        )
        sys.stdout.flush()

        start_time = time.monotonic()
        finish_time = start_time + duration

        while rclpy.ok() and time.monotonic() < finish_time:
            self.publish_command(speed, start_yaw)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.04)

        end_east = self.position_east
        end_north = self.position_north

        self.publish_stop_for(2.0)

        delta_east = end_east - start_east
        delta_north = end_north - start_north

        forward_displacement = (
            delta_east * math.cos(start_yaw)
            + delta_north * math.sin(start_yaw)
        )

        lateral_displacement = (
            -delta_east * math.sin(start_yaw)
            + delta_north * math.cos(start_yaw)
        )

        total_displacement = math.hypot(
            delta_east,
            delta_north,
        )

        average_forward_speed = forward_displacement / duration

        print(
            f"TRIAL {trial_number} RESULT | "
            f"forward={forward_displacement:.3f} m | "
            f"lateral={lateral_displacement:.3f} m | "
            f"total={total_displacement:.3f} m | "
            f"average_forward={average_forward_speed:.3f} m/s"
        )
        sys.stdout.flush()

        return (
            forward_displacement,
            lateral_displacement,
            average_forward_speed,
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("speed", type=float)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--trials", type=int, default=3)
    args = parser.parse_args()

    if not 0.0 < args.speed <= 0.10:
        raise SystemExit(
            "Speed must be greater than 0 and at most 0.10 m/s."
        )

    if args.duration < 2.0:
        raise SystemExit("Duration must be at least 2 seconds.")

    if args.trials < 1 or args.trials > 5:
        raise SystemExit("Trials must be between 1 and 5.")

    rclpy.init()
    node = MinimumSpeedTester()

    try:
        if not node.wait_for_odom():
            raise RuntimeError(
                "No local odometry received within 10 seconds."
            )

        results = []

        for trial in range(1, args.trials + 1):
            results.append(
                node.run_trial(
                    args.speed,
                    args.duration,
                    trial,
                )
            )

        average_speed = sum(
            result[2] for result in results
        ) / len(results)

        minimum_forward = min(
            result[0] for result in results
        )

        print("\n===== MINIMUM-SPEED TEST SUMMARY =====")
        print(f"Commanded speed : {args.speed:.3f} m/s")
        print(f"Trials          : {args.trials}")
        print(f"Average measured: {average_speed:.3f} m/s")
        print(f"Minimum movement: {minimum_forward:.3f} m")
        print("======================================")

    finally:
        try:
            node.publish_stop_for(1.0)
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
