#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State


EARTH_RADIUS_M = 6378137.0
TEST_DURATION_SEC = 60.0


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def global_offset_m(lat0, lon0, lat, lon):
    lat0_rad = math.radians(lat0)
    lat_rad = math.radians(lat)

    north = math.radians(lat - lat0) * EARTH_RADIUS_M
    east = (
        math.radians(lon - lon0)
        * EARTH_RADIUS_M
        * math.cos((lat0_rad + lat_rad) * 0.5)
    )

    return east, north


class StationaryMonitor(Node):
    def __init__(self):
        super().__init__("stationary_localization_monitor")

        self.state = None

        self.local_start = None
        self.local_previous = None
        self.local_latest = None

        self.global_start = None
        self.global_latest = None

        self.local_samples = 0
        self.global_samples = 0

        self.max_local_step = 0.0
        self.max_local_speed = 0.0
        self.max_local_displacement = 0.0
        self.max_global_displacement = 0.0

        self.jumps_005 = 0
        self.jumps_010 = 0
        self.jumps_020 = 0
        self.jumps_050 = 0

        self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self.global_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            qos_profile_sensor_data,
        )

        self.create_timer(1.0, self.print_status)

    def state_callback(self, msg):
        self.state = msg

    def odom_callback(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        timestamp = stamp_seconds(msg.header.stamp)

        if not all(math.isfinite(v) for v in (x, y, timestamp)):
            return

        current = (timestamp, x, y)
        self.local_samples += 1
        self.local_latest = current

        if self.local_start is None:
            self.local_start = current

        start_displacement = math.hypot(
            x - self.local_start[1],
            y - self.local_start[2],
        )

        self.max_local_displacement = max(
            self.max_local_displacement,
            start_displacement,
        )

        if self.local_previous is not None:
            previous_time, previous_x, previous_y = self.local_previous

            dt = timestamp - previous_time
            step = math.hypot(
                x - previous_x,
                y - previous_y,
            )

            self.max_local_step = max(
                self.max_local_step,
                step,
            )

            if dt > 1.0e-4:
                calculated_speed = step / dt

                self.max_local_speed = max(
                    self.max_local_speed,
                    calculated_speed,
                )
            else:
                calculated_speed = math.inf

            if step > 0.05:
                self.jumps_005 += 1

            if step > 0.10:
                self.jumps_010 += 1

                self.get_logger().warning(
                    "LOCAL POSITION JUMP | "
                    f"step={step:.3f}m | "
                    f"dt={dt:.3f}s | "
                    f"calculated_speed={calculated_speed:.3f}m/s | "
                    f"E={x:.3f} | N={y:.3f}"
                )

            if step > 0.20:
                self.jumps_020 += 1

            if step > 0.50:
                self.jumps_050 += 1

        self.local_previous = current

    def global_callback(self, msg):
        latitude = float(msg.latitude)
        longitude = float(msg.longitude)

        if not all(math.isfinite(v) for v in (latitude, longitude)):
            return

        if msg.status.status < 0:
            return

        self.global_samples += 1
        self.global_latest = (latitude, longitude)

        if self.global_start is None:
            self.global_start = (latitude, longitude)

        east, north = global_offset_m(
            self.global_start[0],
            self.global_start[1],
            latitude,
            longitude,
        )

        displacement = math.hypot(east, north)

        self.max_global_displacement = max(
            self.max_global_displacement,
            displacement,
        )

    def print_status(self):
        state_text = "NO STATE"

        if self.state is not None:
            state_text = (
                f"connected={self.state.connected} "
                f"armed={self.state.armed} "
                f"mode={self.state.mode}"
            )

        local_text = "local=NO DATA"

        if self.local_latest is not None and self.local_start is not None:
            _, x, y = self.local_latest

            displacement = math.hypot(
                x - self.local_start[1],
                y - self.local_start[2],
            )

            local_text = (
                f"local_E={x:.3f} "
                f"local_N={y:.3f} "
                f"local_drift={displacement:.3f}m"
            )

        global_text = "global=NO DATA"

        if self.global_latest is not None and self.global_start is not None:
            east, north = global_offset_m(
                self.global_start[0],
                self.global_start[1],
                self.global_latest[0],
                self.global_latest[1],
            )

            global_text = (
                f"global_dE={east:.3f}m "
                f"global_dN={north:.3f}m "
                f"global_drift={math.hypot(east, north):.3f}m"
            )

        self.get_logger().info(
            f"{state_text} | {local_text} | {global_text}"
        )

    def print_summary(self):
        print()
        print("===== STATIONARY LOCALIZATION SUMMARY =====")
        print(f"Local samples: {self.local_samples}")
        print(f"Global samples: {self.global_samples}")
        print(f"Maximum local sample step: {self.max_local_step:.3f} m")
        print(f"Maximum calculated local speed: {self.max_local_speed:.3f} m/s")
        print(
            f"Maximum local displacement from start: "
            f"{self.max_local_displacement:.3f} m"
        )
        print(
            f"Maximum global displacement from start: "
            f"{self.max_global_displacement:.3f} m"
        )
        print(f"Local steps greater than 0.05 m: {self.jumps_005}")
        print(f"Local steps greater than 0.10 m: {self.jumps_010}")
        print(f"Local steps greater than 0.20 m: {self.jumps_020}")
        print(f"Local steps greater than 0.50 m: {self.jumps_050}")

        if self.state is not None:
            print(
                "Final PX4 state: "
                f"connected={self.state.connected} "
                f"armed={self.state.armed} "
                f"mode={self.state.mode}"
            )


def main():
    rclpy.init()
    monitor = StationaryMonitor()

    end_time = time.monotonic() + TEST_DURATION_SEC

    try:
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(monitor, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.print_summary()
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
