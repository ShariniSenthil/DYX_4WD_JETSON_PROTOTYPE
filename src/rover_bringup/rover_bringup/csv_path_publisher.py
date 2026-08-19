#!/usr/bin/env python3

import csv
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class CsvPathPublisher(Node):
    def __init__(self):
        super().__init__('csv_path_publisher')

        self.declare_parameter('csv_file', '/home/flash/rover_ws/missions/mission1.csv')
        self.csv_file = self.get_parameter('csv_file').value

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub = self.create_publisher(Path, '/nav_path', qos)
        self.timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info(f'CSV Path Publisher started: {self.csv_file}')

    def clean_row(self, row):
        return {
            key.strip().lower(): value.strip()
            for key, value in row.items()
            if key is not None and value is not None
        }

    def latlon_to_local_xy(self, lat, lon, origin_lat, origin_lon):
        earth_radius = 6378137.0

        lat_rad = math.radians(lat)
        origin_lat_rad = math.radians(origin_lat)

        x = math.radians(lon - origin_lon) * earth_radius * math.cos(origin_lat_rad)
        y = math.radians(lat - origin_lat) * earth_radius

        return x, y

    def publish_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        try:
            points = []

            with open(self.csv_file, 'r') as f:
                reader = csv.DictReader(f)

                for row in reader:
                    row = self.clean_row(row)

                    if 'x' in row and 'y' in row:
                        x = float(row['x'])
                        y = float(row['y'])
                        points.append((x, y))

                    elif 'latitude' in row and 'longitude' in row:
                        lat = float(row['latitude'])
                        lon = float(row['longitude'])
                        points.append((lat, lon))

                    else:
                        raise RuntimeError(
                            'CSV must contain either x,y or latitude,longitude columns'
                        )

            if len(points) == 0:
                raise RuntimeError('CSV has no path points')

            # If CSV is latitude/longitude, convert to local x/y using first point as origin.
            first_row_is_gps = False
            with open(self.csv_file, 'r') as f:
                reader = csv.DictReader(f)
                first = self.clean_row(next(reader))
                first_row_is_gps = 'latitude' in first and 'longitude' in first

            if first_row_is_gps:
                origin_lat = points[0][0]
                origin_lon = points[0][1]

                local_points = []
                for lat, lon in points:
                    x, y = self.latlon_to_local_xy(lat, lon, origin_lat, origin_lon)
                    local_points.append((x, y))
            else:
                local_points = points

            for x, y in local_points:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = path.header.stamp
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)

            self.pub.publish(path)
            self.get_logger().info(f'Published CSV path with {len(path.poses)} points')

        except Exception as e:
            self.get_logger().error(f'Failed to read CSV path: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CsvPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()