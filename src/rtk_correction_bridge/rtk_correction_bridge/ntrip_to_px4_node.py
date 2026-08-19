#!/usr/bin/env python3

import base64
import math
import os
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from mavros_msgs.msg import RTCM
from std_msgs.msg import Bool, Float32


class NtripToPx4Node(Node):

    def __init__(self):
        super().__init__('ntrip_to_px4_node')

        self.declare_parameter(
            'caster_host',
            'caster.emlid.com',
        )

        self.declare_parameter(
            'caster_port',
            2101,
        )

        self.declare_parameter(
            'mountpoint',
            'YOUR_MOUNTPOINT',
        )

        self.declare_parameter(
            'username',
            'YOUR_USERNAME',
        )

        self.declare_parameter(
            'password',
            '',
        )

        self.declare_parameter(
            'rtcm_topic',
            '/mavros/gps_rtk/send_rtcm',
        )

        self.declare_parameter(
            'connect_timeout_sec',
            10.0,
        )

        self.declare_parameter(
            'socket_timeout_sec',
            1.0,
        )

        self.declare_parameter(
            'healthy_age_sec',
            5.0,
        )

        self.declare_parameter(
            'stale_reconnect_sec',
            10.0,
        )

        self.declare_parameter(
            'reconnect_delay_sec',
            5.0,
        )

        self.declare_parameter(
            'health_log_period_sec',
            5.0,
        )

        # Publish /healthy repeatedly so downstream safety logic can
        # verify topic freshness. This does NOT hold RTK FIX.
        self.declare_parameter(
            'health_heartbeat_sec',
            0.25,
        )

        # Maximum wait for the first RTCM payload after NTRIP connects.
        # Transport supervision only; does NOT change Mosaic RTK timeout.
        self.declare_parameter(
            'first_data_timeout_sec',
            10.0,
        )

        self.declare_parameter(
            'max_rtcm_chunk_bytes',
            180,
        )

        self.caster_host = str(
            self.get_parameter(
                'caster_host'
            ).value
        ).strip()

        self.caster_port = int(
            self.get_parameter(
                'caster_port'
            ).value
        )

        self.mountpoint = str(
            self.get_parameter(
                'mountpoint'
            ).value
        ).strip().lstrip('/')

        self.username = str(
            self.get_parameter(
                'username'
            ).value
        )

        self.password = str(
            self.get_parameter(
                'password'
            ).value
        )

        if not self.password:
            self.password = os.environ.get(
                'NTRIP_PASSWORD',
                '',
            )

        self.rtcm_topic = str(
            self.get_parameter(
                'rtcm_topic'
            ).value
        ).strip()

        self.connect_timeout_sec = float(
            self.get_parameter(
                'connect_timeout_sec'
            ).value
        )

        self.socket_timeout_sec = float(
            self.get_parameter(
                'socket_timeout_sec'
            ).value
        )

        self.healthy_age_sec = float(
            self.get_parameter(
                'healthy_age_sec'
            ).value
        )

        self.stale_reconnect_sec = float(
            self.get_parameter(
                'stale_reconnect_sec'
            ).value
        )

        self.reconnect_delay_sec = float(
            self.get_parameter(
                'reconnect_delay_sec'
            ).value
        )

        self.health_log_period_sec = float(
            self.get_parameter(
                'health_log_period_sec'
            ).value
        )

        self.health_heartbeat_sec = float(
            self.get_parameter(
                'health_heartbeat_sec'
            ).value
        )

        self.first_data_timeout_sec = float(
            self.get_parameter(
                'first_data_timeout_sec'
            ).value
        )

        self.max_rtcm_chunk_bytes = int(
            self.get_parameter(
                'max_rtcm_chunk_bytes'
            ).value
        )

        self._validate_parameters()

        self.rtcm_pub = self.create_publisher(
            RTCM,
            self.rtcm_topic,
            50,
        )

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.health_pub = self.create_publisher(
            Bool,
            '/rtk_correction_bridge/healthy',
            status_qos,
        )

        self.age_pub = self.create_publisher(
            Float32,
            '/rtk_correction_bridge/correction_age_sec',
            status_qos,
        )

        self.connected = False
        self.connection_start = None
        self.last_rx = None

        self.total_bytes = 0
        self.total_chunks = 0

        self.stats_bytes = 0
        self.last_stats_time = time.monotonic()
        self.last_health_log = 0.0
        self.last_health_value = None
        self.last_health_publish_time = 0.0

        self._publish_health(
            force=True
        )

        self.get_logger().warn(
            '===== HARDENED NTRIP TO PX4 STARTED ====='
        )

        self.get_logger().warn(
            f'Caster            : '
            f'{self.caster_host}:{self.caster_port}'
        )

        self.get_logger().warn(
            f'Mountpoint        : {self.mountpoint}'
        )

        self.get_logger().warn(
            f'RTCM output       : {self.rtcm_topic}'
        )

        self.get_logger().warn(
            'Health output     : '
            '/rtk_correction_bridge/healthy'
        )

        self.get_logger().warn(
            'Correction age    : '
            '/rtk_correction_bridge/correction_age_sec'
        )

        self.get_logger().warn(
            f'Healthy age limit : '
            f'{self.healthy_age_sec:.1f} s'
        )

        self.get_logger().warn(
            f'Stale reconnect   : '
            f'{self.stale_reconnect_sec:.1f} s'
        )

        self.get_logger().warn(
            f'Health heartbeat  : '
            f'{self.health_heartbeat_sec:.2f} s'
        )

        self.get_logger().warn(
            f'First data timeout: '
            f'{self.first_data_timeout_sec:.1f} s'
        )

        self.get_logger().warn(
            'RTK FIX hold      : NONE in this node'
        )

        self.get_logger().warn(
            'Initial RTCM bytes after handshake are preserved'
        )

        self.get_logger().warn(
            'Password source   : '
            'parameter or NTRIP_PASSWORD environment'
        )

    def _validate_parameters(self):

        if not self.caster_host:
            raise ValueError(
                'caster_host must not be empty'
            )

        if not 1 <= self.caster_port <= 65535:
            raise ValueError(
                'caster_port must be 1..65535'
            )

        if (
            not self.mountpoint
            or self.mountpoint == 'YOUR_MOUNTPOINT'
        ):
            raise ValueError(
                'mountpoint is not configured'
            )

        if (
            not self.username
            or self.username == 'YOUR_USERNAME'
        ):
            raise ValueError(
                'username is not configured'
            )

        if not self.password:
            raise ValueError(
                'NTRIP password is empty'
            )

        if not self.rtcm_topic.startswith('/'):
            raise ValueError(
                'rtcm_topic must be absolute'
            )

        positive_values = {
            'connect_timeout_sec':
                self.connect_timeout_sec,

            'socket_timeout_sec':
                self.socket_timeout_sec,

            'healthy_age_sec':
                self.healthy_age_sec,

            'stale_reconnect_sec':
                self.stale_reconnect_sec,

            'reconnect_delay_sec':
                self.reconnect_delay_sec,

            'health_log_period_sec':
                self.health_log_period_sec,

            'health_heartbeat_sec':
                self.health_heartbeat_sec,

            'first_data_timeout_sec':
                self.first_data_timeout_sec,
        }

        for name, value in positive_values.items():

            if (
                not math.isfinite(value)
                or value <= 0.0
            ):
                raise ValueError(
                    f'{name} must be finite and > 0'
                )

        if (
            self.stale_reconnect_sec
            <= self.healthy_age_sec
        ):
            raise ValueError(
                'stale_reconnect_sec must be greater '
                'than healthy_age_sec'
            )

        if not 1 <= self.max_rtcm_chunk_bytes <= 180:
            raise ValueError(
                'max_rtcm_chunk_bytes must be 1..180'
            )

    def _build_request(self):

        credentials = (
            f'{self.username}:{self.password}'
        )

        auth = base64.b64encode(
            credentials.encode('utf-8')
        ).decode('ascii')

        request = (
            f'GET /{self.mountpoint} HTTP/1.0\r\n'
            f'Host: {self.caster_host}\r\n'
            'User-Agent: NTRIP JetsonPX4/2.0\r\n'
            f'Authorization: Basic {auth}\r\n'
            'Ntrip-Version: Ntrip/2.0\r\n'
            'Connection: close\r\n'
            '\r\n'
        )

        return request.encode('ascii')

    @staticmethod
    def _looks_ascii(data):

        return all(
            byte in (9, 10, 13)
            or 32 <= byte <= 126
            for byte in data
        )

    def _receive_handshake(
        self,
        sock,
    ):

        buffer = bytearray()

        deadline = (
            time.monotonic()
            + self.connect_timeout_sec
        )

        while rclpy.ok():

            if time.monotonic() > deadline:
                raise TimeoutError(
                    'Timed out waiting for NTRIP response'
                )

            data = sock.recv(4096)

            if not data:
                raise ConnectionError(
                    'Caster closed connection '
                    'during handshake'
                )

            buffer.extend(data)

            if len(buffer) > 65536:
                raise RuntimeError(
                    'NTRIP response header '
                    'is unexpectedly large'
                )

            raw = bytes(buffer)

            # NTRIP v1 style.
            if raw.startswith(
                b'ICY 200 OK\r\n'
            ):

                remainder = raw[
                    len(b'ICY 200 OK\r\n'):
                ]

                if remainder.startswith(
                    b'\r\n'
                ):
                    return remainder[2:]

                header_end = remainder.find(
                    b'\r\n\r\n'
                )

                if (
                    header_end >= 0
                    and self._looks_ascii(
                        remainder[:header_end]
                    )
                ):
                    return remainder[
                        header_end + 4:
                    ]

                if (
                    remainder
                    and not self._looks_ascii(
                        remainder
                    )
                ):
                    return remainder

                continue

            # HTTP / NTRIP v2 style.
            header_end = raw.find(
                b'\r\n\r\n'
            )

            if header_end < 0:
                continue

            header = raw[:header_end]

            payload = raw[
                header_end + 4:
            ]

            status_line = header.split(
                b'\r\n',
                1,
            )[0]

            if status_line.startswith(
                b'SOURCETABLE'
            ):
                raise ConnectionError(
                    'Caster returned source table, '
                    'not RTCM stream'
                )

            parts = status_line.split()

            if (
                len(parts) < 2
                or parts[1] != b'200'
            ):
                text = header.decode(
                    'ascii',
                    errors='replace',
                )

                raise ConnectionError(
                    'NTRIP caster rejected '
                    f'connection: {text}'
                )

            return payload

        return b''

    def _connect(self):

        sock = socket.create_connection(
            (
                self.caster_host,
                self.caster_port,
            ),
            timeout=self.connect_timeout_sec,
        )

        sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_KEEPALIVE,
            1,
        )

        sock.settimeout(
            self.connect_timeout_sec
        )

        sock.sendall(
            self._build_request()
        )

        initial_payload = (
            self._receive_handshake(
                sock
            )
        )

        sock.settimeout(
            self.socket_timeout_sec
        )

        self.connected = True

        self.connection_start = (
            time.monotonic()
        )

        self.last_rx = None

        self._publish_health(
            force=True
        )

        self.get_logger().warn(
            'NTRIP caster connected successfully'
        )

        return (
            sock,
            initial_payload,
        )

    def _publish_rtcm_bytes(
        self,
        data,
    ):

        if not data:
            return

        self.last_rx = time.monotonic()

        self.total_bytes += len(data)
        self.stats_bytes += len(data)

        for start in range(
            0,
            len(data),
            self.max_rtcm_chunk_bytes,
        ):

            chunk = data[
                start:
                start + self.max_rtcm_chunk_bytes
            ]

            msg = RTCM()

            msg.data = list(chunk)

            self.rtcm_pub.publish(msg)

            self.total_chunks += 1

        self._publish_health()

        self._maybe_log_health()

    def _correction_age(self):
        """Age of the most recently received RTCM bytes.

        A TCP/NTRIP connection by itself is not a correction sample. Until
        actual RTCM payload bytes have been received, correction age stays
        infinite so downstream nodes cannot treat an empty connection as
        healthy RTK data.
        """

        now = time.monotonic()

        if self.last_rx is not None:
            return now - self.last_rx

        return math.inf

    def _publish_health(
        self,
        force=False,
    ):

        age = self._correction_age()

        healthy = (
            self.connected
            and self.last_rx is not None
            and math.isfinite(age)
            and age <= self.healthy_age_sec
        )

        now = time.monotonic()

        health_changed = (
            healthy != self.last_health_value
        )

        heartbeat_due = (
            now - self.last_health_publish_time
            >= self.health_heartbeat_sec
        )

        if (
            force
            or health_changed
            or heartbeat_due
        ):

            if rclpy.ok():

                msg = Bool()
                msg.data = healthy
                self.health_pub.publish(msg)

                self.last_health_value = healthy
                self.last_health_publish_time = now

        if rclpy.ok():

            age_msg = Float32()

            age_msg.data = (
                float(age)
                if math.isfinite(age)
                else -1.0
            )

            self.age_pub.publish(age_msg)

        return (
            healthy,
            age,
        )

    def _maybe_log_health(
        self,
        force=False,
    ):

        now = time.monotonic()

        if (
            not force
            and now - self.last_health_log
            < self.health_log_period_sec
        ):
            return

        elapsed = max(
            now - self.last_stats_time,
            1e-6,
        )

        rate = (
            self.stats_bytes
            / elapsed
        )

        healthy, age = (
            self._publish_health()
        )

        age_text = (
            f'{age:.2f}s'
            if math.isfinite(age)
            else 'N/A'
        )

        mavros_subscribers = -1

        if rclpy.ok():

            try:
                mavros_subscribers = (
                    self.rtcm_pub.get_subscription_count()
                )

            except Exception:
                mavros_subscribers = -1

        self.get_logger().info(
            'RTK HEALTH | '
            f'connected={self.connected} '
            f'healthy={healthy} '
            f'age={age_text} '
            f'rate={rate:.0f} B/s '
            f'total={self.total_bytes} B '
            f'chunks={self.total_chunks} '
            f'mavros_subscribers={mavros_subscribers}'
        )

        self.stats_bytes = 0

        self.last_stats_time = now

        self.last_health_log = now

    def _set_disconnected(self):

        self.connected = False
        self.connection_start = None
        self.last_rx = None

        if rclpy.ok():

            self._publish_health(
                force=True
            )

            self._maybe_log_health(
                force=True
            )

    def _sleep_with_ros(
        self,
        seconds,
    ):

        deadline = (
            time.monotonic()
            + seconds
        )

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.2,
            )

    def run(self):

        while rclpy.ok():

            sock = None

            try:

                (
                    sock,
                    initial_payload,
                ) = self._connect()

                self._publish_rtcm_bytes(
                    initial_payload
                )

                while rclpy.ok():

                    try:

                        data = sock.recv(4096)

                        if not data:
                            raise ConnectionError(
                                'NTRIP connection '
                                'closed by caster'
                            )

                        self._publish_rtcm_bytes(
                            data
                        )

                    except socket.timeout:

                        healthy, age = (
                            self._publish_health()
                        )

                        self._maybe_log_health()

                        now = time.monotonic()

                        if self.last_rx is None:

                            connected_for = (
                                now - self.connection_start
                                if self.connection_start is not None
                                else 0.0
                            )

                            if (
                                connected_for
                                > self.first_data_timeout_sec
                            ):

                                raise TimeoutError(
                                    'No first RTCM payload for '
                                    f'{connected_for:.1f} s; reconnecting'
                                )

                            self.get_logger().warn(
                                'Waiting for first RTCM bytes from caster | '
                                f'connected_for={connected_for:.1f}s'
                            )

                        else:

                            if (
                                age
                                > self.stale_reconnect_sec
                            ):

                                raise TimeoutError(
                                    'No RTCM data for '
                                    f'{age:.1f} s; reconnecting'
                                )

                            if not healthy:

                                self.get_logger().warn(
                                    'RTCM stream temporarily unhealthy | '
                                    f'age={age:.1f} s'
                                )

                    rclpy.spin_once(
                        self,
                        timeout_sec=0.0,
                    )

            except Exception as exc:

                if rclpy.ok():

                    self.get_logger().error(
                        f'NTRIP stream error: {exc}'
                    )

            finally:

                if sock is not None:

                    try:
                        sock.close()

                    except OSError:
                        pass

                self._set_disconnected()

            if rclpy.ok():

                self.get_logger().warn(
                    'Reconnecting to NTRIP caster in '
                    f'{self.reconnect_delay_sec:.1f} seconds...'
                )

                self._sleep_with_ros(
                    self.reconnect_delay_sec
                )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = None

    try:

        node = NtripToPx4Node()

        node.run()

    except KeyboardInterrupt:
        pass

    finally:

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()