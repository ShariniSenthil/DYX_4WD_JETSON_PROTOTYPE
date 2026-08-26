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

from rtk_correction_bridge.rtcm_transport import (
    DEFAULT_MAX_MAVROS_RTCM_FRAME_BYTES,
    RtcmWorkerTransport,
    validate_max_mavros_rtcm_frame_bytes,
)


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

        # Maximum wait for the first CRC-valid RTCM frame after NTRIP
        # connects. Transport supervision only; does NOT change Mosaic
        # RTK timeout. Arbitrary TCP bytes do not satisfy this wait.
        self.declare_parameter(
            'first_data_timeout_sec',
            10.0,
        )

        self.declare_parameter(
            'max_mavros_rtcm_frame_bytes',
            DEFAULT_MAX_MAVROS_RTCM_FRAME_BYTES,
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

        self.max_mavros_rtcm_frame_bytes = (
            validate_max_mavros_rtcm_frame_bytes(
                self.get_parameter(
                    'max_mavros_rtcm_frame_bytes'
                ).value
            )
        )

        self._validate_parameters()

        self.transport = RtcmWorkerTransport(
            max_mavros_rtcm_frame_bytes=(
                self.max_mavros_rtcm_frame_bytes
            ),
        )

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

        self._last_logged_socket_bytes = 0
        self._pending_oversize = 0
        self._pending_publish_errors = 0
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
            f'MAVROS RTCM gate  : '
            f'{self.max_mavros_rtcm_frame_bytes} B '
            f'(protocol max 1029 B)'
        )

        self.get_logger().warn(
            'Publication       : one complete CRC-valid '
            'RTCM3 frame per message, no chunking'
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

        validate_max_mavros_rtcm_frame_bytes(
            self.max_mavros_rtcm_frame_bytes
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

        self._new_parser_session()

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

    def _new_parser_session(self):
        """Bind a fresh parser and clear session correction timestamps."""

        self.transport.new_parser_session()

    def _discard_parser_session(self):
        """Drop the current socket parser and any residual bytes."""

        self.transport.discard_parser_session()

    def _process_stream_bytes(
        self,
        data,
        now,
    ):
        """Feed NTRIP body bytes through the session parser and size gate."""

        oversize_before = (
            self.transport.counters.rtcm_frames_oversize_total
        )

        candidates = self.transport.process_stream_bytes(
            data,
            now,
        )

        self._pending_oversize += (
            self.transport.counters.rtcm_frames_oversize_total
            - oversize_before
        )

        self._process_parsed_frames(
            candidates,
            now,
        )

        self._publish_health()

        self._maybe_log_health()

    def _service_parser(
        self,
        now,
    ):
        """Advance parser timeout with no new socket bytes."""

        oversize_before = (
            self.transport.counters.rtcm_frames_oversize_total
        )

        candidates = self.transport.service_parser(
            now
        )

        self._pending_oversize += (
            self.transport.counters.rtcm_frames_oversize_total
            - oversize_before
        )

        self._process_parsed_frames(
            candidates,
            now,
        )

    def _process_parsed_frames(
        self,
        candidates,
        now,
    ):
        """Publish each size-gated complete RTCM3 frame exactly once."""

        for frame_bytes in candidates:

            published = self.transport.attempt_publish(
                frame_bytes,
                now,
                self._publish_rtcm_frame,
            )

            if not published:
                self._pending_publish_errors += 1

    def _publish_rtcm_frame(
        self,
        frame_bytes,
    ):
        """Publish one complete RTCM3 frame, including header and CRC."""

        msg = RTCM()

        msg.data = list(frame_bytes)

        self.rtcm_pub.publish(msg)

    def _correction_age(self):
        """Age of this session's most recently published supported frame.

        Raw socket traffic, CRC-invalid bytes, oversize protocol-valid
        frames, and a previous socket's publishes do not refresh this age.
        Until this session publishes a supported-size CRC-valid frame, age
        stays infinite so /healthy cannot become true from inherited
        freshness or caster traffic alone.
        """

        return self.transport.published_age_sec(
            time.monotonic()
        )

    def _publish_health(
        self,
        force=False,
    ):

        now = time.monotonic()

        age = self.transport.published_age_sec(
            now
        )

        healthy = self.transport.is_healthy(
            self.connected,
            now,
            self.healthy_age_sec,
        )

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

        socket_bytes = (
            self.transport.counters.socket_bytes_received_total
        )

        period_bytes = (
            socket_bytes - self._last_logged_socket_bytes
        )

        rate = (
            period_bytes
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

        counters = self.transport.counters

        if self._pending_oversize:

            self.get_logger().warn(
                'RTCM oversize frames dropped: '
                f'{self._pending_oversize} '
                f'(gate={self.max_mavros_rtcm_frame_bytes} B, '
                f'total={counters.rtcm_frames_oversize_total})'
            )

            self._pending_oversize = 0

        if self._pending_publish_errors:

            self.get_logger().warn(
                'RTCM publish errors: '
                f'{self._pending_publish_errors} '
                f'(total={counters.rtcm_publish_errors_total})'
            )

            self._pending_publish_errors = 0

        self.get_logger().info(
            'RTK HEALTH | '
            f'connected={self.connected} '
            f'healthy={healthy} '
            f'published_age={age_text} '
            f'rate={rate:.0f} B/s '
            f'socket_bytes={socket_bytes} '
            f'valid_frames={counters.rtcm_frames_valid_total} '
            f'published_frames='
            f'{counters.rtcm_frames_published_total} '
            f'crc_invalid='
            f'{counters.rtcm_frames_crc_invalid_total} '
            f'resync_discarded='
            f'{counters.rtcm_resync_bytes_discarded_total} '
            f'oversize={counters.rtcm_frames_oversize_total} '
            f'publish_errors='
            f'{counters.rtcm_publish_errors_total} '
            f'mavros_subscribers={mavros_subscribers}'
        )

        self._last_logged_socket_bytes = socket_bytes

        self.last_stats_time = now

        self.last_health_log = now

    def _set_disconnected(self):

        self.connected = False
        self.connection_start = None

        if rclpy.ok():

            self._publish_health(
                force=True
            )

            self._maybe_log_health(
                force=True
            )

    def _session_connection_start(
        self,
        now,
    ):
        """Return this socket's connection_start, or now if unset."""

        if self.connection_start is not None:
            return self.connection_start

        return now

    def _check_source_deadlines(
        self,
        now,
    ):
        """Raise if first-valid or stale-source deadline elapsed.

        Call after every parser-processing opportunity. Socket-byte arrival
        does not postpone these deadlines. This helper does not log.
        """

        connection_start = self._session_connection_start(
            now
        )

        if self.transport.first_valid_frame_timed_out(
            connection_start,
            now,
            self.first_data_timeout_sec,
        ):

            connected_for = (
                now - connection_start
            )

            raise TimeoutError(
                'No first CRC-valid RTCM frame for '
                f'{connected_for:.1f} s; reconnecting'
            )

        if self.transport.source_is_stale(
            now,
            self.stale_reconnect_sec,
        ):

            last_valid = (
                self.transport.last_valid_frame_at
            )

            valid_age = (
                now - last_valid
                if last_valid is not None
                else 0.0
            )

            raise TimeoutError(
                'No CRC-valid RTCM frame for '
                f'{valid_age:.1f} s; reconnecting'
            )

    def _maybe_log_source_status(
        self,
        now,
        healthy,
        published_age,
    ):
        """Rate-bounded source warnings. Does not enforce deadlines."""

        if self.transport.last_valid_frame_at is None:

            connection_start = self._session_connection_start(
                now
            )

            connected_for = (
                now - connection_start
            )

            self.get_logger().warn(
                'Waiting for first valid RTCM frame '
                'from caster | '
                f'connected_for={connected_for:.1f}s'
            )

            return

        if not healthy:

            age_text = (
                f'{published_age:.1f} s'
                if math.isfinite(published_age)
                else 'N/A'
            )

            self.get_logger().warn(
                'RTCM stream temporarily unhealthy | '
                f'published_age={age_text}'
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

                now = time.monotonic()

                self._process_stream_bytes(
                    initial_payload,
                    now,
                )

                self._check_source_deadlines(
                    now
                )

                while rclpy.ok():

                    try:

                        data = sock.recv(4096)

                        if not data:
                            raise ConnectionError(
                                'NTRIP connection '
                                'closed by caster'
                            )

                        now = time.monotonic()

                        self._process_stream_bytes(
                            data,
                            now,
                        )

                        self._check_source_deadlines(
                            now
                        )

                    except socket.timeout:

                        now = time.monotonic()

                        self._service_parser(
                            now
                        )

                        self._check_source_deadlines(
                            now
                        )

                        healthy, age = (
                            self._publish_health()
                        )

                        self._maybe_log_health()

                        self._maybe_log_source_status(
                            now,
                            healthy,
                            age,
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

                self._discard_parser_session()

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