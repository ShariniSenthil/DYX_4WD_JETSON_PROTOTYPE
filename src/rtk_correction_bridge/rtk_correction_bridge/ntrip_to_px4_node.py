#!/usr/bin/env python3

import base64
import json
import math
import socket
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from mavros_msgs.msg import GPSRAW, RTCM
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, String

from rtk_correction_bridge.nmea_gga import (
    GgaSourceFix,
    format_gga_sentence,
)
from rtk_correction_bridge.ntrip_socket import (
    TLS_DISABLED,
    TLS_REQUIRED,
    open_ntrip_socket,
)
from rtk_correction_bridge.ntrip_failures import (
    NtripAuthError,
    NtripMountpointRejectedError,
    validate_ntrip_status_line,
)
from rtk_correction_bridge.rtcm_transport import (
    DEFAULT_MAX_MAVROS_RTCM_FRAME_BYTES,
    RtcmWorkerTransport,
    validate_max_mavros_rtcm_frame_bytes,
)
from rtk_correction_bridge.status_snapshot import (
    build_correction_status_snapshot,
)


class NtripToPx4Node(Node):

    def __init__(self, worker_config=None):
        super().__init__('ntrip_to_px4_node')

        if worker_config is None:
            raise ValueError(
                'worker_config is required; '
                'start RTK through rover_backend'
            )

        # Only non-authority diagnostic timing remains ROS-configurable.
        # Caster credentials, transport policy and injection configuration
        # arrive exclusively through backend-owned WorkerConfig.
        self.declare_parameter(
            'health_log_period_sec',
            5.0,
        )

        self.declare_parameter(
            'health_heartbeat_sec',
            0.25,
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

        self.caster_host = str(
            worker_config.caster_host
        )

        self.caster_port = int(
            worker_config.caster_port
        )

        self.mountpoint = str(
            worker_config.mountpoint
        )

        self.username = str(
            worker_config.username
        )

        self.password = str(
            worker_config.password
        )

        self.rtcm_topic = str(
            worker_config.rtcm_topic
        )

        self.connect_timeout_sec = float(
            worker_config.connect_timeout_sec
        )

        self.socket_timeout_sec = float(
            worker_config.socket_timeout_sec
        )

        self.healthy_age_sec = float(
            worker_config.healthy_age_sec
        )

        self.stale_reconnect_sec = float(
            worker_config.stale_reconnect_sec
        )

        self.reconnect_delay_sec = float(
            worker_config.reconnect_delay_sec
        )

        self.first_data_timeout_sec = float(
            worker_config.first_data_timeout_sec
        )

        self.tls_mode = str(
            worker_config.tls_mode
        )

        self.gga_enabled = bool(
            worker_config.gga_enabled
        )

        self.gga_interval_sec = float(
            worker_config.gga_interval_sec
        )

        self.gga_max_age_sec = float(
            worker_config.gga_max_age_sec
        )

        self.max_mavros_rtcm_frame_bytes = (
            validate_max_mavros_rtcm_frame_bytes(
                worker_config.max_mavros_rtcm_frame_bytes
            )
        )

        self._password_source = (
            'inherited backend config FD'
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

        self.status_pub = self.create_publisher(
            String,
            '/rtk_correction_bridge/status',
            status_qos,
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self._gga_position_callback,
            sensor_qos,
        )

        self.create_subscription(
            GPSRAW,
            '/mavros/gpsstatus/gps1/raw',
            self._gga_gpsraw_callback,
            sensor_qos,
        )

        self._gga_latitude_deg = None
        self._gga_longitude_deg = None
        self._gga_position_at = None

        self._gga_altitude_msl_m = None
        self._gga_fix_type = 0
        self._gga_satellites_visible = 0
        self._gga_hdop = 99.9
        self._gga_gpsraw_at = None

        self._last_gga_sent_at = None
        self._session_first_gga_sent_at = None
        self.gga_sent_total = 0
        self.gga_send_errors = 0

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
            f'Transport security: {self.tls_mode}'
        )

        if self.tls_mode == TLS_DISABLED:
            self.get_logger().error(
                'INSECURE NTRIP PLAINTEXT explicitly enabled; '
                'Basic credentials are not protected by TLS'
            )

        self.get_logger().warn(
            f'GGA/VRS enabled   : {self.gga_enabled}'
        )

        if self.gga_enabled:
            self.get_logger().warn(
                f'GGA interval      : '
                f'{self.gga_interval_sec:.1f} s'
            )

            self.get_logger().warn(
                f'GGA max source age: '
                f'{self.gga_max_age_sec:.1f} s'
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
            f'Password source   : '
            f'{self._password_source}'
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

        if self.tls_mode not in {
            TLS_REQUIRED,
            TLS_DISABLED,
        }:
            raise ValueError(
                'tls_mode must be REQUIRED or DISABLED'
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

            'gga_interval_sec':
                self.gga_interval_sec,

            'gga_max_age_sec':
                self.gga_max_age_sec,
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

    def _gga_position_callback(
        self,
        message,
    ):
        """Cache finite latitude/longitude from MAVROS."""

        now = time.monotonic()

        latitude = float(
            message.latitude
        )

        longitude = float(
            message.longitude
        )

        if (
            math.isfinite(latitude)
            and math.isfinite(longitude)
            and -90.0 <= latitude <= 90.0
            and -180.0 <= longitude <= 180.0
        ):
            self._gga_latitude_deg = latitude
            self._gga_longitude_deg = longitude
        else:
            self._gga_latitude_deg = None
            self._gga_longitude_deg = None

        self._gga_position_at = now

    def _gga_gpsraw_callback(
        self,
        message,
    ):
        """Cache fix type, MSL altitude, satellites and HDOP."""

        now = time.monotonic()

        self._gga_fix_type = int(
            message.fix_type
        )

        satellites = int(
            message.satellites_visible
        )

        self._gga_satellites_visible = (
            0
            if satellites == 255
            else max(
                0,
                satellites,
            )
        )

        eph = int(
            message.eph
        )

        self._gga_hdop = (
            99.9
            if (
                eph <= 0
                or eph == 65535
            )
            else eph / 100.0
        )

        altitude = (
            int(
                message.alt
            )
            / 1000.0
        )

        self._gga_altitude_msl_m = (
            altitude
            if math.isfinite(altitude)
            else None
        )

        self._gga_gpsraw_at = now

    def _gga_source_age(
        self,
        now,
    ):
        if (
            self._gga_position_at is None
            or self._gga_gpsraw_at is None
        ):
            return None

        return max(
            0.0,
            now - self._gga_position_at,
            now - self._gga_gpsraw_at,
        )

    def _gga_source_state(
        self,
        now,
    ):
        if not self.gga_enabled:
            return 'DISABLED', None

        age = self._gga_source_age(
            now
        )

        if (
            age is None
            or self._gga_latitude_deg is None
            or self._gga_longitude_deg is None
            or self._gga_altitude_msl_m is None
        ):
            return 'WAITING_FOR_FIX', age

        if age > self.gga_max_age_sec:
            return 'STALE', age

        if self._gga_fix_type < 2:
            return 'NO_FIX', age

        return 'READY', age

    def _build_current_gga(
        self,
        now,
    ):
        state, age = (
            self._gga_source_state(
                now
            )
        )

        if state != 'READY':
            return None, state, age

        fix = GgaSourceFix(
            latitude_deg=(
                self._gga_latitude_deg
            ),
            longitude_deg=(
                self._gga_longitude_deg
            ),
            altitude_msl_m=(
                self._gga_altitude_msl_m
            ),
            mavlink_fix_type=(
                self._gga_fix_type
            ),
            satellites_visible=(
                self._gga_satellites_visible
            ),
            hdop=self._gga_hdop,
            utc_epoch_sec=time.time(),
        )

        return (
            format_gga_sentence(
                fix
            ),
            state,
            age,
        )

    def _maybe_send_gga(
        self,
        sock,
        now,
        *,
        force=False,
    ):
        """Send fresh GGA on this authenticated caster socket."""

        if not self.gga_enabled:
            return False

        if (
            not force
            and self._last_gga_sent_at
            is not None
            and (
                now - self._last_gga_sent_at
            ) < self.gga_interval_sec
        ):
            return False

        sentence, _, _ = (
            self._build_current_gga(
                now
            )
        )

        if sentence is None:
            return False

        try:
            sock.sendall(
                sentence
            )

        except OSError:
            self.gga_send_errors += 1
            raise

        self._last_gga_sent_at = now

        if (
            self._session_first_gga_sent_at
            is None
        ):
            self._session_first_gga_sent_at = now

        self.gga_sent_total += 1

        return True

    def _gga_status(
        self,
        now,
    ):
        state, source_age = (
            self._gga_source_state(
                now
            )
        )

        sent_age = None

        if self._last_gga_sent_at is not None:
            sent_age = max(
                0.0,
                now - self._last_gga_sent_at,
            )

        return {
            'enabled': self.gga_enabled,
            'state': state,
            'source_age_sec': source_age,
            'last_sent_age_sec': sent_age,
            'sent_total': self.gga_sent_total,
            'send_errors': self.gga_send_errors,
        }

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

            validate_ntrip_status_line(
                status_line
            )

            return payload

        return b''

    def _connect(self):
        """Establish one authenticated NTRIP connection.

        In REQUIRED mode Basic credentials are not written until verified
        TLS succeeds. Any failure before return closes the owned socket.
        There is no TLS-to-plaintext fallback.
        """

        sock = None

        try:
            sock = open_ntrip_socket(
                host=self.caster_host,
                port=self.caster_port,
                timeout_sec=(
                    self.connect_timeout_sec
                ),
                tls_mode=self.tls_mode,
            )

            sock.settimeout(
                self.connect_timeout_sec
            )

            # Authentication is transmitted only after the transport policy
            # above has been successfully established.
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
                'NTRIP caster connected successfully | '
                f'transport={self.tls_mode}'
            )

            return (
                sock,
                initial_payload,
            )

        except BaseException:
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass

            raise

    def _new_parser_session(self):
        """Bind a fresh parser and clear socket-session timestamps."""

        self.transport.new_parser_session()

        self._session_first_gga_sent_at = None

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

            mavros_subscribers = -1

            try:
                mavros_subscribers = (
                    self.rtcm_pub
                    .get_subscription_count()
                )
            except Exception:
                mavros_subscribers = -1

            gga_status = self._gga_status(
                now
            )

            status_payload = (
                build_correction_status_snapshot(
                    connected=self.connected,
                    healthy=healthy,
                    correction_age_sec=age,
                    counters=self.transport.counters,
                    mavros_subscribers=(
                        mavros_subscribers
                    ),
                    max_mavros_rtcm_frame_bytes=(
                        self.max_mavros_rtcm_frame_bytes
                    ),
                    gga_enabled=(
                        gga_status['enabled']
                    ),
                    gga_state=(
                        gga_status['state']
                    ),
                    gga_source_age_sec=(
                        gga_status[
                            'source_age_sec'
                        ]
                    ),
                    gga_last_sent_age_sec=(
                        gga_status[
                            'last_sent_age_sec'
                        ]
                    ),
                    gga_sent_total=(
                        gga_status['sent_total']
                    ),
                    gga_send_errors=(
                        gga_status['send_errors']
                    ),
                )
            )

            status_msg = String()

            status_msg.data = json.dumps(
                status_payload,
                separators=(',', ':'),
                sort_keys=True,
            )

            self.status_pub.publish(
                status_msg
            )

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
        """Enforce RTCM deadlines without blaming a missing VRS GGA."""

        connection_start = (
            self._session_connection_start(
                now
            )
        )

        first_timeout_start = (
            connection_start
        )

        if (
            self.gga_enabled
            and self.transport.last_valid_frame_at
            is None
        ):
            gga_state, _ = (
                self._gga_source_state(
                    now
                )
            )

            # A VRS caster may legitimately send no RTCM until it receives
            # fresh rover position. If GNSS becomes unusable while waiting
            # for the first RTCM frame, suspend that deadline. A later
            # successful fresh GGA starts a new first-frame deadline.
            if gga_state != 'READY':
                self._session_first_gga_sent_at = None
                return

            if (
                self._session_first_gga_sent_at
                is None
            ):
                return

            first_timeout_start = (
                self._session_first_gga_sent_at
            )

        if self.transport.first_valid_frame_timed_out(
            first_timeout_start,
            now,
            self.first_data_timeout_sec,
        ):

            waiting_for = (
                now - first_timeout_start
            )

            raise TimeoutError(
                'No first CRC-valid RTCM frame for '
                f'{waiting_for:.1f} s; reconnecting'
            )

        if self.gga_enabled:
            gga_state, _ = (
                self._gga_source_state(
                    now
                )
            )

            # Reconnecting cannot repair stale/no rover GNSS. Preserve the
            # caster session and let status truthfully report the GGA problem.
            if gga_state != 'READY':
                return

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

                # Drain a few queued MAVROS callbacks before the
                # first VRS GGA attempt.
                if self.gga_enabled:
                    for _ in range(4):
                        rclpy.spin_once(
                            self,
                            timeout_sec=0.0,
                        )

                now = time.monotonic()

                self._maybe_send_gga(
                    sock,
                    now,
                    force=True,
                )

                self._process_stream_bytes(
                    initial_payload,
                    now,
                )

                self._check_source_deadlines(
                    now
                )

                while rclpy.ok():

                    rclpy.spin_once(
                        self,
                        timeout_sec=0.0,
                    )

                    self._maybe_send_gga(
                        sock,
                        time.monotonic(),
                    )

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

            except (
                NtripAuthError,
                NtripMountpointRejectedError,
            ):
                raise

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
    """Reject unmanaged RTK execution.

    Production correction ownership belongs exclusively to rover_backend's
    supervised worker bootstrap. This function remains so stale installed
    console wrappers fail safely instead of becoming a second authority.
    """

    del args

    print(
        "Standalone RTK launch is disabled. "
        "Use rover_backend /api/rtk/start.",
        file=sys.stderr,
    )

    return 2


if __name__ == '__main__':
    raise SystemExit(main())
