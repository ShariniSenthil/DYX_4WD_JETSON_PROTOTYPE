"""Ask PX4 to stream one MAVLink message at a fixed rate on the MAVROS link.

PX4 sends LOCAL_POSITION_NED (the source of /mavros/local_position/odom) at
about 30 Hz on the Jetson USB link by default. rpp_controller runs at 50 Hz, so
every third control cycle would reuse an old pose. MAV_CMD_SET_MESSAGE_INTERVAL
(through MAVROS /mavros/set_message_interval) raises that rate.

PX4 does not persist the interval: it is lost on every FCU reboot or MAVLink
reconnect. The request is therefore repeated on every new MAVROS connection
and retried until PX4 acknowledges it. Nothing is written to FCU parameters.

This module holds only the retry/state logic so it can be tested without ROS;
cmd_vel_bridge owns the service client.
"""

from __future__ import annotations

import math
from typing import Callable

MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32


class StreamRateRequester:
    """Request one message interval once per FCU connection, with retry."""

    def __init__(
        self,
        *,
        message_id: int,
        rate_hz: float,
        retry_sec: float,
        send: Callable[[int, float], bool],
    ) -> None:
        if not math.isfinite(rate_hz) or rate_hz < 0.0:
            raise ValueError("rate_hz must be finite and >= 0 (0 disables)")
        if not math.isfinite(retry_sec) or retry_sec <= 0.0:
            raise ValueError("retry_sec must be finite and > 0")
        self.message_id = int(message_id)
        self.rate_hz = float(rate_hz)
        self.retry_sec = float(retry_sec)
        self._send = send
        self.confirmed = False
        self.in_flight = False
        self.last_attempt_sec: float | None = None
        self.attempts = 0
        # Incremented on every disconnect so a late answer from a previous
        # MAVLink session cannot mark the new session as configured.
        self.connection_epoch = 0

    @property
    def enabled(self) -> bool:
        return self.rate_hz > 0.0

    def on_state(self, connected: bool, service_ready: bool, now_sec: float) -> bool:
        """Advance on a /mavros/state sample. Returns True if a request was sent."""
        if not self.enabled:
            return False
        if not connected:
            # A new connection is a new PX4 MAVLink session: re-request.
            if self.confirmed or self.in_flight or self.last_attempt_sec is not None:
                self.connection_epoch += 1
            self.confirmed = False
            self.in_flight = False
            self.last_attempt_sec = None
            return False
        if (
            self.in_flight
            and self.last_attempt_sec is not None
            and now_sec - self.last_attempt_sec >= self.retry_sec
        ):
            # No answer within retry_sec: treat as failed and try again.
            self.in_flight = False
        if self.confirmed or self.in_flight or not service_ready:
            return False
        if (
            self.last_attempt_sec is not None
            and now_sec - self.last_attempt_sec < self.retry_sec
        ):
            return False
        self.last_attempt_sec = now_sec
        self.attempts += 1
        self.in_flight = bool(self._send(self.message_id, self.rate_hz))
        return self.in_flight

    def on_response(self, success: bool, epoch: int) -> None:
        """Record PX4's answer. A failure is retried after retry_sec."""
        if epoch != self.connection_epoch:
            return
        self.in_flight = False
        self.confirmed = bool(success)
