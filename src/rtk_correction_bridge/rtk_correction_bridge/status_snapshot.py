"""ROS-free projection of authoritative RTCM correction-stream status."""

from __future__ import annotations

import math
from typing import Any

from rtk_correction_bridge.rtcm_transport import (
    RtcmWorkerCounters,
)


def build_correction_status_snapshot(
    *,
    connected: bool,
    healthy: bool,
    correction_age_sec: float,
    counters: RtcmWorkerCounters,
    mavros_subscribers: int,
    max_mavros_rtcm_frame_bytes: int,
    gga_enabled: bool = False,
    gga_state: str = "DISABLED",
    gga_source_age_sec: float | None = None,
    gga_last_sent_age_sec: float | None = None,
    gga_sent_total: int = 0,
    gga_send_errors: int = 0,
) -> dict[str, Any]:
    """Build one credential-free correction-stream status payload."""

    if not isinstance(
        counters,
        RtcmWorkerCounters,
    ):
        raise TypeError(
            "counters must be RtcmWorkerCounters"
        )

    age = float(correction_age_sec)

    if (
        not math.isfinite(age)
        or age < 0.0
    ):
        age_value = None
    else:
        age_value = age

    is_connected = bool(connected)
    is_healthy = bool(
        is_connected
        and healthy
        and age_value is not None
    )

    published_frames = max(
        0,
        int(
            counters.rtcm_frames_published_total
        ),
    )

    if not is_connected:
        state = "DISCONNECTED"
    elif published_frames <= 0:
        state = (
            "WAITING_FOR_FIRST_PUBLISHED_FRAME"
        )
    elif is_healthy:
        state = "HEALTHY"
    else:
        state = "UNHEALTHY"

    return {
        "state": state,
        "connected": is_connected,
        "healthy": is_healthy,
        "correction_age_sec": age_value,
        "socket_bytes_received": max(
            0,
            int(
                counters.socket_bytes_received_total
            ),
        ),
        "valid_frames": max(
            0,
            int(
                counters.rtcm_frames_valid_total
            ),
        ),
        "published_frames": published_frames,
        "crc_failures": max(
            0,
            int(
                counters.rtcm_frames_crc_invalid_total
            ),
        ),
        "invalid_headers": max(
            0,
            int(
                counters.rtcm_headers_invalid_total
            ),
        ),
        "resync_bytes_discarded": max(
            0,
            int(
                counters.rtcm_resync_bytes_discarded_total
            ),
        ),
        "partial_frame_timeouts": max(
            0,
            int(
                counters.rtcm_partial_frame_timeouts_total
            ),
        ),
        "oversize_drops": max(
            0,
            int(
                counters.rtcm_frames_oversize_total
            ),
        ),
        "publish_errors": max(
            0,
            int(
                counters.rtcm_publish_errors_total
            ),
        ),
        "mavros_subscribers": int(
            mavros_subscribers
        ),
        "max_mavros_rtcm_frame_bytes": int(
            max_mavros_rtcm_frame_bytes
        ),
        "gga": {
            "enabled": bool(
                gga_enabled
            ),
            "state": str(
                gga_state
            ),
            "source_age_sec": (
                float(gga_source_age_sec)
                if (
                    gga_source_age_sec is not None
                    and math.isfinite(
                        float(
                            gga_source_age_sec
                        )
                    )
                    and float(
                        gga_source_age_sec
                    ) >= 0.0
                )
                else None
            ),
            "last_sent_age_sec": (
                float(gga_last_sent_age_sec)
                if (
                    gga_last_sent_age_sec is not None
                    and math.isfinite(
                        float(
                            gga_last_sent_age_sec
                        )
                    )
                    and float(
                        gga_last_sent_age_sec
                    ) >= 0.0
                )
                else None
            ),
            "sent_total": max(
                0,
                int(
                    gga_sent_total
                ),
            ),
            "send_errors": max(
                0,
                int(
                    gga_send_errors
                ),
            ),
        },
    }
