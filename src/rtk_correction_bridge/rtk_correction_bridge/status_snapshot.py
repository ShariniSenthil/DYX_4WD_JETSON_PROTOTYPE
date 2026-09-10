"""ROS-free projection of authoritative RTCM correction-stream status."""

from __future__ import annotations

import math
from typing import Any

from rtk_correction_bridge.rtcm_transport import (
    RtcmWorkerCounters,
)
from rtk_correction_bridge.serial_rtcm_sink import (
    SerialRtcmSinkSnapshot,
)


def build_correction_status_snapshot(
    *,
    connected: bool,
    healthy: bool,
    correction_age_sec: float,
    counters: RtcmWorkerCounters,
    mavros_subscribers: int,
    max_mavros_rtcm_frame_bytes: int,
    injection_mode: str = "mavros_px4",
    direct_inject: bool = False,
    effective_rtcm_frame_limit_bytes: int | None = None,
    direct_serial_snapshot: SerialRtcmSinkSnapshot | None = None,
    direct_serial_device: str | None = None,
    direct_serial_baud: int | None = None,
    now_monotonic_sec: float | None = None,
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

    mode = str(
        injection_mode
    ).strip().lower()

    if mode not in {
        "mavros_px4",
        "direct_serial",
    }:
        raise ValueError(
            "injection_mode must be "
            "mavros_px4 or direct_serial"
        )

    is_direct = bool(
        direct_inject
    )

    if is_direct != (
        mode == "direct_serial"
    ):
        raise ValueError(
            "direct_inject and injection_mode disagree"
        )

    if (
        direct_serial_snapshot is not None
        and not isinstance(
            direct_serial_snapshot,
            SerialRtcmSinkSnapshot,
        )
    ):
        raise TypeError(
            "direct_serial_snapshot must be "
            "SerialRtcmSinkSnapshot or None"
        )

    configured_legacy_limit = int(
        max_mavros_rtcm_frame_bytes
    )

    effective_limit = int(
        configured_legacy_limit
        if effective_rtcm_frame_limit_bytes is None
        else effective_rtcm_frame_limit_bytes
    )

    is_connected = bool(
        connected
    )

    transport_healthy = bool(
        is_connected
        and healthy
        and age_value is not None
    )

    serial_open = bool(
        direct_serial_snapshot is not None
        and direct_serial_snapshot.serial_open
    )

    is_healthy = bool(
        transport_healthy
        and (
            not is_direct
            or serial_open
        )
    )

    published_frames = max(
        0,
        int(
            counters.rtcm_frames_published_total
        ),
    )

    publish_errors = max(
        0,
        int(
            counters.rtcm_publish_errors_total
        ),
    )

    direct_serial = None

    if is_direct:
        snapshot = direct_serial_snapshot

        last_write_age_sec = None

        if (
            snapshot is not None
            and snapshot.last_successful_write_monotonic
            is not None
            and now_monotonic_sec is not None
        ):
            now_value = float(
                now_monotonic_sec
            )

            last_value = float(
                snapshot.last_successful_write_monotonic
            )

            if (
                math.isfinite(now_value)
                and math.isfinite(last_value)
                and now_value >= last_value
            ):
                last_write_age_sec = (
                    now_value - last_value
                )

        direct_serial = {
            "device": direct_serial_device,
            "baud": (
                None
                if direct_serial_baud is None
                else int(direct_serial_baud)
            ),
            "open": serial_open,
            "open_attempts_total": (
                0
                if snapshot is None
                else snapshot.open_attempts_total
            ),
            "open_failures_total": (
                0
                if snapshot is None
                else snapshot.open_failures_total
            ),
            "reopen_total": (
                0
                if snapshot is None
                else snapshot.reopen_total
            ),
            "frames_written_total": (
                0
                if snapshot is None
                else snapshot.frames_written_total
            ),
            "bytes_written_total": (
                0
                if snapshot is None
                else snapshot.bytes_written_total
            ),
            "write_failures_total": (
                0
                if snapshot is None
                else snapshot.write_failures_total
            ),
            "last_successful_write_age_sec": (
                last_write_age_sec
            ),
        }

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
        "injection_mode": mode,
        "direct_inject": is_direct,
        "effective_rtcm_frame_limit_bytes": (
            effective_limit
        ),
        "direct_serial": direct_serial,
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
        # Compatibility fields retain their existing names. Since Phase 3
        # routes both sinks through attempt_publish(), they count successful
        # active-sink deliveries in either mode.
        "published_frames": published_frames,
        "delivery_frames": published_frames,
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
        "publish_errors": publish_errors,
        "delivery_errors": publish_errors,
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
