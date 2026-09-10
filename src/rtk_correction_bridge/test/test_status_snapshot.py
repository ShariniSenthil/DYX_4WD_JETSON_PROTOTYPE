"""Tests for credential-free correction-stream status snapshots."""

from rtk_correction_bridge.rtcm_transport import (
    RtcmWorkerCounters,
)
from rtk_correction_bridge.status_snapshot import (
    build_correction_status_snapshot,
)


def test_disconnected_is_never_healthy():
    payload = build_correction_status_snapshot(
        connected=False,
        healthy=True,
        correction_age_sec=float("inf"),
        counters=RtcmWorkerCounters(),
        mavros_subscribers=0,
        max_mavros_rtcm_frame_bytes=720,
    )

    assert payload["state"] == "DISCONNECTED"
    assert payload["healthy"] is False
    assert payload["correction_age_sec"] is None


def test_connected_without_publish_is_waiting_not_healthy():
    counters = RtcmWorkerCounters(
        rtcm_frames_valid_total=3,
        rtcm_frames_published_total=0,
        rtcm_frames_oversize_total=3,
    )

    payload = build_correction_status_snapshot(
        connected=True,
        healthy=False,
        correction_age_sec=float("inf"),
        counters=counters,
        mavros_subscribers=1,
        max_mavros_rtcm_frame_bytes=720,
    )

    assert (
        payload["state"]
        == "WAITING_FOR_FIRST_PUBLISHED_FRAME"
    )

    assert payload["healthy"] is False
    assert payload["valid_frames"] == 3
    assert payload["published_frames"] == 0
    assert payload["oversize_drops"] == 3


def test_healthy_snapshot_exposes_transport_counters():
    counters = RtcmWorkerCounters(
        socket_bytes_received_total=9000,
        rtcm_frames_valid_total=20,
        rtcm_frames_crc_invalid_total=2,
        rtcm_frames_oversize_total=1,
        rtcm_frames_published_total=19,
        rtcm_publish_errors_total=1,
    )

    payload = build_correction_status_snapshot(
        connected=True,
        healthy=True,
        correction_age_sec=0.25,
        counters=counters,
        mavros_subscribers=1,
        max_mavros_rtcm_frame_bytes=720,
    )

    assert payload["state"] == "HEALTHY"
    assert payload["healthy"] is True
    assert payload["correction_age_sec"] == 0.25
    assert payload["valid_frames"] == 20
    assert payload["published_frames"] == 19
    assert payload["crc_failures"] == 2
    assert payload["oversize_drops"] == 1
    assert payload["publish_errors"] == 1


def test_legacy_status_reports_mavros_mode():
    payload = build_correction_status_snapshot(
        connected=True,
        healthy=True,
        correction_age_sec=0.1,
        counters=RtcmWorkerCounters(
            rtcm_frames_published_total=5,
        ),
        mavros_subscribers=1,
        max_mavros_rtcm_frame_bytes=720,
        injection_mode="mavros_px4",
        direct_inject=False,
        effective_rtcm_frame_limit_bytes=720,
    )

    assert payload["injection_mode"] == "mavros_px4"
    assert payload["direct_inject"] is False
    assert payload["direct_serial"] is None
    assert payload["effective_rtcm_frame_limit_bytes"] == 720
    assert payload["delivery_frames"] == 5


def test_direct_status_exposes_serial_health_and_counters():
    from rtk_correction_bridge.serial_rtcm_sink import (
        SerialRtcmSinkSnapshot,
    )

    serial = SerialRtcmSinkSnapshot(
        serial_open=True,
        open_attempts_total=2,
        open_failures_total=1,
        reopen_total=1,
        frames_written_total=10,
        bytes_written_total=1234,
        write_failures_total=1,
        last_successful_write_monotonic=99.5,
    )

    payload = build_correction_status_snapshot(
        connected=True,
        healthy=True,
        correction_age_sec=0.1,
        counters=RtcmWorkerCounters(
            rtcm_frames_published_total=10,
            rtcm_publish_errors_total=1,
        ),
        mavros_subscribers=1,
        max_mavros_rtcm_frame_bytes=720,
        injection_mode="direct_serial",
        direct_inject=True,
        effective_rtcm_frame_limit_bytes=1029,
        direct_serial_snapshot=serial,
        direct_serial_device="/dev/serial/by-id/mosaic-test",
        direct_serial_baud=230400,
    )

    assert payload["state"] == "HEALTHY"
    assert payload["injection_mode"] == "direct_serial"
    assert payload["effective_rtcm_frame_limit_bytes"] == 1029

    assert payload["delivery_frames"] == 10
    assert payload["delivery_errors"] == 1

    assert payload["direct_serial"]["open"] is True
    assert payload["direct_serial"]["frames_written_total"] == 10
    assert payload["direct_serial"]["bytes_written_total"] == 1234
    assert payload["direct_serial"]["write_failures_total"] == 1


def test_closed_direct_serial_forces_status_unhealthy():
    from rtk_correction_bridge.serial_rtcm_sink import (
        SerialRtcmSinkSnapshot,
    )

    serial = SerialRtcmSinkSnapshot(
        serial_open=False,
        open_attempts_total=2,
        open_failures_total=1,
        reopen_total=0,
        frames_written_total=10,
        bytes_written_total=1234,
        write_failures_total=1,
        last_successful_write_monotonic=99.0,
    )

    payload = build_correction_status_snapshot(
        connected=True,
        healthy=True,
        correction_age_sec=0.2,
        counters=RtcmWorkerCounters(
            rtcm_frames_published_total=10,
        ),
        mavros_subscribers=1,
        max_mavros_rtcm_frame_bytes=720,
        injection_mode="direct_serial",
        direct_inject=True,
        effective_rtcm_frame_limit_bytes=1029,
        direct_serial_snapshot=serial,
        direct_serial_device="/dev/serial/by-id/mosaic-test",
        direct_serial_baud=230400,
    )

    assert payload["healthy"] is False
    assert payload["state"] == "UNHEALTHY"
    assert payload["direct_serial"]["open"] is False
