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
