"""Tests for credential-free GGA status projection."""

from rtk_correction_bridge.rtcm_transport import (
    RtcmWorkerCounters,
)
from rtk_correction_bridge.status_snapshot import (
    build_correction_status_snapshot,
)


def test_gga_disabled_status():
    payload = build_correction_status_snapshot(
        connected=True,
        healthy=False,
        correction_age_sec=float("inf"),
        counters=RtcmWorkerCounters(),
        mavros_subscribers=1,
        max_mavros_rtcm_frame_bytes=720,
    )

    assert payload["gga"] == {
        "enabled": False,
        "state": "DISABLED",
        "source_age_sec": None,
        "last_sent_age_sec": None,
        "sent_total": 0,
        "send_errors": 0,
    }


def test_gga_runtime_truth_is_credential_free():
    payload = build_correction_status_snapshot(
        connected=True,
        healthy=False,
        correction_age_sec=float("inf"),
        counters=RtcmWorkerCounters(),
        mavros_subscribers=1,
        max_mavros_rtcm_frame_bytes=720,
        gga_enabled=True,
        gga_state="STALE",
        gga_source_age_sec=8.2,
        gga_last_sent_age_sec=12.0,
        gga_sent_total=4,
        gga_send_errors=1,
    )

    gga = payload["gga"]

    assert gga["enabled"] is True
    assert gga["state"] == "STALE"
    assert gga["source_age_sec"] == 8.2
    assert gga["sent_total"] == 4
    assert gga["send_errors"] == 1
