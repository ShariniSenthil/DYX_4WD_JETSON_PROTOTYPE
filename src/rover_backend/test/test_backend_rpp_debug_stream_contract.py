"""Protect the backend side of the 50 Hz RPP tuning stream."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BRIDGE = PACKAGE_ROOT / "rover_backend" / "ros_bridge.py"
CONFIG = PACKAGE_ROOT / "rover_backend" / "config.py"
ENV = PACKAGE_ROOT / "config" / "backend.env"
ROUTES = PACKAGE_ROOT / "rover_backend" / "system_routes.py"


def test_backend_broadcast_is_configured_for_50_hz():
    assert "DYX_TELEMETRY_BROADCAST_HZ=50.0" in ENV.read_text(encoding="utf-8")
    source = CONFIG.read_text(encoding="utf-8")
    setting = source[source.index('"DYX_TELEMETRY_BROADCAST_HZ"') :]
    assert "50.0" in setting[:100]


def test_debug_subscription_matches_latest_sample_qos():
    source = BRIDGE.read_text(encoding="utf-8")
    subscription = source[source.index('"/rpp/debug"') :]
    assert "debug_qos" in subscription[:200]
    assert "callback_group=self._rpp_debug_callback_group" in subscription[:300]


def test_backend_exposes_stream_freshness_and_drop_detection():
    bridge = BRIDGE.read_text(encoding="utf-8")
    routes = ROUTES.read_text(encoding="utf-8")
    for field in (
        "rpp_debug_receive_age_ms",
        "rpp_debug_stream_fresh",
        "rpp_debug_dropped_frames",
        "rpp_debug_control_sample_age_ms",
    ):
        assert field in bridge
        assert field in routes
