"""Protect the fixed-rate /rpp/debug tuning telemetry contract."""

from pathlib import Path


NODE_PATH = (
    Path(__file__).resolve().parents[1]
    / "rpp_controller"
    / "rpp_controller_node.py"
)


def _source() -> str:
    return NODE_PATH.read_text(encoding="utf-8")


def test_control_rate_is_unchanged_and_telemetry_is_50_hz():
    source = _source()
    assert "CONTROL_HZ = 20.0" in source
    assert "TELEMETRY_HZ = 50.0" in source
    assert "1.0 / self.TELEMETRY_HZ" in source


def test_debug_transport_is_latest_only_and_non_retained():
    source = _source()
    debug_qos_start = source.index("debug_qos = QoSProfile(")
    debug_qos_end = source.index("self.create_subscription(", debug_qos_start)
    debug_qos = source[debug_qos_start:debug_qos_end]
    assert "depth=1" in debug_qos
    assert "ReliabilityPolicy.BEST_EFFORT" in debug_qos
    assert "DurabilityPolicy.VOLATILE" in debug_qos


def test_every_control_exit_commits_a_snapshot():
    source = _source()
    wrapper_start = source.index("    def _control_timer_callback(self):")
    impl_start = source.index("    def control_loop(self):", wrapper_start)
    wrapper = source[wrapper_start:impl_start]
    assert "self._begin_rpp_debug_cycle()" in wrapper
    assert "finally:" in wrapper
    assert "self._finish_rpp_debug_cycle()" in wrapper


def test_transport_exposes_freshness_and_sequence_fields():
    source = _source()
    required_fields = {
        '"schema_version"',
        '"control_sequence"',
        '"telemetry_sequence"',
        '"control_sample_age_ms"',
        '"odom_age_ms"',
        '"control_dt_ms"',
        '"control_compute_ms"',
        '"control_deadline_missed"',
    }
    for field in required_fields:
        assert field in source


def test_debug_json_rejects_nan_payloads():
    source = _source()
    assert "allow_nan=False" in source
