"""Development-only realtime counters must be free when disabled."""

from rover_backend.realtime_contract import RealtimeMetrics
from rover_backend.realtime_contract import payload_size_bytes
from rover_backend.realtime_contract import timed


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def test_disabled_metrics_record_nothing_and_never_flush():
    metrics = RealtimeMetrics(enabled=False)
    metrics.record_emit("telemetry", {"a": 1})
    with timed(metrics, "build"):
        pass
    assert metrics.maybe_flush() is None
    assert metrics._events == {} and metrics._timings == {}


def test_enabled_metrics_summarise_one_window_then_reset():
    clock = Clock()
    summaries = []
    metrics = RealtimeMetrics(enabled=True, clock=clock, sink=summaries.append)
    assert metrics.maybe_flush() is None  # opens the window
    for _ in range(50):
        metrics.record_emit("telemetry", {"x": 1})
    metrics.record_emit("mission_status", {"points": list(range(100))})
    metrics.record_timing("build_telemetry_payload", 2.0)
    metrics.record_timing("build_telemetry_payload", 4.0)
    metrics.set_gauge("point_results_count", 7)
    clock.now += 0.5
    assert metrics.maybe_flush() is None  # window not elapsed
    clock.now += 0.5
    summary = metrics.maybe_flush()
    assert summaries == [summary]
    assert summary["events"]["telemetry"]["per_sec"] == 50.0
    assert summary["events"]["telemetry"]["avg_bytes"] == payload_size_bytes({"x": 1})
    assert summary["events"]["mission_status"]["max_bytes"] > 100
    assert summary["timings_ms"]["build_telemetry_payload"] == {
        "calls": 2, "avg": 3.0, "max": 4.0,
    }
    assert summary["gauges"]["point_results_count"] == 7
    clock.now += 1.0
    assert metrics.maybe_flush()["events"] == {}
