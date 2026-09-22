"""Workstation fixtures for realtime transport tests.

`system_routes` imports `ros_bridge`, which imports rclpy. These helpers
import `system_routes` against a stub `ros_bridge`, scoped through pytest's
monkeypatch so the stub is removed afterwards and can never shadow the real
module when the suite runs on the Jetson.
"""

from __future__ import annotations

import copy
import importlib
import sys
import types
from typing import Any


def import_system_routes(monkeypatch) -> types.ModuleType:
    stub = types.ModuleType("rover_backend.ros_bridge")

    class RosServiceOutcomeUnknownError(RuntimeError):
        pass

    stub.RosServiceOutcomeUnknownError = RosServiceOutcomeUnknownError
    stub.ros_bridge = types.SimpleNamespace(running=False)
    monkeypatch.setitem(sys.modules, "rover_backend.ros_bridge", stub)
    monkeypatch.delitem(sys.modules, "rover_backend.system_routes", raising=False)
    return importlib.import_module("rover_backend.system_routes")


def synthetic_accuracy(index: int) -> dict[str, Any]:
    """Shape and key count of a Mission Manager RPP_TERMINAL_RESULT snapshot."""

    snapshot: dict[str, Any] = {
        "measurement_source": "RPP_TERMINAL_RESULT",
        "available": True,
        "outcome": "ACCURACY_ACHIEVED",
        "reason": "WITHIN_TOLERANCE",
        "rpp_outcome": "CAPTURED",
        "rpp_reason": "RADIAL20_CERTIFIED",
        "point_id": f"P{index + 1:04d}",
        "point_index": index,
        "path_index": index * 44,
        "timestamp_unix_ns": 1_758_000_000_000_000_000 + index,
        "cross_track_error_m": 0.0123,
        "cross_track_error_mm": 12.3,
        "along_track_error_m": -0.0045,
        "along_track_error_mm": -4.5,
        "radial_error_m": 0.0131,
        "radial_error_mm": 13.1,
        "overall_accuracy_mm": 13.1,
        "total_accuracy_mm": 13.1,
        "tolerance_m": 0.03,
        "tolerance_mm": 30.0,
        "within_tolerance": True,
        "speed_mps": 0.004,
        "precision_certificate_version": 2,
        "precision_pass": True,
        "terminal_identity": f"run-1:{index}",
        "mission_run_id": "run-1",
        "path_signature": "a" * 64,
        "raw_path_index": index * 44,
        "active_goal_identity": f"goal-{index}",
        "goal_instance_id": f"goal-{index}-1",
        "heading_error_deg": 0.8,
        "measured_speed_mps": 0.003,
        "measured_yaw_rate_radps": 0.001,
        "stop_spec_mm": 20.0,
        "telemetry_fresh": True,
        "settle_sec": 0.3,
        "captured_at": "2026-09-22T10:00:00.000000+00:00",
    }
    snapshot["survey"] = {
        "available": True,
        "measurement_source": "RAW_GNSS_SURVEY",
        "point_id": snapshot["point_id"],
        "target_lat": 13.1234567,
        "target_lon": 80.1234567,
        "stop_lat": 13.1234568,
        "stop_lon": 80.1234566,
        "along_track_error_mm": 5.1,
        "cross_track_error_mm": -3.2,
        "radial_error_mm": 6.0,
        "fix_type": 6,
        "satellites": 18,
        "h_acc_m": 0.015,
        "samples": 10,
        "window_sec": 0.3,
        "stationary": True,
        "reason": None,
    }
    return snapshot


def synthetic_point_result(index: int) -> dict[str, Any]:
    accuracy = synthetic_accuracy(index)
    spray = {"attempted": True, "outcome": "SUCCESS", "reason": None, "elapsed_sec": 0.5}
    history = [
        {
            "event": name,
            "state": "COMPLETED",
            "reason": None,
            "timestamp_unix_ns": accuracy["timestamp_unix_ns"],
            "received_at": "2026-09-22T10:00:00.000000+00:00",
            "accuracy": copy.deepcopy(accuracy),
            "spray": dict(spray),
        }
        for name in ("ACCURACY_ACHIEVED", "SPRAY_CONFIRMED", "COMPLETED")
    ]
    return {
        "point_id": accuracy["point_id"],
        "point_index": index,
        "event": "COMPLETED",
        "mission_run_id": "run-1",
        "point_outcome": "COMPLETED",
        "spray": spray,
        "spray_attempted": True,
        "spray_outcome": "SUCCESS",
        "spray_confirmed": True,
        "spray_failure_reason": None,
        "spray_elapsed_sec": 0.5,
        "spray_monitor_only": True,
        "overall_accuracy_mm": accuracy["overall_accuracy_mm"],
        "accuracy_remarks": "13.1 mm",
        "accuracy": accuracy,
        "event_history": history,
        "received_at": "2026-09-22T10:00:00.000000+00:00",
    }


def populate_mission(rover_state, total_points: int, completed: int | None = None) -> None:
    """Install a RUNNING mission with `completed` finished points."""

    completed = total_points if completed is None else completed
    point_results = {
        f"P{i + 1:04d}": synthetic_point_result(i) for i in range(completed)
    }
    point_status = ["COMPLETED"] * completed + ["PENDING"] * (total_points - completed)
    survey = {pid: result["accuracy"]["survey"] for pid, result in point_results.items()}
    last = point_results[f"P{completed:04d}"] if completed else None
    rover_state.update(
        "mission",
        mission_id="mission-1",
        mission_run_id="run-1",
        state="RUNNING",
        loaded=True,
        ready=True,
        trajectory_ready=True,
        total_points=total_points,
        completed_points=completed,
        remaining_points=total_points - completed,
        progress_percent=(100.0 * completed / total_points) if total_points else 0.0,
        active_point_index=min(completed, max(0, total_points - 1)),
        active_point_number=min(completed, max(0, total_points - 1)) + 1,
        point_status=point_status,
        point_results=point_results,
        last_point_event=copy.deepcopy(last),
        waypoint_survey_snapshots=survey,
    )
