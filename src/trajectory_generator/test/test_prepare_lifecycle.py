"""Lifecycle tests for source-topology PREPARE state."""

from __future__ import annotations

import threading
from types import MethodType
from types import SimpleNamespace

import pytest


pytest.importorskip("rclpy")

from trajectory_generator.trajectory_generator_node import TrajectoryGenerator


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


def _bind(instance, name):
    setattr(
        instance,
        name,
        MethodType(
            getattr(TrajectoryGenerator, name),
            instance,
        ),
    )


def _fake_generator():
    node = SimpleNamespace()

    node._lock = threading.RLock()

    node.EXTENSION_TRIGGER_DISTANCE_M = (
        TrajectoryGenerator.EXTENSION_TRIGGER_DISTANCE_M
    )
    node.EXTENSION_DISTANCE_M = (
        TrajectoryGenerator.EXTENSION_DISTANCE_M
    )
    node.ROW_TRANSFER_MIN_ANGLE_DEG = (
        TrajectoryGenerator.ROW_TRANSFER_MIN_ANGLE_DEG
    )
    node.ROW_TRANSFER_MAX_ANGLE_DEG = (
        TrajectoryGenerator.ROW_TRANSFER_MAX_ANGLE_DEG
    )
    node.ROW_REVERSAL_MIN_ANGLE_DEG = (
        TrajectoryGenerator.ROW_REVERSAL_MIN_ANGLE_DEG
    )

    node.minimum_segment_length_m = 0.001
    node.interpolation_spacing_m = 0.05
    node.localization_mode = "px4_origin"

    node.prepare_requested = False
    node.preparing = False
    node.ready = False
    node.rtk_ready_since = None

    node.raw_coordinate_mode = None
    node.raw_marking_points = []

    node.source_topology = None
    node.source_topology_mission_id = None
    node.source_topology_checksum = None
    node.source_topology_compile_ms = None

    node.extension_mode = None
    node.dummy_point_distance_m = None
    node.row_transition_threshold_m = None

    node.mission_id = None
    node.mission_checksum = None

    node.prepared_marking_points = []
    node.prepared_navigation_points = []
    node.prepared_path_types = []
    node.prepared_marking_indices = []
    node.prepared_path_signature = None

    node.localization_shadow_summary = {
        "mode": node.localization_mode,
        "candidate_available": False,
        "reason": "not evaluated",
    }

    node.last_error = None

    node.published_ready = []
    node.status_calls = []
    node.empty_output_count = 0

    node._publish_ready = (
        lambda value: node.published_ready.append(bool(value))
    )

    node._publish_empty_outputs = (
        lambda: setattr(
            node,
            "empty_output_count",
            node.empty_output_count + 1,
        )
    )

    node._publish_status = (
        lambda **kwargs: node.status_calls.append(kwargs)
    )

    node.get_logger = lambda: _Logger()

    _bind(node, "_source_topology_is_current")
    _bind(node, "_trajectory_phase")
    _bind(node, "_build_status_payload")
    _bind(node, "_clear_prepared_state")
    _bind(node, "_reset_all_runtime")
    _bind(node, "_set_error")

    return node


def _response():
    return SimpleNamespace(
        success=False,
        message="",
    )


def _mission(
    mission_id,
    checksum,
    *,
    points=None,
):
    if points is None:
        points = [
            (0.0, 0.0),
            (1.0, 0.0),
        ]

    metadata = {
        "mission_id": mission_id,
        "checksum_sha256": checksum,
        "extension_mode": "DISABLE",
    }

    return metadata, "local", points


def _run_prepare(node):
    response = _response()

    result = TrajectoryGenerator._prepare_callback(
        node,
        None,
        response,
    )

    assert result is response

    return response


def test_successful_prepare_binds_topology_to_current_mission():
    node = _fake_generator()

    node._load_mission_source = lambda: _mission(
        "mission-a",
        "a" * 64,
    )

    response = _run_prepare(node)

    assert response.success is True

    assert node.mission_id == "mission-a"
    assert node.mission_checksum == "a" * 64

    assert node.source_topology is not None
    assert node.source_topology_mission_id == "mission-a"
    assert node.source_topology_checksum == "a" * 64

    assert node._source_topology_is_current() is True

    assert node.prepare_requested is True
    assert node.preparing is True
    assert node.ready is False


def test_failed_load_clears_previous_mission_identity_and_topology():
    node = _fake_generator()

    node.mission_id = "mission-old"
    node.mission_checksum = "0" * 64
    node.raw_coordinate_mode = "local"
    node.raw_marking_points = [
        (0.0, 0.0),
        (1.0, 0.0),
    ]

    node.source_topology = object()
    node.source_topology_mission_id = "mission-old"
    node.source_topology_checksum = "0" * 64

    def fail_load():
        raise ValueError("replacement mission invalid")

    node._load_mission_source = fail_load

    response = _run_prepare(node)

    assert response.success is False

    assert node.mission_id is None
    assert node.mission_checksum is None

    assert node.raw_coordinate_mode is None
    assert node.raw_marking_points == []

    assert node.source_topology is None
    assert node.source_topology_mission_id is None
    assert node.source_topology_checksum is None

    assert node.prepare_requested is False
    assert node.preparing is False
    assert node.ready is False


def test_failed_source_compile_keeps_rejected_mission_identity_only():
    node = _fake_generator()

    node._load_mission_source = lambda: _mission(
        "mission-bad",
        "b" * 64,
        points=[
            (0.0, 0.0),
            (0.0001, 0.0),
        ],
    )

    response = _run_prepare(node)

    assert response.success is False

    assert node.mission_id == "mission-bad"
    assert node.mission_checksum == "b" * 64

    assert node.source_topology is None
    assert node.source_topology_mission_id is None
    assert node.source_topology_checksum is None

    assert node._source_topology_is_current() is False

    assert node.prepare_requested is False


def test_placement_failure_preserves_bound_source_topology():
    node = _fake_generator()

    node._load_mission_source = lambda: _mission(
        "mission-a",
        "a" * 64,
    )

    assert _run_prepare(node).success is True
    assert node._source_topology_is_current() is True

    original_topology = node.source_topology

    node._set_error(
        "placement reference unavailable"
    )

    assert node.source_topology is original_topology

    assert node.source_topology_mission_id == "mission-a"
    assert node.source_topology_checksum == "a" * 64

    assert node._source_topology_is_current() is True

    assert node.prepare_requested is False
    assert node.ready is False


def test_full_clear_removes_source_topology_and_identity():
    node = _fake_generator()

    node._load_mission_source = lambda: _mission(
        "mission-a",
        "a" * 64,
    )

    assert _run_prepare(node).success is True
    assert node._source_topology_is_current() is True

    node._reset_all_runtime(
        publish_empty=True,
    )

    assert node.source_topology is None
    assert node.source_topology_mission_id is None
    assert node.source_topology_checksum is None
    assert node.source_topology_compile_ms is None

    assert node.mission_id is None
    assert node.mission_checksum is None

    assert node.raw_coordinate_mode is None
    assert node.raw_marking_points == []

    assert node._source_topology_is_current() is False


def test_prepare_b_never_exposes_prepare_a_binding():
    node = _fake_generator()

    node._load_mission_source = lambda: _mission(
        "mission-a",
        "a" * 64,
    )

    assert _run_prepare(node).success is True
    assert node._source_topology_is_current() is True

    topology_a = node.source_topology

    def load_b():
        # PREPARE clears the previous transaction before loading B.
        assert node.source_topology is None
        assert node.source_topology_mission_id is None
        assert node.source_topology_checksum is None

        assert node.mission_id is None
        assert node.mission_checksum is None

        return _mission(
            "mission-b",
            "b" * 64,
        )

    node._load_mission_source = load_b

    assert _run_prepare(node).success is True

    assert node.source_topology is not topology_a

    assert node.mission_id == "mission-b"
    assert node.mission_checksum == "b" * 64

    assert node.source_topology_mission_id == "mission-b"
    assert node.source_topology_checksum == "b" * 64

    assert node._source_topology_is_current() is True


def test_binding_check_rejects_mission_or_checksum_mismatch():
    node = _fake_generator()

    node._load_mission_source = lambda: _mission(
        "mission-a",
        "a" * 64,
    )

    assert _run_prepare(node).success is True
    assert node._source_topology_is_current() is True

    node.mission_id = "mission-b"

    assert node._source_topology_is_current() is False

    node.mission_id = "mission-a"
    node.mission_checksum = "b" * 64

    assert node._source_topology_is_current() is False


def test_successful_prepare_reports_compiled_phase():
    node = _fake_generator()

    node._load_mission_source = lambda: _mission(
        "mission-a",
        "a" * 64,
    )

    assert _run_prepare(node).success is True

    payload = node._build_status_payload(
        state="PREPARING",
        message="waiting for placement",
    )

    assert payload["trajectory_phase"] == "COMPILED"
    assert payload["placement_ready"] is False

    assert payload["source_topology_compiled"] is True
    assert payload["source_topology_mission_id"] == "mission-a"
    assert payload["source_topology_checksum"] == "a" * 64

    assert payload["source_topology_segment_count"] == 1
    assert payload["source_topology_compile_ms"] is not None


def test_placed_path_reports_placed_phase():
    node = _fake_generator()

    node._load_mission_source = lambda: _mission(
        "mission-a",
        "a" * 64,
    )

    assert _run_prepare(node).success is True

    node.ready = True
    node.prepared_path_signature = "f" * 64
    node.prepared_marking_points = [
        (0.0, 0.0),
        (1.0, 0.0),
    ]
    node.prepared_navigation_points = [
        (0.0, 0.0),
        (1.0, 0.0),
    ]

    payload = node._build_status_payload(
        state="READY",
        message="placed",
    )

    assert payload["trajectory_phase"] == "PLACED"
    assert payload["placement_ready"] is True
    assert payload["source_topology_compiled"] is True
    assert payload["path_signature"] == "f" * 64


def test_placement_failure_returns_to_compiled_phase():
    node = _fake_generator()

    node._load_mission_source = lambda: _mission(
        "mission-a",
        "a" * 64,
    )

    assert _run_prepare(node).success is True

    topology = node.source_topology

    node.ready = True
    node.prepared_path_signature = "f" * 64

    assert node._trajectory_phase() == "PLACED"

    node._set_error(
        "placement reference unavailable"
    )

    assert node.source_topology is topology
    assert node._source_topology_is_current() is True
    assert node._trajectory_phase() == "COMPILED"

    payload = node._build_status_payload(
        state="ERROR",
        message="placement reference unavailable",
    )

    assert payload["trajectory_phase"] == "COMPILED"
    assert payload["placement_ready"] is False
    assert payload["source_topology_compiled"] is True


def test_status_hides_stale_topology_details_on_identity_mismatch():
    node = _fake_generator()

    node._load_mission_source = lambda: _mission(
        "mission-a",
        "a" * 64,
    )

    assert _run_prepare(node).success is True

    assert node._source_topology_is_current() is True

    node.mission_id = "mission-b"

    payload = node._build_status_payload(
        state="PREPARING",
        message="synthetic identity mismatch",
    )

    assert payload["trajectory_phase"] == "UNCOMPILED"
    assert payload["placement_ready"] is False

    assert payload["source_topology_compiled"] is False
    assert payload["source_topology_mission_id"] is None
    assert payload["source_topology_checksum"] is None
    assert payload["source_topology_segment_count"] == 0
    assert payload["source_topology_dummy_decision_count"] == 0
    assert payload["source_topology_compile_ms"] is None


def test_full_clear_reports_uncompiled_phase():
    node = _fake_generator()

    node._load_mission_source = lambda: _mission(
        "mission-a",
        "a" * 64,
    )

    assert _run_prepare(node).success is True
    assert node._trajectory_phase() == "COMPILED"

    node._reset_all_runtime(
        publish_empty=True,
    )

    assert node._trajectory_phase() == "UNCOMPILED"

    payload = node._build_status_payload(
        state="IDLE",
        message="cleared",
    )

    assert payload["trajectory_phase"] == "UNCOMPILED"
    assert payload["placement_ready"] is False
    assert payload["source_topology_compiled"] is False
