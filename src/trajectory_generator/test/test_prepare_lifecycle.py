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

    node.reference_timeout_sec = 1.0
    node.max_reference_skew_sec = 0.25
    node.origin_consistency_max_m = 0.30
    node.required_gps_fix_type = 6
    node.max_correction_age_sec = 2.0
    node.GP_ORIGIN_REQUEST_MAX_ATTEMPTS = (
        TrajectoryGenerator.GP_ORIGIN_REQUEST_MAX_ATTEMPTS
    )
    node.gp_origin_request_attempts = 0

    node.latest_gp_origin = None
    node.latest_fused_global_fix = None
    node.latest_fused_global_time = None
    node.latest_local_odom = None
    node.latest_local_time = None
    node.latest_gps_status = None
    node.latest_gps_status_time = None
    node.rtk_healthy = False
    node.correction_age_sec = float("inf")

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
    _bind(node, "_log_waiting")
    _bind(node, "_age_seconds")
    _bind(node, "_rtk_diagnostic_status")
    _bind(node, "_placement_reference_is_ready")
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


class _TestTime:
    def __init__(self, nanoseconds):
        self.nanoseconds = int(nanoseconds)

    def __sub__(self, other):
        return _TestTime(
            self.nanoseconds - other.nanoseconds
        )

    def to_msg(self):
        return ("stamp", self.nanoseconds)


class _TestClock:
    def __init__(self, nanoseconds):
        self._now = _TestTime(nanoseconds)

    def now(self):
        return self._now


def test_ready_without_signature_is_not_placed():
    node = _fake_generator()

    node._load_mission_source = lambda: _mission(
        "mission-a",
        "a" * 64,
    )

    assert _run_prepare(node).success is True

    node.ready = True
    node.prepared_path_signature = None

    assert node._trajectory_phase() == "COMPILED"

    payload = node._build_status_payload(
        state="READY",
        message="synthetic missing signature",
    )

    assert payload["trajectory_phase"] == "COMPILED"
    assert payload["placement_ready"] is False


def test_control_loop_local_success_reaches_placed_after_signature_publish():
    node = _fake_generator()

    node._load_mission_source = lambda: _mission(
        "mission-a",
        "a" * 64,
    )

    assert _run_prepare(node).success is True
    assert node._trajectory_phase() == "COMPILED"

    events = []

    node.POINT_TYPE_PASS_THROUGH = (
        TrajectoryGenerator.POINT_TYPE_PASS_THROUGH
    )
    node.POINT_TYPE_MARKING = (
        TrajectoryGenerator.POINT_TYPE_MARKING
    )

    node._maybe_request_gp_origin = lambda: None

    node._convert_markings_to_local = lambda: list(
        node.raw_marking_points
    )

    node._generate_navigation_path = lambda markings: (
        list(markings),
        [
            node.POINT_TYPE_MARKING
            for _ in markings
        ],
        list(range(len(markings))),
        0,
    )

    node.get_clock = lambda: _TestClock(
        5_000_000_000
    )

    node._build_path = lambda points, stamp: (
        tuple(points),
        stamp,
    )

    node._make_signature = (
        TrajectoryGenerator._make_signature
    )

    node.mission_waypoints_pub = SimpleNamespace(
        publish=lambda message: events.append(
            ("mission_waypoints", message)
        )
    )

    node.nav_path_pub = SimpleNamespace(
        publish=lambda message: events.append(
            ("nav_path", message)
        )
    )

    node._publish_path_metadata = (
        lambda **kwargs: events.append(
            ("metadata", kwargs)
        )
    )

    node._publish_path_signature = (
        lambda signature: events.append(
            ("signature", signature)
        )
    )

    node._publish_survey_targets = (
        lambda signature, cleared=False: events.append(
            (
                "survey_targets",
                signature,
                cleared,
            )
        )
    )

    node._publish_ready = (
        lambda value: events.append(
            ("ready", bool(value))
        )
    )

    node._publish_status = (
        lambda **kwargs: events.append(
            (
                "status",
                node._build_status_payload(**kwargs),
            )
        )
    )

    TrajectoryGenerator._control_loop(node)

    assert node.prepare_requested is False
    assert node.preparing is False
    assert node.ready is True

    assert node.prepared_path_signature is not None
    assert node._trajectory_phase() == "PLACED"

    event_names = [
        event[0]
        for event in events
    ]

    signature_index = event_names.index(
        "signature"
    )

    ready_index = next(
        index
        for index, event in enumerate(events)
        if event == ("ready", True)
    )

    assert signature_index < ready_index

    ready_statuses = [
        event[1]
        for event in events
        if event[0] == "status"
        and event[1]["state"] == "READY"
    ]

    assert len(ready_statuses) == 1

    ready_status = ready_statuses[0]

    assert ready_status["ready"] is True
    assert ready_status["trajectory_phase"] == "PLACED"
    assert ready_status["placement_ready"] is True
    assert (
        ready_status["path_signature"]
        == node.prepared_path_signature
    )


def test_control_loop_reference_wait_reports_compiled_preparing():
    node = _fake_generator()

    metadata = {
        "mission_id": "mission-gps",
        "checksum_sha256": "c" * 64,
        "extension_mode": "DISABLE",
    }

    node._load_mission_source = lambda: (
        metadata,
        "gps",
        [
            (13.0000000, 80.0000000),
            (13.0000000, 80.0000100),
        ],
    )

    assert _run_prepare(node).success is True
    assert node._trajectory_phase() == "COMPILED"

    payloads = []
    ready_values = []

    node._maybe_request_gp_origin = lambda: None

    node._placement_reference_is_ready = lambda: (
        False,
        "PX4 gp_origin unavailable",
    )

    node.get_clock = lambda: _TestClock(
        5_000_000_000
    )

    node.last_wait_log_time = _TestTime(0)

    node._publish_ready = (
        lambda value: ready_values.append(
            bool(value)
        )
    )

    node._publish_status = (
        lambda **kwargs: payloads.append(
            node._build_status_payload(**kwargs)
        )
    )

    node._convert_markings_to_local = lambda: pytest.fail(
        "placement must not run while reference is unavailable"
    )

    TrajectoryGenerator._control_loop(node)

    assert node.prepare_requested is True
    assert node.preparing is True
    assert node.ready is False

    assert node.rtk_ready_since is None
    assert node._trajectory_phase() == "COMPILED"

    assert ready_values[-1] is False

    assert payloads

    waiting = payloads[-1]

    assert waiting["state"] == "PREPARING"
    assert waiting["ready"] is False
    assert waiting["trajectory_phase"] == "COMPILED"
    assert waiting["placement_ready"] is False
    assert waiting["source_topology_compiled"] is True
    assert (
        waiting["message"]
        == "PX4 gp_origin unavailable"
    )


def test_control_loop_places_gps_immediately_when_reference_is_valid():
    node = _fake_generator()

    metadata = {
        "mission_id": "mission-gps",
        "checksum_sha256": "d" * 64,
        "extension_mode": "DISABLE",
    }

    node._load_mission_source = lambda: (
        metadata,
        "gps",
        [
            (13.0000000, 80.0000000),
            (13.0000000, 80.0000100),
        ],
    )

    assert _run_prepare(node).success is True
    assert node._trajectory_phase() == "COMPILED"

    # A huge legacy dwell value makes this test prove that placement no
    # longer depends on rtk_stable_sec after the reference itself is valid.
    node.rtk_stable_sec = 999.0
    node.rtk_ready_since = None

    node._maybe_request_gp_origin = lambda: None

    node._placement_reference_is_ready = lambda: (
        True,
        "PX4 gp_origin verified: frame residual=0.010m",
    )

    node._log_waiting = lambda reason: pytest.fail(
        f"valid reference must not enter RTK dwell: {reason}"
    )

    conversion_calls = []

    def convert_markings():
        conversion_calls.append(True)
        return [
            (0.0, 0.0),
            (1.0, 0.0),
        ]

    node._convert_markings_to_local = convert_markings

    node.POINT_TYPE_MARKING = (
        TrajectoryGenerator.POINT_TYPE_MARKING
    )
    node.POINT_TYPE_PASS_THROUGH = (
        TrajectoryGenerator.POINT_TYPE_PASS_THROUGH
    )

    node._generate_navigation_path = lambda markings: (
        list(markings),
        [
            node.POINT_TYPE_MARKING
            for _ in markings
        ],
        list(range(len(markings))),
        0,
    )

    node.get_clock = lambda: _TestClock(
        5_000_000_000
    )

    node._build_path = lambda points, stamp: (
        tuple(points),
        stamp,
    )

    node._make_signature = (
        TrajectoryGenerator._make_signature
    )

    node.mission_waypoints_pub = SimpleNamespace(
        publish=lambda _message: None
    )

    node.nav_path_pub = SimpleNamespace(
        publish=lambda _message: None
    )

    node._publish_path_metadata = (
        lambda **_kwargs: None
    )

    node._publish_path_signature = (
        lambda _signature: None
    )

    node._publish_survey_targets = (
        lambda _signature, cleared=False: None
    )

    node._publish_ready = lambda _value: None
    node._publish_status = lambda **_kwargs: None

    TrajectoryGenerator._control_loop(node)

    assert len(conversion_calls) == 1

    assert node.prepare_requested is False
    assert node.preparing is False
    assert node.ready is True

    assert node.prepared_path_signature is not None
    assert node._trajectory_phase() == "PLACED"

    # The old fixed-time stabilization timer is no longer part of placement.
    assert node.rtk_ready_since is None
