"""Regression tests for the pure mission-topology compiler."""

import pytest

from trajectory_generator.topology import compile_metric_topology
from trajectory_generator.topology import compile_source_topology


COMMON = {
    "row_transition_threshold_m": 3.0,
    "minimum_segment_length_m": 0.001,
    "row_transfer_min_angle_deg": 45.0,
    "row_transfer_max_angle_deg": 135.0,
    "row_reversal_min_angle_deg": 135.0,
}


def test_short_straight_segments_do_not_create_dummy():
    topology = compile_metric_topology(
        [
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
        ],
        extension_mode="ENABLE",
        **COMMON,
    )

    assert [
        segment.use_dummy
        for segment in topology.segments
    ] == [
        False,
        False,
    ]


def test_true_serpentine_transition_creates_dummy_decision():
    topology = compile_metric_topology(
        [
            (0.0, 0.0),
            (0.0, 1.0),
            (1.0, 1.0),
            (1.0, 0.0),
        ],
        extension_mode="ENABLE",
        **COMMON,
    )

    first = topology.segments[0]

    assert first.use_dummy is True
    assert first.incoming_direction is not None
    assert first.transfer_angle_deg == pytest.approx(90.0)
    assert first.reversal_angle_deg == pytest.approx(180.0)


def test_extension_disable_never_creates_dummy():
    topology = compile_metric_topology(
        [
            (0.0, 0.0),
            (0.0, 1.0),
            (1.0, 1.0),
            (1.0, 0.0),
        ],
        extension_mode="DISABLE",
        **COMMON,
    )

    assert not any(
        segment.use_dummy
        for segment in topology.segments
    )


def test_duplicate_consecutive_marking_is_rejected():
    with pytest.raises(
        ValueError,
        match="too close",
    ):
        compile_metric_topology(
            [
                (0.0, 0.0),
                (0.0, 0.0),
                (1.0, 0.0),
            ],
            extension_mode="ENABLE",
            **COMMON,
        )


def test_metric_point_order_is_preserved_exactly():
    source = [
        (4.0, -2.0),
        (7.0, -2.0),
        (7.0, 1.0),
        (3.0, 1.0),
    ]

    topology = compile_metric_topology(
        source,
        extension_mode="ENABLE",
        **COMMON,
    )

    assert topology.metric_points == tuple(source)


def test_same_input_produces_identical_topology():
    source = [
        (0.0, 0.0),
        (0.0, 1.0),
        (1.0, 1.0),
        (1.0, 0.0),
    ]

    first = compile_metric_topology(
        source,
        extension_mode="ENABLE",
        **COMMON,
    )

    second = compile_metric_topology(
        source,
        extension_mode="ENABLE",
        **COMMON,
    )

    assert first == second


def test_gps_source_topology_compiles_without_gp_origin_or_live_state():
    gps_points = [
        (13.0000000, 80.0000000),
        (13.0000000, 80.0000200),
        (12.9999900, 80.0000200),
        (12.9999900, 80.0000000),
    ]

    topology = compile_source_topology(
        coordinate_mode="gps",
        raw_marking_points=gps_points,
        extension_mode="ENABLE",
        **COMMON,
    )

    assert len(topology.metric_points) == len(gps_points)
    assert len(topology.segments) == len(gps_points) - 1

    assert topology.metric_points[0][0] == pytest.approx(
        0.0,
        abs=1e-12,
    )

    assert topology.metric_points[0][1] == pytest.approx(
        0.0,
        abs=1e-12,
    )


def test_gps_source_topology_is_repeatable():
    gps_points = [
        (13.18937434, 80.22219259),
        (13.18936988, 80.22223429),
        (13.18936523, 80.22227606),
        (13.18936129, 80.22231909),
    ]

    first = compile_source_topology(
        coordinate_mode="gps",
        raw_marking_points=gps_points,
        extension_mode="ENABLE",
        **COMMON,
    )

    second = compile_source_topology(
        coordinate_mode="gps",
        raw_marking_points=gps_points,
        extension_mode="ENABLE",
        **COMMON,
    )

    assert first == second


def test_gap_exactly_at_threshold_does_not_create_dummy():
    topology = compile_metric_topology(
        [
            (0.0, 0.0),
            (0.0, 3.0),
            (1.0, 3.0),
            (1.0, 0.0),
        ],
        extension_mode="ENABLE",
        **COMMON,
    )

    assert topology.segments[0].distance_m == pytest.approx(3.0)
    assert topology.segments[0].use_dummy is False


def test_minimum_segment_length_exact_boundary_is_accepted():
    topology = compile_metric_topology(
        [
            (0.0, 0.0),
            (0.001, 0.0),
        ],
        extension_mode="DISABLE",
        **COMMON,
    )

    assert topology.segments[0].distance_m == pytest.approx(0.001)


def test_gps_serpentine_source_topology_decisions():
    gps_points = [
        (13.0000000, 80.0000000),
        (13.0000000, 80.0000200),
        (12.9999900, 80.0000200),
        (12.9999900, 80.0000000),
    ]

    topology = compile_source_topology(
        coordinate_mode="gps",
        raw_marking_points=gps_points,
        extension_mode="ENABLE",
        **COMMON,
    )

    assert [
        segment.use_dummy
        for segment in topology.segments
    ] == [
        True,
        True,
        False,
    ]

    # Longitude increase must be East in the temporary survey-local frame.
    assert topology.metric_points[1][0] > 0.0

    # P1 -> P2 changes longitude only, so North displacement stays tiny.
    assert abs(topology.metric_points[1][1]) < 0.01

    # P2 -> P3 moves south.
    assert (
        topology.metric_points[2][1]
        < topology.metric_points[1][1]
    )
