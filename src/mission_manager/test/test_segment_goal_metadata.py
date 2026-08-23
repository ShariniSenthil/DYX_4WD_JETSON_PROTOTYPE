import pytest

from mission_manager.path_contract import (
    PendingPreparedPath,
    build_segment_goal_metadata,
    is_valid_path_signature,
    make_path_signature,
    resolve_path_signature,
)


SIGNATURE = "a" * 64


def test_path_signature_matches_trajectory_generator_v1_contract():
    assert (
        make_path_signature(
            [(0.0, 0.0), (1.25, -2.5), (3.0, 4.0)],
            [(1.25, -2.5), (3.0, 4.0)],
            [0, 2, 2],
            [-1, 0, 1],
        )
        == "391db71b37accad9da8a436d041e8e4bd07b1f79a6b9bc4911f8316c9df5123d"
    )


@pytest.mark.parametrize(
    "value",
    ["", "a" * 63, "A" * 64, "g" * 64, "a" * 65],
)
def test_path_signature_rejects_noncanonical_values(value):
    assert not is_valid_path_signature(value)


def test_marking_goal_uses_authoritative_point_id():
    payload = build_segment_goal_metadata(
        path_signature=SIGNATURE,
        goal_sequence=3,
        raw_path_index=42,
        point_type=2,
        marking_index=7,
        marking_point_type=2,
    )

    assert payload == {
        "schema_version": 1,
        "path_signature": SIGNATURE,
        "goal_sequence": 3,
        "raw_path_index": 42,
        "point_type": 2,
        "marking_index": 7,
        "point_id": "P0008",
        "active_goal_identity": "P0008",
    }


def test_dummy_goal_identity_is_path_scoped_without_point_id():
    payload = build_segment_goal_metadata(
        path_signature=SIGNATURE,
        goal_sequence=2,
        raw_path_index=19,
        point_type=1,
        marking_index=-1,
        marking_point_type=2,
    )

    assert payload["point_id"] is None
    assert payload["active_goal_identity"] == (
        f"PATH:{SIGNATURE}:RAW:19:TYPE:1"
    )


def test_goal_metadata_rejects_crossed_semantic_identity():
    with pytest.raises(ValueError, match="marking goal"):
        build_segment_goal_metadata(
            path_signature=SIGNATURE,
            goal_sequence=0,
            raw_path_index=1,
            point_type=2,
            marking_index=-1,
            marking_point_type=2,
        )

    with pytest.raises(ValueError, match="navigation-only"):
        build_segment_goal_metadata(
            path_signature=SIGNATURE,
            goal_sequence=0,
            raw_path_index=1,
            point_type=1,
            marking_index=0,
            marking_point_type=2,
        )


def _snapshot_components():
    navigation_path = [(0.0, 0.0), (1.0, 0.0)]
    mission_waypoints = [(1.0, 0.0)]
    point_types = [0, 2]
    marking_indices = [-1, 0]
    return navigation_path, mission_waypoints, point_types, marking_indices


def test_legacy_contract_loads_without_signature():
    decision = resolve_path_signature(
        *_snapshot_components(),
        None,
        signature_required=False,
    )

    assert decision.can_install
    assert decision.synchronized_signature is None
    assert decision.reason == "legacy_without_signature"


def test_precision_contract_holds_until_signature_is_available():
    decision = resolve_path_signature(
        *_snapshot_components(),
        None,
        signature_required=True,
    )

    assert not decision.can_install
    assert decision.synchronized_signature is None
    assert decision.reason == "signature_required"


def test_signature_is_atomic_commit_check_when_present():
    components = _snapshot_components()
    matching_signature = make_path_signature(*components)

    matched = resolve_path_signature(
        *components,
        matching_signature,
        signature_required=False,
    )
    mismatched = resolve_path_signature(
        *components,
        "b" * 64,
        signature_required=False,
    )

    assert matched.can_install
    assert matched.synchronized_signature == matching_signature
    assert matched.reason == "signature_matched"
    assert not mismatched.can_install
    assert mismatched.reason == "signature_mismatch"


def test_clearing_one_pending_source_preserves_newer_cross_topic_values():
    pending = PendingPreparedPath(
        navigation_path=[(0.0, 0.0)],
        mission_waypoints=[(1.0, 0.0)],
        point_types=[2],
        marking_indices=[0],
        path_signature=SIGNATURE,
        ready=True,
    )

    pending.clear_source("navigation_path")

    assert pending.navigation_path is None
    assert pending.mission_waypoints == [(1.0, 0.0)]
    assert pending.point_types == [2]
    assert pending.marking_indices == [0]
    assert pending.path_signature == SIGNATURE
    assert pending.ready


def test_ready_false_retains_all_pending_components():
    pending = PendingPreparedPath(
        navigation_path=[(0.0, 0.0)],
        mission_waypoints=[(1.0, 0.0)],
        point_types=[2],
        marking_indices=[0],
        path_signature=SIGNATURE,
        ready=True,
    )

    pending.invalidate_readiness()

    assert not pending.ready
    assert pending.navigation_path == [(0.0, 0.0)]
    assert pending.mission_waypoints == [(1.0, 0.0)]
    assert pending.point_types == [2]
    assert pending.marking_indices == [0]
    assert pending.path_signature == SIGNATURE


def test_explicit_local_reset_discards_every_pending_component():
    pending = PendingPreparedPath(
        navigation_path=[(0.0, 0.0)],
        mission_waypoints=[(1.0, 0.0)],
        point_types=[2],
        marking_indices=[0],
        path_signature=SIGNATURE,
        ready=True,
    )

    pending.clear_all()

    assert pending.navigation_path is None
    assert pending.mission_waypoints is None
    assert pending.point_types is None
    assert pending.marking_indices is None
    assert pending.path_signature is None
    assert not pending.ready
