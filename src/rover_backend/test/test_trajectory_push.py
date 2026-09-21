"""Contract tests for the verified generator-path snapshot push."""

from __future__ import annotations

import itertools
import math

import pytest

from mission_manager.path_contract import make_path_signature

from rover_backend import trajectory_push
from rover_backend.trajectory_push import (
    EVENT_CLEARED,
    EVENT_PATH,
    TrajectorySnapshotService,
    legacy_points,
)

PASS, DUMMY, MARK = 0, 1, 2
ORIGIN = (13.0, 80.0)


def fake_projector(lat0, lon0, east, north):
    return lat0 + north * 1e-5, lon0 + east * 1e-5


def make_path(marks=3, per_leg=60, y_offset=0.0):
    nav, types, indices, markings = [], [], [], []
    x = 0.0
    for m in range(marks):
        for k in range(per_leg):
            x += 0.05
            nav.append((x, y_offset))
            last = k == per_leg - 1
            types.append(MARK if last else PASS)
            indices.append(m if last else -1)
        markings.append(nav[-1])
    return nav, markings, types, indices


class Fixture:
    def __init__(self, **kw):
        self.nav, self.markings, self.types, self.indices = make_path(**kw)
        self.signature = make_path_signature(
            self.nav, self.markings, self.types, self.indices
        )

    def status(self, *, ready=True, state="READY", signature=None, count=None, mission="m1"):
        return {
            "state": state,
            "ready": ready,
            "mission_id": mission,
            "mission_checksum": "abc",
            "path_signature": self.signature if signature is None else signature,
            "navigation_point_count": len(self.nav) if count is None else count,
            "coordinate_mode": "gps",
            "localization": {
                "origin_latitude_deg": ORIGIN[0],
                "origin_longitude_deg": ORIGIN[1],
            },
        }

    def inputs(self):
        return {
            "nav": lambda s: s.feed_nav(self.nav),
            "wp": lambda s: s.feed_waypoints(self.markings),
            "types": lambda s: s.feed_types(self.types),
            "idx": lambda s: s.feed_indices(self.indices),
            "sig": lambda s: s.feed_signature(self.signature),
            "status": lambda s: s.feed_status(self.status()),
            "origin": lambda s: s.feed_origin(*ORIGIN),
        }


@pytest.fixture
def service():
    svc = TrajectorySnapshotService(projector=fake_projector)
    svc.events = []
    svc.add_listener(svc.events.append)
    return svc


def paths(svc):
    return [e for e in svc.events if e["event"] == EVENT_PATH]


def clears(svc):
    return [e for e in svc.events if e["event"] == EVENT_CLEARED]


def test_every_arrival_order_emits_exactly_one_path_after_last_input():
    fx = Fixture()
    names = list(fx.inputs())
    for order in itertools.permutations(names):
        svc = TrajectorySnapshotService(projector=fake_projector)
        events = []
        svc.add_listener(events.append)
        for step, name in enumerate(order):
            fx.inputs()[name](svc)
            if step < len(order) - 1:
                assert not events, f"emitted early in order {order}"
        assert len(events) == 1 and events[0]["event"] == EVENT_PATH, order


def test_payload_is_complete_and_consistent(service):
    fx = Fixture()
    for feed in fx.inputs().values():
        feed(service)
    payload = paths(service)[0]["payload"]
    n = len(fx.nav)
    assert payload["count"] == n
    for key in ("x", "y", "lat", "lon"):
        assert len(payload[key]) == n
        assert all(math.isfinite(v) for v in payload[key])
    assert payload["signature"] == fx.signature
    assert payload["mission_id"] == "m1"
    assert payload["server_instance_id"] == service.server_instance_id
    assert payload["origin"] == {"lat": ORIGIN[0], "lon": ORIGIN[1]}
    assert payload["x"][0] == round(fx.nav[0][0], 4)
    assert service.live_payload() is payload


def test_ready_status_first_does_not_emit_until_components_match(service):
    fx = Fixture()
    inputs = fx.inputs()
    inputs["origin"](service)
    inputs["status"](service)
    assert not service.events
    for name in ("nav", "wp", "types", "idx"):
        inputs[name](service)
    assert not service.events  # signature topic still missing
    inputs["sig"](service)
    assert len(paths(service)) == 1


def test_stale_path_with_same_point_count_is_never_paired_with_new_signature(service):
    old = Fixture(y_offset=0.0)
    new = Fixture(y_offset=1.0)  # same length, different content, different signature
    assert len(old.nav) == len(new.nav) and old.signature != new.signature
    # old path/waypoints/types/indices stay cached; only the new signature+READY arrive
    old.inputs()["nav"](service)
    old.inputs()["wp"](service)
    old.inputs()["types"](service)
    old.inputs()["idx"](service)
    service.feed_origin(*ORIGIN)
    service.feed_signature(new.signature)
    service.feed_status(new.status())
    assert not service.events
    # once the matching new path components arrive it emits
    new.inputs()["nav"](service)
    new.inputs()["wp"](service)
    assert len(paths(service)) == 1
    assert paths(service)[0]["payload"]["signature"] == new.signature


def test_status_signature_mismatch_or_wrong_count_does_not_emit(service):
    fx = Fixture()
    for name in ("nav", "wp", "types", "idx", "sig", "origin"):
        fx.inputs()[name](service)
    service.feed_status(fx.status(signature="0" * 64))
    service.feed_status(fx.status(count=len(fx.nav) + 1))
    assert not service.events
    service.feed_status(fx.status())
    assert len(paths(service)) == 1


def test_not_ready_status_clears_live_path_once_with_newer_seq(service):
    fx = Fixture()
    for feed in fx.inputs().values():
        feed(service)
    path_seq = paths(service)[0]["payload"]["seq"]
    service.feed_status(fx.status(ready=False, state="PREPARING"))
    service.feed_status(fx.status(ready=False, state="PREPARING"))
    assert len(clears(service)) == 1
    cleared = clears(service)[0]["payload"]
    assert cleared["seq"] > path_seq
    assert cleared["mission_id"] == "m1" and cleared["signature"] == fx.signature
    assert service.live_payload() is None


def test_empty_source_clears_live_path(service):
    fx = Fixture()
    for feed in fx.inputs().values():
        feed(service)
    service.feed_nav([])
    assert len(clears(service)) == 1 and service.live_payload() is None


def test_no_clear_when_nothing_was_live(service):
    service.feed_status({"state": "PREPARING", "ready": False})
    service.feed_nav([])
    assert not service.events


def test_reprepare_same_geometry_emits_new_seq_after_clear(service):
    fx = Fixture()
    for feed in fx.inputs().values():
        feed(service)
    service.feed_status(fx.status(ready=False, state="PREPARING"))
    for name in ("nav", "wp", "types", "idx", "sig"):
        fx.inputs()[name](service)
    fx.inputs()["status"](service)
    seqs = [e["payload"]["seq"] for e in service.events]
    assert seqs == sorted(seqs) and len(set(seqs)) == 3  # path, clear, path


def test_duplicate_feeds_do_not_reemit(service):
    fx = Fixture()
    for feed in fx.inputs().values():
        feed(service)
    for feed in fx.inputs().values():
        feed(service)
    assert len(paths(service)) == 1 and not clears(service)


def test_origin_change_invalidates_instead_of_reprojecting_old_geometry(service):
    fx = Fixture()
    for feed in fx.inputs().values():
        feed(service)
    service.feed_origin(ORIGIN[0], ORIGIN[1])
    assert len(paths(service)) == 1
    service.feed_origin(ORIGIN[0] + 0.001, ORIGIN[1])
    assert len(paths(service)) == 1
    assert len(clears(service)) == 1
    assert service.live_payload() is None
    # Delayed old READY status cannot re-admit the old projection.
    service.feed_status(fx.status())
    assert service.live_payload() is None
    status = fx.status()
    status["localization"]["origin_latitude_deg"] = ORIGIN[0] + 0.001
    service.feed_status(status)
    assert len(paths(service)) == 2
    assert paths(service)[-1]["payload"]["seq"] > clears(service)[0]["payload"]["seq"]


def test_projection_failure_emits_invalid_clear_and_does_not_raise():
    def bad(*_a):
        raise ValueError("bad origin")

    svc = TrajectorySnapshotService(projector=bad)
    events = []
    svc.add_listener(events.append)
    for feed in Fixture().inputs().values():
        feed(svc)
    assert svc.live_payload() is None
    assert len(events) == 1
    assert events[0]["event"] == EVENT_CLEARED
    assert events[0]["payload"]["reason"] == "snapshot_projection_failed"
    assert events[0]["payload"]["requires_reprepare"] is True


def test_long_path_is_sent_whole_not_truncated(service):
    fx = Fixture(marks=50, per_leg=60)  # 3000 points > old 2000 cap
    for feed in fx.inputs().values():
        feed(service)
    payload = paths(service)[0]["payload"]
    assert payload["count"] == 3000 and len(payload["x"]) == 3000


def test_listener_exception_does_not_break_feed(service):
    def boom(_event):
        raise RuntimeError("listener down")

    service.add_listener(boom)
    for feed in Fixture().inputs().values():
        feed(service)
    assert len(paths(service)) == 1


def test_disabled_flag_does_nothing(monkeypatch):
    monkeypatch.setenv("DYX_TRAJECTORY_PUSH", "0")
    svc = TrajectorySnapshotService(projector=fake_projector)
    events = []
    svc.add_listener(events.append)
    for feed in Fixture().inputs().values():
        feed(svc)
    assert not events and svc.live_payload() is None
    assert trajectory_push.push_enabled() is False


def test_legacy_points_shape_matches_old_preview(service):
    fx = Fixture()
    for feed in fx.inputs().values():
        feed(service)
    points = legacy_points(service.live_payload())
    assert len(points) == len(fx.nav)
    assert set(points[0]) == {"index", "x", "y", "latitude", "longitude", "projection"}
    assert [p["index"] for p in points[:3]] == [0, 1, 2]


def test_new_origin_ready_can_arrive_before_origin_topic(service):
    fx = Fixture()
    for feed in fx.inputs().values():
        feed(service)
    status = fx.status()
    status["localization"]["origin_latitude_deg"] += 0.001
    service.feed_status(status)
    assert service.live_payload() is None
    service.feed_origin(ORIGIN[0] + 0.001, ORIGIN[1])
    assert service.live_payload()["origin"]["lat"] == ORIGIN[0] + 0.001


def test_restart_does_not_pair_retained_gps_path_with_unrelated_origin(service):
    fx = Fixture()
    for name, feed in fx.inputs().items():
        if name != "origin":
            feed(service)
    service.feed_origin(ORIGIN[0] + 0.001, ORIGIN[1])
    assert service.live_payload() is None
    assert not paths(service)


def test_local_path_origin_is_pinned_until_new_preparation(service):
    fx = Fixture()
    status = fx.status()
    status.update(coordinate_mode="local", localization={"mode": "local_input"})
    for name, feed in fx.inputs().items():
        if name != "status":
            feed(service)
    service.feed_status(status)
    assert service.live_payload() is not None
    service.feed_origin(ORIGIN[0] + 0.001, ORIGIN[1])
    service.feed_status(status)
    assert service.live_payload() is None
    service.feed_status(dict(status, state="PREPARING", ready=False))
    service.feed_status(status)
    assert service.live_payload()["origin"]["lat"] == ORIGIN[0] + 0.001


@pytest.mark.parametrize("bad_origin", [None, {}, {"origin_latitude_deg": float("nan"), "origin_longitude_deg": 80}])
def test_gps_origin_provenance_is_required(service, bad_origin):
    fx = Fixture()
    for name, feed in fx.inputs().items():
        if name != "status":
            feed(service)
    status = fx.status()
    status["localization"] = bad_origin
    service.feed_status(status)
    assert service.live_payload() is None


def test_mismatched_new_ready_revokes_previous_live_snapshot(service):
    for feed in Fixture().inputs().values():
        feed(service)
    service.feed_status(Fixture(y_offset=1).status(mission="m2"))
    assert service.live_payload() is None
    assert len(clears(service)) == 1


def test_invalidation_suspends_until_successful_prepare_and_rejects_old_nav_stamp(service):
    fx = Fixture()
    for feed in fx.inputs().values():
        feed(service)
    service.feed_nav(fx.nav, (10, 0))
    service.invalidate("prepare_requested")
    for feed in fx.inputs().values():
        feed(service)
    service.feed_nav(fx.nav, (10, 0))
    service.resume()
    assert service.live_payload() is None
    service.feed_nav(fx.nav, (11, 0))
    assert service.live_payload() is not None
    assert len(paths(service)) == 2


@pytest.mark.parametrize("mission", [
    {"mission_id": "m2", "loaded": True, "trajectory_ready": True},
    {"mission_id": "m1", "loaded": True, "trajectory_ready": False},
    {"mission_id": "m1", "loaded": False, "trajectory_ready": True},
])
def test_network_reads_require_current_mission_and_readiness(service, mission):
    for feed in Fixture().inputs().values():
        feed(service)
    assert service.live_payload(mission) is None


def test_queued_path_event_is_rejected_after_clear(service):
    for feed in Fixture().inputs().values():
        feed(service)
    event = service.events[-1]
    mission = {"mission_id": "m1", "loaded": True, "trajectory_ready": True}
    assert service.event_is_current(event, mission)
    service.invalidate("clear_requested")
    assert not service.event_is_current(event, mission)
    assert service.event_is_current(service.events[-1], mission)


@pytest.mark.parametrize("value,enabled", [
    (None, True), ("1", True), ("true", True), (" YES ", True), ("on", True),
    ("0", False), ("false", False), (" NO ", False), ("off", False),
])
def test_feature_flag_uses_standard_boolean_values(monkeypatch, value, enabled):
    if value is None:
        monkeypatch.delenv("DYX_TRAJECTORY_PUSH", raising=False)
    else:
        monkeypatch.setenv("DYX_TRAJECTORY_PUSH", value)
    assert trajectory_push.push_enabled() is enabled


@pytest.mark.parametrize("value", ["", "disabled", "typo"])
def test_feature_flag_rejects_invalid_configuration(monkeypatch, value):
    monkeypatch.setenv("DYX_TRAJECTORY_PUSH", value)
    with pytest.raises(RuntimeError, match="DYX_TRAJECTORY_PUSH"):
        trajectory_push.push_enabled()


def test_disabled_flag_never_serves_previously_cached_payload(service, monkeypatch):
    for feed in Fixture().inputs().values():
        feed(service)
    monkeypatch.setenv("DYX_TRAJECTORY_PUSH", "0")
    assert service.live_payload() is None


# --- pending vs invalid display contract -----------------------------------


def status_first(svc, fx):
    """READY status + origin arrive before any path component."""
    fx.inputs()["origin"](svc)
    fx.inputs()["status"](svc)


def test_pending_assembly_is_reported_assembling_then_times_out_as_invalid(service):
    fx = Fixture()
    now = [100.0]
    service._clock = lambda: now[0]
    mission = {"loaded": True, "trajectory_ready": True, "mission_id": "m1"}
    status_first(service, fx)
    pending = service.display_state(mission)
    assert (pending["state"], pending["reason"], pending["requires_reprepare"]) == (
        "assembling", "snapshot_pending", False,
    )
    now[0] += service.assembly_timeout_s - 0.5
    assert service.display_state(mission)["state"] == "assembling"
    now[0] += 1.0  # past the bound: no longer "loading" forever
    timed_out = service.display_state(mission)
    assert (timed_out["state"], timed_out["reason"], timed_out["requires_reprepare"]) == (
        "invalid", "snapshot_assembly_timeout", True,
    )
    # a late, complete snapshot still heals it
    for name in ("nav", "wp", "types", "idx", "sig"):
        fx.inputs()[name](service)
    assert service.display_state(mission)["state"] == "live"


def test_origin_not_yet_known_is_pending_not_invalid(service):
    fx = Fixture()
    for name in ("nav", "wp", "types", "idx", "sig", "status"):
        fx.inputs()[name](service)
    assert not service.events
    assert service.display_state()["state"] == "assembling"
    fx.inputs()["origin"](service)
    assert service.display_state()["state"] == "live" and len(paths(service)) == 1


def test_origin_change_is_invalid_and_reprepare_required_then_heals(service):
    fx = Fixture()
    for feed in fx.inputs().values():
        feed(service)
    service.feed_origin(ORIGIN[0] + 0.001, ORIGIN[1])
    cleared = clears(service)[0]["payload"]
    assert cleared["reason"] == "prepared_origin_changed"
    assert cleared["requires_reprepare"] is True
    state = service.display_state()
    assert (state["state"], state["reason"], state["requires_reprepare"]) == (
        "invalid", "prepared_origin_changed", True,
    )
    assert state["server_instance_id"] == service.server_instance_id
    assert state["seq"] == clears(service)[0]["payload"]["seq"]
    service.feed_origin(*ORIGIN)  # the condition healed: valid again, no re-prepare needed
    assert service.display_state()["state"] == "live"
    assert len(paths(service)) == 2


def test_ordinary_lifecycle_clears_do_not_require_reprepare(service):
    fx = Fixture()
    for feed in fx.inputs().values():
        feed(service)
    service.feed_status(fx.status(ready=False, state="PREPARING"))
    assert clears(service)[0]["payload"]["requires_reprepare"] is False
    assert service.display_state()["state"] == "idle"


def test_invalidate_reports_assembling_not_invalid_and_resets_state(service):
    fx = Fixture()
    for feed in fx.inputs().values():
        feed(service)
    service.feed_origin(ORIGIN[0] + 0.001, ORIGIN[1])
    assert service.display_state()["state"] == "invalid"
    service.invalidate("prepare_requested")
    assert service.display_state()["state"] == "assembling"  # preparing again
    service.resume()
    assert service.display_state()["state"] == "idle"


def test_display_state_is_disabled_when_push_is_off(monkeypatch):
    monkeypatch.setenv("DYX_TRAJECTORY_PUSH", "0")
    svc = TrajectorySnapshotService(projector=fake_projector)
    assert svc.display_state()["state"] == "disabled"


# --- stale retained navigation message vs. clock behaviour ------------------


def test_timeout_emits_new_invalid_clear_without_rest_polling(service):
    now = [100.0]
    service._clock = lambda: now[0]
    fx = Fixture()
    for feed in fx.inputs().values():
        feed(service)
    service.feed_types([0] * len(fx.types))
    soft = clears(service)[-1]["payload"]
    assert soft["requires_reprepare"] is False
    now[0] += service.assembly_timeout_s + 1
    service.check_timeout()
    invalid = clears(service)[-1]["payload"]
    assert invalid["seq"] > soft["seq"]
    assert invalid["reason"] == "snapshot_assembly_timeout"
    assert invalid["requires_reprepare"] is True
    assert invalid["mission_id"] == "m1"
    count = len(service.events)
    service.check_timeout()
    service.feed_types([0] * len(fx.types))
    assert len(service.events) == count  # no repeating or reset timeout
    assert service.display_state()["seq"] == invalid["seq"]
    fx.inputs()["types"](service)
    assert service.live_payload()["seq"] > invalid["seq"]


def test_hard_rejection_after_soft_clear_emits_new_sequence(service):
    fx = Fixture()
    for feed in fx.inputs().values():
        feed(service)
    service.feed_types([0] * len(fx.types))
    soft = clears(service)[-1]["payload"]
    service.feed_origin(ORIGIN[0] + 0.001, ORIGIN[1])
    invalid = clears(service)[-1]["payload"]
    assert invalid["seq"] > soft["seq"]
    assert invalid["reason"] == "prepared_origin_changed"
    assert invalid["requires_reprepare"] is True
    count = len(service.events)
    service.feed_status(fx.status())
    assert len(service.events) == count


def test_timeout_before_first_path_emits_identity_and_cancels_on_prepare(service):
    now = [100.0]
    service._clock = lambda: now[0]
    status_first(service, Fixture())
    now[0] += service.assembly_timeout_s + 1
    service.check_timeout()
    invalid = clears(service)[-1]["payload"]
    assert invalid["mission_id"] == "m1"
    assert invalid["requires_reprepare"] is True
    service.invalidate("prepare_requested")
    now[0] += 100
    count = len(service.events)
    service.check_timeout()
    assert len(service.events) == count


def prepared_with_stamp(svc, fx, stamp):
    svc.feed_nav(fx.nav, stamp)
    for name in ("wp", "types", "idx", "sig", "origin", "status"):
        fx.inputs()[name](svc)


def test_stale_retained_nav_with_the_same_stamp_is_rejected_after_invalidate(service):
    fx = Fixture()
    prepared_with_stamp(service, fx, (1000, 5))
    assert len(paths(service)) == 1
    service.invalidate("prepare_requested")
    service.resume()
    service.feed_nav(fx.nav, (1000, 5))  # the old retained message again
    assert service._pending.navigation_path is None
    assert len(paths(service)) == 1  # nothing re-emitted from stale data


def test_new_nav_with_a_stamp_that_moved_backwards_is_accepted(service):
    fx = Fixture()
    prepared_with_stamp(service, fx, (2000, 0))
    service.invalidate("prepare_requested")
    service.resume()
    fresh = Fixture(y_offset=1.0)
    prepared_with_stamp(service, fresh, (1500, 0))  # wall clock stepped back
    assert service._nav_stamp == (1500, 0)
    assert paths(service)[-1]["payload"]["signature"] == fresh.signature


def test_unstamped_nav_stays_blocked_until_a_different_stamp_arrives(service):
    fx = Fixture()
    prepared_with_stamp(service, fx, (10, 0))
    service.invalidate("prepare_requested")
    service.resume()
    service.feed_nav(fx.nav)  # cannot be proven fresh
    assert service._pending.navigation_path is None
    service.feed_nav(fx.nav, (11, 0))
    assert service._pending.navigation_path is not None


def test_blocking_is_lifted_after_a_fresh_message(service):
    fx = Fixture()
    prepared_with_stamp(service, fx, (50, 0))
    service.invalidate("prepare_requested")
    service.resume()
    service.feed_nav(fx.nav, (60, 0))
    assert service._blocked_nav_stamp is None
    service.feed_nav(fx.nav, (50, 0))  # equal to the old blocked stamp, but no longer blocked
    assert service._pending.navigation_path is not None
