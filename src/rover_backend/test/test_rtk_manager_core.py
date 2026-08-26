"""Unit tests for the ROS-free RTK manager supervision core.

These tests cover desired-state handling, the MAVROS start gate, run_id
ownership, restart backoff/budget, terminal errors, startup timeout, and
injected monotonic time. They do not spawn processes or import ROS.
"""

from __future__ import annotations

import math
import random
from dataclasses import FrozenInstanceError

import pytest

from rover_backend.rtk_manager_core import (
    DesiredState,
    ErrorReason,
    ManagerState,
    RestartPolicy,
    RtkManagerCore,
    SpawnWorker,
    StopWorker,
    WorkerExitReason,
)


def sequential_ids(prefix: str = 'run'):
    n = 0

    def factory() -> str:
        nonlocal n
        n += 1
        return '%s-%03d' % (prefix, n)

    return factory


def make_manager(run_id_factory=None, **policy_kw) -> RtkManagerCore:
    return RtkManagerCore(
        policy=RestartPolicy(**policy_kw),
        run_id_factory=run_id_factory or sequential_ids(),
    )


def only_spawn(actions) -> str:
    assert len(actions) == 1
    assert isinstance(actions[0], SpawnWorker)
    return actions[0].run_id


def only_stop(actions) -> str:
    assert len(actions) == 1
    assert isinstance(actions[0], StopWorker)
    return actions[0].run_id


def assert_no_actions(actions) -> None:
    assert actions == ()


def start_to_running(mgr: RtkManagerCore, now: float) -> str:
    assert_no_actions(mgr.request_start(now))
    run_id = only_spawn(mgr.set_mavros_ready(True, now))
    assert_no_actions(mgr.on_child_started(run_id, now))
    assert_no_actions(mgr.on_child_ready(run_id, now))
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.RUNNING
    assert snap.active_run_id == run_id
    assert snap.child_started is True
    assert snap.child_ready is True
    return run_id


def crash_retryable(mgr: RtkManagerCore, run_id: str, now: float) -> None:
    assert_no_actions(
        mgr.on_child_exit(run_id, WorkerExitReason.RETRYABLE_FAILURE, now)
    )


def spawn_due(mgr: RtkManagerCore, now: float) -> str:
    return only_spawn(mgr.tick(now))


def assert_invariants(mgr: RtkManagerCore, actions=()) -> None:
    snap = mgr.snapshot
    spawns = [item for item in actions if isinstance(item, SpawnWorker)]
    stops = [item for item in actions if isinstance(item, StopWorker)]
    assert len(spawns) <= 1
    assert len(stops) <= 1
    if snap.manager_state == ManagerState.STOPPED:
        assert snap.active_run_id is None
        assert snap.child_started is False
        assert snap.child_ready is False
    if snap.manager_state == ManagerState.RUNNING:
        assert snap.active_run_id is not None
        assert snap.child_ready is True
        assert snap.desired_state == DesiredState.RUNNING
    if snap.manager_state == ManagerState.ERROR:
        assert not spawns
        assert snap.error_reason is not None
    if snap.desired_state == DesiredState.STOPPED:
        assert not spawns
    if spawns:
        assert snap.active_run_id == spawns[0].run_id
        assert snap.manager_state == ManagerState.STARTING
    if stops:
        assert snap.active_run_id == stops[0].run_id
        assert snap.manager_state == ManagerState.STOPPING


# ---------------------------------------------------------------------------
# 1. Initial STOPPED
# ---------------------------------------------------------------------------


def test_1_initial_stopped():
    mgr = make_manager()
    snap = mgr.snapshot
    assert snap.desired_state == DesiredState.STOPPED
    assert snap.manager_state == ManagerState.STOPPED
    assert snap.mavros_ready is False
    assert snap.active_run_id is None
    assert snap.child_started is False
    assert snap.child_ready is False
    assert snap.next_restart_at is None
    assert snap.consecutive_failures == 0
    assert snap.restart_count_in_window == 0
    assert snap.error_reason is None


# ---------------------------------------------------------------------------
# 2-4. MAVROS start gate and spawn idempotency
# ---------------------------------------------------------------------------


def test_2_start_without_mavros_waits_and_does_not_spawn():
    mgr = make_manager()
    actions = mgr.request_start(0.0)
    assert_no_actions(actions)
    snap = mgr.snapshot
    assert snap.desired_state == DesiredState.RUNNING
    assert snap.manager_state == ManagerState.WAITING_FOR_MAVROS
    assert snap.active_run_id is None


def test_3_mavros_ready_emits_exactly_one_spawn():
    mgr = make_manager()
    mgr.request_start(0.0)
    actions = mgr.set_mavros_ready(True, 0.0)
    run_id = only_spawn(actions)
    assert run_id == 'run-001'
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.STARTING
    assert snap.active_run_id == run_id
    assert snap.child_started is False
    assert snap.child_ready is False


def test_4_repeated_ready_tick_start_do_not_duplicate_spawn():
    mgr = make_manager()
    mgr.request_start(0.0)
    first = only_spawn(mgr.set_mavros_ready(True, 0.0))
    assert_no_actions(mgr.set_mavros_ready(True, 0.0))
    assert_no_actions(mgr.tick(0.0))
    assert_no_actions(mgr.request_start(0.0))
    assert_no_actions(mgr.set_mavros_ready(True, 1.0))
    assert_no_actions(mgr.tick(1.0))
    assert mgr.snapshot.active_run_id == first
    assert mgr.snapshot.manager_state == ManagerState.STARTING


# ---------------------------------------------------------------------------
# 5-8. Matching vs stale child events
# ---------------------------------------------------------------------------


def test_5_child_started_matching_run():
    mgr = make_manager()
    mgr.request_start(0.0)
    run_id = only_spawn(mgr.set_mavros_ready(True, 0.0))
    assert_no_actions(mgr.on_child_started(run_id, 0.0))
    snap = mgr.snapshot
    assert snap.child_started is True
    assert snap.child_ready is False
    assert snap.manager_state == ManagerState.STARTING
    assert snap.active_run_id == run_id


def test_6_child_ready_matching_run_goes_running():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.RUNNING
    assert snap.active_run_id == run_id
    assert snap.child_ready is True


def test_7_stale_ready_wrong_run_id_ignored():
    mgr = make_manager()
    mgr.request_start(0.0)
    run_id = only_spawn(mgr.set_mavros_ready(True, 0.0))
    mgr.on_child_started(run_id, 0.0)
    before = mgr.snapshot
    assert_no_actions(mgr.on_child_ready('run-999', 0.0))
    after = mgr.snapshot
    assert after.manager_state == ManagerState.STARTING
    assert after.child_ready is False
    assert after.active_run_id == run_id
    assert after.manager_state == before.manager_state


def test_8_stale_exit_wrong_run_id_ignored():
    mgr = make_manager()
    mgr.request_start(0.0)
    run_id = only_spawn(mgr.set_mavros_ready(True, 0.0))
    mgr.on_child_started(run_id, 0.0)
    assert_no_actions(
        mgr.on_child_exit(
            'run-999', WorkerExitReason.RETRYABLE_FAILURE, 0.0
        )
    )
    snap = mgr.snapshot
    assert snap.active_run_id == run_id
    assert snap.manager_state == ManagerState.STARTING
    assert snap.child_started is True


# ---------------------------------------------------------------------------
# 9-10. MAVROS loss and recovery while worker is running
# ---------------------------------------------------------------------------


def test_9_mavros_loss_while_running_keeps_worker_degraded():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    actions = mgr.set_mavros_ready(False, 1.0)
    assert_no_actions(actions)
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.RUNNING_MAVROS_STALE
    assert snap.active_run_id == run_id
    assert snap.child_ready is True
    assert snap.mavros_ready is False
    assert_no_actions(mgr.tick(2.0))
    assert mgr.snapshot.active_run_id == run_id
    assert mgr.snapshot.manager_state == ManagerState.RUNNING_MAVROS_STALE


def test_10_mavros_recovery_keeps_same_worker_no_new_spawn():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    mgr.set_mavros_ready(False, 1.0)
    actions = mgr.set_mavros_ready(True, 2.0)
    assert_no_actions(actions)
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.RUNNING
    assert snap.active_run_id == run_id
    assert snap.child_ready is True
    assert_no_actions(mgr.tick(3.0))
    assert mgr.snapshot.active_run_id == run_id


# ---------------------------------------------------------------------------
# 11-14. Retryable failure, backoff deadline, new run_id
# ---------------------------------------------------------------------------


def test_11_retryable_failure_clears_run_and_enters_backoff():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    crash_retryable(mgr, run_id, 1.0)
    snap = mgr.snapshot
    assert snap.active_run_id is None
    assert snap.child_started is False
    assert snap.child_ready is False
    assert snap.manager_state == ManagerState.BACKOFF
    assert snap.next_restart_at == 2.0
    assert snap.consecutive_failures == 1
    assert snap.desired_state == DesiredState.RUNNING


def test_12_before_deadline_no_spawn():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    crash_retryable(mgr, run_id, 1.0)
    assert_no_actions(mgr.tick(1.0))
    assert_no_actions(mgr.tick(1.999))
    assert mgr.snapshot.manager_state == ManagerState.BACKOFF
    assert mgr.snapshot.active_run_id is None


def test_13_exactly_at_deadline_one_new_spawn():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    crash_retryable(mgr, run_id, 1.0)
    new_id = spawn_due(mgr, 2.0)
    assert new_id == 'run-002'
    assert mgr.snapshot.manager_state == ManagerState.STARTING
    assert_no_actions(mgr.tick(2.0))
    assert_no_actions(mgr.tick(3.0))


def test_14_new_run_id_differs_from_old():
    mgr = make_manager()
    old_id = start_to_running(mgr, 0.0)
    crash_retryable(mgr, old_id, 1.0)
    new_id = spawn_due(mgr, 2.0)
    assert new_id != old_id
    assert mgr.snapshot.active_run_id == new_id


# ---------------------------------------------------------------------------
# 15. Exponential backoff 1,2,4,8,16,30 cap
# ---------------------------------------------------------------------------


def test_15_exponential_backoff_sequence_and_cap():
    mgr = make_manager(max_restarts_in_window=20)
    run_id = start_to_running(mgr, 0.0)
    now = 0.0
    expected = [1.0, 2.0, 4.0, 8.0, 16.0, 30.0, 30.0]
    delays = []
    current = run_id
    for _ in expected:
        now += 0.0
        crash_retryable(mgr, current, now)
        snap = mgr.snapshot
        assert snap.manager_state == ManagerState.BACKOFF
        delay = snap.next_restart_at - now
        delays.append(delay)
        now = snap.next_restart_at
        current = spawn_due(mgr, now)
        mgr.on_child_started(current, now)
        mgr.on_child_ready(current, now)
    assert delays == expected


# ---------------------------------------------------------------------------
# 16. Stable run >= 60s resets next backoff to 1s
# ---------------------------------------------------------------------------


def test_16_stable_run_resets_backoff_to_initial():
    mgr = make_manager(max_restarts_in_window=20)
    run_id = start_to_running(mgr, 0.0)
    crash_retryable(mgr, run_id, 1.0)
    assert mgr.snapshot.next_restart_at == 2.0
    run_id = spawn_due(mgr, 2.0)
    mgr.on_child_started(run_id, 2.0)
    mgr.on_child_ready(run_id, 2.0)
    crash_retryable(mgr, run_id, 2.0)
    assert mgr.snapshot.next_restart_at == 4.0
    assert mgr.snapshot.consecutive_failures == 2
    run_id = spawn_due(mgr, 4.0)
    mgr.on_child_started(run_id, 4.0)
    mgr.on_child_ready(run_id, 4.0)
    crash_retryable(mgr, run_id, 64.0)
    snap = mgr.snapshot
    assert snap.consecutive_failures == 1
    assert snap.next_restart_at == 65.0
    assert snap.next_restart_at - 64.0 == 1.0


# ---------------------------------------------------------------------------
# 17-18. Restart budget; initial start is not an automatic restart
# ---------------------------------------------------------------------------


def test_17_and_18_restart_budget_fifth_allowed_sixth_errors():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    assert mgr.snapshot.restart_count_in_window == 0
    now = 0.0
    current = run_id
    auto_spawns = 0
    for _ in range(5):
        crash_retryable(mgr, current, now)
        assert mgr.snapshot.manager_state == ManagerState.BACKOFF
        now = mgr.snapshot.next_restart_at
        current = spawn_due(mgr, now)
        auto_spawns += 1
        mgr.on_child_started(current, now)
        mgr.on_child_ready(current, now)
    assert auto_spawns == 5
    assert mgr.snapshot.restart_count_in_window == 5
    crash_retryable(mgr, current, now)
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.BACKOFF
    assert snap.error_reason is None
    deadline = snap.next_restart_at
    assert deadline is not None
    assert_no_actions(mgr.tick(deadline))
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.ERROR
    assert snap.error_reason == ErrorReason.RESTART_BUDGET_EXHAUSTED
    assert snap.active_run_id is None
    assert_no_actions(mgr.tick(deadline + 100.0))
    assert_no_actions(mgr.set_mavros_ready(True, deadline + 100.0))


def test_restart_budget_ages_out_during_backoff_before_spawn():
    mgr = make_manager(
        restart_window_sec=3.0,
        max_restarts_in_window=2,
    )
    current = start_to_running(mgr, 0.0)
    crash_retryable(mgr, current, 0.0)
    current = spawn_due(mgr, 1.0)
    mgr.on_child_ready(current, 1.0)
    crash_retryable(mgr, current, 1.0)
    current = spawn_due(mgr, 3.0)
    mgr.on_child_ready(current, 3.0)

    crash_retryable(mgr, current, 3.0)
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.BACKOFF
    assert snap.error_reason is None
    assert snap.next_restart_at == 7.0

    actions = mgr.tick(7.0)
    assert len(actions) == 1
    assert isinstance(actions[0], SpawnWorker)
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.STARTING
    assert snap.error_reason is None
    assert snap.restart_count_in_window == 1


def test_restart_budget_full_at_spawn_deadline_latches_error():
    mgr = make_manager(
        restart_window_sec=6.0,
        max_restarts_in_window=2,
    )
    current = start_to_running(mgr, 0.0)
    crash_retryable(mgr, current, 0.0)
    current = spawn_due(mgr, 1.0)
    mgr.on_child_ready(current, 1.0)
    crash_retryable(mgr, current, 1.0)
    current = spawn_due(mgr, 3.0)
    mgr.on_child_ready(current, 3.0)

    crash_retryable(mgr, current, 3.0)
    assert mgr.snapshot.manager_state == ManagerState.BACKOFF
    assert mgr.snapshot.next_restart_at == 7.0
    actions = mgr.tick(7.0)
    assert_no_actions(actions)
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.ERROR
    assert snap.error_reason == ErrorReason.RESTART_BUDGET_EXHAUSTED
    assert snap.active_run_id is None


def test_restart_budget_ages_out_while_waiting_for_mavros():
    mgr = make_manager(
        restart_window_sec=3.0,
        max_restarts_in_window=2,
    )
    current = start_to_running(mgr, 0.0)
    crash_retryable(mgr, current, 0.0)
    current = spawn_due(mgr, 1.0)
    mgr.on_child_ready(current, 1.0)
    crash_retryable(mgr, current, 1.0)
    current = spawn_due(mgr, 3.0)
    mgr.on_child_ready(current, 3.0)
    mgr.set_mavros_ready(False, 3.0)

    crash_retryable(mgr, current, 3.0)
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.WAITING_FOR_MAVROS
    assert snap.error_reason is None
    assert snap.next_restart_at == 7.0
    assert_no_actions(mgr.tick(10.0))

    actions = mgr.set_mavros_ready(True, 10.0)
    assert len(actions) == 1
    assert isinstance(actions[0], SpawnWorker)
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.STARTING
    assert snap.error_reason is None
    assert snap.restart_count_in_window == 1


# ---------------------------------------------------------------------------
# 19-22. Terminal child failures
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    'reason, error',
    [
        (WorkerExitReason.CONFIG_INVALID, ErrorReason.CONFIG_INVALID),
        (WorkerExitReason.OWNERSHIP_CONFLICT, ErrorReason.OWNERSHIP_CONFLICT),
        (WorkerExitReason.AUTH_FAILED, ErrorReason.AUTH_FAILED),
        (WorkerExitReason.MOUNTPOINT_REJECTED, ErrorReason.MOUNTPOINT_REJECTED),
    ],
)
def test_19_to_22_terminal_exit_latches_error_without_retry(reason, error):
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    assert_no_actions(mgr.on_child_exit(run_id, reason, 1.0))
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.ERROR
    assert snap.error_reason == error
    assert snap.active_run_id is None
    assert snap.next_restart_at is None
    assert_no_actions(mgr.tick(2.0))
    assert_no_actions(mgr.set_mavros_ready(True, 2.0))
    assert mgr.snapshot.manager_state == ManagerState.ERROR


# ---------------------------------------------------------------------------
# 23-25. ERROR reset cycle
# ---------------------------------------------------------------------------


def test_23_repeated_request_start_in_error_does_not_restart():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    mgr.on_child_exit(run_id, WorkerExitReason.AUTH_FAILED, 1.0)
    assert mgr.snapshot.manager_state == ManagerState.ERROR
    assert_no_actions(mgr.request_start(2.0))
    assert_no_actions(mgr.request_start(3.0))
    assert_no_actions(mgr.tick(4.0))
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.ERROR
    assert snap.desired_state == DesiredState.RUNNING
    assert snap.error_reason == ErrorReason.AUTH_FAILED
    assert snap.active_run_id is None


def test_24_request_stop_clears_error_when_safe():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    mgr.on_child_exit(run_id, WorkerExitReason.CONFIG_INVALID, 1.0)
    actions = mgr.request_stop(2.0)
    assert_no_actions(actions)
    snap = mgr.snapshot
    assert snap.desired_state == DesiredState.STOPPED
    assert snap.manager_state == ManagerState.STOPPED
    assert snap.error_reason is None
    assert snap.consecutive_failures == 0
    assert snap.restart_count_in_window == 0


def test_25_fresh_start_after_stop_reset_may_start_again():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    mgr.on_child_exit(run_id, WorkerExitReason.OWNERSHIP_CONFLICT, 1.0)
    mgr.request_stop(2.0)
    actions = mgr.request_start(3.0)
    new_id = only_spawn(actions)
    assert new_id != run_id
    assert mgr.snapshot.manager_state == ManagerState.STARTING
    assert mgr.snapshot.error_reason is None


# ---------------------------------------------------------------------------
# 26-28. User stop semantics
# ---------------------------------------------------------------------------


def test_26_request_stop_while_running_emits_one_stop_worker():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    actions = mgr.request_stop(1.0)
    assert only_stop(actions) == run_id
    snap = mgr.snapshot
    assert snap.desired_state == DesiredState.STOPPED
    assert snap.manager_state == ManagerState.STOPPING
    assert snap.active_run_id == run_id


def test_27_repeated_stop_does_not_duplicate_stop_worker():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    assert only_stop(mgr.request_stop(1.0)) == run_id
    assert_no_actions(mgr.request_stop(1.0))
    assert_no_actions(mgr.request_stop(2.0))
    assert_no_actions(mgr.tick(3.0))
    assert mgr.snapshot.manager_state == ManagerState.STOPPING
    assert mgr.snapshot.active_run_id == run_id


def test_28_child_exit_after_manager_stop_is_stopped_no_restart():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    mgr.request_stop(1.0)
    assert_no_actions(
        mgr.on_child_exit(run_id, WorkerExitReason.RETRYABLE_FAILURE, 2.0)
    )
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.STOPPED
    assert snap.desired_state == DesiredState.STOPPED
    assert snap.active_run_id is None
    assert snap.next_restart_at is None
    assert_no_actions(mgr.tick(3.0))
    assert_no_actions(mgr.tick(100.0))


# ---------------------------------------------------------------------------
# 29. Unexpected CLEAN while desired RUNNING uses retry policy
# ---------------------------------------------------------------------------


def test_29_clean_unexpected_exit_while_desired_running_retries():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    assert_no_actions(
        mgr.on_child_exit(run_id, WorkerExitReason.CLEAN, 1.0)
    )
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.BACKOFF
    assert snap.desired_state == DesiredState.RUNNING
    assert snap.next_restart_at == 2.0
    new_id = spawn_due(mgr, 2.0)
    assert new_id != run_id


# ---------------------------------------------------------------------------
# 30-32. MAVROS loss during backoff
# ---------------------------------------------------------------------------


def test_30_mavros_drop_during_backoff_goes_waiting_no_spawn():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    crash_retryable(mgr, run_id, 1.0)
    actions = mgr.set_mavros_ready(False, 1.0)
    assert_no_actions(actions)
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.WAITING_FOR_MAVROS
    assert snap.active_run_id is None
    assert snap.next_restart_at == 2.0


def test_31_backoff_deadline_while_mavros_absent_does_not_spawn():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    crash_retryable(mgr, run_id, 1.0)
    mgr.set_mavros_ready(False, 1.0)
    assert_no_actions(mgr.tick(2.0))
    assert_no_actions(mgr.tick(50.0))
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.WAITING_FOR_MAVROS
    assert snap.active_run_id is None


def test_32_mavros_return_later_emits_one_restart_only():
    mgr = make_manager()
    run_id = start_to_running(mgr, 0.0)
    crash_retryable(mgr, run_id, 1.0)
    mgr.set_mavros_ready(False, 1.0)
    mgr.tick(50.0)
    actions = mgr.set_mavros_ready(True, 50.0)
    new_id = only_spawn(actions)
    assert new_id != run_id
    assert mgr.snapshot.manager_state == ManagerState.STARTING
    assert_no_actions(mgr.set_mavros_ready(True, 50.0))
    assert_no_actions(mgr.tick(50.0))
    assert_no_actions(mgr.request_start(50.0))
    assert mgr.snapshot.active_run_id == new_id


# ---------------------------------------------------------------------------
# 33-36. Startup timeout
# ---------------------------------------------------------------------------


def test_33_startup_timeout_emits_one_stop_worker():
    mgr = make_manager(startup_timeout_sec=10.0)
    mgr.request_start(0.0)
    run_id = only_spawn(mgr.set_mavros_ready(True, 0.0))
    mgr.on_child_started(run_id, 0.0)
    assert_no_actions(mgr.tick(9.999))
    actions = mgr.tick(10.0)
    assert only_stop(actions) == run_id
    assert mgr.snapshot.manager_state == ManagerState.STOPPING
    assert mgr.snapshot.active_run_id == run_id


def test_34_repeated_ticks_after_startup_timeout_do_not_duplicate_stop():
    mgr = make_manager()
    mgr.request_start(0.0)
    run_id = only_spawn(mgr.set_mavros_ready(True, 0.0))
    mgr.on_child_started(run_id, 0.0)
    assert only_stop(mgr.tick(10.0)) == run_id
    assert_no_actions(mgr.tick(10.0))
    assert_no_actions(mgr.tick(11.0))
    assert_no_actions(mgr.tick(20.0))
    assert mgr.snapshot.manager_state == ManagerState.STOPPING


def test_35_no_replacement_child_while_old_child_stopping():
    mgr = make_manager()
    mgr.request_start(0.0)
    run_id = only_spawn(mgr.set_mavros_ready(True, 0.0))
    mgr.on_child_started(run_id, 0.0)
    mgr.tick(10.0)
    assert_no_actions(mgr.tick(11.0))
    assert_no_actions(mgr.request_start(12.0))
    assert_no_actions(mgr.set_mavros_ready(True, 12.0))
    assert mgr.snapshot.active_run_id == run_id
    assert mgr.snapshot.manager_state == ManagerState.STOPPING


def test_36_ready_after_startup_stop_cannot_restore_running():
    mgr = make_manager()
    mgr.request_start(0.0)
    run_id = only_spawn(mgr.set_mavros_ready(True, 0.0))
    mgr.on_child_started(run_id, 0.0)
    mgr.tick(10.0)
    assert_no_actions(mgr.on_child_ready(run_id, 11.0))
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.STOPPING
    assert snap.child_ready is False
    assert snap.active_run_id == run_id


def test_user_stop_upgrades_startup_timeout_stop_intent():
    mgr = make_manager(startup_timeout_sec=10.0)
    assert_no_actions(mgr.request_start(0.0))
    run_a = only_spawn(mgr.set_mavros_ready(True, 0.0))
    assert_no_actions(mgr.on_child_started(run_a, 0.0))

    assert only_stop(mgr.tick(10.0)) == run_a
    assert mgr.snapshot.manager_state == ManagerState.STOPPING

    assert_no_actions(mgr.request_stop(11.0))
    assert mgr.snapshot.desired_state == DesiredState.STOPPED
    assert_no_actions(mgr.request_stop(11.0))
    assert mgr.snapshot.manager_state == ManagerState.STOPPING

    assert_no_actions(mgr.request_start(12.0))
    snap = mgr.snapshot
    assert snap.desired_state == DesiredState.RUNNING
    assert snap.manager_state == ManagerState.STOPPING
    assert snap.active_run_id == run_a

    actions = mgr.on_child_exit(
        run_a, WorkerExitReason.RETRYABLE_FAILURE, 13.0
    )
    run_b = only_spawn(actions)
    assert run_b != run_a
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.STARTING
    assert snap.consecutive_failures == 0
    assert snap.restart_count_in_window == 0


# ---------------------------------------------------------------------------
# 37. Stale event from run A after run B is active
# ---------------------------------------------------------------------------


def test_37_stale_event_from_run_a_after_run_b_active_ignored():
    mgr = make_manager()
    run_a = start_to_running(mgr, 0.0)
    crash_retryable(mgr, run_a, 1.0)
    run_b = spawn_due(mgr, 2.0)
    mgr.on_child_started(run_b, 2.0)
    assert_no_actions(mgr.on_child_ready(run_a, 2.0))
    assert mgr.snapshot.manager_state == ManagerState.STARTING
    assert mgr.snapshot.active_run_id == run_b
    assert mgr.snapshot.child_ready is False
    assert_no_actions(
        mgr.on_child_exit(run_a, WorkerExitReason.RETRYABLE_FAILURE, 2.0)
    )
    assert mgr.snapshot.active_run_id == run_b
    assert_no_actions(mgr.on_child_ready(run_b, 2.0))
    assert mgr.snapshot.manager_state == ManagerState.RUNNING
    assert mgr.snapshot.active_run_id == run_b


def test_run_id_cannot_be_reused_after_an_intervening_run():
    run_ids = iter(('run-A', 'run-B', 'run-A'))
    mgr = make_manager(
        run_id_factory=lambda: next(run_ids),
        max_restarts_in_window=10,
    )
    emitted_run_ids = []
    current = start_to_running(mgr, 0.0)
    emitted_run_ids.append(current)
    crash_retryable(mgr, current, 0.0)
    current = spawn_due(mgr, 1.0)
    emitted_run_ids.append(current)
    mgr.on_child_ready(current, 1.0)
    crash_retryable(mgr, current, 1.0)

    with pytest.raises(RuntimeError, match='reused a run_id'):
        mgr.tick(3.0)

    assert emitted_run_ids == ['run-A', 'run-B']
    assert mgr.snapshot.active_run_id is None


def test_stopped_reset_does_not_allow_run_id_reuse():
    run_ids = iter(('run-A', 'run-A'))
    mgr = make_manager(run_id_factory=lambda: next(run_ids))
    run_id = start_to_running(mgr, 0.0)
    assert run_id == 'run-A'
    assert only_stop(mgr.request_stop(1.0)) == run_id
    assert_no_actions(
        mgr.on_child_exit(run_id, WorkerExitReason.CLEAN, 2.0)
    )
    assert mgr.snapshot.manager_state == ManagerState.STOPPED

    with pytest.raises(RuntimeError, match='reused a run_id'):
        mgr.request_start(3.0)

    assert mgr.snapshot.active_run_id is None
    assert mgr.snapshot.manager_state == ManagerState.STOPPED


# ---------------------------------------------------------------------------
# 38-41. Monotonic injected time
# ---------------------------------------------------------------------------


def test_38_equal_now_sec_accepted():
    mgr = make_manager()
    mgr.request_start(10.0)
    run_id = only_spawn(mgr.set_mavros_ready(True, 10.0))
    mgr.on_child_started(run_id, 10.0)
    mgr.on_child_ready(run_id, 10.0)
    assert_no_actions(mgr.tick(10.0))
    assert mgr.snapshot.manager_state == ManagerState.RUNNING


def test_39_increasing_now_sec_accepted():
    mgr = make_manager()
    mgr.request_start(10.0)
    run_id = only_spawn(mgr.set_mavros_ready(True, 10.5))
    mgr.on_child_started(run_id, 11.0)
    mgr.on_child_ready(run_id, 11.5)
    assert mgr.snapshot.manager_state == ManagerState.RUNNING


def test_40_decreasing_now_sec_rejected_before_mutation():
    mgr = make_manager()
    run_id = start_to_running(mgr, 10.0)
    before = mgr.snapshot
    with pytest.raises(ValueError, match='nondecreasing'):
        mgr.tick(9.9)
    with pytest.raises(ValueError, match='nondecreasing'):
        mgr.request_start(9.0)
    with pytest.raises(ValueError, match='nondecreasing'):
        mgr.set_mavros_ready(False, 9.5)
    with pytest.raises(ValueError, match='nondecreasing'):
        mgr.on_child_ready(run_id, 8.0)
    after = mgr.snapshot
    assert after == before
    assert after.manager_state == ManagerState.RUNNING
    assert after.active_run_id == run_id
    assert_no_actions(mgr.tick(10.0))
    assert mgr.snapshot.manager_state == ManagerState.RUNNING


@pytest.mark.parametrize('now_sec', [math.nan, math.inf, -math.inf])
def test_41_nonfinite_now_sec_rejected(now_sec):
    mgr = make_manager()
    mgr.request_start(0.0)
    before = mgr.snapshot
    with pytest.raises(ValueError, match='finite'):
        mgr.tick(now_sec)
    with pytest.raises(ValueError, match='finite'):
        mgr.request_start(now_sec)
    with pytest.raises(ValueError, match='finite'):
        mgr.set_mavros_ready(True, now_sec)
    assert mgr.snapshot == before


# ---------------------------------------------------------------------------
# 42-43. Immutability
# ---------------------------------------------------------------------------


def test_42_snapshot_is_immutable():
    mgr = make_manager()
    snap = mgr.snapshot
    with pytest.raises(FrozenInstanceError):
        snap.manager_state = ManagerState.RUNNING  # type: ignore[misc]
    with pytest.raises(FrozenInstanceError):
        snap.active_run_id = 'x'  # type: ignore[misc]
    first = mgr.snapshot
    second = mgr.snapshot
    assert first == second
    assert first is not second


def test_43_action_objects_are_immutable():
    mgr = make_manager()
    mgr.request_start(0.0)
    spawn = mgr.set_mavros_ready(True, 0.0)[0]
    assert isinstance(spawn, SpawnWorker)
    with pytest.raises(FrozenInstanceError):
        spawn.run_id = 'mutated'  # type: ignore[misc]
    run_id = spawn.run_id
    mgr.on_child_started(run_id, 0.0)
    mgr.on_child_ready(run_id, 0.0)
    stop = mgr.request_stop(1.0)[0]
    assert isinstance(stop, StopWorker)
    with pytest.raises(FrozenInstanceError):
        stop.run_id = 'mutated'  # type: ignore[misc]


# ---------------------------------------------------------------------------
# 44-45. Randomized invariant / deterministic fuzz
# ---------------------------------------------------------------------------


def _fuzz_once(seed: int, steps: int) -> None:
    rng = random.Random(seed)
    mgr = make_manager(
        run_id_factory=sequential_ids('fuzz'),
        max_restarts_in_window=8,
        startup_timeout_sec=10.0,
    )
    now = 0.0
    live_id = None
    previous_ids = []
    reasons = list(WorkerExitReason)

    def record_actions(actions) -> None:
        nonlocal live_id
        for item in actions:
            if isinstance(item, SpawnWorker):
                assert live_id is None
                live_id = item.run_id
                previous_ids.append(item.run_id)
            elif isinstance(item, StopWorker):
                assert item.run_id == live_id

    for _ in range(steps):
        now += rng.choice([0.0, 0.1, 0.5, 1.0, 5.0, 10.0, 30.0, 61.0])
        kind = rng.randint(0, 10)
        actions = ()
        if kind == 0:
            actions = mgr.request_start(now)
        elif kind == 1:
            actions = mgr.request_stop(now)
        elif kind == 2:
            actions = mgr.set_mavros_ready(True, now)
        elif kind == 3:
            actions = mgr.set_mavros_ready(False, now)
        elif kind == 4:
            actions = mgr.tick(now)
        elif kind == 5:
            target = live_id if live_id is not None else 'stale-none'
            actions = mgr.on_child_started(target, now)
        elif kind == 6:
            target = live_id if live_id is not None else 'stale-none'
            actions = mgr.on_child_ready(target, now)
        elif kind == 7:
            target = live_id if live_id is not None else 'stale-none'
            reason = rng.choice(reasons)
            actions = mgr.on_child_exit(target, reason, now)
            if live_id is not None and target == live_id:
                live_id = None
        elif kind == 8:
            stale = previous_ids[-2] if len(previous_ids) >= 2 else 'run-old'
            actions = mgr.on_child_ready(stale, now)
        elif kind == 9:
            stale = previous_ids[-2] if len(previous_ids) >= 2 else 'run-old'
            actions = mgr.on_child_exit(
                stale, rng.choice(reasons), now
            )
        else:
            actions = mgr.tick(now)
            actions += mgr.request_start(now)
        record_actions(actions)
        snap = mgr.snapshot
        assert_invariants(mgr, actions)
        if snap.active_run_id is None:
            live_id = None
        else:
            if live_id is not None:
                assert snap.active_run_id == live_id
            live_id = snap.active_run_id
        if snap.manager_state == ManagerState.STOPPED:
            assert snap.active_run_id is None
        if snap.manager_state == ManagerState.RUNNING:
            assert snap.active_run_id is not None
            assert snap.child_ready is True
        if snap.manager_state == ManagerState.ERROR:
            assert snap.active_run_id is None
        if snap.desired_state == DesiredState.STOPPED:
            assert snap.manager_state in (
                ManagerState.STOPPED,
                ManagerState.STOPPING,
            )


def test_44_one_active_run_id_invariant_randomized():
    _fuzz_once(seed=20260826, steps=400)


def test_45_deterministic_fuzz_state_invariants():
    for seed in (1, 7, 42, 1337, 20260826):
        _fuzz_once(seed=seed, steps=250)


# ---------------------------------------------------------------------------
# Extra: policy validation and MAVROS-loss-while-starting
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    'kwargs',
    [
        {'initial_backoff_sec': 0.0},
        {'initial_backoff_sec': -1.0},
        {'initial_backoff_sec': math.nan},
        {'backoff_multiplier': 0.5},
        {'max_backoff_sec': 0.5, 'initial_backoff_sec': 1.0},
        {'restart_window_sec': 0.0},
        {'max_restarts_in_window': 0},
        {'stable_run_reset_sec': math.inf},
        {'startup_timeout_sec': -1.0},
    ],
)
def test_policy_rejects_nonsensical_values(kwargs):
    with pytest.raises((TypeError, ValueError)):
        RestartPolicy(**kwargs)


def test_mavros_loss_while_starting_then_ready_is_stale():
    mgr = make_manager()
    mgr.request_start(0.0)
    run_id = only_spawn(mgr.set_mavros_ready(True, 0.0))
    mgr.on_child_started(run_id, 0.0)
    assert_no_actions(mgr.set_mavros_ready(False, 1.0))
    assert mgr.snapshot.manager_state == ManagerState.RUNNING_MAVROS_STALE
    assert_no_actions(mgr.on_child_ready(run_id, 2.0))
    snap = mgr.snapshot
    assert snap.manager_state == ManagerState.RUNNING_MAVROS_STALE
    assert snap.child_ready is True
    assert snap.active_run_id == run_id
    assert_no_actions(mgr.set_mavros_ready(True, 3.0))
    assert mgr.snapshot.manager_state == ManagerState.RUNNING


def test_startup_timeout_exit_then_retry_when_desired_running():
    mgr = make_manager()
    mgr.request_start(0.0)
    run_id = only_spawn(mgr.set_mavros_ready(True, 0.0))
    mgr.on_child_started(run_id, 0.0)
    mgr.tick(10.0)
    assert_no_actions(
        mgr.on_child_exit(run_id, WorkerExitReason.RETRYABLE_FAILURE, 11.0)
    )
    assert mgr.snapshot.manager_state == ManagerState.BACKOFF
    assert mgr.snapshot.desired_state == DesiredState.RUNNING
    new_id = spawn_due(mgr, mgr.snapshot.next_restart_at)
    assert new_id != run_id
