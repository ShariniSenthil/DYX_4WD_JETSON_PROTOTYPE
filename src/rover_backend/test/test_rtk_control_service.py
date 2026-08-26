"""Tests for backend RTK persistence/runtime authority."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from rover_backend.rtk_control_service import (
    RtkControlRuntimeError,
    RtkControlService,
)
from rover_backend.rtk_manager_core import (
    DesiredState,
)
from rover_backend.rtk_profile_store import (
    RtkProfileStore,
)


SECRET = "CONTROL_SECRET_9271"


class FakeRuntime:
    def __init__(self):
        self.start_calls = 0
        self.stop_calls = 0
        self.fail_start = False
        self.fail_stop = False

        self._snapshot = SimpleNamespace(
            running=True,
            shutdown_requested=False,
            owner_thread_id=123,
            mavros_ready=False,
            last_error_code=None,
            runtime=None,
        )

    @property
    def snapshot(self):
        return self._snapshot

    def request_start(self):
        self.start_calls += 1

        if self.fail_start:
            raise RuntimeError(
                "synthetic start failure"
            )

    def request_stop(self):
        self.stop_calls += 1

        if self.fail_stop:
            raise RuntimeError(
                "synthetic stop failure"
            )


@pytest.fixture
def authority(
    tmp_path: Path,
):
    store = RtkProfileStore(
        tmp_path / "rtk.sqlite3"
    )

    store.initialize()

    runtime = FakeRuntime()

    control = RtkControlService(
        store,
        runtime,
    )

    return control, store, runtime


def create_profile(
    control: RtkControlService,
    *,
    name="Base One",
    password=SECRET,
):
    return control.create_profile(
        name=name,
        caster_host="caster.test",
        caster_port=2101,
        mountpoint="MOUNT",
        username="rover",
        password=password,
    )


def activate(
    control,
):
    profile = create_profile(
        control
    )

    control.activate_profile(
        profile.profile_id
    )

    return profile


def test_initial_snapshot_is_stopped_and_secret_free(
    authority,
):
    control, _, runtime = authority

    snapshot = control.snapshot

    assert (
        snapshot.persisted.desired_state
        is DesiredState.STOPPED
    )

    assert snapshot.active_profile is None
    assert snapshot.runtime is runtime.snapshot

    assert SECRET not in repr(snapshot)


def test_create_list_and_get_are_secret_free(
    authority,
):
    control, _, _ = authority

    profile = create_profile(
        control
    )

    assert SECRET not in repr(profile)
    assert SECRET not in repr(
        control.list_profiles()
    )
    assert SECRET not in repr(
        control.get_profile(
            profile.profile_id
        )
    )


def test_start_persists_then_forwards_runtime(
    authority,
):
    control, store, runtime = authority

    activate(control)

    state = control.request_start()

    assert (
        state.desired_state
        is DesiredState.RUNNING
    )

    assert (
        store.runtime_state().desired_state
        is DesiredState.RUNNING
    )

    assert runtime.start_calls == 1


def test_repeated_start_reconciles_without_db_revision_change(
    authority,
):
    control, store, runtime = authority

    activate(control)

    first = control.request_start()

    second = control.request_start()

    assert second.revision == first.revision
    assert runtime.start_calls == 2


def test_failed_start_compensates_persistence_to_stopped(
    authority,
):
    control, store, runtime = authority

    activate(control)

    runtime.fail_start = True

    with pytest.raises(
        RtkControlRuntimeError
    ):
        control.request_start()

    assert (
        store.runtime_state().desired_state
        is DesiredState.STOPPED
    )

    assert runtime.start_calls == 1
    assert runtime.stop_calls == 1


def test_stop_persists_stopped_even_if_runtime_forward_fails(
    authority,
):
    control, store, runtime = authority

    activate(control)
    control.request_start()

    runtime.fail_stop = True

    with pytest.raises(
        RtkControlRuntimeError
    ):
        control.request_stop()

    assert (
        store.runtime_state().desired_state
        is DesiredState.STOPPED
    )


def test_active_runtime_edit_forwards_stop(
    authority,
):
    control, store, runtime = authority

    profile = activate(control)
    control.request_start()

    control.update_profile(
        profile.profile_id,
        caster_host="new-caster.test",
    )

    assert (
        store.runtime_state().desired_state
        is DesiredState.STOPPED
    )

    assert runtime.stop_calls == 1


def test_active_name_only_edit_does_not_stop_runtime(
    authority,
):
    control, store, runtime = authority

    profile = activate(control)
    control.request_start()

    control.update_profile(
        profile.profile_id,
        name="Renamed Base",
    )

    assert (
        store.runtime_state().desired_state
        is DesiredState.RUNNING
    )

    assert runtime.stop_calls == 0


def test_switch_active_profile_while_running_forwards_stop(
    authority,
):
    control, store, runtime = authority

    first = activate(control)

    second = create_profile(
        control,
        name="Base Two",
        password="SECOND_SECRET",
    )

    control.request_start()

    state = control.activate_profile(
        second.profile_id
    )

    assert (
        state.active_profile_id
        == second.profile_id
    )
    assert (
        state.desired_state
        is DesiredState.STOPPED
    )

    assert runtime.stop_calls == 1

    assert (
        store.get_profile(
            first.profile_id
        ).profile_id
        == first.profile_id
    )


def test_delete_active_profile_while_running_forwards_stop(
    authority,
):
    control, store, runtime = authority

    profile = activate(control)
    control.request_start()

    control.delete_profile(
        profile.profile_id
    )

    state = store.runtime_state()

    assert state.active_profile_id is None
    assert (
        state.desired_state
        is DesiredState.STOPPED
    )

    assert runtime.stop_calls == 1


def test_clear_active_profile_while_running_forwards_stop(
    authority,
):
    control, store, runtime = authority

    activate(control)
    control.request_start()

    state = control.clear_active_profile()

    assert state.active_profile_id is None
    assert (
        state.desired_state
        is DesiredState.STOPPED
    )

    assert runtime.stop_calls == 1


def test_disabling_active_profile_while_running_forwards_stop(
    authority,
):
    control, store, runtime = authority

    profile = activate(control)
    control.request_start()

    control.update_profile(
        profile.profile_id,
        enabled=False,
    )

    state = store.runtime_state()

    assert state.active_profile_id is None
    assert (
        state.desired_state
        is DesiredState.STOPPED
    )

    assert runtime.stop_calls == 1


def test_reconcile_running_forwards_start(
    authority,
):
    control, store, runtime = authority

    activate(control)

    store.set_desired_state(
        DesiredState.RUNNING
    )

    state = control.reconcile_runtime()

    assert (
        state.desired_state
        is DesiredState.RUNNING
    )
    assert runtime.start_calls == 1


def test_reconcile_stopped_forwards_stop(
    authority,
):
    control, _, runtime = authority

    state = control.reconcile_runtime()

    assert (
        state.desired_state
        is DesiredState.STOPPED
    )
    assert runtime.stop_calls == 1


def test_snapshot_contains_active_profile_and_runtime(
    authority,
):
    control, _, runtime = authority

    profile = activate(control)

    snapshot = control.snapshot

    assert (
        snapshot.active_profile.profile_id
        == profile.profile_id
    )

    assert snapshot.runtime is runtime.snapshot
    assert SECRET not in repr(snapshot)


def test_failed_repeated_start_compensates_persistence_to_stopped(
    authority,
):
    """A failed explicit repeated START must fail closed to STOPPED."""

    control, store, runtime = authority

    activate(control)

    first = control.request_start()

    assert (
        first.desired_state
        is DesiredState.RUNNING
    )

    assert runtime.start_calls == 1

    runtime.fail_start = True

    with pytest.raises(
        RtkControlRuntimeError
    ):
        control.request_start()

    state = store.runtime_state()

    assert (
        state.desired_state
        is DesiredState.STOPPED
    )

    assert runtime.start_calls == 2

    # Ambiguous START acceptance is compensated with best-effort STOP.
    assert runtime.stop_calls == 1
