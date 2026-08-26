"""Tests for persistent RTK profiles and desired-state ownership."""

from __future__ import annotations

import os
import sqlite3

from pathlib import Path

import pytest

from rover_backend.rtk_manager_core import (
    DesiredState,
)
from rover_backend.rtk_profile_store import (
    RTK_PROFILE_SCHEMA_VERSION,
    RtkProfileConflictError,
    RtkProfileNotFoundError,
    RtkProfileStateError,
    RtkProfileStore,
    RtkProfileStoreError,
    RtkProfileValidationError,
)


SECRET = "RTK_SECRET_PASSWORD_4937"


class FakeClock:
    def __init__(
        self,
        value: float = 1_800_000_000.0,
    ) -> None:
        self.value = value

    def __call__(
        self,
    ) -> float:
        return self.value

    def advance(
        self,
        seconds: float = 1.0,
    ) -> None:
        self.value += seconds


@pytest.fixture
def store(
    tmp_path: Path,
):
    clock = FakeClock()

    value = RtkProfileStore(
        tmp_path / "rtk" / "rtk.sqlite3",
        clock=clock,
    )

    value.initialize()

    return value, clock


def create_profile(
    store: RtkProfileStore,
    *,
    name: str = "Office Base",
    password: str = SECRET,
    enabled: bool = True,
):
    return store.create_profile(
        name=name,
        caster_host="caster.example.test",
        caster_port=2101,
        mountpoint="DYX_RTCM3",
        username="dyx-rover",
        password=password,
        rtcm_topic=(
            "/mavros/gps_rtk/send_rtcm"
        ),
        connect_timeout_sec=10.0,
        socket_timeout_sec=1.0,
        healthy_age_sec=5.0,
        stale_reconnect_sec=10.0,
        reconnect_delay_sec=5.0,
        first_data_timeout_sec=10.0,
        max_mavros_rtcm_frame_bytes=720,
        enabled=enabled,
    )


def test_initialize_creates_stopped_runtime_state(
    store,
):
    value, _ = store

    runtime = value.runtime_state()

    assert runtime.active_profile_id is None

    assert (
        runtime.desired_state
        is DesiredState.STOPPED
    )

    assert runtime.revision == 1


def test_initialize_sets_schema_version(
    store,
):
    value, _ = store

    connection = sqlite3.connect(
        value.database_file
    )

    try:
        version = connection.execute(
            "PRAGMA user_version"
        ).fetchone()[0]
    finally:
        connection.close()

    assert version == RTK_PROFILE_SCHEMA_VERSION


def test_database_file_is_private_on_posix(
    store,
):
    value, _ = store

    if os.name != "posix":
        pytest.skip(
            "POSIX permission assertion"
        )

    mode = (
        value.database_file.stat().st_mode
        & 0o777
    )

    assert mode == 0o600


def test_create_profile_returns_redacted_snapshot(
    store,
):
    value, _ = store

    profile = create_profile(
        value
    )

    assert profile.profile_id > 0
    assert profile.password_configured is True

    assert SECRET not in repr(
        profile
    )

    assert not hasattr(
        profile,
        "password"
    )

    assert not hasattr(
        profile,
        "password_secret"
    )


def test_profile_secret_is_stored_but_not_listed(
    store,
):
    value, _ = store

    create_profile(
        value
    )

    profiles = value.list_profiles()

    assert len(profiles) == 1

    assert SECRET not in repr(
        profiles
    )


def test_duplicate_profile_name_is_case_insensitive(
    store,
):
    value, _ = store

    create_profile(
        value,
        name="Office Base",
    )

    with pytest.raises(
        RtkProfileConflictError
    ):
        create_profile(
            value,
            name="office base",
        )


def test_invalid_port_is_rejected(
    store,
):
    value, _ = store

    with pytest.raises(
        RtkProfileValidationError
    ):
        value.create_profile(
            name="Bad",
            caster_host="caster.test",
            caster_port=0,
            mountpoint="MOUNT",
            username="user",
            password=SECRET,
        )


def test_relative_rtcm_topic_is_rejected(
    store,
):
    value, _ = store

    with pytest.raises(
        RtkProfileValidationError
    ):
        value.create_profile(
            name="Bad",
            caster_host="caster.test",
            caster_port=2101,
            mountpoint="MOUNT",
            username="user",
            password=SECRET,
            rtcm_topic="relative/topic",
        )


def test_stale_reconnect_must_exceed_healthy_age(
    store,
):
    value, _ = store

    with pytest.raises(
        RtkProfileValidationError
    ):
        value.create_profile(
            name="Bad",
            caster_host="caster.test",
            caster_port=2101,
            mountpoint="MOUNT",
            username="user",
            password=SECRET,
            healthy_age_sec=5.0,
            stale_reconnect_sec=5.0,
        )


def test_mountpoint_leading_slash_is_normalised(
    store,
):
    value, _ = store

    profile = value.create_profile(
        name="Slash",
        caster_host="caster.test",
        caster_port=2101,
        mountpoint="/MOUNT",
        username="user",
        password=SECRET,
    )

    assert profile.mountpoint == "MOUNT"


def test_get_missing_profile_rejected(
    store,
):
    value, _ = store

    with pytest.raises(
        RtkProfileNotFoundError
    ):
        value.get_profile(
            999
        )


def test_profile_update_increments_revision(
    store,
):
    value, clock = store

    profile = create_profile(
        value
    )

    clock.advance()

    updated = value.update_profile(
        profile.profile_id,
        caster_host="new-caster.test",
    )

    assert (
        updated.revision
        == profile.revision + 1
    )

    assert (
        updated.updated_at_epoch
        > profile.updated_at_epoch
    )


def test_noop_update_preserves_revision(
    store,
):
    value, _ = store

    profile = create_profile(
        value
    )

    updated = value.update_profile(
        profile.profile_id,
    )

    assert (
        updated.revision
        == profile.revision
    )


def test_update_without_password_preserves_secret(
    store,
):
    value, _ = store

    profile = create_profile(
        value
    )

    value.set_active_profile(
        profile.profile_id
    )

    value.update_profile(
        profile.profile_id,
        caster_host="changed.test",
    )

    config = (
        value.build_active_worker_config(
            "run-preserve"
        )
    )

    assert config.password == SECRET


def test_password_update_replaces_secret_without_snapshot_leak(
    store,
):
    value, _ = store

    profile = create_profile(
        value
    )

    value.set_active_profile(
        profile.profile_id
    )

    updated = value.update_profile(
        profile.profile_id,
        password="NEW_SECRET_8472",
    )

    assert (
        "NEW_SECRET_8472"
        not in repr(updated)
    )

    config = (
        value.build_active_worker_config(
            "run-new-secret"
        )
    )

    assert (
        config.password
        == "NEW_SECRET_8472"
    )


def test_running_requires_active_profile(
    store,
):
    value, _ = store

    with pytest.raises(
        RtkProfileStateError
    ):
        value.set_desired_state(
            DesiredState.RUNNING
        )


def test_disabled_profile_cannot_be_activated(
    store,
):
    value, _ = store

    profile = create_profile(
        value,
        enabled=False,
    )

    with pytest.raises(
        RtkProfileStateError
    ):
        value.set_active_profile(
            profile.profile_id
        )


def test_activate_profile_is_fail_closed(
    store,
):
    value, _ = store

    first = create_profile(
        value,
        name="First",
    )

    second = create_profile(
        value,
        name="Second",
        password="SECOND_SECRET",
    )

    state = value.set_active_profile(
        first.profile_id
    )

    assert (
        state.desired_state
        is DesiredState.STOPPED
    )

    value.set_desired_state(
        DesiredState.RUNNING
    )

    state = value.set_active_profile(
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


def test_same_active_profile_is_idempotent(
    store,
):
    value, _ = store

    profile = create_profile(
        value
    )

    first = value.set_active_profile(
        profile.profile_id
    )

    second = value.set_active_profile(
        profile.profile_id
    )

    assert (
        second.revision
        == first.revision
    )


def test_desired_state_is_idempotent(
    store,
):
    value, _ = store

    profile = create_profile(
        value
    )

    value.set_active_profile(
        profile.profile_id
    )

    first = value.set_desired_state(
        DesiredState.RUNNING
    )

    second = value.set_desired_state(
        "RUNNING"
    )

    assert (
        second.revision
        == first.revision
    )


def test_runtime_relevant_active_edit_forces_stopped(
    store,
):
    value, _ = store

    profile = create_profile(
        value
    )

    value.set_active_profile(
        profile.profile_id
    )

    value.set_desired_state(
        DesiredState.RUNNING
    )

    value.update_profile(
        profile.profile_id,
        caster_host="changed.test",
    )

    runtime = value.runtime_state()

    assert (
        runtime.active_profile_id
        == profile.profile_id
    )

    assert (
        runtime.desired_state
        is DesiredState.STOPPED
    )


def test_active_name_only_edit_does_not_force_stop(
    store,
):
    value, _ = store

    profile = create_profile(
        value
    )

    value.set_active_profile(
        profile.profile_id
    )

    value.set_desired_state(
        DesiredState.RUNNING
    )

    value.update_profile(
        profile.profile_id,
        name="Renamed Base",
    )

    assert (
        value.runtime_state().desired_state
        is DesiredState.RUNNING
    )


def test_disabling_active_profile_clears_and_stops(
    store,
):
    value, _ = store

    profile = create_profile(
        value
    )

    value.set_active_profile(
        profile.profile_id
    )

    value.set_desired_state(
        DesiredState.RUNNING
    )

    value.update_profile(
        profile.profile_id,
        enabled=False,
    )

    runtime = value.runtime_state()

    assert runtime.active_profile_id is None

    assert (
        runtime.desired_state
        is DesiredState.STOPPED
    )


def test_delete_active_profile_clears_and_stops(
    store,
):
    value, _ = store

    profile = create_profile(
        value
    )

    value.set_active_profile(
        profile.profile_id
    )

    value.set_desired_state(
        DesiredState.RUNNING
    )

    value.delete_profile(
        profile.profile_id
    )

    runtime = value.runtime_state()

    assert runtime.active_profile_id is None

    assert (
        runtime.desired_state
        is DesiredState.STOPPED
    )

    with pytest.raises(
        RtkProfileNotFoundError
    ):
        value.get_profile(
            profile.profile_id
        )


def test_delete_inactive_profile_does_not_change_runtime(
    store,
):
    value, _ = store

    first = create_profile(
        value,
        name="First",
    )

    second = create_profile(
        value,
        name="Second",
        password="SECOND_SECRET",
    )

    value.set_active_profile(
        first.profile_id
    )

    value.set_desired_state(
        DesiredState.RUNNING
    )

    before = value.runtime_state()

    value.delete_profile(
        second.profile_id
    )

    after = value.runtime_state()

    assert after == before


def test_clear_active_profile_forces_stopped(
    store,
):
    value, _ = store

    profile = create_profile(
        value
    )

    value.set_active_profile(
        profile.profile_id
    )

    value.set_desired_state(
        DesiredState.RUNNING
    )

    runtime = (
        value.clear_active_profile()
    )

    assert runtime.active_profile_id is None

    assert (
        runtime.desired_state
        is DesiredState.STOPPED
    )


def test_build_worker_config_uses_active_profile(
    store,
):
    value, _ = store

    profile = create_profile(
        value
    )

    value.set_active_profile(
        profile.profile_id
    )

    config = (
        value.build_active_worker_config(
            "run-123"
        )
    )

    assert config.run_id == "run-123"
    assert config.caster_host == (
        "caster.example.test"
    )
    assert config.password == SECRET
    assert (
        config.max_mavros_rtcm_frame_bytes
        == 720
    )

    assert SECRET not in repr(
        config
    )


def test_build_worker_config_without_active_rejected(
    store,
):
    value, _ = store

    create_profile(
        value
    )

    with pytest.raises(
        RtkProfileStateError
    ):
        value.build_active_worker_config(
            "run-none"
        )


def test_runtime_and_profiles_persist_across_store_instances(
    store,
):
    value, clock = store

    profile = create_profile(
        value
    )

    value.set_active_profile(
        profile.profile_id
    )

    value.set_desired_state(
        DesiredState.RUNNING
    )

    reopened = RtkProfileStore(
        value.database_file,
        clock=clock,
    )

    reopened.initialize()

    restored_profile = (
        reopened.get_profile(
            profile.profile_id
        )
    )

    restored_runtime = (
        reopened.runtime_state()
    )

    assert (
        restored_profile.name
        == profile.name
    )

    assert (
        restored_runtime.active_profile_id
        == profile.profile_id
    )

    assert (
        restored_runtime.desired_state
        is DesiredState.RUNNING
    )


def test_unsupported_schema_version_is_rejected(
    tmp_path: Path,
):
    path = (
        tmp_path
        / "rtk"
        / "rtk.sqlite3"
    )

    path.parent.mkdir(
        parents=True
    )

    connection = sqlite3.connect(
        path
    )

    try:
        connection.execute(
            "PRAGMA user_version = 999"
        )
        connection.commit()
    finally:
        connection.close()

    value = RtkProfileStore(
        path
    )

    with pytest.raises(
        RtkProfileStoreError
    ):
        value.initialize()
