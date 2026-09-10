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


def test_password_whitespace_is_preserved_exactly(
    store,
):
    value, _ = store

    secret = (
        " secret-with-significant-spaces "
    )

    profile = create_profile(
        value,
        password=secret,
    )

    value.set_active_profile(
        profile.profile_id
    )

    config = (
        value.build_active_worker_config(
            "run-secret-spaces"
        )
    )

    assert config.password == secret


@pytest.mark.parametrize(
    "password",
    (
        "bad\nsecret",
        "bad\rsecret",
        "bad\tsecret",
        "bad\x00secret",
        "bad\x7fsecret",
    ),
)
def test_password_control_characters_are_rejected(
    store,
    password,
):
    value, _ = store

    with pytest.raises(
        RtkProfileValidationError
    ):
        create_profile(
            value,
            password=password,
        )


@pytest.mark.parametrize(
    (
        "field_name",
        "field_value",
    ),
    (
        (
            "caster_host",
            "caster.test\r\nInjected: yes",
        ),
        (
            "caster_host",
            "caster test",
        ),
        (
            "mountpoint",
            "MOUNT\r\nInjected: yes",
        ),
        (
            "mountpoint",
            "MOUNT POINT",
        ),
        (
            "rtcm_topic",
            "/mavros/rtcm\nother",
        ),
        (
            "rtcm_topic",
            "/mavros/rtcm topic",
        ),
    ),
)
def test_protocol_tokens_reject_controls_and_whitespace(
    store,
    field_name,
    field_value,
):
    value, _ = store

    kwargs = {
        "name": "Bad Protocol",
        "caster_host": "caster.test",
        "caster_port": 2101,
        "mountpoint": "MOUNT",
        "username": "user",
        "password": SECRET,
        "rtcm_topic": (
            "/mavros/gps_rtk/send_rtcm"
        ),
    }

    kwargs[field_name] = field_value

    with pytest.raises(
        RtkProfileValidationError
    ):
        value.create_profile(
            **kwargs
        )


def test_username_control_characters_are_rejected(
    store,
):
    value, _ = store

    with pytest.raises(
        RtkProfileValidationError
    ):
        value.create_profile(
            name="Bad Username",
            caster_host="caster.test",
            caster_port=2101,
            mountpoint="MOUNT",
            username="user\r\nX: injected",
            password=SECRET,
        )


def test_password_update_preserves_significant_spaces(
    store,
):
    value, _ = store

    profile = create_profile(
        value
    )

    value.set_active_profile(
        profile.profile_id
    )

    replacement = " new secret "

    value.update_profile(
        profile.profile_id,
        password=replacement,
    )

    config = (
        value.build_active_worker_config(
            "run-updated-secret"
        )
    )

    assert config.password == replacement


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


@pytest.mark.parametrize(
    ("field_name", "field_value"),
    (
        ("caster_host", "cástér.test"),
        ("mountpoint", "MÖUNT"),
        ("rtcm_topic", "/mavros/rtçm"),
    ),
)
def test_profile_protocol_tokens_reject_non_ascii(
    store,
    field_name,
    field_value,
):
    value, _ = store

    kwargs = {
        "name": "ASCII protocol test",
        "caster_host": "caster.test",
        "caster_port": 443,
        "mountpoint": "MOUNT",
        "username": "rover",
        "password": SECRET,
        "rtcm_topic": (
            "/mavros/gps_rtk/send_rtcm"
        ),
    }

    kwargs[field_name] = field_value

    with pytest.raises(
        RtkProfileValidationError,
        match="ASCII",
    ):
        value.create_profile(
            **kwargs
        )


# ---------------------------------------------------------------------------
# Schema v4 — direct RTCM injection persistence
# ---------------------------------------------------------------------------


def test_direct_injection_profile_defaults_are_legacy(
    store,
):
    value, _ = store

    profile = create_profile(value)

    assert profile.direct_inject is False
    assert profile.direct_serial_device is None
    assert profile.direct_serial_baud == 230400
    assert (
        profile.direct_serial_write_timeout_sec
        == 1.0
    )
    assert (
        profile.direct_serial_reopen_sec
        == 1.0
    )


def test_direct_injection_settings_persist_to_worker_config(
    store,
):
    value, _ = store

    profile = value.create_profile(
        name="Direct USB",
        caster_host="caster.test",
        caster_port=2101,
        mountpoint="MOUNT",
        username="rover",
        password=SECRET,
        direct_inject=True,
        direct_serial_device=(
            "/dev/serial/by-id/"
            "usb-Septentrio_mosaic-H-test"
        ),
        direct_serial_baud=230400,
        direct_serial_write_timeout_sec=0.5,
        direct_serial_reopen_sec=2.0,
    )

    value.set_active_profile(
        profile.profile_id
    )

    config = value.build_active_worker_config(
        "run-direct"
    )

    assert config.direct_inject is True
    assert (
        config.direct_serial_device
        == profile.direct_serial_device
    )
    assert config.direct_serial_baud == 230400
    assert (
        config.direct_serial_write_timeout_sec
        == 0.5
    )
    assert (
        config.direct_serial_reopen_sec
        == 2.0
    )


def test_direct_serial_device_can_be_explicitly_cleared(
    store,
):
    value, _ = store

    profile = value.create_profile(
        name="Configured Direct Device",
        caster_host="caster.test",
        caster_port=2101,
        mountpoint="MOUNT",
        username="rover",
        password=SECRET,
        direct_serial_device=(
            "/dev/serial/by-id/"
            "usb-Septentrio_mosaic-H-test"
        ),
    )

    updated = value.update_profile(
        profile.profile_id,
        direct_serial_device=None,
    )

    assert updated.direct_serial_device is None


def test_omitted_direct_serial_device_preserves_value(
    store,
):
    value, _ = store

    device = (
        "/dev/serial/by-id/"
        "usb-Septentrio_mosaic-H-test"
    )

    profile = value.create_profile(
        name="Preserve Direct Device",
        caster_host="caster.test",
        caster_port=2101,
        mountpoint="MOUNT",
        username="rover",
        password=SECRET,
        direct_serial_device=device,
    )

    updated = value.update_profile(
        profile.profile_id,
        direct_serial_baud=460800,
    )

    assert updated.direct_serial_device == device
    assert updated.direct_serial_baud == 460800


def test_direct_mode_rejects_device_clear(
    store,
):
    value, _ = store

    profile = value.create_profile(
        name="Direct Active Config",
        caster_host="caster.test",
        caster_port=2101,
        mountpoint="MOUNT",
        username="rover",
        password=SECRET,
        direct_inject=True,
        direct_serial_device=(
            "/dev/serial/by-id/"
            "usb-Septentrio_mosaic-H-test"
        ),
    )

    with pytest.raises(
        RtkProfileValidationError,
        match="direct_serial_device is required",
    ):
        value.update_profile(
            profile.profile_id,
            direct_serial_device=None,
        )


def test_schema_v3_database_migrates_to_v4(
    tmp_path: Path,
):
    """An existing production-style v3 DB must open and default to A-side."""

    path = tmp_path / "rtk" / "rtk.sqlite3"
    path.parent.mkdir(parents=True)

    connection = sqlite3.connect(path)

    try:
        connection.executescript(
            """
            CREATE TABLE rtk_profiles (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL COLLATE NOCASE UNIQUE,
                caster_host TEXT NOT NULL,
                caster_port INTEGER NOT NULL,
                mountpoint TEXT NOT NULL,
                username TEXT NOT NULL,
                password_secret TEXT NOT NULL,
                rtcm_topic TEXT NOT NULL,
                connect_timeout_sec REAL NOT NULL,
                socket_timeout_sec REAL NOT NULL,
                healthy_age_sec REAL NOT NULL,
                stale_reconnect_sec REAL NOT NULL,
                reconnect_delay_sec REAL NOT NULL,
                first_data_timeout_sec REAL NOT NULL,
                gga_enabled INTEGER NOT NULL DEFAULT 0
                    CHECK(gga_enabled IN (0, 1)),
                gga_interval_sec REAL NOT NULL DEFAULT 10.0,
                gga_max_age_sec REAL NOT NULL DEFAULT 5.0,
                tls_mode TEXT NOT NULL DEFAULT 'REQUIRED'
                    CHECK(tls_mode IN ('REQUIRED', 'DISABLED')),
                max_mavros_rtcm_frame_bytes INTEGER NOT NULL,
                enabled INTEGER NOT NULL
                    CHECK(enabled IN (0, 1)),
                revision INTEGER NOT NULL
                    CHECK(revision >= 1),
                created_at INTEGER NOT NULL,
                updated_at INTEGER NOT NULL
            );

            CREATE TABLE rtk_runtime_state (
                singleton_id INTEGER PRIMARY KEY
                    CHECK(singleton_id = 1),
                active_profile_id INTEGER,
                desired_state TEXT NOT NULL
                    CHECK(desired_state IN ('STOPPED', 'RUNNING')),
                revision INTEGER NOT NULL
                    CHECK(revision >= 1),
                updated_at INTEGER NOT NULL,
                FOREIGN KEY(active_profile_id)
                    REFERENCES rtk_profiles(id)
                    ON DELETE SET NULL
            );

            INSERT INTO rtk_profiles (
                name,
                caster_host,
                caster_port,
                mountpoint,
                username,
                password_secret,
                rtcm_topic,
                connect_timeout_sec,
                socket_timeout_sec,
                healthy_age_sec,
                stale_reconnect_sec,
                reconnect_delay_sec,
                first_data_timeout_sec,
                gga_enabled,
                gga_interval_sec,
                gga_max_age_sec,
                tls_mode,
                max_mavros_rtcm_frame_bytes,
                enabled,
                revision,
                created_at,
                updated_at
            )
            VALUES (
                'Existing Rover',
                'caster.test',
                2101,
                'MOUNT',
                'rover',
                'existing-secret',
                '/mavros/gps_rtk/send_rtcm',
                10.0,
                1.0,
                5.0,
                10.0,
                5.0,
                10.0,
                0,
                10.0,
                5.0,
                'REQUIRED',
                720,
                1,
                1,
                1800000000,
                1800000000
            );

            INSERT INTO rtk_runtime_state (
                singleton_id,
                active_profile_id,
                desired_state,
                revision,
                updated_at
            )
            VALUES (
                1,
                1,
                'STOPPED',
                1,
                1800000000
            );

            PRAGMA user_version = 3;
            """
        )

        connection.commit()

    finally:
        connection.close()

    value = RtkProfileStore(path)

    value.initialize()

    connection = sqlite3.connect(path)

    try:
        version = connection.execute(
            "PRAGMA user_version"
        ).fetchone()[0]

        columns = {
            row[1]
            for row in connection.execute(
                "PRAGMA table_info(rtk_profiles)"
            ).fetchall()
        }

    finally:
        connection.close()

    assert version == 4

    assert {
        "direct_inject",
        "direct_serial_device",
        "direct_serial_baud",
        "direct_serial_write_timeout_sec",
        "direct_serial_reopen_sec",
    } <= columns

    profile = value.get_profile(1)

    assert profile.name == "Existing Rover"
    assert profile.direct_inject is False
    assert profile.direct_serial_device is None
    assert profile.direct_serial_baud == 230400
    assert (
        profile.direct_serial_write_timeout_sec
        == 1.0
    )
    assert profile.direct_serial_reopen_sec == 1.0

    config = value.build_active_worker_config(
        "migration-check"
    )

    assert config.direct_inject is False
    assert config.direct_serial_device is None
