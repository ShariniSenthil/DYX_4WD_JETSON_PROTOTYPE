"""GGA/VRS persistent-profile and worker-config contract tests."""

from __future__ import annotations

import sqlite3

from pathlib import Path

from rover_backend.rtk_manager_core import (
    DesiredState,
)
from rover_backend.rtk_process_protocol import (
    WORKER_CONFIG_SCHEMA_VERSION,
    WorkerConfig,
    decode_worker_config,
    encode_worker_config,
)
from rover_backend.rtk_profile_store import (
    RTK_PROFILE_SCHEMA_VERSION,
    RtkProfileStore,
)


SECRET = "GGA_PROFILE_SECRET_1842"


def make_worker_config(
    **changes,
):
    values = {
        "schema_version": (
            WORKER_CONFIG_SCHEMA_VERSION
        ),
        "run_id": "gga-contract",
        "caster_host": "caster.test",
        "caster_port": 2101,
        "mountpoint": "MOUNT",
        "username": "rover",
        "password": SECRET,
        "rtcm_topic": (
            "/mavros/gps_rtk/send_rtcm"
        ),
        "connect_timeout_sec": 10.0,
        "socket_timeout_sec": 1.0,
        "healthy_age_sec": 5.0,
        "stale_reconnect_sec": 10.0,
        "reconnect_delay_sec": 5.0,
        "first_data_timeout_sec": 10.0,
        "gga_enabled": True,
        "gga_interval_sec": 10.0,
        "gga_max_age_sec": 5.0,
        "max_mavros_rtcm_frame_bytes": 720,
    }

    values.update(
        changes
    )

    return WorkerConfig(
        **values
    )


def test_worker_config_v3_round_trip_preserves_gga_policy():
    config = make_worker_config()

    decoded = decode_worker_config(
        encode_worker_config(
            config
        )
    )

    assert (
        decoded.schema_version
        == 3
    )

    assert decoded.gga_enabled is True
    assert decoded.gga_interval_sec == 10.0
    assert decoded.gga_max_age_sec == 5.0

    assert SECRET not in repr(decoded)


def test_new_profile_defaults_to_gga_disabled(
    tmp_path: Path,
):
    store = RtkProfileStore(
        tmp_path / "fresh.sqlite3"
    )

    store.initialize()

    profile = store.create_profile(
        name="Fixed Base",
        caster_host="caster.test",
        caster_port=2101,
        mountpoint="FIXED",
        username="rover",
        password=SECRET,
    )

    assert (
        RTK_PROFILE_SCHEMA_VERSION
        == 3
    )

    assert profile.gga_enabled is False
    assert profile.gga_interval_sec == 10.0
    assert profile.gga_max_age_sec == 5.0


def test_gga_runtime_edit_forces_active_running_profile_stopped(
    tmp_path: Path,
):
    store = RtkProfileStore(
        tmp_path / "runtime.sqlite3"
    )

    store.initialize()

    profile = store.create_profile(
        name="VRS",
        caster_host="caster.test",
        caster_port=2101,
        mountpoint="VRS",
        username="rover",
        password=SECRET,
        gga_enabled=True,
    )

    store.set_active_profile(
        profile.profile_id
    )

    store.set_desired_state(
        DesiredState.RUNNING
    )

    updated = store.update_profile(
        profile.profile_id,
        gga_interval_sec=5.0,
    )

    assert updated.gga_enabled is True
    assert updated.gga_interval_sec == 5.0

    assert (
        store.runtime_state()
        .desired_state
        is DesiredState.STOPPED
    )


def test_active_worker_config_contains_gga_policy(
    tmp_path: Path,
):
    store = RtkProfileStore(
        tmp_path / "worker.sqlite3"
    )

    store.initialize()

    profile = store.create_profile(
        name="VRS",
        caster_host="caster.test",
        caster_port=2101,
        mountpoint="VRS",
        username="rover",
        password=SECRET,
        gga_enabled=True,
        gga_interval_sec=7.5,
        gga_max_age_sec=3.0,
    )

    store.set_active_profile(
        profile.profile_id
    )

    config = (
        store.build_active_worker_config(
            "gga-run"
        )
    )

    assert config.gga_enabled is True
    assert config.gga_interval_sec == 7.5
    assert config.gga_max_age_sec == 3.0


def _create_v1_database(
    database_file: Path,
) -> None:
    connection = sqlite3.connect(
        str(database_file)
    )

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
                max_mavros_rtcm_frame_bytes INTEGER NOT NULL,
                enabled INTEGER NOT NULL,
                revision INTEGER NOT NULL,
                created_at INTEGER NOT NULL,
                updated_at INTEGER NOT NULL
            );

            CREATE TABLE rtk_runtime_state (
                singleton_id INTEGER PRIMARY KEY,
                active_profile_id INTEGER,
                desired_state TEXT NOT NULL,
                revision INTEGER NOT NULL,
                updated_at INTEGER NOT NULL,
                FOREIGN KEY(active_profile_id)
                    REFERENCES rtk_profiles(id)
                    ON DELETE SET NULL
            );

            INSERT INTO rtk_profiles (
                id,
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
                max_mavros_rtcm_frame_bytes,
                enabled,
                revision,
                created_at,
                updated_at
            )
            VALUES (
                1,
                'Legacy Base',
                'caster.test',
                2101,
                'LEGACY',
                'rover',
                'legacy-secret',
                '/mavros/gps_rtk/send_rtcm',
                10.0,
                1.0,
                5.0,
                10.0,
                5.0,
                10.0,
                720,
                1,
                1,
                1,
                1
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
                1
            );

            PRAGMA user_version = 1;
            """
        )

        connection.commit()

    finally:
        connection.close()


def test_v1_database_migrates_without_enabling_gga(
    tmp_path: Path,
):
    database_file = (
        tmp_path / "legacy.sqlite3"
    )

    _create_v1_database(
        database_file
    )

    store = RtkProfileStore(
        database_file
    )

    store.initialize()

    profile = store.get_profile(
        1
    )

    assert profile.gga_enabled is False
    assert profile.gga_interval_sec == 10.0
    assert profile.gga_max_age_sec == 5.0
    assert profile.tls_mode == "REQUIRED"

    connection = sqlite3.connect(
        str(database_file)
    )

    try:
        version = connection.execute(
            "PRAGMA user_version"
        ).fetchone()[0]

    finally:
        connection.close()

    assert version == 3


def test_v2_database_migrates_to_required_tls(
    tmp_path: Path,
):
    database_file = (
        tmp_path / "legacy-v2.sqlite3"
    )

    _create_v1_database(
        database_file
    )

    connection = sqlite3.connect(
        str(database_file)
    )

    try:
        connection.execute(
            """
            ALTER TABLE rtk_profiles
            ADD COLUMN gga_enabled INTEGER NOT NULL
                DEFAULT 0
                CHECK(gga_enabled IN (0, 1))
            """
        )

        connection.execute(
            """
            ALTER TABLE rtk_profiles
            ADD COLUMN gga_interval_sec REAL NOT NULL
                DEFAULT 10.0
            """
        )

        connection.execute(
            """
            ALTER TABLE rtk_profiles
            ADD COLUMN gga_max_age_sec REAL NOT NULL
                DEFAULT 5.0
            """
        )

        connection.execute(
            "PRAGMA user_version = 2"
        )

        connection.commit()

    finally:
        connection.close()

    store = RtkProfileStore(
        database_file
    )

    store.initialize()

    profile = store.get_profile(
        1
    )

    assert profile.gga_enabled is False
    assert profile.tls_mode == "REQUIRED"

    connection = sqlite3.connect(
        str(database_file)
    )

    try:
        version = connection.execute(
            "PRAGMA user_version"
        ).fetchone()[0]

    finally:
        connection.close()

    assert version == 3
