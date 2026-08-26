"""TLS policy contract for RTK profiles and worker config."""

from pathlib import Path

import pytest

from rover_backend.rtk_manager_core import (
    DesiredState,
)
from rover_backend.rtk_process_protocol import (
    ConfigValidationError,
    WORKER_CONFIG_SCHEMA_VERSION,
    WorkerConfig,
    decode_worker_config,
    encode_worker_config,
)
from rover_backend.rtk_profile_store import (
    RtkProfileStore,
)


SECRET = "TLS_CONTRACT_SECRET_8042"


def _worker(
    **changes,
):
    values = {
        "schema_version": (
            WORKER_CONFIG_SCHEMA_VERSION
        ),
        "run_id": "tls-contract",
        "caster_host": "caster.test",
        "caster_port": 443,
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
        "gga_enabled": False,
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


def test_worker_defaults_to_required_tls_and_round_trips():
    config = _worker()

    assert config.tls_mode == "REQUIRED"

    decoded = decode_worker_config(
        encode_worker_config(
            config
        )
    )

    assert decoded.tls_mode == "REQUIRED"
    assert SECRET not in repr(decoded)


@pytest.mark.parametrize(
    "value",
    (
        "",
        "required",
        "PREFERRED",
        "PLAINTEXT",
        None,
        True,
    ),
)
def test_worker_rejects_unknown_tls_policy(
    value,
):
    with pytest.raises(
        ConfigValidationError,
        match="tls_mode",
    ):
        _worker(
            tls_mode=value
        )


def test_new_profile_defaults_to_required_tls(
    tmp_path: Path,
):
    store = RtkProfileStore(
        tmp_path / "rtk.sqlite3"
    )

    store.initialize()

    profile = store.create_profile(
        name="Secure",
        caster_host="caster.test",
        caster_port=443,
        mountpoint="MOUNT",
        username="rover",
        password=SECRET,
    )

    assert profile.tls_mode == "REQUIRED"

    store.set_active_profile(
        profile.profile_id
    )

    config = (
        store.build_active_worker_config(
            "tls-run"
        )
    )

    assert config.tls_mode == "REQUIRED"


def test_plaintext_requires_explicit_disabled_policy(
    tmp_path: Path,
):
    store = RtkProfileStore(
        tmp_path / "rtk.sqlite3"
    )

    store.initialize()

    profile = store.create_profile(
        name="Legacy plaintext",
        caster_host="caster.test",
        caster_port=2101,
        mountpoint="MOUNT",
        username="rover",
        password=SECRET,
        tls_mode="DISABLED",
    )

    assert profile.tls_mode == "DISABLED"


def test_transport_policy_edit_forces_running_profile_stopped(
    tmp_path: Path,
):
    store = RtkProfileStore(
        tmp_path / "rtk.sqlite3"
    )

    store.initialize()

    profile = store.create_profile(
        name="Secure",
        caster_host="caster.test",
        caster_port=443,
        mountpoint="MOUNT",
        username="rover",
        password=SECRET,
    )

    store.set_active_profile(
        profile.profile_id
    )

    store.set_desired_state(
        DesiredState.RUNNING
    )

    updated = store.update_profile(
        profile.profile_id,
        tls_mode="DISABLED",
    )

    assert updated.tls_mode == "DISABLED"

    assert (
        store.runtime_state().desired_state
        is DesiredState.STOPPED
    )
