"""Global RTK correction source: NTRIP (default) or LoRa (schema v5).

Operator requirements (2026-09-26): NTRIP stays the default with mountpoint
selection; selecting LoRa is persisted and used on the next boot; switching
while RTK is RUNNING restarts the worker on the new source.
"""

from __future__ import annotations

import sqlite3
from pathlib import Path

import pytest

from rover_backend.rtk_control_service import (
    RtkControlRuntimeError,
    RtkControlService,
)
from rover_backend.rtk_manager_core import DesiredState
from rover_backend.rtk_process_protocol import (
    ConfigValidationError,
    WorkerConfig,
    decode_worker_config,
    encode_worker_config,
)
from rover_backend.rtk_profile_store import (
    RTK_PROFILE_SCHEMA_VERSION,
    RtkProfileStateError,
    RtkProfileStore,
    RtkProfileValidationError,
)
from rover_backend.rtk_serial_ports import (
    ROLE_GNSS_RECEIVER,
    ROLE_OTHER,
    list_serial_ports,
)

LORA_PORT = "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30A1B2C-if00-port0"
MOSAIC_USB2 = "/dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device_3804732-if04"


@pytest.fixture
def store(tmp_path: Path) -> RtkProfileStore:
    value = RtkProfileStore(tmp_path / "rtk" / "rtk.sqlite3")
    value.initialize()
    return value


def _ntrip_profile(store: RtkProfileStore, name: str = "Base") -> int:
    return store.create_profile(
        name=name,
        caster_host="caster.example.test",
        caster_port=2101,
        mountpoint="DYX_RTCM3",
        username="dyx-rover",
        password="secret-1",
    ).profile_id


def _select_lora(store: RtkProfileStore, **extra):
    return store.update_correction_source(
        source="LORA",
        lora_serial_device=LORA_PORT,
        lora_direct_serial_device=MOSAIC_USB2,
        **extra,
    )


class FakeRuntime:
    def __init__(self, fail_start: bool = False) -> None:
        self.calls: list[str] = []
        self.fail_start = fail_start
        self.snapshot = None

    def request_start(self) -> None:
        self.calls.append("start")
        if self.fail_start:
            raise RuntimeError("supervisor refused")

    def request_stop(self) -> None:
        self.calls.append("stop")


# --------------------------------------------------------------------------
# Store
# --------------------------------------------------------------------------


def test_new_database_defaults_to_ntrip(store):
    source = store.correction_source()
    assert source.source == "NTRIP"
    assert source.lora_serial_device is None
    assert source.lora_serial_baud == 57600
    assert source.lora_direct_inject is True
    assert source.revision == 1


def test_schema_v4_database_upgrades_to_v5_without_touching_profiles(tmp_path):
    path = tmp_path / "rtk.sqlite3"
    old = RtkProfileStore(path)
    old.initialize()
    profile_id = _ntrip_profile(old)
    old.set_active_profile(profile_id)
    old.set_desired_state(DesiredState.RUNNING)
    connection = sqlite3.connect(path)
    connection.execute("DROP TABLE rtk_correction_source")
    connection.execute("PRAGMA user_version = 4")
    connection.commit()
    connection.close()

    upgraded = RtkProfileStore(path)
    upgraded.initialize()

    connection = sqlite3.connect(path)
    version = connection.execute("PRAGMA user_version").fetchone()[0]
    connection.close()
    assert version == RTK_PROFILE_SCHEMA_VERSION == 5
    assert upgraded.correction_source().source == "NTRIP"
    runtime = upgraded.runtime_state()
    assert runtime.active_profile_id == profile_id
    assert runtime.desired_state is DesiredState.RUNNING
    assert upgraded.build_active_worker_config("run").correction_source == "NTRIP"


def test_selection_persists_across_reopen(tmp_path):
    path = tmp_path / "rtk.sqlite3"
    first = RtkProfileStore(path)
    first.initialize()
    _select_lora(first)
    first.set_desired_state(DesiredState.RUNNING)

    reopened = RtkProfileStore(path)
    reopened.initialize()
    assert reopened.correction_source().source == "LORA"
    assert reopened.runtime_state().desired_state is DesiredState.RUNNING
    config = reopened.build_active_worker_config("boot")
    assert config.correction_source == "LORA"
    assert config.lora_serial_device == LORA_PORT


def test_partial_update_keeps_omitted_fields_and_null_clears(store):
    _select_lora(store, lora_serial_baud=115200)
    _, after = store.update_correction_source(lora_serial_baud=9600)
    assert after.lora_serial_device == LORA_PORT
    assert after.lora_serial_baud == 9600
    _, cleared = store.update_correction_source(lora_serial_device=None)
    assert cleared.lora_serial_device is None


def test_noop_update_does_not_bump_revision(store):
    _, first = _select_lora(store)
    _, again = _select_lora(store)
    assert again.revision == first.revision


@pytest.mark.parametrize(
    "changes",
    [
        {"source": "RADIO"},
        {"source": 3},
        {"lora_serial_device": "ttyUSB0"},
        {"lora_serial_device": "/dev/tty\x07USB0"},
        {"lora_serial_baud": 0},
        {"lora_serial_baud": True},
        {"lora_direct_inject": "yes"},
        {"lora_direct_serial_baud": -1},
    ],
)
def test_invalid_values_are_rejected(store, changes):
    with pytest.raises(RtkProfileValidationError):
        store.update_correction_source(**changes)


def test_source_is_case_insensitive(store):
    _, after = store.update_correction_source(source="lora")
    assert after.source == "LORA"


def test_radio_and_output_must_be_different_ports(store):
    with pytest.raises(RtkProfileValidationError):
        store.update_correction_source(
            lora_serial_device=MOSAIC_USB2,
            lora_direct_serial_device=MOSAIC_USB2,
        )


def test_lora_running_needs_no_ntrip_profile(store):
    _select_lora(store)
    assert store.set_desired_state(DesiredState.RUNNING).desired_state is (
        DesiredState.RUNNING
    )


def test_lora_running_requires_radio_port(store):
    store.update_correction_source(source="LORA", lora_direct_serial_device=MOSAIC_USB2)
    with pytest.raises(RtkProfileStateError, match="radio serial port"):
        store.set_desired_state(DesiredState.RUNNING)


def test_lora_direct_output_requires_output_port(store):
    store.update_correction_source(source="LORA", lora_serial_device=LORA_PORT)
    with pytest.raises(RtkProfileStateError, match="output port"):
        store.set_desired_state(DesiredState.RUNNING)


def test_lora_via_mavros_needs_no_output_port(store):
    store.update_correction_source(
        source="LORA", lora_serial_device=LORA_PORT, lora_direct_inject=False
    )
    store.set_desired_state(DesiredState.RUNNING)
    config = store.build_active_worker_config("run")
    assert config.direct_inject is False
    assert config.direct_serial_device is None
    assert config.rtcm_topic == "/mavros/gps_rtk/send_rtcm"


def test_lora_worker_config_carries_no_caster_and_default_timing(store):
    _ntrip_profile(store)
    _select_lora(store, lora_serial_baud=115200)
    config = store.build_active_worker_config("run-7")
    assert config.correction_source == "LORA"
    assert config.lora_serial_device == LORA_PORT
    assert config.lora_serial_baud == 115200
    assert config.direct_inject is True
    assert config.direct_serial_device == MOSAIC_USB2
    assert (config.caster_host, config.mountpoint, config.password) == ("", "", "")
    assert config.gga_enabled is False
    assert config.first_data_timeout_sec == 10.0
    assert config.stale_reconnect_sec == 10.0


def test_ntrip_worker_config_unchanged_when_source_is_ntrip(store):
    profile_id = _ntrip_profile(store)
    store.set_active_profile(profile_id)
    _select_lora(store)
    store.update_correction_source(source="NTRIP")
    config = store.build_active_worker_config("run")
    assert config.correction_source == "NTRIP"
    assert config.mountpoint == "DYX_RTCM3"


@pytest.mark.parametrize("operation", ["update", "delete", "activate", "clear"])
def test_profile_changes_do_not_stop_a_running_lora_worker(store, operation):
    first = _ntrip_profile(store, "A")
    second = _ntrip_profile(store, "B")
    store.set_active_profile(first)
    _select_lora(store)
    store.set_desired_state(DesiredState.RUNNING)

    if operation == "update":
        store.update_profile(first, mountpoint="OTHER")
    elif operation == "delete":
        store.delete_profile(first)
    elif operation == "activate":
        store.set_active_profile(second)
    else:
        store.clear_active_profile()

    assert store.runtime_state().desired_state is DesiredState.RUNNING


def test_profile_changes_still_stop_a_running_ntrip_worker(store):
    first = _ntrip_profile(store, "A")
    second = _ntrip_profile(store, "B")
    store.set_active_profile(first)
    store.set_desired_state(DesiredState.RUNNING)
    store.set_active_profile(second)
    assert store.runtime_state().desired_state is DesiredState.STOPPED


# --------------------------------------------------------------------------
# Control service: restart on switch
# --------------------------------------------------------------------------


def test_switch_while_running_restarts_worker_and_stays_running(store):
    profile_id = _ntrip_profile(store)
    store.set_active_profile(profile_id)
    runtime = FakeRuntime()
    control = RtkControlService(store, runtime)
    control.request_start()
    runtime.calls.clear()

    after = control.update_correction_source(
        source="LORA",
        lora_serial_device=LORA_PORT,
        lora_direct_serial_device=MOSAIC_USB2,
    )

    assert after.source == "LORA"
    assert runtime.calls == ["stop", "start"]
    assert store.runtime_state().desired_state is DesiredState.RUNNING
    assert store.build_active_worker_config("next").correction_source == "LORA"


def test_switch_while_stopped_only_saves(store):
    runtime = FakeRuntime()
    control = RtkControlService(store, runtime)
    control.update_correction_source(
        source="LORA", lora_serial_device=LORA_PORT,
        lora_direct_serial_device=MOSAIC_USB2,
    )
    assert runtime.calls == []
    assert store.runtime_state().desired_state is DesiredState.STOPPED


def test_lora_setting_change_while_running_on_lora_restarts(store):
    _select_lora(store)
    runtime = FakeRuntime()
    control = RtkControlService(store, runtime)
    control.request_start()
    runtime.calls.clear()
    control.update_correction_source(lora_serial_baud=115200)
    assert runtime.calls == ["stop", "start"]


def test_lora_setting_change_while_running_on_ntrip_does_not_restart(store):
    profile_id = _ntrip_profile(store)
    store.set_active_profile(profile_id)
    runtime = FakeRuntime()
    control = RtkControlService(store, runtime)
    control.request_start()
    runtime.calls.clear()
    control.update_correction_source(lora_serial_device=LORA_PORT)
    assert runtime.calls == []


def test_switch_that_cannot_start_is_rejected_and_not_saved(store):
    _select_lora(store)
    runtime = FakeRuntime()
    control = RtkControlService(store, runtime)
    control.request_start()
    runtime.calls.clear()

    with pytest.raises(RtkProfileStateError, match="no active NTRIP profile"):
        control.update_correction_source(source="NTRIP")

    assert store.correction_source().source == "LORA"
    assert runtime.calls == []
    assert store.runtime_state().desired_state is DesiredState.RUNNING


def test_failed_restart_fails_closed_to_stopped(store):
    profile_id = _ntrip_profile(store)
    store.set_active_profile(profile_id)
    runtime = FakeRuntime()
    control = RtkControlService(store, runtime)
    control.request_start()
    runtime.fail_start = True

    with pytest.raises(RtkControlRuntimeError):
        control.update_correction_source(
            source="LORA", lora_serial_device=LORA_PORT,
            lora_direct_serial_device=MOSAIC_USB2,
        )

    assert store.runtime_state().desired_state is DesiredState.STOPPED
    assert store.correction_source().source == "LORA"
    assert runtime.calls[-1] == "stop"


# --------------------------------------------------------------------------
# Worker config wire contract
# --------------------------------------------------------------------------


def _lora_config(**overrides) -> WorkerConfig:
    values = dict(
        schema_version=5, run_id="r", caster_host="", caster_port=0,
        mountpoint="", username="", password="",
        rtcm_topic="/mavros/gps_rtk/send_rtcm", connect_timeout_sec=10.0,
        socket_timeout_sec=1.0, healthy_age_sec=5.0, stale_reconnect_sec=10.0,
        reconnect_delay_sec=5.0, first_data_timeout_sec=10.0,
        gga_enabled=False, gga_interval_sec=10.0, gga_max_age_sec=5.0,
        max_mavros_rtcm_frame_bytes=720, direct_inject=True,
        direct_serial_device=MOSAIC_USB2, correction_source="LORA",
        lora_serial_device=LORA_PORT, lora_serial_baud=57600,
    )
    values.update(overrides)
    return WorkerConfig(**values)


def test_lora_worker_config_round_trips():
    config = _lora_config()
    assert decode_worker_config(encode_worker_config(config)) == config


@pytest.mark.parametrize(
    "overrides",
    [
        {"correction_source": "RADIO"},
        {"lora_serial_device": None},
        {"lora_serial_device": "ttyUSB0"},
        {"lora_serial_baud": 0},
        {"lora_serial_device": MOSAIC_USB2},
    ],
)
def test_invalid_lora_worker_config_is_rejected(overrides):
    with pytest.raises(ConfigValidationError):
        _lora_config(**overrides)


def test_ntrip_worker_config_still_requires_caster():
    with pytest.raises(ConfigValidationError):
        _lora_config(correction_source="NTRIP")


# --------------------------------------------------------------------------
# Serial port listing
# --------------------------------------------------------------------------


def test_serial_ports_prefer_by_id_classify_and_hide_flight_controller():
    by_id = {
        "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00": "/dev/ttyACM0",
        "/dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device_3804732-if02": "/dev/ttyACM1",
        MOSAIC_USB2: "/dev/ttyACM2",
        LORA_PORT: "/dev/ttyUSB0",
    }
    bare = {
        "/dev/ttyUSB*": ["/dev/ttyUSB0", "/dev/ttyUSB1"],
        "/dev/ttyACM*": ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"],
        "/dev/ttyTHS*": [],
    }

    def fake_glob(pattern):
        if pattern.startswith("/dev/serial/by-id/"):
            return list(by_id)
        return bare[pattern]

    ports = list_serial_ports(
        glob_fn=fake_glob, realpath=lambda p: by_id.get(p, p)
    )
    paths = {port.path: port for port in ports}

    assert all(port.device != "/dev/ttyACM0" for port in ports)
    assert paths[MOSAIC_USB2].role == ROLE_GNSS_RECEIVER
    assert paths[LORA_PORT].role == ROLE_OTHER
    assert paths[LORA_PORT].device == "/dev/ttyUSB0"
    assert "/dev/ttyUSB0" not in paths          # listed once, by its by-id name
    assert paths["/dev/ttyUSB1"].role == ROLE_OTHER
