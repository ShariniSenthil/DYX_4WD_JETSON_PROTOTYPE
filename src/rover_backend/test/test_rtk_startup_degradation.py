"""Tests for independent RTK deployment/startup degradation."""

from __future__ import annotations

import asyncio
import sys
import types


# main.py imports mission_routes, which imports rover_backend.ros_bridge.
# The workstation test environment intentionally has no ROS 2/rclpy.
# Install the smallest possible module stub before importing main so these
# tests exercise backend startup policy without pretending to test ROS.
class _RosBridgeStub:
    def __init__(self):
        self.running = False

    def rtk_mavros_ready(self):
        return False

    def start(self):
        self.running = True

    def stop(self):
        self.running = False


_ros_bridge_module = types.ModuleType(
    "rover_backend.ros_bridge"
)
_ros_bridge_module.ros_bridge = (
    _RosBridgeStub()
)

sys.modules[
    "rover_backend.ros_bridge"
] = _ros_bridge_module


from rover_backend import main
from rover_backend.config import load_settings
from rover_backend.rtk_backend_lifecycle import (
    RtkBackendLifecycleCleanupError,
    RtkBackendLifecycleError,
)


class FakeRoverState:
    def __init__(self):
        self.updates = []

    def update(
        self,
        section_name,
        **values,
    ):
        self.updates.append(
            (
                section_name,
                values,
            )
        )

        return dict(values)


class CleanFailureLifecycle:
    started = False

    def start(self):
        raise RtkBackendLifecycleError(
            "synthetic clean RTK startup failure"
        )


class CleanupFailureLifecycle:
    started = False

    def start(self):
        raise RtkBackendLifecycleCleanupError(
            "synthetic incomplete RTK cleanup"
        )


class SuccessfulLifecycle:
    started = True

    def start(self):
        return object()


def test_default_rtk_database_is_under_normal_data_directory(
    tmp_path,
    monkeypatch,
):
    data_directory = (
        tmp_path / "data"
    )

    monkeypatch.setenv(
        "DYX_DATA_DIRECTORY",
        str(data_directory),
    )

    monkeypatch.setenv(
        "HOME",
        str(tmp_path / "home"),
    )

    monkeypatch.delenv(
        "DYX_RTK_DATABASE_FILE",
        raising=False,
    )

    loaded = load_settings()

    assert (
        loaded.rtk_database_file
        == (
            data_directory
            / "rtk"
            / "rtk.sqlite3"
        ).resolve()
    )


def test_rtk_database_environment_override_is_supported(
    tmp_path,
    monkeypatch,
):
    custom = (
        tmp_path
        / "custom"
        / "rtk.sqlite3"
    )

    monkeypatch.setenv(
        "HOME",
        str(tmp_path / "home"),
    )

    monkeypatch.setenv(
        "DYX_DATA_DIRECTORY",
        str(tmp_path / "data"),
    )

    monkeypatch.setenv(
        "DYX_RTK_DATABASE_FILE",
        str(custom),
    )

    loaded = load_settings()

    assert (
        loaded.rtk_database_file
        == custom.resolve()
    )


def test_clean_rtk_failure_keeps_backend_startup_available(
    monkeypatch,
):
    fake_state = FakeRoverState()

    monkeypatch.setattr(
        main,
        "rtk_backend_lifecycle",
        CleanFailureLifecycle(),
    )

    monkeypatch.setattr(
        main,
        "rover_state",
        fake_state,
    )

    started = asyncio.run(
        main._start_rtk_backend_degraded()
    )

    assert started is False

    assert fake_state.updates == [
        (
            "rtk",
            {
                "healthy": False,
                "correction_age_sec": None,
                "status": "CONTROL_UNAVAILABLE",
            },
        )
    ]


def test_incomplete_rtk_cleanup_remains_backend_fatal(
    monkeypatch,
):
    fake_state = FakeRoverState()

    monkeypatch.setattr(
        main,
        "rtk_backend_lifecycle",
        CleanupFailureLifecycle(),
    )

    monkeypatch.setattr(
        main,
        "rover_state",
        fake_state,
    )

    try:
        asyncio.run(
            main._start_rtk_backend_degraded()
        )
    except RtkBackendLifecycleCleanupError:
        pass
    else:
        raise AssertionError(
            "cleanup failure must propagate"
        )

    assert fake_state.updates == []


def test_successful_rtk_start_reports_started(
    monkeypatch,
):
    fake_state = FakeRoverState()

    monkeypatch.setattr(
        main,
        "rtk_backend_lifecycle",
        SuccessfulLifecycle(),
    )

    monkeypatch.setattr(
        main,
        "rover_state",
        fake_state,
    )

    started = asyncio.run(
        main._start_rtk_backend_degraded()
    )

    assert started is True
    assert fake_state.updates == []
