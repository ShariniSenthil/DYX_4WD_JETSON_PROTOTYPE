"""Tests for the single-owner RTK supervisor service."""

from __future__ import annotations

import sys
import threading
import time

from types import SimpleNamespace

import pytest

from rover_backend.rtk_manager_core import (
    DesiredState,
    ManagerState,
)
from rover_backend.rtk_runtime_service import (
    RtkRuntimeService,
    RtkRuntimeServiceError,
    RtkRuntimeServiceShuttingDownError,
    default_rtk_worker_command,
)


def runtime_snapshot(
    *,
    state=ManagerState.STOPPED,
    desired=DesiredState.STOPPED,
    run_id=None,
    pid=None,
):
    return SimpleNamespace(
        manager=SimpleNamespace(
            desired_state=desired,
            manager_state=state,
            active_run_id=run_id,
        ),
        process=SimpleNamespace(
            active_run_id=run_id,
            pid=pid,
        ),
    )


class FakeOrchestrator:
    def __init__(
        self,
        *,
        open_error=None,
        delayed_stop_ticks=0,
        fail_tick_once=False,
    ):
        self.open_error = open_error
        self.delayed_stop_ticks = (
            delayed_stop_ticks
        )
        self.fail_tick_once = fail_tick_once

        self.opened = False
        self.closed = False
        self.ready = False

        self.calls = []
        self.state = ManagerState.STOPPED
        self.desired = DesiredState.STOPPED
        self.run_id = None
        self.pid = None

        self._stop_ticks_remaining = 0

    @property
    def snapshot(self):
        return runtime_snapshot(
            state=self.state,
            desired=self.desired,
            run_id=self.run_id,
            pid=self.pid,
        )

    def _record(self, name):
        self.calls.append(
            (
                name,
                threading.get_ident(),
            )
        )

    def open(self):
        self._record("open")

        if self.open_error is not None:
            raise self.open_error

        self.opened = True
        return self

    def close(self):
        self._record("close")

        assert self.run_id is None
        assert self.pid is None

        self.closed = True

    def set_mavros_ready(
        self,
        ready,
        now_sec,
    ):
        self._record(
            "ready:%s" % bool(ready)
        )
        self.ready = bool(ready)
        return ()

    def request_start(
        self,
        now_sec,
    ):
        self._record("start")
        self.desired = DesiredState.RUNNING

        if self.ready:
            self.state = ManagerState.STARTING
            self.run_id = "run-1"
            self.pid = 51001
        else:
            self.state = (
                ManagerState.WAITING_FOR_MAVROS
            )

        return ()

    def request_stop(
        self,
        now_sec,
    ):
        self._record("stop")
        self.desired = DesiredState.STOPPED

        if self.run_id is None:
            self.state = ManagerState.STOPPED
            return ()

        self.state = ManagerState.STOPPING

        self._stop_ticks_remaining = (
            self.delayed_stop_ticks
        )

        if self._stop_ticks_remaining == 0:
            self.run_id = None
            self.pid = None
            self.state = ManagerState.STOPPED

        return ()

    def tick(
        self,
        now_sec,
    ):
        self._record("tick")

        if self.fail_tick_once:
            self.fail_tick_once = False
            raise RuntimeError(
                "synthetic tick failure"
            )

        if (
            self.state
            is ManagerState.STOPPING
            and self._stop_ticks_remaining > 0
        ):
            self._stop_ticks_remaining -= 1

            if self._stop_ticks_remaining == 0:
                self.run_id = None
                self.pid = None
                self.state = ManagerState.STOPPED

        return ()


def make_service(
    orchestrator=None,
    readiness=None,
):
    if orchestrator is None:
        orchestrator = FakeOrchestrator()

    if readiness is None:
        readiness = lambda: False

    service = RtkRuntimeService(
        orchestrator,
        readiness,
        poll_interval_sec=0.01,
        command_timeout_sec=1.0,
        start_timeout_sec=1.0,
    )

    return service, orchestrator


def wait_until(
    predicate,
    timeout=1.0,
):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        if predicate():
            return

        time.sleep(0.005)

    raise AssertionError(
        "condition was not reached"
    )


def test_default_worker_command_uses_current_interpreter():
    command = default_rtk_worker_command()

    assert command == (
        sys.executable,
        "-m",
        "rover_backend.rtk_worker_bootstrap",
    )


def test_start_opens_orchestrator_on_supervisor_thread():
    service, orchestrator = make_service()

    caller_thread = threading.get_ident()

    service.start()

    try:
        owner = service.snapshot.owner_thread_id

        assert owner is not None
        assert owner != caller_thread

        open_call = next(
            call
            for call in orchestrator.calls
            if call[0] == "open"
        )

        assert open_call[1] == owner

    finally:
        assert service.shutdown() is True


def test_start_is_idempotent_while_running():
    service, orchestrator = make_service()

    service.start()
    service.start()

    try:
        assert (
            sum(
                1
                for name, _ in orchestrator.calls
                if name == "open"
            )
            == 1
        )
    finally:
        assert service.shutdown() is True


def test_request_start_runs_only_on_supervisor_thread():
    service, orchestrator = make_service()

    service.start()

    try:
        owner = service.snapshot.owner_thread_id

        service.request_start()

        start_call = next(
            call
            for call in orchestrator.calls
            if call[0] == "start"
        )

        assert start_call[1] == owner

    finally:
        assert service.shutdown() is True


def test_request_stop_runs_only_on_supervisor_thread():
    service, orchestrator = make_service()

    service.start()

    try:
        owner = service.snapshot.owner_thread_id

        service.request_stop()

        stop_call = next(
            call
            for call in orchestrator.calls
            if call[0] == "stop"
        )

        assert stop_call[1] == owner

    finally:
        assert service.shutdown() is True


def test_mavros_ready_transition_is_forwarded_once():
    ready_state = {
        "value": False
    }

    service, orchestrator = make_service(
        readiness=lambda: ready_state["value"]
    )

    service.start()

    try:
        ready_state["value"] = True

        wait_until(
            lambda: any(
                name == "ready:True"
                for name, _ in orchestrator.calls
            )
        )

        time.sleep(0.05)

        assert (
            sum(
                1
                for name, _ in orchestrator.calls
                if name == "ready:True"
            )
            == 1
        )

        assert (
            service.snapshot.mavros_ready
            is True
        )

    finally:
        assert service.shutdown() is True


def test_readiness_provider_failure_fails_closed_and_recovers():
    state = {
        "raise": True,
        "ready": False,
    }

    def provider():
        if state["raise"]:
            raise RuntimeError(
                "synthetic graph failure"
            )

        return state["ready"]

    service, orchestrator = make_service(
        readiness=provider
    )

    service.start()

    try:
        wait_until(
            lambda: (
                service.snapshot.last_error_code
                == "MAVROS_READINESS_ERROR"
            )
        )

        assert (
            service.snapshot.mavros_ready
            is False
        )

        state["raise"] = False
        state["ready"] = True

        wait_until(
            lambda: (
                service.snapshot.mavros_ready
                is True
            )
        )

        assert (
            service.snapshot.last_error_code
            is None
        )

    finally:
        assert service.shutdown() is True


def test_shutdown_waits_for_stop_and_reap():
    orchestrator = FakeOrchestrator(
        delayed_stop_ticks=3
    )

    service, _ = make_service(
        orchestrator=orchestrator,
        readiness=lambda: True,
    )

    service.start()
    service.request_start()

    assert orchestrator.run_id == "run-1"

    assert service.shutdown(
        timeout_sec=1.0
    ) is True

    assert orchestrator.run_id is None
    assert orchestrator.pid is None
    assert orchestrator.closed is True


def test_shutdown_is_idempotent_after_clean_exit():
    service, _ = make_service()

    service.start()

    assert service.shutdown() is True
    assert service.shutdown() is True


def test_new_start_is_rejected_after_shutdown():
    service, _ = make_service()

    service.start()

    assert service.shutdown() is True

    with pytest.raises(
        RtkRuntimeServiceShuttingDownError
    ):
        service.request_start()


def test_open_failure_is_reported():
    orchestrator = FakeOrchestrator(
        open_error=RuntimeError(
            "synthetic manager lock failure"
        )
    )

    service, _ = make_service(
        orchestrator=orchestrator
    )

    with pytest.raises(
        RtkRuntimeServiceError
    ):
        service.start()

    assert (
        service.snapshot.last_error_code
        == "SUPERVISOR_START_FAILED"
    )


def test_tick_failure_enters_fail_closed_shutdown():
    orchestrator = FakeOrchestrator(
        fail_tick_once=True
    )

    service, _ = make_service(
        orchestrator=orchestrator
    )

    service.start()

    wait_until(
        lambda: (
            service.snapshot.shutdown_requested
            is True
        )
    )

    assert (
        service.snapshot.last_error_code
        == "SUPERVISOR_RUNTIME_FAILED"
    )

    assert service.shutdown(
        timeout_sec=1.0
    ) is True

    assert orchestrator.closed is True


def test_snapshot_does_not_expose_config_factory_or_secret():
    service, _ = make_service()

    service.start()

    try:
        snapshot = service.snapshot

        text = repr(snapshot)

        assert "password" not in text.lower()
        assert not hasattr(
            snapshot,
            "config_factory",
        )
    finally:
        assert service.shutdown() is True


def test_readiness_forward_failure_stops_child_before_close():
    """Unexpected runtime failure must not abandon an active worker."""

    ready_state = {
        "value": True,
    }

    service, orchestrator = make_service(
        readiness=lambda: ready_state["value"]
    )

    service.start()

    wait_until(
        lambda: (
            service.snapshot.mavros_ready
            is True
        )
    )

    service.request_start()

    assert orchestrator.run_id == "run-1"
    assert orchestrator.pid == 51001

    original_set_ready = (
        orchestrator.set_mavros_ready
    )

    failed = {
        "once": False,
    }

    def fail_once(
        ready,
        now_sec,
    ):
        if (
            not ready
            and not failed["once"]
        ):
            failed["once"] = True

            raise RuntimeError(
                "synthetic readiness "
                "forward failure"
            )

        return original_set_ready(
            ready,
            now_sec,
        )

    orchestrator.set_mavros_ready = (
        fail_once
    )

    ready_state["value"] = False

    wait_until(
        lambda: (
            service.snapshot.shutdown_requested
            is True
        )
    )

    assert service.shutdown(
        timeout_sec=1.0
    ) is True

    assert failed["once"] is True

    assert orchestrator.run_id is None
    assert orchestrator.pid is None

    assert orchestrator.closed is True

    close_index = next(
        index
        for index, (name, _) in enumerate(
            orchestrator.calls
        )
        if name == "close"
    )

    stop_index = next(
        index
        for index, (name, _) in enumerate(
            orchestrator.calls
        )
        if name == "stop"
    )

    assert stop_index < close_index


def test_clock_failure_after_open_drives_clean_shutdown():
    """A broken injected clock must not terminate the owner thread."""

    orchestrator = FakeOrchestrator()

    calls = {
        "count": 0,
    }

    def broken_clock():
        calls["count"] += 1

        if calls["count"] == 1:
            raise RuntimeError(
                "synthetic monotonic "
                "clock failure"
            )

        return time.monotonic()

    service = RtkRuntimeService(
        orchestrator,
        lambda: False,
        poll_interval_sec=0.01,
        command_timeout_sec=1.0,
        start_timeout_sec=1.0,
        clock=broken_clock,
    )

    # open() succeeds, therefore this is a runtime failure,
    # never a startup failure.
    service.start()

    wait_until(
        lambda: (
            service.snapshot.shutdown_requested
            is True
        )
    )

    assert service.shutdown(
        timeout_sec=1.0
    ) is True

    assert (
        service.snapshot.last_error_code
        == "SUPERVISOR_CLOCK_FAILED"
    )

    assert orchestrator.closed is True
