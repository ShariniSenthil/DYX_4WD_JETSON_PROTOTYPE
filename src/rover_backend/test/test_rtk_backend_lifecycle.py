"""Tests for production RTK backend lifecycle ownership."""

from __future__ import annotations

import sys

from pathlib import Path

import pytest

from fastapi import HTTPException

from rover_backend.rtk_backend_lifecycle import (
    RtkBackendLifecycle,
    RtkBackendLifecycleCleanupError,
    RtkBackendLifecycleError,
    build_production_runtime,
)
from rover_backend.rtk_manager_core import (
    DesiredState,
)
from rover_backend.rtk_profile_store import (
    RtkProfileStore,
)
from rover_backend.rtk_routes import (
    clear_rtk_control_service,
    get_rtk_control_service,
    install_rtk_control_service,
)
from rover_backend.rtk_runtime_service import (
    RtkRuntimeServiceSnapshot,
)


class FakeRuntime:
    def __init__(self):
        self.running = False

        self.start_calls = 0
        self.request_start_calls = 0
        self.request_stop_calls = 0
        self.shutdown_calls = 0

        self.fail_start = False
        self.fail_request_start = False

        self.shutdown_result = True

    @property
    def snapshot(self):
        return RtkRuntimeServiceSnapshot(
            running=self.running,
            shutdown_requested=False,
            owner_thread_id=(
                123
                if self.running
                else None
            ),
            mavros_ready=False,
            last_error_code=None,
            runtime=None,
        )

    def start(self):
        self.start_calls += 1

        if self.fail_start:
            raise RuntimeError(
                "synthetic runtime start failure"
            )

        self.running = True

    def request_start(self):
        self.request_start_calls += 1

        if self.fail_request_start:
            raise RuntimeError(
                "synthetic reconcile START failure"
            )

    def request_stop(self):
        self.request_stop_calls += 1

    def shutdown(
        self,
        timeout_sec=8.0,
    ):
        del timeout_sec

        self.shutdown_calls += 1

        if self.shutdown_result:
            self.running = False

        return self.shutdown_result


class FakeRuntimeFactory:
    def __init__(self):
        self.created = []
        self.configure = None

    def __call__(
        self,
        profile_store,
        readiness_provider,
    ):
        assert isinstance(
            profile_store,
            RtkProfileStore,
        )
        assert callable(
            readiness_provider
        )

        runtime = FakeRuntime()

        if self.configure is not None:
            self.configure(runtime)

        self.created.append(runtime)

        return runtime


@pytest.fixture(autouse=True)
def clean_registry():
    clear_rtk_control_service()

    yield

    clear_rtk_control_service()


def create_running_store(
    database_file: Path,
):
    store = RtkProfileStore(
        database_file
    )

    store.initialize()

    profile = store.create_profile(
        name="Office Base",
        caster_host="caster.test",
        caster_port=2101,
        mountpoint="MOUNT",
        username="rover",
        password="LIFECYCLE_SECRET",
    )

    store.set_active_profile(
        profile.profile_id
    )

    store.set_desired_state(
        DesiredState.RUNNING
    )

    return store


def test_production_runtime_factory_wires_frozen_components(
    tmp_path,
):
    store = RtkProfileStore(
        tmp_path / "factory.sqlite3"
    )

    readiness_provider = lambda: False

    runtime = build_production_runtime(
        store,
        readiness_provider,
    )

    assert (
        runtime._mavros_readiness_provider
        is readiness_provider
    )

    orchestrator = runtime._orchestrator

    assert (
        orchestrator.config_factory.__self__
        is store
    )

    assert (
        orchestrator.adapter.worker_command
        == (
            sys.executable,
            "-m",
            "rover_backend.rtk_worker_bootstrap",
        )
    )

    assert not store.database_file.exists()


def test_start_reconciles_persisted_stopped(
    tmp_path,
):
    store = RtkProfileStore(
        tmp_path / "stopped.sqlite3"
    )

    factory = FakeRuntimeFactory()

    lifecycle = RtkBackendLifecycle(
        store,
        lambda: False,
        runtime_factory=factory,
    )

    control = lifecycle.start()

    runtime = factory.created[0]

    assert lifecycle.started is True
    assert lifecycle.control is control

    assert runtime.start_calls == 1
    assert runtime.request_start_calls == 0
    assert runtime.request_stop_calls == 1

    assert (
        get_rtk_control_service()
        is control
    )


def test_start_restores_persisted_running_without_revision_change(
    tmp_path,
):
    store = create_running_store(
        tmp_path / "running.sqlite3"
    )

    before = store.runtime_state()

    factory = FakeRuntimeFactory()

    lifecycle = RtkBackendLifecycle(
        store,
        lambda: False,
        runtime_factory=factory,
    )

    lifecycle.start()

    runtime = factory.created[0]
    after = store.runtime_state()

    assert (
        after.desired_state
        is DesiredState.RUNNING
    )

    assert after.revision == before.revision

    assert runtime.request_start_calls == 1
    assert runtime.request_stop_calls == 0


def test_start_is_idempotent(
    tmp_path,
):
    store = RtkProfileStore(
        tmp_path / "idempotent.sqlite3"
    )

    factory = FakeRuntimeFactory()

    lifecycle = RtkBackendLifecycle(
        store,
        lambda: False,
        runtime_factory=factory,
    )

    first = lifecycle.start()
    second = lifecycle.start()

    assert first is second
    assert len(factory.created) == 1
    assert factory.created[0].start_calls == 1


def test_normal_stop_preserves_persisted_running_intent(
    tmp_path,
):
    store = create_running_store(
        tmp_path / "preserve.sqlite3"
    )

    factory = FakeRuntimeFactory()

    lifecycle = RtkBackendLifecycle(
        store,
        lambda: False,
        runtime_factory=factory,
    )

    lifecycle.start()

    runtime = factory.created[0]

    lifecycle.stop()

    assert lifecycle.started is False

    assert (
        store.runtime_state().desired_state
        is DesiredState.RUNNING
    )

    # Application shutdown uses supervisor teardown, not operator STOP.
    assert runtime.request_stop_calls == 0
    assert runtime.shutdown_calls == 1

    with pytest.raises(
        HTTPException
    ) as error:
        get_rtk_control_service()

    assert error.value.status_code == 503


def test_reconcile_failure_cleans_runtime_but_preserves_durable_intent(
    tmp_path,
):
    store = create_running_store(
        tmp_path / "reconcile.sqlite3"
    )

    factory = FakeRuntimeFactory()

    def configure(runtime):
        runtime.fail_request_start = True

    factory.configure = configure

    lifecycle = RtkBackendLifecycle(
        store,
        lambda: False,
        runtime_factory=factory,
    )

    with pytest.raises(
        RtkBackendLifecycleError
    ):
        lifecycle.start()

    runtime = factory.created[0]

    assert runtime.shutdown_calls == 1
    assert lifecycle.started is False

    assert (
        store.runtime_state().desired_state
        is DesiredState.RUNNING
    )

    with pytest.raises(
        HTTPException
    ):
        get_rtk_control_service()


def test_runtime_start_failure_gets_shutdown_cleanup(
    tmp_path,
):
    store = RtkProfileStore(
        tmp_path / "start-fail.sqlite3"
    )

    factory = FakeRuntimeFactory()

    def configure(runtime):
        runtime.fail_start = True

    factory.configure = configure

    lifecycle = RtkBackendLifecycle(
        store,
        lambda: False,
        runtime_factory=factory,
    )

    with pytest.raises(
        RtkBackendLifecycleError
    ):
        lifecycle.start()

    assert factory.created[0].shutdown_calls == 1
    assert lifecycle.started is False


def test_profile_store_initialization_failure_is_clean_unavailability(
    tmp_path,
    monkeypatch,
):
    """Persistence failure before runtime creation leaves no RTK authority."""

    store = RtkProfileStore(
        tmp_path / "unavailable.sqlite3"
    )

    factory = FakeRuntimeFactory()

    def fail_initialize():
        raise PermissionError(
            "synthetic RTK persistence permission failure"
        )

    monkeypatch.setattr(
        store,
        "initialize",
        fail_initialize,
    )

    lifecycle = RtkBackendLifecycle(
        store,
        lambda: False,
        runtime_factory=factory,
    )

    with pytest.raises(
        RtkBackendLifecycleError
    ) as caught:
        lifecycle.start()

    assert not isinstance(
        caught.value,
        RtkBackendLifecycleCleanupError,
    )

    assert lifecycle.started is False
    assert factory.created == []

    with pytest.raises(
        HTTPException
    ):
        get_rtk_control_service()


def test_startup_cleanup_failure_has_distinct_fatal_error(
    tmp_path,
):
    """Incomplete startup rollback must never be treated as degradable."""

    store = RtkProfileStore(
        tmp_path / "cleanup-fail.sqlite3"
    )

    factory = FakeRuntimeFactory()

    def configure(runtime):
        runtime.fail_start = True
        runtime.shutdown_result = False

    factory.configure = configure

    lifecycle = RtkBackendLifecycle(
        store,
        lambda: False,
        runtime_factory=factory,
    )

    with pytest.raises(
        RtkBackendLifecycleCleanupError
    ):
        lifecycle.start()

    runtime = factory.created[0]

    assert runtime.shutdown_calls == 1
    assert lifecycle.started is False

    with pytest.raises(
        HTTPException
    ):
        get_rtk_control_service()


def test_shutdown_timeout_is_failure_and_registry_is_removed(
    tmp_path,
):
    store = RtkProfileStore(
        tmp_path / "timeout.sqlite3"
    )

    factory = FakeRuntimeFactory()

    def configure(runtime):
        runtime.shutdown_result = False

    factory.configure = configure

    lifecycle = RtkBackendLifecycle(
        store,
        lambda: False,
        runtime_factory=factory,
    )

    control = lifecycle.start()
    runtime = factory.created[0]

    with pytest.raises(
        RtkBackendLifecycleError
    ):
        lifecycle.stop()

    # Failed physical shutdown retains ownership and therefore
    # prevents construction of a second supervisor.
    assert lifecycle.started is True
    assert lifecycle.control is control
    assert len(factory.created) == 1
    assert runtime.shutdown_calls == 1

    # start() remains idempotent against the same retained authority.
    assert lifecycle.start() is control
    assert len(factory.created) == 1

    # REST authority was removed before physical teardown.
    with pytest.raises(
        HTTPException
    ):
        get_rtk_control_service()

    # Retry teardown on the SAME supervisor.
    runtime.shutdown_result = True
    lifecycle.stop()

    assert lifecycle.started is False
    assert runtime.shutdown_calls == 2

def test_lifecycle_restart_builds_fresh_one_shot_runtime(
    tmp_path,
):
    store = RtkProfileStore(
        tmp_path / "restart.sqlite3"
    )

    factory = FakeRuntimeFactory()

    lifecycle = RtkBackendLifecycle(
        store,
        lambda: False,
        runtime_factory=factory,
    )

    lifecycle.start()
    lifecycle.stop()
    lifecycle.start()

    assert len(factory.created) == 2
    assert factory.created[0] is not factory.created[1]

    lifecycle.stop()


def test_partial_registry_install_failure_is_rolled_back(
    tmp_path,
):
    """Partial registry mutation must be removed during rollback."""

    store = RtkProfileStore(
        tmp_path / "install.sqlite3"
    )

    factory = FakeRuntimeFactory()

    def install_then_fail(control):
        install_rtk_control_service(
            control
        )
        raise RuntimeError(
            "synthetic post-install failure"
        )

    lifecycle = RtkBackendLifecycle(
        store,
        lambda: False,
        runtime_factory=factory,
        control_installer=install_then_fail,
    )

    with pytest.raises(
        RtkBackendLifecycleError
    ):
        lifecycle.start()

    runtime = factory.created[0]

    assert runtime.shutdown_calls == 1
    assert lifecycle.started is False

    with pytest.raises(
        HTTPException
    ):
        get_rtk_control_service()


def test_stop_is_idempotent(
    tmp_path,
):
    """Completed stop must not shut the runtime down twice."""

    store = RtkProfileStore(
        tmp_path / "stop-idempotent.sqlite3"
    )

    factory = FakeRuntimeFactory()

    lifecycle = RtkBackendLifecycle(
        store,
        lambda: False,
        runtime_factory=factory,
    )

    lifecycle.start()
    runtime = factory.created[0]

    lifecycle.stop()
    lifecycle.stop()

    assert lifecycle.started is False
    assert runtime.shutdown_calls == 1
