"""Production assembly and application lifecycle for backend-owned RTK.

This layer joins the already-frozen RTK components into exactly one production
ownership chain:

    profile store
        -> manager core
        -> process adapter
        -> runtime orchestrator
        -> runtime supervisor
        -> control service
        -> authenticated REST registry

Normal backend shutdown intentionally preserves the persisted desired state.
Only an explicit operator STOP changes persisted RUNNING to STOPPED.
"""

from __future__ import annotations

import math
import sys
import threading

from typing import Callable
from typing import Protocol

from rover_backend.rtk_control_service import (
    RtkControlService,
)
from rover_backend.rtk_manager_core import (
    RtkManagerCore,
)
from rover_backend.rtk_process_adapter import (
    RtkProcessAdapter,
)
from rover_backend.rtk_profile_store import (
    RtkProfileStore,
)
from rover_backend.rtk_routes import (
    clear_rtk_control_service,
    install_rtk_control_service,
)
from rover_backend.rtk_runtime_orchestrator import (
    RtkRuntimeOrchestrator,
)
from rover_backend.rtk_runtime_service import (
    DEFAULT_SHUTDOWN_TIMEOUT_SEC,
    RtkRuntimeService,
    RtkRuntimeServiceSnapshot,
)


class RtkBackendLifecycleError(
    RuntimeError
):
    """Production RTK application lifecycle failed."""


class _RuntimeService(Protocol):
    @property
    def snapshot(
        self,
    ) -> RtkRuntimeServiceSnapshot: ...

    def start(
        self,
    ) -> None: ...

    def request_start(
        self,
    ) -> None: ...

    def request_stop(
        self,
    ) -> None: ...

    def shutdown(
        self,
        timeout_sec: float = (
            DEFAULT_SHUTDOWN_TIMEOUT_SEC
        ),
    ) -> bool: ...


RuntimeFactory = Callable[
    [
        RtkProfileStore,
        Callable[[], bool],
    ],
    _RuntimeService,
]


def build_production_runtime(
    profile_store: RtkProfileStore,
    mavros_readiness_provider: Callable[
        [],
        bool,
    ],
) -> RtkRuntimeService:
    """Construct the single production RTK supervisor without starting it."""

    if not isinstance(
        profile_store,
        RtkProfileStore,
    ):
        raise TypeError(
            "profile_store must be an RtkProfileStore"
        )

    if not callable(
        mavros_readiness_provider
    ):
        raise TypeError(
            "mavros_readiness_provider must be callable"
        )

    core = RtkManagerCore()

    adapter = RtkProcessAdapter(
        (
            sys.executable,
            "-m",
            "rover_backend.rtk_worker_bootstrap",
        )
    )

    orchestrator = RtkRuntimeOrchestrator(
        core,
        adapter,
        profile_store.build_active_worker_config,
    )

    return RtkRuntimeService(
        orchestrator,
        mavros_readiness_provider,
    )


class RtkBackendLifecycle:
    """Own production RTK construction, restore and teardown."""

    def __init__(
        self,
        profile_store: RtkProfileStore,
        mavros_readiness_provider: Callable[
            [],
            bool,
        ],
        *,
        runtime_factory: RuntimeFactory = (
            build_production_runtime
        ),
        control_installer: Callable[
            [RtkControlService],
            None,
        ] = install_rtk_control_service,
        control_clearer: Callable[
            [RtkControlService],
            None,
        ] = clear_rtk_control_service,
        shutdown_timeout_sec: float = (
            DEFAULT_SHUTDOWN_TIMEOUT_SEC
        ),
    ) -> None:
        if not isinstance(
            profile_store,
            RtkProfileStore,
        ):
            raise TypeError(
                "profile_store must be an RtkProfileStore"
            )

        if not callable(
            mavros_readiness_provider
        ):
            raise TypeError(
                "mavros_readiness_provider must be callable"
            )

        if not callable(runtime_factory):
            raise TypeError(
                "runtime_factory must be callable"
            )

        if not callable(control_installer):
            raise TypeError(
                "control_installer must be callable"
            )

        if not callable(control_clearer):
            raise TypeError(
                "control_clearer must be callable"
            )

        if (
            isinstance(
                shutdown_timeout_sec,
                bool,
            )
            or not isinstance(
                shutdown_timeout_sec,
                (int, float),
            )
        ):
            raise TypeError(
                "shutdown_timeout_sec must be "
                "a finite number > 0"
            )

        timeout = float(
            shutdown_timeout_sec
        )

        if (
            not math.isfinite(timeout)
            or timeout <= 0.0
        ):
            raise ValueError(
                "shutdown_timeout_sec must be "
                "a finite number > 0"
            )

        self._profile_store = profile_store

        self._mavros_readiness_provider = (
            mavros_readiness_provider
        )

        self._runtime_factory = runtime_factory
        self._control_installer = (
            control_installer
        )
        self._control_clearer = (
            control_clearer
        )

        self._shutdown_timeout_sec = timeout

        self._lock = threading.RLock()

        self._runtime: (
            _RuntimeService | None
        ) = None

        self._control: (
            RtkControlService | None
        ) = None

        self._started = False

    @property
    def started(
        self,
    ) -> bool:
        with self._lock:
            return self._started

    @property
    def control(
        self,
    ) -> RtkControlService:
        with self._lock:
            if (
                not self._started
                or self._control is None
            ):
                raise RtkBackendLifecycleError(
                    "RTK backend lifecycle "
                    "is not started"
                )

            return self._control

    def start(
        self,
    ) -> RtkControlService:
        """Initialize persistence, start supervisor and restore desired state."""

        with self._lock:
            if self._started:
                if self._control is None:
                    raise RtkBackendLifecycleError(
                        "RTK lifecycle started "
                        "without control service"
                    )

                return self._control

            runtime: (
                _RuntimeService | None
            ) = None

            control: (
                RtkControlService | None
            ) = None

            try:
                self._profile_store.initialize()

                runtime = self._runtime_factory(
                    self._profile_store,
                    self._mavros_readiness_provider,
                )

                control = RtkControlService(
                    self._profile_store,
                    runtime,
                )

                runtime.start()

                # Restore durable operator intent only after the supervisor
                # owns its manager/process authority.
                control.reconcile_runtime()

                # Expose REST authority only after runtime start + restore
                # succeeded.
                self._control_installer(
                    control
                )

            except Exception as error:
                cleanup_failed = False

                if control is not None:
                    try:
                        self._control_clearer(
                            control
                        )
                    except Exception:
                        cleanup_failed = True

                if runtime is not None:
                    try:
                        stopped = runtime.shutdown(
                            timeout_sec=(
                                self._shutdown_timeout_sec
                            )
                        )

                        if not stopped:
                            cleanup_failed = True

                    except Exception:
                        cleanup_failed = True

                message = (
                    "RTK backend lifecycle "
                    "startup failed"
                )

                if cleanup_failed:
                    message += (
                        "; fail-closed runtime "
                        "cleanup also failed"
                    )

                raise RtkBackendLifecycleError(
                    message
                ) from error

            if (
                runtime is None
                or control is None
            ):
                raise RtkBackendLifecycleError(
                    "RTK startup completed "
                    "without runtime authority"
                )

            self._runtime = runtime
            self._control = control
            self._started = True

            return control

    def stop(
        self,
    ) -> None:
        """Remove REST authority and stop/reap runtime.

        Persisted desired state is intentionally untouched.
        Ownership is released only after both REST teardown and
        supervisor shutdown have positively completed.
        """

        with self._lock:
            if not self._started:
                return

            runtime = self._runtime
            control = self._control

            clear_error: Exception | None = None
            shutdown_error: Exception | None = None
            shutdown_ok = True

            # Remove public authority first so new API commands cannot
            # race physical supervisor teardown.
            if control is not None:
                try:
                    self._control_clearer(
                        control
                    )
                except Exception as error:
                    clear_error = error

            if runtime is not None:
                try:
                    shutdown_ok = runtime.shutdown(
                        timeout_sec=(
                            self._shutdown_timeout_sec
                        )
                    )
                except Exception as error:
                    shutdown_error = error
                    shutdown_ok = False

            # False/exception means the supervisor may still own its
            # thread, manager lock, child process, or parent FDs.
            # Retain lifecycle ownership and block another start().
            if (
                shutdown_error is not None
                or not shutdown_ok
            ):
                raise RtkBackendLifecycleError(
                    "RTK runtime did not shut "
                    "down cleanly"
                ) from shutdown_error

            # A registry-clear failure can leave a stale public service.
            # Retain ownership here too so start() cannot construct a
            # replacement authority until cleanup succeeds.
            if clear_error is not None:
                raise RtkBackendLifecycleError(
                    "RTK REST authority could "
                    "not be cleared"
                ) from clear_error

            # Both ownership layers are now positively released.
            self._runtime = None
            self._control = None
            self._started = False
