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
import time

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


class RtkBackendLifecycleCleanupError(
    RtkBackendLifecycleError
):
    """Startup failed and RTK ownership cleanup was not proven complete."""


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


# How long a cached "is the active profile direct_inject?" read may be reused
# before re-querying the profile store. The launch-readiness provider is
# polled at DEFAULT_SUPERVISOR_POLL_SEC (10 Hz); direct_inject only changes on
# an operator profile edit, which already forces desired_state to STOPPED and
# requires an explicit re-START (see rtk_control_service.update_profile), so a
# short cache cannot cause a stale launch decision that matters operationally.
_DIRECT_INJECT_MODE_CACHE_TTL_SEC = 1.0


def _make_launch_readiness_provider(
    profile_store: RtkProfileStore,
    mavros_readiness_provider: Callable[
        [],
        bool,
    ],
) -> Callable[[], bool]:
    """Wrap the MAVROS-derived readiness provider so direct-serial injection
    does not depend on PX4/MAVROS being connected.

    Direct-serial RTCM delivery writes bytes straight to the Mosaic-H over its
    own USB2 interface and has no protocol dependency on MAVROS or PX4 (see
    docs/RTCM_DIRECT_INJECTION_AB_REVIEW_PLAN.md). The original A/B design
    deliberately kept one shared launch precondition for both sides "to keep
    the A/B comparison honest" (see that document's Section 6.8) -- with both
    sides now field-validated, that symmetry is no longer needed and the
    PX4/MAVROS dependency was never really structural for the direct sink.

    Receiver-availability health for the direct sink is not re-implemented
    here: SerialRtcmSink already owns open/reopen retry and reports unhealthy
    until the Mosaic-H is actually reachable on the configured USB device, and
    that mechanism self-heals across a receiver unplug/replug without needing
    the worker to be relaunched. This wrapper only decides whether the worker
    is allowed to *start*.

    A/B-side (`direct_inject=false`) behavior is completely unchanged: the
    real `mavros_readiness_provider` is still consulted and still gates the
    launch exactly as before.
    """

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

    cache_lock = threading.Lock()
    cache_state = {
        "checked_at": float("-inf"),
        "direct_inject": False,
    }

    def _active_profile_is_direct_inject() -> bool:
        now = time.monotonic()

        with cache_lock:
            age = now - cache_state["checked_at"]
            if age < _DIRECT_INJECT_MODE_CACHE_TTL_SEC:
                return bool(
                    cache_state["direct_inject"]
                )

        # Fail closed: any read/decode problem here must fall back to the
        # MAVROS-gated path, never to an unconditional bypass.
        direct_inject = False
        try:
            runtime = profile_store.runtime_state()
            if runtime.active_profile_id is not None:
                profile = profile_store.get_profile(
                    runtime.active_profile_id
                )
                direct_inject = bool(
                    profile.direct_inject
                )
        except Exception:
            direct_inject = False

        with cache_lock:
            cache_state["checked_at"] = now
            cache_state["direct_inject"] = (
                direct_inject
            )

        return direct_inject

    def _launch_readiness_provider() -> bool:
        if _active_profile_is_direct_inject():
            return True

        return bool(
            mavros_readiness_provider()
        )

    return _launch_readiness_provider


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

    launch_readiness_provider = (
        _make_launch_readiness_provider(
            profile_store,
            mavros_readiness_provider,
        )
    )

    return RtkRuntimeService(
        orchestrator,
        launch_readiness_provider,
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

                if cleanup_failed:
                    # The backend must NOT continue operating after this
                    # failure. A supervisor, process lock, liveness FD, child,
                    # or REST authority may still be owned.
                    raise (
                        RtkBackendLifecycleCleanupError(
                            "RTK backend lifecycle "
                            "startup failed; fail-closed "
                            "runtime cleanup also failed"
                        )
                    ) from error

                # Normal RTK initialization/runtime restore failures reach this
                # path only after rollback has proven that no RTK authority is
                # left behind. Application startup may safely continue with
                # RTK control unavailable.
                raise RtkBackendLifecycleError(
                    "RTK backend lifecycle "
                    "startup failed"
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
