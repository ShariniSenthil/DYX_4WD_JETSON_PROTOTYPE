"""Backend authority joining persistent RTK intent to runtime supervision.

This layer serializes API/application RTK operations. It is the only future
REST-facing component allowed to combine profile persistence with supervisor
START/STOP intent.

Safety rules:

* STOPPED is persisted before asking runtime to stop.
* START is persisted first, then forwarded to runtime.
* A failed START is compensated back to persisted STOPPED and a best-effort
  runtime STOP.
* Profile mutations that force persisted RUNNING -> STOPPED also forward STOP
  to runtime.
* No method exposes the NTRIP password.
"""

from __future__ import annotations

import logging
import threading

from dataclasses import dataclass
from typing import Any
from typing import Protocol

from rover_backend.rtk_manager_core import (
    DesiredState,
)
from rover_backend.rtk_profile_store import (
    RtkPersistedRuntimeState,
    RtkProfileNotFoundError,
    RtkProfileSnapshot,
    RtkProfileStore,
)
from rover_backend.rtk_runtime_service import (
    RtkRuntimeServiceSnapshot,
)


LOGGER = logging.getLogger(__name__)


class RtkControlError(RuntimeError):
    """Base backend RTK authority failure."""


class RtkControlRuntimeError(
    RtkControlError
):
    """Persistence is safe but runtime command forwarding failed."""


class RtkControlConsistencyError(
    RtkControlError
):
    """Persistence/runtime compensation could not restore safe authority."""


class _RuntimeService(Protocol):
    @property
    def snapshot(
        self,
    ) -> RtkRuntimeServiceSnapshot: ...

    def request_start(
        self,
    ) -> None: ...

    def request_stop(
        self,
    ) -> None: ...


@dataclass(
    frozen=True,
    slots=True,
)
class RtkControlSnapshot:
    """Credential-free combined persisted/runtime status."""

    persisted: RtkPersistedRuntimeState
    active_profile: RtkProfileSnapshot | None
    runtime: RtkRuntimeServiceSnapshot


class RtkControlService:
    """Serialize RTK profile and lifecycle authority."""

    def __init__(
        self,
        profile_store: RtkProfileStore,
        runtime_service: _RuntimeService,
    ) -> None:
        if not isinstance(
            profile_store,
            RtkProfileStore,
        ):
            raise TypeError(
                "profile_store must be an RtkProfileStore"
            )

        self._profile_store = profile_store
        self._runtime_service = runtime_service
        self._lock = threading.RLock()

    @property
    def snapshot(
        self,
    ) -> RtkControlSnapshot:
        with self._lock:
            persisted = (
                self._profile_store.runtime_state()
            )

            active_profile = (
                self._active_profile_for_state(
                    persisted
                )
            )

            return RtkControlSnapshot(
                persisted=persisted,
                active_profile=active_profile,
                runtime=(
                    self._runtime_service.snapshot
                ),
            )

    def list_profiles(
        self,
    ) -> tuple[RtkProfileSnapshot, ...]:
        with self._lock:
            return (
                self._profile_store.list_profiles()
            )

    def get_profile(
        self,
        profile_id: int,
    ) -> RtkProfileSnapshot:
        with self._lock:
            return self._profile_store.get_profile(
                profile_id
            )

    def create_profile(
        self,
        **values: Any,
    ) -> RtkProfileSnapshot:
        with self._lock:
            return (
                self._profile_store.create_profile(
                    **values
                )
            )

    def update_profile(
        self,
        profile_id: int,
        **changes: Any,
    ) -> RtkProfileSnapshot:
        with self._lock:
            before = (
                self._profile_store.runtime_state()
            )

            updated = (
                self._profile_store.update_profile(
                    profile_id,
                    **changes,
                )
            )

            after = (
                self._profile_store.runtime_state()
            )

            self._forward_forced_stop(
                before,
                after,
                operation="profile update",
            )

            return updated

    def delete_profile(
        self,
        profile_id: int,
    ) -> None:
        with self._lock:
            before = (
                self._profile_store.runtime_state()
            )

            self._profile_store.delete_profile(
                profile_id
            )

            after = (
                self._profile_store.runtime_state()
            )

            self._forward_forced_stop(
                before,
                after,
                operation="profile delete",
            )

    def activate_profile(
        self,
        profile_id: int,
    ) -> RtkPersistedRuntimeState:
        with self._lock:
            before = (
                self._profile_store.runtime_state()
            )

            after = (
                self._profile_store.set_active_profile(
                    profile_id
                )
            )

            self._forward_forced_stop(
                before,
                after,
                operation="profile activation",
            )

            return after

    def clear_active_profile(
        self,
    ) -> RtkPersistedRuntimeState:
        with self._lock:
            before = (
                self._profile_store.runtime_state()
            )

            after = (
                self._profile_store.clear_active_profile()
            )

            self._forward_forced_stop(
                before,
                after,
                operation="active profile clear",
            )

            return after

    def request_start(
        self,
    ) -> RtkPersistedRuntimeState:
        """Persist RUNNING and forward it with fail-closed compensation."""

        with self._lock:
            before = (
                self._profile_store.runtime_state()
            )

            if (
                before.desired_state
                is DesiredState.RUNNING
            ):
                # Repeated START is reconciliation and must not increment the
                # persisted revision. But failure of this explicit START must
                # still fail closed to STOPPED.
                running = before

            else:
                running = (
                    self._profile_store.set_desired_state(
                        DesiredState.RUNNING
                    )
                )

            LOGGER.info(
                "RTK_CONTROL event=START_FORWARD "
                "profile_id=%s revision=%s",
                running.active_profile_id,
                running.revision,
            )

            try:
                self._runtime_service.request_start()

            except Exception as start_error:
                persistence_error: Exception | None = (
                    None
                )

                try:
                    self._profile_store.set_desired_state(
                        DesiredState.STOPPED
                    )
                except Exception as error:
                    persistence_error = error

                # Runtime START may have been accepted just before an error
                # or timeout reached the caller. Always issue a best-effort
                # STOP so failed explicit START cannot leave a worker active.
                try:
                    self._runtime_service.request_stop()
                except Exception:
                    pass

                LOGGER.error(
                    "RTK_CONTROL event=START_FAILED "
                    "profile_id=%s "
                    "stop_compensation_persisted=%s",
                    running.active_profile_id,
                    persistence_error is None,
                )

                if persistence_error is not None:
                    raise RtkControlConsistencyError(
                        "RTK START failed and persisted "
                        "STOPPED compensation also failed"
                    ) from persistence_error

                raise RtkControlRuntimeError(
                    "RTK START failed; persisted state "
                    "was restored to STOPPED"
                ) from start_error

            LOGGER.info(
                "RTK_CONTROL event=START_ACCEPTED "
                "profile_id=%s revision=%s",
                running.active_profile_id,
                running.revision,
            )

            return running

    def request_stop(
        self,
    ) -> RtkPersistedRuntimeState:
        """Persist STOPPED before forwarding runtime STOP."""

        with self._lock:
            stopped = (
                self._profile_store.set_desired_state(
                    DesiredState.STOPPED
                )
            )

            try:
                self._runtime_service.request_stop()

            except Exception as error:
                # Never restore RUNNING here. Persisted STOPPED is the safe
                # authoritative target and later reconciliation can retry it.
                LOGGER.error(
                    "RTK_CONTROL event=STOP_FORWARD_FAILED "
                    "profile_id=%s revision=%s",
                    stopped.active_profile_id,
                    stopped.revision,
                )

                raise RtkControlRuntimeError(
                    "RTK STOP is persisted but runtime "
                    "STOP forwarding failed"
                ) from error

            return stopped

    def reconcile_runtime(
        self,
    ) -> RtkPersistedRuntimeState:
        """Apply persisted desired state to a running supervisor."""

        with self._lock:
            persisted = (
                self._profile_store.runtime_state()
            )

            LOGGER.info(
                "RTK_CONTROL event=RECONCILE "
                "profile_id=%s revision=%s desired=%s",
                persisted.active_profile_id,
                persisted.revision,
                persisted.desired_state.value,
            )

            try:
                if (
                    persisted.desired_state
                    is DesiredState.RUNNING
                ):
                    self._runtime_service.request_start()
                else:
                    self._runtime_service.request_stop()

            except Exception as error:
                raise RtkControlRuntimeError(
                    "unable to reconcile persisted RTK "
                    "state with runtime supervisor"
                ) from error

            return persisted

    def _forward_forced_stop(
        self,
        before: RtkPersistedRuntimeState,
        after: RtkPersistedRuntimeState,
        *,
        operation: str,
    ) -> None:
        """Forward a store-generated RUNNING -> STOPPED transition."""

        if not (
            before.desired_state
            is DesiredState.RUNNING
            and after.desired_state
            is DesiredState.STOPPED
        ):
            return

        LOGGER.warning(
            "RTK_CONTROL event=FORCED_STOP "
            "operation=%s profile_id=%s revision=%s",
            operation.replace(
                " ",
                "_",
            ).upper(),
            after.active_profile_id,
            after.revision,
        )

        try:
            self._runtime_service.request_stop()

        except Exception as error:
            # Store mutation remains committed. Do not roll it back to
            # RUNNING; STOPPED remains the fail-closed authoritative target.
            raise RtkControlRuntimeError(
                f"{operation} persisted STOPPED but "
                "runtime STOP forwarding failed"
            ) from error

    def _active_profile_for_state(
        self,
        state: RtkPersistedRuntimeState,
    ) -> RtkProfileSnapshot | None:
        profile_id = state.active_profile_id

        if profile_id is None:
            return None

        try:
            return self._profile_store.get_profile(
                profile_id
            )

        except RtkProfileNotFoundError as error:
            raise RtkControlConsistencyError(
                "persisted active RTK profile "
                "does not exist"
            ) from error
