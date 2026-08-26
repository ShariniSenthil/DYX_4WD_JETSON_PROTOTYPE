"""Authenticated REST API for backend-owned RTK control.

Routes are intentionally separated from application lifecycle construction.
A later backend lifecycle phase installs exactly one RtkControlService before
registering this router with the production FastAPI application.

NTRIP passwords are write-only HTTP inputs. They are never projected into
responses, status payloads, exceptions, or public profile snapshots.
"""

from __future__ import annotations

import threading

from typing import Any
from typing import Callable
from typing import Optional

from fastapi import APIRouter
from fastapi import Depends
from fastapi import HTTPException
from fastapi import status
from pydantic import BaseModel
from pydantic import ConfigDict
from pydantic import SecretStr
from starlette.concurrency import run_in_threadpool

from rover_backend.auth import (
    AuthenticatedSession,
    require_auth,
)
from rover_backend.rtk_control_service import (
    RtkControlConsistencyError,
    RtkControlError,
    RtkControlRuntimeError,
    RtkControlService,
    RtkControlSnapshot,
)
from rover_backend.rtk_profile_store import (
    RtkPersistedRuntimeState,
    RtkProfileConflictError,
    RtkProfileNotFoundError,
    RtkProfileSnapshot,
    RtkProfileStateError,
    RtkProfileStoreError,
    RtkProfileValidationError,
)
from rover_backend.rtk_runtime_service import (
    RtkRuntimeServiceSnapshot,
)


rtk_router = APIRouter(
    prefix="/api/rtk",
    tags=["rtk"],
)


# ---------------------------------------------------------------------------
# Lifecycle-owned service registry
# ---------------------------------------------------------------------------


_registry_lock = threading.RLock()

_control_service: Optional[
    RtkControlService
] = None


def install_rtk_control_service(
    service: RtkControlService,
) -> None:
    """Install exactly one production RTK control authority."""

    if not isinstance(
        service,
        RtkControlService,
    ):
        raise TypeError(
            "service must be an RtkControlService"
        )

    global _control_service

    with _registry_lock:
        if (
            _control_service is not None
            and _control_service is not service
        ):
            raise RuntimeError(
                "RTK control service is "
                "already installed"
            )

        _control_service = service


def clear_rtk_control_service(
    service: Optional[
        RtkControlService
    ] = None,
) -> None:
    """Remove the installed authority during backend shutdown."""

    global _control_service

    with _registry_lock:
        if _control_service is None:
            return

        if (
            service is not None
            and _control_service is not service
        ):
            raise RuntimeError(
                "installed RTK control service "
                "does not match"
            )

        _control_service = None


def get_rtk_control_service(
) -> RtkControlService:
    """FastAPI dependency for the lifecycle-installed authority."""

    with _registry_lock:
        service = _control_service

    if service is None:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail={
                "code": "RTK_CONTROL_UNAVAILABLE",
                "message": (
                    "RTK control service "
                    "is not available."
                ),
            },
        )

    return service


# ---------------------------------------------------------------------------
# Write-only request models
# ---------------------------------------------------------------------------


class RtkProfileCreateRequest(BaseModel):
    """Create one persistent RTK/NTRIP profile."""

    name: Any
    caster_host: Any
    caster_port: Any
    mountpoint: Any
    username: Any

    # SecretStr prevents accidental model repr leakage. Store validation
    # remains the single authority for password length/contents.
    password: SecretStr

    rtcm_topic: Any = (
        "/mavros/gps_rtk/send_rtcm"
    )

    connect_timeout_sec: Any = 10.0
    socket_timeout_sec: Any = 1.0
    healthy_age_sec: Any = 5.0
    stale_reconnect_sec: Any = 10.0
    reconnect_delay_sec: Any = 5.0
    first_data_timeout_sec: Any = 10.0

    max_mavros_rtcm_frame_bytes: Any = 720

    enabled: Any = True

    model_config = ConfigDict(
        extra="forbid",
    )


class RtkProfileUpdateRequest(BaseModel):
    """Partial RTK profile update; password remains write-only."""

    name: Any = None
    caster_host: Any = None
    caster_port: Any = None
    mountpoint: Any = None
    username: Any = None

    password: Optional[
        SecretStr
    ] = None

    rtcm_topic: Any = None

    connect_timeout_sec: Any = None
    socket_timeout_sec: Any = None
    healthy_age_sec: Any = None
    stale_reconnect_sec: Any = None
    reconnect_delay_sec: Any = None
    first_data_timeout_sec: Any = None

    max_mavros_rtcm_frame_bytes: Any = None

    enabled: Any = None

    model_config = ConfigDict(
        extra="forbid",
    )


def _create_values(
    body: RtkProfileCreateRequest,
) -> dict[str, Any]:
    values = body.model_dump()

    values["password"] = (
        body.password.get_secret_value()
    )

    return values


def _update_values(
    body: RtkProfileUpdateRequest,
) -> dict[str, Any]:
    values = body.model_dump(
        exclude_unset=True
    )

    if "password" in values:
        if body.password is None:
            # Explicit null has the same safe meaning as omission:
            # preserve the existing password.
            values.pop(
                "password",
                None,
            )
        else:
            values["password"] = (
                body.password
                .get_secret_value()
            )

    return values


# ---------------------------------------------------------------------------
# Credential-free response projection
# ---------------------------------------------------------------------------


def _profile_payload(
    profile: RtkProfileSnapshot,
) -> dict[str, Any]:
    return {
        "id": profile.profile_id,
        "name": profile.name,
        "caster_host": profile.caster_host,
        "caster_port": profile.caster_port,
        "mountpoint": profile.mountpoint,
        "username": profile.username,
        "password_configured": (
            profile.password_configured
        ),
        "rtcm_topic": profile.rtcm_topic,
        "connect_timeout_sec": (
            profile.connect_timeout_sec
        ),
        "socket_timeout_sec": (
            profile.socket_timeout_sec
        ),
        "healthy_age_sec": (
            profile.healthy_age_sec
        ),
        "stale_reconnect_sec": (
            profile.stale_reconnect_sec
        ),
        "reconnect_delay_sec": (
            profile.reconnect_delay_sec
        ),
        "first_data_timeout_sec": (
            profile.first_data_timeout_sec
        ),
        "max_mavros_rtcm_frame_bytes": (
            profile.max_mavros_rtcm_frame_bytes
        ),
        "enabled": profile.enabled,
        "revision": profile.revision,
        "created_at_epoch": (
            profile.created_at_epoch
        ),
        "updated_at_epoch": (
            profile.updated_at_epoch
        ),
    }


def _persisted_payload(
    persisted: RtkPersistedRuntimeState,
) -> dict[str, Any]:
    return {
        "active_profile_id": (
            persisted.active_profile_id
        ),
        "desired_state": (
            persisted.desired_state.value
        ),
        "revision": persisted.revision,
        "updated_at_epoch": (
            persisted.updated_at_epoch
        ),
    }


def _runtime_payload(
    service: RtkRuntimeServiceSnapshot,
) -> dict[str, Any]:
    runtime = service.runtime

    payload: dict[str, Any] = {
        "supervisor": {
            "running": service.running,
            "shutdown_requested": (
                service.shutdown_requested
            ),
            "mavros_ready": (
                service.mavros_ready
            ),
            "last_error_code": (
                service.last_error_code
            ),
        },
        "manager": None,
        "process": None,
        "last_worker_status": None,
        "last_process_returncode": None,
        "last_protocol_fault_run_id": None,
    }

    if runtime is None:
        return payload

    manager = runtime.manager

    payload["manager"] = {
        "desired_state": (
            manager.desired_state.value
        ),
        "state": (
            manager.manager_state.value
        ),
        "mavros_ready": (
            manager.mavros_ready
        ),
        "active_run_id": (
            manager.active_run_id
        ),
        "child_started": (
            manager.child_started
        ),
        "child_ready": (
            manager.child_ready
        ),
        "next_restart_at_monotonic_sec": (
            manager.next_restart_at
        ),
        "consecutive_failures": (
            manager.consecutive_failures
        ),
        "restart_count_in_window": (
            manager.restart_count_in_window
        ),
        "error_reason": (
            None
            if manager.error_reason is None
            else manager.error_reason.value
        ),
    }

    process = runtime.process

    payload["process"] = {
        "active_run_id": (
            process.active_run_id
        ),
        "pid": process.pid,
        "stop_requested": (
            process.stop_requested
        ),
        "stop_deadline_monotonic_sec": (
            process.stop_deadline
        ),
        "kill_sent": process.kill_sent,
        "exit_reported": (
            process.exit_reported
        ),
    }

    worker_status = (
        runtime.last_worker_status
    )

    if worker_status is not None:
        payload["last_worker_status"] = {
            "run_id": worker_status.run_id,
            "kind": (
                worker_status.kind.value
            ),
            "detail_code": (
                worker_status.detail_code
            ),
        }

    payload["last_process_returncode"] = (
        runtime.last_process_returncode
    )

    payload["last_protocol_fault_run_id"] = (
        runtime.last_protocol_fault_run_id
    )

    return payload


def _control_payload(
    snapshot: RtkControlSnapshot,
) -> dict[str, Any]:
    return {
        "persisted": _persisted_payload(
            snapshot.persisted
        ),
        "active_profile": (
            None
            if snapshot.active_profile is None
            else _profile_payload(
                snapshot.active_profile
            )
        ),
        "runtime": _runtime_payload(
            snapshot.runtime
        ),
    }


# ---------------------------------------------------------------------------
# Domain-to-HTTP error mapping
# ---------------------------------------------------------------------------


def _raise_http_for_domain_error(
    error: Exception,
) -> None:
    if isinstance(
        error,
        RtkProfileValidationError,
    ):
        raise HTTPException(
            status_code=(
                status.HTTP_422_UNPROCESSABLE_CONTENT
            ),
            detail={
                "code": "RTK_PROFILE_INVALID",
                "message": str(error),
            },
        ) from error

    if isinstance(
        error,
        RtkProfileNotFoundError,
    ):
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "code": "RTK_PROFILE_NOT_FOUND",
                "message": (
                    "RTK profile not found."
                ),
            },
        ) from error

    if isinstance(
        error,
        RtkProfileConflictError,
    ):
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail={
                "code": "RTK_PROFILE_CONFLICT",
                "message": (
                    "RTK profile name "
                    "already exists."
                ),
            },
        ) from error

    if isinstance(
        error,
        RtkProfileStateError,
    ):
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail={
                "code": "RTK_STATE_CONFLICT",
                "message": str(error),
            },
        ) from error

    if isinstance(
        error,
        RtkControlConsistencyError,
    ):
        raise HTTPException(
            status_code=(
                status.HTTP_503_SERVICE_UNAVAILABLE
            ),
            detail={
                "code": (
                    "RTK_CONTROL_INCONSISTENT"
                ),
                "message": (
                    "RTK control state "
                    "is inconsistent."
                ),
            },
        ) from error

    if isinstance(
        error,
        RtkControlRuntimeError,
    ):
        raise HTTPException(
            status_code=(
                status.HTTP_503_SERVICE_UNAVAILABLE
            ),
            detail={
                "code": "RTK_RUNTIME_UNAVAILABLE",
                "message": (
                    "RTK runtime command "
                    "could not be completed."
                ),
            },
        ) from error

    if isinstance(
        error,
        RtkProfileStoreError,
    ):
        raise HTTPException(
            status_code=(
                status.HTTP_503_SERVICE_UNAVAILABLE
            ),
            detail={
                "code": (
                    "RTK_PERSISTENCE_UNAVAILABLE"
                ),
                "message": (
                    "RTK persistence "
                    "is unavailable."
                ),
            },
        ) from error

    if isinstance(
        error,
        RtkControlError,
    ):
        raise HTTPException(
            status_code=(
                status.HTTP_503_SERVICE_UNAVAILABLE
            ),
            detail={
                "code": "RTK_CONTROL_FAILED",
                "message": (
                    "RTK control operation "
                    "could not be completed."
                ),
            },
        ) from error

    raise error


async def _control_call(
    operation: Callable[[], Any],
) -> Any:
    try:
        return await run_in_threadpool(
            operation
        )

    except (
        RtkProfileStoreError,
        RtkControlError,
    ) as error:
        _raise_http_for_domain_error(
            error
        )

    raise AssertionError(
        "unreachable RTK control error path"
    )


# ---------------------------------------------------------------------------
# Profile routes
# ---------------------------------------------------------------------------


@rtk_router.get(
    "/profiles",
)
async def list_rtk_profiles(
    _session: AuthenticatedSession = Depends(
        require_auth
    ),
    control: RtkControlService = Depends(
        get_rtk_control_service
    ),
) -> dict[str, Any]:
    profiles = await _control_call(
        control.list_profiles
    )

    return {
        "profiles": [
            _profile_payload(
                profile
            )
            for profile in profiles
        ],
        "count": len(profiles),
    }


@rtk_router.post(
    "/profiles",
    status_code=status.HTTP_201_CREATED,
)
async def create_rtk_profile(
    body: RtkProfileCreateRequest,
    _session: AuthenticatedSession = Depends(
        require_auth
    ),
    control: RtkControlService = Depends(
        get_rtk_control_service
    ),
) -> dict[str, Any]:
    values = _create_values(
        body
    )

    profile = await _control_call(
        lambda: control.create_profile(
            **values
        )
    )

    return {
        "profile": _profile_payload(
            profile
        ),
    }


@rtk_router.get(
    "/profiles/{profile_id}",
)
async def get_rtk_profile(
    profile_id: int,
    _session: AuthenticatedSession = Depends(
        require_auth
    ),
    control: RtkControlService = Depends(
        get_rtk_control_service
    ),
) -> dict[str, Any]:
    profile = await _control_call(
        lambda: control.get_profile(
            profile_id
        )
    )

    return {
        "profile": _profile_payload(
            profile
        ),
    }


@rtk_router.patch(
    "/profiles/{profile_id}",
)
async def update_rtk_profile(
    profile_id: int,
    body: RtkProfileUpdateRequest,
    _session: AuthenticatedSession = Depends(
        require_auth
    ),
    control: RtkControlService = Depends(
        get_rtk_control_service
    ),
) -> dict[str, Any]:
    changes = _update_values(
        body
    )

    profile = await _control_call(
        lambda: control.update_profile(
            profile_id,
            **changes,
        )
    )

    return {
        "profile": _profile_payload(
            profile
        ),
    }


@rtk_router.delete(
    "/profiles/{profile_id}",
)
async def delete_rtk_profile(
    profile_id: int,
    _session: AuthenticatedSession = Depends(
        require_auth
    ),
    control: RtkControlService = Depends(
        get_rtk_control_service
    ),
) -> dict[str, Any]:
    await _control_call(
        lambda: control.delete_profile(
            profile_id
        )
    )

    return {
        "success": True,
        "deleted_profile_id": profile_id,
    }


@rtk_router.post(
    "/profiles/{profile_id}/activate",
)
async def activate_rtk_profile(
    profile_id: int,
    _session: AuthenticatedSession = Depends(
        require_auth
    ),
    control: RtkControlService = Depends(
        get_rtk_control_service
    ),
) -> dict[str, Any]:
    persisted = await _control_call(
        lambda: control.activate_profile(
            profile_id
        )
    )

    return {
        "success": True,
        "persisted": _persisted_payload(
            persisted
        ),
    }


@rtk_router.delete(
    "/active-profile",
)
async def clear_active_rtk_profile(
    _session: AuthenticatedSession = Depends(
        require_auth
    ),
    control: RtkControlService = Depends(
        get_rtk_control_service
    ),
) -> dict[str, Any]:
    persisted = await _control_call(
        control.clear_active_profile
    )

    return {
        "success": True,
        "persisted": _persisted_payload(
            persisted
        ),
    }


# ---------------------------------------------------------------------------
# Runtime routes
# ---------------------------------------------------------------------------


@rtk_router.get(
    "/status",
)
async def read_rtk_status(
    _session: AuthenticatedSession = Depends(
        require_auth
    ),
    control: RtkControlService = Depends(
        get_rtk_control_service
    ),
) -> dict[str, Any]:
    snapshot = await _control_call(
        lambda: control.snapshot
    )

    return {
        "status": _control_payload(
            snapshot
        ),
    }


@rtk_router.post(
    "/start",
)
async def start_rtk(
    _session: AuthenticatedSession = Depends(
        require_auth
    ),
    control: RtkControlService = Depends(
        get_rtk_control_service
    ),
) -> dict[str, Any]:
    persisted = await _control_call(
        control.request_start
    )

    return {
        "success": True,
        "message": (
            "RTK RUNNING intent accepted."
        ),
        "persisted": _persisted_payload(
            persisted
        ),
    }


@rtk_router.post(
    "/stop",
)
async def stop_rtk(
    _session: AuthenticatedSession = Depends(
        require_auth
    ),
    control: RtkControlService = Depends(
        get_rtk_control_service
    ),
) -> dict[str, Any]:
    persisted = await _control_call(
        control.request_stop
    )

    return {
        "success": True,
        "message": (
            "RTK STOPPED intent accepted."
        ),
        "persisted": _persisted_payload(
            persisted
        ),
    }
