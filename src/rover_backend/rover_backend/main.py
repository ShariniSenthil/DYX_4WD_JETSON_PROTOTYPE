"""Application entry point for the DYX 4WD Rover Backend.

This process combines:

- FastAPI REST endpoints;
- persistent authentication;
- one active mission.csv store;
- the ROS 2 backend bridge;
- authenticated Socket.IO status updates.

The backend always starts and stops in the safe rover state:

    /emergency_stop = true
    /mission_enable = false

Path planning remains inside trajectory_generator. This module does not
calculate trajectories and does not modify mission.csv directly.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any

import uvicorn

from fastapi import FastAPI
from fastapi import Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from starlette.types import Receive
from starlette.types import Scope
from starlette.types import Send

from rover_backend.auth import authentication_store
from rover_backend.auth import auth_router
from rover_backend.beacon import rover_beacon
from rover_backend.config import client_ip_is_allowed
from rover_backend.config import settings
from rover_backend.mission_routes import mission_router
from rover_backend.mission_report import mission_report_store
from rover_backend.mission_store import MissionValidationError
from rover_backend.mission_store import mission_store
from rover_backend.realtime import make_asgi_app
from rover_backend.realtime import start_realtime
from rover_backend.realtime import stop_realtime
from rover_backend.ros_bridge import ros_bridge
from rover_backend.rtk_backend_lifecycle import (
    RtkBackendLifecycle,
    RtkBackendLifecycleCleanupError,
    RtkBackendLifecycleError,
)
from rover_backend.rtk_profile_store import rtk_profile_store
from rover_backend.rtk_routes import rtk_router
from rover_backend.spray_routes import spray_router
from rover_backend.state import rover_state
from rover_backend.system_routes import system_router

LOGGER = logging.getLogger(__name__)


rtk_backend_lifecycle = RtkBackendLifecycle(
    profile_store=rtk_profile_store,
    mavros_readiness_provider=ros_bridge.rtk_mavros_ready,
)


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------


def _configure_logging() -> None:
    """Configure backend logging without overriding ROS 2 logging."""

    level_name = str(settings.log_level).strip().upper()

    level = getattr(
        logging,
        level_name,
        logging.INFO,
    )

    logging.basicConfig(
        level=level,
        format=("%(asctime)s | %(levelname)s | " "%(name)s | %(message)s"),
    )


_configure_logging()


# ---------------------------------------------------------------------------
# FastAPI REST application
# ---------------------------------------------------------------------------


fastapi_app = FastAPI(
    title=settings.application_name,
    description=(
        "Production REST API for the DYX 4WD marking rover. "
        "Mission geometry is prepared by the ROS 2 trajectory generator."
    ),
    version=settings.application_version,
    docs_url="/api/docs",
    redoc_url=None,
    openapi_url="/api/openapi.json",
)


# React Native does not require browser CORS permission. Browser-based tools
# are allowed only when explicit origins are configured in the environment.
fastapi_app.add_middleware(
    CORSMiddleware,
    allow_origins=list(settings.cors_origins),
    allow_credentials=False,
    allow_methods=[
        "GET",
        "POST",
        "PATCH",
        "DELETE",
        "OPTIONS",
    ],
    allow_headers=[
        "Authorization",
        "Content-Type",
        "X-Rover-Token",
    ],
    expose_headers=[
        "Content-Disposition",
    ],
    max_age=600,
)


def _request_client_ip(
    request: Request,
) -> str | None:
    """Return the direct TCP peer address without trusting proxy headers."""

    if request.client is None:
        return None

    client_ip = str(request.client.host).strip()

    if client_ip == "::1":
        return "127.0.0.1"

    if client_ip.startswith("::ffff:"):
        return client_ip[7:]

    return client_ip or None


def _is_public_docs_request(request: Request) -> bool:
    """Return True for read-only Swagger/OpenAPI documentation requests.

    The PX4_DXP reference backend serves its API documentation without the
    network gate. Documentation endpoints expose no rover control, so they
    stay reachable for every client that can reach the rover.
    """

    if request.method not in {"GET", "HEAD"}:
        return False

    return request.url.path in {
        "/api/docs",
        "/api/openapi.json",
    }


@fastapi_app.middleware("http")
async def local_network_and_security_headers(
    request: Request,
    call_next: Any,
) -> Any:
    """Restrict HTTP access to configured rover networks."""

    client_ip = _request_client_ip(request)

    if not _is_public_docs_request(request) and not client_ip_is_allowed(
        client_ip,
    ):
        LOGGER.warning(
            "Rejected HTTP client outside allowed network: ip=%s path=%s",
            client_ip,
            request.url.path,
        )

        return JSONResponse(
            status_code=403,
            content={
                "success": False,
                "detail": ("Client network is not allowed."),
            },
            headers={
                "Cache-Control": "no-store",
                "X-Content-Type-Options": "nosniff",
            },
        )

    response = await call_next(request)

    response.headers["X-Content-Type-Options"] = "nosniff"

    response.headers["X-Frame-Options"] = "DENY"

    response.headers["Referrer-Policy"] = "no-referrer"

    if request.url.path.startswith("/api/"):
        response.headers["Cache-Control"] = "no-store"

    return response


fastapi_app.include_router(auth_router)

fastapi_app.include_router(system_router)

fastapi_app.include_router(rtk_router)

fastapi_app.include_router(mission_router)

fastapi_app.include_router(spray_router)


@fastapi_app.get(
    "/",
    include_in_schema=False,
)
async def root() -> dict[str, Any]:
    """Minimal local status response."""

    return {
        "success": True,
        "service": settings.service_name,
        "application": (settings.application_name),
        "version": (settings.application_version),
        "rover_id": settings.rover_id,
        "api": "/api/ping",
    }


# ---------------------------------------------------------------------------
# Process startup and shutdown
# ---------------------------------------------------------------------------


async def _best_effort_stop_ros_bridge() -> None:
    """Stop the ROS bridge without masking another shutdown error."""

    if not ros_bridge.running:
        return

    try:
        await asyncio.wait_for(
            asyncio.to_thread(ros_bridge.stop),
            timeout=5.0,
        )
    except asyncio.TimeoutError:
        LOGGER.error("Timed out while stopping the ROS bridge")
    except Exception:
        LOGGER.exception("ROS bridge shutdown failed")


async def _start_rtk_backend_degraded() -> bool:
    """Start RTK without making ordinary RTK failure backend-fatal.

    A cleanly rolled-back RTK failure leaves mission control, telemetry,
    emergency APIs and realtime services available while RTK routes remain
    unavailable.

    Cleanup failure is different: ownership may still exist, so it must
    propagate and abort backend startup.
    """

    try:
        await asyncio.to_thread(
            rtk_backend_lifecycle.start
        )

    except RtkBackendLifecycleCleanupError:
        LOGGER.exception(
            "RTK startup failed and ownership cleanup is incomplete; "
            "aborting backend startup"
        )
        raise

    except RtkBackendLifecycleError as error:
        LOGGER.exception(
            "RTK control unavailable; backend will continue without RTK"
        )

        rover_state.update(
            "rtk",
            healthy=False,
            correction_age_sec=None,
            status="CONTROL_UNAVAILABLE",
        )

        return False

    return True


async def _best_effort_stop_rtk_backend() -> None:
    """Stop/reap RTK without changing persisted operator intent."""

    if not rtk_backend_lifecycle.started:
        return

    try:
        await asyncio.wait_for(
            asyncio.to_thread(
                rtk_backend_lifecycle.stop
            ),
            timeout=10.0,
        )
    except asyncio.TimeoutError:
        LOGGER.error(
            "Timed out while stopping RTK backend lifecycle"
        )
    except Exception:
        LOGGER.exception(
            "RTK backend lifecycle shutdown failed"
        )


async def _force_safe_before_shutdown() -> None:
    """Assert the emergency stop before stopping backend resources."""

    rover_state.force_safe_runtime_state("BACKEND_SHUTDOWN")

    if not ros_bridge.running:
        return

    try:
        await asyncio.wait_for(
            asyncio.to_thread(ros_bridge.force_emergency_stop),
            timeout=2.0,
        )
    except Exception:
        LOGGER.exception("Unable to assert ROS emergency stop during shutdown")


async def startup_backend() -> None:
    """Initialize persistent state, ROS and realtime services."""

    rover_state.force_safe_runtime_state("BACKEND_STARTUP")

    rover_state.update(
        "network",
        rover_ip=settings.rover_ip,
        frontend_connected=False,
        socket_clients=0,
    )

    realtime_started = False
    ros_started = False
    rtk_started = False
    beacon_started = False

    try:
        await asyncio.to_thread(authentication_store.initialize)

        try:
            await asyncio.to_thread(mission_store.restore_state)
        except MissionValidationError as error:
            # Keep the backend available so the operator can inspect or
            # delete a corrupt stored mission. The rover remains stopped.
            LOGGER.error(
                "Stored mission validation failed: %s",
                error,
            )

            rover_state.set_mission_state(
                "ERROR",
                message=("Stored mission validation failed"),
                error=str(error),
            )

        await asyncio.to_thread(mission_report_store.restore_state)

        await asyncio.to_thread(ros_bridge.start)

        ros_started = True

        if not ros_bridge.running:
            raise RuntimeError("ROS bridge did not enter the running state")

        rtk_started = (
            await _start_rtk_backend_degraded()
        )

        await start_realtime()
        realtime_started = True

        try:
            rover_beacon.start()
            beacon_started = rover_beacon.running or not settings.beacon_enabled
        except Exception:
            LOGGER.exception(
                "UDP discovery beacon failed to start; HTTP API remains available"
            )

        rover_state.mark_backend_online(version=(settings.application_version))

        LOGGER.warning(
            "===== %s STARTED =====",
            settings.application_name,
        )

        LOGGER.warning(
            "HTTP API: http://%s:%d",
            settings.rover_ip,
            settings.backend_port,
        )

        LOGGER.warning("Startup safety: emergency_stop=true, mission_enable=false")

        LOGGER.warning(
            "Active mission source: %s",
            settings.mission_file,
        )

        if settings.beacon_enabled:
            LOGGER.warning(
                "Discovery beacon: UDP %d every %.1fs",
                settings.beacon_port,
                settings.beacon_interval_sec,
            )

    except Exception as error:
        LOGGER.exception("Backend startup failed")

        rover_state.mark_backend_offline(error=str(error))

        rover_state.force_safe_runtime_state("BACKEND_STARTUP_FAILED")

        if beacon_started or rover_beacon.running:
            try:
                rover_beacon.stop()
            except Exception:
                LOGGER.exception(
                    "UDP discovery beacon cleanup failed after startup error"
                )

        if realtime_started:
            try:
                await stop_realtime()
            except Exception:
                LOGGER.exception("Realtime cleanup failed after startup error")

        if (
            rtk_started
            or rtk_backend_lifecycle.started
        ):
            await _best_effort_stop_rtk_backend()

        if ros_started or ros_bridge.running:
            await _best_effort_stop_ros_bridge()

        raise


async def shutdown_backend() -> None:
    """Safely stop the backend and all owned resources."""

    LOGGER.warning(
        "===== %s SHUTDOWN REQUESTED =====",
        settings.application_name,
    )

    await _force_safe_before_shutdown()

    try:
        rover_beacon.stop()
    except Exception:
        LOGGER.exception("UDP discovery beacon shutdown failed")

    try:
        await stop_realtime()
    except Exception:
        LOGGER.exception("Realtime shutdown failed")

    # RTK owns a ROS publisher inside its worker. Reap it before destroying
    # the ROS bridge that supplies MAVROS endpoint readiness.
    await _best_effort_stop_rtk_backend()

    await _best_effort_stop_ros_bridge()

    rover_state.force_safe_runtime_state("BACKEND_STOPPED")

    rover_state.mark_backend_offline(error=None)

    LOGGER.warning("Backend stopped safely")


# ---------------------------------------------------------------------------
# Combined FastAPI + Socket.IO ASGI application
# ---------------------------------------------------------------------------


_socketio_app = make_asgi_app(fastapi_app)


class RoverAsgiApplication:
    """Dispatch Socket.IO and REST while owning the ASGI lifespan.

    The explicit lifespan dispatcher guarantees that backend startup and
    shutdown run even when FastAPI is wrapped by python-socketio.
    """

    def __init__(self) -> None:
        self._started = False
        self._lifecycle_lock = asyncio.Lock()

        socket_path = "/" + settings.socket_path.strip("/")

        self._socket_path = socket_path.rstrip("/") or "/socket.io"

    def _is_socket_request(
        self,
        scope: Scope,
    ) -> bool:
        if scope.get("type") not in {
            "http",
            "websocket",
        }:
            return False

        path = str(scope.get("path", ""))

        return bool(
            path == self._socket_path or path.startswith(self._socket_path + "/")
        )

    async def _startup(self) -> None:
        async with self._lifecycle_lock:
            if self._started:
                return

            await startup_backend()
            self._started = True

    async def _shutdown(self) -> None:
        async with self._lifecycle_lock:
            if not self._started:
                rover_state.force_safe_runtime_state("BACKEND_STOPPED")
                rover_state.mark_backend_offline(error=None)
                return

            try:
                await shutdown_backend()
            finally:
                self._started = False

    async def _lifespan(
        self,
        receive: Receive,
        send: Send,
    ) -> None:
        while True:
            message = await receive()
            message_type = message.get("type")

            if message_type == "lifespan.startup":
                try:
                    await self._startup()
                except Exception as error:
                    await send(
                        {
                            "type": ("lifespan.startup.failed"),
                            "message": str(error),
                        }
                    )
                    return

                await send({"type": ("lifespan.startup.complete")})

            elif message_type == "lifespan.shutdown":
                try:
                    await self._shutdown()
                except Exception as error:
                    await send(
                        {
                            "type": ("lifespan.shutdown.failed"),
                            "message": str(error),
                        }
                    )
                    return

                await send({"type": ("lifespan.shutdown.complete")})
                return

    async def __call__(
        self,
        scope: Scope,
        receive: Receive,
        send: Send,
    ) -> None:
        scope_type = scope.get("type")

        if scope_type == "lifespan":
            await self._lifespan(
                receive,
                send,
            )
            return

        if self._is_socket_request(scope):
            await _socketio_app(
                scope,
                receive,
                send,
            )
            return

        await fastapi_app(
            scope,
            receive,
            send,
        )


app = RoverAsgiApplication()


# ---------------------------------------------------------------------------
# ROS 2 console-script entry point
# ---------------------------------------------------------------------------


def main(
    args: list[str] | None = None,
) -> None:
    """Run one production backend process."""

    del args

    uvicorn.run(
        app,
        host=settings.backend_host,
        port=settings.backend_port,
        log_level=settings.log_level,
        access_log=True,
        reload=False,
        workers=1,
        lifespan="on",
        proxy_headers=False,
        server_header=False,
        timeout_keep_alive=10,
    )


if __name__ == "__main__":
    main()
