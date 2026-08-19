"""Mission REST API for the DYX 4WD Rover Backend.

The production system maintains exactly one active mission source file:

    /home/flash/rover_ws/missions/mission.csv

Responsibilities of this module:

- accept and authenticate frontend mission uploads;
- validate and atomically replace the single active mission.csv;
- request trajectory preparation after every successful upload;
- expose mission status and a bounded prepared-path preview;
- download or delete the active mission.csv;
- forward Start, Pause, Resume, Next Point, Skip Point, Stop and Clear
  commands to the ROS bridge.

This module does not calculate dummy points, interpolation points or path
geometry. All path planning remains inside trajectory_generator.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any
from typing import Callable

from fastapi import APIRouter
from fastapi import Depends
from fastapi import File
from fastapi import Form
from fastapi import HTTPException
from fastapi import UploadFile
from fastapi.responses import FileResponse
from pydantic import BaseModel
from starlette.concurrency import run_in_threadpool

from rover_backend.auth import AuthenticatedSession
from rover_backend.auth import require_auth
from rover_backend.config import settings
from rover_backend.mission_store import MissionValidationError
from rover_backend.mission_store import mission_store
from rover_backend.ros_bridge import ros_bridge
from rover_backend.state import rover_state

mission_router = APIRouter(
    prefix="/api/mission",
    tags=["mission"],
)


ACTIVE_MISSION_STATES = {
    "RUNNING",
    "PAUSED",
    "WAITING_FOR_NEXT",
}


CONTROL_CONFLICT_STATES = {
    "PREPARING",
}


class ExecutionModeRequest(BaseModel):
    execution_mode: str


def _mission_state() -> dict[str, Any]:
    return rover_state.section("mission")


def _normalised_state() -> str:
    return str(_mission_state().get("state", "EMPTY")).strip().upper()


def _require_not_active(
    *,
    operation: str,
) -> None:
    state_name = _normalised_state()
    safety = rover_state.section("safety")

    if bool(safety.get("mission_enable", False)):
        raise HTTPException(
            status_code=409,
            detail=(
                f"Cannot {operation} while rover movement is enabled. "
                "Stop the mission first."
            ),
        )

    if state_name in ACTIVE_MISSION_STATES:
        raise HTTPException(
            status_code=409,
            detail=(
                f"Cannot {operation} while the mission is "
                f"{state_name.lower()}. Stop the mission first."
            ),
        )

    if state_name in CONTROL_CONFLICT_STATES:
        raise HTTPException(
            status_code=409,
            detail=(
                f"Cannot {operation} while trajectory preparation " "is in progress."
            ),
        )


def _require_ros_bridge() -> None:
    if not ros_bridge.running:
        raise HTTPException(
            status_code=503,
            detail="The rover ROS bridge is not running.",
        )


def _control_error(
    *,
    operation: str,
    error: Exception,
) -> HTTPException:
    return HTTPException(
        status_code=409,
        detail={
            "success": False,
            "operation": operation,
            "message": str(error),
            "mission": _mission_state(),
        },
    )


async def _run_ros_operation(
    operation_name: str,
    operation: Callable[[], dict[str, Any]],
) -> dict[str, Any]:
    _require_ros_bridge()

    try:
        mission = await run_in_threadpool(operation)
    except RuntimeError as error:
        raise _control_error(
            operation=operation_name,
            error=error,
        ) from error

    return {
        "success": True,
        "operation": operation_name,
        "mission": mission,
    }


@mission_router.post("/upload")
async def upload_mission(
    file: UploadFile = File(...),
    extension_mode: str = Form(...),
    dummy_point_distance_m: float | None = Form(None),
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Store one validated CSV and prepare its trajectory automatically.

    extension_mode:
        ENABLE or DISABLE.

    ENABLE:
        The trajectory generator automatically checks each consecutive
        marking-point transition. When the distance is below the configured
        2.0 m row-transition threshold, it creates one navigation-only dummy
        point behind the next marking point using the next-row direction.

        dummy_point_distance_m is supplied by the frontend when the operator
        changes it. When omitted, the configured production fallback is used.

    DISABLE:
        No dummy points are generated. Only uploaded CSV points are marking
        points; interpolation may still exist in the ROS navigation path.
    """

    _require_not_active(operation="upload a new mission")

    try:
        raw_bytes = await file.read(settings.maximum_upload_bytes + 1)
    finally:
        await file.close()

    if len(raw_bytes) > settings.maximum_upload_bytes:
        maximum_mb = settings.maximum_upload_bytes / (1024 * 1024)

        raise HTTPException(
            status_code=413,
            detail=("The uploaded CSV exceeds " f"{maximum_mb:.1f} MB."),
        )

    try:
        metadata = await run_in_threadpool(
            mission_store.save,
            raw_bytes=raw_bytes,
            filename=(file.filename or "mission.csv"),
            extension_mode=extension_mode,
            dummy_point_distance_m=(dummy_point_distance_m),
        )
    except MissionValidationError as error:
        raise HTTPException(
            status_code=422,
            detail=str(error),
        ) from error
    except OSError as error:
        raise HTTPException(
            status_code=500,
            detail=("The mission could not be stored safely: " f"{error}"),
        ) from error

    # A valid upload always triggers trajectory calculation. The rover is not
    # started here; it remains in the safe non-driving condition until Start.
    try:
        _require_ros_bridge()
        mission = await run_in_threadpool(ros_bridge.prepare_trajectory)
    except HTTPException:
        rover_state.set_mission_state(
            "LOADED",
            message=(
                "mission.csv stored; trajectory preparation is waiting "
                "for the ROS bridge"
            ),
            error=None,
        )

        raise HTTPException(
            status_code=503,
            detail={
                "success": False,
                "mission_stored": True,
                "message": (
                    "mission.csv was stored successfully, but the ROS "
                    "bridge is not running, so trajectory preparation "
                    "could not start."
                ),
                "upload": metadata,
                "mission": _mission_state(),
            },
        )
    except RuntimeError as error:
        raise HTTPException(
            status_code=422,
            detail={
                "success": False,
                "mission_stored": True,
                "message": str(error),
                "upload": metadata,
                "mission": _mission_state(),
            },
        ) from error

    return {
        "success": True,
        "message": ("Mission uploaded; trajectory preparation started and will complete automatically when RTK is FIXED."),
        "upload": metadata,
        "mission": mission,
    }


@mission_router.post("/prepare")
async def prepare_mission(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Re-read the stored mission.csv and prepare the trajectory again."""

    _require_not_active(operation="prepare the mission")

    try:
        await run_in_threadpool(mission_store.load_metadata)
    except MissionValidationError as error:
        raise HTTPException(
            status_code=409,
            detail=str(error),
        ) from error

    return await _run_ros_operation(
        "prepare",
        ros_bridge.prepare_trajectory,
    )


@mission_router.get("/status")
def mission_status(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    return {
        "success": True,
        "mission": _mission_state(),
    }


@mission_router.get("/loaded-path")
def loaded_path(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Return the bounded navigation-path preview maintained by ros_bridge."""

    mission = _mission_state()

    return {
        "success": True,
        "frame_id": mission.get("path_frame_id"),
        "navigation_point_count": mission.get(
            "navigation_point_count",
            0,
        ),
        "preview_truncated": bool(
            mission.get(
                "navigation_path_preview_truncated",
                False,
            )
        ),
        "points": mission.get(
            "navigation_path_preview",
            [],
        ),
    }


@mission_router.get("/file")
def download_mission_file(
    _session: AuthenticatedSession = Depends(require_auth),
) -> FileResponse:
    mission_path = Path(settings.mission_file)

    if not mission_path.is_file():
        raise HTTPException(
            status_code=404,
            detail="No mission.csv is stored.",
        )

    return FileResponse(
        path=str(mission_path),
        media_type="text/csv; charset=utf-8",
        filename="mission.csv",
    )


@mission_router.delete("/file")
async def delete_mission_file(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Delete the single mission.csv after clearing prepared ROS state."""

    _require_not_active(operation="delete mission.csv")

    # Clear any latched prepared trajectory before deleting its source file.
    if ros_bridge.running:
        try:
            await run_in_threadpool(ros_bridge.clear_mission)
        except RuntimeError as error:
            raise HTTPException(
                status_code=409,
                detail=(
                    "Unable to clear the prepared ROS trajectory; "
                    "mission.csv was not deleted: "
                    f"{error}"
                ),
            ) from error

    try:
        existed = await run_in_threadpool(mission_store.delete)
    except RuntimeError as error:
        raise HTTPException(
            status_code=500,
            detail=str(error),
        ) from error

    return {
        "success": True,
        "deleted": existed,
        "message": (
            "mission.csv deleted successfully."
            if existed
            else "No mission.csv was stored."
        ),
        "mission": _mission_state(),
    }


@mission_router.post("/execution-mode")
async def set_execution_mode(
    request: ExecutionModeRequest,
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Select AUTO or operator-stepped MANUAL mission execution."""

    mode = str(request.execution_mode).strip().upper()

    if mode not in {
        "AUTO",
        "MANUAL",
    }:
        raise HTTPException(
            status_code=422,
            detail=("execution_mode must be AUTO or MANUAL"),
        )

    return await _run_ros_operation(
        "set-execution-mode",
        lambda: ros_bridge.set_execution_mode(mode),
    )


@mission_router.post("/start")
async def start_mission(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    return await _run_ros_operation(
        "start",
        ros_bridge.start_mission,
    )


@mission_router.post("/pause")
async def pause_mission(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    return await _run_ros_operation(
        "pause",
        ros_bridge.pause_mission,
    )


@mission_router.post("/resume")
async def resume_mission(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    return await _run_ros_operation(
        "resume",
        ros_bridge.resume_mission,
    )


@mission_router.post("/next-point")
async def next_point(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    return await _run_ros_operation(
        "next-point",
        ros_bridge.next_point,
    )


@mission_router.post("/skip-point")
async def skip_point(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    return await _run_ros_operation(
        "skip-point",
        ros_bridge.skip_point,
    )


@mission_router.post("/stop")
async def stop_mission(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    return await _run_ros_operation(
        "stop",
        ros_bridge.stop_mission,
    )


@mission_router.post("/clear")
async def clear_mission(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Clear generated ROS paths and progress while retaining mission.csv."""

    return await _run_ros_operation(
        "clear",
        ros_bridge.clear_mission,
    )