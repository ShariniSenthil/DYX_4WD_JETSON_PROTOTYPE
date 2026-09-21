"""Mission REST API for the DYX 4WD Rover Backend.

The production system maintains exactly one active mission source file:

    /home/flash/rover_ws/missions/mission.csv

Responsibilities of this module:

- accept and authenticate frontend mission uploads;
- validate and atomically replace the single active mission.csv;
- request trajectory preparation after every successful upload (preview only);
- confirm a generated preview for START via POST /load;
- archive completed missions and restore/start them by mission_id;
- expose mission status and a bounded prepared-path preview;
- download or delete the active mission.csv;
- forward Start, Pause, Resume, Next Point, Skip Point, Stop and Clear
  commands to the ROS bridge.

This module does not calculate dummy points, interpolation points or path
geometry. All path planning remains inside trajectory_generator.
"""

from __future__ import annotations

import asyncio

from pathlib import Path
from typing import Any
from typing import AsyncIterator
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
from rover_backend.mission_report import MissionReportError
from rover_backend.mission_report import mission_report_store
from rover_backend.mission_store import MissionValidationError
from rover_backend.mission_store import mission_store
from rover_backend.ros_bridge import RosServiceOutcomeUnknownError
from rover_backend.ros_bridge import ros_bridge
from rover_backend.state import rover_state
from rover_backend.trajectory_push import legacy_points
from rover_backend.trajectory_push import push_enabled
from rover_backend.trajectory_push import trajectory_snapshot

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


_mission_mutation_lock = asyncio.Lock()


async def _serialize_mission_mutation() -> AsyncIterator[None]:
    """Keep mission file checks and ROS control mutations in one transaction."""

    async with _mission_mutation_lock:
        yield


class ExecutionModeRequest(BaseModel):
    execution_mode: str


class MissionIdRequest(BaseModel):
    """Optional mission identity used by restore/load/start retries."""

    mission_id: str | None = None


def _mission_state() -> dict[str, Any]:
    mission = rover_state.section("mission")

    # Derived, never stored separately -- cannot disagree with
    # accepted_for_start/mission_id. Kept in sync with
    # system_routes.build_mission_status_payload().
    staged = bool(mission.get("accepted_for_start", False))
    mission["staged"] = staged
    mission["staged_id"] = mission.get("mission_id") if staged else None

    return mission


def _normalised_state() -> str:
    return str(_mission_state().get("state", "EMPTY")).strip().upper()


def _require_not_active(
    *,
    operation: str,
    include_preparing: bool = True,
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

    if include_preparing and state_name in CONTROL_CONFLICT_STATES:
        raise HTTPException(
            status_code=409,
            detail=(
                f"Cannot {operation} while trajectory preparation " "is in progress."
            ),
        )


def _require_not_driving(
    *,
    operation: str,
) -> None:
    _require_not_active(operation=operation, include_preparing=False)


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
    outcome_unknown = isinstance(error, RosServiceOutcomeUnknownError)
    return HTTPException(
        status_code=504 if outcome_unknown else 409,
        detail={
            "success": False,
            "operation": operation,
            "message": str(error),
            "outcome": "UNKNOWN" if outcome_unknown else "FAILED",
            "retry_safe": not outcome_unknown,
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
    _mutation: None = Depends(_serialize_mission_mutation),
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
            mission_report_store.save_new_mission,
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
    except MissionReportError as error:
        raise HTTPException(
            status_code=500,
            detail=str(error),
        ) from error

    trajectory_snapshot.invalidate("mission_uploaded")
    rover_state.update(
        "mission",
        accepted_for_start=False,
    )

    # A valid upload always triggers trajectory calculation. The rover is not
    # started here; it remains in the safe non-driving condition until Start.
    try:
        _require_ros_bridge()
        await run_in_threadpool(ros_bridge.prepare_trajectory)
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
        "message": (
            "Mission uploaded; trajectory preparation started and will "
            "complete automatically when RTK is FIXED. Load the mission "
            "after reviewing the preview to enable START."
        ),
        "upload": metadata,
        "mission": _mission_state(),
    }


@mission_router.post("/load")
async def load_mission(
    request: MissionIdRequest | None = None,
    _session: AuthenticatedSession = Depends(require_auth),
    _mutation: None = Depends(_serialize_mission_mutation),
) -> dict[str, Any]:
    """Confirm the uploaded preview so START may be enabled.

    Does not accept a file and does not regenerate the trajectory.
    """

    _require_not_driving(operation="load the mission")

    mission = _mission_state()

    requested_id = str(request.mission_id).strip() if request and request.mission_id else None
    if requested_id and requested_id != str(mission.get("mission_id") or ""):
        raise HTTPException(
            status_code=409,
            detail="The requested mission is not the currently loaded mission. Restore it first.",
        )

    if mission.get("loaded") is not True:
        raise HTTPException(
            status_code=409,
            detail="Upload a mission and wait for the preview before loading.",
        )

    if mission.get("trajectory_ready") is not True:
        raise HTTPException(
            status_code=409,
            detail="Wait for the rover path preview before loading the mission.",
        )

    rover_state.update(
        "mission",
        accepted_for_start=True,
        message=(
            "Mission loaded. START will enable when the mission manager "
            "is READY."
        ),
        error=None,
    )

    return {
        "success": True,
        "operation": "load",
        "message": (
            "Mission loaded. START is allowed once the rover reports READY."
        ),
        "mission": _mission_state(),
    }


@mission_router.post("/restore")
async def restore_mission(
    request: MissionIdRequest,
    _session: AuthenticatedSession = Depends(require_auth),
    _mutation: None = Depends(_serialize_mission_mutation),
) -> dict[str, Any]:
    """Restore a completed mission by its archived mission ID and prepare it."""

    _require_not_active(operation="restore a completed mission")
    mission_id = str(request.mission_id or "").strip()
    if not mission_id:
        raise HTTPException(status_code=422, detail="mission_id is required.")

    current_id = str(_mission_state().get("mission_id") or "")
    current_loaded = bool(_mission_state().get("loaded", False))
    if current_loaded and current_id and current_id != mission_id:
        raise HTTPException(
            status_code=409,
            detail="A different mission is currently loaded. Upload, clear, or delete it first.",
        )

    try:
        metadata = await run_in_threadpool(mission_store.restore_archived_mission, mission_id)
    except MissionValidationError as error:
        raise HTTPException(status_code=404, detail=str(error)) from error
    except (OSError, RuntimeError) as error:
        raise HTTPException(status_code=500, detail=f"The archived mission could not be restored: {error}") from error

    trajectory_snapshot.invalidate("mission_restored")
    rover_state.update("mission", accepted_for_start=False)
    try:
        _require_ros_bridge()
        await run_in_threadpool(ros_bridge.prepare_trajectory)
    except HTTPException:
        rover_state.set_mission_state(
            "LOADED",
            message="Archived mission restored; trajectory preparation is waiting for the ROS bridge.",
            error=None,
        )
        raise HTTPException(
            status_code=503,
            detail={
                "success": False,
                "mission_restored": True,
                "message": "The archived mission was restored, but trajectory preparation could not start.",
                "mission": _mission_state(),
            },
        )
    except RuntimeError as error:
        raise HTTPException(
            status_code=422,
            detail={
                "success": False,
                "mission_restored": True,
                "message": str(error),
                "mission": _mission_state(),
            },
        ) from error

    return {
        "success": True,
        "operation": "restore",
        "message": "Archived mission restored; review the generated preview, then load and start it.",
        "restore": metadata,
        "mission": _mission_state(),
    }


@mission_router.get("/history")
def mission_history(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """List checksum-verified missions completed on this rover."""

    return {
        "success": True,
        "missions": mission_store.list_archived_missions(),
    }


@mission_router.post("/prepare")
async def prepare_mission(
    request: MissionIdRequest | None = None,
    _session: AuthenticatedSession = Depends(require_auth),
    _mutation: None = Depends(_serialize_mission_mutation),
) -> dict[str, Any]:
    """Re-read the stored mission.csv and prepare the trajectory again.

    Staging (accepted_for_start) is owned by upload/load/delete, not by
    generate. Re-preparing the same already-loaded mission.csv does not
    un-stage it -- the operator already confirmed this mission_id via
    /load; regenerating its geometry (e.g. after mission_manager dropped
    READY on Stop/Complete) is not new content requiring re-confirmation.
    """

    _require_not_active(operation="prepare the mission")

    try:
        metadata = await run_in_threadpool(mission_store.load_metadata)
    except MissionValidationError as error:
        raise HTTPException(
            status_code=409,
            detail=str(error),
        ) from error

    requested_id = str(request.mission_id).strip() if request and request.mission_id else None
    if requested_id and (not metadata or requested_id != str(metadata.get("mission_id") or "")):
        raise HTTPException(status_code=409, detail="The requested mission is not currently loaded.")

    trajectory_snapshot.invalidate("prepare_requested")
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
        "report": mission_report_store.status(),
    }


def _clear_and_delete_active_mission() -> bool:
    """Serialize manual trajectory clear and active-file deletion."""

    with mission_report_store.lifecycle_transaction():
        trajectory_snapshot.invalidate("mission_deleted")
        if ros_bridge.running:
            ros_bridge.clear_mission()
        return mission_store.delete()


@mission_router.get("/loaded-path")
def loaded_path(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Return the loaded navigation path.

    When the verified generator snapshot is live, the FULL path is returned
    (same object the Socket.IO push carries, identified by ``snapshot``).
    With push enabled, pending/invalidated snapshots return no geometry. The
    legacy preview is only used when the feature is disabled, so fallback
    cannot resurrect a path that snapshot verification has just rejected.
    """

    mission = _mission_state()

    live = trajectory_snapshot.live_payload(mission)
    if live is not None:
        return {
            "success": True,
            "frame_id": mission.get("path_frame_id"),
            "navigation_point_count": live["count"],
            "preview_truncated": False,
            "points": legacy_points(live),
            "snapshot": {
                "server_instance_id": live["server_instance_id"],
                "seq": live["seq"],
                "mission_id": live["mission_id"],
                "signature": live["signature"],
            },
        }

    if push_enabled():
        # No live snapshot. Say WHY, so a client can tell "still assembling"
        # (keep waiting) from "invalid" (a new preparation is required) from
        # a genuinely empty path. The real point count is reported so pending
        # assembly is never mistaken for an empty path.
        display = trajectory_snapshot.display_state(mission)
        try:
            expected_count = max(0, int(mission.get("navigation_point_count") or 0))
        except (TypeError, ValueError):
            expected_count = 0
        return {
            "snapshot": None,
            "success": True,
            "frame_id": None,
            "navigation_point_count": expected_count,
            "preview_truncated": False,
            "points": [],
            "snapshot_state": display["state"],
            "snapshot_reason": display["reason"],
            "requires_reprepare": display["requires_reprepare"],
            "assembling": display["state"] == "assembling",
            "server_instance_id": display["server_instance_id"],
            "seq": display["seq"],
        }

    return {
        "snapshot": None,
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


@mission_router.get("/report")
def mission_report(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Return the backend-owned live or terminal report for display."""

    report = mission_report_store.current_report()
    return {
        "success": True,
        "available": report is not None,
        "report": report,
    }


@mission_router.get("/report/download")
def download_mission_report(
    _session: AuthenticatedSession = Depends(require_auth),
) -> FileResponse:
    """Download the durable backend report after mission termination."""

    report_path = Path(settings.mission_report_file)

    terminal_report = mission_report_store.terminal_report()
    if terminal_report is None or not report_path.is_file():
        raise HTTPException(
            status_code=404,
            detail="No terminal mission report is available.",
        )

    return FileResponse(
        path=str(report_path),
        media_type="application/json; charset=utf-8",
        filename="last-mission-report.json",
    )


@mission_router.delete("/file")
async def delete_mission_file(
    _session: AuthenticatedSession = Depends(require_auth),
    _mutation: None = Depends(_serialize_mission_mutation),
) -> dict[str, Any]:
    """Delete the single mission.csv after clearing prepared ROS state."""

    _require_not_active(operation="delete mission.csv")

    try:
        existed = await run_in_threadpool(_clear_and_delete_active_mission)
    except RuntimeError as error:
        raise HTTPException(
            status_code=409,
            detail=(
                "Unable to clear the prepared ROS trajectory or delete "
                f"the active mission; active artifacts were retained: {error}"
            ),
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
    _mutation: None = Depends(_serialize_mission_mutation),
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
    request: MissionIdRequest | None = None,
    _session: AuthenticatedSession = Depends(require_auth),
    _mutation: None = Depends(_serialize_mission_mutation),
) -> dict[str, Any]:
    mission = _mission_state()
    requested_id = str(request.mission_id).strip() if request and request.mission_id else None
    if requested_id and requested_id != str(mission.get("mission_id") or ""):
        # A completed mission no longer occupies the active slot.  Accepting
        # its archived ID here makes retries deterministic while still
        # requiring a fresh trajectory preparation and READY preview.
        _require_not_driving(operation="restore a mission for start")
        if mission.get("loaded") is True:
            raise HTTPException(
                status_code=409,
                detail="A different mission is currently loaded. Restore it after clearing the active mission.",
            )
        try:
            await run_in_threadpool(mission_store.restore_archived_mission, requested_id)
            trajectory_snapshot.invalidate("mission_restored")
            _require_ros_bridge()
            await run_in_threadpool(ros_bridge.prepare_trajectory)
        except MissionValidationError as error:
            raise HTTPException(status_code=404, detail=str(error)) from error
        except HTTPException:
            raise
        except (OSError, RuntimeError) as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
        mission = _mission_state()
        if mission.get("trajectory_ready") is not True:
            raise HTTPException(
                status_code=409,
                detail="The archived mission was restored. Wait for its path preview, then load it before starting.",
            )
        rover_state.update("mission", accepted_for_start=True)
    if mission.get("accepted_for_start") is not True:
        raise HTTPException(
            status_code=409,
            detail=(
                "Load the mission after reviewing the preview before starting."
            ),
        )

    return await _run_ros_operation(
        "start",
        ros_bridge.start_mission,
    )


@mission_router.post("/pause")
async def pause_mission(
    _session: AuthenticatedSession = Depends(require_auth),
    _mutation: None = Depends(_serialize_mission_mutation),
) -> dict[str, Any]:
    return await _run_ros_operation(
        "pause",
        ros_bridge.pause_mission,
    )


@mission_router.post("/resume")
async def resume_mission(
    _session: AuthenticatedSession = Depends(require_auth),
    _mutation: None = Depends(_serialize_mission_mutation),
) -> dict[str, Any]:
    return await _run_ros_operation(
        "resume",
        ros_bridge.resume_mission,
    )


@mission_router.post("/next-point")
async def next_point(
    _session: AuthenticatedSession = Depends(require_auth),
    _mutation: None = Depends(_serialize_mission_mutation),
) -> dict[str, Any]:
    return await _run_ros_operation(
        "next-point",
        ros_bridge.next_point,
    )


@mission_router.post("/skip-point")
async def skip_point(
    _session: AuthenticatedSession = Depends(require_auth),
    _mutation: None = Depends(_serialize_mission_mutation),
) -> dict[str, Any]:
    return await _run_ros_operation(
        "skip-point",
        ros_bridge.skip_point,
    )


@mission_router.post("/stop")
async def stop_mission(
    _session: AuthenticatedSession = Depends(require_auth),
    _mutation: None = Depends(_serialize_mission_mutation),
) -> dict[str, Any]:
    mission_id = str(_mission_state().get("mission_id") or "")

    response = await _run_ros_operation(
        "stop",
        ros_bridge.stop_mission,
    )

    # STOP authority is Mission Manager. Once its STOP service returns
    # successfully, motion has already been disabled and the PX4 disarm
    # contract has completed. Terminal report/file cleanup runs independently
    # from the control response and must never hold the STOP HTTP request.
    report_status = mission_report_store.status()
    report_ready = bool(
        mission_id
        and mission_id == report_status.get("mission_id")
        and report_status.get("cleanup_complete", False)
    )

    response["manager_stop_success"] = True
    response["report_ready"] = report_ready
    response["report"] = report_status

    return response


@mission_router.post("/clear")
async def clear_mission(
    _session: AuthenticatedSession = Depends(require_auth),
    _mutation: None = Depends(_serialize_mission_mutation),
) -> dict[str, Any]:
    """Clear generated ROS paths and progress while retaining mission.csv."""

    trajectory_snapshot.invalidate("clear_requested")
    return await _run_ros_operation(
        "clear",
        ros_bridge.clear_mission,
    )
