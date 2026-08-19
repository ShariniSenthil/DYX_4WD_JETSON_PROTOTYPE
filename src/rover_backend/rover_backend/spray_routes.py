"""Spray-servo REST API for the DYX 4WD Rover Backend.

This module exposes the runtime AUX5 spray-servo configuration to the
frontend.

The frontend works in PWM microseconds:

    1000 us -> -1.0 actuator command
    1500 us ->  0.0 actuator command
    2000 us -> +1.0 actuator command

The conversion and ROS acknowledgement are handled by ros_bridge.py.

This module does not command PX4 directly and does not modify mission logic.
"""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter
from fastapi import Depends
from fastapi import HTTPException
from pydantic import BaseModel
from starlette.concurrency import run_in_threadpool

from rover_backend.auth import AuthenticatedSession
from rover_backend.auth import require_auth
from rover_backend.ros_bridge import ros_bridge

spray_router = APIRouter(
    prefix="/api/spray",
    tags=["spray"],
)


class SprayConfigRequest(BaseModel):
    """Frontend spray-servo PWM configuration."""

    press_pwm_us: float
    release_pwm_us: float


def _require_ros_bridge() -> None:
    if not ros_bridge.running:
        raise HTTPException(
            status_code=503,
            detail={
                "success": False,
                "message": ("The rover ROS bridge is not running."),
            },
        )


def _validate_pwm(
    value: float,
    *,
    field_name: str,
) -> float:
    try:
        pwm = float(value)
    except (TypeError, ValueError) as error:
        raise HTTPException(
            status_code=422,
            detail={
                "success": False,
                "message": (f"{field_name} must be a number."),
            },
        ) from error

    if pwm < 1000.0 or pwm > 2000.0:
        raise HTTPException(
            status_code=422,
            detail={
                "success": False,
                "message": (
                    f"{field_name} must be between " "1000 and 2000 microseconds."
                ),
            },
        )

    return pwm


async def _read_spray_config() -> dict[str, Any]:
    _require_ros_bridge()

    try:
        config = await run_in_threadpool(ros_bridge.get_spray_config)
    except RuntimeError as error:
        raise HTTPException(
            status_code=409,
            detail={
                "success": False,
                "message": str(error),
            },
        ) from error

    return config


@spray_router.get("/status")
async def spray_status(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Return current spray-controller status/configuration."""

    config = await _read_spray_config()

    return {
        "success": True,
        "config": config,
    }


@spray_router.get("/config")
async def get_spray_config(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Return the currently active spray PWM configuration."""

    config = await _read_spray_config()

    return {
        "success": True,
        "config": config,
    }


@spray_router.post("/config")
async def set_spray_config(
    request: SprayConfigRequest,
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Update AUX5 press/release PWM values safely."""

    _require_ros_bridge()

    press_pwm_us = _validate_pwm(
        request.press_pwm_us,
        field_name="press_pwm_us",
    )

    release_pwm_us = _validate_pwm(
        request.release_pwm_us,
        field_name="release_pwm_us",
    )

    try:
        config = await run_in_threadpool(
            lambda: ros_bridge.set_spray_config(
                press_pwm_us=press_pwm_us,
                release_pwm_us=release_pwm_us,
            )
        )

    except RuntimeError as error:
        raise HTTPException(
            status_code=409,
            detail={
                "success": False,
                "message": str(error),
            },
        ) from error

    return {
        "success": True,
        "message": ("Spray PWM configuration updated successfully."),
        "config": config,
    }
