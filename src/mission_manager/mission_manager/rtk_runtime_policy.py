"""ROS-free runtime RTK policy for a RUNNING mission (operator-approved 2026-09-26).

GPSRAW ``fix_type``: 6 = RTK FIXED, 5 = RTK FLOAT, 4 = DGPS, 3 = 3D, 2 = 2D,
0/1 = no fix (7/8, static/PPP, are treated like a loss: only 6 is FIXED).

While RUNNING
  * fresh FIXED                         -> continue
  * fresh FLOAT, no spray in progress   -> stop at once, auto-resumable pause
  * fresh FLOAT, spray in progress      -> let the press finish, then pause
  * anything else, or GPS status stale  -> stop, manual resume (RTK_LOST)

While paused for FLOAT
  * FIXED held continuously for ``fixed_hold_sec``        -> auto-resume
  * FLOAT again                                           -> restart the hold
  * drops below FLOAT, or GPS status stale                -> manual resume
  * still not auto-resumed after ``float_manual_after_sec`` -> manual resume

The caller owns every side effect (stopping, resuming, events). This class
only decides.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional


__all__ = [
    "FIX_RTK_FIXED",
    "FIX_RTK_FLOAT",
    "FloatPauseAction",
    "RtkRunningAction",
    "RtkRuntimeConfig",
    "RtkRuntimePolicy",
]

FIX_RTK_FIXED = 6
FIX_RTK_FLOAT = 5


class RtkRunningAction(str, Enum):
    CONTINUE = "continue"
    DEFER_FLOAT_UNTIL_SPRAY_DONE = "defer_float_until_spray_done"
    PAUSE_FLOAT = "pause_float"
    PAUSE_LOST = "pause_lost"


class FloatPauseAction(str, Enum):
    HOLD = "hold"
    AUTO_RESUME = "auto_resume"
    ESCALATE_LOST = "escalate_lost"
    ESCALATE_TIMEOUT = "escalate_timeout"


@dataclass(frozen=True, slots=True)
class RtkRuntimeConfig:
    fixed_hold_sec: float = 3.0
    float_manual_after_sec: float = 120.0
    gps_stale_sec: float = 10.0

    def __post_init__(self) -> None:
        for name in ("fixed_hold_sec", "float_manual_after_sec", "gps_stale_sec"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
        if self.float_manual_after_sec <= self.fixed_hold_sec:
            raise ValueError("float_manual_after_sec must exceed fixed_hold_sec")


class RtkRuntimePolicy:
    """Decide what a RUNNING / FLOAT-paused mission does about RTK state."""

    def __init__(self, config: RtkRuntimeConfig):
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.float_pause_started_at: Optional[float] = None
        self.fixed_since: Optional[float] = None
        self.spray_deferral_active = False

    def _fresh(self, gps_age_sec: float) -> bool:
        return math.isfinite(gps_age_sec) and gps_age_sec <= self.config.gps_stale_sec

    # --------------------------------------------------------------- RUNNING
    def running(
        self,
        now_sec: float,
        fix_type: int,
        gps_age_sec: float,
        spray_in_progress: bool,
    ) -> RtkRunningAction:
        fresh = self._fresh(gps_age_sec)
        if fresh and fix_type == FIX_RTK_FIXED:
            self.spray_deferral_active = False
            return RtkRunningAction.CONTINUE
        if fresh and fix_type == FIX_RTK_FLOAT:
            if spray_in_progress:
                self.spray_deferral_active = True
                return RtkRunningAction.DEFER_FLOAT_UNTIL_SPRAY_DONE
            self.spray_deferral_active = False
            self.float_pause_started_at = float(now_sec)
            self.fixed_since = None
            return RtkRunningAction.PAUSE_FLOAT
        self.spray_deferral_active = False
        return RtkRunningAction.PAUSE_LOST

    # ------------------------------------------------------- FLOAT-paused
    def float_paused(
        self,
        now_sec: float,
        fix_type: int,
        gps_age_sec: float,
    ) -> FloatPauseAction:
        if self.float_pause_started_at is None:
            self.float_pause_started_at = float(now_sec)
        if not self._fresh(gps_age_sec) or fix_type not in (
            FIX_RTK_FIXED,
            FIX_RTK_FLOAT,
        ):
            self.fixed_since = None
            return FloatPauseAction.ESCALATE_LOST
        if fix_type == FIX_RTK_FIXED:
            if self.fixed_since is None:
                self.fixed_since = float(now_sec)
            if now_sec - self.fixed_since >= self.config.fixed_hold_sec:
                return FloatPauseAction.AUTO_RESUME
        else:
            self.fixed_since = None
        if now_sec - self.float_pause_started_at >= self.config.float_manual_after_sec:
            return FloatPauseAction.ESCALATE_TIMEOUT
        return FloatPauseAction.HOLD

    def fixed_held_sec(self, now_sec: float) -> float:
        if self.fixed_since is None:
            return 0.0
        return max(0.0, now_sec - self.fixed_since)

    def float_paused_sec(self, now_sec: float) -> float:
        if self.float_pause_started_at is None:
            return 0.0
        return max(0.0, now_sec - self.float_pause_started_at)
