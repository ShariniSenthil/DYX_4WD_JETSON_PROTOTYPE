"""Raw-GNSS spray gate: the ONLY accuracy that authorizes spray / COMPLETED.

RPP still owns *stopping*: it brings the rover to rest and certifies (or
fails) its own 20 mm estimator-frame stop.  Its along/cross/overall numbers
are kept in the report as controller debug data only.  Whether the point is
sprayed and counted COMPLETED is decided here, from raw RTK-FIXED GNSS against
the surveyed coordinate that was uploaded with the mission:

    raw radial error <= tolerance_m (30 mm)  -> PASS  -> spray, COMPLETED
    raw radial error >  tolerance_m          -> FAIL  -> no spray, FAILED
    no trustworthy measurement by timeout    -> FAIL  -> no spray, FAILED

Measurement rules (all fail closed):

* Only fixes received AFTER the stop was declared are used, so braking
  motion can never be averaged into the stop position.
* Every fix in the window must be RTK FIXED (fix_type 6).
* At least ``minimum_samples`` fixes spanning ``window_sec`` are required.
* The fixes must form a stationary cluster (``max_scatter_m``).
* The verdict is ONE-SHOT: the first available measurement decides.  A
  failing measurement is never retried in the hope of a luckier sample; only
  an *unavailable* measurement is retried, and only until ``timeout_sec``.

Pure and ROS-free so it can be tested without a node.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Optional, Sequence

from mission_manager.survey_truth import (
    GnssFix,
    SurveyTarget,
    compute_survey_truth,
)


__all__ = [
    "GATE_FAIL",
    "GATE_PASS",
    "GATE_WAITING",
    "RawGnssSprayGateConfig",
    "RawGnssSprayGateDecision",
    "evaluate_raw_gnss_spray_gate",
]

GATE_WAITING = "WAITING"
GATE_PASS = "PASS"
GATE_FAIL = "FAIL"

MEASUREMENT_SOURCE = "RAW_GNSS_SURVEY"


@dataclass(frozen=True)
class RawGnssSprayGateConfig:
    tolerance_m: float = 0.030
    window_sec: float = 1.0
    minimum_samples: int = 3
    max_scatter_m: float = 0.020
    timeout_sec: float = 3.0

    def __post_init__(self) -> None:
        for name in ("tolerance_m", "window_sec", "max_scatter_m", "timeout_sec"):
            value = getattr(self, name)
            if not (isinstance(value, (int, float)) and math.isfinite(value)):
                raise ValueError(f"{name} must be finite")
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if int(self.minimum_samples) < 1:
            raise ValueError("minimum_samples must be >= 1")
        if self.timeout_sec < self.window_sec:
            raise ValueError("timeout_sec must be >= window_sec")


@dataclass(frozen=True)
class RawGnssSprayGateDecision:
    status: str
    reason: Optional[str]
    elapsed_sec: float
    tolerance_m: float
    radial_error_mm: Optional[float] = None
    survey: dict[str, Any] = field(default_factory=dict)

    @property
    def final(self) -> bool:
        return self.status in (GATE_PASS, GATE_FAIL)

    @property
    def passed(self) -> bool:
        return self.status == GATE_PASS

    def to_payload(self) -> dict[str, Any]:
        return {
            "measurement_source": MEASUREMENT_SOURCE,
            "status": self.status,
            "pass": self.passed,
            "final": self.final,
            "reason": self.reason,
            "elapsed_sec": round(self.elapsed_sec, 3),
            "tolerance_m": self.tolerance_m,
            "tolerance_mm": self.tolerance_m * 1000.0,
            "radial_error_mm": self.radial_error_mm,
        }


def evaluate_raw_gnss_spray_gate(
    *,
    target: Optional[SurveyTarget],
    previous_target: Optional[SurveyTarget],
    fixes: Sequence[GnssFix],
    stop_monotonic_sec: float,
    now_monotonic_sec: float,
    config: RawGnssSprayGateConfig,
    fallback_bearing_rad: Optional[float] = None,
    unavailable_reason: Optional[str] = None,
) -> RawGnssSprayGateDecision:
    """Decide PASS / FAIL / WAITING for one stopped marking point.

    ``unavailable_reason`` lets the caller force "no measurement possible"
    (for example survey targets from a different mission) without inventing
    a target; it is treated exactly like any other unavailable measurement.
    """

    elapsed = max(0.0, float(now_monotonic_sec) - float(stop_monotonic_sec))
    tolerance = float(config.tolerance_m)

    def waiting(reason: str, survey: Optional[dict] = None) -> RawGnssSprayGateDecision:
        return RawGnssSprayGateDecision(
            status=GATE_WAITING,
            reason=reason,
            elapsed_sec=elapsed,
            tolerance_m=tolerance,
            survey=dict(survey or {}),
        )

    if elapsed < config.window_sec:
        return waiting("COLLECTING_STATIONARY_RAW_GNSS")

    if unavailable_reason is not None:
        survey = {
            "measurement_source": MEASUREMENT_SOURCE,
            "available": False,
            "reason": unavailable_reason,
        }
        truth_available = False
        truth_reason = unavailable_reason
        radial_m = None
    else:
        post_stop = [
            fix for fix in fixes if fix.monotonic_sec >= float(stop_monotonic_sec)
        ]
        truth = compute_survey_truth(
            target=target,
            previous_target=previous_target,
            fixes=post_stop,
            now_monotonic_sec=float(now_monotonic_sec),
            window_sec=float(config.window_sec),
            minimum_samples=int(config.minimum_samples),
            max_scatter_m=float(config.max_scatter_m),
            tolerance_m=tolerance,
            fallback_bearing_rad=fallback_bearing_rad,
        )
        survey = truth.to_payload()
        truth_available = bool(truth.available)
        truth_reason = truth.reason
        radial_m = truth.radial_error_m
        if truth_available and truth.sample_trimmed_count > 0:
            # survey_truth trims old samples to shed braking motion. Every
            # sample here is already post-stop, so needing a trim means the
            # rover was still moving: the whole window must be stationary.
            survey = dict(survey)
            survey["available"] = False
            survey["reason"] = "GNSS_WINDOW_NOT_STATIONARY"
            truth_available = False
            truth_reason = "GNSS_WINDOW_NOT_STATIONARY"
            radial_m = None

    if truth_available and radial_m is not None and math.isfinite(radial_m):
        radial_mm = float(radial_m) * 1000.0
        passed = float(radial_m) <= tolerance
        return RawGnssSprayGateDecision(
            status=GATE_PASS if passed else GATE_FAIL,
            reason=None if passed else "RAW_GNSS_RADIAL_EXCEEDS_TOLERANCE",
            elapsed_sec=elapsed,
            tolerance_m=tolerance,
            radial_error_mm=radial_mm,
            survey=survey,
        )

    reason = str(truth_reason or "RAW_GNSS_MEASUREMENT_UNAVAILABLE")
    if elapsed >= config.timeout_sec:
        return RawGnssSprayGateDecision(
            status=GATE_FAIL,
            reason=f"RAW_GNSS_UNAVAILABLE:{reason}",
            elapsed_sec=elapsed,
            tolerance_m=tolerance,
            survey=survey,
        )
    return waiting(reason, survey)
