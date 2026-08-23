"""ROS-free tracking stability control and run-level tracking metrics.

This module deliberately owns neither path progress nor completion authority.
It consumes projection/guidance measurements, applies recovery hysteresis, and
produces advisory acceleration and speed-scale signals.  Its metrics are
diagnostic only and never form a mission-completion gate.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
from typing import Deque, Optional


__all__ = [
    "TrackingControlConfig",
    "TrackingControlInput",
    "TrackingControlOutput",
    "TrackingControlState",
    "TrackingMetricsAccumulator",
    "TrackingMetricsSnapshot",
    "TrackingStabilityController",
]


class TrackingControlState(str, Enum):
    """Tracking stability state, separate from the mission motion FSM."""

    TRACKING = "tracking"
    RECOVERY = "recovery"
    RECAPTURE_STABLE = "recapture_stable"


@dataclass(frozen=True, slots=True)
class TrackingControlConfig:
    """Validated recovery hysteresis and aggregation parameters."""

    recovery_enter_cross_track_m: float = 0.050
    recovery_exit_cross_track_m: float = 0.020
    recovery_enter_heading_error_rad: float = math.radians(15.0)
    recovery_exit_heading_error_rad: float = math.radians(5.0)
    recovery_enter_on_bearing_clamp: bool = True
    stable_recapture_dwell_sec: float = 0.30
    control_dt_max_sec: float = 0.10
    recovery_speed_scale: float = 0.35
    recapture_speed_scale: float = 0.50

    metrics_quantile_window_capacity: int = 2048
    metrics_histogram_bin_width_m: float = 0.001
    metrics_histogram_max_m: float = 1.0
    metrics_monotonic_tolerance_m: float = 0.001
    metrics_cruise_speed_threshold_mps: float = 0.80

    def __post_init__(self) -> None:
        finite_names = (
            "recovery_enter_cross_track_m",
            "recovery_exit_cross_track_m",
            "recovery_enter_heading_error_rad",
            "recovery_exit_heading_error_rad",
            "stable_recapture_dwell_sec",
            "control_dt_max_sec",
            "recovery_speed_scale",
            "recapture_speed_scale",
            "metrics_histogram_bin_width_m",
            "metrics_histogram_max_m",
            "metrics_monotonic_tolerance_m",
            "metrics_cruise_speed_threshold_mps",
        )
        for name in finite_names:
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")

        if not isinstance(self.recovery_enter_on_bearing_clamp, bool):
            raise ValueError("recovery_enter_on_bearing_clamp must be boolean")
        if not (
            0.0 <= self.recovery_exit_cross_track_m
            < self.recovery_enter_cross_track_m
        ):
            raise ValueError(
                "cross-track thresholds must satisfy 0 <= exit < enter"
            )
        if not (
            0.0 <= self.recovery_exit_heading_error_rad
            < self.recovery_enter_heading_error_rad
            <= math.pi
        ):
            raise ValueError(
                "heading thresholds must satisfy 0 <= exit < enter <= pi"
            )
        if self.stable_recapture_dwell_sec < 0.0:
            raise ValueError("stable_recapture_dwell_sec must be non-negative")
        if self.control_dt_max_sec <= 0.0:
            raise ValueError("control_dt_max_sec must be greater than zero")
        for name in ("recovery_speed_scale", "recapture_speed_scale"):
            if not 0.0 <= getattr(self, name) <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if self.recovery_speed_scale > self.recapture_speed_scale:
            raise ValueError(
                "recovery_speed_scale must not exceed recapture_speed_scale"
            )
        if (
            isinstance(self.metrics_quantile_window_capacity, bool)
            or not isinstance(self.metrics_quantile_window_capacity, int)
            or self.metrics_quantile_window_capacity <= 0
        ):
            raise ValueError(
                "metrics_quantile_window_capacity must be a positive integer"
            )
        if self.metrics_histogram_bin_width_m <= 0.0:
            raise ValueError("metrics_histogram_bin_width_m must be greater than zero")
        if self.metrics_histogram_max_m <= 0.0:
            raise ValueError("metrics_histogram_max_m must be greater than zero")
        histogram_bin_count = math.ceil(
            self.metrics_histogram_max_m / self.metrics_histogram_bin_width_m
        )
        if histogram_bin_count > 1_000_000:
            raise ValueError("tracking metrics histogram must not exceed 1,000,000 bins")
        if self.metrics_monotonic_tolerance_m < 0.0:
            raise ValueError("metrics_monotonic_tolerance_m must be non-negative")
        if self.metrics_cruise_speed_threshold_mps < 0.0:
            raise ValueError(
                "metrics_cruise_speed_threshold_mps must be non-negative"
            )


@dataclass(frozen=True, slots=True)
class TrackingControlInput:
    """One cycle of projection, guidance, and measured motion state."""

    path_identity: str
    projection_s_m: float
    signed_cross_track_m: float
    heading_error_rad: float
    commanded_speed_mps: float
    measured_speed_mps: float
    bearing_clamped: bool
    telemetry_fresh: bool
    dt_sec: float
    reset: bool = False


@dataclass(frozen=True, slots=True)
class TrackingControlOutput:
    """Advisory tracking stability result for one cycle."""

    state: TrackingControlState
    valid: bool
    acceleration_allowed: bool
    recovery_speed_scale: float
    transition_reason: str
    bounded_dt_sec: float
    stable_dwell_sec: float
    reset_applied: bool
    invalid_reason: Optional[str] = None


class TrackingStabilityController:
    """Recovery hysteresis with a continuous stable-recapture dwell."""

    def __init__(self, config: TrackingControlConfig) -> None:
        self.config = config
        self._state = TrackingControlState.RECAPTURE_STABLE
        self._stable_dwell_sec = 0.0
        self._path_identity: Optional[str] = None

    @property
    def state(self) -> TrackingControlState:
        return self._state

    def reset(self, path_identity: Optional[str] = None) -> None:
        """Reset acceleration permission at a path/control boundary."""

        self._state = TrackingControlState.RECAPTURE_STABLE
        self._stable_dwell_sec = 0.0
        self._path_identity = path_identity

    def step(self, sample: TrackingControlInput) -> TrackingControlOutput:
        identity = str(sample.path_identity)
        identity_changed = self._path_identity != identity
        reset_applied = bool(sample.reset or identity_changed)
        if reset_applied:
            self.reset(identity)

        invalid_reason = self._invalid_reason(sample)
        if invalid_reason is not None:
            self._state = TrackingControlState.RECOVERY
            self._stable_dwell_sec = 0.0
            return TrackingControlOutput(
                state=self._state,
                valid=False,
                acceleration_allowed=False,
                recovery_speed_scale=0.0,
                transition_reason="invalid_input",
                bounded_dt_sec=0.0,
                stable_dwell_sec=0.0,
                reset_applied=reset_applied,
                invalid_reason=invalid_reason,
            )

        bounded_dt = min(float(sample.dt_sec), self.config.control_dt_max_sec)
        enter_recovery = self._recovery_entered(sample)
        inside_exit_gates = self._inside_exit_gates(sample)

        if enter_recovery:
            reason = self._entry_reason(sample)
            self._state = TrackingControlState.RECOVERY
            self._stable_dwell_sec = 0.0
        elif self._state is TrackingControlState.TRACKING:
            reason = "tracking_held"
        elif inside_exit_gates:
            if self._state is TrackingControlState.RECOVERY:
                self._state = TrackingControlState.RECAPTURE_STABLE
                reason = "recapture_dwell_started"
            else:
                reason = "recapture_dwell_held"
            self._stable_dwell_sec += bounded_dt
            if self._stable_dwell_sec >= self.config.stable_recapture_dwell_sec:
                self._stable_dwell_sec = self.config.stable_recapture_dwell_sec
                self._state = TrackingControlState.TRACKING
                reason = "stable_recapture_complete"
        else:
            self._state = TrackingControlState.RECOVERY
            self._stable_dwell_sec = 0.0
            reason = "recapture_exit_gate_lost"

        if reset_applied and reason in {
            "recapture_dwell_started",
            "recapture_dwell_held",
        }:
            reason = "path_reset_recapture"

        if self._state is TrackingControlState.TRACKING:
            acceleration_allowed = True
            speed_scale = 1.0
        elif self._state is TrackingControlState.RECAPTURE_STABLE:
            acceleration_allowed = False
            speed_scale = self.config.recapture_speed_scale
        else:
            acceleration_allowed = False
            speed_scale = self.config.recovery_speed_scale

        return TrackingControlOutput(
            state=self._state,
            valid=True,
            acceleration_allowed=acceleration_allowed,
            recovery_speed_scale=speed_scale,
            transition_reason=reason,
            bounded_dt_sec=bounded_dt,
            stable_dwell_sec=self._stable_dwell_sec,
            reset_applied=reset_applied,
        )

    def _invalid_reason(self, sample: TrackingControlInput) -> Optional[str]:
        if not sample.telemetry_fresh:
            return "stale_telemetry"
        if not isinstance(sample.bearing_clamped, bool):
            return "bearing_clamped_not_boolean"
        if not isinstance(sample.telemetry_fresh, bool):
            return "telemetry_fresh_not_boolean"
        values = (
            ("projection_s_m", sample.projection_s_m),
            ("signed_cross_track_m", sample.signed_cross_track_m),
            ("heading_error_rad", sample.heading_error_rad),
            ("commanded_speed_mps", sample.commanded_speed_mps),
            ("measured_speed_mps", sample.measured_speed_mps),
            ("dt_sec", sample.dt_sec),
        )
        for name, value in values:
            if isinstance(value, bool) or not math.isfinite(value):
                return f"nonfinite_{name}"
        if sample.projection_s_m < 0.0:
            return "negative_projection_s_m"
        if sample.commanded_speed_mps < 0.0:
            return "negative_commanded_speed_mps"
        if sample.measured_speed_mps < 0.0:
            return "negative_measured_speed_mps"
        if sample.dt_sec < 0.0:
            return "negative_dt_sec"
        return None

    def _recovery_entered(self, sample: TrackingControlInput) -> bool:
        return (
            abs(sample.signed_cross_track_m)
            >= self.config.recovery_enter_cross_track_m
            or abs(sample.heading_error_rad)
            >= self.config.recovery_enter_heading_error_rad
            or (
                self.config.recovery_enter_on_bearing_clamp
                and sample.bearing_clamped
            )
        )

    def _inside_exit_gates(self, sample: TrackingControlInput) -> bool:
        return (
            abs(sample.signed_cross_track_m)
            <= self.config.recovery_exit_cross_track_m
            and abs(sample.heading_error_rad)
            <= self.config.recovery_exit_heading_error_rad
            and not sample.bearing_clamped
        )

    def _entry_reason(self, sample: TrackingControlInput) -> str:
        reasons = []
        if (
            abs(sample.signed_cross_track_m)
            >= self.config.recovery_enter_cross_track_m
        ):
            reasons.append("cross_track")
        if (
            abs(sample.heading_error_rad)
            >= self.config.recovery_enter_heading_error_rad
        ):
            reasons.append("heading")
        if self.config.recovery_enter_on_bearing_clamp and sample.bearing_clamped:
            reasons.append("bearing_clamp")
        return "recovery_entered:" + "+".join(reasons)


@dataclass(frozen=True, slots=True)
class TrackingMetricsSnapshot:
    """Immutable diagnostic aggregates for the current identity/run."""

    mission_identity: Optional[str]
    path_identity: Optional[str]
    sample_count: int
    rejected_sample_count: int
    elapsed_sec: float
    mean_abs_cross_track_m: float
    rms_cross_track_m: float
    p95_abs_cross_track_m: float
    trailing_p95_abs_cross_track_m: float
    p95_histogram_saturated: bool
    histogram_overflow_count: int
    max_abs_cross_track_m: float
    mean_abs_heading_error_rad: float
    max_abs_heading_error_rad: float
    monotonic_s_violation_count: int
    recovery_time_sec: float
    recapture_time_sec: float
    cruise_time_sec: float
    mean_commanded_speed_mps: float
    mean_measured_speed_mps: float
    mean_abs_speed_error_mps: float
    rms_speed_error_mps: float
    max_abs_speed_error_mps: float
    quantile_sample_count: int
    quantile_window_capacity: int
    discontinuity_count: int
    last_discontinuity_reason: Optional[str]
    valid_for_acceptance: bool


class TrackingMetricsAccumulator:
    """Bounded-memory, run-level tracking diagnostics.

    Means, RMS values, maxima, times, and the fixed-histogram P95 cover the
    entire current run.  A separate P95 uses the bounded trailing raw-sample
    window; neither quantile mechanism can grow without bound.
    """

    def __init__(self, config: TrackingControlConfig) -> None:
        self.config = config
        self._abs_xtrack_window: Deque[float] = deque(
            maxlen=config.metrics_quantile_window_capacity
        )
        self._histogram_bin_count = math.ceil(
            config.metrics_histogram_max_m / config.metrics_histogram_bin_width_m
        )
        self._xtrack_histogram = [0] * self._histogram_bin_count
        self.reset()

    @property
    def quantile_storage_size(self) -> int:
        return len(self._abs_xtrack_window)

    def reset(
        self,
        mission_identity: Optional[str] = None,
        path_identity: Optional[str] = None,
    ) -> None:
        self._mission_identity = (
            None if mission_identity is None else str(mission_identity)
        )
        self._path_identity = None if path_identity is None else str(path_identity)
        self._sample_count = 0
        self._rejected_sample_count = 0
        self._elapsed_sec = 0.0
        self._sum_abs_xtrack = 0.0
        self._sum_sq_xtrack = 0.0
        self._max_abs_xtrack = 0.0
        self._sum_abs_heading = 0.0
        self._max_abs_heading = 0.0
        self._monotonic_violations = 0
        self._max_projection_s: Optional[float] = None
        self._recovery_time_sec = 0.0
        self._recapture_time_sec = 0.0
        self._cruise_time_sec = 0.0
        self._sum_commanded_speed = 0.0
        self._sum_measured_speed = 0.0
        self._sum_abs_speed_error = 0.0
        self._sum_sq_speed_error = 0.0
        self._max_abs_speed_error = 0.0
        self._abs_xtrack_window.clear()
        for index in range(self._histogram_bin_count):
            self._xtrack_histogram[index] = 0
        self._histogram_overflow_count = 0
        self._discontinuity_count = 0
        self._last_discontinuity_reason: Optional[str] = None
        self._valid_for_acceptance = True

    def note_discontinuity(self, reason: str) -> None:
        """Start a new projection-comparison epoch without erasing evidence.

        A localization jump or explicit geometry reacquisition makes the run
        unsuitable for acceptance even though its pre/post-jump tracking data
        remains valuable.  Only the monotonic projection comparison anchor is
        cleared; all aggregates and bounded quantile storage are preserved.
        """

        normalized_reason = str(reason).strip() or "unspecified"
        self._max_projection_s = None
        self._discontinuity_count += 1
        self._last_discontinuity_reason = normalized_reason
        self._valid_for_acceptance = False

    def add(
        self,
        sample: TrackingControlInput,
        state: TrackingControlState,
        *,
        mission_identity: Optional[str] = None,
    ) -> bool:
        """Add one valid control cycle; return ``False`` for rejected data.

        A mission/path identity change resets aggregates before accepting the
        new sample.  Rejection is finite-safe and never raises into a control
        cycle.
        """

        mission = None if mission_identity is None else str(mission_identity)
        path = str(sample.path_identity)
        if mission != self._mission_identity or path != self._path_identity:
            self.reset(mission, path)

        if (
            not sample.telemetry_fresh
            or not isinstance(sample.telemetry_fresh, bool)
            or not isinstance(sample.bearing_clamped, bool)
            or not isinstance(state, TrackingControlState)
        ):
            self._rejected_sample_count += 1
            return False

        values = (
            sample.projection_s_m,
            sample.signed_cross_track_m,
            sample.heading_error_rad,
            sample.commanded_speed_mps,
            sample.measured_speed_mps,
            sample.dt_sec,
        )
        if any(
            isinstance(value, bool) or not math.isfinite(value) for value in values
        ) or any(
            value < 0.0
            for value in (
                sample.projection_s_m,
                sample.commanded_speed_mps,
                sample.measured_speed_mps,
                sample.dt_sec,
            )
        ):
            self._rejected_sample_count += 1
            return False

        dt = min(float(sample.dt_sec), self.config.control_dt_max_sec)
        xtrack = abs(float(sample.signed_cross_track_m))
        heading = abs(float(sample.heading_error_rad))
        commanded = float(sample.commanded_speed_mps)
        measured = float(sample.measured_speed_mps)
        speed_error = commanded - measured
        abs_speed_error = abs(speed_error)

        self._sample_count += 1
        self._elapsed_sec += dt
        self._sum_abs_xtrack += xtrack
        self._sum_sq_xtrack += xtrack * xtrack
        self._max_abs_xtrack = max(self._max_abs_xtrack, xtrack)
        self._sum_abs_heading += heading
        self._max_abs_heading = max(self._max_abs_heading, heading)
        self._sum_commanded_speed += commanded
        self._sum_measured_speed += measured
        self._sum_abs_speed_error += abs_speed_error
        self._sum_sq_speed_error += speed_error * speed_error
        self._max_abs_speed_error = max(
            self._max_abs_speed_error, abs_speed_error
        )
        self._abs_xtrack_window.append(xtrack)
        if xtrack > self.config.metrics_histogram_max_m:
            self._histogram_overflow_count += 1
        else:
            bin_index = min(
                int(xtrack / self.config.metrics_histogram_bin_width_m),
                self._histogram_bin_count - 1,
            )
            self._xtrack_histogram[bin_index] += 1

        if (
            self._max_projection_s is not None
            and sample.projection_s_m
            < self._max_projection_s - self.config.metrics_monotonic_tolerance_m
        ):
            self._monotonic_violations += 1
        self._max_projection_s = max(
            sample.projection_s_m,
            self._max_projection_s
            if self._max_projection_s is not None
            else sample.projection_s_m,
        )

        if state is TrackingControlState.RECOVERY:
            self._recovery_time_sec += dt
        elif state is TrackingControlState.RECAPTURE_STABLE:
            self._recapture_time_sec += dt
        elif commanded >= self.config.metrics_cruise_speed_threshold_mps:
            self._cruise_time_sec += dt
        return True

    def snapshot(self) -> TrackingMetricsSnapshot:
        count = self._sample_count
        if count:
            mean_abs_xtrack = self._sum_abs_xtrack / count
            rms_xtrack = math.sqrt(self._sum_sq_xtrack / count)
            mean_abs_heading = self._sum_abs_heading / count
            mean_commanded = self._sum_commanded_speed / count
            mean_measured = self._sum_measured_speed / count
            mean_abs_speed_error = self._sum_abs_speed_error / count
            rms_speed_error = math.sqrt(self._sum_sq_speed_error / count)
        else:
            mean_abs_xtrack = 0.0
            rms_xtrack = 0.0
            mean_abs_heading = 0.0
            mean_commanded = 0.0
            mean_measured = 0.0
            mean_abs_speed_error = 0.0
            rms_speed_error = 0.0

        return TrackingMetricsSnapshot(
            mission_identity=self._mission_identity,
            path_identity=self._path_identity,
            sample_count=count,
            rejected_sample_count=self._rejected_sample_count,
            elapsed_sec=self._elapsed_sec,
            mean_abs_cross_track_m=mean_abs_xtrack,
            rms_cross_track_m=rms_xtrack,
            p95_abs_cross_track_m=self._whole_run_p95(),
            trailing_p95_abs_cross_track_m=self._trailing_p95(),
            p95_histogram_saturated=self._p95_histogram_saturated(),
            histogram_overflow_count=self._histogram_overflow_count,
            max_abs_cross_track_m=self._max_abs_xtrack,
            mean_abs_heading_error_rad=mean_abs_heading,
            max_abs_heading_error_rad=self._max_abs_heading,
            monotonic_s_violation_count=self._monotonic_violations,
            recovery_time_sec=self._recovery_time_sec,
            recapture_time_sec=self._recapture_time_sec,
            cruise_time_sec=self._cruise_time_sec,
            mean_commanded_speed_mps=mean_commanded,
            mean_measured_speed_mps=mean_measured,
            mean_abs_speed_error_mps=mean_abs_speed_error,
            rms_speed_error_mps=rms_speed_error,
            max_abs_speed_error_mps=self._max_abs_speed_error,
            quantile_sample_count=len(self._abs_xtrack_window),
            quantile_window_capacity=self.config.metrics_quantile_window_capacity,
            discontinuity_count=self._discontinuity_count,
            last_discontinuity_reason=self._last_discontinuity_reason,
            valid_for_acceptance=self._valid_for_acceptance,
        )

    def _whole_run_p95(self) -> float:
        if not self._sample_count:
            return 0.0
        nearest_rank = max(1, math.ceil(0.95 * self._sample_count))
        cumulative = 0
        for index, count in enumerate(self._xtrack_histogram):
            cumulative += count
            if cumulative >= nearest_rank:
                return min(
                    (index + 1) * self.config.metrics_histogram_bin_width_m,
                    self.config.metrics_histogram_max_m,
                )

        # The requested quantile is inside the overflow population.  Returning
        # the observed run maximum is conservative; saturation and overflow
        # fields make the loss of bin resolution explicit to evidence users.
        return self._max_abs_xtrack

    def _p95_histogram_saturated(self) -> bool:
        if not self._sample_count:
            return False
        nearest_rank = max(1, math.ceil(0.95 * self._sample_count))
        binned_count = self._sample_count - self._histogram_overflow_count
        return nearest_rank > binned_count

    def _trailing_p95(self) -> float:
        if not self._abs_xtrack_window:
            return 0.0
        ordered = sorted(self._abs_xtrack_window)
        nearest_rank = max(1, math.ceil(0.95 * len(ordered)))
        return ordered[nearest_rank - 1]
