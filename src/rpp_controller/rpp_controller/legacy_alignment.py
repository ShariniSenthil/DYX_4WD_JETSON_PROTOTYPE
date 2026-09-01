"""ROS-free legacy native-pivot lifecycle.

Outer ownership remains ``segment_alignment_active``.  Native-carrier
generation stays in ``terminal_native_pivot_command()``.  An internal
native-request latch is not proof a pivot was published; the lifecycle
tracks ``native_carrier_issued`` only after a successful native publish.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional


__all__ = [
    "LegacyAlignmentConfig",
    "LegacyAlignmentDirective",
    "LegacyAlignmentInput",
    "LegacyAlignmentLifecycle",
    "LegacyAlignmentPhase",
    "LegacyAlignmentResult",
]


class LegacyAlignmentPhase(str, Enum):
    """Inner legacy alignment phases under segment_alignment_active."""

    ENTRY = "entry"
    PRE_PIVOT_STOP = "pre_pivot_stop"
    NATIVE_PIVOT = "native_pivot"
    PIVOT_SETTLE = "pivot_settle"
    NON_PIVOT_CAPTURE = "non_pivot_capture"
    SAFETY_HOLD = "safety_hold"


class LegacyAlignmentDirective(str, Enum):
    """Command intent returned to the ROS/PX4 adapter."""

    NATIVE_CARRIER = "native_carrier"
    HOLD_ZERO = "hold_zero"
    REANCHOR_ZERO = "reanchor_zero"
    NON_PIVOT_CAPTURE = "non_pivot_capture"
    FALLBACK_GLOBAL_XTRACK = "fallback_global_xtrack"
    COMPLETE_ZERO = "complete_zero"
    COMPLETE_FALLTHROUGH = "complete_fallthrough"
    SAFETY_HOLD = "safety_hold"


@dataclass(frozen=True, slots=True)
class LegacyAlignmentConfig:
    """Thresholds for the production native-pivot safety lifecycle."""

    native_release_heading_rad: float
    stop_speed_mps: float
    stop_yaw_rate_radps: float
    settle_sec: float
    post_settle_hold_sec: float
    non_pivot_release_xtrack_m: float
    non_pivot_release_heading_rad: float
    non_pivot_hold_sec: float
    fast_capture_max_cross_track_m: float
    pivot_enter_rad: float
    pivot_keeper_timeout_sec: float
    pre_pivot_timeout_sec: float
    stationary_violation_debounce_sec: float
    # A native pivot displaces the rover 300-600 mm off the line it was
    # about to drive (measured across 12 pivots in the Sep-1 P1 bags), most
    # of it the GPS antenna swinging through its lever-arm arc. Reanchoring
    # rebuilds the line from where the rover actually ended up. Historically
    # only the C->P1 entry leg did this, and it is the only leg that lands
    # inside the 30 mm marking latch; later legs start ~180-460 mm off-line
    # and finish 125-191 mm off the surveyed point. When True the same
    # reanchor is offered on every leg. Kept configurable so the field can
    # fall back to entry-leg-only without a rollback.
    reanchor_all_legs: bool = True

    def __post_init__(self) -> None:
        finite_fields = (
            "native_release_heading_rad",
            "stop_speed_mps",
            "stop_yaw_rate_radps",
            "settle_sec",
            "post_settle_hold_sec",
            "non_pivot_release_xtrack_m",
            "non_pivot_release_heading_rad",
            "non_pivot_hold_sec",
            "fast_capture_max_cross_track_m",
            "pivot_enter_rad",
            "pivot_keeper_timeout_sec",
            "pre_pivot_timeout_sec",
            "stationary_violation_debounce_sec",
        )
        for name in finite_fields:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")


@dataclass(frozen=True, slots=True)
class LegacyAlignmentInput:
    """Measured evidence for one legacy-alignment controller cycle."""

    now_sec: float
    telemetry_fresh: bool
    measured_speed_mps: float
    measured_yaw_rate_radps: float
    path_heading_error_rad: float
    alignment_cross_track_m: float
    native_pivot_active: bool
    first_approach: bool
    already_reanchored: bool = False
    current_x: Optional[float] = None
    current_y: Optional[float] = None


@dataclass(frozen=True, slots=True)
class LegacyAlignmentResult:
    """Adapter mapping for one lifecycle cycle."""

    phase: LegacyAlignmentPhase
    previous_phase: LegacyAlignmentPhase
    directive: LegacyAlignmentDirective
    consumed: bool
    reset_native_carrier: bool
    pivot_complete: bool
    transition_reason: str
    reanchor_requested: bool = False
    warn_native_timeout: bool = False
    native_carrier_issued: bool = False


class LegacyAlignmentLifecycle:
    """Explicit pre-stop / native-pivot / settle-reanchor-hold owner."""

    def __init__(self, config: LegacyAlignmentConfig):
        self.config = config
        self.phase = LegacyAlignmentPhase.ENTRY
        self.pivot_complete = False
        self.keeper_started_at: Optional[float] = None
        self.pre_started_at: Optional[float] = None
        self.pre_stop_inside_since: Optional[float] = None
        self.settle_inside_since: Optional[float] = None
        self.post_settle_hold_since: Optional[float] = None
        self.non_pivot_inside_since: Optional[float] = None
        self.reanchor_complete = False
        self.native_carrier_issued = False
        self.native_timeout_warned = False
        self._last_now_sec: Optional[float] = None
        self.violation_started_at: Optional[float] = None

    @property
    def needs_native_command(self) -> bool:
        return self.phase in {
            LegacyAlignmentPhase.ENTRY,
            LegacyAlignmentPhase.PRE_PIVOT_STOP,
            LegacyAlignmentPhase.NATIVE_PIVOT,
        }

    def reset(self, reason: str = "") -> None:
        """Full inner-lifecycle reset. Outer alignment ownership is unchanged."""
        del reason
        self.phase = LegacyAlignmentPhase.ENTRY
        self.pivot_complete = False
        self.keeper_started_at = None
        self.pre_started_at = None
        self.reanchor_complete = False
        self.native_carrier_issued = False
        self.native_timeout_warned = False
        self._last_now_sec = None
        self.reset_dwell_timers()

    def reset_dwell_timers(self) -> None:
        """Clear continuous-gate timers without changing phase or target."""
        self.pre_stop_inside_since = None
        self.settle_inside_since = None
        self.post_settle_hold_since = None
        self.non_pivot_inside_since = None
        self.violation_started_at = None

    def ack_native_carrier_published(self) -> None:
        """Record that a native carrier was actually published."""
        self.native_carrier_issued = True

    def ack_reanchor_completed(self) -> None:
        """Record that C' reanchor actually succeeded."""
        self.reanchor_complete = True

    def enter_safety_hold(self, reason: str = "SAFETY_HOLD") -> None:
        """Enter local SAFETY_HOLD until a semantic lifecycle reset."""
        del reason
        self.phase = LegacyAlignmentPhase.SAFETY_HOLD
        self.pivot_complete = False

    def step(self, sample: LegacyAlignmentInput) -> LegacyAlignmentResult:
        """Advance one cycle and return the adapter directive."""
        previous = self.phase
        if (
            self._last_now_sec is not None
            and sample.now_sec < self._last_now_sec
        ):
            self.reset_dwell_timers()
        self._last_now_sec = sample.now_sec

        if self.phase is LegacyAlignmentPhase.SAFETY_HOLD:
            return self._safety_hold(previous, "SAFETY_HOLD")

        if not sample.telemetry_fresh:
            self.reset_dwell_timers()
            return self._result(
                previous,
                LegacyAlignmentDirective.HOLD_ZERO,
                consumed=True,
                reset_native_carrier=False,
                reason="STALE_ODOMETRY",
            )

        if (
            self.phase is LegacyAlignmentPhase.NON_PIVOT_CAPTURE
            and abs(sample.path_heading_error_rad) >= self.config.pivot_enter_rad
        ):
            self.reset("MID_LEG_REENTRY")
            return self._result(
                previous,
                LegacyAlignmentDirective.HOLD_ZERO,
                consumed=True,
                reset_native_carrier=True,
                reason="MID_LEG_REENTRY_GE45",
            )

        if self.phase is LegacyAlignmentPhase.ENTRY:
            return self._step_entry(previous, sample)
        if self.phase is LegacyAlignmentPhase.PRE_PIVOT_STOP:
            return self._step_pre_pivot_stop(previous, sample)
        if self.phase is LegacyAlignmentPhase.NATIVE_PIVOT:
            return self._step_native_pivot(previous, sample)
        if self.phase is LegacyAlignmentPhase.PIVOT_SETTLE:
            return self._step_pivot_settle(previous, sample)
        return self._step_non_pivot_capture(previous, sample)

    def _step_entry(
        self,
        previous: LegacyAlignmentPhase,
        sample: LegacyAlignmentInput,
    ) -> LegacyAlignmentResult:
        if sample.native_pivot_active:
            self.phase = LegacyAlignmentPhase.PRE_PIVOT_STOP
            self.pivot_complete = False
            self.pre_started_at = sample.now_sec
            self.pre_stop_inside_since = None
            return self._result(
                previous,
                LegacyAlignmentDirective.HOLD_ZERO,
                consumed=True,
                reset_native_carrier=False,
                reason="PRE_PIVOT_STOP",
            )
        self.phase = LegacyAlignmentPhase.NON_PIVOT_CAPTURE
        self.pivot_complete = True
        return self._step_non_pivot_capture(previous, sample)

    def _step_pre_pivot_stop(
        self,
        previous: LegacyAlignmentPhase,
        sample: LegacyAlignmentInput,
    ) -> LegacyAlignmentResult:
        if not sample.native_pivot_active:
            if self.native_carrier_issued:
                return self._enter_pivot_settle(
                    previous,
                    "PRE_CANCEL_AFTER_CARRIER_TO_SETTLE",
                )
            self.phase = LegacyAlignmentPhase.NON_PIVOT_CAPTURE
            self.pivot_complete = True
            self.pre_started_at = None
            self.reset_dwell_timers()
            return self._result(
                previous,
                LegacyAlignmentDirective.HOLD_ZERO,
                consumed=True,
                reset_native_carrier=True,
                reason="PRE_CANCEL_NO_NATIVE_IDENTITY",
            )
        if self.pre_started_at is None:
            self.pre_started_at = sample.now_sec
        if sample.now_sec - self.pre_started_at >= self.config.pre_pivot_timeout_sec:
            self.phase = LegacyAlignmentPhase.SAFETY_HOLD
            return self._safety_hold(
                previous,
                "PRE_PIVOT_TIMEOUT",
                reset_native_carrier=True,
            )
        if not self._chassis_stationary_debounced(sample):
            self.pre_stop_inside_since = None
            return self._result(
                previous,
                LegacyAlignmentDirective.HOLD_ZERO,
                consumed=True,
                reset_native_carrier=False,
                reason="PRE_PIVOT_GATES_OPEN",
            )
        if self.pre_stop_inside_since is None:
            self.pre_stop_inside_since = sample.now_sec
        if sample.now_sec - self.pre_stop_inside_since < self.config.settle_sec:
            return self._result(
                previous,
                LegacyAlignmentDirective.HOLD_ZERO,
                consumed=True,
                reset_native_carrier=False,
                reason="PRE_PIVOT_CERTIFICATE_DWELL",
            )
        self.phase = LegacyAlignmentPhase.NATIVE_PIVOT
        self.keeper_started_at = None
        self.native_timeout_warned = False
        self.reset_dwell_timers()
        return self._result(
            previous,
            LegacyAlignmentDirective.HOLD_ZERO,
            consumed=True,
            reset_native_carrier=False,
            reason="PRE_PIVOT_COMPLETE_ZERO",
        )

    def _step_native_pivot(
        self,
        previous: LegacyAlignmentPhase,
        sample: LegacyAlignmentInput,
    ) -> LegacyAlignmentResult:
        if sample.native_pivot_active:
            if self.keeper_started_at is None:
                self.keeper_started_at = sample.now_sec
            elapsed = sample.now_sec - self.keeper_started_at
            warn = False
            if elapsed > self.config.pivot_keeper_timeout_sec:
                if not self.native_timeout_warned:
                    warn = True
                    self.native_timeout_warned = True
            return self._result(
                previous,
                LegacyAlignmentDirective.NATIVE_CARRIER,
                consumed=True,
                reset_native_carrier=False,
                reason="NATIVE_PIVOT",
                warn_native_timeout=warn,
            )
        if self.native_carrier_issued:
            return self._enter_pivot_settle(previous, "NATIVE_RELEASE_TO_SETTLE")
        self.phase = LegacyAlignmentPhase.NON_PIVOT_CAPTURE
        self.pivot_complete = True
        self.keeper_started_at = None
        self.native_timeout_warned = False
        self.reset_dwell_timers()
        return self._result(
            previous,
            LegacyAlignmentDirective.HOLD_ZERO,
            consumed=True,
            reset_native_carrier=True,
            reason="NATIVE_RELEASE_NO_CARRIER",
        )

    def _step_pivot_settle(
        self,
        previous: LegacyAlignmentPhase,
        sample: LegacyAlignmentInput,
    ) -> LegacyAlignmentResult:
        heading_abs = abs(sample.path_heading_error_rad)
        if heading_abs >= self.config.pivot_enter_rad:
            return self._enter_pre_pivot_stop(previous, sample, "SETTLE_ESCALATE_GE45")
        if not self._chassis_stationary_debounced(sample):
            self.reset_dwell_timers()
            return self._result(
                previous,
                LegacyAlignmentDirective.HOLD_ZERO,
                consumed=True,
                reset_native_carrier=True,
                reason="SETTLE_GATES_OPEN",
            )
        return self._settle_certificate_and_hold(previous, sample)

    def _settle_certificate_and_hold(
        self,
        previous: LegacyAlignmentPhase,
        sample: LegacyAlignmentInput,
    ) -> LegacyAlignmentResult:
        if self.settle_inside_since is None:
            self.settle_inside_since = sample.now_sec
        if sample.now_sec - self.settle_inside_since < self.config.settle_sec:
            self.post_settle_hold_since = None
            return self._result(
                previous,
                LegacyAlignmentDirective.HOLD_ZERO,
                consumed=True,
                reset_native_carrier=True,
                reason="SETTLE_CERTIFICATE_DWELL",
            )
        reanchor = (
            (sample.first_approach or self.config.reanchor_all_legs)
            and not self.reanchor_complete
            and not sample.already_reanchored
            and self.native_carrier_issued
        )
        if reanchor:
            return self._result(
                previous,
                LegacyAlignmentDirective.REANCHOR_ZERO,
                consumed=True,
                reset_native_carrier=True,
                reason="REANCHOR_PENDING",
                reanchor_requested=True,
            )
        if self.post_settle_hold_since is None:
            self.post_settle_hold_since = sample.now_sec
        if (
            sample.now_sec - self.post_settle_hold_since
            < self.config.post_settle_hold_sec
        ):
            return self._result(
                previous,
                LegacyAlignmentDirective.HOLD_ZERO,
                consumed=True,
                reset_native_carrier=True,
                reason="POST_SETTLE_HOLD",
            )
        self.pivot_complete = True
        self.keeper_started_at = None
        return self._result(
            previous,
            LegacyAlignmentDirective.COMPLETE_ZERO,
            consumed=True,
            reset_native_carrier=True,
            reason="SETTLE_HOLD_COMPLETE",
        )

    def _enter_pre_pivot_stop(
        self,
        previous: LegacyAlignmentPhase,
        sample: LegacyAlignmentInput,
        reason: str,
    ) -> LegacyAlignmentResult:
        self.phase = LegacyAlignmentPhase.PRE_PIVOT_STOP
        self.pivot_complete = False
        self.pre_started_at = sample.now_sec
        self.keeper_started_at = None
        self.native_timeout_warned = False
        # A genuine new pivot can displace the rover, so any prior C'->P1
        # reanchor is stale until this pivot certifies and reanchors again.
        self.reanchor_complete = False
        self.reset_dwell_timers()
        return self._result(
            previous,
            LegacyAlignmentDirective.HOLD_ZERO,
            consumed=True,
            reset_native_carrier=True,
            reason=reason,
        )

    def _enter_pivot_settle(
        self,
        previous: LegacyAlignmentPhase,
        reason: str,
    ) -> LegacyAlignmentResult:
        self.phase = LegacyAlignmentPhase.PIVOT_SETTLE
        self.pivot_complete = False
        self.keeper_started_at = None
        self.native_timeout_warned = False
        self.pre_started_at = None
        self.reset_dwell_timers()
        return self._result(
            previous,
            LegacyAlignmentDirective.HOLD_ZERO,
            consumed=True,
            reset_native_carrier=True,
            reason=reason,
        )

    def _step_non_pivot_capture(
        self,
        previous: LegacyAlignmentPhase,
        sample: LegacyAlignmentInput,
    ) -> LegacyAlignmentResult:
        if self.native_carrier_issued:
            return self._enter_pivot_settle(
                previous,
                "INVARIANT_NO_NONPIVOT_AFTER_CARRIER",
            )
        self.pivot_complete = True
        if (
            abs(sample.alignment_cross_track_m)
            > self.config.fast_capture_max_cross_track_m
        ):
            return self._result(
                previous,
                LegacyAlignmentDirective.FALLBACK_GLOBAL_XTRACK,
                consumed=False,
                reset_native_carrier=True,
                reason="NON_PIVOT_XTRACK_FALLBACK",
            )
        release_ok = (
            abs(sample.path_heading_error_rad)
            <= self.config.non_pivot_release_heading_rad
            and abs(sample.alignment_cross_track_m)
            <= self.config.non_pivot_release_xtrack_m
        )
        if release_ok:
            if self.non_pivot_inside_since is None:
                self.non_pivot_inside_since = sample.now_sec
            if (
                sample.now_sec - self.non_pivot_inside_since
                >= self.config.non_pivot_hold_sec
            ):
                return self._result(
                    previous,
                    LegacyAlignmentDirective.COMPLETE_FALLTHROUGH,
                    consumed=False,
                    reset_native_carrier=True,
                    reason="NON_PIVOT_CAPTURE_COMPLETE",
                )
        else:
            self.non_pivot_inside_since = None
        return self._result(
            previous,
            LegacyAlignmentDirective.NON_PIVOT_CAPTURE,
            consumed=True,
            reset_native_carrier=True,
            reason="NON_PIVOT_CAPTURE",
        )

    def _chassis_stationary(self, sample: LegacyAlignmentInput) -> bool:
        return (
            math.isfinite(sample.measured_speed_mps)
            and abs(sample.measured_speed_mps) <= self.config.stop_speed_mps
            and math.isfinite(sample.measured_yaw_rate_radps)
            and abs(sample.measured_yaw_rate_radps)
            <= self.config.stop_yaw_rate_radps
        )

    def _chassis_stationary_debounced(self, sample: LegacyAlignmentInput) -> bool:
        """Stationary gate used to decide whether dwell timers survive.

        A lone out-of-tolerance sample (GPS-antenna lever-arm noise during
        residual yaw settling is the known source here) no longer discards
        already-earned dwell progress. Only a violation that persists past
        ``stationary_violation_debounce_sec`` is treated as real motion and
        allowed to reset the certificate/hold timers.
        """
        if self._chassis_stationary(sample):
            self.violation_started_at = None
            return True
        if self.violation_started_at is None:
            self.violation_started_at = sample.now_sec
        elapsed = sample.now_sec - self.violation_started_at
        if elapsed < self.config.stationary_violation_debounce_sec:
            return True
        return False

    def _safety_hold(
        self,
        previous: LegacyAlignmentPhase,
        reason: str,
        *,
        reset_native_carrier: bool = False,
    ) -> LegacyAlignmentResult:
        self.phase = LegacyAlignmentPhase.SAFETY_HOLD
        self.pivot_complete = False
        return self._result(
            previous,
            LegacyAlignmentDirective.SAFETY_HOLD,
            consumed=True,
            reset_native_carrier=reset_native_carrier,
            reason=reason,
        )

    def _result(
        self,
        previous: LegacyAlignmentPhase,
        directive: LegacyAlignmentDirective,
        *,
        consumed: bool,
        reset_native_carrier: bool,
        reason: str,
        reanchor_requested: bool = False,
        warn_native_timeout: bool = False,
    ) -> LegacyAlignmentResult:
        return LegacyAlignmentResult(
            phase=self.phase,
            previous_phase=previous,
            directive=directive,
            consumed=consumed,
            reset_native_carrier=reset_native_carrier,
            pivot_complete=self.pivot_complete,
            transition_reason=reason,
            reanchor_requested=reanchor_requested,
            warn_native_timeout=warn_native_timeout,
            native_carrier_issued=self.native_carrier_issued,
        )
