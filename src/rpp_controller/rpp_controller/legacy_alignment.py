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
    "compute_low_energy_realign_command",
]


class LegacyAlignmentPhase(str, Enum):
    """Inner legacy alignment phases under segment_alignment_active."""

    ENTRY = "entry"
    PRE_PIVOT_STOP = "pre_pivot_stop"
    NATIVE_PIVOT = "native_pivot"
    PIVOT_SETTLE = "pivot_settle"
    LOW_ENERGY_REALIGN = "low_energy_realign"
    POST_PIVOT_RECAPTURE = "post_pivot_recapture"
    NON_PIVOT_CAPTURE = "non_pivot_capture"
    SAFETY_HOLD = "safety_hold"


class LegacyAlignmentDirective(str, Enum):
    """Command intent returned to the ROS/PX4 adapter."""

    NATIVE_CARRIER = "native_carrier"
    HOLD_ZERO = "hold_zero"
    LOW_ENERGY_REALIGN = "low_energy_realign"
    REANCHOR_ZERO = "reanchor_zero"
    RECAPTURE = "recapture"
    NON_PIVOT_CAPTURE = "non_pivot_capture"
    FALLBACK_GLOBAL_XTRACK = "fallback_global_xtrack"
    COMPLETE_ZERO = "complete_zero"
    COMPLETE_FALLTHROUGH = "complete_fallthrough"
    SAFETY_HOLD = "safety_hold"


@dataclass(frozen=True, slots=True)
class LegacyAlignmentConfig:
    """Thresholds for the production native-pivot safety lifecycle."""

    native_release_heading_rad: float
    tight_heading_rad: float
    stop_speed_mps: float
    stop_yaw_rate_radps: float
    settle_sec: float
    post_settle_hold_sec: float
    recapture_xtrack_m: float
    recapture_heading_rad: float
    recapture_settle_sec: float
    non_pivot_release_xtrack_m: float
    non_pivot_release_heading_rad: float
    non_pivot_hold_sec: float
    fast_capture_max_cross_track_m: float
    pivot_enter_rad: float
    pivot_keeper_timeout_sec: float
    pre_pivot_timeout_sec: float
    realign_grace_sec: float
    realign_split_heading_rad: float
    realign_near_speed_mps: float
    realign_far_speed_mps: float
    realign_bearing_cone_rad: float
    realign_max_translation_m: float
    realign_timeout_sec: float

    def __post_init__(self) -> None:
        finite_fields = (
            "native_release_heading_rad",
            "tight_heading_rad",
            "stop_speed_mps",
            "stop_yaw_rate_radps",
            "settle_sec",
            "post_settle_hold_sec",
            "recapture_xtrack_m",
            "recapture_heading_rad",
            "recapture_settle_sec",
            "non_pivot_release_xtrack_m",
            "non_pivot_release_heading_rad",
            "non_pivot_hold_sec",
            "fast_capture_max_cross_track_m",
            "pivot_enter_rad",
            "pivot_keeper_timeout_sec",
            "pre_pivot_timeout_sec",
            "realign_grace_sec",
            "realign_split_heading_rad",
            "realign_near_speed_mps",
            "realign_far_speed_mps",
            "realign_bearing_cone_rad",
            "realign_max_translation_m",
            "realign_timeout_sec",
        )
        for name in finite_fields:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
        if self.tight_heading_rad > self.native_release_heading_rad:
            raise ValueError(
                "tight_heading_rad must not exceed native_release_heading_rad"
            )
        if self.recapture_xtrack_m > self.fast_capture_max_cross_track_m:
            raise ValueError(
                "recapture_xtrack_m must not exceed "
                "fast_capture_max_cross_track_m"
            )
        if not (
            self.tight_heading_rad
            < self.realign_split_heading_rad
            < self.pivot_enter_rad
        ):
            raise ValueError(
                "realign_split_heading_rad must satisfy 2deg < split < 45deg"
            )
        if self.realign_far_speed_mps > self.realign_near_speed_mps:
            raise ValueError(
                "realign_far_speed_mps must be <= realign_near_speed_mps"
            )
        if self.realign_bearing_cone_rad >= self.pivot_enter_rad:
            raise ValueError(
                "realign_bearing_cone_rad must be in (0, 45deg)"
            )


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
    realign_turn_sign: Optional[float]
    transition_reason: str
    reanchor_requested: bool = False
    warn_native_timeout: bool = False
    native_carrier_issued: bool = False


def compute_low_energy_realign_command(
    heading_error_rad,
    current_yaw_rad,
    *,
    split_heading_rad,
    near_speed_mps,
    far_speed_mps,
    bearing_cone_rad,
):
    """Return bearing, speed, and NED components for low-energy realign."""
    error = math.atan2(math.sin(heading_error_rad), math.cos(heading_error_rad))
    clamped = max(-bearing_cone_rad, min(bearing_cone_rad, error))
    bearing_cmd = math.atan2(
        math.sin(current_yaw_rad + clamped),
        math.cos(current_yaw_rad + clamped),
    )
    speed = near_speed_mps if abs(error) <= split_heading_rad else far_speed_mps
    north = speed * math.sin(bearing_cmd)
    east = speed * math.cos(bearing_cmd)
    return bearing_cmd, speed, north, east


class LegacyAlignmentLifecycle:
    """Explicit pre-stop / native-pivot / settle / recapture owner."""

    def __init__(self, config: LegacyAlignmentConfig):
        self.config = config
        self.phase = LegacyAlignmentPhase.ENTRY
        self.pivot_complete = False
        self.keeper_started_at: Optional[float] = None
        self.pre_started_at: Optional[float] = None
        self.pre_stop_inside_since: Optional[float] = None
        self.settle_inside_since: Optional[float] = None
        self.post_settle_hold_since: Optional[float] = None
        self.realign_grace_since: Optional[float] = None
        self.recapture_inside_since: Optional[float] = None
        self.non_pivot_inside_since: Optional[float] = None
        self.realign_turn_sign: Optional[float] = None
        self.reanchor_complete = False
        self.native_carrier_issued = False
        self.native_timeout_warned = False
        self.realign_start_x: Optional[float] = None
        self.realign_start_y: Optional[float] = None
        self.realign_started_at: Optional[float] = None
        self._last_now_sec: Optional[float] = None

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
        self.realign_turn_sign = None
        self.reanchor_complete = False
        self.native_carrier_issued = False
        self.native_timeout_warned = False
        self.realign_start_x = None
        self.realign_start_y = None
        self.realign_started_at = None
        self._last_now_sec = None
        self.reset_dwell_timers()

    def reset_dwell_timers(self) -> None:
        """Clear continuous-gate timers without changing phase or target."""
        self.pre_stop_inside_since = None
        self.settle_inside_since = None
        self.post_settle_hold_since = None
        self.realign_grace_since = None
        self.recapture_inside_since = None
        self.non_pivot_inside_since = None

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

        if self.phase in {
            LegacyAlignmentPhase.POST_PIVOT_RECAPTURE,
            LegacyAlignmentPhase.NON_PIVOT_CAPTURE,
        } and abs(sample.path_heading_error_rad) >= self.config.pivot_enter_rad:
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
        if self.phase is LegacyAlignmentPhase.LOW_ENERGY_REALIGN:
            return self._step_low_energy_realign(previous, sample)
        if self.phase is LegacyAlignmentPhase.POST_PIVOT_RECAPTURE:
            return self._step_post_pivot_recapture(previous, sample)
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
        if not self._chassis_stationary(sample):
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
        heading_tight = self._heading_tight(sample)
        stationary = self._chassis_stationary(sample)
        if heading_tight and stationary:
            self.realign_grace_since = None
            return self._settle_certificate_and_hold(previous, sample)
        if not stationary:
            self.reset_dwell_timers()
            return self._result(
                previous,
                LegacyAlignmentDirective.HOLD_ZERO,
                consumed=True,
                reset_native_carrier=True,
                reason="SETTLE_GATES_OPEN",
            )
        self.settle_inside_since = None
        self.post_settle_hold_since = None
        if self.realign_grace_since is None:
            self.realign_grace_since = sample.now_sec
        if sample.now_sec - self.realign_grace_since < self.config.realign_grace_sec:
            return self._result(
                previous,
                LegacyAlignmentDirective.HOLD_ZERO,
                consumed=True,
                reset_native_carrier=True,
                reason="REALIGN_GRACE",
            )
        self.phase = LegacyAlignmentPhase.LOW_ENERGY_REALIGN
        self.realign_start_x = sample.current_x
        self.realign_start_y = sample.current_y
        self.realign_started_at = sample.now_sec
        self.reset_dwell_timers()
        return self._result(
            previous,
            LegacyAlignmentDirective.HOLD_ZERO,
            consumed=True,
            reset_native_carrier=True,
            reason="ENTER_LOW_ENERGY_REALIGN_ZERO",
        )

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
        self.phase = LegacyAlignmentPhase.POST_PIVOT_RECAPTURE
        self.pivot_complete = True
        self.keeper_started_at = None
        self.reset_dwell_timers()
        reanchor = (
            sample.first_approach
            and not self.reanchor_complete
            and not sample.already_reanchored
            and self.native_carrier_issued
        )
        return self._result(
            previous,
            (
                LegacyAlignmentDirective.REANCHOR_ZERO
                if reanchor
                else LegacyAlignmentDirective.HOLD_ZERO
            ),
            consumed=True,
            reset_native_carrier=True,
            reason="SETTLE_HOLD_COMPLETE",
            reanchor_requested=reanchor,
        )

    def _step_low_energy_realign(
        self,
        previous: LegacyAlignmentPhase,
        sample: LegacyAlignmentInput,
    ) -> LegacyAlignmentResult:
        heading_abs = abs(sample.path_heading_error_rad)
        if heading_abs >= self.config.pivot_enter_rad:
            return self._enter_pre_pivot_stop(
                previous, sample, "REALIGN_ESCALATE_GE45"
            )
        if self._heading_tight(sample):
            self.phase = LegacyAlignmentPhase.PIVOT_SETTLE
            self.realign_start_x = None
            self.realign_start_y = None
            self.realign_started_at = None
            self.reset_dwell_timers()
            return self._result(
                previous,
                LegacyAlignmentDirective.HOLD_ZERO,
                consumed=True,
                reset_native_carrier=True,
                reason="REALIGN_HEADING_TIGHT",
            )
        if self._low_energy_watchdog_failed(sample):
            self.phase = LegacyAlignmentPhase.SAFETY_HOLD
            return self._safety_hold(previous, "LOW_ENERGY_WATCHDOG")
        return self._result(
            previous,
            LegacyAlignmentDirective.LOW_ENERGY_REALIGN,
            consumed=True,
            reset_native_carrier=True,
            reason="LOW_ENERGY_REALIGN",
        )

    def _low_energy_watchdog_failed(self, sample: LegacyAlignmentInput) -> bool:
        if self.realign_started_at is None:
            self.realign_started_at = sample.now_sec
        if not math.isfinite(sample.now_sec) or not math.isfinite(
            self.realign_started_at
        ):
            return True
        if sample.now_sec - self.realign_started_at >= self.config.realign_timeout_sec:
            return True
        values = (
            sample.current_x,
            sample.current_y,
            self.realign_start_x,
            self.realign_start_y,
        )
        if any(value is None or not math.isfinite(float(value)) for value in values):
            return True
        translation = math.hypot(
            float(sample.current_x) - float(self.realign_start_x),
            float(sample.current_y) - float(self.realign_start_y),
        )
        return translation >= self.config.realign_max_translation_m

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
        self.realign_start_x = None
        self.realign_start_y = None
        self.realign_started_at = None
        self.reset_dwell_timers()
        return self._result(
            previous,
            LegacyAlignmentDirective.HOLD_ZERO,
            consumed=True,
            reset_native_carrier=True,
            reason=reason,
        )

    def _step_post_pivot_recapture(
        self,
        previous: LegacyAlignmentPhase,
        sample: LegacyAlignmentInput,
    ) -> LegacyAlignmentResult:
        if (
            sample.first_approach
            and self.native_carrier_issued
            and not self.reanchor_complete
            and not sample.already_reanchored
        ):
            return self._result(
                previous,
                LegacyAlignmentDirective.REANCHOR_ZERO,
                consumed=True,
                reset_native_carrier=True,
                reason="REANCHOR_PENDING",
                reanchor_requested=True,
            )
        geometry_ok = (
            abs(sample.alignment_cross_track_m) <= self.config.recapture_xtrack_m
            and abs(sample.path_heading_error_rad)
            <= self.config.recapture_heading_rad
        )
        if geometry_ok:
            if self.recapture_inside_since is None:
                self.recapture_inside_since = sample.now_sec
            if (
                sample.now_sec - self.recapture_inside_since
                >= self.config.recapture_settle_sec
            ):
                return self._result(
                    previous,
                    LegacyAlignmentDirective.COMPLETE_ZERO,
                    consumed=True,
                    reset_native_carrier=True,
                    reason="POST_PIVOT_RECAPTURE_COMPLETE",
                )
        else:
            self.recapture_inside_since = None
        return self._result(
            previous,
            LegacyAlignmentDirective.RECAPTURE,
            consumed=True,
            reset_native_carrier=True,
            reason="POST_PIVOT_RECAPTURE",
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

    def _heading_tight(self, sample: LegacyAlignmentInput) -> bool:
        return abs(sample.path_heading_error_rad) <= self.config.tight_heading_rad

    def _chassis_stationary(self, sample: LegacyAlignmentInput) -> bool:
        return (
            math.isfinite(sample.measured_speed_mps)
            and abs(sample.measured_speed_mps) <= self.config.stop_speed_mps
            and math.isfinite(sample.measured_yaw_rate_radps)
            and abs(sample.measured_yaw_rate_radps)
            <= self.config.stop_yaw_rate_radps
        )

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
            realign_turn_sign=self.realign_turn_sign,
            transition_reason=reason,
            reanchor_requested=reanchor_requested,
            warn_native_timeout=warn_native_timeout,
            native_carrier_issued=self.native_carrier_issued,
        )
