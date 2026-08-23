"""ROS-free verified pivot, recenter, and tracking-recapture state machine.

The state machine owns no vehicle commands.  It turns measured motion and
anchor/alignment evidence into explicit directives that a ROS adapter may map
to the existing PX4 command contract.  All stop and release decisions require
fresh telemetry held continuously for a configured dwell.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional


__all__ = [
    "MotionDirective",
    "MotionState",
    "PivotMotionConfig",
    "PivotMotionInput",
    "PivotMotionResult",
    "ReleaseCertificate",
    "StopCertificate",
    "VerifiedPivotStateMachine",
]


class MotionState(str, Enum):
    """Precision corner/pivot states."""

    TRACK = "track"
    CORNER_APPROACH = "corner_approach"
    BRAKE_TO_ANCHOR = "brake_to_anchor"
    STOP_SETTLE = "stop_settle"
    PIVOT = "pivot"
    PIVOT_SETTLE = "pivot_settle"
    POSITION_CHECK = "position_check"
    RECENTER = "recenter"
    REALIGN = "realign"
    RECAPTURE = "recapture"
    HOLD_FAIL = "hold_fail"


class MotionDirective(str, Enum):
    """Command intent returned to the ROS/PX4 adapter."""

    TRACK = "track"
    CORNER_APPROACH = "corner_approach"
    BRAKE_TO_ANCHOR = "brake_to_anchor"
    HOLD_ZERO = "hold_zero"
    PIVOT = "pivot"
    RECENTER = "recenter"
    REALIGN = "realign"
    RECAPTURE = "recapture"
    HOLD_FAIL = "hold_fail"


@dataclass(frozen=True, slots=True)
class PivotMotionConfig:
    """Validated thresholds and watchdogs for one pivot maneuver."""

    pivot_anchor_tolerance_m: float = 0.020
    pivot_recenter_threshold_m: float = 0.020
    stop_speed_tolerance_mps: float = 0.030
    stop_yaw_rate_tolerance_radps: float = 0.050
    release_heading_tolerance_rad: float = math.radians(2.0)
    stop_settle_sec: float = 0.20
    pivot_release_settle_sec: float = 0.20
    control_dt_max_sec: float = 0.10
    brake_timeout_sec: float = 8.0
    pivot_timeout_sec: float = 10.0
    recenter_timeout_sec: float = 5.0
    realign_timeout_sec: float = 10.0
    recapture_timeout_sec: float = 8.0
    max_recenter_attempts: int = 2

    def __post_init__(self) -> None:
        finite_fields = (
            "pivot_anchor_tolerance_m",
            "pivot_recenter_threshold_m",
            "stop_speed_tolerance_mps",
            "stop_yaw_rate_tolerance_radps",
            "release_heading_tolerance_rad",
            "stop_settle_sec",
            "pivot_release_settle_sec",
            "control_dt_max_sec",
            "brake_timeout_sec",
            "pivot_timeout_sec",
            "recenter_timeout_sec",
            "realign_timeout_sec",
            "recapture_timeout_sec",
        )
        for name in finite_fields:
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite")

        positive_fields = (
            "pivot_anchor_tolerance_m",
            "stop_speed_tolerance_mps",
            "stop_yaw_rate_tolerance_radps",
            "release_heading_tolerance_rad",
            "stop_settle_sec",
            "pivot_release_settle_sec",
            "control_dt_max_sec",
            "brake_timeout_sec",
            "pivot_timeout_sec",
            "recenter_timeout_sec",
            "realign_timeout_sec",
            "recapture_timeout_sec",
        )
        for name in positive_fields:
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be greater than zero")

        if not math.isclose(
            self.pivot_recenter_threshold_m,
            self.pivot_anchor_tolerance_m,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise ValueError(
                "pivot_recenter_threshold_m must equal "
                "pivot_anchor_tolerance_m"
            )
        if self.release_heading_tolerance_rad > math.pi:
            raise ValueError("release_heading_tolerance_rad must not exceed pi")
        if (
            isinstance(self.max_recenter_attempts, bool)
            or not isinstance(self.max_recenter_attempts, int)
            or self.max_recenter_attempts < 0
        ):
            raise ValueError("max_recenter_attempts must be a non-negative integer")


@dataclass(frozen=True, slots=True)
class PivotMotionInput:
    """Measured evidence and semantic requests for one controller cycle."""

    monotonic_time_sec: float
    dt_sec: float
    anchor_radial_error_m: float
    measured_linear_speed_mps: float
    measured_yaw_rate_radps: float
    heading_error_rad: float
    telemetry_fresh: bool
    pivot_requested: bool = False
    brake_to_anchor_requested: bool = False
    recapture_complete: bool = False


@dataclass(frozen=True, slots=True)
class StopCertificate:
    """Evidence needed before a stationary alignment command may begin."""

    position_ok: bool
    linear_speed_ok: bool
    yaw_rate_ok: bool
    telemetry_fresh: bool
    held_sec: float
    required_sec: float
    valid: bool


@dataclass(frozen=True, slots=True)
class ReleaseCertificate:
    """Evidence needed before forward tracking may be released."""

    position_ok: bool
    heading_ok: bool
    linear_speed_ok: bool
    yaw_rate_ok: bool
    telemetry_fresh: bool
    held_sec: float
    required_sec: float
    valid: bool


@dataclass(frozen=True, slots=True)
class PivotMotionResult:
    """Auditable state, directive, and certificates after one FSM tick."""

    state: MotionState
    previous_state: MotionState
    directive: MotionDirective
    transition_reason: str
    stop_certificate: StopCertificate
    release_certificate: ReleaseCertificate
    max_pivot_drift_m: float
    recenter_attempts: int
    bounded_dt_sec: float
    state_elapsed_sec: float
    failed: bool


class VerifiedPivotStateMachine:
    """Measured stop/pivot/recenter/release state machine.

    ``reset`` and construction both start in :attr:`MotionState.TRACK`.  The
    adapter requests a maneuver with ``pivot_requested`` and later indicates
    that geometric corner preview has entered its braking region with
    ``brake_to_anchor_requested``.  A completed recapture is an explicit input;
    the FSM never guesses it from anchor geometry after forward motion starts.
    """

    def __init__(self, config: Optional[PivotMotionConfig] = None) -> None:
        self.config = config or PivotMotionConfig()
        self.reset()

    @property
    def state(self) -> MotionState:
        """Return the current state."""

        return self._state

    def reset(self, *, monotonic_time_sec: float = 0.0) -> None:
        """Cancel any maneuver and return to the legacy-safe tracking state."""

        if not math.isfinite(monotonic_time_sec) or monotonic_time_sec < 0.0:
            raise ValueError("monotonic_time_sec must be finite and non-negative")
        self._state = MotionState.TRACK
        self._state_enter_time_sec = monotonic_time_sec
        self._last_time_sec = monotonic_time_sec
        self._alignment_start_time_sec: Optional[float] = None
        self._stop_settle_destination = MotionState.PIVOT
        self._stop_held_sec = 0.0
        self._release_held_sec = 0.0
        self._max_pivot_drift_m = 0.0
        self._recenter_attempts = 0

    def step(self, sample: PivotMotionInput) -> PivotMotionResult:
        """Advance one bounded-dt cycle and return an explicit directive."""

        self._validate_input(sample)
        if sample.monotonic_time_sec < self._last_time_sec:
            raise ValueError("monotonic_time_sec must not move backwards")
        self._last_time_sec = sample.monotonic_time_sec
        bounded_dt = min(max(sample.dt_sec, 0.0), self.config.control_dt_max_sec)

        previous_state = self._state
        transition_reason = ""

        position_ok = (
            sample.telemetry_fresh
            and sample.anchor_radial_error_m
            <= self.config.pivot_anchor_tolerance_m
        )
        speed_ok = (
            sample.telemetry_fresh
            and abs(sample.measured_linear_speed_mps)
            <= self.config.stop_speed_tolerance_mps
        )
        yaw_rate_ok = (
            sample.telemetry_fresh
            and abs(sample.measured_yaw_rate_radps)
            <= self.config.stop_yaw_rate_tolerance_radps
        )
        heading_ok = (
            sample.telemetry_fresh
            and abs(self._wrap_pi(sample.heading_error_rad))
            <= self.config.release_heading_tolerance_rad
        )

        stop_gates_ok = position_ok and speed_ok and yaw_rate_ok
        release_gates_ok = stop_gates_ok and heading_ok
        self._stop_held_sec = (
            self._stop_held_sec + bounded_dt if stop_gates_ok else 0.0
        )
        self._release_held_sec = (
            self._release_held_sec + bounded_dt if release_gates_ok else 0.0
        )
        stop_held_for_result = self._stop_held_sec
        release_held_for_result = self._release_held_sec

        if self._state in {
            MotionState.PIVOT,
            MotionState.PIVOT_SETTLE,
            MotionState.POSITION_CHECK,
            MotionState.RECENTER,
            MotionState.REALIGN,
        }:
            self._max_pivot_drift_m = max(
                self._max_pivot_drift_m, sample.anchor_radial_error_m
            )

        if self._state is MotionState.TRACK:
            if sample.pivot_requested:
                self._begin_maneuver(sample.monotonic_time_sec)
                transition_reason = self._transition(
                    MotionState.CORNER_APPROACH,
                    sample.monotonic_time_sec,
                    "pivot_requested",
                )

        elif self._state is MotionState.CORNER_APPROACH:
            if (
                not sample.telemetry_fresh
                and self._state_elapsed(sample) >= self.config.brake_timeout_sec
            ):
                transition_reason = self._fail(
                    sample.monotonic_time_sec, "corner_approach_telemetry_timeout"
                )
            elif not sample.telemetry_fresh:
                pass
            elif not sample.pivot_requested:
                transition_reason = self._transition(
                    MotionState.TRACK,
                    sample.monotonic_time_sec,
                    "pivot_request_cancelled",
                )
            elif sample.brake_to_anchor_requested:
                transition_reason = self._transition(
                    MotionState.BRAKE_TO_ANCHOR,
                    sample.monotonic_time_sec,
                    "corner_braking_required",
                )

        elif self._state is MotionState.BRAKE_TO_ANCHOR:
            if position_ok:
                self._stop_settle_destination = MotionState.PIVOT
                transition_reason = self._transition(
                    MotionState.STOP_SETTLE,
                    sample.monotonic_time_sec,
                    "anchor_position_reached",
                )
            elif self._state_elapsed(sample) >= self.config.brake_timeout_sec:
                transition_reason = self._fail(
                    sample.monotonic_time_sec, "brake_to_anchor_timeout"
                )

        elif self._state is MotionState.STOP_SETTLE:
            if not sample.telemetry_fresh:
                if self._state_elapsed(sample) >= self.config.brake_timeout_sec:
                    transition_reason = self._fail(
                        sample.monotonic_time_sec, "stop_settle_telemetry_timeout"
                    )
            elif not position_ok:
                destination = (
                    MotionState.BRAKE_TO_ANCHOR
                    if self._stop_settle_destination is MotionState.PIVOT
                    else MotionState.RECENTER
                )
                if destination is MotionState.RECENTER:
                    transition_reason = self._enter_recenter(
                        sample.monotonic_time_sec, "anchor_lost_during_stop_settle"
                    )
                else:
                    transition_reason = self._transition(
                        destination,
                        sample.monotonic_time_sec,
                        "anchor_lost_during_stop_settle",
                    )
            elif self._stop_held_sec >= self.config.stop_settle_sec:
                destination = self._stop_settle_destination
                if destination in {MotionState.PIVOT, MotionState.REALIGN}:
                    self._alignment_start_time_sec = sample.monotonic_time_sec
                transition_reason = self._transition(
                    destination,
                    sample.monotonic_time_sec,
                    "verified_stop_settled",
                )

        elif self._state is MotionState.PIVOT:
            if self._alignment_timed_out(sample, self.config.pivot_timeout_sec):
                transition_reason = self._fail(
                    sample.monotonic_time_sec, "pivot_timeout"
                )
            elif (
                sample.telemetry_fresh
                and sample.anchor_radial_error_m
                > self.config.pivot_recenter_threshold_m
            ):
                transition_reason = self._enter_recenter(
                    sample.monotonic_time_sec, "pivot_drift_exceeded"
                )
            elif heading_ok:
                transition_reason = self._transition(
                    MotionState.PIVOT_SETTLE,
                    sample.monotonic_time_sec,
                    "heading_entered_release_tolerance",
                )

        elif self._state is MotionState.PIVOT_SETTLE:
            if self._alignment_timed_out(sample, self.config.pivot_timeout_sec):
                transition_reason = self._fail(
                    sample.monotonic_time_sec, "pivot_settle_timeout"
                )
            elif not sample.telemetry_fresh:
                pass
            elif not position_ok:
                transition_reason = self._enter_recenter(
                    sample.monotonic_time_sec, "position_bad_during_pivot_settle"
                )
            elif not heading_ok:
                transition_reason = self._transition(
                    MotionState.PIVOT,
                    sample.monotonic_time_sec,
                    "heading_left_release_tolerance",
                )
            elif self._release_held_sec >= self.config.pivot_release_settle_sec:
                transition_reason = self._transition(
                    MotionState.POSITION_CHECK,
                    sample.monotonic_time_sec,
                    "verified_pivot_release_settled",
                )

        elif self._state is MotionState.POSITION_CHECK:
            if not position_ok:
                transition_reason = self._enter_recenter(
                    sample.monotonic_time_sec, "position_check_failed"
                )
            elif (
                not release_gates_ok
                or self._release_held_sec
                < self.config.pivot_release_settle_sec
            ):
                self._alignment_start_time_sec = sample.monotonic_time_sec
                transition_reason = self._transition(
                    MotionState.REALIGN,
                    sample.monotonic_time_sec,
                    "release_evidence_changed",
                )
            else:
                transition_reason = self._transition(
                    MotionState.RECAPTURE,
                    sample.monotonic_time_sec,
                    "position_and_release_verified",
                )

        elif self._state is MotionState.RECENTER:
            if position_ok:
                self._stop_settle_destination = MotionState.REALIGN
                transition_reason = self._transition(
                    MotionState.STOP_SETTLE,
                    sample.monotonic_time_sec,
                    "recenter_position_recovered",
                )
            elif self._state_elapsed(sample) >= self.config.recenter_timeout_sec:
                if self._recenter_attempts < self.config.max_recenter_attempts:
                    transition_reason = self._enter_recenter(
                        sample.monotonic_time_sec, "recenter_retry"
                    )
                else:
                    transition_reason = self._fail(
                        sample.monotonic_time_sec, "recenter_attempts_exhausted"
                    )

        elif self._state is MotionState.REALIGN:
            if self._alignment_timed_out(sample, self.config.realign_timeout_sec):
                transition_reason = self._fail(
                    sample.monotonic_time_sec, "realign_timeout"
                )
            elif not sample.telemetry_fresh:
                pass
            elif not position_ok:
                transition_reason = self._enter_recenter(
                    sample.monotonic_time_sec, "position_bad_during_realign"
                )
            elif self._release_held_sec >= self.config.pivot_release_settle_sec:
                transition_reason = self._transition(
                    MotionState.RECAPTURE,
                    sample.monotonic_time_sec,
                    "verified_realign_release_settled",
                )

        elif self._state is MotionState.RECAPTURE:
            if not sample.telemetry_fresh:
                transition_reason = self._fail(
                    sample.monotonic_time_sec, "telemetry_stale_during_recapture"
                )
            elif sample.recapture_complete:
                transition_reason = self._transition(
                    MotionState.TRACK,
                    sample.monotonic_time_sec,
                    "next_leg_recaptured",
                )
            elif self._state_elapsed(sample) >= self.config.recapture_timeout_sec:
                transition_reason = self._fail(
                    sample.monotonic_time_sec, "recapture_timeout"
                )

        stop_certificate = StopCertificate(
            position_ok=position_ok,
            linear_speed_ok=speed_ok,
            yaw_rate_ok=yaw_rate_ok,
            telemetry_fresh=sample.telemetry_fresh,
            held_sec=stop_held_for_result,
            required_sec=self.config.stop_settle_sec,
            valid=(
                stop_gates_ok
                and stop_held_for_result >= self.config.stop_settle_sec
            ),
        )
        release_certificate = ReleaseCertificate(
            position_ok=position_ok,
            heading_ok=heading_ok,
            linear_speed_ok=speed_ok,
            yaw_rate_ok=yaw_rate_ok,
            telemetry_fresh=sample.telemetry_fresh,
            held_sec=release_held_for_result,
            required_sec=self.config.pivot_release_settle_sec,
            valid=(
                release_gates_ok
                and release_held_for_result
                >= self.config.pivot_release_settle_sec
            ),
        )
        directive = self._directive_for_state(self._state)
        motion_states = {
            MotionState.CORNER_APPROACH,
            MotionState.BRAKE_TO_ANCHOR,
            MotionState.PIVOT,
            MotionState.RECENTER,
            MotionState.REALIGN,
            MotionState.RECAPTURE,
        }
        if (
            not sample.telemetry_fresh
            and self._state is not MotionState.HOLD_FAIL
            and (previous_state in motion_states or self._state in motion_states)
        ):
            directive = MotionDirective.HOLD_ZERO

        return PivotMotionResult(
            state=self._state,
            previous_state=previous_state,
            directive=directive,
            transition_reason=transition_reason,
            stop_certificate=stop_certificate,
            release_certificate=release_certificate,
            max_pivot_drift_m=self._max_pivot_drift_m,
            recenter_attempts=self._recenter_attempts,
            bounded_dt_sec=bounded_dt,
            state_elapsed_sec=self._state_elapsed(sample),
            failed=self._state is MotionState.HOLD_FAIL,
        )

    def _begin_maneuver(self, now_sec: float) -> None:
        self._max_pivot_drift_m = 0.0
        self._recenter_attempts = 0
        self._stop_settle_destination = MotionState.PIVOT
        self._alignment_start_time_sec = None
        self._stop_held_sec = 0.0
        self._release_held_sec = 0.0
        self._state_enter_time_sec = now_sec

    def _transition(
        self, state: MotionState, now_sec: float, reason: str
    ) -> str:
        old_state = self._state
        self._state = state
        self._state_enter_time_sec = now_sec
        if state is MotionState.STOP_SETTLE and old_state is not state:
            self._stop_held_sec = 0.0
        elif state is not MotionState.STOP_SETTLE:
            self._stop_held_sec = 0.0
        if (
            state is MotionState.POSITION_CHECK
            and old_state is MotionState.PIVOT_SETTLE
        ):
            pass
        elif state in {MotionState.PIVOT_SETTLE, MotionState.REALIGN}:
            if old_state is not state:
                self._release_held_sec = 0.0
        else:
            self._release_held_sec = 0.0
        return reason

    def _enter_recenter(self, now_sec: float, reason: str) -> str:
        if self._recenter_attempts >= self.config.max_recenter_attempts:
            return self._fail(now_sec, "recenter_attempts_exhausted")
        self._recenter_attempts += 1
        self._alignment_start_time_sec = None
        return self._transition(MotionState.RECENTER, now_sec, reason)

    def _fail(self, now_sec: float, reason: str) -> str:
        return self._transition(MotionState.HOLD_FAIL, now_sec, reason)

    def _state_elapsed(self, sample: PivotMotionInput) -> float:
        return max(0.0, sample.monotonic_time_sec - self._state_enter_time_sec)

    def _alignment_timed_out(
        self, sample: PivotMotionInput, timeout_sec: float
    ) -> bool:
        if self._alignment_start_time_sec is None:
            self._alignment_start_time_sec = sample.monotonic_time_sec
        return (
            sample.monotonic_time_sec - self._alignment_start_time_sec
            >= timeout_sec
        )

    @staticmethod
    def _wrap_pi(angle_rad: float) -> float:
        return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _directive_for_state(state: MotionState) -> MotionDirective:
        return {
            MotionState.TRACK: MotionDirective.TRACK,
            MotionState.CORNER_APPROACH: MotionDirective.CORNER_APPROACH,
            MotionState.BRAKE_TO_ANCHOR: MotionDirective.BRAKE_TO_ANCHOR,
            MotionState.STOP_SETTLE: MotionDirective.HOLD_ZERO,
            MotionState.PIVOT: MotionDirective.PIVOT,
            MotionState.PIVOT_SETTLE: MotionDirective.HOLD_ZERO,
            MotionState.POSITION_CHECK: MotionDirective.HOLD_ZERO,
            MotionState.RECENTER: MotionDirective.RECENTER,
            MotionState.REALIGN: MotionDirective.REALIGN,
            MotionState.RECAPTURE: MotionDirective.RECAPTURE,
            MotionState.HOLD_FAIL: MotionDirective.HOLD_FAIL,
        }[state]

    @staticmethod
    def _validate_input(sample: PivotMotionInput) -> None:
        finite_fields = (
            "monotonic_time_sec",
            "dt_sec",
            "anchor_radial_error_m",
            "measured_linear_speed_mps",
            "measured_yaw_rate_radps",
            "heading_error_rad",
        )
        for name in finite_fields:
            if not math.isfinite(getattr(sample, name)):
                raise ValueError(f"{name} must be finite")
        if sample.monotonic_time_sec < 0.0:
            raise ValueError("monotonic_time_sec must be non-negative")
        if sample.anchor_radial_error_m < 0.0:
            raise ValueError("anchor_radial_error_m must be non-negative")
        bool_fields = (
            "telemetry_fresh",
            "pivot_requested",
            "brake_to_anchor_requested",
            "recapture_complete",
        )
        for name in bool_fields:
            if not isinstance(getattr(sample, name), bool):
                raise ValueError(f"{name} must be bool")
