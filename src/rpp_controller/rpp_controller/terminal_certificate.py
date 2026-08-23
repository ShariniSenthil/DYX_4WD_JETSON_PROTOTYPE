"""ROS-free precision terminal stop and certification state machine.

This module deliberately owns neither navigation progress nor velocity
generation.  It converts terminal intent and measured controller-frame
evidence into a small set of command directives.  Once a terminal identity is
captured, zero motion is latched until an explicit cancel, identity change, or
semantic-boundary reset.

The resulting certificate proves only what the controller observed in its
estimator frame.  It does not certify localization accuracy or the physical
placement of a marking tool.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real
from typing import Any, Optional


__all__ = [
    "ControllerPose",
    "PrecisionTerminalCertificate",
    "TerminalConfig",
    "TerminalDirective",
    "TerminalInput",
    "TerminalResult",
    "TerminalState",
    "TerminalStopStateMachine",
]


class TerminalState(str, Enum):
    """Explicit phases of a precision terminal stop."""

    TRACK = "track"
    APPROACH = "approach"
    BRAKE = "brake"
    CAPTURE = "capture"
    ZERO_LATCH = "zero_latch"
    SETTLE = "settle"
    CERTIFIED = "certified"
    HOLD_FAIL = "hold_fail"


class TerminalDirective(str, Enum):
    """Command intent for the ROS adapter; this module never emits speed."""

    TRACK = "track"
    APPROACH = "approach"
    BRAKE = "brake"
    HOLD_ZERO = "hold_zero"
    HOLD_FAIL = "hold_fail"


@dataclass(frozen=True, slots=True)
class ControllerPose:
    """Minimal immutable controller-frame pose evidence."""

    x_m: float
    y_m: float
    yaw_rad: Optional[float] = None

    def __post_init__(self) -> None:
        if not _is_finite_real(self.x_m) or not _is_finite_real(self.y_m):
            raise ValueError("controller pose x_m and y_m must be finite")
        if self.yaw_rad is not None and not _is_finite_real(self.yaw_rad):
            raise ValueError("controller pose yaw_rad must be finite when present")

    def to_dict(self) -> dict[str, Optional[float]]:
        """Return a deterministic JSON-serializable representation."""

        return {
            "x_m": float(self.x_m),
            "y_m": float(self.y_m),
            "yaw_rad": None if self.yaw_rad is None else float(self.yaw_rad),
        }


@dataclass(frozen=True, slots=True)
class TerminalConfig:
    """Validated precision-stop thresholds and watchdogs.

    ``minimum_actuatable_speed_mps`` is calibration metadata for the adapter's
    approach regulator.  The terminal FSM never converts it into a command.
    """

    terminal_radial_tolerance_m: float = 0.010
    capture_entry_tolerance_m: float = 0.010
    settle_radial_tolerance_m: float = 0.010
    stop_speed_tolerance_mps: float = 0.010
    stop_yaw_rate_tolerance_radps: float = 0.050
    settle_dwell_sec: float = 0.30
    approach_distance_m: float = 0.75
    brake_distance_m: float = 0.30
    terminal_timeout_sec: float = 15.0
    settle_timeout_sec: float = 5.0
    control_dt_max_sec: float = 0.10
    minimum_actuatable_speed_mps: float = 0.040

    def __post_init__(self) -> None:
        finite_fields = (
            "terminal_radial_tolerance_m",
            "capture_entry_tolerance_m",
            "settle_radial_tolerance_m",
            "stop_speed_tolerance_mps",
            "stop_yaw_rate_tolerance_radps",
            "settle_dwell_sec",
            "approach_distance_m",
            "brake_distance_m",
            "terminal_timeout_sec",
            "settle_timeout_sec",
            "control_dt_max_sec",
            "minimum_actuatable_speed_mps",
        )
        for name in finite_fields:
            if not _is_finite_real(getattr(self, name)):
                raise ValueError(f"{name} must be finite")

        positive_fields = (
            "terminal_radial_tolerance_m",
            "capture_entry_tolerance_m",
            "settle_radial_tolerance_m",
            "stop_speed_tolerance_mps",
            "stop_yaw_rate_tolerance_radps",
            "settle_dwell_sec",
            "approach_distance_m",
            "brake_distance_m",
            "terminal_timeout_sec",
            "settle_timeout_sec",
            "control_dt_max_sec",
        )
        for name in positive_fields:
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be greater than zero")
        if self.minimum_actuatable_speed_mps < 0.0:
            raise ValueError("minimum_actuatable_speed_mps must be non-negative")
        if self.capture_entry_tolerance_m > self.terminal_radial_tolerance_m:
            raise ValueError(
                "capture_entry_tolerance_m must not exceed "
                "terminal_radial_tolerance_m"
            )
        if self.settle_radial_tolerance_m > self.terminal_radial_tolerance_m:
            raise ValueError(
                "settle_radial_tolerance_m must not exceed "
                "terminal_radial_tolerance_m"
            )
        if self.brake_distance_m > self.approach_distance_m:
            raise ValueError("brake_distance_m must not exceed approach_distance_m")
        if self.settle_timeout_sec <= self.settle_dwell_sec:
            raise ValueError(
                "settle_timeout_sec must be greater than settle_dwell_sec"
            )


@dataclass(frozen=True, slots=True)
class TerminalInput:
    """Terminal intent and measured evidence for one controller cycle.

    Non-finite controller evidence is accepted as an invalid sample so the FSM
    can fail safe instead of throwing out of a control tick.  Monotonic time is
    the exception: it is part of the FSM's integrity contract and must be
    finite, non-negative, and non-decreasing.
    """

    monotonic_time_sec: float
    dt_sec: float
    terminal_requested: bool
    terminal_identity: Optional[str]
    distance_to_terminal_m: float
    radial_error_m: float
    cross_track_error_m: float
    along_track_error_m: float
    measured_linear_speed_mps: float
    measured_yaw_rate_radps: float
    telemetry_fresh: bool
    braking_required: bool = False
    heading_error_deg: Optional[float] = None
    current_pose: Optional[ControllerPose] = None


@dataclass(frozen=True, slots=True)
class PrecisionTerminalCertificate:
    """Immutable version-2 controller-frame stop evidence."""

    version: int
    terminal_identity: str
    precision_pass: bool
    radial_error_mm: float
    cross_error_mm: float
    along_error_mm: float
    heading_error_deg: Optional[float]
    measured_speed_mps: float
    measured_yaw_rate_radps: float
    stop_spec_mm: float
    settle_sec: float
    first_capture_pose: Optional[ControllerPose]
    final_settled_pose: Optional[ControllerPose]
    max_radial_during_settle_mm: float
    capture_timestamp_sec: float
    settle_started_timestamp_sec: float
    certified_timestamp_sec: float
    truth_frame: str = "controller_estimator_frame_only"
    localization_accuracy_certified: bool = False
    physical_accuracy_certified: bool = False

    def to_dict(self) -> dict[str, Any]:
        """Return deterministic JSON-safe data with no enums or non-finites."""

        result: dict[str, Any] = {
            "version": self.version,
            "terminal_identity": self.terminal_identity,
            "precision_pass": self.precision_pass,
            "radial_error_mm": float(self.radial_error_mm),
            "cross_error_mm": float(self.cross_error_mm),
            "along_error_mm": float(self.along_error_mm),
            "heading_error_deg": (
                None
                if self.heading_error_deg is None
                else float(self.heading_error_deg)
            ),
            "measured_speed_mps": float(self.measured_speed_mps),
            "measured_yaw_rate_radps": float(self.measured_yaw_rate_radps),
            "stop_spec_mm": float(self.stop_spec_mm),
            "settle_sec": float(self.settle_sec),
            "first_capture_pose": (
                None
                if self.first_capture_pose is None
                else self.first_capture_pose.to_dict()
            ),
            "final_settled_pose": (
                None
                if self.final_settled_pose is None
                else self.final_settled_pose.to_dict()
            ),
            "max_radial_during_settle_mm": float(
                self.max_radial_during_settle_mm
            ),
            "capture_timestamp_sec": float(self.capture_timestamp_sec),
            "settle_started_timestamp_sec": float(
                self.settle_started_timestamp_sec
            ),
            "certified_timestamp_sec": float(self.certified_timestamp_sec),
            "truth_frame": self.truth_frame,
            "localization_accuracy_certified": (
                self.localization_accuracy_certified
            ),
            "physical_accuracy_certified": self.physical_accuracy_certified,
        }
        if not _json_tree_is_finite(result):
            raise ValueError("certificate contains non-finite numeric data")
        return result


@dataclass(frozen=True, slots=True)
class TerminalResult:
    """Auditable state and directive after one terminal FSM tick."""

    state: TerminalState
    previous_state: TerminalState
    directive: TerminalDirective
    terminal_identity: Optional[str]
    zero_latched: bool
    transition_reason: str
    bounded_dt_sec: float
    state_elapsed_sec: float
    settle_held_sec: float
    motion_evidence_valid: bool
    currently_valid: bool
    certificate: Optional[PrecisionTerminalCertificate]


class TerminalStopStateMachine:
    """Measured terminal capture, zero-latch, settle, and certificate FSM."""

    def __init__(self, config: Optional[TerminalConfig] = None) -> None:
        self.config = config or TerminalConfig()
        self._clear(now_sec=None, reset_reason="constructed")

    @property
    def state(self) -> TerminalState:
        return self._state

    @property
    def terminal_identity(self) -> Optional[str]:
        return self._terminal_identity

    @property
    def zero_latched(self) -> bool:
        return self._zero_latched

    @property
    def last_reset_reason(self) -> str:
        return self._last_reset_reason

    def reset(
        self,
        *,
        monotonic_time_sec: float,
        semantic_boundary_reason: str,
    ) -> None:
        """Reset only at an explicitly named semantic boundary.

        Callers should use a reason such as ``path_replaced``, ``goal_changed``,
        or ``localization_jump`` and log it at the ROS integration boundary.
        """

        if not math.isfinite(monotonic_time_sec) or monotonic_time_sec < 0.0:
            raise ValueError("monotonic_time_sec must be finite and non-negative")
        if not isinstance(semantic_boundary_reason, str) or not (
            semantic_boundary_reason.strip()
        ):
            raise ValueError("semantic_boundary_reason must be a non-empty string")
        if (
            self._last_time_sec is not None
            and monotonic_time_sec < self._last_time_sec
        ):
            raise ValueError("monotonic_time_sec must not move backwards")
        self._clear(
            now_sec=monotonic_time_sec,
            reset_reason=semantic_boundary_reason.strip(),
        )

    def step(self, sample: TerminalInput) -> TerminalResult:
        """Advance one cycle, preserving zero ownership after capture."""

        self._validate_input_contract(sample)
        if (
            self._last_time_sec is not None
            and sample.monotonic_time_sec < self._last_time_sec
        ):
            raise ValueError("monotonic_time_sec must not move backwards")
        self._last_time_sec = sample.monotonic_time_sec
        bounded_dt = (
            min(max(sample.dt_sec, 0.0), self.config.control_dt_max_sec)
            if math.isfinite(sample.dt_sec)
            else 0.0
        )
        previous_state = self._state
        transition_reason = ""
        self._motion_evidence_valid = False

        if not sample.terminal_requested:
            if self._state is not TerminalState.TRACK or self._terminal_identity:
                self._clear(
                    now_sec=sample.monotonic_time_sec,
                    reset_reason="terminal_request_cancelled",
                )
                transition_reason = "terminal_request_cancelled"
            return self._result(
                previous_state,
                transition_reason,
                bounded_dt,
                sample.monotonic_time_sec,
            )

        assert sample.terminal_identity is not None
        identity_changed = (
            self._terminal_identity is not None
            and sample.terminal_identity != self._terminal_identity
        )
        if identity_changed:
            self._clear(
                now_sec=sample.monotonic_time_sec,
                reset_reason="terminal_identity_changed",
            )
            transition_reason = "terminal_identity_changed"

        if self._terminal_identity is None:
            self._terminal_identity = sample.terminal_identity
            self._request_started_sec = sample.monotonic_time_sec
            self._state_enter_time_sec = sample.monotonic_time_sec
            if not transition_reason:
                transition_reason = "terminal_request_started"
        self._motion_evidence_valid = self._motion_evidence_is_valid(sample)

        if self._state is TerminalState.HOLD_FAIL:
            return self._result(
                previous_state,
                transition_reason,
                bounded_dt,
                sample.monotonic_time_sec,
            )

        if self._state is TerminalState.CERTIFIED:
            if self._settle_evidence_valid(sample):
                self._currently_valid = True
            else:
                self._currently_valid = False
                transition_reason = self._fail(
                    sample.monotonic_time_sec,
                    self._post_certificate_failure_reason(sample),
                )
            return self._result(
                previous_state,
                transition_reason,
                bounded_dt,
                sample.monotonic_time_sec,
            )

        if self._terminal_watchdog_expired(sample.monotonic_time_sec):
            transition_reason = self._fail(
                sample.monotonic_time_sec, "terminal_watchdog_timeout"
            )
            return self._result(
                previous_state,
                transition_reason,
                bounded_dt,
                sample.monotonic_time_sec,
            )

        capture_ok = self._capture_evidence_valid(sample)
        if not self._zero_latched and capture_ok:
            self._enter_capture(sample)
            transition_reason = _join_reason(transition_reason, "capture_entered")
        elif not self._zero_latched:
            requested_state = self._requested_motion_state(sample)
            if (
                self._state is TerminalState.TRACK
                and requested_state is not TerminalState.TRACK
            ):
                transition_reason = _join_reason(
                    transition_reason,
                    self._transition(
                        requested_state,
                        sample.monotonic_time_sec,
                        "terminal_brake_required"
                        if requested_state is TerminalState.BRAKE
                        else "terminal_approach_required",
                    ),
                )
            elif (
                self._state is TerminalState.APPROACH
                and requested_state is TerminalState.BRAKE
            ):
                transition_reason = self._transition(
                    TerminalState.BRAKE,
                    sample.monotonic_time_sec,
                    "terminal_brake_required",
                )

        if self._zero_latched:
            self._observe_zero_latched_sample(sample)

            if self._settle_watchdog_expired(sample.monotonic_time_sec):
                transition_reason = self._fail(
                    sample.monotonic_time_sec, "terminal_settle_timeout"
                )
            elif self._state is TerminalState.CAPTURE:
                # CAPTURE is observable for the entry tick; the next tick owns
                # the explicit persistent latch state.
                if previous_state is TerminalState.CAPTURE:
                    transition_reason = self._transition(
                        TerminalState.ZERO_LATCH,
                        sample.monotonic_time_sec,
                        "zero_latch_confirmed",
                    )
            elif self._state is TerminalState.ZERO_LATCH:
                transition_reason = self._transition(
                    TerminalState.SETTLE,
                    sample.monotonic_time_sec,
                    "terminal_settle_started",
                )
            elif (
                self._state is TerminalState.SETTLE
                and not self._settle_violation_latched
                and self._settle_started_sec is not None
                and self._settle_held(sample.monotonic_time_sec)
                >= self.config.settle_dwell_sec
            ):
                self._certificate = self._build_certificate(sample)
                self._currently_valid = True
                transition_reason = self._transition(
                    TerminalState.CERTIFIED,
                    sample.monotonic_time_sec,
                    "terminal_stop_certified",
                )

        if (
            not self._zero_latched
            and self._state is not TerminalState.HOLD_FAIL
            and not self._motion_evidence_valid
        ):
            transition_reason = _join_reason(
                transition_reason, "terminal_motion_evidence_invalid"
            )

        return self._result(
            previous_state,
            transition_reason,
            bounded_dt,
            sample.monotonic_time_sec,
        )

    def _enter_capture(self, sample: TerminalInput) -> None:
        self._zero_latched = True
        self._capture_timestamp_sec = sample.monotonic_time_sec
        self._first_capture_pose = sample.current_pose
        self._max_radial_during_settle_m = sample.radial_error_m
        self._settle_started_sec = None
        self._settle_violation_latched = False
        self._transition(
            TerminalState.CAPTURE,
            sample.monotonic_time_sec,
            "capture_entered",
        )

    def _observe_zero_latched_sample(self, sample: TerminalInput) -> None:
        radial_finite = math.isfinite(sample.radial_error_m) and (
            sample.radial_error_m >= 0.0
        )
        if radial_finite:
            self._max_radial_during_settle_m = max(
                self._max_radial_during_settle_m,
                sample.radial_error_m,
            )

        if (
            sample.telemetry_fresh
            and radial_finite
            and sample.radial_error_m > self.config.settle_radial_tolerance_m
        ):
            # An observed exit from the 10 mm stop boundary invalidates this
            # same-identity attempt.  Zero remains owned until the watchdog.
            self._settle_violation_latched = True

        if self._settle_violation_latched or not self._settle_evidence_valid(sample):
            self._settle_started_sec = None
        elif self._settle_started_sec is None:
            # The first qualifying sample starts the dwell at zero elapsed; it
            # can never certify by itself.
            self._settle_started_sec = sample.monotonic_time_sec

    def _build_certificate(
        self, sample: TerminalInput
    ) -> PrecisionTerminalCertificate:
        assert self._terminal_identity is not None
        assert self._capture_timestamp_sec is not None
        assert self._settle_started_sec is not None
        settle_sec = self._settle_held(sample.monotonic_time_sec)
        return PrecisionTerminalCertificate(
            version=2,
            terminal_identity=self._terminal_identity,
            precision_pass=True,
            radial_error_mm=sample.radial_error_m * 1000.0,
            cross_error_mm=sample.cross_track_error_m * 1000.0,
            along_error_mm=sample.along_track_error_m * 1000.0,
            heading_error_deg=sample.heading_error_deg,
            measured_speed_mps=sample.measured_linear_speed_mps,
            measured_yaw_rate_radps=sample.measured_yaw_rate_radps,
            stop_spec_mm=self.config.terminal_radial_tolerance_m * 1000.0,
            settle_sec=settle_sec,
            first_capture_pose=self._first_capture_pose,
            final_settled_pose=sample.current_pose,
            max_radial_during_settle_mm=(
                self._max_radial_during_settle_m * 1000.0
            ),
            capture_timestamp_sec=self._capture_timestamp_sec,
            settle_started_timestamp_sec=self._settle_started_sec,
            certified_timestamp_sec=sample.monotonic_time_sec,
        )

    def _requested_motion_state(self, sample: TerminalInput) -> TerminalState:
        distance_valid = math.isfinite(sample.distance_to_terminal_m) and (
            sample.distance_to_terminal_m >= 0.0
        )
        if (
            distance_valid
            and sample.distance_to_terminal_m > self.config.approach_distance_m
            and not sample.braking_required
        ):
            return TerminalState.TRACK
        distance_requires_brake = distance_valid and (
            sample.distance_to_terminal_m <= self.config.brake_distance_m
        )
        return (
            TerminalState.BRAKE
            if sample.braking_required or distance_requires_brake
            else TerminalState.APPROACH
        )

    def _capture_evidence_valid(self, sample: TerminalInput) -> bool:
        return (
            self._motion_evidence_valid
            and math.isfinite(sample.radial_error_m)
            and sample.radial_error_m >= 0.0
            and sample.radial_error_m <= self.config.capture_entry_tolerance_m
        )

    @staticmethod
    def _motion_evidence_is_valid(sample: TerminalInput) -> bool:
        numeric_values = (
            sample.distance_to_terminal_m,
            sample.radial_error_m,
            sample.cross_track_error_m,
            sample.along_track_error_m,
            sample.measured_linear_speed_mps,
            sample.measured_yaw_rate_radps,
        )
        return (
            sample.telemetry_fresh
            and all(math.isfinite(value) for value in numeric_values)
            and sample.distance_to_terminal_m >= 0.0
            and sample.radial_error_m >= 0.0
            and (
                sample.heading_error_deg is None
                or math.isfinite(sample.heading_error_deg)
            )
        )

    def _settle_evidence_valid(self, sample: TerminalInput) -> bool:
        numeric_values = (
            sample.radial_error_m,
            sample.cross_track_error_m,
            sample.along_track_error_m,
            sample.measured_linear_speed_mps,
            sample.measured_yaw_rate_radps,
        )
        if not sample.telemetry_fresh or not all(
            math.isfinite(value) for value in numeric_values
        ):
            return False
        if sample.heading_error_deg is not None and not math.isfinite(
            sample.heading_error_deg
        ):
            return False
        return (
            sample.radial_error_m >= 0.0
            and sample.radial_error_m
            <= min(
                self.config.settle_radial_tolerance_m,
                self.config.terminal_radial_tolerance_m,
            )
            and abs(sample.measured_linear_speed_mps)
            <= self.config.stop_speed_tolerance_mps
            and abs(sample.measured_yaw_rate_radps)
            <= self.config.stop_yaw_rate_tolerance_radps
        )

    def _terminal_watchdog_expired(self, now_sec: float) -> bool:
        return (
            self._request_started_sec is not None
            and now_sec - self._request_started_sec
            >= self.config.terminal_timeout_sec
        )

    def _settle_watchdog_expired(self, now_sec: float) -> bool:
        return (
            self._capture_timestamp_sec is not None
            and now_sec - self._capture_timestamp_sec
            >= self.config.settle_timeout_sec
        )

    def _settle_held(self, now_sec: float) -> float:
        if self._settle_started_sec is None:
            return 0.0
        return max(0.0, now_sec - self._settle_started_sec)

    def _transition(
        self, state: TerminalState, now_sec: float, reason: str
    ) -> str:
        self._state = state
        self._state_enter_time_sec = now_sec
        return reason

    def _fail(self, now_sec: float, reason: str) -> str:
        return self._transition(TerminalState.HOLD_FAIL, now_sec, reason)

    def _result(
        self,
        previous_state: TerminalState,
        transition_reason: str,
        bounded_dt_sec: float,
        now_sec: float,
    ) -> TerminalResult:
        directive = self._directive_for_state(self._state)
        if (
            self._terminal_identity is not None
            and not self._zero_latched
            and self._state is not TerminalState.HOLD_FAIL
            and not self._motion_evidence_valid
        ):
            directive = TerminalDirective.HOLD_ZERO
        return TerminalResult(
            state=self._state,
            previous_state=previous_state,
            directive=directive,
            terminal_identity=self._terminal_identity,
            zero_latched=self._zero_latched,
            transition_reason=transition_reason,
            bounded_dt_sec=bounded_dt_sec,
            state_elapsed_sec=max(0.0, now_sec - self._state_enter_time_sec),
            settle_held_sec=self._settle_held(now_sec),
            motion_evidence_valid=self._motion_evidence_valid,
            currently_valid=self._currently_valid,
            certificate=self._certificate,
        )

    def _clear(self, *, now_sec: Optional[float], reset_reason: str) -> None:
        self._state = TerminalState.TRACK
        self._terminal_identity: Optional[str] = None
        self._zero_latched = False
        self._request_started_sec: Optional[float] = None
        self._capture_timestamp_sec: Optional[float] = None
        self._settle_started_sec: Optional[float] = None
        self._state_enter_time_sec = 0.0 if now_sec is None else now_sec
        self._last_time_sec = now_sec
        self._first_capture_pose: Optional[ControllerPose] = None
        self._max_radial_during_settle_m = 0.0
        self._settle_violation_latched = False
        self._certificate: Optional[PrecisionTerminalCertificate] = None
        self._motion_evidence_valid = False
        self._currently_valid = False
        self._last_reset_reason = reset_reason

    def _post_certificate_failure_reason(self, sample: TerminalInput) -> str:
        if not sample.telemetry_fresh:
            return "certified_telemetry_stale"
        if not math.isfinite(sample.radial_error_m):
            return "certified_position_nonfinite"
        if (
            sample.radial_error_m < 0.0
            or sample.radial_error_m > self.config.terminal_radial_tolerance_m
        ):
            return "certified_position_outside_tolerance"
        if not math.isfinite(sample.measured_linear_speed_mps):
            return "certified_speed_nonfinite"
        if (
            abs(sample.measured_linear_speed_mps)
            > self.config.stop_speed_tolerance_mps
        ):
            return "certified_speed_outside_tolerance"
        if not math.isfinite(sample.measured_yaw_rate_radps):
            return "certified_yaw_rate_nonfinite"
        if (
            abs(sample.measured_yaw_rate_radps)
            > self.config.stop_yaw_rate_tolerance_radps
        ):
            return "certified_yaw_rate_outside_tolerance"
        return "certified_evidence_invalid"

    @staticmethod
    def _directive_for_state(state: TerminalState) -> TerminalDirective:
        return {
            TerminalState.TRACK: TerminalDirective.TRACK,
            TerminalState.APPROACH: TerminalDirective.APPROACH,
            TerminalState.BRAKE: TerminalDirective.BRAKE,
            TerminalState.CAPTURE: TerminalDirective.HOLD_ZERO,
            TerminalState.ZERO_LATCH: TerminalDirective.HOLD_ZERO,
            TerminalState.SETTLE: TerminalDirective.HOLD_ZERO,
            TerminalState.CERTIFIED: TerminalDirective.HOLD_ZERO,
            TerminalState.HOLD_FAIL: TerminalDirective.HOLD_FAIL,
        }[state]

    @staticmethod
    def _validate_input_contract(sample: TerminalInput) -> None:
        if not _is_finite_real(sample.monotonic_time_sec):
            raise ValueError("monotonic_time_sec must be finite")
        if sample.monotonic_time_sec < 0.0:
            raise ValueError("monotonic_time_sec must be non-negative")
        bool_fields = (
            "terminal_requested",
            "telemetry_fresh",
            "braking_required",
        )
        for name in bool_fields:
            if not isinstance(getattr(sample, name), bool):
                raise ValueError(f"{name} must be bool")
        numeric_evidence_fields = (
            "dt_sec",
            "distance_to_terminal_m",
            "radial_error_m",
            "cross_track_error_m",
            "along_track_error_m",
            "measured_linear_speed_mps",
            "measured_yaw_rate_radps",
        )
        for name in numeric_evidence_fields:
            value = getattr(sample, name)
            if isinstance(value, bool) or not isinstance(value, Real):
                raise ValueError(f"{name} must be numeric")
        if sample.heading_error_deg is not None and (
            isinstance(sample.heading_error_deg, bool)
            or not isinstance(sample.heading_error_deg, Real)
        ):
            raise ValueError("heading_error_deg must be numeric or None")
        if sample.terminal_requested:
            if not isinstance(sample.terminal_identity, str) or not (
                sample.terminal_identity.strip()
            ):
                raise ValueError(
                    "terminal_identity must be a non-empty string when requested"
                )
        elif sample.terminal_identity is not None and not isinstance(
            sample.terminal_identity, str
        ):
            raise ValueError("terminal_identity must be a string or None")
        if sample.current_pose is not None and not isinstance(
            sample.current_pose, ControllerPose
        ):
            raise ValueError("current_pose must be ControllerPose or None")


def _join_reason(first: str, second: str) -> str:
    if not first:
        return second
    if not second:
        return first
    return f"{first};{second}"


def _is_finite_real(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, Real)
        and math.isfinite(value)
    )


def _json_tree_is_finite(value: Any) -> bool:
    if isinstance(value, bool) or value is None or isinstance(value, (str, int)):
        return True
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(
            isinstance(key, str) and _json_tree_is_finite(item)
            for key, item in value.items()
        )
    if isinstance(value, (list, tuple)):
        return all(_json_tree_is_finite(item) for item in value)
    return False
