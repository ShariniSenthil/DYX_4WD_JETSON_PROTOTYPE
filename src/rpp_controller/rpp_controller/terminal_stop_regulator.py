"""ROS-independent measured-speed regulator for a radial 20 mm stop.

The regulator owns only terminal forward-speed magnitude.  It never requests
reverse motion, a pivot, or a change of tracking line.  The ROS adapter keeps
authority over the bearing of the line currently being followed.

All speed evidence supplied to this module is required to be derived from
position.  Stopped certification deliberately ignores estimator velocity and
uses a bounded position window plus measured yaw rate instead.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real
from typing import Deque, Optional


__all__ = [
    "MotionDirection",
    "PositionWindowEvidence",
    "PositionWindowStationaryDetector",
    "RadialStopCertificate",
    "RadialStopConfig",
    "RadialStopFailure",
    "RadialStopInput",
    "RadialStopOutput",
    "RadialStopState",
    "TerminalStopRegulator",
]


class RadialStopState(str, Enum):
    """Phases of the one-shot terminal stop."""

    TRACK = "track"
    TERMINAL_GUIDANCE = "terminal_guidance"
    BRAKE_PROFILE = "brake_profile"
    ZERO_LATCH = "zero_latch"
    SETTLE = "settle"
    CERTIFIED = "certified"
    HOLD_ZERO = "hold_zero"
    HOLD_FAIL = "hold_fail"


class MotionDirection(str, Enum):
    """Direction contract exposed to the navigation adapter."""

    FORWARD = "forward"
    ZERO = "zero"


class RadialStopFailure(str, Enum):
    """Fail-closed reasons which never produce a passing certificate."""

    NONE = "none"
    INVALID_MEASUREMENT = "invalid_measurement"
    STALE_TELEMETRY = "stale_telemetry"
    TERMINAL_REQUEST_LOST = "terminal_request_lost"
    IDENTITY_CHANGED = "identity_changed"
    TERMINAL_TIMEOUT = "terminal_timeout"
    SETTLE_TIMEOUT = "settle_timeout"
    SETTLED_OUTSIDE_TOLERANCE = "settled_outside_tolerance"


@dataclass(frozen=True, slots=True)
class RadialStopConfig:
    """Validated thresholds for a one-shot radial stop.

    Deceleration and the brake margin are calibration inputs.  They must be
    replaced by values measured during the post-speed-PI bench sweep before
    enabling this controller in the field.
    """

    radial_tolerance_m: float = 0.020
    terminal_guidance_distance_m: float = 0.75
    conservative_decel_mps2: float = 0.30
    brake_margin_m: float = 0.010
    stationary_window_sec: float = 0.50
    stationary_displacement_m: float = 0.005
    stationary_yaw_rate_radps: float = 0.050
    maximum_position_sample_gap_sec: float = 0.20
    terminal_timeout_sec: float = 15.0
    settle_timeout_sec: float = 5.0

    def __post_init__(self) -> None:
        fields = (
            "radial_tolerance_m",
            "terminal_guidance_distance_m",
            "conservative_decel_mps2",
            "brake_margin_m",
            "stationary_window_sec",
            "stationary_displacement_m",
            "stationary_yaw_rate_radps",
            "maximum_position_sample_gap_sec",
            "terminal_timeout_sec",
            "settle_timeout_sec",
        )
        for name in fields:
            if not _finite_real(getattr(self, name)):
                raise ValueError(f"{name} must be finite")

        positive = (
            "radial_tolerance_m",
            "terminal_guidance_distance_m",
            "conservative_decel_mps2",
            "stationary_window_sec",
            "stationary_displacement_m",
            "stationary_yaw_rate_radps",
            "maximum_position_sample_gap_sec",
            "terminal_timeout_sec",
            "settle_timeout_sec",
        )
        for name in positive:
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be greater than zero")
        if self.brake_margin_m < 0.0:
            raise ValueError("brake_margin_m must be non-negative")
        if self.brake_margin_m > self.radial_tolerance_m:
            raise ValueError("brake_margin_m must not exceed radial_tolerance_m")
        if self.terminal_guidance_distance_m <= self.radial_tolerance_m:
            raise ValueError(
                "terminal_guidance_distance_m must exceed radial_tolerance_m"
            )
        if self.maximum_position_sample_gap_sec >= self.stationary_window_sec:
            raise ValueError(
                "maximum_position_sample_gap_sec must be less than "
                "stationary_window_sec"
            )
        if self.settle_timeout_sec <= self.stationary_window_sec:
            raise ValueError(
                "settle_timeout_sec must exceed stationary_window_sec"
            )
        if self.terminal_timeout_sec <= self.settle_timeout_sec:
            raise ValueError(
                "terminal_timeout_sec must exceed settle_timeout_sec"
            )


@dataclass(frozen=True, slots=True)
class RadialStopInput:
    """Measured terminal evidence for one deterministic control cycle."""

    monotonic_time_sec: float
    position_sample_time_sec: float
    active: bool
    terminal_identity: Optional[str]
    along_remaining_m: float
    cross_error_m: float
    position_x_m: float
    position_y_m: float
    position_derived_speed_mps: float
    measured_yaw_rate_radps: float
    tracking_speed_command_mps: float
    telemetry_fresh: bool


@dataclass(frozen=True, slots=True)
class PositionWindowEvidence:
    """Result of one stationary-detector update."""

    stationary: bool
    held_sec: float
    maximum_displacement_m: float
    maximum_abs_yaw_rate_radps: float
    sample_count: int


@dataclass(frozen=True, slots=True)
class RadialStopCertificate:
    """Identity-bound estimator-frame evidence for a valid radial stop."""

    version: int
    terminal_identity: str
    certified_timestamp_sec: float
    radial_error_m: float
    along_error_m: float
    cross_error_m: float
    final_position_x_m: float
    final_position_y_m: float
    stationary_window_sec: float
    maximum_stationary_displacement_m: float
    maximum_abs_yaw_rate_radps: float
    speed_source: str = "position_derived"
    truth_frame: str = "px4_estimator_frame_only"


@dataclass(frozen=True, slots=True)
class RadialStopOutput:
    """Forward-only terminal command and certification evidence."""

    previous_state: RadialStopState
    state: RadialStopState
    motion_direction: MotionDirection
    forward_speed_command_mps: float
    hold_zero: bool
    effective_braking_speed_mps: float
    stop_distance_m: float
    profile_speed_mps: float
    radial_error_m: float
    stationary: bool
    stationary_window_sec: float
    failure: RadialStopFailure
    certificate: Optional[RadialStopCertificate]


@dataclass(frozen=True, slots=True)
class _PositionSample:
    timestamp_sec: float
    x_m: float
    y_m: float
    abs_yaw_rate_radps: float


class PositionWindowStationaryDetector:
    """Detect stationarity from a contiguous bounded position window."""

    def __init__(
        self,
        *,
        window_sec: float,
        displacement_limit_m: float,
        yaw_rate_limit_radps: float,
        maximum_sample_gap_sec: float,
    ) -> None:
        for name, value in (
            ("window_sec", window_sec),
            ("displacement_limit_m", displacement_limit_m),
            ("yaw_rate_limit_radps", yaw_rate_limit_radps),
            ("maximum_sample_gap_sec", maximum_sample_gap_sec),
        ):
            if not _finite_real(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero")
        if maximum_sample_gap_sec >= window_sec:
            raise ValueError("maximum_sample_gap_sec must be less than window_sec")

        self._window_sec = float(window_sec)
        self._displacement_limit_m = float(displacement_limit_m)
        self._yaw_rate_limit_radps = float(yaw_rate_limit_radps)
        self._maximum_sample_gap_sec = float(maximum_sample_gap_sec)
        self._samples: Deque[_PositionSample] = deque()
        self._last_timestamp_sec: Optional[float] = None

    def reset(self) -> None:
        """Discard all accumulated stationary evidence."""

        self._samples.clear()
        self._last_timestamp_sec = None

    def update(
        self,
        *,
        position_sample_time_sec: float,
        x_m: float,
        y_m: float,
        yaw_rate_radps: float,
    ) -> PositionWindowEvidence:
        """Add one distinct odometry sample and ignore retained duplicates."""

        values = (position_sample_time_sec, x_m, y_m, yaw_rate_radps)
        if not all(_finite_real(value) for value in values):
            self.reset()
            return self._evidence()
        now = float(position_sample_time_sec)
        if now < 0.0:
            self.reset()
            return self._evidence()
        if self._last_timestamp_sec is not None:
            gap = now - self._last_timestamp_sec
            if gap < 0.0:
                raise ValueError(
                    "position_sample_time_sec must not move backwards"
                )
            if gap == 0.0:
                return self._evidence()
            if gap > self._maximum_sample_gap_sec:
                self.reset()
        self._last_timestamp_sec = now

        sample = _PositionSample(
            timestamp_sec=now,
            x_m=float(x_m),
            y_m=float(y_m),
            abs_yaw_rate_radps=abs(float(yaw_rate_radps)),
        )
        if sample.abs_yaw_rate_radps > self._yaw_rate_limit_radps:
            self._samples.clear()
            return self._evidence()

        if any(
            math.hypot(sample.x_m - prior.x_m, sample.y_m - prior.y_m)
            > self._displacement_limit_m
            for prior in self._samples
        ):
            self._samples.clear()
        self._samples.append(sample)
        self._prune_old_samples(now)
        return self._evidence()

    def _prune_old_samples(self, now_sec: float) -> None:
        """Bound memory while retaining evidence spanning one full window."""

        cutoff = now_sec - self._window_sec
        while (
            len(self._samples) >= 2
            and self._samples[1].timestamp_sec <= cutoff
        ):
            self._samples.popleft()

    def _evidence(self) -> PositionWindowEvidence:
        if not self._samples:
            return PositionWindowEvidence(False, 0.0, 0.0, 0.0, 0)
        first = self._samples[0]
        last = self._samples[-1]
        held_sec = max(0.0, last.timestamp_sec - first.timestamp_sec)
        maximum_displacement_m = 0.0
        samples = tuple(self._samples)
        for index, lhs in enumerate(samples):
            for rhs in samples[index + 1:]:
                maximum_displacement_m = max(
                    maximum_displacement_m,
                    math.hypot(lhs.x_m - rhs.x_m, lhs.y_m - rhs.y_m),
                )
        maximum_yaw_rate = max(
            sample.abs_yaw_rate_radps for sample in samples
        )
        stationary = (
            held_sec >= self._window_sec
            and maximum_displacement_m <= self._displacement_limit_m
            and maximum_yaw_rate <= self._yaw_rate_limit_radps
        )
        return PositionWindowEvidence(
            stationary=stationary,
            held_sec=held_sec,
            maximum_displacement_m=maximum_displacement_m,
            maximum_abs_yaw_rate_radps=maximum_yaw_rate,
            sample_count=len(samples),
        )


class TerminalStopRegulator:
    """One-shot measured-speed terminal regulator with fail-closed latches."""

    CERTIFICATE_VERSION = 2

    def __init__(self, config: Optional[RadialStopConfig] = None) -> None:
        self.config = config or RadialStopConfig()
        self._detector = PositionWindowStationaryDetector(
            window_sec=self.config.stationary_window_sec,
            displacement_limit_m=self.config.stationary_displacement_m,
            yaw_rate_limit_radps=self.config.stationary_yaw_rate_radps,
            maximum_sample_gap_sec=(
                self.config.maximum_position_sample_gap_sec
            ),
        )
        self.reset()

    @property
    def state(self) -> RadialStopState:
        """Return the current state without advancing the controller."""

        return self._state

    def reset(self) -> None:
        """Explicitly clear terminal identity, certificates, and latches."""

        self._state = RadialStopState.TRACK
        self._terminal_identity: Optional[str] = None
        self._terminal_started_sec: Optional[float] = None
        self._zero_latched_sec: Optional[float] = None
        self._last_timestamp_sec: Optional[float] = None
        self._failure = RadialStopFailure.NONE
        self._certificate: Optional[RadialStopCertificate] = None
        self._detector.reset()

    def cancel(self) -> None:
        """Explicitly cancel the terminal operation and return to tracking."""

        self.reset()

    def stopping_distance_m(self, position_derived_speed_mps: float) -> float:
        """Return conservative stopping lead from measured position speed."""

        if not _finite_real(position_derived_speed_mps):
            raise ValueError("position_derived_speed_mps must be finite")
        speed = max(0.0, float(position_derived_speed_mps))
        return (
            speed * speed / (2.0 * self.config.conservative_decel_mps2)
            + self.config.brake_margin_m
        )

    def effective_braking_speed_mps(self, sample: RadialStopInput) -> float:
        """Return the conservative speed used to compute stopping lead."""

        speeds = (
            sample.position_derived_speed_mps,
            sample.tracking_speed_command_mps,
        )
        if not all(_finite_real(speed) and speed >= 0.0 for speed in speeds):
            raise ValueError("braking speed inputs must be finite and non-negative")
        return max(float(speed) for speed in speeds)

    def step(self, sample: RadialStopInput) -> RadialStopOutput:
        """Advance one control cycle using deterministic monotonic time."""

        previous = self._state
        self._validate_time(sample.monotonic_time_sec)

        if self._state in (
            RadialStopState.CERTIFIED,
            RadialStopState.HOLD_ZERO,
        ):
            if self._state is RadialStopState.CERTIFIED:
                self._state = RadialStopState.HOLD_ZERO
            return self._zero_output(previous, sample)
        if self._state is RadialStopState.HOLD_FAIL:
            return self._zero_output(previous, sample)

        if not sample.active and self._state is RadialStopState.TRACK:
            self._detector.reset()
            return self._tracking_output(previous, sample)
        if not sample.active:
            return self._fail(
                previous,
                sample,
                RadialStopFailure.TERMINAL_REQUEST_LOST,
            )

        invalid = not self._valid_measurement(sample)
        if invalid:
            return self._fail(
                previous,
                sample,
                RadialStopFailure.INVALID_MEASUREMENT,
            )
        if not sample.telemetry_fresh:
            return self._fail(
                previous,
                sample,
                RadialStopFailure.STALE_TELEMETRY,
            )

        if self._terminal_identity is None:
            self._terminal_identity = sample.terminal_identity
            self._terminal_started_sec = float(sample.monotonic_time_sec)
        elif sample.terminal_identity != self._terminal_identity:
            return self._fail(
                previous,
                sample,
                RadialStopFailure.IDENTITY_CHANGED,
            )

        if self._terminal_timed_out(sample.monotonic_time_sec):
            return self._fail(
                previous,
                sample,
                RadialStopFailure.TERMINAL_TIMEOUT,
            )

        radial = math.hypot(sample.along_remaining_m, sample.cross_error_m)
        effective_braking_speed = self.effective_braking_speed_mps(sample)
        stop_distance = self.stopping_distance_m(effective_braking_speed)

        if self._state in (
            RadialStopState.ZERO_LATCH,
            RadialStopState.SETTLE,
        ):
            return self._advance_zero_latch(
                previous,
                sample,
                radial,
                stop_distance,
                effective_braking_speed,
            )

        # Entry into the radial circle is a one-way zero latch.  Crossing the
        # goal plane also latches zero regardless of cross-track error.
        if radial <= self.config.radial_tolerance_m or (
            sample.along_remaining_m <= 0.0
        ):
            self._enter_zero_latch(sample)
            return self._zero_output(
                previous,
                sample,
                radial_error_m=radial,
                stop_distance_m=stop_distance,
                effective_braking_speed_mps=effective_braking_speed,
            )

        if self._state is RadialStopState.TRACK:
            if (
                sample.along_remaining_m
                <= self.config.terminal_guidance_distance_m
            ):
                self._state = RadialStopState.TERMINAL_GUIDANCE
            return self._tracking_output(
                previous,
                sample,
                radial_error_m=radial,
                stop_distance_m=stop_distance,
                effective_braking_speed_mps=effective_braking_speed,
            )

        if self._state is RadialStopState.TERMINAL_GUIDANCE:
            if sample.along_remaining_m <= stop_distance:
                self._state = RadialStopState.BRAKE_PROFILE
                return self._braking_output(
                    previous,
                    sample,
                    radial,
                    stop_distance,
                    effective_braking_speed,
                )
            return self._tracking_output(
                previous,
                sample,
                radial_error_m=radial,
                stop_distance_m=stop_distance,
                effective_braking_speed_mps=effective_braking_speed,
            )

        if self._state is RadialStopState.BRAKE_PROFILE:
            return self._braking_output(
                previous,
                sample,
                radial,
                stop_distance,
                effective_braking_speed,
            )

        raise RuntimeError(f"unhandled radial stop state {self._state}")

    def _advance_zero_latch(
        self,
        previous: RadialStopState,
        sample: RadialStopInput,
        radial_error_m: float,
        stop_distance_m: float,
        effective_braking_speed_mps: float,
    ) -> RadialStopOutput:
        if self._zero_latched_sec is None:
            raise RuntimeError("zero latch timestamp is missing")
        if (
            sample.monotonic_time_sec - self._zero_latched_sec
            > self.config.settle_timeout_sec
        ):
            return self._fail(
                previous,
                sample,
                RadialStopFailure.SETTLE_TIMEOUT,
            )

        try:
            evidence = self._detector.update(
                position_sample_time_sec=sample.position_sample_time_sec,
                x_m=sample.position_x_m,
                y_m=sample.position_y_m,
                yaw_rate_radps=sample.measured_yaw_rate_radps,
            )
        except ValueError:
            return self._fail(
                previous,
                sample,
                RadialStopFailure.INVALID_MEASUREMENT,
            )
        if not evidence.stationary:
            self._state = RadialStopState.ZERO_LATCH
            return self._zero_output(
                previous,
                sample,
                radial_error_m=radial_error_m,
                stop_distance_m=stop_distance_m,
                stationary=evidence,
                effective_braking_speed_mps=effective_braking_speed_mps,
            )

        if self._state is RadialStopState.ZERO_LATCH:
            self._state = RadialStopState.SETTLE
            return self._zero_output(
                previous,
                sample,
                radial_error_m=radial_error_m,
                stop_distance_m=stop_distance_m,
                stationary=evidence,
                effective_braking_speed_mps=effective_braking_speed_mps,
            )

        if radial_error_m > self.config.radial_tolerance_m:
            return self._fail(
                previous,
                sample,
                RadialStopFailure.SETTLED_OUTSIDE_TOLERANCE,
            )

        terminal_identity = self._terminal_identity
        if terminal_identity is None:
            raise RuntimeError("terminal identity is missing")
        self._certificate = RadialStopCertificate(
            version=self.CERTIFICATE_VERSION,
            terminal_identity=terminal_identity,
            certified_timestamp_sec=float(sample.monotonic_time_sec),
            radial_error_m=radial_error_m,
            along_error_m=float(sample.along_remaining_m),
            cross_error_m=float(sample.cross_error_m),
            final_position_x_m=float(sample.position_x_m),
            final_position_y_m=float(sample.position_y_m),
            stationary_window_sec=evidence.held_sec,
            maximum_stationary_displacement_m=(
                evidence.maximum_displacement_m
            ),
            maximum_abs_yaw_rate_radps=(
                evidence.maximum_abs_yaw_rate_radps
            ),
        )
        self._state = RadialStopState.CERTIFIED
        return self._zero_output(
            previous,
            sample,
            radial_error_m=radial_error_m,
            stop_distance_m=stop_distance_m,
            stationary=evidence,
            effective_braking_speed_mps=effective_braking_speed_mps,
        )

    def _enter_zero_latch(self, sample: RadialStopInput) -> None:
        self._state = RadialStopState.ZERO_LATCH
        self._zero_latched_sec = float(sample.monotonic_time_sec)
        self._detector.reset()
        self._detector.update(
            position_sample_time_sec=sample.position_sample_time_sec,
            x_m=sample.position_x_m,
            y_m=sample.position_y_m,
            yaw_rate_radps=sample.measured_yaw_rate_radps,
        )

    def _braking_output(
        self,
        previous: RadialStopState,
        sample: RadialStopInput,
        radial_error_m: float,
        stop_distance_m: float,
        effective_braking_speed_mps: float,
    ) -> RadialStopOutput:
        effective_remaining = max(
            0.0,
            sample.along_remaining_m - self.config.brake_margin_m,
        )
        profile_speed = math.sqrt(
            2.0 * self.config.conservative_decel_mps2 * effective_remaining
        )
        command = min(sample.tracking_speed_command_mps, profile_speed)
        if command <= 0.0:
            self._enter_zero_latch(sample)
            return self._zero_output(
                previous,
                sample,
                radial_error_m=radial_error_m,
                stop_distance_m=stop_distance_m,
                effective_braking_speed_mps=effective_braking_speed_mps,
            )
        return RadialStopOutput(
            previous_state=previous,
            state=self._state,
            motion_direction=MotionDirection.FORWARD,
            forward_speed_command_mps=command,
            hold_zero=False,
            effective_braking_speed_mps=effective_braking_speed_mps,
            stop_distance_m=stop_distance_m,
            profile_speed_mps=profile_speed,
            radial_error_m=radial_error_m,
            stationary=False,
            stationary_window_sec=0.0,
            failure=RadialStopFailure.NONE,
            certificate=None,
        )

    def _tracking_output(
        self,
        previous: RadialStopState,
        sample: RadialStopInput,
        *,
        radial_error_m: Optional[float] = None,
        stop_distance_m: Optional[float] = None,
        effective_braking_speed_mps: Optional[float] = None,
    ) -> RadialStopOutput:
        command = max(0.0, float(sample.tracking_speed_command_mps))
        return RadialStopOutput(
            previous_state=previous,
            state=self._state,
            motion_direction=(
                MotionDirection.FORWARD if command > 0.0
                else MotionDirection.ZERO
            ),
            forward_speed_command_mps=command,
            hold_zero=command == 0.0,
            effective_braking_speed_mps=(
                self._safe_effective_braking_speed(sample)
                if effective_braking_speed_mps is None
                else effective_braking_speed_mps
            ),
            stop_distance_m=(
                self._safe_stop_distance(sample)
                if stop_distance_m is None
                else stop_distance_m
            ),
            profile_speed_mps=command,
            radial_error_m=(
                self._safe_radial(sample)
                if radial_error_m is None
                else radial_error_m
            ),
            stationary=False,
            stationary_window_sec=0.0,
            failure=RadialStopFailure.NONE,
            certificate=None,
        )

    def _zero_output(
        self,
        previous: RadialStopState,
        sample: RadialStopInput,
        *,
        radial_error_m: Optional[float] = None,
        stop_distance_m: Optional[float] = None,
        stationary: Optional[PositionWindowEvidence] = None,
        effective_braking_speed_mps: Optional[float] = None,
    ) -> RadialStopOutput:
        evidence = stationary or PositionWindowEvidence(
            False, 0.0, 0.0, 0.0, 0
        )
        return RadialStopOutput(
            previous_state=previous,
            state=self._state,
            motion_direction=MotionDirection.ZERO,
            forward_speed_command_mps=0.0,
            hold_zero=True,
            effective_braking_speed_mps=(
                self._safe_effective_braking_speed(sample)
                if effective_braking_speed_mps is None
                else effective_braking_speed_mps
            ),
            stop_distance_m=(
                self._safe_stop_distance(sample)
                if stop_distance_m is None
                else stop_distance_m
            ),
            profile_speed_mps=0.0,
            radial_error_m=(
                self._safe_radial(sample)
                if radial_error_m is None
                else radial_error_m
            ),
            stationary=evidence.stationary,
            stationary_window_sec=evidence.held_sec,
            failure=self._failure,
            certificate=self._certificate,
        )

    def _fail(
        self,
        previous: RadialStopState,
        sample: RadialStopInput,
        failure: RadialStopFailure,
    ) -> RadialStopOutput:
        self._state = RadialStopState.HOLD_FAIL
        self._failure = failure
        self._certificate = None
        return self._zero_output(previous, sample)

    def _validate_time(self, monotonic_time_sec: float) -> None:
        if not _finite_real(monotonic_time_sec) or monotonic_time_sec < 0.0:
            raise ValueError("monotonic_time_sec must be finite and non-negative")
        now = float(monotonic_time_sec)
        if self._last_timestamp_sec is not None and now < self._last_timestamp_sec:
            raise ValueError("monotonic_time_sec must not move backwards")
        self._last_timestamp_sec = now

    def _valid_measurement(self, sample: RadialStopInput) -> bool:
        values = (
            sample.along_remaining_m,
            sample.cross_error_m,
            sample.position_sample_time_sec,
            sample.position_x_m,
            sample.position_y_m,
            sample.position_derived_speed_mps,
            sample.measured_yaw_rate_radps,
            sample.tracking_speed_command_mps,
        )
        return (
            isinstance(sample.terminal_identity, str)
            and bool(sample.terminal_identity.strip())
            and all(_finite_real(value) for value in values)
            and sample.position_derived_speed_mps >= 0.0
            and sample.tracking_speed_command_mps >= 0.0
        )

    def _terminal_timed_out(self, now_sec: float) -> bool:
        return (
            self._terminal_started_sec is not None
            and now_sec - self._terminal_started_sec
            > self.config.terminal_timeout_sec
        )

    def _safe_radial(self, sample: RadialStopInput) -> float:
        if not (
            _finite_real(sample.along_remaining_m)
            and _finite_real(sample.cross_error_m)
        ):
            return math.inf
        return math.hypot(sample.along_remaining_m, sample.cross_error_m)

    def _safe_stop_distance(self, sample: RadialStopInput) -> float:
        speed = self._safe_effective_braking_speed(sample)
        if not math.isfinite(speed):
            return math.inf
        return self.stopping_distance_m(speed)

    def _safe_effective_braking_speed(self, sample: RadialStopInput) -> float:
        try:
            return self.effective_braking_speed_mps(sample)
        except ValueError:
            return math.inf


def _finite_real(value: object) -> bool:
    """Return true for finite real scalars while rejecting booleans."""

    return (
        isinstance(value, Real)
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )
