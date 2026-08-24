"""ROS-free longitudinal speed regulation for precision path tracking.

The regulator keeps longitudinal authority separate from guidance.  It accepts
geometric progress and preview measurements, computes each independent speed
cap, and returns the minimum cap together with a deterministic owner.  No
coordinate-frame or PX4 assumptions live in this module.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional


__all__ = [
    "LongitudinalRegulator",
    "LongitudinalRegulatorConfig",
    "SpeedCapOwner",
    "SpeedCaps",
    "SpeedRegulatorInput",
    "SpeedRegulatorResult",
    "allowable_speed_for_distance",
    "braking_distance",
]


class SpeedCapOwner(str, Enum):
    """The cap that owns the final requested speed."""

    HARD_ZERO = "hard_zero"
    MISSION = "mission"
    HARDWARE = "hardware"
    RECOVERY = "recovery"
    ACCELERATION = "acceleration"
    HEADING = "heading_alignment"
    CROSS_TRACK = "cross_track_recovery"
    TRACKING = "tracking_stability"
    CORNER = "corner_preview"
    TERMINAL = "terminal_braking"
    CURVATURE = "curvature"


@dataclass(frozen=True, slots=True)
class LongitudinalRegulatorConfig:
    """Validated parameters for the longitudinal resolver."""

    hardware_speed_ceiling_mps: float = 1.0
    acceleration_mps2: float = 0.75
    deceleration_mps2: float = 0.75
    launch_speed_mps: float = 0.10
    control_dt_max_sec: float = 0.10

    heading_accel_full_error_rad: float = math.radians(2.0)
    heading_recovery_start_rad: float = math.radians(4.0)
    heading_recovery_full_rad: float = math.radians(15.0)
    cross_track_accel_full_m: float = 0.010
    cross_track_recovery_start_m: float = 0.020
    cross_track_recovery_full_m: float = 0.100
    recovery_min_speed_mps: float = 0.15

    corner_angle_threshold_rad: float = math.radians(45.0)
    corner_target_speed_mps: float = 0.12
    corner_accel_block_buffer_m: float = 0.10
    terminal_target_speed_mps: float = 0.0
    braking_latency_sec: float = 0.10
    braking_margin_m: float = 0.05

    curvature_enabled: bool = False
    lateral_acceleration_max_mps2: float = 0.30
    curvature_epsilon_inv_m: float = 1.0e-6

    def __post_init__(self) -> None:
        finite_fields = (
            "hardware_speed_ceiling_mps",
            "acceleration_mps2",
            "deceleration_mps2",
            "launch_speed_mps",
            "control_dt_max_sec",
            "heading_accel_full_error_rad",
            "heading_recovery_start_rad",
            "heading_recovery_full_rad",
            "cross_track_accel_full_m",
            "cross_track_recovery_start_m",
            "cross_track_recovery_full_m",
            "recovery_min_speed_mps",
            "corner_angle_threshold_rad",
            "corner_target_speed_mps",
            "corner_accel_block_buffer_m",
            "terminal_target_speed_mps",
            "braking_latency_sec",
            "braking_margin_m",
            "lateral_acceleration_max_mps2",
            "curvature_epsilon_inv_m",
        )
        for name in finite_fields:
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite")

        positive_fields = (
            "hardware_speed_ceiling_mps",
            "acceleration_mps2",
            "deceleration_mps2",
            "control_dt_max_sec",
            "heading_recovery_full_rad",
            "cross_track_recovery_full_m",
            "lateral_acceleration_max_mps2",
            "curvature_epsilon_inv_m",
        )
        for name in positive_fields:
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be greater than zero")

        nonnegative_fields = (
            "launch_speed_mps",
            "heading_accel_full_error_rad",
            "heading_recovery_start_rad",
            "cross_track_accel_full_m",
            "cross_track_recovery_start_m",
            "recovery_min_speed_mps",
            "corner_angle_threshold_rad",
            "corner_target_speed_mps",
            "corner_accel_block_buffer_m",
            "terminal_target_speed_mps",
            "braking_latency_sec",
            "braking_margin_m",
        )
        for name in nonnegative_fields:
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} must be non-negative")

        if not (
            self.heading_accel_full_error_rad
            <= self.heading_recovery_start_rad
            < self.heading_recovery_full_rad
            <= math.pi
        ):
            raise ValueError(
                "heading thresholds must satisfy accel_full <= recovery_start "
                "< recovery_full <= pi"
            )
        if not (
            self.cross_track_accel_full_m
            <= self.cross_track_recovery_start_m
            < self.cross_track_recovery_full_m
        ):
            raise ValueError(
                "cross-track thresholds must satisfy accel_full <= "
                "recovery_start < recovery_full"
            )
        if self.corner_angle_threshold_rad > math.pi:
            raise ValueError("corner_angle_threshold_rad must not exceed pi")

        speed_fields = (
            "launch_speed_mps",
            "recovery_min_speed_mps",
            "corner_target_speed_mps",
            "terminal_target_speed_mps",
        )
        for name in speed_fields:
            if getattr(self, name) > self.hardware_speed_ceiling_mps:
                raise ValueError(
                    f"{name} must not exceed hardware_speed_ceiling_mps"
                )


@dataclass(frozen=True, slots=True)
class SpeedRegulatorInput:
    """Measurements and geometric preview for one control cycle."""

    mission_speed_ceiling_mps: float
    measured_speed_mps: float
    last_commanded_speed_mps: float
    dt_sec: float
    along_track_progress_m: float
    heading_error_rad: float
    cross_track_error_m: float
    distance_to_corner_m: Optional[float] = None
    corner_angle_rad: Optional[float] = None
    distance_to_terminal_m: Optional[float] = None
    # Per-request override used by the separately gated precision terminal
    # adapter.  None preserves the configured Phase-2 target exactly.
    terminal_target_speed_override_mps: Optional[float] = None
    curvature_inv_m: Optional[float] = None
    tracking_acceleration_allowed: bool = True
    tracking_speed_cap_mps: Optional[float] = None
    recovery_requested: bool = False
    hard_zero: bool = False


@dataclass(frozen=True, slots=True)
class SpeedCaps:
    """All speed caps computed in one resolver cycle."""

    mission_mps: float
    hardware_mps: float
    recovery_mps: Optional[float]
    acceleration_mps: float
    heading_mps: float
    cross_track_mps: float
    tracking_mps: Optional[float]
    corner_mps: Optional[float]
    terminal_mps: Optional[float]
    curvature_mps: Optional[float]

    def ordered_items(self) -> tuple[tuple[SpeedCapOwner, float], ...]:
        """Return active caps in the stable winner-precedence order."""

        items = [
            (SpeedCapOwner.MISSION, self.mission_mps),
            (SpeedCapOwner.HARDWARE, self.hardware_mps),
            (SpeedCapOwner.ACCELERATION, self.acceleration_mps),
            (SpeedCapOwner.HEADING, self.heading_mps),
            (SpeedCapOwner.CROSS_TRACK, self.cross_track_mps),
        ]
        optional = (
            (SpeedCapOwner.RECOVERY, self.recovery_mps),
            (SpeedCapOwner.TRACKING, self.tracking_mps),
            (SpeedCapOwner.CORNER, self.corner_mps),
            (SpeedCapOwner.TERMINAL, self.terminal_mps),
            (SpeedCapOwner.CURVATURE, self.curvature_mps),
        )
        items.extend((owner, value) for owner, value in optional if value is not None)
        return tuple(items)


@dataclass(frozen=True, slots=True)
class SpeedRegulatorResult:
    """Resolved speed, ownership, and auditable intermediate values."""

    requested_speed_mps: float
    winning_cap_owner: SpeedCapOwner
    caps: SpeedCaps
    effective_speed_mps: float
    bounded_dt_sec: float
    acceleration_gate_scale: float
    acceleration_progress_m: float
    recovery_active: bool
    recovery_transition: str
    corner_required_braking_distance_m: Optional[float]
    terminal_required_braking_distance_m: Optional[float]


def _finite_nonnegative(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return value


def braking_distance(
    effective_speed_mps: float,
    target_speed_mps: float,
    deceleration_mps2: float,
    latency_sec: float,
    margin_m: float,
) -> float:
    """Return required distance to reduce effective speed to target speed."""

    speed = _finite_nonnegative("effective_speed_mps", effective_speed_mps)
    target = _finite_nonnegative("target_speed_mps", target_speed_mps)
    deceleration = _finite_nonnegative("deceleration_mps2", deceleration_mps2)
    latency = _finite_nonnegative("latency_sec", latency_sec)
    margin = _finite_nonnegative("margin_m", margin_m)
    if deceleration <= 0.0:
        raise ValueError("deceleration_mps2 must be greater than zero")

    kinetic_distance = max(0.0, speed * speed - target * target) / (
        2.0 * deceleration
    )
    return kinetic_distance + latency * speed + margin


def allowable_speed_for_distance(
    remaining_distance_m: float,
    target_speed_mps: float,
    deceleration_mps2: float,
    latency_sec: float,
    margin_m: float,
) -> float:
    """Invert the bounded braking model for the available path distance.

    Solves ``d = (v^2-v_target^2)/(2a) + latency*v + margin`` for the
    greatest non-negative ``v``.  At or inside the margin the target speed is
    returned; terminal hard zero remains a separate, immediate authority.
    """

    remaining = _finite_nonnegative("remaining_distance_m", remaining_distance_m)
    target = _finite_nonnegative("target_speed_mps", target_speed_mps)
    deceleration = _finite_nonnegative("deceleration_mps2", deceleration_mps2)
    latency = _finite_nonnegative("latency_sec", latency_sec)
    margin = _finite_nonnegative("margin_m", margin_m)
    if deceleration <= 0.0:
        raise ValueError("deceleration_mps2 must be greater than zero")
    if remaining <= margin:
        return target

    usable_distance = remaining - margin
    root_term = (
        (deceleration * latency) ** 2
        + target * target
        + 2.0 * deceleration * usable_distance
    )
    return max(target, -deceleration * latency + math.sqrt(root_term))


def _decreasing_cap(
    magnitude: float,
    start: float,
    full: float,
    ceiling: float,
    floor: float,
) -> float:
    if magnitude <= start:
        return ceiling
    if magnitude >= full:
        return min(ceiling, floor)
    ratio = (magnitude - start) / (full - start)
    return min(ceiling, ceiling + ratio * (floor - ceiling))


def _acceleration_gate(magnitude: float, full_speed: float, blocked: float) -> float:
    if magnitude <= full_speed:
        return 1.0
    if magnitude >= blocked:
        return 0.0
    return 1.0 - (magnitude - full_speed) / (blocked - full_speed)


class LongitudinalRegulator:
    """Stateful min-of-caps speed resolver with an explicit reset boundary."""

    def __init__(self, config: LongitudinalRegulatorConfig) -> None:
        self.config = config
        self._start_progress_m = 0.0
        self._initial_speed_mps = 0.0
        self._last_requested_speed_mps = 0.0
        self._recovery_active = False

    def reset(
        self,
        *,
        along_track_progress_m: float = 0.0,
        initial_speed_mps: float = 0.0,
    ) -> None:
        """Re-arm controlled launch after a stop, new leg, or tracking restart."""

        progress = _finite_nonnegative(
            "along_track_progress_m", along_track_progress_m
        )
        initial = _finite_nonnegative("initial_speed_mps", initial_speed_mps)
        if initial > self.config.hardware_speed_ceiling_mps:
            raise ValueError("initial_speed_mps exceeds hardware speed ceiling")
        self._start_progress_m = progress
        self._initial_speed_mps = initial
        self._last_requested_speed_mps = initial
        self._recovery_active = False

    def resolve(self, request: SpeedRegulatorInput) -> SpeedRegulatorResult:
        """Resolve one finite non-negative speed command."""

        config = self.config
        mission_ceiling = _finite_nonnegative(
            "mission_speed_ceiling_mps", request.mission_speed_ceiling_mps
        )
        measured_speed = abs(
            self._finite_runtime("measured_speed_mps", request.measured_speed_mps)
        )
        last_commanded = abs(
            self._finite_runtime(
                "last_commanded_speed_mps", request.last_commanded_speed_mps
            )
        )
        dt_sec = self._finite_runtime("dt_sec", request.dt_sec)
        progress = _finite_nonnegative(
            "along_track_progress_m", request.along_track_progress_m
        )
        heading_error = abs(
            self._finite_runtime("heading_error_rad", request.heading_error_rad)
        )
        cross_track = abs(
            self._finite_runtime(
                "cross_track_error_m", request.cross_track_error_m
            )
        )
        distance_to_corner = self._optional_nonnegative(
            "distance_to_corner_m", request.distance_to_corner_m
        )
        corner_angle = self._optional_finite(
            "corner_angle_rad", request.corner_angle_rad
        )
        distance_to_terminal = self._optional_nonnegative(
            "distance_to_terminal_m", request.distance_to_terminal_m
        )
        terminal_target_speed = self._optional_nonnegative(
            "terminal_target_speed_override_mps",
            request.terminal_target_speed_override_mps,
        )
        if terminal_target_speed is None:
            terminal_target_speed = config.terminal_target_speed_mps
        if terminal_target_speed > config.hardware_speed_ceiling_mps:
            raise ValueError(
                "terminal_target_speed_override_mps exceeds hardware speed ceiling"
            )
        curvature = self._optional_finite(
            "curvature_inv_m", request.curvature_inv_m
        )
        if not isinstance(request.tracking_acceleration_allowed, bool):
            raise ValueError("tracking_acceleration_allowed must be boolean")
        if not isinstance(request.recovery_requested, bool):
            raise ValueError("recovery_requested must be boolean")
        tracking_cap = self._optional_nonnegative(
            "tracking_speed_cap_mps", request.tracking_speed_cap_mps
        )

        bounded_dt = min(max(0.0, dt_sec), config.control_dt_max_sec)
        base_ceiling = min(mission_ceiling, config.hardware_speed_ceiling_mps)
        effective_speed = max(measured_speed, last_commanded)
        accel_progress = max(0.0, progress - self._start_progress_m)

        heading_gate = _acceleration_gate(
            heading_error,
            config.heading_accel_full_error_rad,
            config.heading_recovery_full_rad,
        )
        cross_track_gate = _acceleration_gate(
            cross_track,
            config.cross_track_accel_full_m,
            config.cross_track_recovery_full_m,
        )

        recovery_was_active = self._recovery_active
        recovery_enter = (
            request.recovery_requested
            or heading_error >= config.heading_recovery_full_rad
            or cross_track >= config.cross_track_recovery_full_m
        )
        recovery_exit = (
            not request.recovery_requested
            and heading_error <= config.heading_recovery_start_rad
            and cross_track <= config.cross_track_recovery_start_m
        )
        if self._recovery_active:
            if recovery_exit:
                self._recovery_active = False
        elif recovery_enter:
            self._recovery_active = True

        if self._recovery_active and not recovery_was_active:
            recovery_transition = "ENTERED"
        elif self._recovery_active:
            recovery_transition = "ACTIVE"
        elif recovery_was_active:
            recovery_transition = "EXITED"
        else:
            recovery_transition = "INACTIVE"

        corner_cap = None
        corner_required = None
        corner_gate = 1.0
        is_hard_corner = (
            distance_to_corner is not None
            and corner_angle is not None
            and abs(corner_angle) >= config.corner_angle_threshold_rad
        )
        if is_hard_corner:
            corner_required = braking_distance(
                effective_speed,
                config.corner_target_speed_mps,
                config.deceleration_mps2,
                config.braking_latency_sec,
                config.braking_margin_m,
            )
            corner_cap = min(
                base_ceiling,
                allowable_speed_for_distance(
                    distance_to_corner,
                    config.corner_target_speed_mps,
                    config.deceleration_mps2,
                    config.braking_latency_sec,
                    config.braking_margin_m,
                ),
            )
            if (
                distance_to_corner
                <= corner_required + config.corner_accel_block_buffer_m
            ):
                corner_gate = 0.0

        terminal_cap = None
        terminal_required = None
        terminal_gate = 1.0
        if distance_to_terminal is not None:
            terminal_required = braking_distance(
                effective_speed,
                terminal_target_speed,
                config.deceleration_mps2,
                config.braking_latency_sec,
                config.braking_margin_m,
            )
            terminal_cap = min(
                base_ceiling,
                allowable_speed_for_distance(
                    distance_to_terminal,
                    terminal_target_speed,
                    config.deceleration_mps2,
                    config.braking_latency_sec,
                    config.braking_margin_m,
                ),
            )
            if distance_to_terminal <= terminal_required:
                terminal_gate = 0.0

        acceleration_gate = min(
            heading_gate,
            cross_track_gate,
            corner_gate,
            terminal_gate,
        )
        time_acceleration_cap = (
            self._last_requested_speed_mps
            + acceleration_gate * config.acceleration_mps2 * bounded_dt
        )
        distance_acceleration_cap = math.sqrt(
            max(config.launch_speed_mps, self._initial_speed_mps) ** 2
            + 2.0 * config.acceleration_mps2 * accel_progress
        )
        acceleration_cap = min(
            base_ceiling,
            time_acceleration_cap,
            distance_acceleration_cap,
        )
        if not request.tracking_acceleration_allowed:
            # Stability authority can block a speed increase without owning
            # launch/zero semantics.  The adapter's separately configured
            # minimum-moving floor may still create controlled initial creep.
            acceleration_cap = min(acceleration_cap, last_commanded)

        heading_cap = _decreasing_cap(
            heading_error,
            config.heading_recovery_start_rad,
            config.heading_recovery_full_rad,
            base_ceiling,
            config.recovery_min_speed_mps,
        )
        cross_track_cap = _decreasing_cap(
            cross_track,
            config.cross_track_recovery_start_m,
            config.cross_track_recovery_full_m,
            base_ceiling,
            config.recovery_min_speed_mps,
        )

        curvature_cap = None
        if (
            config.curvature_enabled
            and curvature is not None
            and abs(curvature) >= config.curvature_epsilon_inv_m
        ):
            curvature_cap = min(
                base_ceiling,
                math.sqrt(config.lateral_acceleration_max_mps2 / abs(curvature)),
            )

        caps = SpeedCaps(
            mission_mps=mission_ceiling,
            hardware_mps=config.hardware_speed_ceiling_mps,
            recovery_mps=(
                min(base_ceiling, config.recovery_min_speed_mps)
                if self._recovery_active
                else None
            ),
            acceleration_mps=acceleration_cap,
            heading_mps=heading_cap,
            cross_track_mps=cross_track_cap,
            tracking_mps=(
                min(base_ceiling, tracking_cap)
                if tracking_cap is not None
                else None
            ),
            corner_mps=corner_cap,
            terminal_mps=terminal_cap,
            curvature_mps=curvature_cap,
        )

        if request.hard_zero:
            requested_speed = 0.0
            owner = SpeedCapOwner.HARD_ZERO
        elif self._recovery_active:
            # Recovery is an intentional moving authority, not an
            # acceleration request.  Normal acceleration/heading/xtrack caps
            # therefore cannot collapse it below the calibrated recovery
            # speed.  Higher-priority safety/preview caps remain eligible.
            recovery_candidates = [
                (SpeedCapOwner.MISSION, caps.mission_mps),
                (SpeedCapOwner.HARDWARE, caps.hardware_mps),
            ]
            if caps.terminal_mps is not None:
                recovery_candidates.append(
                    (SpeedCapOwner.TERMINAL, caps.terminal_mps)
                )
            if caps.corner_mps is not None:
                recovery_candidates.append((SpeedCapOwner.CORNER, caps.corner_mps))
            recovery_candidates.append(
                (SpeedCapOwner.RECOVERY, caps.recovery_mps)
            )
            if caps.tracking_mps is not None:
                recovery_candidates.append(
                    (SpeedCapOwner.TRACKING, caps.tracking_mps)
                )
            if caps.curvature_mps is not None:
                recovery_candidates.append(
                    (SpeedCapOwner.CURVATURE, caps.curvature_mps)
                )
            owner, requested_speed = min(
                recovery_candidates, key=lambda item: item[1]
            )
        else:
            owner, requested_speed = min(
                caps.ordered_items(), key=lambda item: item[1]
            )

        if not math.isfinite(requested_speed) or requested_speed < 0.0:
            raise RuntimeError("longitudinal resolver produced an invalid speed")
        requested_speed = min(requested_speed, base_ceiling)
        self._last_requested_speed_mps = requested_speed

        return SpeedRegulatorResult(
            requested_speed_mps=requested_speed,
            winning_cap_owner=owner,
            caps=caps,
            effective_speed_mps=effective_speed,
            bounded_dt_sec=bounded_dt,
            acceleration_gate_scale=acceleration_gate,
            acceleration_progress_m=accel_progress,
            recovery_active=self._recovery_active,
            recovery_transition=recovery_transition,
            corner_required_braking_distance_m=corner_required,
            terminal_required_braking_distance_m=terminal_required,
        )

    @staticmethod
    def _finite_runtime(name: str, value: float) -> float:
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    @classmethod
    def _optional_finite(cls, name: str, value: Optional[float]) -> Optional[float]:
        if value is None:
            return None
        return cls._finite_runtime(name, value)

    @staticmethod
    def _optional_nonnegative(
        name: str, value: Optional[float]
    ) -> Optional[float]:
        if value is None:
            return None
        return _finite_nonnegative(name, value)
