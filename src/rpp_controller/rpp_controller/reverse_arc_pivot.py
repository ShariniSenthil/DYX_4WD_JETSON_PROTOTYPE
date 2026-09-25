"""ROS-free reverse pivot planner (straight-reverse and reverse-arc modes).

A skid-steer rover turns about a centre C that sits ahead of the nozzle
(measured 25_09: 0.39 m on right turns, 0.49 m on left turns). A stationary
pivot therefore swings the nozzle 0.3-0.6 m off the next line. No speed
profile can keep the nozzle still during the turn (its sideways velocity is
always centre-distance x yaw rate), but reversing while turning can make it
*finish* on the next line.

Each control cycle this planner:
1. advances a smooth (cosine-rate) yaw reference from the start heading to the
   target heading; PX4 follows it in DRIVING state, so it can translate and
   rotate at the same time;
2. from the MEASURED nozzle position and the turn still remaining, predicts the
   nozzle's final cross-track to the next line, and solves one gain K so that
   prediction is zero;
3. commands a signed speed v = K * w(progress) * measured_yaw_rate, where w
   front-loads the reversing toward the start of the turn.

Within the final ``freeze_remaining_deg`` of the turn the speed is zero and the
turn finishes as a plain pivot. Any abort (timeout, reverse-travel cap, or the
rover moving forward when reverse was commanded -- e.g. PX4 RD_OFFB_REV = 0)
latches ``fallback`` so the caller reverts to its stationary pivot.

Mode ``straight`` (default, 2026-09-25 field result): the arc above drives a
turn radius of ~0.05-0.8 m, which crosses half the 0.63 m wheel track. There
the inner side is commanded ~0 m/s, stalls in its deadband and is dragged
sideways by the outer side -- a one-sided, skidding turn with current spikes
on uneven ground. ``straight`` never mixes translation with rotation:
1. reverse STRAIGHT on the held start heading (both sides equal) until the
   turning centre sits where a plain pivot will put the nozzle on the next
   line -- the distance is re-solved each cycle from the measured nozzle;
2. report ``REVERSE_DONE``; the caller then runs its normal stationary pivot
   (both sides equal and opposite).
For a turn about C the reverse distance is ~the centre distance (0.39-0.49 m)
at any angle. Turns whose sine is below ``straight_min_sin`` (near-U-turns,
where reversing cannot move the nozzle across the next line) skip the reverse.

Frames: ENU. x = East, y = North, yaw counter-clockwise from East. The centre
offset is expressed in the nozzle body frame: ``ahead`` along the heading,
``left`` to the left of it.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


MODE_STRAIGHT = "straight"
MODE_ARC = "arc"

__all__ = [
    "MODE_ARC",
    "MODE_STRAIGHT",
    "ReverseArcCommand",
    "ReverseArcConfig",
    "ReverseArcPivot",
]


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True, slots=True)
class ReverseArcConfig:
    """Geometry and limits for the reverse-arc pivot."""

    centre_ahead_left_turn_m: float = 0.49
    centre_left_left_turn_m: float = 0.03
    centre_ahead_right_turn_m: float = 0.39
    centre_left_right_turn_m: float = -0.044
    peak_yaw_rate_radps: float = math.radians(35.0)
    min_duration_sec: float = 3.5
    max_speed_mps: float = 0.30
    freeze_remaining_rad: float = math.radians(12.0)
    front_weight: float = 0.9
    min_turn_rad: float = math.radians(20.0)
    max_reverse_travel_m: float = 1.0
    timeout_factor: float = 2.5
    min_yawspeed_radps: float = math.radians(5.0)
    wrong_direction_speed_mps: float = 0.05
    wrong_direction_sec: float = 0.30
    integration_steps: int = 24
    mode: str = MODE_STRAIGHT
    straight_accel_mps2: float = 0.40
    straight_min_speed_mps: float = 0.08
    straight_done_tol_m: float = 0.02
    straight_min_sin: float = 0.34

    def __post_init__(self) -> None:
        positive = (
            "centre_ahead_left_turn_m",
            "centre_ahead_right_turn_m",
            "peak_yaw_rate_radps",
            "min_duration_sec",
            "max_speed_mps",
            "freeze_remaining_rad",
            "min_turn_rad",
            "max_reverse_travel_m",
            "timeout_factor",
            "min_yawspeed_radps",
            "wrong_direction_speed_mps",
            "wrong_direction_sec",
            "straight_accel_mps2",
            "straight_min_speed_mps",
            "straight_done_tol_m",
            "straight_min_sin",
        )
        for name in positive:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
        for name in ("centre_left_left_turn_m", "centre_left_right_turn_m"):
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite")
        if not 0.0 <= self.front_weight < 1.0:
            raise ValueError("front_weight must be in [0, 1)")
        if self.timeout_factor < 1.0:
            raise ValueError("timeout_factor must be >= 1")
        if self.freeze_remaining_rad >= self.min_turn_rad:
            raise ValueError("freeze_remaining must be smaller than min_turn")
        if self.integration_steps < 4:
            raise ValueError("integration_steps must be >= 4")
        if self.mode not in (MODE_STRAIGHT, MODE_ARC):
            raise ValueError(f"mode must be {MODE_STRAIGHT!r} or {MODE_ARC!r}")
        if self.straight_min_speed_mps > self.max_speed_mps:
            raise ValueError("straight_min_speed_mps must be <= max_speed_mps")
        if self.straight_min_sin >= 1.0:
            raise ValueError("straight_min_sin must be < 1")


@dataclass(frozen=True, slots=True)
class ReverseArcCommand:
    """One cycle of planner output. In arc mode ``yaw_rate_radps`` is always
    nonzero while active so a zero-speed sample never reads as a PX4 pivot
    pause; in straight mode the speed never drops below the minimum while
    active, and the yaw is held with zero rate."""

    active: bool
    fallback: bool
    yaw_enu_rad: float
    yaw_rate_radps: float
    speed_mps: float
    reason: str
    predicted_cross_m: float = float("nan")
    gain_k: float = 0.0
    reverse_travel_m: float = 0.0
    elapsed_sec: float = 0.0


class ReverseArcPivot:
    """Plan and track one reverse pivot. Single-use per pivot: reset().

    ``done`` latches when a straight-mode reverse has finished (or was not
    needed); the caller then flies its normal stationary pivot."""

    def __init__(self, config: ReverseArcConfig):
        self.config = config
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self) -> None:
        self.active = False
        self.fallback = False
        self.fallback_reason = ""
        self.done = False
        self.done_reason = ""
        self.start_sec: Optional[float] = None
        self.start_yaw = 0.0
        self.turn = 0.0
        self.target_yaw = 0.0
        self.duration_sec = 0.0
        self.line_x = 0.0
        self.line_y = 0.0
        self.line_bearing = 0.0
        self.ahead = 0.0
        self.left = 0.0
        self.gain_k = 0.0
        self.reverse_travel_m = 0.0
        self._last_centre: Optional[tuple[float, float, float]] = None
        self._wrong_since: Optional[float] = None
        self._last_speed_cmd = 0.0
        self._straight_timeout_sec: Optional[float] = None

    def eligible(self, start_yaw: float, target_yaw: float) -> bool:
        return abs(_wrap(target_yaw - start_yaw)) >= self.config.min_turn_rad

    def start(
        self,
        now_sec: float,
        start_yaw: float,
        target_yaw: float,
        line_x: float,
        line_y: float,
        line_bearing: float,
    ) -> bool:
        """Latch one manoeuvre. Returns False (inactive) if the turn is too
        small or any input is not finite."""
        self.reset()
        values = (now_sec, start_yaw, target_yaw, line_x, line_y, line_bearing)
        if not all(math.isfinite(float(v)) for v in values):
            return False
        if not self.eligible(start_yaw, target_yaw):
            return False
        cfg = self.config
        self.turn = _wrap(target_yaw - start_yaw)
        self.start_yaw = float(start_yaw)
        self.target_yaw = self.start_yaw + self.turn
        self.line_x = float(line_x)
        self.line_y = float(line_y)
        self.line_bearing = float(line_bearing)
        if self.turn > 0.0:
            self.ahead = cfg.centre_ahead_left_turn_m
            self.left = cfg.centre_left_left_turn_m
        else:
            self.ahead = cfg.centre_ahead_right_turn_m
            self.left = cfg.centre_left_right_turn_m
        # Cosine rate profile: peak rate = (pi/2) * mean rate.
        mean_rate = cfg.peak_yaw_rate_radps * 2.0 / math.pi
        self.duration_sec = max(abs(self.turn) / mean_rate, cfg.min_duration_sec)
        self.start_sec = float(now_sec)
        self.active = True
        return True

    # ------------------------------------------------------------- reference
    def yaw_reference(self, now_sec: float) -> tuple[float, float]:
        """(yaw, yaw_rate) of the cosine-ramped reference at now_sec."""
        if self.start_sec is None:
            return self.target_yaw, 0.0
        s = (now_sec - self.start_sec) / self.duration_sec
        if s <= 0.0:
            return self.start_yaw, 0.0
        if s >= 1.0:
            return self.target_yaw, 0.0
        yaw = self.start_yaw + self.turn * (0.5 - 0.5 * math.cos(math.pi * s))
        rate = self.turn * 0.5 * math.pi / self.duration_sec * math.sin(math.pi * s)
        return yaw, rate

    def _weight(self, progress: float) -> float:
        return max(0.0, 1.0 - self.config.front_weight * min(1.0, max(0.0, progress)))

    def _centre(self, nozzle_x: float, nozzle_y: float, yaw: float) -> tuple[float, float]:
        c, s = math.cos(yaw), math.sin(yaw)
        return (
            nozzle_x + self.ahead * c - self.left * s,
            nozzle_y + self.ahead * s + self.left * c,
        )

    def predict_final_cross(
        self, nozzle_x: float, nozzle_y: float, yaw: float, gain_k: float
    ) -> float:
        """Signed final cross-track (left +) of the nozzle to the next line if
        the rest of the turn runs with gain_k."""
        cx, cy = self._centre(nozzle_x, nozzle_y, yaw)
        remaining = _wrap(self.target_yaw - yaw)
        n_x, n_y = -math.sin(self.line_bearing), math.cos(self.line_bearing)
        tc, ts = math.cos(self.target_yaw), math.sin(self.target_yaw)
        final_x = cx - self.ahead * tc + self.left * ts
        final_y = cy - self.ahead * ts - self.left * tc
        cross = (final_x - self.line_x) * n_x + (final_y - self.line_y) * n_y
        return cross + gain_k * self._sensitivity(yaw, remaining, n_x, n_y)

    def _sensitivity(self, yaw: float, remaining: float, n_x: float, n_y: float) -> float:
        """d(final cross)/dK: integral of w(phi) u(phi).n_line over the rest."""
        steps = self.config.integration_steps
        total = 0.0
        for i in range(steps):
            phi = yaw + remaining * (i + 0.5) / steps
            progress = abs(_wrap(phi - self.start_yaw)) / abs(self.turn)
            total += self._weight(progress) * (
                math.cos(phi) * n_x + math.sin(phi) * n_y
            )
        return total * remaining / steps

    # ------------------------------------------------------------------ step
    def step(
        self,
        now_sec: float,
        nozzle_x: float,
        nozzle_y: float,
        yaw: float,
        measured_yaw_rate: float,
    ) -> ReverseArcCommand:
        cfg = self.config
        if not self.active or self.start_sec is None:
            return self._inactive("NOT_STARTED")
        if not all(
            math.isfinite(float(v))
            for v in (now_sec, nozzle_x, nozzle_y, yaw, measured_yaw_rate)
        ):
            return self._abort("NON_FINITE_INPUT", now_sec)

        elapsed = now_sec - self.start_sec
        if elapsed > self.duration_sec * cfg.timeout_factor + 2.0:
            return self._abort("TIMEOUT", now_sec)

        # Signed along-heading speed of the turning centre, from its own track.
        cx, cy = self._centre(nozzle_x, nozzle_y, yaw)
        if self._last_centre is not None:
            px, py, pt = self._last_centre
            dt = now_sec - pt
            if dt > 1e-3:
                along = (cx - px) * math.cos(yaw) + (cy - py) * math.sin(yaw)
                if along < 0.0:
                    self.reverse_travel_m += -along
                centre_speed = along / dt
                if (
                    self._last_speed_cmd < -cfg.wrong_direction_speed_mps
                    and centre_speed > cfg.wrong_direction_speed_mps
                ):
                    if self._wrong_since is None:
                        self._wrong_since = now_sec
                    elif now_sec - self._wrong_since >= cfg.wrong_direction_sec:
                        return self._abort("REVERSE_NOT_EXECUTED", now_sec)
                else:
                    self._wrong_since = None
        self._last_centre = (cx, cy, now_sec)
        if self.reverse_travel_m > cfg.max_reverse_travel_m:
            return self._abort("MAX_REVERSE_TRAVEL", now_sec)

        if cfg.mode == MODE_STRAIGHT:
            return self._step_straight(now_sec, nozzle_x, nozzle_y, yaw, elapsed)

        yaw_ref, rate_ref = self.yaw_reference(now_sec)
        remaining = _wrap(self.target_yaw - yaw)
        n_x, n_y = -math.sin(self.line_bearing), math.cos(self.line_bearing)

        speed = 0.0
        predicted = self.predict_final_cross(nozzle_x, nozzle_y, yaw, 0.0)
        if abs(remaining) > cfg.freeze_remaining_rad:
            sens = self._sensitivity(yaw, remaining, n_x, n_y)
            self.gain_k = -predicted / sens if abs(sens) > 1e-6 else 0.0
            progress = abs(_wrap(yaw - self.start_yaw)) / abs(self.turn)
            speed = self.gain_k * self._weight(progress) * measured_yaw_rate
            speed = max(-cfg.max_speed_mps, min(cfg.max_speed_mps, speed))
            predicted = self.predict_final_cross(nozzle_x, nozzle_y, yaw, self.gain_k)
        else:
            self.gain_k = 0.0
        self._last_speed_cmd = speed

        yawspeed = math.copysign(
            max(abs(rate_ref), cfg.min_yawspeed_radps),
            self.turn,
        )
        return ReverseArcCommand(
            active=True,
            fallback=False,
            yaw_enu_rad=_wrap(yaw_ref),
            yaw_rate_radps=yawspeed,
            speed_mps=speed,
            reason="FROZEN_FINAL_TURN" if speed == 0.0 and abs(remaining)
            <= cfg.freeze_remaining_rad else "TRACKING",
            predicted_cross_m=predicted,
            gain_k=self.gain_k,
            reverse_travel_m=self.reverse_travel_m,
            elapsed_sec=elapsed,
        )

    def straight_remaining(self, nozzle_x: float, nozzle_y: float, yaw: float) -> float:
        """Reverse distance (m, > 0 = still to reverse) after which a plain
        pivot about the centre puts the nozzle on the next line. NaN when the
        turn is too close to 0 or 180 deg for reversing to move the nozzle
        across the next line."""
        n_x, n_y = -math.sin(self.line_bearing), math.cos(self.line_bearing)
        # Reversing s along the heading moves the centre by -s*h, which moves
        # the predicted final cross-track by -s*(h . n).
        h_dot_n = math.cos(yaw) * n_x + math.sin(yaw) * n_y
        if abs(h_dot_n) < self.config.straight_min_sin:
            return math.nan
        return self.predict_final_cross(nozzle_x, nozzle_y, yaw, 0.0) / h_dot_n

    def _step_straight(
        self, now_sec: float, nozzle_x: float, nozzle_y: float, yaw: float,
        elapsed: float,
    ) -> ReverseArcCommand:
        cfg = self.config
        remaining = self.straight_remaining(nozzle_x, nozzle_y, yaw)
        predicted = self.predict_final_cross(nozzle_x, nozzle_y, yaw, 0.0)
        if not math.isfinite(remaining):
            return self._finish("TURN_OUT_OF_RANGE", now_sec, predicted)
        if self._straight_timeout_sec is None:
            # Trapezoid at max speed plus generous slack; the travel cap and
            # the wrong-direction check catch a stuck or forward-only rover.
            travel = max(remaining, 0.0)
            ramp = cfg.max_speed_mps / cfg.straight_accel_mps2
            self._straight_timeout_sec = (
                (travel / cfg.max_speed_mps + 2.0 * ramp) * cfg.timeout_factor + 2.0
            )
            self.duration_sec = travel / cfg.max_speed_mps + 2.0 * ramp
        if elapsed > self._straight_timeout_sec:
            return self._abort("TIMEOUT", now_sec)
        if remaining <= cfg.straight_done_tol_m:
            return self._finish("REVERSE_DONE", now_sec, predicted)

        # Trapezoid: accelerate from rest, brake to reach 0 at the target.
        speed = min(
            cfg.max_speed_mps,
            cfg.straight_accel_mps2 * max(elapsed, 0.0) + cfg.straight_min_speed_mps,
            math.sqrt(2.0 * cfg.straight_accel_mps2 * remaining),
        )
        speed = -max(speed, cfg.straight_min_speed_mps)
        self._last_speed_cmd = speed
        return ReverseArcCommand(
            active=True,
            fallback=False,
            yaw_enu_rad=_wrap(self.start_yaw),
            yaw_rate_radps=0.0,
            speed_mps=speed,
            reason="REVERSING",
            predicted_cross_m=predicted,
            reverse_travel_m=self.reverse_travel_m,
            elapsed_sec=elapsed,
        )

    def _finish(self, reason: str, now_sec: float, predicted: float) -> ReverseArcCommand:
        self.active = False
        self.done = True
        self.done_reason = reason
        self._last_speed_cmd = 0.0
        elapsed = (now_sec - self.start_sec) if self.start_sec is not None else 0.0
        return ReverseArcCommand(
            active=False,
            fallback=False,
            yaw_enu_rad=self.target_yaw,
            yaw_rate_radps=0.0,
            speed_mps=0.0,
            reason=reason,
            predicted_cross_m=predicted,
            reverse_travel_m=self.reverse_travel_m,
            elapsed_sec=elapsed if math.isfinite(elapsed) else 0.0,
        )

    def _abort(self, reason: str, now_sec: float) -> ReverseArcCommand:
        self.active = False
        self.fallback = True
        self.fallback_reason = reason
        elapsed = (now_sec - self.start_sec) if self.start_sec is not None else 0.0
        return ReverseArcCommand(
            active=False,
            fallback=True,
            yaw_enu_rad=self.target_yaw,
            yaw_rate_radps=0.0,
            speed_mps=0.0,
            reason=reason,
            reverse_travel_m=self.reverse_travel_m,
            elapsed_sec=elapsed if math.isfinite(elapsed) else 0.0,
        )

    def _inactive(self, reason: str) -> ReverseArcCommand:
        return ReverseArcCommand(
            active=False,
            fallback=self.fallback,
            yaw_enu_rad=self.target_yaw,
            yaw_rate_radps=0.0,
            speed_mps=0.0,
            reason=self.fallback_reason or self.done_reason or reason,
        )
