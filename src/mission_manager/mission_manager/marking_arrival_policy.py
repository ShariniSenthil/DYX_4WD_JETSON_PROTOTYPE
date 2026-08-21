"""Pure marking-arrival gate used by Mission Manager and bag replay.

Pass: inside 30 mm and stationary -> start the 3 s hold, then spray.
Miss: stationary, not inside 30 mm, and (RPP MISSED or past the goal plane)
      -> no 3 s hold; mark FAILED immediately.
      AUTO continues to the next point. MANUAL waits for NEXT.
Approach: anything else -> keep driving, do not start 3 s.
"""

from __future__ import annotations

START_VERIFICATION = "START_VERIFICATION"
FAIL_NOW = "FAIL_NOW"
KEEP_APPROACHING = "KEEP_APPROACHING"


def phase_a_decision(
    *,
    inside_30mm: bool,
    stationary: bool,
    rpp_outcome: str = "",
    past_goal_plane: bool = False,
) -> str:
    """Decide whether to start the 3 s exam, fail now, or keep approaching."""

    if inside_30mm and stationary:
        return START_VERIFICATION

    handshake = str(rpp_outcome or "").strip().upper()
    # A handshake outside 30 mm is a miss, including a false CAPTURED.
    # Do not start the 3 s hold. AUTO will advance; MANUAL waits for NEXT.
    missed = handshake in {"MISSED", "CAPTURED"}
    if stationary and not inside_30mm and (missed or past_goal_plane):
        return FAIL_NOW

    return KEEP_APPROACHING


def after_fail_mode_action(execution_mode: str) -> str:
    """AUTO continues. MANUAL waits for NEXT."""

    mode = str(execution_mode or "AUTO").strip().upper()
    if mode == "MANUAL":
        return "WAITING_FOR_NEXT"
    return "AUTO_CONTINUE"


def should_publish_rpp_captured(
    *,
    target_distance_m: float,
    speed_mps: float,
    waypoint_tolerance_m: float,
    stationary_speed_mps: float,
) -> bool:
    """CAPTURED is a stop-inside-30 mm handshake, not a moving entry ping."""

    return (
        target_distance_m <= waypoint_tolerance_m
        and speed_mps <= stationary_speed_mps
    )
