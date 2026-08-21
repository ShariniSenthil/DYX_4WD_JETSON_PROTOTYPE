from mission_manager.marking_arrival_policy import (
    FAIL_NOW,
    KEEP_APPROACHING,
    START_VERIFICATION,
    after_fail_mode_action,
    phase_a_decision,
    should_publish_rpp_captured,
)


def test_pass_starts_three_second_hold():
    assert (
        phase_a_decision(
            inside_30mm=True,
            stationary=True,
            rpp_outcome="CAPTURED",
            past_goal_plane=False,
        )
        == START_VERIFICATION
    )


def test_missed_outside_30mm_fails_now():
    assert (
        phase_a_decision(
            inside_30mm=False,
            stationary=True,
            rpp_outcome="MISSED",
            past_goal_plane=True,
        )
        == FAIL_NOW
    )


def test_captured_outside_30mm_is_a_miss():
    assert (
        phase_a_decision(
            inside_30mm=False,
            stationary=True,
            rpp_outcome="CAPTURED",
            past_goal_plane=False,
        )
        == FAIL_NOW
    )


def test_still_approaching_keeps_driving():
    assert (
        phase_a_decision(
            inside_30mm=False,
            stationary=False,
            rpp_outcome="",
            past_goal_plane=False,
        )
        == KEEP_APPROACHING
    )


def test_auto_continues_manual_waits():
    assert after_fail_mode_action("AUTO") == "AUTO_CONTINUE"
    assert after_fail_mode_action("MANUAL") == "WAITING_FOR_NEXT"


def test_captured_requires_stationary_inside_radius():
    assert should_publish_rpp_captured(
        target_distance_m=0.02,
        speed_mps=0.005,
        waypoint_tolerance_m=0.03,
        stationary_speed_mps=0.01,
    )
    assert not should_publish_rpp_captured(
        target_distance_m=0.02,
        speed_mps=0.15,
        waypoint_tolerance_m=0.03,
        stationary_speed_mps=0.01,
    )
