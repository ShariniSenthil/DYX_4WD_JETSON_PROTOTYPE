from rpp_controller.point_event_policy import (
    first_marking_approach_is_active,
    latched_stop_terminal_outcome,
    should_release_first_marking,
)


def test_terminal_p1_events_release_first_marking_guidance():
    for event in ("COMPLETED", "FAILED", "ACCURACY_FAILED", "SKIPPED"):
        assert should_release_first_marking(event, 0)


def test_nonterminal_or_non_p1_events_do_not_release_first_marking_guidance():
    assert not should_release_first_marking("MISSION_STOPPED", 0)
    assert not should_release_first_marking("ACCURACY_FAILED", 1)


def test_first_marking_guidance_requires_p1_as_current_semantic_goal():
    common = {
        "first_marking_resolved": False,
        "c_line_locked": True,
        "c_line_bearing_available": True,
        "has_marking_waypoints": True,
    }

    assert first_marking_approach_is_active(segment_goal_number=1, **common)
    assert not first_marking_approach_is_active(segment_goal_number=2, **common)


def test_resolved_p1_cannot_reactivate_first_marking_guidance():
    assert not first_marking_approach_is_active(
        first_marking_resolved=True,
        segment_goal_number=1,
        c_line_locked=True,
        c_line_bearing_available=True,
        has_marking_waypoints=True,
    )


def test_latched_stop_waits_until_rover_is_stationary():
    assert latched_stop_terminal_outcome(
        target_distance=0.028935,
        current_speed=0.089207,
        waypoint_tolerance=0.03,
        stationary_speed_tolerance=0.01,
    ) is None


def test_latched_stop_captures_when_stationary_inside_radius():
    assert latched_stop_terminal_outcome(
        target_distance=0.025,
        current_speed=0.009,
        waypoint_tolerance=0.03,
        stationary_speed_tolerance=0.01,
    ) == "CAPTURED"


def test_latched_stop_captures_on_radius_boundary():
    assert latched_stop_terminal_outcome(
        target_distance=0.03,
        current_speed=0.01,
        waypoint_tolerance=0.03,
        stationary_speed_tolerance=0.01,
    ) == "CAPTURED"


def test_latched_stop_reports_miss_when_stationary_outside_radius():
    assert latched_stop_terminal_outcome(
        target_distance=0.044230,
        current_speed=0.009576,
        waypoint_tolerance=0.03,
        stationary_speed_tolerance=0.01,
    ) == "MISSED"


def test_latched_stop_waits_when_speed_is_not_finite():
    assert latched_stop_terminal_outcome(
        target_distance=0.025,
        current_speed=float("nan"),
        waypoint_tolerance=0.03,
        stationary_speed_tolerance=0.01,
    ) is None
