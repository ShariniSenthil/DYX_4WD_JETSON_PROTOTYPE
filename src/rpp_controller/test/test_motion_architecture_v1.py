from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RPP = ROOT / "src/rpp_controller/rpp_controller/rpp_controller_node.py"
LAUNCH = ROOT / "src/rover_bringup/launch/rover.launch.py"
BRIDGE = ROOT / "src/jetson_4wd_control/jetson_4wd_control/cmd_vel_bridge.py"
LEGACY = ROOT / "src/rpp_controller/rpp_controller/legacy_alignment.py"


def test_three_control_owners_exist():
    text = RPP.read_text()
    assert "def acceleration_speed_limit" in text
    assert "def deceleration_speed_limit" in text
    assert "def moving_steering_yaw_rate_command" in text
    assert "def stationary_pivot_yaw_rate_command" in text


def test_pivot_has_no_realtime_slow_ramp_or_moving_handover():
    text = LAUNCH.read_text()
    assert "pivot_yaw_rate_slew_radps2" not in text
    assert "pivot_moving_handover_enabled" not in text
    assert "pivot_dynamic_target_enabled" not in text


def test_v6_predictive_steering_is_kept():
    text = LAUNCH.read_text()
    assert '"xtrack_prediction_time_sec": 0.40' in text
    assert '"xtrack_unwind_slew_rate_degps": 60.0' in text
    assert '"trajectory_precision_prediction_time_sec": 0.55' in text
    assert '"straight_lateral_yaw_authority_enabled": False' in text


def test_realtime_course_bias_is_added_to_moving_steering():
    text = RPP.read_text()
    assert "moving_course_bias_enabled" in text
    assert "def _update_moving_course_bias" in text
    start = text.index("def moving_steering_yaw_rate_command")
    end = text.index("def explicit_yaw_rate_command", start)
    assert "_update_moving_course_bias" in text[start:end]


def test_bridge_matches_explicit_yaw_rate_transport():
    text = BRIDGE.read_text()
    assert 'self.declare_parameter("maximum_yaw_rate_radps", 0.75)' in text
    assert "forwards Jetson yaw+yaw-rate atomically" in text


def test_measured_pivot_settle_is_kept():
    text = LEGACY.read_text()
    assert "PIVOT_SETTLE" in text
    assert "stop_yaw_rate_radps" in text
