#!/usr/bin/env python3

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float64, String
from tf_transformations import euler_from_quaternion


class RPPController(Node):
    """Production straight-segment controller with hardened speed shaping.

    Motion contract:
      - The RPP controller owns acceleration and deceleration.
      - Every new straight segment accelerates from zero toward 0.40 m/s.
      - Nominal acceleration distance is 0.50 m.
      - Every marking approach decelerates during the final 0.50 m measured
        to the 30 mm waypoint-radius boundary.
      - Deceleration reduces 0.40 m/s to a controlled 0.15 m/s floor.
      - Exact radial distance <=30 mm bypasses the floor and commands 0.00 m/s.
      - Acceleration magnitude is 0.16 m/s^2.
      - Derived deceleration magnitude is 0.1375 m/s^2 for
        0.40 -> 0.15 m/s over 0.50 m.
      - Long segments use a trapezoidal profile. On short segments, the
        acceleration envelope remains authoritative when it is below the
        deceleration floor, preventing an unsafe speed jump.
      - PX4 native pivot/alignment bypasses straight speed shaping.
      - Exact radial distance <=30 mm commands immediate zero and latches it.
      - No reverse-throttle command is generated.

    Existing fixed-line, cross-track, terminal capture, marking and safety
    behavior from the canonical workspace is retained unless it conflicts
    with the exact-radius acceleration/deceleration profile.
    """

    CONTROL_HZ = 20.0
    MAXIMUM_MOVING_SPEED_MPS = 0.40
    MAX_MOVING_HEADING_ERROR_RAD = math.radians(30.0)
    WAYPOINT_CHANGE_EPSILON_M = 0.001

    def __init__(self):
        super().__init__("rpp_controller")

        self.declare_parameter("local_frame", "map")
        self.declare_parameter("cruise_speed_mps", 0.40)

        # RPP-owned acceleration-only profile. At 0.40 m/s over 0.50 m,
        # the derived constant acceleration is 0.16 m/s^2 and the nominal
        # ramp duration is 2.50 seconds.
        self.declare_parameter("acceleration_enabled", True)
        self.declare_parameter("acceleration_distance_m", 0.50)
        self.declare_parameter("acceleration_startup_ceiling_mps", 0.15)
        self.declare_parameter("acceleration_max_progress_jump_m", 0.10)
        self.declare_parameter("acceleration_max_dt_sec", 0.10)

        # RPP-owned final approach profile:
        #   0.40 -> 0.15 m/s over 0.50 m,
        #   then immediate 0.00 m/s at the 30 mm radial boundary.
        self.declare_parameter("deceleration_enabled", True)
        self.declare_parameter("deceleration_distance_m", 0.50)
        self.declare_parameter(
            "deceleration_floor_speed_mps",
            0.15,
        )
        self.declare_parameter("deceleration_max_progress_jump_m", 0.10)
        self.declare_parameter("deceleration_max_dt_sec", 0.10)

        self.declare_parameter("minimum_speed_mps", 0.40)
        self.declare_parameter("segment_alignment_speed_mps", 0.40)
        self.declare_parameter(
            "segment_alignment_recovery_speed_mps",
            0.40,
        )
        self.declare_parameter(
            "segment_alignment_deadband_enter_cross_track_m",
            0.15,
        )
        self.declare_parameter(
            "segment_alignment_deadband_exit_cross_track_m",
            0.08,
        )
        self.declare_parameter(
            "segment_alignment_min_effective_heading_error_deg",
            10.0,
        )
        self.declare_parameter(
            "segment_alignment_correction_limit_deg",
            18.0,
        )
        self.declare_parameter(
            "segment_alignment_cross_track_tolerance_m",
            0.03,
        )
        self.declare_parameter(
            "segment_alignment_reentry_cross_track_m",
            0.15,
        )
        self.declare_parameter(
            "segment_alignment_max_cross_track_m",
            0.60,
        )
        self.declare_parameter(
            "terminal_line_entry_cross_track_m",
            0.03,
        )
        self.declare_parameter(
            "xtrack_priority_enter_m",
            0.015,
        )
        self.declare_parameter(
            "xtrack_priority_exit_m",
            0.008,
        )
        self.declare_parameter(
            "xtrack_priority_hold_sec",
            0.30,
        )
        self.declare_parameter(
            "xtrack_priority_speed_mps",
            0.40,
        )
        self.declare_parameter(
            "xtrack_priority_lookahead_m",
            0.55,
        )
        self.declare_parameter(
            "xtrack_priority_correction_limit_deg",
            22.0,
        )
        self.declare_parameter(
            "xtrack_prediction_time_sec",
            0.25,
        )
        self.declare_parameter(
            "xtrack_rate_filter_alpha",
            0.20,
        )
        self.declare_parameter(
            "xtrack_correction_slew_rate_degps",
            30.0,
        )
        self.declare_parameter(
            "xtrack_neutral_crossing_band_m",
            0.015,
        )
        self.declare_parameter(
            "xtrack_priority_release_heading_deg",
            12.0,
        )

        # Dedicated final straight-line profile. These values are used only
        # inside terminal_goal_intercept_distance_m.
        self.declare_parameter(
            "terminal_xtrack_lookahead_m",
            0.50,
        )
        self.declare_parameter(
            "terminal_xtrack_correction_limit_deg",
            22.0,
        )
        self.declare_parameter(
            "terminal_xtrack_prediction_time_sec",
            0.25,
        )
        self.declare_parameter(
            "terminal_xtrack_neutral_crossing_band_m",
            0.004,
        )
        self.declare_parameter(
            "terminal_xtrack_correction_slew_rate_degps",
            35.0,
        )
        self.declare_parameter(
            "terminal_xtrack_unwind_slew_rate_degps",
            50.0,
        )
        self.declare_parameter(
            "terminal_xtrack_away_lookahead_m",
            0.30,
        )
        self.declare_parameter(
            "terminal_xtrack_away_correction_limit_deg",
            28.0,
        )
        self.declare_parameter(
            "terminal_xtrack_away_rate_threshold_mps",
            0.008,
        )
        self.declare_parameter(
            "terminal_xtrack_crossing_prediction_time_sec",
            0.55,
        )
        self.declare_parameter(
            "terminal_xtrack_crossing_lookahead_m",
            0.40,
        )
        self.declare_parameter(
            "terminal_xtrack_crossing_correction_limit_deg",
            24.0,
        )
        self.declare_parameter(
            "terminal_xtrack_crossing_rate_threshold_mps",
            0.010,
        )
        self.declare_parameter(
            "terminal_xtrack_crossing_predicted_threshold_m",
            0.004,
        )
        self.declare_parameter("slow_distance_m", 0.80)
        self.declare_parameter(
            "decel_profile_distance_1_m",
            0.60,
        )
        self.declare_parameter(
            "decel_profile_speed_1_mps",
            0.40,
        )
        self.declare_parameter(
            "decel_profile_distance_2_m",
            0.35,
        )
        self.declare_parameter(
            "decel_profile_speed_2_mps",
            0.40,
        )
        self.declare_parameter(
            "decel_profile_distance_3_m",
            0.20,
        )
        self.declare_parameter(
            "decel_profile_speed_3_mps",
            0.40,
        )
        self.declare_parameter("final_speed_distance_m", 0.12)
        self.declare_parameter("waypoint_tolerance_m", 0.03)

        self.declare_parameter("pivot_enter_angle_deg", 45.0)
        self.declare_parameter("pivot_exit_angle_deg", 12.0)
        self.declare_parameter("alignment_hold_sec", 0.50)
        self.declare_parameter("maximum_yaw_rate_radps", 0.20)
        self.declare_parameter("minimum_yaw_rate_radps", 0.06)
        self.declare_parameter("pivot_yaw_kp", 1.00)
        self.declare_parameter(
            "alignment_reentry_goal_distance_m",
            0.60,
        )

        self.declare_parameter("path_correction_limit_deg", 18.0)
        self.declare_parameter(
            "terminal_line_correction_limit_deg",
            18.0,
        )
        self.declare_parameter(
            "terminal_line_alignment_distance_m",
            0.15,
        )
        self.declare_parameter(
            "line_tracking_lookahead_m",
            0.55,
        )
        self.declare_parameter(
            "alignment_release_accel_distance_m",
            0.30,
        )
        self.declare_parameter("heading_full_speed_deg", 2.0)
        self.declare_parameter("heading_min_speed_deg", 4.0)

        self.declare_parameter(
            "marking_terminal_speed_start_distance_m",
            0.15,
        )
        self.declare_parameter(
            "marking_terminal_max_speed_mps",
            0.40,
        )
        self.declare_parameter(
            "marking_final_creep_start_distance_m",
            0.08,
        )
        self.declare_parameter(
            "marking_final_creep_speed_mps",
            0.40,
        )
        self.declare_parameter(
            "marking_final_creep_cross_track_m",
            0.025,
        )
        self.declare_parameter(
            "terminal_capture_gate_cross_track_m",
            0.020,
        )
        self.declare_parameter(
            "terminal_capture_gate_heading_deg",
            4.0,
        )
        self.declare_parameter(
            "terminal_capture_gate_hold_sec",
            0.20,
        )
        self.declare_parameter(
            "terminal_recovery_min_heading_error_deg",
            8.0,
        )
        self.declare_parameter(
            "terminal_recovery_correction_limit_deg",
            22.0,
        )
        self.declare_parameter(
            "terminal_recovery_lookahead_min_m",
            0.30,
        )
        self.declare_parameter(
            "terminal_exact_target_start_distance_m",
            0.08,
        )
        self.declare_parameter(
            "terminal_goal_intercept_distance_m",
            1.20,
        )
        self.declare_parameter(
            "terminal_goal_intercept_bearing_limit_deg",
            22.0,
        )
        self.declare_parameter(
            "terminal_native_pivot_enter_error_deg",
            45.0,
        )
        self.declare_parameter(
            "terminal_native_pivot_release_error_deg",
            12.0,
        )
        self.declare_parameter(
            "terminal_native_pivot_request_error_deg",
            60.0,
        )
        self.declare_parameter(
            "terminal_close_recovery_distance_m",
            0.10,
        )
        self.declare_parameter(
            "terminal_close_recovery_speed_mps",
            0.40,
        )
        self.declare_parameter(
            "terminal_unready_hold_along_m",
            0.05,
        )
        self.declare_parameter(
            "marking_stop_settle_timeout_sec",
            3.0,
        )
        self.declare_parameter(
            "stationary_speed_tolerance_mps",
            0.01,
        )
        self.declare_parameter("marking_stop_latency_sec", 0.24)
        self.declare_parameter("marking_stop_extra_margin_m", 0.015)
        self.declare_parameter("marking_stop_min_buffer_m", 0.060)
        self.declare_parameter("marking_stop_max_buffer_m", 0.100)
        self.declare_parameter("marking_stop_xtrack_limit_m", 0.020)

        self.declare_parameter("marking_capture_arm_distance_m", 0.30)
        self.declare_parameter("marking_capture_abort_distance_m", 0.45)
        self.declare_parameter("miss_margin_m", 0.02)
        self.declare_parameter(
            "marking_along_track_abort_m",
            0.05,
        )

        self.declare_parameter("waypoint_match_tolerance_m", 0.001)
        self.declare_parameter("odom_timeout_sec", 0.50)
        self.declare_parameter("waypoint_timeout_sec", 1.00)

        self.local_frame = str(
            self.get_parameter("local_frame").value
        ).strip()
        self.cruise_speed = float(
            self.get_parameter("cruise_speed_mps").value
        )
        self.acceleration_enabled = bool(
            self.get_parameter("acceleration_enabled").value
        )
        self.acceleration_distance = float(
            self.get_parameter("acceleration_distance_m").value
        )
        self.acceleration_startup_ceiling = float(
            self.get_parameter(
                "acceleration_startup_ceiling_mps"
            ).value
        )
        self.acceleration_max_progress_jump = float(
            self.get_parameter(
                "acceleration_max_progress_jump_m"
            ).value
        )
        self.acceleration_max_dt_sec = float(
            self.get_parameter("acceleration_max_dt_sec").value
        )
        self.acceleration_rate = (
            self.cruise_speed * self.cruise_speed
            / (2.0 * self.acceleration_distance)
        )
        self.acceleration_duration = (
            self.cruise_speed / self.acceleration_rate
        )

        self.deceleration_enabled = bool(
            self.get_parameter("deceleration_enabled").value
        )
        self.deceleration_distance = float(
            self.get_parameter("deceleration_distance_m").value
        )
        self.deceleration_floor_speed = float(
            self.get_parameter(
                "deceleration_floor_speed_mps"
            ).value
        )
        self.deceleration_max_progress_jump = float(
            self.get_parameter(
                "deceleration_max_progress_jump_m"
            ).value
        )
        self.deceleration_max_dt_sec = float(
            self.get_parameter("deceleration_max_dt_sec").value
        )
        self.deceleration_rate = (
            (
                self.cruise_speed * self.cruise_speed
                - self.deceleration_floor_speed
                * self.deceleration_floor_speed
            )
            / (2.0 * self.deceleration_distance)
        )
        self.deceleration_duration = (
            (
                self.cruise_speed
                - self.deceleration_floor_speed
            )
            / self.deceleration_rate
        )

        self.minimum_speed = float(
            self.get_parameter("minimum_speed_mps").value
        )
        self.segment_alignment_speed = float(
            self.get_parameter(
                "segment_alignment_speed_mps"
            ).value
        )
        self.segment_alignment_recovery_speed = float(
            self.get_parameter(
                "segment_alignment_recovery_speed_mps"
            ).value
        )
        self.segment_alignment_deadband_enter_cross_track = float(
            self.get_parameter(
                "segment_alignment_deadband_enter_cross_track_m"
            ).value
        )
        self.segment_alignment_deadband_exit_cross_track = float(
            self.get_parameter(
                "segment_alignment_deadband_exit_cross_track_m"
            ).value
        )
        self.segment_alignment_min_effective_heading_error = (
            math.radians(float(
                self.get_parameter(
                    "segment_alignment_min_effective_heading_error_deg"
                ).value
            ))
        )
        self.segment_alignment_correction_limit = math.radians(float(
            self.get_parameter(
                "segment_alignment_correction_limit_deg"
            ).value
        ))
        self.segment_alignment_cross_track_tolerance = float(
            self.get_parameter(
                "segment_alignment_cross_track_tolerance_m"
            ).value
        )
        self.segment_alignment_reentry_cross_track = float(
            self.get_parameter(
                "segment_alignment_reentry_cross_track_m"
            ).value
        )
        self.segment_alignment_max_cross_track = float(
            self.get_parameter(
                "segment_alignment_max_cross_track_m"
            ).value
        )
        self.terminal_line_entry_cross_track = float(
            self.get_parameter(
                "terminal_line_entry_cross_track_m"
            ).value
        )
        self.xtrack_priority_enter = float(
            self.get_parameter(
                "xtrack_priority_enter_m"
            ).value
        )
        self.xtrack_priority_exit = float(
            self.get_parameter(
                "xtrack_priority_exit_m"
            ).value
        )
        self.xtrack_priority_hold_sec = float(
            self.get_parameter(
                "xtrack_priority_hold_sec"
            ).value
        )
        self.xtrack_priority_speed = float(
            self.get_parameter(
                "xtrack_priority_speed_mps"
            ).value
        )
        self.xtrack_priority_lookahead = float(
            self.get_parameter(
                "xtrack_priority_lookahead_m"
            ).value
        )
        self.xtrack_priority_correction_limit = math.radians(float(
            self.get_parameter(
                "xtrack_priority_correction_limit_deg"
            ).value
        ))
        self.xtrack_prediction_time_sec = float(
            self.get_parameter("xtrack_prediction_time_sec").value
        )
        self.xtrack_rate_filter_alpha = float(
            self.get_parameter("xtrack_rate_filter_alpha").value
        )
        self.xtrack_correction_slew_rate = math.radians(float(
            self.get_parameter(
                "xtrack_correction_slew_rate_degps"
            ).value
        ))
        self.xtrack_neutral_crossing_band = float(
            self.get_parameter(
                "xtrack_neutral_crossing_band_m"
            ).value
        )
        self.xtrack_priority_release_heading = math.radians(float(
            self.get_parameter(
                "xtrack_priority_release_heading_deg"
            ).value
        ))
        self.terminal_xtrack_lookahead = float(
            self.get_parameter(
                "terminal_xtrack_lookahead_m"
            ).value
        )
        self.terminal_xtrack_correction_limit = math.radians(float(
            self.get_parameter(
                "terminal_xtrack_correction_limit_deg"
            ).value
        ))
        self.terminal_xtrack_prediction_time_sec = float(
            self.get_parameter(
                "terminal_xtrack_prediction_time_sec"
            ).value
        )
        self.terminal_xtrack_neutral_crossing_band = float(
            self.get_parameter(
                "terminal_xtrack_neutral_crossing_band_m"
            ).value
        )
        self.terminal_xtrack_correction_slew_rate = math.radians(float(
            self.get_parameter(
                "terminal_xtrack_correction_slew_rate_degps"
            ).value
        ))
        self.terminal_xtrack_unwind_slew_rate = math.radians(float(
            self.get_parameter(
                "terminal_xtrack_unwind_slew_rate_degps"
            ).value
        ))
        self.terminal_xtrack_away_lookahead = float(
            self.get_parameter(
                "terminal_xtrack_away_lookahead_m"
            ).value
        )
        self.terminal_xtrack_away_correction_limit = math.radians(float(
            self.get_parameter(
                "terminal_xtrack_away_correction_limit_deg"
            ).value
        ))
        self.terminal_xtrack_away_rate_threshold = float(
            self.get_parameter(
                "terminal_xtrack_away_rate_threshold_mps"
            ).value
        )
        self.terminal_xtrack_crossing_prediction_time_sec = float(
            self.get_parameter(
                "terminal_xtrack_crossing_prediction_time_sec"
            ).value
        )
        self.terminal_xtrack_crossing_lookahead = float(
            self.get_parameter(
                "terminal_xtrack_crossing_lookahead_m"
            ).value
        )
        self.terminal_xtrack_crossing_correction_limit = math.radians(float(
            self.get_parameter(
                "terminal_xtrack_crossing_correction_limit_deg"
            ).value
        ))
        self.terminal_xtrack_crossing_rate_threshold = float(
            self.get_parameter(
                "terminal_xtrack_crossing_rate_threshold_mps"
            ).value
        )
        self.terminal_xtrack_crossing_predicted_threshold = float(
            self.get_parameter(
                "terminal_xtrack_crossing_predicted_threshold_m"
            ).value
        )
        self.slow_distance = float(
            self.get_parameter("slow_distance_m").value
        )
        self.decel_profile_distance_1 = float(
            self.get_parameter(
                "decel_profile_distance_1_m"
            ).value
        )
        self.decel_profile_speed_1 = float(
            self.get_parameter(
                "decel_profile_speed_1_mps"
            ).value
        )
        self.decel_profile_distance_2 = float(
            self.get_parameter(
                "decel_profile_distance_2_m"
            ).value
        )
        self.decel_profile_speed_2 = float(
            self.get_parameter(
                "decel_profile_speed_2_mps"
            ).value
        )
        self.decel_profile_distance_3 = float(
            self.get_parameter(
                "decel_profile_distance_3_m"
            ).value
        )
        self.decel_profile_speed_3 = float(
            self.get_parameter(
                "decel_profile_speed_3_mps"
            ).value
        )
        self.final_speed_distance = float(
            self.get_parameter("final_speed_distance_m").value
        )
        self.waypoint_tolerance = float(
            self.get_parameter("waypoint_tolerance_m").value
        )

        self.pivot_enter_angle = math.radians(float(
            self.get_parameter("pivot_enter_angle_deg").value
        ))
        self.pivot_exit_angle = math.radians(float(
            self.get_parameter("pivot_exit_angle_deg").value
        ))
        self.alignment_hold_sec = float(
            self.get_parameter("alignment_hold_sec").value
        )
        self.maximum_yaw_rate = float(
            self.get_parameter("maximum_yaw_rate_radps").value
        )
        self.minimum_yaw_rate = float(
            self.get_parameter("minimum_yaw_rate_radps").value
        )
        self.pivot_yaw_kp = float(
            self.get_parameter("pivot_yaw_kp").value
        )
        self.alignment_reentry_goal_distance = float(
            self.get_parameter(
                "alignment_reentry_goal_distance_m"
            ).value
        )

        self.path_correction_limit = math.radians(float(
            self.get_parameter("path_correction_limit_deg").value
        ))
        self.terminal_line_correction_limit = math.radians(float(
            self.get_parameter(
                "terminal_line_correction_limit_deg"
            ).value
        ))
        self.terminal_line_alignment_distance = float(
            self.get_parameter(
                "terminal_line_alignment_distance_m"
            ).value
        )
        self.line_tracking_lookahead = float(
            self.get_parameter(
                "line_tracking_lookahead_m"
            ).value
        )
        self.alignment_release_accel_distance = float(
            self.get_parameter(
                "alignment_release_accel_distance_m"
            ).value
        )
        self.heading_full_speed = math.radians(float(
            self.get_parameter("heading_full_speed_deg").value
        ))
        self.heading_min_speed = math.radians(float(
            self.get_parameter("heading_min_speed_deg").value
        ))

        self.marking_terminal_speed_start_distance = float(
            self.get_parameter(
                "marking_terminal_speed_start_distance_m"
            ).value
        )
        self.marking_terminal_max_speed = float(
            self.get_parameter(
                "marking_terminal_max_speed_mps"
            ).value
        )
        self.marking_final_creep_start_distance = float(
            self.get_parameter(
                "marking_final_creep_start_distance_m"
            ).value
        )
        self.marking_final_creep_speed = float(
            self.get_parameter(
                "marking_final_creep_speed_mps"
            ).value
        )
        self.marking_final_creep_cross_track = float(
            self.get_parameter(
                "marking_final_creep_cross_track_m"
            ).value
        )
        self.terminal_capture_gate_cross_track = float(
            self.get_parameter(
                "terminal_capture_gate_cross_track_m"
            ).value
        )
        self.terminal_capture_gate_heading = math.radians(float(
            self.get_parameter(
                "terminal_capture_gate_heading_deg"
            ).value
        ))
        self.terminal_capture_gate_hold_sec = float(
            self.get_parameter(
                "terminal_capture_gate_hold_sec"
            ).value
        )
        self.terminal_recovery_min_heading_error = math.radians(float(
            self.get_parameter(
                "terminal_recovery_min_heading_error_deg"
            ).value
        ))
        self.terminal_recovery_correction_limit = math.radians(float(
            self.get_parameter(
                "terminal_recovery_correction_limit_deg"
            ).value
        ))
        self.terminal_recovery_lookahead_min = float(
            self.get_parameter(
                "terminal_recovery_lookahead_min_m"
            ).value
        )
        self.terminal_exact_target_start_distance = float(
            self.get_parameter(
                "terminal_exact_target_start_distance_m"
            ).value
        )
        self.terminal_goal_intercept_distance = float(
            self.get_parameter(
                "terminal_goal_intercept_distance_m"
            ).value
        )
        self.terminal_goal_intercept_bearing_limit = math.radians(float(
            self.get_parameter(
                "terminal_goal_intercept_bearing_limit_deg"
            ).value
        ))
        self.terminal_native_pivot_enter_error = math.radians(float(
            self.get_parameter(
                "terminal_native_pivot_enter_error_deg"
            ).value
        ))
        self.terminal_native_pivot_release_error = math.radians(float(
            self.get_parameter(
                "terminal_native_pivot_release_error_deg"
            ).value
        ))
        self.terminal_native_pivot_request_error = math.radians(float(
            self.get_parameter(
                "terminal_native_pivot_request_error_deg"
            ).value
        ))
        self.terminal_close_recovery_distance = float(
            self.get_parameter(
                "terminal_close_recovery_distance_m"
            ).value
        )
        self.terminal_close_recovery_speed = float(
            self.get_parameter(
                "terminal_close_recovery_speed_mps"
            ).value
        )
        self.terminal_unready_hold_along = float(
            self.get_parameter(
                "terminal_unready_hold_along_m"
            ).value
        )
        self.marking_stop_settle_timeout_sec = float(
            self.get_parameter(
                "marking_stop_settle_timeout_sec"
            ).value
        )
        self.stationary_speed_tolerance = float(
            self.get_parameter(
                "stationary_speed_tolerance_mps"
            ).value
        )
        self.marking_stop_latency_sec = float(self.get_parameter("marking_stop_latency_sec").value)
        self.marking_stop_extra_margin = float(self.get_parameter("marking_stop_extra_margin_m").value)
        self.marking_stop_min_buffer = float(self.get_parameter("marking_stop_min_buffer_m").value)
        self.marking_stop_max_buffer = float(self.get_parameter("marking_stop_max_buffer_m").value)
        self.marking_stop_xtrack_limit = float(self.get_parameter("marking_stop_xtrack_limit_m").value)

        self.marking_capture_arm_distance = float(
            self.get_parameter(
                "marking_capture_arm_distance_m"
            ).value
        )
        self.marking_capture_abort_distance = float(
            self.get_parameter(
                "marking_capture_abort_distance_m"
            ).value
        )
        self.miss_margin = float(
            self.get_parameter("miss_margin_m").value
        )
        self.marking_along_track_abort = float(
            self.get_parameter(
                "marking_along_track_abort_m"
            ).value
        )

        self.waypoint_match_tolerance = float(
            self.get_parameter(
                "waypoint_match_tolerance_m"
            ).value
        )
        self.odom_timeout_sec = float(
            self.get_parameter("odom_timeout_sec").value
        )
        self.waypoint_timeout_sec = float(
            self.get_parameter("waypoint_timeout_sec").value
        )

        # --------------------------------------------------------------
        # RPP ACCELERATION-ONLY CONTRACT
        # --------------------------------------------------------------
        self.validate_parameters()
        self.get_logger().warn(
            "RPP ACCELERATION ENABLED | "
            f"distance={self.acceleration_distance:.2f}m | "
            f"target={self.cruise_speed:.2f}m/s | "
            f"rate={self.acceleration_rate:.3f}m/s^2 | "
            f"nominal_time={self.acceleration_duration:.2f}s | "
            "no deceleration / no reverse"
        )

        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        retained_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self.odom_callback,
            odom_qos,
        )
        self.create_subscription(
            PoseStamped,
            "/active_waypoint",
            self.waypoint_callback,
            command_qos,
        )
        self.create_subscription(
            PoseStamped,
            "/segment_goal",
            self.segment_goal_callback,
            command_qos,
        )
        self.create_subscription(
            Path,
            "/mission_waypoints",
            self.marking_waypoints_callback,
            retained_qos,
        )
        self.create_subscription(
            Bool,
            "/mission_enable",
            self.mission_enable_callback,
            command_qos,
        )
        self.create_subscription(
            Bool,
            "/emergency_stop",
            self.emergency_stop_callback,
            command_qos,
        )
        self.create_subscription(
            Bool,
            "/marking_active",
            self.marking_active_callback,
            command_qos,
        )
        self.create_subscription(
            String,
            "/mission_manager/point_event",
            self.point_event_callback,
            command_qos,
        )

        self.velocity_pub = self.create_publisher(
            Vector3Stamped,
            "/rpp/velocity_ned",
            command_qos,
        )

        self.acceleration_active_pub = self.create_publisher(
            Bool,
            "/rpp/acceleration_active",
            retained_qos,
        )
        self.acceleration_progress_pub = self.create_publisher(
            Float64,
            "/rpp/acceleration_progress_m",
            command_qos,
        )
        self.deceleration_active_pub = self.create_publisher(
            Bool,
            "/rpp/deceleration_active",
            retained_qos,
        )
        self.deceleration_progress_pub = self.create_publisher(
            Float64,
            "/rpp/deceleration_progress_m",
            command_qos,
        )
        self.deceleration_remaining_pub = self.create_publisher(
            Float64,
            "/rpp/deceleration_remaining_m",
            command_qos,
        )
        self.command_speed_pub = self.create_publisher(
            Float64,
            "/rpp/command_speed_mps",
            command_qos,
        )

        # Monitoring only. These publishers do not participate in control.
        self.xtrack_mm_pub = self.create_publisher(
            Float64,
            "/rpp/xtrack_mm",
            retained_qos,
        )
        self.goal_distance_mm_pub = self.create_publisher(
            Float64,
            "/rpp/goal_distance_mm",
            retained_qos,
        )
        self.along_remaining_mm_pub = self.create_publisher(
            Float64,
            "/rpp/along_track_remaining_mm",
            retained_qos,
        )
        self.closest_distance_mm_pub = self.create_publisher(
            Float64,
            "/rpp/closest_goal_distance_mm",
            retained_qos,
        )

        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.current_speed_mps = math.inf
        self.last_odom_time = None

        self.target_x = None
        self.target_y = None
        self.target_path_bearing = None
        self.last_waypoint_time = None
        self.target_is_marking = False

        self.segment_goal_x = None
        self.segment_goal_y = None
        self.segment_goal_number = 0

        self.marking_waypoints = []
        self.marking_metadata_received = False

        self.mission_enabled = False
        self.emergency_stop = True
        self.marking_active = False

        self.segment_alignment_active = True
        self.alignment_forward_heading_recovery_active = False
        self.alignment_deadband_recovery_active = False
        self.alignment_inside_since = None
        self.alignment_release_x = None
        self.alignment_release_y = None
        self.alignment_safety_hold = False

        # Global cross-track recovery is independent of marking distance.
        self.xtrack_priority_active = False
        self.xtrack_priority_inside_since = None

        # Filtered predictive cross-track recovery state.
        self.last_xtrack_sample = None
        self.last_xtrack_sample_time = None
        self.filtered_xtrack_rate = 0.0
        self.last_xtrack_correction = 0.0
        self.last_xtrack_correction_time = None

        self.marking_missed = False
        self.capture_monitor_armed = False
        self.closest_marking_distance = math.inf

        self.marking_stop_latched = False
        self.marking_stop_latched_at = None
        self.marking_stop_trigger_radius = None

        # First marking state. C is captured when the mission is enabled.
        self.first_marking_completed = False
        self.first_marking_hold_seen = False
        self.c_line_locked = False
        self.c_line_start_x = None
        self.c_line_start_y = None
        self.c_line_bearing = None
        self.c_line_reanchored_after_pivot = False

        # Continuous final-corridor gate.
        self.terminal_gate_inside_since = None
        self.terminal_gate_ready = False

        # Latched true stationary-pivot target. Pivot output is zero N/E
        # velocity plus a bounded ENU yaw rate. No bearing-carrier translation
        # is permitted while this state is active.
        self.terminal_native_pivot_active = False
        self.terminal_native_pivot_true_bearing = None
        self.terminal_native_pivot_request_bearing = None  # legacy, unused
        self.terminal_native_pivot_reason = ""

        # Straight-segment acceleration state. It is reset by every literal
        # zero, every new segment goal, mission disable and native pivot.
        self.acceleration_active = False
        self.acceleration_complete = False
        self.acceleration_start_x = None
        self.acceleration_start_y = None
        self.acceleration_progress_m = 0.0
        self.acceleration_elapsed_sec = 0.0
        self.acceleration_output_speed = 0.0
        self.acceleration_last_update_time = None
        self.acceleration_jump_warning_emitted = False

        # Symmetric final-500 mm deceleration state. Progress is monotonic so
        # GNSS/odometry jitter cannot command an acceleration near the point.
        self.deceleration_active = False
        self.deceleration_complete = False
        self.deceleration_progress_m = 0.0
        self.deceleration_remaining_m = self.deceleration_distance
        self.deceleration_output_speed = self.cruise_speed
        self.deceleration_last_update_time = None
        self.deceleration_jump_warning_emitted = False

        now = self.get_clock().now()
        self.last_log_time = now
        self.last_wait_log_time = now
        self.last_mm_monitor_log_time = now

        self.timer = self.create_timer(
            1.0 / self.CONTROL_HZ,
            self.control_loop,
        )
        self.publish_motion_profile_monitor(0.0)

        self.get_logger().warn(
            "===== C->P1->PN INTERPOLATED PRECISION CONTROLLER STARTED ====="
        )
        self.get_logger().warn(
            "MM monitor topics: /rpp/xtrack_mm, "
            "/rpp/goal_distance_mm, "
            "/rpp/along_track_remaining_mm, "
            "/rpp/closest_goal_distance_mm"
        )
        self.get_logger().warn(
            "First marking contract: pre-pivot C defines alignment bearing; "
            "post-pivot measured C defines final C->P1 travel line; "
            "generated 50mm points remain pass-through guidance"
        )
        self.get_logger().warn(
            f"Cruise speed         : {self.cruise_speed:.3f} m/s"
        )
        self.get_logger().warn(
            f"Fixed non-zero speed : {self.minimum_speed:.3f} m/s"
        )
        self.get_logger().warn(
            "Moving segment entry : "
            f"speed={self.segment_alignment_speed:.3f}m/s, "
            f"correction="
            f"{math.degrees(self.segment_alignment_correction_limit):.1f}deg, "
            f"release xtrack<="
            f"{self.segment_alignment_cross_track_tolerance:.3f}m"
        )
        self.get_logger().warn(
            "Deadband-aware entry recovery: "
            f"speed={self.segment_alignment_recovery_speed:.3f}m/s, "
            f"enter xtrack>="
            f"{self.segment_alignment_deadband_enter_cross_track:.3f}m, "
            "hold until xtrack<="
            f"{self.segment_alignment_cross_track_tolerance:.3f}m, "
            f"minimum effective heading error="
            f"{math.degrees(self.segment_alignment_min_effective_heading_error):.1f}deg"
        )
        self.get_logger().warn(
            f"Marking tolerance    : {self.waypoint_tolerance:.3f} m"
        )
        self.get_logger().warn(
            "Continuous line tracker: "
            f"lookahead={self.line_tracking_lookahead:.2f}m, "
            f"far correction="
            f"{math.degrees(self.path_correction_limit):.1f}deg, "
            f"terminal correction="
            f"{math.degrees(self.terminal_line_correction_limit):.1f}deg"
        )
        self.get_logger().warn(
            "Adaptive terminal capture: "
            f"starts at {self.terminal_line_alignment_distance:.3f}m, "
            f"line-ready xtrack<="
            f"{self.terminal_line_entry_cross_track:.3f}m"
        )
        self.get_logger().warn(
            "RPP SPEED PROFILE: "
            f"0.00->{self.cruise_speed:.2f}m/s over "
            f"{self.acceleration_distance:.2f}m; "
            f"{self.cruise_speed:.2f}->"
            f"{self.deceleration_floor_speed:.2f}m/s over final "
            f"{self.deceleration_distance:.2f}m; "
            f"then 0.00m/s at radial <= "
            f"{self.waypoint_tolerance:.3f}m"
        )
        self.get_logger().warn(
            "Terminal alignment corridor: starts at "
            f"{self.terminal_line_alignment_distance:.2f}m from the "
            "exact marking coordinate, "
            f"terminal speed={self.marking_terminal_max_speed:.3f}m/s, "
            f"capture gate xtrack<="
            f"{self.terminal_capture_gate_cross_track:.3f}m, "
            f"heading<="
            f"{math.degrees(self.terminal_capture_gate_heading):.1f}deg "
            f"for {self.terminal_capture_gate_hold_sec:.2f}s"
        )
        self.get_logger().warn(
            "Terminal/final movement: fixed guidance at "
            f"{self.cruise_speed:.3f}m/s until the exact zero latch"
        )
        self.get_logger().warn(
            "Terminal steering contract: stationary absolute-yaw attitude "
            "setpoint; "
            f"enter={math.degrees(self.terminal_native_pivot_enter_error):.1f}deg, "
            f"release={math.degrees(self.terminal_native_pivot_release_error):.1f}deg; "
            "translation remains exactly 0.40m/s only after alignment"
        )
        self.get_logger().warn(
            "Close terminal movement: constant "
            f"{self.cruise_speed:.3f}m/s; no reduced-speed state"
        )
        self.get_logger().warn(
            "Terminal forward-heading recovery: "
            f"speed={self.segment_alignment_recovery_speed:.3f}m/s, "
            f"minimum effective heading error="
            f"{math.degrees(self.segment_alignment_min_effective_heading_error):.1f}deg"
        )
        self.get_logger().warn(
            "Global cross-track priority: enter at "
            f"{self.xtrack_priority_enter:.3f}m, recover at "
            f"{self.xtrack_priority_speed:.3f}m/s, release at "
            f"{self.xtrack_priority_exit:.3f}m with heading <= "
            f"{math.degrees(self.xtrack_priority_release_heading):.1f}deg "
            f"for {self.xtrack_priority_hold_sec:.2f}s"
        )
        self.get_logger().warn(
            "Alignment state priority: cross-track recovery outside "
            f"{self.segment_alignment_cross_track_tolerance:.3f}m, "
            "then forward-heading correction"
        )
        self.get_logger().warn(
            "Alignment safety hold: stop when abs(xtrack) >= "
            f"{self.segment_alignment_max_cross_track:.3f}m"
        )
        self.get_logger().warn(
            "Forward-heading correction: active inside the line corridor "
            f"until path error <= {math.degrees(self.pivot_exit_angle):.1f}deg"
        )
        self.get_logger().warn(
            "Forward marking gate: handoff requires "
            f"path heading <= {math.degrees(self.pivot_exit_angle):.1f}deg "
            "and cross-track <= "
            f"{self.segment_alignment_cross_track_tolerance:.3f}m"
        )
        self.get_logger().warn(
            "Exact marking zero: command zero only when "
            f"distance <= {self.waypoint_tolerance:.3f}m"
        )

    def validate_parameters(self):
        if not self.local_frame:
            raise ValueError("local_frame must not be empty")

        positive_values = {
            "cruise_speed_mps": self.cruise_speed,
            "acceleration_distance_m": self.acceleration_distance,
            "acceleration_startup_ceiling_mps": (
                self.acceleration_startup_ceiling
            ),
            "acceleration_max_progress_jump_m": (
                self.acceleration_max_progress_jump
            ),
            "acceleration_max_dt_sec": self.acceleration_max_dt_sec,
            "deceleration_distance_m": self.deceleration_distance,
            "deceleration_floor_speed_mps": (
                self.deceleration_floor_speed
            ),
            "deceleration_max_progress_jump_m": (
                self.deceleration_max_progress_jump
            ),
            "deceleration_max_dt_sec": self.deceleration_max_dt_sec,
            "minimum_speed_mps": self.minimum_speed,
            "segment_alignment_speed_mps": (
                self.segment_alignment_speed
            ),
            "segment_alignment_recovery_speed_mps": (
                self.segment_alignment_recovery_speed
            ),
            "segment_alignment_deadband_enter_cross_track_m": (
                self.segment_alignment_deadband_enter_cross_track
            ),
            "segment_alignment_deadband_exit_cross_track_m": (
                self.segment_alignment_deadband_exit_cross_track
            ),
            "segment_alignment_min_effective_heading_error_deg": (
                self.segment_alignment_min_effective_heading_error
            ),
            "segment_alignment_cross_track_tolerance_m": (
                self.segment_alignment_cross_track_tolerance
            ),
            "segment_alignment_reentry_cross_track_m": (
                self.segment_alignment_reentry_cross_track
            ),
            "segment_alignment_max_cross_track_m": (
                self.segment_alignment_max_cross_track
            ),
            "terminal_line_entry_cross_track_m": (
                self.terminal_line_entry_cross_track
            ),
            "xtrack_priority_enter_m": (
                self.xtrack_priority_enter
            ),
            "xtrack_priority_exit_m": (
                self.xtrack_priority_exit
            ),
            "xtrack_priority_hold_sec": (
                self.xtrack_priority_hold_sec
            ),
            "xtrack_priority_speed_mps": (
                self.xtrack_priority_speed
            ),
            "xtrack_priority_lookahead_m": (
                self.xtrack_priority_lookahead
            ),
            "xtrack_priority_correction_limit_deg": (
                self.xtrack_priority_correction_limit
            ),
            "xtrack_prediction_time_sec": (
                self.xtrack_prediction_time_sec
            ),
            "xtrack_rate_filter_alpha": (
                self.xtrack_rate_filter_alpha
            ),
            "xtrack_correction_slew_rate_degps": (
                self.xtrack_correction_slew_rate
            ),
            "xtrack_neutral_crossing_band_m": (
                self.xtrack_neutral_crossing_band
            ),
            "xtrack_priority_release_heading_deg": (
                self.xtrack_priority_release_heading
            ),
            "terminal_xtrack_lookahead_m": (
                self.terminal_xtrack_lookahead
            ),
            "terminal_xtrack_correction_limit_deg": (
                self.terminal_xtrack_correction_limit
            ),
            "terminal_xtrack_prediction_time_sec": (
                self.terminal_xtrack_prediction_time_sec
            ),
            "terminal_xtrack_neutral_crossing_band_m": (
                self.terminal_xtrack_neutral_crossing_band
            ),
            "terminal_xtrack_correction_slew_rate_degps": (
                self.terminal_xtrack_correction_slew_rate
            ),
            "terminal_xtrack_unwind_slew_rate_degps": (
                self.terminal_xtrack_unwind_slew_rate
            ),
            "terminal_xtrack_away_lookahead_m": (
                self.terminal_xtrack_away_lookahead
            ),
            "terminal_xtrack_away_correction_limit_deg": (
                self.terminal_xtrack_away_correction_limit
            ),
            "terminal_xtrack_away_rate_threshold_mps": (
                self.terminal_xtrack_away_rate_threshold
            ),
            "terminal_xtrack_crossing_prediction_time_sec": (
                self.terminal_xtrack_crossing_prediction_time_sec
            ),
            "terminal_xtrack_crossing_lookahead_m": (
                self.terminal_xtrack_crossing_lookahead
            ),
            "terminal_xtrack_crossing_correction_limit_deg": (
                self.terminal_xtrack_crossing_correction_limit
            ),
            "terminal_xtrack_crossing_rate_threshold_mps": (
                self.terminal_xtrack_crossing_rate_threshold
            ),
            "terminal_xtrack_crossing_predicted_threshold_m": (
                self.terminal_xtrack_crossing_predicted_threshold
            ),
            "slow_distance_m": self.slow_distance,
            "decel_profile_distance_1_m": (
                self.decel_profile_distance_1
            ),
            "decel_profile_speed_1_mps": (
                self.decel_profile_speed_1
            ),
            "decel_profile_distance_2_m": (
                self.decel_profile_distance_2
            ),
            "decel_profile_speed_2_mps": (
                self.decel_profile_speed_2
            ),
            "decel_profile_distance_3_m": (
                self.decel_profile_distance_3
            ),
            "decel_profile_speed_3_mps": (
                self.decel_profile_speed_3
            ),
            "final_speed_distance_m": self.final_speed_distance,
            "waypoint_tolerance_m": self.waypoint_tolerance,
            "alignment_hold_sec": self.alignment_hold_sec,
            "maximum_yaw_rate_radps": self.maximum_yaw_rate,
            "minimum_yaw_rate_radps": self.minimum_yaw_rate,
            "pivot_yaw_kp": self.pivot_yaw_kp,
            "alignment_reentry_goal_distance_m": (
                self.alignment_reentry_goal_distance
            ),
            "terminal_line_alignment_distance_m": (
                self.terminal_line_alignment_distance
            ),
            "line_tracking_lookahead_m": (
                self.line_tracking_lookahead
            ),
            "alignment_release_accel_distance_m": (
                self.alignment_release_accel_distance
            ),
            "marking_terminal_speed_start_distance_m": (
                self.marking_terminal_speed_start_distance
            ),
            "marking_terminal_max_speed_mps": (
                self.marking_terminal_max_speed
            ),
            "marking_final_creep_start_distance_m": (
                self.marking_final_creep_start_distance
            ),
            "marking_final_creep_speed_mps": (
                self.marking_final_creep_speed
            ),
            "marking_final_creep_cross_track_m": (
                self.marking_final_creep_cross_track
            ),
            "terminal_capture_gate_cross_track_m": (
                self.terminal_capture_gate_cross_track
            ),
            "terminal_capture_gate_heading_deg": (
                self.terminal_capture_gate_heading
            ),
            "terminal_capture_gate_hold_sec": (
                self.terminal_capture_gate_hold_sec
            ),
            "terminal_recovery_min_heading_error_deg": (
                self.terminal_recovery_min_heading_error
            ),
            "terminal_recovery_correction_limit_deg": (
                self.terminal_recovery_correction_limit
            ),
            "terminal_recovery_lookahead_min_m": (
                self.terminal_recovery_lookahead_min
            ),
            "terminal_exact_target_start_distance_m": (
                self.terminal_exact_target_start_distance
            ),
            "terminal_goal_intercept_distance_m": (
                self.terminal_goal_intercept_distance
            ),
            "terminal_goal_intercept_bearing_limit_deg": (
                self.terminal_goal_intercept_bearing_limit
            ),
            "terminal_native_pivot_enter_error_deg": (
                self.terminal_native_pivot_enter_error
            ),
            "terminal_native_pivot_release_error_deg": (
                self.terminal_native_pivot_release_error
            ),
            "terminal_native_pivot_request_error_deg": (
                self.terminal_native_pivot_request_error
            ),
            "terminal_close_recovery_distance_m": (
                self.terminal_close_recovery_distance
            ),
            "terminal_close_recovery_speed_mps": (
                self.terminal_close_recovery_speed
            ),
            "terminal_unready_hold_along_m": (
                self.terminal_unready_hold_along
            ),
            "marking_stop_settle_timeout_sec": (
                self.marking_stop_settle_timeout_sec
            ),
            "stationary_speed_tolerance_mps": (
                self.stationary_speed_tolerance
            ),
            "marking_stop_latency_sec": self.marking_stop_latency_sec,
            "marking_stop_extra_margin_m": self.marking_stop_extra_margin,
            "marking_stop_min_buffer_m": self.marking_stop_min_buffer,
            "marking_stop_max_buffer_m": self.marking_stop_max_buffer,
            "marking_stop_xtrack_limit_m": self.marking_stop_xtrack_limit,
            "marking_capture_arm_distance_m": (
                self.marking_capture_arm_distance
            ),
            "marking_capture_abort_distance_m": (
                self.marking_capture_abort_distance
            ),
            "miss_margin_m": self.miss_margin,
            "marking_along_track_abort_m": (
                self.marking_along_track_abort
            ),
            "waypoint_match_tolerance_m": (
                self.waypoint_match_tolerance
            ),
            "odom_timeout_sec": self.odom_timeout_sec,
            "waypoint_timeout_sec": self.waypoint_timeout_sec,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")

        if self.minimum_speed > self.cruise_speed:
            raise ValueError(
                "minimum_speed_mps must be <= cruise_speed_mps"
            )
        if (
            self.segment_alignment_speed
            < self.minimum_speed
        ):
            raise ValueError(
                "segment_alignment_speed_mps must be >= minimum speed"
            )
        if not (
            self.segment_alignment_speed
            <= self.segment_alignment_recovery_speed
            <= self.cruise_speed
        ):
            raise ValueError(
                "segment_alignment_recovery_speed_mps must be between "
                "segment_alignment_speed_mps and cruise_speed_mps"
            )
        if not (
            self.segment_alignment_cross_track_tolerance
            < self.segment_alignment_deadband_exit_cross_track
            < self.segment_alignment_deadband_enter_cross_track
        ):
            raise ValueError(
                "deadband exit cross-track must be greater than the "
                "alignment release tolerance and less than the deadband "
                "entry cross-track"
            )
        if not (
            0.0
            < self.segment_alignment_min_effective_heading_error
            <= self.segment_alignment_correction_limit
        ):
            raise ValueError(
                "segment_alignment_min_effective_heading_error_deg must "
                "be positive and no greater than "
                "segment_alignment_correction_limit_deg"
            )
        if not (
            self.path_correction_limit
            <= self.segment_alignment_correction_limit
            < math.pi / 2.0
        ):
            raise ValueError(
                "segment_alignment_correction_limit_deg must be "
                ">= path_correction_limit_deg and < 90 degrees"
            )
        if (
            self.segment_alignment_cross_track_tolerance
            >= self.segment_alignment_reentry_cross_track
        ):
            raise ValueError(
                "segment alignment release cross-track tolerance must "
                "be less than re-entry cross-track threshold"
            )
        if (
            self.segment_alignment_max_cross_track
            <= self.segment_alignment_reentry_cross_track
        ):
            raise ValueError(
                "segment_alignment_max_cross_track_m must be greater "
                "than segment_alignment_reentry_cross_track_m"
            )
        if (
            self.terminal_line_entry_cross_track
            > self.segment_alignment_cross_track_tolerance
        ):
            raise ValueError(
                "terminal_line_entry_cross_track_m must be <= "
                "segment_alignment_cross_track_tolerance_m"
            )
        if not (
            0.0
            < self.xtrack_priority_exit
            < self.xtrack_priority_enter
            <= self.segment_alignment_cross_track_tolerance
        ):
            raise ValueError(
                "xtrack priority requires "
                "0 < exit < enter <= alignment tolerance"
            )
        if not (
            self.minimum_speed
            <= self.xtrack_priority_speed
            <= self.cruise_speed
        ):
            raise ValueError(
                "xtrack_priority_speed_mps must be between minimum speed "
                "and cruise speed"
            )
        if not (
            0.0 < self.xtrack_rate_filter_alpha <= 1.0
        ):
            raise ValueError(
                "xtrack_rate_filter_alpha must be in (0, 1]"
            )
        if (
            self.xtrack_neutral_crossing_band
            < self.xtrack_priority_enter
        ):
            raise ValueError(
                "xtrack_neutral_crossing_band_m must be >= "
                "xtrack_priority_enter_m"
            )
        if not (
            self.path_correction_limit
            <= self.xtrack_priority_correction_limit
            < math.radians(45.0)
        ):
            raise ValueError(
                "xtrack priority correction must be >= normal correction "
                "and below 45deg"
            )
        if not (
            0.0
            < self.xtrack_priority_release_heading
            < math.radians(45.0)
        ):
            raise ValueError(
                "xtrack priority release heading must be below 45deg"
            )
        if not (
            self.path_correction_limit
            <= self.terminal_xtrack_correction_limit
            < math.radians(45.0)
        ):
            raise ValueError(
                "terminal_xtrack_correction_limit_deg must be >= "
                "path correction and below 45deg"
            )
        if (
            self.terminal_xtrack_neutral_crossing_band
            > self.terminal_capture_gate_cross_track
        ):
            raise ValueError(
                "terminal_xtrack_neutral_crossing_band_m must be <= "
                "terminal_capture_gate_cross_track_m"
            )
        if not (
            self.terminal_xtrack_correction_limit
            <= self.terminal_xtrack_away_correction_limit
            < math.radians(45.0)
        ):
            raise ValueError(
                "terminal_xtrack_away_correction_limit_deg must be >= "
                "terminal base correction and below 45deg"
            )
        if not (
            self.terminal_xtrack_away_lookahead
            < self.terminal_xtrack_lookahead
        ):
            raise ValueError(
                "terminal_xtrack_away_lookahead_m must be smaller than "
                "terminal_xtrack_lookahead_m"
            )
        if not (
            self.terminal_xtrack_unwind_slew_rate
            >= self.terminal_xtrack_correction_slew_rate
        ):
            raise ValueError(
                "terminal_xtrack_unwind_slew_rate_degps must be >= "
                "terminal_xtrack_correction_slew_rate_degps"
            )
        if not (
            self.terminal_xtrack_away_lookahead
            <= self.terminal_xtrack_crossing_lookahead
            < self.terminal_xtrack_lookahead
        ):
            raise ValueError(
                "terminal crossing lookahead must be between away and "
                "base terminal lookahead"
            )
        if not (
            self.terminal_xtrack_correction_limit
            <= self.terminal_xtrack_crossing_correction_limit
            <= self.terminal_xtrack_away_correction_limit
        ):
            raise ValueError(
                "terminal crossing correction limit must be between base "
                "and away correction limits"
            )
        if not (
            self.terminal_xtrack_crossing_prediction_time_sec
            > self.terminal_xtrack_prediction_time_sec
        ):
            raise ValueError(
                "terminal crossing prediction time must exceed base "
                "terminal prediction time"
            )
        if self.cruise_speed > self.MAXIMUM_MOVING_SPEED_MPS + 1.0e-9:
            raise ValueError(
                "cruise_speed_mps must not exceed 0.40 m/s"
            )
        if self.acceleration_startup_ceiling >= self.cruise_speed:
            raise ValueError(
                "acceleration_startup_ceiling_mps must be below cruise speed"
            )
        if not math.isfinite(self.acceleration_rate) or self.acceleration_rate <= 0.0:
            raise ValueError("derived acceleration rate must be finite and > 0")
        if not math.isfinite(self.acceleration_duration) or self.acceleration_duration <= 0.0:
            raise ValueError("derived acceleration duration must be finite and > 0")
        if not math.isfinite(self.deceleration_rate) or self.deceleration_rate <= 0.0:
            raise ValueError("derived deceleration rate must be finite and > 0")
        if not math.isfinite(self.deceleration_duration) or self.deceleration_duration <= 0.0:
            raise ValueError("derived deceleration duration must be finite and > 0")
        if not (
            0.0
            < self.deceleration_floor_speed
            < self.cruise_speed
        ):
            raise ValueError(
                "deceleration_floor_speed_mps must be greater than "
                "zero and below cruise_speed_mps"
            )

        profile_distances = (
            self.slow_distance,
            self.decel_profile_distance_1,
            self.decel_profile_distance_2,
            self.decel_profile_distance_3,
            self.final_speed_distance,
            self.marking_final_creep_start_distance,
            self.waypoint_tolerance,
        )
        if not all(
            upper > lower
            for upper, lower in zip(
                profile_distances,
                profile_distances[1:],
            )
        ):
            raise ValueError(
                "speed-profile distances must be strictly descending"
            )

        profile_speeds = (
            self.cruise_speed,
            self.decel_profile_speed_1,
            self.decel_profile_speed_2,
            self.decel_profile_speed_3,
            self.marking_terminal_max_speed,
            self.marking_final_creep_speed,
        )
        if not all(
            upper >= lower
            for upper, lower in zip(
                profile_speeds,
                profile_speeds[1:],
            )
        ):
            raise ValueError(
                "speed-profile speeds must be monotonically decreasing"
            )

        if self.final_speed_distance >= self.slow_distance:
            raise ValueError(
                "final_speed_distance_m must be less than slow_distance_m"
            )
        if not math.isclose(
            self.final_speed_distance,
            self.terminal_line_alignment_distance,
            rel_tol=0.0,
            abs_tol=1.0e-6,
        ):
            raise ValueError(
                "final_speed_distance_m and "
                "terminal_line_alignment_distance_m must match"
            )
        if not math.isclose(
            self.final_speed_distance,
            self.marking_terminal_speed_start_distance,
            rel_tol=0.0,
            abs_tol=1.0e-6,
        ):
            raise ValueError(
                "final_speed_distance_m and "
                "marking_terminal_speed_start_distance_m must match"
            )
        if self.waypoint_tolerance >= self.final_speed_distance:
            raise ValueError(
                "waypoint_tolerance_m must be less than "
                "final_speed_distance_m"
            )
        if not (0.0 < self.marking_stop_min_buffer <= self.marking_stop_max_buffer):
            raise ValueError("marking stop buffers require 0 < min <= max")
        if not (0.0 < self.marking_stop_xtrack_limit < self.waypoint_tolerance):
            raise ValueError("marking_stop_xtrack_limit_m must be below waypoint_tolerance_m")
        if self.pivot_exit_angle >= self.pivot_enter_angle:
            raise ValueError(
                "pivot_exit_angle_deg must be less than "
                "pivot_enter_angle_deg"
            )
        if not (
            math.isfinite(self.minimum_yaw_rate)
            and math.isfinite(self.maximum_yaw_rate)
            and 0.0 < self.minimum_yaw_rate <= self.maximum_yaw_rate
        ):
            raise ValueError(
                "0 < minimum_yaw_rate_radps <= maximum_yaw_rate_radps "
                "is required"
            )
        if not math.isfinite(self.pivot_yaw_kp) or self.pivot_yaw_kp <= 0.0:
            raise ValueError("pivot_yaw_kp must be finite and > 0")
        if self.heading_full_speed >= self.heading_min_speed:
            raise ValueError(
                "heading_full_speed_deg must be less than "
                "heading_min_speed_deg"
            )
        if not (
            0.0
            < self.terminal_line_correction_limit
            <= self.path_correction_limit
            < math.pi / 2.0
        ):
            raise ValueError(
                "terminal/far line correction limits are invalid"
            )
        if self.marking_terminal_max_speed < self.minimum_speed:
            raise ValueError(
                "marking_terminal_max_speed_mps must be >= "
                "minimum_speed_mps"
            )
        if self.marking_terminal_max_speed > self.cruise_speed:
            raise ValueError(
                "marking_terminal_max_speed_mps must be <= "
                "cruise_speed_mps"
            )
        if not (
            self.waypoint_tolerance
            < self.marking_final_creep_start_distance
            < self.final_speed_distance
        ):
            raise ValueError(
                "marking_final_creep_start_distance_m must be "
                "greater than waypoint tolerance and less than "
                "final_speed_distance_m"
            )
        if not (
            0.0
            < self.marking_final_creep_speed
            <= self.marking_terminal_max_speed
        ):
            raise ValueError(
                "marking_final_creep_speed_mps must be finite, positive "
                "and <= marking_terminal_max_speed_mps"
            )
        if not (
            0.0
            < self.marking_final_creep_cross_track
            <= self.terminal_line_entry_cross_track
        ):
            raise ValueError(
                "marking_final_creep_cross_track_m must be positive "
                "and <= terminal_line_entry_cross_track_m"
            )
        if not (
            0.0
            < self.terminal_capture_gate_cross_track
            <= self.marking_final_creep_cross_track
            <= self.terminal_line_entry_cross_track
        ):
            raise ValueError(
                "terminal capture cross-track gates are inconsistent"
            )
        if not (
            0.0
            < self.terminal_capture_gate_heading
            <= self.xtrack_priority_release_heading
        ):
            raise ValueError(
                "terminal_capture_gate_heading_deg must be positive "
                "and <= xtrack_priority_release_heading_deg"
            )
        if not (
            0.0
            < self.terminal_recovery_min_heading_error
            <= self.terminal_recovery_correction_limit
            < math.radians(45.0)
        ):
            raise ValueError(
                "terminal recovery heading values require "
                "0 < minimum <= correction < 45 degrees"
            )
        if not (
            self.waypoint_tolerance
            < self.terminal_exact_target_start_distance
            <= self.terminal_line_alignment_distance
        ):
            raise ValueError(
                "terminal_exact_target_start_distance_m must be between "
                "waypoint tolerance and terminal alignment distance"
            )
        if self.terminal_recovery_lookahead_min > self.line_tracking_lookahead:
            raise ValueError(
                "terminal_recovery_lookahead_min_m must be <= "
                "line_tracking_lookahead_m"
            )
        if (
            self.terminal_goal_intercept_distance
            <= self.terminal_line_alignment_distance
        ):
            raise ValueError(
                "terminal_goal_intercept_distance_m must be greater than "
                "terminal_line_alignment_distance_m"
            )
        if not (
            0.0
            < self.terminal_goal_intercept_bearing_limit
            < math.radians(45.0)
        ):
            raise ValueError(
                "terminal_goal_intercept_bearing_limit_deg must be "
                "between 0 and 45 degrees"
            )
        if not (
            0.0
            < self.terminal_native_pivot_release_error
            < self.terminal_native_pivot_enter_error
            <= math.radians(45.0)
        ):
            raise ValueError(
                "native pivot requires 0 < release < enter <= 45deg"
            )
        if not (
            math.radians(45.0)
            < self.terminal_native_pivot_request_error
            < math.radians(90.0)
        ):
            raise ValueError(
                "terminal_native_pivot_request_error_deg must be "
                "between 45 and 90 degrees"
            )
        if not (
            self.marking_final_creep_start_distance
            < self.terminal_close_recovery_distance
            < self.terminal_line_alignment_distance
        ):
            raise ValueError(
                "terminal close recovery distance must be between "
                "final creep start and terminal alignment distance"
            )
        if not (
            0.0
            < self.terminal_close_recovery_speed
            <= self.marking_terminal_max_speed
        ):
            raise ValueError(
                "terminal_close_recovery_speed_mps must be positive "
                "and <= marking_terminal_max_speed_mps"
            )
        if (
            self.marking_capture_arm_distance
            >= self.marking_capture_abort_distance
        ):
            raise ValueError(
                "marking_capture_arm_distance_m must be less than "
                "marking_capture_abort_distance_m"
            )

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def smoothstep(value):
        value = max(0.0, min(1.0, value))
        return value * value * (3.0 - 2.0 * value)

    @staticmethod
    def pose_bearing(msg):
        q = msg.pose.orientation
        quaternion = (
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        )
        if not all(math.isfinite(value) for value in quaternion):
            return None
        norm_sq = sum(value * value for value in quaternion)
        if norm_sq <= 1.0e-12:
            return None
        _, _, yaw = euler_from_quaternion(quaternion)
        return yaw if math.isfinite(yaw) else None

    def odom_callback(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        linear = msg.twist.twist.linear
        quaternion = (
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        )
        speed_x = float(linear.x)
        speed_y = float(linear.y)
        if not all(
            math.isfinite(value)
            for value in (
                x,
                y,
                *quaternion,
                speed_x,
                speed_y,
            )
        ):
            return

        try:
            _, _, yaw = euler_from_quaternion(quaternion)
        except (TypeError, ValueError):
            return
        if not math.isfinite(yaw):
            return

        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw
        self.current_speed_mps = math.hypot(
            speed_x,
            speed_y,
        )
        self.last_odom_time = self.get_clock().now()

    def marking_waypoints_callback(self, msg):
        if msg.header.frame_id.strip() != self.local_frame:
            return
        points = []
        for pose in msg.poses:
            x = float(pose.pose.position.x)
            y = float(pose.pose.position.y)
            if not all(math.isfinite(value) for value in (x, y)):
                return
            points.append((x, y))
        if not points:
            return

        previous_p1 = (
            self.marking_waypoints[0]
            if self.marking_waypoints
            else None
        )
        self.marking_waypoints = points
        self.marking_metadata_received = True

        if (
            previous_p1 is None
            or math.hypot(
                previous_p1[0] - points[0][0],
                previous_p1[1] - points[0][1],
            ) > self.waypoint_match_tolerance
        ):
            self.first_marking_completed = False
            self.first_marking_hold_seen = False
            self.c_line_locked = False
            self.c_line_bearing = None

        if self.mission_enabled and not self.first_marking_completed:
            self.lock_c_to_p1_line("marking metadata received")

    def target_matches_marking(self, x, y):
        for marking_x, marking_y in self.marking_waypoints:
            if math.hypot(
                x - marking_x,
                y - marking_y,
            ) <= self.waypoint_match_tolerance:
                return True
        return False

    def waypoint_callback(self, msg):
        if msg.header.frame_id.strip() != self.local_frame:
            return

        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        if not all(math.isfinite(value) for value in (x, y)):
            return

        changed = (
            self.target_x is None
            or self.target_y is None
            or math.hypot(
                x - self.target_x,
                y - self.target_y,
            ) > self.WAYPOINT_CHANGE_EPSILON_M
        )

        self.target_x = x
        self.target_y = y
        self.target_path_bearing = self.pose_bearing(msg)
        self.target_is_marking = self.target_matches_marking(x, y)
        self.last_waypoint_time = self.get_clock().now()

        if changed:
            if not self.target_is_marking:
                self.capture_monitor_armed = False
                self.closest_marking_distance = math.inf
                self.marking_stop_latched = False
                self.marking_stop_latched_at = None
            self.marking_stop_trigger_radius = None
            self.get_logger().info(
                "NEW MARKING TARGET"
                if self.target_is_marking
                else "NEW PASS-THROUGH TARGET"
            )

    def segment_goal_callback(self, msg):
        if msg.header.frame_id.strip() != self.local_frame:
            return
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        if not all(math.isfinite(value) for value in (x, y)):
            return

        previous_number = self.segment_goal_number
        changed = (
            self.segment_goal_x is None
            or self.segment_goal_y is None
            or math.hypot(
                x - self.segment_goal_x,
                y - self.segment_goal_y,
            ) > self.WAYPOINT_CHANGE_EPSILON_M
        )
        self.segment_goal_x = x
        self.segment_goal_y = y

        if not changed:
            return

        self.segment_goal_number = self.find_marking_number(x, y)

        # Returning from a later point to P1 represents a new mission.
        if self.segment_goal_number == 1 and previous_number != 1:
            self.first_marking_completed = False
            self.first_marking_hold_seen = False
            self.c_line_locked = False
            self.c_line_bearing = None

        # Every real marking segment, including C->P1, must complete
        # strict heading/cross-track alignment before the immediate
        # fixed 0.40 m/s movement command is released.
        self.segment_alignment_active = True
        self.alignment_forward_heading_recovery_active = False
        self.alignment_deadband_recovery_active = False
        self.alignment_inside_since = None
        self.alignment_release_x = None
        self.alignment_release_y = None
        self.xtrack_priority_active = False
        self.xtrack_priority_inside_since = None
        self.reset_xtrack_damping_state()
        self.reset_speed_profiles()

        self.marking_missed = False
        self.capture_monitor_armed = False
        self.closest_marking_distance = math.inf
        self.marking_stop_latched = False
        self.marking_stop_latched_at = None
        self.marking_stop_trigger_radius = None
        self.terminal_gate_inside_since = None
        self.terminal_gate_ready = False
        self.reset_terminal_native_pivot()

        if (
            self.segment_goal_number == 1
            and not self.first_marking_completed
        ):
            if self.mission_enabled:
                self.lock_c_to_p1_line("segment goal received")
            self.get_logger().warn(
                "STRAIGHT SEGMENT GOAL ACTIVE | P1 | "
                "direct approach / fixed terminal capture line"
            )
        else:
            self.get_logger().warn(
                f"STRAIGHT SEGMENT GOAL ACTIVE | "
                f"P{self.segment_goal_number or '?'} | "
                "interpolated segment alignment required"
            )

    def find_marking_number(self, x, y):
        for index, (marking_x, marking_y) in enumerate(
            self.marking_waypoints,
            start=1,
        ):
            if math.hypot(
                x - marking_x,
                y - marking_y,
            ) <= self.waypoint_match_tolerance:
                return index
        return 0

    def mission_enable_callback(self, msg):
        enabled = bool(msg.data)
        previous = self.mission_enabled
        if enabled != previous:
            self.get_logger().warn(
                "MISSION ENABLED" if enabled else "MISSION DISABLED"
            )
        self.mission_enabled = enabled

        if enabled and not previous:
            self.terminal_gate_inside_since = None
            self.terminal_gate_ready = False
            if not self.first_marking_completed:
                self.c_line_locked = False
                self.c_line_bearing = None
                self.c_line_reanchored_after_pivot = False
                self.lock_c_to_p1_line("mission enabled")
        elif not enabled:
            self.reset_motion_state()
            if not self.first_marking_completed:
                self.c_line_locked = False
                self.c_line_bearing = None

    def emergency_stop_callback(self, msg):
        active = bool(msg.data)
        if active != self.emergency_stop:
            self.get_logger().warn(
                "EMERGENCY STOP ACTIVE"
                if active
                else "EMERGENCY STOP RELEASED"
            )
        self.emergency_stop = active
        if active:
            self.publish_stop()

    def marking_active_callback(self, msg):
        """Track the active three-second marking hold.

        /marking_active means the mission manager is currently timing a hold.
        It is not a completion signal. P1 guidance is released only after a
        COMPLETED point event is received from /mission_manager/point_event.
        """
        active = bool(msg.data)
        previous = self.marking_active
        if active != previous:
            self.get_logger().warn(
                "MARKING HOLD ACTIVE"
                if active
                else "MARKING HOLD RELEASED"
            )

        if active:
            self.reset_terminal_native_pivot()

        self.marking_active = active

    def point_event_callback(self, msg):
        """Release P1 guidance only after the verified completion event."""
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().error(
                "IGNORED INVALID MISSION POINT EVENT"
            )
            return

        event = str(payload.get("event", "")).upper()
        try:
            point_index = int(payload.get("point_index", -1))
        except (TypeError, ValueError):
            point_index = -1

        if (
            event == "COMPLETED"
            and point_index == 0
            and not self.first_marking_completed
        ):
            self.first_marking_hold_seen = True
            self.first_marking_completed = True
            self.c_line_locked = False
            self.c_line_bearing = None
            self.c_line_reanchored_after_pivot = False
            self.get_logger().warn(
                "P1 VERIFIED 3S MARKING COMPLETION EVENT / "
                "P1->P2 INTERPOLATED GUIDANCE RELEASED"
            )

    def reset_motion_state(self):
        self.segment_alignment_active = True
        self.alignment_forward_heading_recovery_active = False
        self.alignment_deadband_recovery_active = False
        self.alignment_inside_since = None
        self.alignment_release_x = None
        self.alignment_release_y = None
        self.alignment_safety_hold = False
        self.xtrack_priority_active = False
        self.xtrack_priority_inside_since = None
        self.reset_xtrack_damping_state()
        self.reset_speed_profiles()

        self.marking_missed = False
        self.capture_monitor_armed = False
        self.closest_marking_distance = math.inf

        self.marking_stop_latched = False
        self.marking_stop_latched_at = None
        self.terminal_gate_inside_since = None
        self.terminal_gate_ready = False
        self.c_line_reanchored_after_pivot = False
        self.reset_terminal_native_pivot()

    def reset_terminal_native_pivot(self):
        self.terminal_native_pivot_active = False
        self.terminal_native_pivot_true_bearing = None
        self.terminal_native_pivot_request_bearing = None
        self.terminal_native_pivot_reason = ""

    def terminal_native_pivot_command(
        self,
        true_bearing,
        reason,
    ):
        """Latch the fixed bearing used for PX4 native rover pivot.

        RPP keeps sending a 0.40 m/s horizontal vector on the latched true
        bearing. PX4 enters native differential pivot at 45 degrees and
        remains in turn mode until true error reaches 12 degrees.
        """
        true_error = self.normalize_angle(
            true_bearing - self.current_yaw
        )

        if self.terminal_native_pivot_active:
            true_bearing = self.terminal_native_pivot_true_bearing
            true_error = self.normalize_angle(
                true_bearing - self.current_yaw
            )
            if abs(true_error) <= self.terminal_native_pivot_release_error:
                self.get_logger().warn(
                    "PX4 NATIVE PIVOT VECTOR RELEASED | "
                    f"reason={self.terminal_native_pivot_reason} | "
                    f"true_error={math.degrees(true_error):.1f}deg"
                )
                self.reset_terminal_native_pivot()
                return False, None, true_error
        else:
            if abs(true_error) < self.terminal_native_pivot_enter_error:
                return False, None, true_error

            self.terminal_native_pivot_active = True
            self.terminal_native_pivot_true_bearing = true_bearing
            self.terminal_native_pivot_request_bearing = true_bearing
            self.terminal_native_pivot_reason = reason
            self.get_logger().warn(
                "PX4 NATIVE PIVOT VECTOR LATCHED | "
                f"reason={reason} | "
                f"bearing={math.degrees(true_bearing):.1f}deg | "
                f"true_error={math.degrees(true_error):.1f}deg"
            )

        return True, true_bearing, true_error


    def lock_c_to_p1_line(self, reason):
        if (
            self.first_marking_completed
            or not self.marking_waypoints
            or self.current_x is None
            or self.current_y is None
        ):
            return False

        p1_x, p1_y = self.marking_waypoints[0]
        delta_east = p1_x - self.current_x
        delta_north = p1_y - self.current_y
        distance = math.hypot(delta_east, delta_north)

        if distance <= 1.0e-6:
            return False

        self.c_line_start_x = self.current_x
        self.c_line_start_y = self.current_y
        self.c_line_bearing = math.atan2(
            delta_north,
            delta_east,
        )
        self.c_line_locked = True
        self.c_line_reanchored_after_pivot = False
        self.segment_alignment_active = True
        self.alignment_inside_since = None

        self.get_logger().warn(
            "C->P1 FIXED LINE LOCKED | "
            f"reason={reason} | "
            f"C_E={self.c_line_start_x:.3f} | "
            f"C_N={self.c_line_start_y:.3f} | "
            f"distance={distance:.3f}m | "
            f"bearing={math.degrees(self.c_line_bearing):.1f}deg"
        )
        return True

    def reanchor_c_to_p1_after_pivot(self):
        """Lock the final travel line from the post-pivot position to P1."""
        if (
            self.first_marking_completed
            or self.c_line_reanchored_after_pivot
            or not self.marking_waypoints
            or self.current_x is None
            or self.current_y is None
        ):
            return False

        p1_x, p1_y = self.marking_waypoints[0]
        old_bearing = self.c_line_bearing
        old_start_x = self.c_line_start_x
        old_start_y = self.c_line_start_y

        delta_east = p1_x - self.current_x
        delta_north = p1_y - self.current_y
        distance = math.hypot(delta_east, delta_north)
        if distance <= self.waypoint_tolerance:
            return False

        old_xtrack = 0.0
        if (
            old_bearing is not None
            and old_start_x is not None
            and old_start_y is not None
        ):
            old_xtrack = (
                -math.sin(old_bearing)
                * (self.current_x - old_start_x)
                + math.cos(old_bearing)
                * (self.current_y - old_start_y)
            )

        self.c_line_start_x = self.current_x
        self.c_line_start_y = self.current_y
        self.c_line_bearing = math.atan2(
            delta_north,
            delta_east,
        )
        self.c_line_reanchored_after_pivot = True
        self.reset_xtrack_damping_state()
        self.xtrack_priority_active = False
        self.xtrack_priority_inside_since = None

        self.get_logger().warn(
            "C->P1 POST-PIVOT TRAVEL LINE RE-ANCHORED | "
            f"old_xtrack={old_xtrack * 1000.0:+.1f}mm | "
            f"C_E={self.c_line_start_x:.3f} | "
            f"C_N={self.c_line_start_y:.3f} | "
            f"distance={distance:.3f}m | "
            f"bearing={math.degrees(self.c_line_bearing):.1f}deg"
        )
        return True

    def first_marking_approach_active(self):
        return (
            not self.first_marking_completed
            and self.c_line_locked
            and self.c_line_bearing is not None
            and bool(self.marking_waypoints)
        )

    def adaptive_terminal_line_guidance(
        self,
        path_bearing,
        target_x,
        target_y,
        target_distance,
    ):
        """Compatibility helper; terminal steering now follows the predictive line."""
        delta_east = self.current_x - target_x
        delta_north = self.current_y - target_y
        signed_cross_track = (
            -math.sin(path_bearing) * delta_east
            + math.cos(path_bearing) * delta_north
        )

        direct_goal_bearing = math.atan2(
            target_y - self.current_y,
            target_x - self.current_x,
        )
        guidance_bearing = self.bounded_bearing(
            path_bearing,
            direct_goal_bearing,
            self.terminal_goal_intercept_bearing_limit,
        )

        return guidance_bearing, signed_cross_track


    def amplify_terminal_moving_error(
        self,
        desired_bearing,
        fallback_direction,
    ):
        error = self.normalize_angle(
            desired_bearing - self.current_yaw
        )
        if abs(error) >= self.terminal_recovery_min_heading_error:
            return desired_bearing, error

        if abs(error) > 1.0e-6:
            direction = 1.0 if error > 0.0 else -1.0
        else:
            direction = fallback_direction

        error = (
            direction
            * self.terminal_recovery_min_heading_error
        )
        desired_bearing = self.normalize_angle(
            self.current_yaw + error
        )
        return desired_bearing, error

    def is_fresh(self, timestamp, timeout):
        if timestamp is None:
            return False
        age = (
            self.get_clock().now() - timestamp
        ).nanoseconds / 1e9
        return age <= timeout

    def publish_motion_profile_monitor(self, command_speed):
        acceleration_message = Bool()
        acceleration_message.data = bool(
            self.acceleration_active
            and not self.acceleration_complete
        )
        self.acceleration_active_pub.publish(acceleration_message)

        deceleration_message = Bool()
        deceleration_message.data = bool(
            self.deceleration_active
            and not self.deceleration_complete
        )
        self.deceleration_active_pub.publish(deceleration_message)

        self._publish_float64(
            self.acceleration_progress_pub,
            self.acceleration_progress_m,
        )
        self._publish_float64(
            self.deceleration_progress_pub,
            self.deceleration_progress_m,
        )
        self._publish_float64(
            self.deceleration_remaining_pub,
            self.deceleration_remaining_m,
        )
        self._publish_float64(
            self.command_speed_pub,
            command_speed,
        )

    def reset_acceleration_profile(self):
        self.acceleration_active = False
        self.acceleration_complete = False
        self.acceleration_start_x = None
        self.acceleration_start_y = None
        self.acceleration_progress_m = 0.0
        self.acceleration_elapsed_sec = 0.0
        self.acceleration_output_speed = 0.0
        self.acceleration_last_update_time = None
        self.acceleration_jump_warning_emitted = False

    def reset_deceleration_profile(self):
        self.deceleration_active = False
        self.deceleration_complete = False
        self.deceleration_progress_m = 0.0
        self.deceleration_remaining_m = self.deceleration_distance
        self.deceleration_output_speed = self.cruise_speed
        self.deceleration_last_update_time = None
        self.deceleration_jump_warning_emitted = False

    def reset_speed_profiles(self):
        self.reset_acceleration_profile()
        self.reset_deceleration_profile()

    def start_deceleration_profile(self, remaining_to_radius):
        self.deceleration_active = True
        self.deceleration_complete = False
        self.deceleration_progress_m = max(
            0.0,
            self.deceleration_distance - remaining_to_radius,
        )
        self.deceleration_remaining_m = max(
            0.0,
            min(self.deceleration_distance, remaining_to_radius),
        )
        self.deceleration_output_speed = self.cruise_speed
        self.deceleration_last_update_time = self.get_clock().now()
        self.deceleration_jump_warning_emitted = False
        self.get_logger().warn(
            "RPP 500MM DECELERATION STARTED | "
            "0.40 -> 0.15m/s, THEN 0.00m/s AT 30MM RADIUS | "
            "no reverse"
        )

    def start_acceleration_profile(self):
        self.acceleration_active = True
        self.acceleration_complete = False
        self.acceleration_start_x = self.current_x
        self.acceleration_start_y = self.current_y
        self.acceleration_progress_m = 0.0
        self.acceleration_elapsed_sec = 0.0
        self.acceleration_output_speed = 0.0
        self.acceleration_last_update_time = self.get_clock().now()
        self.acceleration_jump_warning_emitted = False
        self.get_logger().warn(
            "RPP 500MM ACCELERATION STARTED | "
            "0.00 -> 0.40m/s | final 0.40 -> 0.12 -> 0 enabled"
        )

    def update_acceleration_progress(self):
        if (
            self.current_x is None
            or self.current_y is None
            or self.acceleration_start_x is None
            or self.acceleration_start_y is None
        ):
            return self.acceleration_progress_m

        measured_progress = math.hypot(
            self.current_x - self.acceleration_start_x,
            self.current_y - self.acceleration_start_y,
        )
        if not math.isfinite(measured_progress):
            return self.acceleration_progress_m

        increase = measured_progress - self.acceleration_progress_m
        if increase > self.acceleration_max_progress_jump:
            if not self.acceleration_jump_warning_emitted:
                self.acceleration_jump_warning_emitted = True
                self.get_logger().error(
                    "ACCELERATION ODOMETRY JUMP REJECTED | "
                    f"increase={increase:.3f}m | "
                    f"limit={self.acceleration_max_progress_jump:.3f}m"
                )
            return self.acceleration_progress_m

        if measured_progress > self.acceleration_progress_m:
            self.acceleration_progress_m = measured_progress
        return self.acceleration_progress_m

    def acceleration_speed_limit(self, requested_speed):
        requested_speed = min(
            max(0.0, requested_speed),
            self.cruise_speed,
            self.MAXIMUM_MOVING_SPEED_MPS,
        )
        if requested_speed <= 1.0e-9:
            return 0.0
        if not self.acceleration_enabled:
            return requested_speed
        if self.acceleration_complete:
            return requested_speed
        if not self.acceleration_active:
            self.start_acceleration_profile()

        now = self.get_clock().now()
        if self.acceleration_last_update_time is None:
            dt = 1.0 / self.CONTROL_HZ
        else:
            dt = (
                now - self.acceleration_last_update_time
            ).nanoseconds / 1e9
            if not math.isfinite(dt) or dt <= 0.0:
                dt = 1.0 / self.CONTROL_HZ
            dt = min(dt, self.acceleration_max_dt_sec)
        self.acceleration_last_update_time = now
        self.acceleration_elapsed_sec += dt

        progress = self.update_acceleration_progress()

        # Bootstrap rises smoothly from zero and is capped at 0.15 m/s.
        # This avoids a deadlock when the Sabertooth/PWM drivetrain does not
        # physically move at extremely small velocity setpoints.
        bootstrap_speed = min(
            self.acceleration_startup_ceiling,
            self.acceleration_rate
            * self.acceleration_elapsed_sec,
        )

        # Once movement is measurable, the constant-acceleration relation
        # v^2 = 2*a*s ties the ramp to actual odometry distance. It reaches
        # 0.40 m/s at 0.50 m.
        distance_limited_speed = math.sqrt(
            max(0.0, 2.0 * self.acceleration_rate * progress)
        )
        profile_target = min(
            requested_speed,
            max(bootstrap_speed, distance_limited_speed),
        )

        # Allow 50% timing margin while still rejecting an abrupt command
        # jump caused by an odometry discontinuity or delayed callback.
        maximum_next_speed = (
            self.acceleration_output_speed
            + 1.5 * self.acceleration_rate * dt
        )
        self.acceleration_output_speed = min(
            requested_speed,
            profile_target,
            maximum_next_speed,
        )

        if (
            progress >= self.acceleration_distance
            and self.acceleration_output_speed
            >= requested_speed - 1.0e-6
        ):
            self.acceleration_output_speed = requested_speed
            self.acceleration_complete = True
            self.acceleration_active = False
            self.get_logger().warn(
                "RPP 500MM ACCELERATION COMPLETE | "
                f"distance={progress:.3f}m | "
                f"time={self.acceleration_elapsed_sec:.2f}s | "
                f"speed={requested_speed:.3f}m/s"
            )

        return self.acceleration_output_speed

    def deceleration_speed_limit(
        self,
        requested_speed,
        goal_distance,
    ):
        requested_speed = min(
            max(0.0, requested_speed),
            self.cruise_speed,
            self.MAXIMUM_MOVING_SPEED_MPS,
        )
        if requested_speed <= 1.0e-9:
            return 0.0
        if not self.deceleration_enabled:
            return requested_speed
        if goal_distance is None or not math.isfinite(goal_distance):
            self.get_logger().error(
                "DECELERATION DISABLED FOR CYCLE: invalid goal distance"
            )
            return requested_speed

        remaining_to_radius = max(
            0.0,
            goal_distance - self.waypoint_tolerance,
        )

        if (
            not self.deceleration_active
            and remaining_to_radius > self.deceleration_distance
        ):
            self.deceleration_remaining_m = self.deceleration_distance
            return requested_speed

        if not self.deceleration_active:
            self.start_deceleration_profile(remaining_to_radius)

        now = self.get_clock().now()
        if self.deceleration_last_update_time is None:
            dt = 1.0 / self.CONTROL_HZ
        else:
            dt = (
                now - self.deceleration_last_update_time
            ).nanoseconds / 1e9
            if not math.isfinite(dt) or dt <= 0.0:
                dt = 1.0 / self.CONTROL_HZ
            dt = min(dt, self.deceleration_max_dt_sec)
        self.deceleration_last_update_time = now

        measured_progress = max(
            0.0,
            self.deceleration_distance - remaining_to_radius,
        )
        progress_increase = (
            measured_progress - self.deceleration_progress_m
        )
        if progress_increase > self.deceleration_max_progress_jump:
            if not self.deceleration_jump_warning_emitted:
                self.deceleration_jump_warning_emitted = True
                self.get_logger().error(
                    "DECELERATION DISTANCE JUMP REJECTED | "
                    f"increase={progress_increase:.3f}m | "
                    f"limit={self.deceleration_max_progress_jump:.3f}m"
                )
            measured_progress = self.deceleration_progress_m
        elif measured_progress > self.deceleration_progress_m:
            self.deceleration_progress_m = measured_progress

        self.deceleration_remaining_m = max(
            0.0,
            self.deceleration_distance
            - self.deceleration_progress_m,
        )

        # Constant-deceleration envelope with a non-zero approach floor:
        #
        #   v^2 = v_floor^2 + 2*a*s
        #
        # where s is distance remaining to the 30 mm radial boundary.
        # This gives 0.40 m/s at 500 mm remaining and 0.15 m/s immediately
        # before the boundary. Exact boundary entry separately commands zero.
        profile_target = math.sqrt(
            max(
                self.deceleration_floor_speed
                * self.deceleration_floor_speed,
                (
                    self.deceleration_floor_speed
                    * self.deceleration_floor_speed
                    + 2.0
                    * self.deceleration_rate
                    * self.deceleration_remaining_m
                ),
            )
        )

        # Rate hardening prevents a single delayed callback or noisy distance
        # sample from causing an abrupt non-safety speed drop. Exact-radius
        # entry still bypasses this limiter and commands immediate zero.
        minimum_next_speed = max(
            self.deceleration_floor_speed,
            self.deceleration_output_speed
            - 1.5 * self.deceleration_rate * dt,
        )
        deceleration_ceiling = max(
            profile_target,
            minimum_next_speed,
        )
        self.deceleration_output_speed = min(
            requested_speed,
            deceleration_ceiling,
        )

        if self.deceleration_remaining_m <= 1.0e-6:
            self.deceleration_output_speed = 0.0
            self.deceleration_complete = True
            self.deceleration_active = False
            self.get_logger().warn(
                "RPP FINAL STOP COMPLETE | "
                "0.15 -> 0.00m/s AT 30MM RADIUS"
            )

        return self.deceleration_output_speed

    def publish_velocity_ned(
        self,
        north,
        east,
        *,
        apply_acceleration=True,
        apply_deceleration=False,
        goal_distance=None,
    ):
        if not all(math.isfinite(value) for value in (north, east)):
            self.get_logger().error(
                "Rejected non-finite RPP velocity command"
            )
            north = 0.0
            east = 0.0

        raw_speed = math.hypot(north, east)
        if raw_speed > 1.0e-9:
            requested_speed = min(
                raw_speed,
                self.MAXIMUM_MOVING_SPEED_MPS,
            )
            direction_north = north / raw_speed
            direction_east = east / raw_speed

            if apply_acceleration:
                output_speed = self.acceleration_speed_limit(
                    requested_speed
                )
            else:
                self.reset_acceleration_profile()
                output_speed = requested_speed

            if apply_deceleration:
                output_speed = self.deceleration_speed_limit(
                    output_speed,
                    goal_distance,
                )
            else:
                self.reset_deceleration_profile()

            north = direction_north * output_speed
            east = direction_east * output_speed
        else:
            self.reset_speed_profiles()
            output_speed = 0.0
            north = 0.0
            east = 0.0

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map_ned"
        msg.vector.x = float(north)
        msg.vector.y = float(east)
        msg.vector.z = 0.0
        self.velocity_pub.publish(msg)
        self.publish_motion_profile_monitor(output_speed)
        return north, east, output_speed

    def publish_stop(self):
        self.publish_velocity_ned(0.0, 0.0)


    @staticmethod
    def _publish_float64(publisher, value):
        message = Float64()
        message.data = float(value)
        publisher.publish(message)

    def publish_mm_monitor(
        self,
        path_bearing,
        goal_x,
        goal_y,
        goal_distance,
    ):
        """Publish exact segment metrics in millimetres.

        xtrack_mm is signed perpendicular error from the fixed segment line.
        goal_distance_mm is radial distance to the exact marking coordinate.
        along_track_remaining_mm is positive before, zero at, and negative
        after the exact goal plane.
        """
        delta_east = self.current_x - goal_x
        delta_north = self.current_y - goal_y

        signed_xtrack = (
            -math.sin(path_bearing) * delta_east
            + math.cos(path_bearing) * delta_north
        )
        along_remaining = self.along_track_remaining(
            path_bearing,
            goal_x,
            goal_y,
        )

        if math.isfinite(self.closest_marking_distance):
            closest_distance = self.closest_marking_distance
        else:
            closest_distance = goal_distance

        xtrack_mm = signed_xtrack * 1000.0
        goal_distance_mm = goal_distance * 1000.0
        along_remaining_mm = along_remaining * 1000.0
        closest_distance_mm = closest_distance * 1000.0

        self._publish_float64(
            self.xtrack_mm_pub,
            xtrack_mm,
        )
        self._publish_float64(
            self.goal_distance_mm_pub,
            goal_distance_mm,
        )
        self._publish_float64(
            self.along_remaining_mm_pub,
            along_remaining_mm,
        )
        self._publish_float64(
            self.closest_distance_mm_pub,
            closest_distance_mm,
        )

        now = self.get_clock().now()
        if (
            now - self.last_mm_monitor_log_time
        ).nanoseconds >= 500_000_000:
            self.last_mm_monitor_log_time = now
            self.get_logger().info(
                "MM MONITOR | "
                f"xtrack={xtrack_mm:+.1f}mm | "
                f"goal_distance={goal_distance_mm:.1f}mm | "
                f"along_remaining={along_remaining_mm:+.1f}mm | "
                f"closest={closest_distance_mm:.1f}mm"
            )

    def log_waiting(self, reason):
        now = self.get_clock().now()
        if (
            now - self.last_wait_log_time
        ).nanoseconds < 1_000_000_000:
            return
        self.last_wait_log_time = now
        self.get_logger().info(f"WAITING: {reason}")

    def log_control(
        self,
        status,
        target_distance,
        goal_distance,
        heading_error,
        speed,
        north,
        east,
        yaw_rate_enu=0.0,
    ):
        now = self.get_clock().now()
        if (
            now - self.last_log_time
        ).nanoseconds < 1_000_000_000:
            return
        self.last_log_time = now
        self.get_logger().info(
            f"{status} | "
            f"target_dist={target_distance:.3f}m | "
            f"goal_dist={goal_distance:.3f}m | "
            f"heading_error={math.degrees(heading_error):.1f}deg | "
            f"speed={speed:.3f} | "
            f"vN={north:.3f} | vE={east:.3f} | "
            f"yawRateENU={yaw_rate_enu:.3f}"
        )

    def goal_distance_and_bearing(self):
        if (
            self.segment_goal_x is None
            or self.segment_goal_y is None
            or self.current_x is None
            or self.current_y is None
        ):
            return math.inf, None

        delta_east = self.segment_goal_x - self.current_x
        delta_north = self.segment_goal_y - self.current_y
        distance = math.hypot(delta_east, delta_north)
        bearing = math.atan2(delta_north, delta_east)
        return distance, bearing

    def reset_xtrack_damping_state(self):
        self.last_xtrack_sample = None
        self.last_xtrack_sample_time = None
        self.filtered_xtrack_rate = 0.0
        self.last_xtrack_correction = 0.0
        self.last_xtrack_correction_time = None

    def xtrack_priority_guidance(
        self,
        path_bearing,
        line_point_x,
        line_point_y,
        *,
        terminal_mode=False,
    ):
        """Predictive, filtered and slew-limited fixed-line recovery.

        Global travel retains the conservative 40 mm crossing band.
        The final 1.00 m uses a dedicated profile that keeps predicting and
        counter-steering until the predicted crossing error is within 5 mm.
        """
        delta_east = self.current_x - line_point_x
        delta_north = self.current_y - line_point_y

        signed_cross_track = (
            -math.sin(path_bearing) * delta_east
            + math.cos(path_bearing) * delta_north
        )

        now = self.get_clock().now()
        sample_dt = 1.0 / self.CONTROL_HZ

        if (
            self.last_xtrack_sample is not None
            and self.last_xtrack_sample_time is not None
        ):
            sample_dt = max(
                1.0 / self.CONTROL_HZ,
                (
                    now - self.last_xtrack_sample_time
                ).nanoseconds / 1e9,
            )
            raw_rate = (
                signed_cross_track - self.last_xtrack_sample
            ) / sample_dt
            alpha = self.xtrack_rate_filter_alpha
            self.filtered_xtrack_rate = (
                alpha * raw_rate
                + (1.0 - alpha) * self.filtered_xtrack_rate
            )

        xtrack_rate = self.filtered_xtrack_rate
        self.last_xtrack_sample = signed_cross_track
        self.last_xtrack_sample_time = now

        moving_away = False
        crossing_imminent = False
        crossing_projection = signed_cross_track
        profile_name = "GLOBAL"

        if terminal_mode:
            prediction_time = self.terminal_xtrack_prediction_time_sec
            neutral_band = self.terminal_xtrack_neutral_crossing_band

            moving_away = (
                signed_cross_track * xtrack_rate > 0.0
                and abs(xtrack_rate)
                >= self.terminal_xtrack_away_rate_threshold
                and abs(signed_cross_track) > neutral_band
            )

            crossing_projection = (
                signed_cross_track
                + xtrack_rate
                * self.terminal_xtrack_crossing_prediction_time_sec
            )
            moving_toward_line = (
                signed_cross_track * xtrack_rate < 0.0
            )
            projected_to_cross = (
                signed_cross_track * crossing_projection <= 0.0
                or abs(crossing_projection)
                >= self.terminal_xtrack_crossing_predicted_threshold
                and (
                    signed_cross_track * crossing_projection < 0.0
                )
            )
            crossing_imminent = (
                not moving_away
                and moving_toward_line
                and abs(xtrack_rate)
                >= self.terminal_xtrack_crossing_rate_threshold
                and projected_to_cross
            )

            if moving_away:
                prediction_time = (
                    self.terminal_xtrack_prediction_time_sec
                )
                lookahead = self.terminal_xtrack_away_lookahead
                correction_limit = (
                    self.terminal_xtrack_away_correction_limit
                )
                profile_name = "TERMINAL_AWAY_BOOST"
            elif crossing_imminent:
                prediction_time = (
                    self.terminal_xtrack_crossing_prediction_time_sec
                )
                lookahead = self.terminal_xtrack_crossing_lookahead
                correction_limit = (
                    self.terminal_xtrack_crossing_correction_limit
                )
                profile_name = "TERMINAL_CROSSING_BRAKE"
            else:
                lookahead = self.terminal_xtrack_lookahead
                correction_limit = self.terminal_xtrack_correction_limit
                profile_name = "TERMINAL_CAPTURE"

            correction_slew_rate = (
                self.terminal_xtrack_correction_slew_rate
            )
            hard_correction_limit = (
                self.terminal_xtrack_away_correction_limit
            )
        else:
            prediction_time = self.xtrack_prediction_time_sec
            lookahead = self.xtrack_priority_lookahead
            correction_limit = self.xtrack_priority_correction_limit
            neutral_band = self.xtrack_neutral_crossing_band
            correction_slew_rate = self.xtrack_correction_slew_rate
            hard_correction_limit = correction_limit

        predicted_cross_track = (
            signed_cross_track
            + xtrack_rate * prediction_time
        )

        moving_toward_line = (
            signed_cross_track * xtrack_rate < 0.0
        )
        if (
            abs(signed_cross_track) <= neutral_band
            and abs(predicted_cross_track) <= neutral_band
            and moving_toward_line
        ):
            predicted_cross_track = 0.0

        desired_correction = -math.atan2(
            predicted_cross_track,
            lookahead,
        )
        desired_correction = max(
            -correction_limit,
            min(
                correction_limit,
                desired_correction,
            ),
        )

        correction_dt = sample_dt
        if self.last_xtrack_correction_time is not None:
            correction_dt = max(
                1.0 / self.CONTROL_HZ,
                (
                    now - self.last_xtrack_correction_time
                ).nanoseconds / 1e9,
            )

        desired_reverses_sign = (
            desired_correction * self.last_xtrack_correction < 0.0
        )
        desired_reduces_magnitude = (
            abs(desired_correction)
            < abs(self.last_xtrack_correction)
        )

        if (
            terminal_mode
            and (
                desired_reverses_sign
                or desired_reduces_magnitude
            )
        ):
            active_slew_rate = (
                self.terminal_xtrack_unwind_slew_rate
            )
        else:
            active_slew_rate = correction_slew_rate

        max_change = active_slew_rate * correction_dt
        correction_delta = self.normalize_angle(
            desired_correction - self.last_xtrack_correction
        )
        correction_delta = max(
            -max_change,
            min(max_change, correction_delta),
        )
        correction = (
            self.last_xtrack_correction + correction_delta
        )
        correction = max(
            -hard_correction_limit,
            min(
                hard_correction_limit,
                correction,
            ),
        )

        self.last_xtrack_correction = correction
        self.last_xtrack_correction_time = now

        return (
            self.normalize_angle(path_bearing + correction),
            signed_cross_track,
            xtrack_rate,
            predicted_cross_track,
            correction,
            moving_away,
            crossing_imminent,
            crossing_projection,
            profile_name,
            lookahead,
            correction_limit,
            active_slew_rate,
        )


    @staticmethod
    def interpolate_profile_section(
        distance,
        low_distance,
        low_speed,
        high_distance,
        high_speed,
    ):
        """Compatibility helper: this build never interpolates speed."""
        return 0.40

    def speed_for_goal_distance(self, goal_distance):
        """Return cruise request; publish_velocity_ned applies the envelope."""
        return self.cruise_speed

    def terminal_speed_for_goal_distance(self, goal_distance):
        """Return cruise request; final 500 mm deceleration is applied later."""
        return self.cruise_speed

    def apply_heading_speed_limit(self, base_speed, heading_error):
        """Heading changes direction only, never speed magnitude."""
        return self.cruise_speed


    def limit_moving_guidance_bearing(self, desired_bearing):
        """Keep moving recovery below the PX4 45-degree pivot threshold."""
        command_error = self.normalize_angle(
            desired_bearing - self.current_yaw
        )
        command_error = max(
            -self.MAX_MOVING_HEADING_ERROR_RAD,
            min(
                self.MAX_MOVING_HEADING_ERROR_RAD,
                command_error,
            ),
        )
        limited_bearing = self.normalize_angle(
            self.current_yaw + command_error
        )
        return limited_bearing, command_error

    def bounded_bearing(self, base_bearing, desired_bearing, limit):
        correction = self.normalize_angle(
            desired_bearing - base_bearing
        )
        correction = max(-limit, min(limit, correction))
        return self.normalize_angle(base_bearing + correction)

    def line_guidance(
        self,
        line_bearing,
        line_point_x,
        line_point_y,
        correction_limit,
    ):
        """
        Follow the infinite straight line through the active generated point.
        All 50 mm points in one segment share this bearing, so advancing the
        lookahead target does not create a heading discontinuity.
        """
        delta_east = self.current_x - line_point_x
        delta_north = self.current_y - line_point_y

        signed_cross_track = (
            -math.sin(line_bearing) * delta_east
            + math.cos(line_bearing) * delta_north
        )

        correction = -math.atan2(
            signed_cross_track,
            self.line_tracking_lookahead,
        )
        correction = max(
            -correction_limit,
            min(correction_limit, correction),
        )

        guidance_bearing = self.normalize_angle(
            line_bearing + correction
        )
        return guidance_bearing, signed_cross_track

    def along_track_remaining(
        self,
        path_bearing,
        target_x,
        target_y,
    ):
        """Return signed remaining distance along the incoming segment."""
        delta_east = target_x - self.current_x
        delta_north = target_y - self.current_y

        return (
            math.cos(path_bearing) * delta_east
            + math.sin(path_bearing) * delta_north
        )

    def apply_alignment_release_ramp(self, requested_speed):
        """Alignment release does not ramp speed in this test build."""
        return self.cruise_speed


    def latch_exact_marking_stop(
        self,
        target_distance,
        signed_cross_track,
        along_remaining,
    ):
        """Latch zero only after entering the exact radial waypoint circle."""
        if self.marking_stop_latched:
            return True

        if target_distance <= self.waypoint_tolerance:
            self.marking_stop_latched = True
            self.marking_stop_latched_at = self.get_clock().now()
            self.marking_stop_trigger_radius = self.waypoint_tolerance
            self.closest_marking_distance = min(
                self.closest_marking_distance,
                target_distance,
            )
            self.reset_speed_profiles()
            self.get_logger().warn(
                "EXACT WP_RADIUS ENTERED / ZERO LATCHED | "
                f"radial={target_distance * 1000.0:.1f}mm | "
                f"xtrack={signed_cross_track * 1000.0:+.1f}mm | "
                f"along={along_remaining * 1000.0:+.1f}mm"
            )

        return self.marking_stop_latched

    def evaluate_latched_marking_stop(self, target_distance):
        self.publish_stop()

        if self.marking_stop_latched_at is None:
            return

        elapsed = (
            self.get_clock().now() - self.marking_stop_latched_at
        ).nanoseconds / 1e9

        if (
            elapsed >= self.marking_stop_settle_timeout_sec
            and self.current_speed_mps
            <= self.stationary_speed_tolerance
            and target_distance > self.waypoint_tolerance
        ):
            self.marking_missed = True
            self.get_logger().error(
                "EXACT WP_RADIUS STOP DRIFTED OUTSIDE 30MM / SAFE HOLD | "
                f"distance={target_distance:.3f}m | "
                f"speed={self.current_speed_mps:.3f}m/s"
            )

    def control_loop(self):
        if self.emergency_stop:
            self.publish_stop()
            self.log_waiting("emergency stop active")
            return
        if not self.mission_enabled:
            self.publish_stop()
            self.log_waiting("mission disabled")
            return
        if self.marking_active:
            self.publish_stop()
            self.log_waiting("continuous marking hold")
            return
        if not self.marking_metadata_received:
            self.publish_stop()
            self.log_waiting("waiting for marking metadata")
            return
        if (
            self.current_x is None
            or self.current_y is None
            or self.current_yaw is None
        ):
            self.publish_stop()
            self.log_waiting("waiting for odometry")
            return
        if not self.is_fresh(
            self.last_odom_time,
            self.odom_timeout_sec,
        ):
            self.publish_stop()
            self.log_waiting("odometry timeout")
            return
        if self.marking_missed:
            self.publish_stop()
            self.log_waiting("marking capture safe hold active")
            return
        if self.alignment_safety_hold:
            self.publish_stop()
            self.log_waiting("alignment cross-track safe hold active")
            return

        first_approach = self.first_marking_approach_active()
        if (
            not first_approach
            and not self.first_marking_completed
            and self.segment_goal_number == 1
        ):
            if self.lock_c_to_p1_line("control-loop readiness"):
                first_approach = True

        if first_approach:
            p1_x, p1_y = self.marking_waypoints[0]

            # Exact P1 is always the independent marking/stop goal.
            # The active 50 mm point is only a pass-through path target.
            goal_x = p1_x
            goal_y = p1_y
            goal_is_marking = True
            goal_distance = math.hypot(
                goal_x - self.current_x,
                goal_y - self.current_y,
            )
            path_bearing = self.c_line_bearing

            active_target_ready = (
                self.target_x is not None
                and self.target_y is not None
                and self.is_fresh(
                    self.last_waypoint_time,
                    self.waypoint_timeout_sec,
                )
            )

            if active_target_ready:
                target_x = self.target_x
                target_y = self.target_y
                target_is_marking = self.target_is_marking
                target_label = "ACTIVE 50MM INTERPOLATED TARGET"
            else:
                target_x = goal_x
                target_y = goal_y
                target_is_marking = True
                target_label = "P1 FALLBACK TARGET"

            mode_prefix = (
                "C->P1 FIXED C-LINE / "
                + target_label
                + " / "
            )
        else:
            if self.target_x is None or self.target_y is None:
                self.publish_stop()
                self.log_waiting("waiting for active waypoint")
                return
            if not self.is_fresh(
                self.last_waypoint_time,
                self.waypoint_timeout_sec,
            ):
                self.publish_stop()
                self.log_waiting("active waypoint timeout")
                return

            target_x = self.target_x
            target_y = self.target_y
            target_is_marking = self.target_is_marking

            if (
                self.segment_goal_x is not None
                and self.segment_goal_y is not None
            ):
                goal_x = self.segment_goal_x
                goal_y = self.segment_goal_y
                goal_is_marking = (
                    self.segment_goal_number is not None
                )
            else:
                goal_x = target_x
                goal_y = target_y
                goal_is_marking = target_is_marking

            goal_distance = math.hypot(
                goal_x - self.current_x,
                goal_y - self.current_y,
            )
            goal_bearing = math.atan2(
                goal_y - self.current_y,
                goal_x - self.current_x,
            )

            path_bearing = self.target_path_bearing
            if path_bearing is None:
                path_bearing = goal_bearing
            mode_prefix = ""

        if path_bearing is None:
            self.publish_stop()
            self.log_waiting("waiting for fixed segment bearing")
            return

        delta_east = target_x - self.current_x
        delta_north = target_y - self.current_y
        target_distance = math.hypot(
            delta_east,
            delta_north,
        )
        direct_target_bearing = math.atan2(
            delta_north,
            delta_east,
        )
        path_heading_error = self.normalize_angle(
            path_bearing - self.current_yaw
        )

        self.publish_mm_monitor(
            path_bearing,
            goal_x,
            goal_y,
            goal_distance,
        )

        goal_along_remaining = self.along_track_remaining(path_bearing, goal_x, goal_y)
        goal_delta_east = self.current_x - goal_x
        goal_delta_north = self.current_y - goal_y
        goal_signed_cross_track = (
            -math.sin(path_bearing) * goal_delta_east
            + math.cos(path_bearing) * goal_delta_north
        )

        if goal_is_marking and self.marking_stop_latched:
            self.evaluate_latched_marking_stop(goal_distance)
            self.log_control(
                mode_prefix + "EXACT WP_RADIUS 30MM / ZERO LATCH / WAIT 3S HOLD",
                goal_distance,
                goal_distance,
                path_heading_error,
                0.0,
                0.0,
                0.0,
            )
            return

        # Exact marking-goal radial distance is checked independently of
        # whichever 50 mm pass-through point is currently active.
        if (
            goal_is_marking
            and self.latch_exact_marking_stop(
                goal_distance,
                goal_signed_cross_track,
                goal_along_remaining,
            )
        ):
            self.evaluate_latched_marking_stop(goal_distance)
            self.log_control(
                mode_prefix + "EXACT WP_RADIUS 30MM / ZERO LATCH / WAIT 3S HOLD",
                goal_distance,
                goal_distance,
                path_heading_error,
                0.0,
                0.0,
                0.0,
            )
            return

        # Exact goal pass-plane safety remains active on every cycle.
        # It prevents indefinite travel after P1 is crossed.
        if goal_is_marking:
            if (
                not self.capture_monitor_armed
                and goal_distance
                <= self.marking_capture_arm_distance
            ):
                self.capture_monitor_armed = True
                self.closest_marking_distance = goal_distance
                self.get_logger().warn(
                    "EXACT MARKING GOAL MONITOR ARMED | "
                    f"distance={goal_distance * 1000.0:.1f}mm | "
                    f"along_remaining="
                    f"{goal_along_remaining * 1000.0:+.1f}mm"
                )

            if self.capture_monitor_armed:
                self.closest_marking_distance = min(
                    self.closest_marking_distance,
                    goal_distance,
                )

            crossed_goal_plane = (
                goal_along_remaining
                < -self.marking_along_track_abort
            )
            moving_away_after_capture = (
                self.capture_monitor_armed
                and goal_distance
                > self.closest_marking_distance
                + self.miss_margin
            )

            if crossed_goal_plane and (
                moving_away_after_capture
                or not self.capture_monitor_armed
            ):
                if not math.isfinite(self.closest_marking_distance):
                    self.closest_marking_distance = goal_distance
                self.marking_missed = True
                self.reset_terminal_native_pivot()
                self.publish_stop()
                self.get_logger().error(
                    "EXACT MARKING GOAL CROSSED / SAFE HOLD | "
                    f"closest="
                    f"{self.closest_marking_distance * 1000.0:.1f}mm | "
                    f"current={goal_distance * 1000.0:.1f}mm | "
                    f"along_remaining="
                    f"{goal_along_remaining * 1000.0:+.1f}mm"
                )
                return

        # --------------------------------------------------------------
        # PX4 NATIVE SEGMENT ALIGNMENT
        #
        # Always command the fixed segment velocity-vector bearing. PX4
        # pivots at >=45deg and changes to straight drive at <=12deg.
        # There is no 6deg gate, AttitudeTarget, yaw-rate command, or
        # zero-speed alignment hold.
        # --------------------------------------------------------------
        if (
            not self.segment_alignment_active
            and goal_distance > self.terminal_goal_intercept_distance
            and abs(path_heading_error) >= self.pivot_enter_angle
        ):
            self.segment_alignment_active = True
            self.reset_terminal_native_pivot()
            self.get_logger().warn(
                "PX4 NATIVE ALIGNMENT RE-ENTERED | "
                f"path_error={math.degrees(path_heading_error):.1f}deg"
            )

        if (
            self.segment_alignment_active
            and goal_distance > self.terminal_goal_intercept_distance
        ):
            _, alignment_cross_track = self.line_guidance(
                path_bearing,
                goal_x,
                goal_y,
                self.segment_alignment_correction_limit,
            )

            if abs(alignment_cross_track) >= self.segment_alignment_max_cross_track:
                self.alignment_safety_hold = True
                self.publish_stop()
                self.get_logger().error(
                    "SEGMENT ALIGNMENT CROSS-TRACK LIMIT / SAFE HOLD | "
                    f"xtrack={alignment_cross_track:.3f}m | "
                    f"limit={self.segment_alignment_max_cross_track:.3f}m | "
                    f"path_error={math.degrees(path_heading_error):.1f}deg"
                )
                return

            if abs(path_heading_error) <= self.pivot_exit_angle:
                self.segment_alignment_active = False
                self.reset_terminal_native_pivot()

                if first_approach:
                    self.reanchor_c_to_p1_after_pivot()
                    path_bearing = self.c_line_bearing
                    path_heading_error = self.normalize_angle(
                        path_bearing - self.current_yaw
                    )
                    alignment_cross_track = 0.0

                self.get_logger().warn(
                    "PX4 NATIVE PIVOT COMPLETE / 0.40MPS STRAIGHT TRAVEL | "
                    f"path_error={math.degrees(path_heading_error):.1f}deg | "
                    f"xtrack={alignment_cross_track * 1000.0:+.1f}mm"
                )
            else:
                pivot_active, pivot_bearing, true_error = (
                    self.terminal_native_pivot_command(
                        path_bearing,
                        "STRICT-SEGMENT-ENTRY",
                    )
                )
                command_bearing = pivot_bearing if pivot_active else path_bearing
                speed = self.segment_alignment_speed
                north = speed * math.sin(command_bearing)
                east = speed * math.cos(command_bearing)
                north, east, speed = self.publish_velocity_ned(
                    north,
                    east,
                    apply_acceleration=False,
                    apply_deceleration=False,
                )

                if abs(true_error) >= self.pivot_enter_angle:
                    status = (
                        "PX4 NATIVE PIVOT REQUEST >=45DEG / "
                        "FIXED SEGMENT VECTOR 0.40MPS"
                    )
                else:
                    status = (
                        "PX4 TURN-TO-DRIVE TRANSITION 45->12DEG / "
                        "FIXED SEGMENT VECTOR 0.40MPS"
                    )

                self.log_control(
                    mode_prefix + status
                    + f" | bearing={math.degrees(command_bearing):.1f}deg"
                    + f" | true_error={math.degrees(true_error):.1f}deg"
                    + f" | xtrack={alignment_cross_track:.3f}m",
                    target_distance,
                    goal_distance,
                    true_error,
                    speed,
                    north,
                    east,
                )
                return

        terminal_active = (
            goal_is_marking
            and goal_distance
            <= self.terminal_goal_intercept_distance
        )

        if terminal_active and self.xtrack_priority_active:
            self.xtrack_priority_active = False
            self.xtrack_priority_inside_since = None

            # Preserve filtered xtrack-rate and correction history. Clearing
            # it here caused the controller to forget that the rover was
            # already moving laterally across the fixed line.
            self.get_logger().warn(
                "TERMINAL PREDICTIVE LINE CAPTURE TOOK PRIORITY | "
                f"distance={goal_distance * 1000.0:.1f}mm | "
                "xtrack-rate state preserved"
            )

        # --------------------------------------------------------------
        # GLOBAL MOVING CROSS-TRACK RECOVERY
        #
        # Recovery never creates an explicit zero-speed pivot. Desired
        # bearing stays within +/-12deg of the fixed segment and is always
        # sent as a 0.40 m/s velocity vector.
        # --------------------------------------------------------------
        (
            xtrack_guidance_bearing,
            global_signed_cross_track,
            global_xtrack_rate,
            predicted_cross_track,
            applied_xtrack_correction,
            terminal_moving_away,
            terminal_crossing_imminent,
            terminal_crossing_projection,
            xtrack_profile_name,
            active_xtrack_lookahead,
            active_xtrack_correction_limit,
            active_xtrack_slew_rate,
        ) = self.xtrack_priority_guidance(
            path_bearing,
            goal_x,
            goal_y,
            terminal_mode=terminal_active,
        )

        if (
            not terminal_active
            and abs(global_signed_cross_track)
            >= self.xtrack_priority_enter
        ):
            self.xtrack_priority_active = True
            self.xtrack_priority_inside_since = None

        if not terminal_active and self.xtrack_priority_active:
            release_geometry_valid = (
                abs(global_signed_cross_track) <= self.xtrack_priority_exit
                and abs(path_heading_error) <= self.xtrack_priority_release_heading
            )

            if release_geometry_valid:
                if self.xtrack_priority_inside_since is None:
                    self.xtrack_priority_inside_since = self.get_clock().now()
                release_elapsed = (
                    self.get_clock().now() - self.xtrack_priority_inside_since
                ).nanoseconds / 1e9
                if release_elapsed >= self.xtrack_priority_hold_sec:
                    self.xtrack_priority_active = False
                    self.xtrack_priority_inside_since = None
                    self.reset_xtrack_damping_state()
                    self.get_logger().warn(
                        "GLOBAL MOVING XTRACK RECOVERY RELEASED | "
                        f"xtrack={global_signed_cross_track:.3f}m | "
                        f"path_error={math.degrees(path_heading_error):.1f}deg"
                    )
            else:
                self.xtrack_priority_inside_since = None

            if self.xtrack_priority_active:
                (
                    xtrack_guidance_bearing,
                    command_heading_error,
                ) = self.limit_moving_guidance_bearing(
                    xtrack_guidance_bearing
                )
                speed = self.cruise_speed
                north = speed * math.sin(xtrack_guidance_bearing)
                east = speed * math.cos(xtrack_guidance_bearing)
                north, east, speed = self.publish_velocity_ned(
                    north,
                    east,
                )
                self.log_control(
                    mode_prefix
                    + "GLOBAL DAMPED XTRACK RECOVERY / NO EXPLICIT PIVOT / "
                    + "0.40MPS"
                    + f" / correction_limit="
                    + f"{math.degrees(self.xtrack_priority_correction_limit):.1f}deg"
                    + f" | xtrack={global_signed_cross_track * 1000.0:+.1f}mm"
                    + f" | xtrack_rate={global_xtrack_rate * 1000.0:+.1f}mm/s"
                    + f" | predicted={predicted_cross_track * 1000.0:+.1f}mm"
                    + f" | correction="
                    + f"{math.degrees(applied_xtrack_correction):+.1f}deg"
                    + f" | path_error={math.degrees(path_heading_error):.1f}deg"
                    + f" | command_error="
                    + f"{math.degrees(command_heading_error):.1f}deg",
                    target_distance,
                    goal_distance,
                    command_heading_error,
                    speed,
                    north,
                    east,
                )
                return

        # --------------------------------------------------------------
        # Continuous terminal marking corridor.
        #
        # Constant-speed test: every non-zero terminal and
        # PX4 native pivot uses requested velocity-vector bearing; all translation is 0.40 m/s.
        # No ramp, deceleration, creep, pulse, or yaw-rate-only command.
        # --------------------------------------------------------------
        if terminal_active:
            # Continue the predictive fixed-line controller through the P1
            # plane. This acts as a virtual look-through target beyond P1,
            # while exact P1 remains the independent stop/marking coordinate.
            guidance_bearing = xtrack_guidance_bearing
            signed_cross_track = global_signed_cross_track
            terminal_xtrack_rate = global_xtrack_rate
            terminal_predicted_xtrack = predicted_cross_track
            terminal_applied_correction = applied_xtrack_correction

            path_gate_valid = (
                abs(path_heading_error)
                <= self.terminal_capture_gate_heading
            )
            cross_track_gate_valid = (
                abs(signed_cross_track)
                <= self.terminal_capture_gate_cross_track
            )
            gate_geometry_valid = (
                cross_track_gate_valid
                and path_gate_valid
            )

            if gate_geometry_valid:
                if self.terminal_gate_inside_since is None:
                    self.terminal_gate_inside_since = (
                        self.get_clock().now()
                    )
                gate_elapsed = (
                    self.get_clock().now()
                    - self.terminal_gate_inside_since
                ).nanoseconds / 1e9
                self.terminal_gate_ready = (
                    gate_elapsed
                    >= self.terminal_capture_gate_hold_sec
                )
            else:
                self.terminal_gate_inside_since = None
                self.terminal_gate_ready = False

            # Closest-distance safety monitoring is independent of the
            # precision gate. The gate controls final release, not whether
            # overshoot protection is available.
            if (
                not self.capture_monitor_armed
                and goal_distance
                <= self.marking_capture_arm_distance
            ):
                self.capture_monitor_armed = True
                self.closest_marking_distance = goal_distance
                self.get_logger().warn(
                    "MARKING CAPTURE SAFETY MONITOR ARMED | "
                    f"distance={goal_distance:.3f}m | "
                    f"xtrack={signed_cross_track:.3f}m | "
                    f"path_error="
                    f"{math.degrees(path_heading_error):.1f}deg | "
                    f"gate_ready={self.terminal_gate_ready}"
                )

            along_remaining = self.along_track_remaining(
                path_bearing,
                goal_x,
                goal_y,
            )

            # Distance does not pre-empt cross-track recovery in this build.
            # The global 30 mm guard runs before this terminal branch.
            # Exact radial distance remains only for the final zero latch.

            if self.terminal_gate_ready:
                self.reset_terminal_native_pivot()

                guidance_bearing = path_bearing
                heading_error = path_heading_error
                speed = self.terminal_speed_for_goal_distance(goal_distance)

                status = (
                    "MARKING TERMINAL STABLE SPEED PROFILE / "
                    "SYMMETRIC FINAL 500MM DECELERATION / STRICT HEADING+XTRACK GATE"
                )

            else:
                (
                    guidance_bearing,
                    heading_error,
                ) = self.limit_moving_guidance_bearing(
                    guidance_bearing
                )
                speed = self.terminal_speed_for_goal_distance(goal_distance)

                if abs(heading_error) >= self.pivot_enter_angle:
                    transition_status = "PX4 NATIVE PIVOT REQUEST >=45DEG"
                elif abs(heading_error) > self.pivot_exit_angle:
                    transition_status = "PX4 TURN-TO-DRIVE TRANSITION 45->12DEG"
                else:
                    transition_status = "STRAIGHT MOVEMENT <=12DEG"

                status = (
                    "MARKING TERMINAL LINE CAPTURE + 500MM DECELERATION / "
                    + transition_status
                    + " / 0.40MPS | "
                    + f"xtrack={signed_cross_track * 1000.0:+.1f}mm | "
                    + f"xtrack_rate="
                    + f"{terminal_xtrack_rate * 1000.0:+.1f}mm/s | "
                    + f"predicted="
                    + f"{terminal_predicted_xtrack * 1000.0:+.1f}mm | "
                    + f"correction="
                    + f"{math.degrees(terminal_applied_correction):+.1f}deg | "
                    + f"profile={xtrack_profile_name} | "
                    + f"moving_away={terminal_moving_away} | "
                    + f"crossing_imminent="
                    + f"{terminal_crossing_imminent} | "
                    + f"cross_projection="
                    + f"{terminal_crossing_projection * 1000.0:+.1f}mm | "
                    + f"lookahead={active_xtrack_lookahead:.2f}m | "
                    + f"limit="
                    + f"{math.degrees(active_xtrack_correction_limit):.1f}deg | "
                    + f"slew="
                    + f"{math.degrees(active_xtrack_slew_rate):.0f}degps | "
                    + f"prediction={self.terminal_xtrack_prediction_time_sec:.2f}s | "
                    + f"path_error={math.degrees(path_heading_error):.1f}deg"
                )

            if self.capture_monitor_armed:
                self.closest_marking_distance = min(
                    self.closest_marking_distance,
                    goal_distance,
                )
                radial_increased = (
                    goal_distance
                    > self.closest_marking_distance
                    + self.miss_margin
                )
                confirmed_overshoot = (
                    along_remaining
                    < -self.marking_along_track_abort
                )
                if radial_increased and confirmed_overshoot:
                    self.marking_missed = True
                    self.reset_terminal_native_pivot()
                    self.publish_stop()
                    self.get_logger().error(
                        "MARKING CAPTURE MISSED / SAFE HOLD | "
                        f"closest="
                        f"{self.closest_marking_distance:.3f}m | "
                        f"current={goal_distance:.3f}m | "
                        f"xtrack={signed_cross_track:.3f}m | "
                        f"along_remaining={along_remaining:.3f}m"
                    )
                    return

            north = speed * math.sin(guidance_bearing)
            east = speed * math.cos(guidance_bearing)
            north, east, speed = self.publish_velocity_ned(
                north,
                east,
                apply_deceleration=goal_is_marking,
                goal_distance=goal_distance,
            )
            self.log_control(
                mode_prefix + status,
                goal_distance,
                goal_distance,
                heading_error,
                speed,
                north,
                east,
            )
            return

        # --------------------------------------------------------------
        # Normal pass-through and non-terminal movement.
        # --------------------------------------------------------------
        (
            guidance_bearing,
            signed_cross_track,
        ) = self.line_guidance(
            path_bearing,
            goal_x,
            goal_y,
            self.path_correction_limit,
        )
        heading_error = self.normalize_angle(
            guidance_bearing - self.current_yaw
        )

        speed = self.cruise_speed

        if first_approach:
            status = (
                "C->P1 FIXED C-LINE / ACTIVE 50MM PATH / "
                "RPP ACCEL/DECEL 500MM SYMMETRIC | "
                f"xtrack={signed_cross_track:.3f}m"
            )
        else:
            status = (
                "INTERPOLATED 50MM STRAIGHT PATH / "
                "RPP ACCEL/DECEL 500MM SYMMETRIC | "
                f"xtrack={signed_cross_track:.3f}m"
            )

        north = speed * math.sin(guidance_bearing)
        east = speed * math.cos(guidance_bearing)
        north, east, speed = self.publish_velocity_ned(
            north,
            east,
            apply_deceleration=goal_is_marking,
            goal_distance=goal_distance,
        )
        self.log_control(
            status,
            target_distance,
            goal_distance,
            heading_error,
            speed,
            north,
            east,
        )


def main(args=None):
    rclpy.init(args=args)
    node = RPPController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()