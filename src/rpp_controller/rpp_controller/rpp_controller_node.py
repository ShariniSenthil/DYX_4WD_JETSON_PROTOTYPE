#!/usr/bin/env python3

import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float64, Int32MultiArray, String, UInt8MultiArray
from tf_transformations import euler_from_quaternion

from rpp_controller.point_event_policy import (
    first_marking_approach_is_active,
    latched_stop_terminal_outcome,
    should_release_first_marking,
)
from rpp_controller.runtime_entry import (
    build_runtime_entry_path,
    track_runtime_entry_path,
)
from rpp_controller.path_geometry import (
    GeometryProgressTracker,
    GeometryResetReason,
    PathGeometryIndex,
    POINT_TYPE_MARKING,
    make_path_signature,
    validate_goal_metadata,
)
from rpp_controller.guidance import (
    GuidanceConfig,
    compute_precision_guidance,
)
from rpp_controller.feature_gates import (
    PRECISION_FEATURE_GATES,
    geometry_processing_requested,
    validate_precision_feature_gates,
)
from rpp_controller.speed_regulator import (
    LongitudinalRegulator,
    LongitudinalRegulatorConfig,
    SpeedRegulatorInput,
)
from rpp_controller.legacy_alignment import (
    LegacyAlignmentConfig,
    LegacyAlignmentDirective,
    LegacyAlignmentInput,
    LegacyAlignmentLifecycle,
    LegacyAlignmentPhase,
)
from rpp_controller.motion_state_machine import (
    MotionDirective,
    MotionState,
    PivotMotionConfig,
    PivotMotionInput,
    VerifiedPivotStateMachine,
)
from rpp_controller.tracking_control import (
    TrackingControlConfig,
    TrackingControlInput,
    TrackingControlState,
    TrackingMetricsAccumulator,
    TrackingStabilityController,
)
from rpp_controller.terminal_certificate import (
    ControllerPose,
    TerminalConfig,
    TerminalDirective,
    TerminalInput,
    TerminalState,
    TerminalStopStateMachine,
)
from rpp_controller.terminal_stop_regulator import (
    MotionDirection as RadialStopMotionDirection,
    RadialStopConfig,
    RadialStopInput,
    RadialStopState,
    TerminalStopRegulator,
)

# Parameters rclpy or the launch system may legitimately set at runtime and
# from which this node derives nothing. Everything else is restart-only --
# see _on_set_precision_feature_gates for why refusing beats silently
# ignoring.
_RUNTIME_SETTABLE_EXEMPT = frozenset({"use_sim_time"}) | frozenset(
    PRECISION_FEATURE_GATES
)


class RPPController(Node):
    """Precision waypoint marking controller.

    Active motion contract:
      - Cruise speed is fixed at 1.00 m/s outside the terminal stop profile.
      - Every translational start accelerates from zero over 0.20 m to the
        selected fixed cruise speed.
      - Subscribe to trajectory_generator /nav_path and follow its retained
        50 mm interpolated points. /segment_goal is semantic metadata only.
      - Use a nav-path lookahead target while keeping cross-track guidance on
        the local path tangent before and during slowdown.
      - Never stop merely because a terminal alignment gate was not met.
      - Keep small steering correction available all the way to the waypoint.
      - Decelerate for every SEMANTIC STOP GOAL over the final 0.50 m:
        original marking points P1/P2/P3/... AND extension/dummy points.
        At 1.00 m/s cruise the profile is 1.00 -> 0.15 m/s.
      - Pass-through/interpolation points never activate terminal deceleration.
      - Command exact zero when radial distance to the semantic stop goal is
        <= waypoint_tolerance_m (30 mm in rover.launch.py).
      - If a semantic stop goal is crossed without entering the 30 mm circle,
        stop in safe hold; do not reverse automatically.

    Node name, executable, topics, and tablet launch contract are unchanged.
    """

    CONTROL_HZ = 20.0
    TELEMETRY_HZ = 50.0
    MAXIMUM_MOVING_SPEED_MPS = 1.00
    MAX_MOVING_HEADING_ERROR_RAD = math.radians(30.0)
    WAYPOINT_CHANGE_EPSILON_M = 0.001
    RUNTIME_ENTRY_SPACING_M = 0.05

    def __init__(self):
        super().__init__("rpp_controller")

        self.declare_parameter("local_frame", "map")
        self.declare_parameter("cruise_speed_mps", 1.00)

        # Distance-based acceleration profile. It is used at mission start
        # and re-armed after every completed marking before the next leg.
        self.declare_parameter("acceleration_enabled", True)
        self.declare_parameter("acceleration_distance_m", 0.20)
        self.declare_parameter("acceleration_startup_ceiling_mps", 0.15)
        self.declare_parameter("acceleration_max_progress_jump_m", 0.10)
        self.declare_parameter("acceleration_max_dt_sec", 0.10)

        # Global non-zero slew limits provide a second safety envelope.
        # Literal safety stops remain immediate and are never rate-limited.
        self.declare_parameter("command_speed_rise_limit_mps2", 3.00)
        self.declare_parameter("command_speed_fall_limit_mps2", 2.00)

        # RPP-owned semantic-stop deceleration profile.
        # Deceleration starts exactly 0.50 m before each ORIGINAL marking OR
        # extension/dummy coordinate and reaches 0.15 m/s at the 30 mm boundary.
        # Pass-through/interpolation points do not activate this profile.
        self.declare_parameter("deceleration_enabled", True)
        self.declare_parameter("deceleration_distance_m", 0.50)
        self.declare_parameter(
            "deceleration_floor_speed_mps",
            0.15,
        )
        self.declare_parameter("deceleration_max_progress_jump_m", 0.10)
        self.declare_parameter("deceleration_max_dt_sec", 0.10)

        # Uneven-surface terminal control. The speed profile is staged by
        # along-track distance and zero is still controlled only by the exact
        # radial waypoint tolerance.
        self.declare_parameter("terminal_decel_correction_limit_deg", 3.0)
        # Reverted 2026-09-02: the 4.5deg value this held briefly was never
        # grounded in a bag-based diagnosis (P1->P2 cross-track root cause is
        # still unconfirmed -- see CLAUDE.md). Back to the known production
        # baseline pending that diagnosis.
        self.declare_parameter("terminal_near_correction_limit_deg", 1.0)
        self.declare_parameter("terminal_near_correction_start_distance_m", 0.15)
        self.declare_parameter("terminal_bearing_freeze_distance_m", 0.06)
        self.declare_parameter("terminal_correction_slew_rate_degps", 8.0)
        self.declare_parameter("terminal_frozen_xtrack_abort_m", 0.035)

        self.declare_parameter("minimum_speed_mps", 0.04)
        self.declare_parameter("segment_alignment_speed_mps", 1.00)
        self.declare_parameter(
            "segment_alignment_recovery_speed_mps",
            1.00,
        )
        self.declare_parameter(
            "segment_alignment_deadband_enter_cross_track_m",
            0.12,
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
            0.12,
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
            1.00,
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
            4.0,
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
        self.declare_parameter("slow_distance_m", 1.00)
        self.declare_parameter(
            "decel_profile_distance_1_m",
            0.80,
        )
        self.declare_parameter(
            "decel_profile_speed_1_mps",
            1.00,
        )
        self.declare_parameter(
            "decel_profile_distance_2_m",
            0.40,
        )
        self.declare_parameter(
            "decel_profile_speed_2_mps",
            1.00,
        )
        self.declare_parameter(
            "decel_profile_distance_3_m",
            0.15,
        )
        self.declare_parameter(
            "decel_profile_speed_3_mps",
            1.00,
        )
        self.declare_parameter("final_speed_distance_m", 0.10)
        self.declare_parameter("waypoint_tolerance_m", 0.03)

        self.declare_parameter("pivot_enter_angle_deg", 45.0)
        self.declare_parameter("pivot_exit_angle_deg", 12.0)
        self.declare_parameter("alignment_hold_sec", 0.20)
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
            0.10,
        )
        self.declare_parameter(
            "line_tracking_lookahead_m",
            0.55,
        )
        # Speed- and cross-track-adaptive lookahead for line_guidance().
        # A fixed lookahead means the correction reacts far ahead in TIME at
        # low speed (e.g. mid-accel-ramp) and close in time at cruise --
        # inconsistent dynamic response across the speed range. Scaling with
        # commanded speed keeps look-ahead TIME roughly constant instead
        # (line_tracking_lookahead_m / cruise_speed_mps is used as that time
        # gain, so behaviour at cruise is unchanged from today's tuning).
        # The xtrack term widens the lookahead on large deviations so
        # re-acquisition is a softer curve instead of saturating straight
        # into the atan2 correction-limit clamp.
        self.declare_parameter(
            "line_tracking_lookahead_min_m",
            0.35,
        )
        self.declare_parameter(
            "line_tracking_lookahead_max_m",
            0.80,
        )
        self.declare_parameter(
            "line_tracking_lookahead_xtrack_gain",
            1.0,
        )
        # /nav_path is generated at 50 mm spacing. The cursor advances through
        # those points without stopping; a farther point is selected only as
        # the RPP lookahead reference. Semantic /segment_goal still owns the
        # 500 mm deceleration and exact 30 mm zero capture.
        self.declare_parameter("nav_path_lookahead_m", 0.55)
        self.declare_parameter("nav_path_point_reach_m", 0.075)
        self.declare_parameter(
            "alignment_release_accel_distance_m",
            0.30,
        )
        self.declare_parameter("heading_full_speed_deg", 2.0)
        self.declare_parameter("heading_min_speed_deg", 4.0)

        self.declare_parameter(
            "marking_terminal_speed_start_distance_m",
            0.10,
        )
        self.declare_parameter(
            "marking_terminal_max_speed_mps",
            0.15,
        )
        self.declare_parameter(
            "marking_final_creep_start_distance_m",
            0.06,
        )
        self.declare_parameter(
            "marking_final_creep_speed_mps",
            0.15,
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
            0.06,
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
            4.0,
        )
        self.declare_parameter(
            "terminal_native_pivot_request_error_deg",
            60.0,
        )
        self.declare_parameter(
            "segment_pivot_keeper_timeout_sec",
            10.0,
        )
        self.declare_parameter(
            "segment_fast_capture_max_cross_track_m",
            0.05,
        )
        self.declare_parameter(
            "terminal_close_recovery_distance_m",
            0.08,
        )
        self.declare_parameter(
            "terminal_close_recovery_speed_mps",
            0.15,
        )
        self.declare_parameter(
            "terminal_unready_hold_along_m",
            0.70,
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

        # Gate-1 geometry was accepted after four forward/reverse field runs.
        # It is now installed by default while the legacy cursor remains the
        # motion authority until the independently gated Phase-2 controls run.
        self.declare_parameter("geometry_tracking_enabled", True)
        self.declare_parameter("geometry_diagnostics_enabled", False)
        self.declare_parameter("geometry_corner_threshold_deg", 45.0)
        self.declare_parameter("geometry_projection_back_window_segments", 2)
        self.declare_parameter("geometry_projection_forward_window_segments", 4)
        self.declare_parameter("geometry_projection_reacquire_distance_m", 0.30)
        self.declare_parameter("geometry_localization_jump_reset_m", 0.50)
        self.declare_parameter("geometry_max_backward_jump_m", 0.10)
        self.declare_parameter("geometry_max_forward_jump_m", 1.00)

        # Projection guidance is field-accepted and enabled by default with
        # the tested 0.90 s horizon. Longitudinal speed control remains an
        # independent default-OFF authority.
        self.declare_parameter("precision_guidance_enabled", True)
        self.declare_parameter("precision_speed_control_enabled", False)
        self.declare_parameter("precision_lookahead_min_m", 0.20)
        self.declare_parameter("precision_lookahead_max_m", 1.00)
        self.declare_parameter("precision_lookahead_time_s", 0.90)
        self.declare_parameter("precision_xtrack_lookahead_gain", 0.0)
        self.declare_parameter("precision_moving_bearing_cone_deg", 30.0)

        self.declare_parameter("precision_hardware_speed_ceiling_mps", 1.00)
        self.declare_parameter("precision_acceleration_mps2", 0.75)
        self.declare_parameter("precision_deceleration_mps2", 0.75)
        self.declare_parameter("precision_launch_speed_mps", 0.10)
        self.declare_parameter("precision_control_dt_max_sec", 0.10)
        self.declare_parameter("precision_heading_accel_full_error_deg", 2.0)
        self.declare_parameter("precision_heading_recovery_start_deg", 4.0)
        self.declare_parameter("precision_heading_recovery_full_deg", 15.0)
        self.declare_parameter("precision_xtrack_accel_full_m", 0.010)
        self.declare_parameter("precision_xtrack_recovery_start_m", 0.020)
        self.declare_parameter("precision_xtrack_recovery_full_m", 0.100)
        self.declare_parameter("precision_recovery_min_speed_mps", 0.15)
        self.declare_parameter("precision_corner_angle_threshold_deg", 45.0)
        self.declare_parameter("precision_corner_target_speed_mps", 0.12)
        self.declare_parameter("precision_corner_accel_block_buffer_m", 0.10)
        # Phase 2 must never create an early terminal zero.  The existing
        # radial 30 mm latch remains the sole normal zero owner until Phase 5.
        self.declare_parameter("precision_terminal_target_speed_mps", 0.15)
        self.declare_parameter("precision_minimum_moving_speed_mps", 0.04)
        self.declare_parameter("precision_braking_latency_sec", 0.10)
        self.declare_parameter("precision_braking_margin_m", 0.05)
        self.declare_parameter("precision_curvature_enabled", False)
        self.declare_parameter("precision_lateral_acceleration_max_mps2", 0.30)
        self.declare_parameter("precision_curvature_epsilon_inv_m", 1.0e-6)

        # Phase-4 centimetre tracking stability and run metrics.  This layer
        # is separately default-OFF and advisory to completion: it can gate
        # acceleration/cap translation, but never certifies a mission point.
        self.declare_parameter("precision_tracking_control_enabled", False)
        self.declare_parameter("precision_tracking_recovery_enter_xtrack_m", 0.050)
        self.declare_parameter("precision_tracking_recovery_exit_xtrack_m", 0.020)
        self.declare_parameter("precision_tracking_recovery_enter_heading_deg", 15.0)
        self.declare_parameter("precision_tracking_recovery_exit_heading_deg", 5.0)
        self.declare_parameter("precision_tracking_stable_dwell_sec", 0.30)
        self.declare_parameter("precision_tracking_recovery_speed_scale", 0.35)
        self.declare_parameter("precision_tracking_recapture_speed_scale", 0.50)
        self.declare_parameter("precision_tracking_metrics_capacity", 2048)
        self.declare_parameter("precision_tracking_histogram_bin_width_m", 0.001)
        self.declare_parameter("precision_tracking_histogram_max_m", 1.0)
        self.declare_parameter("precision_tracking_monotonic_tolerance_m", 0.001)
        self.declare_parameter("precision_tracking_cruise_threshold_mps", 0.80)

        # Phase-5 measured terminal certificate.  Separately default-OFF: the
        # legacy 30 mm latch remains untouched unless the complete precision
        # geometry/guidance/speed/tracking stack is deliberately enabled.
        self.declare_parameter("precision_terminal_enabled", False)
        self.declare_parameter("precision_terminal_radial_tolerance_m", 0.010)
        self.declare_parameter("precision_terminal_capture_tolerance_m", 0.010)
        self.declare_parameter("precision_terminal_settle_tolerance_m", 0.010)
        self.declare_parameter("precision_terminal_stop_speed_tolerance_mps", 0.010)
        self.declare_parameter(
            "precision_terminal_stop_yaw_rate_tolerance_radps", 0.050
        )
        self.declare_parameter("precision_terminal_settle_dwell_sec", 0.30)
        self.declare_parameter("precision_terminal_telemetry_timeout_sec", 0.25)
        self.declare_parameter("precision_terminal_approach_distance_m", 0.75)
        self.declare_parameter("precision_terminal_brake_distance_m", 0.30)
        self.declare_parameter("precision_terminal_timeout_sec", 15.0)
        self.declare_parameter("precision_terminal_settle_timeout_sec", 5.0)
        self.declare_parameter("precision_terminal_min_actuatable_speed_mps", 0.04)

        # Radial-20mm one-shot terminal stop regulator
        # (terminal_stop_regulator.py). Distinct purpose from the Phase-5 FSM
        # above -- see the plan review: fixed configured deceleration rate
        # replaced by a measured-speed stopping lead, no speed floor is
        # preserved through the goal, and it fails closed on timeout. Exactly
        # one terminal authority may be selected via terminal_stop_mode; this
        # module is only reachable when terminal_stop_mode=radial20 below.
        self.declare_parameter("terminal_stop_mode", "legacy")
        self.declare_parameter("radial_stop_radial_tolerance_m", 0.020)
        self.declare_parameter("radial_stop_terminal_guidance_distance_m", 0.75)
        self.declare_parameter("radial_stop_conservative_decel_mps2", 0.30)
        self.declare_parameter("radial_stop_brake_margin_m", 0.010)
        self.declare_parameter("radial_stop_stationary_window_sec", 0.50)
        self.declare_parameter("radial_stop_stationary_displacement_m", 0.005)
        self.declare_parameter("radial_stop_stationary_yaw_rate_radps", 0.050)
        self.declare_parameter("radial_stop_max_position_sample_gap_sec", 0.20)
        self.declare_parameter("radial_stop_terminal_timeout_sec", 15.0)
        self.declare_parameter("radial_stop_settle_timeout_sec", 5.0)
        self.declare_parameter("radial_stop_telemetry_timeout_sec", 0.25)

        # Phase-3 measured pivot/recenter controller.  It is independent and
        # default-OFF: the production Phase-A/B keeper below remains byte-for-
        # byte authoritative until this flag is explicitly enabled.
        self.declare_parameter("precision_pivot_enabled", False)
        self.declare_parameter("precision_pivot_anchor_tolerance_m", 0.030)
        self.declare_parameter("precision_pivot_recenter_threshold_m", 0.030)
        self.declare_parameter("precision_pivot_stop_speed_tolerance_mps", 0.010)
        self.declare_parameter("precision_pivot_stop_yaw_rate_tolerance_radps", 0.050)
        self.declare_parameter("precision_pivot_telemetry_timeout_sec", 0.25)
        self.declare_parameter("precision_pivot_stop_settle_sec", 0.20)
        self.declare_parameter("precision_pivot_heading_tolerance_deg", 2.0)
        self.declare_parameter("precision_pivot_release_settle_sec", 0.20)
        self.declare_parameter("precision_pivot_brake_timeout_sec", 8.0)
        self.declare_parameter("precision_pivot_timeout_sec", 9.0)
        self.declare_parameter("precision_pivot_recenter_speed_mps", 0.12)
        self.declare_parameter("precision_pivot_recenter_timeout_sec", 5.0)
        self.declare_parameter("precision_pivot_max_recenter_attempts", 2)
        self.declare_parameter("precision_pivot_realign_timeout_sec", 9.0)
        self.declare_parameter("precision_pivot_recapture_timeout_sec", 8.0)
        self.declare_parameter("post_pivot_capture_speed_mps", 0.20)
        self.declare_parameter("precision_pivot_recapture_xtrack_m", 0.020)
        self.declare_parameter("precision_pivot_recapture_heading_deg", 2.0)
        self.declare_parameter("precision_pivot_recapture_settle_sec", 0.20)
        self.declare_parameter("precision_pivot_recenter_forward_cone_deg", 30.0)
        # Extra literal-zero hold after the measured native-pivot stop
        # certificate.  Independent of the default-off precision pivot FSM.
        self.declare_parameter("legacy_pivot_post_settle_hold_sec", 1.00)
        # A lone speed/yaw-rate sample outside tolerance (GPS-antenna
        # lever-arm noise during residual yaw settling) must persist past
        # this window before the settle/hold dwell timers are discarded.
        self.declare_parameter(
            "legacy_pivot_stationary_violation_debounce_sec", 0.10
        )
        # Offer the post-pivot runtime reanchor on every leg, not just the
        # C->P1 entry leg. A pivot walks the rover 300-600 mm off the line it
        # is about to drive; only the entry leg used to rebuild its line from
        # the post-pivot position, and it is the only leg that lands inside
        # the 30 mm marking latch. Set False to restore entry-leg-only.
        self.declare_parameter("post_pivot_reanchor_all_legs", True)

        self.local_frame = str(self.get_parameter("local_frame").value).strip()
        self.cruise_speed = float(self.get_parameter("cruise_speed_mps").value)
        self.acceleration_enabled = bool(
            self.get_parameter("acceleration_enabled").value
        )
        self.acceleration_distance = float(
            self.get_parameter("acceleration_distance_m").value
        )
        self.acceleration_startup_ceiling = float(
            self.get_parameter("acceleration_startup_ceiling_mps").value
        )
        self.acceleration_max_progress_jump = float(
            self.get_parameter("acceleration_max_progress_jump_m").value
        )
        self.acceleration_max_dt_sec = float(
            self.get_parameter("acceleration_max_dt_sec").value
        )
        self.acceleration_rate = (
            self.cruise_speed * self.cruise_speed / (2.0 * self.acceleration_distance)
        )
        self.acceleration_duration = self.cruise_speed / self.acceleration_rate
        self.command_speed_rise_limit = float(
            self.get_parameter("command_speed_rise_limit_mps2").value
        )
        self.command_speed_fall_limit = float(
            self.get_parameter("command_speed_fall_limit_mps2").value
        )

        self.deceleration_enabled = bool(
            self.get_parameter("deceleration_enabled").value
        )
        self.deceleration_distance = float(
            self.get_parameter("deceleration_distance_m").value
        )
        self.deceleration_floor_speed = float(
            self.get_parameter("deceleration_floor_speed_mps").value
        )
        self.deceleration_max_progress_jump = float(
            self.get_parameter("deceleration_max_progress_jump_m").value
        )
        self.deceleration_max_dt_sec = float(
            self.get_parameter("deceleration_max_dt_sec").value
        )
        # The configured 0.50 m is measured to the exact semantic goal centre.
        # Since the rover stops at the 30 mm capture boundary, the physical
        # deceleration span is (0.50 - 0.03) = 0.47 m.
        deceleration_waypoint_tolerance = float(
            self.get_parameter("waypoint_tolerance_m").value
        )

        self.deceleration_profile_span = (
            self.deceleration_distance - deceleration_waypoint_tolerance
        )
        # The fixed 1.00 m/s cruise is above the 0.15 m/s terminal floor,
        # so semantic-goal deceleration is active for every precision stop.
        self.deceleration_required = (
            self.cruise_speed > self.deceleration_floor_speed + 1.0e-9
        )
        if self.deceleration_required:
            self.deceleration_rate = (
                self.cruise_speed * self.cruise_speed
                - self.deceleration_floor_speed * self.deceleration_floor_speed
            ) / (2.0 * self.deceleration_profile_span)
            self.deceleration_duration = (
                self.cruise_speed - self.deceleration_floor_speed
            ) / self.deceleration_rate
        else:
            self.deceleration_rate = 0.0
            self.deceleration_duration = 0.0

        self.terminal_decel_correction_limit = math.radians(
            float(self.get_parameter("terminal_decel_correction_limit_deg").value)
        )
        self.terminal_near_correction_limit = math.radians(
            float(self.get_parameter("terminal_near_correction_limit_deg").value)
        )
        self.terminal_near_correction_start_distance = float(
            self.get_parameter("terminal_near_correction_start_distance_m").value
        )
        self.terminal_bearing_freeze_distance = float(
            self.get_parameter("terminal_bearing_freeze_distance_m").value
        )
        self.terminal_correction_slew_rate = math.radians(
            float(self.get_parameter("terminal_correction_slew_rate_degps").value)
        )
        self.terminal_frozen_xtrack_abort = float(
            self.get_parameter("terminal_frozen_xtrack_abort_m").value
        )

        self.minimum_speed = float(self.get_parameter("minimum_speed_mps").value)
        self.segment_alignment_speed = float(
            self.get_parameter("segment_alignment_speed_mps").value
        )
        self.segment_alignment_recovery_speed = float(
            self.get_parameter("segment_alignment_recovery_speed_mps").value
        )
        self.segment_alignment_deadband_enter_cross_track = float(
            self.get_parameter("segment_alignment_deadband_enter_cross_track_m").value
        )
        self.segment_alignment_deadband_exit_cross_track = float(
            self.get_parameter("segment_alignment_deadband_exit_cross_track_m").value
        )
        self.segment_alignment_min_effective_heading_error = math.radians(
            float(
                self.get_parameter(
                    "segment_alignment_min_effective_heading_error_deg"
                ).value
            )
        )
        self.segment_alignment_correction_limit = math.radians(
            float(self.get_parameter("segment_alignment_correction_limit_deg").value)
        )
        self.segment_alignment_cross_track_tolerance = float(
            self.get_parameter("segment_alignment_cross_track_tolerance_m").value
        )
        self.segment_alignment_reentry_cross_track = float(
            self.get_parameter("segment_alignment_reentry_cross_track_m").value
        )
        self.segment_alignment_max_cross_track = float(
            self.get_parameter("segment_alignment_max_cross_track_m").value
        )
        self.terminal_line_entry_cross_track = float(
            self.get_parameter("terminal_line_entry_cross_track_m").value
        )
        self.xtrack_priority_enter = float(
            self.get_parameter("xtrack_priority_enter_m").value
        )
        self.xtrack_priority_exit = float(
            self.get_parameter("xtrack_priority_exit_m").value
        )
        self.xtrack_priority_hold_sec = float(
            self.get_parameter("xtrack_priority_hold_sec").value
        )
        self.xtrack_priority_speed = float(
            self.get_parameter("xtrack_priority_speed_mps").value
        )
        self.xtrack_priority_lookahead = float(
            self.get_parameter("xtrack_priority_lookahead_m").value
        )
        self.xtrack_priority_correction_limit = math.radians(
            float(self.get_parameter("xtrack_priority_correction_limit_deg").value)
        )
        self.xtrack_prediction_time_sec = float(
            self.get_parameter("xtrack_prediction_time_sec").value
        )
        self.xtrack_rate_filter_alpha = float(
            self.get_parameter("xtrack_rate_filter_alpha").value
        )
        self.xtrack_correction_slew_rate = math.radians(
            float(self.get_parameter("xtrack_correction_slew_rate_degps").value)
        )
        self.xtrack_neutral_crossing_band = float(
            self.get_parameter("xtrack_neutral_crossing_band_m").value
        )
        self.xtrack_priority_release_heading = math.radians(
            float(self.get_parameter("xtrack_priority_release_heading_deg").value)
        )
        self.terminal_xtrack_lookahead = float(
            self.get_parameter("terminal_xtrack_lookahead_m").value
        )
        self.terminal_xtrack_correction_limit = math.radians(
            float(self.get_parameter("terminal_xtrack_correction_limit_deg").value)
        )
        self.terminal_xtrack_prediction_time_sec = float(
            self.get_parameter("terminal_xtrack_prediction_time_sec").value
        )
        self.terminal_xtrack_neutral_crossing_band = float(
            self.get_parameter("terminal_xtrack_neutral_crossing_band_m").value
        )
        self.terminal_xtrack_correction_slew_rate = math.radians(
            float(
                self.get_parameter("terminal_xtrack_correction_slew_rate_degps").value
            )
        )
        self.terminal_xtrack_unwind_slew_rate = math.radians(
            float(self.get_parameter("terminal_xtrack_unwind_slew_rate_degps").value)
        )
        self.terminal_xtrack_away_lookahead = float(
            self.get_parameter("terminal_xtrack_away_lookahead_m").value
        )
        self.terminal_xtrack_away_correction_limit = math.radians(
            float(self.get_parameter("terminal_xtrack_away_correction_limit_deg").value)
        )
        self.terminal_xtrack_away_rate_threshold = float(
            self.get_parameter("terminal_xtrack_away_rate_threshold_mps").value
        )
        self.terminal_xtrack_crossing_prediction_time_sec = float(
            self.get_parameter("terminal_xtrack_crossing_prediction_time_sec").value
        )
        self.terminal_xtrack_crossing_lookahead = float(
            self.get_parameter("terminal_xtrack_crossing_lookahead_m").value
        )
        self.terminal_xtrack_crossing_correction_limit = math.radians(
            float(
                self.get_parameter(
                    "terminal_xtrack_crossing_correction_limit_deg"
                ).value
            )
        )
        self.terminal_xtrack_crossing_rate_threshold = float(
            self.get_parameter("terminal_xtrack_crossing_rate_threshold_mps").value
        )
        self.terminal_xtrack_crossing_predicted_threshold = float(
            self.get_parameter("terminal_xtrack_crossing_predicted_threshold_m").value
        )
        self.slow_distance = float(self.get_parameter("slow_distance_m").value)
        self.decel_profile_distance_1 = float(
            self.get_parameter("decel_profile_distance_1_m").value
        )
        self.decel_profile_speed_1 = float(
            self.get_parameter("decel_profile_speed_1_mps").value
        )
        self.decel_profile_distance_2 = float(
            self.get_parameter("decel_profile_distance_2_m").value
        )
        self.decel_profile_speed_2 = float(
            self.get_parameter("decel_profile_speed_2_mps").value
        )
        self.decel_profile_distance_3 = float(
            self.get_parameter("decel_profile_distance_3_m").value
        )
        self.decel_profile_speed_3 = float(
            self.get_parameter("decel_profile_speed_3_mps").value
        )
        self.final_speed_distance = float(
            self.get_parameter("final_speed_distance_m").value
        )
        self.waypoint_tolerance = float(
            self.get_parameter("waypoint_tolerance_m").value
        )

        self.pivot_enter_angle = math.radians(
            float(self.get_parameter("pivot_enter_angle_deg").value)
        )
        self.pivot_exit_angle = math.radians(
            float(self.get_parameter("pivot_exit_angle_deg").value)
        )
        self.alignment_hold_sec = float(self.get_parameter("alignment_hold_sec").value)
        self.maximum_yaw_rate = float(
            self.get_parameter("maximum_yaw_rate_radps").value
        )
        self.minimum_yaw_rate = float(
            self.get_parameter("minimum_yaw_rate_radps").value
        )
        self.pivot_yaw_kp = float(self.get_parameter("pivot_yaw_kp").value)
        self.alignment_reentry_goal_distance = float(
            self.get_parameter("alignment_reentry_goal_distance_m").value
        )

        self.path_correction_limit = math.radians(
            float(self.get_parameter("path_correction_limit_deg").value)
        )
        self.terminal_line_correction_limit = math.radians(
            float(self.get_parameter("terminal_line_correction_limit_deg").value)
        )
        self.terminal_line_alignment_distance = float(
            self.get_parameter("terminal_line_alignment_distance_m").value
        )
        self.line_tracking_lookahead = float(
            self.get_parameter("line_tracking_lookahead_m").value
        )
        self.line_tracking_lookahead_min = float(
            self.get_parameter("line_tracking_lookahead_min_m").value
        )
        self.line_tracking_lookahead_max = float(
            self.get_parameter("line_tracking_lookahead_max_m").value
        )
        self.line_tracking_lookahead_xtrack_gain = float(
            self.get_parameter("line_tracking_lookahead_xtrack_gain").value
        )
        # Derived time gain: reproduces line_tracking_lookahead_m exactly at
        # cruise_speed_mps (xtrack=0), so cruise-speed behaviour is unchanged.
        self.line_tracking_lookahead_speed_gain = (
            self.line_tracking_lookahead / self.cruise_speed
        )
        self.nav_path_lookahead = float(
            self.get_parameter("nav_path_lookahead_m").value
        )
        self.nav_path_point_reach = float(
            self.get_parameter("nav_path_point_reach_m").value
        )
        self.alignment_release_accel_distance = float(
            self.get_parameter("alignment_release_accel_distance_m").value
        )
        self.heading_full_speed = math.radians(
            float(self.get_parameter("heading_full_speed_deg").value)
        )
        self.heading_min_speed = math.radians(
            float(self.get_parameter("heading_min_speed_deg").value)
        )

        self.marking_terminal_speed_start_distance = float(
            self.get_parameter("marking_terminal_speed_start_distance_m").value
        )
        self.marking_terminal_max_speed = float(
            self.get_parameter("marking_terminal_max_speed_mps").value
        )
        self.marking_final_creep_start_distance = float(
            self.get_parameter("marking_final_creep_start_distance_m").value
        )
        self.marking_final_creep_speed = float(
            self.get_parameter("marking_final_creep_speed_mps").value
        )
        self.marking_final_creep_cross_track = float(
            self.get_parameter("marking_final_creep_cross_track_m").value
        )
        self.terminal_capture_gate_cross_track = float(
            self.get_parameter("terminal_capture_gate_cross_track_m").value
        )
        self.terminal_capture_gate_heading = math.radians(
            float(self.get_parameter("terminal_capture_gate_heading_deg").value)
        )
        self.terminal_capture_gate_hold_sec = float(
            self.get_parameter("terminal_capture_gate_hold_sec").value
        )
        self.terminal_recovery_min_heading_error = math.radians(
            float(self.get_parameter("terminal_recovery_min_heading_error_deg").value)
        )
        self.terminal_recovery_correction_limit = math.radians(
            float(self.get_parameter("terminal_recovery_correction_limit_deg").value)
        )
        self.terminal_recovery_lookahead_min = float(
            self.get_parameter("terminal_recovery_lookahead_min_m").value
        )
        self.terminal_exact_target_start_distance = float(
            self.get_parameter("terminal_exact_target_start_distance_m").value
        )
        self.terminal_goal_intercept_distance = float(
            self.get_parameter("terminal_goal_intercept_distance_m").value
        )
        self.terminal_goal_intercept_bearing_limit = math.radians(
            float(self.get_parameter("terminal_goal_intercept_bearing_limit_deg").value)
        )
        self.terminal_native_pivot_enter_error = math.radians(
            float(self.get_parameter("terminal_native_pivot_enter_error_deg").value)
        )
        self.terminal_native_pivot_release_error = math.radians(
            float(self.get_parameter("terminal_native_pivot_release_error_deg").value)
        )
        self.terminal_native_pivot_request_error = math.radians(
            float(self.get_parameter("terminal_native_pivot_request_error_deg").value)
        )
        self.segment_pivot_keeper_timeout_sec = float(
            self.get_parameter("segment_pivot_keeper_timeout_sec").value
        )
        self.segment_fast_capture_max_cross_track = float(
            self.get_parameter("segment_fast_capture_max_cross_track_m").value
        )
        self.terminal_close_recovery_distance = float(
            self.get_parameter("terminal_close_recovery_distance_m").value
        )
        self.terminal_close_recovery_speed = float(
            self.get_parameter("terminal_close_recovery_speed_mps").value
        )
        self.terminal_unready_hold_along = float(
            self.get_parameter("terminal_unready_hold_along_m").value
        )
        self.marking_stop_settle_timeout_sec = float(
            self.get_parameter("marking_stop_settle_timeout_sec").value
        )
        self.stationary_speed_tolerance = float(
            self.get_parameter("stationary_speed_tolerance_mps").value
        )
        self.marking_stop_latency_sec = float(
            self.get_parameter("marking_stop_latency_sec").value
        )
        self.marking_stop_extra_margin = float(
            self.get_parameter("marking_stop_extra_margin_m").value
        )
        self.marking_stop_min_buffer = float(
            self.get_parameter("marking_stop_min_buffer_m").value
        )
        self.marking_stop_max_buffer = float(
            self.get_parameter("marking_stop_max_buffer_m").value
        )
        self.marking_stop_xtrack_limit = float(
            self.get_parameter("marking_stop_xtrack_limit_m").value
        )

        self.marking_capture_arm_distance = float(
            self.get_parameter("marking_capture_arm_distance_m").value
        )
        self.marking_capture_abort_distance = float(
            self.get_parameter("marking_capture_abort_distance_m").value
        )
        self.miss_margin = float(self.get_parameter("miss_margin_m").value)
        self.marking_along_track_abort = float(
            self.get_parameter("marking_along_track_abort_m").value
        )

        self.waypoint_match_tolerance = float(
            self.get_parameter("waypoint_match_tolerance_m").value
        )
        self.odom_timeout_sec = float(self.get_parameter("odom_timeout_sec").value)
        self.waypoint_timeout_sec = float(
            self.get_parameter("waypoint_timeout_sec").value
        )
        self.geometry_tracking_enabled = bool(
            self.get_parameter("geometry_tracking_enabled").value
        )
        self.geometry_diagnostics_enabled = bool(
            self.get_parameter("geometry_diagnostics_enabled").value
        )
        self.geometry_corner_threshold = math.radians(
            float(self.get_parameter("geometry_corner_threshold_deg").value)
        )
        self.geometry_back_window_segments = int(
            self.get_parameter("geometry_projection_back_window_segments").value
        )
        self.geometry_forward_window_segments = int(
            self.get_parameter("geometry_projection_forward_window_segments").value
        )
        self.geometry_reacquire_distance = float(
            self.get_parameter("geometry_projection_reacquire_distance_m").value
        )
        self.geometry_localization_jump_reset = float(
            self.get_parameter("geometry_localization_jump_reset_m").value
        )
        self.geometry_max_backward_jump = float(
            self.get_parameter("geometry_max_backward_jump_m").value
        )
        self.geometry_max_forward_jump = float(
            self.get_parameter("geometry_max_forward_jump_m").value
        )

        self.precision_guidance_enabled = bool(
            self.get_parameter("precision_guidance_enabled").value
        )
        self.precision_speed_control_enabled = bool(
            self.get_parameter("precision_speed_control_enabled").value
        )
        self.precision_tracking_control_enabled = bool(
            self.get_parameter("precision_tracking_control_enabled").value
        )
        _TERMINAL_STOP_MODES = frozenset({"legacy", "precision_fsm", "radial20"})
        terminal_stop_mode_value = self.get_parameter("terminal_stop_mode").value
        if not isinstance(terminal_stop_mode_value, str):
            raise ValueError("terminal_stop_mode must be a string")
        self.terminal_stop_mode = terminal_stop_mode_value.strip().lower()
        if self.terminal_stop_mode not in _TERMINAL_STOP_MODES:
            allowed = ", ".join(sorted(_TERMINAL_STOP_MODES))
            raise ValueError(f"terminal_stop_mode must be one of: {allowed}")
        precision_terminal_param_value = bool(
            self.get_parameter("precision_terminal_enabled").value
        )
        if (
            precision_terminal_param_value
            and self.terminal_stop_mode != "precision_fsm"
        ):
            raise ValueError(
                "precision_terminal_enabled is deprecated and may be true "
                "only when terminal_stop_mode=precision_fsm"
            )
        self.precision_fsm_active = self.terminal_stop_mode == "precision_fsm"
        self.legacy_terminal_stop_active = self.terminal_stop_mode == "legacy"
        self.radial20_active = self.terminal_stop_mode == "radial20"
        # Compatibility alias: every existing precision_terminal_enabled call
        # site in this file continues to mean "the Phase-5 FSM branch is the
        # authoritative terminal path" -- never radial20, which is gated
        # separately on self.radial20_active throughout.
        self.precision_terminal_enabled = self.precision_fsm_active
        # Loaded fully below after Phase-2 configs are constructed.  Read the
        # gate here so geometry subscription/install authority includes the
        # Phase-3 consumer from the first retained DDS sample.
        precision_pivot_requested = bool(
            self.get_parameter("precision_pivot_enabled").value
        )
        self.geometry_processing_enabled = (
            self.geometry_tracking_enabled
            or self.geometry_diagnostics_enabled
            or self.precision_guidance_enabled
            or self.precision_speed_control_enabled
            or self.precision_tracking_control_enabled
            or self.precision_terminal_enabled
            or self.radial20_active
            or precision_pivot_requested
        )
        self.precision_guidance_config = GuidanceConfig(
            lookahead_min_m=float(
                self.get_parameter("precision_lookahead_min_m").value
            ),
            lookahead_max_m=float(
                self.get_parameter("precision_lookahead_max_m").value
            ),
            lookahead_time_s=float(
                self.get_parameter("precision_lookahead_time_s").value
            ),
            xtrack_lookahead_gain=float(
                self.get_parameter("precision_xtrack_lookahead_gain").value
            ),
            moving_bearing_cone_rad=math.radians(
                float(self.get_parameter("precision_moving_bearing_cone_deg").value)
            ),
        )
        self.precision_minimum_moving_speed = float(
            self.get_parameter("precision_minimum_moving_speed_mps").value
        )
        self.precision_speed_config = LongitudinalRegulatorConfig(
            hardware_speed_ceiling_mps=float(
                self.get_parameter("precision_hardware_speed_ceiling_mps").value
            ),
            acceleration_mps2=float(
                self.get_parameter("precision_acceleration_mps2").value
            ),
            deceleration_mps2=float(
                self.get_parameter("precision_deceleration_mps2").value
            ),
            launch_speed_mps=float(
                self.get_parameter("precision_launch_speed_mps").value
            ),
            control_dt_max_sec=float(
                self.get_parameter("precision_control_dt_max_sec").value
            ),
            heading_accel_full_error_rad=math.radians(
                float(
                    self.get_parameter("precision_heading_accel_full_error_deg").value
                )
            ),
            heading_recovery_start_rad=math.radians(
                float(self.get_parameter("precision_heading_recovery_start_deg").value)
            ),
            heading_recovery_full_rad=math.radians(
                float(self.get_parameter("precision_heading_recovery_full_deg").value)
            ),
            cross_track_accel_full_m=float(
                self.get_parameter("precision_xtrack_accel_full_m").value
            ),
            cross_track_recovery_start_m=float(
                self.get_parameter("precision_xtrack_recovery_start_m").value
            ),
            cross_track_recovery_full_m=float(
                self.get_parameter("precision_xtrack_recovery_full_m").value
            ),
            recovery_min_speed_mps=float(
                self.get_parameter("precision_recovery_min_speed_mps").value
            ),
            recovery_exit_dwell_sec=self.xtrack_priority_hold_sec,
            corner_angle_threshold_rad=math.radians(
                float(self.get_parameter("precision_corner_angle_threshold_deg").value)
            ),
            corner_target_speed_mps=float(
                self.get_parameter("precision_corner_target_speed_mps").value
            ),
            corner_accel_block_buffer_m=float(
                self.get_parameter("precision_corner_accel_block_buffer_m").value
            ),
            terminal_target_speed_mps=float(
                self.get_parameter("precision_terminal_target_speed_mps").value
            ),
            braking_latency_sec=float(
                self.get_parameter("precision_braking_latency_sec").value
            ),
            braking_margin_m=float(
                self.get_parameter("precision_braking_margin_m").value
            ),
            curvature_enabled=bool(
                self.get_parameter("precision_curvature_enabled").value
            ),
            lateral_acceleration_max_mps2=float(
                self.get_parameter("precision_lateral_acceleration_max_mps2").value
            ),
            curvature_epsilon_inv_m=float(
                self.get_parameter("precision_curvature_epsilon_inv_m").value
            ),
        )
        self.precision_speed_regulator = LongitudinalRegulator(
            self.precision_speed_config
        )
        self.precision_tracking_config = TrackingControlConfig(
            recovery_enter_cross_track_m=float(
                self.get_parameter("precision_tracking_recovery_enter_xtrack_m").value
            ),
            recovery_exit_cross_track_m=float(
                self.get_parameter("precision_tracking_recovery_exit_xtrack_m").value
            ),
            recovery_enter_heading_error_rad=math.radians(
                float(
                    self.get_parameter(
                        "precision_tracking_recovery_enter_heading_deg"
                    ).value
                )
            ),
            recovery_exit_heading_error_rad=math.radians(
                float(
                    self.get_parameter(
                        "precision_tracking_recovery_exit_heading_deg"
                    ).value
                )
            ),
            stable_recapture_dwell_sec=float(
                self.get_parameter("precision_tracking_stable_dwell_sec").value
            ),
            control_dt_max_sec=self.precision_speed_config.control_dt_max_sec,
            recovery_speed_scale=float(
                self.get_parameter("precision_tracking_recovery_speed_scale").value
            ),
            recapture_speed_scale=float(
                self.get_parameter("precision_tracking_recapture_speed_scale").value
            ),
            metrics_quantile_window_capacity=int(
                self.get_parameter("precision_tracking_metrics_capacity").value
            ),
            metrics_histogram_bin_width_m=float(
                self.get_parameter("precision_tracking_histogram_bin_width_m").value
            ),
            metrics_histogram_max_m=float(
                self.get_parameter("precision_tracking_histogram_max_m").value
            ),
            metrics_monotonic_tolerance_m=float(
                self.get_parameter("precision_tracking_monotonic_tolerance_m").value
            ),
            metrics_cruise_speed_threshold_mps=float(
                self.get_parameter("precision_tracking_cruise_threshold_mps").value
            ),
        )
        self.precision_tracking_controller = TrackingStabilityController(
            self.precision_tracking_config
        )
        self.precision_tracking_metrics = TrackingMetricsAccumulator(
            self.precision_tracking_config
        )

        self.precision_terminal_telemetry_timeout_sec = float(
            self.get_parameter("precision_terminal_telemetry_timeout_sec").value
        )
        self.precision_terminal_config = TerminalConfig(
            terminal_radial_tolerance_m=float(
                self.get_parameter("precision_terminal_radial_tolerance_m").value
            ),
            capture_entry_tolerance_m=float(
                self.get_parameter("precision_terminal_capture_tolerance_m").value
            ),
            settle_radial_tolerance_m=float(
                self.get_parameter("precision_terminal_settle_tolerance_m").value
            ),
            stop_speed_tolerance_mps=float(
                self.get_parameter("precision_terminal_stop_speed_tolerance_mps").value
            ),
            stop_yaw_rate_tolerance_radps=float(
                self.get_parameter(
                    "precision_terminal_stop_yaw_rate_tolerance_radps"
                ).value
            ),
            settle_dwell_sec=float(
                self.get_parameter("precision_terminal_settle_dwell_sec").value
            ),
            approach_distance_m=float(
                self.get_parameter("precision_terminal_approach_distance_m").value
            ),
            brake_distance_m=float(
                self.get_parameter("precision_terminal_brake_distance_m").value
            ),
            terminal_timeout_sec=float(
                self.get_parameter("precision_terminal_timeout_sec").value
            ),
            settle_timeout_sec=float(
                self.get_parameter("precision_terminal_settle_timeout_sec").value
            ),
            control_dt_max_sec=self.precision_speed_config.control_dt_max_sec,
            minimum_actuatable_speed_mps=float(
                self.get_parameter("precision_terminal_min_actuatable_speed_mps").value
            ),
        )
        self.precision_terminal_fsm = TerminalStopStateMachine(
            self.precision_terminal_config
        )

        self.radial_stop_telemetry_timeout_sec = float(
            self.get_parameter("radial_stop_telemetry_timeout_sec").value
        )
        self.radial_stop_config = RadialStopConfig(
            radial_tolerance_m=float(
                self.get_parameter("radial_stop_radial_tolerance_m").value
            ),
            terminal_guidance_distance_m=float(
                self.get_parameter("radial_stop_terminal_guidance_distance_m").value
            ),
            conservative_decel_mps2=float(
                self.get_parameter("radial_stop_conservative_decel_mps2").value
            ),
            brake_margin_m=float(
                self.get_parameter("radial_stop_brake_margin_m").value
            ),
            stationary_window_sec=float(
                self.get_parameter("radial_stop_stationary_window_sec").value
            ),
            stationary_displacement_m=float(
                self.get_parameter("radial_stop_stationary_displacement_m").value
            ),
            stationary_yaw_rate_radps=float(
                self.get_parameter("radial_stop_stationary_yaw_rate_radps").value
            ),
            maximum_position_sample_gap_sec=float(
                self.get_parameter("radial_stop_max_position_sample_gap_sec").value
            ),
            terminal_timeout_sec=float(
                self.get_parameter("radial_stop_terminal_timeout_sec").value
            ),
            settle_timeout_sec=float(
                self.get_parameter("radial_stop_settle_timeout_sec").value
            ),
        )
        self.radial_stop_regulator = TerminalStopRegulator(self.radial_stop_config)
        self.radial_stop_request_armed = False
        self.radial_stop_identity = None
        self.radial_stop_identity_components = None
        self.radial_stop_last_result = None
        self.radial_stop_last_sample = None
        self._radial_stop_speed_last_sample = None

        self.precision_pivot_enabled = bool(
            self.get_parameter("precision_pivot_enabled").value
        )
        self.precision_pivot_telemetry_timeout_sec = float(
            self.get_parameter("precision_pivot_telemetry_timeout_sec").value
        )
        self.precision_pivot_recenter_speed = float(
            self.get_parameter("precision_pivot_recenter_speed_mps").value
        )
        self.post_pivot_capture_speed = float(
            self.get_parameter("post_pivot_capture_speed_mps").value
        )
        self.precision_pivot_recapture_xtrack = float(
            self.get_parameter("precision_pivot_recapture_xtrack_m").value
        )
        self.precision_pivot_recapture_heading = math.radians(
            float(self.get_parameter("precision_pivot_recapture_heading_deg").value)
        )
        self.precision_pivot_recapture_settle_sec = float(
            self.get_parameter("precision_pivot_recapture_settle_sec").value
        )
        self.precision_pivot_recenter_forward_cone = math.radians(
            float(self.get_parameter("precision_pivot_recenter_forward_cone_deg").value)
        )
        self.legacy_pivot_post_settle_hold_sec = float(
            self.get_parameter("legacy_pivot_post_settle_hold_sec").value
        )
        self.legacy_pivot_stationary_violation_debounce_sec = float(
            self.get_parameter(
                "legacy_pivot_stationary_violation_debounce_sec"
            ).value
        )
        self.post_pivot_reanchor_all_legs = bool(
            self.get_parameter("post_pivot_reanchor_all_legs").value
        )
        self.precision_pivot_config = PivotMotionConfig(
            pivot_anchor_tolerance_m=float(
                self.get_parameter("precision_pivot_anchor_tolerance_m").value
            ),
            pivot_recenter_threshold_m=float(
                self.get_parameter("precision_pivot_recenter_threshold_m").value
            ),
            stop_speed_tolerance_mps=float(
                self.get_parameter("precision_pivot_stop_speed_tolerance_mps").value
            ),
            stop_yaw_rate_tolerance_radps=float(
                self.get_parameter(
                    "precision_pivot_stop_yaw_rate_tolerance_radps"
                ).value
            ),
            release_heading_tolerance_rad=math.radians(
                float(self.get_parameter("precision_pivot_heading_tolerance_deg").value)
            ),
            stop_settle_sec=float(
                self.get_parameter("precision_pivot_stop_settle_sec").value
            ),
            pivot_release_settle_sec=float(
                self.get_parameter("precision_pivot_release_settle_sec").value
            ),
            control_dt_max_sec=float(
                self.get_parameter("precision_control_dt_max_sec").value
            ),
            brake_timeout_sec=float(
                self.get_parameter("precision_pivot_brake_timeout_sec").value
            ),
            pivot_timeout_sec=float(
                self.get_parameter("precision_pivot_timeout_sec").value
            ),
            recenter_timeout_sec=float(
                self.get_parameter("precision_pivot_recenter_timeout_sec").value
            ),
            realign_timeout_sec=float(
                self.get_parameter("precision_pivot_realign_timeout_sec").value
            ),
            recapture_timeout_sec=float(
                self.get_parameter("precision_pivot_recapture_timeout_sec").value
            ),
            max_recenter_attempts=int(
                self.get_parameter("precision_pivot_max_recenter_attempts").value
            ),
        )
        self.precision_pivot_fsm = VerifiedPivotStateMachine(
            self.precision_pivot_config
        )
        self.legacy_alignment = None

        # --------------------------------------------------------------
        # RPP ACCELERATION + MARKING-ONLY DECELERATION CONTRACT
        # --------------------------------------------------------------
        self.validate_parameters()
        self.legacy_alignment = LegacyAlignmentLifecycle(
            LegacyAlignmentConfig(
                native_release_heading_rad=self.terminal_native_pivot_release_error,
                stop_speed_mps=self.precision_pivot_config.stop_speed_tolerance_mps,
                stop_yaw_rate_radps=(
                    self.precision_pivot_config.stop_yaw_rate_tolerance_radps
                ),
                settle_sec=self.precision_pivot_config.pivot_release_settle_sec,
                post_settle_hold_sec=self.legacy_pivot_post_settle_hold_sec,
                non_pivot_release_xtrack_m=self.xtrack_priority_exit,
                non_pivot_release_heading_rad=(
                    self.terminal_native_pivot_release_error
                ),
                non_pivot_hold_sec=self.alignment_hold_sec,
                fast_capture_max_cross_track_m=(
                    self.segment_fast_capture_max_cross_track
                ),
                pivot_enter_rad=self.pivot_enter_angle,
                pivot_keeper_timeout_sec=self.segment_pivot_keeper_timeout_sec,
                pre_pivot_timeout_sec=self.precision_pivot_config.brake_timeout_sec,
                stationary_violation_debounce_sec=(
                    self.legacy_pivot_stationary_violation_debounce_sec
                ),
                reanchor_all_legs=self.post_pivot_reanchor_all_legs,
            )
        )
        self.get_logger().warn(
            "RPP ACCELERATION + MARKING DECELERATION ENABLED | "
            f"accel={self.acceleration_rate:.4f}m/s^2 over "
            f"{self.acceleration_distance:.2f}m (200mm) | "
            f"cruise={self.cruise_speed:.2f}m/s | "
            f"final_deceleration={self.deceleration_distance:.2f}m | "
            f"decel_floor={self.deceleration_floor_speed:.2f}m/s | "
            f"deceleration={self.deceleration_rate:.4f}m/s^2 | "
            "acceleration re-arms after every literal stop/completed marking | "
            "exact radial stop / no reverse"
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
        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
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
            "/nav_path",
            self.nav_path_callback,
            retained_qos,
        )
        self.create_subscription(
            Path,
            "/mission_waypoints",
            self.marking_waypoints_callback,
            retained_qos,
        )
        self.create_subscription(
            UInt8MultiArray,
            "/trajectory_generator/path_types",
            self.path_types_callback,
            retained_qos,
        )
        self.create_subscription(
            Int32MultiArray,
            "/trajectory_generator/marking_indices",
            self.marking_indices_callback,
            retained_qos,
        )
        self.create_subscription(
            String,
            "/trajectory_generator/path_signature",
            self.path_signature_callback,
            retained_qos,
        )
        self.create_subscription(
            Bool,
            "/trajectory_generator/ready",
            self.trajectory_ready_callback,
            retained_qos,
        )
        self.create_subscription(
            String,
            "/mission_manager/segment_goal_metadata",
            self.segment_goal_metadata_callback,
            command_qos,
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
        self.xtrack_speed_cap_active_pub = self.create_publisher(
            Bool,
            "/rpp/xtrack_speed_cap_active",
            command_qos,
        )
        self.xtrack_speed_cap_value_pub = self.create_publisher(
            Float64,
            "/rpp/xtrack_speed_cap_mps",
            command_qos,
        )
        self.terminal_precision_armed_pub = self.create_publisher(
            Bool,
            "/rpp/terminal_precision_armed",
            retained_qos,
        )
        self.terminal_bearing_frozen_pub = self.create_publisher(
            Bool,
            "/rpp/terminal_bearing_frozen",
            retained_qos,
        )
        self.terminal_correction_deg_pub = self.create_publisher(
            Float64,
            "/rpp/terminal_correction_deg",
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

        # Consolidated live accuracy telemetry consumed by rover_backend.
        # Monitoring only: this publisher never changes steering, speed,
        # waypoint capture, stopping, or mission state.
        self.accuracy_pub = self.create_publisher(
            String,
            "/rpp/accuracy",
            retained_qos,
        )

        # Exact runtime control telemetry.
        # Monitoring only: values are copied from the same RPP control cycle
        # that produces the velocity command. No downstream reconstruction.
        self.rpp_debug_pub = self.create_publisher(
            String,
            "/rpp/debug",
            debug_qos,
        )

        self.geometry_debug_pub = self.create_publisher(
            String,
            "/rpp/geometry_debug",
            command_qos,
        )
        self.guidance_debug_pub = self.create_publisher(
            String,
            "/rpp/guidance_debug",
            command_qos,
        )
        self.speed_debug_pub = self.create_publisher(
            String,
            "/rpp/speed_debug",
            command_qos,
        )
        self.tracking_debug_pub = self.create_publisher(
            String,
            "/rpp/tracking_debug",
            command_qos,
        )
        self.pivot_debug_pub = self.create_publisher(
            String,
            "/rpp/pivot_debug",
            command_qos,
        )
        self.terminal_certificate_pub = self.create_publisher(
            String,
            "/rpp/terminal_certificate",
            command_qos,
        )

        # Explicit terminal-result handshake to Mission Manager.
        # Motion ownership stays in RPP; Mission Manager still performs the
        # authoritative 3-second verification before ACHIEVED/FAILED.
        self.terminal_result_pub = self.create_publisher(
            String,
            "/rpp/terminal_result",
            command_qos,
        )

        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.current_speed_mps = math.inf
        self.current_yaw_rate_radps = math.inf
        self.last_odom_time = None

        self.target_x = None
        self.target_y = None
        self.target_path_bearing = None
        self.last_waypoint_time = None
        self.target_is_marking = False

        self.segment_goal_x = None
        self.segment_goal_y = None
        self.segment_goal_number = 0
        self.segment_start_x = None
        self.segment_start_y = None

        # When an extension/dummy reaches the exact 30 mm radius, Mission
        # Manager may publish the next semantic goal immediately. Keep zero
        # commanded until odometry confirms the chassis is stationary before
        # the next pivot/translation begins.
        self.post_extension_stationary_hold = False

        # Full retained 50 mm trajectory from trajectory_generator. RPP never
        # treats interpolation points as stop goals; they are path-following
        # references only.
        self.nav_path_points = []
        self.nav_path_received = False
        self.nav_path_segment_start_index = 0
        self.nav_path_cursor_index = 0
        self.nav_path_goal_index = None
        self.nav_path_lookahead_index = None

        # Temporary START->P1 sidecar; fixed /nav_path remains P1->Pn.
        self.runtime_entry_points = []
        self.runtime_entry_cursor_index = 0
        self.runtime_entry_lookahead_index = 0
        self.runtime_entry_goal_index = None

        self.marking_waypoints = []
        self.marking_metadata_received = False

        # Separately retained path components are installed atomically only
        # after their canonical signature matches.  These fields never mutate
        # the authoritative arrays above.
        self.geometry_pending_nav_points = None
        self.geometry_pending_marking_waypoints = None
        self.geometry_pending_path_types = None
        self.geometry_pending_marking_indices = None
        self.geometry_pending_path_signature = None
        self.geometry_trajectory_ready = False
        self.geometry_contract_synchronized = False
        self.path_geometry = None
        self.geometry_progress_tracker = None
        self.geometry_installed_signature = None
        self.geometry_previous_installed_signature = None
        self.geometry_pending_goal_metadata = None
        self.geometry_goal_binding = None
        self.geometry_active_span = None
        self.geometry_last_goal_raw_index = None
        self.geometry_last_projection = None
        self.geometry_last_projection_cycle_token = None
        self.geometry_last_odom_point = None
        self.geometry_last_reset_reason = GeometryResetReason.INITIAL_INSTALL.value
        self.geometry_reset_count = 0

        # New command math owns one projection/guidance/speed solution per
        # controller cycle.  Tokens make stale Phase-1 projection state
        # impossible to reuse after an early-return or failed projection.
        self.precision_cycle_token = 0
        self.geometry_last_projection_cycle_token = None
        self.precision_cycle_dt_sec = 1.0 / self.CONTROL_HZ
        self.precision_last_cycle_time = None
        self.precision_guidance_cycle_token = None
        self.precision_guidance_result = None
        self.precision_speed_cycle_token = None
        self.precision_speed_result = None
        self.precision_speed_request = None
        self.precision_terminal_speed_override_mps = None
        self.precision_tracking_cycle_token = None
        self.precision_tracking_output = None
        self.precision_tracking_input = None
        self.precision_tracking_reset_reason = "INITIALIZE"
        self.precision_tracking_reset_count = 0
        self.precision_tracking_mission_sequence = 0
        self.precision_tracking_mission_identity = None
        self.precision_last_published_translational_speed_mps = 0.0
        self.precision_regulator_reset_reason = "INITIALIZE"
        self.precision_regulator_reset_count = 0
        self.precision_speed_regulator.reset()
        self.precision_tracking_controller.reset()
        self.precision_tracking_metrics.reset()

        # Phase-3 adapter state.  Mission geometry remains authoritative; this
        # is a latched maneuver anchor/identity sidecar only.
        self.precision_pivot_anchor_x = None
        self.precision_pivot_anchor_y = None
        self.precision_pivot_anchor_identity = None
        self.precision_pivot_target_bearing = None
        self.precision_pivot_last_result = None
        self.precision_pivot_last_reset_reason = "INITIALIZE"
        self.precision_pivot_reset_count = 0
        self.precision_pivot_last_time_sec = 0.0
        self.precision_pivot_recapture_inside_since = None
        self.precision_pivot_reanchor_complete = False
        self.precision_pivot_release_certified = False

        self.mission_enabled = False
        self.emergency_stop = True
        self.marking_active = False

        self.segment_alignment_active = True
        self.segment_alignment_pivot_complete = False
        self.segment_pivot_keeper_started_at = None
        self._reset_legacy_alignment_lifecycle("INITIALIZE")
        self.alignment_forward_heading_recovery_active = False
        self.alignment_deadband_recovery_active = False
        self.alignment_inside_since = None
        self.alignment_release_x = None
        self.alignment_release_y = None

        # Cross-track speed-cap recovery is shared by normal and terminal motion.
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
        self._terminal_result_sent = None

        # Phase-5 adapter state. The pure FSM is stepped at most once per
        # control-cycle token. Identity components are copied from the raw,
        # synchronized semantic-goal metadata and never inferred from pose.
        self.precision_terminal_cycle_token = None
        self.precision_terminal_last_result = None
        self.precision_terminal_request_armed = False
        self.precision_terminal_identity = None
        self.precision_terminal_identity_components = None
        self.precision_terminal_last_sample = None
        self.precision_terminal_last_reset_reason = "INITIALIZE"
        self.precision_terminal_reset_count = 0
        self.precision_terminal_historical_certificate = None
        # Latched once per terminal identity by
        # _resolve_precision_terminal_measurement_bearing(); frozen until the
        # next _reset_precision_terminal() semantic boundary.
        self.precision_terminal_measurement_bearing = None
        self.precision_terminal_measurement_bearing_source = None

        # First marking state. C is captured when the mission is enabled.
        self.first_marking_completed = False
        self.first_marking_hold_seen = False
        self.c_line_locked = False
        self.c_line_start_x = None
        self.c_line_start_y = None
        self.c_line_bearing = None
        self.c_line_reanchored_after_pivot = False
        # Per-leg twin of c_line_reanchored_after_pivot for legs after the
        # C->P1 entry leg. Cleared on every semantic segment change so each
        # leg gets exactly one post-pivot reanchor.
        self.segment_runtime_reanchored = False
        # True for any cycle whose steering follows a locally generated
        # runtime line rather than the surveyed /nav_path. Precision guidance
        # derives its correction from the /nav_path geometry projection, so it
        # must not hold bearing authority on those cycles -- it would steer
        # back to the surveyed line the runtime path deliberately replaced.
        # The C->P1 entry leg has always run this way; a post-pivot reanchored
        # leg now does too. Recomputed every control cycle at path selection.
        self.following_runtime_line = False

        # Continuous final-corridor gate and latched precision state.
        self.terminal_gate_inside_since = None
        self.terminal_gate_ready = False
        self.terminal_precision_armed = False
        self.terminal_bearing_frozen = False
        self.terminal_limited_correction = 0.0
        self.terminal_correction_last_update_time = None

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

        # Monotonic along-track terminal speed state. GNSS/odometry jitter
        # cannot re-accelerate the rover near the point.
        self.deceleration_active = False
        self.deceleration_complete = False
        self.deceleration_progress_m = 0.0
        self.deceleration_remaining_m = self.deceleration_distance
        self.deceleration_output_speed = self.cruise_speed
        self.deceleration_last_update_time = None
        self.deceleration_jump_warning_emitted = False

        # Global speed-slew state. This prevents a sudden 0.12->0.40 jump
        # after alignment and a sudden 0.40->terminal-profile transition.
        self.command_slew_speed = 0.0
        self.command_slew_last_time = None

        now = self.get_clock().now()
        self.last_log_time = now
        self.last_wait_log_time = now
        self.last_mm_monitor_log_time = now

        # /rpp/debug is a 50 Hz latest-sample transport. The controller remains
        # 20 Hz so telemetry cannot change motion dynamics. Each transport
        # frame carries both a telemetry sequence and the source control-cycle
        # sequence/age, making repeated samples explicit and measurable.
        self._rpp_debug_lock = threading.Lock()
        self._rpp_debug_control_sequence = 0
        self._rpp_debug_telemetry_sequence = 0
        self._rpp_debug_last_control_start_ns = None
        self._rpp_debug_cycle_start_ns = None
        self._rpp_debug_pending = None
        self._rpp_debug_snapshot = None
        self._rpp_debug_last_error_log_ns = 0
        self._rpp_debug_callback_group = MutuallyExclusiveCallbackGroup()

        # Feature gates are runtime mutable only while the vehicle is in the
        # same safe state required for stationary configuration.  Register
        # after all adapter state exists so an accepted transition can install
        # already-retained geometry atomically without commanding motion.
        self._precision_gate_parameter_callback = self.add_on_set_parameters_callback(
            self._on_set_precision_feature_gates
        )

        self.timer = self.create_timer(
            1.0 / self.CONTROL_HZ,
            self._control_timer_callback,
        )
        self.rpp_debug_timer = self.create_timer(
            1.0 / self.TELEMETRY_HZ,
            self._publish_rpp_debug_telemetry,
            callback_group=self._rpp_debug_callback_group,
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
        self.get_logger().warn(f"Cruise speed         : {self.cruise_speed:.3f} m/s")
        self.get_logger().warn(f"Fixed non-zero speed : {self.minimum_speed:.3f} m/s")
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
            "RPP SPEED PROFILE: 200mm start acceleration + semantic-goal deceleration over final "
            f"{self.deceleration_distance:.2f}m to the "
            f"{self.waypoint_tolerance * 1000.0:.0f}mm boundary | "
            f"{self.cruise_speed:.2f}->"
            f"{self.deceleration_floor_speed:.2f}m/s | "
            "zero only at measured radial boundary entry"
        )
        self.get_logger().warn(
            "Final deceleration magnitude: "
            f"{self.deceleration_rate:.4f}m/s^2 | "
            f"{self.cruise_speed:.3f}m/s at "
            f"{self.deceleration_distance * 1000.0:.0f}mm from semantic goal centre | "
            f"{self.deceleration_floor_speed:.3f}m/s at the "
            f"{self.waypoint_tolerance * 1000.0:.0f}mm capture boundary"
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
            "Terminal/final movement: bounded correction while slowing, "
            f"bearing frozen inside {self.terminal_bearing_freeze_distance:.2f}m, "
            f"creep={self.marking_final_creep_speed:.3f}m/s until exact zero"
        )
        self.get_logger().warn(
            "Terminal steering contract: stationary absolute-yaw attitude "
            "setpoint; "
            f"enter={math.degrees(self.terminal_native_pivot_enter_error):.1f}deg, "
            f"release={math.degrees(self.terminal_native_pivot_release_error):.1f}deg; "
            "translation targets the fixed 1.00m/s mission speed after alignment"
        )
        self.get_logger().warn(
            "Legacy native-pivot lifecycle: >4deg requests PRE-stop then native "
            "carrier; 4deg release holds zero until measured stop "
            "(2deg, 0.01m/s, 0.05rad/s) for "
            f"{self.precision_pivot_config.pivot_release_settle_sec:.2f}s plus "
            f"{self.legacy_pivot_post_settle_hold_sec:.2f}s; "
            f"post-pivot recapture<={self.post_pivot_capture_speed:.2f}m/s; "
            "precision_pivot_enabled remains off"
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
            "Alignment cross-track monitor: warn when abs(xtrack) >= "
            f"{self.segment_alignment_max_cross_track:.3f}m; recovery continues"
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

    def _precision_feature_gate_values(self):
        """Snapshot the control-authoritative values of all precision gates."""
        return {name: bool(getattr(self, name)) for name in PRECISION_FEATURE_GATES}

    def _on_set_precision_feature_gates(self, parameters):
        """Apply a safe, dependency-valid feature-gate transaction."""
        requested = {
            parameter.name: parameter
            for parameter in parameters
            if parameter.name in PRECISION_FEATURE_GATES
        }
        if not requested:
            # Nothing runtime-settable was asked for. Do NOT silently succeed:
            # every other parameter on this node is read once during __init__
            # and, in many cases, used to DERIVE further values -- cruise speed
            # alone sets acceleration_rate, deceleration_rate, and (via
            # rover.launch.py's CRUISE_SPEED_MPS) the alignment, recovery,
            # xtrack-priority, decel-profile and terminal-cap speeds. Accepting
            # a set for one of those changed what `ros2 param get` reported
            # while the controller kept driving on the original value, which is
            # worse than refusing: it invites a field operator to believe a
            # change took effect when it did not.
            unsupported = sorted(
                parameter.name
                for parameter in parameters
                if parameter.name not in _RUNTIME_SETTABLE_EXEMPT
            )
            if unsupported:
                return SetParametersResult(
                    successful=False,
                    reason=(
                        "restart-only parameter(s) "
                        + ", ".join(unsupported)
                        + "; this node accepts runtime changes only to "
                        "precision feature gates. Edit "
                        "rover_bringup/launch/rover.launch.py and restart the "
                        "stack so every derived value is recomputed."
                    ),
                )
            return SetParametersResult(successful=True)

        current = self._precision_feature_gate_values()
        prospective = dict(current)
        for name, parameter in requested.items():
            if parameter.type_ != Parameter.Type.BOOL:
                return SetParametersResult(
                    successful=False,
                    reason=f"{name} must be a boolean",
                )
            prospective[name] = bool(parameter.value)

        changed = {name for name in requested if prospective[name] != current[name]}
        if changed and (self.mission_enabled or not self.emergency_stop):
            return SetParametersResult(
                successful=False,
                reason=(
                    "precision feature gates may change only while mission_enable "
                    "is false and emergency_stop is true"
                ),
            )

        rejection_reason = validate_precision_feature_gates(prospective)
        if rejection_reason is not None:
            return SetParametersResult(
                successful=False,
                reason=rejection_reason,
            )
        if not changed:
            return SetParametersResult(successful=True)

        processing_was_enabled = self.geometry_processing_enabled
        processing_is_enabled = geometry_processing_requested(prospective)

        # No command publisher is reachable from this callback.  The complete
        # prospective set becomes authoritative together only after all safety
        # and dependency checks above have succeeded.
        for name in PRECISION_FEATURE_GATES:
            setattr(self, name, prospective[name])
        self.geometry_processing_enabled = processing_is_enabled
        if "precision_pivot_enabled" in changed:
            # Clear any carrier latch even when the new gate value is false;
            # _reset_precision_pivot() intentionally follows the active mode.
            self.reset_terminal_native_pivot()

        if processing_was_enabled and not processing_is_enabled:
            self._invalidate_installed_geometry("FEATURE_GATES_DISABLED")
        elif not processing_was_enabled and processing_is_enabled:
            self._try_install_path_geometry()
            self._try_bind_geometry_goal(log_error=False)
        else:
            self._reset_precision_regulator(
                "FEATURE_GATES_CHANGED",
                progress_s=(
                    self.geometry_active_span.start_s
                    if self.geometry_active_span is not None
                    else 0.0
                ),
            )
            self._reset_precision_tracking(
                "FEATURE_GATES_CHANGED",
                reset_metrics=False,
                path_identity=self.geometry_installed_signature,
            )
            self._reset_precision_pivot(
                "FEATURE_GATES_CHANGED",
                clear_anchor=True,
            )
            self._reset_precision_terminal("FEATURE_GATES_CHANGED")
            self._try_install_path_geometry()
            self._try_bind_geometry_goal(log_error=False)

        changed_labels = ", ".join(
            f"{name}={str(prospective[name]).lower()}" for name in sorted(changed)
        )
        self.get_logger().warn("PRECISION FEATURE GATES UPDATED | " + changed_labels)
        return SetParametersResult(successful=True)

    def validate_parameters(self):
        if not self.local_frame:
            raise ValueError("local_frame must not be empty")

        if (
            self.precision_guidance_enabled
            or self.precision_speed_control_enabled
            or self.precision_tracking_control_enabled
            or self.precision_pivot_enabled
        ) and not self.geometry_tracking_enabled:
            raise ValueError(
                "precision guidance, speed control, and pivot require "
                "geometry_tracking_enabled=true when enabled; Phase-2 features "
                "require geometry_tracking_enabled=true"
            )
        if self.precision_tracking_control_enabled and not (
            self.geometry_tracking_enabled
            and self.precision_guidance_enabled
            and self.precision_speed_control_enabled
        ):
            raise ValueError(
                "precision_tracking_control_enabled requires geometry_tracking, "
                "precision_guidance, and precision_speed_control"
            )
        if self.precision_terminal_enabled and not (
            self.geometry_tracking_enabled
            and self.precision_guidance_enabled
            and self.precision_speed_control_enabled
            and self.precision_tracking_control_enabled
            and self.precision_pivot_enabled
        ):
            raise ValueError(
                "precision_terminal_enabled requires geometry_tracking, "
                "precision_guidance, precision_speed_control, and "
                "precision_tracking_control, and precision_pivot"
            )
        if not (
            math.isfinite(self.precision_terminal_telemetry_timeout_sec)
            and 0.0
            < self.precision_terminal_telemetry_timeout_sec
            <= self.odom_timeout_sec
        ):
            raise ValueError(
                "precision_terminal_telemetry_timeout_sec must be finite, "
                "positive, and no greater than odom_timeout_sec"
            )
        if self.precision_terminal_enabled and not math.isclose(
            self.precision_terminal_config.minimum_actuatable_speed_mps,
            self.precision_minimum_moving_speed,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            raise ValueError(
                "precision_terminal_min_actuatable_speed_mps must equal "
                "precision_minimum_moving_speed_mps"
            )
        if (
            self.precision_terminal_enabled
            and self.precision_terminal_config.minimum_actuatable_speed_mps
            > self.precision_speed_config.hardware_speed_ceiling_mps
        ):
            raise ValueError(
                "precision terminal minimum actuatable speed exceeds hardware ceiling"
            )
        if not (
            math.isfinite(self.precision_pivot_telemetry_timeout_sec)
            and 0.0
            < self.precision_pivot_telemetry_timeout_sec
            <= self.odom_timeout_sec
        ):
            raise ValueError(
                "precision_pivot_telemetry_timeout_sec must be finite, positive, "
                "and no greater than odom_timeout_sec"
            )
        if not (
            math.isfinite(self.precision_pivot_recenter_speed)
            and self.minimum_speed
            <= self.precision_pivot_recenter_speed
            <= self.cruise_speed
        ):
            raise ValueError(
                "precision_pivot_recenter_speed_mps must be between minimum "
                "and cruise speed"
            )
        if not (
            math.isfinite(self.post_pivot_capture_speed)
            and self.minimum_speed <= self.post_pivot_capture_speed <= self.cruise_speed
        ):
            raise ValueError(
                "post_pivot_capture_speed_mps must be between minimum and cruise speed"
            )
        if not (
            math.isfinite(self.legacy_pivot_post_settle_hold_sec)
            and self.legacy_pivot_post_settle_hold_sec > 0.0
        ):
            raise ValueError(
                "legacy_pivot_post_settle_hold_sec must be finite and > 0"
            )
        if not (
            math.isfinite(self.legacy_pivot_stationary_violation_debounce_sec)
            and self.legacy_pivot_stationary_violation_debounce_sec > 0.0
        ):
            raise ValueError(
                "legacy_pivot_stationary_violation_debounce_sec must be "
                "finite and > 0"
            )
        if not (
            math.isfinite(self.precision_pivot_recapture_xtrack)
            and 0.0
            < self.precision_pivot_recapture_xtrack
            <= self.segment_alignment_cross_track_tolerance
        ):
            raise ValueError(
                "precision_pivot_recapture_xtrack_m must be positive and no "
                "greater than segment alignment tolerance"
            )
        if not (
            math.isfinite(self.precision_pivot_recapture_heading)
            and 0.0 < self.precision_pivot_recapture_heading < math.radians(45.0)
        ):
            raise ValueError(
                "precision_pivot_recapture_heading_deg must be finite and in (0,45)"
            )
        if not (
            math.isfinite(self.precision_pivot_recapture_settle_sec)
            and self.precision_pivot_recapture_settle_sec > 0.0
        ):
            raise ValueError(
                "precision_pivot_recapture_settle_sec must be finite and > 0"
            )
        if not (
            math.isfinite(self.precision_pivot_recenter_forward_cone)
            and 0.0
            < self.precision_pivot_recenter_forward_cone
            <= self.MAX_MOVING_HEADING_ERROR_RAD
        ):
            raise ValueError(
                "precision_pivot_recenter_forward_cone_deg must be within the "
                "verified moving-command cone"
            )
        if not (1.0 <= self.precision_pivot_config.pivot_timeout_sec <= 30.0):
            raise ValueError("precision_pivot_timeout_sec must be in [1,30]")
        if not math.isclose(
            self.precision_pivot_config.pivot_recenter_threshold_m,
            self.precision_pivot_config.pivot_anchor_tolerance_m,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            raise ValueError(
                "precision_pivot_recenter_threshold_m must equal "
                "precision_pivot_anchor_tolerance_m"
            )
        if not (
            0.0
            < self.precision_speed_config.terminal_target_speed_mps
            <= self.precision_speed_config.hardware_speed_ceiling_mps
        ):
            raise ValueError(
                "precision_terminal_target_speed_mps must be positive and not "
                "exceed precision_hardware_speed_ceiling_mps; Phase 2 must not "
                "create an early terminal zero"
            )
        if not (
            math.isfinite(self.precision_minimum_moving_speed)
            and 0.0
            < self.precision_minimum_moving_speed
            <= self.precision_speed_config.terminal_target_speed_mps
        ):
            raise ValueError(
                "precision_minimum_moving_speed_mps must be finite, positive, "
                "and no greater than precision_terminal_target_speed_mps"
            )
        if (
            self.precision_speed_config.recovery_min_speed_mps
            <= self.precision_minimum_moving_speed
        ):
            raise ValueError(
                "precision_recovery_min_speed_mps must be greater than "
                "precision_minimum_moving_speed_mps"
            )
        if (
            self.precision_speed_config.hardware_speed_ceiling_mps
            > self.MAXIMUM_MOVING_SPEED_MPS
        ):
            raise ValueError(
                "precision_hardware_speed_ceiling_mps must not exceed the "
                "verified moving hardware ceiling"
            )
        if (
            self.precision_guidance_config.moving_bearing_cone_rad
            > self.MAX_MOVING_HEADING_ERROR_RAD
        ):
            raise ValueError(
                "precision_moving_bearing_cone_deg must not exceed the "
                "verified 30 degree moving-command cone"
            )

        if not (0.0 < self.geometry_corner_threshold <= math.pi):
            raise ValueError("geometry_corner_threshold_deg must be in (0, 180]")
        for name, value in (
            (
                "geometry_projection_back_window_segments",
                self.geometry_back_window_segments,
            ),
            (
                "geometry_projection_forward_window_segments",
                self.geometry_forward_window_segments,
            ),
        ):
            if not (0 <= value <= 1000):
                raise ValueError(f"{name} must be an integer in [0, 1000]")
        for name, value in (
            (
                "geometry_projection_reacquire_distance_m",
                self.geometry_reacquire_distance,
            ),
            (
                "geometry_localization_jump_reset_m",
                self.geometry_localization_jump_reset,
            ),
        ):
            if not math.isfinite(value) or not (0.0 < value <= 1000.0):
                raise ValueError(f"{name} must be finite and in (0, 1000]")
        for name, value in (
            ("geometry_max_backward_jump_m", self.geometry_max_backward_jump),
            ("geometry_max_forward_jump_m", self.geometry_max_forward_jump),
        ):
            if not math.isfinite(value) or not (0.0 <= value <= 1000.0):
                raise ValueError(f"{name} must be finite and in [0, 1000]")

        positive_values = {
            "cruise_speed_mps": self.cruise_speed,
            "acceleration_distance_m": self.acceleration_distance,
            "acceleration_startup_ceiling_mps": (self.acceleration_startup_ceiling),
            "acceleration_max_progress_jump_m": (self.acceleration_max_progress_jump),
            "acceleration_max_dt_sec": self.acceleration_max_dt_sec,
            "command_speed_rise_limit_mps2": self.command_speed_rise_limit,
            "command_speed_fall_limit_mps2": self.command_speed_fall_limit,
            "deceleration_distance_m": self.deceleration_distance,
            "deceleration_floor_speed_mps": (self.deceleration_floor_speed),
            "deceleration_max_progress_jump_m": (self.deceleration_max_progress_jump),
            "deceleration_max_dt_sec": self.deceleration_max_dt_sec,
            "terminal_decel_correction_limit_deg": (
                self.terminal_decel_correction_limit
            ),
            "terminal_near_correction_limit_deg": (self.terminal_near_correction_limit),
            "terminal_near_correction_start_distance_m": (
                self.terminal_near_correction_start_distance
            ),
            "terminal_bearing_freeze_distance_m": (
                self.terminal_bearing_freeze_distance
            ),
            "terminal_correction_slew_rate_degps": (self.terminal_correction_slew_rate),
            "terminal_frozen_xtrack_abort_m": (self.terminal_frozen_xtrack_abort),
            "minimum_speed_mps": self.minimum_speed,
            "segment_alignment_speed_mps": (self.segment_alignment_speed),
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
            "terminal_line_entry_cross_track_m": (self.terminal_line_entry_cross_track),
            "xtrack_priority_enter_m": (self.xtrack_priority_enter),
            "xtrack_priority_exit_m": (self.xtrack_priority_exit),
            "xtrack_priority_hold_sec": (self.xtrack_priority_hold_sec),
            "xtrack_priority_speed_mps": (self.xtrack_priority_speed),
            "xtrack_priority_lookahead_m": (self.xtrack_priority_lookahead),
            "xtrack_priority_correction_limit_deg": (
                self.xtrack_priority_correction_limit
            ),
            "xtrack_prediction_time_sec": (self.xtrack_prediction_time_sec),
            "xtrack_rate_filter_alpha": (self.xtrack_rate_filter_alpha),
            "xtrack_correction_slew_rate_degps": (self.xtrack_correction_slew_rate),
            "xtrack_neutral_crossing_band_m": (self.xtrack_neutral_crossing_band),
            "xtrack_priority_release_heading_deg": (
                self.xtrack_priority_release_heading
            ),
            "terminal_xtrack_lookahead_m": (self.terminal_xtrack_lookahead),
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
            "terminal_xtrack_away_lookahead_m": (self.terminal_xtrack_away_lookahead),
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
            "decel_profile_distance_1_m": (self.decel_profile_distance_1),
            "decel_profile_speed_1_mps": (self.decel_profile_speed_1),
            "decel_profile_distance_2_m": (self.decel_profile_distance_2),
            "decel_profile_speed_2_mps": (self.decel_profile_speed_2),
            "decel_profile_distance_3_m": (self.decel_profile_distance_3),
            "decel_profile_speed_3_mps": (self.decel_profile_speed_3),
            "final_speed_distance_m": self.final_speed_distance,
            "waypoint_tolerance_m": self.waypoint_tolerance,
            "alignment_hold_sec": self.alignment_hold_sec,
            "maximum_yaw_rate_radps": self.maximum_yaw_rate,
            "minimum_yaw_rate_radps": self.minimum_yaw_rate,
            "pivot_yaw_kp": self.pivot_yaw_kp,
            "alignment_reentry_goal_distance_m": (self.alignment_reentry_goal_distance),
            "terminal_line_alignment_distance_m": (
                self.terminal_line_alignment_distance
            ),
            "line_tracking_lookahead_m": (self.line_tracking_lookahead),
            "line_tracking_lookahead_min_m": (self.line_tracking_lookahead_min),
            "line_tracking_lookahead_max_m": (self.line_tracking_lookahead_max),
            "line_tracking_lookahead_xtrack_gain": (
                self.line_tracking_lookahead_xtrack_gain
            ),
            "nav_path_lookahead_m": self.nav_path_lookahead,
            "nav_path_point_reach_m": self.nav_path_point_reach,
            "alignment_release_accel_distance_m": (
                self.alignment_release_accel_distance
            ),
            "marking_terminal_speed_start_distance_m": (
                self.marking_terminal_speed_start_distance
            ),
            "marking_terminal_max_speed_mps": (self.marking_terminal_max_speed),
            "marking_final_creep_start_distance_m": (
                self.marking_final_creep_start_distance
            ),
            "marking_final_creep_speed_mps": (self.marking_final_creep_speed),
            "marking_final_creep_cross_track_m": (self.marking_final_creep_cross_track),
            "terminal_capture_gate_cross_track_m": (
                self.terminal_capture_gate_cross_track
            ),
            "terminal_capture_gate_heading_deg": (self.terminal_capture_gate_heading),
            "terminal_capture_gate_hold_sec": (self.terminal_capture_gate_hold_sec),
            "terminal_recovery_min_heading_error_deg": (
                self.terminal_recovery_min_heading_error
            ),
            "terminal_recovery_correction_limit_deg": (
                self.terminal_recovery_correction_limit
            ),
            "terminal_recovery_lookahead_min_m": (self.terminal_recovery_lookahead_min),
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
            "segment_pivot_keeper_timeout_sec": (self.segment_pivot_keeper_timeout_sec),
            "legacy_pivot_post_settle_hold_sec": (
                self.legacy_pivot_post_settle_hold_sec
            ),
            "segment_fast_capture_max_cross_track_m": (
                self.segment_fast_capture_max_cross_track
            ),
            "terminal_close_recovery_distance_m": (
                self.terminal_close_recovery_distance
            ),
            "terminal_close_recovery_speed_mps": (self.terminal_close_recovery_speed),
            "terminal_unready_hold_along_m": (self.terminal_unready_hold_along),
            "marking_stop_settle_timeout_sec": (self.marking_stop_settle_timeout_sec),
            "stationary_speed_tolerance_mps": (self.stationary_speed_tolerance),
            "marking_stop_latency_sec": self.marking_stop_latency_sec,
            "marking_stop_extra_margin_m": self.marking_stop_extra_margin,
            "marking_stop_min_buffer_m": self.marking_stop_min_buffer,
            "marking_stop_max_buffer_m": self.marking_stop_max_buffer,
            "marking_stop_xtrack_limit_m": self.marking_stop_xtrack_limit,
            "marking_capture_arm_distance_m": (self.marking_capture_arm_distance),
            "marking_capture_abort_distance_m": (self.marking_capture_abort_distance),
            "miss_margin_m": self.miss_margin,
            "marking_along_track_abort_m": (self.marking_along_track_abort),
            "waypoint_match_tolerance_m": (self.waypoint_match_tolerance),
            "odom_timeout_sec": self.odom_timeout_sec,
            "waypoint_timeout_sec": self.waypoint_timeout_sec,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")

        if self.minimum_speed > self.cruise_speed:
            raise ValueError("minimum_speed_mps must be <= cruise_speed_mps")
        # The native-pivot carrier-vector magnitude and the post-pivot
        # translational line-capture speed are intentionally independent.
        # A 1.00 m/s carrier vector can be used to request PX4's differential
        # pivot while real forward translation remains zero; after alignment,
        # forward line capture immediately uses the fixed 1.00 m/s mission speed.
        if not (
            self.minimum_speed <= self.segment_alignment_speed <= self.cruise_speed
        ):
            raise ValueError(
                "segment_alignment_speed_mps must be between minimum "
                "speed and cruise_speed_mps"
            )
        if not (
            self.minimum_speed
            <= self.segment_alignment_recovery_speed
            <= self.cruise_speed
        ):
            raise ValueError(
                "segment_alignment_recovery_speed_mps must be between "
                "minimum_speed_mps and cruise_speed_mps"
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
                "xtrack priority requires " "0 < exit < enter <= alignment tolerance"
            )
        if not (self.minimum_speed <= self.xtrack_priority_speed <= self.cruise_speed):
            raise ValueError(
                "xtrack_priority_speed_mps must be between minimum speed "
                "and cruise speed"
            )
        if not (0.0 < self.xtrack_rate_filter_alpha <= 1.0):
            raise ValueError("xtrack_rate_filter_alpha must be in (0, 1]")
        if self.xtrack_neutral_crossing_band < self.xtrack_priority_enter:
            raise ValueError(
                "xtrack_neutral_crossing_band_m must be >= " "xtrack_priority_enter_m"
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
        if not (0.0 < self.xtrack_priority_release_heading < math.radians(45.0)):
            raise ValueError("xtrack priority release heading must be below 45deg")
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
        if not (self.terminal_xtrack_away_lookahead < self.terminal_xtrack_lookahead):
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
            raise ValueError("cruise_speed_mps must not exceed 1.00 m/s")
        # Previously pinned to exactly 1.00. Relaxed to a bounded range on
        # 2026-09-01 for staged speed bring-up: the FCU's RO_SPEED_LIM was
        # 0.40, so every mission so far was clamped there and the stack has
        # never actually been driven at its own configured cruise speed. The
        # 1.00 ceiling above is unchanged and still enforced; this only allows
        # deliberately commanding LESS. Every derived quantity -- acceleration
        # rate, deceleration rate, and the alignment/xtrack/terminal speeds
        # that rover.launch.py computes from CRUISE_SPEED_MPS -- is recomputed
        # from this value at init, which is why cruise speed remains a
        # restart-time setting and is not runtime-settable.
        if self.cruise_speed < self.minimum_speed:
            raise ValueError("cruise_speed_mps must be >= minimum_speed_mps")
        if self.acceleration_startup_ceiling >= self.cruise_speed:
            raise ValueError(
                "acceleration_startup_ceiling_mps must be below cruise speed"
            )
        if not math.isfinite(self.acceleration_rate) or self.acceleration_rate <= 0.0:
            raise ValueError("derived acceleration rate must be finite and > 0")
        if (
            not math.isfinite(self.acceleration_duration)
            or self.acceleration_duration <= 0.0
        ):
            raise ValueError("derived acceleration duration must be finite and > 0")
        if self.deceleration_required:
            if (
                not math.isfinite(self.deceleration_rate)
                or self.deceleration_rate <= 0.0
            ):
                raise ValueError(
                    "derived deceleration rate must be finite and > 0 "
                    "when cruise speed is above the deceleration floor"
                )
            if (
                not math.isfinite(self.deceleration_duration)
                or self.deceleration_duration <= 0.0
            ):
                raise ValueError(
                    "derived deceleration duration must be finite and > 0 "
                    "when cruise speed is above the deceleration floor"
                )
        elif not math.isclose(
            self.deceleration_rate, 0.0, abs_tol=1.0e-12
        ) or not math.isclose(self.deceleration_duration, 0.0, abs_tol=1.0e-12):
            raise ValueError(
                "zero-rate terminal profile is only valid when cruise speed "
                "equals the configured deceleration floor"
            )
        if (
            not math.isfinite(self.command_speed_rise_limit)
            or self.command_speed_rise_limit <= 0.0
            or not math.isfinite(self.command_speed_fall_limit)
            or self.command_speed_fall_limit <= 0.0
        ):
            raise ValueError("command speed rise/fall limits must be finite and > 0")
        if not (0.0 < self.deceleration_floor_speed <= self.cruise_speed):
            raise ValueError(
                "deceleration_floor_speed_mps must be greater than zero "
                "and <= cruise_speed_mps"
            )

        if not (
            self.deceleration_distance > self.waypoint_tolerance
            and math.isfinite(self.deceleration_profile_span)
            and self.deceleration_profile_span > 0.0
        ):
            raise ValueError(
                "deceleration_distance_m must be greater than " "waypoint_tolerance_m"
            )

        if not (
            0.0
            < self.terminal_near_correction_limit
            <= self.terminal_decel_correction_limit
            <= math.radians(22.0)
        ):
            raise ValueError(
                "terminal correction limits require 0 < near <= decel <= 22deg"
            )
        if not (
            self.waypoint_tolerance
            < self.terminal_bearing_freeze_distance
            < self.terminal_near_correction_start_distance
            < self.slow_distance
        ):
            raise ValueError(
                "terminal distances require tolerance < freeze < near < slow"
            )
        if self.terminal_frozen_xtrack_abort <= self.waypoint_tolerance:
            raise ValueError(
                "terminal_frozen_xtrack_abort_m must exceed waypoint tolerance"
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
            raise ValueError("speed-profile distances must be strictly descending")

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
            raise ValueError("speed-profile speeds must be monotonically decreasing")

        if self.final_speed_distance >= self.slow_distance:
            raise ValueError("final_speed_distance_m must be less than slow_distance_m")
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
                "waypoint_tolerance_m must be less than " "final_speed_distance_m"
            )
        if not (0.0 < self.marking_stop_min_buffer <= self.marking_stop_max_buffer):
            raise ValueError("marking stop buffers require 0 < min <= max")
        if not (0.0 < self.marking_stop_xtrack_limit <= self.waypoint_tolerance):
            raise ValueError(
                "marking_stop_xtrack_limit_m must be positive and not exceed waypoint_tolerance_m"
            )
        if self.pivot_exit_angle >= self.pivot_enter_angle:
            raise ValueError(
                "pivot_exit_angle_deg must be less than " "pivot_enter_angle_deg"
            )
        if not (
            math.isfinite(self.minimum_yaw_rate)
            and math.isfinite(self.maximum_yaw_rate)
            and 0.0 < self.minimum_yaw_rate <= self.maximum_yaw_rate
        ):
            raise ValueError(
                "0 < minimum_yaw_rate_radps <= maximum_yaw_rate_radps " "is required"
            )
        if not math.isfinite(self.pivot_yaw_kp) or self.pivot_yaw_kp <= 0.0:
            raise ValueError("pivot_yaw_kp must be finite and > 0")
        if self.heading_full_speed >= self.heading_min_speed:
            raise ValueError(
                "heading_full_speed_deg must be less than " "heading_min_speed_deg"
            )
        if not (
            0.0
            < self.terminal_line_correction_limit
            <= self.path_correction_limit
            < math.pi / 2.0
        ):
            raise ValueError("terminal/far line correction limits are invalid")
        if self.marking_terminal_max_speed < self.minimum_speed:
            raise ValueError(
                "marking_terminal_max_speed_mps must be >= " "minimum_speed_mps"
            )
        if self.marking_terminal_max_speed > self.cruise_speed:
            raise ValueError(
                "marking_terminal_max_speed_mps must be <= " "cruise_speed_mps"
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
            0.0 < self.marking_final_creep_speed <= self.marking_terminal_max_speed
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
            raise ValueError("terminal capture cross-track gates are inconsistent")
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
        if not (
            0.0
            < self.line_tracking_lookahead_min
            <= self.line_tracking_lookahead
            <= self.line_tracking_lookahead_max
        ):
            raise ValueError(
                "line_tracking_lookahead_min_m must be > 0 and <= "
                "line_tracking_lookahead_m <= line_tracking_lookahead_max_m"
            )
        if self.line_tracking_lookahead_xtrack_gain < 0.0:
            raise ValueError(
                "line_tracking_lookahead_xtrack_gain must be >= 0"
            )
        if (
            self.terminal_goal_intercept_distance
            <= self.terminal_line_alignment_distance
        ):
            raise ValueError(
                "terminal_goal_intercept_distance_m must be greater than "
                "terminal_line_alignment_distance_m"
            )
        if not (0.0 < self.terminal_goal_intercept_bearing_limit < math.radians(45.0)):
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
            raise ValueError("native pivot requires 0 < release < enter <= 45deg")
        if not (
            math.radians(45.0)
            < self.terminal_native_pivot_request_error
            < math.radians(90.0)
        ):
            raise ValueError(
                "terminal_native_pivot_request_error_deg must be "
                "between 45 and 90 degrees"
            )
        if (
            not math.isfinite(self.segment_pivot_keeper_timeout_sec)
            or self.segment_pivot_keeper_timeout_sec <= 0.0
        ):
            raise ValueError("segment_pivot_keeper_timeout_sec must be finite and > 0")
        if not (
            self.xtrack_priority_exit
            < self.segment_fast_capture_max_cross_track
            < self.segment_alignment_max_cross_track
        ):
            raise ValueError(
                "segment_fast_capture_max_cross_track_m must be greater "
                "than xtrack_priority_exit_m and less than "
                "segment_alignment_max_cross_track_m"
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
            0.0 < self.terminal_close_recovery_speed <= self.marking_terminal_max_speed
        ):
            raise ValueError(
                "terminal_close_recovery_speed_mps must be positive "
                "and <= marking_terminal_max_speed_mps"
            )
        if self.marking_capture_arm_distance >= self.marking_capture_abort_distance:
            raise ValueError(
                "marking_capture_arm_distance_m must be less than "
                "marking_capture_abort_distance_m"
            )

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def ground_xtrack(value):
        """Convert internal RPP xtrack to ground convention.

        Internal RPP:
            LEFT  = positive
            RIGHT = negative

        Ground/report convention:
            LEFT  = negative
            RIGHT = positive
        """
        return -float(value)

    def ground_terminal_certificate_payload(self, certificate):
        """Convert internal terminal certificate for external reporting only.

        Terminal FSM/certificate storage keeps the controller convention:
        LEFT positive, RIGHT negative.

        Published JSON uses the ground/report convention:
        LEFT negative, RIGHT positive.
        """
        if certificate is None:
            return None

        payload = certificate.to_dict()
        cross_error_mm = payload.get("cross_error_mm")

        if cross_error_mm is not None:
            try:
                cross_error_mm = float(cross_error_mm)
            except (TypeError, ValueError):
                pass
            else:
                if math.isfinite(cross_error_mm):
                    payload["cross_error_mm"] = self.ground_xtrack(cross_error_mm)

        return payload

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

    def _record_geometry_reset(self, reason):
        if isinstance(reason, GeometryResetReason):
            reason = reason.value
        self.geometry_last_reset_reason = str(reason)
        self.geometry_reset_count += 1

    def _invalidate_installed_geometry(self, reason):
        """Revoke installed precision authority without discarding staged DDS data."""

        if self.geometry_installed_signature is not None:
            self.geometry_previous_installed_signature = (
                self.geometry_installed_signature
            )
        self.geometry_contract_synchronized = False
        self.path_geometry = None
        self.geometry_progress_tracker = None
        self.geometry_installed_signature = None
        self.geometry_goal_binding = None
        self.geometry_active_span = None
        self.geometry_last_goal_raw_index = None
        self.geometry_last_projection = None
        self.geometry_last_projection_cycle_token = None
        self.geometry_last_odom_point = None
        self._reset_precision_regulator("GEOMETRY_INVALIDATED", progress_s=0.0)
        self._reset_precision_tracking(
            "GEOMETRY_INVALIDATED",
            reset_metrics=True,
            path_identity=None,
        )
        self._reset_precision_pivot("GEOMETRY_INVALIDATED", clear_anchor=True)
        self._reset_precision_terminal("GEOMETRY_INVALIDATED")
        self._reset_legacy_alignment_lifecycle("GEOMETRY_INVALIDATED")
        self._record_geometry_reset(reason)
        self.get_logger().warn(
            f"PRECISION GEOMETRY AUTHORITY INVALIDATED | reason={reason}"
        )

    def path_types_callback(self, msg):
        values = [int(value) for value in msg.data]
        if not values:
            self.geometry_pending_path_types = None
            if self.geometry_processing_enabled:
                self._invalidate_installed_geometry(GeometryResetReason.SOURCE_CLEARED)
            return
        self.geometry_pending_path_types = values
        self._try_install_path_geometry()

    def marking_indices_callback(self, msg):
        values = [int(value) for value in msg.data]
        if not values:
            self.geometry_pending_marking_indices = None
            if self.geometry_processing_enabled:
                self._invalidate_installed_geometry(GeometryResetReason.SOURCE_CLEARED)
            return
        self.geometry_pending_marking_indices = values
        self._try_install_path_geometry()

    def path_signature_callback(self, msg):
        signature = str(msg.data).strip()
        if not signature:
            self.geometry_pending_path_signature = None
            if self.geometry_processing_enabled:
                self._invalidate_installed_geometry(GeometryResetReason.SOURCE_CLEARED)
            return
        self.geometry_pending_path_signature = signature
        self._try_install_path_geometry()

    def trajectory_ready_callback(self, msg):
        self.geometry_trajectory_ready = bool(msg.data)
        if not self.geometry_trajectory_ready:
            if self.geometry_processing_enabled:
                self._invalidate_installed_geometry(GeometryResetReason.SOURCE_CLEARED)
            return
        self._try_install_path_geometry()

    def segment_goal_metadata_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.geometry_pending_goal_metadata = None
            self.geometry_goal_binding = None
            self.geometry_active_span = None
            self.get_logger().error("IGNORED INVALID SEGMENT GOAL METADATA JSON")
            return
        if not isinstance(payload, dict):
            self.geometry_pending_goal_metadata = None
            self.geometry_goal_binding = None
            self.geometry_active_span = None
            self.get_logger().error("IGNORED NON-OBJECT SEGMENT GOAL METADATA")
            return
        previous_metadata = self.geometry_pending_goal_metadata
        previous_instance = (
            (
                previous_metadata.get("mission_run_id"),
                previous_metadata.get("goal_instance_id"),
            )
            if isinstance(previous_metadata, dict)
            else (None, None)
        )
        new_instance = (
            payload.get("mission_run_id"),
            payload.get("goal_instance_id"),
        )
        if (
            self.geometry_processing_enabled
            and previous_instance != new_instance
            and any(new_instance)
        ):
            self._reset_precision_terminal("SEGMENT_GOAL_IDENTITY_CHANGED")
        self.geometry_pending_goal_metadata = payload
        self.geometry_goal_binding = None
        self.geometry_active_span = None
        self._try_bind_geometry_goal(log_error=True)

    def _try_install_path_geometry(self):
        if not self.geometry_processing_enabled or not self.geometry_trajectory_ready:
            return False
        components = (
            self.geometry_pending_nav_points,
            self.geometry_pending_marking_waypoints,
            self.geometry_pending_path_types,
            self.geometry_pending_marking_indices,
            self.geometry_pending_path_signature,
        )
        if any(value is None for value in components):
            return False

        navigation_points = list(self.geometry_pending_nav_points)
        marking_points = list(self.geometry_pending_marking_waypoints)
        point_types = list(self.geometry_pending_path_types)
        marking_indices = list(self.geometry_pending_marking_indices)
        signature = str(self.geometry_pending_path_signature)
        if not navigation_points or not marking_points:
            return False
        if not (len(navigation_points) == len(point_types) == len(marking_indices)):
            return False

        try:
            calculated_signature = make_path_signature(
                navigation_points,
                marking_points,
                point_types,
                marking_indices,
            )
        except (OverflowError, TypeError, ValueError):
            return False
        if calculated_signature != signature:
            return False

        try:
            geometry = PathGeometryIndex.build(
                navigation_points,
                point_types=point_types,
                marking_indices=marking_indices,
                corner_threshold_rad=self.geometry_corner_threshold,
            )
            marking_anchors = [
                anchor
                for anchor in geometry.semantic_anchors
                if anchor.point_type == POINT_TYPE_MARKING
            ]
        except (TypeError, ValueError) as error:
            self.get_logger().error(f"REJECTED PRECISION PATH GEOMETRY | {error}")
            return False

        if len(marking_anchors) != len(marking_points):
            self.get_logger().error(
                "REJECTED PRECISION PATH GEOMETRY | marking count mismatch"
            )
            return False
        for anchor, marking in zip(marking_anchors, marking_points):
            marking_error = math.hypot(
                anchor.point.x - marking[0],
                anchor.point.y - marking[1],
            )
            if marking_error > max(self.waypoint_match_tolerance, 0.002):
                self.get_logger().error(
                    "REJECTED PRECISION PATH GEOMETRY | marking coordinate mismatch"
                )
                return False

        previous_signature = (
            self.geometry_installed_signature
            or self.geometry_previous_installed_signature
        )
        if previous_signature == signature and self.path_geometry is not None:
            self.geometry_contract_synchronized = True
            self._try_bind_geometry_goal(log_error=False)
            return True

        self.path_geometry = geometry
        self.geometry_progress_tracker = GeometryProgressTracker(geometry)
        self.geometry_installed_signature = signature
        self.geometry_previous_installed_signature = signature
        self.geometry_contract_synchronized = True
        self.geometry_goal_binding = None
        self.geometry_active_span = None
        self.geometry_last_goal_raw_index = None
        self.geometry_last_projection = None
        self.geometry_last_projection_cycle_token = None
        self.geometry_last_odom_point = None
        if previous_signature is not None and previous_signature != signature:
            self.geometry_progress_tracker.reset(GeometryResetReason.PATH_REPLACED)
            self._record_geometry_reset(GeometryResetReason.PATH_REPLACED)
        else:
            self._record_geometry_reset(GeometryResetReason.INITIAL_INSTALL)
        self._reset_precision_regulator("PATH_INSTALLED", progress_s=0.0)
        self._reset_precision_tracking(
            "PATH_INSTALLED",
            reset_metrics=True,
            path_identity=signature,
        )
        self._reset_precision_pivot("PATH_INSTALLED", clear_anchor=True)
        self._reset_precision_terminal("PATH_INSTALLED")
        self._reset_legacy_alignment_lifecycle("PATH_INSTALLED")
        self._try_bind_geometry_goal(log_error=False)
        self.get_logger().warn(
            "PRECISION PATH GEOMETRY INSTALLED | "
            f"raw_points={len(geometry.raw_points)} | "
            f"segments={len(geometry.segments)} | "
            f"corners={len(geometry.corners)} | signature={signature[:12]}"
        )
        return True

    def _try_bind_geometry_goal(self, *, log_error):
        if not (
            self.geometry_processing_enabled
            and self.geometry_contract_synchronized
            and self.path_geometry is not None
            and self.geometry_progress_tracker is not None
            and self.geometry_installed_signature is not None
            and self.geometry_pending_goal_metadata is not None
            and self.segment_goal_x is not None
            and self.segment_goal_y is not None
        ):
            return False
        try:
            binding = validate_goal_metadata(
                self.geometry_pending_goal_metadata,
                expected_path_signature=self.geometry_installed_signature,
                geometry=self.path_geometry,
                goal_point=(self.segment_goal_x, self.segment_goal_y),
                coordinate_tolerance_m=max(self.waypoint_match_tolerance, 0.002),
            )
        except (TypeError, ValueError) as error:
            self.geometry_goal_binding = None
            self.geometry_active_span = None
            if log_error:
                self.get_logger().error(
                    f"REJECTED SEGMENT GOAL GEOMETRY BINDING | {error}"
                )
            return False

        previous_raw_index = self.geometry_last_goal_raw_index
        self.geometry_goal_binding = binding
        self.geometry_active_span = binding.active_span
        if previous_raw_index != binding.raw_path_index:
            hint = binding.active_span.first_segment_index
            self.geometry_progress_tracker.reset(
                GeometryResetReason.ACTIVE_GOAL_ADVANCED,
                progress_s=binding.active_span.start_s,
                hint_segment_index=hint,
            )
            self._record_geometry_reset(GeometryResetReason.ACTIVE_GOAL_ADVANCED)
            self.geometry_last_projection = None
            self.geometry_last_projection_cycle_token = None
            self._reset_precision_regulator(
                "ACTIVE_GOAL_ADVANCED",
                progress_s=binding.active_span.start_s,
            )
        self.geometry_last_goal_raw_index = binding.raw_path_index
        return True

    def odom_callback(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        quaternion = (
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        )
        speed_x = float(linear.x)
        speed_y = float(linear.y)
        yaw_rate = float(angular.z)
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

        if (
            self.geometry_processing_enabled
            and self.geometry_progress_tracker is not None
            and self.geometry_last_odom_point is not None
            and math.hypot(
                x - self.geometry_last_odom_point[0],
                y - self.geometry_last_odom_point[1],
            )
            > self.geometry_localization_jump_reset
        ):
            self.geometry_progress_tracker.reset(GeometryResetReason.LOCALIZATION_JUMP)
            self._record_geometry_reset(GeometryResetReason.LOCALIZATION_JUMP)
            self.geometry_last_projection = None
            self.geometry_last_projection_cycle_token = None
            self._reset_precision_regulator("LOCALIZATION_JUMP", progress_s=0.0)
            self._reset_precision_tracking(
                "LOCALIZATION_JUMP",
                reset_metrics=False,
                path_identity=self.geometry_installed_signature,
            )
            self.precision_tracking_metrics.note_discontinuity("LOCALIZATION_JUMP")
            self._reset_precision_pivot(
                "LOCALIZATION_JUMP",
                clear_anchor=True,
            )
            self._reset_precision_terminal("LOCALIZATION_JUMP")
            self._reset_legacy_alignment_lifecycle("LOCALIZATION_JUMP")
            self.get_logger().warn(
                "PRECISION GEOMETRY PROGRESS RESET | reason=LOCALIZATION_JUMP"
            )
        if (
            self.geometry_processing_enabled
            and self.geometry_progress_tracker is not None
        ):
            self.geometry_last_odom_point = (x, y)

        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw
        self.current_speed_mps = math.hypot(
            speed_x,
            speed_y,
        )
        # MAVROS odometry twist and pose share one callback/freshness stamp.
        # Whether angular.z is the physical chassis yaw rate at pivot dynamics
        # remains a field-validation item and is exposed in pivot diagnostics.
        self.current_yaw_rate_radps = yaw_rate if math.isfinite(yaw_rate) else math.inf
        self.last_odom_time = self.get_clock().now()

    def nav_path_callback(self, msg):
        """Receive the complete retained 50 mm navigation trajectory.

        /nav_path owns path geometry. Mission Manager /segment_goal owns only
        the current semantic stop endpoint (marking or extension/dummy).
        """
        frame = msg.header.frame_id.strip()
        if frame and frame != self.local_frame:
            self.get_logger().error(
                f"IGNORED /nav_path FRAME {frame!r}; expected {self.local_frame!r}"
            )
            return

        points = []
        for pose in msg.poses:
            x = float(pose.pose.position.x)
            y = float(pose.pose.position.y)
            if not all(math.isfinite(value) for value in (x, y)):
                self.get_logger().error("IGNORED /nav_path WITH NON-FINITE POINT")
                return
            points.append((x, y))

        if not points:
            self.geometry_pending_nav_points = None
            if self.geometry_processing_enabled:
                self._invalidate_installed_geometry(GeometryResetReason.SOURCE_CLEARED)
            self.nav_path_points = []
            self.nav_path_received = False
            self.nav_path_segment_start_index = 0
            self.nav_path_cursor_index = 0
            self.nav_path_goal_index = None
            self.nav_path_lookahead_index = None
            self.get_logger().warn("/nav_path CLEARED / RPP PATH HOLD")
            return

        self.geometry_pending_nav_points = list(points)
        self.nav_path_points = points
        self.nav_path_received = True
        self.nav_path_segment_start_index = 0
        self.nav_path_cursor_index = 0
        self.nav_path_goal_index = None
        self.nav_path_lookahead_index = None

        # If the retained path arrives after the semantic goal, bind it now.
        if self.segment_goal_x is not None and self.segment_goal_y is not None:
            fresh_p1 = (
                self.segment_goal_number == 1 and not self.first_marking_completed
            )
            self.bind_nav_path_goal(
                self.segment_start_x,
                self.segment_start_y,
                self.segment_goal_x,
                self.segment_goal_y,
                fresh_p1=fresh_p1,
            )

        self.get_logger().warn(
            f"/nav_path RECEIVED | points={len(points)} | "
            f"lookahead={self.nav_path_lookahead:.2f}m"
        )
        self._try_install_path_geometry()

    def find_nav_path_index(self, x, y, start_index=0, end_index=None):
        if not self.nav_path_points:
            return None
        start = max(0, int(start_index))
        end = (
            len(self.nav_path_points) - 1
            if end_index is None
            else min(
                len(self.nav_path_points) - 1,
                int(end_index),
            )
        )
        if end < start:
            return None

        best_index = None
        best_distance = math.inf
        tolerance = max(self.waypoint_match_tolerance, 0.002)
        for index in range(start, end + 1):
            px, py = self.nav_path_points[index]
            distance = math.hypot(px - x, py - y)
            if distance <= tolerance and distance < best_distance:
                best_index = index
                best_distance = distance
        return best_index

    def nearest_nav_path_index(self, start_index, end_index):
        if not self.nav_path_points or self.current_x is None or self.current_y is None:
            return None
        start = max(0, int(start_index))
        end = min(len(self.nav_path_points) - 1, int(end_index))
        if end < start:
            return None
        best_index = start
        best_distance = math.inf
        for index in range(start, end + 1):
            px, py = self.nav_path_points[index]
            distance = math.hypot(px - self.current_x, py - self.current_y)
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index

    def bind_nav_path_goal(
        self,
        start_x,
        start_y,
        goal_x,
        goal_y,
        *,
        fresh_p1=False,
    ):
        """Bind one semantic goal to its exact /nav_path segment."""
        if not self.nav_path_received or not self.nav_path_points:
            self.nav_path_goal_index = None
            return False

        goal_index = self.find_nav_path_index(goal_x, goal_y)
        if goal_index is None:
            self.nav_path_goal_index = None
            self.get_logger().error(
                "SEMANTIC GOAL NOT FOUND IN /nav_path / SAFE PATH HOLD | "
                f"goal=({goal_x:.3f},{goal_y:.3f})"
            )
            return False

        if fresh_p1:
            start_index = 0
        elif start_x is not None and start_y is not None:
            start_index = self.find_nav_path_index(
                start_x,
                start_y,
                start_index=0,
                end_index=goal_index,
            )
            if start_index is None:
                start_index = self.nearest_nav_path_index(0, goal_index)
        else:
            start_index = self.nearest_nav_path_index(0, goal_index)

        if start_index is None:
            start_index = 0
        start_index = min(start_index, goal_index)

        self.nav_path_segment_start_index = start_index
        self.nav_path_goal_index = goal_index
        self.nav_path_cursor_index = min(start_index + 1, goal_index)
        self.nav_path_lookahead_index = self.nav_path_cursor_index

        self.get_logger().warn(
            "NAV_PATH SEGMENT BOUND | "
            f"start_idx={start_index} | goal_idx={goal_index} | "
            f"goal=({goal_x:.3f},{goal_y:.3f})"
        )
        return True

    def nav_path_tracking_solution(self, goal_x, goal_y):
        """Select legacy cursor or synchronized projection geometry."""

        if not self.geometry_tracking_enabled:
            # Run the production cursor path first and return that exact result.
            # Optional diagnostics are strictly shadow evaluation and cannot
            # veto or replace legacy motion.
            legacy_solution = self._legacy_nav_path_tracking_solution(goal_x, goal_y)
            if self.geometry_diagnostics_enabled:
                try:
                    self._geometry_tracking_solution(goal_x, goal_y, shadow=True)
                except Exception as error:  # Observability must not stop control.
                    # Do not retry the failed diagnostics hook here. Even the
                    # failure logger is isolated so the legacy command result
                    # below remains unconditional.
                    try:
                        self.get_logger().error(
                            "PRECISION GEOMETRY SHADOW EVALUATION FAILED | "
                            f"type={type(error).__name__}"
                        )
                    except Exception:
                        pass
            return legacy_solution
        return self._geometry_tracking_solution(goal_x, goal_y, shadow=False)

    def _publish_geometry_debug(self, *, projection=None, lookup=None, status):
        if not self.geometry_processing_enabled:
            return
        allowed_statuses = {
            "CONTRACT_UNAVAILABLE",
            "GOAL_COORDINATE_MISMATCH",
            "INTERNAL_ERROR",
            "PROJECTION_REJECTED",
            "SHADOW_READY",
            "TANGENT_UNAVAILABLE",
            "TRACKING_READY",
        }
        if status not in allowed_statuses:
            status = "INTERNAL_ERROR"
        tracker = self.geometry_progress_tracker
        binding = self.geometry_goal_binding
        active_span = self.geometry_active_span
        nearest_raw_index = None
        if projection is not None:
            nearest_raw_index = (
                projection.raw_start_index
                if projection.t < 0.5
                else projection.raw_end_index
            )
        payload = {
            "schema_version": 1,
            "status": status,
            "ros_time_ns": self.get_clock().now().nanoseconds,
            "geometry_tracking_enabled": self.geometry_tracking_enabled,
            "geometry_diagnostics_enabled": self.geometry_diagnostics_enabled,
            "precision_guidance_enabled": self.precision_guidance_enabled,
            "precision_speed_control_enabled": self.precision_speed_control_enabled,
            "cycle_token": self.precision_cycle_token,
            "projection_cycle_token": (
                self.geometry_last_projection_cycle_token
                if projection is not None
                else None
            ),
            "path_signature": self.geometry_installed_signature,
            "raw_path_index": nearest_raw_index,
            "nearest_raw_index": nearest_raw_index,
            "raw_segment_start_index": (
                projection.raw_start_index if projection is not None else None
            ),
            "raw_segment_end_index": (
                projection.raw_end_index if projection is not None else None
            ),
            "geometry_segment": (
                projection.segment_index if projection is not None else None
            ),
            "projection_t": projection.t if projection is not None else None,
            "projection_x": projection.point.x if projection is not None else None,
            "projection_y": projection.point.y if projection is not None else None,
            "raw_projected_s": (
                projection.projected_s if projection is not None else None
            ),
            "projected_s": projection.progress_s if projection is not None else None,
            "cross_track_mm": (
                self.ground_xtrack(projection.signed_cross_track_m) * 1000.0
                if projection is not None
                else None
            ),
            "remaining_m": (
                projection.remaining_to_active_stop_m
                if projection is not None
                else None
            ),
            "remaining_path_m": (
                projection.remaining_path_m if projection is not None else None
            ),
            "next_corner_distance_m": (
                projection.next_corner_distance_m if projection is not None else None
            ),
            "next_corner_angle_deg": (
                math.degrees(projection.next_corner_angle_rad)
                if projection is not None
                and projection.next_corner_angle_rad is not None
                else None
            ),
            "active_goal_identity": (
                binding.active_goal_identity if binding is not None else None
            ),
            "active_span_start_raw_index": (
                active_span.start_raw_index if active_span is not None else None
            ),
            "active_span_stop_raw_index": (
                active_span.stop_raw_index if active_span is not None else None
            ),
            "active_span_first_segment": (
                active_span.first_segment_index if active_span is not None else None
            ),
            "active_span_last_segment": (
                active_span.last_segment_index if active_span is not None else None
            ),
            "geometry_reset_reason": self.geometry_last_reset_reason,
            "geometry_reset_count": self.geometry_reset_count,
            "tracker_reset_count": tracker.reset_count if tracker is not None else None,
            "monotonic_clamped": (
                projection.monotonic_clamped if projection is not None else None
            ),
            "used_full_reacquire": (
                projection.used_full_reacquire if projection is not None else None
            ),
            "lookup_target": (
                {
                    "segment": lookup.segment_index,
                    "t": lookup.t,
                    "x": lookup.point.x,
                    "y": lookup.point.y,
                    "s": lookup.s,
                }
                if lookup is not None
                else None
            ),
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.geometry_debug_pub.publish(message)

    def _geometry_tracking_solution(self, goal_x, goal_y, *, shadow):
        if not (
            self.geometry_contract_synchronized
            and self.path_geometry is not None
            and self.geometry_progress_tracker is not None
            and self.geometry_goal_binding is not None
            and self.geometry_active_span is not None
            and self.current_x is not None
            and self.current_y is not None
        ):
            self._publish_geometry_debug(status="CONTRACT_UNAVAILABLE")
            return None

        raw_goal = self.path_geometry.raw_points[
            self.geometry_goal_binding.raw_path_index
        ].point
        if math.hypot(raw_goal.x - goal_x, raw_goal.y - goal_y) > max(
            self.waypoint_match_tolerance, 0.002
        ):
            self.geometry_goal_binding = None
            self.geometry_active_span = None
            self._publish_geometry_debug(status="GOAL_COORDINATE_MISMATCH")
            return None

        try:
            projection = self.geometry_progress_tracker.update(
                (self.current_x, self.current_y),
                active_span=self.geometry_active_span,
                back_window_segments=self.geometry_back_window_segments,
                forward_window_segments=self.geometry_forward_window_segments,
                max_backward_jump_m=self.geometry_max_backward_jump,
                max_forward_jump_m=self.geometry_max_forward_jump,
                full_reacquire_distance_m=self.geometry_reacquire_distance,
            )
            lookup = self.path_geometry.lookahead_target(
                projection.progress_s,
                self.nav_path_lookahead,
                active_span=self.geometry_active_span,
            )
        except (TypeError, ValueError):
            self._publish_geometry_debug(status="PROJECTION_REJECTED")
            return None

        self.geometry_last_projection = projection
        self.geometry_last_projection_cycle_token = self.precision_cycle_token
        path_bearing = None
        if projection.segment_index is not None:
            path_bearing = self.path_geometry.segments[
                projection.segment_index
            ].heading_rad
        elif lookup.heading_rad is not None:
            path_bearing = lookup.heading_rad
        else:
            dx = goal_x - self.current_x
            dy = goal_y - self.current_y
            if math.hypot(dx, dy) > self.WAYPOINT_CHANGE_EPSILON_M:
                path_bearing = math.atan2(dy, dx)
            elif self.current_yaw is not None:
                path_bearing = self.current_yaw
        if path_bearing is None:
            self._publish_geometry_debug(
                projection=projection,
                lookup=lookup,
                status="TANGENT_UNAVAILABLE",
            )
            return None

        self._publish_geometry_debug(
            projection=projection,
            lookup=lookup,
            status="SHADOW_READY" if shadow else "TRACKING_READY",
        )
        geometry_segment = (
            projection.segment_index if projection.segment_index is not None else 0
        )
        lookup_segment = (
            lookup.segment_index
            if lookup.segment_index is not None
            else geometry_segment
        )
        return (
            projection.point.x,
            projection.point.y,
            path_bearing,
            geometry_segment,
            lookup_segment,
            self.geometry_goal_binding.raw_path_index,
        )

    def _legacy_nav_path_tracking_solution(self, goal_x, goal_y):
        """Advance through 50 mm points and return local tangent + lookahead.

        Returns:
          (line_x, line_y, path_bearing, cursor_index,
           lookahead_index, goal_index)

        Interpolated points never stop the rover. The cursor advances when the
        rover enters the point-reach radius or passes the point plane. The
        lookahead point is selected by accumulated path distance.
        """
        if not self.nav_path_received or not self.nav_path_points:
            return None

        goal_index = self.nav_path_goal_index
        goal_matches = False
        if goal_index is not None and 0 <= goal_index < len(self.nav_path_points):
            gx, gy = self.nav_path_points[goal_index]
            goal_matches = math.hypot(gx - goal_x, gy - goal_y) <= max(
                self.waypoint_match_tolerance, 0.002
            )

        if not goal_matches:
            fresh_p1 = (
                self.segment_goal_number == 1 and not self.first_marking_completed
            )
            if not self.bind_nav_path_goal(
                self.segment_start_x,
                self.segment_start_y,
                goal_x,
                goal_y,
                fresh_p1=fresh_p1,
            ):
                return None
            goal_index = self.nav_path_goal_index

        if goal_index is None:
            return None

        cursor = max(
            self.nav_path_segment_start_index,
            min(self.nav_path_cursor_index, goal_index),
        )

        # Progress through dense 50 mm points without any intermediate stop.
        while cursor < goal_index:
            px, py = self.nav_path_points[cursor]
            distance_to_point = math.hypot(
                px - self.current_x,
                py - self.current_y,
            )

            previous_index = max(
                self.nav_path_segment_start_index,
                cursor - 1,
            )
            ax, ay = self.nav_path_points[previous_index]
            sx = px - ax
            sy = py - ay
            length = math.hypot(sx, sy)
            passed_point = False
            if length > self.WAYPOINT_CHANGE_EPSILON_M:
                ux = sx / length
                uy = sy / length
                passed_point = (
                    (self.current_x - px) * ux + (self.current_y - py) * uy
                ) >= 0.0

            if distance_to_point <= self.nav_path_point_reach or passed_point:
                cursor += 1
                continue
            break

        self.nav_path_cursor_index = cursor

        # Select a path-distance lookahead, capped at the semantic goal.
        lookahead_index = cursor
        accumulated = math.hypot(
            self.nav_path_points[cursor][0] - self.current_x,
            self.nav_path_points[cursor][1] - self.current_y,
        )
        while lookahead_index < goal_index and accumulated < self.nav_path_lookahead:
            nx = lookahead_index + 1
            x0, y0 = self.nav_path_points[lookahead_index]
            x1, y1 = self.nav_path_points[nx]
            accumulated += math.hypot(x1 - x0, y1 - y0)
            lookahead_index = nx
        self.nav_path_lookahead_index = lookahead_index

        # Local path tangent is derived from /nav_path itself. This is the
        # bearing RPP follows; Mission Manager quaternion is never consulted.
        tangent_from = max(
            self.nav_path_segment_start_index,
            cursor - 1,
        )
        tangent_to = min(goal_index, max(cursor, tangent_from + 1))

        if tangent_to > tangent_from:
            ax, ay = self.nav_path_points[tangent_from]
            bx, by = self.nav_path_points[tangent_to]
            dx = bx - ax
            dy = by - ay
            if math.hypot(dx, dy) > self.WAYPOINT_CHANGE_EPSILON_M:
                path_bearing = math.atan2(dy, dx)
            else:
                path_bearing = None
        else:
            path_bearing = None

        if path_bearing is None:
            dx = goal_x - self.current_x
            dy = goal_y - self.current_y
            if math.hypot(dx, dy) > self.WAYPOINT_CHANGE_EPSILON_M:
                path_bearing = math.atan2(dy, dx)
            elif self.current_yaw is not None:
                path_bearing = self.current_yaw
            else:
                return None

        # Cross-track line point is the current dense path cursor. The farther
        # lookahead index remains available for monitoring and future curved
        # path regulation without allowing interpolation points to stop motion.
        line_x, line_y = self.nav_path_points[cursor]
        return (
            line_x,
            line_y,
            path_bearing,
            cursor,
            lookahead_index,
            goal_index,
        )

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
            self.geometry_pending_marking_waypoints = None
            if self.geometry_processing_enabled:
                self._invalidate_installed_geometry(GeometryResetReason.SOURCE_CLEARED)
            return

        self.geometry_pending_marking_waypoints = list(points)

        previous_p1 = self.marking_waypoints[0] if self.marking_waypoints else None
        self.marking_waypoints = points
        self.marking_metadata_received = True
        self._try_install_path_geometry()

        # Reclassify an already-received semantic goal if retained marking
        # metadata arrives after /segment_goal. This closes a startup DDS
        # ordering race without using Mission Manager orientation.
        if self.segment_goal_x is not None and self.segment_goal_y is not None:
            detected_number = self.find_marking_number(
                self.segment_goal_x,
                self.segment_goal_y,
            )
            if detected_number != self.segment_goal_number:
                self.segment_goal_number = detected_number
                self.target_is_marking = detected_number > 0
                self.get_logger().warn(
                    "SEMANTIC GOAL RECLASSIFIED AFTER MARKING METADATA | "
                    + (
                        f"P{detected_number}"
                        if detected_number > 0
                        else "EXTENSION/DUMMY"
                    )
                )

        if (
            previous_p1 is None
            or math.hypot(
                previous_p1[0] - points[0][0],
                previous_p1[1] - points[0][1],
            )
            > self.waypoint_match_tolerance
        ):
            self.first_marking_completed = False
            self.first_marking_hold_seen = False
            self.c_line_locked = False
            self.c_line_bearing = None

        if self.mission_enabled and not self.first_marking_completed:
            self.lock_c_to_p1_line("marking metadata received")

    def target_matches_marking(self, x, y):
        for marking_x, marking_y in self.marking_waypoints:
            if (
                math.hypot(
                    x - marking_x,
                    y - marking_y,
                )
                <= self.waypoint_match_tolerance
            ):
                return True
        return False

    def waypoint_callback(self, msg):
        """Receive the semantic goal published by Mission Manager.

        Mission Manager intentionally publishes an identity quaternion. RPP
        therefore NEVER reads pose.orientation as path heading. /segment_goal
        owns the fixed incoming-segment bearing calculation.
        """
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
            )
            > self.WAYPOINT_CHANGE_EPSILON_M
        )

        self.target_x = x
        self.target_y = y
        self.target_is_marking = self.target_matches_marking(x, y)
        self.last_waypoint_time = self.get_clock().now()

        if changed:
            # DDS ordering between /active_waypoint and /segment_goal is not
            # guaranteed. Clear a stale bearing only when the current segment
            # goal is still a different coordinate. If /segment_goal already
            # arrived first, preserve the newly calculated bearing.
            segment_matches = (
                self.segment_goal_x is not None
                and self.segment_goal_y is not None
                and math.hypot(
                    x - self.segment_goal_x,
                    y - self.segment_goal_y,
                )
                <= self.WAYPOINT_CHANGE_EPSILON_M
            )
            if not segment_matches:
                self.target_path_bearing = None

            self.get_logger().info(
                "NEW MARKING SEMANTIC TARGET"
                if self.target_is_marking
                else "NEW EXTENSION/DUMMY SEMANTIC TARGET"
            )

    def segment_goal_callback(self, msg):
        """Latch a semantic stop goal and calculate its incoming bearing.

        Bearing ownership is entirely inside RPP:
          C -> P1
          P1 -> P2
          P1 -> EXT -> P2
          P2 -> P3 ...
        The Mission Manager quaternion is intentionally ignored.
        """
        if msg.header.frame_id.strip() != self.local_frame:
            return
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        if not all(math.isfinite(value) for value in (x, y)):
            return

        previous_x = self.segment_goal_x
        previous_y = self.segment_goal_y
        previous_number = self.segment_goal_number
        previous_was_extension = (
            previous_x is not None and previous_y is not None and previous_number == 0
        )
        previous_extension_captured = (
            previous_was_extension and self.marking_stop_latched
        )

        changed = (
            previous_x is None
            or previous_y is None
            or math.hypot(
                x - previous_x,
                y - previous_y,
            )
            > self.WAYPOINT_CHANGE_EPSILON_M
        )

        new_number = self.find_marking_number(x, y)

        # Make /segment_goal sufficient by itself. This also removes any
        # transient dependence on cross-topic DDS delivery ordering.
        self.segment_goal_x = x
        self.segment_goal_y = y
        self.segment_goal_number = new_number
        self.target_x = x
        self.target_y = y
        self.target_is_marking = new_number > 0
        self.last_waypoint_time = self.get_clock().now()
        # The Pose and additive metadata are separate volatile topics. Bind
        # whichever arrived second; an old metadata/pose pair cannot survive
        # the coordinate and raw-index validation.
        self.geometry_goal_binding = None
        self.geometry_active_span = None
        self._try_bind_geometry_goal(log_error=False)

        if not changed:
            return

        # Choose the fixed incoming segment start. P1 of a fresh mission owns
        # a C->P1 line from the current rover pose. Every later semantic goal
        # uses the previous semantic goal, including extension->P2.
        fresh_p1 = new_number == 1 and previous_number != 1
        if fresh_p1 or previous_x is None or previous_y is None:
            start_x = self.current_x
            start_y = self.current_y
        else:
            start_x = previous_x
            start_y = previous_y

        if start_x is None or start_y is None:
            self.segment_start_x = None
            self.segment_start_y = None
            self.target_path_bearing = None
        else:
            delta_east = x - start_x
            delta_north = y - start_y
            segment_length = math.hypot(delta_east, delta_north)
            self.segment_start_x = start_x
            self.segment_start_y = start_y
            if segment_length > self.WAYPOINT_CHANGE_EPSILON_M:
                self.target_path_bearing = math.atan2(
                    delta_north,
                    delta_east,
                )
            else:
                self.target_path_bearing = None

        # Returning from a later point to P1 represents a new mission.
        if new_number == 1 and previous_number != 1:
            self.first_marking_completed = False
            self.first_marking_hold_seen = False
            self.c_line_locked = False
            self.c_line_bearing = None

        # If the just-finished semantic goal was an extension/dummy captured
        # inside 30 mm, hold zero until measured speed is <=0.01 m/s before
        # releasing the newly received next segment.
        if previous_extension_captured:
            self.post_extension_stationary_hold = True
            self.get_logger().warn(
                "EXTENSION 30MM CAPTURED / WAITING FOR STATIONARY HANDOFF"
            )

        # Bind this semantic endpoint to the exact 50 mm /nav_path segment.
        # target_path_bearing above is retained only as a diagnostic fallback;
        # active control derives its tangent from /nav_path every cycle.
        self.bind_nav_path_goal(
            self.segment_start_x,
            self.segment_start_y,
            x,
            y,
            fresh_p1=fresh_p1,
        )

        # Every semantic segment (marking OR extension/dummy) must complete
        # heading/cross-track alignment before translational drive is released.
        self.segment_alignment_active = True
        self.segment_runtime_reanchored = False
        self._reset_legacy_alignment_lifecycle("SEGMENT_GOAL_CHANGED")
        self.alignment_forward_heading_recovery_active = False
        self.alignment_deadband_recovery_active = False
        self.alignment_inside_since = None
        self.alignment_release_x = None
        self.alignment_release_y = None
        self.xtrack_priority_active = False
        self.xtrack_priority_inside_since = None
        self.reset_xtrack_damping_state()
        self.reset_speed_profiles()
        goal_progress_s = (
            self.geometry_active_span.start_s
            if self.geometry_active_span is not None
            else 0.0
        )
        self._reset_precision_regulator(
            "SEGMENT_GOAL_CHANGED",
            progress_s=goal_progress_s,
        )
        self._reset_precision_pivot("SEGMENT_GOAL_CHANGED", clear_anchor=True)
        self._reset_precision_terminal("SEGMENT_GOAL_CHANGED")
        if self.segment_start_x is not None and self.segment_start_y is not None:
            anchor_identity = (
                "C_TO_P1_START"
                if fresh_p1
                else f"SEMANTIC_SEGMENT_START_TO_{new_number or 'EXT'}"
            )
            self._latch_precision_pivot_anchor(
                self.segment_start_x,
                self.segment_start_y,
                anchor_identity,
                target_bearing=self.target_path_bearing,
            )

        self.marking_missed = False
        self.capture_monitor_armed = False
        self.closest_marking_distance = math.inf
        self.marking_stop_latched = False
        self.marking_stop_latched_at = None
        self.marking_stop_trigger_radius = None
        self._terminal_result_sent = None
        self.terminal_gate_inside_since = None
        self.terminal_gate_ready = False
        self.reset_terminal_precision_state()
        self.reset_terminal_native_pivot()

        goal_label = f"P{new_number}" if new_number > 0 else "EXTENSION/DUMMY"
        bearing_label = (
            f"{math.degrees(self.target_path_bearing):.1f}deg"
            if self.target_path_bearing is not None
            else "pending"
        )

        if new_number == 1 and not self.first_marking_completed:
            if self.mission_enabled:
                self.lock_c_to_p1_line("segment goal received")
            self.get_logger().warn(
                "SEMANTIC GOAL ACTIVE | P1 | "
                "C->P1 direct precision approach | "
                f"bearing={bearing_label}"
            )
        else:
            self.get_logger().warn(
                f"SEMANTIC GOAL ACTIVE | {goal_label} | "
                "500MM DECEL + 30MM ZERO CAPTURE | "
                f"bearing={bearing_label}"
            )

    def find_marking_number(self, x, y):
        for index, (marking_x, marking_y) in enumerate(
            self.marking_waypoints,
            start=1,
        ):
            if (
                math.hypot(
                    x - marking_x,
                    y - marking_y,
                )
                <= self.waypoint_match_tolerance
            ):
                return index
        return 0

    def mission_enable_callback(self, msg):
        enabled = bool(msg.data)
        previous = self.mission_enabled
        if enabled != previous:
            self.get_logger().warn("MISSION ENABLED" if enabled else "MISSION DISABLED")
        self.mission_enabled = enabled

        if enabled and not previous:
            self.precision_tracking_mission_sequence += 1
            self.precision_tracking_mission_identity = (
                f"mission_run:{self.precision_tracking_mission_sequence}"
            )
            self._reset_precision_regulator("MISSION_ENABLED", progress_s=0.0)
            self._reset_precision_tracking(
                "MISSION_ENABLED",
                reset_metrics=True,
                path_identity=self.geometry_installed_signature,
            )
            self._reset_precision_pivot("MISSION_ENABLED", clear_anchor=True)
            self._reset_precision_terminal("MISSION_ENABLED")
            self.segment_alignment_active = True
            self._reset_legacy_alignment_lifecycle("MISSION_ENABLED")
            self.terminal_gate_inside_since = None
            self.terminal_gate_ready = False
            if not self.first_marking_completed:
                self.c_line_locked = False
                self.c_line_bearing = None
                self.c_line_reanchored_after_pivot = False
                self.lock_c_to_p1_line("mission enabled")
            anchor_x = (
                self.current_x
                if not self.first_marking_completed
                else self.segment_start_x
            )
            anchor_y = (
                self.current_y
                if not self.first_marking_completed
                else self.segment_start_y
            )
            if anchor_x is not None and anchor_y is not None:
                self._latch_precision_pivot_anchor(
                    anchor_x,
                    anchor_y,
                    (
                        "MISSION_ENABLE_C"
                        if not self.first_marking_completed
                        else "MISSION_ENABLE_SEGMENT_START"
                    ),
                    target_bearing=self.target_path_bearing,
                )
        elif not enabled:
            self.reset_motion_state()
            if not self.first_marking_completed:
                self.c_line_locked = False
                self.c_line_bearing = None

    def emergency_stop_callback(self, msg):
        active = bool(msg.data)
        previous = self.emergency_stop
        if active != previous:
            self.get_logger().warn(
                "EMERGENCY STOP ACTIVE" if active else "EMERGENCY STOP RELEASED"
            )
        self.emergency_stop = active
        if active:
            self._reset_precision_pivot("EMERGENCY_STOP", clear_anchor=True)
            self._reset_legacy_alignment_lifecycle("EMERGENCY_STOP")
            if not previous:
                self._reset_precision_terminal("EMERGENCY_STOP")
            self.publish_stop()

    def marking_active_callback(self, msg):
        """Track the active marking/spray hold.

        /marking_active means the mission manager is currently timing a hold.
        It is not a completion signal. P1 guidance is released only after a
        COMPLETED point event is received from /mission_manager/point_event.
        """
        active = bool(msg.data)
        previous = self.marking_active
        if active != previous:
            self.get_logger().warn(
                "MARKING HOLD ACTIVE" if active else "MARKING HOLD RELEASED"
            )

        if active:
            self.reset_terminal_native_pivot()
            self._reset_precision_pivot("MARKING_HOLD", clear_anchor=True)
            self._reset_legacy_alignment_lifecycle("MARKING_HOLD")

        self.marking_active = active

    def point_event_callback(self, msg):
        """Release P1 guidance after Mission Manager resolves P1."""
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().error("IGNORED INVALID MISSION POINT EVENT")
            return

        event = str(payload.get("event", "")).upper()
        try:
            point_index = int(payload.get("point_index", -1))
        except (TypeError, ValueError):
            point_index = -1

        # Every successful marking ends at an exact zero-speed hold.
        # Re-arm the distance-based acceleration profile so the NEXT leg
        # always starts gently from zero and accelerates toward cruise speed.
        if event == "COMPLETED":
            self.reset_acceleration_profile()
            self.command_slew_speed = 0.0
            self.command_slew_last_time = None
            self._reset_precision_regulator("POINT_COMPLETED", progress_s=0.0)
            self._reset_precision_pivot("POINT_COMPLETED", clear_anchor=True)
            self._reset_precision_terminal("POINT_COMPLETED")
            self._reset_legacy_alignment_lifecycle("POINT_COMPLETED")
            self.get_logger().warn(
                "MARKING COMPLETED / NEXT-LEG ACCELERATION ARMED | "
                f"point_index={point_index} | "
                f"accel_distance={self.acceleration_distance:.2f}m | "
                f"accel_rate={self.acceleration_rate:.3f}m/s^2"
            )

        if (
            should_release_first_marking(event, point_index)
            and not self.first_marking_completed
        ):
            self.first_marking_hold_seen = event == "COMPLETED"
            self.first_marking_completed = True
            self.c_line_locked = False
            self.c_line_bearing = None
            self.c_line_reanchored_after_pivot = False
            self.get_logger().warn(
                f"P1 RESOLVED ({event}) / " "P1->P2 INTERPOLATED GUIDANCE RELEASED"
            )

    def _reset_legacy_alignment_lifecycle(self, reason):
        """Reset the inner native-pivot lifecycle. Outer alignment is caller-owned."""
        if getattr(self, "legacy_alignment", None) is not None:
            self.legacy_alignment.reset(reason)
        self.segment_alignment_pivot_complete = False
        self.segment_pivot_keeper_started_at = None
        self.alignment_inside_since = None
        self.reset_terminal_native_pivot()

    def _sync_legacy_alignment_shadow(self):
        if getattr(self, "legacy_alignment", None) is None:
            self.segment_alignment_pivot_complete = False
            return
        self.segment_alignment_pivot_complete = bool(
            self.legacy_alignment.pivot_complete
        )

    def reset_motion_state(self):
        self.post_extension_stationary_hold = False
        self.segment_alignment_active = True
        self._reset_legacy_alignment_lifecycle("MOTION_STATE_RESET")
        self.alignment_forward_heading_recovery_active = False
        self.alignment_deadband_recovery_active = False
        self.alignment_inside_since = None
        self.alignment_release_x = None
        self.alignment_release_y = None
        self.xtrack_priority_active = False
        self.xtrack_priority_inside_since = None
        self.reset_xtrack_damping_state()
        self.reset_speed_profiles()
        self._reset_precision_regulator("MOTION_STATE_RESET", progress_s=0.0)
        self._reset_precision_pivot("MOTION_STATE_RESET", clear_anchor=True)
        self._reset_precision_terminal("MOTION_STATE_RESET")

        self.marking_missed = False
        self.capture_monitor_armed = False
        self.closest_marking_distance = math.inf

        self.marking_stop_latched = False
        self.marking_stop_latched_at = None
        self._terminal_result_sent = None
        self.terminal_gate_inside_since = None
        self.terminal_gate_ready = False
        self.reset_terminal_precision_state()
        self.c_line_reanchored_after_pivot = False
        self.segment_runtime_reanchored = False
        self.reset_terminal_native_pivot()

    def _precision_now_sec(self):
        return max(0.0, self.get_clock().now().nanoseconds / 1.0e9)

    def _reset_precision_pivot(self, reason, *, clear_anchor):
        """Cancel Phase-3 authority at every mission/geometry safety boundary."""
        now_sec = self._precision_now_sec()
        self.precision_pivot_fsm.reset(monotonic_time_sec=now_sec)
        self.precision_pivot_last_time_sec = now_sec
        self.precision_pivot_last_result = None
        self.precision_pivot_last_reset_reason = str(reason)
        self.precision_pivot_reset_count += 1
        self.precision_pivot_recapture_inside_since = None
        self.precision_pivot_reanchor_complete = False
        self.precision_pivot_release_certified = False
        if clear_anchor:
            self.precision_pivot_anchor_x = None
            self.precision_pivot_anchor_y = None
            self.precision_pivot_anchor_identity = None
            self.precision_pivot_target_bearing = None
        if self.precision_pivot_enabled:
            self.reset_terminal_native_pivot()

    def _latch_precision_pivot_anchor(
        self,
        x,
        y,
        identity,
        *,
        target_bearing=None,
    ):
        values = (x, y)
        if not all(
            value is not None and math.isfinite(float(value)) for value in values
        ):
            return False
        self.precision_pivot_anchor_x = float(x)
        self.precision_pivot_anchor_y = float(y)
        self.precision_pivot_anchor_identity = str(identity)
        self.precision_pivot_target_bearing = (
            float(target_bearing)
            if target_bearing is not None and math.isfinite(float(target_bearing))
            else None
        )
        self.precision_pivot_reanchor_complete = False
        self.precision_pivot_release_certified = False
        return True

    def _ensure_precision_pivot_anchor(self, path_bearing, first_approach):
        if (
            self.precision_pivot_anchor_x is not None
            and self.precision_pivot_anchor_y is not None
        ):
            self.precision_pivot_target_bearing = path_bearing
            return True
        if first_approach and self.c_line_start_x is not None:
            return self._latch_precision_pivot_anchor(
                self.c_line_start_x,
                self.c_line_start_y,
                "C_TO_P1_LATCHED_C",
                target_bearing=path_bearing,
            )
        if self.segment_start_x is not None and self.segment_start_y is not None:
            return self._latch_precision_pivot_anchor(
                self.segment_start_x,
                self.segment_start_y,
                "SEMANTIC_SEGMENT_START",
                target_bearing=path_bearing,
            )
        # Exceptional mid-leg re-entry has no trustworthy prior semantic
        # anchor.  Capturing current pose is explicit and visible in bags.
        return self._latch_precision_pivot_anchor(
            self.current_x,
            self.current_y,
            "MID_LEG_REENTRY_CURRENT_POSE",
            target_bearing=path_bearing,
        )

    def precision_pivot_carrier_command(self, true_bearing, reason):
        """Return the unchanged dynamic +/-60deg PX4 native-pivot carrier.

        Unlike terminal_native_pivot_command(), this precision-only generator
        never releases at the legacy 4deg threshold.  The measured FSM is the
        sole release authority.
        """
        true_bearing = self.normalize_angle(true_bearing)
        true_error = self.normalize_angle(true_bearing - self.current_yaw)
        if not self.terminal_native_pivot_active:
            self.terminal_native_pivot_active = True
            self.terminal_native_pivot_true_bearing = true_bearing
            self.terminal_native_pivot_reason = str(reason)
        else:
            true_bearing = self.terminal_native_pivot_true_bearing
            true_error = self.normalize_angle(true_bearing - self.current_yaw)
        turn_sign = 1.0 if true_error >= 0.0 else -1.0
        carrier_error = turn_sign * self.terminal_native_pivot_request_error
        request_bearing = self.normalize_angle(self.current_yaw + carrier_error)
        return request_bearing, true_error

    def _publish_precision_anchor_approach(self):
        """Move toward the latched anchor with a low, forward-cone command."""
        if not all(
            value is not None and math.isfinite(float(value))
            for value in (
                self.current_x,
                self.current_y,
                self.current_yaw,
                self.precision_pivot_anchor_x,
                self.precision_pivot_anchor_y,
            )
        ):
            self.publish_stop()
            return 0.0, 0.0, 0.0
        delta_east = self.precision_pivot_anchor_x - self.current_x
        delta_north = self.precision_pivot_anchor_y - self.current_y
        distance = math.hypot(delta_east, delta_north)
        if distance <= self.precision_pivot_config.pivot_anchor_tolerance_m:
            self.publish_stop()
            return 0.0, 0.0, 0.0
        desired = math.atan2(delta_north, delta_east)
        error = self.normalize_angle(desired - self.current_yaw)
        error = max(
            -self.precision_pivot_recenter_forward_cone,
            min(self.precision_pivot_recenter_forward_cone, error),
        )
        command_bearing = self.normalize_angle(self.current_yaw + error)
        speed = self.precision_pivot_recenter_speed
        north = speed * math.sin(command_bearing)
        east = speed * math.cos(command_bearing)
        return self.publish_velocity_ned(
            north,
            east,
            apply_acceleration=False,
            apply_deceleration=False,
            hard_speed_cap_mps=speed,
        )

    def _publish_pivot_debug(self, result, *, anchor_error, heading_error):
        """Best-effort diagnostics; serialization/DDS cannot escape a tick."""
        try:
            payload = {
                "enabled": bool(self.precision_pivot_enabled),
                "state": result.state.value if result is not None else None,
                "previous_state": (
                    result.previous_state.value if result is not None else None
                ),
                "directive": result.directive.value if result is not None else None,
                "transition_reason": (
                    result.transition_reason if result is not None else ""
                ),
                "anchor_x": self.precision_pivot_anchor_x,
                "anchor_y": self.precision_pivot_anchor_y,
                "anchor_identity": self.precision_pivot_anchor_identity,
                "anchor_radial_error_m": anchor_error,
                "heading_error_deg": math.degrees(heading_error),
                "measured_speed_mps": self.current_speed_mps,
                "measured_yaw_rate_radps": self.current_yaw_rate_radps,
                "odom_twist_yaw_rate_field_validation_required": True,
                "telemetry_fresh": (
                    result.stop_certificate.telemetry_fresh
                    if result is not None
                    else False
                ),
                "stop_certificate_valid": (
                    result.stop_certificate.valid if result is not None else False
                ),
                "release_certificate_valid": (
                    result.release_certificate.valid if result is not None else False
                ),
                "release_certified_before_recapture": bool(
                    self.precision_pivot_release_certified
                ),
                "max_pivot_drift_m": (
                    result.max_pivot_drift_m if result is not None else 0.0
                ),
                "recenter_attempts": (
                    result.recenter_attempts if result is not None else 0
                ),
                "reset_reason": self.precision_pivot_last_reset_reason,
                "reset_count": self.precision_pivot_reset_count,
            }
            message = String()
            message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
            self.pivot_debug_pub.publish(message)
        except Exception as error:  # diagnostics are never control authority
            self.get_logger().error(f"PIVOT DEBUG PUBLISH FAILED | {error}")

    def _run_precision_pivot_alignment(
        self,
        *,
        path_bearing,
        alignment_guidance_bearing,
        alignment_cross_track,
        target_x,
        target_y,
        first_approach,
    ):
        """Run one Phase-3 measured pivot cycle and publish its directive."""
        if not self._ensure_precision_pivot_anchor(path_bearing, first_approach):
            self.publish_stop()
            return True

        if (
            self.precision_pivot_fsm.state is MotionState.RECAPTURE
            and first_approach
            and self.precision_pivot_reanchor_complete
            and self.c_line_bearing is not None
        ):
            path_bearing = self.c_line_bearing
            alignment_guidance_bearing, alignment_cross_track = self.line_guidance(
                path_bearing,
                target_x,
                target_y,
                self.segment_alignment_correction_limit,
            )

        now_sec = self._precision_now_sec()
        if now_sec < self.precision_pivot_last_time_sec:
            self._reset_precision_pivot("ROS_TIME_REGRESSION", clear_anchor=False)
            now_sec = self._precision_now_sec()
        self.precision_pivot_last_time_sec = now_sec
        anchor_error = math.hypot(
            self.current_x - self.precision_pivot_anchor_x,
            self.current_y - self.precision_pivot_anchor_y,
        )
        heading_error = self.normalize_angle(path_bearing - self.current_yaw)
        telemetry_fresh = self.is_fresh(
            self.last_odom_time,
            self.precision_pivot_telemetry_timeout_sec,
        ) and math.isfinite(self.current_yaw_rate_radps)

        # Invalid/stale measured motion evidence is a hard adapter stop.  Do
        # not even ask the pure FSM for a motion directive because no carrier,
        # anchor approach, realign, or recapture command may be mapped without
        # current measured yaw-rate evidence.
        if not telemetry_fresh:
            self._publish_pivot_debug(
                None,
                anchor_error=anchor_error,
                heading_error=heading_error,
            )
            self.publish_stop()
            return True

        recapture_complete = False
        if self.precision_pivot_fsm.state is MotionState.RECAPTURE:
            geometry_ok = (
                abs(alignment_cross_track) <= self.precision_pivot_recapture_xtrack
                and abs(heading_error) <= self.precision_pivot_recapture_heading
                and telemetry_fresh
                and self.precision_pivot_release_certified
            )
            if geometry_ok:
                if self.precision_pivot_recapture_inside_since is None:
                    self.precision_pivot_recapture_inside_since = now_sec
                recapture_complete = (
                    now_sec - self.precision_pivot_recapture_inside_since
                    >= self.precision_pivot_recapture_settle_sec
                )
            else:
                self.precision_pivot_recapture_inside_since = None

        try:
            result = self.precision_pivot_fsm.step(
                PivotMotionInput(
                    monotonic_time_sec=now_sec,
                    dt_sec=self.precision_cycle_dt_sec,
                    anchor_radial_error_m=anchor_error,
                    measured_linear_speed_mps=self.current_speed_mps,
                    measured_yaw_rate_radps=self.current_yaw_rate_radps,
                    heading_error_rad=heading_error,
                    telemetry_fresh=telemetry_fresh,
                    pivot_requested=True,
                    brake_to_anchor_requested=True,
                    recapture_complete=recapture_complete,
                )
            )
        except (TypeError, ValueError) as error:
            self.publish_stop()
            self.get_logger().error(f"PRECISION PIVOT INPUT REJECTED / HOLD | {error}")
            return True

        self.precision_pivot_last_result = result
        if result.state is MotionState.RECAPTURE and result.release_certificate.valid:
            self.precision_pivot_release_certified = True
        self._publish_pivot_debug(
            result,
            anchor_error=anchor_error,
            heading_error=heading_error,
        )

        if result.failed or result.directive is MotionDirective.HOLD_FAIL:
            self.publish_stop()
            return True

        if result.directive in {
            MotionDirective.CORNER_APPROACH,
            MotionDirective.BRAKE_TO_ANCHOR,
            MotionDirective.RECENTER,
        }:
            self._publish_precision_anchor_approach()
            return True

        if result.directive in {MotionDirective.HOLD_ZERO}:
            self.publish_stop()
            return True

        if result.directive in {MotionDirective.PIVOT, MotionDirective.REALIGN}:
            # REALIGN holds zero once heading enters tolerance so measured
            # speed/yaw-rate evidence can accumulate without carrier chatter.
            if (
                result.directive is MotionDirective.REALIGN
                and abs(heading_error)
                <= self.precision_pivot_config.release_heading_tolerance_rad
            ):
                self.publish_stop()
                return True
            carrier_bearing, true_error = self.precision_pivot_carrier_command(
                path_bearing,
                "PRECISION-PIVOT-FSM",
            )
            speed = self.segment_alignment_speed
            north = speed * math.sin(carrier_bearing)
            east = speed * math.cos(carrier_bearing)
            self.publish_velocity_ned(
                north,
                east,
                apply_acceleration=False,
                apply_deceleration=False,
            )
            return True

        if result.directive is MotionDirective.RECAPTURE:
            # Position and measured release have been certified before this
            # re-anchor/translation point.  P1 is never re-anchored earlier.
            if first_approach and not self.precision_pivot_reanchor_complete:
                if not self.precision_pivot_release_certified:
                    self.publish_stop()
                    return True
                self.reanchor_c_to_p1_after_pivot()
                self.precision_pivot_reanchor_complete = True
                self.reset_terminal_native_pivot()
                self.publish_stop()
                return True
            if first_approach and self.c_line_bearing is not None:
                path_bearing = self.c_line_bearing
                alignment_guidance_bearing, alignment_cross_track = self.line_guidance(
                    path_bearing,
                    target_x,
                    target_y,
                    self.segment_alignment_correction_limit,
                )
            command_bearing, _ = self.limit_moving_guidance_bearing(
                alignment_guidance_bearing
            )
            speed = self.post_pivot_capture_speed
            north = speed * math.sin(command_bearing)
            east = speed * math.cos(command_bearing)
            self.publish_velocity_ned(
                north,
                east,
                apply_acceleration=False,
                apply_deceleration=False,
                hard_speed_cap_mps=speed,
            )
            return True

        if (
            result.state is MotionState.TRACK
            and result.previous_state is MotionState.RECAPTURE
        ):
            self.segment_alignment_active = False
            self.segment_alignment_pivot_complete = False
            self.segment_pivot_keeper_started_at = None
            self.reset_terminal_native_pivot()
            self.reset_speed_profiles()
            self.command_slew_speed = 0.0
            self.command_slew_last_time = None
            self._reset_precision_regulator("PRECISION_PIVOT_RECAPTURE_COMPLETE")
            self._reset_precision_tracking(
                "PRECISION_PIVOT_RECAPTURE_COMPLETE",
                reset_metrics=False,
                path_identity=self.geometry_installed_signature,
            )
            # A literal-zero boundary ensures neither legacy nor Phase-2
            # longitudinal state inherits the carrier/capture magnitude.
            self.publish_stop()
            return True

        self.publish_stop()
        return True

    def _legacy_alignment_telemetry_fresh(self):
        return (
            self.is_fresh(
                self.last_odom_time,
                self.precision_pivot_telemetry_timeout_sec,
            )
            and math.isfinite(self.current_yaw_rate_radps)
            and math.isfinite(self.current_speed_mps)
        )

    def _publish_legacy_native_carrier(
        self,
        request_bearing,
        true_error,
        alignment_cross_track,
        mode_prefix,
        target_distance,
        goal_distance,
        status_prefix,
    ):
        speed = self.segment_alignment_speed
        north = speed * math.sin(request_bearing)
        east = speed * math.cos(request_bearing)
        north, east, speed = self.publish_velocity_ned(
            north,
            east,
            apply_acceleration=False,
            apply_deceleration=False,
        )
        carrier_error = self.normalize_angle(request_bearing - self.current_yaw)
        self.log_control(
            mode_prefix
            + status_prefix
            + f" {self.segment_alignment_speed:.2f}MPS VECTOR"
            + f" | carrier_error={math.degrees(carrier_error):+.1f}deg"
            + f" | true_error={math.degrees(true_error):+.1f}deg"
            + f" | xtrack="
            + f"{self.ground_xtrack(alignment_cross_track) * 1000.0:+.1f}mm"
            + f" | phase={self.legacy_alignment.phase.value}",
            target_distance,
            goal_distance,
            true_error,
            speed,
            north,
            east,
        )
        return north, east, speed

    def _run_legacy_segment_alignment(
        self,
        *,
        path_bearing,
        path_heading_error,
        alignment_guidance_bearing,
        alignment_cross_track,
        target_x,
        target_y,
        goal_x,
        goal_y,
        first_approach,
        precision_guidance,
        mode_prefix,
        target_distance,
        goal_distance,
    ):
        """Map the legacy lifecycle directive and return True if the cycle is consumed."""
        native_active = False
        pivot_request_bearing = path_bearing
        true_error = path_heading_error
        telemetry_fresh = self._legacy_alignment_telemetry_fresh()
        if self.legacy_alignment.needs_native_command and telemetry_fresh:
            (
                native_active,
                pivot_request_bearing,
                true_error,
            ) = self.terminal_native_pivot_command(
                path_bearing,
                "SEGMENT-ENTRY-PIVOT-KEEPER",
            )

        previous_phase = self.legacy_alignment.phase
        result = self.legacy_alignment.step(
            LegacyAlignmentInput(
                now_sec=self._precision_now_sec(),
                telemetry_fresh=telemetry_fresh,
                measured_speed_mps=self.current_speed_mps,
                measured_yaw_rate_radps=self.current_yaw_rate_radps,
                path_heading_error_rad=path_heading_error,
                alignment_cross_track_m=alignment_cross_track,
                native_pivot_active=bool(native_active),
                first_approach=bool(first_approach),
                already_reanchored=bool(
                    self.c_line_reanchored_after_pivot
                    if first_approach
                    else self.segment_runtime_reanchored
                ),
                current_x=self.current_x,
                current_y=self.current_y,
            )
        )
        self._sync_legacy_alignment_shadow()
        if result.reset_native_carrier:
            self.reset_terminal_native_pivot()
        if result.phase is not previous_phase:
            self.get_logger().warn(
                "LEGACY ALIGNMENT "
                f"{previous_phase.value}->{result.phase.value} | "
                f"reason={result.transition_reason} | "
                f"heading={math.degrees(path_heading_error):+.1f}deg | "
                f"xtrack={self.ground_xtrack(alignment_cross_track) * 1000.0:+.1f}mm"
            )

        if result.directive is LegacyAlignmentDirective.SAFETY_HOLD:
            self.publish_stop()
            self.get_logger().error(
                "LEGACY ALIGNMENT LOCAL SAFETY HOLD | "
                f"phase={result.phase.value} | "
                f"reason={result.transition_reason} | "
                f"heading={math.degrees(path_heading_error):+.1f}deg | "
                f"xtrack={self.ground_xtrack(alignment_cross_track) * 1000.0:+.1f}mm"
            )
            return True

        if result.directive is LegacyAlignmentDirective.NATIVE_CARRIER:
            if result.warn_native_timeout:
                self.get_logger().warn(
                    "PX4 PIVOT KEEPER TIMEOUT EXCEEDED / CONTINUING PIVOT | "
                    f"true_error={math.degrees(true_error):+.1f}deg | "
                    f"xtrack={self.ground_xtrack(alignment_cross_track) * 1000.0:+.1f}mm"
                )
            if native_active:
                try:
                    self._publish_legacy_native_carrier(
                        pivot_request_bearing,
                        true_error,
                        alignment_cross_track,
                        mode_prefix,
                        target_distance,
                        goal_distance,
                        "PX4 PIVOT KEEPER / NATIVE TURN HELD",
                    )
                except Exception as error:
                    self.publish_stop()
                    self.get_logger().error(
                        f"NATIVE CARRIER PUBLISH FAILED / NO ACK | {error}"
                    )
                    return True
                self.legacy_alignment.ack_native_carrier_published()
                if first_approach and self.c_line_reanchored_after_pivot:
                    # A new pivot can displace the rover away from the prior
                    # C'->P1 anchor. Invalidate that stale reanchor without
                    # touching c_line_bearing: the old line remains the pivot
                    # heading target until this pivot finishes and reanchors.
                    self.c_line_reanchored_after_pivot = False
            else:
                self.publish_stop()
            return True

        if result.directive is LegacyAlignmentDirective.HOLD_ZERO:
            self.publish_stop()
            return True

        if result.directive is LegacyAlignmentDirective.REANCHOR_ZERO:
            if first_approach:
                success = bool(self.reanchor_c_to_p1_after_pivot())
            else:
                success = bool(
                    self.reanchor_runtime_path_after_pivot(goal_x, goal_y)
                )
            if success:
                self.legacy_alignment.ack_reanchor_completed()
                self.reset_terminal_native_pivot()
                self._reset_precision_regulator("PIVOT_COMPLETE_RECAPTURE_ARMED")
                self.publish_stop()
                return True
            if not first_approach:
                # Nothing to reanchor to (goal already inside
                # waypoint_tolerance, or the path build declined). Fall back
                # to the pre-existing /nav_path behaviour for this leg
                # instead of stopping the mission: this feature may only ever
                # improve on the old path, never halt where it used to drive.
                self.legacy_alignment.ack_reanchor_completed()
                self.segment_runtime_reanchored = True
                self.publish_stop()
                self.get_logger().warn(
                    "POST-PIVOT LEG REANCHOR DECLINED / "
                    "CONTINUING ON /nav_path | "
                    f"goal=P{self.segment_goal_number} | "
                    f"xtrack="
                    f"{self.ground_xtrack(alignment_cross_track) * 1000.0:+.1f}mm"
                )
                return True
            self.legacy_alignment.enter_safety_hold("REANCHOR_FAILED")
            self.publish_stop()
            self.get_logger().error(
                "C-PRIME REANCHOR FAILED / LOCAL SAFETY HOLD | "
                f"heading={math.degrees(path_heading_error):+.1f}deg | "
                f"xtrack={self.ground_xtrack(alignment_cross_track) * 1000.0:+.1f}mm"
            )
            return True

        if result.directive is LegacyAlignmentDirective.COMPLETE_ZERO:
            self.segment_alignment_active = False
            self._reset_legacy_alignment_lifecycle("PIVOT_SETTLE_HOLD_COMPLETE")
            self.reset_speed_profiles()
            self.command_slew_speed = 0.0
            self.command_slew_last_time = None
            self.xtrack_priority_active = False
            self.xtrack_priority_inside_since = None
            self.reset_xtrack_damping_state()
            self._reset_precision_regulator("PIVOT_SETTLE_HOLD_COMPLETE")
            self._reset_precision_tracking(
                "PIVOT_SETTLE_HOLD_COMPLETE",
                reset_metrics=False,
                path_identity=self.geometry_installed_signature,
            )
            self.publish_stop()
            self.get_logger().warn(
                "PIVOT SETTLE HOLD COMPLETE / RELEASING TO PATH TRACKING | "
                f"heading={math.degrees(path_heading_error):+.1f}deg | "
                f"xtrack={self.ground_xtrack(alignment_cross_track) * 1000.0:+.1f}mm"
            )
            return True

        if result.directive is LegacyAlignmentDirective.FALLBACK_GLOBAL_XTRACK:
            self.segment_alignment_active = False
            self._reset_legacy_alignment_lifecycle("NON_PIVOT_CAPTURE_FALLBACK")
            self.xtrack_priority_active = True
            self.xtrack_priority_inside_since = None
            self.reset_xtrack_damping_state()
            self._reset_precision_regulator("PIVOT_RECAPTURE_FALLBACK")
            self.get_logger().error(
                "FAST CAPTURE XTRACK GUARD / FALLBACK TO 1.00MPS | "
                f"xtrack={self.ground_xtrack(alignment_cross_track) * 1000.0:+.1f}mm | "
                f"guard="
                f"{self.segment_fast_capture_max_cross_track * 1000.0:.1f}mm"
            )
            return False

        if result.directive is LegacyAlignmentDirective.COMPLETE_FALLTHROUGH:
            self.segment_alignment_active = False
            self._reset_legacy_alignment_lifecycle("NON_PIVOT_CAPTURE_COMPLETE")
            self.xtrack_priority_active = False
            self.xtrack_priority_inside_since = None
            self.reset_xtrack_damping_state()
            self._reset_precision_regulator("PIVOT_RECAPTURE_COMPLETE")
            self.get_logger().warn(
                "ALIGNED-START LINE CAPTURE RELEASED | "
                f"heading={math.degrees(path_heading_error):+.1f}deg | "
                f"xtrack={self.ground_xtrack(alignment_cross_track) * 1000.0:+.1f}mm | "
                f"speed={self.segment_alignment_recovery_speed:.3f}m/s"
            )
            return False

        # NON_PIVOT_CAPTURE preserves the previous aligned-start Phase B path,
        # including immediate C->P1 reanchor when no native carrier latched.
        if (
            result.phase is LegacyAlignmentPhase.NON_PIVOT_CAPTURE
            and previous_phase is LegacyAlignmentPhase.ENTRY
            and first_approach
        ):
            self.reanchor_c_to_p1_after_pivot()
            if self.c_line_bearing is not None:
                path_bearing = self.c_line_bearing
                path_heading_error = self.normalize_angle(
                    path_bearing - self.current_yaw
                )
                (
                    alignment_guidance_bearing,
                    alignment_cross_track,
                ) = self.line_guidance(
                    path_bearing,
                    goal_x,
                    goal_y,
                    self.segment_alignment_correction_limit,
                )
            self.reset_xtrack_damping_state()
            self.xtrack_priority_active = False
            self.xtrack_priority_inside_since = None
            self._reset_precision_regulator("ALIGNED_START_CAPTURE_ARMED")

        if self.precision_guidance_enabled and not self.following_runtime_line:
            alignment_guidance_bearing = (
                precision_guidance.limited_command_bearing_rad
            )
        command_bearing, command_heading_error = (
            self.limit_moving_guidance_bearing(alignment_guidance_bearing)
        )
        speed = self.segment_alignment_recovery_speed
        if self.precision_speed_control_enabled:
            speed_result = self._resolve_precision_speed_for_cycle()
            if speed_result is None:
                self.publish_stop()
                self.get_logger().error(
                    "PRECISION SPEED REJECTED / POST-PIVOT SAFE HOLD"
                )
                return True
            north, east, speed = self.publish_precision_velocity_ned(
                command_bearing,
                speed_result,
            )
        else:
            north = speed * math.sin(command_bearing)
            east = speed * math.cos(command_bearing)
            north, east, speed = self.publish_velocity_ned(
                north,
                east,
                apply_acceleration=True,
                apply_deceleration=False,
            )
            self._record_published_translational_speed(speed)
        self.log_control(
            mode_prefix
            + "ALIGNED-START LINE CAPTURE "
            + f"{self.segment_alignment_recovery_speed:.2f}MPS"
            + f" | release_heading<="
            + f"{math.degrees(self.terminal_native_pivot_release_error):.1f}deg"
            + f" | release_xtrack<="
            + f"{self.xtrack_priority_exit * 1000.0:.1f}mm"
            + f" | xtrack="
            + f"{self.ground_xtrack(alignment_cross_track) * 1000.0:+.1f}mm"
            + f" | command_error="
            + f"{math.degrees(command_heading_error):+.1f}deg",
            target_distance,
            goal_distance,
            command_heading_error,
            speed,
            north,
            east,
        )
        return True

    def reset_terminal_native_pivot(self):
        self.terminal_native_pivot_active = False
        self.terminal_native_pivot_true_bearing = None
        self.terminal_native_pivot_request_bearing = None
        self.terminal_native_pivot_reason = ""

    def reset_terminal_precision_state(self):
        self.terminal_precision_armed = False
        self.terminal_bearing_frozen = False
        self.terminal_limited_correction = 0.0
        self.terminal_correction_last_update_time = None

    def terminal_native_pivot_command(
        self,
        true_bearing,
        reason,
    ):
        """Keep PX4 in native differential pivot until the real line is aligned.

        PX4 changes from native turn to drive when the commanded velocity-vector
        error falls below its RD_TRANS_TRN_DRV threshold. Sending the true line
        bearing directly therefore creates the observed 45->12 degree moving
        transition and large cross-track drift.

        Production pivot keeper:
        - latch the real segment bearing;
        - while real heading error is above the RPP release threshold, publish
          a dynamic carrier vector terminal_native_pivot_request_error_deg
          (60deg) to the required turn side of the current yaw;
        - this keeps PX4's commanded vector error safely above its 45deg native
          turn-entry threshold, so PX4 remains in differential pivot;
        - once the REAL segment heading error is <=4deg, release the carrier and
          return the true line bearing for moving line capture.

        The carrier bearing is recomputed every controller cycle. It is never a
        travel target and must not be used after pivot release.
        """
        true_error = self.normalize_angle(true_bearing - self.current_yaw)

        if self.terminal_native_pivot_active:
            true_bearing = self.terminal_native_pivot_true_bearing
            true_error = self.normalize_angle(true_bearing - self.current_yaw)
        else:
            # Enter stationary/native pivot only when heading error is >=45 deg.
            # Once pivot is active, the existing <=4 deg release remains unchanged.
            if abs(true_error) < self.terminal_native_pivot_enter_error:
                return False, true_bearing, true_error

            self.terminal_native_pivot_active = True
            self.terminal_native_pivot_true_bearing = true_bearing
            self.terminal_native_pivot_reason = reason
            self.get_logger().warn(
                "PX4 PIVOT KEEPER LATCHED | "
                f"reason={reason} | "
                f"true_bearing={math.degrees(true_bearing):.1f}deg | "
                f"true_error={math.degrees(true_error):+.1f}deg | "
                f"carrier_error="
                f"{math.degrees(self.terminal_native_pivot_request_error):.1f}deg"
            )

        if abs(true_error) <= self.terminal_native_pivot_release_error:
            self.get_logger().warn(
                "PX4 PIVOT KEEPER RELEASED | "
                f"reason={self.terminal_native_pivot_reason} | "
                f"true_error={math.degrees(true_error):+.1f}deg"
            )
            self.reset_terminal_native_pivot()
            return False, true_bearing, true_error

        turn_sign = 1.0 if true_error > 0.0 else -1.0
        carrier_error = turn_sign * self.terminal_native_pivot_request_error
        request_bearing = self.normalize_angle(self.current_yaw + carrier_error)

        self.terminal_native_pivot_request_bearing = request_bearing

        # Internal invariant: the carrier must always stay above PX4's
        # 45-degree native pivot entry boundary.
        request_error = abs(self.normalize_angle(request_bearing - self.current_yaw))
        if request_error <= self.terminal_native_pivot_enter_error:
            raise RuntimeError("pivot keeper carrier fell below native pivot threshold")

        return True, request_bearing, true_error

    def _install_runtime_entry_path(self, start_x, start_y, p1_x, p1_y, reason):
        """Generate same-frame C->P1 at <=50 mm spacing."""
        try:
            points = build_runtime_entry_path(
                start_x,
                start_y,
                p1_x,
                p1_y,
                spacing_m=self.RUNTIME_ENTRY_SPACING_M,
            )
        except (TypeError, ValueError) as error:
            self.get_logger().error(
                "C->P1 RUNTIME PATH BUILD FAILED | "
                f"reason={reason} | error={error}"
            )
            return False

        if len(points) < 2:
            return False

        self.runtime_entry_points = list(points)
        self.runtime_entry_cursor_index = 1
        self.runtime_entry_lookahead_index = 1
        self.runtime_entry_goal_index = len(points) - 1
        return True

    def runtime_entry_tracking_solution(self, goal_x, goal_y):
        """Follow temporary START->P1; fixed /nav_path remains P1->Pn."""
        if (
            not self.runtime_entry_points
            or self.current_x is None
            or self.current_y is None
        ):
            return None

        runtime_goal_x, runtime_goal_y = self.runtime_entry_points[-1]
        if math.hypot(
            runtime_goal_x - goal_x,
            runtime_goal_y - goal_y,
        ) > max(self.waypoint_match_tolerance, 0.002):
            return None

        try:
            solution = track_runtime_entry_path(
                self.runtime_entry_points,
                current_x=self.current_x,
                current_y=self.current_y,
                cursor_index=self.runtime_entry_cursor_index,
                lookahead_m=self.nav_path_lookahead,
                point_reach_m=self.nav_path_point_reach,
                waypoint_epsilon_m=self.WAYPOINT_CHANGE_EPSILON_M,
            )
        except (TypeError, ValueError):
            return None

        if solution is None:
            return None

        (
            _target_x,
            _target_y,
            _path_bearing,
            cursor,
            lookahead,
            goal_index,
        ) = solution
        self.runtime_entry_cursor_index = cursor
        self.runtime_entry_lookahead_index = lookahead
        self.runtime_entry_goal_index = goal_index
        return solution

    def lock_c_to_p1_line(self, reason):
        """On START, create C->P1 from current PX4/MAVROS local odometry."""
        if (
            self.first_marking_completed
            or not self.marking_waypoints
            or self.current_x is None
            or self.current_y is None
        ):
            return False

        if self.c_line_locked and self.runtime_entry_points:
            return True

        start_x = float(self.current_x)
        start_y = float(self.current_y)

        p1_x, p1_y = self.marking_waypoints[0]
        delta_east = p1_x - start_x
        delta_north = p1_y - start_y
        distance = math.hypot(delta_east, delta_north)
        if distance <= 1.0e-6:
            return False

        bearing = math.atan2(delta_north, delta_east)
        if not self._install_runtime_entry_path(start_x, start_y, p1_x, p1_y, reason):
            return False

        self.c_line_start_x = start_x
        self.c_line_start_y = start_y
        self.c_line_bearing = bearing
        self.c_line_locked = True
        self.c_line_reanchored_after_pivot = False
        self.segment_alignment_active = True
        self._reset_legacy_alignment_lifecycle("C_LINE_LOCKED")

        self.get_logger().warn(
            "C->P1 LOCAL ODOM RUNTIME PATH LOCKED | "
            f"reason={reason} | "
            f"C_E={start_x:.3f} | C_N={start_y:.3f} | "
            f"P1_E={p1_x:.3f} | P1_N={p1_y:.3f} | "
            f"distance={distance:.3f}m | "
            f"points={len(self.runtime_entry_points)} | "
            f"bearing={math.degrees(bearing):.1f}deg"
        )
        return True

    def reanchor_c_to_p1_after_pivot(self):
        """After pivot settle, regenerate C'->P1 from current local odometry."""
        if (
            self.first_marking_completed
            or self.c_line_reanchored_after_pivot
            or not self.marking_waypoints
            or self.current_x is None
            or self.current_y is None
        ):
            return False

        start_x = float(self.current_x)
        start_y = float(self.current_y)
        p1_x, p1_y = self.marking_waypoints[0]
        old_bearing = self.c_line_bearing
        old_start_x = self.c_line_start_x
        old_start_y = self.c_line_start_y

        delta_east = p1_x - start_x
        delta_north = p1_y - start_y
        distance = math.hypot(delta_east, delta_north)
        if distance <= self.waypoint_tolerance:
            return False

        old_xtrack = 0.0
        if (
            old_bearing is not None
            and old_start_x is not None
            and old_start_y is not None
        ):
            old_xtrack = -math.sin(old_bearing) * (
                self.current_x - old_start_x
            ) + math.cos(old_bearing) * (self.current_y - old_start_y)

        bearing = math.atan2(delta_north, delta_east)
        if not self._install_runtime_entry_path(
            start_x,
            start_y,
            p1_x,
            p1_y,
            "post-pivot reanchor",
        ):
            return False

        self.c_line_start_x = start_x
        self.c_line_start_y = start_y
        self.c_line_bearing = bearing
        self.c_line_reanchored_after_pivot = True
        self.reset_xtrack_damping_state()
        self.xtrack_priority_active = False
        self.xtrack_priority_inside_since = None

        self.get_logger().warn(
            "C->P1 POST-PIVOT LOCAL ODOM REANCHOR + PATH REGENERATED | "
            f"old_xtrack={self.ground_xtrack(old_xtrack) * 1000.0:+.1f}mm | "
            f"C_E={start_x:.3f} | C_N={start_y:.3f} | "
            f"distance={distance:.3f}m | "
            f"points={len(self.runtime_entry_points)} | "
            f"bearing={math.degrees(bearing):.1f}deg"
        )
        return True

    def reanchor_runtime_path_after_pivot(self, goal_x, goal_y):
        """After pivot settle on a non-entry leg, rebuild the line to the goal.

        The entry leg has always done this via reanchor_c_to_p1_after_pivot();
        this is the same idea for P1->P2 and beyond, which previously stayed
        bound to the surveyed /nav_path segment they were already 180-460 mm
        off after the pivot walked them off it.

        Safe because nothing is painted between marking points: the mission's
        marking_indices fire only at the surveyed nav_path indices, so the
        inter-point path determines how the rover ARRIVES, not what it marks.
        The goal itself is never moved -- only the line taken to reach it.

        Does not touch c_line_* (entry-leg-only state) and does not modify
        /nav_path. The installed runtime path is self-invalidating:
        runtime_entry_tracking_solution() declines any path whose endpoint no
        longer matches the requested goal, so a stale one cannot be followed.
        """
        if (
            self.segment_runtime_reanchored
            or self.current_x is None
            or self.current_y is None
            or goal_x is None
            or goal_y is None
        ):
            return False

        start_x = float(self.current_x)
        start_y = float(self.current_y)
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        if distance <= self.waypoint_tolerance:
            return False

        if not self._install_runtime_entry_path(
            start_x,
            start_y,
            float(goal_x),
            float(goal_y),
            "post-pivot leg reanchor",
        ):
            return False

        self.segment_runtime_reanchored = True
        self.reset_xtrack_damping_state()
        self.xtrack_priority_active = False
        self.xtrack_priority_inside_since = None

        bearing = math.atan2(goal_y - start_y, goal_x - start_x)
        self.get_logger().warn(
            "POST-PIVOT LEG REANCHOR + PATH REGENERATED | "
            f"goal=P{self.segment_goal_number} | "
            f"E={start_x:.3f} | N={start_y:.3f} | "
            f"distance={distance:.3f}m | "
            f"points={len(self.runtime_entry_points)} | "
            f"bearing={math.degrees(bearing):.1f}deg"
        )
        return True

    def first_marking_approach_active(self):
        return first_marking_approach_is_active(
            first_marking_resolved=self.first_marking_completed,
            segment_goal_number=self.segment_goal_number,
            c_line_locked=self.c_line_locked,
            c_line_bearing_available=self.c_line_bearing is not None,
            has_marking_waypoints=bool(self.marking_waypoints),
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
            -math.sin(path_bearing) * delta_east + math.cos(path_bearing) * delta_north
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
        error = self.normalize_angle(desired_bearing - self.current_yaw)
        if abs(error) >= self.terminal_recovery_min_heading_error:
            return desired_bearing, error

        if abs(error) > 1.0e-6:
            direction = 1.0 if error > 0.0 else -1.0
        else:
            direction = fallback_direction

        error = direction * self.terminal_recovery_min_heading_error
        desired_bearing = self.normalize_angle(self.current_yaw + error)
        return desired_bearing, error

    def is_fresh(self, timestamp, timeout):
        if timestamp is None:
            return False
        age = (self.get_clock().now() - timestamp).nanoseconds / 1e9
        return age <= timeout

    def publish_motion_profile_monitor(self, command_speed):
        if self._rpp_debug_pending is not None:
            command_speed_value = self._finite_or_none(command_speed)
            self._rpp_debug_pending["command_speed_mps"] = command_speed_value
            self._rpp_debug_pending["command_valid"] = command_speed_value is not None

        acceleration_message = Bool()
        acceleration_message.data = bool(
            self.acceleration_active and not self.acceleration_complete
        )
        self.acceleration_active_pub.publish(acceleration_message)

        deceleration_message = Bool()
        deceleration_message.data = bool(
            self.deceleration_active and not self.deceleration_complete
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

        xtrack_cap_message = Bool()
        xtrack_cap_message.data = bool(self.xtrack_priority_active)
        self.xtrack_speed_cap_active_pub.publish(xtrack_cap_message)
        self._publish_float64(
            self.xtrack_speed_cap_value_pub,
            (
                self.xtrack_priority_speed
                if self.xtrack_priority_active
                else self.cruise_speed
            ),
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

    def start_acceleration_profile(self):
        self.acceleration_active = True
        self.acceleration_complete = False
        self.acceleration_start_x = self.current_x
        self.acceleration_start_y = self.current_y
        self.acceleration_progress_m = 0.0

        # Every translational start owns a fresh 0 -> cruise ramp. A PX4
        # pivot/carrier-vector command may have left command_slew_speed nonzero;
        # carrying that value into forward motion would bypass the requested
        # 200 mm acceleration. Re-synchronise the translational speed state to
        # literal zero here.
        initial_speed = 0.0
        self.acceleration_output_speed = 0.0
        self.acceleration_elapsed_sec = 0.0
        self.command_slew_speed = 0.0
        now = self.get_clock().now()
        self.command_slew_last_time = now
        self.acceleration_last_update_time = now
        self.acceleration_jump_warning_emitted = False
        self.get_logger().warn(
            "RPP 200MM ACCELERATION STARTED | "
            f"from={initial_speed:.3f}m/s -> {self.cruise_speed:.3f}m/s | "
            f"distance={self.acceleration_distance:.2f}m | "
            f"derived_accel={self.acceleration_rate:.3f}m/s^2"
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
            dt = (now - self.acceleration_last_update_time).nanoseconds / 1e9
            if not math.isfinite(dt) or dt <= 0.0:
                dt = 1.0 / self.CONTROL_HZ
            dt = min(dt, self.acceleration_max_dt_sec)
        self.acceleration_last_update_time = now
        self.acceleration_elapsed_sec += dt

        progress = self.update_acceleration_progress()

        # Bootstrap rises smoothly from zero and is capped at the configured
        # startup ceiling. This avoids a deadlock at tiny commands while still
        # preserving the 0 -> selected-cruise acceleration profile.
        bootstrap_speed = min(
            self.acceleration_startup_ceiling,
            self.acceleration_rate * self.acceleration_elapsed_sec,
        )

        # Once movement is measurable, the constant-acceleration relation
        # v^2 = 2*a*s ties the ramp to actual odometry distance. With the
        # production launch it reaches the selected fixed cruise speed at
        # approximately 0.20 m from the translational start position.
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
            self.acceleration_output_speed + 1.5 * self.acceleration_rate * dt
        )
        self.acceleration_output_speed = min(
            requested_speed,
            profile_target,
            maximum_next_speed,
        )

        if (
            progress >= self.acceleration_distance
            and self.acceleration_output_speed >= requested_speed - 1.0e-6
        ):
            self.acceleration_output_speed = requested_speed
            self.acceleration_complete = True
            self.acceleration_active = False
            self.get_logger().warn(
                "RPP 200MM ACCELERATION COMPLETE | "
                f"distance={progress:.3f}m | "
                f"time={self.acceleration_elapsed_sec:.2f}s | "
                f"speed={requested_speed:.3f}m/s"
            )

        return self.acceleration_output_speed

    def deceleration_speed_limit(
        self,
        requested_speed,
        along_remaining,
    ):
        """Apply the final 500 mm semantic-goal deceleration envelope.

        `along_remaining` is the signed distance to the exact waypoint plane.
        The commanded profile is based on distance remaining to the 30 mm
        waypoint-radius boundary:

            s_boundary = max(0, along_remaining - waypoint_tolerance)

            v = sqrt(
                deceleration_floor_speed^2
                + 2 * deceleration_rate * s_boundary
            )

        With the production launch parameters this gives:
            500 mm from semantic goal centre -> selected fixed cruise speed
             30 mm capture boundary    -> 0.15 m/s

        This function never commands zero. The exact radial <=30 mm latch in
        the main control loop remains the only normal zero-command owner.
        """
        requested_speed = min(
            max(0.0, requested_speed),
            self.cruise_speed,
            self.MAXIMUM_MOVING_SPEED_MPS,
        )
        if requested_speed <= 1.0e-9:
            return 0.0
        if not self.deceleration_enabled:
            return requested_speed

        # A zero-rate profile is only possible if cruise equals the floor.
        # With the production 0.15 m/s floor, both supported cruise settings
        # (0.40 and 1.00 m/s) use real semantic-goal deceleration.
        if not self.deceleration_required:
            self.reset_deceleration_profile()
            return requested_speed

        if along_remaining is None or not math.isfinite(along_remaining):
            self.get_logger().error(
                "TERMINAL DECELERATION DISABLED FOR CYCLE: " "invalid along distance"
            )
            return requested_speed

        distance_to_boundary = max(
            0.0,
            along_remaining - self.waypoint_tolerance,
        )

        # Start exactly 500 mm before the ORIGINAL marking coordinate.
        if along_remaining > self.deceleration_distance:
            self.reset_deceleration_profile()
            return requested_speed

        if not self.deceleration_active:
            self.deceleration_active = True
            self.deceleration_complete = False
            self.deceleration_output_speed = requested_speed
            self.deceleration_last_update_time = self.get_clock().now()
            self.get_logger().warn(
                "RPP FINAL 500MM DECELERATION STARTED | "
                f"{self.cruise_speed:.2f}->"
                f"{self.deceleration_floor_speed:.2f}m/s at "
                f"{self.waypoint_tolerance * 1000.0:.0f}mm boundary | "
                "zero remains controlled only by exact radial entry"
            )

        now = self.get_clock().now()
        if self.deceleration_last_update_time is None:
            dt = 1.0 / self.CONTROL_HZ
        else:
            dt = (now - self.deceleration_last_update_time).nanoseconds / 1e9
            if not math.isfinite(dt) or dt <= 0.0:
                dt = 1.0 / self.CONTROL_HZ
            dt = min(dt, self.deceleration_max_dt_sec)
        self.deceleration_last_update_time = now

        profile_target = self.terminal_speed_for_along_remaining(along_remaining)

        # The profile has a positive 0.15 m/s floor outside the radial gate.
        # Exact radial <=30 mm is checked before this function and bypasses
        # every non-zero rate limiter by calling publish_stop().
        profile_target = max(
            self.deceleration_floor_speed,
            profile_target,
        )

        # Reject impossible one-cycle speed collapses caused by a position
        # jump. Under normal motion the constant-deceleration profile changes
        # at deceleration_rate and this 1.5x guard does not alter the envelope.
        minimum_next_speed = max(
            self.deceleration_floor_speed,
            self.deceleration_output_speed - 1.5 * self.deceleration_rate * dt,
        )
        next_speed = max(profile_target, minimum_next_speed)

        # Monotonic: odometry jitter must not re-accelerate the terminal
        # profile after a lower speed has already been issued.
        self.deceleration_output_speed = min(
            requested_speed,
            self.deceleration_output_speed,
            next_speed,
        )
        self.deceleration_progress_m = max(
            0.0,
            self.deceleration_distance - max(along_remaining, self.waypoint_tolerance),
        )
        self.deceleration_remaining_m = distance_to_boundary

        return self.deceleration_output_speed

    def command_speed_slew_limit(self, target_speed):
        """Rate-limit every non-zero velocity magnitude transition.

        This protects native alignment, post-alignment acceleration, and
        terminal entry. publish_stop() remains immediate because a literal
        zero command resets the slew state instead of passing through here.
        """
        target_speed = max(
            0.0,
            min(
                target_speed,
                self.cruise_speed,
                self.MAXIMUM_MOVING_SPEED_MPS,
            ),
        )
        now = self.get_clock().now()
        if self.command_slew_last_time is None:
            dt = 1.0 / self.CONTROL_HZ
        else:
            dt = (now - self.command_slew_last_time).nanoseconds / 1e9
            if not math.isfinite(dt) or dt <= 0.0:
                dt = 1.0 / self.CONTROL_HZ
            dt = min(dt, self.deceleration_max_dt_sec)
        self.command_slew_last_time = now

        if target_speed >= self.command_slew_speed:
            if not self.acceleration_enabled:
                # No Jetson acceleration profile: normal forward movement
                # commands the requested fixed 1.00 m/s speed immediately.
                self.command_slew_speed = target_speed
            else:
                max_change = self.command_speed_rise_limit * dt
                self.command_slew_speed = min(
                    target_speed,
                    self.command_slew_speed + max_change,
                )
        else:
            max_change = self.command_speed_fall_limit * dt
            self.command_slew_speed = max(
                target_speed,
                self.command_slew_speed - max_change,
            )
        return self.command_slew_speed

    def _begin_precision_cycle(self):
        """Create the bounded timing/token authority for one control tick."""

        self.precision_cycle_token += 1
        now = self.get_clock().now()
        if self.precision_last_cycle_time is None:
            dt_sec = 1.0 / self.CONTROL_HZ
        else:
            dt_sec = (now - self.precision_last_cycle_time).nanoseconds / 1e9
            if not math.isfinite(dt_sec):
                dt_sec = 0.0
            dt_sec = max(
                0.0,
                min(dt_sec, self.precision_speed_config.control_dt_max_sec),
            )
        self.precision_last_cycle_time = now
        self.precision_cycle_dt_sec = dt_sec

        self.geometry_last_projection = None
        self.geometry_last_projection_cycle_token = None
        self.precision_guidance_cycle_token = None
        self.precision_guidance_result = None
        self.precision_speed_cycle_token = None
        self.precision_speed_result = None
        self.precision_speed_request = None
        self.precision_terminal_speed_override_mps = None

    def _current_cycle_projection(self):
        if self.geometry_last_projection_cycle_token != self.precision_cycle_token:
            return None
        return self.geometry_last_projection

    def _publish_guidance_debug(self, result, *, status):
        if not (
            self.precision_guidance_enabled or self.precision_speed_control_enabled
        ):
            return
        payload = {
            "schema_version": 1,
            "status": status,
            "ros_time_ns": self.get_clock().now().nanoseconds,
            "cycle_token": self.precision_cycle_token,
            "projection_cycle_token": self.geometry_last_projection_cycle_token,
            "precision_guidance_enabled": self.precision_guidance_enabled,
            "precision_speed_control_enabled": self.precision_speed_control_enabled,
            "path_signature": self.geometry_installed_signature,
            "active_goal_identity": (
                self.geometry_goal_binding.active_goal_identity
                if self.geometry_goal_binding is not None
                else None
            ),
            "rover_x": self.current_x,
            "rover_y": self.current_y,
            "rover_yaw_rad": self.current_yaw,
            "lookahead_distance_m": (
                result.lookahead_distance_m if result is not None else None
            ),
            "requested_lookahead_m": (
                result.lookahead_distance_m if result is not None else None
            ),
            "lookahead_target_s": (
                result.lookahead_target_s if result is not None else None
            ),
            "lookahead_segment_index": (
                result.lookahead_segment_index if result is not None else None
            ),
            "lookahead_x": result.lookahead_point.x if result is not None else None,
            "lookahead_y": result.lookahead_point.y if result is not None else None,
            "steering_target_x": (
                result.steering_target_point.x if result is not None else None
            ),
            "steering_target_y": (
                result.steering_target_point.y if result is not None else None
            ),
            "actual_steering_target_distance_m": (
                result.actual_steering_target_distance_m if result is not None else None
            ),
            "endpoint_extension_used": (
                result.endpoint_extension_used if result is not None else None
            ),
            "endpoint_extension_distance_m": (
                result.endpoint_extension_distance_m if result is not None else None
            ),
            "target_behind_rover": (
                result.target_behind_rover if result is not None else None
            ),
            "path_heading_rad": (
                result.local_path_heading_rad if result is not None else None
            ),
            "path_heading_error_rad": (
                result.path_heading_error_rad if result is not None else None
            ),
            "lookahead_bearing_rad": (
                result.lookahead_bearing_rad if result is not None else None
            ),
            "lookahead_heading_error_rad": (
                result.heading_error_rad if result is not None else None
            ),
            "heading_error_rad": (
                result.heading_error_rad if result is not None else None
            ),
            "cross_track_m": (
                self.ground_xtrack(result.signed_cross_track_m)
                if result is not None
                else None
            ),
            "desired_command_bearing_rad": (
                result.desired_movement_bearing_rad if result is not None else None
            ),
            "limited_command_bearing_rad": (
                result.limited_command_bearing_rad if result is not None else None
            ),
            "final_command_correction_rad": (
                result.final_command_correction_rad if result is not None else None
            ),
            "bearing_clamp_fired": (
                result.bearing_clamp_fired if result is not None else None
            ),
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.guidance_debug_pub.publish(message)

    def _compute_precision_guidance_for_cycle(self):
        """Return guidance only from this tick's successful projection."""

        if self.precision_guidance_cycle_token == self.precision_cycle_token:
            return self.precision_guidance_result
        projection = self._current_cycle_projection()
        if not (
            projection is not None
            and self.path_geometry is not None
            and self.geometry_active_span is not None
            and self.current_x is not None
            and self.current_y is not None
            and self.current_yaw is not None
        ):
            self._publish_guidance_debug(None, status="CURRENT_PROJECTION_UNAVAILABLE")
            return None
        measured_speed = (
            self.current_speed_mps if math.isfinite(self.current_speed_mps) else 0.0
        )
        effective_speed = max(
            0.0,
            measured_speed,
            self.precision_last_published_translational_speed_mps,
        )
        try:
            result = compute_precision_guidance(
                self.precision_guidance_config,
                geometry=self.path_geometry,
                projection=projection,
                active_span=self.geometry_active_span,
                rover_position=(self.current_x, self.current_y),
                rover_yaw_rad=self.current_yaw,
                speed_mps=effective_speed,
            )
        except (TypeError, ValueError):
            self._publish_guidance_debug(None, status="GUIDANCE_REJECTED")
            return None
        self.precision_guidance_cycle_token = self.precision_cycle_token
        self.precision_guidance_result = result
        self._publish_guidance_debug(result, status="READY")
        return result

    def _scoped_precision_corner_preview(self, projection):
        """Return only corners at or before the active semantic stop."""

        if self.path_geometry is None or self.geometry_active_span is None:
            return None, None
        corner = self.path_geometry.next_corner(projection.progress_s)
        if corner is None or corner.s > self.geometry_active_span.stop_s + 1.0e-9:
            return None, None
        return max(0.0, corner.s - projection.progress_s), corner.turn_angle_rad

    def _compute_precision_tracking_for_cycle(self):
        """Compute one current-cycle stability decision before speed resolve."""

        if self.precision_tracking_cycle_token == self.precision_cycle_token:
            return self.precision_tracking_output
        if not self.precision_tracking_control_enabled:
            return None
        projection = self._current_cycle_projection()
        guidance = self._compute_precision_guidance_for_cycle()
        path_identity = self.geometry_installed_signature
        if projection is None or guidance is None or not path_identity:
            self._publish_tracking_debug(status="CURRENT_INPUT_UNAVAILABLE")
            return None
        measured_speed = (
            abs(self.current_speed_mps)
            if math.isfinite(self.current_speed_mps)
            else math.inf
        )
        sample = TrackingControlInput(
            path_identity=path_identity,
            projection_s_m=projection.progress_s,
            signed_cross_track_m=guidance.signed_cross_track_m,
            heading_error_rad=guidance.heading_error_rad,
            commanded_speed_mps=(self.precision_last_published_translational_speed_mps),
            measured_speed_mps=measured_speed,
            bearing_clamped=guidance.bearing_clamp_fired,
            telemetry_fresh=self.is_fresh(
                self.last_odom_time,
                self.odom_timeout_sec,
            ),
            dt_sec=self.precision_cycle_dt_sec,
        )
        try:
            output = self.precision_tracking_controller.step(sample)
        except (TypeError, ValueError):
            self._publish_tracking_debug(status="TRACKING_INPUT_REJECTED")
            return None
        self.precision_tracking_cycle_token = self.precision_cycle_token
        self.precision_tracking_input = sample
        self.precision_tracking_output = output
        self._publish_tracking_debug(status="READY" if output.valid else "INVALID_STOP")
        return output

    def _reset_precision_tracking(
        self,
        reason,
        *,
        reset_metrics,
        path_identity=None,
    ):
        if not hasattr(self, "precision_tracking_controller"):
            return
        identity = path_identity or self.geometry_installed_signature
        self.precision_tracking_controller.reset(identity)
        self.precision_tracking_cycle_token = None
        self.precision_tracking_output = None
        self.precision_tracking_input = None
        self.precision_tracking_reset_reason = str(reason)
        self.precision_tracking_reset_count += 1
        if reset_metrics:
            self.precision_tracking_metrics.reset(
                self.precision_tracking_mission_identity,
                identity,
            )

    def _publish_tracking_debug(self, *, status):
        """Publish guarded controller/EKF-frame diagnostics only."""

        if not self.precision_tracking_control_enabled:
            return
        try:
            output = self.precision_tracking_output
            snapshot = self.precision_tracking_metrics.snapshot()
            tracking_cap = None
            if output is not None:
                tracking_cap = output.recovery_speed_scale * min(
                    self.cruise_speed,
                    self.precision_speed_config.hardware_speed_ceiling_mps,
                    self.MAXIMUM_MOVING_SPEED_MPS,
                )
            payload = {
                "schema_version": 1,
                "status": str(status),
                "ros_time_ns": self.get_clock().now().nanoseconds,
                "cycle_token": self.precision_cycle_token,
                "projection_cycle_token": self.geometry_last_projection_cycle_token,
                "precision_tracking_control_enabled": True,
                "authority_frame": "controller_ekf_local_frame",
                "physical_ground_truth_certified": False,
                "kpi_targets_diagnostic_only": {
                    "straight_rms_cross_track_mm_preferred": 10.0,
                    "straight_p95_cross_track_mm": 20.0,
                },
                "path_signature": self.geometry_installed_signature,
                "mission_identity": self.precision_tracking_mission_identity,
                "state": output.state.value if output is not None else None,
                "transition_reason": (
                    output.transition_reason if output is not None else None
                ),
                "valid": output.valid if output is not None else False,
                "invalid_reason": (
                    output.invalid_reason if output is not None else None
                ),
                "acceleration_allowed": (
                    output.acceleration_allowed if output is not None else False
                ),
                "tracking_speed_cap_mps": tracking_cap,
                "winning_speed_cap_owner": (
                    self.precision_speed_result.winning_cap_owner.value
                    if self.precision_speed_result is not None
                    and self.precision_speed_cycle_token == self.precision_cycle_token
                    else None
                ),
                "stable_dwell_sec": (
                    output.stable_dwell_sec if output is not None else 0.0
                ),
                "bounded_dt_sec": (
                    output.bounded_dt_sec if output is not None else 0.0
                ),
                "reset_reason": self.precision_tracking_reset_reason,
                "reset_count": self.precision_tracking_reset_count,
                "metrics": {
                    "sample_count": snapshot.sample_count,
                    "rejected_sample_count": snapshot.rejected_sample_count,
                    "mean_abs_cross_track_mm": (
                        snapshot.mean_abs_cross_track_m * 1000.0
                    ),
                    "rms_cross_track_mm": snapshot.rms_cross_track_m * 1000.0,
                    "p95_abs_cross_track_mm": (snapshot.p95_abs_cross_track_m * 1000.0),
                    "whole_run_p95_abs_cross_track_mm": (
                        snapshot.p95_abs_cross_track_m * 1000.0
                    ),
                    "trailing_p95_abs_cross_track_mm": (
                        snapshot.trailing_p95_abs_cross_track_m * 1000.0
                    ),
                    "p95_histogram_saturated": (snapshot.p95_histogram_saturated),
                    "histogram_overflow_count": (snapshot.histogram_overflow_count),
                    "max_abs_cross_track_mm": (snapshot.max_abs_cross_track_m * 1000.0),
                    "mean_abs_heading_error_deg": math.degrees(
                        snapshot.mean_abs_heading_error_rad
                    ),
                    "max_abs_heading_error_deg": math.degrees(
                        snapshot.max_abs_heading_error_rad
                    ),
                    "raw_projection_monotonic_violations": (
                        snapshot.monotonic_s_violation_count
                    ),
                    "recovery_time_sec": snapshot.recovery_time_sec,
                    "recapture_time_sec": snapshot.recapture_time_sec,
                    "cruise_time_sec": snapshot.cruise_time_sec,
                    "mean_commanded_speed_mps": (snapshot.mean_commanded_speed_mps),
                    "mean_measured_speed_mps": snapshot.mean_measured_speed_mps,
                    "mean_abs_speed_error_mps": (snapshot.mean_abs_speed_error_mps),
                    "rms_speed_error_mps": snapshot.rms_speed_error_mps,
                    "max_abs_speed_error_mps": snapshot.max_abs_speed_error_mps,
                    "quantile_sample_count": snapshot.quantile_sample_count,
                    "quantile_window_capacity": snapshot.quantile_window_capacity,
                    "histogram_bin_width_mm": (
                        self.precision_tracking_config.metrics_histogram_bin_width_m
                        * 1000.0
                    ),
                    "histogram_max_mm": (
                        self.precision_tracking_config.metrics_histogram_max_m * 1000.0
                    ),
                    "discontinuity_count": snapshot.discontinuity_count,
                    "last_discontinuity_reason": (snapshot.last_discontinuity_reason),
                    # Evidence quality only. This field is never read by the
                    # motion FSM, terminal latch, or mission completion path.
                    "valid_for_acceptance": snapshot.valid_for_acceptance,
                },
            }
            message = String()
            message.data = json.dumps(
                payload,
                separators=(",", ":"),
                sort_keys=True,
            )
            self.tracking_debug_pub.publish(message)
        except Exception as error:
            try:
                self.get_logger().error(
                    "TRACKING DIAGNOSTICS FAILED / CONTROL UNAFFECTED | "
                    f"type={type(error).__name__}"
                )
            except Exception:
                pass

    def _record_precision_tracking_metrics(self, published_speed_mps):
        """Add only a successfully published precision translation sample."""

        if not self.precision_tracking_control_enabled:
            return
        projection = self._current_cycle_projection()
        guidance = self.precision_guidance_result
        output = self.precision_tracking_output
        if not (
            projection is not None
            and guidance is not None
            and output is not None
            and output.valid
            and self.precision_tracking_cycle_token == self.precision_cycle_token
        ):
            return
        try:
            sample = TrackingControlInput(
                path_identity=str(self.geometry_installed_signature),
                # Raw projected_s intentionally exposes snap-back that the
                # monotonic progress authority clamps out of control.
                projection_s_m=projection.projected_s,
                signed_cross_track_m=guidance.signed_cross_track_m,
                heading_error_rad=guidance.heading_error_rad,
                commanded_speed_mps=float(published_speed_mps),
                measured_speed_mps=abs(float(self.current_speed_mps)),
                bearing_clamped=guidance.bearing_clamp_fired,
                telemetry_fresh=self.is_fresh(
                    self.last_odom_time,
                    self.odom_timeout_sec,
                ),
                dt_sec=self.precision_cycle_dt_sec,
            )
            self.precision_tracking_metrics.add(
                sample,
                output.state,
                mission_identity=self.precision_tracking_mission_identity,
            )
            self._publish_tracking_debug(status="PUBLISHED_SAMPLE")
        except Exception:
            self._publish_tracking_debug(status="METRIC_SAMPLE_REJECTED")

    def _resolve_precision_speed_for_cycle(self):
        """Resolve exactly one longitudinal command for this control tick."""

        if self.precision_speed_cycle_token == self.precision_cycle_token:
            return self.precision_speed_result
        projection = self._current_cycle_projection()
        guidance = self._compute_precision_guidance_for_cycle()
        if projection is None or guidance is None:
            return None
        tracking = None
        if self.precision_tracking_control_enabled:
            tracking = self._compute_precision_tracking_for_cycle()
            if tracking is None or not tracking.valid:
                return None
        corner_distance, corner_angle = self._scoped_precision_corner_preview(
            projection
        )
        measured_speed = (
            self.current_speed_mps if math.isfinite(self.current_speed_mps) else 0.0
        )
        request = SpeedRegulatorInput(
            mission_speed_ceiling_mps=self.cruise_speed,
            measured_speed_mps=measured_speed,
            last_commanded_speed_mps=(
                self.precision_last_published_translational_speed_mps
            ),
            dt_sec=self.precision_cycle_dt_sec,
            along_track_progress_m=projection.progress_s,
            heading_error_rad=guidance.heading_error_rad,
            cross_track_error_m=guidance.signed_cross_track_m,
            distance_to_corner_m=corner_distance,
            corner_angle_rad=corner_angle,
            distance_to_terminal_m=projection.remaining_to_active_stop_m,
            terminal_target_speed_override_mps=(
                self.precision_terminal_speed_override_mps
            ),
            curvature_inv_m=None,
            tracking_acceleration_allowed=(
                tracking.acceleration_allowed if tracking is not None else True
            ),
            tracking_speed_cap_mps=(
                tracking.recovery_speed_scale
                * min(
                    self.cruise_speed,
                    self.precision_speed_config.hardware_speed_ceiling_mps,
                    self.MAXIMUM_MOVING_SPEED_MPS,
                )
                if tracking is not None
                else None
            ),
            hard_zero=False,
        )
        try:
            result = self.precision_speed_regulator.resolve(request)
        except (RuntimeError, TypeError, ValueError):
            return None
        self.precision_speed_cycle_token = self.precision_cycle_token
        self.precision_speed_request = request
        self.precision_speed_result = result
        return result

    def _reset_precision_regulator(self, reason, *, progress_s=None):
        if not hasattr(self, "precision_speed_regulator"):
            return
        if progress_s is None:
            projection = self._current_cycle_projection()
            progress_s = projection.progress_s if projection is not None else 0.0
        if not math.isfinite(progress_s) or progress_s < 0.0:
            progress_s = 0.0
        self.precision_speed_regulator.reset(
            along_track_progress_m=progress_s,
            initial_speed_mps=0.0,
        )
        self.precision_last_published_translational_speed_mps = 0.0
        self.precision_speed_cycle_token = None
        self.precision_speed_result = None
        self.precision_speed_request = None
        self.precision_regulator_reset_reason = str(reason)
        self.precision_regulator_reset_count += 1

    def _record_published_translational_speed(self, speed_mps):
        if math.isfinite(speed_mps) and speed_mps >= 0.0:
            self.precision_last_published_translational_speed_mps = speed_mps

    def _publish_speed_debug(self, result, *, published_speed, floor_applied):
        request = self.precision_speed_request
        caps = result.caps
        winning_owner = (
            "minimum_moving_floor" if floor_applied else result.winning_cap_owner.value
        )
        payload = {
            "schema_version": 1,
            "status": "READY",
            "ros_time_ns": self.get_clock().now().nanoseconds,
            "cycle_token": self.precision_cycle_token,
            "projection_cycle_token": self.geometry_last_projection_cycle_token,
            "precision_speed_control_enabled": self.precision_speed_control_enabled,
            "requested_speed_mps": result.requested_speed_mps,
            "published_speed_mps": published_speed,
            "winning_cap_owner": winning_owner,
            "resolver_winning_cap_owner": result.winning_cap_owner.value,
            "minimum_moving_floor_applied": floor_applied,
            "minimum_moving_speed_mps": self.precision_minimum_moving_speed,
            "caps": {
                "mission_mps": caps.mission_mps,
                "hardware_mps": caps.hardware_mps,
                "recovery_mps": caps.recovery_mps,
                "acceleration_mps": caps.acceleration_mps,
                "heading_mps": caps.heading_mps,
                "cross_track_mps": caps.cross_track_mps,
                "tracking_mps": caps.tracking_mps,
                "corner_mps": caps.corner_mps,
                "terminal_mps": caps.terminal_mps,
                "curvature_mps": caps.curvature_mps,
            },
            "bounded_dt_sec": result.bounded_dt_sec,
            "effective_speed_mps": result.effective_speed_mps,
            "acceleration_gate_scale": result.acceleration_gate_scale,
            "acceleration_progress_m": result.acceleration_progress_m,
            "corner_required_braking_distance_m": (
                result.corner_required_braking_distance_m
            ),
            "terminal_required_braking_distance_m": (
                result.terminal_required_braking_distance_m
            ),
            "distance_to_corner_m": (
                request.distance_to_corner_m if request is not None else None
            ),
            "corner_angle_rad": (
                request.corner_angle_rad if request is not None else None
            ),
            "distance_to_terminal_m": (
                request.distance_to_terminal_m if request is not None else None
            ),
            "terminal_target_speed_override_mps": (
                request.terminal_target_speed_override_mps
                if request is not None
                else None
            ),
            "effective_terminal_target_speed_mps": (
                request.terminal_target_speed_override_mps
                if request is not None
                and request.terminal_target_speed_override_mps is not None
                else self.precision_speed_config.terminal_target_speed_mps
            ),
            "last_published_translational_speed_mps": (
                request.last_commanded_speed_mps if request is not None else None
            ),
            "measured_speed_mps": (
                request.measured_speed_mps if request is not None else None
            ),
            "tracking_acceleration_allowed": (
                request.tracking_acceleration_allowed if request is not None else None
            ),
            "tracking_speed_cap_mps": (
                request.tracking_speed_cap_mps if request is not None else None
            ),
            "recovery_active": result.recovery_active,
            "recovery_transition": result.recovery_transition,
            "recovery_exit_dwell_sec": result.recovery_exit_dwell_sec,
            "recovery_exit_dwell_required_sec": (
                result.recovery_exit_dwell_required_sec
            ),
            "regulator_reset_reason": self.precision_regulator_reset_reason,
            "regulator_reset_count": self.precision_regulator_reset_count,
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.speed_debug_pub.publish(message)

    def publish_precision_velocity_ned(self, command_bearing, result):
        """Publish one already-resolved translational command.

        This path deliberately bypasses all legacy acceleration, deceleration,
        hard-cap, and slew profiles.  The pure resolver owns longitudinal math;
        this adapter retains only finite/vector/hardware safety clamps.  Native
        pivot carrier commands never call this method.
        """

        if result is None or not math.isfinite(command_bearing):
            self.get_logger().error("INVALID PRECISION COMMAND / IMMEDIATE ZERO")
            self.publish_stop()
            return 0.0, 0.0, 0.0
        resolved_speed = float(result.requested_speed_mps)
        if not math.isfinite(resolved_speed) or resolved_speed < 0.0:
            self.get_logger().error("NON-FINITE PRECISION SPEED / IMMEDIATE ZERO")
            self.publish_stop()
            return 0.0, 0.0, 0.0

        # A non-latched Phase-2 cycle must remain translational.  Phase 5 owns
        # the future zero/certificate state machine; today the unchanged 30 mm
        # radial latch reaches publish_stop() before this method is selected.
        output_speed = max(resolved_speed, self.precision_minimum_moving_speed)
        floor_applied = output_speed > resolved_speed + 1.0e-12
        output_speed = min(
            output_speed,
            self.cruise_speed,
            self.precision_speed_config.hardware_speed_ceiling_mps,
            self.MAXIMUM_MOVING_SPEED_MPS,
        )
        north = output_speed * math.sin(command_bearing)
        east = output_speed * math.cos(command_bearing)
        if not all(math.isfinite(value) for value in (north, east, output_speed)):
            self.get_logger().error("NON-FINITE PRECISION VECTOR / IMMEDIATE ZERO")
            self.publish_stop()
            return 0.0, 0.0, 0.0

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map_ned"
        msg.vector.x = float(north)
        msg.vector.y = float(east)
        msg.vector.z = 0.0
        self.velocity_pub.publish(msg)
        self._record_published_translational_speed(output_speed)
        self._record_precision_tracking_metrics(output_speed)
        self.publish_motion_profile_monitor(output_speed)
        self._publish_speed_debug(
            result,
            published_speed=output_speed,
            floor_applied=floor_applied,
        )
        return north, east, output_speed

    def publish_velocity_ned(
        self,
        north,
        east,
        *,
        apply_acceleration=True,
        apply_deceleration=False,
        goal_distance=None,
        hard_speed_cap_mps=None,
    ):
        if not all(math.isfinite(value) for value in (north, east)):
            self.get_logger().error("Rejected non-finite RPP velocity command")
            north = 0.0
            east = 0.0

        hard_speed_cap = None
        if hard_speed_cap_mps is not None:
            hard_speed_cap = float(hard_speed_cap_mps)
            if not math.isfinite(hard_speed_cap) or hard_speed_cap <= 0.0:
                self.get_logger().error(
                    "INVALID HARD SPEED CAP / IMMEDIATE ZERO | "
                    f"value={hard_speed_cap_mps!r}"
                )
                north = 0.0
                east = 0.0
                hard_speed_cap = 0.0
            else:
                hard_speed_cap = min(
                    hard_speed_cap,
                    self.cruise_speed,
                    self.MAXIMUM_MOVING_SPEED_MPS,
                )

        raw_speed = math.hypot(north, east)
        if raw_speed > 1.0e-9:
            requested_speed = min(
                raw_speed,
                self.MAXIMUM_MOVING_SPEED_MPS,
            )
            direction_north = north / raw_speed
            direction_east = east / raw_speed

            if apply_acceleration:
                output_speed = self.acceleration_speed_limit(requested_speed)
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

            # Distance deceleration is calculated first. The xtrack cap is
            # then applied as a strict upper bound:
            # command = min(distance_profile, xtrack_cap).
            if hard_speed_cap is not None:
                output_speed = min(
                    output_speed,
                    hard_speed_cap,
                )

            output_speed = self.command_speed_slew_limit(output_speed)

            # Do not let a normal fall-rate limiter delay a safety cap.
            if hard_speed_cap is not None and output_speed > hard_speed_cap:
                output_speed = hard_speed_cap
                self.command_slew_speed = hard_speed_cap

            north = direction_north * output_speed
            east = direction_east * output_speed
        else:
            self.reset_speed_profiles()
            # Safety/30 mm zero is immediate and must not be ramped down.
            self.command_slew_speed = 0.0
            self.command_slew_last_time = None
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
        self._reset_precision_regulator("LITERAL_STOP")
        self.publish_velocity_ned(0.0, 0.0)
        self._record_rpp_debug_command(0.0, 0.0, 0.0)

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
        """Publish exact segment metrics and consolidated accuracy telemetry.

        Monitoring only. This function does not alter rover control logic.

        Cross-track sign:
            positive = LEFT of the incoming segment
            negative = RIGHT of the incoming segment

        Front/back sign:
            positive = BEFORE the exact goal plane
            negative = AFTER the exact goal plane

        Radial error:
            direct Euclidean distance to the exact segment goal.
        """
        delta_east = self.current_x - goal_x
        delta_north = self.current_y - goal_y

        signed_xtrack = (
            -math.sin(path_bearing) * delta_east + math.cos(path_bearing) * delta_north
        )

        along_remaining = self.along_track_remaining(
            path_bearing,
            goal_x,
            goal_y,
        )

        radial_error = float(goal_distance)

        if math.isfinite(self.closest_marking_distance):
            closest_distance = float(self.closest_marking_distance)
        else:
            closest_distance = radial_error

        # Internal RPP sign is kept unchanged for control:
        # LEFT = positive, RIGHT = negative.
        internal_xtrack_mm = signed_xtrack * 1000.0

        # Ground-facing sign:
        # LEFT = negative, RIGHT = positive.
        ground_xtrack_m = self.ground_xtrack(signed_xtrack)
        xtrack_mm = ground_xtrack_m * 1000.0

        radial_error_mm = radial_error * 1000.0
        along_remaining_mm = along_remaining * 1000.0
        closest_distance_mm = closest_distance * 1000.0

        # Ground-facing xtrack monitoring topic:
        # LEFT = negative, RIGHT = positive.
        self._publish_float64(
            self.xtrack_mm_pub,
            xtrack_mm,
        )
        self._publish_float64(
            self.goal_distance_mm_pub,
            radial_error_mm,
        )
        self._publish_float64(
            self.along_remaining_mm_pub,
            along_remaining_mm,
        )
        self._publish_float64(
            self.closest_distance_mm_pub,
            closest_distance_mm,
        )

        cross_epsilon_mm = 0.5
        front_back_epsilon_mm = 0.5

        if internal_xtrack_mm > cross_epsilon_mm:
            cross_track_side = "LEFT"
        elif internal_xtrack_mm < -cross_epsilon_mm:
            cross_track_side = "RIGHT"
        else:
            cross_track_side = "CENTER"

        if along_remaining_mm > front_back_epsilon_mm:
            front_back_position = "BEFORE_POINT"
        elif along_remaining_mm < -front_back_epsilon_mm:
            front_back_position = "AFTER_POINT"
        else:
            front_back_position = "AT_POINT"

        try:
            goal_number = int(self.segment_goal_number)
        except (TypeError, ValueError):
            goal_number = 0

        if goal_number < 0:
            goal_number = 0

        accuracy_target_mm = float(self.waypoint_tolerance) * 1000.0
        test_tolerance_mm = 50.0

        if radial_error_mm <= accuracy_target_mm:
            accuracy_status = "ACCURACY_PASS"
            accuracy_pass = True
            within_test_tolerance = True
        elif radial_error_mm <= test_tolerance_mm:
            accuracy_status = "TEST_PROCEED_BAND"
            accuracy_pass = False
            within_test_tolerance = True
        else:
            accuracy_status = "OUTSIDE_TOLERANCE"
            accuracy_pass = False
            within_test_tolerance = False

        accuracy_payload = {
            "available": True,
            "source": "/rpp/accuracy",
            "goal_number": goal_number,
            "cross_track_error_m": float(ground_xtrack_m),
            "cross_track_abs_mm": float(abs(xtrack_mm)),
            "cross_track_error_mm": float(xtrack_mm),
            "cross_track_side": cross_track_side,
            "front_back_error_m": float(along_remaining),
            "front_back_error_mm": float(along_remaining_mm),
            "front_back_abs_mm": float(abs(along_remaining_mm)),
            "front_back_position": front_back_position,
            "radial_error_m": float(radial_error),
            "radial_error_mm": float(radial_error_mm),
            "closest_radial_error_m": float(closest_distance),
            "closest_radial_error_mm": float(closest_distance_mm),
            "accuracy_target_m": float(self.waypoint_tolerance),
            "accuracy_target_mm": float(accuracy_target_mm),
            "test_tolerance_m": float(test_tolerance_mm / 1000.0),
            "test_tolerance_mm": float(test_tolerance_mm),
            "accuracy_status": accuracy_status,
            "accuracy_pass": accuracy_pass,
            "within_test_tolerance": within_test_tolerance,
        }

        accuracy_message = String()
        accuracy_message.data = json.dumps(
            accuracy_payload,
            separators=(",", ":"),
        )
        self.accuracy_pub.publish(accuracy_message)

        now = self.get_clock().now()
        if (now - self.last_mm_monitor_log_time).nanoseconds >= 500_000_000:
            self.last_mm_monitor_log_time = now
            self.get_logger().info(
                "MM MONITOR | "
                f"P{goal_number or '?'} | "
                f"xtrack={xtrack_mm:+.1f}mm ({cross_track_side}) | "
                f"front_back={along_remaining_mm:+.1f}mm "
                f"({front_back_position}) | "
                f"radial={radial_error_mm:.1f}mm | "
                f"closest={closest_distance_mm:.1f}mm | "
                f"{accuracy_status}"
            )

    @staticmethod
    def _finite_or_none(value):
        if value is None:
            return None
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) else None

    def _begin_rpp_debug_cycle(self):
        now = self.get_clock().now()
        now_mono_ns = time.monotonic_ns()
        previous_start_ns = self._rpp_debug_last_control_start_ns
        self._rpp_debug_last_control_start_ns = now_mono_ns
        self._rpp_debug_cycle_start_ns = now_mono_ns
        self._rpp_debug_control_sequence += 1

        control_dt_ms = (
            (now_mono_ns - previous_start_ns) / 1_000_000.0
            if previous_start_ns is not None
            else 1000.0 / self.CONTROL_HZ
        )
        odom_age_ms = None
        if self.last_odom_time is not None:
            odom_age_ms = max(
                0.0,
                (now - self.last_odom_time).nanoseconds / 1_000_000.0,
            )

        self._rpp_debug_pending = {
            "schema_version": 2,
            "source": "/rpp/debug",
            "publisher_alive": True,
            "available": False,
            "geometry_valid": False,
            "command_valid": False,
            "odom_fresh": bool(
                odom_age_ms is not None
                and odom_age_ms <= self.odom_timeout_sec * 1000.0
            ),
            "control_sequence": self._rpp_debug_control_sequence,
            "control_stamp_ros_ns": int(now.nanoseconds),
            "control_dt_ms": float(control_dt_ms),
            "control_compute_ms": None,
            "control_deadline_missed": False,
            "control_hz": float(self.CONTROL_HZ),
            "telemetry_hz": float(self.TELEMETRY_HZ),
            "odom_age_ms": odom_age_ms,
            "control_mode": "CONTROL_CYCLE",
            "reason": "NO_ACTIVE_CONTROL_OUTPUT",
            "goal_number": max(0, int(self.segment_goal_number or 0)),
            "actual_speed_mps": self._finite_or_none(self.current_speed_mps),
            "command_speed_mps": None,
            "command_north_mps": None,
            "command_east_mps": None,
            "current_yaw_rad": self._finite_or_none(self.current_yaw),
            "current_yaw_deg": (
                math.degrees(self.current_yaw)
                if self._finite_or_none(self.current_yaw) is not None
                else None
            ),
            "path_bearing_rad": None,
            "path_bearing_deg": None,
            "guidance_bearing_rad": None,
            "guidance_bearing_deg": None,
            "heading_error_rad": None,
            "heading_error_deg": None,
            "distance_to_goal_m": None,
            "distance_to_goal_mm": None,
            "cross_track_error_m": None,
            "cross_track_error_mm": None,
            "cross_track_side": "UNKNOWN",
            "along_remaining_m": None,
            "along_remaining_mm": None,
            "along_position": "UNKNOWN",
        }

    def _set_rpp_debug_status(self, control_mode, reason):
        if self._rpp_debug_pending is None:
            return
        self._rpp_debug_pending["control_mode"] = str(control_mode)
        self._rpp_debug_pending["reason"] = str(reason)

    def _record_rpp_debug_command(self, north, east, speed):
        if self._rpp_debug_pending is None:
            return
        north = self._finite_or_none(north)
        east = self._finite_or_none(east)
        speed = self._finite_or_none(speed)
        self._rpp_debug_pending.update(
            {
                "command_valid": all(
                    value is not None for value in (north, east, speed)
                ),
                "command_north_mps": north,
                "command_east_mps": east,
                "command_speed_mps": speed,
            }
        )

    def _record_rpp_debug_geometry(
        self,
        *,
        path_bearing_rad,
        heading_error_rad,
        distance_to_goal_m,
        signed_cross_track_m,
        along_remaining_m,
    ):
        if self._rpp_debug_pending is None:
            return
        values = tuple(
            self._finite_or_none(value)
            for value in (
                path_bearing_rad,
                heading_error_rad,
                distance_to_goal_m,
                signed_cross_track_m,
                along_remaining_m,
            )
        )
        if any(value is None for value in values):
            return
        (
            path_bearing_rad,
            heading_error_rad,
            distance_to_goal_m,
            signed_cross_track_m,
            along_remaining_m,
        ) = values
        ground_xtrack_m = self.ground_xtrack(signed_cross_track_m)
        self._rpp_debug_pending.update(
            {
                "available": True,
                "geometry_valid": True,
                "path_bearing_rad": path_bearing_rad,
                "path_bearing_deg": math.degrees(path_bearing_rad),
                "heading_error_rad": heading_error_rad,
                "heading_error_deg": math.degrees(heading_error_rad),
                "distance_to_goal_m": distance_to_goal_m,
                "distance_to_goal_mm": distance_to_goal_m * 1000.0,
                "cross_track_error_m": ground_xtrack_m,
                "cross_track_error_mm": ground_xtrack_m * 1000.0,
                "cross_track_side": (
                    "LEFT"
                    if signed_cross_track_m > 0.0005
                    else "RIGHT"
                    if signed_cross_track_m < -0.0005
                    else "CENTER"
                ),
                "along_remaining_m": along_remaining_m,
                "along_remaining_mm": along_remaining_m * 1000.0,
                "along_position": (
                    "BEFORE_POINT"
                    if along_remaining_m > 0.0005
                    else "AFTER_POINT"
                    if along_remaining_m < -0.0005
                    else "AT_POINT"
                ),
            }
        )

    def _finish_rpp_debug_cycle(self):
        pending = self._rpp_debug_pending
        if pending is None:
            return
        finish_ns = time.monotonic_ns()
        start_ns = self._rpp_debug_cycle_start_ns or finish_ns
        compute_ms = max(0.0, (finish_ns - start_ns) / 1_000_000.0)
        pending["control_compute_ms"] = compute_ms
        pending["control_deadline_missed"] = compute_ms > 1000.0 / self.CONTROL_HZ

        if pending["control_mode"] == "CONTROL_CYCLE":
            if self.segment_alignment_active:
                pending["control_mode"] = "ALIGNMENT"
                pending["reason"] = "SEGMENT_ALIGNMENT_ACTIVE"
            elif pending["command_valid"]:
                command_speed = pending["command_speed_mps"] or 0.0
                pending["control_mode"] = (
                    "STOP" if command_speed <= 1.0e-9 else "CONTROL_OUTPUT"
                )
                pending["reason"] = "FINAL_COMMAND_RECORDED"
            else:
                pending["control_mode"] = "WAITING"

        pending["actual_speed_mps"] = self._finite_or_none(self.current_speed_mps)
        pending["sample_complete_monotonic_ns"] = finish_ns
        with self._rpp_debug_lock:
            self._rpp_debug_snapshot = dict(pending)
        self._rpp_debug_pending = None
        self._rpp_debug_cycle_start_ns = None

    def _publish_rpp_debug_telemetry(self):
        now_mono_ns = time.monotonic_ns()
        now_ros = self.get_clock().now()
        with self._rpp_debug_lock:
            if self._rpp_debug_snapshot is None:
                return
            payload = dict(self._rpp_debug_snapshot)
            self._rpp_debug_telemetry_sequence += 1
            telemetry_sequence = self._rpp_debug_telemetry_sequence

        sample_complete_ns = payload.pop("sample_complete_monotonic_ns", now_mono_ns)
        payload.update(
            {
                "telemetry_sequence": telemetry_sequence,
                "telemetry_stamp_ros_ns": int(now_ros.nanoseconds),
                "control_sample_age_ms": max(
                    0.0,
                    (now_mono_ns - sample_complete_ns) / 1_000_000.0,
                ),
            }
        )
        try:
            message = String()
            message.data = json.dumps(
                payload,
                separators=(",", ":"),
                allow_nan=False,
            )
            self.rpp_debug_pub.publish(message)
        except Exception as exc:
            if now_mono_ns - self._rpp_debug_last_error_log_ns >= 1_000_000_000:
                self._rpp_debug_last_error_log_ns = now_mono_ns
                self.get_logger().error(
                    f"RPP DEBUG PUBLISH FAILED: {type(exc).__name__}: {exc}"
                )

    def publish_rpp_debug(
        self,
        *,
        control_mode,
        command_speed_mps,
        path_bearing_rad,
        guidance_bearing_rad,
        heading_error_rad,
        distance_to_goal_m,
        signed_cross_track_m,
        along_remaining_m,
    ):
        """Publish exact values from the active RPP control cycle.

        This is monitoring only. Heading, cross-track, goal distance and
        command speed are passed from the exact variables used by RPP.
        Backend/frontend consumers must not reconstruct control geometry.
        """
        values = (
            self.current_speed_mps,
            self.current_yaw,
            command_speed_mps,
            path_bearing_rad,
            guidance_bearing_rad,
            heading_error_rad,
            distance_to_goal_m,
            signed_cross_track_m,
            along_remaining_m,
        )
        available = all(
            value is not None and math.isfinite(float(value))
            for value in values
        )

        if available:
            actual_speed_mps = float(self.current_speed_mps)
            current_yaw_rad = float(self.current_yaw)
            command_speed_mps = float(command_speed_mps)
            path_bearing_rad = float(path_bearing_rad)
            guidance_bearing_rad = float(guidance_bearing_rad)
            heading_error_rad = float(heading_error_rad)
            distance_to_goal_m = float(distance_to_goal_m)
            signed_cross_track_m = float(signed_cross_track_m)
            along_remaining_m = float(along_remaining_m)

            # Only RPP's existing reporting-sign conversion is applied here.
            # No path geometry is recomputed.
            ground_xtrack_m = float(
                self.ground_xtrack(signed_cross_track_m)
            )

            if signed_cross_track_m > 0.0005:
                cross_track_side = "LEFT"
            elif signed_cross_track_m < -0.0005:
                cross_track_side = "RIGHT"
            else:
                cross_track_side = "CENTER"

            if along_remaining_m > 0.0005:
                along_position = "BEFORE_POINT"
            elif along_remaining_m < -0.0005:
                along_position = "AFTER_POINT"
            else:
                along_position = "AT_POINT"
        else:
            actual_speed_mps = None
            current_yaw_rad = None
            command_speed_mps = None
            path_bearing_rad = None
            guidance_bearing_rad = None
            heading_error_rad = None
            distance_to_goal_m = None
            ground_xtrack_m = None
            along_remaining_m = None
            cross_track_side = "UNKNOWN"
            along_position = "UNKNOWN"

        try:
            goal_number = max(0, int(self.segment_goal_number))
        except (TypeError, ValueError):
            goal_number = 0

        payload = {
            "available": available,
            "source": "/rpp/debug",
            "control_mode": str(control_mode),
            "goal_number": goal_number,

            "actual_speed_mps": actual_speed_mps,
            "command_speed_mps": command_speed_mps,

            "current_yaw_rad": current_yaw_rad,
            "current_yaw_deg": (
                math.degrees(current_yaw_rad)
                if current_yaw_rad is not None
                else None
            ),

            "path_bearing_rad": path_bearing_rad,
            "path_bearing_deg": (
                math.degrees(path_bearing_rad)
                if path_bearing_rad is not None
                else None
            ),

            "guidance_bearing_rad": guidance_bearing_rad,
            "guidance_bearing_deg": (
                math.degrees(guidance_bearing_rad)
                if guidance_bearing_rad is not None
                else None
            ),

            # IMPORTANT: this is the final existing RPP heading_error.
            "heading_error_rad": heading_error_rad,
            "heading_error_deg": (
                math.degrees(heading_error_rad)
                if heading_error_rad is not None
                else None
            ),

            "distance_to_goal_m": distance_to_goal_m,
            "distance_to_goal_mm": (
                distance_to_goal_m * 1000.0
                if distance_to_goal_m is not None
                else None
            ),

            "cross_track_error_m": ground_xtrack_m,
            "cross_track_error_mm": (
                ground_xtrack_m * 1000.0
                if ground_xtrack_m is not None
                else None
            ),
            "cross_track_side": cross_track_side,

            "along_remaining_m": along_remaining_m,
            "along_remaining_mm": (
                along_remaining_m * 1000.0
                if along_remaining_m is not None
                else None
            ),
            "along_position": along_position,
        }
        if self._rpp_debug_pending is not None:
            self._rpp_debug_pending.update(payload)
            self._rpp_debug_pending["geometry_valid"] = available
            self._rpp_debug_pending["command_valid"] = command_speed_mps is not None
            self._rpp_debug_pending["reason"] = "ACTIVE_CONTROL_OUTPUT"

    def log_waiting(self, reason):
        self._set_rpp_debug_status("WAITING", reason)
        now = self.get_clock().now()
        if (now - self.last_wait_log_time).nanoseconds < 1_000_000_000:
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
        if (now - self.last_log_time).nanoseconds < 1_000_000_000:
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

    def update_xtrack_speed_cap_state(
        self,
        signed_cross_track,
        predicted_cross_track,
        path_heading_error,
    ):
        """Update the hardened 0.15 m/s xtrack speed-cap latch.

        Entry uses the worse of measured and predicted cross-track error.
        Release requires measured and predicted error to be inside the exit
        band, heading to be inside the release gate, and all conditions to
        remain stable for xtrack_priority_hold_sec.

        This changes only speed ownership. Steering remains continuous and
        exact radial <=30 mm remains the only normal zero-command owner.
        """
        values = (
            signed_cross_track,
            predicted_cross_track,
            path_heading_error,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("non-finite xtrack speed-cap state input")

        error_metric = max(
            abs(signed_cross_track),
            abs(predicted_cross_track),
        )

        if (
            not self.xtrack_priority_active
            and error_metric >= self.xtrack_priority_enter
        ):
            self.xtrack_priority_active = True
            self.xtrack_priority_inside_since = None
            self.get_logger().warn(
                "XTRACK SPEED CAP ENGAGED | "
                f"measured={self.ground_xtrack(signed_cross_track) * 1000.0:+.1f}mm | "
                f"predicted={self.ground_xtrack(predicted_cross_track) * 1000.0:+.1f}mm | "
                f"metric={error_metric * 1000.0:.1f}mm | "
                f"cap={self.xtrack_priority_speed:.3f}m/s"
            )

        if not self.xtrack_priority_active:
            return False, error_metric, 0.0

        release_geometry_valid = (
            abs(signed_cross_track) <= self.xtrack_priority_exit
            and abs(predicted_cross_track) <= self.xtrack_priority_exit
            and abs(path_heading_error) <= self.xtrack_priority_release_heading
        )

        release_elapsed = 0.0
        if release_geometry_valid:
            if self.xtrack_priority_inside_since is None:
                self.xtrack_priority_inside_since = self.get_clock().now()

            release_elapsed = (
                self.get_clock().now() - self.xtrack_priority_inside_since
            ).nanoseconds / 1e9

            if release_elapsed >= self.xtrack_priority_hold_sec:
                self.xtrack_priority_active = False
                self.xtrack_priority_inside_since = None

                # Preserve filter state to avoid a steering discontinuity.
                self.get_logger().warn(
                    "XTRACK SPEED CAP RELEASED | "
                    f"measured={self.ground_xtrack(signed_cross_track) * 1000.0:+.1f}mm | "
                    f"predicted={self.ground_xtrack(predicted_cross_track) * 1000.0:+.1f}mm | "
                    f"heading={math.degrees(path_heading_error):.1f}deg | "
                    f"stable={release_elapsed:.2f}s"
                )
        else:
            self.xtrack_priority_inside_since = None

        return (
            self.xtrack_priority_active,
            error_metric,
            release_elapsed,
        )

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
            -math.sin(path_bearing) * delta_east + math.cos(path_bearing) * delta_north
        )

        now = self.get_clock().now()
        sample_dt = 1.0 / self.CONTROL_HZ

        if (
            self.last_xtrack_sample is not None
            and self.last_xtrack_sample_time is not None
        ):
            sample_dt = max(
                1.0 / self.CONTROL_HZ,
                (now - self.last_xtrack_sample_time).nanoseconds / 1e9,
            )
            raw_rate = (signed_cross_track - self.last_xtrack_sample) / sample_dt
            alpha = self.xtrack_rate_filter_alpha
            self.filtered_xtrack_rate = (
                alpha * raw_rate + (1.0 - alpha) * self.filtered_xtrack_rate
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
                and abs(xtrack_rate) >= self.terminal_xtrack_away_rate_threshold
                and abs(signed_cross_track) > neutral_band
            )

            crossing_projection = (
                signed_cross_track
                + xtrack_rate * self.terminal_xtrack_crossing_prediction_time_sec
            )
            moving_toward_line = signed_cross_track * xtrack_rate < 0.0
            projected_to_cross = (
                signed_cross_track * crossing_projection <= 0.0
                or abs(crossing_projection)
                >= self.terminal_xtrack_crossing_predicted_threshold
                and (signed_cross_track * crossing_projection < 0.0)
            )
            crossing_imminent = (
                not moving_away
                and moving_toward_line
                and abs(xtrack_rate) >= self.terminal_xtrack_crossing_rate_threshold
                and projected_to_cross
            )

            if moving_away:
                prediction_time = self.terminal_xtrack_prediction_time_sec
                lookahead = self.terminal_xtrack_away_lookahead
                correction_limit = self.terminal_xtrack_away_correction_limit
                profile_name = "TERMINAL_AWAY_BOOST"
            elif crossing_imminent:
                prediction_time = self.terminal_xtrack_crossing_prediction_time_sec
                lookahead = self.terminal_xtrack_crossing_lookahead
                correction_limit = self.terminal_xtrack_crossing_correction_limit
                profile_name = "TERMINAL_CROSSING_BRAKE"
            else:
                lookahead = self.terminal_xtrack_lookahead
                correction_limit = self.terminal_xtrack_correction_limit
                profile_name = "TERMINAL_CAPTURE"

            correction_slew_rate = self.terminal_xtrack_correction_slew_rate
            hard_correction_limit = self.terminal_xtrack_away_correction_limit
        else:
            prediction_time = self.xtrack_prediction_time_sec
            lookahead = self.xtrack_priority_lookahead
            correction_limit = self.xtrack_priority_correction_limit
            neutral_band = self.xtrack_neutral_crossing_band
            correction_slew_rate = self.xtrack_correction_slew_rate
            hard_correction_limit = correction_limit

        predicted_cross_track = signed_cross_track + xtrack_rate * prediction_time

        moving_toward_line = signed_cross_track * xtrack_rate < 0.0
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
                (now - self.last_xtrack_correction_time).nanoseconds / 1e9,
            )

        desired_reverses_sign = desired_correction * self.last_xtrack_correction < 0.0
        desired_reduces_magnitude = abs(desired_correction) < abs(
            self.last_xtrack_correction
        )

        if terminal_mode and (desired_reverses_sign or desired_reduces_magnitude):
            active_slew_rate = self.terminal_xtrack_unwind_slew_rate
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
        correction = self.last_xtrack_correction + correction_delta
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
        """Linearly interpolate a monotonic speed section."""
        if high_distance <= low_distance:
            return low_speed
        ratio = (distance - low_distance) / (high_distance - low_distance)
        ratio = max(0.0, min(1.0, ratio))
        return low_speed + ratio * (high_speed - low_speed)

    def speed_for_goal_distance(self, goal_distance):
        return self.cruise_speed

    @staticmethod
    def smoothstep01(value):
        value = max(0.0, min(1.0, value))
        return value * value * (3.0 - 2.0 * value)

    def terminal_speed_for_along_remaining(self, along_remaining):
        """Return smooth semantic-goal deceleration speed.

        The deceleration window is measured from the exact marking coordinate:

            >= 0.50 m from marking point : 1.00 m/s
             0.50 m -> 0.03 m           : smooth constant deceleration
             0.03 m capture boundary    : 0.15 m/s
            <= 0.03 m radial distance   : immediate 0.00 m/s elsewhere

        This function itself never commands zero. The exact radial capture
        latch remains the owner of the final stop.
        """
        if not math.isfinite(along_remaining):
            return self.cruise_speed

        if not self.deceleration_required:
            return self.cruise_speed

        if along_remaining >= self.deceleration_distance:
            return self.cruise_speed

        # Distance still available before the 30 mm stop boundary.
        distance_to_boundary = max(
            0.0,
            along_remaining - self.waypoint_tolerance,
        )

        speed_squared = (
            self.deceleration_floor_speed * self.deceleration_floor_speed
            + 2.0 * self.deceleration_rate * distance_to_boundary
        )
        profile_speed = math.sqrt(max(0.0, speed_squared))

        return max(
            self.deceleration_floor_speed,
            min(self.cruise_speed, profile_speed),
        )

    def terminal_correction_limit_for_along(self, along_remaining):
        """Smoothly reduce steering authority without removing it.

        The previous build froze the bearing and then entered HOLD when the
        uneven surface created additional cross-track error. For the 2 m test
        we retain a small correction (terminal_near_correction_limit) until
        the exact 30 mm stop latch.
        """
        self.terminal_bearing_frozen = False
        if not math.isfinite(along_remaining):
            return self.terminal_decel_correction_limit

        distance = max(self.waypoint_tolerance, along_remaining)
        if distance >= self.terminal_near_correction_start_distance:
            return self.terminal_decel_correction_limit

        span = max(
            1.0e-6,
            self.terminal_near_correction_start_distance - self.waypoint_tolerance,
        )
        ratio = (distance - self.waypoint_tolerance) / span
        blend = self.smoothstep01(ratio)
        return self.terminal_near_correction_limit + blend * (
            self.terminal_decel_correction_limit - self.terminal_near_correction_limit
        )

    def terminal_bounded_guidance(
        self,
        path_bearing,
        desired_bearing,
        along_remaining,
    ):
        """Use the same line-guidance demand with smooth bounded correction."""
        limit = self.terminal_correction_limit_for_along(along_remaining)
        requested = self.normalize_angle(desired_bearing - path_bearing)
        requested = max(-limit, min(limit, requested))

        now = self.get_clock().now()
        if self.terminal_correction_last_update_time is None:
            # Match the correction that was active before entering the
            # terminal branch. This removes the sudden snap back to zero.
            self.terminal_limited_correction = requested
            dt = 1.0 / self.CONTROL_HZ
        else:
            dt = (now - self.terminal_correction_last_update_time).nanoseconds / 1e9
            if not math.isfinite(dt) or dt <= 0.0:
                dt = 1.0 / self.CONTROL_HZ
            dt = min(dt, self.deceleration_max_dt_sec)
        self.terminal_correction_last_update_time = now

        maximum_change = self.terminal_correction_slew_rate * dt
        correction_delta = requested - self.terminal_limited_correction
        correction_delta = max(
            -maximum_change,
            min(maximum_change, correction_delta),
        )
        self.terminal_limited_correction += correction_delta
        self.terminal_limited_correction = max(
            -limit,
            min(limit, self.terminal_limited_correction),
        )
        return self.normalize_angle(path_bearing + self.terminal_limited_correction)

    def publish_terminal_state(self):
        armed = Bool()
        armed.data = bool(self.terminal_precision_armed)
        self.terminal_precision_armed_pub.publish(armed)

        frozen = Bool()
        frozen.data = bool(self.terminal_bearing_frozen)
        self.terminal_bearing_frozen_pub.publish(frozen)

        self._publish_float64(
            self.terminal_correction_deg_pub,
            math.degrees(self.terminal_limited_correction),
        )

    def apply_heading_speed_limit(self, base_speed, heading_error):
        """Heading changes direction only, never speed magnitude."""
        return self.cruise_speed

    def limit_moving_guidance_bearing(self, desired_bearing):
        """Keep moving recovery below the PX4 45-degree pivot threshold."""
        command_error = self.normalize_angle(desired_bearing - self.current_yaw)
        command_error = max(
            -self.MAX_MOVING_HEADING_ERROR_RAD,
            min(
                self.MAX_MOVING_HEADING_ERROR_RAD,
                command_error,
            ),
        )
        limited_bearing = self.normalize_angle(self.current_yaw + command_error)
        return limited_bearing, command_error

    def bounded_bearing(self, base_bearing, desired_bearing, limit):
        correction = self.normalize_angle(desired_bearing - base_bearing)
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
        Follow the local tangent line through the active /nav_path cursor.
        The cursor advances through the trajectory generator's 50 mm points;
        interpolation points shape guidance but never become stop goals.

        The lookahead is speed- and cross-track-adaptive. A fixed lookahead
        reacts far ahead in TIME at low speed and close in time at cruise --
        inconsistent dynamic response across the accel/decel speed range.
        Scaling with the last commanded translational speed keeps look-ahead
        time roughly constant instead (line_tracking_lookahead_speed_gain is
        line_tracking_lookahead_m / cruise_speed_mps, so behaviour at cruise
        is unchanged from the previous fixed-lookahead tuning). The xtrack
        term widens the lookahead on large deviations for a softer
        re-acquisition curve instead of saturating straight into the
        correction-limit clamp below. Drivetrain-agnostic: this only shapes
        the commanded bearing/velocity vector, the same output contract used
        by every caller regardless of how PX4 turns it into wheel commands.
        """
        delta_east = self.current_x - line_point_x
        delta_north = self.current_y - line_point_y

        signed_cross_track = (
            -math.sin(line_bearing) * delta_east + math.cos(line_bearing) * delta_north
        )

        lookahead = (
            self.line_tracking_lookahead_speed_gain * abs(self.command_slew_speed)
            + self.line_tracking_lookahead_xtrack_gain * abs(signed_cross_track)
        )
        lookahead = max(
            self.line_tracking_lookahead_min,
            min(self.line_tracking_lookahead_max, lookahead),
        )

        correction = -math.atan2(
            signed_cross_track,
            lookahead,
        )
        correction = max(
            -correction_limit,
            min(correction_limit, correction),
        )

        guidance_bearing = self.normalize_angle(line_bearing + correction)
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
            math.cos(path_bearing) * delta_east + math.sin(path_bearing) * delta_north
        )

    def _terminal_cross_internal(self, path_bearing, target_x, target_y):
        """Return the internal terminal Cross measurement.

        Internal convention: LEFT = positive, RIGHT = negative. Shared by the
        live precision-terminal cycle and its hold/refresh path so both use
        the identical formula against the same latched bearing.
        """
        goal_delta_east = self.current_x - target_x
        goal_delta_north = self.current_y - target_y
        return (
            -math.sin(path_bearing) * goal_delta_east
            + math.cos(path_bearing) * goal_delta_north
        )

    def apply_alignment_release_ramp(self, requested_speed):
        """Compatibility helper; preserve the configured acceleration slew."""
        return self.command_speed_slew_limit(requested_speed)

    #
    #
    # Current branch signature is preserved exactly.

    def _reset_precision_terminal(self, reason):
        """Reset Phase-5 authority only at an explicit semantic boundary."""

        if not hasattr(self, "precision_terminal_fsm"):
            return
        now_sec = self._precision_now_sec()
        try:
            self.precision_terminal_fsm.reset(
                monotonic_time_sec=now_sec,
                semantic_boundary_reason=str(reason),
            )
        except ValueError:
            # A ROS-time regression is itself an explicit control boundary.
            self.precision_terminal_fsm = TerminalStopStateMachine(
                self.precision_terminal_config
            )
        self.precision_terminal_cycle_token = None
        self.precision_terminal_last_result = None
        self.precision_terminal_request_armed = False
        self.precision_terminal_identity = None
        self.precision_terminal_identity_components = None
        self.precision_terminal_last_sample = None
        self.precision_terminal_speed_override_mps = None
        self.precision_terminal_historical_certificate = None
        self.precision_terminal_measurement_bearing = None
        self.precision_terminal_measurement_bearing_source = None
        self.precision_terminal_last_reset_reason = str(reason)
        self.precision_terminal_reset_count += 1
        self._reset_radial20_terminal(reason)

    def _reset_radial20_terminal(self, reason):
        """Reset radial20 authority at the same semantic boundaries as Phase-5.

        Called from _reset_precision_terminal() so every one of its existing
        call sites (goal change, mission enable/disable, e-stop, geometry
        invalidation, localization jump, point completion, ...) resets
        radial20 identically without duplicating those call sites.
        """

        if not hasattr(self, "radial_stop_regulator"):
            return
        self.radial_stop_regulator.reset()
        self.radial_stop_request_armed = False
        self.radial_stop_identity = None
        self.radial_stop_identity_components = None
        self.radial_stop_last_result = None
        self.radial_stop_last_sample = None
        self._radial_stop_speed_last_sample = None

    def _current_precision_terminal_identity(self):
        """Return a synchronized, run-scoped semantic terminal identity."""

        binding = self.geometry_goal_binding
        metadata = self.geometry_pending_goal_metadata
        if not (
            self.geometry_contract_synchronized
            and binding is not None
            and isinstance(metadata, dict)
            and self.geometry_installed_signature is not None
            and metadata.get("path_signature") == self.geometry_installed_signature
            and metadata.get("raw_path_index") == binding.raw_path_index
            and metadata.get("active_goal_identity") == binding.active_goal_identity
        ):
            return None, None
        mission_run_id = metadata.get("mission_run_id")
        goal_instance_id = metadata.get("goal_instance_id")
        if not (
            isinstance(mission_run_id, str)
            and mission_run_id.strip()
            and isinstance(goal_instance_id, str)
            and goal_instance_id.strip()
        ):
            return None, None
        components = {
            "mission_run_id": mission_run_id.strip(),
            "path_signature": self.geometry_installed_signature,
            "raw_path_index": int(binding.raw_path_index),
            "active_goal_identity": str(binding.active_goal_identity),
            "goal_instance_id": goal_instance_id.strip(),
        }
        identity = (
            f"RUN:{components['mission_run_id']}|"
            f"PATH:{components['path_signature']}|"
            f"RAW:{components['raw_path_index']}|"
            f"GOAL:{components['active_goal_identity']}|"
            f"INSTANCE:{components['goal_instance_id']}"
        )
        return identity, components

    def _resolve_precision_terminal_measurement_bearing(
        self,
        *,
        first_approach,
        path_bearing,
    ):
        """Resolve and latch the terminal Along/Cross measurement tangent.

        Latched at most once per terminal identity; frozen thereafter until
        the next _reset_precision_terminal() semantic boundary. Never derives
        a bearing from self.current_yaw, a literal 0.0, or a custom segment
        scan -- only runtime-entry authority, the resolved semantic anchor,
        or the already-resolved active-nav path_bearing fallback.
        """

        if self.precision_terminal_measurement_bearing is not None:
            return (
                self.precision_terminal_measurement_bearing,
                self.precision_terminal_measurement_bearing_source,
            )

        if first_approach:
            bearing = self.c_line_bearing
            source = "RUNTIME_ENTRY_C_TO_P1"
            if bearing is None or not math.isfinite(bearing):
                return None, None
        else:
            bearing = None
            source = None
            binding = self.geometry_goal_binding
            if self.path_geometry is not None and binding is not None:
                anchor = self.path_geometry.semantic_anchor_at(
                    binding.raw_path_index
                )
                if (
                    anchor is not None
                    and anchor.incoming_heading_rad is not None
                    and math.isfinite(anchor.incoming_heading_rad)
                ):
                    bearing = anchor.incoming_heading_rad
                    source = "SEMANTIC_INCOMING"
            if bearing is None:
                if path_bearing is not None and math.isfinite(path_bearing):
                    bearing = path_bearing
                    source = "ACTIVE_NAV_FALLBACK"
                    self.get_logger().warn(
                        "PRECISION TERMINAL MEASUREMENT TANGENT / "
                        "SEMANTIC INCOMING HEADING UNAVAILABLE / "
                        "ACTIVE_NAV_FALLBACK | "
                        f"path_bearing={math.degrees(path_bearing):.1f}deg"
                    )
                else:
                    return None, None

        self.precision_terminal_measurement_bearing = bearing
        self.precision_terminal_measurement_bearing_source = source
        self.get_logger().warn(
            "PRECISION TERMINAL MEASUREMENT TANGENT LATCHED | "
            f"bearing={math.degrees(bearing):.1f}deg | source={source}"
        )
        return bearing, source

    def _check_precision_terminal_measurement_consistency(
        self,
        *,
        radial_error_m,
        along_error_m,
        cross_error_m,
    ):
        """Diagnostic-only radial**2 ~= along**2 + cross**2 sanity check.

        Never raises and never influences FSM state, speed, or control.
        """

        if not all(
            math.isfinite(value)
            for value in (radial_error_m, along_error_m, cross_error_m)
        ):
            return
        residual_m2 = (radial_error_m ** 2) - (
            along_error_m ** 2 + cross_error_m ** 2
        )
        if abs(residual_m2) > 1.0e-6:
            self.get_logger().warn(
                "PRECISION TERMINAL MEASUREMENT CONSISTENCY WARNING | "
                f"radial_m={radial_error_m:.6f} | "
                f"along_m={along_error_m:.6f} | "
                f"cross_m={cross_error_m:.6f} | "
                f"residual_m2={residual_m2:.9f}"
            )

    def _step_precision_terminal_for_cycle(
        self,
        *,
        goal_distance,
        path_heading_error,
        first_approach,
        path_bearing,
        goal_x,
        goal_y,
    ):
        """Step the terminal FSM exactly once using current-cycle geometry."""

        if self.precision_terminal_cycle_token == self.precision_cycle_token:
            return self.precision_terminal_last_result
        projection = self._current_cycle_projection()
        guidance = self.precision_guidance_result
        identity, components = self._current_precision_terminal_identity()
        if projection is None or guidance is None or identity is None:
            return None

        (
            latched_terminal_bearing,
            _latched_terminal_bearing_source,
        ) = self._resolve_precision_terminal_measurement_bearing(
            first_approach=first_approach,
            path_bearing=path_bearing,
        )
        if latched_terminal_bearing is None:
            return None

        now_sec = self._precision_now_sec()
        telemetry_fresh = self.is_fresh(
            self.last_odom_time,
            self.precision_terminal_telemetry_timeout_sec,
        )
        # Control remaining distance: unchanged clamped span projection.
        # This alone continues to feed distance_to_terminal_m / brake gating.
        control_remaining_m = max(
            0.0,
            float(self.geometry_active_span.stop_s - projection.projected_s),
        )
        # Signed measurement Along: latched-tangent projection of the exact
        # semantic goal. + = before waypoint, - = past/overshoot.
        terminal_along_error_m = self.along_track_remaining(
            latched_terminal_bearing,
            goal_x,
            goal_y,
        )
        # Internal measurement Cross: LEFT = positive, RIGHT = negative.
        terminal_cross_internal_m = self._terminal_cross_internal(
            latched_terminal_bearing,
            goal_x,
            goal_y,
        )
        self._check_precision_terminal_measurement_consistency(
            radial_error_m=float(goal_distance),
            along_error_m=terminal_along_error_m,
            cross_error_m=terminal_cross_internal_m,
        )
        sample = TerminalInput(
            monotonic_time_sec=now_sec,
            dt_sec=self.precision_cycle_dt_sec,
            terminal_requested=True,
            terminal_identity=identity,
            distance_to_terminal_m=control_remaining_m,
            radial_error_m=float(goal_distance),
            cross_track_error_m=terminal_cross_internal_m,
            along_track_error_m=terminal_along_error_m,
            measured_linear_speed_mps=float(self.current_speed_mps),
            measured_yaw_rate_radps=float(self.current_yaw_rate_radps),
            telemetry_fresh=telemetry_fresh,
            braking_required=(
                control_remaining_m
                <= self.precision_terminal_config.brake_distance_m
            ),
            heading_error_deg=math.degrees(path_heading_error),
            current_pose=ControllerPose(
                x_m=float(self.current_x),
                y_m=float(self.current_y),
                yaw_rad=float(self.current_yaw),
            ),
        )
        try:
            result = self.precision_terminal_fsm.step(sample)
        except (TypeError, ValueError):
            return None
        self.precision_terminal_cycle_token = self.precision_cycle_token
        self.precision_terminal_last_result = result
        self.precision_terminal_last_sample = sample
        self.precision_terminal_request_armed = True
        self.precision_terminal_identity = identity
        self.precision_terminal_identity_components = components
        if result.certificate is not None:
            self.precision_terminal_historical_certificate = result.certificate
        if result.directive in {
            TerminalDirective.APPROACH,
            TerminalDirective.BRAKE,
        }:
            self.precision_terminal_speed_override_mps = (
                self.precision_terminal_config.minimum_actuatable_speed_mps
            )
        self._publish_precision_terminal_heartbeat()
        return result

    def _step_precision_terminal_stale_cycle(self):
        """Revoke live validity on stale telemetry without clearing zero latch."""

        prior = self.precision_terminal_last_sample
        if not self.precision_terminal_request_armed or prior is None:
            return None
        if self.precision_terminal_cycle_token == self.precision_cycle_token:
            return self.precision_terminal_last_result
        sample = TerminalInput(
            monotonic_time_sec=self._precision_now_sec(),
            dt_sec=self.precision_cycle_dt_sec,
            terminal_requested=True,
            terminal_identity=self.precision_terminal_identity,
            distance_to_terminal_m=prior.distance_to_terminal_m,
            radial_error_m=prior.radial_error_m,
            cross_track_error_m=prior.cross_track_error_m,
            along_track_error_m=prior.along_track_error_m,
            measured_linear_speed_mps=prior.measured_linear_speed_mps,
            measured_yaw_rate_radps=prior.measured_yaw_rate_radps,
            telemetry_fresh=False,
            braking_required=True,
            heading_error_deg=prior.heading_error_deg,
            current_pose=prior.current_pose,
        )
        try:
            result = self.precision_terminal_fsm.step(sample)
        except (TypeError, ValueError):
            return None
        self.precision_terminal_cycle_token = self.precision_cycle_token
        self.precision_terminal_last_result = result
        self.precision_terminal_last_sample = sample
        if result.certificate is not None:
            self.precision_terminal_historical_certificate = result.certificate
        self._publish_precision_terminal_heartbeat()
        return result

    def _step_precision_terminal_hold_cycle(self):
        """Refresh a latched/certified stop while Mission Manager verifies it."""

        prior = self.precision_terminal_last_sample
        if not self.precision_terminal_request_armed or prior is None:
            return None
        if self.precision_terminal_cycle_token == self.precision_cycle_token:
            return self.precision_terminal_last_result
        telemetry_fresh = self.is_fresh(
            self.last_odom_time,
            self.precision_terminal_telemetry_timeout_sec,
        )
        pose_valid = all(
            value is not None and math.isfinite(float(value))
            for value in (self.current_x, self.current_y, self.current_yaw)
        )
        goal_valid = all(
            value is not None and math.isfinite(float(value))
            for value in (self.segment_goal_x, self.segment_goal_y)
        )
        # A refreshed radial must never be combined with a stale Along/Cross
        # (or vice versa): either all three come from the current pose/goal,
        # or the entire prior geometric measurement is preserved together.
        latched_terminal_bearing = self.precision_terminal_measurement_bearing
        geometry_refreshable = (
            pose_valid and goal_valid and latched_terminal_bearing is not None
        )
        if geometry_refreshable:
            radial_error = math.hypot(
                self.segment_goal_x - self.current_x,
                self.segment_goal_y - self.current_y,
            )
            along_error = self.along_track_remaining(
                latched_terminal_bearing,
                self.segment_goal_x,
                self.segment_goal_y,
            )
            cross_error = self._terminal_cross_internal(
                latched_terminal_bearing,
                self.segment_goal_x,
                self.segment_goal_y,
            )
        else:
            radial_error = prior.radial_error_m
            along_error = prior.along_track_error_m
            cross_error = prior.cross_track_error_m
        self._check_precision_terminal_measurement_consistency(
            radial_error_m=radial_error,
            along_error_m=along_error,
            cross_error_m=cross_error,
        )
        sample = TerminalInput(
            monotonic_time_sec=self._precision_now_sec(),
            dt_sec=self.precision_cycle_dt_sec,
            terminal_requested=True,
            terminal_identity=self.precision_terminal_identity,
            distance_to_terminal_m=prior.distance_to_terminal_m,
            radial_error_m=radial_error,
            cross_track_error_m=cross_error,
            along_track_error_m=along_error,
            measured_linear_speed_mps=float(self.current_speed_mps),
            measured_yaw_rate_radps=float(self.current_yaw_rate_radps),
            telemetry_fresh=telemetry_fresh,
            braking_required=True,
            heading_error_deg=prior.heading_error_deg,
            current_pose=(
                ControllerPose(
                    x_m=float(self.current_x),
                    y_m=float(self.current_y),
                    yaw_rad=float(self.current_yaw),
                )
                if pose_valid
                else prior.current_pose
            ),
        )
        try:
            result = self.precision_terminal_fsm.step(sample)
        except (TypeError, ValueError):
            return None
        self.precision_terminal_cycle_token = self.precision_cycle_token
        self.precision_terminal_last_result = result
        self.precision_terminal_last_sample = sample
        if result.certificate is not None:
            self.precision_terminal_historical_certificate = result.certificate
        self._publish_precision_terminal_heartbeat()
        self._publish_precision_terminal_result_if_ready(result)
        return result

    def _publish_precision_terminal_heartbeat(self):
        """Publish non-authoritative live FSM evidence; failures are isolated."""

        if not self.precision_terminal_request_armed:
            return
        result = self.precision_terminal_last_result
        sample = self.precision_terminal_last_sample
        components = self.precision_terminal_identity_components or {}
        if result is None or sample is None:
            return
        certificate = result.certificate
        try:
            payload = {
                "schema_version": 2,
                "source": "RPP_PRECISION_TERMINAL_HEARTBEAT",
                "ros_time_ns": self.get_clock().now().nanoseconds,
                "precision_terminal_enabled": self.precision_terminal_enabled,
                "state": result.state.value,
                "directive": result.directive.value,
                "zero_latched": result.zero_latched,
                "motion_evidence_valid": result.motion_evidence_valid,
                "currently_valid": result.currently_valid,
                "transition_reason": result.transition_reason,
                "terminal_identity": result.terminal_identity,
                "terminal_identity_components": components,
                "mission_run_id": components.get("mission_run_id"),
                "goal_instance_id": components.get("goal_instance_id"),
                "path_signature": components.get("path_signature"),
                "raw_path_index": components.get("raw_path_index"),
                "active_goal_identity": components.get("active_goal_identity"),
                "radial_error_mm": sample.radial_error_m * 1000.0,
                "cross_error_mm": (
                    self.ground_xtrack(sample.cross_track_error_m) * 1000.0
                ),
                "along_error_mm": sample.along_track_error_m * 1000.0,
                "heading_error_deg": sample.heading_error_deg,
                "measured_speed_mps": sample.measured_linear_speed_mps,
                "measured_yaw_rate_radps": sample.measured_yaw_rate_radps,
                "telemetry_fresh": sample.telemetry_fresh,
                "settle_held_sec": result.settle_held_sec,
                "max_radial_during_settle_mm": (
                    certificate.max_radial_during_settle_mm
                    if certificate is not None
                    else None
                ),
                "certificate": self.ground_terminal_certificate_payload(certificate),
                "precision_certificate_version": (
                    certificate.version if certificate is not None else None
                ),
                "precision_pass": bool(
                    certificate is not None and certificate.precision_pass
                ),
                "truth_frame": "controller_estimator_frame_only",
                "localization_accuracy_certified": False,
                "physical_accuracy_certified": False,
                "reset_reason": self.precision_terminal_last_reset_reason,
                "reset_count": self.precision_terminal_reset_count,
            }
            message = String()
            message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
            self.terminal_certificate_pub.publish(message)
        except Exception as error:  # diagnostics never alter stop authority
            try:
                self.get_logger().error(
                    f"TERMINAL CERTIFICATE HEARTBEAT PUBLISH FAILED | {error}"
                )
            except Exception:
                pass

    def _publish_precision_terminal_result_if_ready(self, result):
        """Bridge FSM completion to the unchanged legacy result topic once."""

        if result is None or self._terminal_result_sent is not None:
            return
        sample = self.precision_terminal_last_sample
        if sample is None:
            return
        certificate = result.certificate
        if (
            result.state is TerminalState.CERTIFIED
            and result.currently_valid
            and certificate is not None
            and certificate.version == 2
        ):
            self.publish_terminal_result(
                "CAPTURED",
                reason="PRECISION_TERMINAL_CERTIFIED_V2",
                target_distance=sample.radial_error_m,
                signed_cross_track=sample.cross_track_error_m,
                along_remaining=sample.along_track_error_m,
                precision_certificate=certificate,
                tolerance_override_m=(
                    self.precision_terminal_config.terminal_radial_tolerance_m
                ),
            )
        elif (
            result.state is TerminalState.HOLD_FAIL
            and self.precision_terminal_historical_certificate is None
        ):
            self.publish_terminal_result(
                "MISSED",
                reason=result.transition_reason or "PRECISION_TERMINAL_FAILED",
                target_distance=sample.radial_error_m,
                signed_cross_track=sample.cross_track_error_m,
                along_remaining=sample.along_track_error_m,
                precision_certificate=None,
                tolerance_override_m=(
                    self.precision_terminal_config.terminal_radial_tolerance_m
                ),
            )

    def _radial_stop_position_sample_time_sec(self):
        """Return a real sensor-timestamped sample time for the detector.

        Uses the actual odometry timestamp (not the control-loop's own
        monotonic cycle time) so the stationary detector's duplicate-sample
        and sample-gap handling see genuine odom cadence, per its own
        designed-for input contract.
        """

        if self.last_odom_time is None:
            return self._precision_now_sec()
        return max(0.0, self.last_odom_time.nanoseconds / 1.0e9)

    def _radial_stop_position_derived_speed_mps(self, position_sample_time_sec):
        """Differentiate position over the most recent sample gap.

        Never derived from EKF twist. Returns 0.0 (never a negative or
        stale value) whenever the gap is invalid/too large/non-finite so the
        regulator's max(position_derived, commanded) braking-lead formula
        conservatively falls back to the commanded speed instead of
        under-estimating danger.
        """

        last = self._radial_stop_speed_last_sample
        current_x = float(self.current_x)
        current_y = float(self.current_y)
        self._radial_stop_speed_last_sample = (
            position_sample_time_sec,
            current_x,
            current_y,
        )
        if last is None:
            return 0.0
        last_t, last_x, last_y = last
        dt = position_sample_time_sec - last_t
        if (
            not math.isfinite(dt)
            or dt <= 1.0e-6
            or dt > self.radial_stop_config.maximum_position_sample_gap_sec
            or not all(
                math.isfinite(value)
                for value in (current_x, current_y, last_x, last_y)
            )
        ):
            return 0.0
        distance = math.hypot(current_x - last_x, current_y - last_y)
        return max(0.0, distance / dt)

    def _step_radial20_terminal_for_cycle(
        self,
        *,
        along_remaining,
        cross_error,
    ):
        """Advance the radial20 regulator for one control cycle.

        Mirrors _step_precision_terminal_for_cycle's identity/telemetry
        pattern but feeds TerminalStopRegulator instead of the Phase-5 FSM.
        """

        identity, components = self._current_precision_terminal_identity()
        if identity is None:
            return None
        now_sec = self._precision_now_sec()
        telemetry_fresh = self.is_fresh(
            self.last_odom_time,
            self.radial_stop_telemetry_timeout_sec,
        )
        position_sample_time_sec = self._radial_stop_position_sample_time_sec()
        position_derived_speed = self._radial_stop_position_derived_speed_mps(
            position_sample_time_sec
        )
        tracking_speed_command = max(
            0.0,
            float(self.precision_last_published_translational_speed_mps),
        )
        yaw_rate = (
            float(self.current_yaw_rate_radps)
            if math.isfinite(self.current_yaw_rate_radps)
            else 0.0
        )
        sample = RadialStopInput(
            monotonic_time_sec=now_sec,
            position_sample_time_sec=position_sample_time_sec,
            active=True,
            terminal_identity=identity,
            along_remaining_m=float(along_remaining),
            cross_error_m=float(cross_error),
            position_x_m=float(self.current_x),
            position_y_m=float(self.current_y),
            position_derived_speed_mps=position_derived_speed,
            measured_yaw_rate_radps=yaw_rate,
            tracking_speed_command_mps=tracking_speed_command,
            telemetry_fresh=telemetry_fresh,
        )
        try:
            result = self.radial_stop_regulator.step(sample)
        except (TypeError, ValueError):
            return None
        self.radial_stop_request_armed = True
        self.radial_stop_identity = identity
        self.radial_stop_identity_components = components
        self.radial_stop_last_result = result
        self.radial_stop_last_sample = sample
        self._publish_radial20_heartbeat(result, sample)
        return result

    def _radial_stop_certificate_payload(self, certificate):
        """Convert a RadialStopCertificate for external reporting.

        Ground/report cross-track convention (LEFT negative, RIGHT positive)
        matches ground_terminal_certificate_payload's contract for the
        Phase-5 certificate.
        """

        if certificate is None:
            return None
        return {
            "version": certificate.version,
            "terminal_identity": certificate.terminal_identity,
            "certified_timestamp_sec": certificate.certified_timestamp_sec,
            "radial_error_m": certificate.radial_error_m,
            "along_error_m": certificate.along_error_m,
            "cross_error_m": self.ground_xtrack(certificate.cross_error_m),
            "final_position_x_m": certificate.final_position_x_m,
            "final_position_y_m": certificate.final_position_y_m,
            "stationary_window_sec": certificate.stationary_window_sec,
            "maximum_stationary_displacement_m": (
                certificate.maximum_stationary_displacement_m
            ),
            "maximum_abs_yaw_rate_radps": (
                certificate.maximum_abs_yaw_rate_radps
            ),
            "speed_source": certificate.speed_source,
            "truth_frame": certificate.truth_frame,
            "precision_pass": True,
        }

    def _publish_radial20_heartbeat(self, result, sample):
        """Publish the radial20 heartbeat on the shared /rpp/terminal_certificate
        wire schema mission_manager's existing precision-terminal consumer
        already validates (schema_version 2 / RPP_PRECISION_TERMINAL_HEARTBEAT).
        "precision_terminal_enabled": True below is a fixed wire-contract
        literal meaning "this heartbeat is certificate-backed" -- it is not
        self.precision_terminal_enabled, which stays scoped to the Phase-5
        FSM branch for legacy-latch mutual exclusion.
        """

        components = self.radial_stop_identity_components or {}
        certificate = result.certificate
        currently_valid = result.state in (
            RadialStopState.CERTIFIED,
            RadialStopState.HOLD_ZERO,
        )
        try:
            payload = {
                "schema_version": 2,
                "source": "RPP_PRECISION_TERMINAL_HEARTBEAT",
                "ros_time_ns": self.get_clock().now().nanoseconds,
                "precision_terminal_enabled": True,
                "state": result.state.value,
                "directive": (
                    "hold_zero" if result.hold_zero else "approach"
                ),
                "zero_latched": bool(result.hold_zero),
                "motion_evidence_valid": bool(result.stationary),
                "currently_valid": currently_valid,
                "transition_reason": (
                    result.failure.value
                    if result.failure.value != "none"
                    else None
                ),
                "terminal_identity": self.radial_stop_identity,
                "terminal_identity_components": components,
                "mission_run_id": components.get("mission_run_id"),
                "goal_instance_id": components.get("goal_instance_id"),
                "path_signature": components.get("path_signature"),
                "raw_path_index": components.get("raw_path_index"),
                "active_goal_identity": components.get("active_goal_identity"),
                "radial_error_mm": result.radial_error_m * 1000.0,
                "cross_error_mm": (
                    self.ground_xtrack(sample.cross_error_m) * 1000.0
                ),
                "along_error_mm": sample.along_remaining_m * 1000.0,
                "measured_speed_mps": sample.position_derived_speed_mps,
                "measured_yaw_rate_radps": sample.measured_yaw_rate_radps,
                "telemetry_fresh": sample.telemetry_fresh,
                "settle_held_sec": result.stationary_window_sec,
                "max_radial_during_settle_mm": (
                    certificate.radial_error_m * 1000.0
                    if certificate is not None
                    else None
                ),
                "certificate": self._radial_stop_certificate_payload(certificate),
                "precision_certificate_version": (
                    certificate.version if certificate is not None else None
                ),
                "precision_pass": bool(certificate is not None),
                "truth_frame": "controller_estimator_frame_only",
                "localization_accuracy_certified": False,
                "physical_accuracy_certified": False,
                "reset_reason": None,
                "reset_count": 0,
            }
            message = String()
            message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
            self.terminal_certificate_pub.publish(message)
        except Exception as error:  # diagnostics never alter stop authority
            try:
                self.get_logger().error(
                    f"RADIAL20 CERTIFICATE HEARTBEAT PUBLISH FAILED | {error}"
                )
            except Exception:
                pass

    def _publish_radial20_result_if_ready(self, result):
        """Bridge radial20 completion to the shared terminal-result topic."""

        if result is None or self._terminal_result_sent is not None:
            return
        sample = self.radial_stop_last_sample
        if sample is None:
            return
        certificate = result.certificate
        if result.state is RadialStopState.CERTIFIED and certificate is not None:
            self.publish_terminal_result(
                "CAPTURED",
                reason="RADIAL20_CERTIFIED",
                target_distance=result.radial_error_m,
                signed_cross_track=sample.cross_error_m,
                along_remaining=sample.along_remaining_m,
                precision_certificate=certificate,
                tolerance_override_m=self.radial_stop_config.radial_tolerance_m,
            )
        elif result.state is RadialStopState.HOLD_FAIL:
            self.publish_terminal_result(
                "MISSED",
                reason=f"RADIAL20_{result.failure.value.upper()}",
                target_distance=result.radial_error_m,
                signed_cross_track=sample.cross_error_m,
                along_remaining=sample.along_remaining_m,
                precision_certificate=None,
                tolerance_override_m=self.radial_stop_config.radial_tolerance_m,
            )

    def publish_terminal_result(
        self,
        outcome,
        *,
        reason,
        target_distance,
        signed_cross_track,
        along_remaining,
        precision_certificate=None,
        tolerance_override_m=None,
    ):
        """Publish the authoritative terminal result for the active semantic goal.

        RPP is the ONLY owner of final marking accuracy.

        Downstream nodes may store/display these values but must never reconstruct
        the final marking accuracy from odometry, GPS, target coordinates or live
        telemetry.

        ``tolerance_override_m`` reports the tolerance this specific result was
        actually certified against: None keeps the existing
        ``self.waypoint_tolerance`` (legacy) behavior; an explicit value is used
        for this call only and does not alter ``self.waypoint_tolerance``.
        """
        outcome = str(outcome).strip().upper()

        if outcome not in {"CAPTURED", "MISSED"}:
            return

        if self._terminal_result_sent is not None:
            return

        if self.segment_goal_x is None or self.segment_goal_y is None:
            return

        # --------------------------------------------------------------
        # FINAL ACCURACY IS CALCULATED ONCE -- HERE IN RPP.
        # --------------------------------------------------------------

        radial_m = float(target_distance)

        # Convert only for external/final accuracy reporting.
        cross_track_m = self.ground_xtrack(signed_cross_track)

        along_track_m = float(along_remaining)
        tolerance_m = (
            float(self.waypoint_tolerance)
            if tolerance_override_m is None
            else float(tolerance_override_m)
        )

        radial_mm = radial_m * 1000.0
        cross_track_mm = cross_track_m * 1000.0
        along_track_mm = along_track_m * 1000.0
        tolerance_mm = tolerance_m * 1000.0

        payload = {
            "source": "RPP_TERMINAL_RESULT",
            "measurement_source": "RPP_TERMINAL_RESULT",
            "available": True,
            "outcome": outcome,
            "reason": str(reason or ""),
            "goal_x": float(self.segment_goal_x),
            "goal_y": float(self.segment_goal_y),
            "marking_number": int(self.segment_goal_number or 0),
            "is_marking": bool((self.segment_goal_number or 0) > 0),
            # Raw SI values kept for diagnostics/compatibility.
            "radial_error_m": radial_m,
            "cross_track_error_m": cross_track_m,
            "along_track_remaining_m": along_track_m,
            "along_track_error_m": along_track_m,
            "tolerance_m": tolerance_m,
            # ----------------------------------------------------------
            # CANONICAL FINAL MISSION-REPORT VALUES.
            # No downstream node should recalculate these.
            # ----------------------------------------------------------
            "cross_track_error_mm": cross_track_mm,
            "along_track_error_mm": along_track_mm,
            "along_track_remaining_mm": along_track_mm,
            "radial_error_mm": radial_mm,
            "overall_accuracy_mm": radial_mm,
            "total_accuracy_mm": radial_mm,
            "tolerance_mm": tolerance_mm,
            # RPP outcome is already the authoritative decision.
            "within_tolerance": outcome == "CAPTURED",
            "speed_mps": (
                float(self.current_speed_mps)
                if math.isfinite(self.current_speed_mps)
                else None
            ),
            "stop_commanded": True,
            "timestamp_unix_ns": self.get_clock().now().nanoseconds,
        }

        if self.radial20_active:
            # radial20 certificates are RadialStopCertificate, a distinct
            # type from PrecisionTerminalCertificate below -- never pass one
            # through the Phase-5 branch's attribute access.
            components = self.radial_stop_identity_components or {}
            certificate_payload = self._radial_stop_certificate_payload(
                precision_certificate
            )
            payload.update(
                {
                    "controller_outcome": outcome,
                    "precision_certificate_version": (
                        precision_certificate.version
                        if precision_certificate is not None
                        else 2
                    ),
                    "terminal_identity": self.radial_stop_identity,
                    "terminal_identity_components": components,
                    "mission_run_id": components.get("mission_run_id"),
                    "goal_instance_id": components.get("goal_instance_id"),
                    "path_signature": components.get("path_signature"),
                    "raw_path_index": components.get("raw_path_index"),
                    "active_goal_identity": components.get("active_goal_identity"),
                    "precision_pass": bool(precision_certificate is not None),
                    "cross_error_mm": cross_track_mm,
                    "along_error_mm": along_track_mm,
                    "stop_spec_mm": (
                        self.radial_stop_config.radial_tolerance_m * 1000.0
                    ),
                    "precision_stop_spec_mm": (
                        self.radial_stop_config.radial_tolerance_m * 1000.0
                    ),
                    "measured_yaw_rate_radps": (
                        float(self.current_yaw_rate_radps)
                        if math.isfinite(self.current_yaw_rate_radps)
                        else None
                    ),
                    "speed_at_release_mps": (
                        float(self.current_speed_mps)
                        if math.isfinite(self.current_speed_mps)
                        else None
                    ),
                    "yaw_rate_at_release_radps": (
                        precision_certificate.maximum_abs_yaw_rate_radps
                        if precision_certificate is not None
                        else (
                            float(self.current_yaw_rate_radps)
                            if math.isfinite(self.current_yaw_rate_radps)
                            else None
                        )
                    ),
                    "telemetry_fresh": self.is_fresh(
                        self.last_odom_time,
                        self.radial_stop_telemetry_timeout_sec,
                    ),
                    "settle_sec": (
                        precision_certificate.stationary_window_sec
                        if precision_certificate is not None
                        else 0.0
                    ),
                    "max_radial_during_settle_mm": (
                        precision_certificate.radial_error_m * 1000.0
                        if precision_certificate is not None
                        else None
                    ),
                    "first_capture_pose": None,
                    "final_settled_pose": (
                        {
                            "x_m": precision_certificate.final_position_x_m,
                            "y_m": precision_certificate.final_position_y_m,
                        }
                        if precision_certificate is not None
                        else None
                    ),
                    "truth_frame": "controller_estimator_frame_only",
                    "localization_accuracy_certified": False,
                    "physical_accuracy_certified": False,
                    "precision_certificate": certificate_payload,
                }
            )
        elif self.precision_terminal_enabled:
            components = self.precision_terminal_identity_components or {}
            certificate_payload = self.ground_terminal_certificate_payload(
                precision_certificate
            )
            payload.update(
                {
                    "controller_outcome": outcome,
                    "precision_certificate_version": (
                        precision_certificate.version
                        if precision_certificate is not None
                        else 2
                    ),
                    "terminal_identity": self.precision_terminal_identity,
                    "terminal_identity_components": components,
                    "mission_run_id": components.get("mission_run_id"),
                    "goal_instance_id": components.get("goal_instance_id"),
                    "path_signature": components.get("path_signature"),
                    "raw_path_index": components.get("raw_path_index"),
                    "active_goal_identity": components.get("active_goal_identity"),
                    "precision_pass": bool(
                        precision_certificate is not None
                        and precision_certificate.precision_pass
                    ),
                    "cross_error_mm": cross_track_mm,
                    "along_error_mm": along_track_mm,
                    "stop_spec_mm": (
                        self.precision_terminal_config.terminal_radial_tolerance_m
                        * 1000.0
                    ),
                    "precision_stop_spec_mm": (
                        self.precision_terminal_config.terminal_radial_tolerance_m
                        * 1000.0
                    ),
                    "measured_yaw_rate_radps": (
                        float(self.current_yaw_rate_radps)
                        if math.isfinite(self.current_yaw_rate_radps)
                        else None
                    ),
                    "speed_at_release_mps": (
                        precision_certificate.measured_speed_mps
                        if precision_certificate is not None
                        else (
                            float(self.current_speed_mps)
                            if math.isfinite(self.current_speed_mps)
                            else None
                        )
                    ),
                    "yaw_rate_at_release_radps": (
                        precision_certificate.measured_yaw_rate_radps
                        if precision_certificate is not None
                        else (
                            float(self.current_yaw_rate_radps)
                            if math.isfinite(self.current_yaw_rate_radps)
                            else None
                        )
                    ),
                    "telemetry_fresh": self.is_fresh(
                        self.last_odom_time,
                        self.precision_terminal_telemetry_timeout_sec,
                    ),
                    "settle_sec": (
                        precision_certificate.settle_sec
                        if precision_certificate is not None
                        else 0.0
                    ),
                    "max_radial_during_settle_mm": (
                        precision_certificate.max_radial_during_settle_mm
                        if precision_certificate is not None
                        else None
                    ),
                    "first_capture_pose": (
                        precision_certificate.first_capture_pose.to_dict()
                        if precision_certificate is not None
                        and precision_certificate.first_capture_pose is not None
                        else None
                    ),
                    "final_settled_pose": (
                        precision_certificate.final_settled_pose.to_dict()
                        if precision_certificate is not None
                        and precision_certificate.final_settled_pose is not None
                        else None
                    ),
                    "truth_frame": "controller_estimator_frame_only",
                    "localization_accuracy_certified": False,
                    "physical_accuracy_certified": False,
                    "precision_certificate": certificate_payload,
                }
            )

        msg = String()
        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
            sort_keys=True,
        )

        self.terminal_result_pub.publish(msg)
        self._terminal_result_sent = outcome

        self.get_logger().warn(
            "RPP TERMINAL RESULT -> MISSION MANAGER | "
            f"outcome={outcome} | reason={reason} | "
            f"overall={radial_mm:.1f}mm | "
            f"cross={cross_track_mm:+.1f}mm | "
            f"along={along_track_mm:+.1f}mm"
        )

    def latch_exact_marking_stop(
        self,
        target_distance,
        signed_cross_track,
        along_remaining,
    ):
        """Latch zero after entering the exact 30 mm semantic-goal circle."""
        if target_distance <= self.waypoint_tolerance:
            if not self.marking_stop_latched:
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
                    f"xtrack={self.ground_xtrack(signed_cross_track) * 1000.0:+.1f}mm | "
                    f"along={along_remaining * 1000.0:+.1f}mm"
                )
            speed = (
                float(self.current_speed_mps)
                if math.isfinite(self.current_speed_mps)
                else math.inf
            )
            if speed <= self.stationary_speed_tolerance:
                self.publish_terminal_result(
                    "CAPTURED",
                    reason="RADIUS_30MM_STATIONARY",
                    target_distance=target_distance,
                    signed_cross_track=signed_cross_track,
                    along_remaining=along_remaining,
                )
            return True

        return self.marking_stop_latched

    def evaluate_latched_marking_stop(
        self,
        target_distance,
        signed_cross_track,
        along_remaining,
    ):
        """Keep zero latched and emit one outcome after the rover settles."""
        self.publish_stop()

        if self._terminal_result_sent is not None:
            return

        outcome = latched_stop_terminal_outcome(
            target_distance=target_distance,
            current_speed=self.current_speed_mps,
            waypoint_tolerance=self.waypoint_tolerance,
            stationary_speed_tolerance=self.stationary_speed_tolerance,
        )
        if outcome == "CAPTURED":
            self.publish_terminal_result(
                "CAPTURED",
                reason="RADIUS_30MM_STATIONARY",
                target_distance=target_distance,
                signed_cross_track=signed_cross_track,
                along_remaining=along_remaining,
            )
        elif outcome == "MISSED":
            self.publish_terminal_result(
                "MISSED",
                reason="SETTLED_OUTSIDE_30MM_AFTER_ENTRY",
                target_distance=target_distance,
                signed_cross_track=signed_cross_track,
                along_remaining=along_remaining,
            )

    def _control_timer_callback(self):
        """Run one motion cycle and always commit one coherent debug sample."""
        self._begin_rpp_debug_cycle()
        try:
            self.control_loop()
        finally:
            try:
                self._finish_rpp_debug_cycle()
            except Exception as exc:
                self.get_logger().error(
                    f"RPP DEBUG FINALIZE FAILED: {type(exc).__name__}: {exc}"
                )

    def control_loop(self):
        self._begin_precision_cycle()
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
            if self.precision_terminal_enabled:
                self._step_precision_terminal_hold_cycle()
            self.log_waiting("continuous marking hold")
            return
        if not self.marking_metadata_received:
            self.publish_stop()
            self.log_waiting("waiting for marking metadata")
            return
        if not self.nav_path_received or not self.nav_path_points:
            self.publish_stop()
            self.log_waiting("waiting for trajectory /nav_path")
            return
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            self.publish_stop()
            self.log_waiting("waiting for odometry")
            return
        if not self.is_fresh(
            self.last_odom_time,
            self.odom_timeout_sec,
        ):
            self.publish_stop()
            if self.segment_alignment_active and getattr(
                self, "legacy_alignment", None
            ) is not None:
                self.legacy_alignment.reset_dwell_timers()
            if self.precision_terminal_enabled:
                stale_result = self._step_precision_terminal_stale_cycle()
                self._publish_precision_terminal_result_if_ready(stale_result)
            self.log_waiting("odometry timeout")
            return
        if self.marking_missed:
            self.publish_stop()
            self.log_waiting("marking capture safe hold active")
            return

        if self.post_extension_stationary_hold:
            self.publish_stop()
            if self.current_speed_mps <= self.stationary_speed_tolerance:
                self.post_extension_stationary_hold = False
                self.reset_speed_profiles()
                self._reset_precision_regulator("EXTENSION_STATIONARY_RECAPTURE")
                self.get_logger().warn(
                    "EXTENSION STATIONARY CONFIRMED / NEXT SEGMENT RELEASED | "
                    f"speed={self.current_speed_mps:.3f}m/s"
                )
            else:
                self.log_waiting("extension captured; waiting for stationary chassis")
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

            # Exact P1 is the first semantic precision-stop goal.
            # Mission Manager publishes the same coordinate on both
            # /active_waypoint and /segment_goal.
            goal_x = p1_x
            goal_y = p1_y
            goal_is_marking = True
            goal_is_extension = False
            goal_requires_precision_stop = True
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
                target_label = "ACTIVE P1 SEMANTIC TARGET"
            else:
                target_x = goal_x
                target_y = goal_y
                target_is_marking = True
                target_label = "P1 FALLBACK TARGET"

            mode_prefix = "C->P1 LOCAL ODOM 50MM / " + target_label + " / "
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

            if self.segment_goal_x is not None and self.segment_goal_y is not None:
                goal_x = self.segment_goal_x
                goal_y = self.segment_goal_y
                # find_marking_number() returns 1..N for an original marking
                # point and 0 for an extension/dummy semantic goal. BOTH are
                # precision-stop goals. Only the marking class is sprayable.
                goal_is_marking = (
                    self.segment_goal_number is not None
                    and self.segment_goal_number > 0
                )
                goal_is_extension = not goal_is_marking
                goal_requires_precision_stop = True
            else:
                goal_x = target_x
                goal_y = target_y
                goal_is_marking = target_is_marking
                goal_is_extension = not target_is_marking
                goal_requires_precision_stop = True

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

        # Same map frame, three path lifetimes:
        #   START->P1 : current local-odom C -> P1 runtime path.
        #   Pn->Pn+1  : post-pivot reanchored runtime line, when one was
        #               installed for this goal (post_pivot_reanchor_all_legs).
        #   otherwise : fixed surveyed /nav_path prepared during LOAD.
        # Only the line taken to the goal differs; the goal is never moved,
        # and nothing is painted between marking points.
        if first_approach:
            nav_solution = self.runtime_entry_tracking_solution(goal_x, goal_y)
            if nav_solution is None:
                self.publish_stop()
                self.log_waiting("runtime local-odom C->P1 trajectory unavailable")
                return
            path_label = "ENTRY_PATH"
        else:
            # A later leg follows its post-pivot reanchored line when one was
            # installed for THIS goal, otherwise the surveyed /nav_path exactly
            # as before. runtime_entry_tracking_solution() returns None unless
            # the installed path's endpoint matches goal_x/goal_y within
            # waypoint_match_tolerance, so a path left over from a previous leg
            # can never be followed -- the fallback below is what runs then.
            nav_solution = None
            if self.segment_runtime_reanchored:
                nav_solution = self.runtime_entry_tracking_solution(goal_x, goal_y)
            path_label = "LEG_PATH"
            if nav_solution is None:
                path_label = "NAV_PATH"
                nav_solution = self.nav_path_tracking_solution(goal_x, goal_y)
            if nav_solution is None:
                self.publish_stop()
                self.log_waiting("semantic goal not bound to /nav_path")
                return

        # Bearing authority follows whichever path is actually being tracked.
        self.following_runtime_line = path_label in ("ENTRY_PATH", "LEG_PATH")

        (
            target_x,
            target_y,
            path_bearing,
            nav_cursor_index,
            nav_lookahead_index,
            nav_goal_index,
        ) = nav_solution
        mode_prefix += (
            f"{path_label} {nav_cursor_index}->{nav_lookahead_index}/"
            f"{nav_goal_index} / "
        )

        if path_bearing is None:
            self.publish_stop()
            self.log_waiting("waiting for /nav_path tangent")
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
        path_heading_error = self.normalize_angle(path_bearing - self.current_yaw)

        precision_guidance = None
        if self.precision_guidance_enabled or self.precision_speed_control_enabled:
            precision_guidance = self._compute_precision_guidance_for_cycle()
            if precision_guidance is None and not self.following_runtime_line:
                self.publish_stop()
                self.log_waiting("precision guidance lacks current-cycle projection")
                return
        precision_tracking_authority = (
            self.precision_tracking_control_enabled
            and not self.following_runtime_line
        )
        if precision_tracking_authority:
            tracking_output = self._compute_precision_tracking_for_cycle()
            if tracking_output is None or not tracking_output.valid:
                self.publish_stop()
                self.log_waiting("precision tracking input invalid or stale")
                return

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
        self._record_rpp_debug_geometry(
            path_bearing_rad=path_bearing,
            heading_error_rad=path_heading_error,
            distance_to_goal_m=goal_distance,
            signed_cross_track_m=goal_signed_cross_track,
            along_remaining_m=goal_along_remaining,
        )

        # Phase-5 terminal authority is evaluated after current projection,
        # guidance and exact semantic-goal errors, but before the legacy 30 mm
        # latch.  Once armed it remains active for this goal even if estimator
        # noise briefly moves the radial distance outside the approach region.
        if (
            self.precision_terminal_enabled
            and goal_requires_precision_stop
            and (
                self.precision_terminal_request_armed
                or goal_distance <= self.precision_terminal_config.approach_distance_m
            )
        ):
            terminal_result = self._step_precision_terminal_for_cycle(
                goal_distance=goal_distance,
                path_heading_error=path_heading_error,
                first_approach=first_approach,
                path_bearing=path_bearing,
                goal_x=goal_x,
                goal_y=goal_y,
            )
            if terminal_result is None:
                self.publish_stop()
                self.log_waiting(
                    "precision terminal lacks synchronized current goal binding"
                )
                return
            if terminal_result.directive in {
                TerminalDirective.HOLD_ZERO,
                TerminalDirective.HOLD_FAIL,
            }:
                self.publish_stop()
                self._publish_precision_terminal_result_if_ready(terminal_result)
                self.log_control(
                    mode_prefix
                    + "PRECISION TERMINAL "
                    + terminal_result.state.value.upper()
                    + " / ZERO OWNED",
                    goal_distance,
                    goal_distance,
                    path_heading_error,
                    0.0,
                    0.0,
                    0.0,
                )
                return

        # radial20 terminal authority: mutually exclusive with both the
        # Phase-5 branch above and the legacy 30 mm latch below, via
        # self.terminal_stop_mode. Bearing stays owned by the guidance
        # pipeline already resolved into path_bearing above (following_runtime_line
        # authority is already correctly gated by the time path_bearing
        # reaches this point) -- this branch only overrides speed magnitude.
        if (
            self.radial20_active
            and goal_requires_precision_stop
            and (
                self.radial_stop_request_armed
                or goal_distance
                <= self.radial_stop_config.terminal_guidance_distance_m
            )
        ):
            radial_result = self._step_radial20_terminal_for_cycle(
                along_remaining=goal_along_remaining,
                cross_error=goal_signed_cross_track,
            )
            if radial_result is None:
                self.publish_stop()
                self.log_waiting(
                    "radial20 terminal lacks synchronized current goal binding"
                )
                return
            self._publish_radial20_result_if_ready(radial_result)
            if radial_result.motion_direction is RadialStopMotionDirection.ZERO:
                self.publish_stop()
                self.log_control(
                    mode_prefix
                    + "RADIAL20 TERMINAL "
                    + radial_result.state.value.upper()
                    + " / ZERO OWNED",
                    goal_distance,
                    goal_distance,
                    path_heading_error,
                    0.0,
                    0.0,
                    0.0,
                )
                return
            desired_goal_bearing = math.atan2(
                goal_y - self.current_y,
                goal_x - self.current_x,
            )
            guidance_bearing = self.terminal_bounded_guidance(
                path_bearing,
                desired_goal_bearing,
                goal_along_remaining,
            )
            speed = radial_result.forward_speed_command_mps
            north = speed * math.sin(guidance_bearing)
            east = speed * math.cos(guidance_bearing)
            north, east, published_speed = self.publish_velocity_ned(
                north,
                east,
                apply_acceleration=True,
                apply_deceleration=False,
                hard_speed_cap_mps=speed,
            )
            self._record_published_translational_speed(published_speed)
            self.log_control(
                mode_prefix
                + "RADIAL20 TERMINAL "
                + radial_result.state.value.upper(),
                goal_distance,
                goal_distance,
                path_heading_error,
                published_speed,
                0.0,
                0.0,
            )
            return

        if (
            self.legacy_terminal_stop_active
            and goal_requires_precision_stop
            and self.marking_stop_latched
        ):
            self.evaluate_latched_marking_stop(
                goal_distance,
                goal_signed_cross_track,
                goal_along_remaining,
            )
            self.log_control(
                mode_prefix
                + "EXACT WP_RADIUS 30MM / ZERO LATCH / WAIT MISSION MANAGER",
                goal_distance,
                goal_distance,
                path_heading_error,
                0.0,
                0.0,
                0.0,
            )
            return

        # Exact radial distance is checked independently for every semantic
        # stop goal: marking point or extension/dummy.
        if (
            self.legacy_terminal_stop_active
            and goal_requires_precision_stop
            and self.latch_exact_marking_stop(
                goal_distance,
                goal_signed_cross_track,
                goal_along_remaining,
            )
        ):
            self.evaluate_latched_marking_stop(
                goal_distance,
                goal_signed_cross_track,
                goal_along_remaining,
            )
            self.log_control(
                mode_prefix
                + "EXACT WP_RADIUS 30MM / ZERO LATCH / WAIT MISSION MANAGER",
                goal_distance,
                goal_distance,
                path_heading_error,
                0.0,
                0.0,
                0.0,
            )
            return

        # Exact semantic-goal pass-plane safety remains active on every cycle.
        # It prevents indefinite travel after either a marking or extension is crossed.
        if goal_requires_precision_stop:
            if (
                not self.capture_monitor_armed
                and goal_distance <= self.marking_capture_arm_distance
            ):
                self.capture_monitor_armed = True
                self.closest_marking_distance = goal_distance
                self.get_logger().warn(
                    "EXACT PRECISION GOAL MONITOR ARMED | "
                    f"distance={goal_distance * 1000.0:.1f}mm | "
                    f"along_remaining="
                    f"{goal_along_remaining * 1000.0:+.1f}mm"
                )

            if self.capture_monitor_armed:
                self.closest_marking_distance = min(
                    self.closest_marking_distance,
                    goal_distance,
                )

            crossed_goal_plane = goal_along_remaining < -self.marking_along_track_abort
            moving_away_after_capture = (
                self.capture_monitor_armed
                and goal_distance > self.closest_marking_distance + self.miss_margin
            )

            if crossed_goal_plane and (
                moving_away_after_capture or not self.capture_monitor_armed
            ):
                if not math.isfinite(self.closest_marking_distance):
                    self.closest_marking_distance = goal_distance
                self.marking_missed = True
                self.reset_terminal_native_pivot()
                self._reset_legacy_alignment_lifecycle("TERMINAL_MISS")
                self.publish_stop()
                self.publish_terminal_result(
                    "MISSED",
                    reason="GOAL_PASSED_MOVING_AWAY",
                    target_distance=goal_distance,
                    signed_cross_track=goal_signed_cross_track,
                    along_remaining=goal_along_remaining,
                )
                self.get_logger().error(
                    "EXACT PRECISION GOAL CROSSED / SAFE HOLD | "
                    f"closest="
                    f"{self.closest_marking_distance * 1000.0:.1f}mm | "
                    f"current={goal_distance * 1000.0:.1f}mm | "
                    f"along_remaining="
                    f"{goal_along_remaining * 1000.0:+.1f}mm"
                )
                return

        # --------------------------------------------------------------
        # LEGACY NATIVE-PIVOT LIFECYCLE
        #
        # Native ±60deg carrier is unchanged.  A genuine latch no longer
        # falls through into 1.00 m/s capture at the 4deg heading gate.
        # After native release the chassis must prove a measured stop,
        # reanchor C->P1 to the actual post-pivot position once, hold
        # literal zero for legacy_pivot_post_settle_hold_sec, then release
        # straight into normal path tracking and the existing acceleration
        # ramp -- there is no moving recapture phase.  Aligned starts that
        # never latch a carrier keep the previous non-pivot capture path.
        # --------------------------------------------------------------
        if (
            not self.segment_alignment_active
            and goal_distance > self.terminal_goal_intercept_distance
            and abs(path_heading_error) >= self.pivot_enter_angle
        ):
            self.segment_alignment_active = True
            self._reset_legacy_alignment_lifecycle("MID_LEG_ALIGNMENT_REENTRY")
            if self.precision_pivot_enabled:
                self._reset_precision_pivot(
                    "MID_LEG_ALIGNMENT_REENTRY",
                    clear_anchor=True,
                )
                self._latch_precision_pivot_anchor(
                    self.current_x,
                    self.current_y,
                    "MID_LEG_REENTRY_CURRENT_POSE",
                    target_bearing=path_bearing,
                )
            self.get_logger().warn(
                "SEGMENT ALIGNMENT RE-ENTERED | "
                f"path_error={math.degrees(path_heading_error):+.1f}deg"
            )

        if self.segment_alignment_active:
            (
                alignment_guidance_bearing,
                alignment_cross_track,
            ) = self.line_guidance(
                path_bearing,
                target_x,
                target_y,
                self.segment_alignment_correction_limit,
            )
            if (
                self.precision_guidance_enabled
                and not self.following_runtime_line
            ):
                alignment_guidance_bearing = (
                    precision_guidance.limited_command_bearing_rad
                )
                alignment_cross_track = precision_guidance.signed_cross_track_m

            # Cross-track monitoring only.
            # Do NOT stop or latch the rover because of large xtrack.
            # Pivot/recovery logic below remains responsible for correcting it.
            if abs(alignment_cross_track) >= self.segment_alignment_max_cross_track:
                self.get_logger().warn(
                    "SEGMENT ALIGNMENT LARGE CROSS-TRACK / CONTINUING RECOVERY | "
                    f"xtrack={self.ground_xtrack(alignment_cross_track):+.3f}m | "
                    f"monitor_limit={self.segment_alignment_max_cross_track:.3f}m | "
                    f"path_error={math.degrees(path_heading_error):+.1f}deg"
                )

            if self.precision_pivot_enabled:
                self._run_precision_pivot_alignment(
                    path_bearing=path_bearing,
                    alignment_guidance_bearing=alignment_guidance_bearing,
                    alignment_cross_track=alignment_cross_track,
                    target_x=target_x,
                    target_y=target_y,
                    first_approach=first_approach,
                )
                return

            if self._run_legacy_segment_alignment(
                path_bearing=path_bearing,
                path_heading_error=path_heading_error,
                alignment_guidance_bearing=alignment_guidance_bearing,
                alignment_cross_track=alignment_cross_track,
                target_x=target_x,
                target_y=target_y,
                goal_x=goal_x,
                goal_y=goal_y,
                first_approach=first_approach,
                precision_guidance=precision_guidance,
                mode_prefix=mode_prefix,
                target_distance=target_distance,
                goal_distance=goal_distance,
            ):
                return

        gate2_active = (
            self.precision_guidance_enabled or self.precision_speed_control_enabled
        )
        # radial20 owns the entire terminal approach once
        # goal_requires_precision_stop is true (its own branch above always
        # returns before this point once armed/in range); this generic decel
        # zone would otherwise still fire in the gap between
        # terminal_goal_intercept_distance_m (0.90 m production) and
        # radial_stop_terminal_guidance_distance_m (0.75 m), running the
        # legacy speed/guidance profile for a sliver of approach distance in
        # a mode meant to have exactly one terminal authority.
        terminal_active = (
            not self.radial20_active
            and goal_requires_precision_stop
            and (
                goal_distance <= self.terminal_goal_intercept_distance
                or (gate2_active and self.terminal_precision_armed)
            )
        )

        # Preserve xtrack speed-cap state across the terminal boundary.
        # Terminal command is min(distance profile, xtrack cap).

        # --------------------------------------------------------------
        # GLOBAL MOVING CROSS-TRACK RECOVERY
        #
        # Recovery never creates an explicit zero-speed pivot. Desired
        # bearing stays within the configured moving-guidance limit and uses
        # the fixed 1.00 m/s mission speed outside terminal deceleration.
        # --------------------------------------------------------------
        if precision_tracking_authority:
            # Phase-4 authority is projection guidance plus its pure hysteresis
            # controller.  Do not call or mutate either legacy derivative/
            # filtered-guidance state or the legacy xtrack priority latch.
            xtrack_guidance_bearing = precision_guidance.limited_command_bearing_rad
            global_signed_cross_track = precision_guidance.signed_cross_track_m
            global_xtrack_rate = 0.0
            predicted_cross_track = global_signed_cross_track
            applied_xtrack_correction = self.normalize_angle(
                xtrack_guidance_bearing - path_bearing
            )
            terminal_moving_away = False
            terminal_crossing_imminent = False
            terminal_crossing_projection = None
            xtrack_profile_name = "PRECISION_TRACKING"
            active_xtrack_lookahead = precision_guidance.lookahead_distance_m
            active_xtrack_correction_limit = (
                self.precision_guidance_config.moving_bearing_cone_rad
            )
            active_xtrack_slew_rate = 0.0
        else:
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
                target_x,
                target_y,
                terminal_mode=terminal_active,
            )

        if not all(
            math.isfinite(value)
            for value in (
                global_signed_cross_track,
                predicted_cross_track,
                path_heading_error,
                xtrack_guidance_bearing,
            )
        ):
            self.publish_stop()
            self.get_logger().error("NON-FINITE XTRACK GUIDANCE / SAFE HOLD")
            return

        if precision_tracking_authority:
            xtrack_speed_cap_active = False
            xtrack_error_metric = abs(global_signed_cross_track)
            xtrack_release_elapsed = self.precision_tracking_output.stable_dwell_sec
        else:
            (
                xtrack_speed_cap_active,
                xtrack_error_metric,
                xtrack_release_elapsed,
            ) = self.update_xtrack_speed_cap_state(
                global_signed_cross_track,
                predicted_cross_track,
                path_heading_error,
            )

        if (
            not precision_tracking_authority
            and not terminal_active
            and xtrack_speed_cap_active
        ):
            if (
                self.precision_guidance_enabled
                and not self.following_runtime_line
            ):
                xtrack_guidance_bearing = precision_guidance.limited_command_bearing_rad
            (
                xtrack_guidance_bearing,
                command_heading_error,
            ) = self.limit_moving_guidance_bearing(xtrack_guidance_bearing)
            speed = self.xtrack_priority_speed
            if self.precision_speed_control_enabled:
                speed_result = self._resolve_precision_speed_for_cycle()
                if speed_result is None:
                    self.publish_stop()
                    self.get_logger().error(
                        "PRECISION SPEED REJECTED / RECOVERY SAFE HOLD"
                    )
                    return
                north, east, speed = self.publish_precision_velocity_ned(
                    xtrack_guidance_bearing,
                    speed_result,
                )
            else:
                north = speed * math.sin(xtrack_guidance_bearing)
                east = speed * math.cos(xtrack_guidance_bearing)
                north, east, speed = self.publish_velocity_ned(
                    north,
                    east,
                    apply_acceleration=True,
                    apply_deceleration=False,
                    hard_speed_cap_mps=self.xtrack_priority_speed,
                )
                self._record_published_translational_speed(speed)
            self.log_control(
                mode_prefix
                + "GLOBAL DAMPED XTRACK RECOVERY / "
                + f"HARD SPEED CAP {self.xtrack_priority_speed:.2f}MPS"
                + f" / correction_limit="
                + f"{math.degrees(self.xtrack_priority_correction_limit):.1f}deg"
                + f" | xtrack={self.ground_xtrack(global_signed_cross_track) * 1000.0:+.1f}mm"
                + f" | xtrack_rate={self.ground_xtrack(global_xtrack_rate) * 1000.0:+.1f}mm/s"
                + f" | predicted={self.ground_xtrack(predicted_cross_track) * 1000.0:+.1f}mm"
                + f" | metric={xtrack_error_metric * 1000.0:.1f}mm"
                + f" | release_hold={xtrack_release_elapsed:.2f}/"
                + f"{self.xtrack_priority_hold_sec:.2f}s"
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
        # CONTINUOUS TWO-METRE TERMINAL APPROACH
        #
        # There is no hard line-capture gate and no early HOLD. The same
        # filtered line-guidance demand continues through deceleration. Only
        # its correction authority and speed magnitude are reduced smoothly.
        # Exact radial <=30 mm remains the only normal zero-latch condition.
        # --------------------------------------------------------------
        if terminal_active:
            signed_cross_track = global_signed_cross_track
            along_remaining = self.along_track_remaining(
                path_bearing,
                goal_x,
                goal_y,
            )

            if not self.terminal_precision_armed:
                self.terminal_precision_armed = True
                current_correction = self.normalize_angle(
                    xtrack_guidance_bearing - path_bearing
                )
                current_limit = self.terminal_correction_limit_for_along(
                    along_remaining
                )
                self.terminal_limited_correction = max(
                    -current_limit,
                    min(current_limit, current_correction),
                )
                self.terminal_correction_last_update_time = None
                self.reset_deceleration_profile()
                self.reset_terminal_native_pivot()
                self.get_logger().warn(
                    "CONTINUOUS TERMINAL GUIDANCE ENTERED | "
                    f"xtrack={self.ground_xtrack(signed_cross_track) * 1000.0:+.1f}mm | "
                    f"heading={math.degrees(path_heading_error):+.1f}deg | "
                    f"along={along_remaining * 1000.0:+.1f}mm"
                )

            if (
                not self.capture_monitor_armed
                and goal_distance <= self.marking_capture_arm_distance
            ):
                self.capture_monitor_armed = True
                self.closest_marking_distance = goal_distance
                self.get_logger().warn(
                    "PRECISION-GOAL CAPTURE SAFETY MONITOR ARMED | "
                    f"distance={goal_distance:.3f}m | "
                    f"along={along_remaining:.3f}m"
                )

            if self.capture_monitor_armed:
                self.closest_marking_distance = min(
                    self.closest_marking_distance,
                    goal_distance,
                )
                radial_increased = (
                    goal_distance > self.closest_marking_distance + self.miss_margin
                )
                confirmed_overshoot = along_remaining < -self.marking_along_track_abort
                if radial_increased and confirmed_overshoot:
                    self.marking_missed = True
                    self.reset_terminal_native_pivot()
                    self._reset_legacy_alignment_lifecycle("TERMINAL_MISS")
                    self.publish_stop()
                    self.publish_terminal_result(
                        "MISSED",
                        reason="GOAL_PASSED_MOVING_AWAY",
                        target_distance=goal_distance,
                        signed_cross_track=signed_cross_track,
                        along_remaining=along_remaining,
                    )
                    self.get_logger().error(
                        "PRECISION GOAL PASSED WITHOUT 30MM ENTRY / SAFE HOLD | "
                        f"closest={self.closest_marking_distance:.3f}m | "
                        f"current={goal_distance:.3f}m | "
                        f"along={along_remaining:.3f}m"
                    )
                    self.publish_terminal_state()
                    return

            guidance_bearing = self.terminal_bounded_guidance(
                path_bearing,
                xtrack_guidance_bearing,
                along_remaining,
            )
            heading_error = self.normalize_angle(guidance_bearing - self.current_yaw)

            profile_target = self.terminal_speed_for_along_remaining(along_remaining)
            terminal_speed_cap = (
                self.xtrack_priority_speed if xtrack_speed_cap_active else None
            )

            # The established terminal profile owns speed for Gate-2 as well
            # as legacy operation.  Generic precision speed must not regrow
            # the command after terminal entry or compete with its zero latch.
            speed = self.cruise_speed
            north = speed * math.sin(guidance_bearing)
            east = speed * math.cos(guidance_bearing)
            north, east, speed = self.publish_velocity_ned(
                north,
                east,
                apply_acceleration=True,
                apply_deceleration=True,
                goal_distance=along_remaining,
                hard_speed_cap_mps=terminal_speed_cap,
            )
            self._record_published_translational_speed(speed)

            self.publish_rpp_debug(
                control_mode="TERMINAL",
                command_speed_mps=speed,
                path_bearing_rad=path_bearing,
                guidance_bearing_rad=guidance_bearing,
                heading_error_rad=heading_error,
                distance_to_goal_m=goal_distance,
                signed_cross_track_m=global_signed_cross_track,
                along_remaining_m=along_remaining,
            )

            if xtrack_speed_cap_active:
                speed_owner = (
                    "MIN(DISTANCE_PROFILE,"
                    f"XTRACK_CAP={self.xtrack_priority_speed:.3f})"
                )
            else:
                speed_owner = "DISTANCE_PROFILE"

            status = (
                "CONTINUOUS LINE + HARDENED SPEED ARBITRATION / "
                f"profile_target={profile_target:.3f}mps / "
                f"speed_owner={speed_owner} / "
                f"xtrack={self.ground_xtrack(global_signed_cross_track) * 1000.0:+.1f}mm / "
                f"predicted={self.ground_xtrack(predicted_cross_track) * 1000.0:+.1f}mm / "
                f"correction={math.degrees(self.terminal_limited_correction):+.2f}deg"
            )
            self.publish_terminal_state()
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
        if precision_tracking_authority:
            guidance_bearing = precision_guidance.limited_command_bearing_rad
            signed_cross_track = precision_guidance.signed_cross_track_m
        else:
            (
                guidance_bearing,
                signed_cross_track,
            ) = self.line_guidance(
                path_bearing,
                target_x,
                target_y,
                self.path_correction_limit,
            )
            if (
                self.precision_guidance_enabled
                and not self.following_runtime_line
            ):
                guidance_bearing = precision_guidance.limited_command_bearing_rad
                signed_cross_track = precision_guidance.signed_cross_track_m
        heading_error = self.normalize_angle(guidance_bearing - self.current_yaw)

        speed = self.cruise_speed

        if first_approach:
            status = (
                "C->P1 FIXED C-LINE / ACTIVE 50MM PATH / "
                "200MM ACCEL / SEMANTIC-GOAL 500MM DECEL | "
                f"xtrack={self.ground_xtrack(signed_cross_track):+.3f}m"
            )
        else:
            status = (
                "INTERPOLATED 50MM STRAIGHT PATH / "
                "200MM ACCEL / SEMANTIC-GOAL 500MM DECEL | "
                f"xtrack={self.ground_xtrack(signed_cross_track):+.3f}m"
            )

        if self.precision_guidance_enabled or self.precision_speed_control_enabled:
            status += (
                " | PHASE2 "
                f"guidance={'ON' if self.precision_guidance_enabled else 'OFF'}"
                f" speed={'ON' if self.precision_speed_control_enabled else 'OFF'}"
            )

        # Normal/pass-through movement uses only the gentle start ramp.
        # Terminal deceleration is activated exclusively after the final-line
        # precision gate is latched inside the terminal corridor.
        if self.precision_speed_control_enabled:
            speed_result = self._resolve_precision_speed_for_cycle()
            if speed_result is None:
                self.publish_stop()
                self.get_logger().error("PRECISION SPEED REJECTED / SAFE HOLD")
                return
            north, east, speed = self.publish_precision_velocity_ned(
                guidance_bearing,
                speed_result,
            )
        else:
            north = speed * math.sin(guidance_bearing)
            east = speed * math.cos(guidance_bearing)
            north, east, speed = self.publish_velocity_ned(
                north,
                east,
            )
            self._record_published_translational_speed(speed)

        self.publish_rpp_debug(
            control_mode="FIRST_APPROACH" if first_approach else "TRACKING",
            command_speed_mps=speed,
            path_bearing_rad=path_bearing,
            guidance_bearing_rad=guidance_bearing,
            heading_error_rad=heading_error,
            distance_to_goal_m=goal_distance,
            signed_cross_track_m=signed_cross_track,
            along_remaining_m=goal_along_remaining,
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
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node._reset_legacy_alignment_lifecycle("SHUTDOWN")
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
