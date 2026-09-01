#!/usr/bin/env python3

"""Production launch for the complete DYX 4WD rover stack.

Startup remains safe:

- backend publishes /emergency_stop=true;
- backend publishes /mission_enable=false;
- no mission is prepared or started automatically;
- cmd_vel_bridge requires a fresh backend heartbeat.

RTK correction ownership is backend-managed. rover_backend restores the
persisted RTK desired state and supervises the only production correction
worker. Operators must use the authenticated /api/rtk control surface rather
than launching an NTRIP/RTCM ROS node directly.
"""

from launch import LaunchDescription

from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import TimerAction

from launch_ros.actions import Node

MISSION_FILE = "/home/flash/rover_ws/" "missions/mission.csv"

MISSION_METADATA_FILE = (
    "/home/flash/.local/share/" "dyx_rover/runtime/mission_metadata.json"
)

# Fixed production field-test cruise speed. Forward line capture and normal
# xtrack recovery target 1.00 m/s. The only planned reductions are the normal
# start ramp and the final 500 mm semantic-goal deceleration needed to stop.
CRUISE_SPEED_MPS = 1.00
TERMINAL_FLOOR_SPEED_MPS = 0.15


def generate_launch_description() -> LaunchDescription:
    mavros = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "source "
            "/opt/ros/humble/setup.bash && "
            "ros2 launch mavros node.launch "
            "fcu_url:="
            "/dev/ttyACM0:921600 "
            "gcs_url:="
            "udp://:14550@192.168.3.105:14550 "
            "pluginlists_yaml:="
            "/opt/ros/humble/share/mavros/"
            "launch/px4_pluginlists.yaml "
            "config_yaml:="
            "/opt/ros/humble/share/mavros/"
            "launch/px4_config.yaml "
            "tgt_system:=1 "
            "tgt_component:=1 "
            "fcu_protocol:=v2.0",
        ],
        output="screen",
    )

    rover_backend_node = Node(
        package="rover_backend",
        executable="rover_backend",
        name="rover_backend",
        output="screen",
        emulate_tty=True,
        # Keep one visible backend process during manual testing. If it exits,
        # the operator can inspect the error without a bind-failure respawn loop.
        respawn=False,
    )

    rover_backend = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg=("Starting production " "rover_backend")),
            rover_backend_node,
        ],
    )

    cmd_vel_bridge = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg=("Starting heartbeat-gated " "cmd_vel_bridge")),
            Node(
                package="jetson_4wd_control",
                executable="cmd_vel_bridge",
                name="cmd_vel_bridge",
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "command_timeout_sec": (0.25),
                        ("backend_heartbeat_" "timeout_sec"): 1.5,
                        # Bridge preserves the RPP ramp and only clamps
                        # commands above this production safety maximum.
                        "maximum_speed_mps": (1.00),
                        ("maximum_yaw_rate_" "radps"): 0.20,
                    }
                ],
            ),
        ],
    )

    trajectory_generator = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg=("Starting dynamic " "trajectory_generator")),
            Node(
                package=("trajectory_generator"),
                executable=("trajectory_generator_node"),
                name=("trajectory_generator"),
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "mission_file": (MISSION_FILE),
                        ("mission_metadata_file"): (MISSION_METADATA_FILE),
                        "frame_id": "map",
                        ("global_position_topic"): (
                            "/mavros/" "global_position/" "raw/fix"
                        ),
                        "gp_origin_topic": (
                            "/mavros/" "global_position/" "gp_origin"
                        ),
                        ("fused_global_position_" "topic"): (
                            "/mavros/" "global_position/" "global"
                        ),
                        # Survey GPS -> gp_origin -> NED -> MAVROS ENU.
                        "localization_mode": "px4_origin",
                        "local_odom_topic": ("/mavros/" "local_position/odom"),
                        "gps_status_topic": ("/mavros/" "gpsstatus/gps1/raw"),
                        "rtk_health_topic": ("/rtk_correction_bridge/" "healthy"),
                        ("rtk_correction_age_" "topic"): (
                            "/rtk_correction_bridge/" "correction_age_sec"
                        ),
                        ("required_gps_fix_type"): 6,
                        "rtk_stable_sec": 3.0,
                        ("max_correction_age_sec"): 2.0,
                        ("reference_timeout_sec"): 1.0,
                        ("max_reference_skew_sec"): 0.25,
                        # Origin health only; not marking tolerance.
                        "origin_consistency_max_m": 0.30,
                        ("max_target_distance_m"): 1000.0,
                        ("max_abs_coordinate_m"): 10000.0,
                        ("maximum_marking_points"): 10000,
                        ("maximum_navigation_" "points"): 200000,
                        ("interpolation_spacing_m"): 0.05,
                        ("minimum_segment_length_m"): 0.001,
                        ("minimum_dummy_clearance_m"): 0.05,
                    }
                ],
            ),
        ],
    )

    spray_controller = TimerAction(
        period=7.5,
        actions=[
            LogInfo(msg=("Starting production " "AUX5 spray controller")),
            Node(
                package="spray_controller",
                executable=("spray_controller_node"),
                name="spray_controller",
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        # Holybro Pixhawk 6X AUX5:
                        # PWM_AUX_FUNC5 = 301
                        # Peripheral via Actuator Set 1.
                        "enabled": True,
                        # Confirmed Jetson/PX4 command path:
                        # MAV_CMD_DO_SET_ACTUATOR (187)
                        # param1 -> Actuator Set 1 -> AUX5.
                        "press_value": 1.0,
                        # Must be the physically verified
                        # fully-released servo position.
                        "release_value": 0.0,
                        "spray_duration_sec": 0.50,
                        # mission_manager already requires <=30 mm radial error,
                        # <=0.01 m/s rover speed, then holds the marking point
                        # for 3.0 s total. Spray actuator ON time is only 0.5 s.
                        "pre_spray_stable_sec": 0.25,
                        "command_timeout_sec": 1.0,
                        ("release_retry_interval_sec"): 0.25,
                        # Independent hard watchdog.
                        "hard_press_timeout_sec": 5.0,
                        "mavros_state_timeout_sec": 2.5,
                        ("mission_status_timeout_sec"): 1.0,
                        ("marking_active_timeout_sec"): 0.50,
                        "require_px4_armed": True,
                        "require_px4_offboard": True,
                        "journal_path": (
                            "/home/flash/.ros/" "dyx_spray_controller_journal.json"
                        ),
                    }
                ],
            ),
        ],
    )

    mission_manager = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg=("Starting production " "mission_manager")),
            Node(
                package="mission_manager",
                executable=("mission_manager_node"),
                name="mission_manager",
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "local_frame": "map",
                        # Exact marking acceptance for P1/P2/P3/...
                        "marking_tolerance_m": 0.03,
                        # Marking must remain inside 30 mm AND stationary
                        # continuously before spray is requested.
                        "arrival_settle_sec": 0.30,
                        # After arrival validation, keep the rover at ZERO for
                        # 3.0 s total at every real marking point P1/P2/P3/...
                        "marking_hold_sec": 3.00,
                        "stationary_speed_tolerance_mps": 0.01,
                        # Extension/dummy semantic goal uses the SAME exact
                        # 30 mm waypoint radius as marking points, but is never
                        # sprayed or counted as a marking point.
                        "dummy_arrival_tolerance_m": 0.03,
                        # Spray is requested only after positional verification.
                        # Its SUCCESS/FAILED/TIMEOUT outcome is reported for
                        # monitoring and does not gate mission progression.
                        "spray_required": True,
                        "spray_confirmation_timeout_sec": 7.0,
                        "spray_status_timeout_sec": 2.0,
                        "waypoint_match_tolerance_m": 0.002,
                        "odom_timeout_sec": 0.50,
                        "maximum_navigation_points": 200000,
                        # Gate-1 accepted in four forward/reverse field runs.
                        # Signed semantic path identity is now the default.
                        "precision_path_contract_enabled": True,
                        # Phase-6 verifier remains dormant until Phase-5 RPP
                        # certificate authority is deliberately enabled.
                        "precision_terminal_enabled": False,
                        "precision_terminal_heartbeat_timeout_sec": 0.50,
                        "maximum_marking_points": 10000,
                    }
                ],
            ),
        ],
    )

    rpp_controller = TimerAction(
        period=9.0,
        actions=[
            LogInfo(msg=("Starting precision " "RPP controller")),
            Node(
                package="rpp_controller",
                executable=("rpp_controller_node"),
                name="rpp_controller",
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "local_frame": "map",
                        # RPP motion profile + trajectory following.
                        # Extension/dummy generation remains owned by the
                        # trajectory generator; RPP only follows /nav_path.
                        #
                        # START / RESTART:
                        #   0 -> fixed CRUISE_SPEED_MPS over 0.20 m from the
                        #   current translational start position.
                        #
                        # SEMANTIC STOP GOALS:
                        #   P1/P2/P3/... AND extension/dummy goals both use
                        #   the final 0.50 m deceleration profile.
                        #   Fixed 1.00 m/s mission: 1.00 -> 0.15 m/s.
                        #   exact zero remains radial <=30 mm for BOTH classes.
                        #   Only P1/P2/P3/... are spray/marking points.
                        # Pass-through/interpolation points do not stop/decelerate.
                        "cruise_speed_mps": CRUISE_SPEED_MPS,
                        "acceleration_enabled": True,
                        # Lengthened from 0.20m -- P4 bag forensic analysis
                        # showed the two catastrophic later-leg swings
                        # (163805 t=127s, 164150 t=68s) peaked at 92-268mm
                        # of travel, i.e. inside the launch ramp's own
                        # distance window. Matches deceleration_distance_m
                        # for a symmetric envelope. Derived accel rate drops
                        # from 2.5 m/s^2 to 1.0 m/s^2. Does not fix leg1
                        # cruise-phase swings (150503/151020 pattern), which
                        # peak past 1m, well outside this window, and does
                        # not fix the later-leg missing-reanchor root cause
                        # -- it only reduces speed while that pre-existing
                        # offset is still being corrected. See P4 bag
                        # forensic analysis (2026-08-27).
                        "acceleration_distance_m": 0.50,
                        # Small bootstrap ceiling only prevents drivetrain deadlock;
                        # the profile itself starts from literal zero. Lowered
                        # from 0.15 -- field bags show this jump (0->0.15 m/s
                        # within ~0.1s) combined with residual heading/
                        # lookahead correction produces a visible left/right
                        # swing at launch. See P3 bag forensic analysis
                        # (2026-08-27).
                        "acceleration_startup_ceiling_mps": 0.08,
                        "acceleration_max_progress_jump_m": 0.10,
                        "acceleration_max_dt_sec": 0.10,
                        # 1.00 m/s over 0.20 m requires 2.50 m/s^2, so this
                        # secondary slew guard must sit above the derived profile.
                        "command_speed_rise_limit_mps2": 3.00,
                        "command_speed_fall_limit_mps2": 2.00,
                        # Final 500 mm profile for marking + extension goals.
                        "deceleration_enabled": True,
                        "deceleration_distance_m": 0.50,
                        "deceleration_floor_speed_mps": TERMINAL_FLOOR_SPEED_MPS,
                        "deceleration_max_progress_jump_m": 0.10,
                        "deceleration_max_dt_sec": 0.10,
                        # Uneven-ground terminal steering separation.
                        "terminal_decel_correction_limit_deg": 12.0,
                        "terminal_near_correction_limit_deg": 3.0,
                        "terminal_near_correction_start_distance_m": 0.79,
                        "terminal_bearing_freeze_distance_m": 0.04,
                        "terminal_correction_slew_rate_degps": 15.0,
                        "terminal_frozen_xtrack_abort_m": 0.20,
                        "minimum_speed_mps": 0.04,
                        # Segment transition ownership:
                        # - RPP sends a dynamic +/-60deg carrier vector while
                        #   REAL segment heading error is >4deg;
                        # - this keeps PX4 in native differential pivot and
                        #   eliminates the moving 45->12deg transition;
                        # - after real heading <=4deg, forward line capture is capped
                        #   at 1.00 m/s until xtrack <=8mm for 0.20s;
                        # - if capture xtrack >50mm, fall back to the global
                        #   1.00 m/s xtrack recovery controller;
                        # - final 500 mm semantic-goal deceleration ends at 0.15 m/s
                        #   for the fixed 1.00 m/s cruise speed.
                        # Recovery contract for field precision:
                        #   pivot carrier              = CRUISE_SPEED_MPS (no translational accel)
                        #   genuine native-pivot release holds zero through
                        #   measured settle, reanchors C->P1 once, holds zero
                        #   for 1.00 s more, then releases straight into the
                        #   normal acceleration ramp (no moving recapture)
                        #   aligned-start (no carrier latch) still uses 1.00 m/s
                        #   global xtrack recovery      = 1.00 m/s
                        #   xtrack engage/release      = 15 mm / 8 mm
                        #   release heading            = <=4 deg stable for 0.30 s
                        "segment_alignment_speed_mps": CRUISE_SPEED_MPS,
                        "segment_alignment_recovery_speed_mps": CRUISE_SPEED_MPS,
                        "xtrack_priority_speed_mps": CRUISE_SPEED_MPS,
                        "decel_profile_speed_1_mps": CRUISE_SPEED_MPS,
                        "decel_profile_speed_2_mps": CRUISE_SPEED_MPS,
                        "decel_profile_speed_3_mps": CRUISE_SPEED_MPS,
                        "marking_terminal_max_speed_mps": min(
                            TERMINAL_FLOOR_SPEED_MPS, CRUISE_SPEED_MPS
                        ),
                        "marking_final_creep_speed_mps": min(
                            TERMINAL_FLOOR_SPEED_MPS, CRUISE_SPEED_MPS
                        ),
                        "terminal_close_recovery_speed_mps": min(
                            TERMINAL_FLOOR_SPEED_MPS, CRUISE_SPEED_MPS
                        ),
                        # PX4 native rover alignment:
                        # pivot begins at >=45deg,
                        # straight drive begins at <=12deg after 0.20s stable confirmation,
                        # no 6deg gate.
                        # Normal line correction remains +/-12deg.
                        # Post-pivot C->P1 line re-anchor removes the
                        # displacement created during the native pivot.
                        # Predictive xtrack recovery remains continuous
                        # through the final 1.00m. Exact P1 is stop/marking only.
                        "segment_alignment_deadband_enter_cross_track_m": 0.08,
                        "segment_alignment_deadband_exit_cross_track_m": 0.04,
                        "segment_alignment_min_effective_heading_error_deg": 10.0,
                        "segment_alignment_correction_limit_deg": 18.0,
                        "segment_alignment_cross_track_tolerance_m": 0.03,
                        "segment_alignment_reentry_cross_track_m": 0.08,
                        "segment_alignment_max_cross_track_m": 0.60,
                        "terminal_line_entry_cross_track_m": 0.03,
                        "pivot_enter_angle_deg": 45.0,
                        "pivot_exit_angle_deg": 12.0,
                        # Fast post-pivot capture releases only after
                        # terminal_native_pivot_release_error_deg=4.0 and
                        # xtrack_priority_exit_m=0.008 remain valid for this
                        # hold time.
                        "alignment_hold_sec": 0.20,
                        "maximum_yaw_rate_radps": 0.20,
                        "minimum_yaw_rate_radps": 0.06,
                        "pivot_yaw_kp": 1.00,
                        "alignment_reentry_goal_distance_m": 0.60,
                        # Hardened speed arbitration:
                        # - post-pivot line capture and global xtrack recovery are capped
                        #   at 1.00 m/s; normal cruise remains CRUISE_SPEED_MPS;
                        # - final distance profile is semantic-goal-only and uses the
                        #   configured cruise/floor pair;
                        # - terminal command is min(distance profile, xtrack cap);
                        # - measured radial <=30 mm still commands immediate zero.
                        # Trajectory-generator path ownership:
                        # - /nav_path is the full retained 50 mm interpolated path;
                        # - RPP advances through those points without stopping;
                        # - /segment_goal is only the semantic endpoint used for
                        #   500 mm final deceleration and the exact 30 mm stop.
                        # Straight-line and cross-track guidance uses the local
                        # /nav_path tangent and a path-distance lookahead.
                        "path_correction_limit_deg": 18.0,
                        "terminal_line_correction_limit_deg": 18.0,
                        "line_tracking_lookahead_m": 0.55,
                        # Speed-adaptive lookahead: reproduces the 0.55 m
                        # value above exactly at cruise_speed_mps (1.00),
                        # scales down toward the min during the accel/decel
                        # ramps, and widens toward the max on large
                        # cross-track deviations for a softer re-acquisition.
                        "line_tracking_lookahead_min_m": 0.35,
                        "line_tracking_lookahead_max_m": 0.80,
                        "line_tracking_lookahead_xtrack_gain": 1.0,
                        "nav_path_lookahead_m": 0.55,
                        # Gate-1 accepted in four forward/reverse field runs.
                        # Geometry is installed by default but cannot change
                        # motion until a downstream precision gate is enabled.
                        "geometry_tracking_enabled": True,
                        # Shadow projection/diagnostics without changing the
                        # production cursor solution. Also default-OFF so old
                        # bags do not require the additive path signature.
                        "geometry_diagnostics_enabled": False,
                        "geometry_corner_threshold_deg": 45.0,
                        "geometry_projection_back_window_segments": 2,
                        "geometry_projection_forward_window_segments": 4,
                        "geometry_projection_reacquire_distance_m": 0.30,
                        "geometry_localization_jump_reset_m": 0.50,
                        "geometry_max_backward_jump_m": 0.10,
                        "geometry_max_forward_jump_m": 1.00,
                        # Projection guidance is field-accepted and enabled
                        # with the tested 0.90 s horizon. Longitudinal speed
                        # regulation remains independently default-OFF.
                        "precision_guidance_enabled": True,
                        "precision_speed_control_enabled": False,
                        "precision_lookahead_min_m": 0.20,
                        "precision_lookahead_max_m": 1.00,
                        "precision_lookahead_time_s": 0.90,
                        "precision_xtrack_lookahead_gain": 0.0,
                        "precision_moving_bearing_cone_deg": 30.0,
                        "precision_hardware_speed_ceiling_mps": 1.00,
                        "precision_acceleration_mps2": 0.75,
                        "precision_deceleration_mps2": 0.75,
                        "precision_launch_speed_mps": 0.10,
                        "precision_control_dt_max_sec": 0.10,
                        "precision_heading_accel_full_error_deg": 2.0,
                        "precision_heading_recovery_start_deg": 4.0,
                        "precision_heading_recovery_full_deg": 15.0,
                        "precision_xtrack_accel_full_m": 0.010,
                        "precision_xtrack_recovery_start_m": 0.020,
                        "precision_xtrack_recovery_full_m": 0.100,
                        "precision_recovery_min_speed_mps": 0.15,
                        "precision_corner_angle_threshold_deg": 45.0,
                        "precision_corner_target_speed_mps": 0.12,
                        "precision_corner_accel_block_buffer_m": 0.10,
                        # Positive by design: Phase 2 never owns terminal zero.
                        # The existing 30 mm radial latch remains authoritative.
                        "precision_terminal_target_speed_mps": 0.15,
                        "precision_minimum_moving_speed_mps": 0.04,
                        "precision_braking_latency_sec": 0.10,
                        "precision_braking_margin_m": 0.05,
                        "precision_curvature_enabled": False,
                        "precision_lateral_acceleration_max_mps2": 0.30,
                        "precision_curvature_epsilon_inv_m": 1.0e-6,
                        # Phase-4 recovery hysteresis and controller-frame run
                        # metrics. Default-OFF; requires geometry + Phase-2
                        # guidance and speed authority when enabled.
                        "precision_tracking_control_enabled": False,
                        "precision_tracking_recovery_enter_xtrack_m": 0.050,
                        "precision_tracking_recovery_exit_xtrack_m": 0.020,
                        "precision_tracking_recovery_enter_heading_deg": 15.0,
                        "precision_tracking_recovery_exit_heading_deg": 5.0,
                        "precision_tracking_stable_dwell_sec": 0.30,
                        "precision_tracking_recovery_speed_scale": 0.35,
                        "precision_tracking_recapture_speed_scale": 0.50,
                        "precision_tracking_metrics_capacity": 2048,
                        "precision_tracking_histogram_bin_width_m": 0.001,
                        "precision_tracking_histogram_max_m": 1.0,
                        "precision_tracking_monotonic_tolerance_m": 0.001,
                        "precision_tracking_cruise_threshold_mps": 0.80,
                        # Phase-5 measured <=10 mm controller-frame stop.
                        # Default-OFF preserves the production 30 mm latch.
                        "precision_terminal_enabled": False,
                        "precision_terminal_radial_tolerance_m": 0.010,
                        "precision_terminal_capture_tolerance_m": 0.010,
                        "precision_terminal_settle_tolerance_m": 0.010,
                        "precision_terminal_stop_speed_tolerance_mps": 0.010,
                        "precision_terminal_stop_yaw_rate_tolerance_radps": 0.050,
                        "precision_terminal_settle_dwell_sec": 0.30,
                        "precision_terminal_telemetry_timeout_sec": 0.25,
                        "precision_terminal_approach_distance_m": 0.75,
                        "precision_terminal_brake_distance_m": 0.30,
                        "precision_terminal_timeout_sec": 15.0,
                        "precision_terminal_settle_timeout_sec": 5.0,
                        "precision_terminal_min_actuatable_speed_mps": 0.04,
                        # Phase-3 measured pivot/recenter FSM. Default-OFF
                        # preserves the production native keeper and adapter.
                        "precision_pivot_enabled": False,
                        "precision_pivot_anchor_tolerance_m": 0.030,
                        "precision_pivot_recenter_threshold_m": 0.030,
                        # Shared with the legacy pivot lifecycle's stationary
                        # certificate (LegacyAlignmentConfig.stop_speed_mps)
                        # even though precision_pivot_enabled is False -- this
                        # value gates the legacy settle/hold dwell too. Raised
                        # from 0.010 because measured/estimated speed while
                        # genuinely stationary (yaw_rate<0.01) sits at a
                        # median of 0.016-0.033 m/s in field bags, causing the
                        # dwell certificate to repeatedly reset and stall for
                        # 15-20+ seconds. See P3 bag forensic analysis
                        # (2026-08-27).
                        "precision_pivot_stop_speed_tolerance_mps": 0.030,
                        "precision_pivot_stop_yaw_rate_tolerance_radps": 0.050,
                        "precision_pivot_telemetry_timeout_sec": 0.25,
                        "precision_pivot_stop_settle_sec": 0.20,
                        "precision_pivot_heading_tolerance_deg": 2.0,
                        "precision_pivot_release_settle_sec": 0.20,
                        "precision_pivot_brake_timeout_sec": 8.0,
                        "precision_pivot_timeout_sec": 9.0,
                        "precision_pivot_recenter_speed_mps": 0.12,
                        "precision_pivot_recenter_timeout_sec": 5.0,
                        "precision_pivot_max_recenter_attempts": 2,
                        "precision_pivot_realign_timeout_sec": 9.0,
                        "precision_pivot_recapture_timeout_sec": 8.0,
                        "post_pivot_capture_speed_mps": 0.20,
                        "legacy_pivot_post_settle_hold_sec": 1.00,
                        "precision_pivot_recapture_xtrack_m": 0.020,
                        "precision_pivot_recapture_heading_deg": 2.0,
                        "precision_pivot_recapture_settle_sec": 0.20,
                        "precision_pivot_recenter_forward_cone_deg": 30.0,
                        # 75 mm lets a 20 Hz rover moving at 1 m/s advance
                        # reliably through 50 mm interpolation samples without
                        # treating them as arrival/stop tolerances.
                        "nav_path_point_reach_m": 0.075,
                        "xtrack_priority_enter_m": 0.015,
                        "xtrack_priority_exit_m": 0.008,
                        "xtrack_priority_hold_sec": 0.30,
                        "xtrack_priority_lookahead_m": 0.55,
                        "xtrack_priority_correction_limit_deg": 22.0,
                        "xtrack_prediction_time_sec": 0.25,
                        "xtrack_rate_filter_alpha": 0.20,
                        "xtrack_correction_slew_rate_degps": 30.0,
                        "xtrack_neutral_crossing_band_m": 0.015,
                        "xtrack_priority_release_heading_deg": 4.0,
                        # Adaptive final 1.50m xtrack profile with projected crossing brake.
                        "terminal_xtrack_lookahead_m": 0.50,
                        "terminal_xtrack_correction_limit_deg": 22.0,
                        "terminal_xtrack_prediction_time_sec": 0.25,
                        "terminal_xtrack_neutral_crossing_band_m": 0.004,
                        "terminal_xtrack_correction_slew_rate_degps": 35.0,
                        "terminal_xtrack_unwind_slew_rate_degps": 50.0,
                        "terminal_xtrack_away_lookahead_m": 0.30,
                        "terminal_xtrack_away_correction_limit_deg": 28.0,
                        "terminal_xtrack_away_rate_threshold_mps": 0.008,
                        "terminal_xtrack_crossing_prediction_time_sec": 0.55,
                        "terminal_xtrack_crossing_lookahead_m": 0.40,
                        "terminal_xtrack_crossing_correction_limit_deg": 24.0,
                        "terminal_xtrack_crossing_rate_threshold_mps": 0.010,
                        "terminal_xtrack_crossing_predicted_threshold_m": 0.004,
                        # Active speed profile:
                        # - distance is measured to the 30 mm radius boundary;
                        # - final 0.50 m uses constant deceleration to 0.15 m/s
                        #   for either supported fixed cruise speed;
                        # - measured radial <=30 mm commands immediate zero.
                        # The legacy staged values below remain compatibility
                        # parameters for validation and telemetry only.
                        "slow_distance_m": 0.80,
                        "decel_profile_distance_1_m": 0.55,
                        "decel_profile_distance_2_m": 0.35,
                        "decel_profile_distance_3_m": 0.15,
                        "final_speed_distance_m": 0.12,
                        "terminal_line_alignment_distance_m": 0.12,
                        "marking_terminal_speed_start_distance_m": 0.12,
                        "marking_final_creep_start_distance_m": 0.05,
                        "alignment_release_accel_distance_m": 0.20,
                        "heading_full_speed_deg": 2.0,
                        "heading_min_speed_deg": 4.0,
                        # Strict 30 mm circular stop gate for marking AND extension.
                        "waypoint_tolerance_m": 0.03,
                        "marking_final_creep_cross_track_m": 0.025,
                        "terminal_capture_gate_cross_track_m": 0.025,
                        "terminal_capture_gate_heading_deg": 4.0,
                        "terminal_capture_gate_hold_sec": 0.20,
                        "terminal_recovery_min_heading_error_deg": 8.0,
                        "terminal_recovery_correction_limit_deg": 22.0,
                        "terminal_recovery_lookahead_min_m": 0.30,
                        "terminal_exact_target_start_distance_m": 0.06,
                        "terminal_goal_intercept_distance_m": 0.90,
                        "terminal_goal_intercept_bearing_limit_deg": 22.0,
                        "terminal_native_pivot_enter_error_deg": 45.0,
                        "terminal_native_pivot_release_error_deg": 4.0,
                        # Deprecated compatibility parameter. It is no
                        # longer used to create a moving 60-degree vector;
                        # alignment now uses a zero-translation yaw-rate pivot.
                        "terminal_native_pivot_request_error_deg": 60.0,
                        "segment_pivot_keeper_timeout_sec": 10.0,
                        "segment_fast_capture_max_cross_track_m": 0.05,
                        "terminal_close_recovery_distance_m": 0.08,
                        "terminal_unready_hold_along_m": 0.05,
                        # Exact 30 mm stop/capture safety. The latency/buffer
                        # parameters remain compatibility-only and do not
                        # trigger zero in this build.
                        "marking_stop_settle_timeout_sec": 3.0,
                        "stationary_speed_tolerance_mps": 0.01,
                        "marking_stop_latency_sec": 0.24,
                        "marking_stop_extra_margin_m": 0.015,
                        "marking_stop_min_buffer_m": 0.060,
                        "marking_stop_max_buffer_m": 0.100,
                        "marking_stop_xtrack_limit_m": 0.020,
                        "marking_capture_arm_distance_m": 0.30,
                        "marking_capture_abort_distance_m": 0.45,
                        "miss_margin_m": 0.02,
                        "marking_along_track_abort_m": 0.02,
                        "waypoint_match_tolerance_m": 0.001,
                        "odom_timeout_sec": 0.50,
                        "waypoint_timeout_sec": 1.00,
                    }
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "Starting DYX rover safely: "
                    "E-stop asserted and mission "
                    "disabled until an authenticated "
                    "Start command"
                )
            ),
            mavros,
            rover_backend,
            cmd_vel_bridge,
            trajectory_generator,
            spray_controller,
            mission_manager,
            rpp_controller,
        ]
    )
