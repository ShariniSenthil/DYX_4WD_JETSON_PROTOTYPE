#!/usr/bin/env python3

"""Production launch for the DYX 4WD rover.

This launch keeps the current production stack for MAVROS, backend, RTK,
trajectory generation, mission management, spray control and cmd_vel safety.

RPP is intentionally simplified around the Aug-13 production controller:
- retained 50 mm /nav_path tracking
- 30 mm semantic stop
- native pivot only for true heading error >=45 deg
- dynamic +/-60 deg PX4 native-pivot carrier
- true-heading pivot release at 12 deg
- original surveyed path tracking; no all-leg re-anchor
- no precision guidance / precision pivot / precision terminal FSM
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, TimerAction
from launch_ros.actions import Node

MISSION_FILE = "/home/flash/rover_ws/missions/mission.csv"
MISSION_METADATA_FILE = (
    "/home/flash/.local/share/dyx_rover/runtime/mission_metadata.json"
)

CRUISE_SPEED_MPS = 0.60
TERMINAL_FLOOR_SPEED_MPS = 0.15


def generate_launch_description() -> LaunchDescription:
    mavros = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "source /opt/ros/humble/setup.bash && "
            "ros2 launch mavros node.launch "
            "fcu_url:=/dev/ttyACM0:921600 "
            "gcs_url:=udp://:14550@192.168.3.105:14550 "
            "pluginlists_yaml:=/opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml "
            "config_yaml:=/opt/ros/humble/share/mavros/launch/px4_config.yaml "
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
        respawn=False,
    )

    rover_backend = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="Starting production rover_backend"),
            rover_backend_node,
        ],
    )

    cmd_vel_bridge = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg="Starting heartbeat-gated cmd_vel_bridge"),
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
                        "command_timeout_sec": 0.25,
                        "backend_heartbeat_timeout_sec": 1.5,
                        "maximum_speed_mps": 1.00,
                        "maximum_yaw_rate_radps": 0.20,
                    }
                ],
            ),
        ],
    )

    trajectory_generator = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg="Starting production trajectory_generator"),
            Node(
                package="trajectory_generator",
                executable="trajectory_generator_node",
                name="trajectory_generator",
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "mission_file": MISSION_FILE,
                        "mission_metadata_file": MISSION_METADATA_FILE,
                        "frame_id": "map",
                        "global_position_topic": "/mavros/global_position/raw/fix",
                        "gp_origin_topic": "/mavros/global_position/gp_origin",
                        "fused_global_position_topic": "/mavros/global_position/global",
                        "localization_mode": "px4_origin",
                        "local_odom_topic": "/mavros/local_position/odom",
                        "gps_status_topic": "/mavros/gpsstatus/gps1/raw",
                        "rtk_health_topic": "/rtk_correction_bridge/healthy",
                        "rtk_correction_age_topic": (
                            "/rtk_correction_bridge/correction_age_sec"
                        ),
                        "required_gps_fix_type": 6,
                        "rtk_stable_sec": 3.0,
                        "max_correction_age_sec": 2.0,
                        "reference_timeout_sec": 1.0,
                        "max_reference_skew_sec": 0.25,
                        "origin_consistency_max_m": 0.30,
                        "max_target_distance_m": 1000.0,
                        "max_abs_coordinate_m": 10000.0,
                        "maximum_marking_points": 10000,
                        "maximum_navigation_points": 200000,
                        "interpolation_spacing_m": 0.05,
                        "minimum_segment_length_m": 0.001,
                        "minimum_dummy_clearance_m": 0.05,
                    }
                ],
            ),
        ],
    )

    spray_controller = TimerAction(
        period=7.5,
        actions=[
            LogInfo(msg="Starting production AUX5 spray controller"),
            Node(
                package="spray_controller",
                executable="spray_controller_node",
                name="spray_controller",
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "enabled": True,
                        "press_value": 1.0,
                        "release_value": 0.0,
                        "spray_duration_sec": 0.50,
                        "pre_spray_stable_sec": 0.25,
                        "command_timeout_sec": 1.0,
                        "release_retry_interval_sec": 0.25,
                        "hard_press_timeout_sec": 5.0,
                        "mavros_state_timeout_sec": 2.5,
                        "mission_status_timeout_sec": 1.0,
                        "marking_active_timeout_sec": 0.50,
                        "require_px4_armed": True,
                        "require_px4_offboard": True,
                        "journal_path": (
                            "/home/flash/.ros/dyx_spray_controller_journal.json"
                        ),
                    }
                ],
            ),
        ],
    )

    mission_manager = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="Starting production mission_manager"),
            Node(
                package="mission_manager",
                executable="mission_manager_node",
                name="mission_manager",
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "local_frame": "map",
                        "marking_tolerance_m": 0.03,
                        "arrival_settle_sec": 0.30,
                        "marking_hold_sec": 3.00,
                        "stationary_speed_tolerance_mps": 0.01,
                        "dummy_arrival_tolerance_m": 0.03,
                        "spray_required": True,
                        "spray_confirmation_timeout_sec": 7.0,
                        "spray_status_timeout_sec": 2.0,
                        "waypoint_match_tolerance_m": 0.002,
                        "odom_timeout_sec": 0.50,
                        "maximum_navigation_points": 200000,
                        "precision_path_contract_enabled": True,
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
            LogInfo(msg="Starting Aug-13 production RPP controller"),
            Node(
                package="rpp_controller",
                executable="rpp_controller_node",
                name="rpp_controller",
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "local_frame": "map",
                        "cruise_speed_mps": CRUISE_SPEED_MPS,
                        "minimum_speed_mps": 0.04,

                        "acceleration_enabled": True,
                        "acceleration_distance_m": 0.20,
                        "acceleration_startup_ceiling_mps": 0.15,
                        "acceleration_max_progress_jump_m": 0.10,
                        "acceleration_max_dt_sec": 0.10,
                        "command_speed_rise_limit_mps2": 3.00,
                        "command_speed_fall_limit_mps2": 2.00,

                        "deceleration_enabled": True,
                        "deceleration_distance_m": 0.50,
                        "deceleration_floor_speed_mps": TERMINAL_FLOOR_SPEED_MPS,
                        "deceleration_max_progress_jump_m": 0.10,
                        "deceleration_max_dt_sec": 0.10,

                        "waypoint_tolerance_m": 0.03,
                        "marking_stop_settle_timeout_sec": 3.0,
                        "stationary_speed_tolerance_mps": 0.01,

                        "pivot_enter_angle_deg": 45.0,
                        "pivot_exit_angle_deg": 12.0,
                        "terminal_native_pivot_enter_error_deg": 45.0,
                        "terminal_native_pivot_release_error_deg": 12.0,
                        "terminal_native_pivot_request_error_deg": 60.0,
                        "alignment_hold_sec": 0.20,
                        "maximum_yaw_rate_radps": 0.20,
                        "minimum_yaw_rate_radps": 0.06,
                        "pivot_yaw_kp": 1.00,

                        "segment_alignment_speed_mps": CRUISE_SPEED_MPS,
                        "segment_alignment_recovery_speed_mps": CRUISE_SPEED_MPS,
                        "segment_alignment_correction_limit_deg": 18.0,
                        "segment_alignment_cross_track_tolerance_m": 0.03,
                        "segment_alignment_max_cross_track_m": 0.60,
                        "path_correction_limit_deg": 18.0,
                        "terminal_line_correction_limit_deg": 18.0,
                        "line_tracking_lookahead_m": 0.55,

                        "xtrack_priority_enter_m": 0.015,
                        "xtrack_priority_exit_m": 0.008,
                        "xtrack_priority_hold_sec": 0.30,
                        "xtrack_priority_speed_mps": CRUISE_SPEED_MPS,
                        "xtrack_priority_lookahead_m": 0.55,
                        "xtrack_priority_correction_limit_deg": 22.0,
                        "xtrack_correction_slew_rate_degps": 30.0,
                        "xtrack_priority_release_heading_deg": 12.0,

                        "nav_path_lookahead_m": 0.55,
                        "nav_path_point_reach_m": 0.075,

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
                    "Starting DYX rover safely: E-stop asserted and mission "
                    "disabled until an authenticated Start command"
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
