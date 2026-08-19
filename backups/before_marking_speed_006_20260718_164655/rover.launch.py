#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """
    P1-only direct-A1 plus one-metre corridor mission bringup.

    This launch does not arm PX4, select OFFBOARD, release the E-stop,
    or enable the mission.
    """

    mavros = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "source /opt/ros/humble/setup.bash && "
            "ros2 launch mavros node.launch "
            "fcu_url:=/dev/ttyACM0:921600 "
            "gcs_url:=udp://:14550@192.168.3.102:14550 "
            "pluginlists_yaml:="
            "/opt/ros/humble/share/mavros/launch/"
            "px4_pluginlists.yaml "
            "config_yaml:="
            "/opt/ros/humble/share/mavros/launch/"
            "px4_config.yaml "
            "tgt_system:=1 "
            "tgt_component:=1 "
            "fcu_protocol:=v2.0",
        ],
        output="screen",
    )

    cmd_vel_bridge = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="Starting autonomous cmd_vel_bridge"),
            Node(
                package="jetson_4wd_control",
                executable="cmd_vel_bridge",
                name="cmd_vel_bridge",
                output="screen",
                parameters=[{
                    "command_timeout_sec": 0.25,
                    "maximum_speed_mps": 0.40,
                }],
            ),
        ],
    )

    trajectory_generator = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="Starting interpolated trajectory_generator"),
            Node(
                package="trajectory_generator",
                executable="trajectory_generator_node",
                name="trajectory_generator",
                output="screen",
                parameters=[{
                    "mission_file": (
                        "/home/flash/rover_ws/missions/mission1.csv"
                    ),
                    "frame_id": "map",
                    "input_mode": "gps",
                    "expected_marking_waypoints": 4,
                    "global_position_topic": (
                        "/mavros/global_position/raw/fix"
                    ),
                    "local_odom_topic": (
                        "/mavros/local_position/odom"
                    ),
                    "gps_status_topic": (
                        "/mavros/gpsstatus/gps1/raw"
                    ),
                    "rtk_health_topic": (
                        "/rtk_correction_bridge/healthy"
                    ),
                    "rtk_correction_age_topic": (
                        "/rtk_correction_bridge/correction_age_sec"
                    ),
                    "required_gps_fix_type": 6,
                    "rtk_stable_sec": 3.0,
                    "max_correction_age_sec": 2.0,
                    "reference_timeout_sec": 1.0,
                    "max_reference_skew_sec": 0.25,
                    "max_target_distance_m": 1000.0,
                    "max_abs_coordinate_m": 10000.0,
                    "max_path_points": 10000,
                    "interpolation_spacing_m": 0.05,
                }],
            ),
        ],
    )

    mission_manager = TimerAction(
        period=11.0,
        actions=[
            LogInfo(msg="Starting direct-A1 P1 mission_manager"),
            Node(
                package="mission_manager",
                executable="mission_manager_node",
                name="mission_manager",
                output="screen",
                parameters=[{
                    "local_frame": "map",
                    "expected_marking_waypoints": 4,
                    "navigation_tolerance_m": 0.03,
                    "intermediate_switch_distance_m": 0.10,
                    "pass_through_lookahead_distance_m": 0.90,
                    "marking_lookahead_distance_m": 0.75,
                    "post_marking_alignment_distance_m": 0.75,
                    "marking_dwell_sec": 3.0,
                    "waypoint_match_tolerance_m": 0.001,
                    "odom_timeout_sec": 0.50,
                    "max_path_points": 10000,
                    "p1_approach_distance_m": 1.00,
                    "p1_corridor_spacing_m": 0.05,
                    "p1_marking_handoff_distance_m": 0.18,
                    "p1_a1_switch_distance_m": 0.30,
                    "p1_corridor_lookahead_distance_m": 0.25,
                }],
            ),
        ],
    )

    rpp_controller = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg="Starting P1-approach precision RPP"),
            Node(
                package="rpp_controller",
                executable="rpp_controller_node",
                name="rpp_controller",
                output="screen",
                parameters=[{
                    "local_frame": "map",
                    "p1_approach_speed_mps": 0.25,
                    "p1_a1_switch_distance_m": 0.30,
                    "p1_a1_slow_distance_m": 0.50,
                    "p1_a1_handoff_speed_mps": 0.10,
                    "p1_corridor_speed_mps": 0.12,
                    "p1_corridor_start_distance_m": 1.00,
                    "p1_corridor_correction_limit_deg": 8.0,
                    "forward_speed_mps": 0.40,
                    "turn_speed_mps": 0.06,
                    "marking_approach_speed_mps": 0.06,
                    "marking_near_speed_mps": 0.03,
                    "marking_near_distance_m": 0.15,
                    "pivot_vector_speed_mps": 0.06,
                    "slow_heading_error_deg": 20.0,
                    "pivot_enter_angle_deg": 12.0,
                    "pivot_exit_angle_deg": 3.0,
                    "marking_alignment_pivot_deg": 15.0,
                    "pivot_hold_sec": 0.30,
                    "marking_alignment_distance_m": 1.10,
                    "marking_no_pivot_distance_m": 1.00,
                    "final_bearing_lock_distance_m": 0.18,
                    "final_bearing_lock_error_deg": 10.0,
                    "final_alignment_abort_distance_m": 0.08,
                    "pass_through_no_pivot_distance_m": 0.25,
                    "waypoint_tolerance_m": 0.03,
                    "waypoint_match_tolerance_m": 0.001,
                    "miss_margin_m": 0.02,
                    "odom_timeout_sec": 0.50,
                    "waypoint_timeout_sec": 1.00,
                }],
            ),
        ],
    )

    return LaunchDescription([
        mavros,
        cmd_vel_bridge,
        trajectory_generator,
        mission_manager,
        rpp_controller,
    ])