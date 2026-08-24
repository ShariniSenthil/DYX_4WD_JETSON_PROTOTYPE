#!/usr/bin/env bash
set -u

# MONITORING ONLY: starts a CSV logger + rosbag. It does not publish any rover
# control command and does not modify RPP/Mission Manager/PX4 state.

source /opt/ros/humble/setup.bash
if [ -f "$HOME/rover_ws/install/setup.bash" ]; then
  source "$HOME/rover_ws/install/setup.bash"
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${1:-$HOME/rover_ws/field_logs/field_${STAMP}}"
mkdir -p "$RUN_DIR"
export ROVER_FIELD_LOG_DIR="$RUN_DIR"

INFO="$RUN_DIR/run_info.txt"
{
  echo "DYX 4WD FIELD TEST"
  echo "started_local=$(date --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "kernel=$(uname -a)"
  echo "ros_distro=${ROS_DISTRO:-unknown}"
  echo "run_dir=$RUN_DIR"
  echo
  echo "=== ROS NODES ==="
  ros2 node list 2>&1 || true
  echo
  echo "=== ROS TOPICS ==="
  ros2 topic list -t 2>&1 || true
} > "$INFO"

PARAM_DIR="$RUN_DIR/params"
mkdir -p "$PARAM_DIR"
for NODE in \
  /mission_manager \
  /trajectory_generator \
  /rpp_controller \
  /cmd_vel_bridge \
  /spray_controller \
  /ntrip_to_px4_node; do
  SAFE="$(echo "$NODE" | tr '/' '_' | sed 's/^_//')"
  timeout 4 ros2 param dump "$NODE" > "$PARAM_DIR/${SAFE}.yaml" 2> "$PARAM_DIR/${SAFE}.err" || true
done

CHECKSUMS="$RUN_DIR/source_sha256.txt"
: > "$CHECKSUMS"
for FILE in \
  "$HOME/rover_ws/src/trajectory_generator/trajectory_generator/trajectory_generator_node.py" \
  "$HOME/rover_ws/src/mission_manager/mission_manager/mission_manager_node.py" \
  "$HOME/rover_ws/src/rpp_controller/rpp_controller/rpp_controller_node.py" \
  "$HOME/rover_ws/src/jetson_4wd_control/jetson_4wd_control/cmd_vel_bridge.py" \
  "$HOME/rover_ws/src/spray_controller/spray_controller/spray_controller_node.py" \
  "$HOME/rover_ws/src/rover_bringup/launch/rover.launch.py"; do
  if [ -f "$FILE" ]; then
    sha256sum "$FILE" >> "$CHECKSUMS"
  fi
done

python3 "$HOME/rover_ws/scripts/field_test_logger.py" > "$RUN_DIR/field_test_logger_console.log" 2>&1 &
LOGGER_PID=$!

TOPICS=(
  /mavros/local_position/odom
  /mavros/global_position/global
  /mavros/gpsstatus/gps1/raw
  /mavros/state
  /mavros/setpoint_raw/local
  /nav_path
  /mission_waypoints
  /trajectory_generator/path_types
  /trajectory_generator/marking_indices
  /trajectory_generator/path_signature
  /active_waypoint
  /segment_goal
  /mission_manager/segment_goal_metadata
  /mission_enable
  /emergency_stop
  /marking_active
  /trajectory_generator/ready
  /mission_manager/status
  /mission_manager/point_event
  /rpp/velocity_ned
  /rpp/command_speed_mps
  /rpp/acceleration_active
  /rpp/acceleration_progress_m
  /rpp/deceleration_active
  /rpp/deceleration_progress_m
  /rpp/deceleration_remaining_m
  /rpp/xtrack_speed_cap_active
  /rpp/xtrack_speed_cap_mps
  /rpp/terminal_precision_armed
  /rpp/terminal_bearing_frozen
  /rpp/terminal_correction_deg
  /rpp/xtrack_mm
  /rpp/goal_distance_mm
  /rpp/along_track_remaining_mm
  /rpp/closest_goal_distance_mm
  /rpp/accuracy
  /rpp/geometry_debug
  /rpp/guidance_debug
  /rpp/speed_debug
  /rpp/tracking_debug
  /rpp/pivot_debug
  /rpp/terminal_certificate
  /rpp/terminal_result
  /cmd_vel_bridge/backend_heartbeat_healthy
  /rtk_correction_bridge/healthy
  /rtk_correction_bridge/correction_age_sec
  /spray/status
  /spray/result
)

ros2 bag record -o "$RUN_DIR/rosbag" "${TOPICS[@]}" > "$RUN_DIR/rosbag_console.log" 2>&1 &
BAG_PID=$!

CLEANED=0
cleanup() {
  if [ "$CLEANED" -eq 1 ]; then
    return
  fi
  CLEANED=1
  echo
  echo "Stopping field logging..."
  kill -INT "$BAG_PID" 2>/dev/null || true
  kill -INT "$LOGGER_PID" 2>/dev/null || true
  wait "$BAG_PID" 2>/dev/null || true
  wait "$LOGGER_PID" 2>/dev/null || true
  {
    echo
    echo "stopped_local=$(date --iso-8601=seconds)"
  } >> "$INFO"
  echo "Saved field log: $RUN_DIR"
  echo "Use: bash ~/rover_ws/scripts/bundle_field_test_log.sh '$RUN_DIR'"
}
trap cleanup INT TERM EXIT

cat <<EOF

============================================================
FIELD LOGGING ACTIVE
Run folder: $RUN_DIR
============================================================
Now START the mission from your normal frontend/backend.
When the mission/test is finished, return here and press Ctrl+C ONCE.

Also download the matching PX4 .ulg from QGroundControl after the run.
============================================================
EOF

wait "$BAG_PID"
