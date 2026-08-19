#!/usr/bin/env bash
# Source ROS 2 + the overlay, then exec the auto bag recorder.
set -euo pipefail

set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f /home/flash/rover_ws/install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /home/flash/rover_ws/install/setup.bash
elif [[ -f "$(dirname "$0")/../install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$(dirname "$0")/../install/setup.bash"
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export BAGS_DIR="${BAGS_DIR:-$HOME/bags_jet}"
mkdir -p "$BAGS_DIR"

exec python3 "$(cd "$(dirname "$0")" && pwd)/bag_autorecord.py"
