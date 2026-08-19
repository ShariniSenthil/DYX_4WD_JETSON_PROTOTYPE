#!/usr/bin/env bash
set -eo pipefail

# ROS setup files may read unset AMENT variables. Source them before
# enabling Bash nounset mode.
source /opt/ros/humble/setup.bash
source "${HOME}/rover_ws/install/setup.bash"

set -u

SCRIPT="${HOME}/rover_ws/scripts/jetson_px4_timestamp_latency_test.py"

if [[ ! -f "${SCRIPT}" ]]; then
    echo "ERROR: missing ${SCRIPT}" >&2
    exit 1
fi

chmod +x "${SCRIPT}"

echo "===== READ-ONLY JETSON / PX4 TIMESTAMP TEST ====="
echo "This script does not publish control commands."
echo "Run it during the same Offboard mission that generates the PX4 ULog."
echo

python3 "${SCRIPT}" \
    --duration "${1:-120}" \
    --output-dir "${HOME}/rover_ws/logs/timing"