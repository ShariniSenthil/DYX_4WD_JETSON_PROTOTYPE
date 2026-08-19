#!/usr/bin/env bash
set -eo pipefail

# ROS setup files may read unset AMENT variables. Source them before
# enabling Bash nounset mode.
source /opt/ros/humble/setup.bash
source "${HOME}/rover_ws/install/setup.bash"

set -u

echo "===== REQUIRED TOPICS ====="

required_topics=(
  "/rpp/velocity_ned"
  "/mavros/setpoint_raw/local"
  "/mavros/timesync_status"
  "/mavros/local_position/odom"
)

topic_list="$(ros2 topic list)"

failed=0
for topic in "${required_topics[@]}"; do
  if grep -Fxq "${topic}" <<<"${topic_list}"; then
    type="$(ros2 topic type "${topic}" 2>/dev/null || true)"
    echo "PASS | ${topic} | ${type}"
  else
    echo "FAIL | ${topic} | missing"
    failed=1
  fi
done

echo
echo "===== CURRENT TOPIC RATES ====="
timeout 6s ros2 topic hz /rpp/velocity_ned || true
timeout 6s ros2 topic hz /mavros/setpoint_raw/local || true
timeout 6s ros2 topic hz /mavros/timesync_status || true

echo
echo "===== JETSON CLOCK ====="
date --iso-8601=ns
timedatectl show \
  -p NTPSynchronized \
  -p NTP \
  -p TimeUSec || true

if command -v chronyc >/dev/null 2>&1; then
  chronyc tracking || true
fi

exit "${failed}"