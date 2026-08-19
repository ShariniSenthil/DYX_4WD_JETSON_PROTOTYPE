#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rover_ws/install/setup.bash"
set -u

fail() {
    echo "FAIL: $*" >&2
    exit 1
}

for node in \
    /cmd_vel_bridge \
    /mission_manager \
    /rover_backend \
    /rpp_controller \
    /trajectory_generator
do
    count="$(ros2 node list 2>/dev/null | grep -xc "$node" || true)"
    [[ "$count" == "1" ]] || fail "$node count=$count"
    echo "PASS node $node"
done

mode="$(ros2 param get /mission_manager marking_error_mode 2>/dev/null || true)"
tolerance="$(ros2 param get /mission_manager marking_tolerance_m 2>/dev/null || true)"
hold="$(ros2 param get /mission_manager marking_hold_sec 2>/dev/null || true)"
speed="$(ros2 param get /mission_manager stationary_speed_tolerance_mps 2>/dev/null || true)"
cruise="$(ros2 param get /rpp_controller cruise_speed_mps 2>/dev/null || true)"
maximum="$(ros2 param get /cmd_vel_bridge maximum_speed_mps 2>/dev/null || true)"
spacing="$(ros2 param get /trajectory_generator interpolation_spacing_m 2>/dev/null || true)"

[[ "$mode" == *"L1_XTRACK_ALONG"* ]] || fail "marking_error_mode: $mode"
[[ "$tolerance" == *"0.03"* ]] || fail "marking_tolerance_m: $tolerance"
[[ "$hold" == *"3.0"* ]] || fail "marking_hold_sec: $hold"
[[ "$speed" == *"0.01"* ]] || fail "stationary_speed_tolerance_mps: $speed"
[[ "$cruise" == *"0.4"* ]] || fail "cruise_speed_mps: $cruise"
[[ "$maximum" == *"0.4"* ]] || fail "maximum_speed_mps: $maximum"
[[ "$spacing" == *"0.05"* ]] || fail "interpolation_spacing_m: $spacing"

echo "PASS marking_error_mode=L1_XTRACK_ALONG"
echo "PASS combined budget=30 mm"
echo "PASS stationary hold=3.0 s at <=10 mm/s"
echo "PASS normal speed=0.40 m/s"
echo "PASS interpolation spacing=50 mm"
echo
echo "L1 COMBINED 30MM MARKING CONTRACT VERIFIED"