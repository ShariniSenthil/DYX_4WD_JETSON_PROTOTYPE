#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source "$HOME/rover_ws/install/setup.bash"

set -euo pipefail

SPEED="${1:-}"

if [[ -z "$SPEED" ]]; then
    echo "Usage: $0 SPEED_MPS"
    exit 1
fi

python3 - "$SPEED" <<'PY'
import sys

speed = float(sys.argv[1])

if not 0.0 < speed <= 0.10:
    raise SystemExit("Speed must be greater than 0 and at most 0.10 m/s.")
PY

STATE_FILE="$HOME/rover_ws/.rover_gate_state"

cleanup() {
    echo
    echo "===== SAFETY STOP ====="

    printf 'false true\n' > "$STATE_FILE"

    ros2 service call \
        /mavros/cmd/arming \
        mavros_msgs/srv/CommandBool \
        "{value: false}" >/dev/null 2>&1 || true

    ros2 service call \
        /mavros/set_mode \
        mavros_msgs/srv/SetMode \
        "{base_mode: 0, custom_mode: 'MANUAL'}" \
        >/dev/null 2>&1 || true

    echo "Mission disabled, E-stop active, disarmed, MANUAL."
}

trap cleanup EXIT INT TERM

if ros2 node list | grep -qx "/rpp_controller"; then
    echo "ABORT: /rpp_controller is running."
    echo "Stop it before this direct speed test."
    exit 1
fi

for node in /cmd_vel_bridge /mission_manager /trajectory_generator; do
    if ! ros2 node list | grep -qx "$node"; then
        echo "ABORT: missing node $node"
        exit 1
    fi
done

if ! pgrep -f \
    "$HOME/rover_ws/scripts/rover_gate_keeper.py" \
    >/dev/null; then
    nohup python3 \
        "$HOME/rover_ws/scripts/rover_gate_keeper.py" \
        >"$HOME/rover_gate_keeper.log" 2>&1 &

    sleep 2
fi

echo "===== PREPARE OFFBOARD ====="

printf 'true true\n' > "$STATE_FILE"
sleep 3

ros2 service call \
    /mavros/set_mode \
    mavros_msgs/srv/SetMode \
    "{base_mode: 0, custom_mode: 'OFFBOARD'}"

sleep 1

ros2 service call \
    /mavros/cmd/arming \
    mavros_msgs/srv/CommandBool \
    "{value: true}"

sleep 0.2

STATE_OUTPUT="$(
    timeout 6s ros2 topic echo /mavros/state --once
)"

echo "$STATE_OUTPUT"

if ! grep -q "armed: true" <<<"$STATE_OUTPUT"; then
    echo "WARNING: armed state not yet visible; continuing speed test."
fi

if ! grep -q "mode: OFFBOARD" <<<"$STATE_OUTPUT"; then
    echo "ABORT: rover is not in OFFBOARD."
    exit 1
fi

echo
echo "===== SPEED TEST: ${SPEED} m/s ====="
echo "Three four-second start-from-rest trials will run."
echo

printf 'true false\n' > "$STATE_FILE"
sleep 0.5

python3 \
    "$HOME/rover_ws/scripts/test_minimum_speed.py" \
    "$SPEED" \
    --duration 4 \
    --trials 3
