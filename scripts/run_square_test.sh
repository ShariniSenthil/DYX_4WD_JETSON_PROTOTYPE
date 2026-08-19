#!/usr/bin/env bash

# ROS setup scripts must be sourced before enabling nounset.
source /opt/ros/humble/setup.bash
source "$HOME/rover_ws/install/setup.bash"

set -uo pipefail

LOG="$HOME/rover_test.log"
STATE_FILE="$HOME/rover_ws/.rover_gate_state"
KEEPER="$HOME/rover_ws/scripts/rover_gate_keeper.py"
KEEPER_LOG="$HOME/rover_ws/rover_gate_keeper.log"

RESULT="240-SECOND TIME LIMIT"
START_LINE=1


set_gate() {
    local mission="$1"
    local estop="$2"
    local temporary="${STATE_FILE}.tmp"

    printf '%s %s\n' "$mission" "$estop" > "$temporary"
    mv "$temporary" "$STATE_FILE"
    sleep 1
}


safe_stop() {
    echo
    echo "===== SAFETY STOP ====="

    set_gate false true

    timeout 8s ros2 service call \
        /mavros/cmd/arming \
        mavros_msgs/srv/CommandBool \
        "{value: false}" \
        >/dev/null 2>&1 || true

    timeout 8s ros2 service call \
        /mavros/set_mode \
        mavros_msgs/srv/SetMode \
        "{base_mode: 0, custom_mode: 'MANUAL'}" \
        >/dev/null 2>&1 || true

    sleep 2
}


echo "===== START GATE KEEPER ====="

# Remove the obsolete /tmp gate keeper if it still exists.
pkill -TERM -f "/tmp/hold_rover_gate.py" \
    2>/dev/null || true

printf 'false true\n' > "$STATE_FILE"

if ! pgrep -f "$KEEPER" >/dev/null; then
    nohup python3 "$KEEPER" \
        > "$KEEPER_LOG" 2>&1 &

    sleep 2
fi

KEEPER_COUNT=$(pgrep -fc "$KEEPER")

echo "Gate keeper count: $KEEPER_COUNT"

if [ "$KEEPER_COUNT" -ne 1 ]; then
    echo "ABORT: expected exactly one gate keeper"
    exit 1
fi


trap safe_stop EXIT INT TERM


echo
echo "===== PRE-FLIGHT CHECK ====="

if [ ! -f "$LOG" ]; then
    echo "ABORT: $LOG does not exist"
    exit 1
fi

LAUNCH_COUNT=$(pgrep -fc \
    "ros2 launch rover_bringup rover.launch.py")

echo "Rover launch count: $LAUNCH_COUNT"

if [ "$LAUNCH_COUNT" -ne 1 ]; then
    echo "ABORT: expected exactly one rover launch"
    exit 1
fi

NODE_LIST=$(ros2 node list 2>/dev/null)

for required_node in \
    /cmd_vel_bridge \
    /mission_manager \
    /trajectory_generator \
    /rpp_controller
do
    if ! grep -qx "$required_node" <<<"$NODE_LIST"; then
        echo "ABORT: missing node $required_node"
        exit 1
    fi
done


safe_stop


echo
echo "===== SAFE PX4 STATE ====="

INITIAL_STATE=$(
    timeout 5s ros2 topic echo \
        /mavros/state --once 2>/dev/null || true
)

echo "$INITIAL_STATE"

if ! grep -q "armed: false" <<<"$INITIAL_STATE" ||
   ! grep -q "mode: MANUAL" <<<"$INITIAL_STATE"; then
    echo "ABORT: PX4 is not disarmed in MANUAL"
    exit 1
fi


echo
echo "===== RPP PARAMETERS ====="

ros2 param get /rpp_controller marking_near_speed_mps
ros2 param get /rpp_controller waypoint_tolerance_m
ros2 param get /rpp_controller final_bearing_lock_distance_m
ros2 param get /rpp_controller final_bearing_lock_error_deg


echo
echo "===== ENABLE MISSION WITH ESTOP ACTIVE ====="

set_gate true true


echo
echo "===== REQUEST OFFBOARD ====="

timeout 8s ros2 service call \
    /mavros/set_mode \
    mavros_msgs/srv/SetMode \
    "{base_mode: 0, custom_mode: 'OFFBOARD'}"


sleep 1


echo
echo "===== ARM PX4 ====="

timeout 8s ros2 service call \
    /mavros/cmd/arming \
    mavros_msgs/srv/CommandBool \
    "{value: true}"


echo
echo "===== WAIT FOR ARMED OFFBOARD ====="

ARMED_OK=0

for attempt in {1..20}; do
    STATE=$(
        timeout 4s ros2 topic echo \
            /mavros/state --once 2>/dev/null || true
    )

    if grep -q "armed: true" <<<"$STATE" &&
       grep -q "mode: OFFBOARD" <<<"$STATE"; then
        ARMED_OK=1
        echo "$STATE"
        break
    fi

    sleep 0.5
done

if [ "$ARMED_OK" -ne 1 ]; then
    echo "ABORT: PX4 did not enter armed OFFBOARD"
    exit 1
fi


# Ignore all previous log messages.
START_LINE=$(( $(wc -l < "$LOG") + 1 ))


echo
echo "===== RELEASE ESTOP ====="

set_gate true false


GATE_OK=0

for attempt in {1..30}; do
    NEW_LOG=$(sed -n "${START_LINE},\$p" "$LOG")

    if grep -q "mission=True" <<<"$NEW_LOG" &&
       grep -q "estop=False" <<<"$NEW_LOG"; then
        GATE_OK=1
        break
    fi

    sleep 0.2
done

if [ "$GATE_OK" -ne 1 ]; then
    echo "ABORT: bridge did not receive mission=True estop=False"
    exit 1
fi


echo
echo "===== FULL SQUARE TEST STARTED ====="
echo "Physical E-stop must remain ready."


for step in {1..1200}; do
    sleep 0.2

    NEW_LOG=$(sed -n "${START_LINE},\$p" "$LOG")

    if grep -qiE \
        "SAFE HOLD|marking capture missed" \
        <<<"$NEW_LOG"; then
        RESULT="SAFE HOLD DETECTED"
        break
    fi

    if grep -q "MISSION COMPLETE" <<<"$NEW_LOG"; then
        RESULT="MISSION COMPLETE"
        break
    fi

    if (( step % 25 == 0 )); then
        elapsed=$((step / 5))

        echo
        echo "===== RUNNING: ${elapsed} SECONDS ====="

        grep -E \
        "MARKING WP|ACTIVE MARKING|POST-MARKING|LOOKAHEAD|PIVOT|SAFE HOLD|CAPTURE" \
        <<<"$NEW_LOG" |
        tail -n 14

        STATE_OK=0

        for attempt in 1 2 3; do
            STATE=$(
                timeout 4s ros2 topic echo \
                    /mavros/state --once 2>/dev/null || true
            )

            if grep -q "armed: true" <<<"$STATE" &&
               grep -q "mode: OFFBOARD" <<<"$STATE"; then
                STATE_OK=1
                break
            fi
        done

        if [ "$STATE_OK" -ne 1 ]; then
            RESULT="PX4 LEFT ARMED OFFBOARD"
            break
        fi

        CURRENT_GATE=$(cat "$STATE_FILE" 2>/dev/null || true)

        if [ "$CURRENT_GATE" != "true false" ]; then
            RESULT="SAFETY GATE CHANGED"
            break
        fi
    fi
done


safe_stop
trap - EXIT INT TERM


echo
echo "===== TEST RESULT: $RESULT ====="

echo
echo "===== FINAL PX4 STATE ====="

timeout 5s ros2 topic echo \
    /mavros/state --once


echo
echo "===== FULL SQUARE MISSION LOG ====="

sed -n "${START_LINE},\$p" "$LOG" |
grep -E \
"MARKING WP|ACTIVE MARKING|POST-MARKING|LOOKAHEAD|PIVOT|SAFE HOLD|CAPTURE|MISSION COMPLETE" |
tail -n 300
