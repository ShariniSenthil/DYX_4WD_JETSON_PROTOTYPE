#!/usr/bin/env bash

# ROS setup scripts must be sourced before enabling nounset.
source /opt/ros/humble/setup.bash
source "$HOME/rover_ws/install/setup.bash"

set -uo pipefail

LOG="$HOME/rover_test.log"
STATE_FILE="$HOME/rover_ws/.rover_gate_state"
KEEPER="$HOME/rover_ws/scripts/rover_gate_keeper.py"
KEEPER_LOG="$HOME/rover_ws/rover_gate_keeper.log"

RESULT="120-SECOND TIME LIMIT"
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

# Remove an obsolete temporary gate keeper if it still exists.
pkill -TERM -f "/tmp/hold_rover_gate.py" \
    2>/dev/null || true

printf 'false true\n' > "$STATE_FILE"

if [ ! -f "$KEEPER" ]; then
    echo "ABORT: gate keeper does not exist:"
    echo "$KEEPER"
    exit 1
fi

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
    echo "Start rover.launch.py first."
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
echo "===== RTK CHECK ====="

RTK_HEALTH=$(
    timeout 5s ros2 topic echo \
        /rtk_correction_bridge/healthy \
        --once 2>/dev/null || true
)

echo "$RTK_HEALTH"

if ! grep -q "data: true" <<<"$RTK_HEALTH"; then
    echo "ABORT: RTK correction bridge is not healthy"
    exit 1
fi

GPS_FIX=$(
    timeout 5s ros2 topic echo \
        /mavros/gpsstatus/gps1/raw \
        --once \
        --field fix_type 2>/dev/null || true
)

echo "GPS fix type: $GPS_FIX"

if ! grep -q "^6$" <<<"$GPS_FIX"; then
    echo "ABORT: GPS fix_type is not 6"
    exit 1
fi


echo
echo "===== TRAJECTORY READY CHECK ====="

TRAJECTORY_READY=$(
    timeout 5s ros2 topic echo \
        /trajectory_generator/ready \
        --once 2>/dev/null || true
)

echo "$TRAJECTORY_READY"

if ! grep -q "data: true" <<<"$TRAJECTORY_READY"; then
    echo "ABORT: trajectory generator is not ready"
    exit 1
fi


echo
echo "===== MISSION PARAMETERS ====="

ros2 param get /trajectory_generator interpolation_spacing_m
ros2 param get /mission_manager navigation_tolerance_m
ros2 param get /mission_manager p1_approach_distance_m
ros2 param get /mission_manager p1_corridor_spacing_m
ros2 param get /mission_manager p1_a1_switch_distance_m
ros2 param get /rpp_controller waypoint_tolerance_m
ros2 param get /rpp_controller pivot_exit_angle_deg


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
echo "===== P1 TO P2 TEST STARTED ====="
echo "Physical E-stop must remain ready."


for step in {1..600}; do
    sleep 0.2

    NEW_LOG=$(sed -n "${START_LINE},\$p" "$LOG")

    if grep -q "MARKING WP 2 REACHED" <<<"$NEW_LOG"; then
        RESULT="WP2 REACHED"
        break
    fi

    if grep -qiE \
        "SAFE HOLD|marking capture missed|CAPTURE MISSED" \
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
        "P1 RUNTIME APPROACH BUILT|A1 distance to P1|A1 -> P1 points|DIRECT CURRENT->A1|PIVOT RIGHT|PIVOT LEFT|PIVOT ALIGNED|A1 HANDOFF|FINAL P1 APPROACH CORRIDOR|P1 CORRIDOR|MARKING APPROACH ACTIVATED|MARKING FINAL STRAIGHT|MARKING FINAL PULSE|MARKING TOLERANCE HOLD|MARKING WP|SAFE HOLD|CAPTURE" \
        <<<"$NEW_LOG" |
        tail -n 20

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
echo "===== P1 TO P2 MISSION LOG ====="

sed -n "${START_LINE},\$p" "$LOG" |
grep -E \
"P1 RUNTIME APPROACH BUILT|A1 distance to P1|A1 -> P1 points|DIRECT CURRENT->A1|PIVOT RIGHT|PIVOT LEFT|PIVOT ALIGNED|A1 HANDOFF|FINAL P1 APPROACH CORRIDOR|P1 CORRIDOR|MARKING APPROACH ACTIVATED|MARKING FINAL STRAIGHT|MARKING FINAL PULSE|MARKING TOLERANCE HOLD|MARKING WP|SAFE HOLD|CAPTURE|MISSION COMPLETE" |
tail -n 300