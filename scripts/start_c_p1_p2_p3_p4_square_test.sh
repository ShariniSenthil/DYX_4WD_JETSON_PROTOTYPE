#!/usr/bin/env bash

# Production C -> P1 -> P2 -> P3 -> P4 square mission field test.
#
# Canonical baseline:
#   rover_ws_full_backup_20260723_132953.tar.gz
#
# This script:
# - uses rover_backend REST operations for prepare/start/stop;
# - does not use rover_gate_keeper.py;
# - does not publish /mission_enable or /emergency_stop directly;
# - does not modify mission.csv;
# - temporarily injects a four-point runtime path:
#       fresh C -> P1 -> P2 -> P3 -> P4
#   with generated spacing <= 0.05 m;
# - waits for production mission-manager COMPLETED events at P1, P2, P3 and P4;
# - always asserts E-stop, stops the mission, disarms and requests MANUAL;
# - restores the normal full prepared mission only after a temporary
#   test-path supervisor has been started.
# - verifies the 0.40 m/s PX4 native velocity-vector pivot contract.

source /opt/ros/humble/setup.bash
source "$HOME/rover_ws/install/setup.bash"

set -Eeuo pipefail

BACKEND_BASE_URL="${DYX_TEST_BACKEND_URL:-http://127.0.0.1:5001}"
LOG_FILE="${DYX_TEST_ROVER_LOG:-$HOME/rover_test.log}"
SUPERVISOR="$HOME/rover_ws/scripts/c_p1_p2_p3_p4_square_test_supervisor.py"

RUNTIME_DIR="/tmp/dyx_square_p1_p2_p3_p4_test"
READY_FILE="$RUNTIME_DIR/ready.json"
RESULT_FILE="$RUNTIME_DIR/result.json"
EVENT_FILE="$RUNTIME_DIR/events.jsonl"
SUPERVISOR_LOG="$RUNTIME_DIR/supervisor.log"
SUPERVISOR_PID_FILE="$RUNTIME_DIR/supervisor.pid"

TEST_TIMEOUT_SEC="${DYX_TEST_TIMEOUT_SEC:-180}"
HEADING_TOLERANCE_DEG="${DYX_TEST_HEADING_TOLERANCE_DEG:-4.0}"

TOKEN=""
SUPERVISOR_PID=""
START_LINE=1
FINAL_RESULT="ABORTED"
CLEANUP_STARTED=0
TEST_PATH_STARTED=0

BACKEND_PID=""
CREDENTIAL_SOURCE=""
DYX_STATIC_USERNAME=""
DYX_STATIC_PASSWORD=""


json_get() {
    local expression="$1"

    python3 -c '
import json
import sys

expression = sys.argv[1]
payload = json.load(sys.stdin)

value = payload
for token in expression.split("."):
    if token == "":
        continue
    if isinstance(value, dict):
        value = value.get(token)
    else:
        value = None
        break

if isinstance(value, bool):
    print("true" if value else "false")
elif value is None:
    print("")
elif isinstance(value, (dict, list)):
    print(json.dumps(value, separators=(",", ":"), sort_keys=True))
else:
    print(value)
' "$expression"
}


find_running_backend_pid() {
    local pid=""
    local cmdline=""

    while IFS= read -r pid; do
        [[ -n "$pid" ]] || continue
        [[ -r "/proc/$pid/cmdline" ]] || continue

        cmdline="$(
            tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null ||
            true
        )"

        if [[ "$cmdline" == *"/lib/rover_backend/rover_backend"* ]] ||
           [[ "$cmdline" == *"rover_backend.main"* ]]; then
            BACKEND_PID="$pid"
            return 0
        fi
    done < <(
        pgrep -f "rover_backend" 2>/dev/null || true
    )

    echo "ABORT: unable to locate the running rover_backend process"
    return 1
}


read_process_environment_value() {
    local pid="$1"
    local variable_name="$2"

    python3 - "$pid" "$variable_name" <<'PY'
from pathlib import Path
import sys

pid = sys.argv[1]
variable_name = sys.argv[2]
prefix = (variable_name + "=").encode("utf-8")

try:
    entries = Path(f"/proc/{pid}/environ").read_bytes().split(b"\0")
except OSError:
    raise SystemExit(0)

for entry in entries:
    if entry.startswith(prefix):
        value = entry[len(prefix):].decode(
            "utf-8",
            errors="surrogateescape",
        )
        print(value, end="")
        break
PY
}


load_backend_credentials() {
    # Explicit field-test override has highest priority.
    if [[ -n "${DYX_TEST_USERNAME:-}" ]] ||
       [[ -n "${DYX_TEST_PASSWORD:-}" ]]; then

        if [[ -z "${DYX_TEST_USERNAME:-}" ]] ||
           [[ -z "${DYX_TEST_PASSWORD:-}" ]]; then
            echo "ABORT: set both DYX_TEST_USERNAME and DYX_TEST_PASSWORD"
            return 1
        fi

        DYX_STATIC_USERNAME="$DYX_TEST_USERNAME"
        DYX_STATIC_PASSWORD="$DYX_TEST_PASSWORD"
        CREDENTIAL_SOURCE="DYX_TEST_USERNAME/DYX_TEST_PASSWORD override"
        return 0
    fi

    find_running_backend_pid

    # The ROS launch starts rover_backend directly. It inherits the launch
    # terminal environment; it does not automatically load the systemd
    # EnvironmentFile. Read the actual process environment, which is the
    # source of truth for this running process.
    DYX_STATIC_USERNAME="$(
        read_process_environment_value \
            "$BACKEND_PID" \
            DYX_STATIC_USERNAME
    )"

    DYX_STATIC_PASSWORD="$(
        read_process_environment_value \
            "$BACKEND_PID" \
            DYX_STATIC_PASSWORD
    )"

    # These are the validated defaults in rover_backend/config.py. A missing
    # process variable means the running backend used the corresponding
    # default.
    if [[ -z "$DYX_STATIC_USERNAME" ]]; then
        DYX_STATIC_USERNAME="admin"
    fi

    if [[ -z "$DYX_STATIC_PASSWORD" ]]; then
        DYX_STATIC_PASSWORD="dyx@2026"
    fi

    CREDENTIAL_SOURCE=(
        "running rover_backend process PID "
        "$BACKEND_PID environment plus backend defaults"
    )
}


api_login() {
    local response_file=""
    local http_status=""
    local response=""
    local detail=""

    response_file="$(mktemp)"

    http_status="$(
        curl \
            --silent \
            --show-error \
            --connect-timeout 3 \
            --max-time 15 \
            --output "$response_file" \
            --write-out "%{http_code}" \
            -H "Content-Type: application/json" \
            -X POST \
            "$BACKEND_BASE_URL/api/auth/login" \
            --data "$(
                DYX_USER="$DYX_STATIC_USERNAME" \
                DYX_PASS="$DYX_STATIC_PASSWORD" \
                python3 - <<'PY'
import json
import os

print(json.dumps({
    "username": os.environ["DYX_USER"],
    "password": os.environ["DYX_PASS"],
}))
PY
            )" ||
        true
    )"

    response="$(cat "$response_file" 2>/dev/null || true)"
    rm -f "$response_file"

    if [[ "$http_status" != "200" ]]; then
        detail="$(
            printf '%s' "$response" |
            python3 -c '
import json
import sys

try:
    payload = json.load(sys.stdin)
except Exception:
    print("")
else:
    print(payload.get("detail", ""))
' 2>/dev/null || true
        )"

        echo "ABORT: backend login failed"
        echo "HTTP status: ${http_status:-no response}"
        echo "Credential source: $CREDENTIAL_SOURCE"

        if [[ -n "$detail" ]]; then
            echo "Backend detail: $detail"
        fi

        if [[ "$http_status" == "401" ]]; then
            echo
            echo "The running backend rejected the username/password."
            echo "Do not keep retrying the old script: five failed attempts"
            echo "cause a temporary 15-minute authentication lockout."
            echo
            echo "Either restart rover.launch.py from a terminal that exports"
            echo "the intended DYX_STATIC_USERNAME/DYX_STATIC_PASSWORD, or run:"
            echo
            echo "  DYX_TEST_USERNAME='<actual username>' \\"
            echo "  DYX_TEST_PASSWORD='<actual password>' \\"
            echo "  bash ~/rover_ws/scripts/start_c_p1_p2_p3_p4_square_test.sh"
        elif [[ "$http_status" == "429" ]]; then
            echo
            echo "Authentication is temporarily locked after repeated failures."
            echo "Wait for the Retry-After period before testing again."
        fi

        return 1
    fi

    TOKEN="$(printf '%s' "$response" | json_get token)"

    if [[ -z "$TOKEN" ]]; then
        echo "ABORT: backend login succeeded but returned no token"
        return 1
    fi

    # Confirm the token before any mission or safety operation.
    local session_file=""
    local session_status=""

    session_file="$(mktemp)"

    session_status="$(
        curl \
            --silent \
            --show-error \
            --connect-timeout 3 \
            --max-time 10 \
            --output "$session_file" \
            --write-out "%{http_code}" \
            -H "Authorization: Bearer $TOKEN" \
            "$BACKEND_BASE_URL/api/auth/session" ||
        true
    )"

    rm -f "$session_file"

    if [[ "$session_status" != "200" ]]; then
        TOKEN=""
        echo "ABORT: login token failed session validation"
        echo "HTTP status: ${session_status:-no response}"
        return 1
    fi

    echo "Backend authentication successful."
    echo "Credential source: $CREDENTIAL_SOURCE"
}


api_request() {
    local method="$1"
    local path="$2"

    if [[ -z "$TOKEN" ]]; then
        echo "ABORT: backend token is unavailable" >&2
        return 1
    fi

    curl \
        --silent \
        --show-error \
        --fail-with-body \
        --connect-timeout 3 \
        --max-time 45 \
        -H "Authorization: Bearer $TOKEN" \
        -X "$method" \
        "$BACKEND_BASE_URL$path"
}


best_effort_api_post() {
    local path="$1"

    if [[ -n "$TOKEN" ]]; then
        api_request POST "$path" >/dev/null 2>&1 || true
    fi
}


fallback_px4_safe() {
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
}


stop_supervisor() {
    if [[ -n "$SUPERVISOR_PID" ]]; then
        kill -TERM "$SUPERVISOR_PID" 2>/dev/null || true
        wait "$SUPERVISOR_PID" 2>/dev/null || true
        SUPERVISOR_PID=""
    elif [[ -f "$SUPERVISOR_PID_FILE" ]]; then
        local pid
        pid="$(cat "$SUPERVISOR_PID_FILE" 2>/dev/null || true)"
        if [[ -n "$pid" ]]; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
    fi
}


restore_full_prepared_mission() {
    echo
    echo "===== RESTORE NORMAL FULL PREPARED MISSION ====="

    if [[ -z "$TOKEN" ]]; then
        echo "Skipped: backend token unavailable"
        return
    fi

    if [[ "$TEST_PATH_STARTED" -ne 1 ]]; then
        echo "Skipped: temporary C->P1->P2->P3->P4 path was never started."
        echo "The normal mission path was not replaced."
        return
    fi

    local response

    if response="$(api_request POST /api/mission/prepare 2>&1)"; then
        printf '%s\n' "$response" | python3 -m json.tool || true
        echo "Normal mission.csv trajectory restored."
    else
        echo "WARNING: unable to restore full prepared trajectory:"
        echo "$response"
    fi
}


cleanup() {
    local exit_code=$?

    if [[ "$CLEANUP_STARTED" -eq 1 ]]; then
        return
    fi
    CLEANUP_STARTED=1

    set +e

    echo
    echo "===== PRODUCTION SAFETY CLEANUP ====="

    best_effort_api_post /api/estop
    best_effort_api_post /api/mission/stop

    stop_supervisor
    fallback_px4_safe

    sleep 2

    echo
    echo "===== FINAL PX4 STATE ====="
    timeout 5s ros2 topic echo \
        /mavros/state \
        --once 2>/dev/null || true

    restore_full_prepared_mission

    rm -f "$SUPERVISOR_PID_FILE"

    if [[ "$exit_code" -ne 0 && "$FINAL_RESULT" == "ABORTED" ]]; then
        FINAL_RESULT="SCRIPT_ERROR"
    fi

    echo
    echo "===== FINAL TEST RESULT: $FINAL_RESULT ====="

    return "$exit_code"
}


trap cleanup EXIT INT TERM


echo "===== C -> P1 -> P2 -> P3 -> P4 SQUARE PRODUCTION TEST ====="
echo "Physical E-stop must remain ready."
echo
echo "Contract:"
echo "  Every non-zero translational movement = exactly 0.40 m/s"
echo "  No acceleration ramp, deceleration profile, creep speed or reverse throttle"
echo "  Smooth xtrack alignment along straight 50 mm interpolated segments"
echo "  Direct predictive stop command: 0.40 m/s -> 0.00 m/s"
echo "  C -> P1 straight generated spacing <= 0.05 m"
echo "  P1 circular waypoint radius <= 30 mm"
echo "  P1 continuous stationary hold >= 3.00 s"
echo "  P1 -> P2 straight generated spacing <= 0.05 m"
echo "  P2 circular waypoint radius <= 30 mm"
echo "  P2 continuous stationary hold >= 3.00 s"
echo "  P2 -> P3 straight generated spacing <= 0.05 m"
echo "  P3 circular waypoint radius <= 30 mm"
echo "  P3 continuous stationary hold >= 3.00 s"
echo "  P3 -> P4 straight generated spacing <= 0.05 m"
echo "  P4 circular waypoint radius <= 30 mm"
echo "  P4 continuous stationary hold >= 3.00 s"
echo "  Circular rule: sqrt(xtrack^2 + along^2) <= 30 mm"
echo "  Master-antenna heading check <= ${HEADING_TOLERANCE_DEG} deg"


echo
echo "===== REMOVE OLD TEST GATE KEEPER ====="

pkill -TERM -f "$HOME/rover_ws/scripts/rover_gate_keeper.py" \
    2>/dev/null || true
pkill -TERM -f "/tmp/hold_rover_gate.py" \
    2>/dev/null || true
rm -f "$HOME/rover_ws/.rover_gate_state"

sleep 1

if pgrep -f "rover_gate_keeper.py" >/dev/null; then
    echo "ABORT: old rover_gate_keeper.py is still running"
    exit 1
fi


echo
echo "===== PRE-FLIGHT FILE CHECK ====="

for required_file in \
    "$SUPERVISOR" \
    "$HOME/rover_ws/missions/mission.csv" \
    "$LOG_FILE"
do
    if [[ ! -f "$required_file" ]]; then
        echo "ABORT: missing file $required_file"
        exit 1
    fi
done

if [[ ! -x "$SUPERVISOR" ]]; then
    chmod +x "$SUPERVISOR"
fi

python3 -m py_compile "$SUPERVISOR"


echo
echo "===== PRE-FLIGHT NODE CHECK ====="

NODE_LIST="$(ros2 node list 2>/dev/null || true)"

for required_node in \
    /rover_backend \
    /cmd_vel_bridge \
    /trajectory_generator \
    /mission_manager \
    /rpp_controller
do
    if ! grep -qx "$required_node" <<<"$NODE_LIST"; then
        echo "ABORT: missing node $required_node"
        exit 1
    fi
done

LAUNCH_COUNT="$(
    pgrep -fc '[r]os2 launch rover_bringup rover.launch.py' || true
)"
echo "Rover launch count: $LAUNCH_COUNT"

if [[ "$LAUNCH_COUNT" -ne 1 ]]; then
    echo "ABORT: expected exactly one production rover launch"
    exit 1
fi


echo
echo "===== BACKEND LOGIN ====="

load_backend_credentials
api_login

PING_RESPONSE="$(
    curl \
        --silent \
        --show-error \
        --fail-with-body \
        --connect-timeout 3 \
        --max-time 10 \
        "$BACKEND_BASE_URL/api/ping"
)"
printf '%s\n' "$PING_RESPONSE" | python3 -m json.tool


echo
echo "===== FORCE SAFE INITIAL STATE ====="

best_effort_api_post /api/estop
best_effort_api_post /api/mission/stop
fallback_px4_safe
sleep 2

INITIAL_STATE="$(
    timeout 5s ros2 topic echo \
        /mavros/state \
        --once 2>/dev/null || true
)"
echo "$INITIAL_STATE"

if ! grep -q "armed: false" <<<"$INITIAL_STATE"; then
    echo "ABORT: PX4 is still armed"
    exit 1
fi

if ! grep -q "mode: MANUAL" <<<"$INITIAL_STATE"; then
    echo "ABORT: PX4 is not in MANUAL"
    exit 1
fi


echo
echo "===== FAST PRODUCTION PARAMETER CHECK ====="
echo "One ROS 2 process, four batched parameter-service requests."

# The former script started a new `ros2 param get` process for every
# parameter. ROS discovery startup made this section slow. This check uses
# one rclpy process and one batched GetParameters request per node.
#
# Only the field-test-critical contract is checked here:
#   - 50 mm trajectory spacing
#   - circular 30 mm waypoint radius and stationary hold
#   - 0.40 m/s normal travel with tuned marking-only low-speed capture
#   - PX4 native 45deg pivot / 12deg drive transition
#   - current adaptive crossing-brake xtrack profile
#
# Safety checks, RTK/GPS checks, mission restore and cleanup remain unchanged.

timeout 12s python3 - <<'PY'
import math
import sys
from typing import Dict, Iterable, Tuple

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters


EXPECTED: Dict[str, Dict[str, object]] = {
    "/trajectory_generator": {
        "interpolation_spacing_m": 0.05,
    },
    "/mission_manager": {
        "marking_tolerance_m": 0.03,
        "marking_error_mode": "RADIAL_2D",
        "marking_hold_sec": 3.0,
        "stationary_speed_tolerance_mps": 0.01,
    },
    "/cmd_vel_bridge": {
        "maximum_speed_mps": 0.40,
    },
    "/rpp_controller": {
        # Every non-zero translational movement command must be 0.40 m/s.
        "cruise_speed_mps": 0.40,
        "minimum_speed_mps": 0.40,
        "segment_alignment_speed_mps": 0.40,
        "segment_alignment_recovery_speed_mps": 0.40,
        "xtrack_priority_speed_mps": 0.40,
        "decel_profile_speed_1_mps": 0.40,
        "decel_profile_speed_2_mps": 0.40,
        "decel_profile_speed_3_mps": 0.40,
        "marking_terminal_max_speed_mps": 0.40,
        "marking_final_creep_speed_mps": 0.40,
        "terminal_close_recovery_speed_mps": 0.40,

        # Current smooth 50 mm interpolation line-tracking setup.
        "pivot_enter_angle_deg": 45.0,
        "pivot_exit_angle_deg": 12.0,
        "waypoint_tolerance_m": 0.03,
        "line_tracking_lookahead_m": 0.55,
        "path_correction_limit_deg": 18.0,
        "xtrack_priority_lookahead_m": 0.55,
        "xtrack_priority_correction_limit_deg": 22.0,
        "xtrack_prediction_time_sec": 0.25,
        "xtrack_correction_slew_rate_degps": 30.0,
        "terminal_goal_intercept_distance_m": 1.20,
        "terminal_xtrack_lookahead_m": 0.50,
        "terminal_xtrack_correction_limit_deg": 22.0,
        "terminal_xtrack_prediction_time_sec": 0.25,
        "terminal_xtrack_neutral_crossing_band_m": 0.004,
        "terminal_xtrack_correction_slew_rate_degps": 35.0,
        "terminal_xtrack_unwind_slew_rate_degps": 50.0,
        "terminal_xtrack_away_lookahead_m": 0.30,
        "terminal_xtrack_away_correction_limit_deg": 28.0,
        "terminal_xtrack_away_rate_threshold_mps": 0.008,
        "terminal_xtrack_crossing_prediction_time_sec": 0.55,
        "terminal_xtrack_crossing_lookahead_m": 0.40,
        "terminal_xtrack_crossing_correction_limit_deg": 24.0,
        "terminal_xtrack_crossing_rate_threshold_mps": 0.010,
        "terminal_xtrack_crossing_predicted_threshold_m": 0.004,

        # Direct 0.40 -> 0.00 marking stop. No intermediate speed or reverse.
        "marking_stop_latency_sec": 0.24,
        "marking_stop_min_buffer_m": 0.060,
        "marking_stop_max_buffer_m": 0.100,
        "marking_stop_xtrack_limit_m": 0.020,
    },
}


def numeric_value(parameter_value) -> float:
    if parameter_value.type == ParameterType.PARAMETER_DOUBLE:
        return float(parameter_value.double_value)
    if parameter_value.type == ParameterType.PARAMETER_INTEGER:
        return float(parameter_value.integer_value)
    raise RuntimeError(
        f"parameter is not numeric; ROS type={parameter_value.type}"
    )


def string_value(parameter_value) -> str:
    if parameter_value.type == ParameterType.PARAMETER_STRING:
        return str(parameter_value.string_value)
    raise RuntimeError(
        f"parameter is not a string; ROS type={parameter_value.type}"
    )


rclpy.init()
node = rclpy.create_node("dyx_fast_parameter_verifier")

try:
    failures = []

    for target_node, expected_parameters in EXPECTED.items():
        service_name = target_node.rstrip("/") + "/get_parameters"
        client = node.create_client(GetParameters, service_name)

        if not client.wait_for_service(timeout_sec=2.0):
            failures.append(
                f"{target_node}: parameter service unavailable"
            )
            continue

        parameter_names = list(expected_parameters)
        request = GetParameters.Request()
        request.names = parameter_names

        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=2.0,
        )

        if not future.done():
            failures.append(
                f"{target_node}: parameter request timed out"
            )
            continue

        exception = future.exception()
        if exception is not None:
            failures.append(
                f"{target_node}: parameter request failed: {exception}"
            )
            continue

        response = future.result()
        if response is None:
            failures.append(
                f"{target_node}: empty parameter response"
            )
            continue

        if len(response.values) != len(parameter_names):
            failures.append(
                f"{target_node}: response count mismatch"
            )
            continue

        for name, value_message in zip(
            parameter_names,
            response.values,
        ):
            expected = expected_parameters[name]

            try:
                if isinstance(expected, str):
                    actual = string_value(value_message)
                    matches = actual == expected
                else:
                    actual = numeric_value(value_message)
                    matches = math.isclose(
                        actual,
                        float(expected),
                        rel_tol=0.0,
                        abs_tol=1.0e-9,
                    )
            except RuntimeError as exc:
                failures.append(
                    f"{target_node}.{name}: {exc}"
                )
                continue

            if not matches:
                failures.append(
                    f"{target_node}.{name}: "
                    f"actual={actual}, expected={expected}"
                )

    if failures:
        print("ABORT: fast production parameter check failed")
        for failure in failures:
            print(f"  - {failure}")
        raise SystemExit(1)

    checked_count = sum(
        len(parameters)
        for parameters in EXPECTED.values()
    )
    print(
        "FAST PARAMETER CHECK PASSED | "
        f"{checked_count} critical values | "
        "constant 0.40m/s movement | smooth xtrack alignment | "
        "50mm interpolation | circular 30mm | direct zero stop"
    )
finally:
    node.destroy_node()
    rclpy.shutdown()
PY

echo
echo "===== RTK AND GPS CHECK ====="

RTK_HEALTH="$(
    timeout 5s ros2 topic echo \
        /rtk_correction_bridge/healthy \
        --once 2>/dev/null || true
)"
echo "$RTK_HEALTH"

if ! grep -q "data: true" <<<"$RTK_HEALTH"; then
    echo "ABORT: RTK correction bridge is not healthy"
    exit 1
fi

GPS_FIX="$(
    timeout 5s ros2 topic echo \
        /mavros/gpsstatus/gps1/raw \
        --once \
        --field fix_type 2>/dev/null || true
)"
echo "GPS fix type: $GPS_FIX"

if ! grep -q "^6$" <<<"$GPS_FIX"; then
    echo "ABORT: GPS fix_type is not 6"
    exit 1
fi


echo
echo "===== PREPARE PRODUCTION SOURCE MISSION ====="

PREPARE_RESPONSE="$(
    api_request POST /api/mission/prepare
)"
printf '%s\n' "$PREPARE_RESPONSE" | python3 -m json.tool

SOURCE_TOTAL="$(
    printf '%s' "$PREPARE_RESPONSE" |
    json_get mission.total_points
)"

if [[ "$SOURCE_TOTAL" != "4" ]]; then
    echo "ABORT: prepared mission must contain exactly four marking points"
    exit 1
fi


echo
echo "===== START TEST-PATH SUPERVISOR ====="

rm -rf "$RUNTIME_DIR"
mkdir -p "$RUNTIME_DIR"

python3 "$SUPERVISOR" \
    --spacing-m 0.05 \
    --waypoint-radius-m 0.03 \
    --hold-required-sec 3.0 \
    --stationary-speed-mps 0.01 \
    --missed-point-abort-m 0.10 \
    --heading-tolerance-deg "$HEADING_TOLERANCE_DEG" \
    --odom-timeout-sec 0.50 \
    --ready-file "$READY_FILE" \
    --result-file "$RESULT_FILE" \
    --event-file "$EVENT_FILE" \
    >"$SUPERVISOR_LOG" 2>&1 &

SUPERVISOR_PID=$!
TEST_PATH_STARTED=1
printf '%s\n' "$SUPERVISOR_PID" > "$SUPERVISOR_PID_FILE"


echo
echo "===== WAIT FOR TEMPORARY C->P1->P2->P3->P4 PATH ====="

for _ in $(seq 1 300); do
    if [[ -f "$RESULT_FILE" ]]; then
        break
    fi
    if [[ -f "$READY_FILE" ]]; then
        break
    fi
    if ! kill -0 "$SUPERVISOR_PID" 2>/dev/null; then
        echo "ABORT: test supervisor exited unexpectedly"
        cat "$SUPERVISOR_LOG" || true
        exit 1
    fi
    sleep 0.1
done

if [[ -f "$RESULT_FILE" && ! -f "$READY_FILE" ]]; then
    cat "$RESULT_FILE" | python3 -m json.tool
    echo "ABORT: supervisor could not build/load test path"
    exit 1
fi

if [[ ! -f "$READY_FILE" ]]; then
    echo "ABORT: timed out waiting for temporary test path"
    cat "$SUPERVISOR_LOG" || true
    exit 1
fi

cat "$READY_FILE" | python3 -m json.tool

MAX_SPACING="$(
    cat "$READY_FILE" |
    json_get maximum_generated_spacing_m
)"

python3 - "$MAX_SPACING" <<'PY'
import sys

spacing = float(sys.argv[1])
if spacing > 0.05 + 1.0e-9:
    raise SystemExit(
        f"Generated spacing is too large: {spacing}"
    )
PY


echo
echo "===== CONFIRM FOUR-POINT MISSION MANAGER STATE ====="

STATUS_RESPONSE="$(
    api_request GET /api/mission/status
)"
printf '%s\n' "$STATUS_RESPONSE" | python3 -m json.tool

TEST_TOTAL="$(
    printf '%s' "$STATUS_RESPONSE" |
    json_get mission.total_points
)"
TEST_READY="$(
    printf '%s' "$STATUS_RESPONSE" |
    json_get mission.ready
)"
TEST_STATE="$(
    printf '%s' "$STATUS_RESPONSE" |
    json_get mission.state
)"

if [[ "$TEST_TOTAL" != "4" ]]; then
    echo "ABORT: mission manager did not load exactly P1, P2, P3 and P4"
    exit 1
fi

if [[ "$TEST_READY" != "true" || "$TEST_STATE" != "READY" ]]; then
    echo "ABORT: mission manager is not READY with the test path"
    exit 1
fi


START_LINE=$(( $(wc -l < "$LOG_FILE") + 1 ))


echo
echo "===== START THROUGH PRODUCTION BACKEND ====="

START_RESPONSE="$(
    api_request POST /api/mission/start
)"
printf '%s\n' "$START_RESPONSE" | python3 -m json.tool


echo
echo "===== WAIT FOR ARMED OFFBOARD ====="

ARMED_OFFBOARD=0

for _ in $(seq 1 30); do
    STATE="$(
        timeout 4s ros2 topic echo \
            /mavros/state \
            --once 2>/dev/null || true
    )"

    if grep -q "armed: true" <<<"$STATE" &&
       grep -q "mode: OFFBOARD" <<<"$STATE"; then
        echo "$STATE"
        ARMED_OFFBOARD=1
        break
    fi

    sleep 0.5
done

if [[ "$ARMED_OFFBOARD" -ne 1 ]]; then
    echo "ABORT: PX4 did not reach armed OFFBOARD"
    exit 1
fi


echo
echo "===== TEST RUNNING ====="
echo "Do not move the rover or either GNSS antenna."
echo "Physical E-stop must remain ready."
echo "Live mm monitor (another terminal):"
echo "  ros2 topic echo /test/marking_error_mm --field data"

START_EPOCH="$(date +%s)"

while true; do
    sleep 0.2

    if [[ -f "$RESULT_FILE" ]]; then
        FINAL_RESULT="$(
            cat "$RESULT_FILE" |
            json_get result
        )"
        break
    fi

    NEW_LOG="$(
        sed -n "${START_LINE},\$p" "$LOG_FILE"
    )"

    if grep -qiE \
        "INVALID_MARKING_COMPLETION|MISSED_MARKING_POINT|SAFE HOLD|entered ERROR|MISSION ERROR" \
        <<<"$NEW_LOG"; then
        FINAL_RESULT="SAFE_HOLD_OR_ERROR"
        break
    fi

    NOW_EPOCH="$(date +%s)"
    ELAPSED=$((NOW_EPOCH - START_EPOCH))

    if (( ELAPSED >= TEST_TIMEOUT_SEC )); then
        FINAL_RESULT="TIMEOUT"
        break
    fi

    if (( ELAPSED > 0 && ELAPSED % 5 == 0 )); then
        if [[ "${LAST_PRINTED_ELAPSED:-}" != "$ELAPSED" ]]; then
            LAST_PRINTED_ELAPSED="$ELAPSED"

            echo
            echo "===== RUNNING: ${ELAPSED} SECONDS ====="

            sed -n "${START_LINE},\$p" "$LOG_FILE" |
            grep -E \
                "STRAIGHT SEGMENT GOAL|SEGMENT ALIGNMENT|SEGMENT ALIGNED|ABSOLUTE YAW PIVOT|50MM LOOKAHEAD|EXACT MARKING CONVERGENCE|MARKING CAPTURE|MARKING FINAL|Holding at marking|completed with radius|MISSION COMPLETED|ERROR" |
            tail -n 25 || true

            api_request GET /api/mission/status |
            python3 -c '
import json
import sys

payload = json.load(sys.stdin)["mission"]

print(
    "state={state} active={active} completed={completed}/4 "
    "marking_active={marking} hold={hold:.2f}/{required:.2f}".format(
        state=payload.get("state"),
        active=payload.get("active_point_id"),
        completed=payload.get("completed_points", 0),
        marking=payload.get("marking_active", False),
        hold=float(payload.get("hold_elapsed_sec", 0.0) or 0.0),
        required=float(payload.get("hold_required_sec", 3.0) or 3.0),
    )
)
' || true
        fi
    fi
done


echo
echo "===== TEST SUPERVISOR RESULT ====="

if [[ -f "$RESULT_FILE" ]]; then
    cat "$RESULT_FILE" | python3 -m json.tool
else
    echo "No supervisor result file."
    echo "Result detected from runtime: $FINAL_RESULT"
fi


echo
echo "===== RELEVANT ROVER LOG ====="

sed -n "${START_LINE},\$p" "$LOG_FILE" |
grep -E \
    "STRAIGHT SEGMENT GOAL|SEGMENT ALIGNMENT|SEGMENT ALIGNED|ABSOLUTE YAW PIVOT|50MM LOOKAHEAD|EXACT MARKING CONVERGENCE|MARKING CAPTURE|MARKING FINAL|Holding at marking|Marking P|completed with radius|MISSION COMPLETED|SAFE HOLD|ERROR" |
tail -n 400 || true


if [[ "$FINAL_RESULT" == "PASS" ]]; then
    exit 0
fi

exit 1