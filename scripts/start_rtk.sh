#!/usr/bin/env bash
set -euo pipefail

# Compatibility helper only.
#
# Production RTK ownership belongs exclusively to rover_backend.
# Never launch an NTRIP client, ROS RTK node, or RTCM publisher here.

BACKEND_URL="${DYX_BACKEND_URL:-http://127.0.0.1:5001}"
ROVER_TOKEN="${DYX_ROVER_TOKEN:-}"

if [[ -z "${ROVER_TOKEN}" ]]; then
    echo "DYX_ROVER_TOKEN is required." >&2
    echo "Authenticate with rover_backend and export the returned token." >&2
    exit 2
fi

RTK_START_URL="${BACKEND_URL%/}/api/rtk/start"

# Feed authentication through curl config stdin so the bearer value is not
# placed directly into curl's process argv.
curl --config - <<EOF
url = "${RTK_START_URL}"
request = "POST"
fail-with-body
silent
show-error
header = "X-Rover-Token: ${ROVER_TOKEN}"
EOF

printf '\n'
