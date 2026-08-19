#!/usr/bin/env bash

say_step()
{
    local duration="$1"
    local message="$2"

    echo
    echo "=================================================="
    echo "$message"
    echo "=================================================="

    for ((seconds=duration; seconds>=1; seconds--)); do
        printf "\rRemaining: %2d seconds " "$seconds"
        sleep 1
    done

    printf "\n"
}

echo
echo "PX4 ACRO YAW FEED-FORWARD TEST"
echo
echo "Before continuing:"
echo "  1. Rover is in a large clear area."
echo "  2. Physical E-stop is accessible."
echo "  3. PX4 is in ACRO mode."
echo "  4. Forward/reverse throttle remains centred."
echo "  5. Do not arm until instructed."
echo
read -r -p "Press ENTER when ready..."

say_step 5 "ARM PX4 — KEEP BOTH STICKS CENTRED"

say_step 3 "CENTRE YAW STICK"
say_step 4 "HOLD RIGHT YAW AT 25 PERCENT"
say_step 3 "CENTRE YAW STICK"
say_step 4 "HOLD LEFT YAW AT 25 PERCENT"

say_step 3 "CENTRE YAW STICK"
say_step 4 "HOLD RIGHT YAW AT 50 PERCENT"
say_step 3 "CENTRE YAW STICK"
say_step 4 "HOLD LEFT YAW AT 50 PERCENT"

say_step 3 "CENTRE YAW STICK"
say_step 4 "HOLD RIGHT YAW AT 75 PERCENT"
say_step 3 "CENTRE YAW STICK"
say_step 4 "HOLD LEFT YAW AT 75 PERCENT"

say_step 5 "CENTRE BOTH STICKS"

echo
echo "DISARM PX4 NOW."
echo "Test completed."
