#!/usr/bin/env bash
# MIT License
# Copyright 2026 Jack Sidman Smith
#
# speed_test.sh — UArm Swift Pro speed calibration via TopicFS.
# Moves the arm a fixed distance at increasing speeds, times each move,
# and compares actual vs expected duration to verify mm/min units.
#
# Long diagonal distance (~156mm) used to minimise accel/decel distortion.
# Return move always at RETURN_SPEED (max) to minimise iteration time.
#
# Usage: bash speed_test.sh [MOUNT_POINT]
# Default mount point: /home/jack/fuse_mount

set -uo pipefail

MOUNT="${1:-/home/jack/fuse_mount}"
SWIFTPRO="${MOUNT}/swiftpro"

# --- Test parameters ---
SPEED_MIN=500           # mm/min — starting speed
SPEED_MAX=20000         # mm/min — ending speed
SPEED_STEP=500          # mm/min — increment
RETURN_SPEED=20000      # mm/min — fixed return speed (max — return is not under test)
RETURN_SETTLE=2.0       # seconds — settle after return before next test move

SERVICE_TIMEOUT=120     # seconds — large enough for slowest move

# Long diagonal move ~156mm
# sqrt((210-150)^2 + (60-(-60))^2 + (130-50)^2) = sqrt(3600+14400+6400) = 156mm
POS_A_X=150.0;  POS_A_Y=-60.0;  POS_A_Z=50.0
POS_B_X=210.0;  POS_B_Y=60.0;   POS_B_Z=130.0
DISTANCE=156.0          # mm — Euclidean A->B (verified below)

# --- Colours ---
RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[0;33m'
CYN='\033[0;36m'
BLD='\033[1m'
RST='\033[0m'

# ============================================================
# Helpers
# ============================================================

call_move() {
    local x="$1" y="$2" z="$3" speed="$4"
    local cmd="${SWIFTPRO}/move_to/command"
    local rsp="${SWIFTPRO}/move_to/response"

    echo "{\"x\": ${x}, \"y\": ${y}, \"z\": ${z}, \"speed\": ${speed}, \"wait\": true}" > "${cmd}"

    local deadline=$(( $(date +%s) + SERVICE_TIMEOUT ))
    while [[ $(date +%s) -lt ${deadline} ]]; do
        local content
        content=$(cat "${rsp}" 2>/dev/null)
        if [[ -n "${content}" && "${content}" != *"no response yet"* ]]; then
            echo "${content}"
            return 0
        fi
        sleep 0.1
    done
    echo ""
    return 1
}

json_field() {
    echo "$1" | grep -oP "\"$2\"\\s*:\\s*\\K[^,}]+" | tr -d '"' | xargs
}

# ============================================================
# Pre-flight
# ============================================================

if [[ ! -d "${SWIFTPRO}" ]]; then
    echo -e "${RED}SwiftPro not found at ${SWIFTPRO} — is SwiftRos2 running?${RST}"
    exit 1
fi

# Verify DISTANCE constant matches positions (parenthesise negatives for awk)
computed=$(awk "BEGIN {
    dx=(${POS_B_X})-(${POS_A_X})
    dy=(${POS_B_Y})-(${POS_A_Y})
    dz=(${POS_B_Z})-(${POS_A_Z})
    printf \"%.1f\", sqrt(dx*dx + dy*dy + dz*dz)
}")

echo ""
echo -e "${BLD}${CYN}UArm Swift Pro — Speed Calibration Test${RST}"
echo -e "A=(${POS_A_X},${POS_A_Y},${POS_A_Z})  B=(${POS_B_X},${POS_B_Y},${POS_B_Z})"
echo -e "Distance: ${computed}mm   Range: ${SPEED_MIN}–${SPEED_MAX} mm/min   Step: ${SPEED_STEP} mm/min"
echo -e "Return speed: ${RETURN_SPEED} mm/min (max — not under test)"
echo ""
echo -e "${BLD}$(printf '%-12s  %-12s  %-12s  %-8s  %-8s  %s' \
    'Speed' 'Expected(s)' 'Actual(s)' 'Ratio' 'Success' 'Note')${RST}"
echo "──────────────────────────────────────────────────────────────────────────"

# Move to start position before sweep begins
call_move "${POS_A_X}" "${POS_A_Y}" "${POS_A_Z}" ${RETURN_SPEED} > /dev/null
sleep ${RETURN_SETTLE}

# ============================================================
# Speed sweep
# ============================================================

prev_actual="0"
speed=${SPEED_MIN}
while [[ ${speed} -le ${SPEED_MAX} ]]; do

    # Expected duration: distance / (speed / 60)
    expected=$(awk "BEGIN { printf \"%.2f\", ${DISTANCE} * 60 / ${speed} }")

    # Time the A -> B move
    t_start=$(date +%s%3N)
    rsp=$(call_move "${POS_B_X}" "${POS_B_Y}" "${POS_B_Z}" "${speed}")
    t_end=$(date +%s%3N)

    actual_ms=$(( t_end - t_start ))
    actual=$(awk "BEGIN { printf \"%.2f\", ${actual_ms} / 1000 }")
    success=$(json_field "${rsp}" "success")

    # Ratio actual/expected
    ratio=$(awk "BEGIN { printf \"%.2f\", ${actual} / ${expected} }")

    # Plateau: actual time barely decreased from previous iteration
    note=""
    plateau=$(awk "BEGIN { exit (${prev_actual} > 0 && (${prev_actual} - ${actual}) < 0.1 ? 0 : 1) }" \
        && echo "yes" || echo "no")
    if [[ "${plateau}" == "yes" ]]; then
        note="⚠ plateau — likely at firmware cap"
    elif awk "BEGIN { exit (${ratio} > 1.5 ? 0 : 1) }"; then
        note="accel/decel dominated"
    fi
    [[ "${success}" != "true" ]] && note="✗ service failed"

    color="${GRN}"
    awk "BEGIN { exit (${ratio} > 1.5 ? 0 : 1) }" && color="${YLW}"
    [[ "${plateau}" == "yes" ]] && color="${YLW}"
    [[ "${success}" != "true" ]] && color="${RED}"

    echo -e "${color}$(printf '%-12s  %-12s  %-12s  %-8s  %-8s  %s' \
        "${speed}mm/min" "${expected}s" "${actual}s" "${ratio}x" "${success}" "${note}")${RST}"

    prev_actual="${actual}"

    # Return to A at maximum speed — not under test
    call_move "${POS_A_X}" "${POS_A_Y}" "${POS_A_Z}" ${RETURN_SPEED} > /dev/null
    sleep ${RETURN_SETTLE}

    speed=$(( speed + SPEED_STEP ))
done

echo "──────────────────────────────────────────────────────────────────────────"
echo ""
echo "Ratio 1.0  = commanded speed achieved"
echo "Ratio >1.0 = arm slower than commanded (accel/decel ramp or firmware cap)"
echo "Plateau    = actual time not decreasing — firmware speed cap reached"
echo ""
