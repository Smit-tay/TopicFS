#!/usr/bin/env bash
# MIT License
# Copyright 2026 Jack Sidman Smith
#
# rom_test.sh — UArm Swift Pro range of motion test via TopicFS.
# Moves to positions in an outward spiral, records commanded vs actual
# position and success/failure. Outputs CSV for 3D visualisation.
#
# For missed points (arm didn't move), plot_x/y/z = cmd_x/y/z so the
# viewer shows where the point SHOULD have been, not where the arm sat.
#
# Usage: bash rom_test.sh [MOUNT_POINT] [OUTPUT_CSV]
# Default mount point: /home/jack/fuse_mount
# Default output:      rom_results.csv

set -uo pipefail

MOUNT="${1:-/home/jack/fuse_mount}"
OUTPUT="${2:-rom_results.csv}"
SWIFTPRO="${MOUNT}/swiftpro"

# --- Parameters ---
MOVE_SPEED=3000         # mm/min
RETURN_SPEED=5000       # mm/min — return to centre between moves
SERVICE_TIMEOUT=30      # seconds
MOVE_SETTLE=0.5         # seconds — wait after move before reading position
POSITION_TOLERANCE=5.0  # mm — delta threshold to declare a move "reached"

# Centre of spiral — known reachable safe position
CENTRE_X=180.0
CENTRE_Y=0.0
CENTRE_Z=80.0

# --- Colours ---
RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[0;33m'
CYN='\033[0;36m'
BLD='\033[1m'
RST='\033[0m'

# ============================================================
# Spiral point generation
# Archimedean spiral in XY plane, Z varies sinusoidally.
# 20 points, radius grows from 30mm to 150mm.
# ============================================================

POINTS=()
N=20
for i in $(seq 0 $(( N - 1 ))); do
    angle=$(awk "BEGIN { printf \"%.6f\", ${i} * 2 * 3.14159265 * 2.5 / ${N} }")
    radius=$(awk "BEGIN { printf \"%.1f\", 30 + (150 - 30) * ${i} / (${N} - 1) }")
    z=$(awk "BEGIN { printf \"%.1f\", 80 + 50 * sin(${i} * 3.14159265 / (${N} / 2)) }")
    px=$(awk "BEGIN { printf \"%.1f\", ${CENTRE_X} + ${radius} * cos(${angle}) }")
    py=$(awk "BEGIN { printf \"%.1f\", ${CENTRE_Y} + ${radius} * sin(${angle}) }")
    POINTS+=("${px} ${py} ${z}")
done

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

float_delta() {
    awk "BEGIN { d=$1 - $2; print (d < 0 ? -d : d) }"
}

float_le() {
    awk "BEGIN { exit ($1 <= $2 ? 0 : 1) }"
}

# ============================================================
# Pre-flight
# ============================================================

if [[ ! -d "${SWIFTPRO}" ]]; then
    echo -e "${RED}SwiftPro not found at ${SWIFTPRO} — is SwiftRos2 running?${RST}"
    exit 1
fi

echo ""
echo -e "${BLD}${CYN}UArm Swift Pro — Range of Motion Test (proof of concept)${RST}"
echo -e "Points: ${N}   Speed: ${MOVE_SPEED}mm/min   Output: ${OUTPUT}"
echo ""

# CSV header — plot_x/y/z is what the viewer should use for positioning
echo "point,cmd_x,cmd_y,cmd_z,actual_x,actual_y,actual_z,plot_x,plot_y,plot_z,dx,dy,dz,reached,service_success,error_code" > "${OUTPUT}"

echo -e "Moving to centre (${CENTRE_X},${CENTRE_Y},${CENTRE_Z})..."
call_move "${CENTRE_X}" "${CENTRE_Y}" "${CENTRE_Z}" "${RETURN_SPEED}" > /dev/null
sleep "${MOVE_SETTLE}"

echo ""
echo -e "${BLD}$(printf '%-4s  %-24s  %-24s  %-8s  %s' \
    'Pt' 'Commanded' 'Actual' 'Reached' 'Note')${RST}"
echo "────────────────────────────────────────────────────────────────────"

# ============================================================
# Main sweep
# ============================================================

point=1
for pt in "${POINTS[@]}"; do
    read -r tx ty tz <<< "${pt}"

    rsp=$(call_move "${tx}" "${ty}" "${tz}" "${MOVE_SPEED}")
    sleep "${MOVE_SETTLE}"

    service_success="false"
    error_code="-99"
    if [[ -n "${rsp}" ]]; then
        service_success=$(json_field "${rsp}" "success")
        error_code=$(json_field "${rsp}" "error_code")
    fi

    pos=$(cat "${SWIFTPRO}/position/latest" 2>/dev/null)
    rx=$(json_field "${pos}" "x"); rx="${rx:-0}"
    ry=$(json_field "${pos}" "y"); ry="${ry:-0}"
    rz=$(json_field "${pos}" "z"); rz="${rz:-0}"

    dx=$(float_delta "${rx}" "${tx}")
    dy=$(float_delta "${ry}" "${ty}")
    dz=$(float_delta "${rz}" "${tz}")

    reached=true
    float_le "${dx}" "${POSITION_TOLERANCE}" || reached=false
    float_le "${dy}" "${POSITION_TOLERANCE}" || reached=false
    float_le "${dz}" "${POSITION_TOLERANCE}" || reached=false

    # plot_x/y/z: use commanded position for missed points so the viewer
    # shows where the point should have been, not where the arm sat idle.
    if [[ "${reached}" == "true" ]]; then
        plot_x="${rx}"; plot_y="${ry}"; plot_z="${rz}"
    else
        plot_x="${tx}"; plot_y="${ty}"; plot_z="${tz}"
    fi

    note=""
    [[ "${service_success}" != "true" ]] && note="service failed"
    [[ "${reached}" == "false" ]] && note="${note:+${note}, }out of reach"

    color="${GRN}"
    [[ "${reached}" == "false" ]] && color="${YLW}"
    [[ "${service_success}" != "true" && "${reached}" == "false" ]] && color="${RED}"

    echo -e "${color}$(printf '%-4s  %-24s  %-24s  %-8s  %s' \
        "${point}" \
        "(${tx},${ty},${tz})" \
        "(${rx},${ry},${rz})" \
        "${reached}" \
        "${note}")${RST}"

    echo "${point},${tx},${ty},${tz},${rx},${ry},${rz},${plot_x},${plot_y},${plot_z},${dx},${dy},${dz},${reached},${service_success},${error_code}" >> "${OUTPUT}"

    call_move "${CENTRE_X}" "${CENTRE_Y}" "${CENTRE_Z}" "${RETURN_SPEED}" > /dev/null
    sleep "${MOVE_SETTLE}"

    (( point++ ))
done

echo "────────────────────────────────────────────────────────────────────"
echo ""
echo -e "Results written to: ${BLD}${OUTPUT}${RST}"

reached_count=$(tail -n +2 "${OUTPUT}" | cut -d',' -f14 | grep -c "^true$" || true)
echo -e "Reached: ${GRN}${reached_count}${RST} / ${N} points within ${POSITION_TOLERANCE}mm tolerance"
echo ""
