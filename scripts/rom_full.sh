#!/usr/bin/env bash
# MIT License
# Copyright 2026 Jack Sidman Smith
#
# rom_full.sh — UArm Swift Pro full workspace envelope sweep via TopicFS.
#
# Systematic spherical-coordinate grid sweep using move_arm action.
# Sweeps azimuth slices (-90° to +90°, 5° steps). Within each slice,
# sweeps Z heights then radii. Adjacent points are always geometrically
# close — the arm never makes a large uncontrolled jump.
#
# Uses move_arm action (not move_to service) — polls status/latest for
# SUCCEEDED(4) before reading position. This is the reliable completion
# signal; the service response does not guarantee the arm has stopped.
#
# Position verification: waits for a fresh position/latest reading after
# each move (detects stale topic), then computes euclidean distance from
# target. Per-axis deltas retained in CSV for analysis.
#
# Coordinate system (established by physical calibration 2026-04-07):
#   Firmware origin = shoulder pivot, 19mm forward of base rotation centre.
#   +X = forward, +Y = left, +Z = up.
#   All commanded coordinates in this script are in firmware space.
#   CENTRE_X=200 is 181mm from the base rotation centre.
#
# Dead zone guards — two conditions, both must pass:
#   1. 2D radius sqrt(tx²+ty²) >= DEAD_ZONE_RADIUS (arm can't fold inward)
#   2. tx >= MIN_FORWARD_X (arm can't reach behind/beside itself)
#      At extreme azimuths (e.g. ±75°) tx becomes very small even at large
#      radii, putting the target physically behind the arm's shoulder.
#      Calibration confirmed x=143mm reachable at table Z; MIN_FORWARD_X=100
#      is a conservative guard that eliminates silent failures at those azimuths.
#
# Returns to centre ONCE per azimuth slice (safe transit between slices).
#
# Grid: 37 azimuths × 10 radii × 16 Z heights = 5,920 points
# Estimated runtime: ~4.5 hours at 5000 mm/min sweep speed.
#
# Output CSV is compatible with rom_full_viewer.html (convex hull envelope).
#
# Debug mode: set DEBUG=1 to print raw send/receive with ROS timestamps.
#   DEBUG=1 bash rom_full.sh
#
# Usage: bash rom_full.sh [MOUNT_POINT] [OUTPUT_CSV]
# Default mount: /home/jack/fuse_mount
# Default output: rom_full_results.csv

set -uo pipefail

MOUNT="${1:-/home/jack/fuse_mount}"
OUTPUT="${2:-rom_full_results.csv}"
SWIFTPRO="${MOUNT}/swiftpro"
MOVE_ARM="${SWIFTPRO}/move_arm"

# ── Debug mode ────────────────────────────────────────────────────────────────
DEBUG="${DEBUG:-0}"

# ── Parameters ────────────────────────────────────────────────────────────────

MOVE_SPEED=50           # mm/min — sweep moves
MOVE_SETTLE=0.5           # seconds — wait after fresh position detected before final read
TRANSIT_SPEED=200        # mm/min — return to centre between azimuth slices
ACTION_TIMEOUT=60         # seconds — max wait for action SUCCEEDED status
POSITION_FRESH_TIMEOUT=5  # seconds — max wait for position topic to update after move
POSITION_TOLERANCE=20.0   # mm — euclidean distance threshold to declare a move "reached"

DEAD_ZONE_RADIUS=133      # mm — inner dead zone: 2D radius sqrt(x²+y²) below this skipped
                          #      spec 132mm, confirmed empirically at 133mm

MIN_FORWARD_X=100         # mm — minimum X (forward) component of any commanded point.
                          #      At extreme azimuths tx collapses even at large radii,
                          #      putting targets physically behind the shoulder pivot.
                          #      Calibration 2026-04-07: x=143mm reachable at table Z.
                          #      100mm is a conservative guard below that.

# Centre — safe known-reachable transit position (firmware coords)
CENTRE_X=200.0
CENTRE_Y=0.0
CENTRE_Z=150.0

# Grid definition
AZ_MIN=-90    # degrees — right side of arm
AZ_MAX=90     # degrees — left side of arm
AZ_STEP=5     # degrees → 31 slices

R_MIN=150     # mm — inner radii removed (unreachable empirically)
R_MAX=320     # mm — workspace limit per spec
R_STEPS=10    # → 10 radii per slice

Z_MIN=-150    # mm — below base plane (empirically finding real floor; observed >= -120)
Z_MAX=190     # mm — near upper limit
Z_STEPS=16    # → ~22mm spacing across 340mm range

# ── Colours ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[0;33m'
CYN='\033[0;36m'
MGN='\033[0;35m'
BLD='\033[1m'
DIM='\033[2m'
RST='\033[0m'

# ── Timestamp ─────────────────────────────────────────────────────────────────
# Nanoseconds since epoch — matches ROS2 log timestamp format for correlation
# with worker node logs.
now_ts() {
    date +%s%N
}

# ── Debug helpers ─────────────────────────────────────────────────────────────
# All debug output goes to stderr — does not pollute stdout or the CSV.

dbg_send() {
    [[ "${DEBUG}" == "1" ]] || return 0
    echo -e "${MGN}[$(now_ts)] >> SEND  $*${RST}" >&2
}

dbg_recv() {
    [[ "${DEBUG}" == "1" ]] || return 0
    echo -e "${MGN}[$(now_ts)] << RECV  $*${RST}" >&2
}

dbg_info() {
    [[ "${DEBUG}" == "1" ]] || return 0
    echo -e "${MGN}[$(now_ts)]    INFO  $*${RST}" >&2
}

# ── Helpers ───────────────────────────────────────────────────────────────────

json_field() {
    echo "$1" | grep -oP "\"$2\"\\s*:\\s*\\K[^,}]+" | tr -d '"' | xargs
}

float_le() {
    awk "BEGIN { exit ($1 <= $2 ? 0 : 1) }"
}

euclidean_dist() {
    awk "BEGIN { dx=($1)-($4); dy=($2)-($5); dz=($3)-($6); print sqrt(dx*dx+dy*dy+dz*dz) }"
}

float_delta() {
    awk "BEGIN { d=$1 - $2; print (d < 0 ? -d : d) }"
}

# Returns 0 (true) if point is in the spherical dead zone
in_dead_zone() {
    awk "BEGIN { r=sqrt($1*$1 + $2*$2); exit (r < ${DEAD_ZONE_RADIUS} ? 0 : 1) }"
}

# Returns 0 (true) if X component is below the minimum forward reach
below_min_forward() {
    awk "BEGIN { exit ($1 < ${MIN_FORWARD_X} ? 0 : 1) }"
}

# Read current position/latest, echo "x y z"
read_position() {
    local pos
    pos=$(cat "${SWIFTPRO}/position/latest" 2>/dev/null)
    dbg_recv "position/latest: ${pos}"
    local x y z
    x=$(json_field "${pos}" "x"); x="${x:-0}"
    y=$(json_field "${pos}" "y"); y="${y:-0}"
    z=$(json_field "${pos}" "z"); z="${z:-0}"
    echo "${x} ${y} ${z}"
}

# Poll position/latest until it differs from prev values, or timeout.
# Echoes "x y z" of the fresh reading.
wait_for_fresh_position() {
    local prev_x="$1" prev_y="$2" prev_z="$3"
    dbg_info "waiting for fresh position (prev=${prev_x},${prev_y},${prev_z})"
    local deadline=$(( $(date +%s) + POSITION_FRESH_TIMEOUT ))
    while [[ $(date +%s) -lt ${deadline} ]]; do
        local rx ry rz
        read -r rx ry rz <<< "$(read_position)"
        if [[ "${rx}" != "${prev_x}" || "${ry}" != "${prev_y}" || "${rz}" != "${prev_z}" ]]; then
            dbg_recv "fresh position: ${rx} ${ry} ${rz}"
            echo "${rx} ${ry} ${rz}"
            return 0
        fi
        sleep 0.1
    done
    dbg_info "fresh position timeout — returning current"
    read_position
}

# Send a move_arm action goal and wait for SUCCEEDED(4).
# Sets LAST_GOAL_RESULT to "succeeded" or "missed".
#
# Two-phase poll:
#   Phase 1 — wait for ACCEPTED(1) or EXECUTING(2) to confirm the action
#             server has picked up THIS goal (guards against reading stale
#             SUCCEEDED from the previous goal).
#   Phase 2 — wait for terminal status: SUCCEEDED(4), CANCELED(5), ABORTED(6).
send_move_goal() {
    local tx="$1" ty="$2" tz="$3" speed="$4"

    local ts
    ts=$(date +%s%N)
    local b0=$(( (ts      ) & 0xFF ))
    local b1=$(( (ts >>  8) & 0xFF ))
    local b2=$(( (ts >> 16) & 0xFF ))
    local b3=$(( (ts >> 24) & 0xFF ))
    local b4=$(( (ts >> 32) & 0xFF ))
    local b5=$(( (ts >> 40) & 0xFF ))
    local uuid_arr="[${b0},${b1},${b2},${b3},${b4},${b5},0,0,0,0,0,0,0,0,0,0]"

    local goal="{\"goal_id\":{\"uuid\":${uuid_arr}},\"goal\":{\"x\":${tx},\"y\":${ty},\"z\":${tz},\"speed\":${speed}}}"

    dbg_send "send_goal/command: ${goal}"
    echo "${goal}" > "${MOVE_ARM}/send_goal/command" 2>/dev/null || { LAST_GOAL_RESULT="missed"; return 1; }

    # Phase 1 — wait for ACCEPTED(1) or EXECUTING(2)
    local accepted=false
    local accept_deadline=$(( $(date +%s) + 5 ))
    while [[ $(date +%s) -lt ${accept_deadline} ]]; do
        local status_json status
        status_json=$(cat "${MOVE_ARM}/status/latest" 2>/dev/null)
        status=$(echo "${status_json}" | grep -oP '"status":\K[0-9]+' | tail -1)
        dbg_recv "status/latest (phase1): ${status_json}"
        if [[ "${status}" == "1" || "${status}" == "2" ]]; then
            dbg_info "goal accepted/executing (status=${status})"
            accepted=true
            break
        fi
        sleep 0.05
    done

    if [[ "${accepted}" == "false" ]]; then
        dbg_info "goal not accepted within 5s — rejected or server busy"
        LAST_GOAL_RESULT="missed"
        return 0
    fi

    # Phase 2 — wait for terminal status
    local deadline=$(( $(date +%s) + ACTION_TIMEOUT ))
    local final_status=""
    while [[ $(date +%s) -lt ${deadline} ]]; do
        local status_json status
        status_json=$(cat "${MOVE_ARM}/status/latest" 2>/dev/null)
        if [[ -n "${status_json}" ]]; then
            status=$(echo "${status_json}" | grep -oP '"status":\K[0-9]+' | tail -1)
            dbg_recv "status/latest (phase2): ${status_json}"
            if [[ "${status}" == "4" || "${status}" == "5" || "${status}" == "6" ]]; then
                final_status="${status}"
                dbg_info "terminal status=${final_status}"
                break
            fi
        fi
        sleep 0.2
    done

    if [[ "${final_status}" == "4" ]]; then
        LAST_GOAL_RESULT="succeeded"
    else
        LAST_GOAL_RESULT="missed"
    fi
}

# Transit to a position — no position check needed
transit_move() {
    send_move_goal "$1" "$2" "$3" "${TRANSIT_SPEED}"
}

# ── Pre-flight ────────────────────────────────────────────────────────────────

if [[ ! -d "${SWIFTPRO}" ]]; then
    echo -e "${RED}SwiftPro not found at ${SWIFTPRO} — is SwiftRos2 running?${RST}"
    exit 1
fi

if [[ ! -d "${MOVE_ARM}" ]]; then
    echo -e "${RED}move_arm action not found at ${MOVE_ARM} — is action support working?${RST}"
    exit 1
fi

connected=$(cat "${SWIFTPRO}/connected/latest" 2>/dev/null)
if [[ $(json_field "${connected}" "data") != "true" ]]; then
    echo -e "${RED}Arm not connected: '${connected}'${RST}"
    exit 1
fi

TOTAL_AZIMUTHS=$(awk "BEGIN { print int((${AZ_MAX} - ${AZ_MIN}) / ${AZ_STEP}) + 1 }")
TOTAL_POINTS=$(( TOTAL_AZIMUTHS * R_STEPS * Z_STEPS ))

echo ""
echo -e "${BLD}${CYN}UArm Swift Pro — Full Workspace Envelope Sweep${RST}"
echo -e "Grid:      ${TOTAL_AZIMUTHS} azimuths × ${R_STEPS} radii × ${Z_STEPS} Z heights = ${BLD}${TOTAL_POINTS} points${RST}"
echo -e "Speed:     sweep=${MOVE_SPEED}mm/min  transit=${TRANSIT_SPEED}mm/min"
echo -e "Tolerance: ${POSITION_TOLERANCE}mm euclidean"
echo -e "Dead zone: 2D radius < ${DEAD_ZONE_RADIUS}mm  OR  X < ${MIN_FORWARD_X}mm → skipped"
echo -e "Debug:     ${DEBUG}"
echo -e "Output:    ${OUTPUT}"
echo -e "Started:   $(now_ts)"
echo ""

# CSV header
echo "point,azimuth,radius,cmd_x,cmd_y,cmd_z,actual_x,actual_y,actual_z,plot_x,plot_y,plot_z,dx,dy,dz,dist,reached,action_succeeded" > "${OUTPUT}"

# Reset arm to home before sweep
echo -e "Resetting arm to home..."
dbg_send "reset/command: {}"
echo '{}' > "${SWIFTPRO}/reset/command"
sleep 4.0

# Move to centre to start
echo -e "Moving to centre (${CENTRE_X},${CENTRE_Y},${CENTRE_Z})..."
transit_move "${CENTRE_X}" "${CENTRE_Y}" "${CENTRE_Z}"
sleep 1.0
echo ""

# ── Main sweep ────────────────────────────────────────────────────────────────

point=1
reached_total=0
skipped_total=0
START_TIME=$(date +%s)

# Seed previous position for stale-detection on the first point
read -r prev_rx prev_ry prev_rz <<< "$(read_position)"

for az_i in $(seq 0 $(( TOTAL_AZIMUTHS - 1 ))); do
    az_deg=$(( AZ_MIN + az_i * AZ_STEP ))
    az_rad=$(awk "BEGIN { printf \"%.8f\", ${az_deg} * 3.14159265358979 / 180.0 }")

    echo -e "${BLD}── Azimuth ${az_deg}°  (slice $(( az_i + 1 ))/${TOTAL_AZIMUTHS})${RST}"

    for z_i in $(seq 0 $(( Z_STEPS - 1 ))); do
        tz=$(awk "BEGIN { printf \"%.1f\", ${Z_MIN} + (${Z_MAX} - ${Z_MIN}) * ${z_i} / (${Z_STEPS} - 1) }")

        for r_i in $(seq 0 $(( R_STEPS - 1 ))); do
            tr=$(awk "BEGIN { printf \"%.1f\", ${R_MIN} + (${R_MAX} - ${R_MIN}) * ${r_i} / (${R_STEPS} - 1) }")
            tx=$(awk "BEGIN { printf \"%.1f\", ${tr} * cos(${az_rad}) }")
            ty=$(awk "BEGIN { printf \"%.1f\", ${tr} * sin(${az_rad}) }")

            # ── Dead zone guards ──────────────────────────────────────────────
            skip_reason=""
            if in_dead_zone "${tx}" "${ty}"; then
                skip_reason="2D radius < ${DEAD_ZONE_RADIUS}mm"
            elif below_min_forward "${tx}"; then
                skip_reason="X=${tx} < MIN_FORWARD_X=${MIN_FORWARD_X}mm"
            fi

            if [[ -n "${skip_reason}" ]]; then
                (( skipped_total++ )) || true
                echo -e "${DIM}$(printf '%4d/%d  az=%4d°  r=%5.1f  z=%5.1f  cmd=(%6.1f,%6.1f,%5.1f)  skipped (%s)' \
                    "${point}" "${TOTAL_POINTS}" \
                    "${az_deg}" "${tr}" "${tz}" \
                    "${tx}" "${ty}" "${tz}" \
                    "${skip_reason}")${RST}"
                echo "${point},${az_deg},${tr},${tx},${ty},${tz},0,0,0,${tx},${ty},${tz},0,0,0,0,false,false" >> "${OUTPUT}"
                (( point++ )) || true
                continue
            fi

            # ── Send action goal, wait for SUCCEEDED ──────────────────────────
            send_move_goal "${tx}" "${ty}" "${tz}" "${MOVE_SPEED}"
            action_succeeded="false"
            [[ "${LAST_GOAL_RESULT}" == "succeeded" ]] && action_succeeded="true"

            # Wait for fresh position, settle, then final read
            read -r rx ry rz <<< "$(wait_for_fresh_position "${prev_rx}" "${prev_ry}" "${prev_rz}")"
            sleep "${MOVE_SETTLE}"
            read -r rx ry rz <<< "$(read_position)"

            # Update stale-detection seed for next iteration
            prev_rx="${rx}"; prev_ry="${ry}"; prev_rz="${rz}"

            dx=$(float_delta "${rx}" "${tx}")
            dy=$(float_delta "${ry}" "${ty}")
            dz=$(float_delta "${rz}" "${tz}")
            dist=$(euclidean_dist "${rx}" "${ry}" "${rz}" "${tx}" "${ty}" "${tz}")

            reached=true
            float_le "${dist}" "${POSITION_TOLERANCE}" || reached=false

            if [[ "${reached}" == "true" ]]; then
                plot_x="${rx}"; plot_y="${ry}"; plot_z="${rz}"
                (( reached_total++ )) || true
                color="${GRN}"
            else
                plot_x="${tx}"; plot_y="${ty}"; plot_z="${tz}"
                color="${YLW}"
            fi

            # ETA
            now=$(date +%s)
            elapsed=$(( now - START_TIME ))
            if (( point > 1 )); then
                eta=$(awk "BEGIN { printf \"%d\", ${elapsed} * (${TOTAL_POINTS} - ${point}) / (${point} - 1) }")
                eta_str=$(printf '%dh%02dm' $(( eta / 3600 )) $(( (eta % 3600) / 60 )))
            else
                eta_str="--"
            fi

            echo -e "${color}$(printf '%4d/%d  az=%4d°  r=%5.1f  z=%5.1f  cmd=(%6.1f,%6.1f,%5.1f)  actual=(%6.1f,%6.1f,%5.1f)  dist=%6.2fmm  %s' \
                "${point}" "${TOTAL_POINTS}" \
                "${az_deg}" "${tr}" "${tz}" \
                "${tx}" "${ty}" "${tz}" \
                "${rx}" "${ry}" "${rz}" \
                "${dist}" \
                "${reached}")${RST}  ${DIM}ETA ${eta_str}${RST}"

            echo "${point},${az_deg},${tr},${tx},${ty},${tz},${rx},${ry},${rz},${plot_x},${plot_y},${plot_z},${dx},${dy},${dz},${dist},${reached},${action_succeeded}" >> "${OUTPUT}"

            (( point++ )) || true
        done
    done

    # Return to centre between azimuth slices, reseed stale-detection
    echo -e "  ${DIM}→ transit to centre${RST}"
    transit_move "${CENTRE_X}" "${CENTRE_Y}" "${CENTRE_Z}"
    sleep 0.5
    read -r prev_rx prev_ry prev_rz <<< "$(read_position)"
    echo ""
done

# ── Summary ───────────────────────────────────────────────────────────────────

END_TIME=$(date +%s)
ELAPSED=$(( END_TIME - START_TIME ))
ELAPSED_STR=$(printf '%dh%02dm%02ds' $(( ELAPSED / 3600 )) $(( (ELAPSED % 3600) / 60 )) $(( ELAPSED % 60 )))

echo "════════════════════════════════════════════════════════════════════"
echo ""
echo -e "Finished:  $(now_ts)"
echo -e "Runtime:   ${ELAPSED_STR}"
echo -e "Reached:   ${BLD}${reached_total}${RST} / ${TOTAL_POINTS} within ${POSITION_TOLERANCE}mm"
echo -e "Skipped:   ${BLD}${skipped_total}${RST} points (dead zone or min forward X)"
echo -e "Output:    ${BLD}${OUTPUT}${RST}"
echo ""
