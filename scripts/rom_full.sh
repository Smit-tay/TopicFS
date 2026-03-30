#!/usr/bin/env bash
# MIT License
# Copyright 2026 Jack Sidman Smith
#
# rom_full.sh — UArm Swift Pro full workspace envelope sweep via TopicFS.
#
# Systematic spherical-coordinate grid sweep using Cartesian move_to.
# Sweeps azimuth slices (-90° to +90°, 5° steps). Within each slice,
# sweeps Z heights outer-to-inner at each radius. Adjacent points are
# always geometrically close — the arm never makes a large uncontrolled jump.
#
# Dead zone guard: any point whose 2D radius sqrt(tx²+ty²) falls below
# DEAD_ZONE_RADIUS is skipped and recorded as unreachable. This eliminates
# the firmware rejection failures at extreme azimuths where tx→0.
# Official inner dead zone radius: 132mm (confirmed empirically at 133mm).
#
# Returns to centre ONCE per azimuth slice (safe transit between slices).
#
# Grid: 37 azimuths × 10 radii × 15 Z heights = 5,550 points
# Estimated runtime: ~3.5 hours at 5000 mm/min sweep speed.
#
# Output CSV is compatible with rom_full_viewer.html (convex hull envelope).
#
# Usage: bash rom_full.sh [MOUNT_POINT] [OUTPUT_CSV]
# Default mount: /home/jack/fuse_mount
# Default output: rom_full_results.csv

set -uo pipefail

MOUNT="${1:-/home/jack/fuse_mount}"
OUTPUT="${2:-rom_full_results.csv}"
SWIFTPRO="${MOUNT}/swiftpro"

# ── Parameters ────────────────────────────────────────────────────────────────

MOVE_SPEED=5000         # mm/min — sweep moves (within 100mm/s rated max)
MOVE_SETTLE=0.5         # seconds — wait after service response before reading position
TRANSIT_SPEED=6000      # mm/min — return to centre between azimuth slices
SERVICE_TIMEOUT=45      # seconds — generous timeout for distant points
POSITION_TOLERANCE=8.0  # mm — threshold to declare a move "reached"
                        # (0.2mm repeatability spec; 8mm covers thermal drift
                        #  and residual calibration offset)

DEAD_ZONE_RADIUS=133    # mm — inner dead zone boundary (spec 132mm,
                        #  confirmed empirically at 133mm). Points whose
                        #  2D radius sqrt(x²+y²) falls below this are
                        #  skipped, not commanded.

# Centre — safe known-reachable transit position
CENTRE_X=200.0
CENTRE_Y=0.0
CENTRE_Z=100.0

# Grid definition
AZ_MIN=-75      # degrees — right side of arm
AZ_MAX=75       # degrees — left side of arm
AZ_STEP=5       # degrees → 37 slices

R_MIN=100       # mm — start of sweep (just inside dead zone boundary;
                #  dead zone guard skips any point that falls short)
R_MAX=320       # mm — workspace limit per spec
R_STEPS=10      # steps → 10 radii per slice

Z_MIN=-120      # mm — below shelf surface (arm hanging off edge;
                #  empirically verified floor at -126mm)
Z_MAX=190       # mm — near upper limit
Z_STEPS=15      # steps → ~22mm spacing across 310mm range

# ── Colours ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[0;33m'
CYN='\033[0;36m'
BLD='\033[1m'
DIM='\033[2m'
RST='\033[0m'

# ── Helpers ───────────────────────────────────────────────────────────────────

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

# Returns 0 (true) if 2D radius of (x,y) is below dead zone threshold
in_dead_zone() {
    awk "BEGIN { r=sqrt($1*$1 + $2*$2); exit (r < ${DEAD_ZONE_RADIUS} ? 0 : 1) }"
}

# ── Pre-flight ─────────────────────────────────────────────────────────────────

if [[ ! -d "${SWIFTPRO}" ]]; then
    echo -e "${RED}SwiftPro not found at ${SWIFTPRO} — is SwiftRos2 running?${RST}"
    exit 1
fi

TOTAL_AZIMUTHS=$(awk "BEGIN { print int((${AZ_MAX} - ${AZ_MIN}) / ${AZ_STEP}) + 1 }")
TOTAL_POINTS=$(( TOTAL_AZIMUTHS * R_STEPS * Z_STEPS ))

echo ""
echo -e "${BLD}${CYN}UArm Swift Pro — Full Workspace Envelope Sweep${RST}"
echo -e "Grid:    ${TOTAL_AZIMUTHS} azimuths × ${R_STEPS} radii × ${Z_STEPS} Z heights = ${BLD}${TOTAL_POINTS} points${RST}"
echo -e "Speed:   sweep=${MOVE_SPEED}mm/min  transit=${TRANSIT_SPEED}mm/min"
echo -e "Mode:    move_to Cartesian  (dead zone guard: r<${DEAD_ZONE_RADIUS}mm skipped)"
echo -e "Output:  ${OUTPUT}"
echo -e "Started: $(date '+%Y-%m-%d %H:%M:%S')"
echo ""

# CSV header
echo "point,azimuth,radius,cmd_x,cmd_y,cmd_z,actual_x,actual_y,actual_z,plot_x,plot_y,plot_z,dx,dy,dz,reached,service_success,error_code" > "${OUTPUT}"

# Reset arm to calibrated home position before sweep
echo -e "Resetting arm to home..."
echo '{}' > "${SWIFTPRO}/reset/command"
sleep 4.0

# Move to centre to start
echo -e "Moving to centre (${CENTRE_X},${CENTRE_Y},${CENTRE_Z})..."
call_move "${CENTRE_X}" "${CENTRE_Y}" "${CENTRE_Z}" "${TRANSIT_SPEED}" > /dev/null
sleep 1.0
echo ""

# ── Main sweep ────────────────────────────────────────────────────────────────

point=1
reached_total=0
skipped_total=0
START_TIME=$(date +%s)

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

            # ── Dead zone guard ───────────────────────────────────────────────
            if in_dead_zone "${tx}" "${ty}"; then
                (( skipped_total++ )) || true
                echo -e "${DIM}$(printf '%4d/%d  az=%4d°  r=%5.1f  z=%5.1f  cmd=(%6.1f,%6.1f,%5.1f)  DEAD ZONE — skipped' \
                    "${point}" "${TOTAL_POINTS}" \
                    "${az_deg}" "${tr}" "${tz}" \
                    "${tx}" "${ty}" "${tz}")${RST}"
                echo "${point},${az_deg},${tr},${tx},${ty},${tz},0,0,0,${tx},${ty},${tz},0,0,0,false,false,-98" >> "${OUTPUT}"
                (( point++ )) || true
                continue
            fi

            # ── Command and record ────────────────────────────────────────────
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

            if [[ "${reached}" == "true" ]]; then
                plot_x="${rx}"; plot_y="${ry}"; plot_z="${rz}"
                (( reached_total++ )) || true
                color="${GRN}"
            else
                plot_x="${tx}"; plot_y="${ty}"; plot_z="${tz}"
                color="${YLW}"
            fi

            # Elapsed + ETA
            now=$(date +%s)
            elapsed=$(( now - START_TIME ))
            if (( point > 1 )); then
                eta=$(awk "BEGIN { printf \"%d\", ${elapsed} * (${TOTAL_POINTS} - ${point}) / (${point} - 1) }")
                eta_str=$(printf '%dh%02dm' $(( eta / 3600 )) $(( (eta % 3600) / 60 )))
            else
                eta_str="--"
            fi

            echo -e "${color}$(printf '%4d/%d  az=%4d°  r=%5.1f  z=%5.1f  cmd=(%6.1f,%6.1f,%5.1f)  actual=(%6.1f,%6.1f,%5.1f)  %s' \
                "${point}" "${TOTAL_POINTS}" \
                "${az_deg}" "${tr}" "${tz}" \
                "${tx}" "${ty}" "${tz}" \
                "${rx}" "${ry}" "${rz}" \
                "${reached}")${RST}  ${DIM}ETA ${eta_str}${RST}"

            echo "${point},${az_deg},${tr},${tx},${ty},${tz},${rx},${ry},${rz},${plot_x},${plot_y},${plot_z},${dx},${dy},${dz},${reached},${service_success},${error_code}" >> "${OUTPUT}"

            (( point++ )) || true
        done
    done

    # Return to centre between azimuth slices (safe transit)
    echo -e "  ${DIM}→ transit to centre${RST}"
    call_move "${CENTRE_X}" "${CENTRE_Y}" "${CENTRE_Z}" "${TRANSIT_SPEED}" > /dev/null
    sleep 0.5
    echo ""
done

# ── Summary ───────────────────────────────────────────────────────────────────

END_TIME=$(date +%s)
ELAPSED=$(( END_TIME - START_TIME ))
ELAPSED_STR=$(printf '%dh%02dm%02ds' $(( ELAPSED / 3600 )) $(( (ELAPSED % 3600) / 60 )) $(( ELAPSED % 60 )))

echo "════════════════════════════════════════════════════════════════════"
echo ""
echo -e "Finished:  $(date '+%Y-%m-%d %H:%M:%S')"
echo -e "Runtime:   ${ELAPSED_STR}"
echo -e "Reached:   ${BLD}${reached_total}${RST} / ${TOTAL_POINTS} within ${POSITION_TOLERANCE}mm"
echo -e "Skipped:   ${BLD}${skipped_total}${RST} dead zone points (2D radius < ${DEAD_ZONE_RADIUS}mm)"
echo -e "Output:    ${BLD}${OUTPUT}${RST}"
echo ""
