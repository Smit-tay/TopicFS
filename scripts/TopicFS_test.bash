#!/usr/bin/env bash
# MIT License
# Copyright 2026 Jack Sidman Smith
#
# TopicFS_test.bash — Integration test for the TopicFS / SwiftRos2 end-to-end
# stack. Exercises topics, services, and actions via plain shell tools.
# No ROS2 on host required — talks to the arm purely through filesystem reads
# and writes on the FUSE mount.
#
# Changes vs v1:
#   - Removed phantom topic reads (mode, polar, is_moving — those topics don't
#     exist; they're either services only or moved to motion_complete).
#   - Reset.srv schema fix: request now carries x/y/z/speed (was sending {}).
#   - Pre-flight failures (other than missing mount point) report fail and
#     continue, so we see the whole picture.
#   - motion_reset NOT exercised — libswiftpro Bug 2 would brick the arm.
#   - Added topic reads for connected, gripper_status, motion_complete, power
#     (the actual topic surface).
#   - Stress test count reduced to 10 (was 20) — keeps test cycle manageable.
#
# Usage: bash TopicFS_test.bash [MOUNT_POINT]
# Default mount point: /home/jack/fuse_mount

set -uo pipefail

MOUNT="${1:-/home/jack/fuse_mount}"
SWIFTPRO="${MOUNT}/swiftpro"
MOVE_ARM="${SWIFTPRO}/move_arm"

# --- Tunables ---
SERVICE_TIMEOUT=30       # seconds to wait for a service response
ACTION_TIMEOUT=60        # seconds to wait for an action to complete
POSITION_TOLERANCE=15.0  # mm — acceptable distance between commanded and reported position
MOVE_SETTLE=0.5          # seconds to wait for position topic to update after move

# --- Move speeds (mm/min per Reset.srv schema; MoveTo may differ) ---
SPEED_SLOW=10
SPEED_NORMAL=50
SPEED_FAST=100

# --- Home position ---
HOME_X=200.0
HOME_Y=0.0
HOME_Z=150.0

# --- UUID counter — incremented for each goal ---
UUID_COUNTER=1

# --- Counters ---
PASS=0
FAIL=0
SKIP=0

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

header() {
    echo ""
    echo -e "${BLD}${CYN}━━━  $1  ━━━${RST}"
}

pass() {
    echo -e "  ${GRN}✓${RST}  $1"
    PASS=$((PASS + 1))
}

fail() {
    echo -e "  ${RED}✗${RST}  $1"
    FAIL=$((FAIL + 1))
}

skip() {
    echo -e "  ${YLW}–${RST}  $1 ${YLW}(skipped)${RST}"
    SKIP=$((SKIP + 1))
}

# Read a /latest file; return empty string if missing or 0-byte.
read_topic() {
    local file="$1"
    local content
    content=$(cat "${file}" 2>/dev/null) || { echo ""; return 1; }
    [[ -z "${content}" ]] && { echo ""; return 1; }
    echo "${content}"
}

# Extract a flat JSON field value (does not handle nested objects/arrays).
json_field() {
    local json="$1"
    local field="$2"
    echo "${json}" | grep -oP "\"${field}\"\\s*:\\s*\\K[^,}]+" | tr -d '"' | xargs
}

float_le() {
    awk "BEGIN { exit (($1) <= ($2) ? 0 : 1) }"
}

euclidean_dist() {
    awk "BEGIN { dx=($1)-($4); dy=($2)-($5); dz=($3)-($6); print sqrt(dx*dx+dy*dy+dz*dz) }"
}

# Write request to a plain service, poll response until timeout.
# Stdout: response body on success, empty on timeout/failure.
call_service() {
    local svc="$1"
    local request="$2"
    local timeout="${3:-${SERVICE_TIMEOUT}}"
    local cmd="${SWIFTPRO}/${svc}/command"
    local rsp="${SWIFTPRO}/${svc}/response"

    echo "${request}" > "${cmd}" 2>/dev/null || { echo ""; return 1; }

    local deadline=$(( $(date +%s) + timeout ))
    while [[ $(date +%s) -lt ${deadline} ]]; do
        local content
        content=$(cat "${rsp}" 2>/dev/null)
        if [[ -n "${content}" && "${content}" != *"no response yet"* ]]; then
            echo "${content}"
            return 0
        fi
        sleep 0.2
    done
    echo ""
    return 1
}

# Send a move_arm action goal and wait for completion.
# Returns 0 on SUCCEEDED + position within tolerance; 1 otherwise.
# Args: label x y z speed
send_move_goal() {
    local label="$1"
    local tx="$2"
    local ty="$3"
    local tz="$4"
    local speed="$5"

    local uuid="${UUID_COUNTER}"
    UUID_COUNTER=$((UUID_COUNTER + 1))

    # Build UUID array — first byte is the counter, rest zeros.
    # ROS2 needs a 16-byte UUID; uniqueness within the run is sufficient.
    local uuid_arr="[${uuid},0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]"
    local goal="{\"goal_id\":{\"uuid\":${uuid_arr}},\"goal\":{\"x\":${tx},\"y\":${ty},\"z\":${tz},\"speed\":${speed}}}"

    echo "${goal}" > "${MOVE_ARM}/send_goal/command" 2>/dev/null || {
        fail "${label}: failed to write send_goal"
        return 1
    }

    # Wait for accepted response
    sleep 0.3
    local accepted
    accepted=$(cat "${MOVE_ARM}/send_goal/response" 2>/dev/null)
    if [[ -z "${accepted}" || "${accepted}" == *"error"* || "${accepted}" == *"no response"* ]]; then
        fail "${label}: goal not accepted — ${accepted}"
        return 1
    fi

    # Poll status until SUCCEEDED(4), CANCELED(5), ABORTED(6), or timeout
    local deadline=$(( $(date +%s) + ACTION_TIMEOUT ))
    local final_status=""
    while [[ $(date +%s) -lt ${deadline} ]]; do
        local status_json
        status_json=$(cat "${MOVE_ARM}/status/latest" 2>/dev/null)
        if [[ -n "${status_json}" ]]; then
            local status
            status=$(echo "${status_json}" | grep -oP '"status":\K[0-9]+' | tail -1)
            if [[ "${status}" == "4" || "${status}" == "5" || "${status}" == "6" ]]; then
                final_status="${status}"
                break
            fi
        fi
        sleep 0.2
    done

    if [[ -z "${final_status}" ]]; then
        fail "${label}: action timed out after ${ACTION_TIMEOUT}s"
        return 1
    fi

    if [[ "${final_status}" != "4" ]]; then
        fail "${label}: action ended with status ${final_status} (expected 4=SUCCEEDED)"
        return 1
    fi

    # Get result
    echo "{\"goal_id\":{\"uuid\":${uuid_arr}}}" > "${MOVE_ARM}/get_result/command" 2>/dev/null
    sleep 0.3

    # Verify position by reading the position topic
    sleep "${MOVE_SETTLE}"
    local pos
    pos=$(read_topic "${SWIFTPRO}/position/latest")
    if [[ -z "${pos}" ]]; then
        fail "${label}: no position data after move"
        return 1
    fi

    local rx ry rz dist
    rx=$(json_field "${pos}" "x")
    ry=$(json_field "${pos}" "y")
    rz=$(json_field "${pos}" "z")
    dist=$(euclidean_dist "${rx}" "${ry}" "${rz}" "${tx}" "${ty}" "${tz}")

    if float_le "${dist}" "${POSITION_TOLERANCE}"; then
        pass "${label} [${speed}mm/min]: target=(${tx},${ty},${tz}) actual=(${rx},${ry},${rz}) dist=${dist}mm"
        return 0
    else
        fail "${label} [${speed}mm/min]: target=(${tx},${ty},${tz}) actual=(${rx},${ry},${rz}) dist=${dist}mm > ${POSITION_TOLERANCE}mm tolerance"
        return 1
    fi
}

# ============================================================
# Pre-flight
# ============================================================

header "Pre-flight checks"

# Hard fail: mount point missing means nothing else can run.
if [[ ! -d "${MOUNT}" ]]; then
    fail "Mount point ${MOUNT} does not exist"
    echo -e "\n${RED}Aborting — mount point missing.${RST}"
    exit 1
fi
pass "Mount point accessible: ${MOUNT}"

# Hard fail: no swiftpro/ means SwiftRos2 isn't visible at all.
if [[ ! -d "${SWIFTPRO}" ]]; then
    fail "SwiftPro directory not found at ${SWIFTPRO} — is SwiftRos2 running?"
    echo -e "\n${RED}Aborting — SwiftRos2 not visible via TopicFS.${RST}"
    exit 1
fi
pass "SwiftPro directory present: ${SWIFTPRO}"

# Soft fail from here on — report and continue so the operator sees the full picture.
connected=$(read_topic "${SWIFTPRO}/connected/latest")
if [[ $(json_field "${connected}" "data") == "true" ]]; then
    pass "Arm connected: ${connected}"
else
    fail "Arm not connected: '${connected}'"
fi

if [[ -d "${MOVE_ARM}" ]]; then
    pass "move_arm action directory found"
else
    fail "move_arm action not found at ${MOVE_ARM} — is action support working?"
fi

# ============================================================
# Topic reads — confirmed surface only
# ============================================================

header "Topic reads"

# joint_states lives at root, not under swiftpro/
js=$(read_topic "${MOUNT}/joint_states/latest")
[[ -n "${js}" ]] && pass "joint_states: present" || fail "joint_states: no data"

pos=$(read_topic "${SWIFTPRO}/position/latest")
if [[ -n "${pos}" ]]; then
    x=$(json_field "${pos}" "x"); y=$(json_field "${pos}" "y"); z=$(json_field "${pos}" "z")
    pass "position: x=${x}mm  y=${y}mm  z=${z}mm"
else
    fail "position: no data"
fi

pump=$(read_topic "${SWIFTPRO}/pump_status/latest")
[[ -n "${pump}" ]] && pass "pump_status: ${pump}" || fail "pump_status: no data"

gripper=$(read_topic "${SWIFTPRO}/gripper_status/latest")
[[ -n "${gripper}" ]] && pass "gripper_status: ${gripper}" || fail "gripper_status: no data"

power=$(read_topic "${SWIFTPRO}/power/latest")
[[ -n "${power}" ]] && pass "power: ${power}" || fail "power: no data"

connected=$(read_topic "${SWIFTPRO}/connected/latest")
[[ -n "${connected}" ]] && pass "connected: ${connected}" || fail "connected: no data"

# motion_complete and limit_switch fire on edges — file may exist but
# contain no data at rest. File-existence is the assertion here.
mc_file="${SWIFTPRO}/motion_complete/latest"
if [[ -f "${mc_file}" ]]; then
    mc_data=$(cat "${mc_file}" 2>/dev/null)
    if [[ -n "${mc_data}" ]]; then
        pass "motion_complete: ${mc_data}"
    else
        pass "motion_complete: file present, no edge yet (expected at rest)"
    fi
else
    fail "motion_complete: file missing"
fi

ls_file="${SWIFTPRO}/limit_switch/latest"
if [[ -f "${ls_file}" ]]; then
    ls_data=$(cat "${ls_file}" 2>/dev/null)
    if [[ -n "${ls_data}" ]]; then
        pass "limit_switch: ${ls_data}"
    else
        pass "limit_switch: file present, no trigger data (expected)"
    fi
else
    fail "limit_switch: file missing"
fi

# ============================================================
# Action interface check
# ============================================================

header "Action interface"

for sub in send_goal cancel get_result feedback status; do
    if [[ -d "${MOVE_ARM}/${sub}" ]]; then
        pass "${sub} directory present"
    else
        fail "${sub} directory missing"
    fi
done

status=$(read_topic "${MOVE_ARM}/status/latest")
[[ -n "${status}" ]] && pass "status/latest readable: ${status}" || fail "status/latest: no data"

# ============================================================
# Read-only services
# ============================================================

header "Read-only services"

rsp=$(call_service "get_analog" '{"pin": 0}')
if [[ -n "${rsp}" ]]; then
    val=$(json_field "${rsp}" "value"); ok=$(json_field "${rsp}" "success")
    [[ "${ok}" == "true" ]] && pass "get_analog pin=0: value=${val}" || fail "get_analog: ${rsp}"
else
    fail "get_analog: no response"
fi

rsp=$(call_service "get_digital" '{"pin": 0}')
if [[ -n "${rsp}" ]]; then
    val=$(json_field "${rsp}" "value"); ok=$(json_field "${rsp}" "success")
    [[ "${ok}" == "true" ]] && pass "get_digital pin=0: value=${val}" || fail "get_digital: ${rsp}"
else
    fail "get_digital: no response"
fi

rsp=$(call_service "get_encoder_status" '{"structure_needs_at_least_one_member":0}')
if [[ -n "${rsp}" ]]; then
    s=$(json_field "${rsp}" "status"); ok=$(json_field "${rsp}" "success")
    [[ "${ok}" == "true" ]] && pass "get_encoder_status: status=${s} (0=all healthy)" || fail "get_encoder_status: ${rsp}"
else
    fail "get_encoder_status: no response"
fi

# get_servo_attach across all 4 servos
for sid in 0 1 2 3; do
    rsp=$(call_service "get_servo_attach" "{\"servo_id\": ${sid}}")
    if [[ -n "${rsp}" ]]; then
        att=$(json_field "${rsp}" "attached")
        pass "get_servo_attach servo_id=${sid}: attached=${att}"
    else
        fail "get_servo_attach servo_id=${sid}: no response"
    fi
done

# is_reachable — positive and negative cases
rsp=$(call_service "is_reachable" '{"x": 200.0, "y": 0.0, "z": 150.0}')
if [[ -n "${rsp}" ]]; then
    r=$(json_field "${rsp}" "reachable")
    [[ "${r}" == "true" ]] && pass "is_reachable (200,0,150) = true" || fail "is_reachable (200,0,150): got ${r}"
else
    fail "is_reachable (200,0,150): no response"
fi

rsp=$(call_service "is_reachable" '{"x": 1000.0, "y": 0.0, "z": 150.0}')
if [[ -n "${rsp}" ]]; then
    r=$(json_field "${rsp}" "reachable")
    [[ "${r}" == "false" ]] && pass "is_reachable (1000,0,150) = false" || fail "is_reachable (1000,0,150): got ${r}"
else
    fail "is_reachable (1000,0,150): no response"
fi

# Kinematics queries
rsp=$(call_service "coord_to_angles" '{"x": 200.0, "y": 0.0, "z": 150.0}')
[[ -n "${rsp}" ]] && pass "coord_to_angles (200,0,150): ${rsp}" || fail "coord_to_angles: no response"

rsp=$(call_service "angles_to_coord" '{"base": 0.0, "left": 90.0, "right": 0.0}')
[[ -n "${rsp}" ]] && pass "angles_to_coord (0,90,0): ${rsp}" || fail "angles_to_coord: no response"

# ============================================================
# Buzzer
# ============================================================

header "Buzzer (verify audibly)"

rsp=$(call_service "set_buzzer" '{"frequency": 1000, "duration": 0.3}')
[[ -n "${rsp}" ]] && pass "set_buzzer 1000Hz/0.3s: ${rsp}" || fail "set_buzzer 1000Hz: no response"
sleep 0.5

rsp=$(call_service "set_buzzer" '{"frequency": 2000, "duration": 0.3}')
[[ -n "${rsp}" ]] && pass "set_buzzer 2000Hz/0.3s: ${rsp} (verify higher pitch)" || fail "set_buzzer 2000Hz: no response"
sleep 0.5

# ============================================================
# Configuration
# ============================================================

header "Configuration"

rsp=$(call_service "set_mode" '{"mode": 0}')
[[ -n "${rsp}" ]] && pass "set_mode 0 (normal): ${rsp}" || fail "set_mode 0: no response"

rsp=$(call_service "set_acceleration" '{"acc": 50.0}')
[[ -n "${rsp}" ]] && pass "set_acceleration 50: ${rsp}" || fail "set_acceleration 50: no response"

# ============================================================
# End effectors
# ============================================================

header "Pump on/off"

rsp=$(call_service "set_pump" '{"on": true}')
if [[ -n "${rsp}" ]]; then
    sleep 1; pump=$(read_topic "${SWIFTPRO}/pump_status/latest")
    pass "Pump ON: response=${rsp}  status=${pump}"
else
    fail "set_pump ON: no response"
fi
sleep 1

rsp=$(call_service "set_pump" '{"on": false}')
if [[ -n "${rsp}" ]]; then
    sleep 1; pump=$(read_topic "${SWIFTPRO}/pump_status/latest")
    pass "Pump OFF: response=${rsp}  status=${pump}"
else
    fail "set_pump OFF: no response"
fi

header "Gripper catch/release"

rsp=$(call_service "set_gripper" '{"catch_object": true}')
if [[ -n "${rsp}" ]]; then
    sleep 1; g=$(read_topic "${SWIFTPRO}/gripper_status/latest")
    pass "Gripper CATCH: response=${rsp}  status=${g}"
else
    fail "set_gripper CATCH: no response"
fi
sleep 1

rsp=$(call_service "set_gripper" '{"catch_object": false}')
if [[ -n "${rsp}" ]]; then
    sleep 1; g=$(read_topic "${SWIFTPRO}/gripper_status/latest")
    pass "Gripper RELEASE: response=${rsp}  status=${g}"
else
    fail "set_gripper RELEASE: no response"
fi

# ============================================================
# Reset — schema fix: must carry x/y/z/speed, not {}
# ============================================================

header "Reset service"

reset_req="{\"x\":${HOME_X},\"y\":${HOME_Y},\"z\":${HOME_Z},\"speed\":${SPEED_NORMAL}}"
rsp=$(call_service "reset" "${reset_req}" 60)
if [[ -n "${rsp}" ]]; then
    sleep 3
    pos=$(read_topic "${SWIFTPRO}/position/latest")
    pass "Reset to home: response=${rsp}  position=${pos}"
else
    fail "Reset: no response (arm may have moved — check physically)"
fi

# ============================================================
# motion_reset — SKIPPED (libswiftpro Bug 2)
# ============================================================

header "Motion reset (deferred)"

skip "motion_reset: libswiftpro Bug 2 — mc_reset() during motion → STATE_ALARM, subsequent commands time out"

# ============================================================
# Movement — basic action test
# ============================================================

header "Movement — home via action"
send_move_goal "Home" ${HOME_X} ${HOME_Y} ${HOME_Z} ${SPEED_NORMAL}

header "Movement — cardinal positions"
send_move_goal "Forward"  220.0    0.0   80.0  ${SPEED_NORMAL}
send_move_goal "Left"     180.0   60.0   80.0  ${SPEED_NORMAL}
send_move_goal "Right"    180.0  -60.0   80.0  ${SPEED_NORMAL}
send_move_goal "Back"     150.0    0.0   80.0  ${SPEED_NORMAL}
send_move_goal "Home"     ${HOME_X} ${HOME_Y} ${HOME_Z} ${SPEED_NORMAL}

header "Movement — vertical range"
send_move_goal "Low"   180.0  0.0   20.0  ${SPEED_NORMAL}
send_move_goal "Mid"   180.0  0.0   80.0  ${SPEED_NORMAL}
send_move_goal "High"  180.0  0.0  150.0  ${SPEED_NORMAL}

header "Movement — diagonal / combined axes"
send_move_goal "Forward-left-high"   210.0   50.0  130.0  ${SPEED_NORMAL}
send_move_goal "Forward-right-low"   210.0  -50.0   30.0  ${SPEED_NORMAL}
send_move_goal "Back-left-mid"       155.0   40.0   80.0  ${SPEED_NORMAL}
send_move_goal "Back-right-mid"      155.0  -40.0   80.0  ${SPEED_NORMAL}
send_move_goal "Home"                ${HOME_X} ${HOME_Y} ${HOME_Z} ${SPEED_NORMAL}

header "Movement — speed variation"
send_move_goal "Slow"    200.0   30.0   80.0  ${SPEED_SLOW}
send_move_goal "Home"    ${HOME_X} ${HOME_Y} ${HOME_Z} ${SPEED_NORMAL}
send_move_goal "Normal"  200.0  -30.0   80.0  ${SPEED_NORMAL}
send_move_goal "Home"    ${HOME_X} ${HOME_Y} ${HOME_Z} ${SPEED_NORMAL}
send_move_goal "Fast"    200.0   30.0   80.0  ${SPEED_FAST}
send_move_goal "Home"    ${HOME_X} ${HOME_Y} ${HOME_Z} ${SPEED_NORMAL}

# ============================================================
# Stress test — 10 consecutive action goals
# Rapid sequence of moves, interleaved with service calls.
# Tests executor stability under sustained load.
# ============================================================

header "Stress test — 10 consecutive goals"

STRESS_POINTS=(
    "190.0  30.0  100.0"
    "210.0 -30.0   60.0"
    "170.0  50.0  120.0"
    "200.0   0.0   80.0"
    "220.0 -50.0   40.0"
    "160.0  40.0  140.0"
    "195.0  10.0   70.0"
    "215.0 -20.0  110.0"
    "175.0  60.0   50.0"
    "205.0 -40.0   90.0"
)

stress_pass=0
stress_fail=0

for i in "${!STRESS_POINTS[@]}"; do
    read -r sx sy sz <<< "${STRESS_POINTS[$i]}"
    n=$(( i + 1 ))

    if send_move_goal "Stress #${n}" "${sx}" "${sy}" "${sz}" ${SPEED_FAST}; then
        stress_pass=$((stress_pass + 1))
    else
        stress_fail=$((stress_fail + 1))
    fi

    # Every 5 moves, fire a service call to verify executor health
    if (( n % 5 == 0 )); then
        rsp=$(call_service "get_digital" '{"pin": 0}' 5)
        if [[ -n "${rsp}" && "${rsp}" != *"error"* ]]; then
            echo -e "  ${CYN}↳ executor health check #$((n/5)): get_digital OK${RST}"
        else
            echo -e "  ${RED}↳ executor health check #$((n/5)): get_digital FAILED — ${rsp}${RST}"
            stress_fail=$((stress_fail + 1))
            stress_pass=$((stress_pass - 1))
        fi
    fi
done

echo ""
echo -e "  Stress test: ${GRN}${stress_pass} passed${RST}  ${RED}${stress_fail} failed${RST} / 10 goals"

send_move_goal "Stress — final home" ${HOME_X} ${HOME_Y} ${HOME_Z} ${SPEED_NORMAL}

# ============================================================
# Summary
# ============================================================

TOTAL=$(( PASS + FAIL + SKIP ))
echo ""
echo -e "${BLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RST}"
echo -e "${BLD}  Results: ${GRN}${PASS} passed${RST}  ${RED}${FAIL} failed${RST}  ${YLW}${SKIP} skipped${RST}  / ${TOTAL} total${RST}"
echo -e "${BLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RST}"
echo ""

[[ ${FAIL} -eq 0 ]] && exit 0 || exit 1
