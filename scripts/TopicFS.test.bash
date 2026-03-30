#!/usr/bin/env bash
# MIT License
# Copyright 2026 Jack Sidman Smith
#
# TopicFS_test.bash — Integration test for TopicFS / SwiftRos2 end-to-end stack.
# Exercises topics, services, actions, and movement via plain shell tools.
# No ROS2 on host required.
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

# --- Move speeds (mm/min) ---
SPEED_SLOW=1000
SPEED_NORMAL=5000
SPEED_FAST=10000

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
    ((PASS++))
}

fail() {
    echo -e "  ${RED}✗${RST}  $1"
    ((FAIL++))
}

skip() {
    echo -e "  ${YLW}–${RST}  $1 ${YLW}(skipped)${RST}"
    ((SKIP++))
}

read_topic() {
    local file="$1"
    local content
    content=$(cat "${file}" 2>/dev/null) || { echo ""; return 1; }
    [[ -z "${content}" ]] && { echo ""; return 1; }
    echo "${content}"
}

json_field() {
    local json="$1"
    local field="$2"
    echo "${json}" | grep -oP "\"${field}\"\\s*:\\s*\\K[^,}]+" | tr -d '"' | xargs
}

float_delta() {
    awk "BEGIN { d=$1 - $2; print (d < 0 ? -d : d) }"
}

float_le() {
    awk "BEGIN { exit (($1) <= ($2) ? 0 : 1) }"
}

euclidean_dist() {
    awk "BEGIN { dx=($1)-($4); dy=($2)-($5); dz=($3)-($6); print sqrt(dx*dx+dy*dy+dz*dz) }"
}

# Write request to a plain service, poll response until timeout.
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
# Returns 0 on success (goal succeeded within tolerance), 1 on failure.
# Args: label x y z speed
send_move_goal() {
    local label="$1"
    local tx="$2"
    local ty="$3"
    local tz="$4"
    local speed="$5"

    local uuid="${UUID_COUNTER}"
    ((UUID_COUNTER++))

    # Build UUID array — first byte is the counter, rest zeros
    local uuid_arr="[${uuid},0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]"
    local goal="{\"goal_id\":{\"uuid\":${uuid_arr}},\"goal\":{\"x\":${tx},\"y\":${ty},\"z\":${tz},\"speed\":${speed}}}"

    # Send goal
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
    local result
    result=$(cat "${MOVE_ARM}/get_result/response" 2>/dev/null)
    local success
    success=$(json_field "${result}" "success")

    # Verify position
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

if [[ ! -d "${MOUNT}" ]]; then
    fail "Mount point ${MOUNT} does not exist"
    echo -e "\n${RED}Aborting — mount point missing.${RST}"
    exit 1
fi

if [[ ! -d "${SWIFTPRO}" ]]; then
    fail "SwiftPro directory not found at ${SWIFTPRO} — is SwiftRos2 running?"
    echo -e "\n${RED}Aborting — SwiftRos2 not visible via TopicFS.${RST}"
    exit 1
fi

pass "Mount point accessible: ${MOUNT}"

connected=$(read_topic "${SWIFTPRO}/connected/latest")
if [[ $(json_field "${connected}" "data") == "true" ]]; then
    pass "Arm connected: ${connected}"
else
    fail "Arm not connected: '${connected}'"
    echo -e "\n${RED}Aborting — arm not connected.${RST}"
    exit 1
fi

if [[ ! -d "${MOVE_ARM}" ]]; then
    fail "move_arm action not found at ${MOVE_ARM} — is action support working?"
    echo -e "\n${RED}Aborting — move_arm action missing.${RST}"
    exit 1
fi
pass "move_arm action directory found"

# ============================================================
# Topic reads
# ============================================================

header "Topic reads"

pos=$(read_topic "${SWIFTPRO}/position/latest")
if [[ -n "${pos}" ]]; then
    x=$(json_field "${pos}" "x"); y=$(json_field "${pos}" "y"); z=$(json_field "${pos}" "z")
    pass "Position: x=${x}mm  y=${y}mm  z=${z}mm"
else
    fail "Position: no data"
fi

js=$(read_topic "${MOUNT}/joint_states/latest")
[[ -n "${js}" ]] && pass "Joint states: present" || fail "Joint states: no data"

pump=$(read_topic "${SWIFTPRO}/pump_status/latest")
[[ -n "${pump}" ]] && pass "Pump status: ${pump}" || fail "Pump status: no data"

mode=$(read_topic "${SWIFTPRO}/mode/latest")
[[ -n "${mode}" ]] && pass "Mode: ${mode}" || fail "Mode: no data"

polar=$(read_topic "${SWIFTPRO}/polar/latest")
[[ -n "${polar}" ]] && pass "Polar: present" || fail "Polar: no data"

moving=$(read_topic "${SWIFTPRO}/is_moving/latest")
[[ -n "${moving}" ]] && pass "is_moving: ${moving}" || fail "is_moving: no data"

ls_file="${SWIFTPRO}/limit_switch/latest"
if [[ -f "${ls_file}" ]]; then
    ls_data=$(cat "${ls_file}" 2>/dev/null)
    if [[ -n "${ls_data}" ]]; then
        pass "Limit switch: ${ls_data}"
    else
        pass "Limit switch: file present, no trigger data (expected)"
    fi
else
    fail "Limit switch: file missing"
fi

# ============================================================
# Action interface check
# ============================================================

header "Action interface"

[[ -d "${MOVE_ARM}/send_goal" ]] && pass "send_goal directory present" || fail "send_goal directory missing"
[[ -d "${MOVE_ARM}/cancel" ]]    && pass "cancel directory present"    || fail "cancel directory missing"
[[ -d "${MOVE_ARM}/get_result" ]] && pass "get_result directory present" || fail "get_result directory missing"
[[ -d "${MOVE_ARM}/feedback" ]]  && pass "feedback directory present"  || fail "feedback directory missing"
[[ -d "${MOVE_ARM}/status" ]]    && pass "status directory present"    || fail "status directory missing"

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
# Pump
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

# ============================================================
# Reset
# ============================================================

header "Reset service"

rsp=$(call_service "reset" '{}' 60)
if [[ -n "${rsp}" ]]; then
    sleep 3
    pos=$(read_topic "${SWIFTPRO}/position/latest")
    pass "Reset: response=${rsp}  position after reset=${pos}"
else
    fail "Reset: no response (arm may have moved — check physically)"
fi

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
# Stress test — 20 consecutive action goals
# Rapid sequence of moves, interleaved with service calls.
# Tests executor stability under sustained load.
# ============================================================

header "Stress test — 20 consecutive goals"

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
    "185.0  20.0  130.0"
    "200.0 -10.0   60.0"
    "220.0  40.0   80.0"
    "165.0 -50.0  100.0"
    "195.0  30.0   40.0"
    "210.0 -30.0  120.0"
    "180.0  50.0   70.0"
    "200.0 -20.0   90.0"
    "190.0  10.0  110.0"
    "200.0   0.0  150.0"
)

stress_pass=0
stress_fail=0

for i in "${!STRESS_POINTS[@]}"; do
    read -r sx sy sz <<< "${STRESS_POINTS[$i]}"
    n=$(( i + 1 ))

    if send_move_goal "Stress #${n}" "${sx}" "${sy}" "${sz}" ${SPEED_FAST}; then
        ((stress_pass++))
    else
        ((stress_fail++))
    fi

    # Every 5 moves, fire a service call to verify executor health
    if (( n % 5 == 0 )); then
        rsp=$(call_service "get_digital" '{"pin": 0}' 5)
        if [[ -n "${rsp}" && "${rsp}" != *"error"* ]]; then
            echo -e "  ${CYN}↳ executor health check #$((n/5)): get_digital OK${RST}"
        else
            echo -e "  ${RED}↳ executor health check #$((n/5)): get_digital FAILED — ${rsp}${RST}"
            ((stress_fail++))
            ((stress_pass--))
        fi
    fi
done

echo ""
echo -e "  Stress test: ${GRN}${stress_pass} passed${RST}  ${RED}${stress_fail} failed${RST} / 20 goals"

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
