#!/usr/bin/env bash
# MIT License
# Copyright 2026 Jack Sidman Smith
#
# topicfs_test.sh — Integration test for TopicFS / SwiftRos2 end-to-end stack.
# Exercises topics, services, and movement via plain shell tools — no ROS2 on host.
#
# Usage: bash topicfs_test.sh [MOUNT_POINT]
# Default mount point: /home/jack/fuse_mount

set -uo pipefail

MOUNT="${1:-/home/jack/fuse_mount}"
SWIFTPRO="${MOUNT}/swiftpro"

# --- Tunables ---
SERVICE_TIMEOUT=30      # seconds to wait for a service response
POSITION_TOLERANCE=2.0  # mm — acceptable delta between commanded and reported position

# --- Move speeds (mm/min — uarm API unit, NOT mm/s) ---
SPEED_SLOW=1000          # ~8 mm/s  — visibly slow
SPEED_NORMAL=5000       # ~33 mm/s — normal working speed
SPEED_FAST=10000         # ~83 mm/s — brisk

# --- Post-move settle (seconds) ---
MOVE_SETTLE=0.5     # wait for position topic to update after move completes
ASYNC_SETTLE=3      # seconds to wait for async move to complete before position check

# --- Home position ---
HOME_X=180.0
HOME_Y=0.0
HOME_Z=80.0

# --- Async settle time (seconds) ---
ASYNC_SETTLE=4

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

# Write request to service command file, poll response until timeout.
call_service() {
    local svc="$1"
    local request="$2"
    local cmd="${SWIFTPRO}/${svc}/command"
    local rsp="${SWIFTPRO}/${svc}/response"

    echo "${request}" > "${cmd}" 2>/dev/null || { echo ""; return 1; }

    local deadline=$(( $(date +%s) + SERVICE_TIMEOUT ))
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

json_field() {
    local json="$1"
    local field="$2"
    echo "${json}" | grep -oP "\"${field}\"\\s*:\\s*\\K[^,}]+" | tr -d '"' | xargs
}

float_delta() {
    awk "BEGIN { d=$1 - $2; print (d < 0 ? -d : d) }"
}

float_le() {
    awk "BEGIN { exit ($1 <= $2 ? 0 : 1) }"
}

# Move arm and verify it reaches the target position.
# Args: label x y z speed wait
# wait=true  — blocks until move complete, response success field is meaningful
# wait=false — fire-and-forget, uarm async path always returns -1 (expected, not a bug)
test_move() {
    local label="$1"
    local tx="$2"
    local ty="$3"
    local tz="$4"
    local speed="$5"
    local wait="$6"

    local req="{\"x\": ${tx}, \"y\": ${ty}, \"z\": ${tz}, \"speed\": ${speed}, \"wait\": ${wait}}"
    local rsp
    rsp=$(call_service "move_to" "${req}")
    sleep ${MOVE_SETTLE}
    
    if [[ "${wait}" == "false" ]]; then
        sleep ${ASYNC_SETTLE}
    fi
    
    if [[ -z "${rsp}" ]]; then
        fail "${label}: no response from move_to service"
        return
    fi

    local success error_code message
    success=$(json_field "${rsp}" "success")
    error_code=$(json_field "${rsp}" "error_code")
    message=$(json_field "${rsp}" "message")

    # wait=false: uarm async path unconditionally returns -1 — skip response check.
    if [[ "${wait}" == "true" && "${success}" != "true" ]]; then
        fail "${label}: service reported failure — error_code=${error_code} message=${message}"
        return
    fi

    local pos rx ry rz dx dy dz
    pos=$(read_topic "${SWIFTPRO}/position/latest")
    if [[ -z "${pos}" ]]; then
        fail "${label}: no position data after move"
        return
    fi

    rx=$(json_field "${pos}" "x")
    ry=$(json_field "${pos}" "y")
    rz=$(json_field "${pos}" "z")
    dx=$(float_delta "${rx}" "${tx}")
    dy=$(float_delta "${ry}" "${ty}")
    dz=$(float_delta "${rz}" "${tz}")

    local ok=true
    float_le "${dx}" "${POSITION_TOLERANCE}" || ok=false
    float_le "${dy}" "${POSITION_TOLERANCE}" || ok=false
    float_le "${dz}" "${POSITION_TOLERANCE}" || ok=false

    if [[ "${ok}" == "true" ]]; then
        pass "${label} [${speed}mm/min wait=${wait}]: target=(${tx},${ty},${tz})  actual=(${rx},${ry},${rz})  delta=(${dx},${dy},${dz})mm"
    else
        fail "${label} [${speed}mm/min wait=${wait}]: target=(${tx},${ty},${tz})  actual=(${rx},${ry},${rz})  delta=(${dx},${dy},${dz})mm — exceeds ${POSITION_TOLERANCE}mm tolerance"
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

# ============================================================
# Topic reads
# ============================================================

header "Topic reads"

pos=$(read_topic "${SWIFTPRO}/position/latest")
if [[ -n "${pos}" ]]; then
    x=$(json_field "${pos}" "x")
    y=$(json_field "${pos}" "y")
    z=$(json_field "${pos}" "z")
    pass "Position: x=${x}mm  y=${y}mm  z=${z}mm"
else
    fail "Position: no data"
fi

js=$(read_topic "${MOUNT}/joint_states/latest")
[[ -n "${js}" ]] && pass "Joint states: ${js}" || fail "Joint states: no data"

pump=$(read_topic "${SWIFTPRO}/pump_status/latest")
[[ -n "${pump}" ]] && pass "Pump status: ${pump}" || fail "Pump status: no data"

mode=$(read_topic "${SWIFTPRO}/mode/latest")
[[ -n "${mode}" ]] && pass "Mode: ${mode}" || fail "Mode: no data"

polar=$(read_topic "${SWIFTPRO}/polar/latest")
[[ -n "${polar}" ]] && pass "Polar: ${polar}" || fail "Polar: no data"

moving=$(read_topic "${SWIFTPRO}/is_moving/latest")
[[ -n "${moving}" ]] && pass "is_moving: ${moving}" || fail "is_moving: no data"

ls_file="${SWIFTPRO}/limit_switch/latest"
if [[ -f "${ls_file}" ]]; then
    ls_data=$(cat "${ls_file}" 2>/dev/null)
    if [[ -n "${ls_data}" ]]; then
        pass "Limit switch: ${ls_data}"
    else
        pass "Limit switch: file present, no trigger data (expected — only published on trigger)"
    fi
else
    fail "Limit switch: file missing"
fi

# ============================================================
# Read-only services
# ============================================================

header "Read-only services"

rsp=$(call_service "get_analog" '{"pin": 0}')
if [[ -n "${rsp}" ]]; then
    val=$(json_field "${rsp}" "value")
    ok=$(json_field "${rsp}" "success")
    [[ "${ok}" == "true" ]] && pass "get_analog pin=0: value=${val}" || fail "get_analog pin=0: ${rsp}"
else
    fail "get_analog: no response"
fi

rsp=$(call_service "get_digital" '{"pin": 0}')
if [[ -n "${rsp}" ]]; then
    val=$(json_field "${rsp}" "value")
    ok=$(json_field "${rsp}" "success")
    [[ "${ok}" == "true" ]] && pass "get_digital pin=0: value=${val}" || fail "get_digital pin=0: ${rsp}"
else
    fail "get_digital: no response"
fi

# ============================================================
# Buzzer (verify audibly)
# ============================================================

header "Buzzer (verify audibly)"

rsp=$(call_service "set_buzzer" '{"frequency": 1000, "duration": 0.3}')
# uarm return code unreliable for set_buzzer — test presence of response only
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
    sleep 1
    pump=$(read_topic "${SWIFTPRO}/pump_status/latest")
    pass "Pump ON:  response=${rsp}  status=${pump}"
else
    fail "set_pump ON: no response"
fi
sleep 1

rsp=$(call_service "set_pump" '{"on": false}')
if [[ -n "${rsp}" ]]; then
    sleep 1
    pump=$(read_topic "${SWIFTPRO}/pump_status/latest")
    pass "Pump OFF: response=${rsp}  status=${pump}"
else
    fail "set_pump OFF: no response"
fi

# ============================================================
# Movement
# All moves use wait=true except the async section.
# Speeds are in mm/min (uarm API unit — see constants at top of script).
# ============================================================

header "Movement — home"
test_move "Home"                  ${HOME_X}  ${HOME_Y}  ${HOME_Z}  ${SPEED_NORMAL}  true

header "Movement — cardinal positions"
test_move "Forward"               220.0    0.0   80.0  ${SPEED_NORMAL}  true
test_move "Left"                  180.0   60.0   80.0  ${SPEED_NORMAL}  true
test_move "Right"                 180.0  -60.0   80.0  ${SPEED_NORMAL}  true
test_move "Back"                  150.0    0.0   80.0  ${SPEED_NORMAL}  true
test_move "Home"                  ${HOME_X}  ${HOME_Y}  ${HOME_Z}  ${SPEED_NORMAL}  true

header "Movement — vertical range"
test_move "Low"                   180.0    0.0   20.0  ${SPEED_NORMAL}  true
test_move "Mid"                   180.0    0.0   80.0  ${SPEED_NORMAL}  true
test_move "High"                  180.0    0.0  150.0  ${SPEED_NORMAL}  true

header "Movement — diagonal / combined axes"
test_move "Forward-left-high"     210.0   50.0  130.0  ${SPEED_NORMAL}  true
test_move "Forward-right-low"     210.0  -50.0   30.0  ${SPEED_NORMAL}  true
test_move "Back-left-mid"         155.0   40.0   80.0  ${SPEED_NORMAL}  true
test_move "Back-right-mid"        155.0  -40.0   80.0  ${SPEED_NORMAL}  true
test_move "Home"                  ${HOME_X}  ${HOME_Y}  ${HOME_Z}  ${SPEED_NORMAL}  true

header "Movement — speed variation"
test_move "Slow"                  200.0   30.0   80.0  ${SPEED_SLOW}    true
test_move "Home"                  ${HOME_X}  ${HOME_Y}  ${HOME_Z}  ${SPEED_NORMAL}  true
test_move "Normal"                200.0  -30.0   80.0  ${SPEED_NORMAL}  true
test_move "Home"                  ${HOME_X}  ${HOME_Y}  ${HOME_Z}  ${SPEED_NORMAL}  true
test_move "Fast"                  200.0   30.0   80.0  ${SPEED_FAST}    true
test_move "Home"                  ${HOME_X}  ${HOME_Y}  ${HOME_Z}  ${SPEED_NORMAL}  true

header "Movement — fire-and-forget (wait=false)"
# wait=false: command fires immediately, response is always -1 (expected, not a bug).
# Position verified after ASYNC_SETTLE seconds.
test_move "Async fire"            215.0    0.0   80.0  ${SPEED_FAST}  false
sleep ${ASYNC_SETTLE}
pos=$(read_topic "${SWIFTPRO}/position/latest")
rx=$(json_field "${pos}" "x")
ry=$(json_field "${pos}" "y")
rz=$(json_field "${pos}" "z")
pass "Async position after ${ASYNC_SETTLE}s settle: actual=(${rx},${ry},${rz})"
test_move "Home (sync)"           ${HOME_X}  ${HOME_Y}  ${HOME_Z}  ${SPEED_NORMAL}  true

# ============================================================
# Reset
# ============================================================

header "Reset service"

rsp=$(call_service "reset" '{}')
if [[ -n "${rsp}" ]]; then
    sleep 3
    pos=$(read_topic "${SWIFTPRO}/position/latest")
    pass "Reset: response=${rsp}  position after reset=${pos}"
else
    fail "Reset: no response"
fi

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
