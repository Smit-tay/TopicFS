#!/usr/bin/env bash
# record_positions.sh
# Copyright 2025 Jack Sidman Smith
# Licensed under the MIT License.
#
# Records two arm positions using the physical learn mode button.
#
# IMPORTANT: True learn mode (free-move with live position reporting)
# requires the physical button on the arm base. API servo detach does NOT
# work the same way — firmware stops updating position when detached via API.
#
# This script reads joint angles (which ARE updated in learn mode) and
# uses our FK to compute XYZ. This is more accurate than firmware position
# since it uses our calibrated kinematic model.
#
# Usage: source record_positions.sh
#   (source rather than execute so POS1/POS2 remain in your shell)
#
# Output variables:
#   POS1_X, POS1_Y, POS1_Z  — pick position (mm, firmware frame)
#   POS2_X, POS2_Y, POS2_Z  — place position (mm, firmware frame)

MOUNT="${FUSE_MOUNT:-/home/jack/fuse_mount}"
SWIFTPRO="$MOUNT/swiftpro"

# ── Read joint angles and compute FK via compute_fk service ──────────────────
read_fk_position() {
    # Read current joint angles (radians)
    local js_json
    js_json=$(cat "$MOUNT/joint_states/latest" 2>/dev/null)

    # Extract radians
    local j1_rad j2_rad j3_rad
    j1_rad=$(echo "$js_json" | grep -oP '"position":\[.*?\]' \
             | grep -oP '[\d.\-E]+' | sed -n '1p')
    j2_rad=$(echo "$js_json" | grep -oP '"position":\[.*?\]' \
             | grep -oP '[\d.\-E]+' | sed -n '2p')
    j3_rad=$(echo "$js_json" | grep -oP '"position":\[.*?\]' \
             | grep -oP '[\d.\-E]+' | sed -n '3p')

    # Convert to degrees
    local j1 j2 j3
    j1=$(python3 -c "import math; print(f'{math.degrees($j1_rad):.4f}')")
    j2=$(python3 -c "import math; print(f'{math.degrees($j2_rad):.4f}')")
    j3=$(python3 -c "import math; print(f'{math.degrees($j3_rad):.4f}')")

    # Call compute_fk service
    echo "{\"j1\": $j1, \"j2\": $j2, \"j3\": $j3}" \
        > "$SWIFTPRO/compute_fk/command"
    sleep 0.3
    local fk_json
    fk_json=$(cat "$SWIFTPRO/compute_fk/response" 2>/dev/null)

    echo "$fk_json"
}

json_field() {
    echo "$1" | grep -oP "\"$2\":\s*\K[^,}]+" | tr -d '"' | xargs
}

echo "════════════════════════════════════════"
echo "  UArm Swift Pro — Position Recorder"
echo "════════════════════════════════════════"
echo ""
echo "  This script uses joint angles + FK for position reading."
echo "  Joint encoders remain active during learn mode."
echo ""
echo "  ► Press the LEARN MODE BUTTON on the arm base."
echo "    The arm will become free-moving."
echo ""
echo "  Press Enter when learn mode is active..."
read -r

# ── Position 1 (pick) ─────────────────────────────────────────────────────────
echo ""
echo "► Move arm to PICK location (position 1)"
echo "  Press Enter when in position..."
read -r

sleep 0.3
fk1=$(read_fk_position)
POS1_X=$(json_field "$fk1" "x")
POS1_Y=$(json_field "$fk1" "y")
POS1_Z=$(json_field "$fk1" "z")
echo "  ✓ POS1: x=$POS1_X  y=$POS1_Y  z=$POS1_Z"

# ── Position 2 (place) ────────────────────────────────────────────────────────
echo ""
echo "► Move arm to PLACE location (position 2)"
echo "  Press Enter when in position..."
read -r

sleep 0.3
fk2=$(read_fk_position)
POS2_X=$(json_field "$fk2" "x")
POS2_Y=$(json_field "$fk2" "y")
POS2_Z=$(json_field "$fk2" "z")
echo "  ✓ POS2: x=$POS2_X  y=$POS2_Y  z=$POS2_Z"

# ── Disable learn mode ────────────────────────────────────────────────────────
echo ""
echo "  ► Press the LEARN MODE BUTTON again to exit learn mode."
echo "    Note: the arm will jump slightly when learn mode is disabled."
echo ""
echo "  Press Enter when learn mode is disabled..."
read -r

echo ""
echo "════════════════════════════════════════"
echo "  Recorded positions:"
echo "  POS1 (pick) : ($POS1_X, $POS1_Y, $POS1_Z)"
echo "  POS2 (place): ($POS2_X, $POS2_Y, $POS2_Z)"
echo "════════════════════════════════════════"
echo ""
echo "Run: bash move_object.sh $POS1_X $POS1_Y $POS1_Z $POS2_X $POS2_Y $POS2_Z"
