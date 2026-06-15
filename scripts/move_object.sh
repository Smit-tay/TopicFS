#!/usr/bin/env bash
# move_object.sh
# Copyright 2025 Jack Sidman Smith
# Licensed under the MIT License.
#
# Picks an object at position 1 and places it at position 2
# using smooth_move for all motion.
#
# Usage: bash move_object.sh <x1> <y1> <z1> <x2> <y2> <z2>
#
# Sequence:
#   1. Smooth move to above pick position (z + APPROACH_LIFT)
#   2. Smooth move down to pick position
#   3. Pump ON — pick up object
#   4. Smooth move up to approach height
#   5. Smooth move to above place position
#   6. Smooth move down to place position
#   7. Pump OFF — release object
#   8. Smooth move up (retract)
#   9. Smooth move to reset position

set -euo pipefail

MOUNT="${FUSE_MOUNT:-/home/jack/fuse_mount}"
SWIFTPRO="$MOUNT/swiftpro"

# ── Arguments ─────────────────────────────────────────────────────────────────
if [[ $# -ne 6 ]]; then
    echo "Usage: bash move_object.sh <x1> <y1> <z1> <x2> <y2> <z2>" >&2
    exit 1
fi

PICK_X=$1;  PICK_Y=$2;  PICK_Z=$3
PLACE_X=$4; PLACE_Y=$5; PLACE_Z=$6

# ── Tunable constants ─────────────────────────────────────────────────────────
APPROACH_LIFT=40.0    # mm above pick/place Z for approach/retract
APPROACH_Z_MAX=180.0  # mm maximum approach height (IK limit)
SPEED=120.0           # mm/s
LIFT=50.0             # mm via-point lift for smooth_move arcs
PUMP_SETTLE=0.8       # seconds after pump on/off before moving
RESET_X=200.0
RESET_Y=0.0
RESET_Z=150.0

# ── Helpers ───────────────────────────────────────────────────────────────────
smooth_move() {
    local x=$1 y=$2 z=$3
    echo "{\"x\":$x,\"y\":$y,\"z\":$z,\"speed\":$SPEED,\"lift\":$LIFT,\"wait\":true}" \
        > "$SWIFTPRO/smooth_move/command"
    local resp
    resp=$(cat "$SWIFTPRO/smooth_move/response")
    local ok
    ok=$(echo "$resp" | grep -oP '"success":\s*\K(true|false)')
    if [[ "$ok" != "true" ]]; then
        echo "ERROR: smooth_move to ($x,$y,$z) failed: $resp" >&2
	echo '{"on": false}' > "$SWIFTPRO/set_pump/command"
        echo "Pump turned OFF (safety)" >&2
        exit 1
    fi
}

pump_on() {
    echo '{"on": true}' > "$SWIFTPRO/set_pump/command"
    sleep "$PUMP_SETTLE"
}

pump_off() {
    echo '{"on": false}' > "$SWIFTPRO/set_pump/command"
    sleep "$PUMP_SETTLE"
}

# ── Safety check ──────────────────────────────────────────────────────────────
echo "════════════════════════════════════════"
echo "  Pick : ($PICK_X, $PICK_Y, $PICK_Z)"
echo "  Place: ($PLACE_X, $PLACE_Y, $PLACE_Z)"
echo "════════════════════════════════════════"
echo "Press Enter to start, Ctrl-C to abort..."
read -r

PICK_APPROACH_Z=$(python3 -c "print(min($PICK_Z + $APPROACH_LIFT, $APPROACH_Z_MAX))")
PLACE_APPROACH_Z=$(python3 -c "print(min($PLACE_Z + $APPROACH_LIFT, $APPROACH_Z_MAX))")

# ── Pick sequence ─────────────────────────────────────────────────────────────
echo "→ Approach pick..."
smooth_move "$PICK_X" "$PICK_Y" "$PICK_APPROACH_Z"

echo "→ Descend to pick..."
smooth_move "$PICK_X" "$PICK_Y" "$PICK_Z"

echo "→ Pump ON..."
pump_on

echo "→ Retract from pick..."
smooth_move "$PICK_X" "$PICK_Y" "$PICK_APPROACH_Z"

# ── Place sequence ────────────────────────────────────────────────────────────
echo "→ Approach place..."
smooth_move "$PLACE_X" "$PLACE_Y" "$PLACE_APPROACH_Z"

echo "→ Descend to place..."
smooth_move "$PLACE_X" "$PLACE_Y" "$PLACE_Z"

echo "→ Pump OFF..."
pump_off

echo "→ Retract from place..."
smooth_move "$PLACE_X" "$PLACE_Y" "$PLACE_APPROACH_Z"

# ── Reset ─────────────────────────────────────────────────────────────────────
echo "→ Return to reset..."
smooth_move "$RESET_X" "$RESET_Y" "$RESET_Z"

echo ""
echo "✓ Done."
