#!/usr/bin/env bash
# collision_probe.sh — probe the J2/J3 safe corridor boundary
# Moves through a grid of (J2, J3) points and records whether each is safe.
# Output: collision_probe.csv
#
# Copyright 2025 Jack Sidman Smith — MIT License

set -euo pipefail

MOUNT=${1:-/home/jack/fuse_mount}
RESET_CMD="$MOUNT/swiftpro/reset/command"
ANGLES_CMD="$MOUNT/swiftpro/set_servo_angles/command"
EE_POS="$MOUNT/swiftpro/end_effector_position/latest"
OUTPUT="collision_probe.csv"
J1=89.3

# ── Points to probe: (J2, J3) pairs sweeping the boundary region ─────────────
# Arranged so we approach from safe side (high J2, low J3) toward boundary
POINTS=(
  "80 0"
  "70 0"
  "60 0"
  "60 5"
  "60 10"
  "50 5"
  "50 10"
  "50 15"
  "40 10"
  "40 15"
  "40 20"
  "40 25"
  "30 15"
  "30 20"
  "30 26"
  "25 25"
  "25 30"
  "21 28"
  "21 32"
  "21 35"
)

echo "J1,J2,J3,x,y,z,safe" > "$OUTPUT"
echo "Collision boundary probe — J1 fixed at ${J1}°"
echo "Output: $OUTPUT"
echo "Press Ctrl+C to abort at any time"
echo ""

# ── Reset to start ────────────────────────────────────────────────────────────
echo "Resetting arm..."
echo '{"speed": 2000}' > "$RESET_CMD"
sleep 5

total=${#POINTS[@]}
count=0

for pt in "${POINTS[@]}"; do
  j2=$(echo $pt | awk '{print $1}')
  j3=$(echo $pt | awk '{print $2}')
  count=$((count + 1))

  echo "────────────────────────────────────────"
  echo "Point $count/$total — J2=${j2}°  J3=${j3}°"

  # Command the move
  echo "{\"j1\": ${J1}, \"j2\": ${j2}, \"j3\": ${j3}, \"speed\": 500, \"wait\": true}" \
    > "$ANGLES_CMD"
  sleep 4

  # Read position
  pos=$(cat "$EE_POS" 2>/dev/null || echo '{"point":{"x":0,"y":0,"z":0}}')
  x=$(echo "$pos" | grep -o '"x":[^,}]*' | head -1 | cut -d: -f2)
  y=$(echo "$pos" | grep -o '"y":[^,}]*' | head -1 | cut -d: -f2)
  z=$(echo "$pos" | grep -o '"z":[^,}]*' | head -1 | cut -d: -f2)

  echo "  Position: x=${x}  y=${y}  z=${z}"
  echo ""

  # Ask user
  read -rp "  Collision? [y/N]: " answer
  answer=${answer:-n}

  if [[ "$answer" =~ ^[Yy]$ ]]; then
    echo "J1,J2,J3=${J1},${j2},${j3}  — COLLISION"
    echo "${J1},${j2},${j3},${x},${y},${z},collision" >> "$OUTPUT"
    echo ""
    echo "Collision recorded. Resetting and stopping."
    echo '{"speed": 2000}' > "$RESET_CMD"
    sleep 4
    echo ""
    echo "Results so far in $OUTPUT:"
    cat "$OUTPUT"
    exit 0
  else
    echo "${J1},${j2},${j3},${x},${y},${z},safe" >> "$OUTPUT"
    echo "  ✓ Safe — recorded"
  fi

done

echo ""
echo "════════════════════════════════════════"
echo "All $total points probed without collision."
echo "Results in $OUTPUT:"
cat "$OUTPUT"
