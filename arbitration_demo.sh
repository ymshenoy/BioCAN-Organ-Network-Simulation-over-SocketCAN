#!/usr/bin/env bash
#
# arbitration_demo.sh — Step 3b: bus-load demo for ID_EMERGENCY_ALERT priority
#
# Real CAN hardware resolves simultaneous transmission by bitwise arbitration
# on the wire: the lowest ID (most dominant bits) always wins, so a lower-ID
# frame is never delayed by lower-priority traffic. `vcan` is a software
# loopback interface, not a shared electrical bus — there is no contention to
# arbitrate, so frames are simply delivered FIFO. This script does not (and
# cannot) reproduce wire-level arbitration; it demonstrates the one thing
# that *is* true on vcan: even under heavy low-priority bus load, sending
# ID_EMERGENCY_ALERT (0x001) is not blocked or starved — it shows up
# immediately in the traffic, because vcan has no queue for it to get stuck
# behind. See the README's Step 3b note for the real-hardware caveat.
#
# Usage: ./arbitration_demo.sh [iface]

set -euo pipefail

IFACE="${1:-vcan0}"

if ! ip link show "$IFACE" &>/dev/null; then
    echo "Interface $IFACE not found. Set it up first, e.g.:" >&2
    echo "  sudo modprobe vcan && sudo ip link add dev $IFACE type vcan && sudo ip link set up $IFACE" >&2
    exit 1
fi

command -v cangen >/dev/null || { echo "cangen not found (part of can-utils)" >&2; exit 1; }
command -v cansend >/dev/null || { echo "cansend not found (part of can-utils)" >&2; exit 1; }
command -v candump >/dev/null || { echo "candump not found (part of can-utils)" >&2; exit 1; }

echo "== Step 3b arbitration demo on $IFACE =="
echo "Flooding $IFACE with low-priority frames (ID 0x7FF, ~1000/s)..."

cangen "$IFACE" -I 7FF -D r -L 8 -g 1 &
CANGEN_PID=$!
trap 'kill "$CANGEN_PID" 2>/dev/null || true' EXIT

candump -tz "$IFACE" &
CANDUMP_PID=$!
trap 'kill "$CANGEN_PID" "$CANDUMP_PID" 2>/dev/null || true' EXIT

sleep 1
echo ""
echo ">>> Sending EMERGENCY_ALERT (0x001) mid-flood now <<<"
cansend "$IFACE" 001#0102
sleep 1

kill "$CANDUMP_PID" "$CANGEN_PID" 2>/dev/null || true
wait "$CANDUMP_PID" "$CANGEN_PID" 2>/dev/null || true

echo ""
echo "== Result =="
echo "Look for the '001' line in the dump above, marked by the '>>> Sending'"
echo "line printed just before it went out. It appears immediately amid the"
echo "0x7FF flood — vcan never made it wait, because vcan has no bus for"
echo "low-priority frames to occupy. This is the one guarantee vcan *can*"
echo "demonstrate; true bitwise arbitration only exists on real CAN hardware."
