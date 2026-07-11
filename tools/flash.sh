#!/usr/bin/env bash
# Flash build/xxcar.px4 through the factory PX4 bootloader (kept intact).
# The bootloader only exposes its serial port for a few seconds after reset,
# so unplug/replug the board when prompted.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${1:-$REPO/build/xxcar.px4}"
PORTS="/dev/pixhawk_6c,/dev/ttyACM*"

[ -f "$IMAGE" ] || { echo "error: $IMAGE not found. run tools/build.sh first"; exit 1; }

echo ">> Ready to flash: $IMAGE"
echo ">> Now UNPLUG and RE-PLUG the Pixhawk 6C USB to enter the bootloader window..."
python3 "$REPO/tools/px4/px4_uploader.py" --port "$PORTS" "$IMAGE"
