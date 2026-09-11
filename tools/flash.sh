#!/usr/bin/env bash
# Flash a board-specific image through its PX4-compatible bootloader.
# The bootloader only exposes its serial port for a few seconds after reset,
# so unplug/replug the board when prompted.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BOARD_ARG="${1:-${BOARD:-pixhawk6c}}"
case "${BOARD_ARG,,}" in
  pixhawk6c|pixhawk-6c|fmuv6c)
    BOARD="pixhawk6c"
    LABEL="Pixhawk 6C"
    IMAGE="${2:-$REPO/build/pixhawk6c/xxcar_pixhawk6c.px4}"
    PORTS="/dev/pixhawk_6c,/dev/ttyACM*"
    ;;
  matek|matekh743|matekh743-slim-v4|h743-slim-v4)
    BOARD="matekh743"
    LABEL="MATEKSYS H743-SLIM-V4"
    IMAGE="${2:-$REPO/build/matekh743/xxcar_matekh743.px4}"
    PORTS="/dev/matekh743,/dev/ttyACM*"
    ;;
  *)
    echo "error: unknown board '$BOARD_ARG' (use pixhawk6c or matekh743)" >&2
    exit 2
    ;;
esac

[ -f "$IMAGE" ] || { echo "error: $IMAGE not found. run tools/build.sh first"; exit 1; }

echo ">> Ready to flash: $IMAGE"
echo ">> Now UNPLUG and RE-PLUG the $LABEL USB to enter the bootloader window..."
python3 "$REPO/tools/px4/px4_uploader.py" --port "$PORTS" "$IMAGE"
