#!/usr/bin/env bash
# Host-side check of the FDCAN message RAM placement.
#
# Overlapping regions are not a build error and not a runtime error either -
# the hardware just corrupts frames. This runs the arithmetic on the host so
# a bad offset is caught before it reaches a bus.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -Wall -Wextra -Werror \
   -I"$REPO/boards/fmuv6c/src" \
   -o "$OUT/test" "$REPO/tests/fdcan_ram_test.c"
"$OUT/test"
