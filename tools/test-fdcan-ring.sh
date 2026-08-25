#!/usr/bin/env bash
# Host-side test for the FDCAN receive ring's index arithmetic.
#
# The ring is what lets the interrupt handler and the daemon task share a
# buffer with no lock, which only holds if "full" and "empty" stay
# distinguishable from the two indices alone. An off-by-one does not crash -
# it drops or re-delivers a frame - so the boundary is checked here.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -Wall -Wextra -Werror \
   -I"$REPO/boards/fmuv6c/src" \
   -o "$OUT/test" "$REPO/tests/fdcan_ring_test.c"
"$OUT/test"
