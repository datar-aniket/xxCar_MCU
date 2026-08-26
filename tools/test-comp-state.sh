#!/usr/bin/env bash
# Host-side test for the VEHICLE_STATE assembly.
#
# The frame conversions are the part that fails silently: a velocity rotated
# by the quaternion instead of its transpose is still a velocity, and at zero
# yaw the two agree exactly. Every meaningful case here runs at a non-trivial
# attitude.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
   -I"$REPO/apps/companion" \
   -o "$OUT/test" "$REPO/tests/comp_state_test.c" \
   "$REPO/apps/companion/comp_state.c" -lm
"$OUT/test"

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
   -fsanitize=undefined,address -fno-sanitize-recover=all \
   -I"$REPO/apps/companion" \
   -o "$OUT/test-san" "$REPO/tests/comp_state_test.c" \
   "$REPO/apps/companion/comp_state.c" -lm
"$OUT/test-san"
