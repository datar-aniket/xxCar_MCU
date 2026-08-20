#!/usr/bin/env bash
# Host-side tests for BMI055 startup period acquisition and tracking.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -Wall -Wextra -Werror \
   -I"$REPO/boards/fmuv6c/src" \
   -o "$OUT/bmi055_timing_test" \
   "$REPO/tests/bmi055_timing_test.c" \
   "$REPO/boards/fmuv6c/src/bmi055_timing.c"
"$OUT/bmi055_timing_test"
