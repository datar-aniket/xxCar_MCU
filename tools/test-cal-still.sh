#!/usr/bin/env bash
# Host-side unit test for the stillness detector (apps/cal/cal_still.c).
#
# This is the gate on every captured orientation: declaring still too early
# smears a moving gravity vector into the average with no outward sign of
# being wrong, and never declaring still makes calibration impossible to
# finish. Both are cheap to provoke here and painful to diagnose on a bench
# with a board in your hand.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

mkdir -p "$OUT/nuttx"
: > "$OUT/nuttx/config.h"

cp "$REPO/apps/cal/cal_still.h" "$OUT/"
cp "$REPO/apps/cal/cal_still.c" "$OUT/"
cp "$REPO/tests/cal_still_test.c" "$OUT/test.c"

cc -std=c11 -Wall -Wextra -Wno-unused-parameter -I"$OUT" -DFAR= \
   -DCAL_STILL_HOST_TEST -o "$OUT/stilltest" "$OUT/test.c" "$OUT/cal_still.c" -lm
"$OUT/stilltest"
