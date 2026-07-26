#!/usr/bin/env bash
# Host-side unit test for six-position accel calibration (apps/cal/cal_accel.c).
#
# The solver is where this procedure fails quietly: a swapped position or an
# inverted scale still yields an offset of a few hundredths and a scale near
# 1.0, and the only symptom on hardware is slow drift. So it is driven with a
# known error injected and checked for recovering exactly that.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

mkdir -p "$OUT/nuttx"
: > "$OUT/nuttx/config.h"

cp "$REPO/apps/cal/cal_accel.h" "$OUT/"
cp "$REPO/apps/cal/cal_accel.c" "$OUT/"
cp "$REPO/tests/cal_accel_test.c" "$OUT/test.c"

cc -std=c11 -Wall -Wextra -Wno-unused-parameter -I"$OUT" -DFAR= \
   -DCAL_ACCEL_HOST_TEST -o "$OUT/acctest" "$OUT/test.c" "$OUT/cal_accel.c" -lm
"$OUT/acctest"
