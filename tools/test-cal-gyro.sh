#!/usr/bin/env bash
# Host-side unit test for gyro zero-rate bias (apps/cal/cal_gyro.c).
#
# A gyro bias calibration has no gravity to check itself against, so the only
# defence against measuring the wrong thing is refusing conditions that cannot
# produce a bias. The one that hides is a board turning at a CONSTANT rate: it
# is perfectly steady by any standard-deviation test, and its rotation would be
# stored as zero-rate offset and subtracted from every later reading.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cp "$REPO/apps/cal/cal_gyro.h" "$OUT/"
cp "$REPO/apps/cal/cal_gyro.c" "$OUT/"
cp "$REPO/tests/cal_gyro_test.c" "$OUT/test.c"

cc -std=c11 -Wall -Wextra -Wno-unused-parameter \
   -DCAL_GYRO_HOST_TEST -DFAR= \
   -I"$OUT" -o "$OUT/calgyrotest" "$OUT/test.c" "$OUT/cal_gyro.c" -lm
"$OUT/calgyrotest"
