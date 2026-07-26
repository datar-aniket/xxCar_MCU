#!/usr/bin/env bash
# Host-side unit test for the calibration apply math (apps/imu_cal/imu_cal.c).
#
# The matrix multiply is the one piece of this subsystem that silently produces
# plausible garbage when it is wrong - a transposed index or a sign error still
# yields numbers of about the right magnitude. So it is checked against known
# exact values, on the host, before anything is flashed.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

mkdir -p "$OUT/nuttx"
: > "$OUT/nuttx/config.h"

cp "$REPO/apps/imu_cal/imu_cal.h" "$OUT/"
cp "$REPO/apps/imu_cal/imu_cal.c" "$OUT/"
cp "$REPO/tests/imu_cal_test.c"   "$OUT/test.c"

cc -std=c11 -Wall -Wextra -Wno-unused-parameter -I"$OUT" -DFAR= \
   -DIMU_CAL_HOST_TEST -o "$OUT/imucaltest" "$OUT/test.c" "$OUT/imu_cal.c" -lm
"$OUT/imucaltest"
