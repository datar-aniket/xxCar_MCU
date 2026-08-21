#!/usr/bin/env bash
# Host-side unit test for the full 3D magnetometer ellipsoid calibration.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cp "$REPO/apps/cal/cal_mag.h" "$OUT/"
cp "$REPO/apps/cal/cal_mag.c" "$OUT/"
cp "$REPO/tests/cal_mag_test.c" "$OUT/test.c"

cc -std=c11 -Wall -Wextra -Werror -I"$OUT" -DFAR= \
   -DCAL_MAG_HOST_TEST -o "$OUT/magtest" "$OUT/test.c" \
   "$OUT/cal_mag.c" -lm
"$OUT/magtest"

cc -std=c11 -Wall -Wextra -Werror -I"$OUT" -DFAR= \
   -DCAL_MAG_HOST_TEST -fsanitize=undefined,float-divide-by-zero \
   -fno-sanitize-recover=all -o "$OUT/magtest-ubsan" "$OUT/test.c" \
   "$OUT/cal_mag.c" -lm
"$OUT/magtest-ubsan"
