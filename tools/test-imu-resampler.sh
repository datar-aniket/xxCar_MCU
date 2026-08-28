#!/usr/bin/env bash
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -O2 -Wall -Wextra -Werror \
   -DFAR= -I"$REPO/apps/imu_delta" \
   -o "$OUT/imu-resampler-test" \
   "$REPO/tests/imu_resampler_test.c" \
   "$REPO/apps/imu_delta/imu_resampler.c" -lm

"$OUT/imu-resampler-test"
