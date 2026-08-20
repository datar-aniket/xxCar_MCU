#!/usr/bin/env bash
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -O2 -Wall -Wextra -Werror \
   -DIMU_INTEGRATOR_HOST_TEST -DFAR= \
   -I"$REPO/apps/imu_delta" \
   -o "$OUT/imu-integrator-test" \
   "$REPO/tests/imu_integrator_test.c" \
   "$REPO/apps/imu_delta/imu_integrator.c" -lm

"$OUT/imu-integrator-test"
