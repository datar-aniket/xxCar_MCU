#!/usr/bin/env bash
# Host-side unit test for magnetic heading: tilt compensation, alignment
# initialisation from accel+mag, and the gated yaw update.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -O2 -Wall -Wextra -Werror \
   -DEKF_CORE_HOST_TEST -DFAR= \
   -I"$REPO/apps/ekf3" \
   -o "$OUT/ekf-mag-test" \
   "$REPO/tests/ekf_mag_test.c" \
   "$REPO/apps/ekf3/ekf_core.c" -lm

"$OUT/ekf-mag-test"

cc -std=c11 -O1 -g -Wall -Wextra -Werror \
   -fsanitize=undefined -fno-sanitize-recover=undefined \
   -DEKF_CORE_HOST_TEST -DFAR= \
   -I"$REPO/apps/ekf3" \
   -o "$OUT/ekf-mag-ubsan" \
   "$REPO/tests/ekf_mag_test.c" \
   "$REPO/apps/ekf3/ekf_core.c" -lm

"$OUT/ekf-mag-ubsan"
