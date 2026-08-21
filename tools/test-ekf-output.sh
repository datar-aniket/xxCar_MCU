#!/usr/bin/env bash
# Host-side unit test for the EKF output predictor: re-propagation from the delayed
# filter state to the present.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -O2 -Wall -Wextra -Werror \
   -DEKF_CORE_HOST_TEST -DFAR= \
   -I"$REPO/apps/ekf3" \
   -o "$OUT/ekf-output-test" \
   "$REPO/tests/ekf_output_test.c" \
   "$REPO/apps/ekf3/ekf_core.c" -lm

"$OUT/ekf-output-test"

cc -std=c11 -O1 -g -Wall -Wextra -Werror \
   -fsanitize=undefined -fno-sanitize-recover=undefined \
   -DEKF_CORE_HOST_TEST -DFAR= \
   -I"$REPO/apps/ekf3" \
   -o "$OUT/ekf-output-ubsan" \
   "$REPO/tests/ekf_output_test.c" \
   "$REPO/apps/ekf3/ekf_core.c" -lm

"$OUT/ekf-output-ubsan"
