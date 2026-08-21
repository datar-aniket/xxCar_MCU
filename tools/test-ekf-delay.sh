#!/usr/bin/env bash
# Host-side unit test for the EKF fusion horizon: ring ordering, horizon
# arithmetic and overflow accounting. No filter mathematics involved.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -O2 -Wall -Wextra -Werror \
   -DEKF_CORE_HOST_TEST -DFAR= \
   -I"$REPO/apps/ekf3" \
   -o "$OUT/ekf-delay-test" \
   "$REPO/tests/ekf_delay_test.c" \
   "$REPO/apps/ekf3/ekf_delay.c" -lm

"$OUT/ekf-delay-test"

cc -std=c11 -O1 -g -Wall -Wextra -Werror \
   -fsanitize=undefined -fno-sanitize-recover=undefined \
   -DEKF_CORE_HOST_TEST -DFAR= \
   -I"$REPO/apps/ekf3" \
   -o "$OUT/ekf-delay-ubsan" \
   "$REPO/tests/ekf_delay_test.c" \
   "$REPO/apps/ekf3/ekf_delay.c" -lm

"$OUT/ekf-delay-ubsan"
