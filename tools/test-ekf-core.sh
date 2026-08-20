#!/usr/bin/env bash
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -O2 -Wall -Wextra -Werror \
   -DEKF_CORE_HOST_TEST -DFAR= \
   -I"$REPO/apps/ekf3" \
   -o "$OUT/ekf-core-test" \
   "$REPO/tests/ekf_core_test.c" \
   "$REPO/apps/ekf3/ekf_core.c" -lm

"$OUT/ekf-core-test"

cc -std=c11 -O1 -g -Wall -Wextra -Werror \
   -fsanitize=undefined -fno-sanitize-recover=undefined \
   -DEKF_CORE_HOST_TEST -DFAR= \
   -I"$REPO/apps/ekf3" \
   -o "$OUT/ekf-core-ubsan" \
   "$REPO/tests/ekf_core_test.c" \
   "$REPO/apps/ekf3/ekf_core.c" -lm

"$OUT/ekf-core-ubsan"
