#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -O2 -Wall -Wextra -Werror -DEKF_CORE_HOST_TEST -DFAR= \
  -I"$REPO/apps/ekf3" "$REPO/tests/ekf_wheel_test.c" \
  "$REPO/apps/ekf3/ekf_wheel.c" -lm -o "$OUT/test"
"$OUT/test"

cc -std=c11 -O1 -g -Wall -Wextra -Werror -DEKF_CORE_HOST_TEST -DFAR= \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I"$REPO/apps/ekf3" "$REPO/tests/ekf_wheel_test.c" \
  "$REPO/apps/ekf3/ekf_wheel.c" -lm -o "$OUT/test-san"
"$OUT/test-san"
