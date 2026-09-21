#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NUTTX="$REPO/deps/nuttx"
TEST_OUT="$(mktemp -d)"
trap 'rm -rf "$TEST_OUT"' EXIT
FLAGS=(-std=c11 -O2 -Wall -Wextra -Werror -fno-builtin
       -D__STDC_NO_ATOMICS__=1 -ffunction-sections -fdata-sections
       -I"$NUTTX/include" -I"$NUTTX/arch/arm/src/stm32h7"
       -I"$NUTTX/arch/arm/src/common" -Wl,--gc-sections)
cc "${FLAGS[@]}" "$REPO/tests/fdcan_driver_test.c" -o "$TEST_OUT/test"
"$TEST_OUT/test"
cc "${FLAGS[@]}" -fsanitize=address,undefined -fno-sanitize-recover=all \
  "$REPO/tests/fdcan_driver_test.c" -o "$TEST_OUT/test-san"
ASAN_OPTIONS=detect_leaks=0 "$TEST_OUT/test-san"
