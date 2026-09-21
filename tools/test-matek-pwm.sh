#!/usr/bin/env bash
# Host execution of the production driver using actual NuttX register macros.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NUTTX="$REPO/deps/nuttx"
TEST_OUT="$(mktemp -d)"
trap 'rm -rf "$TEST_OUT"' EXIT
FLAGS=(-std=c11 -O2 -Wall -Wextra -Werror -fno-builtin
       -D__STDC_NO_ATOMICS__=1
       -I"$NUTTX/include" -I"$NUTTX/arch/arm/src/stm32h7"
       -I"$NUTTX/arch/arm/src/common")
cc "${FLAGS[@]}" "$REPO/tests/matek_pwm_test.c" -o "$TEST_OUT/pwm-test"
"$TEST_OUT/pwm-test"

cc "${FLAGS[@]}" -fsanitize=address,undefined -fno-sanitize-recover=all \
  "$REPO/tests/matek_pwm_test.c" -o "$TEST_OUT/pwm-test-san"
ASAN_OPTIONS=detect_leaks=0 "$TEST_OUT/pwm-test-san"

# Demonstrate that the rollover test actually detects the original race.
cc "${FLAGS[@]}" -DTEST_WITHOUT_UDIS \
  "$REPO/tests/matek_pwm_test.c" -o "$TEST_OUT/pwm-test-unsafe"
if "$TEST_OUT/pwm-test-unsafe" >"$TEST_OUT/unsafe.log" 2>&1; then
  echo 'FAIL: missing UDIS guard was not detected' >&2
  exit 1
fi
grep -Fq 'was || now' "$TEST_OUT/unsafe.log"
echo 'PASS: negative control rejects the unguarded rollover race'
