#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NUTTX="$REPO/deps/nuttx"
TEST_OUT="$(mktemp -d)"
trap 'rm -rf "$TEST_OUT"' EXIT
cc -std=c11 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -Wno-address \
  -fno-builtin -D__STDC_NO_ATOMICS__=1 -ffunction-sections -fdata-sections \
  -I"$NUTTX/include" -I"$NUTTX/arch/arm/src/stm32h7" \
  -I"$NUTTX/arch/arm/src/common" -Wl,--gc-sections \
  "$REPO/tests/icm_config_test.c" -o "$TEST_OUT/test"
"$TEST_OUT/test"
