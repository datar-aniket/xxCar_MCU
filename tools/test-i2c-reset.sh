#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEST_OUT="$(mktemp -d)"
trap 'rm -rf "$TEST_OUT"' EXIT
SOURCE=arch/arm/src/stm32h7/stm32_i2c.c
# Compile the exact production function with fake bus/pin boundaries.
sed -n '/^static int stm32_i2c_reset(struct i2c_master_s \*dev)$/,/^#endif/p' \
  "$REPO/deps/nuttx/$SOURCE" | sed '$d' > "$TEST_OUT/i2c_reset_under_test.h"
cc -std=c11 -Wall -Wextra -Werror -I"$TEST_OUT" \
  "$REPO/tests/i2c_reset_test.c" -o "$TEST_OUT/test"
"$TEST_OUT/test"
# Original upstream must fail the same recovery-cleanup tests.
git -C "$REPO/deps/nuttx" show "HEAD:$SOURCE" |
  sed -n '/^static int stm32_i2c_reset(struct i2c_master_s \*dev)$/,/^#endif/p' |
  sed '$d' > "$TEST_OUT/i2c_reset_under_test.h"
cc -std=c11 -Wall -Wextra -Werror -I"$TEST_OUT" \
  "$REPO/tests/i2c_reset_test.c" -o "$TEST_OUT/test-original"
ulimit -c 0
if "$TEST_OUT/test-original" >"$TEST_OUT/original.log" 2>&1; then
  echo 'FAIL: negative control accepted original failed-reset cleanup' >&2
  exit 1
fi
grep -Fq 'restores == 2' "$TEST_OUT/original.log"
echo 'PASS: negative control catches original I2C cleanup fault'
