#!/usr/bin/env bash
# Host test for the logger's write-it-all helper (apps/logger/log_write.c).
#
# Reproduces the behaviour that ruined two real recordings: NuttX's FAT driver
# writes part of a buffer, advances the file position, then returns a negative
# errno with the partial count discarded. Resuming at either the requested or
# reported offset has corrupted real logs. The test asserts that any partial
# progress stops immediately so logger.c can truncate the whole flush.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT
mkdir -p "$OUT/nuttx"; : > "$OUT/nuttx/config.h"
cp "$REPO/apps/logger/log_write.h" "$REPO/apps/logger/log_write.c" "$OUT/"
cp "$REPO/tests/log_write_test.c" "$OUT/test.c"
cc -std=c11 -Wall -Wextra -Wno-unused-parameter -D_DEFAULT_SOURCE -I"$OUT" -DFAR= \
   -DLOG_WRITE_HOST_TEST -o "$OUT/t" "$OUT/test.c" "$OUT/log_write.c"
"$OUT/t"
