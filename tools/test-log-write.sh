#!/usr/bin/env bash
# Host test for the logger's write-it-all helper (apps/logger/log_write.c).
#
# Reproduces the behaviour that ruined two real recordings: NuttX's FAT driver
# writes part of a buffer, advances the file position, then returns a negative
# errno with the partial count discarded. Dropping the remainder tears a
# record; retrying from the same offset duplicates what landed. The stub does
# exactly that and the tests assert on the bytes reaching the file.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT
mkdir -p "$OUT/nuttx"; : > "$OUT/nuttx/config.h"
cp "$REPO/apps/logger/log_write.h" "$REPO/apps/logger/log_write.c" "$OUT/"
cp "$REPO/tests/log_write_test.c" "$OUT/test.c"
cc -std=c11 -Wall -Wextra -Wno-unused-parameter -D_DEFAULT_SOURCE -I"$OUT" -DFAR= \
   -DLOG_WRITE_HOST_TEST -o "$OUT/t" "$OUT/test.c" "$OUT/log_write.c"
"$OUT/t"
