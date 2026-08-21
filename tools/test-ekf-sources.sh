#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

mkdir -p "$OUT/nuttx"
cat > "$OUT/nuttx/config.h" <<'STUB'
#ifndef OK
#  define OK 0
#endif
STUB

cc -std=c11 -D_POSIX_C_SOURCE=200809L -Wall -Wextra -Werror \
  -Wno-unused-parameter \
  -Wno-missing-field-initializers -DFAR= -I"$OUT" \
  -I"$REPO/apps/ekf3" -I"$REPO/apps/param" \
  -DPARAM_FILE='"/tmp/xxcar-ekf-sources-no-file"' \
  -DPARAM_TMPFILE='"/tmp/xxcar-ekf-sources-no-temp"' \
  "$REPO/tests/ekf_sources_test.c" "$REPO/apps/ekf3/ekf_sources.c" \
  "$REPO/apps/param/param.c" -lm -o "$OUT/test"
"$OUT/test"
