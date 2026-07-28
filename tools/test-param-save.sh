#!/usr/bin/env bash
# Host-side unit test for parameter persistence (apps/param/param.c).
#
# param_save() used to open the LIVE params.txt with "w", which truncates it
# before anything new is known to be storable, and then ignored every write
# result. A save that could not complete therefore destroyed the previous
# values while reporting success - the failure mode that loses a calibration.
#
# The board has no way to demonstrate that safely, but the host does: point
# PARAM_FILE/PARAM_TMPFILE at a temp directory and make the staging path
# unwritable.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

# Stand in for the NuttX environment param.c is compiled against (see
# test-param-range.sh, which stubs the same header).
mkdir -p "$OUT/nuttx"
cat > "$OUT/nuttx/config.h" <<'STUB'
#ifndef OK
#  define OK 0
#endif
STUB

cp "$REPO/apps/param/param.h" "$OUT/"
cp "$REPO/apps/param/param.c" "$OUT/"
cp "$REPO/tests/param_save_test.c" "$OUT/test.c"

# -Wno-missing-field-initializers: see test-param-range.sh.
cc -std=c11 -Wall -Wextra -Wno-unused-parameter \
   -Wno-missing-field-initializers \
   -D_DEFAULT_SOURCE \
   -DPARAM_FILE="\"$OUT/params.txt\"" \
   -DPARAM_TMPFILE="\"$OUT/params.tmp\"" \
   -I"$OUT" -DFAR= -o "$OUT/paramsavetest" "$OUT/test.c" "$OUT/param.c" -lm
"$OUT/paramsavetest"
