#!/usr/bin/env bash
# Host-side unit test for magnetometer parameter loading and body-frame
# rotation. The calibration mathematics is cal_mag_apply()'s and is covered
# by test-cal-mag.sh; this only covers what mag_frame.c adds on top.
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
  -Wno-unused-parameter -Wno-missing-field-initializers \
  -DFAR= -DMAG_FRAME_HOST_TEST -DCAL_MAG_HOST_TEST -I"$OUT" \
  -I"$REPO/apps/sensors" -I"$REPO/apps/param" \
  -DPARAM_FILE='"/tmp/xxcar-mag-frame-no-file"' \
  -DPARAM_TMPFILE='"/tmp/xxcar-mag-frame-no-temp"' \
  "$REPO/tests/mag_frame_test.c" "$REPO/apps/sensors/mag_frame.c" \
  "$REPO/apps/sensors/rotation.c" "$REPO/apps/cal/cal_mag.c" \
  "$REPO/apps/param/param.c" -lm -o "$OUT/test"
"$OUT/test"

cc -std=c11 -D_POSIX_C_SOURCE=200809L -Wall -Wextra -Werror \
  -Wno-unused-parameter -Wno-missing-field-initializers \
  -fsanitize=undefined,float-divide-by-zero -fno-sanitize-recover=all \
  -DFAR= -DMAG_FRAME_HOST_TEST -DCAL_MAG_HOST_TEST -I"$OUT" \
  -I"$REPO/apps/sensors" -I"$REPO/apps/param" \
  -DPARAM_FILE='"/tmp/xxcar-mag-frame-no-file"' \
  -DPARAM_TMPFILE='"/tmp/xxcar-mag-frame-no-temp"' \
  "$REPO/tests/mag_frame_test.c" "$REPO/apps/sensors/mag_frame.c" \
  "$REPO/apps/sensors/rotation.c" "$REPO/apps/cal/cal_mag.c" \
  "$REPO/apps/param/param.c" -lm -o "$OUT/test-ubsan"
"$OUT/test-ubsan"
