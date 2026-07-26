#!/usr/bin/env bash
# Host-side unit test for out-of-range parameter handling (apps/param/param.c).
#
# A scalar and a selector must not be treated the same way when params.txt holds
# a value the firmware does not recognise. Clamping a scalar to the nearest
# bound is reasonable; clamping a selector picks a DIFFERENT FUNCTION, which is
# how SER_USB_FUNC=5 became RC_IN and put an SBUS decoder on the USB CDC port.
#
# That failure only showed itself on hardware, as an ENOTTY loop with a
# misleading message about a config option that was in fact enabled. It is
# cheap to pin here, so it is pinned here.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

# Stand in for the NuttX environment param.c is compiled against. OK is NuttX's
# success return; on the host there is no such thing, so the stub supplies it.
mkdir -p "$OUT/nuttx"
cat > "$OUT/nuttx/config.h" <<'STUB'
#ifndef OK
#  define OK 0
#endif
STUB

cp "$REPO/apps/param/param.h" "$OUT/"
cp "$REPO/apps/param/param.c" "$OUT/"
cp "$REPO/tests/param_range_test.c" "$OUT/test.c"

# -Wno-missing-field-initializers: the parameter table deliberately omits the
# trailing `range` member on every scalar, which C zero-initialises to
# PARAM_RANGE_CLAMP. Spelling it out on all 40-odd rows would push them past the
# column limit for no gain.
cc -std=c11 -Wall -Wextra -Wno-unused-parameter \
   -Wno-missing-field-initializers \
   -I"$OUT" -DFAR= -o "$OUT/paramtest" "$OUT/test.c" "$OUT/param.c" -lm
"$OUT/paramtest"
