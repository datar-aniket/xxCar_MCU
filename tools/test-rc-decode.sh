#!/usr/bin/env bash
# Host-side unit test for the SBUS/CRSF decoders (apps/rc/rc_decode.c).
#
# The decoders are the one part of the RC driver that can be tested without a
# receiver, and they are also the part where a bug is hardest to SEE: a wrong
# bit-shift or the wrong raw range produces channel values that look entirely
# plausible and are wrong. So they are checked against the endpoints PX4
# defines, on the host, before anything is flashed.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

mkdir -p "$OUT/nuttx"
: > "$OUT/nuttx/config.h"

# rc.h pulls in the uORB-backed rc_in.h, which the host has no business
# building. The decoder only needs one constant out of it.
sed 's|#include "../rc_in/rc_in.h"|#define RC_IN_MAX_CHANNELS 18|' \
    "$REPO/apps/rc/rc.h" > "$OUT/rc.h"
cp "$REPO/apps/rc/rc_decode.c" "$OUT/"
cp "$REPO/tests/rc_decode_test.c" "$OUT/test.c"

cc -std=c11 -Wall -Wextra -Wno-unused-parameter -I"$OUT" -DFAR= \
   -o "$OUT/rctest" "$OUT/test.c" "$OUT/rc_decode.c"
"$OUT/rctest"
