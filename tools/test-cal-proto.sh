#!/usr/bin/env bash
# Host-side unit test for the calibration protocol (apps/cal/cal_proto.c).
#
# The protocol is the contract between the board and the host GUI, and it is
# meant to stay drivable by hand from a terminal. Both halves of that - a parser
# that tolerates what people type, and JSON that is actually well formed and
# properly escaped - are cheap to get wrong and expensive to debug over a
# serial cable. So they are checked here first.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

mkdir -p "$OUT/nuttx"
: > "$OUT/nuttx/config.h"

cp "$REPO/apps/cal/cal_proto.h" "$OUT/"
cp "$REPO/apps/cal/cal_proto.c" "$OUT/"
cp "$REPO/tests/cal_proto_test.c" "$OUT/test.c"

cc -std=c11 -Wall -Wextra -Wno-unused-parameter -I"$OUT" -DFAR= \
   -DCAL_PROTO_HOST_TEST -o "$OUT/protoTest" "$OUT/test.c" "$OUT/cal_proto.c" -lm
"$OUT/protoTest"
