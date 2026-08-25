#!/usr/bin/env bash
# Host-side test for the VESC CAN codec.
#
# The payloads are big-endian and this MCU is not. A sign-extension mistake
# across a byte-swapped int16 produces a current that is wrong and still
# looks like a current, which is why the negative and extreme cases are here
# rather than left to a bench.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
   -I"$REPO/apps/vesc" \
   -o "$OUT/test" "$REPO/tests/vesc_proto_test.c" \
   "$REPO/apps/vesc/vesc_proto.c" -lm
"$OUT/test"

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
   -fsanitize=undefined,address -fno-sanitize-recover=all \
   -I"$REPO/apps/vesc" \
   -o "$OUT/test-san" "$REPO/tests/vesc_proto_test.c" \
   "$REPO/apps/vesc/vesc_proto.c" -lm
"$OUT/test-san"
