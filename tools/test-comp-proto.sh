#!/usr/bin/env bash
# Host-side test for the companion link codec: framing, CRC, resynchronisation
# and the id/length checks. Byte-at-a-time feeding is the one that earns its
# keep - a parser that only works on whole frames passes everything else and
# fails the first time a UART splits one.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
   -I"$REPO/apps/companion" \
   -o "$OUT/test" "$REPO/tests/comp_proto_test.c" \
   "$REPO/apps/companion/comp_proto.c"
"$OUT/test"

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
   -fsanitize=undefined,address -fno-sanitize-recover=all \
   -I"$REPO/apps/companion" \
   -o "$OUT/test-san" "$REPO/tests/comp_proto_test.c" \
   "$REPO/apps/companion/comp_proto.c"
"$OUT/test-san"

# Two implementations of one wire format is worth being nervous about: a CRC
# or a layout that disagrees produces frames the far end silently drops, with
# no indication of which end is wrong. Compare the bytes directly.
python3 "$REPO/tests/comp_proto_cross_test.py"
