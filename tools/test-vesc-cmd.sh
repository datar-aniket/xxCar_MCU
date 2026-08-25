#!/usr/bin/env bash
# Host-side test for the VESC command policy: the arm gate and the failsafe.
#
# This is the part of the link that is dangerous when it is wrong, and none
# of it needs hardware, so every state is driven here rather than discovered
# on a bench with a vehicle on its wheels.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
   -I"$REPO/apps/vesc" \
   -o "$OUT/test" "$REPO/tests/vesc_cmd_test.c" \
   "$REPO/apps/vesc/vesc_cmd.c" "$REPO/apps/vesc/vesc_proto.c" -lm
"$OUT/test"

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
   -fsanitize=undefined,address -fno-sanitize-recover=all \
   -I"$REPO/apps/vesc" \
   -o "$OUT/test-san" "$REPO/tests/vesc_cmd_test.c" \
   "$REPO/apps/vesc/vesc_cmd.c" "$REPO/apps/vesc/vesc_proto.c" -lm
"$OUT/test-san"
