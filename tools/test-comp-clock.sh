#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
   -I"$REPO/apps/companion" \
   -o "$OUT/test" "$REPO/tests/comp_clock_test.c" \
   "$REPO/apps/companion/comp_clock.c"
"$OUT/test"

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
   -fsanitize=undefined -fno-sanitize-recover=all \
   -I"$REPO/apps/companion" \
   -o "$OUT/test-san" "$REPO/tests/comp_clock_test.c" \
   "$REPO/apps/companion/comp_clock.c"
"$OUT/test-san"
