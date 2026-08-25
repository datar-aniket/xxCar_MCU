#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -Wall -Wextra -Werror \
  -I"$REPO/apps/control_router" \
  "$REPO/tests/control_router_test.c" \
  "$REPO/apps/control_router/control_router_policy.c" -lm -o "$OUT/test"
"$OUT/test"

cc -std=c11 -Wall -Wextra -Werror \
  -fsanitize=address,undefined -fno-sanitize-recover=all \
  -I"$REPO/apps/control_router" \
  "$REPO/tests/control_router_test.c" \
  "$REPO/apps/control_router/control_router_policy.c" -lm -o "$OUT/test-san"
ASAN_OPTIONS=detect_leaks=0 "$OUT/test-san"
