#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
  -I"$REPO/boards/fmuv6c/src" \
  "$REPO/tests/ms5611_comp_test.c" \
  "$REPO/boards/fmuv6c/src/ms5611_comp.c" -o "$OUT/test"
"$OUT/test"

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
  -fsanitize=undefined -fno-sanitize-recover=all \
  -I"$REPO/boards/fmuv6c/src" \
  "$REPO/tests/ms5611_comp_test.c" \
  "$REPO/boards/fmuv6c/src/ms5611_comp.c" -o "$OUT/test-ubsan"
"$OUT/test-ubsan"
