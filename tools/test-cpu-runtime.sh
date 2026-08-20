#!/usr/bin/env bash
# Host-side tests for DWT wrap accumulation and CPU runtime formatting math.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -Wall -Wextra -Werror \
   -I"$REPO/apps/cpu_status" \
   -o "$OUT/cpu_runtime_test" \
   "$REPO/tests/cpu_runtime_test.c" \
   "$REPO/apps/cpu_status/cpu_runtime.c"
"$OUT/cpu_runtime_test"
