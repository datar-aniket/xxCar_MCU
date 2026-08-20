#!/usr/bin/env bash
# Host test for logger bulk-read record accounting and stride.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -Wall -Wextra -Werror -I"$REPO/apps/logger" \
   -o "$OUT/test-log-batch" "$REPO/tests/log_batch_test.c"
"$OUT/test-log-batch"
