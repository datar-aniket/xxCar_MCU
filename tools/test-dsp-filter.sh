#!/usr/bin/env bash
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -O2 -Wall -Wextra -Werror \
   -DDSP_FILTER_HOST_TEST -DFAR= \
   -I"$REPO/apps/sensors" \
   -o "$OUT/dsp-filter-test" \
   "$REPO/tests/dsp_filter_test.c" \
   "$REPO/apps/sensors/dsp_filter.c" -lm

"$OUT/dsp-filter-test"
