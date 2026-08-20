#!/usr/bin/env bash
# Host-side tests for sensor_status raw timestamp statistics.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -Wall -Wextra -Werror \
   -I"$REPO/apps/sensor_status" \
   -o "$OUT/timing_stats_test" \
   "$REPO/tests/timing_stats_test.c" \
   "$REPO/apps/sensor_status/timing_stats.c" -lm
"$OUT/timing_stats_test"
