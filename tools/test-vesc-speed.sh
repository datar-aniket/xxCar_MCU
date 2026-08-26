#!/usr/bin/env bash
# Host-side test for the VESC motor-speed filter.
#
# This is the anti-alias filter for the 200 Hz companion downlink against a
# 400 Hz telemetry stream, so its rolloff is a correctness property rather
# than a tuning preference.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT

for extra in "" "-fsanitize=undefined,address -fno-sanitize-recover=all"; do
  # shellcheck disable=SC2086
  cc -std=c11 -Wall -Wextra -Werror -DFAR= $extra \
     -I"$REPO/apps/vesc" \
     -o "$OUT/test" "$REPO/tests/vesc_speed_test.c" \
     "$REPO/apps/vesc/vesc_speed.c" -lm
  "$OUT/test"
done
