#!/usr/bin/env bash
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -O2 -Wall -Wextra -Werror -DFAR= \
   -I"$REPO/apps/ekf3" \
   -o "$OUT/ekf-extnav-time-test" \
   "$REPO/tests/ekf_extnav_time_test.c" \
   "$REPO/apps/ekf3/ekf_extnav_time.c"

"$OUT/ekf-extnav-time-test"
