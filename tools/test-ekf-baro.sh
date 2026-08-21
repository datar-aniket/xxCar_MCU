#!/usr/bin/env bash
# Host-side unit test for barometric height fusion: conversion, reference
# capture, NED sign convention and gating.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -O2 -Wall -Wextra -Werror \
   -DEKF_CORE_HOST_TEST -DFAR= \
   -I"$REPO/apps/ekf3" \
   -o "$OUT/ekf-baro-test" \
   "$REPO/tests/ekf_baro_test.c" \
   "$REPO/apps/ekf3/ekf_core.c" -lm

"$OUT/ekf-baro-test"

cc -std=c11 -O1 -g -Wall -Wextra -Werror \
   -fsanitize=undefined -fno-sanitize-recover=undefined \
   -DEKF_CORE_HOST_TEST -DFAR= \
   -I"$REPO/apps/ekf3" \
   -o "$OUT/ekf-baro-ubsan" \
   "$REPO/tests/ekf_baro_test.c" \
   "$REPO/apps/ekf3/ekf_core.c" -lm

"$OUT/ekf-baro-ubsan"
