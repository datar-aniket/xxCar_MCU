#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEST_OUT="$(mktemp -d)"
trap 'rm -rf "$TEST_OUT"' EXIT
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -fno-sanitize-recover=all "$REPO/tests/ppm_decode_test.c" -o "$TEST_OUT/test"
ASAN_OPTIONS=detect_leaks=0 "$TEST_OUT/test"
