#!/usr/bin/env bash
# Host-side test for the MAVLink codec (deps/mavlink) as we use it: pack, frame,
# x25 CRC, parse-back, and struct decode. Proves the wire format and CRC_EXTRA
# tables agree with an independent build, and that a corrupted frame is rejected
# - without needing a GCS or the MTF-02 wired up.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT
cc -std=gnu11 -w \
   -I"$REPO/deps/mavlink/common" -I"$REPO/deps/mavlink" \
   -o "$OUT/mavtest" "$REPO/tests/mavlink_codec_test.c"
"$OUT/mavtest"
