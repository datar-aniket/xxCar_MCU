#!/usr/bin/env bash
# Host-side unit test for the fixed sensor rotations (apps/sensors/rotation.c).
#
# The table is transcribed by hand from PX4, and a transcription error is close
# to undetectable on hardware: the vehicle just believes it is oriented
# differently than it is, with no reading out of range and no error raised.
#
# The determinant check is the one that earns its keep. A single lost minus
# sign turns a rotation into a reflection - every length and angle preserved,
# so nothing looks wrong - while mirroring the frame, inverting gyro signs and
# running an estimator's yaw backwards.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cp "$REPO/apps/sensors/rotation.h" "$OUT/"
cp "$REPO/apps/sensors/rotation.c" "$OUT/"
cp "$REPO/tests/rotation_test.c" "$OUT/test.c"

cc -std=c11 -Wall -Wextra -Wno-unused-parameter \
   -DROTATION_HOST_TEST -DFAR= \
   -I"$OUT" -o "$OUT/rottest" "$OUT/test.c" "$OUT/rotation.c" -lm
"$OUT/rottest"
