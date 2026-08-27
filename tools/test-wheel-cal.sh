#!/usr/bin/env bash
# Host-side checks for the tachometer-to-velocity scale fit.
#
# Most of it is the refusal to produce a number: a scale fitted through
# wheelspin, a scrubbing turn, or a stretch where the estimator did not know
# its own speed is worse than none, because it looks calibrated.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$REPO/tests/wheel_cal_host_test.py"
