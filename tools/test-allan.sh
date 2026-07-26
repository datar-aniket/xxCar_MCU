#!/usr/bin/env bash
# Host test for the Allan variance engine (tools/allan.py).
#
# Driven with synthetic noise of known coefficients, because a wrong
# extraction produces a plausible small number rather than an obvious error -
# it would quietly mistune the EKF with nothing to show for it.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -c "import numpy" 2>/dev/null || { echo "allan: numpy missing - skipped"; exit 0; }
exec python3 "$REPO/tests/allan_test.py"
