#!/usr/bin/env bash
# Host-side test for the companion GUI's DIRECT_CONTROL panel.
#
# The panel moves a vehicle and its failure modes are quiet: commands sent
# before a clock sync are dropped by the board without a word back, a throttle
# mode switched under a loaded slider reinterprets the number already there,
# and a stream left running against a closed port keeps showing the last
# command it managed. None of those raise, so they are checked against the
# bytes the panel actually emits.
#
# Skips cleanly when there is no display.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$REPO/tests/companion_gui_test.py"
