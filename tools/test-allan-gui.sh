#!/usr/bin/env bash
# End-to-end test of the Allan window against synthetic .ulg parts.
#
# The only test that exercises the join between parts. A loader that dropped a
# part or ordered them wrongly would still yield a plausible curve - from less
# data, or with a fabricated discontinuity - and the plot would look fine.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -c "import numpy, pyulog" 2>/dev/null || { echo "allan_gui: numpy/pyulog missing - skipped"; exit 0; }
exec python3 "$REPO/tests/allan_gui_test.py"
