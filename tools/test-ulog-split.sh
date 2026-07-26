#!/usr/bin/env bash
# Verifies that a ULog part written by the 100 MB rollover opens on its own.
#
# If the per-part prologue were wrong or missing, every part after the first
# would be unreadable and an overnight run would leave 100 MB of data and
# gigabytes of rubble - with nothing on the board reporting a problem.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$REPO/tests/ulog_split_test.py"
