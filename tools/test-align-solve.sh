#!/usr/bin/env bash
# Host-side alignment solver: rotation table, gravity columns, magnetometer
# variance solve, flow correlation, and every refusal path.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$REPO/tests/align_solve_host_test.py"
