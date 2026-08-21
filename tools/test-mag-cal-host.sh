#!/usr/bin/env bash
# Host-side NumPy ellipsoid recovery, outlier, coverage and preview checks.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$REPO/tests/mag_cal_host_test.py"
