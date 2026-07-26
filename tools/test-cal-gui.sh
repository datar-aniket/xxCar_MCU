#!/usr/bin/env bash
# Host-side test for the calibration GUI's decode and wizard state machine.
#
# The wizard fails quietly: a checklist that advances on a REJECTED capture, or
# a Save enabled before all six positions are in, both yield a calibration that
# looks finished and is wrong, with no exception anywhere. Driving the handlers
# with the board's real JSON is the only cheap way to pin that.
#
# Skips cleanly when there is no display.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$REPO/tests/cal_gui_test.py"
