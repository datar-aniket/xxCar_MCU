#!/usr/bin/env bash
# Build and test gate. Run this before claiming anything works.
#
# It exists because grepping build output for "error:" is not a build check and
# quietly said "0 errors" through a link failure: the linker reports
# "undefined reference to `lrintf'" and make reports "Error 1", neither of
# which contains that string. Worse, the stale .px4 from the previous build was
# still sitting there, so a file-exists check passed too.
#
# So: trust the exit status, and prove the artifact is NEWER than the sources
# that went into it.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

PX4="build/xxcar.px4"
LOG="$(mktemp)"
trap 'rm -f "$LOG"' EXIT
fail=0

echo "=== host tests ==="
for t in tools/test-*.sh; do
  [ "$t" = "tools/verify.sh" ] && continue
  name=$(basename "$t" .sh)
  if bash "$t" >/dev/null 2>&1; then
    printf '  %-22s PASS\n' "$name"
  else
    printf '  %-22s FAIL\n' "$name"; fail=1
  fi
done

echo "=== firmware build ==="
# stamp first, so "newer than this" proves the artifact was rebuilt
STAMP="$(mktemp)"; sleep 1

if ./tools/build.sh >"$LOG" 2>&1; then
  echo "  build.sh exited 0"
else
  echo "  build.sh FAILED (exit $?) — last lines:"
  grep -iE 'error|undefined reference|No rule|cannot find' "$LOG" | tail -12 \
    | sed 's/^/    /'
  rm -f "$STAMP"
  exit 1
fi

if [ ! -f "$PX4" ]; then
  echo "  $PX4 MISSING"; rm -f "$STAMP"; exit 1
fi

if [ "$PX4" -nt "$STAMP" ]; then
  echo "  $PX4 rebuilt ($(stat -c%s "$PX4") bytes)"
else
  echo "  $PX4 is STALE — build produced no new artifact"; fail=1
fi
rm -f "$STAMP"

# A link failure can still leave exit 0 in some make setups, so check that the
# symbols really made it into the image.
#
# Dump once into a file rather than piping into `grep -q` per symbol: grep -q
# exits at the first match, the producer takes SIGPIPE, and under `pipefail`
# that 141 becomes the pipeline's status - so every symbol reports MISSING
# while sitting plainly in the binary. This gate reported exactly that.
echo "=== symbols ==="
NM="$(mktemp)"
arm-none-eabi-nm deps/nuttx/nuttx >"$NM" 2>/dev/null || true
for sym in cal_session cal_main param_init serial_manager_start \
           cal_mag_validate \
           sensors_start rotation_apply g_orb_vehicle_accel \
           imu_integrator_add g_orb_vehicle_imu ekf_core_process \
           g_orb_estimator_state; do
  if grep -qE "^[0-9a-f]+ T $sym\$" "$NM"; then
    printf '  %-22s linked\n' "$sym"
  else
    printf '  %-22s MISSING\n' "$sym"; fail=1
  fi
done
rm -f "$NM"

# Scoped to the app under active development. The rest of the tree carries
# pre-existing overruns (px4io 22, uorb_msgs 16, serial 13, logger 13) that
# predate this work; failing on them would make the gate useless noise.
echo "=== style (apps/cal) ==="
long=$(awk 'length>80 {print FILENAME":"FNR}' apps/cal/*.c apps/cal/*.h \
        2>/dev/null | wc -l)
echo "  lines over 80 columns: $long"
if [ "$long" -gt 0 ]; then
  awk 'length>80 {print "    "FILENAME":"FNR}' apps/cal/*.c apps/cal/*.h
  fail=1
fi

[ "$fail" -eq 0 ] && echo "ALL GREEN" || echo "FAILURES ABOVE"
exit "$fail"
