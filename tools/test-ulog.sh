#!/usr/bin/env bash
# Host-side check of the ULog writer's format strings and record sizes against a
# real parser. The ULog record layout is unforgiving - pyulog strips a trailing
# _padding field from BOTH the expected and maximum record size, so a padded
# struct written at sizeof() is silently dropped as "corrupt". This test
# generates a file with the exact formats + rec_sizes apps/logger uses and
# confirms pyulog decodes every topic AND the field values.
#
# Skips (does not fail) if pyulog is not installed.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT

python3 -c "import pyulog" 2>/dev/null || {
  echo "pyulog not installed (pip install pyulog) - skipping ULog parse test"
  exit 0
}

cc -o "$OUT/gen" "$REPO/tests/ulog_codec_test.c"
( cd "$OUT" && ./gen )
python3 - "$OUT/all.ulg" <<'PY'
import sys
from pyulog import ULog
u = ULog(sys.argv[1])
names = sorted(d.name for d in u.data_list)
assert names == ['distance_sensor','optical_flow','rc_input','sensor_accel','sensor_baro','sensor_mag'], names
by = {d.name: d for d in u.data_list}
assert by['rc_input'].data['channel[0]'][0] == 1500
assert abs(by['sensor_baro'].data['pressure'][0] - 1013.25) < 0.01
assert by['sensor_mag'].data['status'][0] == 1
assert abs(by['optical_flow'].data['distance'][0] - 1.25) < 0.01
assert by['optical_flow'].data['quality'][0] == 200
assert abs(by['distance_sensor'].data['current_distance'][0] - 1.30) < 0.01
print("ULog: 6 topics decoded, values verified - OK")
PY
