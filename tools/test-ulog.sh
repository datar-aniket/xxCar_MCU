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
assert names == ['distance_sensor','estimator_diag','estimator_state','external_pose','optical_flow','rc_input','sensor_accel','sensor_baro','sensor_mag','vehicle_accel','vehicle_imu','vehicle_state_tx'], names
by = {d.name: d for d in u.data_list}
assert by['rc_input'].data['channel[0]'][0] == 1500
assert abs(by['sensor_baro'].data['pressure'][0] - 1013.25) < 0.01
assert by['sensor_mag'].data['status'][0] == 1
assert abs(by['optical_flow'].data['distance'][0] - 1.25) < 0.01
assert by['optical_flow'].data['quality'][0] == 200
assert abs(by['distance_sensor'].data['current_distance'][0] - 1.30) < 0.01
assert by['vehicle_imu'].data['samples'][0] == 5
assert abs(by['estimator_state'].data['position[2]'][0] - 3.0) < 0.01
assert abs(by['external_pose'].data['x'][0] - 1.2) < 0.01
assert abs(by['estimator_diag'].data['residual_accel_body[2]'][0] + 0.01665) < 1e-4
assert by['estimator_diag'].data['flags'][0] == 0x1143
assert by['estimator_diag'].data['extnav_timestamp'][0] == 850
assert abs(by['vehicle_accel'].data['y'][0] - 0.08) < 1e-4
assert by['vehicle_state_tx'].data['accel_timestamp_sample'][0] == 990
assert abs(by['vehicle_state_tx'].data['accel[1]'][0] - 0.08) < 1e-4
assert by['vehicle_state_tx'].data['source_valid'][0] == 0x1f
assert by['vehicle_state_tx'].data['rc_status'][0] == 0x075c85f0
print("ULog: 12 topics decoded, values verified - OK")
PY
