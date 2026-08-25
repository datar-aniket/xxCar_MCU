#!/usr/bin/env python3
"""Prove the Python codec and the C codec produce identical bytes.

Two implementations of one wire format is a thing worth being nervous about.
A CRC or a struct layout that disagrees produces frames the far end silently
drops as corrupt, and the only symptom is a counter climbing on one end with
no indication of WHICH end is wrong.

So the C encoder is compiled and asked to frame the same payloads, and the
bytes are compared. Anything that drifts - polynomial, seed, field order,
padding - shows up here rather than on a bench.
"""

import pathlib
import subprocess
import sys
import tempfile

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))

import comp_link  # noqa: E402

HARNESS = r"""
#include <stdio.h>
#include <string.h>
#include "comp_proto.h"

static void dump(const unsigned char *d, int n)
{
  int i;
  for (i = 0; i < n; i++)
    {
      printf("%02x", d[i]);
    }
  printf("\n");
}

int main(void)
{
  unsigned char frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  struct comp_external_pose_s ext;
  struct comp_estimator_pose_s est;
  int n;

  printf("%zu %zu\n", sizeof(ext), sizeof(est));

  memset(&ext, 0, sizeof(ext));
  ext.timestamp_us = 1234567890123ull;
  ext.x = -12.5f;
  ext.y = 3.25f;
  ext.yaw = 1.5707963f;
  ext.cov[0] = 0.01f; ext.cov[1] = 0.02f; ext.cov[2] = 0.03f;
  ext.cov[3] = 0.04f; ext.cov[4] = 0.05f; ext.cov[5] = 0.06f;
  ext.flags = COMP_POSE_FLAG_VALID;
  ext.reset_counter = 7;
  n = comp_encode(COMP_MSG_EXTERNAL_POSE, &ext, sizeof(ext), frame,
                  sizeof(frame));
  dump(frame, n);

  memset(&est, 0, sizeof(est));
  est.timestamp_us = 999999999ull;
  est.position[0] = 1.0f; est.position[1] = -2.0f; est.position[2] = 3.5f;
  est.quaternion[0] = 0.7071068f; est.quaternion[3] = 0.7071068f;
  est.velocity[0] = 0.25f; est.velocity[1] = -0.5f; est.velocity[2] = 0.125f;
  est.solution_status = 0x4f;
  est.reset_counter = 3;
  n = comp_encode(COMP_MSG_ESTIMATOR_POSE, &est, sizeof(est), frame,
                  sizeof(frame));
  dump(frame, n);

  return 0;
}
"""


def main():
    with tempfile.TemporaryDirectory() as tmp:
        src = pathlib.Path(tmp) / "cross.c"
        exe = pathlib.Path(tmp) / "cross"
        src.write_text(HARNESS)
        subprocess.run(
            ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-DFAR=",
             "-I", str(REPO / "apps" / "companion"), str(src),
             str(REPO / "apps" / "companion" / "comp_proto.c"),
             "-o", str(exe)], check=True)
        out = subprocess.run([str(exe)], check=True, capture_output=True,
                             text=True).stdout.split()

    c_ext_size, c_est_size = int(out[0]), int(out[1])
    c_ext_frame, c_est_frame = bytes.fromhex(out[2]), bytes.fromhex(out[3])

    assert c_ext_size == comp_link.EXTERNAL_POSE.size, (
        f"external_pose: C says {c_ext_size}, "
        f"Python says {comp_link.EXTERNAL_POSE.size}")
    assert c_est_size == comp_link.ESTIMATOR_POSE.size, (
        f"estimator_pose: C says {c_est_size}, "
        f"Python says {comp_link.ESTIMATOR_POSE.size}")

    py_ext = comp_link.encode_external_pose(
        -12.5, 3.25, 1.5707963,
        cov=(0.01, 0.02, 0.03, 0.04, 0.05, 0.06),
        valid=True, reset_counter=7, timestamp_us=1234567890123)
    assert py_ext == c_ext_frame, (
        f"EXTERNAL_POSE bytes differ\n  C:  {c_ext_frame.hex()}\n"
        f"  py: {py_ext.hex()}")

    py_est = comp_link.encode(
        comp_link.MSG_ESTIMATOR_POSE,
        comp_link.ESTIMATOR_POSE.pack(
            999999999, 1.0, -2.0, 3.5,
            0.7071068, 0.0, 0.0, 0.7071068,
            0.25, -0.5, 0.125, 0x4f, 3))
    assert py_est == c_est_frame, (
        f"ESTIMATOR_POSE bytes differ\n  C:  {c_est_frame.hex()}\n"
        f"  py: {py_est.hex()}")

    # And the Python parser must accept what C produced.
    parser = comp_link.Parser()
    got = None
    for b in c_est_frame:
        result = parser.feed(b)
        if result is not None:
            got = result
    assert got is not None and got[0] == comp_link.MSG_ESTIMATOR_POSE
    pose = comp_link.decode_estimator_pose(got[1])
    assert abs(pose["position"][2] - 3.5) < 1e-6
    assert pose["reset_counter"] == 3

    # A 90-degree yaw quaternion must read back as 90 degrees, which pins the
    # Euler convention against the firmware's own ekf_core_euler().
    import math
    _r, _p, yaw = comp_link.quaternion_to_euler(pose["quaternion"])
    assert abs(math.degrees(yaw) - 90.0) < 1e-3, math.degrees(yaw)

    print("comp_proto cross-check: Python and C agree byte for byte - OK")


if __name__ == "__main__":
    main()
