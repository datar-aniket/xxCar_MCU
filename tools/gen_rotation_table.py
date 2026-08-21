#!/usr/bin/env python3
"""Generate tools/rotation_table.py from apps/sensors/rotation.c.

rotation.h is explicit that a wrong rotation does not fail, it silently reads
the vehicle sideways. Transcribing 24 sign-and-swap matrices by hand into
Python is exactly the kind of thing that goes wrong once and is never noticed,
so the table is produced by ASKING the C code what it does.
"""

import pathlib
import subprocess
import sys
import tempfile

REPO = pathlib.Path(__file__).resolve().parents[1]

HARNESS = r"""
#include <stdio.h>
#include "rotation.h"

int main(void)
{
  int rot;

  for (rot = 0; rot < ROTATION_MAX_SUPPORTED; rot++)
    {
      float basis[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
      int axis;

      if (!rotation_supported((unsigned char)rot))
        {
          continue;
        }

      printf("%d", rot);

      for (axis = 0; axis < 3; axis++)
        {
          rotation_apply((unsigned char)rot, basis[axis]);
          printf(" %.0f %.0f %.0f",
                 (double)basis[axis][0], (double)basis[axis][1],
                 (double)basis[axis][2]);
        }

      printf("\n");
    }

  return 0;
}
"""


def generate() -> str:
    with tempfile.TemporaryDirectory() as tmp:
        src = pathlib.Path(tmp) / "harness.c"
        exe = pathlib.Path(tmp) / "harness"
        src.write_text(HARNESS)
        subprocess.run(
            ["cc", "-std=c11", "-Wall", "-Wextra", "-Wno-unused-parameter",
             "-DROTATION_HOST_TEST", "-DFAR=",
             "-I", str(REPO / "apps" / "sensors"), str(src),
             str(REPO / "apps" / "sensors" / "rotation.c"),
             "-lm", "-o", str(exe)],
            check=True)
        out = subprocess.run([str(exe)], check=True,
                             capture_output=True, text=True).stdout

    lines = ["#!/usr/bin/env python3",
             '"""The representable rotations, GENERATED from '
             'apps/sensors/rotation.c.',
             "",
             "Do not edit. Run tools/gen_rotation_table.py. The mapping is",
             "verified against the C implementation by",
             "tools/test-align-solve.sh, so the two cannot drift apart.",
             "",
             "There are 26 supported enum values but only 24 DISTINCT",
             "rotations: ROLL_180_YAW_90 (10) and PITCH_180_YAW_270 (27) are",
             "the same rotation reached by different Euler routes, as are",
             "ROLL_180_YAW_270 (14) and PITCH_180_YAW_90 (26). That is",
             "inherent to the PX4 enum, which rotation.c reproduces exactly.",
             "Callers matching a matrix back to a name must therefore choose",
             "one deliberately - align_solve.snap() takes the lowest value.",
             '"""',
             "",
             "import numpy as np",
             "",
             "ROTATIONS = {"]

    for line in out.strip().splitlines():
        parts = line.split()
        rot = int(parts[0])
        v = [float(x) for x in parts[1:]]

        # rotation_apply transforms a vector, so applying it to each basis
        # vector yields the COLUMNS of the matrix transposed - basis i maps to
        # column i of M^T, i.e. row i of M. Emit rows directly.
        # Normalise negative zero. "%.0f" of -0.0 prints "-0", which is
        # harmless to numpy and confusing to read.
        rows = [[x + 0.0 for x in v[0:3]],
                [x + 0.0 for x in v[3:6]],
                [x + 0.0 for x in v[6:9]]]
        body = ", ".join(
            "({:.0f}, {:.0f}, {:.0f})".format(*(int(x) for x in r))
            for r in rows)
        lines.append(f"    {rot}: np.array(({body})).T,")

    lines.append("}")
    lines.append("")
    return "\n".join(lines)


if __name__ == "__main__":
    dest = REPO / "tools" / "rotation_table.py"
    dest.write_text(generate())
    print(f"wrote {dest}", file=sys.stderr)
