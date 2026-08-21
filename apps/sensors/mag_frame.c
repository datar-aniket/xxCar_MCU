/****************************************************************************
 * apps/sensors/mag_frame.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef MAG_FRAME_HOST_TEST
#  include <nuttx/config.h>
#endif

#include <math.h>
#include <string.h>

#include "mag_frame.h"
#include "rotation.h"
#include "../param/param.h"

/* Below this the rotation vector is indistinguishable from no rotation and
 * the axis is not recoverable, so dividing by the angle would be a
 * zero-divide dressed up as mathematics.
 */

#define MAG_FINE_ROTATION_MIN 1.0e-9f

static bool vector_finite(FAR const float v[3])
{
  return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

/* Full Rodrigues rotation about the axis of rv by |rv|.
 *
 * Not the small-angle form. CAL_MAG0_RV* is bounded at 0.35 rad, where a
 * first-order approximation is already about 2 percent wrong AND no longer
 * norm-preserving - which would corrupt the field-magnitude health check the
 * estimator uses to decide whether the reading can be trusted at all.
 */

static void rotate_by_vector(FAR const float rv[3], FAR float v[3])
{
  float angle = sqrtf(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]);
  float axis[3];
  float cross[3];
  float dot;
  float sine;
  float cosine;
  int i;

  if (!isfinite(angle) || angle < MAG_FINE_ROTATION_MIN)
    {
      return;
    }

  for (i = 0; i < 3; i++)
    {
      axis[i] = rv[i] / angle;
    }

  sine = sinf(angle);
  cosine = cosf(angle);
  dot = axis[0] * v[0] + axis[1] * v[1] + axis[2] * v[2];
  cross[0] = axis[1] * v[2] - axis[2] * v[1];
  cross[1] = axis[2] * v[0] - axis[0] * v[2];
  cross[2] = axis[0] * v[1] - axis[1] * v[0];

  for (i = 0; i < 3; i++)
    {
      v[i] = v[i] * cosine + cross[i] * sine +
             axis[i] * dot * (1.0f - cosine);
    }
}

bool mag_frame_load(FAR struct mag_frame_s *frame)
{
  static const FAR char *const offset_names[3] =
  {
    "CAL_MAG0_XOFF", "CAL_MAG0_YOFF", "CAL_MAG0_ZOFF"
  };

  static const FAR char *const fine_names[3] =
  {
    "CAL_MAG0_RVX", "CAL_MAG0_RVY", "CAL_MAG0_RVZ"
  };

  float xy;
  float xz;
  float yz;
  int i;

  if (frame == NULL)
    {
      return false;
    }

  memset(frame, 0, sizeof(*frame));

  for (i = 0; i < 3; i++)
    {
      frame->fit.offset[i] = param_f32(offset_names[i]);
      frame->fine_rv[i] = param_f32(fine_names[i]);
    }

  /* Six stored parameters, nine matrix entries: the soft-iron correction is
   * symmetric by construction. Expanded the same way cal.c does it.
   */

  xy = param_f32("CAL_MAG0_XY");
  xz = param_f32("CAL_MAG0_XZ");
  yz = param_f32("CAL_MAG0_YZ");

  frame->fit.matrix[0][0] = param_f32("CAL_MAG0_XX");
  frame->fit.matrix[1][1] = param_f32("CAL_MAG0_YY");
  frame->fit.matrix[2][2] = param_f32("CAL_MAG0_ZZ");
  frame->fit.matrix[0][1] = xy;
  frame->fit.matrix[1][0] = xy;
  frame->fit.matrix[0][2] = xz;
  frame->fit.matrix[2][0] = xz;
  frame->fit.matrix[1][2] = yz;
  frame->fit.matrix[2][1] = yz;

  frame->fit.field = param_f32("CAL_MAG0_FIELD");
  frame->mag_rot = (uint8_t)param_i32("SENS_MAG0_ROT");
  frame->board_rot = (uint8_t)param_i32("SENS_BOARD_ROT");

  /* CAL_MAG0_OK says a calibration was written, not that the values are
   * still sane - the parameter ranges are wider than the validator's bounds,
   * so re-validate rather than trusting the flag alone.
   */

  frame->valid = param_i32("CAL_MAG0_OK") == 1 &&
                 cal_mag_validate(&frame->fit) == CAL_MAG_OK &&
                 rotation_supported(frame->mag_rot) &&
                 rotation_supported(frame->board_rot);

  return frame->valid;
}

bool mag_frame_apply(FAR const struct mag_frame_s *frame,
                     FAR const float raw[3], FAR float out[3])
{
  if (frame == NULL || raw == NULL || out == NULL || !vector_finite(raw))
    {
      return false;
    }

  if (!frame->valid)
    {
      memcpy(out, raw, 3 * sizeof(float));
      return false;
    }

  /* Hard iron then soft iron, in the sensor's own axes. */

  cal_mag_apply(&frame->fit, raw, out);

  /* IST8310 (+x fwd, +y RIGHT, +z up) -> vehicle convention
   * (+x fwd, +y LEFT, +z up). Negating y is the whole conversion.
   *
   * It cannot be a SENS_MAG0_ROT value. The part's triad is left handed
   * against ours, and no rotation changes handedness - only a reflection
   * does. The enum in rotation.h contains proper rotations only, and
   * deliberately so.
   *
   * It happens HERE, after cal_mag_apply, and not in the driver: a
   * calibration is fitted against the raw stream, so a flip applied before
   * the fit would invalidate every stored CAL_MAG0_* value. Applied after,
   * the frame can be changed freely without recalibrating.
   *
   * Left uncorrected this is invisible in five of six axes. Heading is
   * atan2(-y, x), so a flipped y flips the heading - while roll and pitch
   * stay perfect, because neither of them ever touches mag y.
   */

  out[1] = -out[1];

  /* Sensor axes -> board axes -> vehicle axes. Both rotations were validated
   * at load, so a failure here cannot happen; treat it as one anyway rather
   * than publishing a field in the wrong frame.
   */

  if (!rotation_apply(frame->mag_rot, out))
    {
      return false;
    }

  rotate_by_vector(frame->fine_rv, out);

  if (!rotation_apply(frame->board_rot, out))
    {
      return false;
    }

  return vector_finite(out);
}
