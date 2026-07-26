/****************************************************************************
 * apps/imu_cal/imu_cal.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See imu_cal.h.
 ****************************************************************************/

#ifndef IMU_CAL_HOST_TEST
#  include <nuttx/config.h>
#endif

#include <stdio.h>
#include <string.h>

#include "imu_cal.h"

#ifndef IMU_CAL_HOST_TEST
#  include <errno.h>
#  include "../param/param.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void imu_cal_apply(FAR const struct imu_cal_s *cal,
                   FAR const float in[3], FAR float out[3])
{
  float d[3];
  int i;

  if (cal == NULL || !cal->valid)
    {
      /* Uncalibrated: hand back exactly what came in. Doing anything else here
       * would be worse than doing nothing.
       */

      if (out != in)
        {
          out[0] = in[0];
          out[1] = in[1];
          out[2] = in[2];
        }

      return;
    }

  /* Subtract into a local first, so a caller may pass the same array as both
   * input and output.
   */

  d[0] = in[0] - cal->b[0];
  d[1] = in[1] - cal->b[1];
  d[2] = in[2] - cal->b[2];

  for (i = 0; i < 3; i++)
    {
      out[i] = cal->M[i * 3 + 0] * d[0] +
               cal->M[i * 3 + 1] * d[1] +
               cal->M[i * 3 + 2] * d[2];
    }
}

#ifndef IMU_CAL_HOST_TEST

static const char *const g_prefix[IMU_CAL_NSENSORS] =
{
  "ACC0", "GYR0", "ACC1", "GYR1"
};

int imu_cal_load(enum imu_cal_sensor_e sensor, FAR struct imu_cal_s *out)
{
  static const char *const elem[9] =
  {
    "M00", "M01", "M02", "M10", "M11", "M12", "M20", "M21", "M22"
  };

  static const char *const axis[3] = { "BX", "BY", "BZ" };

  char name[PARAM_NAME_MAX + 1];
  int i;

  if (out == NULL || sensor < 0 || sensor >= IMU_CAL_NSENSORS)
    {
      return -EINVAL;
    }

  memset(out, 0, sizeof(*out));
  out->M[0] = out->M[4] = out->M[8] = 1.0f;

  snprintf(name, sizeof(name), "CAL_%s_OK", g_prefix[sensor]);
  if (param_i32(name) != 1)
    {
      return OK;                 /* not calibrated: identity, valid = false */
    }

  for (i = 0; i < 9; i++)
    {
      snprintf(name, sizeof(name), "CAL_%s_%s", g_prefix[sensor], elem[i]);
      out->M[i] = param_f32(name);
    }

  for (i = 0; i < 3; i++)
    {
      snprintf(name, sizeof(name), "CAL_%s_%s", g_prefix[sensor], axis[i]);
      out->b[i] = param_f32(name);
    }

  out->valid = true;
  return OK;
}

#endif /* !IMU_CAL_HOST_TEST */
