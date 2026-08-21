/****************************************************************************
 * apps/cal/cal_mag.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Safety boundary for host-fitted magnetometer calibration candidates.
 ****************************************************************************/

#ifndef __APPS_CAL_CAL_MAG_H
#define __APPS_CAL_CAL_MAG_H

#ifndef FAR
#  define FAR
#endif

enum cal_mag_result_e
{
  CAL_MAG_OK = 0,
  CAL_MAG_NONFINITE,
  CAL_MAG_FIELD_RANGE,
  CAL_MAG_OFFSET_RANGE,
  CAL_MAG_NOT_POSITIVE_DEFINITE,
  CAL_MAG_SCALE_RANGE
};

struct cal_mag_fit_s
{
  float offset[3];
  float matrix[3][3];
  float field;
};

/* The MCU does not fit. It independently rejects unsafe host candidates. */

enum cal_mag_result_e cal_mag_validate(FAR const struct cal_mag_fit_s *fit);

void cal_mag_apply(FAR const struct cal_mag_fit_s *fit,
                   FAR const float raw[3], FAR float corrected[3]);

FAR const char *cal_mag_result_string(enum cal_mag_result_e result);

#endif /* __APPS_CAL_CAL_MAG_H */
