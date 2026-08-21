/****************************************************************************
 * apps/cal/cal_mag.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Full three-dimensional magnetometer ellipsoid calibration. The model is
 *
 *     corrected = matrix * (raw - offset)
 *
 * where matrix is symmetric. Its diagonal corrects axis scale and its three
 * off-diagonal terms correct soft-iron cross-axis distortion. A norm-only
 * tumble cannot distinguish an arbitrary rotation, so storing a symmetric
 * matrix avoids inventing one; sensor mounting rotation is handled later.
 ****************************************************************************/

#ifndef __APPS_CAL_CAL_MAG_H
#define __APPS_CAL_CAL_MAG_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

#define CAL_MAG_MAX_SAMPLES  320
#define CAL_MAG_MIN_SAMPLES  120

enum cal_mag_result_e
{
  CAL_MAG_OK = 0,
  CAL_MAG_NEED_SAMPLES,
  CAL_MAG_POOR_COVERAGE,
  CAL_MAG_SINGULAR,
  CAL_MAG_NOT_ELLIPSOID,
  CAL_MAG_FIELD_RANGE,
  CAL_MAG_OFFSET_RANGE,
  CAL_MAG_SCALE_RANGE,
  CAL_MAG_RESIDUAL_HIGH
};

struct cal_mag_s
{
  float samples[CAL_MAG_MAX_SAMPLES][3];
  float minimum[3];
  float maximum[3];
  uint16_t count;
  uint16_t seen;
};

struct cal_mag_fit_s
{
  float offset[3];
  float matrix[3][3];
  float field;
  float rms;
  float maximum_error;
  float condition;
  uint16_t used;
  uint16_t rejected;
  uint8_t octants;
};

void cal_mag_reset(FAR struct cal_mag_s *cal);

/* Add a finite, physically plausible, spatially distinct sample. Returns true
 * if retained. Dense samples from one orientation are deliberately ignored.
 */

bool cal_mag_add(FAR struct cal_mag_s *cal, FAR const float sample[3]);

/* Fit and validate a complete ellipsoid. No result is usable unless this
 * returns CAL_MAG_OK.
 */

enum cal_mag_result_e cal_mag_solve(FAR const struct cal_mag_s *cal,
                                    FAR struct cal_mag_fit_s *fit);

void cal_mag_apply(FAR const struct cal_mag_fit_s *fit,
                   FAR const float raw[3], FAR float corrected[3]);

FAR const char *cal_mag_result_string(enum cal_mag_result_e result);

#endif /* __APPS_CAL_CAL_MAG_H */
