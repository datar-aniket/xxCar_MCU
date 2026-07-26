/****************************************************************************
 * apps/imu_cal/imu_cal.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Applies stored IMU calibration to a raw sample.
 *
 * The correction is  true = M * (measured - b),  where M is stored ALREADY
 * INVERTED by whatever produced the calibration. That choice is what keeps this
 * on the hot path: correcting a sample is nine multiplies and three subtracts,
 * with no solve and no division.
 *
 * This library only APPLIES a calibration. It never computes one - fits run on
 * the host, where they can be visualised and improved without a reflash. And it
 * never touches the raw uORB topics: Allan variance and temperature fitting both
 * need genuinely raw data, permanently.
 ****************************************************************************/

#ifndef __APPS_IMU_CAL_IMU_CAL_H
#define __APPS_IMU_CAL_IMU_CAL_H

#include <stdbool.h>

#ifndef FAR
#  define FAR
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum imu_cal_sensor_e
{
  IMU_CAL_ACC0 = 0,
  IMU_CAL_GYR0,
  IMU_CAL_ACC1,
  IMU_CAL_GYR1,
  IMU_CAL_NSENSORS
};

struct imu_cal_s
{
  float M[9];    /* row-major 3x3, pre-inverted */
  float b[3];    /* bias, in the sensor's own units */
  bool  valid;   /* false: apply() passes samples through untouched */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Read one sensor's calibration out of the parameter store.
 *
 * Returns OK and sets valid=true only when CAL_<S>_OK is 1 AND every element is
 * inside its parameter bounds. A diverged fit therefore degrades to raw
 * passthrough instead of corrupting every sample downstream.
 */

int imu_cal_load(enum imu_cal_sensor_e sensor, FAR struct imu_cal_s *out);

/* Correct one sample. in and out may be the same array. */

void imu_cal_apply(FAR const struct imu_cal_s *cal,
                   FAR const float in[3], FAR float out[3]);

#endif /* __APPS_IMU_CAL_IMU_CAL_H */
