/****************************************************************************
 * apps/cal/cal_still.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Decides when the board has been held motionless long enough to average a
 * sample from it.
 *
 * This is the gate on every captured orientation, and both ways of getting it
 * wrong are costly: declaring still too early averages a gravity vector that is
 * still moving, which poisons the fit with no outward sign; never declaring
 * still makes the procedure impossible to finish. Motion resets progress rather
 * than pausing it, so a window can never span a movement.
 ****************************************************************************/

#ifndef __APPS_CAL_CAL_STILL_H
#define __APPS_CAL_CAL_STILL_H

#include <stdbool.h>

#ifndef FAR
#  define FAR
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct cal_still_s
{
  float gyro_thresh;    /* rad/s, per axis */
  float accel_thresh;   /* m/s^2, deviation from the running mean */
  int   min_samples;
  int   count;          /* consecutive still samples so far */
  float acc_sum[3];
  float gyr_sum[3];
  float acc_mean[3];    /* mean of the current still run */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void cal_still_reset(FAR struct cal_still_s *s, float gyro_thresh,
                     float accel_thresh, int min_samples);

/* Feed one sample. Returns true on the sample that completes a full still
 * window, and on every still sample after it, so a caller may capture on the
 * first true and keep averaging.
 */

bool cal_still_update(FAR struct cal_still_s *s, FAR const float acc[3],
                      FAR const float gyr[3]);

/* Mean over the current still run. Undefined unless update() has returned
 * true.
 */

void cal_still_mean(FAR const struct cal_still_s *s, FAR float acc_out[3],
                    FAR float gyr_out[3]);

#endif /* __APPS_CAL_CAL_STILL_H */
