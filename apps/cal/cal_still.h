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
  float gyro_thresh;    /* rad/s, per axis, instantaneous */
  float accel_thresh;   /* m/s^2, standard deviation over the window */
  int   min_samples;
  int   count;          /* consecutive still samples so far */
  float acc_sum[3];
  float acc_sq[3];      /* sum of squares, for the dispersion test */
  float gyr_sum[3];

  /* Diagnostics, so a refusal can say what it actually measured rather than
   * just "not steady" - see cal_still_report().
   */

  int   best_count;     /* highest count reached since reset */
  float last_sd[3];     /* accel stddev at the last evaluation */
  int   gyro_resets;    /* windows thrown away because the gyro moved */
  int   accel_resets;   /* windows thrown away because accel dispersion was high */
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

/* What the detector actually measured, for a refusal message. Without this a
 * failed capture is indistinguishable from a broken sensor, a threshold set
 * below the noise floor, or a hand that genuinely will not hold still - and
 * those need completely different responses.
 */

void cal_still_report(FAR const struct cal_still_s *s, FAR int *best_count,
                      FAR float sd_out[3], FAR int *gyro_resets,
                      FAR int *accel_resets);

#endif /* __APPS_CAL_CAL_STILL_H */
