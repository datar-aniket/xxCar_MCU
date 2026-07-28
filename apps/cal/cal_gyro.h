/****************************************************************************
 * apps/cal/cal_gyro.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Gyroscope zero-rate bias, measured by holding the board still.
 *
 * There is no six-position equivalent here and there should not be: gravity
 * gives the accelerometer a known reference in every orientation, and a
 * gyroscope has none. Its scale factor cannot be measured without a rate
 * table, so - as PX4 and every other flight stack does - only the bias is
 * calibrated, and the manufacturer's sensitivity is trusted.
 ****************************************************************************/

#ifndef __APPS_CAL_CAL_GYRO_H
#define __APPS_CAL_CAL_GYRO_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

/* Accumulator. Streaming rather than a sample buffer, because the useful
 * window is longer than a buffer worth keeping: the accel still-capture stops
 * at 200 samples, which at 500 Hz is 0.4 s, and for a gyro bias that leaves
 * white noise where the bias instability floor should be.
 *
 * Welford's method, not sum and sum-of-squares. That is not a preference. The
 * naive form computes the variance as sumsq/n - mean^2, and this measurement
 * has exactly the shape that destroys: a board turning steadily produces a
 * LARGE mean with a TINY spread about it, so the two terms are nearly equal
 * and subtracting them cancels away the answer. Measured, at 0.9 rad/s with
 * 1.6e-3 of noise over 8000 samples, single-precision sums give
 *
 *     true sd 0.00159      sum/sumsq in float: 0.00000
 *
 * - not degraded, gone. A standard deviation of exactly zero reads as
 * "perfectly still", which is the one conclusion that must never be reached
 * about a rotating board. Double sums survive this particular case, but only
 * by margin, and relying on margin against catastrophic cancellation is how it
 * comes back at a different rate. Welford has no such term.
 *
 * From the board's own Allan run, N is about 0.24 deg/sqrt(h), so
 * 7e-5 rad/s/sqrt(Hz), and at 500 Hz a single sample carries roughly
 * 1.6e-3 rad/s of noise. Averaging 200 of them leaves 1.1e-4 rad/s
 * (0.0063 deg/s); averaging 2000 leaves 3.5e-5 (0.002 deg/s), which is down at
 * the bias instability itself. Four seconds is therefore worth ten times what
 * 0.4 s is, and costs the operator nothing.
 */

struct cal_bias_s
{
  double mean[3];
  double m2[3];
  int    n;
};

/* Largest per-axis standard deviation still considered "held still", rad/s.
 *
 * 0.01 rad/s is 0.57 deg/s, about six times the ~1.6e-3 rad/s a single sample
 * carries at 500 Hz - the same margin over sensor noise the accelerometer's
 * 0.08 m/s^2 stillness test uses - and far below anything a board being
 * touched, or resting on a surface someone is leaning on, produces.
 */

#define CAL_GYRO_SD_MAX    0.01f

/* Largest bias that can be a bias rather than motion, rad/s.
 *
 * A board rotating at a CONSTANT rate passes the stillness test perfectly -
 * the standard deviation of a steady rotation is zero - and its rate would be
 * stored as zero-rate offset. This is the check that catches it.
 *
 * 0.2 rad/s is 11.5 deg/s. The ICM-42688's zero-rate output is specified at
 * +/-0.5 deg/s typical and a few deg/s over temperature, so this is an order
 * of magnitude above any honest bias and still far below a board being turned.
 * Earth's rotation, 7.3e-5 rad/s, is two orders below the measurement floor
 * and is neither corrected for nor detectable here.
 */

#define CAL_GYRO_BIAS_MAX  0.2f

/* Fewest samples that may be averaged into a stored bias. Below this the
 * estimate is dominated by white noise rather than by the bias it is meant to
 * measure - see the arithmetic above.
 */

#define CAL_GYRO_MIN_N     1000

void cal_bias_reset(FAR struct cal_bias_s *b);
void cal_bias_add(FAR struct cal_bias_s *b, FAR const float v[3]);

/* Mean and per-axis standard deviation of everything added so far.
 * Returns the sample count, or 0 if nothing was added.
 */

int cal_bias_result(FAR const struct cal_bias_s *b, FAR float mean[3],
                    FAR float sd[3]);

/* Why a bias measurement cannot be stored, or CAL_GYRO_OK. */

enum cal_gyro_verdict_e
{
  CAL_GYRO_OK = 0,
  CAL_GYRO_TOO_FEW,      /* not enough samples to average */
  CAL_GYRO_NOT_STILL,    /* standard deviation over CAL_GYRO_SD_MAX */
  CAL_GYRO_TOO_LARGE     /* steady, but turning - or a broken sensor */
};

enum cal_gyro_verdict_e cal_gyro_judge(int n, FAR const float mean[3],
                                       FAR const float sd[3]);

#endif /* __APPS_CAL_CAL_GYRO_H */
