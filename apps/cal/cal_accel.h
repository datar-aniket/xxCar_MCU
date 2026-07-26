/****************************************************************************
 * apps/cal/cal_accel.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Six-position accelerometer calibration.
 *
 * The model is per-axis offset and scale, applied as
 *
 *     corrected = (raw - offset) * scale
 *
 * With an axis pointing up and then down, gravity gives two readings that
 * bracket that axis's response: the offset is their midpoint and the scale is
 * the gravity span divided by their difference. Each axis is independent, so
 * there is no solver - which matters because it means the result cannot be
 * quietly wrong in the way a badly conditioned fit can.
 *
 * Gravity is a free and exact 1 g reference, so this is genuinely accurate for
 * offset and scale. It does NOT recover cross-axis misalignment: three
 * measurements of a diagonal cannot say anything about off-diagonal terms.
 * That needs the tumble-and-ellipsoid procedure, and is deliberately not
 * pretended at here.
 *
 * No NuttX dependencies: this is the part worth testing on the host, because a
 * sign error or a swapped position still produces numbers of entirely
 * plausible magnitude.
 ****************************************************************************/

#ifndef __APPS_CAL_CAL_ACCEL_H
#define __APPS_CAL_CAL_ACCEL_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

#define CAL_G          9.80665f
#define CAL_NPOS       6

/* Position index: axis * 2, plus one when that axis reads negative.
 *
 *   0  +X up    1  -X up
 *   2  +Y up    3  -Y up
 *   4  +Z up    5  -Z up
 */

struct cal_accel_s
{
  float mean[CAL_NPOS][3];
  bool  have[CAL_NPOS];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void cal_accel_reset(FAR struct cal_accel_s *s);

/* Which of the six positions this reading represents, or -1 if the board is
 * not close enough to an axis for the reading to mean one.
 *
 * Rejecting is the point. A board resting at 30 degrees still produces a
 * confident-looking vector, and accepting it would put a systematically wrong
 * value into the fit with nothing downstream able to notice.
 */

int cal_accel_classify(FAR const float a[3]);

/* Record an averaged reading. Returns the position filled, or -1 if the
 * reading is not axis-aligned enough to be one.
 */

int cal_accel_add(FAR struct cal_accel_s *s, FAR const float a[3]);

/* How many of the six are done, and whether all are. */

int  cal_accel_count(FAR const struct cal_accel_s *s);
bool cal_accel_complete(FAR const struct cal_accel_s *s);

/* Solve for offset and scale.
 *
 * `residual` is the RMS error in the corrected gravity magnitude across all
 * six positions, in m/s^2 - it answers "is this calibration any good?", which
 * the offsets alone cannot. Returns 0, or a negative errno if the six are not
 * all present or a pair is degenerate.
 */

int cal_accel_solve(FAR const struct cal_accel_s *s, FAR float off[3],
                    FAR float scl[3], FAR float *residual);

/* Mean and standard deviation of an interleaved sample block.
 *
 * `samples` holds n frames of nv values. Standard deviation is what decides
 * stillness: judging it by whether any single sample strays from the mean
 * fails, because sensor noise alone exceeds any threshold tight enough to
 * catch real motion.
 */

void cal_stats(FAR const float *samples, int n, int nv, FAR float *mean,
               FAR float *sd);

#endif /* __APPS_CAL_CAL_ACCEL_H */
