/****************************************************************************
 * apps/sensors/mag_frame.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Gets a calibrated magnetometer reading into the vehicle's frame.
 *
 *     corrected = cal_mag_apply(fit, raw)          apps/cal/cal_mag.c
 *     body      = BOARD_ROT( FINE_ROT( MAG_ROT( corrected ) ) )
 *
 * The calibration mathematics is NOT repeated here. cal_mag_apply() already
 * does the hard-iron subtraction and the 3x3 soft-iron multiply, and it is
 * host- and UBSan-tested through tools/test-cal-mag.sh. This file adds only
 * the two things that did not exist: loading the stored CAL_MAG0_*
 * parameters into a fit, and rotating the result into the body frame.
 *
 * Rotation is the gap that mattered. ist8310.c publishes in raw sensor axes
 * and defers framing to "the fusion stage", but no fusion stage existed, so
 * SENS_MAG0_ROT and CAL_MAG0_RV* had never been applied by anything. Heading
 * comes from the HORIZONTAL components of the field, so an unrotated field
 * yields a heading in the sensor's frame rather than the vehicle's - a fixed
 * error on any board where the magnetometer is not mounted square.
 *
 * The order is the same argument sensors.h makes for the IMU: calibration is
 * measured in the SENSOR's own axes, so it must be applied before any
 * rotation. A soft-iron matrix means nothing once the axes have been mixed.
 ****************************************************************************/

#ifndef __APPS_SENSORS_MAG_FRAME_H
#define __APPS_SENSORS_MAG_FRAME_H

#include <stdbool.h>
#include <stdint.h>

#include "../cal/cal_mag.h"

#ifndef FAR
#  define FAR
#endif

struct mag_frame_s
{
  struct cal_mag_fit_s fit;   /* CAL_MAG0_{X,Y,Z}OFF, _{XX,YY,ZZ,XY,XZ,YZ},
                               * _FIELD - the same shape cal_mag_validate()
                               * and cal_mag_apply() already take. */
  float   fine_rv[3];         /* CAL_MAG0_RV{X,Y,Z}, rotation vector, rad */
  uint8_t mag_rot;            /* SENS_MAG0_ROT, sensor to board */
  uint8_t board_rot;          /* SENS_BOARD_ROT, board to vehicle */
  bool    valid;              /* CAL_MAG0_OK and both rotations supported */
};

/* Read the stored calibration and mounting into frame.
 *
 * Always fills frame and always sets frame->valid, returning the same value.
 * False means CAL_MAG0_OK is clear, the stored fit does not survive
 * cal_mag_validate(), or a rotation is one rotation_apply() cannot perform
 * exactly. That is not an error: the caller publishes the raw field marked
 * uncalibrated so the topic stays observable, and the estimator declines to
 * fuse it.
 */

bool mag_frame_load(FAR struct mag_frame_s *frame);

/* Calibrate raw and rotate it into the body frame.
 *
 * With frame->valid false, copies raw to out unchanged and returns false: the
 * reading is the right shape, just not in the right frame. Returns false
 * leaving out untouched when raw is not finite.
 */

bool mag_frame_apply(FAR const struct mag_frame_s *frame,
                     FAR const float raw[3], FAR float out[3]);

#endif /* __APPS_SENSORS_MAG_FRAME_H */
