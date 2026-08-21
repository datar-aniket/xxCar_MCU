/****************************************************************************
 * apps/sensors/mag_frame.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Gets a calibrated magnetometer reading into the vehicle's frame.
 *
 *     corrected = cal_mag_apply(fit, raw)          apps/cal/cal_mag.c
 *     handed    = (x, -y, z)                       IST8310 FRU -> FLU
 *     body      = BOARD_ROT( FINE_ROT( MAG_ROT( handed ) ) )
 *
 * The vehicle convention is +x FORWARD, +y LEFT, +z UP, which is what the
 * ICM-42688-P and BMI055 already report - PX4's own drivers describe both
 * parts as "+x forward, +y left, +z up" before flipping them into its FRD
 * board frame, a flip this project does not make.
 *
 * The IST8310 is the odd one out: PX4 describes it as "+x forward, +y RIGHT,
 * +z up", a LEFT-handed triad against ours. Negating y is the whole
 * conversion, and it cannot be a SENS_MAG0_ROT value, because no rotation
 * changes handedness - only a reflection does, and rotation.h contains
 * proper rotations only.
 *
 * ORDER. Calibration is fitted against the driver's RAW stream, so it must
 * be applied first and nothing before it may touch the axes. Everything
 * after - handedness, mounting, board - can then be changed freely without
 * recalibrating the sensor. That is the reason the flip lives here and not
 * in ist8310.c, and the reason a soft-iron matrix means nothing once the
 * axes have been mixed.
 *
 * The calibration mathematics itself is NOT repeated here. cal_mag_apply()
 * already does the hard-iron subtraction and the 3x3 soft-iron multiply, and
 * is host- and UBSan-tested through tools/test-cal-mag.sh.
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
