/****************************************************************************
 * apps/sensors/rotation.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fixed sensor rotations, numbered exactly as PX4 numbers them.
 *
 * The values are PX4's `enum Rotation` from src/lib/conversion/rotation.h,
 * fetched and transcribed rather than remembered, so a rotation copied from a
 * PX4 airframe or a forum answer means here what it means there. Diverging by
 * one would be invisible: every value is a plausible orientation, so a
 * mismatch does not fail, it silently reads the vehicle sideways.
 *
 * An ENUM, not roll/pitch/yaw angles. Free-form angles let a parameter file
 * describe a frame that is not orthogonal - or one that is orthogonal but
 * arrived at through rounding - and a diagonal scale calibration is only
 * meaningful under a rotation that maps axes onto axes. With an enum the set
 * of representable frames is exactly the set of physically sensible ones for a
 * board bolted to a chassis.
 *
 * Only the 24 rotations that ARE axis permutations are supported: the 90-degree
 * multiples, done as exact swaps and sign flips with no trigonometry, so a
 * rotated reading is bit-for-bit the same magnitude as the original. PX4's own
 * rotate_3() handles precisely this set with exact swaps and returns false for
 * the rest; the 45-degree entries need a sqrt(2)/2 matrix. Those are REFUSED
 * here rather than approximated - see rotation_supported().
 ****************************************************************************/

#ifndef __APPS_SENSORS_ROTATION_H
#define __APPS_SENSORS_ROTATION_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

enum rotation_e
{
  ROTATION_NONE                     = 0,
  ROTATION_YAW_45                   = 1,
  ROTATION_YAW_90                   = 2,
  ROTATION_YAW_135                  = 3,
  ROTATION_YAW_180                  = 4,
  ROTATION_YAW_225                  = 5,
  ROTATION_YAW_270                  = 6,
  ROTATION_YAW_315                  = 7,
  ROTATION_ROLL_180                 = 8,
  ROTATION_ROLL_180_YAW_45          = 9,
  ROTATION_ROLL_180_YAW_90          = 10,
  ROTATION_ROLL_180_YAW_135         = 11,
  ROTATION_PITCH_180                = 12,
  ROTATION_ROLL_180_YAW_225         = 13,
  ROTATION_ROLL_180_YAW_270         = 14,
  ROTATION_ROLL_180_YAW_315         = 15,
  ROTATION_ROLL_90                  = 16,
  ROTATION_ROLL_90_YAW_45           = 17,
  ROTATION_ROLL_90_YAW_90           = 18,
  ROTATION_ROLL_90_YAW_135          = 19,
  ROTATION_ROLL_270                 = 20,
  ROTATION_ROLL_270_YAW_45          = 21,
  ROTATION_ROLL_270_YAW_90          = 22,
  ROTATION_ROLL_270_YAW_135         = 23,
  ROTATION_PITCH_90                 = 24,
  ROTATION_PITCH_270                = 25,
  ROTATION_PITCH_180_YAW_90         = 26,
  ROTATION_PITCH_180_YAW_270        = 27,
  ROTATION_ROLL_90_PITCH_90         = 28,
  ROTATION_ROLL_180_PITCH_90        = 29,
  ROTATION_ROLL_270_PITCH_90        = 30,
  ROTATION_ROLL_90_PITCH_180        = 31,
  ROTATION_ROLL_270_PITCH_180       = 32,
  ROTATION_ROLL_90_PITCH_270        = 33,
  ROTATION_ROLL_180_PITCH_270       = 34,
  ROTATION_ROLL_270_PITCH_270       = 35,
  ROTATION_ROLL_90_PITCH_180_YAW_90 = 36,
  ROTATION_ROLL_90_YAW_270          = 37,
  ROTATION_MAX_SUPPORTED            = 38
};

/* Rotate v in place. Returns false, leaving v untouched, for a rotation this
 * implementation cannot perform exactly - the 45-degree entries.
 *
 * A caller must not treat false as "no rotation needed": that is precisely the
 * case where the data is in the wrong frame and must not be published as if it
 * were in the right one.
 */

bool rotation_apply(uint8_t rot, FAR float v[3]);

/* Whether rotation_apply() will accept this value. Used to validate a
 * parameter at load rather than discovering it per sample.
 */

bool rotation_supported(uint8_t rot);

/* Human-readable name, or "unsupported". */

FAR const char *rotation_name(uint8_t rot);

#endif /* __APPS_SENSORS_ROTATION_H */
