/****************************************************************************
 * apps/sensors/rotation.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See rotation.h. The swaps below are transcribed case for case from PX4's
 * rotate_3() template (src/lib/conversion/rotation.h), including its two
 * aliased pairs - ROLL_180_YAW_90 with PITCH_180_YAW_270, and ROLL_180_YAW_270
 * with PITCH_180_YAW_90 - which really are the same permutation reached by
 * different Euler routes.
 ****************************************************************************/

#ifndef ROTATION_HOST_TEST
#  include <nuttx/config.h>
#endif

#include "rotation.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool rotation_apply(uint8_t rot, FAR float v[3])
{
  float tmp;

  switch (rot)
    {
      case ROTATION_NONE:
        return true;

      case ROTATION_YAW_90:
        tmp = v[0]; v[0] = -v[1]; v[1] = tmp;
        return true;

      case ROTATION_YAW_180:
        v[0] = -v[0]; v[1] = -v[1];
        return true;

      case ROTATION_YAW_270:
        tmp = v[0]; v[0] = v[1]; v[1] = -tmp;
        return true;

      case ROTATION_ROLL_180:
        v[1] = -v[1]; v[2] = -v[2];
        return true;

      case ROTATION_ROLL_180_YAW_90:
      case ROTATION_PITCH_180_YAW_270:
        tmp = v[0]; v[0] = v[1]; v[1] = tmp; v[2] = -v[2];
        return true;

      case ROTATION_PITCH_180:
        v[0] = -v[0]; v[2] = -v[2];
        return true;

      case ROTATION_ROLL_180_YAW_270:
      case ROTATION_PITCH_180_YAW_90:
        tmp = v[0]; v[0] = -v[1]; v[1] = -tmp; v[2] = -v[2];
        return true;

      case ROTATION_ROLL_90:
        tmp = v[2]; v[2] = v[1]; v[1] = -tmp;
        return true;

      case ROTATION_ROLL_90_YAW_90:
        tmp = v[0]; v[0] = v[2]; v[2] = v[1]; v[1] = tmp;
        return true;

      case ROTATION_ROLL_270:
        tmp = v[2]; v[2] = -v[1]; v[1] = tmp;
        return true;

      case ROTATION_ROLL_270_YAW_90:
        tmp = v[0]; v[0] = -v[2]; v[2] = -v[1]; v[1] = tmp;
        return true;

      case ROTATION_PITCH_90:
        tmp = v[2]; v[2] = -v[0]; v[0] = tmp;
        return true;

      case ROTATION_PITCH_270:
        tmp = v[2]; v[2] = v[0]; v[0] = -tmp;
        return true;

      case ROTATION_ROLL_180_PITCH_270:
        tmp = v[2]; v[2] = v[0]; v[0] = tmp; v[1] = -v[1];
        return true;

      case ROTATION_ROLL_90_YAW_270:
        tmp = v[0]; v[0] = -v[2]; v[2] = v[1]; v[1] = -tmp;
        return true;

      case ROTATION_ROLL_90_PITCH_90:
        tmp = v[0]; v[0] = v[1]; v[1] = -v[2]; v[2] = -tmp;
        return true;

      case ROTATION_ROLL_180_PITCH_90:
        tmp = v[0]; v[0] = -v[2]; v[1] = -v[1]; v[2] = -tmp;
        return true;

      case ROTATION_ROLL_270_PITCH_90:
        tmp = v[0]; v[0] = -v[1]; v[1] = v[2]; v[2] = -tmp;
        return true;

      case ROTATION_ROLL_90_PITCH_180:
        tmp = v[1]; v[0] = -v[0]; v[1] = -v[2]; v[2] = -tmp;
        return true;

      case ROTATION_ROLL_270_PITCH_180:
        tmp = v[1]; v[0] = -v[0]; v[1] = v[2]; v[2] = tmp;
        return true;

      case ROTATION_ROLL_90_PITCH_270:
        tmp = v[0]; v[0] = -v[1]; v[1] = -v[2]; v[2] = tmp;
        return true;

      case ROTATION_ROLL_270_PITCH_270:
        tmp = v[0]; v[0] = v[1]; v[1] = v[2]; v[2] = tmp;
        return true;

      case ROTATION_ROLL_90_PITCH_180_YAW_90:
        tmp = v[0]; v[0] = v[2]; v[2] = -v[1]; v[1] = -tmp;
        return true;

      default:

        /* The 45-degree rotations, and anything out of range. Leave the vector
         * alone and say so: silently passing it through would publish a
         * reading in the sensor's frame labelled as the vehicle's.
         */

        return false;
    }
}

bool rotation_supported(uint8_t rot)
{
  float probe[3] =
  {
    0.0f, 0.0f, 0.0f
  };

  return rotation_apply(rot, probe);
}

FAR const char *rotation_name(uint8_t rot)
{
  switch (rot)
    {
      case ROTATION_NONE:                     return "none";
      case ROTATION_YAW_90:                   return "yaw 90";
      case ROTATION_YAW_180:                  return "yaw 180";
      case ROTATION_YAW_270:                  return "yaw 270";
      case ROTATION_ROLL_180:                 return "roll 180";
      case ROTATION_ROLL_180_YAW_90:          return "roll 180 yaw 90";
      case ROTATION_PITCH_180:                return "pitch 180";
      case ROTATION_ROLL_180_YAW_270:         return "roll 180 yaw 270";
      case ROTATION_ROLL_90:                  return "roll 90";
      case ROTATION_ROLL_90_YAW_90:           return "roll 90 yaw 90";
      case ROTATION_ROLL_270:                 return "roll 270";
      case ROTATION_ROLL_270_YAW_90:          return "roll 270 yaw 90";
      case ROTATION_PITCH_90:                 return "pitch 90";
      case ROTATION_PITCH_270:                return "pitch 270";
      case ROTATION_PITCH_180_YAW_90:         return "pitch 180 yaw 90";
      case ROTATION_PITCH_180_YAW_270:        return "pitch 180 yaw 270";
      case ROTATION_ROLL_90_PITCH_90:         return "roll 90 pitch 90";
      case ROTATION_ROLL_180_PITCH_90:        return "roll 180 pitch 90";
      case ROTATION_ROLL_270_PITCH_90:        return "roll 270 pitch 90";
      case ROTATION_ROLL_90_PITCH_180:        return "roll 90 pitch 180";
      case ROTATION_ROLL_270_PITCH_180:       return "roll 270 pitch 180";
      case ROTATION_ROLL_90_PITCH_270:        return "roll 90 pitch 270";
      case ROTATION_ROLL_180_PITCH_270:       return "roll 180 pitch 270";
      case ROTATION_ROLL_270_PITCH_270:       return "roll 270 pitch 270";
      case ROTATION_ROLL_90_PITCH_180_YAW_90: return "roll 90 pitch 180 yaw 90";
      case ROTATION_ROLL_90_YAW_270:          return "roll 90 yaw 270";
      default:                                return "unsupported";
    }
}
