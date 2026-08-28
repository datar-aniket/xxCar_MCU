/****************************************************************************
 * apps/ekf3/ekf_extnav_frame.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <math.h>
#include <stddef.h>

#include "ekf_extnav_frame.h"

static bool vector_finite(FAR const float vector[3])
{
  return isfinite(vector[0]) && isfinite(vector[1]) && isfinite(vector[2]);
}

static void euler_to_rotation(FAR const float euler[3], float rotation[3][3])
{
  float cr = cosf(euler[0]);
  float sr = sinf(euler[0]);
  float cp = cosf(euler[1]);
  float sp = sinf(euler[1]);
  float cy = cosf(euler[2]);
  float sy = sinf(euler[2]);

  rotation[0][0] = cy * cp;
  rotation[0][1] = cy * sp * sr - sy * cr;
  rotation[0][2] = cy * sp * cr + sy * sr;
  rotation[1][0] = sy * cp;
  rotation[1][1] = sy * sp * sr + cy * cr;
  rotation[1][2] = sy * sp * cr - cy * sr;
  rotation[2][0] = -sp;
  rotation[2][1] = cp * sr;
  rotation[2][2] = cp * cr;
}

static float wrap_pi(float angle)
{
  const float pi = 3.14159265358979323846f;

  while (angle > pi)
    {
      angle -= 2.0f * pi;
    }

  while (angle < -pi)
    {
      angle += 2.0f * pi;
    }

  return angle;
}

bool ekf_extnav_apply_extrinsics(
  FAR const struct ekf_extnav_extrinsics_s *extrinsics,
  FAR const float marker_position[3], FAR const float marker_rpy[3],
  FAR float body_position[3], FAR float body_rpy[3])
{
  float marker_rotation[3][3];
  float relative_rotation[3][3];
  float body_rotation[3][3];
  float pitch_argument;
  int row;
  int column;
  int inner;

  if (extrinsics == NULL || marker_position == NULL || marker_rpy == NULL ||
      body_position == NULL || body_rpy == NULL ||
      !vector_finite(extrinsics->position) ||
      !vector_finite(extrinsics->rotation) ||
      !vector_finite(marker_position) || !vector_finite(marker_rpy))
    {
      return false;
    }

  euler_to_rotation(marker_rpy, marker_rotation);
  euler_to_rotation(extrinsics->rotation, relative_rotation);

  for (row = 0; row < 3; row++)
    {
      body_position[row] = marker_position[row];

      for (inner = 0; inner < 3; inner++)
        {
          body_position[row] += marker_rotation[row][inner] *
                                extrinsics->position[inner];
        }

      for (column = 0; column < 3; column++)
        {
          body_rotation[row][column] = 0.0f;

          for (inner = 0; inner < 3; inner++)
            {
              body_rotation[row][column] +=
                marker_rotation[row][inner] *
                relative_rotation[inner][column];
            }
        }
    }

  pitch_argument = -body_rotation[2][0];

  if (pitch_argument > 1.0f)
    {
      pitch_argument = 1.0f;
    }
  else if (pitch_argument < -1.0f)
    {
      pitch_argument = -1.0f;
    }

  body_rpy[0] = atan2f(body_rotation[2][1], body_rotation[2][2]);
  body_rpy[1] = asinf(pitch_argument);
  body_rpy[2] = wrap_pi(atan2f(body_rotation[1][0],
                               body_rotation[0][0]));
  return vector_finite(body_position) && vector_finite(body_rpy);
}

bool ekf_extnav_transform_planar_covariance(
  FAR const struct ekf_extnav_extrinsics_s *extrinsics, float marker_yaw,
  FAR const float marker_covariance[6], FAR float body_covariance[6])
{
  float covariance[3][3];
  float jacobian[3][3] = {{1.0f, 0.0f, 0.0f},
                          {0.0f, 1.0f, 0.0f},
                          {0.0f, 0.0f, 1.0f}};
  float intermediate[3][3] = {{0.0f}};
  float transformed[3][3] = {{0.0f}};
  float c;
  float s;
  int row;
  int column;
  int inner;

  if (extrinsics == NULL || marker_covariance == NULL ||
      body_covariance == NULL || !vector_finite(extrinsics->position) ||
      !isfinite(marker_yaw))
    {
      return false;
    }

  for (row = 0; row < 6; row++)
    {
      if (!isfinite(marker_covariance[row]))
        {
          return false;
        }
    }

  covariance[0][0] = marker_covariance[0];
  covariance[0][1] = covariance[1][0] = marker_covariance[1];
  covariance[0][2] = covariance[2][0] = marker_covariance[2];
  covariance[1][1] = marker_covariance[3];
  covariance[1][2] = covariance[2][1] = marker_covariance[4];
  covariance[2][2] = marker_covariance[5];

  c = cosf(marker_yaw);
  s = sinf(marker_yaw);
  jacobian[0][2] = -s * extrinsics->position[0] -
                    c * extrinsics->position[1];
  jacobian[1][2] = c * extrinsics->position[0] -
                    s * extrinsics->position[1];

  for (row = 0; row < 3; row++)
    {
      for (column = 0; column < 3; column++)
        {
          for (inner = 0; inner < 3; inner++)
            {
              intermediate[row][column] +=
                jacobian[row][inner] * covariance[inner][column];
            }
        }
    }

  for (row = 0; row < 3; row++)
    {
      for (column = 0; column < 3; column++)
        {
          for (inner = 0; inner < 3; inner++)
            {
              transformed[row][column] +=
                intermediate[row][inner] * jacobian[column][inner];
            }
        }
    }

  body_covariance[0] = transformed[0][0];
  body_covariance[1] = transformed[0][1];
  body_covariance[2] = transformed[0][2];
  body_covariance[3] = transformed[1][1];
  body_covariance[4] = transformed[1][2];
  body_covariance[5] = transformed[2][2];
  return true;
}
