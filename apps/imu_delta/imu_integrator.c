/****************************************************************************
 * apps/imu_delta/imu_integrator.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Native-rate strapdown downsampler. Angular-rate and acceleration samples
 * are integrated over their measured timestamp intervals. A quaternion
 * carries the rotation across the 400 Hz window, preventing coning error, and
 * each midpoint delta velocity is rotated into the frame at the start of the
 * window, preventing sculling error during downsampling.
 ****************************************************************************/

#ifndef IMU_INTEGRATOR_HOST_TEST
#  include <nuttx/config.h>
#endif

#include <math.h>
#include <string.h>

#include "imu_integrator.h"

static bool vector_finite(FAR const float value[3])
{
  return isfinite(value[0]) && isfinite(value[1]) && isfinite(value[2]);
}

static void window_reset(FAR struct imu_integrator_s *integrator,
                         uint64_t timestamp_us)
{
  integrator->first_timestamp_us = timestamp_us;
  integrator->integration_us = 0;
  integrator->quaternion[0] = 1.0f;
  integrator->quaternion[1] = 0.0f;
  integrator->quaternion[2] = 0.0f;
  integrator->quaternion[3] = 0.0f;
  memset(integrator->delta_velocity, 0,
         sizeof(integrator->delta_velocity));
  integrator->samples = 0;
  integrator->clipping = 0;
}

static void seed(FAR struct imu_integrator_s *integrator,
                 uint64_t timestamp_us, FAR const float accel[3],
                 FAR const float gyro[3])
{
  memcpy(integrator->previous_accel, accel,
         sizeof(integrator->previous_accel));
  memcpy(integrator->previous_gyro, gyro,
         sizeof(integrator->previous_gyro));
  integrator->last_timestamp_us = timestamp_us;
  integrator->seeded = true;
  window_reset(integrator, timestamp_us);
}

static void cross(FAR const float a[3], FAR const float b[3],
                  FAR float result[3])
{
  result[0] = a[1] * b[2] - a[2] * b[1];
  result[1] = a[2] * b[0] - a[0] * b[2];
  result[2] = a[0] * b[1] - a[1] * b[0];
}

static void quaternion_multiply(FAR const float a[4], FAR const float b[4],
                                FAR float result[4])
{
  result[0] = a[0] * b[0] - a[1] * b[1] -
              a[2] * b[2] - a[3] * b[3];
  result[1] = a[0] * b[1] + a[1] * b[0] +
              a[2] * b[3] - a[3] * b[2];
  result[2] = a[0] * b[2] - a[1] * b[3] +
              a[2] * b[0] + a[3] * b[1];
  result[3] = a[0] * b[3] + a[1] * b[2] -
              a[2] * b[1] + a[3] * b[0];
}

/* The largest accepted interval at full gyro scale is about 0.035 rad. The
 * fourth-order small-angle quaternion is therefore much more accurate than
 * the sensor while avoiding sinf/cosf in the 2 kHz path.
 */

static void rotation_vector_quaternion(FAR const float vector[3], float scale,
                                       FAR float quaternion[4])
{
  float x = vector[0] * scale;
  float y = vector[1] * scale;
  float z = vector[2] * scale;
  float angle2 = x * x + y * y + z * z;
  float angle4 = angle2 * angle2;
  float vector_scale = 0.5f - angle2 * (1.0f / 48.0f) +
                       angle4 * (1.0f / 3840.0f);

  quaternion[0] = 1.0f - angle2 * (1.0f / 8.0f) +
                  angle4 * (1.0f / 384.0f);
  quaternion[1] = x * vector_scale;
  quaternion[2] = y * vector_scale;
  quaternion[3] = z * vector_scale;
}

static void quaternion_rotate(FAR const float quaternion[4],
                              FAR const float vector[3], FAR float result[3])
{
  float qvector[3] =
  {
    quaternion[1], quaternion[2], quaternion[3]
  };

  float first[3];
  float second[3];

  cross(qvector, vector, first);
  first[0] *= 2.0f;
  first[1] *= 2.0f;
  first[2] *= 2.0f;
  cross(qvector, first, second);

  result[0] = vector[0] + quaternion[0] * first[0] + second[0];
  result[1] = vector[1] + quaternion[0] * first[1] + second[1];
  result[2] = vector[2] + quaternion[0] * first[2] + second[2];
}

static bool quaternion_to_rotation_vector(FAR float quaternion[4],
                                          FAR float vector[3])
{
  float norm2 = quaternion[0] * quaternion[0] +
                quaternion[1] * quaternion[1] +
                quaternion[2] * quaternion[2] +
                quaternion[3] * quaternion[3];
  float inverse_norm;
  float sine_half2;
  float sine_half4;
  float scale;

  if (!isfinite(norm2) || norm2 < 0.5f || norm2 > 1.5f)
    {
      return false;
    }

  inverse_norm = 1.0f / sqrtf(norm2);
  quaternion[0] *= inverse_norm;
  quaternion[1] *= inverse_norm;
  quaternion[2] *= inverse_norm;
  quaternion[3] *= inverse_norm;
  sine_half2 = quaternion[1] * quaternion[1] +
               quaternion[2] * quaternion[2] +
               quaternion[3] * quaternion[3];

  /* For q=[cos(a/2), axis*sin(a/2)], the rotation-vector multiplier is
   * 2*asin(s)/s. Even simultaneous full-scale XYZ rates produce less than
   * 0.16 rad over one packet, so this sixth-order series is effectively exact
   * while removing a second square root and atan2f from the 400 Hz path.
   */

  if (quaternion[0] <= 0.0f || sine_half2 > 0.25f)
    {
      return false;
    }

  sine_half4 = sine_half2 * sine_half2;
  scale = 2.0f + sine_half2 * (1.0f / 3.0f) +
          sine_half4 * (3.0f / 20.0f) +
          sine_half4 * sine_half2 * (5.0f / 56.0f);

  vector[0] = quaternion[1] * scale;
  vector[1] = quaternion[2] * scale;
  vector[2] = quaternion[3] * scale;
  return vector_finite(vector);
}

void imu_integrator_init(FAR struct imu_integrator_s *integrator)
{
  memset(integrator, 0, sizeof(*integrator));
  integrator->quaternion[0] = 1.0f;
}

int imu_integrator_add(FAR struct imu_integrator_s *integrator,
                       uint64_t timestamp_us, FAR const float accel[3],
                       FAR const float gyro[3], uint8_t clipping,
                       FAR struct imu_delta_output_s *output)
{
  uint64_t dt_us;
  float dt;
  float delta_angle[3];
  float delta_velocity[3];
  float increment[4];
  float half_increment[4];
  float midpoint[4];
  float next_quaternion[4];
  float rotated_velocity[3];
  int axis;

  if (integrator == NULL || output == NULL || accel == NULL || gyro == NULL)
    {
      return -1;
    }

  if (!vector_finite(accel) || !vector_finite(gyro))
    {
      integrator->invalid++;
      integrator->resets++;
      integrator->seeded = false;
      window_reset(integrator, 0);
      return -1;
    }

  if (!integrator->seeded)
    {
      seed(integrator, timestamp_us, accel, gyro);
      return 0;
    }

  if (timestamp_us == integrator->last_timestamp_us)
    {
      integrator->duplicates++;
      integrator->resets++;
      seed(integrator, timestamp_us, accel, gyro);
      return -1;
    }

  if (timestamp_us < integrator->last_timestamp_us)
    {
      integrator->backwards++;
      integrator->resets++;
      seed(integrator, timestamp_us, accel, gyro);
      return -1;
    }

  dt_us = timestamp_us - integrator->last_timestamp_us;

  if (dt_us < IMU_DELTA_MIN_DT_US || dt_us > IMU_DELTA_MAX_DT_US)
    {
      integrator->gaps++;
      integrator->resets++;
      seed(integrator, timestamp_us, accel, gyro);
      return -1;
    }

  dt = (float)dt_us * 1.0e-6f;

  for (axis = 0; axis < 3; axis++)
    {
      delta_angle[axis] =
        0.5f * (integrator->previous_gyro[axis] + gyro[axis]) * dt;
      delta_velocity[axis] =
        0.5f * (integrator->previous_accel[axis] + accel[axis]) * dt;
    }

  rotation_vector_quaternion(delta_angle, 0.5f, half_increment);
  quaternion_multiply(integrator->quaternion, half_increment, midpoint);
  quaternion_rotate(midpoint, delta_velocity, rotated_velocity);

  rotation_vector_quaternion(delta_angle, 1.0f, increment);
  quaternion_multiply(integrator->quaternion, increment, next_quaternion);
  memcpy(integrator->quaternion, next_quaternion,
         sizeof(integrator->quaternion));

  for (axis = 0; axis < 3; axis++)
    {
      integrator->delta_velocity[axis] += rotated_velocity[axis];
    }

  integrator->integration_us += dt_us;
  integrator->samples++;
  integrator->clipping |= clipping;
  memcpy(integrator->previous_accel, accel,
         sizeof(integrator->previous_accel));
  memcpy(integrator->previous_gyro, gyro,
         sizeof(integrator->previous_gyro));
  integrator->last_timestamp_us = timestamp_us;

  if (integrator->integration_us + dt_us / 2u < IMU_DELTA_TARGET_US)
    {
      return 0;
    }

  memset(output, 0, sizeof(*output));
  output->first_timestamp_us = integrator->first_timestamp_us;
  output->last_timestamp_us = timestamp_us;
  memcpy(output->delta_velocity, integrator->delta_velocity,
         sizeof(output->delta_velocity));
  output->delta_angle_dt = (float)integrator->integration_us * 1.0e-6f;
  output->delta_velocity_dt = output->delta_angle_dt;
  output->samples = integrator->samples;
  output->clipping = integrator->clipping;

  if (!quaternion_to_rotation_vector(integrator->quaternion,
                                     output->delta_angle) ||
      !vector_finite(output->delta_velocity))
    {
      integrator->invalid++;
      integrator->resets++;
      seed(integrator, timestamp_us, accel, gyro);
      return -1;
    }

  integrator->packets++;
  window_reset(integrator, timestamp_us);
  return 1;
}
