/****************************************************************************
 * apps/ekf3/ekf_core.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 15-state error-state inertial prediction core. Nominal state propagation
 * runs for every 400 Hz vehicle_imu packet. Covariance propagation accumulates
 * four packets and runs at 100 Hz. There is deliberately no aiding update in
 * this stage, so velocity and position are not advertised as valid solutions.
 ****************************************************************************/

#ifndef EKF_CORE_HOST_TEST
#  include <nuttx/config.h>
#endif

#include <math.h>
#include <string.h>

#include "ekf_core.h"

#define EKF_GRAVITY                 9.80665f
#define EKF_MIN_PACKET_DT           0.0015f
#define EKF_MAX_PACKET_DT           0.0040f
#define EKF_WINDOW_TOLERANCE_US       200.0f
#define EKF_BOUNDARY_TOLERANCE_US       200ull

#define EKF_ALIGN_TIME_S            1.0f
#define EKF_ALIGN_MIN_SAMPLES       300u
#define EKF_ALIGN_ACCEL_MIN         (0.75f * EKF_GRAVITY)
#define EKF_ALIGN_ACCEL_MAX         (1.25f * EKF_GRAVITY)
#define EKF_ALIGN_GYRO_INSTANT_MAX  0.35f
#define EKF_ALIGN_GYRO_MEAN_MAX     0.05f
#define EKF_ALIGN_ACCEL_RMS_MAX     0.50f
#define EKF_ALIGN_GYRO_RMS_MAX      0.02f

/* Continuous-time noise densities. These deliberately start conservative;
 * later stages will expose bounded parameters and vibration adaptation.
 */

#define EKF_GYRO_NOISE              0.015f
#define EKF_ACCEL_NOISE             0.35f
#define EKF_GYRO_BIAS_RW            0.00010f
#define EKF_ACCEL_BIAS_RW           0.010f
#define EKF_CLIP_NOISE_SCALE        10.0f
#define EKF_MIN_VARIANCE            1.0e-10f

static bool vector_finite(FAR const float value[3])
{
  return isfinite(value[0]) && isfinite(value[1]) && isfinite(value[2]);
}

static float vector_norm_squared(FAR const float value[3])
{
  return value[0] * value[0] + value[1] * value[1] +
         value[2] * value[2];
}

static float vector_norm(FAR const float value[3])
{
  return sqrtf(vector_norm_squared(value));
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

static bool quaternion_normalize(FAR float quaternion[4])
{
  float norm2 = quaternion[0] * quaternion[0] +
                quaternion[1] * quaternion[1] +
                quaternion[2] * quaternion[2] +
                quaternion[3] * quaternion[3];
  float inverse_norm;

  if (!isfinite(norm2) || norm2 < 0.5f || norm2 > 1.5f)
    {
      return false;
    }

  inverse_norm = 1.0f / sqrtf(norm2);
  quaternion[0] *= inverse_norm;
  quaternion[1] *= inverse_norm;
  quaternion[2] *= inverse_norm;
  quaternion[3] *= inverse_norm;

  if (quaternion[0] < 0.0f)
    {
      quaternion[0] = -quaternion[0];
      quaternion[1] = -quaternion[1];
      quaternion[2] = -quaternion[2];
      quaternion[3] = -quaternion[3];
    }

  return true;
}

static void rotation_vector_quaternion(FAR const float vector[3],
                                       FAR float quaternion[4])
{
  float angle2 = vector_norm_squared(vector);
  float angle4 = angle2 * angle2;
  float scale = 0.5f - angle2 * (1.0f / 48.0f) +
                angle4 * (1.0f / 3840.0f);

  quaternion[0] = 1.0f - angle2 * (1.0f / 8.0f) +
                  angle4 * (1.0f / 384.0f);
  quaternion[1] = vector[0] * scale;
  quaternion[2] = vector[1] * scale;
  quaternion[3] = vector[2] * scale;
}

static void quaternion_to_rotation(FAR const float q[4], FAR float r[3][3])
{
  float xx = q[1] * q[1];
  float yy = q[2] * q[2];
  float zz = q[3] * q[3];
  float xy = q[1] * q[2];
  float xz = q[1] * q[3];
  float yz = q[2] * q[3];
  float wx = q[0] * q[1];
  float wy = q[0] * q[2];
  float wz = q[0] * q[3];

  r[0][0] = 1.0f - 2.0f * (yy + zz);
  r[0][1] = 2.0f * (xy - wz);
  r[0][2] = 2.0f * (xz + wy);
  r[1][0] = 2.0f * (xy + wz);
  r[1][1] = 1.0f - 2.0f * (xx + zz);
  r[1][2] = 2.0f * (yz - wx);
  r[2][0] = 2.0f * (xz - wy);
  r[2][1] = 2.0f * (yz + wx);
  r[2][2] = 1.0f - 2.0f * (xx + yy);
}

static void quaternion_from_euler(float roll, float pitch, float yaw,
                                  FAR float q[4])
{
  float cr = cosf(0.5f * roll);
  float sr = sinf(0.5f * roll);
  float cp = cosf(0.5f * pitch);
  float sp = sinf(0.5f * pitch);
  float cy = cosf(0.5f * yaw);
  float sy = sinf(0.5f * yaw);

  q[0] = cr * cp * cy + sr * sp * sy;
  q[1] = sr * cp * cy - cr * sp * sy;
  q[2] = cr * sp * cy + sr * cp * sy;
  q[3] = cr * cp * sy - sr * sp * cy;
}

void ekf_core_euler(FAR const struct ekf_core_s *ekf, FAR float euler[3])
{
  FAR const float *q = ekf->quaternion;
  float pitch_sine = 2.0f * (q[0] * q[2] - q[3] * q[1]);

  if (pitch_sine > 1.0f)
    {
      pitch_sine = 1.0f;
    }
  else if (pitch_sine < -1.0f)
    {
      pitch_sine = -1.0f;
    }

  euler[0] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                    1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
  euler[1] = asinf(pitch_sine);
  euler[2] = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                    1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

static void covariance_initialize(FAR struct ekf_core_s *ekf)
{
  const float degree = 0.017453292519943295f;
  const float diagonal[EKF_STATE_DIM] =
  {
    3.0f * 3.0f * degree * degree,
    3.0f * 3.0f * degree * degree,
    180.0f * 180.0f * degree * degree,
    1.0f, 1.0f, 1.0f,
    10.0f * 10.0f, 10.0f * 10.0f, 10.0f * 10.0f,
    0.02f * 0.02f, 0.02f * 0.02f, 0.02f * 0.02f,
    0.20f * 0.20f, 0.20f * 0.20f, 0.20f * 0.20f
  };
  int index;

  memset(ekf->covariance, 0, sizeof(ekf->covariance));

  for (index = 0; index < EKF_STATE_DIM; index++)
    {
      ekf->covariance[EKF_P_INDEX(index, index)] = diagonal[index];
    }
}

static void alignment_clear(FAR struct ekf_core_s *ekf)
{
  memset(ekf->align_accel_sum, 0, sizeof(ekf->align_accel_sum));
  memset(ekf->align_gyro_sum, 0, sizeof(ekf->align_gyro_sum));
  ekf->align_accel_norm2_sum = 0.0f;
  ekf->align_gyro_norm2_sum = 0.0f;
  ekf->align_time_s = 0.0f;
  ekf->align_samples = 0;
}

static void covariance_accumulator_clear(FAR struct ekf_core_s *ekf)
{
  memset(ekf->covariance_delta_angle, 0,
         sizeof(ekf->covariance_delta_angle));
  memset(ekf->covariance_delta_velocity, 0,
         sizeof(ekf->covariance_delta_velocity));
  ekf->covariance_dt = 0.0f;
  ekf->covariance_phase = 0;
  ekf->covariance_clipping = 0;
}

static void restart_alignment(FAR struct ekf_core_s *ekf)
{
  ekf->initialized = false;
  ekf->quaternion[0] = 1.0f;
  ekf->quaternion[1] = 0.0f;
  ekf->quaternion[2] = 0.0f;
  ekf->quaternion[3] = 0.0f;
  memset(ekf->velocity, 0, sizeof(ekf->velocity));
  memset(ekf->position, 0, sizeof(ekf->position));
  memset(ekf->gyro_bias, 0, sizeof(ekf->gyro_bias));
  memset(ekf->accel_bias, 0, sizeof(ekf->accel_bias));
  memset(ekf->covariance, 0, sizeof(ekf->covariance));
  alignment_clear(ekf);
  covariance_accumulator_clear(ekf);
  ekf->first_predict_timestamp = 0;
}

void ekf_core_init(FAR struct ekf_core_s *ekf)
{
  memset(ekf, 0, sizeof(*ekf));
  ekf->quaternion[0] = 1.0f;
}

static bool sample_valid(FAR const struct ekf_imu_sample_s *sample)
{
  float window_us;

  if (sample == NULL || sample->instance != 0 || sample->samples == 0 ||
      sample->timestamp_sample <= sample->timestamp_first ||
      !vector_finite(sample->delta_angle) ||
      !vector_finite(sample->delta_velocity) ||
      !isfinite(sample->delta_angle_dt) ||
      !isfinite(sample->delta_velocity_dt) ||
      sample->delta_angle_dt < EKF_MIN_PACKET_DT ||
      sample->delta_angle_dt > EKF_MAX_PACKET_DT ||
      sample->delta_velocity_dt < EKF_MIN_PACKET_DT ||
      sample->delta_velocity_dt > EKF_MAX_PACKET_DT ||
      fabsf(sample->delta_angle_dt - sample->delta_velocity_dt) > 0.0001f)
    {
      return false;
    }

  window_us = (float)(sample->timestamp_sample - sample->timestamp_first);
  return fabsf(window_us - sample->delta_angle_dt * 1.0e6f) <=
         EKF_WINDOW_TOLERANCE_US;
}

static bool alignment_add(FAR struct ekf_core_s *ekf,
                          FAR const struct ekf_imu_sample_s *sample)
{
  float accel[3];
  float gyro[3];
  float accel_norm;
  float gyro_norm;
  int axis;

  for (axis = 0; axis < 3; axis++)
    {
      accel[axis] = sample->delta_velocity[axis] /
                    sample->delta_velocity_dt;
      gyro[axis] = sample->delta_angle[axis] / sample->delta_angle_dt;
    }

  accel_norm = vector_norm(accel);
  gyro_norm = vector_norm(gyro);

  if (sample->clipping != 0 || accel_norm < EKF_ALIGN_ACCEL_MIN ||
      accel_norm > EKF_ALIGN_ACCEL_MAX ||
      gyro_norm > EKF_ALIGN_GYRO_INSTANT_MAX)
    {
      if (ekf->align_samples != 0)
        {
          ekf->alignment_restart_count++;
        }

      alignment_clear(ekf);
      return false;
    }

  for (axis = 0; axis < 3; axis++)
    {
      ekf->align_accel_sum[axis] += accel[axis];
      ekf->align_gyro_sum[axis] += gyro[axis];
    }

  ekf->align_accel_norm2_sum += accel_norm * accel_norm;
  ekf->align_gyro_norm2_sum += gyro_norm * gyro_norm;
  ekf->align_time_s += sample->delta_angle_dt;
  ekf->align_samples++;

  if (ekf->align_time_s >= EKF_ALIGN_TIME_S &&
      ekf->align_samples >= EKF_ALIGN_MIN_SAMPLES)
    {
      float inverse_count = 1.0f / (float)ekf->align_samples;
      float mean_accel[3];
      float mean_gyro[3];
      float accel_mean_norm2;
      float gyro_mean_norm2;
      float accel_variance;
      float gyro_variance;
      float roll;
      float pitch;

      for (axis = 0; axis < 3; axis++)
        {
          mean_accel[axis] = ekf->align_accel_sum[axis] * inverse_count;
          mean_gyro[axis] = ekf->align_gyro_sum[axis] * inverse_count;
        }

      accel_mean_norm2 = vector_norm_squared(mean_accel);
      gyro_mean_norm2 = vector_norm_squared(mean_gyro);
      accel_variance = ekf->align_accel_norm2_sum * inverse_count -
                       accel_mean_norm2;
      gyro_variance = ekf->align_gyro_norm2_sum * inverse_count -
                      gyro_mean_norm2;

      if (accel_variance < 0.0f)
        {
          accel_variance = 0.0f;
        }

      if (gyro_variance < 0.0f)
        {
          gyro_variance = 0.0f;
        }

      if (sqrtf(gyro_mean_norm2) > EKF_ALIGN_GYRO_MEAN_MAX ||
          sqrtf(accel_variance) > EKF_ALIGN_ACCEL_RMS_MAX ||
          sqrtf(gyro_variance) > EKF_ALIGN_GYRO_RMS_MAX)
        {
          ekf->alignment_restart_count++;
          alignment_clear(ekf);
          return false;
        }

      roll = atan2f(mean_accel[1], mean_accel[2]);
      pitch = -atan2f(mean_accel[0],
                      sqrtf(mean_accel[1] * mean_accel[1] +
                            mean_accel[2] * mean_accel[2]));
      quaternion_from_euler(roll, pitch, 0.0f, ekf->quaternion);
      memcpy(ekf->gyro_bias, mean_gyro, sizeof(ekf->gyro_bias));
      covariance_initialize(ekf);
      covariance_accumulator_clear(ekf);
      ekf->initialized = true;
      ekf->first_predict_timestamp = sample->timestamp_sample;
      return true;
    }

  return false;
}

static bool covariance_predict(FAR struct ekf_core_s *ekf)
{
  float f[EKF_STATE_DIM * EKF_STATE_DIM];
  float fp[EKF_STATE_DIM * EKF_STATE_DIM];
  float rotation[3][3];
  float omega[3];
  float specific_force[3];
  float velocity_attitude[3][3];
  float noise_scale;
  float dt = ekf->covariance_dt;
  int row;
  int column;
  int source;
  int axis;

  if (!isfinite(dt) || dt <= 0.0f || dt > 0.02f)
    {
      return false;
    }

  for (axis = 0; axis < 3; axis++)
    {
      omega[axis] = ekf->covariance_delta_angle[axis] / dt -
                    ekf->gyro_bias[axis];
      specific_force[axis] = ekf->covariance_delta_velocity[axis] / dt -
                             ekf->accel_bias[axis];
    }

  quaternion_to_rotation(ekf->quaternion, rotation);

  /* -R_nb * skew(f_b), coupling attitude error into NED velocity. */

  for (row = 0; row < 3; row++)
    {
      velocity_attitude[row][0] =
        rotation[row][1] * specific_force[2] -
        rotation[row][2] * specific_force[1];
      velocity_attitude[row][1] =
       -rotation[row][0] * specific_force[2] +
        rotation[row][2] * specific_force[0];
      velocity_attitude[row][2] =
        rotation[row][0] * specific_force[1] -
        rotation[row][1] * specific_force[0];
    }

  memset(f, 0, sizeof(f));

  /* attitude: -skew(omega) * dtheta - dbg */

  f[EKF_P_INDEX(0, 1)] = omega[2];
  f[EKF_P_INDEX(0, 2)] = -omega[1];
  f[EKF_P_INDEX(1, 0)] = -omega[2];
  f[EKF_P_INDEX(1, 2)] = omega[0];
  f[EKF_P_INDEX(2, 0)] = omega[1];
  f[EKF_P_INDEX(2, 1)] = -omega[0];

  for (axis = 0; axis < 3; axis++)
    {
      f[EKF_P_INDEX(axis, 9 + axis)] = -1.0f;
    }

  /* velocity: -R*skew(f) * dtheta - R*dba */

  for (row = 0; row < 3; row++)
    {
      for (column = 0; column < 3; column++)
        {
          f[EKF_P_INDEX(3 + row, column)] =
            velocity_attitude[row][column];
          f[EKF_P_INDEX(3 + row, 12 + column)] =
            -rotation[row][column];
        }
    }

  /* position: dvelocity */

  for (axis = 0; axis < 3; axis++)
    {
      f[EKF_P_INDEX(6 + axis, 3 + axis)] = 1.0f;
    }

  /* fp = F*P. F is mostly zero, so skip zero coefficients. */

  memset(fp, 0, sizeof(fp));

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      for (source = 0; source < EKF_STATE_DIM; source++)
        {
          float coefficient = f[EKF_P_INDEX(row, source)];

          if (coefficient != 0.0f)
            {
              for (column = 0; column < EKF_STATE_DIM; column++)
                {
                  fp[EKF_P_INDEX(row, column)] += coefficient *
                    ekf->covariance[EKF_P_INDEX(source, column)];
                }
            }
        }
    }

  /* (I + F*dt) P (I + F*dt)'.
   *
   * Keeping the dt^2 FPF' term costs little at 100 Hz and preserves positive
   * semidefiniteness much better than an Euler update of the Riccati equation.
   * The upper triangle is evaluated from the unchanged P/FP workspaces, then
   * mirrored exactly.
   */

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      for (column = row; column < EKF_STATE_DIM; column++)
        {
          float second_order = 0.0f;
          float value = ekf->covariance[EKF_P_INDEX(row, column)] + dt *
            (fp[EKF_P_INDEX(row, column)] +
             fp[EKF_P_INDEX(column, row)]);

          for (source = 0; source < EKF_STATE_DIM; source++)
            {
              second_order += fp[EKF_P_INDEX(row, source)] *
                              f[EKF_P_INDEX(column, source)];
            }

          value += dt * dt * second_order;

          if (!isfinite(value))
            {
              return false;
            }

          ekf->covariance[EKF_P_INDEX(row, column)] = value;
          ekf->covariance[EKF_P_INDEX(column, row)] = value;
        }
    }

  noise_scale = ekf->covariance_clipping != 0 ?
                EKF_CLIP_NOISE_SCALE : 1.0f;

  for (axis = 0; axis < 3; axis++)
    {
      ekf->covariance[EKF_P_INDEX(axis, axis)] +=
        EKF_GYRO_NOISE * EKF_GYRO_NOISE * noise_scale * dt;
      ekf->covariance[EKF_P_INDEX(3 + axis, 3 + axis)] +=
        EKF_ACCEL_NOISE * EKF_ACCEL_NOISE * noise_scale * dt;
      ekf->covariance[EKF_P_INDEX(9 + axis, 9 + axis)] +=
        EKF_GYRO_BIAS_RW * EKF_GYRO_BIAS_RW * dt;
      ekf->covariance[EKF_P_INDEX(12 + axis, 12 + axis)] +=
        EKF_ACCEL_BIAS_RW * EKF_ACCEL_BIAS_RW * dt;
    }

  for (axis = 0; axis < EKF_STATE_DIM; axis++)
    {
      float *diagonal =
        &ekf->covariance[EKF_P_INDEX(axis, axis)];

      if (!isfinite(*diagonal))
        {
          return false;
        }

      if (*diagonal < EKF_MIN_VARIANCE)
        {
          *diagonal = EKF_MIN_VARIANCE;
        }
    }

  ekf->covariance_count++;
  return true;
}

static bool nominal_predict(FAR struct ekf_core_s *ekf,
                            FAR const struct ekf_imu_sample_s *sample)
{
  float corrected_angle[3];
  float corrected_velocity[3];
  float half_angle[3];
  float increment[4];
  float half_increment[4];
  float midpoint[4];
  float next_quaternion[4];
  float rotation[3][3];
  float nav_delta_velocity[3];
  float old_velocity[3];
  float dt = sample->delta_angle_dt;
  int row;
  int axis;

  for (axis = 0; axis < 3; axis++)
    {
      corrected_angle[axis] = sample->delta_angle[axis] -
                              ekf->gyro_bias[axis] * dt;
      corrected_velocity[axis] = sample->delta_velocity[axis] -
                                 ekf->accel_bias[axis] * dt;
      half_angle[axis] = 0.5f * corrected_angle[axis];
      old_velocity[axis] = ekf->velocity[axis];
    }

  rotation_vector_quaternion(half_angle, half_increment);
  quaternion_multiply(ekf->quaternion, half_increment, midpoint);

  if (!quaternion_normalize(midpoint))
    {
      return false;
    }

  quaternion_to_rotation(midpoint, rotation);

  for (row = 0; row < 3; row++)
    {
      nav_delta_velocity[row] =
        rotation[row][0] * corrected_velocity[0] +
        rotation[row][1] * corrected_velocity[1] +
        rotation[row][2] * corrected_velocity[2];
    }

  nav_delta_velocity[2] -= EKF_GRAVITY * dt;

  for (axis = 0; axis < 3; axis++)
    {
      ekf->velocity[axis] += nav_delta_velocity[axis];
      ekf->position[axis] +=
        (old_velocity[axis] + 0.5f * nav_delta_velocity[axis]) * dt;
      ekf->covariance_delta_angle[axis] += sample->delta_angle[axis];
      ekf->covariance_delta_velocity[axis] +=
        sample->delta_velocity[axis];
    }

  rotation_vector_quaternion(corrected_angle, increment);
  quaternion_multiply(ekf->quaternion, increment, next_quaternion);
  memcpy(ekf->quaternion, next_quaternion, sizeof(ekf->quaternion));

  if (!quaternion_normalize(ekf->quaternion) ||
      !vector_finite(ekf->velocity) || !vector_finite(ekf->position))
    {
      return false;
    }

  ekf->covariance_dt += dt;
  ekf->covariance_phase++;
  ekf->covariance_clipping |= sample->clipping;
  ekf->predict_count++;

  if (ekf->covariance_phase >= EKF_COVARIANCE_INTERVAL)
    {
      if (!covariance_predict(ekf))
        {
          return false;
        }

      covariance_accumulator_clear(ekf);
    }

  return true;
}

int ekf_core_process(FAR struct ekf_core_s *ekf,
                     FAR const struct ekf_imu_sample_s *sample)
{
  uint64_t sequence_dt;

  if (ekf == NULL || !sample_valid(sample))
    {
      if (ekf != NULL)
        {
          ekf->rejected_count++;
        }

      return EKF_PROCESS_REJECTED;
    }

  ekf->input_count++;

  if (!sample->accel_calibrated || !sample->gyro_calibrated)
    {
      ekf->uncalibrated_count++;
      ekf->rejected_count++;

      if (ekf->initialized || ekf->align_samples != 0)
        {
          ekf->reset_counter++;
        }

      restart_alignment(ekf);
      ekf->last_timestamp_sample = sample->timestamp_sample;
      return EKF_PROCESS_REJECTED;
    }

  if (!ekf->have_source_reset)
    {
      ekf->source_reset_counter = sample->reset_counter;
      ekf->have_source_reset = true;
    }
  else if (sample->reset_counter != ekf->source_reset_counter)
    {
      ekf->source_reset_counter = sample->reset_counter;
      ekf->source_reset_count++;
      ekf->reset_counter++;
      restart_alignment(ekf);
    }

  if (ekf->last_timestamp_sample != 0)
    {
      if (sample->timestamp_sample == ekf->last_timestamp_sample)
        {
          ekf->duplicate_count++;
          ekf->rejected_count++;
          return EKF_PROCESS_REJECTED;
        }

      if (sample->timestamp_sample < ekf->last_timestamp_sample)
        {
          ekf->backward_count++;
          ekf->rejected_count++;
          ekf->reset_counter++;
          restart_alignment(ekf);
          ekf->last_timestamp_sample = sample->timestamp_sample;
          return EKF_PROCESS_REJECTED;
        }

      sequence_dt = sample->timestamp_sample -
                    ekf->last_timestamp_sample;

      if (sequence_dt > 2ull * (uint64_t)(EKF_MAX_PACKET_DT * 1.0e6f) ||
          sample->timestamp_first >
            ekf->last_timestamp_sample + EKF_BOUNDARY_TOLERANCE_US ||
          sample->timestamp_first + EKF_BOUNDARY_TOLERANCE_US <
            ekf->last_timestamp_sample)
        {
          ekf->gap_count++;
          ekf->reset_counter++;
          restart_alignment(ekf);
        }
    }

  ekf->last_timestamp_sample = sample->timestamp_sample;

  if (sample->clipping != 0)
    {
      ekf->clipping_count++;
    }

  if (!ekf->initialized)
    {
      return alignment_add(ekf, sample) ? EKF_PROCESS_INITIALIZED :
             EKF_PROCESS_ALIGNING;
    }

  if (!nominal_predict(ekf, sample))
    {
      ekf->numerical_reset_count++;
      ekf->rejected_count++;
      ekf->reset_counter++;
      restart_alignment(ekf);
      return EKF_PROCESS_REJECTED;
    }

  return EKF_PROCESS_PREDICTED;
}

uint8_t ekf_core_solution_status(FAR const struct ekf_core_s *ekf)
{
  return ekf != NULL && ekf->initialized ?
         EKF_SOLUTION_ATTITUDE | EKF_SOLUTION_YAW_RELATIVE : 0;
}
