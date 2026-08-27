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

#define EKF_DYNAMICS_TIME_CONSTANT  0.25f
#define EKF_DYNAMICS_ENTRY_TIME     0.50f
#define EKF_DYNAMICS_ACCEL_DEV_IN   0.60f
#define EKF_DYNAMICS_ACCEL_DEV_OUT  1.20f
#define EKF_DYNAMICS_GYRO_IN        0.08f
#define EKF_DYNAMICS_GYRO_OUT       0.20f
#define EKF_DYNAMICS_ACCEL_RMS_IN   0.50f
#define EKF_DYNAMICS_ACCEL_RMS_OUT  1.00f
#define EKF_DYNAMICS_GYRO_RMS_IN    0.03f
#define EKF_DYNAMICS_GYRO_RMS_OUT   0.08f
#define EKF_DYNAMICS_SPEED_IN        0.15f
#define EKF_DYNAMICS_SPEED_OUT       0.35f
#define EKF_DYNAMICS_LINEAR_ACCEL_IN 0.60f
#define EKF_DYNAMICS_LINEAR_ACCEL_OUT 1.20f

#define EKF_GRAVITY_MEAS_NOISE      0.35f
#define EKF_MEASUREMENT_NIS_GATE    16.3f
#define EKF_GYRO_BIAS_LIMIT_VAR     (0.02f * 0.02f)
#define EKF_ACCEL_BIAS_LIMIT_VAR    (0.20f * 0.20f)

/* Continuous-time noise densities. These deliberately start conservative;
 * later stages will expose bounded parameters and vibration adaptation.
 */

#define EKF_GYRO_NOISE              0.015f
#define EKF_ACCEL_NOISE             0.35f
#define EKF_GYRO_BIAS_RW            0.00010f
#define EKF_ACCEL_BIAS_RW           0.010f
#define EKF_CLIP_NOISE_SCALE        10.0f
#define EKF_MIN_VARIANCE            1.0e-10f

/* Kalman-gain row masks.
 *
 * Measurement availability is a state-update contract, not a hope that the
 * corresponding covariance cross-terms happen to be zero.  ArduPilot makes
 * this explicit with its kalman_mask before applying each innovation.  This
 * error-state core does the same: each fusion path names the state families
 * that observation is allowed to correct, and every other gain row is zeroed
 * before both the nominal-state and Joseph covariance updates.
 */

#define EKF_GAIN_BIT(state)         ((uint16_t)1u << (state))
#define EKF_GAIN_ALL                ((uint16_t)((1u << EKF_STATE_DIM) - 1u))
#define EKF_GAIN_ATTITUDE           ((uint16_t)0x0007u)
#define EKF_GAIN_VELOCITY_XY        ((uint16_t)0x0018u)
#define EKF_GAIN_VELOCITY_Z         EKF_GAIN_BIT(5)
#define EKF_GAIN_POSITION_XY        ((uint16_t)0x00c0u)
#define EKF_GAIN_POSITION_Z         EKF_GAIN_BIT(8)
#define EKF_GAIN_GYRO_BIAS          ((uint16_t)0x0e00u)
#define EKF_GAIN_ACCEL_BIAS         ((uint16_t)0x7000u)
#define EKF_GAIN_ACCEL_BIAS_Z       EKF_GAIN_BIT(14)

#define EKF_GAIN_GRAVITY \
  (EKF_GAIN_ATTITUDE | EKF_GAIN_GYRO_BIAS | EKF_GAIN_ACCEL_BIAS)
#define EKF_GAIN_YAW \
  (EKF_GAIN_ATTITUDE | EKF_GAIN_GYRO_BIAS)
#define EKF_GAIN_HORIZONTAL_POSITION \
  (EKF_GAIN_VELOCITY_XY | EKF_GAIN_POSITION_XY)
#define EKF_GAIN_HEIGHT \
  (EKF_GAIN_VELOCITY_Z | EKF_GAIN_POSITION_Z | EKF_GAIN_ACCEL_BIAS_Z)

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
  memset(ekf->align_mag_sum, 0, sizeof(ekf->align_mag_sum));
  ekf->align_mag_samples = 0;
  ekf->align_accel_norm2_sum = 0.0f;
  ekf->align_gyro_norm2_sum = 0.0f;
  ekf->align_time_s = 0.0f;
  ekf->align_samples = 0;
}

static void dynamics_clear(FAR struct ekf_core_s *ekf)
{
  memset(ekf->dynamics_accel_mean, 0,
         sizeof(ekf->dynamics_accel_mean));
  memset(ekf->dynamics_gyro_mean, 0,
         sizeof(ekf->dynamics_gyro_mean));
  memset(ekf->dynamics_accel_variance, 0,
         sizeof(ekf->dynamics_accel_variance));
  memset(ekf->dynamics_gyro_variance, 0,
         sizeof(ekf->dynamics_gyro_variance));
  ekf->low_dynamics_dwell_s = 0.0f;
  ekf->dynamics_seeded = false;

  if (ekf->low_dynamics)
    {
      ekf->low_dynamics = false;
      ekf->low_dynamics_exit_count++;
    }
}

static void dynamics_update(FAR struct ekf_core_s *ekf,
                            FAR const struct ekf_imu_sample_s *sample)
{
  float accel[3];
  float gyro[3];
  float accel_variance = 0.0f;
  float gyro_variance = 0.0f;
  float alpha;
  bool entry_candidate;
  bool remain_candidate;
  bool position_aided;
  float horizontal_speed;
  float linear_accel = 0.0f;
  int axis;

  for (axis = 0; axis < 3; axis++)
    {
      accel[axis] = sample->delta_velocity[axis] /
                    sample->delta_velocity_dt - ekf->accel_bias[axis];
      gyro[axis] = sample->delta_angle[axis] /
                   sample->delta_angle_dt - ekf->gyro_bias[axis];
    }

  if (!ekf->dynamics_seeded)
    {
      memcpy(ekf->dynamics_accel_mean, accel,
             sizeof(ekf->dynamics_accel_mean));
      memcpy(ekf->dynamics_gyro_mean, gyro,
             sizeof(ekf->dynamics_gyro_mean));
      ekf->dynamics_seeded = true;
    }
  else
    {
      alpha = sample->delta_angle_dt /
              (EKF_DYNAMICS_TIME_CONSTANT + sample->delta_angle_dt);

      for (axis = 0; axis < 3; axis++)
        {
          float accel_delta = accel[axis] -
                              ekf->dynamics_accel_mean[axis];
          float gyro_delta = gyro[axis] -
                             ekf->dynamics_gyro_mean[axis];

          ekf->dynamics_accel_mean[axis] += alpha * accel_delta;
          ekf->dynamics_gyro_mean[axis] += alpha * gyro_delta;
          ekf->dynamics_accel_variance[axis] =
            (1.0f - alpha) *
            (ekf->dynamics_accel_variance[axis] +
             alpha * accel_delta * accel_delta);
          ekf->dynamics_gyro_variance[axis] =
            (1.0f - alpha) *
            (ekf->dynamics_gyro_variance[axis] +
             alpha * gyro_delta * gyro_delta);
        }
    }

  for (axis = 0; axis < 3; axis++)
    {
      accel_variance += ekf->dynamics_accel_variance[axis];
      gyro_variance += ekf->dynamics_gyro_variance[axis];
    }

  /* Low IMU variance is not the same thing as stationary. A car under smooth,
   * constant acceleration has an accelerometer norm close to one g and almost
   * zero variance - exactly the case that used to pass these tests and feed
   * its longitudinal acceleration into the gravity/tilt update. Once an
   * absolute position source makes speed observable, use that independent
   * evidence to refuse the standstill pseudo-measurement while moving.
   */

  position_aided = ekf->initialized && ekf_core_position_aided(ekf);
  horizontal_speed = sqrtf(ekf->velocity[0] * ekf->velocity[0] +
                           ekf->velocity[1] * ekf->velocity[1]);

  if (position_aided)
    {
      float rotation[3][3];
      float nongravity[3];

      quaternion_to_rotation(ekf->quaternion, rotation);

      for (axis = 0; axis < 3; axis++)
        {
          /* R_nb' * [0, 0, g] is the accelerometer value expected from
           * gravity alone.  Unlike |accel|-g, this catches smooth horizontal
           * acceleration immediately while still allowing a stationary car
           * parked on a slope.
           */

          nongravity[axis] = accel[axis] - rotation[2][axis] * EKF_GRAVITY;
        }

      linear_accel = vector_norm(nongravity);
    }

  entry_candidate =
    sample->clipping == 0 &&
    fabsf(vector_norm(accel) - EKF_GRAVITY) <
      EKF_DYNAMICS_ACCEL_DEV_IN &&
    vector_norm(gyro) < EKF_DYNAMICS_GYRO_IN &&
    sqrtf(accel_variance) < EKF_DYNAMICS_ACCEL_RMS_IN &&
    sqrtf(gyro_variance) < EKF_DYNAMICS_GYRO_RMS_IN &&
    (!position_aided || linear_accel < EKF_DYNAMICS_LINEAR_ACCEL_IN) &&
    (!position_aided || horizontal_speed < EKF_DYNAMICS_SPEED_IN);

  remain_candidate =
    sample->clipping == 0 &&
    fabsf(vector_norm(accel) - EKF_GRAVITY) <
      EKF_DYNAMICS_ACCEL_DEV_OUT &&
    vector_norm(gyro) < EKF_DYNAMICS_GYRO_OUT &&
    sqrtf(accel_variance) < EKF_DYNAMICS_ACCEL_RMS_OUT &&
    sqrtf(gyro_variance) < EKF_DYNAMICS_GYRO_RMS_OUT &&
    (!position_aided || linear_accel < EKF_DYNAMICS_LINEAR_ACCEL_OUT) &&
    (!position_aided || horizontal_speed < EKF_DYNAMICS_SPEED_OUT);

  if (ekf->low_dynamics)
    {
      if (!remain_candidate)
        {
          ekf->low_dynamics = false;
          ekf->low_dynamics_dwell_s = 0.0f;
          ekf->low_dynamics_exit_count++;
        }
      else
        {
          ekf->low_dynamics_dwell_s += sample->delta_angle_dt;
        }

      return;
    }

  if (!entry_candidate)
    {
      ekf->low_dynamics_dwell_s = 0.0f;
      return;
    }

  ekf->low_dynamics_dwell_s += sample->delta_angle_dt;

  if (ekf->low_dynamics_dwell_s >= EKF_DYNAMICS_ENTRY_TIME)
    {
      ekf->low_dynamics = true;
      ekf->low_dynamics_entry_count++;
    }
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
  dynamics_clear(ekf);
  covariance_accumulator_clear(ekf);
  ekf->first_predict_timestamp = 0;

  /* The barometer reference is the height datum, and the datum is the
   * alignment point. Keeping it across a re-alignment would silently re-datum
   * the height to whatever the pressure was somewhere else.
   */

  ekf->baro_reference_hpa = 0.0f;
  ekf->baro_have_reference = false;
  ekf->baro_consecutive_rejects = 0;

  /* The local frame is gone, so map coordinates for it would be a lie. */

  ekf->extnav_datum_set = false;
  ekf->last_extnav_rx_timestamp = 0;
  ekf->extnav_healthy = true;
  ekf->extnav_test_ratio = 0.0f;
  ekf->extnav_fault_since = 0;
  ekf->extnav_bias_inhibited = false;
  ekf->inhibit_mask = 0;
  ekf->have_extnav_reset = false;
  ekf->extnav_consecutive_rejects = 0;
  ekf->last_baro_height = 0.0f;

  /* The heading datum goes with the alignment that established it. Keeping
   * yaw_absolute across a restart would claim north for a heading that is
   * about to be re-derived from scratch.
   */

  ekf->yaw_absolute = false;
  ekf->mag_consecutive_rejects = 0;
  ekf->last_mag_timestamp = 0;
}

/* Drop the filter back to alignment on request.
 *
 * The same path a fault takes, plus the reset generation, so consumers see a
 * commanded reset exactly as they see an automatic one - there is no second
 * kind of discontinuity for them to learn about.
 *
 * Biases go with it. They were estimated against a trajectory that is being
 * abandoned, and a bias carried across a reset is the one piece of state
 * that could make the new alignment worse than a cold start.
 */

void ekf_core_reset(FAR struct ekf_core_s *ekf)
{
  if (ekf == NULL)
    {
      return;
    }

  ekf->reset_counter++;
  ekf->commanded_reset_count++;
  restart_alignment(ekf);
}

void ekf_core_init(FAR struct ekf_core_s *ekf)
{
  memset(ekf, 0, sizeof(*ekf));
  ekf->quaternion[0] = 1.0f;

  /* AFTER the memset, obviously - and set here rather than in
   * restart_alignment because these are configuration: a re-alignment must
   * not silently discard an operator's tightened bound and go back to the
   * compiled ceiling. Same reasoning as the mag and extnav config setters.
   */

  ekf->gyro_bias_limit = EKF_GYRO_BIAS_LIMIT;
  ekf->accel_bias_limit = EKF_ACCEL_BIAS_LIMIT;
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
      float yaw;

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

      /* Heading from the averaged field, tilted by the roll and pitch just
       * derived - accelerometer and magnetometer together, which is the only
       * way to get an absolute attitude at rest.
       *
       * Averaged over the same window and subject to the same stillness
       * gates the accelerometer already passed, so a field captured during
       * motion cannot set the datum.
       */

      yaw = 0.0f;
      ekf->yaw_absolute = false;

      if (ekf->align_mag_samples > 0)
        {
          float mean_mag[3];
          float level[4];

          for (axis = 0; axis < 3; axis++)
            {
              mean_mag[axis] = ekf->align_mag_sum[axis] /
                               (float)ekf->align_mag_samples;
            }

          quaternion_from_euler(roll, pitch, 0.0f, level);

          if (ekf_mag_heading(level, mean_mag, ekf->align_declination,
                              &yaw))
            {
              ekf->yaw_absolute = true;
              ekf->last_mag_field = vector_norm(mean_mag);
              ekf->last_mag_heading = yaw;

              /* Alignment is itself a magnetometer fix, so it starts the
               * staleness clock. Without this the solution would report a
               * relative heading until the first runtime fusion, having just
               * derived an absolute one.
               */

              ekf->last_mag_timestamp = sample->timestamp_sample;
            }
          else
            {
              yaw = 0.0f;
            }
        }

      quaternion_from_euler(roll, pitch, yaw, ekf->quaternion);
      memcpy(ekf->gyro_bias, mean_gyro, sizeof(ekf->gyro_bias));
      covariance_initialize(ekf);

      /* A heading that means something deserves a covariance that says so.
       * Leaving the 180-degree entry would leave the state correct and the
       * filter still convinced it knew nothing.
       */

      if (ekf->yaw_absolute && ekf->align_yaw_variance > 0.0f)
        {
          ekf->covariance[EKF_P_INDEX(2, 2)] = ekf->align_yaw_variance;
        }

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

  /* -R_nb * skew(f_b), coupling attitude error into navigation-frame
   * velocity. The frame is ENU - east, north, up - not NED; see
   * uorb_msgs.h.
   *
   * The nominal quaternion and correction are both right-multiplied, so the
   * error is body-frame and R_true = R_nom * (I + skew(dtheta)). Therefore
   * dv = -R_nom * skew(f_b) * dtheta. Reversing this sign reverses the
   * position/attitude cross-covariance and makes a later position innovation
   * push tilt away from the physically consistent direction.
   */

  for (row = 0; row < 3; row++)
    {
      velocity_attitude[row][0] =
       -rotation[row][1] * specific_force[2] +
        rotation[row][2] * specific_force[1];
      velocity_attitude[row][1] =
        rotation[row][0] * specific_force[2] -
        rotation[row][2] * specific_force[0];
      velocity_attitude[row][2] =
       -rotation[row][0] * specific_force[1] +
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

static bool invert_symmetric_3x3(FAR const float matrix[3][3],
                                 FAR float inverse[3][3])
{
  float determinant =
    matrix[0][0] * (matrix[1][1] * matrix[2][2] -
                    matrix[1][2] * matrix[1][2]) -
    matrix[0][1] * (matrix[0][1] * matrix[2][2] -
                    matrix[1][2] * matrix[0][2]) +
    matrix[0][2] * (matrix[0][1] * matrix[1][2] -
                    matrix[1][1] * matrix[0][2]);

  if (!isfinite(determinant) || determinant <= 1.0e-18f)
    {
      return false;
    }

  inverse[0][0] = (matrix[1][1] * matrix[2][2] -
                   matrix[1][2] * matrix[1][2]) / determinant;
  inverse[0][1] = (matrix[0][2] * matrix[1][2] -
                   matrix[0][1] * matrix[2][2]) / determinant;
  inverse[0][2] = (matrix[0][1] * matrix[1][2] -
                   matrix[0][2] * matrix[1][1]) / determinant;
  inverse[1][0] = inverse[0][1];
  inverse[1][1] = (matrix[0][0] * matrix[2][2] -
                   matrix[0][2] * matrix[0][2]) / determinant;
  inverse[1][2] = (matrix[0][1] * matrix[0][2] -
                   matrix[0][0] * matrix[1][2]) / determinant;
  inverse[2][0] = inverse[0][2];
  inverse[2][1] = inverse[1][2];
  inverse[2][2] = (matrix[0][0] * matrix[1][1] -
                   matrix[0][1] * matrix[0][1]) / determinant;
  return isfinite(inverse[0][0]) && isfinite(inverse[1][1]) &&
         isfinite(inverse[2][2]);
}

static bool covariance_reset_attitude(FAR struct ekf_core_s *ekf,
                                      FAR const float correction[3])
{
  float reset[3][3] =
  {
    {1.0f, 0.5f * correction[2], -0.5f * correction[1]},
    {-0.5f * correction[2], 1.0f, 0.5f * correction[0]},
    {0.5f * correction[1], -0.5f * correction[0], 1.0f}
  };
  float old_block[3][3];
  float temporary[3][3];
  float transformed[3][3];
  int row;
  int column;
  int inner;

  for (row = 0; row < 3; row++)
    {
      for (column = 0; column < 3; column++)
        {
          old_block[row][column] =
            ekf->covariance[EKF_P_INDEX(row, column)];
        }
    }

  for (column = 3; column < EKF_STATE_DIM; column++)
    {
      float old_cross[3];

      for (row = 0; row < 3; row++)
        {
          old_cross[row] =
            ekf->covariance[EKF_P_INDEX(row, column)];
        }

      for (row = 0; row < 3; row++)
        {
          float value = 0.0f;

          for (inner = 0; inner < 3; inner++)
            {
              value += reset[row][inner] * old_cross[inner];
            }

          if (!isfinite(value))
            {
              return false;
            }

          ekf->covariance[EKF_P_INDEX(row, column)] = value;
          ekf->covariance[EKF_P_INDEX(column, row)] = value;
        }
    }

  for (row = 0; row < 3; row++)
    {
      for (column = 0; column < 3; column++)
        {
          temporary[row][column] = 0.0f;

          for (inner = 0; inner < 3; inner++)
            {
              temporary[row][column] +=
                reset[row][inner] * old_block[inner][column];
            }
        }
    }

  for (row = 0; row < 3; row++)
    {
      for (column = 0; column < 3; column++)
        {
          transformed[row][column] = 0.0f;

          for (inner = 0; inner < 3; inner++)
            {
              transformed[row][column] +=
                temporary[row][inner] * reset[column][inner];
            }

          if (!isfinite(transformed[row][column]))
            {
              return false;
            }
        }
    }

  for (row = 0; row < 3; row++)
    {
      for (column = row; column < 3; column++)
        {
          float value = 0.5f *
            (transformed[row][column] + transformed[column][row]);

          ekf->covariance[EKF_P_INDEX(row, column)] = value;
          ekf->covariance[EKF_P_INDEX(column, row)] = value;
        }
    }

  return true;
}

/* Reset the horizontal position states and their covariance.
 *
 * Zeroing the row AND column, not just the diagonal: a cross-covariance to
 * velocity or attitude describes a correlation with the OLD position, and
 * keeping it would let the next update correct velocity through a
 * relationship that no longer exists.
 */

static void covariance_reset_position_xy(FAR struct ekf_core_s *ekf,
                                         float variance)
{
  int axis;
  int i;

  for (axis = 6; axis <= 7; axis++)
    {
      for (i = 0; i < EKF_STATE_DIM; i++)
        {
          ekf->covariance[EKF_P_INDEX(axis, i)] = 0.0f;
          ekf->covariance[EKF_P_INDEX(i, axis)] = 0.0f;
        }

      ekf->covariance[EKF_P_INDEX(axis, axis)] = variance;
    }
}

/* Reset yaw to an absolute value, and its covariance with it.
 *
 * NOT covariance_reset_attitude: that linearises about a SMALL correction,
 * and a datum yaw is a finite rotation. Roll and pitch are preserved exactly
 * - they come from gravity, and the external source says nothing about them.
 */

static void reset_yaw_absolute(FAR struct ekf_core_s *ekf, float yaw,
                               float variance)
{
  float euler[3];
  int i;

  ekf_core_euler(ekf, euler);
  quaternion_from_euler(euler[0], euler[1], yaw, ekf->quaternion);
  quaternion_normalize(ekf->quaternion);

  for (i = 0; i < EKF_STATE_DIM; i++)
    {
      ekf->covariance[EKF_P_INDEX(2, i)] = 0.0f;
      ekf->covariance[EKF_P_INDEX(i, 2)] = 0.0f;
    }

  ekf->covariance[EKF_P_INDEX(2, 2)] = variance;
}

/* Hold the vertical state inside its bound.
 *
 * A car stays on the road. Without something saying so, z is free to walk -
 * a mis-estimated accel bias or a drifting baro reference integrates into
 * height and nothing in the filter objects, because nothing in the filter
 * knows what the vehicle is.
 *
 * Three things happen together, and the third is the one that matters:
 *
 *   the state is clamped, so the published height stops being nonsense;
 *   the vertical velocity is zeroed IF it is still pushing outwards, so the
 *     clamp is not re-violated on the very next step;
 *   the vertical variance is floored, so the filter reports that it does not
 *     actually know where it is.
 *
 * Clamping the state alone would be a lie: the covariance would go on
 * claiming a confident height while the number was being held in place by
 * hand. Anything reading position_variance to decide whether to trust the
 * solution has to see this.
 *
 * Velocity is zeroed only when it points further out. A vehicle at the bound
 * on a slope may legitimately be coming back, and stopping that would fight
 * the recovery.
 */

uint8_t ekf_core_observability(FAR const struct ekf_core_s *ekf)
{
  if (ekf == NULL)
    {
      return EKF_OBS_ATTITUDE;
    }

  /* An absolute fix gives position, and velocity with it by derivative. */

  if (ekf_core_position_aided(ekf))
    {
      return EKF_OBS_POSITION;
    }

  /* A recent zero-velocity update measures motion without measuring where,
   * which is exactly this tier. Optical flow belongs here too when it
   * arrives.
   */

  if (ekf->last_zupt_timestamp != 0 && ekf->extnav_timeout_us > 0 &&
      ekf->last_timestamp_sample >= ekf->last_zupt_timestamp &&
      ekf->last_timestamp_sample - ekf->last_zupt_timestamp <
        (uint64_t)ekf->extnav_timeout_us)
    {
      return EKF_OBS_VELOCITY;
    }

  return EKF_OBS_ATTITUDE;
}

bool ekf_core_position_aided(FAR const struct ekf_core_s *ekf)
{
  if (ekf == NULL)
    {
      return false;
    }

  /* A claim about a source actually CORRECTING, not about it being
   * selected. A sustained rejection run withdraws it, and so does silence:
   * a source that simply stopped talking leaves no rejections behind, so
   * without the age check the claim would stand for ever on a dead link.
   */

  return ekf->extnav_datum_set && ekf->extnav_accept_count > 0 &&
         ekf->extnav_healthy &&
         ekf->extnav_consecutive_rejects < EKF_EXTNAV_REJECT_RUN_MAX &&
         ekf->extnav_timeout_us > 0 &&
         ekf->last_timestamp_sample >= ekf->last_extnav_timestamp &&
         ekf->last_timestamp_sample - ekf->last_extnav_timestamp <
           (uint64_t)ekf->extnav_timeout_us;
}

void ekf_core_set_bias_learning(FAR struct ekf_core_s *ekf, bool learn_gyro,
                                bool learn_accel)
{
  if (ekf == NULL)
    {
      return;
    }

  /* Disabling learning FREEZES the state where it stands; it does not zero
   * it. A converged bias is worth keeping, and a bad one is cleared by
   * `ekf3 reset`, which is the explicit action rather than a side effect of
   * changing a parameter.
   */

  ekf->bias_learn_inhibit = 0;

  if (!learn_gyro)
    {
      ekf->bias_learn_inhibit |= EKF_INHIBIT_GYRO_BIAS;
    }

  if (!learn_accel)
    {
      ekf->bias_learn_inhibit |= EKF_INHIBIT_ACCEL_BIAS;
    }
}

void ekf_core_set_bias_limits(FAR struct ekf_core_s *ekf, float gyro_limit,
                              float accel_limit)
{
  if (ekf == NULL)
    {
      return;
    }

  /* A limit may be tightened but never loosened. Past the ceiling the
   * "bias" is big enough to be hiding a real acceleration, and a parameter
   * is not evidence that it is not.
   */

  ekf->gyro_bias_limit =
    (gyro_limit > 0.0f && gyro_limit < EKF_GYRO_BIAS_LIMIT) ?
      gyro_limit : EKF_GYRO_BIAS_LIMIT;

  ekf->accel_bias_limit =
    (accel_limit > 0.0f && accel_limit < EKF_ACCEL_BIAS_LIMIT) ?
      accel_limit : EKF_ACCEL_BIAS_LIMIT;
}

void ekf_core_set_position_hold(FAR struct ekf_core_s *ekf, float limit_m)
{
  if (ekf != NULL)
    {
      ekf->position_hold_limit = (limit_m > 0.0f && isfinite(limit_m)) ?
                                 limit_m : 0.0f;
    }
}

/* Hold horizontal position where the last valid fix left it.
 *
 * Position is not observable from an IMU. Left alone, the strapdown
 * integrates accelerometer error twice and the estimate leaves
 * quadratically - metres within seconds on a ground vehicle, and it never
 * comes back. Worse, the filter has a way to "explain" the excursion: it
 * learns an accelerometer bias to match, and that bias corrupts the gravity
 * reference and takes attitude with it.
 *
 * So while nothing is aiding position, the estimate is bounded to where the
 * last fix put it and accel bias learning is frozen. The result is wrong -
 * the vehicle really is moving - but it is wrong by a bounded amount, it
 * says so through POSITION_HORIZ, and it is recoverable the instant a fix
 * returns. Unbounded dead reckoning is none of those things.
 */

static void constrain_position(FAR struct ekf_core_s *ekf)
{
  int axis;

  if (!(ekf->position_hold_limit > 0.0f) || ekf_core_position_aided(ekf))
    {
      ekf->position_holding = false;
      return;
    }

  /* Entering the hold: remember where the last valid fix left us. */

  if (!ekf->position_holding)
    {
      ekf->position_holding = true;
      ekf->position_hold_latch[0] = ekf->position[0];
      ekf->position_hold_latch[1] = ekf->position[1];
      ekf->position_hold_count++;
    }

  /* Unobservable without a fix, and the state whose corruption takes
   * attitude with it. Frozen for as long as the hold lasts.
   */

  ekf->inhibit_mask |= EKF_INHIBIT_ACCEL_BIAS;

  /* The inertial equations continue propagating, as they do in ArduPilot;
   * observability controls fusion, never the process model. The hold is a
   * bounded fallback around that propagation. Its covariance must admit that
   * an unaided position is not known.
   */

  for (axis = 0; axis < 2; axis++)
    {
      if (ekf->covariance[EKF_P_INDEX(6 + axis, 6 + axis)] <
          EKF_POSITION_HOLD_VAR)
        {
          ekf->covariance[EKF_P_INDEX(6 + axis, 6 + axis)] =
            EKF_POSITION_HOLD_VAR;
        }
    }

  for (axis = 0; axis < 2; axis++)
    {
      if (ekf->covariance[EKF_P_INDEX(3 + axis, 3 + axis)] <
          EKF_VELOCITY_HOLD_VAR)
        {
          ekf->covariance[EKF_P_INDEX(3 + axis, 3 + axis)] =
            EKF_VELOCITY_HOLD_VAR;
        }
    }

  /* The bound is a backstop around the propagated position. Crossing it is
   * expected during prolonged unaided motion, but the state is not allowed
   * to run farther away or retain an outward horizontal velocity.
   */

  if (!(ekf->position_hold_limit > 0.0f))
    {
      return;
    }

  for (axis = 0; axis < 2; axis++)
    {
      float excursion = ekf->position[axis] - ekf->position_hold_latch[axis];
      float sign;

      if (fabsf(excursion) <= ekf->position_hold_limit)
        {
          continue;
        }

      sign = excursion > 0.0f ? 1.0f : -1.0f;
      ekf->position[axis] = ekf->position_hold_latch[axis] +
                            sign * ekf->position_hold_limit;

      if (ekf->velocity[axis] * sign > 0.0f)
        {
          ekf->velocity[axis] = 0.0f;
        }
    }
}

static void constrain_height(FAR struct ekf_core_s *ekf)
{
  float excess;

  if (!(ekf->height_limit > 0.0f))
    {
      return;
    }

  if (fabsf(ekf->position[2]) <= ekf->height_limit)
    {
      return;
    }

  excess = ekf->position[2] > 0.0f ? 1.0f : -1.0f;
  ekf->position[2] = excess * ekf->height_limit;

  if (ekf->velocity[2] * excess > 0.0f)
    {
      ekf->velocity[2] = 0.0f;
    }

  if (ekf->covariance[EKF_P_INDEX(8, 8)] < EKF_HEIGHT_LIMIT_VAR)
    {
      ekf->covariance[EKF_P_INDEX(8, 8)] = EKF_HEIGHT_LIMIT_VAR;
    }

  ekf->height_clamp_count++;
}

void ekf_core_set_height_limit(FAR struct ekf_core_s *ekf, float limit_m)
{
  if (ekf != NULL)
    {
      ekf->height_limit = (limit_m > 0.0f && isfinite(limit_m)) ?
                          limit_m : 0.0f;
    }
}

static void constrain_biases(FAR struct ekf_core_s *ekf)
{
  int axis;

  for (axis = 0; axis < 3; axis++)
    {
      if (ekf->gyro_bias[axis] > ekf->gyro_bias_limit)
        {
          ekf->gyro_bias[axis] = ekf->gyro_bias_limit;
          ekf->bias_limit_count++;
        }
      else if (ekf->gyro_bias[axis] < -ekf->gyro_bias_limit)
        {
          ekf->gyro_bias[axis] = -ekf->gyro_bias_limit;
          ekf->bias_limit_count++;
        }

      if (ekf->accel_bias[axis] > ekf->accel_bias_limit)
        {
          ekf->accel_bias[axis] = ekf->accel_bias_limit;
          ekf->bias_limit_count++;
        }
      else if (ekf->accel_bias[axis] < -ekf->accel_bias_limit)
        {
          ekf->accel_bias[axis] = -ekf->accel_bias_limit;
          ekf->bias_limit_count++;
        }

      if (fabsf(ekf->gyro_bias[axis]) >= EKF_GYRO_BIAS_LIMIT &&
          ekf->covariance[EKF_P_INDEX(9 + axis, 9 + axis)] <
          EKF_GYRO_BIAS_LIMIT_VAR)
        {
          ekf->covariance[EKF_P_INDEX(9 + axis, 9 + axis)] =
            EKF_GYRO_BIAS_LIMIT_VAR;
        }

      if (fabsf(ekf->accel_bias[axis]) >= EKF_ACCEL_BIAS_LIMIT &&
          ekf->covariance[EKF_P_INDEX(12 + axis, 12 + axis)] <
          EKF_ACCEL_BIAS_LIMIT_VAR)
        {
          ekf->covariance[EKF_P_INDEX(12 + axis, 12 + axis)] =
            EKF_ACCEL_BIAS_LIMIT_VAR;
        }
    }
}

/* Returns one for an accepted update, zero for a gated innovation, and minus
 * one for a numerical failure. The covariance expression is the expanded
 * Joseph form:
 *
 *   P - KHP - PH'K' + K(HPH' + R)K'
 *
 * which avoids two 15x15 workspaces without dropping the stabilizing terms.
 */

/* The inhibit mask actually in force: whatever this update set, plus
 * whatever the operator has frozen permanently through EK3_ABIAS_EN and
 * EK3_GBIAS_EN.
 */

static uint8_t effective_inhibit(FAR const struct ekf_core_s *ekf)
{
  return (uint8_t)(ekf->inhibit_mask | ekf->bias_learn_inhibit);
}

static int measurement_update_3d(
  FAR struct ekf_core_s *ekf,
  FAR const float h[3][EKF_STATE_DIM],
  FAR const float residual[3], float noise_variance, FAR float *nis,
  FAR const float unobservable_axis[3], FAR float *suppressed_correction,
  uint16_t gain_mask)
{
  float pht[EKF_STATE_DIM][3];
  float innovation[3][3];
  float innovation_inverse[3][3];
  float gain[EKF_STATE_DIM][3];
  float correction[EKF_STATE_DIM];
  float delta_quaternion[4];
  float next_quaternion[4];
  float local_nis = 0.0f;
  float local_suppressed = 0.0f;
  int row;
  int column;
  int measurement;
  int inner;
  int axis;

  if (ekf == NULL || h == NULL || residual == NULL || nis == NULL ||
      !isfinite(noise_variance) || noise_variance <= 0.0f)
    {
      return -1;
    }

  if (suppressed_correction != NULL)
    {
      *suppressed_correction = 0.0f;
    }

  memset(pht, 0, sizeof(pht));

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      for (measurement = 0; measurement < 3; measurement++)
        {
          for (inner = 0; inner < EKF_STATE_DIM; inner++)
            {
              pht[row][measurement] +=
                ekf->covariance[EKF_P_INDEX(row, inner)] *
                h[measurement][inner];
            }
        }
    }

  for (row = 0; row < 3; row++)
    {
      for (column = 0; column < 3; column++)
        {
          innovation[row][column] = row == column ?
                                    noise_variance : 0.0f;

          for (inner = 0; inner < EKF_STATE_DIM; inner++)
            {
              innovation[row][column] +=
                h[row][inner] * pht[inner][column];
            }
        }
    }

  if (!invert_symmetric_3x3(innovation, innovation_inverse))
    {
      return -1;
    }

  for (row = 0; row < 3; row++)
    {
      for (column = 0; column < 3; column++)
        {
          local_nis += residual[row] * innovation_inverse[row][column] *
                       residual[column];
        }
    }

  *nis = local_nis;

  if (!isfinite(local_nis))
    {
      return -1;
    }

  if (local_nis > EKF_MEASUREMENT_NIS_GATE)
    {
      return 0;
    }

  memset(gain, 0, sizeof(gain));
  memset(correction, 0, sizeof(correction));

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      for (measurement = 0; measurement < 3; measurement++)
        {
          for (inner = 0; inner < 3; inner++)
            {
              gain[row][measurement] +=
                pht[row][inner] * innovation_inverse[inner][measurement];
            }
        }

      /* An observation may only update the state families its fusion path
       * made available. Do this before every projection and correction so
       * the same gain reaches the state and Joseph covariance updates.
       */

      if ((gain_mask & EKF_GAIN_BIT(row)) == 0)
        {
          for (measurement = 0; measurement < 3; measurement++)
            {
              gain[row][measurement] = 0.0f;
            }
        }
    }

  /* A gravity observation has no information about rotation around gravity,
   * nor about gyro bias along that axis. Cross-covariance can nevertheless
   * create Kalman gain in those gauge directions. Project it out before both
   * state and Joseph covariance updates so gravity cannot corrupt the stable
   * gyro-propagated yaw path.
   */

  if (unobservable_axis != NULL)
    {
      for (measurement = 0; measurement < 3; measurement++)
        {
          float attitude_component = 0.0f;
          float bias_component = 0.0f;

          for (axis = 0; axis < 3; axis++)
            {
              attitude_component +=
                unobservable_axis[axis] * gain[axis][measurement];
              bias_component +=
                unobservable_axis[axis] * gain[9 + axis][measurement];
            }

          local_suppressed += attitude_component * residual[measurement];

          for (axis = 0; axis < 3; axis++)
            {
              gain[axis][measurement] -=
                unobservable_axis[axis] * attitude_component;
              gain[9 + axis][measurement] -=
                unobservable_axis[axis] * bias_component;
            }
        }
    }

  /* Form the correction only after projecting the gain. This also guarantees
   * the state injection and Joseph covariance update use exactly the same
   * observable subspace.
   */

  /* Freeze the states this update is not allowed to move.
   *
   * Zeroing the gain ROW rather than skipping the update keeps the Joseph
   * covariance update below consistent with the correction actually
   * applied, and it has to happen HERE - before both - or P would claim an
   * information gain the state never received.
   *
   * This path matters more than it looks: low_dynamics_updates observes
   * accelerometer bias DIRECTLY, through h[axis][12 + axis], so it is the
   * dominant way that bias is learned. An inhibit that only covered the
   * scalar update, as this one did, left the main path wide open.
   */

  {
    uint8_t inhibit = effective_inhibit(ekf);

    for (row = 9; row < EKF_STATE_DIM; row++)
      {
        if ((inhibit & (1u << (row - 9))) == 0)
          {
            continue;
          }

        for (measurement = 0; measurement < 3; measurement++)
          {
            gain[row][measurement] = 0.0f;
          }

        ekf->inhibit_applied_count++;
      }
  }

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      for (measurement = 0; measurement < 3; measurement++)
        {
          correction[row] += gain[row][measurement] *
                             residual[measurement];
        }
    }

  /* Remove the last floating-point residue in the two gauge components from
   * the correction itself. The gain projection above remains authoritative
   * for the covariance update.
   */

  if (unobservable_axis != NULL)
    {
      float attitude_component = 0.0f;
      float bias_component = 0.0f;

      for (axis = 0; axis < 3; axis++)
        {
          attitude_component += unobservable_axis[axis] * correction[axis];
          bias_component += unobservable_axis[axis] * correction[9 + axis];
        }

      for (axis = 0; axis < 3; axis++)
        {
          correction[axis] -=
            unobservable_axis[axis] * attitude_component;
          correction[9 + axis] -=
            unobservable_axis[axis] * bias_component;
        }
    }

  if (suppressed_correction != NULL)
    {
      *suppressed_correction = local_suppressed;
    }

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      for (column = row; column < EKF_STATE_DIM; column++)
        {
          float first = 0.0f;
          float second = 0.0f;
          float third = 0.0f;
          float value;

          for (measurement = 0; measurement < 3; measurement++)
            {
              first += gain[row][measurement] *
                       pht[column][measurement];
              second += pht[row][measurement] *
                        gain[column][measurement];

              for (inner = 0; inner < 3; inner++)
                {
                  third += gain[row][measurement] *
                           innovation[measurement][inner] *
                           gain[column][inner];
                }
            }

          value = ekf->covariance[EKF_P_INDEX(row, column)] -
                  first - second + third;

          if (!isfinite(value))
            {
              return -1;
            }

          ekf->covariance[EKF_P_INDEX(row, column)] = value;
          ekf->covariance[EKF_P_INDEX(column, row)] = value;
        }
    }

  rotation_vector_quaternion(correction, delta_quaternion);
  quaternion_multiply(ekf->quaternion, delta_quaternion,
                      next_quaternion);
  memcpy(ekf->quaternion, next_quaternion, sizeof(ekf->quaternion));

  if (!quaternion_normalize(ekf->quaternion))
    {
      return -1;
    }

  for (row = 0; row < 3; row++)
    {
      ekf->velocity[row] += correction[3 + row];
      ekf->position[row] += correction[6 + row];
      ekf->gyro_bias[row] += correction[9 + row];
      ekf->accel_bias[row] += correction[12 + row];
    }

  if (!covariance_reset_attitude(ekf, correction))
    {
      return -1;
    }

  constrain_biases(ekf);
  constrain_height(ekf);
  constrain_position(ekf);

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      float *diagonal =
        &ekf->covariance[EKF_P_INDEX(row, row)];

      if (!isfinite(*diagonal))
        {
          return -1;
        }

      if (*diagonal < EKF_MIN_VARIANCE)
        {
          *diagonal = EKF_MIN_VARIANCE;
        }
    }

  return 1;
}

/* Scalar form of measurement_update_3d(). Same expanded Joseph covariance
 * form, same numerical guards, same post-update attitude covariance reset and
 * bias constraint - so the two cannot develop different ideas about what a
 * safe update is.
 *
 * Returns one for an accepted update, zero for a gated innovation, and minus
 * one for a numerical failure. On a gated innovation NOTHING is modified: a
 * partially applied correction would be worse than either outcome, because it
 * is neither the old estimate nor the new one.
 */

static int measurement_update_1d(FAR struct ekf_core_s *ekf,
                                 FAR const float h[EKF_STATE_DIM],
                                 float residual, float noise_variance,
                                 float gate_sigma, FAR float *nis,
                                 uint16_t gain_mask,
                                 FAR const float attitude_axis[3])
{
  float pht[EKF_STATE_DIM];
  float gain[EKF_STATE_DIM];
  float correction[EKF_STATE_DIM];
  float delta_quaternion[4];
  float next_quaternion[4];
  float innovation = noise_variance;
  float local_nis;
  int row;
  int column;
  int inner;

  if (ekf == NULL || h == NULL || !isfinite(residual) ||
      !isfinite(noise_variance) || noise_variance <= 0.0f ||
      !isfinite(gate_sigma) || gate_sigma <= 0.0f)
    {
      return -1;
    }

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      pht[row] = 0.0f;

      for (inner = 0; inner < EKF_STATE_DIM; inner++)
        {
          pht[row] += ekf->covariance[EKF_P_INDEX(row, inner)] * h[inner];
        }
    }

  for (inner = 0; inner < EKF_STATE_DIM; inner++)
    {
      innovation += h[inner] * pht[inner];
    }

  if (!isfinite(innovation) || innovation <= 0.0f)
    {
      return -1;
    }

  local_nis = residual * residual / innovation;

  if (nis != NULL)
    {
      *nis = local_nis;
    }

  if (!isfinite(local_nis))
    {
      return -1;
    }

  if (local_nis > gate_sigma * gate_sigma)
    {
      return 0;
    }

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      gain[row] = pht[row] / innovation;

      if ((gain_mask & EKF_GAIN_BIT(row)) == 0)
        {
          gain[row] = 0.0f;
        }

      /* Freeze the states this update is not allowed to move.
       *
       * Zeroing the gain row rather than skipping the update is what
       * ArduPilot does for inhibited states: the covariance update below
       * still runs, so P stays consistent with the correction that was
       * actually applied instead of claiming an information gain the state
       * never received.
       */

      if (row >= 9 &&
          (effective_inhibit(ekf) & (1u << (row - 9))) != 0)
        {
          gain[row] = 0.0f;
          ekf->inhibit_applied_count++;
        }
    }

  /* A yaw innovation is observable only about navigation up. In a
   * right-error state that direction is expressed in body axes and generally
   * has x/y components when tilted, so zeroing error-state indices 0 and 1
   * would be wrong. Project attitude and gyro-bias gain onto the physical yaw
   * axis instead; the resulting quaternion correction preserves roll/pitch.
   */

  if (attitude_axis != NULL)
    {
      float attitude_component = 0.0f;
      float bias_component = 0.0f;
      int axis;

      for (axis = 0; axis < 3; axis++)
        {
          attitude_component += attitude_axis[axis] * gain[axis];
          bias_component += attitude_axis[axis] * gain[9 + axis];
        }

      for (axis = 0; axis < 3; axis++)
        {
          gain[axis] = attitude_axis[axis] * attitude_component;
          gain[9 + axis] = attitude_axis[axis] * bias_component;
        }
    }

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      correction[row] = gain[row] * residual;
    }

  /* P - KHP - PH'K' + K(HPH' + R)K', evaluated on the upper triangle from
   * the unchanged P/PHt workspaces and mirrored exactly.
   */

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      for (column = row; column < EKF_STATE_DIM; column++)
        {
          float value = ekf->covariance[EKF_P_INDEX(row, column)] -
                        gain[row] * pht[column] -
                        pht[row] * gain[column] +
                        gain[row] * innovation * gain[column];

          if (!isfinite(value))
            {
              return -1;
            }

          ekf->covariance[EKF_P_INDEX(row, column)] = value;
          ekf->covariance[EKF_P_INDEX(column, row)] = value;
        }
    }

  rotation_vector_quaternion(correction, delta_quaternion);
  quaternion_multiply(ekf->quaternion, delta_quaternion, next_quaternion);
  memcpy(ekf->quaternion, next_quaternion, sizeof(ekf->quaternion));

  if (!quaternion_normalize(ekf->quaternion))
    {
      return -1;
    }

  for (row = 0; row < 3; row++)
    {
      ekf->velocity[row] += correction[3 + row];
      ekf->position[row] += correction[6 + row];
      ekf->gyro_bias[row] += correction[9 + row];
      ekf->accel_bias[row] += correction[12 + row];
    }

  if (!covariance_reset_attitude(ekf, correction))
    {
      return -1;
    }

  constrain_biases(ekf);
  constrain_height(ekf);
  constrain_position(ekf);

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      FAR float *diagonal = &ekf->covariance[EKF_P_INDEX(row, row)];

      if (!isfinite(*diagonal))
        {
          return -1;
        }

      if (*diagonal < EKF_MIN_VARIANCE)
        {
          *diagonal = EKF_MIN_VARIANCE;
        }
    }

  return 1;
}

/* Continuous tilt reference for an attitude-only lane.
 *
 * Same geometry as low_dynamics_updates - the measurement is that specific
 * force should equal gravity rotated into the body - but it runs on EVERY
 * step rather than only at a standstill, with the noise scaled by how far
 * the measured magnitude departs from g.
 *
 * The scaling is what makes that legitimate. Under acceleration the gravity
 * assumption is simply false, and a fixed noise would pull the attitude
 * towards the acceleration vector. De-weighting instead lets the update
 * contribute when the specific force looks like gravity and fade out when it
 * does not, which is what a complementary AHRS does and why one holds tilt
 * on a moving vehicle where a standstill-only update cannot.
 *
 * Accel bias is NOT learned here. Without velocity aiding a bias and a tilt
 * produce the identical measurement, so the filter would trade one for the
 * other and wander - the exact divergence this lane exists to detect in
 * somebody else.
 */

static bool attitude_only_tilt_update(FAR struct ekf_core_s *ekf)
{
  float h[3][EKF_STATE_DIM];
  float residual[3];
  float rotation[3][3];
  float gravity_body[3];
  float gravity_axis[3];
  float measured[3];
  float suppressed_correction = 0.0f;
  float dt = ekf->covariance_dt;
  float magnitude;
  float departure_g;
  float noise;
  int result;
  int axis;

  if (dt <= 0.0f)
    {
      return true;
    }

  for (axis = 0; axis < 3; axis++)
    {
      measured[axis] = ekf->covariance_delta_velocity[axis] / dt;
    }

  magnitude = vector_norm(measured);

  if (!isfinite(magnitude))
    {
      return true;
    }

  departure_g = fabsf(magnitude - EKF_GRAVITY) / EKF_GRAVITY;

  /* Too far from gravity to carry tilt information at all. */

  if (departure_g > EKF_TILT_REJECT_G)
    {
      ekf->tilt_skipped_count++;
      return true;
    }

  noise = EKF_TILT_MEAS_NOISE * (1.0f + EKF_TILT_NOISE_PER_G * departure_g);

  quaternion_to_rotation(ekf->quaternion, rotation);
  gravity_body[0] = rotation[2][0] * EKF_GRAVITY;
  gravity_body[1] = rotation[2][1] * EKF_GRAVITY;
  gravity_body[2] = rotation[2][2] * EKF_GRAVITY;
  memset(h, 0, sizeof(h));

  h[0][1] = -gravity_body[2];
  h[0][2] = gravity_body[1];
  h[1][0] = gravity_body[2];
  h[1][2] = -gravity_body[0];
  h[2][0] = -gravity_body[1];
  h[2][1] = gravity_body[0];

  for (axis = 0; axis < 3; axis++)
    {
      gravity_axis[axis] = gravity_body[axis] / EKF_GRAVITY;
      residual[axis] = measured[axis] - gravity_body[axis];
    }

  /* Accel bias frozen for the duration of this update, for the reason in the
   * comment above. h has no bias columns either, so this is belt and braces
   * against the cross-covariance path.
   */

  ekf->inhibit_mask = EKF_INHIBIT_ACCEL_BIAS;

  result = measurement_update_3d(ekf, h, residual, noise * noise,
                                 &ekf->last_gravity_nis, gravity_axis,
                                 &suppressed_correction, EKF_GAIN_GRAVITY);

  ekf->inhibit_mask = 0;

  if (result < 0)
    {
      return false;
    }

  if (result == 0)
    {
      ekf->gravity_reject_count++;
    }
  else
    {
      ekf->tilt_update_count++;
    }

  return true;
}

void ekf_core_set_tilt_fusion_moving(FAR struct ekf_core_s *ekf, bool enable)
{
  if (ekf != NULL)
    {
      ekf->tilt_fusion_moving = enable;
    }
}

void ekf_core_set_attitude_only(FAR struct ekf_core_s *ekf, bool enable)
{
  if (ekf != NULL)
    {
      ekf->attitude_only = enable;
    }
}

void ekf_core_up_in_body(FAR const float quaternion[4], FAR float up[3])
{
  float rotation[3][3];

  if (quaternion == NULL || up == NULL)
    {
      return;
    }

  quaternion_to_rotation(quaternion, rotation);

  /* The third ROW of R, which is R' applied to (0,0,1). Using the third
   * column instead would give where body z points in the nav frame - a
   * different vector, and one that is not yaw-free.
   */

  up[0] = rotation[2][0];
  up[1] = rotation[2][1];
  up[2] = rotation[2][2];
}

float ekf_core_tilt_difference(FAR const float quaternion_a[4],
                               FAR const float quaternion_b[4],
                               FAR float error[3])
{
  float up_a[3];
  float up_b[3];
  float cross[3];
  float dot;
  float angle;

  if (error != NULL)
    {
      error[0] = 0.0f;
      error[1] = 0.0f;
      error[2] = 0.0f;
    }

  if (quaternion_a == NULL || quaternion_b == NULL)
    {
      return 0.0f;
    }

  ekf_core_up_in_body(quaternion_a, up_a);
  ekf_core_up_in_body(quaternion_b, up_b);

  cross[0] = up_a[1] * up_b[2] - up_a[2] * up_b[1];
  cross[1] = up_a[2] * up_b[0] - up_a[0] * up_b[2];
  cross[2] = up_a[0] * up_b[1] - up_a[1] * up_b[0];

  dot = up_a[0] * up_b[0] + up_a[1] * up_b[1] + up_a[2] * up_b[2];

  /* atan2 of the cross magnitude against the dot, not acos of the dot: acos
   * loses all resolution near zero, which is precisely where a monitor
   * spends its time and where it has to be able to see a trend.
   */

  angle = atan2f(vector_norm(cross), dot);

  if (error != NULL)
    {
      /* Small-angle rotation vector taking a onto b, in body axes. The z
       * component is dropped rather than reported: gravity cannot see yaw,
       * so whatever lands there is numerical noise, not a measurement.
       */

      error[0] = cross[0];
      error[1] = cross[1];
      error[2] = 0.0f;
    }

  return angle;
}

static bool low_dynamics_updates(FAR struct ekf_core_s *ekf)
{
  float h[3][EKF_STATE_DIM];
  float residual[3];
  float rotation[3][3];
  float gravity_body[3];
  float gravity_axis[3];
  float suppressed_correction = 0.0f;
  float dt = ekf->covariance_dt;
  int result;
  int axis;

  if (!ekf->low_dynamics)
    {
      return true;
    }

  quaternion_to_rotation(ekf->quaternion, rotation);
  gravity_body[0] = rotation[2][0] * EKF_GRAVITY;
  gravity_body[1] = rotation[2][1] * EKF_GRAVITY;
  gravity_body[2] = rotation[2][2] * EKF_GRAVITY;
  memset(h, 0, sizeof(h));

  h[0][1] = -gravity_body[2];
  h[0][2] = gravity_body[1];
  h[1][0] = gravity_body[2];
  h[1][2] = -gravity_body[0];
  h[2][0] = -gravity_body[1];
  h[2][1] = gravity_body[0];

  for (axis = 0; axis < 3; axis++)
    {
      gravity_axis[axis] = gravity_body[axis] / EKF_GRAVITY;
      h[axis][12 + axis] = 1.0f;
      residual[axis] = ekf->covariance_delta_velocity[axis] / dt -
                       gravity_body[axis] - ekf->accel_bias[axis];
    }

  result = measurement_update_3d(
    ekf, h, residual,
    EKF_GRAVITY_MEAS_NOISE * EKF_GRAVITY_MEAS_NOISE,
    &ekf->last_gravity_nis, gravity_axis, &suppressed_correction,
    EKF_GAIN_GRAVITY);

  if (result < 0)
    {
      return false;
    }
  else if (result == 0)
    {
      ekf->gravity_reject_count++;
    }
  else
    {
      float magnitude = fabsf(suppressed_correction);

      ekf->gravity_accept_count++;
      ekf->last_gravity_yaw_suppressed = suppressed_correction;
      ekf->gravity_yaw_projection_count++;

      if (magnitude > ekf->max_gravity_yaw_suppressed)
        {
          ekf->max_gravity_yaw_suppressed = magnitude;
        }
    }

  return true;
}

/* One strapdown integration step, shared by the filter and the output
 * predictor.
 *
 * Factored out rather than duplicated because two copies of this that were
 * meant to be identical would be a slow, silent divergence between what the
 * filter believes and what the vehicle is told. Returns false when the step
 * produces a non-finite state.
 */

static bool strapdown_step(FAR float quaternion[4], FAR float velocity[3],
                           FAR float position[3],
                           FAR const float gyro_bias[3],
                           FAR const float accel_bias[3],
                           FAR const struct ekf_imu_sample_s *sample)
{
  float corrected_angle[3];
  float corrected_velocity[3];
  float increment[4];
  float next_quaternion[4];
  float rotation[3][3];
  float nav_delta_velocity[3];
  float old_velocity[3];
  float angle_dt = sample->delta_angle_dt;
  float velocity_dt = sample->delta_velocity_dt;
  int row;
  int axis;

  for (axis = 0; axis < 3; axis++)
    {
      corrected_angle[axis] = sample->delta_angle[axis] -
                              gyro_bias[axis] * angle_dt;
      corrected_velocity[axis] = sample->delta_velocity[axis] -
                                 accel_bias[axis] * velocity_dt;
      old_velocity[axis] = velocity[axis];
    }

  /* vehicle_imu delta_velocity is already sculling-corrected and expressed
   * in the body frame at the START of the packet. imu_integrator rotated each
   * native-rate increment into that frame while accumulating it. Rotating it
   * with a packet-midpoint attitude here applies half of the packet rotation
   * a second time; use the matching start attitude exactly once.
   */

  quaternion_to_rotation(quaternion, rotation);

  for (row = 0; row < 3; row++)
    {
      nav_delta_velocity[row] =
        rotation[row][0] * corrected_velocity[0] +
        rotation[row][1] * corrected_velocity[1] +
        rotation[row][2] * corrected_velocity[2];
    }

  nav_delta_velocity[2] -= EKF_GRAVITY * velocity_dt;

  /* Propagation is the process model and is independent of which
   * measurements happen to be available. Stopping velocity or position when
   * aiding disappears makes the nominal state follow different dynamics from
   * the covariance (which still applies F), and it is not how ArduPilot's EKF
   * handles loss of aiding. Availability is enforced by fusion gain masks;
   * the ground-vehicle position hold bounds unaided drift afterwards.
   */

  for (axis = 0; axis < 3; axis++)
    {
      velocity[axis] += nav_delta_velocity[axis];
      position[axis] +=
        (old_velocity[axis] + 0.5f * nav_delta_velocity[axis]) * velocity_dt;
    }

  rotation_vector_quaternion(corrected_angle, increment);
  quaternion_multiply(quaternion, increment, next_quaternion);
  memcpy(quaternion, next_quaternion, 4 * sizeof(float));

  return quaternion_normalize(quaternion) &&
         vector_finite(velocity) && vector_finite(position);
}

/* Choose the tilt reference for this step.
 *
 * At a standstill low_dynamics_updates is the better of the two: it observes
 * accelerometer bias DIRECTLY, which is the one time that bias is cleanly
 * separable from tilt, and giving that up would cost the filter its only
 * good look at it.
 *
 * While MOVING that update does not run at all - its entry conditions
 * require the vehicle to be nearly still. Historically nothing replaced it,
 * so a driving vehicle had no tilt reference whatsoever and roll and pitch
 * were pure gyro integration between stops. That is survivable while
 * position aiding can correct tilt through the covariance, and becomes a
 * real defect once that path is masked: a half-degree of accumulated tilt is
 * 0.086 m/s^2 of specific force that the filter reads as real acceleration,
 * which ramps velocity in dead reckoning and makes every position fix arrive
 * as a large correction that does nothing about the cause.
 *
 * So when moving, fall back to the continuous tilt update: gravity fused
 * every step with its noise scaled by how far the measurement departs from
 * g, trusted when the vehicle is coasting and faded out under acceleration.
 * Accelerometer bias stays inhibited there, because that is exactly the
 * regime where bias and tilt are indistinguishable.
 */

static bool attitude_updates(FAR struct ekf_core_s *ekf)
{
  if (ekf->attitude_only)
    {
      return attitude_only_tilt_update(ekf);
    }

  if (ekf->low_dynamics)
    {
      return low_dynamics_updates(ekf);
    }

  if (ekf->tilt_fusion_moving)
    {
      return attitude_only_tilt_update(ekf);
    }

  return true;
}

static bool nominal_predict(FAR struct ekf_core_s *ekf,
                            FAR const struct ekf_imu_sample_s *sample)
{
  float dt = sample->delta_angle_dt;
  int axis;

  if (!strapdown_step(ekf->quaternion, ekf->velocity, ekf->position,
                      ekf->gyro_bias, ekf->accel_bias, sample))
    {
      return false;
    }

  for (axis = 0; axis < 3; axis++)
    {
      ekf->covariance_delta_angle[axis] += sample->delta_angle[axis];
      ekf->covariance_delta_velocity[axis] +=
        sample->delta_velocity[axis];
    }

  ekf->covariance_dt += dt;
  ekf->covariance_phase++;
  ekf->covariance_clipping |= sample->clipping;
  ekf->predict_count++;

  if (ekf->covariance_phase >= EKF_COVARIANCE_INTERVAL)
    {
      /* An attitude-only lane takes the continuous tilt reference instead
       * of the standstill-only one. Not as well as: they are the same
       * measurement, and running both would fuse it twice.
       */

      if (!covariance_predict(ekf) ||
          !attitude_updates(ekf))
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

  dynamics_update(ekf, sample);

  if (!ekf->initialized)
    {
      return alignment_add(ekf, sample) ? EKF_PROCESS_INITIALIZED :
             EKF_PROCESS_ALIGNING;
    }

  /* Snapshot which solution components have measurement support. This is a
   * validity classification only; propagation always follows the same
   * inertial process model.
   */

  ekf->observability = ekf_core_observability(ekf);

  if (!nominal_predict(ekf, sample))
    {
      ekf->numerical_reset_count++;
      ekf->rejected_count++;
      ekf->reset_counter++;
      restart_alignment(ekf);
      return EKF_PROCESS_REJECTED;
    }

  /* Also after the strapdown. The update paths constrain height too, but a
   * filter coasting with no aiding never reaches them, and coasting is
   * exactly when the vertical state runs away.
   */

  constrain_height(ekf);
  constrain_position(ekf);

  /* Yaw pinned at zero, once per sample and after the strapdown - which is
   * where yaw actually accumulates, from the z gyro.
   *
   * An unaided lane has no heading reference of any kind, so an integrated
   * yaw is drift dressed up as a measurement. Zero is the honest value, and
   * the comparison this lane exists for is yaw-free anyway: see
   * ekf_core_tilt_difference.
   */

  if (ekf->attitude_only)
    {
      float euler[3];

      ekf_core_euler(ekf, euler);
      quaternion_from_euler(euler[0], euler[1], 0.0f, ekf->quaternion);
      quaternion_normalize(ekf->quaternion);
    }

  return EKF_PROCESS_PREDICTED;
}

static float wrap_pi(float angle)
{
  const float two_pi = 6.283185307179586f;

  if (!isfinite(angle))
    {
      return 0.0f;
    }

  while (angle > 3.141592653589793f)
    {
      angle -= two_pi;
    }

  while (angle < -3.141592653589793f)
    {
      angle += two_pi;
    }

  return angle;
}

bool ekf_mag_heading(FAR const float quaternion[4],
                     FAR const float field[3], float declination,
                     FAR float *heading)
{
  float roll;
  float pitch;
  float sr;
  float cr;
  float sp;
  float cp;
  float level_x;
  float level_y;
  float rotation[3][3];

  if (quaternion == NULL || field == NULL || heading == NULL ||
      !vector_finite(field) || !isfinite(declination))
    {
      return false;
    }

  /* Roll and pitch ONLY. The current yaw is the quantity being measured, so
   * using it would make this a very expensive way to read back the state it
   * is supposed to correct.
   *
   * Taken from the rotation matrix rather than through an euler conversion,
   * so a pitch near vertical degrades into a short horizontal projection -
   * caught below - instead of a singularity.
   */

  quaternion_to_rotation(quaternion, rotation);
  roll = atan2f(rotation[2][1], rotation[2][2]);
  pitch = -asinf(rotation[2][0] > 1.0f ? 1.0f :
                 rotation[2][0] < -1.0f ? -1.0f : rotation[2][0]);

  sr = sinf(roll);
  cr = cosf(roll);
  sp = sinf(pitch);
  cp = cosf(pitch);

  /* Rotate the field into the vehicle's level frame: Ry(pitch) * Rx(roll)
   * applied to the body-frame measurement.
   */

  level_x = cp * field[0] + sp * sr * field[1] + sp * cr * field[2];
  level_y = cr * field[1] - sr * field[2];

  /* A horizontal projection this short carries no direction. It happens when
   * the field is nearly parallel to the vertical - at extreme dip, or when
   * the sensor has failed to something constant.
   */

  if (!isfinite(level_x) || !isfinite(level_y) ||
      level_x * level_x + level_y * level_y < 1.0e-8f)
    {
      return false;
    }

  /* ENU: yaw is measured COUNTER-CLOCKWISE FROM EAST about the up axis, the
   * ROS REP-103 convention.
   *
   * The field points toward magnetic north. With the vehicle at yaw psi, a
   * level body sees it at (B_h*sin(psi), B_h*cos(psi), -B_d) - entirely to
   * the LEFT when facing east, straight ahead when facing north - so the
   * heading is atan2(x, y), not the atan2(y, x) a bearing-from-north frame
   * would use.
   *
   * Declination is SUBTRACTED. It is positive east, meaning magnetic north
   * lies east of true north, so a vehicle whose compass reads magnetic north
   * is actually pointing east of true north - a SMALLER counter-clockwise
   * angle. Adding it doubles the error instead of removing it.
   */

  *heading = wrap_pi(atan2f(level_x, level_y) - declination);
  return isfinite(*heading);
}

void ekf_core_set_mag_config(FAR struct ekf_core_s *ekf, float declination,
                             float yaw_variance)
{
  if (ekf == NULL)
    {
      return;
    }

  ekf->align_declination = isfinite(declination) ? declination : 0.0f;
  ekf->align_yaw_variance = isfinite(yaw_variance) && yaw_variance > 0.0f ?
                            yaw_variance : 0.0f;
}

void ekf_core_add_align_mag(FAR struct ekf_core_s *ekf,
                            FAR const float field[3])
{
  int axis;

  if (ekf == NULL || field == NULL || ekf->initialized ||
      !vector_finite(field))
    {
      return;
    }

  for (axis = 0; axis < 3; axis++)
    {
      ekf->align_mag_sum[axis] += field[axis];
    }

  ekf->align_mag_samples++;
}

/* Fuse an absolute yaw measurement.
 *
 * Shared by the magnetometer and by external navigation. Two copies of this
 * would be two ideas about what a yaw update is, free to drift apart while
 * both look correct.
 *
 * A yaw error is a rotation about the navigation UP axis. The state is a
 * BODY-frame rotation vector, so the observation is that axis expressed in
 * body - the third row of the body-to-nav rotation, which is exactly the
 * gauge direction the gravity update projects OUT.
 */

static int fuse_yaw(FAR struct ekf_core_s *ekf, float yaw_meas, float noise,
                    float gate_sigma, FAR float *nis)
{
  float h[EKF_STATE_DIM];
  float rotation[3][3];
  float euler[3];
  float residual;
  int axis;

  if (ekf == NULL || !isfinite(yaw_meas) || !isfinite(noise) ||
      noise <= 0.0f)
    {
      return -1;
    }

  ekf_core_euler(ekf, euler);
  residual = wrap_pi(yaw_meas - euler[2]);

  quaternion_to_rotation(ekf->quaternion, rotation);
  memset(h, 0, sizeof(h));

  for (axis = 0; axis < 3; axis++)
    {
      h[axis] = rotation[2][axis];
    }

  return measurement_update_1d(ekf, h, residual, noise * noise, gate_sigma,
                               nis, EKF_GAIN_YAW, h);
}

int ekf_core_fuse_mag(FAR struct ekf_core_s *ekf,
                      FAR const float field[3], float declination,
                      float expected_field, float noise,
                      float gate_sigma)
{
  float heading;
  float magnitude;
  int result;

  if (ekf == NULL || field == NULL || !ekf->initialized ||
      !vector_finite(field) || !isfinite(noise) || noise <= 0.0f)
    {
      return -1;
    }

  /* Without an absolute datum this would be a yaw JUMP rather than a
   * correction: the filter's heading is an arbitrary reference, and pulling
   * it to magnetic north in one step is not something a Kalman update should
   * be asked to express.
   */

  if (!ekf->yaw_absolute)
    {
      return -2;
    }

  magnitude = vector_norm(field);
  ekf->last_mag_field = magnitude;

  /* A magnitude far from the calibrated field is a magnet, a motor or a
   * failed sensor. Refuse it before it reaches the filter - the innovation
   * gate would catch a wrong heading, but not a right heading derived from a
   * field that is physically impossible.
   */

  if (expected_field > 0.0f &&
      fabsf(magnitude - expected_field) >
        EKF_MAG_FIELD_TOLERANCE * expected_field)
    {
      ekf->mag_unhealthy_count++;
      ekf->mag_consecutive_rejects++;
      return -1;
    }

  if (!ekf_mag_heading(ekf->quaternion, field, declination, &heading))
    {
      ekf->mag_unhealthy_count++;
      ekf->mag_consecutive_rejects++;
      return -1;
    }

  ekf->last_mag_heading = heading;

  result = fuse_yaw(ekf, heading, noise, gate_sigma, &ekf->last_mag_nis);

  if (result == 1)
    {
      ekf->mag_accept_count++;
      ekf->mag_consecutive_rejects = 0;
      ekf->last_mag_timestamp = ekf->last_timestamp_sample;
    }
  else if (result == 0)
    {
      ekf->mag_reject_count++;
      ekf->mag_consecutive_rejects++;
    }

  return result;
}

/* ISA height above a reference pressure. Positive is UP.
 *
 * Taken relative to the reference captured at alignment rather than to a
 * sea-level constant, which is what makes this a height above the alignment
 * point and consistent with the filter's local ENU origin.
 */

float ekf_baro_height(float pressure_hpa, float reference_hpa)
{
  if (!isfinite(pressure_hpa) || !isfinite(reference_hpa) ||
      pressure_hpa <= 0.0f || reference_hpa <= 0.0f)
    {
      return 0.0f;
    }

  return 44330.77f * (1.0f - powf(pressure_hpa / reference_hpa,
                                  0.1902632f));
}

int ekf_core_fuse_zero_velocity(FAR struct ekf_core_s *ekf, float noise,
                                float gate)
{
  float h[EKF_STATE_DIM];
  int accepted = 0;
  int axis;

  if (ekf == NULL || !ekf->initialized || !(noise > 0.0f) ||
      !isfinite(noise) || !(gate > 0.0f))
    {
      return -1;
    }

  /* Measured in the NAV frame, not the body frame, and that is the whole
   * economy of it: a stationary vehicle has zero velocity in every frame, so
   * there is no rotation to apply and no attitude error to leak in. A
   * body-frame wheel-speed measurement would need both.
   */

  for (axis = 0; axis < 3; axis++)
    {
      int result;

      memset(h, 0, sizeof(h));
      h[3 + axis] = 1.0f;

      /* Velocity and accelerometer bias only.
       *
       * Bias is the prize: a standstill is the one regime where it is
       * cleanly separable from tilt, which is why the low-dynamics update
       * goes after it too.
       *
       * Attitude is deliberately NOT permitted. Gravity is a better tilt
       * measurement than one inferred from velocity, the low-dynamics
       * update already takes it at exactly this moment, and stationary
       * wheels do not always mean a stationary vehicle - a skid, a tow, or
       * a jacked-up axle would otherwise put that lie straight into roll
       * and pitch. Position is not permitted either: this says the vehicle
       * is not moving, not where it is.
       */

      result = measurement_update_1d(ekf, h, -ekf->velocity[axis],
                                     noise * noise, gate,
                                     &ekf->last_zupt_nis[axis],
                                     EKF_GAIN_VELOCITY_XY |
                                     EKF_GAIN_VELOCITY_Z |
                                     EKF_GAIN_ACCEL_BIAS,
                                     NULL);

      if (result < 0)
        {
          return -1;
        }

      if (result > 0)
        {
          accepted++;
        }
    }

  if (accepted == 0)
    {
      ekf->zupt_reject_count++;
      return 0;
    }

  ekf->zupt_accept_count++;
  ekf->last_zupt_timestamp = ekf->last_timestamp_sample;
  return 1;
}

int ekf_core_fuse_baro(FAR struct ekf_core_s *ekf, float pressure_hpa,
                       float noise, float gate_sigma)
{
  float h[EKF_STATE_DIM];
  float height;
  float residual;
  int result;

  if (ekf == NULL || !ekf->initialized || !isfinite(pressure_hpa) ||
      pressure_hpa < EKF_BARO_PRESSURE_MIN ||
      pressure_hpa > EKF_BARO_PRESSURE_MAX ||
      !isfinite(noise) || noise <= 0.0f)
    {
      return -1;
    }

  /* The first good sample defines where zero is. Correcting against a
   * reference that does not exist yet would inject the whole altitude of the
   * site as an error.
   */

  if (!ekf->baro_have_reference)
    {
      ekf->baro_reference_hpa = pressure_hpa;
      ekf->baro_have_reference = true;
      ekf->last_baro_height = 0.0f;
      return -2;
    }

  height = ekf_baro_height(pressure_hpa, ekf->baro_reference_hpa);
  ekf->last_baro_height = height;

  /* Both are UP-positive, so the residual is a plain difference.
   *
   * The navigation frame's z is UP, not down. At rest a level +x fwd, +y
   * left, +z up accelerometer reads +g on z, and nominal_predict removes
   * gravity as `nav_delta_velocity[2] -= EKF_GRAVITY * dt`; those cancel to
   * zero only in a z-up frame. The gravity update agrees - it predicts the
   * specific force as +g along the nav z axis expressed in body.
   *
   * An earlier version of this negated the height, on the strength of the
   * word "NED" in a comment rather than the arithmetic above. A stationary
   * bench test cannot tell the two apart, because both leave the innovation
   * at zero.
   */

  memset(h, 0, sizeof(h));
  h[8] = 1.0f;
  residual = height - ekf->position[2];

  result = measurement_update_1d(ekf, h, residual, noise * noise,
                                 gate_sigma, &ekf->last_baro_nis,
                                 EKF_GAIN_HEIGHT, NULL);

  if (result == 1)
    {
      ekf->baro_accept_count++;
      ekf->baro_consecutive_rejects = 0;
    }
  else if (result == 0)
    {
      ekf->baro_reject_count++;
      ekf->baro_consecutive_rejects++;
    }

  return result;
}

uint8_t ekf_core_solution_status(FAR const struct ekf_core_s *ekf)
{
  uint8_t status;

  if (ekf == NULL || !ekf->initialized)
    {
      return 0;
    }

  /* Roll and pitch come from gravity, which is always available. Heading is
   * absolute only while a magnetometer is actually holding it there: it
   * needs a datum from alignment, an accepted update, no sustained rejection
   * run, and a measurement that is not stale. Any of those failing leaves a
   * heading that still exists and still integrates - just without north.
   */

  status = EKF_SOLUTION_ATTITUDE;

  if (ekf->yaw_absolute &&
      ekf->mag_consecutive_rejects < EKF_MAG_REJECT_RUN_MAX &&
      ekf->last_mag_timestamp != 0 &&
      ekf->last_timestamp_sample <=
        ekf->last_mag_timestamp + EKF_MAG_MAX_AGE_US)
    {
      status |= EKF_SOLUTION_YAW_ABSOLUTE;
    }
  else
    {
      status |= EKF_SOLUTION_YAW_RELATIVE;
    }

  /* Vertical validity is a claim about the barometer actually correcting,
   * not about it being selected. A sustained rejection run withdraws it
   * while leaving attitude alone.
   */

  if (ekf->baro_have_reference && ekf->baro_accept_count > 0 &&
      ekf->baro_consecutive_rejects < EKF_BARO_REJECT_RUN_MAX)
    {
      status |= EKF_SOLUTION_POSITION_VERT | EKF_SOLUTION_VELOCITY_VERT;
    }

  /* Horizontal validity is a claim about external navigation actually
   * CORRECTING, not about it being selected. A sustained rejection run
   * withdraws it, and so does silence: a source that simply stopped talking
   * leaves no rejections behind, so without the age check the claim would
   * stand for ever on a dead link.
   */

  if (ekf_core_position_aided(ekf))
    {
      status |= EKF_SOLUTION_POSITION_HORIZ | EKF_SOLUTION_VELOCITY_HORIZ;
    }

  return status;
}

void ekf_core_output_predict(FAR const struct ekf_core_s *ekf,
                             FAR const struct ekf_imu_sample_s *const *samples,
                             uint16_t count,
                             FAR struct ekf_output_s *out)
{
  uint16_t index;

  if (ekf == NULL || out == NULL)
    {
      return;
    }

  memset(out, 0, sizeof(*out));
  memcpy(out->quaternion, ekf->quaternion, sizeof(out->quaternion));
  memcpy(out->velocity, ekf->velocity, sizeof(out->velocity));
  memcpy(out->position, ekf->position, sizeof(out->position));
  out->timestamp_sample = ekf->last_timestamp_sample;
  out->valid = ekf->initialized;

  if (!ekf->initialized || samples == NULL)
    {
      return;
    }

  for (index = 0; index < count; index++)
    {
      FAR const struct ekf_imu_sample_s *sample = samples[index];

      if (sample == NULL)
        {
          break;
        }

      /* The bias estimates are the filter's, held fixed across the replay.
       * They change on the scale of minutes; the replay spans milliseconds.
       */

      /* Replay the same process model as the core so the delayed state and
       * published state remain dynamically consistent.
       */

      if (!strapdown_step(out->quaternion, out->velocity, out->position,
                          ekf->gyro_bias, ekf->accel_bias, sample))
        {
          out->valid = false;
          return;
        }

      out->timestamp_sample = sample->timestamp_sample;
      out->samples_replayed++;
    }
}

#ifdef EKF_CORE_HOST_TEST

void constrain_position_for_test(FAR struct ekf_core_s *ekf)
{
  constrain_position(ekf);
}

int ekf_core_test_update_1d(FAR struct ekf_core_s *ekf,
                            FAR const float h[EKF_STATE_DIM],
                            float residual, float noise_variance,
                            float gate_sigma, FAR float *nis)
{
  return measurement_update_1d(ekf, h, residual, noise_variance,
                               gate_sigma, nis, EKF_GAIN_ALL, NULL);
}

int ekf_core_test_update_3d(FAR struct ekf_core_s *ekf,
                            FAR const float h[3][EKF_STATE_DIM],
                            FAR const float residual[3],
                            float noise_variance, FAR float *nis)
{
  return measurement_update_3d(ekf, h, residual, noise_variance, nis,
                               NULL, NULL, EKF_GAIN_ALL);
}

#endif

void ekf_core_set_extnav_config(FAR struct ekf_core_s *ekf,
                                uint32_t timeout_us)
{
  if (ekf != NULL)
    {
      ekf->extnav_timeout_us = timeout_us;
    }
}

static void extnav_set_datum(FAR struct ekf_core_s *ekf,
                             FAR const struct ekf_extnav_sample_s *s,
                             float pos_noise, float yaw_noise,
                             bool want_position, bool want_yaw)
{
  if (want_position)
    {
      ekf->position[0] = s->x;
      ekf->position[1] = s->y;
      covariance_reset_position_xy(ekf, pos_noise * pos_noise);

      ekf->extnav_datum_set = true;
      ekf->extnav_datum_count++;
      ekf->extnav_consecutive_rejects = 0;
      ekf->last_extnav_noise = pos_noise;
      ekf->last_extnav_timestamp = ekf->last_timestamp_sample;
      ekf->last_extnav_rx_timestamp = ekf->last_timestamp_sample;

      /* A datum is the strongest possible accepted position update. */

      ekf->extnav_accept_count++;
    }

  if (want_yaw)
    {
      reset_yaw_absolute(ekf, s->yaw, yaw_noise * yaw_noise);
      ekf->yaw_absolute = true;
    }
}

/* Has the source gone quiet, as opposed to gone wrong?
 *
 * Only silence earns a re-datum. This is measured from the last received
 * horizontal pose, so a source that keeps sending poses the gate refuses
 * never looks like it disconnected and cannot silently acquire a new datum.
 */

static bool extnav_dropped_out(FAR const struct ekf_core_s *ekf)
{
  uint64_t age;

  if (ekf->last_extnav_rx_timestamp == 0 || ekf->extnav_timeout_us == 0)
    {
      return false;
    }

  if (ekf->last_timestamp_sample <= ekf->last_extnav_rx_timestamp)
    {
      return false;
    }

  age = ekf->last_timestamp_sample - ekf->last_extnav_rx_timestamp;
  return age > (uint64_t)ekf->extnav_timeout_us;
}

/* Track how far the source and the strapdown disagree, over time.
 *
 * The gate already refuses individual poses. This is the separate question
 * ArduPilot answers with a low-passed posTestRatio: whether the source is
 * wrong as a matter of course. The IMU is taken as the reference because it
 * is redundant at board level and its errors are bounded by calibration,
 * while an external source can be arbitrarily wrong and still look
 * plausible.
 */

static void extnav_track_ratio(FAR struct ekf_core_s *ekf, float ratio,
                               uint64_t now)
{
  if (!isfinite(ratio))
    {
      return;
    }

  ekf->extnav_test_ratio += EKF_EXTNAV_RATIO_ALPHA *
                            (ratio - ekf->extnav_test_ratio);

  if (ekf->extnav_test_ratio <= EKF_EXTNAV_RATIO_FAULT)
    {
      ekf->extnav_fault_since = 0;

      if (!ekf->extnav_healthy)
        {
          /* Recovering is allowed, but only by agreeing again for a while -
           * the filtered ratio cannot fall below the threshold quickly.
           */

          ekf->extnav_healthy = true;
        }

      return;
    }

  if (ekf->extnav_fault_since == 0)
    {
      ekf->extnav_fault_since = now;
      return;
    }

  if (now > ekf->extnav_fault_since &&
      now - ekf->extnav_fault_since > (uint64_t)ekf->extnav_timeout_us &&
      ekf->extnav_healthy)
    {
      ekf->extnav_healthy = false;
      ekf->extnav_fault_count++;
    }
}

int ekf_core_fuse_extnav(FAR struct ekf_core_s *ekf,
                         FAR const struct ekf_extnav_sample_s *s,
                         float pos_noise_floor, float pos_gate,
                         float yaw_noise_floor, float yaw_gate,
                         bool want_position, bool want_yaw)
{
  float pos_noise;
  float yaw_noise;
  float h[EKF_STATE_DIM];
  int accepted = 0;
  int gated = 0;
  bool position_accepted = false;
  bool position_gated = false;
  int axis;

  if (ekf == NULL || s == NULL || !ekf->initialized || !s->valid ||
      (!want_position && !want_yaw) ||
      (want_position &&
       (!isfinite(s->x) || !isfinite(s->y) ||
        !isfinite(pos_noise_floor) || pos_noise_floor <= 0.0f ||
        !isfinite(pos_gate) || pos_gate <= 0.0f)) ||
      (want_yaw &&
       (!isfinite(s->yaw) || !isfinite(yaw_noise_floor) ||
        yaw_noise_floor <= 0.0f || !isfinite(yaw_gate) || yaw_gate <= 0.0f)))
    {
      return -1;
    }

  /* The parameter is a FLOOR under whatever the source reported, not a
   * default. ArduPilot does the same with posErr: a source claiming
   * millimetre accuracy must not talk the filter into trusting it more than
   * the operator configured.
   */

  pos_noise = want_position ? pos_noise_floor : 1.0f;

  for (axis = 0; want_position && axis < 2; axis++)
    {
      if (isfinite(s->pos_sigma[axis]) && s->pos_sigma[axis] > pos_noise)
        {
          pos_noise = s->pos_sigma[axis];
        }
    }

  yaw_noise = want_yaw && isfinite(s->yaw_sigma) &&
              s->yaw_sigma > yaw_noise_floor ?
              s->yaw_sigma : (want_yaw ? yaw_noise_floor : 1.0f);

  /* The source relocalised. That is worth more than twenty gated
   * innovations telling us the same thing more slowly.
   */

  if (!ekf->have_extnav_reset)
    {
      ekf->extnav_source_reset = s->reset_counter;
      ekf->have_extnav_reset = true;
    }
  else if (s->reset_counter != ekf->extnav_source_reset)
    {
      ekf->extnav_source_reset = s->reset_counter;
      extnav_set_datum(ekf, s, pos_noise, yaw_noise, want_position,
                       want_yaw);
      return -2;
    }

  /* A datum is granted for two reasons only: there has never been one, or
   * the source went AWAY and came back. Never because it disagrees.
   *
   * The difference is everything. A source that stopped and restarted may
   * legitimately have a new origin. A source that never stopped talking and
   * is consistently wrong is simply wrong, and re-datuming to it hands the
   * filter its error as truth - which is how a companion publishing a frozen
   * position drove accel bias to its limit and took attitude with it.
   */

  {
    /* Stamped BEFORE the datum decision, and read after, so this call's own
     * arrival cannot make itself look like a dropout.
     */

    bool dropped = want_position && extnav_dropped_out(ekf);
    bool need_position_datum = want_position &&
      (!ekf->extnav_datum_set || dropped || ekf->position_holding);
    bool need_yaw_datum = want_yaw && !ekf->yaw_absolute;

    if (want_position)
      {
        ekf->last_extnav_rx_timestamp = ekf->last_timestamp_sample;
      }

    /* A datum is granted when there has never been one, when the source
     * went away and came back - and now also whenever position has been
     * HELD, because a held position is not an estimate the source can be
     * asked to agree with.
     *
     * That last case reverses an earlier rule, and only became safe once
     * the hold existed. Re-datuming on disagreement used to be dangerous
     * because the strapdown went on integrating real motion while the
     * source insisted otherwise, and the only way to reconcile the two was
     * accelerometer bias. With position bounded and that bias frozen while
     * unaided, there is nothing left to reconcile: the filter has no
     * competing opinion about where it is, so the source's is simply
     * adopted.
     *
     * And without it the filter would lock out. A condemned source has to
     * agree again to recover, but a position bounded around an old fix can be
     * far from a returning source - so the fix could be refused forever.
     */

    if (need_position_datum || need_yaw_datum)
      {
        if (need_position_datum && ekf->position_holding)
          {
            ekf->position_snap_count++;
          }

        extnav_set_datum(ekf, s, pos_noise, yaw_noise,
                         need_position_datum, need_yaw_datum ||
                         (need_position_datum && want_yaw));

        if (need_position_datum)
          {
            ekf->extnav_test_ratio = 0.0f;
            ekf->extnav_fault_since = 0;
            ekf->extnav_healthy = true;
            ekf->position_holding = false;
            ekf->inhibit_mask = 0;
          }

        return -2;
      }
  }

  if (want_position)
    {
      ekf->last_extnav_noise = pos_noise;
    }

  /* East and north are gated TOGETHER and fused together, or neither.
   *
   * Gating them independently lets one axis mask the other: a pose 500 m
   * wrong in x but unchanged in y has y accepted, so the pose as a whole
   * reports accepted, the rejection run never accumulates and the re-datum
   * that recovers from a dropout never fires.
   *
   * ArduPilot forms one posTestRatio from both horizontal axes for exactly
   * this reason. The fusion itself stays sequential and scalar, ignoring the
   * measurement cross-covariance, which is also what FuseVelPosNED does.
   */

  if (want_position)
    {
      const float measured[2] = {s->x, s->y};
      float innovation_sq = 0.0f;
      float variance_sum = 0.0f;
      float ratio;

      for (axis = 0; axis < 2; axis++)
        {
          float innovation = measured[axis] - ekf->position[axis];
          float variance = ekf->covariance[EKF_P_INDEX(6 + axis,
                                                       6 + axis)] +
                           pos_noise * pos_noise;

          ekf->last_extnav_innov[axis] = innovation;
          innovation_sq += innovation * innovation;
          variance_sum += variance;
          ekf->last_extnav_nis[axis] = variance > 0.0f ?
                                       innovation * innovation / variance :
                                       0.0f;
        }

      if (!isfinite(innovation_sq) || !isfinite(variance_sum) ||
          variance_sum <= 0.0f)
        {
          return -1;
        }

      ratio = innovation_sq / (pos_gate * pos_gate * variance_sum);
      extnav_track_ratio(ekf, ratio, ekf->last_timestamp_sample);

      /* Assert the accelerometer-bias inhibit while the source is arguing
       * with the IMU.
       *
       * Horizontal-position fusion already excludes every bias gain row.
       * This inhibit is a second line of defence and retained as an explicit
       * diagnostic for persistent source disagreement.
       */

      ekf->inhibit_mask = 0;

      if (ekf->extnav_test_ratio > EKF_EXTNAV_INHIBIT_RATIO ||
          !ekf->extnav_healthy)
        {
          ekf->inhibit_mask = EKF_INHIBIT_ACCEL_BIAS;

          if (!ekf->extnav_bias_inhibited)
            {
              ekf->extnav_bias_inhibited = true;
              ekf->extnav_inhibit_count++;
            }
        }
      else
        {
          ekf->extnav_bias_inhibited = false;
        }

      /* An unhealthy source is not fused at all. It still updates the ratio
       * above, which is how it earns its way back.
       */

      if (ratio >= 1.0f || !ekf->extnav_healthy)
        {
          gated++;
          position_gated = true;
        }
      else
        {
          for (axis = 0; axis < 2; axis++)
            {
              int result;

              memset(h, 0, sizeof(h));
              h[6 + axis] = 1.0f;

              /* The joint gate above is authoritative, so the scalar update
               * is left permissive - a second, per-axis gate here could
               * reject half of a pose the combined test just accepted.
               */

              result = measurement_update_1d(ekf, h,
                                             measured[axis] -
                                             ekf->position[axis],
                                             pos_noise * pos_noise, 1.0e6f,
                                             &ekf->last_extnav_nis[axis],
                                             EKF_GAIN_HORIZONTAL_POSITION,
                                             NULL);

              if (result < 0)
                {
                  return -1;
                }

              accepted++;
            }

          position_accepted = true;
        }
    }

  ekf->inhibit_mask = 0;

  if (want_yaw)
    {
      float nis = 0.0f;
      int result;

      /* Yaw from a source the position check has condemned is not
       * trustworthy either - they come from the same estimate.
       */

      if (!ekf->extnav_healthy)
        {
          gated++;
        }
      else
        {
          result = fuse_yaw(ekf, s->yaw, yaw_noise, yaw_gate, &nis);

          if (result < 0)
            {
              return -1;
            }

          if (result == 0)
            {
              gated++;
            }
          else
            {
              accepted++;
              ekf->yaw_absolute = true;
            }
        }
    }

  /* Position health is updated only by position. A simultaneously accepted
   * yaw must not clear a horizontal rejection run or keep horizontal aiding
   * alive after x/y stopped passing the gate.
   */

  if (position_accepted)
    {
      ekf->extnav_accept_count++;
      ekf->extnav_consecutive_rejects = 0;
      ekf->last_extnav_timestamp = ekf->last_timestamp_sample;
    }
  else if (position_gated)
    {
      ekf->extnav_reject_count++;
      ekf->extnav_consecutive_rejects++;
    }

  if (accepted > 0)
    {
      return 1;
    }

  if (gated > 0)
    {
      return 0;
    }

  return -1;
}
