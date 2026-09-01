/****************************************************************************
 * apps/companion/comp_state.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <math.h>
#include <string.h>

#include "comp_state.h"

/* Body-to-nav rotation from a quaternion.
 *
 * This mirrors quaternion_to_rotation() in apps/ekf3/ekf_core.c, which is
 * static there. It is repeated rather than shared because linking ekf_core.c
 * into the companion would pull the entire filter in for twenty lines of
 * algebra. The host test pins it against rotations whose answers are known
 * independently, so the two cannot drift into disagreeing silently.
 */

static void comp_quat_rotation(FAR const float q[4], FAR float r[3][3])
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

void comp_state_enu_to_body(FAR const float q[4], FAR const float v[3],
                            FAR float out[3])
{
  float r[3][3];
  int i;

  comp_quat_rotation(q, r);

  /* The TRANSPOSE: r maps body to nav, and this wants nav to body. Applying
   * r itself here is the classic error, and it survives every test done at
   * zero yaw because the matrix is the identity there.
   */

  for (i = 0; i < 3; i++)
    {
      out[i] = r[0][i] * v[0] + r[1][i] * v[1] + r[2][i] * v[2];
    }
}

void comp_state_remove_gravity(FAR const float q[4], FAR const float accel[3],
                               FAR float out[3])
{
  float r[3][3];
  int i;

  comp_quat_rotation(q, r);

  /* An accelerometer measures SPECIFIC FORCE, so at rest it reads +g along
   * the nav frame's up axis rather than zero. Gravity in body coordinates is
   * the transpose applied to (0, 0, g), which is the third row of r scaled -
   * subtracting it leaves zero at rest.
   */

  for (i = 0; i < 3; i++)
    {
      out[i] = accel[i] - r[2][i] * COMP_STATE_GRAVITY;
    }
}

void comp_state_build(FAR const struct comp_state_inputs_s *in,
                      uint64_t timestamp_us,
                      FAR struct comp_vehicle_state_s *out)
{
  float corrected_accel[3];
  float speed;
  int axis;

  if (in == NULL || out == NULL)
    {
      return;
    }

  memset(out, 0, sizeof(*out));
  out->timestamp_us = timestamp_us;

  /* NaN until there is a velocity to measure it against. Not zero: zero is a
   * perfectly good slip angle - "travelling straight ahead" - and a consumer
   * could not tell the two apart.
   */

  out->side_slip_rad = NAN;

  if (in->est_valid)
    {
      memcpy(out->position, in->position, sizeof(out->position));
      memcpy(out->quaternion, in->quaternion, sizeof(out->quaternion));
      comp_state_enu_to_body(in->quaternion, in->velocity_enu,
                             out->velocity);

      /* Side slip: the angle between where the vehicle is GOING and where it
       * is POINTING. The body velocity is already exactly that comparison -
       * body x is the heading and body y is left - so it is atan2 of the two
       * and needs no separate yaw.
       *
       * Horizontal speed only. A climb or descent is not side slip.
       *
       * Left-positive, matching body y and ISO 8855: sliding to the left of
       * the nose gives a positive angle.
       */

      speed = sqrtf(out->velocity[0] * out->velocity[0] +
                    out->velocity[1] * out->velocity[1]);

      if (speed >= COMP_SLIP_MIN_SPEED)
        {
          out->side_slip_rad = atan2f(out->velocity[1], out->velocity[0]);
        }

      out->solution_status = in->solution_status;
      out->reset_counter = in->reset_counter;
      out->source_valid |= COMP_SRC_ESTIMATOR;
    }
  else
    {
      /* An identity quaternion is a lie a consumer cannot detect, so leave
       * the whole pose zeroed and say so in source_valid instead. A
       * zero quaternion is not a rotation and reads as obviously absent.
       */

      out->source_valid &= (uint8_t)~COMP_SRC_ESTIMATOR;
    }

  if (in->gyro_valid)
    {
      memcpy(out->angular_velocity, in->gyro, sizeof(out->angular_velocity));
      out->source_valid |= COMP_SRC_GYRO;
    }

  /* Gravity removal needs an attitude. Without the estimator there is no way
   * to know which way is down, and subtracting nothing would report the
   * whole 9.8 m/s^2 as vehicle acceleration.
   */

  if (in->accel_valid && in->est_valid)
    {
      /* estimator_state.accel_bias uses the same body-frame sign convention
       * as strapdown propagation: corrected specific force is measured minus
       * bias. Gravity removal alone leaves that modeled bias in a field named
       * linear acceleration, producing a non-zero stationary output even
       * while the EKF itself has correctly learned the residual.
       */

      for (axis = 0; axis < 3; axis++)
        {
          corrected_accel[axis] = in->accel[axis] - in->accel_bias[axis];
        }

      comp_state_remove_gravity(in->quaternion, corrected_accel, out->accel);
      out->source_valid |= COMP_SRC_ACCEL;
    }

  if (in->vesc_valid)
    {
      out->wheel_torque_nm = in->current_a * in->torque_k;
      out->steering_angle = in->adc_volts * in->steer_k;
      out->motor_speed_ms = in->motor_counts_per_s * in->speed_k;
      out->source_valid |= COMP_SRC_VESC;
    }

  if (in->rc_valid)
    {
      out->rc_status =
        (((uint32_t)in->rc_steering_pwm & COMP_RC_PWM_MASK) <<
         COMP_RC_STEER_SHIFT) |
        (((uint32_t)in->rc_throttle_pwm & COMP_RC_PWM_MASK) <<
         COMP_RC_THROTTLE_SHIFT);
      out->source_valid |= COMP_SRC_RC;
    }

  if (in->control_armed)
    {
      out->rc_status |= COMP_RC_ARMED;
    }

  if (in->control_auto)
    {
      out->rc_status |= COMP_RC_AUTO;
    }

  if (in->trigger_high)
    {
      out->rc_status |= COMP_RC_TRIGGER_HIGH;
    }

  if (in->control_current)
    {
      out->rc_status |= COMP_RC_CURRENT;
    }
}
