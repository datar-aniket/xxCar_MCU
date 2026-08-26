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

void comp_speed_reset(FAR struct comp_speed_filter_s *f)
{
  if (f != NULL)
    {
      memset(f, 0, sizeof(*f));
    }
}

float comp_speed_update(FAR struct comp_speed_filter_s *f,
                        int32_t tachometer, uint64_t timestamp_us)
{
  int32_t delta;
  float dt;
  float rate;
  float alpha;

  if (f == NULL)
    {
      return 0.0f;
    }

  /* The first reading establishes a reference and nothing else. Emitting a
   * rate from it would divide by the time since boot.
   */

  if (!f->primed || timestamp_us <= f->last_us ||
      timestamp_us - f->last_us > COMP_SPEED_MAX_GAP_US)
    {
      f->primed = true;
      f->last_tach = tachometer;
      f->last_us = timestamp_us;
      f->value = 0.0f;
      return 0.0f;
    }

  /* Unsigned subtraction, then read back as signed. The tachometer is a
   * 32-bit accumulator and it does wrap; the signed difference of the two
   * raw values would be enormous and the wrong sign at exactly that moment.
   */

  delta = (int32_t)((uint32_t)tachometer - (uint32_t)f->last_tach);

  dt = (float)(timestamp_us - f->last_us) * 1.0e-6f;
  rate = (float)delta / dt;

  /* First-order low pass. The count is an integer, so at a short dt the
   * quantisation alone is a large fraction of the reading.
   */

  alpha = dt / (COMP_SPEED_TAU_S + dt);
  f->value += alpha * (rate - f->value);

  f->last_tach = tachometer;
  f->last_us = timestamp_us;
  return f->value;
}

void comp_state_build(FAR const struct comp_state_inputs_s *in,
                      uint64_t timestamp_us,
                      FAR struct comp_vehicle_state_s *out)
{
  if (in == NULL || out == NULL)
    {
      return;
    }

  memset(out, 0, sizeof(*out));
  out->timestamp_us = timestamp_us;

  /* Not computed yet. NaN rather than zero, because zero is a perfectly good
   * slip angle - "travelling straight ahead" - and a consumer could not tell
   * the two apart.
   */

  out->side_slip_rad = NAN;

  if (in->est_valid)
    {
      memcpy(out->position, in->position, sizeof(out->position));
      memcpy(out->quaternion, in->quaternion, sizeof(out->quaternion));
      comp_state_enu_to_body(in->quaternion, in->velocity_enu,
                             out->velocity);
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
      comp_state_remove_gravity(in->quaternion, in->accel, out->accel);
      out->source_valid |= COMP_SRC_ACCEL;
    }

  if (in->vesc_valid)
    {
      out->wheel_torque_nm = in->current_a * in->torque_k;
      out->steering_angle = in->adc_volts * in->steer_k;
      out->motor_speed_ms = in->motor_counts_per_s * in->speed_k;
      out->source_valid |= COMP_SRC_VESC;
    }
}
