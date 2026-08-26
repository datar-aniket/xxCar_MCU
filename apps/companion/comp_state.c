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

void comp_speed_init(FAR struct comp_speed_filter_s *f, float cutoff_hz)
{
  if (f == NULL)
    {
      return;
    }

  memset(f, 0, sizeof(*f));

  /* Reject a nonsense cutoff rather than build a filter from it. Zero is a
   * legitimate request meaning "no filtering".
   */

  f->cutoff_hz = (cutoff_hz > 0.0f && isfinite(cutoff_hz)) ? cutoff_hz : 0.0f;
}

void comp_speed_reset(FAR struct comp_speed_filter_s *f)
{
  float cutoff;

  if (f == NULL)
    {
      return;
    }

  cutoff = f->cutoff_hz;
  memset(f, 0, sizeof(*f));
  f->cutoff_hz = cutoff;
}

float comp_speed_update(FAR struct comp_speed_filter_s *f,
                        int32_t tachometer, uint64_t timestamp_us)
{
  int32_t delta;
  float dt;
  float fs;
  float cutoff;
  float tau;
  float alpha;
  float rate;

  if (f == NULL)
    {
      return 0.0f;
    }

  /* The first reading establishes a reference and nothing else. Emitting a
   * rate from it would divide the whole accumulated count by the time since
   * boot.
   *
   * The same path handles a gap so long the count can no longer be related
   * to the previous one - the VESC rebooted, the bus dropped - where
   * carrying on would emit a single enormous spike.
   */

  if (!f->primed || timestamp_us <= f->last_us ||
      timestamp_us - f->last_us > COMP_SPEED_MAX_GAP_US)
    {
      f->primed = true;
      f->last_tach = tachometer;
      f->last_us = timestamp_us;
      f->stage[0] = 0.0f;
      f->stage[1] = 0.0f;
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
  fs = 1.0f / dt;

  /* Observed arrival rate, lightly smoothed. Reported so that a cutoff which
   * was silently clamped below is visible rather than mysterious.
   */

  f->rate_hz += 0.1f * (fs - f->rate_hz);

  f->last_tach = tachometer;
  f->last_us = timestamp_us;

  if (f->cutoff_hz <= 0.0f)
    {
      f->stage[0] = rate;
      f->stage[1] = rate;
      f->value = rate;
      return f->value;
    }

  /* Clamp to a fraction of the ACTUAL interval, not a nominal rate. A cutoff
   * at or above Nyquist is not a filter; this makes asking for one harmless
   * instead of letting alpha run past 1 and the section oscillate.
   */

  cutoff = f->cutoff_hz;

  if (cutoff > COMP_SPEED_MAX_FS_FRACTION * fs)
    {
      cutoff = COMP_SPEED_MAX_FS_FRACTION * fs;
    }

  tau = 1.0f / (2.0f * 3.14159265358979323846f * cutoff);
  alpha = dt / (tau + dt);

  /* Two one-pole sections in series: -40 dB/decade. Each recomputes alpha
   * from the interval just measured, so jitter in the arrival rate changes
   * the coefficient rather than the cutoff.
   */

  f->stage[0] += alpha * (rate - f->stage[0]);
  f->stage[1] += alpha * (f->stage[0] - f->stage[1]);
  f->value = f->stage[1];
  return f->value;
}

void comp_state_build(FAR const struct comp_state_inputs_s *in,
                      uint64_t timestamp_us,
                      FAR struct comp_vehicle_state_s *out)
{
  float speed;

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
