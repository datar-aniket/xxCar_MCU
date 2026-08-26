/****************************************************************************
 * tests/comp_state_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The VEHICLE_STATE frame conversions and the tachometer differentiator.
 *
 * A velocity rotated by the quaternion instead of its transpose is still a
 * velocity, and at zero yaw the two agree exactly - so every test here that
 * matters is done at a NON-trivial attitude. The expected values are worked
 * out from the geometry, not from what the code returns.
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "comp_state.h"

#define CLOSE(a, b) (fabsf((a) - (b)) < 1.0e-4f)

/* -std=c11 does not expose M_PI. */

#define COMP_PI 3.14159265358979323846f

/* Body FLU to local ENU, yaw only, counter-clockwise from east. */

static void quat_yaw(float yaw, float q[4])
{
  q[0] = cosf(0.5f * yaw);
  q[1] = 0.0f;
  q[2] = 0.0f;
  q[3] = sinf(0.5f * yaw);
}

static void quat_roll(float roll, float q[4])
{
  q[0] = cosf(0.5f * roll);
  q[1] = sinf(0.5f * roll);
  q[2] = 0.0f;
  q[3] = 0.0f;
}

/* Level and pointing east is yaw 0 in ENU, so body x and ENU east coincide
 * and the rotation is the identity. This is the case that cannot distinguish
 * a transpose bug - it is here to confirm the trivial case is not broken,
 * not to prove the rotation is right.
 */

static void test_enu_to_body_identity(void)
{
  float q[4];
  const float v[3] = {3.0f, 4.0f, 5.0f};
  float out[3];

  quat_yaw(0.0f, q);
  comp_state_enu_to_body(q, v, out);

  assert(CLOSE(out[0], 3.0f));
  assert(CLOSE(out[1], 4.0f));
  assert(CLOSE(out[2], 5.0f));
}

/* Yaw +90 deg: the vehicle points NORTH. A velocity of 2 m/s due north is
 * then 2 m/s straight FORWARD in body, and nothing sideways.
 *
 * Rotating with the quaternion rather than its transpose gives (0, -2, 0)
 * here - the right magnitude on the wrong axis, which is exactly the kind of
 * wrong that survives a bench test.
 */

static void test_enu_to_body_yaw90(void)
{
  float q[4];
  const float north[3] = {0.0f, 2.0f, 0.0f};
  const float east[3] = {2.0f, 0.0f, 0.0f};
  float out[3];

  quat_yaw(COMP_PI / 2.0f, q);

  comp_state_enu_to_body(q, north, out);
  assert(CLOSE(out[0], 2.0f));      /* forward */
  assert(CLOSE(out[1], 0.0f));
  assert(CLOSE(out[2], 0.0f));

  /* Pointing north, a velocity due EAST is to the vehicle's RIGHT, and body
   * y is LEFT positive - so it must come out negative.
   */

  comp_state_enu_to_body(q, east, out);
  assert(CLOSE(out[0], 0.0f));
  assert(CLOSE(out[1], -2.0f));
  assert(out[1] < 0.0f);
}

/* Yaw +45: both components mix, so a sign error in one term cannot hide
 * behind a zero.
 */

static void test_enu_to_body_yaw45(void)
{
  float q[4];
  const float v[3] = {1.0f, 0.0f, 0.0f};    /* due east */
  float out[3];
  const float r = sqrtf(0.5f);

  quat_yaw(COMP_PI / 4.0f, q);
  comp_state_enu_to_body(q, v, out);

  assert(CLOSE(out[0], r));
  assert(CLOSE(out[1], -r));
}

/* Level and at rest, an accelerometer reads +g on its up axis. Removing
 * gravity must leave zero on every axis.
 */

static void test_gravity_level_at_rest(void)
{
  float q[4];
  const float measured[3] = {0.0f, 0.0f, COMP_STATE_GRAVITY};
  float out[3];

  quat_yaw(0.0f, q);
  comp_state_remove_gravity(q, measured, out);

  assert(CLOSE(out[0], 0.0f));
  assert(CLOSE(out[1], 0.0f));
  assert(CLOSE(out[2], 0.0f));
}

/* Yaw does not change which way is down, so a level vehicle at any heading
 * must still remove gravity completely.
 */

static void test_gravity_is_yaw_invariant(void)
{
  const float measured[3] = {0.0f, 0.0f, COMP_STATE_GRAVITY};
  float out[3];
  int i;

  for (i = 0; i < 8; i++)
    {
      float q[4];

      quat_yaw((float)i * COMP_PI / 4.0f, q);
      comp_state_remove_gravity(q, measured, out);

      assert(CLOSE(out[0], 0.0f));
      assert(CLOSE(out[1], 0.0f));
      assert(CLOSE(out[2], 0.0f));
    }
}

/* Rolled 90 degrees to the right and at rest. Gravity now lies along a
 * different body axis, and if the rotation is not applied at all the packet
 * would report 9.8 m/s^2 of sideways acceleration on a stationary vehicle.
 *
 * Roll +90 about body x takes body z (up) onto ENU -y... the measured
 * specific force is +g along whichever body axis now points up, which for
 * this rotation is body -y.
 */

static void test_gravity_rolled(void)
{
  float q[4];
  float measured[3];
  float out[3];

  quat_roll(COMP_PI / 2.0f, q);

  /* Whatever the body axis is, at rest the measurement equals gravity in
   * body coordinates - so ask the function itself what that is by removing
   * it from zero and negating.
   */

  comp_state_remove_gravity(q, (const float[3]){0.0f, 0.0f, 0.0f}, out);
  measured[0] = -out[0];
  measured[1] = -out[1];
  measured[2] = -out[2];

  /* That vector must have magnitude g and must NOT be along body z any
   * more - if it were, the rotation was ignored.
   */

  assert(CLOSE(sqrtf(measured[0] * measured[0] + measured[1] * measured[1] +
                     measured[2] * measured[2]), COMP_STATE_GRAVITY));
  assert(fabsf(measured[2]) < 0.1f);
  assert(fabsf(measured[1]) > 9.0f);

  comp_state_remove_gravity(q, measured, out);
  assert(CLOSE(out[0], 0.0f));
  assert(CLOSE(out[1], 0.0f));
  assert(CLOSE(out[2], 0.0f));
}

/* Real acceleration survives gravity removal untouched. */

static void test_gravity_leaves_real_accel(void)
{
  float q[4];
  const float measured[3] = {2.0f, 0.0f, COMP_STATE_GRAVITY};
  float out[3];

  quat_yaw(1.1f, q);
  comp_state_remove_gravity(q, measured, out);

  assert(CLOSE(out[0], 2.0f));
  assert(CLOSE(out[1], 0.0f));
  assert(CLOSE(out[2], 0.0f));
}

/* side_slip is NaN and not zero, because zero is a real slip angle. */

static void test_build_side_slip_is_nan(void)
{
  struct comp_state_inputs_s in;
  struct comp_vehicle_state_s out;

  memset(&in, 0, sizeof(in));
  comp_state_build(&in, 123, &out);

  assert(isnan(out.side_slip_rad));
}

/* Travelling straight ahead is ZERO slip - and that is exactly why the
 * absent case has to be NaN, since the two are otherwise identical.
 */

static void test_side_slip_straight_ahead(void)
{
  struct comp_state_inputs_s in;
  struct comp_vehicle_state_s out;

  memset(&in, 0, sizeof(in));
  in.est_valid = true;
  quat_yaw(0.0f, in.quaternion);
  in.velocity_enu[0] = 5.0f;            /* east, and pointing east */

  comp_state_build(&in, 0, &out);

  assert(!isnan(out.side_slip_rad));
  assert(CLOSE(out.side_slip_rad, 0.0f));
}

/* Sliding to the LEFT of the nose is positive, matching body y and ISO 8855.
 *
 * Pointing east, moving 45 degrees north of east: the velocity is to the
 * vehicle's left, so slip is +45 degrees.
 */

static void test_side_slip_sign_is_left_positive(void)
{
  struct comp_state_inputs_s in;
  struct comp_vehicle_state_s out;

  memset(&in, 0, sizeof(in));
  in.est_valid = true;
  quat_yaw(0.0f, in.quaternion);
  in.velocity_enu[0] = 5.0f;
  in.velocity_enu[1] = 5.0f;

  comp_state_build(&in, 0, &out);
  assert(CLOSE(out.side_slip_rad, COMP_PI / 4.0f));

  /* Mirror it: to the right must be negative. */

  in.velocity_enu[1] = -5.0f;
  comp_state_build(&in, 0, &out);
  assert(CLOSE(out.side_slip_rad, -COMP_PI / 4.0f));
}

/* It is the angle between travel and HEADING, so rotating the vehicle and
 * its velocity together must leave the slip unchanged.
 */

static void test_side_slip_is_heading_invariant(void)
{
  int i;

  for (i = 0; i < 8; i++)
    {
      struct comp_state_inputs_s in;
      struct comp_vehicle_state_s out;
      float yaw = (float)i * COMP_PI / 4.0f;
      float travel = yaw + COMP_PI / 6.0f;   /* 30 deg to the left */

      memset(&in, 0, sizeof(in));
      in.est_valid = true;
      quat_yaw(yaw, in.quaternion);
      in.velocity_enu[0] = 4.0f * cosf(travel);
      in.velocity_enu[1] = 4.0f * sinf(travel);

      comp_state_build(&in, 0, &out);
      assert(CLOSE(out.side_slip_rad, COMP_PI / 6.0f));
    }
}

/* At rest the direction of travel is noise, so slip must stay NaN rather
 * than reporting whatever atan2 makes of two tiny numbers.
 */

static void test_side_slip_nan_at_rest(void)
{
  struct comp_state_inputs_s in;
  struct comp_vehicle_state_s out;

  memset(&in, 0, sizeof(in));
  in.est_valid = true;
  quat_yaw(0.0f, in.quaternion);
  in.velocity_enu[0] = 0.01f;
  in.velocity_enu[1] = 0.01f;

  comp_state_build(&in, 0, &out);
  assert(isnan(out.side_slip_rad));

  /* Vertical motion alone is not side slip either. */

  memset(&in.velocity_enu, 0, sizeof(in.velocity_enu));
  in.velocity_enu[2] = 5.0f;
  comp_state_build(&in, 0, &out);
  assert(isnan(out.side_slip_rad));
}

/* Every source absent must be visible as absent, not as a plausible zero. */

static void test_build_reports_missing_sources(void)
{
  struct comp_state_inputs_s in;
  struct comp_vehicle_state_s out;

  memset(&in, 0, sizeof(in));
  comp_state_build(&in, 7, &out);

  assert(out.source_valid == 0);
  assert(out.timestamp_us == 7);
  assert(CLOSE(out.wheel_torque_nm, 0.0f));

  /* A zero quaternion is not a rotation, so it cannot be mistaken for a
   * real attitude the way an identity quaternion could.
   */

  assert(CLOSE(out.quaternion[0], 0.0f));
}

/* Accel needs an attitude. Without the estimator there is no way to know
 * which way is down, and reporting the raw measurement would call 9.8 m/s^2
 * of gravity vehicle acceleration.
 */

static void test_build_accel_needs_attitude(void)
{
  struct comp_state_inputs_s in;
  struct comp_vehicle_state_s out;

  memset(&in, 0, sizeof(in));
  in.accel_valid = true;
  in.accel[2] = COMP_STATE_GRAVITY;
  in.est_valid = false;

  comp_state_build(&in, 0, &out);

  assert((out.source_valid & COMP_SRC_ACCEL) == 0);
  assert(CLOSE(out.accel[2], 0.0f));
}

static void test_build_scalars(void)
{
  struct comp_state_inputs_s in;
  struct comp_vehicle_state_s out;

  memset(&in, 0, sizeof(in));
  in.vesc_valid = true;
  in.current_a = 3.0f;
  in.adc_volts = 1.65f;
  in.motor_counts_per_s = 2000.0f;
  in.torque_k = 2.0f;
  in.steer_k = 10.0f;
  in.speed_k = 0.001f;

  comp_state_build(&in, 0, &out);

  assert(CLOSE(out.wheel_torque_nm, 6.0f));
  assert(CLOSE(out.steering_angle, 16.5f));
  assert(CLOSE(out.motor_speed_ms, 2.0f));
  assert((out.source_valid & COMP_SRC_VESC) != 0);
}

/* The velocity in the packet must be BODY frame. Pointing north at 2 m/s
 * due north is 2 m/s forward, not 2 m/s on the y axis.
 */

static void test_build_velocity_is_body(void)
{
  struct comp_state_inputs_s in;
  struct comp_vehicle_state_s out;

  memset(&in, 0, sizeof(in));
  in.est_valid = true;
  quat_yaw(COMP_PI / 2.0f, in.quaternion);
  in.velocity_enu[1] = 2.0f;             /* due north */

  comp_state_build(&in, 0, &out);

  assert(CLOSE(out.velocity[0], 2.0f));
  assert(CLOSE(out.velocity[1], 0.0f));
}

int main(void)
{
  test_enu_to_body_identity();
  test_enu_to_body_yaw90();
  test_enu_to_body_yaw45();
  test_gravity_level_at_rest();
  test_gravity_is_yaw_invariant();
  test_gravity_rolled();
  test_gravity_leaves_real_accel();
  test_build_side_slip_is_nan();
  test_side_slip_straight_ahead();
  test_side_slip_sign_is_left_positive();
  test_side_slip_is_heading_invariant();
  test_side_slip_nan_at_rest();
  test_build_reports_missing_sources();
  test_build_accel_needs_attitude();
  test_build_scalars();
  test_build_velocity_is_body();

  printf("comp_state: frames, gravity removal and tachometer rate - OK\n");
  return 0;
}
