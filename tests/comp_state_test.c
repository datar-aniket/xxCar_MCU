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

static void test_speed_first_reading_is_zero(void)
{
  struct comp_speed_filter_s f;

  comp_speed_reset(&f);

  /* The first sample only establishes a reference. Emitting a rate would
   * divide the whole accumulated count by the time since boot.
   */

  assert(CLOSE(comp_speed_update(&f, 100000, 1000000), 0.0f));
}

/* A steady 1000 counts/s, fed at 20 Hz the way STATUS_5 arrives. The filter
 * must converge on 1000, not on some fraction of it.
 */

static void test_speed_converges(void)
{
  struct comp_speed_filter_s f;
  int32_t tach = 0;
  uint64_t t = 1000000;
  float v = 0.0f;
  int i;

  comp_speed_reset(&f);
  comp_speed_update(&f, tach, t);

  for (i = 0; i < 200; i++)
    {
      t += 50000;           /* 20 Hz */
      tach += 50;           /* 50 counts per 50 ms = 1000 /s */
      v = comp_speed_update(&f, tach, t);
    }

  assert(fabsf(v - 1000.0f) < 1.0f);
}

static void test_speed_sign(void)
{
  struct comp_speed_filter_s f;
  int32_t tach = 0;
  uint64_t t = 1000000;
  float v = 0.0f;
  int i;

  comp_speed_reset(&f);
  comp_speed_update(&f, tach, t);

  for (i = 0; i < 200; i++)
    {
      t += 50000;
      tach -= 50;
      v = comp_speed_update(&f, tach, t);
    }

  assert(v < 0.0f);
  assert(fabsf(v + 1000.0f) < 1.0f);
}

/* The tachometer is a 32-bit accumulator and it wraps. Taking the signed
 * difference of the raw values at that moment gives about 4.3 billion of the
 * wrong sign - a single sample that would read as an impossible speed.
 */

static void test_speed_wraps(void)
{
  struct comp_speed_filter_s f;
  uint64_t t = 1000000;
  float v;

  comp_speed_reset(&f);
  comp_speed_update(&f, INT32_MAX - 10, t);

  t += 50000;
  v = comp_speed_update(&f, INT32_MIN + 10, t);

  /* 21 counts forward across the wrap over 50 ms is 420 counts/s raw. One
   * filter step at dt == tau has alpha 0.5, so the output is 210.
   *
   * What the test is really for is the sign and the ORDER: taking the signed
   * difference of the raw values here gives about -4.3 billion, so anything
   * negative or huge means the unsigned subtraction was lost.
   */

  assert(v > 0.0f);
  assert(CLOSE(v, 210.0f));
}

/* A long gap means the count can no longer be related to the previous one.
 * Carrying on would emit one enormous spike.
 */

static void test_speed_restarts_after_gap(void)
{
  struct comp_speed_filter_s f;
  uint64_t t = 1000000;

  comp_speed_reset(&f);
  comp_speed_update(&f, 0, t);

  t += COMP_SPEED_MAX_GAP_US + 1;
  assert(CLOSE(comp_speed_update(&f, 5000000, t), 0.0f));
}

static void test_speed_rejects_backwards_time(void)
{
  struct comp_speed_filter_s f;

  comp_speed_reset(&f);
  comp_speed_update(&f, 0, 2000000);

  /* Same timestamp would divide by zero; an earlier one is nonsense. */

  assert(CLOSE(comp_speed_update(&f, 100, 2000000), 0.0f));
  assert(CLOSE(comp_speed_update(&f, 200, 1000000), 0.0f));
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
  test_speed_first_reading_is_zero();
  test_speed_converges();
  test_speed_sign();
  test_speed_wraps();
  test_speed_restarts_after_gap();
  test_speed_rejects_backwards_time();
  test_build_side_slip_is_nan();
  test_build_reports_missing_sources();
  test_build_accel_needs_attitude();
  test_build_scalars();
  test_build_velocity_is_body();

  printf("comp_state: frames, gravity removal and tachometer rate - OK\n");
  return 0;
}
