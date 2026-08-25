/****************************************************************************
 * tests/ekf_mag_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Magnetic heading: the tilt-compensated conversion, initialisation from
 * accelerometer plus magnetometer, and the gated yaw update.
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ekf_core.h"

#define TEST_DT     0.0025f
#define TEST_DT_US  2500ull
#define TEST_G      9.80665f
#define TEST_PI     3.14159265358979323846f
#define DEG         (TEST_PI / 180.0f)

static float wrap_pi(float a)
{
  while (a > TEST_PI)
    {
      a -= 2.0f * TEST_PI;
    }

  while (a < -TEST_PI)
    {
      a += 2.0f * TEST_PI;
    }

  return a;
}

static bool close_angle(float a, float b, float tol)
{
  return fabsf(wrap_pi(a - b)) < tol;
}

/* A northern-hemisphere field: 0.45 G total, dipping 60 degrees down. Held in
 * the LEVEL vehicle frame, then rotated into the body frame per test.
 */

#define FIELD_TOTAL 0.45f
#define FIELD_DIP   (60.0f * DEG)

static void level_field(float yaw, FAR float out[3])
{
  float h = FIELD_TOTAL * cosf(FIELD_DIP);
  float d = FIELD_TOTAL * sinf(FIELD_DIP);

  /* ENU: yaw is counter-clockwise FROM EAST. The field points toward
   * magnetic north, which sits 90 degrees counter-clockwise from east, so in
   * the body frame it lies at (90 - yaw) from forward.
   *
   *   facing east  (yaw 0)   -> field entirely to the LEFT, (0, h, -d)
   *   facing north (yaw 90)  -> field straight AHEAD,       (h, 0, -d)
   */

  out[0] = h * sinf(yaw);
  out[1] = h * cosf(yaw);
  out[2] = -d;   /* northern hemisphere: the field dips DOWN, and z is UP */
}

static void quaternion_from_euler_test(float roll, float pitch, float yaw,
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

/* Rotate a LEVEL-frame vector into the body frame for a given roll/pitch,
 * i.e. apply Rx(-roll) after Ry(-pitch): the inverse of the tilt the heading
 * routine has to undo.
 */

static void tilt_into_body(float roll, float pitch,
                           FAR const float level[3], FAR float body[3])
{
  float cr = cosf(roll);
  float sr = sinf(roll);
  float cp = cosf(pitch);
  float sp = sinf(pitch);
  float t[3];

  /* Inverse of Ry(pitch) */

  t[0] = cp * level[0] - sp * level[2];
  t[1] = level[1];
  t[2] = sp * level[0] + cp * level[2];

  /* Inverse of Rx(roll) */

  body[0] = t[0];
  body[1] = cr * t[1] + sr * t[2];
  body[2] = -sr * t[1] + cr * t[2];
}

/* Level and pointing north gives zero heading; the four cardinals follow. */

/* East is zero, north is +90. Pinned explicitly rather than only
 * round-tripped, so a frame change cannot quietly redefine what these mean.
 */

static void test_heading_cardinals(void)
{
  const float yaws[4] = {0.0f, 90.0f * DEG, 180.0f * DEG, 270.0f * DEG};
  float q[4];
  float field[3];
  float heading;
  int i;

  for (i = 0; i < 4; i++)
    {
      quaternion_from_euler_test(0.0f, 0.0f, yaws[i], q);
      level_field(yaws[i], field);

      assert(ekf_mag_heading(q, field, 0.0f, &heading));
      assert(close_angle(heading, yaws[i], 1.0e-3f));
    }
}

/* THE test this file exists for: heading must be independent of tilt.
 *
 * An uncompensated compass reads a heading that swings with roll and pitch,
 * and on a car that would look like heading noise correlated with the road
 * surface rather than a bug in the conversion.
 */

static void test_heading_is_tilt_compensated(void)
{
  const float tilts[5][2] =
  {
    {0.0f, 0.0f},
    {25.0f * DEG, 0.0f},
    {-25.0f * DEG, 0.0f},
    {0.0f, 20.0f * DEG},
    {15.0f * DEG, -18.0f * DEG}
  };
  const float yaw = 40.0f * DEG;
  float level[3];
  float body[3];
  float q[4];
  float heading;
  int i;

  level_field(yaw, level);

  for (i = 0; i < 5; i++)
    {
      float roll = tilts[i][0];
      float pitch = tilts[i][1];

      tilt_into_body(roll, pitch, level, body);
      quaternion_from_euler_test(roll, pitch, yaw, q);

      assert(ekf_mag_heading(q, body, 0.0f, &heading));
      assert(close_angle(heading, yaw, 2.0e-3f));
    }
}

/* Declination carries MAGNETIC heading to TRUE heading, and the sign is
 * asserted from the physics rather than from what the code happens to do.
 *
 * This test previously asserted heading == declination, which is a
 * round-trip of the implementation and agreed with a sign error for as long
 * as it existed. Declination is positive EAST: magnetic north lies east of
 * true north. So a vehicle whose compass reads magnetic north is pointing
 * east of true north, which in ENU - counter-clockwise from east - is a
 * SMALLER angle than 90, not a larger one.
 */

static void test_declination_applied(void)
{
  float q[4];
  float field[3];
  float heading;

  /* Point the vehicle at MAGNETIC north: the synthetic field is built for
   * yaw 90, so with zero declination the heading reads 90.
   */

  quaternion_from_euler_test(0.0f, 0.0f, 90.0f * DEG, q);
  level_field(90.0f * DEG, field);

  assert(ekf_mag_heading(q, field, 0.0f, &heading));
  assert(close_angle(heading, 90.0f * DEG, 1.0e-3f));

  /* +10 degrees EAST declination. Magnetic north is 10 east of true north,
   * so pointing at magnetic north is pointing 10 east of true north: 80 in
   * ENU, NOT 100. Getting this backwards is a 2x declination error.
   */

  assert(ekf_mag_heading(q, field, 10.0f * DEG, &heading));
  assert(close_angle(heading, 80.0f * DEG, 1.0e-3f));

  /* A WEST declination goes the other way, and must not be clamped away. */

  assert(ekf_mag_heading(q, field, -13.5f * DEG, &heading));
  assert(close_angle(heading, 103.5f * DEG, 1.0e-3f));
}

static void test_heading_refuses_degenerate_field(void)
{
  float q[4];
  float zero[3] = {0.0f, 0.0f, 0.0f};
  float nonfinite[3] = {0.0f, NAN, 0.0f};
  float heading = 99.0f;

  quaternion_from_euler_test(0.0f, 0.0f, 0.0f, q);

  assert(!ekf_mag_heading(q, zero, 0.0f, &heading));
  assert(!ekf_mag_heading(q, nonfinite, 0.0f, &heading));
}

/* Align the filter while feeding it a field, and confirm the resulting yaw is
 * the magnetic heading rather than the hardcoded zero it used to be.
 */

static void align_with_mag(FAR struct ekf_core_s *ekf, float yaw)
{
  struct ekf_imu_sample_s s;
  float field[3];
  uint64_t t = 1000000ull;
  int i;

  level_field(yaw, field);
  ekf_core_init(ekf);

  for (i = 0; i < 500 && !ekf->initialized; i++)
    {
      memset(&s, 0, sizeof(s));
      t += TEST_DT_US;
      s.timestamp_sample = t;
      s.timestamp_first = t - TEST_DT_US;
      s.delta_angle_dt = TEST_DT;
      s.delta_velocity_dt = TEST_DT;
      s.delta_velocity[2] = TEST_G * TEST_DT;   /* at rest: specific force is UP */
      s.samples = 5;
      s.accel_calibrated = true;
      s.gyro_calibrated = true;

      ekf_core_add_align_mag(ekf, field);
      ekf_core_process(ekf, &s);
    }

  assert(ekf->initialized);
}

static void test_alignment_uses_mag_heading(void)
{
  struct ekf_core_s ekf;
  float euler[3];

  align_with_mag(&ekf, 35.0f * DEG);

  ekf_core_euler(&ekf, euler);
  assert(close_angle(euler[2], 35.0f * DEG, 2.0e-2f));
  assert(ekf.yaw_absolute);
  assert((ekf_core_solution_status(&ekf) & EKF_SOLUTION_YAW_ABSOLUTE) != 0);
  assert((ekf_core_solution_status(&ekf) & EKF_SOLUTION_YAW_RELATIVE) == 0);
}

/* Without a field, alignment must behave exactly as it did before: yaw zero,
 * heading relative. Refusing to start would be worse than starting without
 * an absolute datum.
 */

static void test_alignment_without_mag_stays_relative(void)
{
  struct ekf_core_s ekf;
  struct ekf_imu_sample_s s;
  uint64_t t = 1000000ull;
  float euler[3];
  int i;

  ekf_core_init(&ekf);

  for (i = 0; i < 500 && !ekf.initialized; i++)
    {
      memset(&s, 0, sizeof(s));
      t += TEST_DT_US;
      s.timestamp_sample = t;
      s.timestamp_first = t - TEST_DT_US;
      s.delta_angle_dt = TEST_DT;
      s.delta_velocity_dt = TEST_DT;
      s.delta_velocity[2] = TEST_G * TEST_DT;   /* at rest: specific force is UP */
      s.samples = 5;
      s.accel_calibrated = true;
      s.gyro_calibrated = true;
      ekf_core_process(&ekf, &s);
    }

  assert(ekf.initialized);
  ekf_core_euler(&ekf, euler);
  assert(close_angle(euler[2], 0.0f, 1.0e-3f));
  assert(!ekf.yaw_absolute);
  assert((ekf_core_solution_status(&ekf) & EKF_SOLUTION_YAW_RELATIVE) != 0);
  assert((ekf_core_solution_status(&ekf) & EKF_SOLUTION_YAW_ABSOLUTE) == 0);
}

/* Fusing into a filter whose heading has no absolute datum would be a yaw
 * JUMP, not a correction. It must be refused rather than applied.
 */

static void test_fusion_refused_without_datum(void)
{
  struct ekf_core_s ekf;
  struct ekf_imu_sample_s s;
  uint64_t t = 1000000ull;
  float field[3];
  int i;

  ekf_core_init(&ekf);

  for (i = 0; i < 500 && !ekf.initialized; i++)
    {
      memset(&s, 0, sizeof(s));
      t += TEST_DT_US;
      s.timestamp_sample = t;
      s.timestamp_first = t - TEST_DT_US;
      s.delta_angle_dt = TEST_DT;
      s.delta_velocity_dt = TEST_DT;
      s.delta_velocity[2] = TEST_G * TEST_DT;   /* at rest: specific force is UP */
      s.samples = 5;
      s.accel_calibrated = true;
      s.gyro_calibrated = true;
      ekf_core_process(&ekf, &s);
    }

  level_field(30.0f * DEG, field);
  assert(ekf_core_fuse_mag(&ekf, field, 0.0f, FIELD_TOTAL, 0.5f, 5.0f) == -2);
  assert(ekf.mag_accept_count == 0);
}

/* A field agreeing with the current heading is accepted and shrinks the yaw
 * variance. That variance is what tells the rest of the filter the heading
 * is worth something.
 */

static void test_fusion_accepts_and_shrinks_yaw_variance(void)
{
  struct ekf_core_s ekf;
  float field[3];
  float before;
  int i;

  align_with_mag(&ekf, 35.0f * DEG);
  level_field(35.0f * DEG, field);
  before = ekf.covariance[EKF_P_INDEX(2, 2)];

  for (i = 0; i < 20; i++)
    {
      assert(ekf_core_fuse_mag(&ekf, field, 0.0f, FIELD_TOTAL, 0.5f,
                               5.0f) == 1);
    }

  assert(ekf.mag_accept_count == 20);
  assert(ekf.covariance[EKF_P_INDEX(2, 2)] < before);
  assert(ekf.covariance[EKF_P_INDEX(2, 2)] > 0.0f);
}

/* A field magnitude far from CAL_MAG0_FIELD is a magnet, a motor or a failed
 * sensor. It must be refused before it reaches the filter, not merely gated
 * afterwards.
 */

static void test_field_magnitude_gate(void)
{
  struct ekf_core_s ekf;
  float field[3];
  float before_yaw;
  int i;

  align_with_mag(&ekf, 35.0f * DEG);
  level_field(35.0f * DEG, field);

  for (i = 0; i < 3; i++)
    {
      field[i] *= 3.0f;
    }

  before_yaw = ekf.quaternion[3];

  assert(ekf_core_fuse_mag(&ekf, field, 0.0f, FIELD_TOTAL, 0.5f,
                           5.0f) == -1);
  assert(ekf.mag_unhealthy_count == 1);
  assert(ekf.mag_accept_count == 0);
  assert(fabsf(ekf.quaternion[3] - before_yaw) < 1.0e-9f);
}

/* A wildly wrong heading inside a healthy-looking field is what the
 * innovation gate is for. It must leave the state untouched.
 */

static void test_heading_gate_rejects_cleanly(void)
{
  struct ekf_core_s ekf;
  struct ekf_core_s before;
  float field[3];
  int i;

  align_with_mag(&ekf, 0.0f);

  /* Converge first, so a 150-degree error is genuinely outside the gate
   * rather than merely surprising a wide initial covariance.
   */

  level_field(0.0f, field);

  for (i = 0; i < 50; i++)
    {
      ekf_core_fuse_mag(&ekf, field, 0.0f, FIELD_TOTAL, 0.5f, 5.0f);
    }

  before = ekf;
  level_field(150.0f * DEG, field);

  assert(ekf_core_fuse_mag(&ekf, field, 0.0f, FIELD_TOTAL, 0.5f,
                           5.0f) == 0);
  assert(ekf.mag_reject_count == before.mag_reject_count + 1);
  assert(ekf.mag_consecutive_rejects ==
         before.mag_consecutive_rejects + 1);
  assert(fabsf(ekf.quaternion[3] - before.quaternion[3]) < 1.0e-9f);
}

/* A sustained rejection run withdraws the absolute claim while attitude and
 * relative heading carry on. Rejecting a bad heading and continuing on the
 * gyro is always better than injecting it.
 */

static void test_sustained_rejection_drops_absolute(void)
{
  struct ekf_core_s ekf;
  float field[3];
  uint8_t status;
  int i;

  align_with_mag(&ekf, 0.0f);
  level_field(0.0f, field);

  for (i = 0; i < 50; i++)
    {
      ekf_core_fuse_mag(&ekf, field, 0.0f, FIELD_TOTAL, 0.5f, 5.0f);
    }

  assert((ekf_core_solution_status(&ekf) &
          EKF_SOLUTION_YAW_ABSOLUTE) != 0);

  level_field(150.0f * DEG, field);

  for (i = 0; i < (int)EKF_MAG_REJECT_RUN_MAX + 1; i++)
    {
      ekf_core_fuse_mag(&ekf, field, 0.0f, FIELD_TOTAL, 0.5f, 5.0f);
    }

  status = ekf_core_solution_status(&ekf);
  assert((status & EKF_SOLUTION_YAW_ABSOLUTE) == 0);
  assert((status & EKF_SOLUTION_YAW_RELATIVE) != 0);
  assert((status & EKF_SOLUTION_ATTITUDE) != 0);
}

/* An accepted update clears the run, so a single disturbance does not
 * permanently demote the solution.
 */

static void test_accept_clears_run(void)
{
  struct ekf_core_s ekf;
  float field[3];

  align_with_mag(&ekf, 0.0f);

  level_field(150.0f * DEG, field);
  ekf_core_fuse_mag(&ekf, field, 0.0f, FIELD_TOTAL, 0.5f, 5.0f);

  level_field(0.0f, field);
  assert(ekf_core_fuse_mag(&ekf, field, 0.0f, FIELD_TOTAL, 0.5f,
                           5.0f) == 1);
  assert(ekf.mag_consecutive_rejects == 0);
}

/* Yaw fusion must not disturb roll and pitch. With no earth-field or
 * body-field states in a 15-state filter, that separation is the whole
 * argument for fusing heading rather than the full vector.
 */

static void test_yaw_fusion_leaves_roll_pitch(void)
{
  struct ekf_core_s ekf;
  float field[3];
  float before[3];
  float after[3];
  int i;

  align_with_mag(&ekf, 0.0f);
  ekf_core_euler(&ekf, before);

  /* A 20-degree heading error, repeatedly. */

  level_field(20.0f * DEG, field);

  for (i = 0; i < 30; i++)
    {
      ekf_core_fuse_mag(&ekf, field, 0.0f, FIELD_TOTAL, 0.5f, 5.0f);
    }

  ekf_core_euler(&ekf, after);

  assert(close_angle(after[0], before[0], 1.0e-3f));
  assert(close_angle(after[1], before[1], 1.0e-3f));

  /* And it did move the heading it was supposed to move. */

  assert(fabsf(wrap_pi(after[2] - before[2])) > 5.0f * DEG);
}

/* A re-alignment throws the datum away with everything else. */

static void test_realignment_clears_absolute(void)
{
  struct ekf_core_s ekf;
  struct ekf_imu_sample_s s;
  uint64_t t;

  align_with_mag(&ekf, 35.0f * DEG);
  assert(ekf.yaw_absolute);

  t = ekf.last_timestamp_sample + TEST_DT_US;
  memset(&s, 0, sizeof(s));
  s.timestamp_sample = t;
  s.timestamp_first = t - TEST_DT_US;
  s.delta_angle_dt = TEST_DT;
  s.delta_velocity_dt = TEST_DT;
  s.delta_velocity[2] = TEST_G * TEST_DT;   /* at rest: specific force is UP */
  s.samples = 5;
  s.accel_calibrated = false;
  s.gyro_calibrated = false;

  assert(ekf_core_process(&ekf, &s) == EKF_PROCESS_REJECTED);
  assert(!ekf.yaw_absolute);
  assert(ekf.align_mag_samples == 0);
}

int main(void)
{
  test_heading_cardinals();
  test_heading_is_tilt_compensated();
  test_declination_applied();
  test_heading_refuses_degenerate_field();
  test_alignment_uses_mag_heading();
  test_alignment_without_mag_stays_relative();
  test_fusion_refused_without_datum();
  test_fusion_accepts_and_shrinks_yaw_variance();
  test_field_magnitude_gate();
  test_heading_gate_rejects_cleanly();
  test_sustained_rejection_drops_absolute();
  test_accept_clears_run();
  test_yaw_fusion_leaves_roll_pitch();
  test_realignment_clears_absolute();

  puts("ekf_mag: tilt compensation, init and gated yaw fusion verified - OK");
  return 0;
}
