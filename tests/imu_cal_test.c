/* Host unit test for apps/imu_cal/imu_cal.c - the apply math only.
 *
 * imu_cal_load() needs the parameter store and so belongs on hardware; the
 * arithmetic does not, and the arithmetic is where a transposed index or a
 * sign error hides. A wrong M[1][2] still produces plausible-looking numbers.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "imu_cal.h"

static int g_fail;

static void expect_vec(const char *what, const float got[3],
                       float x, float y, float z)
{
  const float tol = 1e-5f;

  if (fabsf(got[0] - x) > tol || fabsf(got[1] - y) > tol ||
      fabsf(got[2] - z) > tol)
    {
      printf("FAIL %s: got (%f, %f, %f) want (%f, %f, %f)\n",
             what, got[0], got[1], got[2], x, y, z);
      g_fail++;
    }
}

/* An uncalibrated sensor must pass through untouched. Returning zeros or a
 * half-applied matrix here would be far worse than doing nothing.
 */

static void test_invalid_is_passthrough(void)
{
  struct imu_cal_s cal;
  const float in[3] = { 1.5f, -2.5f, 9.8f };
  float out[3];

  memset(&cal, 0, sizeof(cal));
  cal.valid = false;

  imu_cal_apply(&cal, in, out);
  expect_vec("invalid passthrough", out, 1.5f, -2.5f, 9.8f);
}

static void test_identity(void)
{
  struct imu_cal_s cal;
  const float in[3] = { 1.0f, 2.0f, 3.0f };
  float out[3];

  memset(&cal, 0, sizeof(cal));
  cal.M[0] = cal.M[4] = cal.M[8] = 1.0f;
  cal.valid = true;

  imu_cal_apply(&cal, in, out);
  expect_vec("identity", out, 1.0f, 2.0f, 3.0f);
}

/* The real case. A sensor with error matrix diag(2, 4, 5) and bias (1, 2, 3)
 * measures  m = M_err * t + b.  For t = (1,1,1) that is (3, 6, 8). Storing the
 * INVERSE, diag(0.5, 0.25, 0.2), must recover t exactly. These values are
 * exact in binary floating point, so the test has no rounding slack to hide in.
 */

static void test_recovers_known_truth(void)
{
  struct imu_cal_s cal;
  const float measured[3] = { 3.0f, 6.0f, 8.0f };
  float out[3];

  memset(&cal, 0, sizeof(cal));
  cal.M[0] = 0.5f;
  cal.M[4] = 0.25f;
  cal.M[8] = 0.2f;
  cal.b[0] = 1.0f;
  cal.b[1] = 2.0f;
  cal.b[2] = 3.0f;
  cal.valid = true;

  imu_cal_apply(&cal, measured, out);
  expect_vec("recover truth", out, 1.0f, 1.0f, 1.0f);
}

/* Row-major, and rows are not columns. With a single off-diagonal term at
 * M[0][1] the X output must pick up Y, and nothing else may change. A
 * transposed implementation passes every diagonal test and fails this one.
 */

static void test_row_major_not_transposed(void)
{
  struct imu_cal_s cal;
  const float in[3] = { 0.0f, 10.0f, 0.0f };
  float out[3];

  memset(&cal, 0, sizeof(cal));
  cal.M[0] = 1.0f;
  cal.M[1] = 0.5f;      /* M[0][1]: X picks up Y */
  cal.M[4] = 1.0f;
  cal.M[8] = 1.0f;
  cal.valid = true;

  imu_cal_apply(&cal, in, out);
  expect_vec("row major", out, 5.0f, 10.0f, 0.0f);
}

/* Callers will want to correct a sample where it sits. If the implementation
 * writes out[0] before reading in[1], aliasing corrupts the result silently.
 */

static void test_in_place_aliasing(void)
{
  struct imu_cal_s cal;
  float v[3] = { 3.0f, 6.0f, 8.0f };

  memset(&cal, 0, sizeof(cal));
  cal.M[0] = 0.5f;
  cal.M[4] = 0.25f;
  cal.M[8] = 0.2f;
  cal.b[0] = 1.0f;
  cal.b[1] = 2.0f;
  cal.b[2] = 3.0f;
  cal.valid = true;

  imu_cal_apply(&cal, v, v);
  expect_vec("in place", v, 1.0f, 1.0f, 1.0f);
}

int main(void)
{
  test_invalid_is_passthrough();
  test_identity();
  test_recovers_known_truth();
  test_row_major_not_transposed();
  test_in_place_aliasing();

  if (g_fail != 0)
    {
      printf("imu_cal: %d failure(s)\n", g_fail);
      return 1;
    }

  printf("imu_cal: apply math verified - OK\n");
  return 0;
}
