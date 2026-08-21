/****************************************************************************
 * tests/cal_mag_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "cal_mag.h"

#define TEST_PI 3.14159265358979323846f

static void invert(FAR const float matrix[3][3], FAR float result[3][3])
{
  float determinant =
    matrix[0][0] * (matrix[1][1] * matrix[2][2] -
                    matrix[1][2] * matrix[2][1]) -
    matrix[0][1] * (matrix[1][0] * matrix[2][2] -
                    matrix[1][2] * matrix[2][0]) +
    matrix[0][2] * (matrix[1][0] * matrix[2][1] -
                    matrix[1][1] * matrix[2][0]);

  assert(fabsf(determinant) > 1.0e-6f);
  result[0][0] = (matrix[1][1] * matrix[2][2] -
                  matrix[1][2] * matrix[2][1]) / determinant;
  result[0][1] = (matrix[0][2] * matrix[2][1] -
                  matrix[0][1] * matrix[2][2]) / determinant;
  result[0][2] = (matrix[0][1] * matrix[1][2] -
                  matrix[0][2] * matrix[1][1]) / determinant;
  result[1][0] = (matrix[1][2] * matrix[2][0] -
                  matrix[1][0] * matrix[2][2]) / determinant;
  result[1][1] = (matrix[0][0] * matrix[2][2] -
                  matrix[0][2] * matrix[2][0]) / determinant;
  result[1][2] = (matrix[0][2] * matrix[1][0] -
                  matrix[0][0] * matrix[1][2]) / determinant;
  result[2][0] = (matrix[1][0] * matrix[2][1] -
                  matrix[1][1] * matrix[2][0]) / determinant;
  result[2][1] = (matrix[0][1] * matrix[2][0] -
                  matrix[0][0] * matrix[2][1]) / determinant;
  result[2][2] = (matrix[0][0] * matrix[1][1] -
                  matrix[0][1] * matrix[1][0]) / determinant;
}

static float determinant(FAR const float matrix[3][3])
{
  return matrix[0][0] *
         (matrix[1][1] * matrix[2][2] -
          matrix[1][2] * matrix[2][1]) -
         matrix[0][1] *
         (matrix[1][0] * matrix[2][2] -
          matrix[1][2] * matrix[2][0]) +
         matrix[0][2] *
         (matrix[1][0] * matrix[2][1] -
          matrix[1][1] * matrix[2][0]);
}

static void normalize_determinant(FAR float matrix[3][3])
{
  float scale = powf(determinant(matrix), -1.0f / 3.0f);
  int row;
  int column;

  for (row = 0; row < 3; row++)
    {
      for (column = 0; column < 3; column++)
        {
          matrix[row][column] *= scale;
        }
    }
}

static void synthesize(FAR const float inverse[3][3],
                       FAR const float offset[3], float field, int index,
                       int count, FAR float raw[3])
{
  const float golden = 2.39996322972865332f;
  float z = 1.0f - 2.0f * ((float)index + 0.5f) / (float)count;
  float radius = sqrtf(1.0f - z * z);
  float direction[3] =
  {
    radius * cosf(golden * index),
    radius * sinf(golden * index),
    z
  };
  int row;
  int column;

  for (row = 0; row < 3; row++)
    {
      raw[row] = offset[row];

      for (column = 0; column < 3; column++)
        {
          raw[row] += inverse[row][column] * field * direction[column];
        }

      raw[row] += 0.0012f * sinf(0.37f * index + 1.7f * row);
    }
}

static void test_full_ellipsoid(void)
{
  struct cal_mag_s calibration;
  struct cal_mag_fit_s fit;
  float matrix[3][3] =
  {
    {1.20f, 0.08f, -0.04f},
    {0.08f, 0.88f, 0.05f},
    {-0.04f, 0.05f, 0.96f}
  };
  float inverse_matrix[3][3];
  const float offset[3] = {0.18f, -0.12f, 0.07f};
  const float field = 0.47f;
  float raw[3];
  int index;
  int row;
  int column;

  normalize_determinant(matrix);
  invert(matrix, inverse_matrix);
  cal_mag_reset(&calibration);

  for (index = 0; index < 260; index++)
    {
      synthesize(inverse_matrix, offset, field, index, 260, raw);
      cal_mag_add(&calibration, raw);
    }

  /* Short magnetic transients are not part of the vehicle's hard/soft-iron
   * calibration. Ensure isolated, spatially distinct disturbances are
   * trimmed rather than pulling the ellipsoid toward them.
   */

  for (index = 0; index < 8; index++)
    {
      synthesize(inverse_matrix, offset, field, index * 29 + 7, 263, raw);
      raw[index % 3] += index & 1 ? 0.16f : -0.16f;
      cal_mag_add(&calibration, raw);
    }

  assert(calibration.count >= CAL_MAG_MIN_SAMPLES);
  assert(cal_mag_solve(&calibration, &fit) == CAL_MAG_OK);
  assert(fit.used >= CAL_MAG_MIN_SAMPLES);
  assert(fit.rejected >= 6);
  assert(fit.octants == 0xff);
  assert(fabsf(fit.field - field) < 0.010f);
  assert(fit.rms < 0.004f);
  assert(fit.maximum_error < 0.010f);

  for (row = 0; row < 3; row++)
    {
      assert(fabsf(fit.offset[row] - offset[row]) < 0.008f);

      for (column = 0; column < 3; column++)
        {
          assert(fabsf(fit.matrix[row][column] -
                       matrix[row][column]) < 0.025f);
        }
    }
}

static void test_poor_coverage_rejected(void)
{
  struct cal_mag_s calibration;
  struct cal_mag_fit_s fit;
  int index;

  cal_mag_reset(&calibration);

  for (index = 0; index < 300; index++)
    {
      float angle = 2.0f * TEST_PI * index / 300.0f;
      float raw[3] =
      {
        0.5f * cosf(angle),
        0.5f * sinf(angle),
        0.03f * sinf(3.0f * angle)
      };

      cal_mag_add(&calibration, raw);
    }

  assert(cal_mag_solve(&calibration, &fit) != CAL_MAG_OK);
}

static void test_too_few_rejected(void)
{
  struct cal_mag_s calibration;
  struct cal_mag_fit_s fit;
  float raw[3] = {0.3f, 0.1f, 0.2f};
  int index;

  cal_mag_reset(&calibration);

  for (index = 0; index < 20; index++)
    {
      raw[0] += 0.02f;
      cal_mag_add(&calibration, raw);
    }

  assert(cal_mag_solve(&calibration, &fit) == CAL_MAG_NEED_SAMPLES);
}

int main(void)
{
  test_full_ellipsoid();
  test_poor_coverage_rejected();
  test_too_few_rejected();
  puts("cal_mag: full ellipsoid fit and rejection checks verified - OK");
  return 0;
}
