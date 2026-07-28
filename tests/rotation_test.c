/* Host unit test for the fixed sensor rotations (apps/sensors/rotation.c).
 *
 * Every one of these swaps was transcribed by hand from PX4, and a transcription
 * error here is close to undetectable downstream: the vehicle simply believes it
 * is oriented differently than it is, consistently, with no reading out of range
 * and no error anywhere. So the properties are checked rather than the values.
 *
 * The important one is the DETERMINANT. A dropped or added minus sign turns a
 * rotation into a REFLECTION, which preserves every vector length and every
 * angle magnitude - so a magnitude test passes happily - while mirroring the
 * frame. Gyro signs invert, an estimator's yaw runs the wrong way, and nothing
 * about the numbers looks wrong. det = +1 is what separates the two.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "rotation.h"

static int g_fail;

static void fail(const char *what)
{
  printf("FAIL %s\n", what);
  g_fail++;
}

/* Columns are the images of the basis vectors, so m[i][j] is component i of
 * the rotated e_j.
 */

static int matrix_of(uint8_t rot, float m[3][3])
{
  int j;

  for (j = 0; j < 3; j++)
    {
      float v[3] = { 0.0f, 0.0f, 0.0f };
      int i;

      v[j] = 1.0f;

      if (!rotation_apply(rot, v))
        {
          return 0;
        }

      for (i = 0; i < 3; i++)
        {
          m[i][j] = v[i];
        }
    }

  return 1;
}

static float det_of(const float m[3][3])
{
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
       - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
       + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

/* ---- 1. every supported rotation is a proper axis permutation ---------- */

static void test_supported_are_proper_rotations(void)
{
  int found = 0;
  int rot;

  for (rot = 0; rot < 64; rot++)
    {
      float m[3][3];
      int i;
      int j;

      if (!matrix_of((uint8_t)rot, m))
        {
          continue;                       /* refused; checked separately */
        }

      found++;

      /* Exactly one non-zero per row and per column, and it is +/-1. Anything
       * else is not an axis permutation, and a diagonal scale calibration
       * would stop being meaningful under it.
       */

      for (i = 0; i < 3; i++)
        {
          int row_nz = 0;
          int col_nz = 0;

          for (j = 0; j < 3; j++)
            {
              if (m[i][j] != 0.0f)
                {
                  row_nz++;
                  if (fabsf(m[i][j]) != 1.0f)
                    {
                      printf("FAIL rot %d: entry (%d,%d) is %g, not +/-1\n",
                             rot, i, j, m[i][j]);
                      g_fail++;
                    }
                }

              if (m[j][i] != 0.0f)
                {
                  col_nz++;
                }
            }

          if (row_nz != 1 || col_nz != 1)
            {
              printf("FAIL rot %d: row %d has %d non-zeros, column %d has %d "
                     "- not a permutation\n", rot, i, row_nz, i, col_nz);
              g_fail++;
            }
        }

      /* The one that matters. */

      if (fabsf(det_of(m) - 1.0f) > 1e-6f)
        {
          printf("FAIL rot %d (%s): determinant %g, not +1 - this is a "
                 "REFLECTION, not a rotation\n",
                 rot, rotation_name((uint8_t)rot), det_of(m));
          g_fail++;
        }
    }

  /* PX4's exact-swap set is 26 enum values covering 24 distinct rotations. */

  if (found != 26)
    {
      printf("FAIL %d rotations accepted, expected 26\n", found);
      g_fail++;
    }
}

/* ---- 2. the 45-degree ones are refused, and refusal is inert ----------- */

static void test_unsupported_are_refused_untouched(void)
{
  const uint8_t forty_fives[] = { 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23 };
  size_t k;

  for (k = 0; k < sizeof(forty_fives) / sizeof(forty_fives[0]); k++)
    {
      float v[3] = { 1.0f, 2.0f, 3.0f };

      if (rotation_apply(forty_fives[k], v))
        {
          printf("FAIL rot %u: a 45-degree rotation was accepted, and this "
                 "implementation cannot do it exactly\n", forty_fives[k]);
          g_fail++;
        }

      if (v[0] != 1.0f || v[1] != 2.0f || v[2] != 3.0f)
        {
          printf("FAIL rot %u: refused but modified the vector\n",
                 forty_fives[k]);
          g_fail++;
        }

      if (rotation_supported(forty_fives[k]))
        {
          fail("rotation_supported disagrees with rotation_apply");
        }
    }

  /* Out of range must be refused too, not indexed. */

  if (rotation_supported(200) || rotation_supported(41))
    {
      fail("an out-of-range rotation was reported as supported");
    }
}

/* ---- 3. distinctness, and only the two documented aliases -------------- */

static void test_only_documented_aliases_collide(void)
{
  int a;
  int b;

  for (a = 0; a < 64; a++)
    {
      float ma[3][3];

      if (!matrix_of((uint8_t)a, ma))
        {
          continue;
        }

      for (b = a + 1; b < 64; b++)
        {
          float mb[3][3];
          int allowed;

          if (!matrix_of((uint8_t)b, mb))
            {
              continue;
            }

          if (memcmp(ma, mb, sizeof(ma)) != 0)
            {
              continue;
            }

          /* PX4 documents exactly these two coincidences: the same permutation
           * reached by different Euler routes.
           */

          allowed = (a == ROTATION_ROLL_180_YAW_90 &&
                     b == ROTATION_PITCH_180_YAW_270) ||
                    (a == ROTATION_ROLL_180_YAW_270 &&
                     b == ROTATION_PITCH_180_YAW_90);

          if (!allowed)
            {
              printf("FAIL rot %d (%s) and %d (%s) are the same rotation - "
                     "one of them is transcribed wrong\n",
                     a, rotation_name((uint8_t)a),
                     b, rotation_name((uint8_t)b));
              g_fail++;
            }
        }
    }
}

/* ---- 4. the cases this board actually depends on ----------------------- */

static void test_known_cases(void)
{
  float v[3];

  /* YAW_90 is (x,y,z) -> (-y,x,z). Spelled out because everything else in
   * this file is a property test, and a property test cannot catch the whole
   * table being consistently rotated the wrong way round.
   */

  v[0] = 1.0f; v[1] = 2.0f; v[2] = 3.0f;
  rotation_apply(ROTATION_YAW_90, v);
  if (v[0] != -2.0f || v[1] != 1.0f || v[2] != 3.0f)
    {
      printf("FAIL YAW_90 gave (%g,%g,%g), want (-2,1,3)\n",
             v[0], v[1], v[2]);
      g_fail++;
    }

  /* docs/imu-timestamp-audit-2026-07-26.md measured, from motion correlation
   * on this board: ICM x = -BMI y, ICM y = BMI x, ICM z = BMI z. So YAW_90
   * applied to a BMI reading must land in the ICM frame. This is the default
   * SENS_IMU1_ROT, and if this assertion ever has to change, that default
   * changes with it.
   */

  v[0] = 0.5f; v[1] = -0.25f; v[2] = 9.81f;       /* a BMI reading */
  rotation_apply(ROTATION_YAW_90, v);
  if (v[0] != 0.25f || v[1] != 0.5f || v[2] != 9.81f)
    {
      printf("FAIL BMI->ICM mapping gave (%g,%g,%g), want (0.25,0.5,9.81)\n",
             v[0], v[1], v[2]);
      g_fail++;
    }

  /* Upside down: roll 180 flips Y and Z, so gravity reverses and X does not. */

  v[0] = 0.1f; v[1] = 0.2f; v[2] = 9.81f;
  rotation_apply(ROTATION_ROLL_180, v);
  if (v[0] != 0.1f || v[1] != -0.2f || v[2] != -9.81f)
    {
      printf("FAIL ROLL_180 gave (%g,%g,%g), want (0.1,-0.2,-9.81)\n",
             v[0], v[1], v[2]);
      g_fail++;
    }
}

/* ---- 5. rotations compose, which is how board + sensor is applied ------ */

static void test_composition_is_a_rotation(void)
{
  /* The pipeline applies the sensor's rotation and then the board's. Two
   * proper rotations compose to a proper rotation, so the result must still
   * preserve magnitude - if it does not, the two are being applied to
   * different things.
   */

  float v[3] = { 1.0f, -2.0f, 3.0f };
  float before = sqrtf(1.0f + 4.0f + 9.0f);
  float after;

  rotation_apply(ROTATION_YAW_90, v);
  rotation_apply(ROTATION_ROLL_90, v);
  after = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

  if (fabsf(after - before) > 1e-5f)
    {
      printf("FAIL composed rotation changed magnitude %g -> %g\n",
             before, after);
      g_fail++;
    }
}

int main(void)
{
  test_supported_are_proper_rotations();
  test_unsupported_are_refused_untouched();
  test_only_documented_aliases_collide();
  test_known_cases();
  test_composition_is_a_rotation();

  if (g_fail != 0)
    {
      printf("rotation: %d failure(s)\n", g_fail);
      return 1;
    }

  printf("rotation: 26 values, all proper rotations, 45s refused - OK\n");
  return 0;
}
