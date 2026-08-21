/****************************************************************************
 * apps/cal/cal_mag.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef CAL_MAG_HOST_TEST
#  include <nuttx/config.h>
#endif

#include <math.h>
#include <string.h>

#include "cal_mag.h"

#define CAL_MAG_FIELD_MIN       0.15f
#define CAL_MAG_FIELD_MAX       0.80f
#define CAL_MAG_OFFSET_MAX      1.50f
#define CAL_MAG_EIGEN_MIN       0.25f
#define CAL_MAG_EIGEN_MAX       4.00f
#define CAL_MAG_CONDITION_MAX   4.00f
#define CAL_MAG_SYMMETRY_EPS    1.0e-5f

/* Eigenvalues only: bounded Jacobi rotations for a symmetric 3x3 matrix. */

static int symmetric_eigenvalues(FAR const float input[3][3],
                                 FAR float value[3])
{
  double a[3][3];
  int iteration;
  int row;
  int column;

  for (row = 0; row < 3; row++)
    {
      for (column = 0; column < 3; column++)
        {
          a[row][column] = input[row][column];
        }
    }

  for (iteration = 0; iteration < 24; iteration++)
    {
      int p = 0;
      int q = 1;
      double largest = fabs(a[0][1]);
      double angle;
      double cosine;
      double sine;

      if (fabs(a[0][2]) > largest)
        {
          largest = fabs(a[0][2]);
          q = 2;
        }

      if (fabs(a[1][2]) > largest)
        {
          largest = fabs(a[1][2]);
          p = 1;
          q = 2;
        }

      if (largest < 1.0e-12)
        {
          break;
        }

      angle = 0.5 * atan2(2.0 * a[p][q], a[q][q] - a[p][p]);
      cosine = cos(angle);
      sine = sin(angle);

      for (row = 0; row < 3; row++)
        {
          double arp = a[row][p];
          double arq = a[row][q];

          a[row][p] = cosine * arp - sine * arq;
          a[row][q] = sine * arp + cosine * arq;
        }

      for (column = 0; column < 3; column++)
        {
          double apc = a[p][column];
          double aqc = a[q][column];

          a[p][column] = cosine * apc - sine * aqc;
          a[q][column] = sine * apc + cosine * aqc;
        }
    }

  for (row = 0; row < 3; row++)
    {
      value[row] = (float)a[row][row];

      if (!isfinite(value[row]))
        {
          return -1;
        }
    }

  return 0;
}

enum cal_mag_result_e cal_mag_validate(FAR const struct cal_mag_fit_s *fit)
{
  float eigenvalue[3];
  float minimum;
  float maximum;
  int row;
  int column;

  if (fit == NULL || !isfinite(fit->field))
    {
      return CAL_MAG_NONFINITE;
    }

  if (fit->field < CAL_MAG_FIELD_MIN || fit->field > CAL_MAG_FIELD_MAX)
    {
      return CAL_MAG_FIELD_RANGE;
    }

  for (row = 0; row < 3; row++)
    {
      if (!isfinite(fit->offset[row]))
        {
          return CAL_MAG_NONFINITE;
        }

      if (fabsf(fit->offset[row]) > CAL_MAG_OFFSET_MAX)
        {
          return CAL_MAG_OFFSET_RANGE;
        }

      for (column = 0; column < 3; column++)
        {
          if (!isfinite(fit->matrix[row][column]))
            {
              return CAL_MAG_NONFINITE;
            }

          if (fabsf(fit->matrix[row][column] -
                    fit->matrix[column][row]) > CAL_MAG_SYMMETRY_EPS)
            {
              return CAL_MAG_NOT_POSITIVE_DEFINITE;
            }
        }
    }

  if (symmetric_eigenvalues(fit->matrix, eigenvalue) < 0)
    {
      return CAL_MAG_NONFINITE;
    }

  minimum = maximum = eigenvalue[0];

  for (row = 1; row < 3; row++)
    {
      minimum = fminf(minimum, eigenvalue[row]);
      maximum = fmaxf(maximum, eigenvalue[row]);
    }

  if (!(minimum > 0.0f))
    {
      return CAL_MAG_NOT_POSITIVE_DEFINITE;
    }

  if (minimum < CAL_MAG_EIGEN_MIN || maximum > CAL_MAG_EIGEN_MAX ||
      maximum / minimum > CAL_MAG_CONDITION_MAX)
    {
      return CAL_MAG_SCALE_RANGE;
    }

  return CAL_MAG_OK;
}

void cal_mag_apply(FAR const struct cal_mag_fit_s *fit,
                   FAR const float raw[3], FAR float corrected[3])
{
  float centered[3];
  int row;

  centered[0] = raw[0] - fit->offset[0];
  centered[1] = raw[1] - fit->offset[1];
  centered[2] = raw[2] - fit->offset[2];

  for (row = 0; row < 3; row++)
    {
      corrected[row] = fit->matrix[row][0] * centered[0] +
                       fit->matrix[row][1] * centered[1] +
                       fit->matrix[row][2] * centered[2];
    }
}

FAR const char *cal_mag_result_string(enum cal_mag_result_e result)
{
  switch (result)
    {
      case CAL_MAG_OK:
        return "ok";
      case CAL_MAG_NONFINITE:
        return "non-finite value";
      case CAL_MAG_FIELD_RANGE:
        return "field strength out of range";
      case CAL_MAG_OFFSET_RANGE:
        return "hard-iron offset out of range";
      case CAL_MAG_NOT_POSITIVE_DEFINITE:
        return "matrix is not symmetric positive definite";
      case CAL_MAG_SCALE_RANGE:
        return "soft-iron scale or condition out of range";
      default:
        return "unknown";
    }
}
