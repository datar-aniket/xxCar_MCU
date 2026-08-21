/****************************************************************************
 * apps/cal/cal_mag.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See cal_mag.h.
 ****************************************************************************/

#ifndef CAL_MAG_HOST_TEST
#  include <nuttx/config.h>
#endif

#include <errno.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include "cal_mag.h"

#define CAL_MAG_MIN_SEPARATION  0.018f
#define CAL_MAG_RAW_MAX         4.0f
#define CAL_MAG_FIELD_MIN       0.15f
#define CAL_MAG_FIELD_MAX       0.80f
#define CAL_MAG_OFFSET_MAX      1.50f
#define CAL_MAG_EIGEN_MIN       0.25f
#define CAL_MAG_EIGEN_MAX       4.00f
#define CAL_MAG_CONDITION_MAX   4.00f
#define CAL_MAG_RMS_MAX         0.030f
#define CAL_MAG_REL_RMS_MAX     0.070f
#define CAL_MAG_MAX_ERROR       0.100f
#define CAL_MAG_POLE_COS        0.55f
#define CAL_MAG_REFINE_STEPS    10

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool vector_finite(FAR const float value[3])
{
  return isfinite(value[0]) && isfinite(value[1]) && isfinite(value[2]);
}

static double vector_norm(FAR const float value[3])
{
  return sqrt((double)value[0] * value[0] +
              (double)value[1] * value[1] +
              (double)value[2] * value[2]);
}

/* Partial-pivot Gaussian elimination. Calibration runs once on demand, so a
 * compact and auditable solver is preferable to pulling a general matrix
 * package into the flight image.
 */

static bool solve_linear(FAR double *a, FAR double *b, FAR double *x, int n)
{
  int column;
  int row;
  int inner;

  for (column = 0; column < n; column++)
    {
      int pivot = column;
      double best = fabs(a[column * n + column]);

      for (row = column + 1; row < n; row++)
        {
          double candidate = fabs(a[row * n + column]);

          if (candidate > best)
            {
              best = candidate;
              pivot = row;
            }
        }

      if (!(best > 1.0e-12) || !isfinite(best))
        {
          return false;
        }

      if (pivot != column)
        {
          for (inner = column; inner < n; inner++)
            {
              double temporary = a[column * n + inner];

              a[column * n + inner] = a[pivot * n + inner];
              a[pivot * n + inner] = temporary;
            }

          {
            double temporary = b[column];

            b[column] = b[pivot];
            b[pivot] = temporary;
          }
        }

      for (row = column + 1; row < n; row++)
        {
          double factor = a[row * n + column] /
                          a[column * n + column];

          a[row * n + column] = 0.0;

          for (inner = column + 1; inner < n; inner++)
            {
              a[row * n + inner] -= factor * a[column * n + inner];
            }

          b[row] -= factor * b[column];
        }
    }

  for (row = n - 1; row >= 0; row--)
    {
      double value = b[row];

      for (inner = row + 1; inner < n; inner++)
        {
          value -= a[row * n + inner] * x[inner];
        }

      x[row] = value / a[row * n + row];

      if (!isfinite(x[row]))
        {
          return false;
        }
    }

  return true;
}

static bool inverse_3x3(FAR const double a[3][3], FAR double inverse[3][3])
{
  double determinant =
    a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
    a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
    a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

  if (!(fabs(determinant) > 1.0e-12) || !isfinite(determinant))
    {
      return false;
    }

  inverse[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) /
                  determinant;
  inverse[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) /
                  determinant;
  inverse[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) /
                  determinant;
  inverse[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) /
                  determinant;
  inverse[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) /
                  determinant;
  inverse[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) /
                  determinant;
  inverse[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) /
                  determinant;
  inverse[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) /
                  determinant;
  inverse[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) /
                  determinant;
  return true;
}

/* Jacobi eigen decomposition for a symmetric 3x3 matrix. Eigenvectors are
 * returned in columns. The fixed iteration bound is ample at this dimension.
 */

static bool symmetric_eigen(FAR const double input[3][3],
                            FAR double value[3], FAR double vector[3][3])
{
  double a[3][3];
  int iteration;
  int row;
  int column;

  memcpy(a, input, sizeof(a));
  memset(vector, 0, 9 * sizeof(double));
  vector[0][0] = vector[1][1] = vector[2][2] = 1.0;

  for (iteration = 0; iteration < 24; iteration++)
    {
      int p = 0;
      int q = 1;
      double largest = fabs(a[0][1]);

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

      {
        double angle = 0.5 * atan2(2.0 * a[p][q], a[q][q] - a[p][p]);
        double cosine = cos(angle);
        double sine = sin(angle);

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

        for (row = 0; row < 3; row++)
          {
            double vrp = vector[row][p];
            double vrq = vector[row][q];

            vector[row][p] = cosine * vrp - sine * vrq;
            vector[row][q] = sine * vrp + cosine * vrq;
          }
      }
    }

  for (row = 0; row < 3; row++)
    {
      value[row] = a[row][row];

      if (!isfinite(value[row]))
        {
          return false;
        }
    }

  return true;
}

static bool matrix_eigen_range(FAR const float matrix[3][3],
                               FAR float *minimum, FAR float *maximum)
{
  double input[3][3];
  double value[3];
  double vector[3][3];
  int row;
  int column;

  for (row = 0; row < 3; row++)
    {
      for (column = 0; column < 3; column++)
        {
          input[row][column] = matrix[row][column];
        }
    }

  if (!symmetric_eigen(input, value, vector))
    {
      return false;
    }

  *minimum = (float)value[0];
  *maximum = (float)value[0];

  for (row = 1; row < 3; row++)
    {
      if (value[row] < *minimum)
        {
          *minimum = (float)value[row];
        }

      if (value[row] > *maximum)
        {
          *maximum = (float)value[row];
        }
    }

  return *minimum > 0.0f;
}

static void design_row(FAR const double u[3], FAR double d[9])
{
  d[0] = u[0] * u[0];
  d[1] = u[1] * u[1];
  d[2] = u[2] * u[2];
  d[3] = 2.0 * u[0] * u[1];
  d[4] = 2.0 * u[0] * u[2];
  d[5] = 2.0 * u[1] * u[2];
  d[6] = u[0];
  d[7] = u[1];
  d[8] = u[2];
}

/* Algebraic ellipsoid fit, normalized before forming the normal equations.
 * This supplies a deterministic initial state for the geometric refinement.
 */

static bool algebraic_fit(FAR const struct cal_mag_s *cal,
                          FAR const bool include[CAL_MAG_MAX_SAMPLES],
                          FAR struct cal_mag_fit_s *fit)
{
  double normal[9 * 9] = {0.0};
  double right[9] = {0.0};
  double solution[9] = {0.0};
  double mean[3] = {0.0};
  double scale_sum = 0.0;
  double scale;
  double shape[3][3];
  double inverse[3][3];
  double center[3] = {0.0};
  double q[3][3];
  double eigenvalue[3];
  double eigenvector[3][3];
  double determinant;
  double beta = 1.0;
  int used = 0;
  int sample;
  int row;
  int column;

  for (sample = 0; sample < cal->count; sample++)
    {
      if (include != NULL && !include[sample])
        {
          continue;
        }

      for (row = 0; row < 3; row++)
        {
          mean[row] += cal->samples[sample][row];
        }

      used++;
    }

  if (used < CAL_MAG_MIN_SAMPLES)
    {
      return false;
    }

  for (row = 0; row < 3; row++)
    {
      mean[row] /= used;
    }

  for (sample = 0; sample < cal->count; sample++)
    {
      if (include != NULL && !include[sample])
        {
          continue;
        }

      for (row = 0; row < 3; row++)
        {
          double delta = cal->samples[sample][row] - mean[row];

          scale_sum += delta * delta;
        }
    }

  scale = sqrt(scale_sum / used);

  if (!(scale > 0.05) || !isfinite(scale))
    {
      return false;
    }

  for (sample = 0; sample < cal->count; sample++)
    {
      double u[3];
      double d[9];

      if (include != NULL && !include[sample])
        {
          continue;
        }

      for (row = 0; row < 3; row++)
        {
          u[row] = (cal->samples[sample][row] - mean[row]) / scale;
        }

      design_row(u, d);

      for (row = 0; row < 9; row++)
        {
          right[row] += d[row];

          for (column = 0; column < 9; column++)
            {
              normal[row * 9 + column] += d[row] * d[column];
            }
        }
    }

  if (!solve_linear(normal, right, solution, 9))
    {
      return false;
    }

  shape[0][0] = solution[0];
  shape[1][1] = solution[1];
  shape[2][2] = solution[2];
  shape[0][1] = shape[1][0] = solution[3];
  shape[0][2] = shape[2][0] = solution[4];
  shape[1][2] = shape[2][1] = solution[5];

  if (!inverse_3x3(shape, inverse))
    {
      return false;
    }

  for (row = 0; row < 3; row++)
    {
      for (column = 0; column < 3; column++)
        {
          center[row] -= 0.5 * inverse[row][column] *
                         solution[6 + column];
        }
    }

  for (row = 0; row < 3; row++)
    {
      for (column = 0; column < 3; column++)
        {
          beta += center[row] * shape[row][column] * center[column];
        }
    }

  if (!(beta > 0.0) || !isfinite(beta))
    {
      return false;
    }

  for (row = 0; row < 3; row++)
    {
      for (column = 0; column < 3; column++)
        {
          q[row][column] = shape[row][column] / beta;
        }
    }

  if (!symmetric_eigen(q, eigenvalue, eigenvector) ||
      eigenvalue[0] <= 0.0 || eigenvalue[1] <= 0.0 ||
      eigenvalue[2] <= 0.0)
    {
      return false;
    }

  determinant = eigenvalue[0] * eigenvalue[1] * eigenvalue[2];
  fit->field = (float)(scale * pow(determinant, -1.0 / 6.0));

  for (row = 0; row < 3; row++)
    {
      fit->offset[row] = (float)(mean[row] + scale * center[row]);

      for (column = 0; column < 3; column++)
        {
          double value = 0.0;
          int axis;

          for (axis = 0; axis < 3; axis++)
            {
              value += eigenvector[row][axis] *
                       sqrt(eigenvalue[axis]) *
                       eigenvector[column][axis];
            }

          fit->matrix[row][column] =
            (float)(fit->field * value / scale);
        }
    }

  return vector_finite(fit->offset) && isfinite(fit->field);
}

static double sample_error(FAR const struct cal_mag_fit_s *fit,
                           FAR const float sample[3])
{
  float corrected[3];

  cal_mag_apply(fit, sample, corrected);
  return vector_norm(corrected) - fit->field;
}

static void sort_values(FAR float *value, int count)
{
  int index;

  for (index = 1; index < count; index++)
    {
      float item = value[index];
      int position = index;

      while (position > 0 && value[position - 1] > item)
        {
          value[position] = value[position - 1];
          position--;
        }

      value[position] = item;
    }
}

/* Remove isolated disturbances using median absolute deviation. The floor is
 * above IST8310 noise, so clean data is not trimmed merely for being clean.
 */

static int robust_mask(FAR const struct cal_mag_s *cal,
                       FAR const struct cal_mag_fit_s *fit,
                       FAR bool include[CAL_MAG_MAX_SAMPLES])
{
  float error[CAL_MAG_MAX_SAMPLES];
  float deviation[CAL_MAG_MAX_SAMPLES];
  float median;
  float threshold;
  int used = 0;
  int sample;

  for (sample = 0; sample < cal->count; sample++)
    {
      error[sample] = (float)fabs(sample_error(fit,
                                              cal->samples[sample]));
      deviation[sample] = error[sample];
    }

  sort_values(deviation, cal->count);
  median = deviation[cal->count / 2];

  for (sample = 0; sample < cal->count; sample++)
    {
      deviation[sample] = fabsf(error[sample] - median);
    }

  sort_values(deviation, cal->count);
  threshold = median + 4.0f * 1.4826f * deviation[cal->count / 2];

  if (threshold < 0.015f)
    {
      threshold = 0.015f;
    }

  for (sample = 0; sample < cal->count; sample++)
    {
      include[sample] = error[sample] <= threshold;
      used += include[sample] ? 1 : 0;
    }

  return used;
}

static void matrix_parameters(FAR const float matrix[3][3],
                              FAR float parameter[6])
{
  parameter[0] = matrix[0][0];
  parameter[1] = matrix[1][1];
  parameter[2] = matrix[2][2];
  parameter[3] = matrix[0][1];
  parameter[4] = matrix[0][2];
  parameter[5] = matrix[1][2];
}

static void parameters_matrix(FAR const float parameter[6],
                              FAR float matrix[3][3])
{
  matrix[0][0] = parameter[0];
  matrix[1][1] = parameter[1];
  matrix[2][2] = parameter[2];
  matrix[0][1] = matrix[1][0] = parameter[3];
  matrix[0][2] = matrix[2][0] = parameter[4];
  matrix[1][2] = matrix[2][1] = parameter[5];
}

static double fit_cost(FAR const struct cal_mag_s *cal,
                       FAR const bool include[CAL_MAG_MAX_SAMPLES],
                       FAR const struct cal_mag_fit_s *fit)
{
  double cost = 0.0;
  int sample;

  for (sample = 0; sample < cal->count; sample++)
    {
      if (include[sample])
        {
          double residual = sample_error(fit, cal->samples[sample]);

          cost += residual * residual;
        }
    }

  return cost;
}

/* Geometric Gauss-Newton refinement minimizes radial error, unlike the
 * algebraic initializer. Field magnitude remains fixed, removing the scale
 * ambiguity inherent in a norm-only tumble.
 */

static void refine_fit(FAR const struct cal_mag_s *cal,
                       FAR const bool include[CAL_MAG_MAX_SAMPLES],
                       FAR struct cal_mag_fit_s *fit)
{
  int iteration;

  for (iteration = 0; iteration < CAL_MAG_REFINE_STEPS; iteration++)
    {
      double normal[9 * 9] = {0.0};
      double right[9] = {0.0};
      double delta[9] = {0.0};
      double old_cost = fit_cost(cal, include, fit);
      int sample;
      int row;
      int column;
      bool accepted = false;

      for (sample = 0; sample < cal->count; sample++)
        {
          float v[3];
          float w[3] = {0.0f, 0.0f, 0.0f};
          double jacobian[9];
          double norm;
          double residual;
          int axis;

          if (!include[sample])
            {
              continue;
            }

          for (axis = 0; axis < 3; axis++)
            {
              v[axis] = cal->samples[sample][axis] - fit->offset[axis];

              for (column = 0; column < 3; column++)
                {
                  w[axis] += fit->matrix[axis][column] * v[column];
                }
            }

          norm = vector_norm(w);

          if (!(norm > 1.0e-9))
            {
              continue;
            }

          residual = norm - fit->field;

          for (axis = 0; axis < 3; axis++)
            {
              jacobian[axis] = 0.0;

              for (row = 0; row < 3; row++)
                {
                  jacobian[axis] -= (w[row] / norm) *
                                    fit->matrix[row][axis];
                }
            }

          jacobian[3] = w[0] * v[0] / norm;
          jacobian[4] = w[1] * v[1] / norm;
          jacobian[5] = w[2] * v[2] / norm;
          jacobian[6] = (w[0] * v[1] + w[1] * v[0]) / norm;
          jacobian[7] = (w[0] * v[2] + w[2] * v[0]) / norm;
          jacobian[8] = (w[1] * v[2] + w[2] * v[1]) / norm;

          for (row = 0; row < 9; row++)
            {
              right[row] -= jacobian[row] * residual;

              for (column = 0; column < 9; column++)
                {
                  normal[row * 9 + column] +=
                    jacobian[row] * jacobian[column];
                }
            }
        }

      for (row = 0; row < 9; row++)
        {
          normal[row * 9 + row] +=
            1.0e-5 * (normal[row * 9 + row] + 1.0);
        }

      if (!solve_linear(normal, right, delta, 9))
        {
          break;
        }

      for (row = 0; row < 6; row++)
        {
          struct cal_mag_fit_s trial = *fit;
          float parameter[6];
          float minimum;
          float maximum;
          double step = ldexp(1.0, -row);
          double trial_cost;
          int axis;

          for (axis = 0; axis < 3; axis++)
            {
              trial.offset[axis] += (float)(step * delta[axis]);
            }

          matrix_parameters(fit->matrix, parameter);

          for (axis = 0; axis < 6; axis++)
            {
              parameter[axis] += (float)(step * delta[3 + axis]);
            }

          parameters_matrix(parameter, trial.matrix);

          if (!matrix_eigen_range(trial.matrix, &minimum, &maximum) ||
              minimum < 0.10f || maximum > 10.0f)
            {
              continue;
            }

          trial_cost = fit_cost(cal, include, &trial);

          if (isfinite(trial_cost) && trial_cost < old_cost)
            {
              *fit = trial;
              accepted = true;
              break;
            }
        }

      if (!accepted)
        {
          break;
        }
    }
}

static bool coverage_ok(FAR const struct cal_mag_s *cal,
                        FAR const bool include[CAL_MAG_MAX_SAMPLES],
                        FAR struct cal_mag_fit_s *fit)
{
  float minimum[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
  float maximum[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
  uint8_t octants = 0;
  int sample;
  int axis;

  for (sample = 0; sample < cal->count; sample++)
    {
      float corrected[3];
      double norm;
      int octant = 0;

      if (!include[sample])
        {
          continue;
        }

      cal_mag_apply(fit, cal->samples[sample], corrected);
      norm = vector_norm(corrected);

      if (!(norm > 1.0e-9))
        {
          continue;
        }

      for (axis = 0; axis < 3; axis++)
        {
          float direction = (float)(corrected[axis] / norm);

          if (direction < minimum[axis])
            {
              minimum[axis] = direction;
            }

          if (direction > maximum[axis])
            {
              maximum[axis] = direction;
            }

          if (direction >= 0.0f)
            {
              octant |= 1 << axis;
            }
        }

      octants |= (uint8_t)(1u << octant);
    }

  fit->octants = octants;

  if (octants != 0xff)
    {
      return false;
    }

  for (axis = 0; axis < 3; axis++)
    {
      if (minimum[axis] > -CAL_MAG_POLE_COS ||
          maximum[axis] < CAL_MAG_POLE_COS)
        {
          return false;
        }
    }

  return true;
}

static void residual_statistics(FAR const struct cal_mag_s *cal,
                                FAR const bool include[CAL_MAG_MAX_SAMPLES],
                                FAR struct cal_mag_fit_s *fit)
{
  double sum = 0.0;
  double maximum = 0.0;
  int used = 0;
  int sample;

  for (sample = 0; sample < cal->count; sample++)
    {
      if (include[sample])
        {
          double error = fabs(sample_error(fit, cal->samples[sample]));

          sum += error * error;
          maximum = error > maximum ? error : maximum;
          used++;
        }
    }

  fit->used = (uint16_t)used;
  fit->rejected = (uint16_t)(cal->count - used);
  fit->rms = used > 0 ? (float)sqrt(sum / used) : INFINITY;
  fit->maximum_error = (float)maximum;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void cal_mag_reset(FAR struct cal_mag_s *cal)
{
  int axis;

  memset(cal, 0, sizeof(*cal));

  for (axis = 0; axis < 3; axis++)
    {
      cal->minimum[axis] = FLT_MAX;
      cal->maximum[axis] = -FLT_MAX;
    }
}

bool cal_mag_add(FAR struct cal_mag_s *cal, FAR const float sample[3])
{
  const float separation2 =
    CAL_MAG_MIN_SEPARATION * CAL_MAG_MIN_SEPARATION;
  double norm;
  int stored;
  int axis;

  cal->seen++;

  if (!vector_finite(sample) || cal->count >= CAL_MAG_MAX_SAMPLES)
    {
      return false;
    }

  norm = vector_norm(sample);

  if (norm < 0.02 || norm > CAL_MAG_RAW_MAX)
    {
      return false;
    }

  for (stored = 0; stored < cal->count; stored++)
    {
      float distance2 = 0.0f;

      for (axis = 0; axis < 3; axis++)
        {
          float delta = sample[axis] - cal->samples[stored][axis];

          distance2 += delta * delta;
        }

      if (distance2 < separation2)
        {
          return false;
        }
    }

  for (axis = 0; axis < 3; axis++)
    {
      cal->samples[cal->count][axis] = sample[axis];

      if (sample[axis] < cal->minimum[axis])
        {
          cal->minimum[axis] = sample[axis];
        }

      if (sample[axis] > cal->maximum[axis])
        {
          cal->maximum[axis] = sample[axis];
        }
    }

  cal->count++;
  return true;
}

enum cal_mag_result_e cal_mag_solve(FAR const struct cal_mag_s *cal,
                                    FAR struct cal_mag_fit_s *fit)
{
  bool include[CAL_MAG_MAX_SAMPLES];
  float minimum;
  float maximum;
  int sample;
  int axis;

  memset(fit, 0, sizeof(*fit));

  if (cal->count < CAL_MAG_MIN_SAMPLES)
    {
      return CAL_MAG_NEED_SAMPLES;
    }

  for (sample = 0; sample < cal->count; sample++)
    {
      include[sample] = true;
    }

  if (!algebraic_fit(cal, include, fit))
    {
      return CAL_MAG_SINGULAR;
    }

  if (robust_mask(cal, fit, include) < CAL_MAG_MIN_SAMPLES ||
      !algebraic_fit(cal, include, fit))
    {
      return CAL_MAG_SINGULAR;
    }

  refine_fit(cal, include, fit);
  residual_statistics(cal, include, fit);

  if (!coverage_ok(cal, include, fit))
    {
      return CAL_MAG_POOR_COVERAGE;
    }

  if (!(fit->field >= CAL_MAG_FIELD_MIN &&
        fit->field <= CAL_MAG_FIELD_MAX))
    {
      return CAL_MAG_FIELD_RANGE;
    }

  for (axis = 0; axis < 3; axis++)
    {
      if (!(fabsf(fit->offset[axis]) <= CAL_MAG_OFFSET_MAX))
        {
          return CAL_MAG_OFFSET_RANGE;
        }
    }

  if (!matrix_eigen_range(fit->matrix, &minimum, &maximum))
    {
      return CAL_MAG_NOT_ELLIPSOID;
    }

  fit->condition = maximum / minimum;

  if (!(minimum >= CAL_MAG_EIGEN_MIN && maximum <= CAL_MAG_EIGEN_MAX &&
        fit->condition <= CAL_MAG_CONDITION_MAX))
    {
      return CAL_MAG_SCALE_RANGE;
    }

  if (!(fit->rms <= CAL_MAG_RMS_MAX &&
        fit->rms <= CAL_MAG_REL_RMS_MAX * fit->field &&
        fit->maximum_error <= CAL_MAG_MAX_ERROR))
    {
      return CAL_MAG_RESIDUAL_HIGH;
    }

  return CAL_MAG_OK;
}

void cal_mag_apply(FAR const struct cal_mag_fit_s *fit,
                   FAR const float raw[3], FAR float corrected[3])
{
  float centered[3];
  int row;
  int column;

  for (row = 0; row < 3; row++)
    {
      centered[row] = raw[row] - fit->offset[row];
    }

  for (row = 0; row < 3; row++)
    {
      corrected[row] = 0.0f;

      for (column = 0; column < 3; column++)
        {
          corrected[row] += fit->matrix[row][column] * centered[column];
        }
    }
}

FAR const char *cal_mag_result_string(enum cal_mag_result_e result)
{
  switch (result)
    {
      case CAL_MAG_OK:
        return "ok";
      case CAL_MAG_NEED_SAMPLES:
        return "need more samples";
      case CAL_MAG_POOR_COVERAGE:
        return "poor 3D coverage";
      case CAL_MAG_SINGULAR:
        return "singular fit";
      case CAL_MAG_NOT_ELLIPSOID:
        return "fit is not an ellipsoid";
      case CAL_MAG_FIELD_RANGE:
        return "field strength out of range";
      case CAL_MAG_OFFSET_RANGE:
        return "offset out of range";
      case CAL_MAG_SCALE_RANGE:
        return "soft-iron matrix out of range";
      case CAL_MAG_RESIDUAL_HIGH:
        return "fit residual too high";
      default:
        return "unknown fit error";
    }
}
