/****************************************************************************
 * Host checks for the MCU magnetometer candidate safety boundary.
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "cal_mag.h"

static struct cal_mag_fit_s valid_fit(void)
{
  struct cal_mag_fit_s fit;

  memset(&fit, 0, sizeof(fit));
  fit.offset[0] = 0.13f;
  fit.offset[1] = -0.08f;
  fit.offset[2] = 0.055f;
  fit.matrix[0][0] = 1.18f;
  fit.matrix[1][1] = 0.84f;
  fit.matrix[2][2] = 1.03f;
  fit.matrix[0][1] = fit.matrix[1][0] = 0.07f;
  fit.matrix[0][2] = fit.matrix[2][0] = -0.03f;
  fit.matrix[1][2] = fit.matrix[2][1] = 0.04f;
  fit.field = 0.47f;
  return fit;
}

int main(void)
{
  struct cal_mag_fit_s fit = valid_fit();
  float raw[3] = {0.51f, -0.22f, 0.19f};
  float corrected[3];

  assert(cal_mag_validate(&fit) == CAL_MAG_OK);
  cal_mag_apply(&fit, raw, corrected);
  assert(isfinite(corrected[0]) && isfinite(corrected[1]) &&
         isfinite(corrected[2]));

  fit = valid_fit();
  fit.field = NAN;
  assert(cal_mag_validate(&fit) == CAL_MAG_NONFINITE);
  fit = valid_fit();
  fit.field = 1.2f;
  assert(cal_mag_validate(&fit) == CAL_MAG_FIELD_RANGE);
  fit = valid_fit();
  fit.offset[1] = 1.6f;
  assert(cal_mag_validate(&fit) == CAL_MAG_OFFSET_RANGE);
  fit = valid_fit();
  fit.matrix[0][0] = -1.0f;
  assert(cal_mag_validate(&fit) == CAL_MAG_NOT_POSITIVE_DEFINITE);
  fit = valid_fit();
  fit.matrix[0][0] = 4.5f;
  assert(cal_mag_validate(&fit) == CAL_MAG_SCALE_RANGE);
  fit = valid_fit();
  fit.matrix[1][0] += 0.1f;
  assert(cal_mag_validate(&fit) == CAL_MAG_NOT_POSITIVE_DEFINITE);

  puts("cal_mag MCU: candidate validation and application verified - OK");
  return 0;
}
