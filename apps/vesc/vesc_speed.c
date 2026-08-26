/****************************************************************************
 * apps/vesc/vesc_speed.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <math.h>
#include <string.h>

#include "vesc_speed.h"

#define VESC_SPEED_PI 3.14159265358979323846f

void vesc_speed_init(FAR struct vesc_speed_s *f, float nominal_hz,
                     float cutoff_hz)
{
  if (f == NULL)
    {
      return;
    }

  memset(f, 0, sizeof(*f));

  if (!(nominal_hz > 0.0f) || !isfinite(nominal_hz))
    {
      nominal_hz = 400.0f;
    }

  f->dt_nominal = 1.0f / nominal_hz;
  f->dt_ema = f->dt_nominal;
  f->cutoff_hz = (cutoff_hz > 0.0f && isfinite(cutoff_hz)) ? cutoff_hz : 0.0f;
}

void vesc_speed_reset(FAR struct vesc_speed_s *f)
{
  if (f != NULL)
    {
      vesc_speed_init(f, 1.0f / f->dt_nominal, f->cutoff_hz);
    }
}

float vesc_speed_update(FAR struct vesc_speed_s *f, int32_t tachometer,
                        uint64_t timestamp_us)
{
  int32_t delta;
  float raw_dt;
  float fs;
  float cutoff;
  float tau;
  float alpha;
  float rate;

  if (f == NULL)
    {
      return 0.0f;
    }

  /* The first reading establishes a reference and nothing else. The same
   * path handles a gap long enough that the count can no longer be related
   * to the previous one.
   */

  if (!f->primed || timestamp_us <= f->last_us ||
      timestamp_us - f->last_us > VESC_SPEED_MAX_GAP_US)
    {
      f->primed = true;
      f->last_tach = tachometer;
      f->last_us = timestamp_us;
      f->dt_ema = f->dt_nominal;
      f->stage[0] = 0.0f;
      f->stage[1] = 0.0f;
      f->value = 0.0f;
      return 0.0f;
    }

  /* Unsigned subtraction read back as signed. The tachometer is a 32-bit
   * accumulator and it wraps, but a finite difference across the wrap is
   * still the correct small step - exactly as it would be for an angle. The
   * signed difference of the raw values is what would be wrong.
   */

  delta = (int32_t)((uint32_t)tachometer - (uint32_t)f->last_tach);
  raw_dt = (float)(timestamp_us - f->last_us) * 1.0e-6f;

  f->last_tach = tachometer;
  f->last_us = timestamp_us;

  /* Average the interval rather than using the one just measured.
   *
   * The count is an integer, so at 400 Hz a single interval carries only a
   * few counts and dividing by a jittery dt multiplies the quantisation by
   * the timing noise. The average is the real telemetry period; outliers are
   * refused rather than averaged in, so one late frame cannot drag it.
   */

  if (raw_dt > f->dt_nominal * VESC_SPEED_DT_MIN_RATIO &&
      raw_dt < f->dt_nominal * VESC_SPEED_DT_MAX_RATIO)
    {
      f->dt_ema += VESC_SPEED_DT_BETA * (raw_dt - f->dt_ema);
    }

  rate = (float)delta / f->dt_ema;
  fs = 1.0f / f->dt_ema;

  if (f->cutoff_hz <= 0.0f)
    {
      f->stage[0] = rate;
      f->stage[1] = rate;
      f->value = rate;
      return f->value;
    }

  cutoff = f->cutoff_hz;

  if (cutoff > VESC_SPEED_MAX_FS_FRACTION * fs)
    {
      cutoff = VESC_SPEED_MAX_FS_FRACTION * fs;
    }

  tau = 1.0f / (2.0f * VESC_SPEED_PI * cutoff);
  alpha = f->dt_ema / (tau + f->dt_ema);

  /* Two one-pole sections in series: -40 dB/decade. This is the ANTI-ALIAS
   * filter for the 200 Hz downlink, which samples this 400 Hz stream at half
   * rate - without it, everything between 100 and 200 Hz folds down into the
   * band the consumer cares about.
   */

  f->stage[0] += alpha * (rate - f->stage[0]);
  f->stage[1] += alpha * (f->stage[0] - f->stage[1]);
  f->value = f->stage[1];
  return f->value;
}
