/****************************************************************************
 * tests/dsp_filter_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "dsp_filter.h"

#define TEST_PI_F 3.14159265358979323846f

static float response(FAR struct dsp_biquad3_s *filter, float sample_rate,
                      float frequency)
{
  const int total = 16000;
  const int settle = 4000;
  float in_energy = 0.0f;
  float out_energy = 0.0f;
  int i;

  for (i = 0; i < total; i++)
    {
      float input = sinf(2.0f * TEST_PI_F * frequency * i / sample_rate);
      float value[3] = {input, input, input};

      dsp_biquad3_apply(filter, value);

      if (i >= settle)
        {
          in_energy += input * input;
          out_energy += value[0] * value[0];
        }
    }

  return sqrtf(out_energy / in_energy);
}

int main(void)
{
  struct dsp_biquad3_s filter;
  float value[3];
  float gain;

  memset(&filter, 0, sizeof(filter));
  assert(dsp_biquad3_lowpass(&filter, 2000.0f, 100.0f));

  value[0] = 1.0f;
  value[1] = -2.0f;
  value[2] = 9.8f;
  dsp_biquad3_reset(&filter, value);
  dsp_biquad3_apply(&filter, value);
  assert(fabsf(value[0] - 1.0f) < 1e-5f);
  assert(fabsf(value[1] + 2.0f) < 1e-5f);
  assert(fabsf(value[2] - 9.8f) < 1e-4f);

  assert(dsp_biquad3_lowpass(&filter, 2000.0f, 100.0f));
  gain = response(&filter, 2000.0f, 100.0f);
  assert(gain > 0.69f && gain < 0.72f);

  assert(dsp_biquad3_lowpass(&filter, 2000.0f, 100.0f));
  gain = response(&filter, 2000.0f, 500.0f);
  assert(gain < 0.04f);

  assert(dsp_biquad3_notch(&filter, 2000.0f, 300.0f, 20.0f));
  gain = response(&filter, 2000.0f, 300.0f);
  assert(gain < 0.01f);

  assert(dsp_biquad3_notch(&filter, 2000.0f, 0.0f, 20.0f));
  value[0] = 1.0f;
  value[1] = 2.0f;
  value[2] = 3.0f;
  dsp_biquad3_apply(&filter, value);
  assert(value[0] == 1.0f && value[1] == 2.0f && value[2] == 3.0f);

  assert(!dsp_biquad3_lowpass(&filter, 2000.0f, 1000.0f));
  assert(!dsp_biquad3_notch(&filter, 2000.0f, 300.0f, 300.0f));

  puts("dsp filter tests: PASS");
  return 0;
}
