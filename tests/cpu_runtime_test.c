/****************************************************************************
 * tests/cpu_runtime_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "cpu_runtime.h"

static void test_accumulation_and_wrap(void)
{
  struct cpu_runtime_counter_s counter = {0};

  assert(cpu_runtime_update(&counter, 100) == 0);
  assert(cpu_runtime_update(&counter, 160) == 60);
  assert(counter.cycles == 60);

  counter.previous = UINT32_MAX - 9;
  assert(cpu_runtime_update(&counter, 20) == 30);
  assert(counter.cycles == 90);
}

static void test_conversions(void)
{
  assert(cpu_runtime_tenths_percent(1, 4) == 250);
  assert(cpu_runtime_tenths_percent(2, 3) == 667);
  assert(cpu_runtime_tenths_percent(1, 0) == 0);
  assert(cpu_runtime_cycles_to_us(480000000, 480000000) == 1000000);
  assert(cpu_runtime_cycles_to_us(240, 480000000) == 0);
  assert(cpu_runtime_cycles_to_us(100, 0) == 0);
}

int main(void)
{
  test_accumulation_and_wrap();
  test_conversions();
  puts("cpu_runtime_test: PASS");
  return 0;
}
