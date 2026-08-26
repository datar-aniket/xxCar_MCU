/****************************************************************************
 * tests/cpu_runtime_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "cpu_runtime.h"

/* The counter accumulation and 32-bit DWT wrap that used to be tested here
 * moved into cpu_audit.c when CPU load switched to hardware cycles
 * (9fd5a1c). It still exists - see the unsigned subtraction at
 * cpu_audit.c:164 - but it is now coupled to the snapshot and entry structs
 * and is NOT covered by any host test. This file stopped compiling at that
 * commit and the failure went unnoticed because it looked like an
 * environment problem rather than a stale test.
 *
 * Left as a note rather than a deleted function so the gap is visible.
 */

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
  test_conversions();
  puts("cpu_runtime_test: PASS");
  return 0;
}
