/****************************************************************************
 * apps/cpu_status/cpu_runtime.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <stdint.h>

#include "cpu_runtime.h"

uint32_t cpu_runtime_update(struct cpu_runtime_counter_s *counter,
                            uint32_t current)
{
  uint32_t delta = 0;

  if (counter->initialized)
    {
      /* Unsigned subtraction is deliberately modulo 2^32. Sampling faster
       * than one DWT wrap makes this correct across a counter rollover.
       */

      delta = current - counter->previous;
      counter->cycles += delta;
    }
  else
    {
      counter->initialized = true;
    }

  counter->previous = current;
  return delta;
}

uint32_t cpu_runtime_tenths_percent(uint64_t part, uint64_t whole)
{
  if (whole == 0)
    {
      return 0;
    }

  return (uint32_t)((part * 1000ull + whole / 2ull) / whole);
}

uint64_t cpu_runtime_cycles_to_us(uint64_t cycles, uint32_t frequency)
{
  if (frequency == 0)
    {
      return 0;
    }

  return cycles * 1000000ull / frequency;
}
