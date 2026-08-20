/****************************************************************************
 * apps/cpu_status/cpu_runtime.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __APPS_CPU_STATUS_CPU_RUNTIME_H
#define __APPS_CPU_STATUS_CPU_RUNTIME_H

#include <stdint.h>

uint32_t cpu_runtime_tenths_percent(uint64_t part, uint64_t whole);
uint64_t cpu_runtime_cycles_to_us(uint64_t cycles, uint32_t frequency);

#endif /* __APPS_CPU_STATUS_CPU_RUNTIME_H */
