/****************************************************************************
 * apps/cpu_status/cpu_audit.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __APPS_CPU_STATUS_CPU_AUDIT_H
#define __APPS_CPU_STATUS_CPU_AUDIT_H

#include <nuttx/config.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#define CPU_AUDIT_MAX_THREADS 64
#define CPU_AUDIT_SAMPLE_MS   1000

struct cpu_audit_entry_s
{
  pid_t pid;
  uint8_t priority;
  uint64_t cycles;
  uint32_t previous;
  bool active;
  bool seen;
  char name[CONFIG_TASK_NAME_SIZE + 1];
};

struct cpu_audit_report_s
{
  struct cpu_audit_entry_s *entry;
  void *scratch;
  size_t entry_count;
  uint64_t wall_cycles;
  uint64_t wall_us;
  uint32_t frequency;
  int requested_ms;
  bool truncated;
};

int cpu_audit_measure(int duration_ms, struct cpu_audit_report_s *report);
void cpu_audit_print(struct cpu_audit_report_s *report, bool verbose);
void cpu_audit_destroy(struct cpu_audit_report_s *report);

#endif /* __APPS_CPU_STATUS_CPU_AUDIT_H */
