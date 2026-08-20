/****************************************************************************
 * apps/cpu_status/cpu_audit.c
 *
 * Exact per-thread runtime sampling using the scheduler's DWT cycle totals.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "cpu_audit.h"
#include "cpu_runtime.h"

struct cpu_audit_snapshot_s
{
  pid_t pid;
  uint8_t priority;
  uint32_t runtime;
  char name[CONFIG_TASK_NAME_SIZE + 1];
};

struct cpu_audit_capture_s
{
  struct cpu_audit_snapshot_s task[CPU_AUDIT_MAX_THREADS];
  size_t count;
  bool truncated;
};

static void cpu_audit_capture_task(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct cpu_audit_capture_s *capture = arg;
  FAR struct cpu_audit_snapshot_s *snapshot;
  uint32_t runtime;

  if (capture->count >= CPU_AUDIT_MAX_THREADS)
    {
      capture->truncated = true;
      return;
    }

  snapshot = &capture->task[capture->count++];
  runtime = (uint32_t)tcb->run_time;

  /* run_time is committed when a task switches out. Include the current
   * slice so every captured task has the same snapshot boundary.
   */

  if (tcb->task_state == TSTATE_TASK_RUNNING)
    {
      runtime += (uint32_t)up_perf_gettime() - (uint32_t)tcb->run_start;
    }

  snapshot->pid = tcb->pid;
  snapshot->priority = tcb->sched_priority;
  snapshot->runtime = runtime;
  strlcpy(snapshot->name, get_task_name(tcb), sizeof(snapshot->name));
}

static void cpu_audit_capture(struct cpu_audit_capture_s *capture)
{
  memset(capture, 0, sizeof(*capture));
  nxsched_foreach(cpu_audit_capture_task, capture);
}

static uint64_t cpu_audit_monotonic_us(void)
{
  struct timespec now;

  if (clock_gettime(CLOCK_MONOTONIC, &now) < 0)
    {
      return 0;
    }

  return (uint64_t)now.tv_sec * 1000000ull +
         (uint64_t)now.tv_nsec / 1000ull;
}

static FAR struct cpu_audit_entry_s *
cpu_audit_find_active(struct cpu_audit_report_s *report,
                      FAR const struct cpu_audit_snapshot_s *snapshot)
{
  size_t i;

  for (i = 0; i < report->entry_count; i++)
    {
      FAR struct cpu_audit_entry_s *entry = &report->entry[i];

      if (entry->active && entry->pid == snapshot->pid &&
          strcmp(entry->name, snapshot->name) == 0)
        {
          return entry;
        }
    }

  return NULL;
}

static FAR struct cpu_audit_entry_s *
cpu_audit_new_entry(struct cpu_audit_report_s *report,
                    FAR const struct cpu_audit_snapshot_s *snapshot,
                    bool baseline, uint64_t max_delta)
{
  FAR struct cpu_audit_entry_s *entry;

  if (report->entry_count >= CPU_AUDIT_MAX_THREADS)
    {
      report->truncated = true;
      return NULL;
    }

  entry = &report->entry[report->entry_count++];
  memset(entry, 0, sizeof(*entry));
  entry->pid = snapshot->pid;
  entry->priority = snapshot->priority;
  entry->previous = snapshot->runtime;
  entry->active = true;
  entry->seen = true;
  strlcpy(entry->name, snapshot->name, sizeof(entry->name));

  /* A task first seen after the baseline was created during this window, so
   * its lifetime runtime belongs to the interval. Reject an implausible value
   * in case a PID/name pair was recycled between two snapshots.
   */

  if (!baseline && snapshot->runtime <= max_delta)
    {
      entry->cycles = snapshot->runtime;
    }

  return entry;
}

static void cpu_audit_merge(struct cpu_audit_report_s *report,
                            FAR const struct cpu_audit_capture_s *capture,
                            bool baseline, uint64_t max_delta)
{
  size_t i;

  for (i = 0; i < report->entry_count; i++)
    {
      report->entry[i].seen = false;
    }

  for (i = 0; i < capture->count; i++)
    {
      FAR const struct cpu_audit_snapshot_s *snapshot = &capture->task[i];
      FAR struct cpu_audit_entry_s *entry =
        cpu_audit_find_active(report, snapshot);

      if (entry != NULL)
        {
          uint32_t delta = snapshot->runtime - entry->previous;

          /* Unsigned subtraction handles a real 32-bit DWT wrap. A single
           * thread cannot consume more cycles than elapsed wall time; a larger
           * delta means the runtime counter was reset or the PID was reused.
           */

          if ((uint64_t)delta <= max_delta)
            {
              entry->cycles += delta;
              entry->previous = snapshot->runtime;
              entry->priority = snapshot->priority;
              entry->seen = true;
              continue;
            }

          entry->active = false;
        }

      cpu_audit_new_entry(report, snapshot, baseline, max_delta);
    }

  for (i = 0; i < report->entry_count; i++)
    {
      if (report->entry[i].active && !report->entry[i].seen)
        {
          report->entry[i].active = false;
        }
    }

  report->truncated |= capture->truncated;
}

static int cpu_audit_compare(FAR const void *lhs, FAR const void *rhs)
{
  FAR const struct cpu_audit_entry_s *a = lhs;
  FAR const struct cpu_audit_entry_s *b = rhs;

  if (a->cycles < b->cycles)
    {
      return 1;
    }

  if (a->cycles > b->cycles)
    {
      return -1;
    }

  return a->pid > b->pid ? 1 : a->pid < b->pid ? -1 : 0;
}

int cpu_audit_measure(int duration_ms, struct cpu_audit_report_s *report)
{
  FAR struct cpu_audit_capture_s *capture;
  uint64_t wall_start_us;
  uint64_t previous_us;
  int elapsed_ms = 0;

  report->frequency = (uint32_t)up_perf_getfreq();
  report->requested_ms = duration_ms;
  if (duration_ms <= 0 || report->frequency == 0)
    {
      return -1;
    }

  capture = report->scratch;
  if (report->entry == NULL)
    {
      report->entry = calloc(CPU_AUDIT_MAX_THREADS, sizeof(*report->entry));
    }

  if (capture == NULL)
    {
      capture = calloc(1, sizeof(*capture));
      report->scratch = capture;
    }

  if (capture == NULL || report->entry == NULL)
    {
      cpu_audit_destroy(report);
      return -1;
    }

  memset(capture, 0, sizeof(*capture));
  memset(report->entry, 0,
         CPU_AUDIT_MAX_THREADS * sizeof(*report->entry));
  report->entry_count = 0;
  report->wall_cycles = 0;
  report->wall_us = 0;
  report->truncated = false;

  wall_start_us = cpu_audit_monotonic_us();
  previous_us = wall_start_us;
  cpu_audit_capture(capture);
  cpu_audit_merge(report, capture, true, 0);

  while (elapsed_ms < duration_ms)
    {
      uint64_t now_us;
      uint64_t interval_cycles;
      uint64_t max_delta;
      int interval_ms = duration_ms - elapsed_ms;

      if (interval_ms > CPU_AUDIT_SAMPLE_MS)
        {
          interval_ms = CPU_AUDIT_SAMPLE_MS;
        }

      usleep((useconds_t)interval_ms * 1000u);
      elapsed_ms += interval_ms;
      now_us = cpu_audit_monotonic_us();
      interval_cycles = (now_us - previous_us) * report->frequency /
                        1000000ull;

      /* Allow 5% for clock quantization and snapshot overhead. This is still
       * far below the false ~2^32-cycle delta produced by a reset counter.
       */

      max_delta = interval_cycles + interval_cycles / 20u +
                  report->frequency / 1000u;
      cpu_audit_capture(capture);
      cpu_audit_merge(report, capture, false, max_delta);
      previous_us = now_us;
    }

  report->wall_us = cpu_audit_monotonic_us() - wall_start_us;
  report->wall_cycles = report->wall_us * report->frequency / 1000000ull;
  return 0;
}

void cpu_audit_print(struct cpu_audit_report_s *report, bool verbose)
{
  uint64_t idle_cycles = 0;
  uint64_t accounted_cycles = 0;
  uint64_t busy_cycles;
  uint64_t residual;
  uint32_t busy_tenths;
  uint32_t idle_tenths;
  uint32_t residual_tenths;
  size_t i;

  qsort(report->entry, report->entry_count, sizeof(*report->entry),
        cpu_audit_compare);

  for (i = 0; i < report->entry_count; i++)
    {
      accounted_cycles += report->entry[i].cycles;
      if (report->entry[i].pid == 0)
        {
          idle_cycles += report->entry[i].cycles;
        }
    }

  if (idle_cycles > accounted_cycles)
    {
      idle_cycles = accounted_cycles;
    }

  busy_cycles = accounted_cycles - idle_cycles;
  if (busy_cycles > report->wall_cycles)
    {
      busy_cycles = report->wall_cycles;
    }

  busy_tenths = cpu_runtime_tenths_percent(busy_cycles,
                                            report->wall_cycles);
  idle_tenths = 1000u - busy_tenths;
  residual = accounted_cycles >= report->wall_cycles ?
             accounted_cycles - report->wall_cycles :
             report->wall_cycles - accounted_cycles;
  residual_tenths = cpu_runtime_tenths_percent(residual,
                                                report->wall_cycles);

  if (verbose)
    {
      printf("CPU cycle audit: %d ms, counter=%" PRIu32
             " Hz, sample=%d ms\n",
             report->requested_ms, report->frequency, CPU_AUDIT_SAMPLE_MS);
    }

  printf("CPU true: busy=%" PRIu32 ".%01" PRIu32
         "%% idle=%" PRIu32 ".%01" PRIu32
         "%% window=%" PRIu64 "us residual=%c%" PRIu32 ".%01" PRIu32
         "%%\n",
         busy_tenths / 10, busy_tenths % 10,
         idle_tenths / 10, idle_tenths % 10,
         report->wall_us, accounted_cycles >= report->wall_cycles ? '+' : '-',
         residual_tenths / 10, residual_tenths % 10);
  printf("PID  PRI  CPU       RUNTIME_US  STATE   THREAD\n");

  if (verbose)
    {
      printf("note: WFI may appear in residual; IRQ time is charged to the "
             "interrupted thread\n");
    }

  for (i = 0; i < report->entry_count; i++)
    {
      FAR struct cpu_audit_entry_s *entry = &report->entry[i];
      uint32_t percent = cpu_runtime_tenths_percent(entry->cycles,
                                                    report->wall_cycles);

      if (entry->cycles == 0)
        {
          continue;
        }

      printf("%3d  %3u  %3" PRIu32 ".%01" PRIu32
             "%%  %10" PRIu64 "  %-7s %s%s\n",
             entry->pid, entry->priority,
             percent / 10, percent % 10,
             cpu_runtime_cycles_to_us(entry->cycles, report->frequency),
             entry->active ? "active" : "exited",
             entry->name, entry->pid == 0 ? " [idle]" : "");
    }

  if (report->truncated)
    {
      printf("warning: task generations exceeded %d entries; table is "
             "incomplete\n", CPU_AUDIT_MAX_THREADS);
    }
}

void cpu_audit_destroy(struct cpu_audit_report_s *report)
{
  free(report->entry);
  free(report->scratch);
  memset(report, 0, sizeof(*report));
}
