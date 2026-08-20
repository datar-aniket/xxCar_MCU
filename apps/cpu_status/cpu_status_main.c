/****************************************************************************
 * apps/cpu_status/cpu_status_main.c
 *
 * Exact task runtime audit using Cortex-M DWT cycles recorded by NuttX at
 * every scheduler context switch.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/sched.h>

#include "cpu_runtime.h"

#define CPU_STATUS_DEFAULT_MS       10000
#define CPU_STATUS_SAMPLE_MS        1000
#define CPU_STATUS_MIN_MS           1000
#define CPU_STATUS_MAX_MS           3600000
#define CPU_STATUS_MAX_THREADS      64

struct cpu_status_snapshot_s
{
  pid_t pid;
  uint8_t priority;
  uint32_t runtime;
  char name[CONFIG_TASK_NAME_SIZE + 1];
};

struct cpu_status_entry_s
{
  pid_t pid;
  uint8_t priority;
  struct cpu_runtime_counter_s runtime;
  char name[CONFIG_TASK_NAME_SIZE + 1];
};

struct cpu_status_capture_s
{
  struct cpu_status_snapshot_s task[CPU_STATUS_MAX_THREADS];
  size_t count;
  bool truncated;
};

static void cpu_status_capture_task(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct cpu_status_capture_s *capture = arg;
  FAR struct cpu_status_snapshot_s *snapshot;
  uint32_t runtime;

  if (capture->count >= CPU_STATUS_MAX_THREADS)
    {
      capture->truncated = true;
      return;
    }

  snapshot = &capture->task[capture->count++];
  runtime = (uint32_t)tcb->run_time;

  /* run_time is committed when a task switches out. Include the current
   * slice when the task is running so the snapshot has a common boundary.
   * Unsigned arithmetic also handles one DWT wrap within the slice.
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

static void cpu_status_capture(struct cpu_status_capture_s *capture)
{
  memset(capture, 0, sizeof(*capture));
  nxsched_foreach(cpu_status_capture_task, capture);
}

static uint64_t cpu_status_monotonic_us(void)
{
  struct timespec now;

  if (clock_gettime(CLOCK_MONOTONIC, &now) < 0)
    {
      return 0;
    }

  return (uint64_t)now.tv_sec * 1000000ull +
         (uint64_t)now.tv_nsec / 1000ull;
}

static FAR struct cpu_status_entry_s *
cpu_status_find(struct cpu_status_entry_s *entry, size_t count,
                FAR const struct cpu_status_snapshot_s *snapshot)
{
  size_t i;

  for (i = 0; i < count; i++)
    {
      if (entry[i].pid == snapshot->pid &&
          strcmp(entry[i].name, snapshot->name) == 0)
        {
          return &entry[i];
        }
    }

  return NULL;
}

static void cpu_status_merge(struct cpu_status_entry_s *entry,
                             size_t *entry_count,
                             FAR const struct cpu_status_capture_s *capture)
{
  size_t i;

  for (i = 0; i < capture->count; i++)
    {
      FAR const struct cpu_status_snapshot_s *snapshot = &capture->task[i];
      FAR struct cpu_status_entry_s *current =
        cpu_status_find(entry, *entry_count, snapshot);

      if (current == NULL)
        {
          if (*entry_count >= CPU_STATUS_MAX_THREADS)
            {
              continue;
            }

          current = &entry[(*entry_count)++];
          memset(current, 0, sizeof(*current));
          current->pid = snapshot->pid;
          current->priority = snapshot->priority;
          strlcpy(current->name, snapshot->name, sizeof(current->name));
        }

      current->priority = snapshot->priority;
      cpu_runtime_update(&current->runtime, snapshot->runtime);
    }
}

static int cpu_status_compare(FAR const void *lhs, FAR const void *rhs)
{
  FAR const struct cpu_status_entry_s *a = lhs;
  FAR const struct cpu_status_entry_s *b = rhs;

  if (a->runtime.cycles < b->runtime.cycles)
    {
      return 1;
    }

  if (a->runtime.cycles > b->runtime.cycles)
    {
      return -1;
    }

  return a->pid > b->pid ? 1 : a->pid < b->pid ? -1 : 0;
}

static void cpu_status_print(struct cpu_status_entry_s *entry,
                             size_t entry_count, uint64_t wall_cycles,
                             uint32_t frequency, int requested_ms,
                             bool truncated)
{
  uint64_t idle_cycles = 0;
  uint64_t accounted_cycles = 0;
  uint64_t busy_cycles;
  uint64_t residual;
  uint32_t busy_tenths;
  uint32_t idle_tenths;
  uint32_t residual_tenths;
  size_t i;

  qsort(entry, entry_count, sizeof(*entry), cpu_status_compare);

  for (i = 0; i < entry_count; i++)
    {
      accounted_cycles += entry[i].runtime.cycles;
      if (entry[i].pid == 0)
        {
          idle_cycles = entry[i].runtime.cycles;
        }
    }

  if (idle_cycles > accounted_cycles)
    {
      idle_cycles = accounted_cycles;
    }

  /* DWT can pause while the Cortex-M7 is in WFI. Therefore wall time is
   * deliberately supplied by CLOCK_MONOTONIC, while busy time is the sum of
   * actual non-idle execution cycles. This remains correct whether a specific
   * debug configuration leaves CYCCNT running or stops it during sleep.
   */

  busy_cycles = accounted_cycles - idle_cycles;
  if (busy_cycles > wall_cycles)
    {
      busy_cycles = wall_cycles;
    }

  busy_tenths = cpu_runtime_tenths_percent(busy_cycles, wall_cycles);
  idle_tenths = 1000u - busy_tenths;
  residual = accounted_cycles >= wall_cycles ?
             accounted_cycles - wall_cycles : wall_cycles - accounted_cycles;
  residual_tenths = cpu_runtime_tenths_percent(residual, wall_cycles);

  printf("CPU cycle audit: %d ms, counter=%" PRIu32 " Hz, sample=%d ms\n",
         requested_ms, frequency, CPU_STATUS_SAMPLE_MS);
  printf("CPU true: busy=%" PRIu32 ".%01" PRIu32
         "%% idle=%" PRIu32 ".%01" PRIu32
         "%% wall_cycles=%" PRIu64
         " accounting_residual=%c%" PRIu32 ".%01" PRIu32
         "%%\n",
         busy_tenths / 10, busy_tenths % 10,
         idle_tenths / 10, idle_tenths % 10,
         wall_cycles, accounted_cycles >= wall_cycles ? '+' : '-',
         residual_tenths / 10, residual_tenths % 10);
  printf("PID  PRI  CPU       RUNTIME_US  THREAD\n");
  printf("note: residual includes WFI if DWT pauses; IRQ time is charged "
         "to the interrupted thread\n");

  for (i = 0; i < entry_count; i++)
    {
      uint32_t percent = cpu_runtime_tenths_percent(
        entry[i].runtime.cycles, wall_cycles);

      if (entry[i].runtime.cycles == 0)
        {
          continue;
        }

      printf("%3d  %3u  %3" PRIu32 ".%01" PRIu32
             "%%  %10" PRIu64 "  %s%s\n",
             entry[i].pid, entry[i].priority,
             percent / 10, percent % 10,
             cpu_runtime_cycles_to_us(entry[i].runtime.cycles, frequency),
             entry[i].name, entry[i].pid == 0 ? " [idle]" : "");
    }

  if (truncated)
    {
      printf("warning: task list exceeded %d entries; "
             "per-thread table is incomplete\n",
             CPU_STATUS_MAX_THREADS);
    }
}

int main(int argc, FAR char *argv[])
{
  FAR struct cpu_status_capture_s *capture;
  FAR struct cpu_status_entry_s *entry;
  uint64_t wall_start_us;
  uint64_t wall_us;
  uint64_t wall_cycles;
  uint32_t frequency = (uint32_t)up_perf_getfreq();
  size_t entry_count = 0;
  int duration_ms = CPU_STATUS_DEFAULT_MS;
  int elapsed_ms = 0;
  int option;
  bool truncated = false;

  while ((option = getopt(argc, argv, "t:h")) != ERROR)
    {
      switch (option)
        {
          case 't':
            duration_ms = atoi(optarg);
            break;

          case 'h':
          default:
            printf("Usage: cpu_status [-t <ms>]\n"
                   "  hardware-cycle audit, %d-%d ms (default %d ms)\n",
                   CPU_STATUS_MIN_MS, CPU_STATUS_MAX_MS,
                   CPU_STATUS_DEFAULT_MS);
            return option == 'h' ? EXIT_SUCCESS : EXIT_FAILURE;
        }
    }

  if (duration_ms < CPU_STATUS_MIN_MS || duration_ms > CPU_STATUS_MAX_MS ||
      frequency == 0)
    {
      fprintf(stderr,
              "cpu_status: invalid duration or cycle-counter frequency\n");
      return EXIT_FAILURE;
    }

  capture = calloc(1, sizeof(*capture));
  entry = calloc(CPU_STATUS_MAX_THREADS, sizeof(*entry));
  if (capture == NULL || entry == NULL)
    {
      fprintf(stderr, "cpu_status: insufficient memory for task snapshots\n");
      free(capture);
      free(entry);
      return EXIT_FAILURE;
    }

  wall_start_us = cpu_status_monotonic_us();
  cpu_status_capture(capture);
  cpu_status_merge(entry, &entry_count, capture);
  truncated |= capture->truncated;

  while (elapsed_ms < duration_ms)
    {
      int interval_ms = duration_ms - elapsed_ms;

      if (interval_ms > CPU_STATUS_SAMPLE_MS)
        {
          interval_ms = CPU_STATUS_SAMPLE_MS;
        }

      usleep((useconds_t)interval_ms * 1000u);
      elapsed_ms += interval_ms;
      cpu_status_capture(capture);
      cpu_status_merge(entry, &entry_count, capture);
      truncated |= capture->truncated;
    }

  wall_us = cpu_status_monotonic_us() - wall_start_us;
  wall_cycles = wall_us * frequency / 1000000ull;
  cpu_status_print(entry, entry_count, wall_cycles, frequency,
                   duration_ms, truncated);
  free(capture);
  free(entry);
  optind = 0;
  return EXIT_SUCCESS;
}
