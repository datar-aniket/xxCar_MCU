/****************************************************************************
 * apps/cpu_status/cpu_status_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "cpu_audit.h"

#define CPU_STATUS_DEFAULT_MS 10000
#define CPU_STATUS_MIN_MS      1000
#define CPU_STATUS_MAX_MS   3600000

int main(int argc, FAR char *argv[])
{
  struct cpu_audit_report_s report = {0};
  int duration_ms = CPU_STATUS_DEFAULT_MS;
  int option;

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
                   "  true hardware-cycle audit, %d-%d ms (default %d ms)\n",
                   CPU_STATUS_MIN_MS, CPU_STATUS_MAX_MS,
                   CPU_STATUS_DEFAULT_MS);
            optind = 0;
            return option == 'h' ? EXIT_SUCCESS : EXIT_FAILURE;
        }
    }

  optind = 0;
  if (duration_ms < CPU_STATUS_MIN_MS || duration_ms > CPU_STATUS_MAX_MS)
    {
      fprintf(stderr, "cpu_status: duration must be %d-%d ms\n",
              CPU_STATUS_MIN_MS, CPU_STATUS_MAX_MS);
      return EXIT_FAILURE;
    }

  if (cpu_audit_measure(duration_ms, &report) < 0)
    {
      fprintf(stderr, "cpu_status: unable to capture CPU runtime\n");
      return EXIT_FAILURE;
    }

  cpu_audit_print(&report, true);
  cpu_audit_destroy(&report);
  return EXIT_SUCCESS;
}
