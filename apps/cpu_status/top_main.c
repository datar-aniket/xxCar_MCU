/****************************************************************************
 * apps/cpu_status/top_main.c
 *
 * Live DWT-cycle CPU monitor. This intentionally does not use NuttX's
 * scheduler-tick CPU-load sampler.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "cpu_audit.h"

#define TOP_DEFAULT_MS 1000
#define TOP_MIN_MS      250
#define TOP_MAX_MS    60000

static volatile sig_atomic_t g_top_stop;

static void top_signal(int signo)
{
  (void)signo;
  g_top_stop = 1;
}

int main(int argc, FAR char *argv[])
{
  struct cpu_audit_report_s report = {0};
  struct sigaction action;
  struct sigaction old_action;
  int duration_ms = TOP_DEFAULT_MS;
  int iterations = 0;
  int completed = 0;
  int option;
  int status = EXIT_SUCCESS;
  bool terminal = isatty(STDOUT_FILENO);

  while ((option = getopt(argc, argv, "d:n:h")) != ERROR)
    {
      switch (option)
        {
          case 'd':
            duration_ms = atoi(optarg);
            break;

          case 'n':
            iterations = atoi(optarg);
            break;

          case 'h':
          default:
            printf("Usage: top [-d <ms>] [-n <count>]\n"
                   "  true DWT-cycle monitor, interval %d-%d ms\n"
                   "  count 0 runs until Ctrl-C (default)\n",
                   TOP_MIN_MS, TOP_MAX_MS);
            optind = 0;
            return option == 'h' ? EXIT_SUCCESS : EXIT_FAILURE;
        }
    }

  optind = 0;
  if (duration_ms < TOP_MIN_MS || duration_ms > TOP_MAX_MS || iterations < 0)
    {
      fprintf(stderr, "top: invalid interval or count\n");
      return EXIT_FAILURE;
    }

  g_top_stop = 0;
  sigemptyset(&action.sa_mask);
  action.sa_handler = top_signal;
  action.sa_flags = 0;

  if (sigaction(SIGINT, &action, &old_action) < 0)
    {
      fprintf(stderr, "top: cannot install Ctrl-C handler\n");
      return EXIT_FAILURE;
    }

  while (!g_top_stop && (iterations == 0 || completed < iterations))
    {
      if (cpu_audit_measure(duration_ms, &report) < 0)
        {
          fprintf(stderr, "top: unable to capture CPU runtime\n");
          status = EXIT_FAILURE;
          break;
        }

      if (g_top_stop)
        {
          break;
        }

      if (terminal)
        {
          printf("\033[2J\033[H");
        }

      cpu_audit_print(&report, false);
      printf("true DWT cycles; Ctrl-C exits; ps FILLED/STACK%% is stack use\n");
      fflush(stdout);
      completed++;
    }

  cpu_audit_destroy(&report);
  sigaction(SIGINT, &old_action, NULL);
  return status;
}
