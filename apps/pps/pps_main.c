/****************************************************************************
 * apps/pps/pps_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * pps start | stop | status
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "pps.h"
#include "../../boards/fmuv6c/src/fmuv6c.h"

static FAR const char *pps_state_name(enum fmuv6c_pps_state_e state)
{
  switch (state)
    {
      case FMUV6C_PPS_NO_SIGNAL:
        return "NO SIGNAL";
      case FMUV6C_PPS_ACQUIRING:
        return "ACQUIRING";
      case FMUV6C_PPS_LOCKED:
        return "LOCKED";
      case FMUV6C_PPS_HOLDOVER:
        return "HOLDOVER";
      default:
        return "UNKNOWN";
    }
}

static void pps_print_status(void)
{
  struct fmuv6c_pps_status_s s;
  uint64_t now_us;
  uint64_t age_us;

  fmuv6c_pps_status(&s);
  now_us = fmuv6c_imu_time_now();
  age_us = s.last_edge_us == 0 ? 0 : now_us - s.last_edge_us;

  printf("pps: %s, TELEM2 CTS PC9/TIM3_CH4 rising edge, 1 MHz\n",
         s.running ? pps_state_name(s.state) : "stopped");
  if (!s.running)
    {
      return;
    }

  if (s.last_edge_us == 0)
    {
      printf("  waiting for a 3.3 V pulse from Jetson (100 ms high is OK)\n");
    }
  else
    {
      printf("  last edge %" PRIu64 " us  age %.3f ms  period %.3f ms\n",
             s.last_edge_us, (double)age_us / 1000.0,
             (double)s.last_period_us / 1000.0);
    }

  printf("  lock %u/3  valid period [%.3f, %.3f] ms\n",
         s.good_intervals, (double)s.min_period_us / 1000.0,
         (double)s.max_period_us / 1000.0);
  printf("  edges raw %" PRIu32 " accepted %" PRIu32
         " glitches %" PRIu32 " bad_period %" PRIu32
         " missed %" PRIu32 " overcapture %" PRIu32 "\n",
         s.raw_edges, s.accepted_edges, s.glitches, s.bad_periods,
         s.missed_pulses, s.overcaptures);
  printf("  clock observe-only: monotonic time is never stepped; loss uses "
         "holdover\n");
}

int main(int argc, FAR char *argv[])
{
  int ret;

  if (argc != 2)
    {
      printf("Usage: pps start | stop | status\n");
      return 1;
    }

  if (strcmp(argv[1], "start") == 0)
    {
      ret = pps_start();
      if (ret == -EALREADY)
        {
          printf("pps: already running\n");
        }
      else if (ret < 0)
        {
          printf("pps: start failed: %d\n", ret);
          return 1;
        }

      pps_print_status();
      return 0;
    }

  if (strcmp(argv[1], "stop") == 0)
    {
      ret = pps_stop();
      if (ret < 0)
        {
          printf("pps: stop failed: %d\n", ret);
          return 1;
        }

      printf("pps: stopped\n");
      return 0;
    }

  if (strcmp(argv[1], "status") == 0)
    {
      pps_print_status();
      return 0;
    }

  printf("Usage: pps start | stop | status\n");
  return 1;
}
