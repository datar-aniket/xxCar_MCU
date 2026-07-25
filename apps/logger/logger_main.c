/****************************************************************************
 * apps/logger/logger_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `log` - control the ULog logger.
 *
 *   log start       begin a session (writes /fs/microsd/log/log_NNN.ulg)
 *   log stop        end it
 *   log status      what it is recording, and how much
 *
 * What gets logged is set by parameters, not flags:
 *
 *   param set LOG_IMU 1     accel + gyro of both IMUs
 *   param set LOG_MAG 1
 *   param set LOG_BARO 1
 *   param set LOG_RC 1
 *   param set LOG_RATE 0    0 = every sample (native 2 kHz IMU); N = cap to N Hz
 *   param set LOG_ENABLE 1  also start logging at boot
 *   param save
 *
 * Copy the .ulg off with `sdmsc on`, then open it in pyulog / PlotJuggler.
 * Stop logging before `sdmsc on` - the card cannot be handed to the host while
 * a file on it is open for writing.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <unistd.h>

#include "logger.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int log_do_status(void)
{
  struct logger_status_s s;

  logger_get_status(&s);

  if (!s.running)
    {
      printf("logger: stopped\n");
      printf("  'log start', or set LOG_ENABLE=1 to start at boot.\n");
      return 0;
    }

  printf("logger: recording -> %s\n", s.path);
  printf("  topics    %" PRIu32 "\n", s.topics);
  printf("  rate      %s\n", s.rate > 0 ? "capped" : "native (every sample)");
  printf("  samples   %" PRIu32 "\n", s.samples);
  printf("  written   %" PRIu32 " bytes\n", s.bytes);

  if (s.dropped > 0)
    {
      printf("  DROPPED   %" PRIu32 "  (card cannot keep up)\n", s.dropped);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

  if (argc < 2)
    {
      printf("Usage: log start | stop | status\n"
             "  what is logged is set by LOG_IMU/MAG/BARO/RC and LOG_RATE\n");
      return 1;
    }

  if (strcmp(argv[1], "start") == 0)
    {
      ret = logger_start();

      if (ret == -EALREADY)
        {
          fprintf(stderr, "log: already running (log status)\n");
          return 1;
        }

      if (ret < 0)
        {
          fprintf(stderr, "log: start failed: %d\n", ret);
          return 1;
        }

      /* The session's path is known once the thread has opened the file. */

      usleep(50000);
      return log_do_status();
    }

  if (strcmp(argv[1], "stop") == 0)
    {
      logger_stop();
      printf("log: stopped\n");
      return 0;
    }

  if (strcmp(argv[1], "status") == 0)
    {
      return log_do_status();
    }

  fprintf(stderr, "log: unknown command '%s'\n", argv[1]);
  return 1;
}
