/****************************************************************************
 * apps/logger/logger_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `log` - control the ULog logger.
 *
 *   log start       begin a session with whatever the LOG_* params say
 *   log allan [hz]  set up and begin an Allan-variance run
 *   log ekf [hz]    record raw IMU plus EKF inputs/diagnostics/output
 *   log stop        end it
 *   log status      what it is recording, and how much
 *
 * Files are written to /fs/microsd/log/ as log_NNN_PP.ulg and split every
 * 100 MB, so a long run has no time limit and a power loss costs one part.
 * Every part is a complete ULog file and opens on its own.
 *
 * What gets logged is set by parameters, not flags:
 *
 *   param set LOG_IMU0 1    accel + gyro of IMU0 (ICM-42688)
 *   param set LOG_IMU1 1    accel + gyro of IMU1 (BMI055)
 *   param set LOG_MAG 1
 *   param set LOG_BARO 1
 *   param set LOG_RC 1
 *   param set LOG_EKF 1    exact EKF deltas, residuals, innovations + state
 *   param set LOG_RATE 0    0 = every sample (native 2 kHz); N = cap to N Hz
 *   param set LOG_ENABLE 1  also start logging at boot
 *   param save
 *
 * `log allan` is those settings for a noise run in one command: both IMUs on,
 * everything else off, rate defaulting to 200 Hz. It matches exactly what the
 * calibration GUI's Record button does, so a run started from the shell and
 * one started from the GUI produce the same dataset.
 *
 * Copy the .ulg off with `sdmsc on`, then open it in pyulog / PlotJuggler.
 * Stop logging before `sdmsc on` - the card cannot be handed to the host while
 * a file on it is open for writing.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <unistd.h>

#include "logger.h"
#include "../param/param.h"

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
  printf("  written   %" PRIu64 " bytes (%.1f MB)\n",
         s.bytes, (double)s.bytes / (1024.0 * 1024.0));

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
      printf("Usage: log start | stop | status | allan [hz] | ekf [hz]\n"
             "  start   log whatever LOG_IMU/MAG/BARO/RC and LOG_RATE say\n"
             "  allan   set up and start an Allan-variance run: both IMUs\n"
             "          only, at [hz] (default 200), everything else off\n"
             "  ekf     log IMU0, mag, baro, external pose and synchronized\n"
             "          EKF diagnostics at [hz] (default 400)\n"
             "  Files split at 100 MB as log_NNN_PP.ulg.\n");
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

  if (strcmp(argv[1], "allan") == 0)
    {
      /* The same setup the GUI's `record start` performs, so a run started
       * from the shell and one started from the GUI produce the same dataset.
       *
       * Mag and baro default to ON, and for an Allan run they are just bytes
       * the analysis discards - so they are turned off explicitly rather than
       * left to whatever the card's params.txt happened to say.
       */

      long hz = argc > 2 ? strtol(argv[2], NULL, 10) : 200;

      if (hz < 0 || hz > 2000)
        {
          fprintf(stderr, "log: rate must be 0-2000 (0 = native)\n");
          return 1;
        }

      param_set_i32("LOG_IMU0", 1);
      param_set_i32("LOG_IMU1", 1);
      param_set_i32("LOG_MAG",  0);
      param_set_i32("LOG_BARO", 0);
      param_set_i32("LOG_RC",   0);
      param_set_i32("LOG_FLOW", 0);
      param_set_i32("LOG_DIST", 0);
      param_set_i32("LOG_EKF",  0);
      param_set_i32("LOG_RATE", (int32_t)hz);

      if (hz > 0)
        {
          printf("log: IMU0+IMU1 only, %ld Hz\n", hz);
        }
      else
        {
          printf("log: IMU0+IMU1 only, native rate (every sample)\n");
        }

      if (hz == 0)
        {
          printf("  native rate is ~227 KB/s - a 100 MB part every 7.5 min.\n"
                 "  200 Hz is the better long run: it still reaches the\n"
                 "  bias-instability knee with 10x the SD-stall tolerance.\n");
        }

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

      usleep(50000);
      return log_do_status();
    }

  if (strcmp(argv[1], "ekf") == 0)
    {
      long hz = argc > 2 ? strtol(argv[2], NULL, 10) : 400;

      if (hz < 0 || hz > 2000)
        {
          fprintf(stderr, "log: rate must be 0-2000 (0 = native)\n");
          return 1;
        }

      /* Capture both ends of the estimator path: raw IMU0 at the sensor
       * boundary, the exact matched-LPF delta packets consumed by the EKF,
       * every aiding input, and the horizon/output states. At 400 Hz the raw
       * 2 kHz stream is decimated while LOG_EKF records bypass the cap, so no
       * EKF delta packet or fusion event is discarded.
       */

      param_set_i32("LOG_IMU0", 1);
      param_set_i32("LOG_IMU1", 0);
      param_set_i32("LOG_MAG",  1);
      param_set_i32("LOG_BARO", 1);
      param_set_i32("LOG_RC",   0);
      param_set_i32("LOG_FLOW", 0);
      param_set_i32("LOG_DIST", 0);
      param_set_i32("LOG_EKF",  1);
      param_set_i32("LOG_RATE", (int32_t)hz);

      if (hz > 0)
        {
          printf("log: EKF diagnostic profile, %ld Hz\n", hz);
        }
      else
        {
          printf("log: EKF diagnostic profile, native rate\n");
        }
      printf("  IMU0 raw + vehicle_imu + estimator_diag + estimator_state\n"
             "  external_pose + vehicle_accel + transmitted vehicle_state\n"
             "  magnetometer + barometer\n");

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
