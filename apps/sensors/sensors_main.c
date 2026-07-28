/****************************************************************************
 * apps/sensors/sensors_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `sensors start | stop | status`
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>

#include "sensors.h"
#include "rotation.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void usage(void)
{
  printf("Usage: sensors start | stop | status\n"
         "\n"
         "  Applies the stored IMU calibration and rotates into the body\n"
         "  frame, publishing vehicle_acceleration and\n"
         "  vehicle_angular_velocity. The raw sensor_accel/sensor_gyro\n"
         "  topics are left untouched.\n"
         "\n"
         "  Which IMU:  SENS_IMU_SEL       0 = ICM-42688, 1 = BMI055\n"
         "  Orientation: SENS_IMU0_ROT / SENS_IMU1_ROT (sensor to board)\n"
         "               SENS_BOARD_ROT               (board to vehicle)\n");
}

static void print_status(void)
{
  struct sensors_status_s s;

  sensors_status(&s);

  if (!s.running)
    {
      printf("sensors: stopped\n");
      return;
    }

  printf("sensors: running on IMU%u, rotation %s\n",
         s.instance, rotation_name(s.accel_rot));

  /* Say "raw passthrough" rather than showing zeros, so an uncalibrated
   * sensor cannot be mistaken for a calibrated one whose offsets came out
   * small - which, for a gyro, is what a good calibration looks like.
   */

  if (s.accel_calibrated)
    {
      printf("  accel  off %+.4f %+.4f %+.4f  scl %.4f %.4f %.4f\n",
             (double)s.accel_off[0], (double)s.accel_off[1],
             (double)s.accel_off[2], (double)s.accel_scl[0],
             (double)s.accel_scl[1], (double)s.accel_scl[2]);
    }
  else
    {
      printf("  accel  NOT CALIBRATED - raw passthrough\n");
    }

  if (s.gyro_calibrated)
    {
      printf("  gyro   off %+.6f %+.6f %+.6f rad/s\n",
             (double)s.gyro_off[0], (double)s.gyro_off[1],
             (double)s.gyro_off[2]);
    }
  else
    {
      printf("  gyro   NOT CALIBRATED - raw passthrough\n");
    }

  printf("  published  accel %" PRIu32 " (%" PRIu32 " skipped)"
         "  gyro %" PRIu32 " (%" PRIu32 " skipped)\n",
         s.accel_out, s.accel_skipped, s.gyro_out, s.gyro_skipped);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

  if (argc < 2)
    {
      usage();
      return EXIT_FAILURE;
    }

  if (strcmp(argv[1], "start") == 0)
    {
      ret = sensors_start();

      if (ret == -EALREADY)
        {
          printf("sensors: already running\n");
          return EXIT_FAILURE;
        }

      if (ret < 0)
        {
          printf("sensors: failed to start (%d) - check the syslog; an\n"
                 "         unsupported rotation or a missing IMU stops it\n",
                 ret);
          return EXIT_FAILURE;
        }

      print_status();
      return EXIT_SUCCESS;
    }

  if (strcmp(argv[1], "stop") == 0)
    {
      ret = sensors_stop();

      if (ret == -ESRCH)
        {
          printf("sensors: not running\n");
          return EXIT_FAILURE;
        }

      printf("sensors: %s\n", ret == OK ? "stopped" : "did not stop");
      return ret == OK ? EXIT_SUCCESS : EXIT_FAILURE;
    }

  if (strcmp(argv[1], "status") == 0)
    {
      print_status();
      return EXIT_SUCCESS;
    }

  usage();
  return EXIT_FAILURE;
}
