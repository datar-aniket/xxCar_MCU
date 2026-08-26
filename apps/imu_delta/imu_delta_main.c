/****************************************************************************
 * apps/imu_delta/imu_delta_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "imu_delta.h"
#include "../sensors/rotation.h"

static void usage(void)
{
  printf("Usage: imu_delta start|stop|status [instance]\n"
         "  Calibrated, unfiltered samples to 400 Hz\n"
         "  coning/sculling-corrected vehicle_imu packets.\n"
         "\n"
         "  instance 0 = ICM42688, feeds the estimator\n"
         "  instance 1 = BMI055, feeds the attitude monitor lane\n");
}

static void print_status(int instance)
{
  struct imu_delta_status_s status;
  double rate = 0.0;
  double mean_window = 0.0;

  imu_delta_status(instance, &status);

  printf("imu_delta%d (%s IMU)\n", instance,
         instance == 0 ? "primary" : "secondary");

  if (status.packets > 1 && status.last_packet_us > status.first_packet_us)
    {
      rate = (double)(status.packets - 1u) * 1000000.0 /
             (double)(status.last_packet_us - status.first_packet_us);
    }

  if (status.packets > 0)
    {
      mean_window = (double)status.total_window_us / status.packets;
    }

  printf("imu_delta: %s, IMU0 ICM42688, sensor %s, board %s, "
         "cal A:%s G:%s\n",
         status.running ? "running" : "stopped",
         rotation_name(status.sensor_rotation),
         rotation_name(status.board_rotation),
         status.accel_calibrated ? "on" : "off",
         status.gyro_calibrated ? "on" : "off");
  printf("  packets %" PRIu32 " rate %.2fHz paired %" PRIu32
         " sync_drop %" PRIu32 " queue_overrun %" PRIu32 " pub_error %" PRIu32
         "\n", status.packets, rate, status.paired_samples,
         status.sync_drops, status.queue_overruns, status.publish_errors);
  printf("  window %.2fus [%" PRIu32 "/%" PRIu32
         "] samples [%u/%u] clipped %" PRIu32 "\n",
         mean_window, status.min_window_us, status.max_window_us,
         status.min_samples, status.max_samples, status.clipped_packets);
  printf("  faults reset %" PRIu32 " gap %" PRIu32 " duplicate %" PRIu32
         " backward %" PRIu32 " invalid %" PRIu32 "\n",
         status.resets, status.gaps, status.duplicates,
         status.backwards, status.invalid);
  printf("  sum dAng %+.6f %+.6f %+.6f rad\n",
         (double)status.total_delta_angle[0],
         (double)status.total_delta_angle[1],
         (double)status.total_delta_angle[2]);
  printf("  sum dVel %+.4f %+.4f %+.4f m/s (window-frame audit sum)\n",
         (double)status.total_delta_velocity[0],
         (double)status.total_delta_velocity[1],
         (double)status.total_delta_velocity[2]);
}

int main(int argc, FAR char *argv[])
{
  int result;
  int instance = 0;

  if (argc != 2)
    {
      usage();
      return EXIT_FAILURE;
    }

  /* An optional trailing instance: `imu_delta start 1` runs the secondary.
   * Defaults to 0, so every existing invocation means what it did before.
   */

  if (argc > 2)
    {
      instance = atoi(argv[2]);

      if (instance < 0 || instance >= IMU_DELTA_INSTANCES)
        {
          printf("imu_delta: instance must be 0..%d\n",
                 IMU_DELTA_INSTANCES - 1);
          return EXIT_FAILURE;
        }
    }

  if (strcmp(argv[1], "start") == 0)
    {
      result = imu_delta_start(instance);

      if (result < 0)
        {
          printf("imu_delta: start failed (%d)\n", result);
          return EXIT_FAILURE;
        }

      print_status(instance);
      return EXIT_SUCCESS;
    }

  if (strcmp(argv[1], "stop") == 0)
    {
      result = imu_delta_stop(instance);

      if (result < 0)
        {
          printf("imu_delta: stop failed (%d)\n", result);
          return EXIT_FAILURE;
        }

      print_status(instance);
      return EXIT_SUCCESS;
    }

  if (strcmp(argv[1], "status") == 0)
    {
      print_status(instance);
      return EXIT_SUCCESS;
    }

  usage();
  return EXIT_FAILURE;
}
