/****************************************************************************
 * apps/ekf3/ekf3_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ekf3.h"

static void usage(void)
{
  printf("Usage: ekf3 start | stop | status\n"
         "  Requires imu_delta. Runs 400 Hz 15-state prediction and\n"
         "  100 Hz covariance; this stage publishes attitude only.\n");
}

static void print_status(void)
{
  struct ekf3_status_s status;
  FAR const struct ekf_core_s *core;
  float euler[3];
  float progress;
  float accel_rms;
  float gyro_rms;
  double output_rate = 0.0;
  double predict_rate = 0.0;
  const float rad_to_deg = 57.29577951308232f;

  ekf3_status(&status);
  core = &status.core;
  ekf_core_euler(core, euler);

  if (status.publish_count > 1 &&
      status.last_output_us > status.first_output_us)
    {
      output_rate = (double)(status.publish_count - 1u) * 1000000.0 /
                    (double)(status.last_output_us -
                             status.first_output_us);
    }

  if (core->predict_count > 0 && core->last_timestamp_sample >
      core->first_predict_timestamp)
    {
      predict_rate = (double)core->predict_count * 1000000.0 /
                     (double)(core->last_timestamp_sample -
                              core->first_predict_timestamp);
    }

  progress = core->align_time_s * 100.0f;
  accel_rms = sqrtf(core->dynamics_accel_variance[0] +
                    core->dynamics_accel_variance[1] +
                    core->dynamics_accel_variance[2]);
  gyro_rms = sqrtf(core->dynamics_gyro_variance[0] +
                   core->dynamics_gyro_variance[1] +
                   core->dynamics_gyro_variance[2]);

  if (progress > 100.0f)
    {
      progress = 100.0f;
    }

  printf("ekf3: %s, %s, solution %s\n",
         status.running ? "running" : "stopped",
         core->initialized ? "initialized" : "aligning",
         core->initialized ? "ATTITUDE+REL_YAW" : "NONE");
  printf("  output %" PRIu32 " %.2fHz predict %" PRIu32 " %.2fHz"
         " covariance %" PRIu32 " %.2fHz\n",
         status.publish_count, output_rate, core->predict_count,
         predict_rate, core->covariance_count,
         predict_rate / EKF_COVARIANCE_INTERVAL);
  printf("  alignment %.1f%% samples %" PRIu32
         " restarts %" PRIu32 "\n",
         (double)progress, core->align_samples,
         core->alignment_restart_count);
  printf("  dynamics %s dwell %.2fs accel_rms %.3f gyro_rms %.4f"
         " entries %" PRIu32 " exits %" PRIu32 "\n",
         core->low_dynamics ? "LOW" : "MOTION",
         (double)core->low_dynamics_dwell_s,
         (double)accel_rms, (double)gyro_rms,
         core->low_dynamics_entry_count,
         core->low_dynamics_exit_count);
  printf("  attitude RPY %+.3f %+.3f %+.3f deg (yaw relative)\n",
         (double)(euler[0] * rad_to_deg),
         (double)(euler[1] * rad_to_deg),
         (double)(euler[2] * rad_to_deg));
  printf("  quaternion %+.6f %+.6f %+.6f %+.6f\n",
         (double)core->quaternion[0], (double)core->quaternion[1],
         (double)core->quaternion[2], (double)core->quaternion[3]);
  printf("  velocity NED %+.4f %+.4f %+.4f m/s [INVALID]\n",
         (double)core->velocity[0], (double)core->velocity[1],
         (double)core->velocity[2]);
  printf("  position NED %+.4f %+.4f %+.4f m [INVALID]\n",
         (double)core->position[0], (double)core->position[1],
         (double)core->position[2]);
  printf("  gyro bias %+.6f %+.6f %+.6f rad/s\n",
         (double)core->gyro_bias[0], (double)core->gyro_bias[1],
         (double)core->gyro_bias[2]);
  printf("  accel bias %+.4f %+.4f %+.4f m/s2\n",
         (double)core->accel_bias[0], (double)core->accel_bias[1],
         (double)core->accel_bias[2]);
  printf("  variance angle %.3g %.3g %.3g velocity %.3g %.3g %.3g\n",
         (double)core->covariance[EKF_P_INDEX(0, 0)],
         (double)core->covariance[EKF_P_INDEX(1, 1)],
         (double)core->covariance[EKF_P_INDEX(2, 2)],
         (double)core->covariance[EKF_P_INDEX(3, 3)],
         (double)core->covariance[EKF_P_INDEX(4, 4)],
         (double)core->covariance[EKF_P_INDEX(5, 5)]);
  printf("  gravity update accept %" PRIu32 " reject %" PRIu32
         " NIS %.3f bias_limit %" PRIu32 "\n",
         core->gravity_accept_count, core->gravity_reject_count,
         (double)core->last_gravity_nis, core->bias_limit_count);
  printf("  faults reject %" PRIu32 " stale %" PRIu32
         " uncal %" PRIu32 " clip %" PRIu32 " dup %" PRIu32
         " back %" PRIu32 " gap %" PRIu32 " source_reset %" PRIu32
         " numeric %" PRIu32 " pub_error %" PRIu32 "\n",
         core->rejected_count, status.stale_count,
         core->uncalibrated_count, core->clipping_count,
         core->duplicate_count, core->backward_count, core->gap_count,
         core->source_reset_count, core->numerical_reset_count,
         status.publish_errors);
}

int main(int argc, FAR char *argv[])
{
  int result;

  if (argc != 2)
    {
      usage();
      return EXIT_FAILURE;
    }

  if (strcmp(argv[1], "start") == 0)
    {
      result = ekf3_start();

      if (result < 0)
        {
          printf("ekf3: start failed (%d)\n", result);
          return EXIT_FAILURE;
        }

      print_status();
      return EXIT_SUCCESS;
    }

  if (strcmp(argv[1], "stop") == 0)
    {
      result = ekf3_stop();

      if (result < 0)
        {
          printf("ekf3: stop failed (%d)\n", result);
          return EXIT_FAILURE;
        }

      print_status();
      return EXIT_SUCCESS;
    }

  if (strcmp(argv[1], "status") == 0)
    {
      print_status();
      return EXIT_SUCCESS;
    }

  usage();
  return EXIT_FAILURE;
}
