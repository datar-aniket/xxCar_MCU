/****************************************************************************
 * apps/ekf3/ekf3_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "ekf3.h"

static void usage(void)
{
  printf("Usage: ekf3 start | stop | status | reset\n"
         "  Requires imu_delta. Runs 400 Hz 15-state prediction and\n"
         "  100 Hz covariance at a EK3_DELAY_MS fusion horizon.\n"
         "  Barometric height is fused when EK3_SRCn_POSZ selects it and\n"
         "  `sensors aux` is publishing vehicle_baro.\n");
}

/* The solution used to be one of two fixed strings. Now that validity is per
 * state, print what is actually valid: a barometer that is correcting has to
 * be distinguishable from one that is merely selected.
 */

static void print_solution(uint8_t status)
{
  if (status == 0)
    {
      printf("NONE");
      return;
    }

  printf("%s%s%s%s%s%s",
         (status & EKF_SOLUTION_ATTITUDE) ? "ATTITUDE " : "",
         (status & EKF_SOLUTION_YAW_ABSOLUTE) ? "YAW_ABS " :
           (status & EKF_SOLUTION_YAW_RELATIVE) ? "YAW_REL " : "",
         (status & EKF_SOLUTION_VELOCITY_HORIZ) ? "VELXY " : "",
         (status & EKF_SOLUTION_VELOCITY_VERT) ? "VELZ " : "",
         (status & EKF_SOLUTION_POSITION_HORIZ) ? "POSXY " : "",
         (status & EKF_SOLUTION_POSITION_VERT) ? "POSZ" : "");
}

static bool source_yaw_is_compass(FAR const struct ekf3_status_s *status)
{
  return status->sources.set[status->sources.active_set].yaw ==
         EKF_SOURCE_BARO_OR_COMPASS;
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

  printf("ekf3: %s, %s, solution ",
         status.running ? "running" : "stopped",
         core->initialized ? "initialized" : "aligning");
  print_solution(ekf_core_solution_status(core));
  printf("\n");
  printf("  output %" PRIu32 " %.2fHz predict %" PRIu32 " %.2fHz"
         " covariance %" PRIu32 " %.2fHz\n",
         status.publish_count, output_rate, core->predict_count,
         predict_rate, core->covariance_count,
         predict_rate / EKF_COVARIANCE_INTERVAL);
  printf("  alignment %.1f%% samples %" PRIu32
         " restarts %" PRIu32 "\n",
         (double)progress, core->align_samples,
         core->alignment_restart_count);
  printf("  resets commanded %" PRIu32 "\n",
         core->commanded_reset_count);
  {
    FAR const struct ekf_source_set_s *source =
      &status.sources.set[status.sources.active_set];

    printf("  sources set %u POSXY=%u VELXY=%u POSZ=%u VELZ=%u YAW=%u"
           " options=0x%02x\n",
           status.sources.active_set + 1u, source->position_xy,
           source->velocity_xy, source->position_z, source->velocity_z,
           source->yaw, status.sources.options);
  }

  printf("  horizon %" PRIu32 " ms  replay %u samples"
         "  ring overflow imu %" PRIu32 " mag %" PRIu32 " baro %" PRIu32
         "\n",
         status.horizon_ms, status.output_replay,
         status.imu_overflow, status.mag_overflow, status.baro_overflow);
  if (core->extnav_datum_set)
    {
      printf("  extnav datum set  innov %+.3f %+.3f m  NIS %.3f %.3f\n",
             (double)core->last_extnav_innov[0],
             (double)core->last_extnav_innov[1],
             (double)core->last_extnav_nis[0],
             (double)core->last_extnav_nis[1]);

      /* The noise ACTUALLY used, after the floor. A source under-reporting
       * its own error is invisible otherwise.
       */

      printf("    accept %" PRIu32 " reject %" PRIu32 " (run %" PRIu32
             ")  redatum %" PRIu32 "  noise %.3f m\n",
             core->extnav_accept_count, core->extnav_reject_count,
             core->extnav_consecutive_rejects, core->extnav_datum_count,
             (double)core->last_extnav_noise);

      /* The health line is the one that explains a filter that has stopped
       * trusting the companion. test_ratio is the low-passed innovation
       * ratio against the IMU: above 1 the two disagree by more than the
       * gate allows, and staying there is what condemns the source.
       */

      printf("    health  %s  test_ratio %.2f  faults %" PRIu32
             "  accel-bias %s (%" PRIu32 ")\n",
             core->extnav_healthy ? "OK" : "UNHEALTHY - NOT FUSED",
             (double)core->extnav_test_ratio, core->extnav_fault_count,
             core->extnav_bias_inhibited ? "FROZEN" : "learning",
             core->extnav_inhibit_count);

      if (!core->extnav_healthy)
        {
          printf("    the source disagrees with the IMU persistently. It is\n"
                 "    no longer fused and POSITION_HORIZ is withdrawn, so\n"
                 "    autonomy should stop. It re-earns trust by agreeing.\n");
        }

      if (status.extnav_untimed > 0)
        {
          printf("    arrival-stamped %" PRIu32 " (source sent no "
                 "timestamp)\n", status.extnav_untimed);
        }
    }
  else
    {
      printf("  extnav no datum yet (queued %" PRIu32 ", bad time %" PRIu32
             ", untimed %" PRIu32 ")%s\n",
             status.extnav_in, status.extnav_bad_time,
             status.extnav_untimed,
             status.extnav_in > 0 &&
             status.sources.set[status.sources.active_set].position_xy !=
               EKF_SOURCE_EXTERNAL_NAV ?
               "   <- EK3_SRC1_POSXY is not 6" : "");
    }

  printf("  aiding in  mag %" PRIu32 " (%s)  baro %" PRIu32 " (%s)\n",
         status.mag_in, status.mag_available ? "subscribed" : "ABSENT",
         status.baro_in, status.baro_available ? "subscribed" : "ABSENT");

  if (core->yaw_absolute)
    {
      printf("  mag heading %+.2f deg  |B| %.4f expected %.4f"
             "  decl %+.2f deg\n",
             (double)(core->last_mag_heading * rad_to_deg),
             (double)core->last_mag_field, (double)status.mag_expected,
             (double)(status.declination * rad_to_deg));
      printf("  mag accept %" PRIu32 " reject %" PRIu32 " (run %" PRIu32
             ") unhealthy %" PRIu32 " NIS %.3f\n",
             core->mag_accept_count, core->mag_reject_count,
             core->mag_consecutive_rejects, core->mag_unhealthy_count,
             (double)core->last_mag_nis);
    }
  else
    {
      /* Say WHY the heading has no datum. "relative" alone cost nothing to
       * print and everything to diagnose: an uncalibrated compass, a
       * deselected source and an aux daemon that was never started all look
       * identical from the outside.
       */

      printf("  mag heading NONE - %s\n",
             !status.mag_available ?
               "vehicle_mag absent; run `sensors aux start`" :
             source_yaw_is_compass(&status) ?
               (status.mag_align_used == 0 ?
                  "no calibrated field reached alignment "
                  "(CAL_MAG0_OK? aux started before ekf3?)" :
                  "field rejected at alignment") :
               "EK3_SRCn_YAW does not select the compass");
    }

  if (core->baro_have_reference)
    {
      printf("  baro ref %.2f hPa  height %+.3f m  accept %" PRIu32
             " reject %" PRIu32 " (run %" PRIu32 ") NIS %.3f\n",
             (double)core->baro_reference_hpa,
             (double)core->last_baro_height,
             core->baro_accept_count, core->baro_reject_count,
             core->baro_consecutive_rejects, (double)core->last_baro_nis);
    }
  else
    {
      printf("  baro no reference yet (noise %.2f m gate %.1f sigma)\n",
             (double)status.alt_noise, (double)status.alt_gate);
    }
  printf("  dynamics %s dwell %.2fs accel_rms %.3f gyro_rms %.4f"
         " entries %" PRIu32 " exits %" PRIu32 "\n",
         core->low_dynamics ? "LOW" : "MOTION",
         (double)core->low_dynamics_dwell_s,
         (double)accel_rms, (double)gyro_rms,
         core->low_dynamics_entry_count,
         core->low_dynamics_exit_count);
  printf("  attitude RPY %+.3f %+.3f %+.3f deg (yaw %s)\n",
         (double)(euler[0] * rad_to_deg),
         (double)(euler[1] * rad_to_deg),
         (double)(euler[2] * rad_to_deg),
         (ekf_core_solution_status(core) & EKF_SOLUTION_YAW_ABSOLUTE) ?
           "absolute" : "relative");
  printf("  quaternion %+.6f %+.6f %+.6f %+.6f\n",
         (double)core->quaternion[0], (double)core->quaternion[1],
         (double)core->quaternion[2], (double)core->quaternion[3]);
  printf("  velocity ENU %+.4f %+.4f %+.4f m/s [%s]\n",
         (double)core->velocity[0], (double)core->velocity[1],
         (double)core->velocity[2],
         (ekf_core_solution_status(core) & EKF_SOLUTION_VELOCITY_VERT) ?
           "VERT ONLY" : "INVALID");
  printf("  position ENU %+.4f %+.4f %+.4f m [%s]\n",
         (double)core->position[0], (double)core->position[1],
         (double)core->position[2],
         (ekf_core_solution_status(core) & EKF_SOLUTION_POSITION_VERT) ?
           "VERT ONLY" : "INVALID");
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
  printf("  yaw gauge suppressed last %+.6f max %.6f deg count %" PRIu32
         "\n",
         (double)(core->last_gravity_yaw_suppressed * rad_to_deg),
         (double)(core->max_gravity_yaw_suppressed * rad_to_deg),
         core->gravity_yaw_projection_count);
  printf("  faults reject %" PRIu32 " stale %" PRIu32
         " uncal %" PRIu32 " clip %" PRIu32 " dup %" PRIu32
         " back %" PRIu32 " gap %" PRIu32 " source_reset %" PRIu32
         " numeric %" PRIu32 " pub_error %" PRIu32 "\n",
         core->rejected_count, status.stale_count,
         core->uncalibrated_count, core->clipping_count,
         core->duplicate_count, core->backward_count, core->gap_count,
         core->source_reset_count, core->numerical_reset_count,
         status.publish_errors);
  /* The horizontal hold. Holding is not a fault in itself - it is the
   * correct response to having no fix - but a solution in hold is one whose
   * position is a bound rather than an estimate, and nothing should be
   * driving on it.
   */

  if (status.position_hold_limit > 0.0f)
    {
      printf("  position   %s  bound %.1f m  holds %" PRIu32 "  snaps %"
             PRIu32 "\n",
             core->position_holding ? "HELD (no fix)" : "aided",
             (double)status.position_hold_limit,
             core->position_hold_count, core->position_snap_count);

      if (core->position_holding)
        {
          printf("             dead reckoning is bounded, not running: the\n"
                 "             position is where the last fix left it, and\n"
                 "             the next valid fix is adopted outright\n");
        }
    }
  else
    {
      printf("  position   free dead reckoning (EK3_POSHOLD_M=0)\n");
    }

  /* The vertical bound. A clamp count that is climbing says the filter is
   * being held on the road by hand, which is a fault report, not a feature
   * working - the height it publishes is a bound, not an estimate.
   */

  if (status.height_limit > 0.0f)
    {
      printf("  height     bound %.1f m  clamped %" PRIu32 "%s\n",
             (double)status.height_limit, core->height_clamp_count,
             core->height_clamp_count > 0 ?
               "   <- height is being held, not estimated" : "");
    }
  else
    {
      printf("  height     unbounded (EK3_HGT_LIM=0)\n");
    }

  /* The attitude monitor.
   *
   * Two questions, and the lane pairing is what separates them: "aiding" is
   * the estimator against a lane running its OWN IMU with no aiding, so a
   * difference there cannot be the sensor. "imu" is that lane against one
   * running the other IMU, with no aiding on either side, so a difference
   * there IS the sensor.
   *
   * Both are yaw-free tilt angles - the monitors pin yaw at zero, so only
   * roll and pitch are comparable, and the metric is built to ignore yaw
   * entirely.
   */

  if (!status.mon_enabled)
    {
      printf("  monitor    off (EKF_MON_EN=0)\n");
    }
  else
    {
      printf("  monitor    aiding %.3f rad  imu %s  limit %.3f  hold %"
             PRIu32 " ms%s\n",
             (double)status.mon_aiding_tilt,
             status.mon_secondary_live ? "" :
               (status.mon_subscribed ? "-- (subscribed, no data yet)" :
                                        "-- (no vehicle_imu1; run "
                                        "`imu_delta status`)"),
             (double)status.mon_tilt_limit, status.mon_hold_ms,
             status.mon_act ? "" : "   REPORT ONLY");

      if (status.mon_secondary_live)
        {
          printf("             imu %.3f rad  roll %+.3f pitch %+.3f\n",
                 (double)status.mon_imu_tilt,
                 (double)status.mon_imu_err[0],
                 (double)status.mon_imu_err[1]);
        }

      printf("             aiding roll %+.3f pitch %+.3f  faults a=%"
             PRIu32 " i=%" PRIu32 "\n",
             (double)status.mon_aiding_err[0],
             (double)status.mon_aiding_err[1],
             status.mon_aiding_faults, status.mon_imu_faults);

      if (status.mon_aiding_fault)
        {
          printf("             AIDING FAULT: the estimator disagrees with "
                 "its own IMU.\n"
                 "             An aiding source is corrupting attitude - the "
                 "sensor is ruled out.\n");
        }

      if (status.mon_imu_fault)
        {
          printf("             IMU FAULT: the two IMUs disagree with no "
                 "aiding on either.\n");
        }
    }
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

  if (strcmp(argv[1], "reset") == 0)
    {
      result = ekf3_reset();

      if (result == -ESRCH)
        {
          printf("ekf3: not running\n");
          return EXIT_FAILURE;
        }

      if (result < 0)
        {
          printf("ekf3: reset was not acted on (%d)\n", result);
          return EXIT_FAILURE;
        }

      printf("ekf3: reset - realigning, hold the vehicle still\n");
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
