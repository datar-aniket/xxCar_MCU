/****************************************************************************
 * apps/ekf3/ekf3.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/uorb.h>
#include <uORB/uORB.h>

#include "ekf3.h"
#include "ekf_extnav_frame.h"
#include "ekf_extnav_time.h"
#include "ekf_wheel.h"
#include "../param/param.h"
#include "../uorb_msgs/uorb_msgs.h"
#include "../../boards/fmuv6c/src/fmuv6c.h"

#define EKF3_PRIORITY          (SCHED_PRIORITY_DEFAULT + 22)
#define EKF3_STACK             6144
#define EKF3_DRAIN_MAX           32
#define EKF3_MAX_INPUT_AGE_US  50000ull

/* How stale an aiding measurement may be at the horizon before it is dropped
 * rather than fused. Beyond this the filter has propagated far enough past
 * the sample that the correction would land somewhere else on the trajectory.
 */

#define EKF3_BARO_MAX_AGE_US  500000ull
#define EKF3_MAG_MAX_AGE_US   500000ull
#define EKF3_EXT_MAX_AGE_US   500000ull
#define EKF3_WHEEL_MAX_AGE_US 200000ull

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool g_running;
static volatile bool g_should_stop;
static volatile bool g_should_reset;
static struct ekf3_status_s g_status;

/* ~3 kB. A file-scope static, not a member of g_status and not on the stack:
 * ekf3 runs on 6144 bytes and g_status is copied wholesale under a mutex.
 */

static struct ekf_delay_s g_delay;
static struct ekf_wheel_accel_filter_s g_wheel_accel_filter;
static struct ekf_wheel_lpf_s g_imu_accel_filter;

/* The attitude monitor lanes. File-scope for the same reason the delay ring
 * is: 1400 bytes each, and ekf3_status() copies its struct wholesale under a
 * mutex on a 6144-byte stack.
 *
 * Both run in REAL TIME, on samples as they arrive, and are compared against
 * the output-predicted attitude - the one actually published - rather than
 * against the core's delayed horizon state. Comparing a real-time monitor
 * with a state 100 ms behind would report a difference proportional to turn
 * rate: at 1 rad/s that is 0.1 rad of pure artefact, comparable to the fault
 * threshold itself.
 */

static struct ekf_core_s g_mon_primary;
static struct ekf_core_s g_mon_secondary;

/* The EKF clock.
 *
 * Every timestamp that participates in propagation or fusion is in the
 * shared TIM5 domain: IMU DRDY edges, FDCAN receive edges and synchronized
 * companion measurements.  CLOCK_MONOTONIC has the same boot epoch but is a
 * different, tick-quantized counter.  Comparing one against the other makes
 * the fusion horizon move by their relative phase and rate error.
 */

static uint64_t ekf_now_us(void)
{
  return fmuv6c_imu_time_now();
}

/* Diagnostic/publication clock only.  It must never be compared with a
 * timestamp_sample or used to decide the fusion horizon.
 */

static uint64_t monotonic_now_us(void)
{
  struct timespec timestamp;

  clock_gettime(CLOCK_MONOTONIC, &timestamp);
  return (uint64_t)timestamp.tv_sec * 1000000ull +
         (uint64_t)timestamp.tv_nsec / 1000ull;
}

static void status_publish(FAR const struct ekf3_status_s *status)
{
  pthread_mutex_lock(&g_lock);
  g_status = *status;
  pthread_mutex_unlock(&g_lock);
}

static void fill_core_sample(FAR const struct vehicle_imu_s *message,
                             FAR struct ekf_imu_sample_s *sample)
{
  memset(sample, 0, sizeof(*sample));
  sample->timestamp_sample = message->timestamp_sample;
  sample->timestamp_first = message->timestamp_first;
  memcpy(sample->delta_angle, message->delta_angle,
         sizeof(sample->delta_angle));
  memcpy(sample->delta_velocity, message->delta_velocity,
         sizeof(sample->delta_velocity));
  sample->delta_angle_dt = message->delta_angle_dt;
  sample->delta_velocity_dt = message->delta_velocity_dt;
  sample->samples = message->samples;
  sample->reset_counter = message->reset_counter;
  sample->instance = message->instance;
  sample->clipping = message->clipping;
  sample->accel_calibrated = message->accel_calibrated != 0;
  sample->gyro_calibrated = message->gyro_calibrated != 0;
}

/* The kinematic state comes from the PREDICTED output - the filter state
 * replayed forward to the present. Everything else comes from the filter
 * itself: the covariance belongs to the state at the horizon and there is no
 * meaningful way to forward-propagate it here, which is precisely why the two
 * are kept separate.
 */

static void fill_output(FAR const struct ekf_core_s *core,
                        FAR const struct ekf_output_s *predicted,
                        uint64_t publication_time,
                        FAR struct estimator_state_s *output)
{
  int axis;

  memset(output, 0, sizeof(*output));
  output->timestamp = publication_time;
  output->timestamp_sample = predicted->timestamp_sample;
  memcpy(output->quaternion, predicted->quaternion,
         sizeof(output->quaternion));
  memcpy(output->velocity, predicted->velocity, sizeof(output->velocity));
  memcpy(output->position, predicted->position, sizeof(output->position));
  memcpy(output->gyro_bias, core->gyro_bias, sizeof(output->gyro_bias));
  memcpy(output->accel_bias, core->accel_bias, sizeof(output->accel_bias));

  for (axis = 0; axis < 3; axis++)
    {
      output->angle_variance[axis] =
        core->covariance[EKF_P_INDEX(axis, axis)];
      output->velocity_variance[axis] =
        core->covariance[EKF_P_INDEX(3 + axis, 3 + axis)];
      output->position_variance[axis] =
        core->covariance[EKF_P_INDEX(6 + axis, 6 + axis)];
    }

  output->predict_count = core->predict_count;
  output->covariance_count = core->covariance_count;
  output->reset_counter = core->reset_counter;
  output->solution_status = ekf_core_solution_status(core);
  output->instance = 0;
}

static void publish_diagnostics(
  int publisher, FAR struct ekf3_status_s *status,
  FAR const struct ekf_imu_sample_s *sample,
  FAR const struct ekf_extnav_sample_s *extnav, int process_result,
  uint32_t extnav_accept_before, uint32_t extnav_reject_before,
  uint32_t zupt_accept_before, uint32_t zupt_reject_before,
  uint32_t gravity_accept_before, uint32_t gravity_reject_before)
{
  FAR const struct ekf_core_s *core = &status->core;
  struct estimator_diag_s message;
  bool predicted = process_result == EKF_PROCESS_PREDICTED &&
                   core->last_predict_timestamp == sample->timestamp_sample;
  float norm_sq = 0.0f;
  int axis;

  if (publisher < 0)
    {
      return;
    }

  memset(&message, 0, sizeof(message));

  /* The first timestamp deliberately uses the TIM5 sample domain rather than
   * task publication time. LOG_RATE decimates using this field, and the
   * diagnostic must retain one deterministic record per filter step even if
   * task scheduling bunches two publications together.
   */

  message.timestamp = sample->timestamp_sample;
  message.timestamp_sample = sample->timestamp_sample;

  if (extnav != NULL)
    {
      message.extnav_timestamp = extnav->timestamp_sample;
      message.extnav_measurement[0] = extnav->x;
      message.extnav_measurement[1] = extnav->y;
      message.extnav_measurement[2] = extnav->yaw;
    }

  for (axis = 0; axis < 3; axis++)
    {
      if (predicted)
        {
          message.specific_force[axis] = core->last_specific_force[axis];
          message.corrected_force[axis] = core->last_corrected_force[axis];
          message.gravity_body[axis] = core->last_gravity_body[axis];
          message.residual_accel_body[axis] =
            core->last_residual_accel_body[axis];
          message.nav_accel[axis] = core->last_nav_accel[axis];
        }
      else
        {
          /* Alignment/rejected packets still remain visible in the ULog.
           * Only the directly measured and bias-corrected quantities are
           * meaningful before a strapdown prediction has run.
           */

          message.specific_force[axis] =
            sample->delta_velocity[axis] / sample->delta_velocity_dt;
          message.corrected_force[axis] =
            message.specific_force[axis] - core->accel_bias[axis];
        }

      norm_sq += message.corrected_force[axis] *
                 message.corrected_force[axis];
    }

  memcpy(message.quaternion, core->quaternion, sizeof(message.quaternion));
  memcpy(message.velocity, core->velocity, sizeof(message.velocity));
  memcpy(message.position, core->position, sizeof(message.position));
  memcpy(message.gyro_bias, core->gyro_bias, sizeof(message.gyro_bias));
  memcpy(message.accel_bias, core->accel_bias, sizeof(message.accel_bias));
  memcpy(message.extnav_innov, core->last_extnav_innov,
         sizeof(message.extnav_innov));
  memcpy(message.extnav_nis, core->last_extnav_nis,
         sizeof(message.extnav_nis));
  memcpy(message.zupt_nis, core->last_zupt_nis,
         sizeof(message.zupt_nis));

  message.gravity_nis = core->last_gravity_nis;
  message.accel_norm = sqrtf(norm_sq);
  message.accel_variance = status->zupt_accel_variance;
  message.gravity_deviation = status->zupt_gravity_deviation;
  message.extnav_test_ratio = core->extnav_test_ratio;
  message.wheel_speed_cps = status->last_wheel_cps;
  message.wheel_speed_mps = status->wheel_diag_speed_mps;
  message.wheel_accel_mps2 = status->wheel_diag_accel_mps2;
  message.imu_accel_mps2 = status->wheel_diag_imu_accel_mps2;
  memcpy(message.wheel_innov, core->last_wheel_innov,
         sizeof(message.wheel_innov));
  memcpy(message.wheel_nis, core->last_wheel_nis,
         sizeof(message.wheel_nis));
  message.wheel_timestamp = status->wheel_diag_timestamp;
  message.extnav_accept_count = core->extnav_accept_count;
  message.extnav_reject_count = core->extnav_reject_count;
  message.zupt_accept_count = core->zupt_accept_count;
  message.zupt_reject_count = core->zupt_reject_count;
  message.gravity_accept_count = core->gravity_accept_count;
  message.gravity_reject_count = core->gravity_reject_count;
  message.wheel_accept_count = core->wheel_accept_count;
  message.wheel_reject_count = core->wheel_reject_count;
  message.reset_counter = core->reset_counter;
  message.instance = sample->instance;

  if (core->initialized)
    {
      message.flags |= EST_DIAG_INITIALIZED;
    }

  if (predicted)
    {
      message.flags |= EST_DIAG_PREDICTED;
    }

  if (core->low_dynamics)
    {
      message.flags |= EST_DIAG_LOW_DYNAMICS;
    }

  if (status->zupt_stopped)
    {
      message.flags |= EST_DIAG_WHEEL_STOPPED;
    }

  if (status->wheel_fresh)
    {
      message.flags |= EST_DIAG_WHEEL_FRESH;
    }

  if (status->zupt_imu_stationary)
    {
      message.flags |= EST_DIAG_ZUPT_IMU_OK;
    }

  if (core->extnav_healthy)
    {
      message.flags |= EST_DIAG_EXTNAV_HEALTHY;
    }

  if (ekf_core_position_aided(core))
    {
      message.flags |= EST_DIAG_POSITION_AIDED;
    }

  if (core->zupt_accept_count != zupt_accept_before)
    {
      message.flags |= EST_DIAG_ZUPT_ACCEPT;
    }

  if (core->zupt_reject_count != zupt_reject_before)
    {
      message.flags |= EST_DIAG_ZUPT_REJECT;
    }

  if (core->gravity_accept_count != gravity_accept_before)
    {
      message.flags |= EST_DIAG_GRAVITY_ACCEPT;
    }

  if (core->gravity_reject_count != gravity_reject_before)
    {
      message.flags |= EST_DIAG_GRAVITY_REJECT;
    }

  if (core->extnav_accept_count != extnav_accept_before)
    {
      message.flags |= EST_DIAG_EXTNAV_ACCEPT;
    }

  if (core->extnav_reject_count != extnav_reject_before)
    {
      message.flags |= EST_DIAG_EXTNAV_REJECT;
    }

  if (sample->clipping != 0)
    {
      message.flags |= EST_DIAG_CLIPPING;
    }

  if (process_result == EKF_PROCESS_REJECTED)
    {
      message.flags |= EST_DIAG_PROCESS_REJECTED;
    }

  if (estimator_diag_publish(publisher, &message) < 0)
    {
      status->publish_errors++;
    }
}

/* Take exactly ONE fresh IMU packet into the ring. Returns false when there
 * is nothing left to read.
 *
 * One per iteration, not all of them. Draining every pending packet and
 * publishing once afterwards makes the output rate the rate at which this
 * task happens to be SCHEDULED rather than the rate at which data arrives -
 * which collapsed a 400 Hz topic to about 270 Hz, since the task typically
 * woke with one and a half packets waiting.
 *
 * It also kept the published timestamp_sample identical across every
 * publication in an iteration, because the newest ring entry - which is what
 * the output is re-propagated to - had not moved.
 *
 * Nothing is lost by taking one: poll() returns immediately while packets
 * are still pending, so a task that falls behind spins until it catches up.
 *
 * Stale packets are skipped rather than returned, so one arriving late
 * cannot cost this iteration its publication.
 */

static bool take_imu_sample(int sub, FAR struct ekf3_status_s *status,
                            FAR uint64_t *newest_sample_time)
{
  int skipped = 0;

  while (skipped++ < EKF3_DRAIN_MAX)
    {
      struct vehicle_imu_s message;
      struct ekf_imu_sample_s sample;
      uint64_t now;

      if (orb_copy(ORB_ID(vehicle_imu), sub, &message) < 0)
        {
          return false;
        }

      now = ekf_now_us();

      if (now > message.timestamp_sample &&
          now - message.timestamp_sample > EKF3_MAX_INPUT_AGE_US)
        {
          status->stale_count++;
          continue;
        }

      fill_core_sample(&message, &sample);
      ekf_delay_push_imu(&g_delay, &sample);

      if (newest_sample_time != NULL)
        {
          *newest_sample_time = sample.timestamp_sample;
        }

      /* The primary monitor sees exactly what the estimator sees, with no
       * aiding and no horizon. Sharing the IMU is the whole point: any
       * attitude difference that develops cannot be the sensor.
       */

      if (status->mon_enabled)
        {
          ekf_core_process(&g_mon_primary, &sample);
        }

      return true;
    }

  return false;
}

static void drain_mag(int sub, FAR struct ekf3_status_s *status)
{
  int drained = 0;

  if (sub < 0)
    {
      return;
    }

  while (drained++ < EKF3_DRAIN_MAX)
    {
      struct vehicle_mag_s message;
      struct ekf_mag_sample_s sample;

      if (orb_copy(ORB_ID(vehicle_mag), sub, &message) < 0)
        {
          return;
        }

      memset(&sample, 0, sizeof(sample));
      sample.timestamp_sample = message.timestamp_sample;
      memcpy(sample.field, message.field, sizeof(sample.field));
      sample.calibrated = message.calibrated != 0;

      ekf_delay_push_mag(&g_delay, &sample);
      status->mag_in++;
    }
}

static void drain_extnav(int sub, FAR struct ekf3_status_s *status)
{
  int drained = 0;

  if (sub < 0)
    {
      return;
    }

  while (drained++ < EKF3_DRAIN_MAX)
    {
      struct external_pose_s message;
      struct ekf_extnav_sample_s sample;
      float marker_position[3];
      float marker_rpy[3];
      float body_position[3];
      float body_rpy[3];
      float body_covariance[6];
      uint64_t receive_time;
      uint64_t corrected_time;
      uint64_t now;
      int64_t age_us = 0;
      int time_result;

      if (orb_copy(ORB_ID(external_pose), sub, &message) < 0)
        {
          return;
        }

      now = ekf_now_us();

      /* external_pose.timestamp is the TIM5 receive constraint captured at
       * the final UART byte. Fall back to drain time only for another
       * publisher that did not provide a usable receive timestamp.
       */

      receive_time = message.timestamp;

      if (receive_time == 0 || receive_time > now ||
          now - receive_time > EKF3_EXT_MAX_AGE_US)
        {
          receive_time = now;
        }

      /* ZERO means "not timestamped": the source has no shared clock yet and
       * is saying so rather than inventing a time. Stamp it on arrival.
       *
       * It has to be a distinct case. The age check below measures against
       * the board's monotonic clock, so a zero would read as "older than the
       * board has been running" and every pose from an untimed source would
       * be refused as stale - which is exactly what happened.
       *
       * Counted separately, so a timesync that IS working is never silently
       * replaced by arrival stamping. Arrival time carries the whole link
       * latency as position error, which is the cost this buys.
       */

      if (message.timestamp_sample == 0)
        {
          message.timestamp_sample = receive_time;
          status->extnav_untimed++;
        }

      /* Match ArduPilot's external-navigation timing protections without
       * throwing away our stronger UTC/TIM5 synchronisation:
       *
       *  - subtract the configured sensor delay;
       *  - a small future or already-overtaken timestamp is clock jitter and
       *    is clamped to the physical boundary;
       *  - a larger error is rejected rather than fused against the wrong
       *    trajectory point.
       */

      time_result = ekf_extnav_time_prepare(message.timestamp_sample,
                                            receive_time,
                                            status->core.last_timestamp_sample,
                                            status->ext_delay_us,
                                            status->ext_jitter_us,
                                            &corrected_time, &age_us);

      status->extnav_age_us = age_us;

      if (status->extnav_age_samples == 0)
        {
          status->extnav_age_min_us = age_us;
          status->extnav_age_max_us = age_us;
        }
      else
        {
          if (age_us < status->extnav_age_min_us)
            {
              status->extnav_age_min_us = age_us;
            }

          if (age_us > status->extnav_age_max_us)
            {
              status->extnav_age_max_us = age_us;
            }
        }

      status->extnav_age_samples++;

      if (time_result < 0)
        {
          status->extnav_bad_time++;
          continue;
        }

      if (time_result > 0)
        {
          status->extnav_time_clamped++;
        }

      memset(&sample, 0, sizeof(sample));
      sample.timestamp_sample = corrected_time;
      marker_position[0] = message.x;
      marker_position[1] = message.y;
      marker_position[2] = 0.0f;
      marker_rpy[0] = 0.0f;
      marker_rpy[1] = 0.0f;
      marker_rpy[2] = message.yaw;

      /* The companion reports the mocap marker origin. The EKF propagates
       * the IMU/body origin. Apply T_map_marker * T_marker_body before the
       * pose enters the delay queue, so both datum creation and innovations
       * refer to the same physical point.
       */

      if (!ekf_extnav_apply_extrinsics(&status->ext_extrinsics,
                                       marker_position, marker_rpy,
                                       body_position, body_rpy))
        {
          status->extnav_bad_frame++;
          continue;
        }

      sample.x = body_position[0];
      sample.y = body_position[1];
      sample.yaw = body_rpy[2];

      /* cov holds VARIANCES; the sample carries sigmas. Zero means the
       * source supplied no estimate, and the floor applies either way.
       */

      if (!ekf_extnav_transform_planar_covariance(
            &status->ext_extrinsics, message.yaw, message.cov,
            body_covariance))
        {
          memset(body_covariance, 0, sizeof(body_covariance));
        }

      sample.pos_sigma[0] = body_covariance[0] > 0.0f ?
                            sqrtf(body_covariance[0]) : 0.0f;
      sample.pos_sigma[1] = body_covariance[3] > 0.0f ?
                            sqrtf(body_covariance[3]) : 0.0f;
      sample.yaw_sigma = body_covariance[5] > 0.0f ?
                         sqrtf(body_covariance[5]) : 0.0f;
      sample.time_sigma = (float)status->ext_jitter_us * 1.0e-6f;
      sample.reset_counter = message.reset_counter;
      sample.valid = (message.flags & EXTERNAL_POSE_VALID) != 0;
      sample.reset_datum =
        (message.flags & EXTERNAL_POSE_RESET_DATUM) != 0;

      ekf_delay_push_extnav(&g_delay, &sample);
      status->extnav_in++;
    }
}

static void drain_baro(int sub, FAR struct ekf3_status_s *status)
{
  int drained = 0;

  if (sub < 0)
    {
      return;
    }

  while (drained++ < EKF3_DRAIN_MAX)
    {
      struct vehicle_baro_s message;
      struct ekf_baro_sample_s sample;

      if (orb_copy(ORB_ID(vehicle_baro), sub, &message) < 0)
        {
          return;
        }

      memset(&sample, 0, sizeof(sample));
      sample.timestamp_sample = message.timestamp_sample;
      sample.pressure = message.pressure;
      sample.temperature = message.temperature;

      ekf_delay_push_baro(&g_delay, &sample);
      status->baro_in++;
    }
}

/* The secondary IMU, feeding the other monitor lane.
 *
 * No delay ring and no aiding - just strapdown plus the continuous tilt
 * reference. Its whole job is to have an opinion about attitude that owes
 * nothing to the primary sensor.
 */

static void drain_monitor_imu(int sub, FAR struct ekf3_status_s *status)
{
  int drained = 0;

  if (sub < 0 || !status->mon_enabled)
    {
      return;
    }

  while (drained++ < EKF3_DRAIN_MAX)
    {
      struct vehicle_imu_s message;
      struct ekf_imu_sample_s sample;
      uint64_t now;

      if (orb_copy(ORB_ID(vehicle_imu), sub, &message) < 0)
        {
          return;
        }

      now = ekf_now_us();

      if (now > message.timestamp_sample &&
          now - message.timestamp_sample > EKF3_MAX_INPUT_AGE_US)
        {
          continue;
        }

      fill_core_sample(&message, &sample);
      ekf_core_process(&g_mon_secondary, &sample);
      status->mon_secondary_live = true;
      status->mon_secondary_status =
        ekf_core_solution_status(&g_mon_secondary);
    }
}

/* Compare the three lanes and hold the verdict.
 *
 * Two questions, and the pairing is what separates them:
 *
 *   published vs monitor_primary   - same IMU, so a difference is the AIDING
 *   monitor_primary vs monitor_secondary - no aiding either side, so a
 *                                    difference is the IMU
 *
 * A difference has to persist for mon_hold_ms before it counts. Attitude
 * disagreement spikes briefly on any hard manoeuvre, when the tilt reference
 * is de-weighted in both monitors at once and they free-run for a moment;
 * calling that a fault would cry wolf on every pothole.
 */

static void monitor_compare(FAR struct ekf3_status_s *status,
                            FAR const float published_quaternion[4],
                            uint64_t now)
{
  bool aiding_bad;
  bool imu_bad;

  if (!status->mon_enabled)
    {
      return;
    }

  status->mon_primary_status = ekf_core_solution_status(&g_mon_primary);

  status->mon_aiding_tilt =
    ekf_core_tilt_difference(published_quaternion, g_mon_primary.quaternion,
                             status->mon_aiding_err);

  if (status->mon_secondary_live)
    {
      status->mon_imu_tilt =
        ekf_core_tilt_difference(g_mon_primary.quaternion,
                                 g_mon_secondary.quaternion,
                                 status->mon_imu_err);
    }

  /* A lane that has not finished aligning has no opinion worth comparing. */

  if (!g_mon_primary.initialized || !status->core.initialized)
    {
      status->mon_aiding_since = 0;
      status->mon_imu_since = 0;
      return;
    }

  aiding_bad = status->mon_aiding_tilt > status->mon_tilt_limit;
  imu_bad = status->mon_secondary_live && g_mon_secondary.initialized &&
            status->mon_imu_tilt > status->mon_tilt_limit;

  if (!aiding_bad)
    {
      status->mon_aiding_since = 0;
      status->mon_aiding_fault = false;
    }
  else if (status->mon_aiding_since == 0)
    {
      status->mon_aiding_since = now;
    }
  else if (now - status->mon_aiding_since >
           (uint64_t)status->mon_hold_ms * 1000ull &&
           !status->mon_aiding_fault)
    {
      status->mon_aiding_fault = true;
      status->mon_aiding_faults++;
      syslog(LOG_ERR, "[ekf3] AIDING fault: the estimator and its own IMU "
             "disagree by %.3f rad\n", (double)status->mon_aiding_tilt);
    }

  if (!imu_bad)
    {
      status->mon_imu_since = 0;
      status->mon_imu_fault = false;
    }
  else if (status->mon_imu_since == 0)
    {
      status->mon_imu_since = now;
    }
  else if (now - status->mon_imu_since >
           (uint64_t)status->mon_hold_ms * 1000ull &&
           !status->mon_imu_fault)
    {
      status->mon_imu_fault = true;
      status->mon_imu_faults++;
      syslog(LOG_ERR, "[ekf3] IMU fault: the two IMUs disagree by %.3f "
             "rad\n", (double)status->mon_imu_tilt);
    }

  /* EKF_MON_ACT is off by default and nothing acts on these yet - the lanes
   * run and report so the numbers can be watched on a real vehicle before a
   * threshold is trusted to drop a solution or reset the filter.
   */
}

/* Take the newest wheel rate and decide whether the vehicle is stopped.
 *
 * The threshold is in tachometer COUNTS PER SECOND on purpose. Zero is zero
 * at any scale, so this decision does not depend on VESC_SPEED_K and stays
 * correct before that scale has ever been calibrated - which is the whole
 * reason a zero-velocity update is worth having early.
 */

static void drain_wheel(int sub, FAR struct ekf3_status_s *status)
{
  struct vesc_status_s wheel;
  struct ekf_wheel_sample_s sample;
  uint64_t corrected_time;
  uint32_t period_us;
  float speed_mps;
  bool stopped;

  if (sub < 0 ||
      (!status->zupt_enabled && !status->wheel_fusion_selected))
    {
      return;
    }

  if (orb_copy(ORB_ID(vesc_status), sub, &wheel) < 0)
    {
      return;
    }

  stopped = fabsf(wheel.speed_cps) <= status->zupt_threshold_cps;
  status->wheel_available = true;
  status->last_wheel_sample_us = wheel.timestamp_sample;
  status->last_wheel_cps = wheel.speed_cps;
  status->wheel_in++;

  if (stopped)
    {
      if (!status->zupt_stopped || status->zupt_stop_since_us == 0 ||
          wheel.timestamp_sample < status->zupt_stop_since_us)
        {
          status->zupt_stop_since_us = wheel.timestamp_sample;
        }
    }
  else
    {
      status->zupt_stop_since_us = 0;
      status->zupt_dwell_complete = false;
    }

  status->zupt_stopped = stopped;

  if (!status->wheel_fusion_selected ||
      !isfinite(wheel.speed_cps) || !isfinite(status->wheel_speed_k) ||
      fabsf(status->wheel_speed_k) < 1.0e-12f ||
      wheel.timestamp_sample == 0)
    {
      if (status->wheel_fusion_selected)
        {
          status->wheel_bad++;
        }

      return;
    }

  speed_mps = wheel.speed_cps * status->wheel_speed_k;
  status->wheel_speed_mps = speed_mps;

  if (!ekf_wheel_accel_update(&g_wheel_accel_filter, speed_mps,
                               wheel.timestamp_sample,
                               status->wheel_accel_tau,
                               &status->wheel_accel_raw))
    {
      status->wheel_accel_filtered = g_wheel_accel_filter.accel_mps2;
      return;
    }

  status->wheel_accel_filtered = g_wheel_accel_filter.accel_mps2;
  period_us = 1000000u / status->wheel_fusion_rate_hz;

  if (status->last_wheel_queued_us != 0 &&
      wheel.timestamp_sample - status->last_wheel_queued_us < period_us)
    {
      status->wheel_decimated++;
      return;
    }

  status->last_wheel_queued_us = wheel.timestamp_sample;
  corrected_time = wheel.timestamp_sample > status->wheel_delay_us ?
                   wheel.timestamp_sample - status->wheel_delay_us : 1u;
  memset(&sample, 0, sizeof(sample));
  sample.timestamp_sample = corrected_time;
  sample.speed_mps = speed_mps;
  sample.accel_mps2 = status->wheel_accel_filtered;

  if (!ekf_delay_push_wheel(&g_delay, &sample))
    {
      status->wheel_overflow = g_delay.wheel_overflow_count;
    }
}

static void publish_output(int publisher, FAR struct ekf3_status_s *status,
                           uint64_t now, uint64_t publication_time)
{
  FAR const struct ekf_imu_sample_s *replay[EKF_IMU_RING_SIZE];
  struct ekf_output_s output;
  struct estimator_state_s message;
  uint16_t count = ekf_delay_output_count(&g_delay);
  uint16_t i;

  for (i = 0; i < count; i++)
    {
      replay[i] = ekf_delay_output_at(&g_delay, i);
    }

  ekf_core_output_predict(&status->core, replay, count, &output);
  status->output_replay = output.samples_replayed;

  /* Compared against the OUTPUT-PREDICTED attitude, which is what actually
   * leaves this daemon, and which is at the same real time as the monitors.
   */

  if (output.valid)
    {
      monitor_compare(status, output.quaternion, now);
    }

  /* Nothing has reached the filter yet. At a 100 ms horizon the first ~40
   * packets are buffered before a single one is released, and publishing
   * here would stamp first_output_us with a zero the reported rate never
   * recovers from.
   */

  if (output.timestamp_sample == 0)
    {
      return;
    }

  fill_output(&status->core, &output, publication_time, &message);

  if (estimator_state_publish(publisher, &message) < 0)
    {
      status->publish_errors++;
    }

  if (status->publish_count == 0)
    {
      status->first_output_us = output.timestamp_sample;
    }

  status->publish_count++;
  status->last_output_us = output.timestamp_sample;
}

static int ekf3_daemon(int argc, FAR char *argv[])
{
  struct ekf3_status_s status;
  struct pollfd fds[4];   /* imu, mag, baro, extnav */
  char source_error[80];
  int nfds = 0;
  int subscriber = -1;
  int mag_sub = -1;
  int baro_sub = -1;
  int extnav_sub = -1;
  int mon_sub = -1;
  int wheel_sub = -1;
  int publisher = -1;
  int diag_publisher = -1;
  int result = EXIT_FAILURE;

  memset(&status, 0, sizeof(status));
  ekf_core_init(&status.core);

  status.clock_skew_start_us = (int64_t)ekf_now_us() -
                               (int64_t)monotonic_now_us();
  status.clock_skew_us = status.clock_skew_start_us;

  status.mon_enabled = param_i32("EKF_MON_EN") != 0;
  status.mon_act = param_i32("EKF_MON_ACT") != 0;
  status.mon_tilt_limit = param_f32("EKF_MON_TILT");
  status.mon_hold_ms = (uint32_t)param_i32("EKF_MON_MS");

  if (status.mon_enabled)
    {
      ekf_core_init(&g_mon_primary);
      ekf_core_init(&g_mon_secondary);
      ekf_core_set_attitude_only(&g_mon_primary, true);
      ekf_core_set_attitude_only(&g_mon_secondary, true);
    }

  if (ekf_sources_load(&status.sources, source_error,
                       sizeof(source_error)) < 0)
    {
      syslog(LOG_ERR, "[ekf3] source configuration rejected: %s\n",
             source_error);
      status_publish(&status);
      goto out;
    }

  status.wheel_fusion_selected =
    ekf_sources_use_velocity_xy(&status.sources, EKF_SOURCE_WHEEL);

  subscriber = orb_subscribe(ORB_ID(vehicle_imu));
  publisher = estimator_state_advertise();
  diag_publisher = estimator_diag_advertise();

  if (subscriber < 0 || publisher < 0 || diag_publisher < 0)
    {
      syslog(LOG_ERR,
             "[ekf3] IMU or estimator publisher unavailable; start "
             "imu_delta first\n");
      goto out;
    }

  /* The aiding topics are optional. Their absence means no aiding, not a
   * failure to start: attitude from the IMU alone is still a useful output,
   * and it is what this estimator produced before there was any aiding.
   */

  mag_sub = orb_subscribe(ORB_ID(vehicle_mag));
  baro_sub = orb_subscribe(ORB_ID(vehicle_baro));
  extnav_sub = orb_subscribe(ORB_ID(external_pose));

  /* Read before the subscription that depends on it, which the parameter
   * block further down is too late for.
   */

  status.zupt_enabled = param_i32("EK3_ZUPT_EN") != 0;
  status.zupt_threshold_cps = param_f32("EK3_ZUPT_CPS");
  status.zupt_noise = param_f32("EK3_ZUPT_NSE");
  status.zupt_gate = param_f32("EK3_ZUPT_GATE");
  status.zupt_gravity_limit = param_f32("EK3_ZUPT_GDEV");
  status.zupt_variance_limit = param_f32("EK3_ZUPT_AVAR");
  status.zupt_dwell_us =
    (uint32_t)param_i32("EK3_ZUPT_DW_MS") * 1000u;
  status.wheel_speed_k = param_f32("VESC_SPEED_K");
  status.wheel_noise = param_f32("EK3_WHL_NSE");
  status.wheel_lateral_noise = param_f32("EK3_WHL_LAT_NSE");
  status.wheel_gate = param_f32("EK3_WHL_GATE");
  status.wheel_accel_tau = param_f32("EK3_WHL_ACC_TC");
  status.wheel_slip_margin = param_f32("EK3_WHL_SLIP");
  status.wheel_fusion_rate_hz = (uint32_t)param_i32("EK3_WHL_RATE");
  status.wheel_position[0] = param_f32("EK3_WHL_POS_X");
  status.wheel_position[1] = param_f32("EK3_WHL_POS_Y");
  {
    float delay_us = param_f32("EK3_WHL_DLY_MS") * 1000.0f;
    status.wheel_delay_us = (uint32_t)(delay_us + 0.5f);
  }

  /* The wheels. Absent if the VESC link is not running, which costs the
   * zero-velocity aid and nothing else.
   */

  if (status.zupt_enabled || status.wheel_fusion_selected)
    {
      wheel_sub = orb_subscribe(ORB_ID(vesc_status));
    }

  /* Instance 1 of vehicle_imu, the secondary IMU. Absent if imu_delta's
   * second lane did not start, which is not a failure here: the estimator
   * runs unchanged and only the IMU cross-check is lost.
   */

  if (status.mon_enabled)
    {
      mon_sub = orb_subscribe_multi(ORB_ID(vehicle_imu), 1);

      if (mon_sub < 0)
        {
          syslog(LOG_WARNING, "[ekf3] no vehicle_imu instance 1 (%d); the "
                 "IMU cross-check is inactive\n", errno);
        }

      status.mon_subscribed = mon_sub >= 0;
    }
  status.mag_available = mag_sub >= 0;
  status.baro_available = baro_sub >= 0;
  status.extnav_available = extnav_sub >= 0;

  if (mag_sub < 0 || baro_sub < 0)
    {
      syslog(LOG_WARNING,
             "[ekf3] %s%s%s unavailable; run `sensors aux start` for aiding\n",
             mag_sub < 0 ? "vehicle_mag" : "",
             mag_sub < 0 && baro_sub < 0 ? " and " : "",
             baro_sub < 0 ? "vehicle_baro" : "");
    }

  status.horizon_ms = (uint32_t)param_i32("EK3_DELAY_MS");
  status.alt_noise = param_f32("EK3_ALT_M_NSE");
  status.alt_gate = param_f32("EK3_ALT_I_GATE");
  status.ext_noise = param_f32("EK3_EXT_M_NSE");
  status.ext_gate = param_f32("EK3_EXT_I_GATE");
  status.ext_yaw_noise = param_f32("EK3_EXT_YAW_NSE");
  status.ext_extrinsics.position[0] = param_f32("EK3_EXT_POS_X");
  status.ext_extrinsics.position[1] = param_f32("EK3_EXT_POS_Y");
  status.ext_extrinsics.position[2] = param_f32("EK3_EXT_POS_Z");
  status.ext_extrinsics.rotation[0] =
    param_f32("EK3_EXT_ROLL") * 0.017453292519943295f;
  status.ext_extrinsics.rotation[1] =
    param_f32("EK3_EXT_PITCH") * 0.017453292519943295f;
  status.ext_extrinsics.rotation[2] =
    param_f32("EK3_EXT_YAW") * 0.017453292519943295f;
  {
    float delay_us = param_f32("EK3_EXT_DLY_MS") * 1000.0f;
    float jitter_us = param_f32("EK3_EXT_JIT_MS") * 1000.0f;

    status.ext_delay_us = (int32_t)(delay_us >= 0.0f ?
                                    delay_us + 0.5f : delay_us - 0.5f);
    status.ext_jitter_us = (uint32_t)(jitter_us + 0.5f);
  }
  status.ext_timeout_ms = (uint32_t)param_i32("EK3_EXT_TIMEOUT");
  ekf_core_set_extnav_config(&status.core, status.ext_timeout_ms * 1000u);
  status.height_limit = param_f32("EK3_HGT_LIM");
  ekf_core_set_height_limit(&status.core, status.height_limit);
  ekf_core_set_tilt_fusion_moving(&status.core,
                                  param_i32("EK3_TILT_MOVE") != 0);
  ekf_core_set_process_noise(&status.core, param_f32("EK3_GYR_P_NSE"),
                             param_f32("EK3_ACC_P_NSE"),
                             param_f32("EK3_GBIAS_P_NSE"),
                             param_f32("EK3_ABIAS_P_NSE"));
  status.position_hold_limit = param_f32("EK3_POSHOLD_M");
  ekf_core_set_position_hold(&status.core, status.position_hold_limit);
  ekf_core_set_bias_limits(&status.core, param_f32("EK3_GBIAS_LIM"),
                           param_f32("EK3_ABIAS_LIM"));
  ekf_core_set_bias_learning(&status.core, param_i32("EK3_GBIAS_EN") != 0,
                             param_i32("EK3_ABIAS_EN") != 0);
  status.declination = param_f32("EK3_MAG_DEC") * 0.017453292519943295f;
  status.yaw_noise = param_f32("EK3_YAW_M_NSE");
  status.yaw_gate = param_f32("EK3_YAW_I_GATE");
  status.mag_expected = param_f32("CAL_MAG0_FIELD");

  /* The core reads no parameters itself, so hand it what alignment needs. */

  ekf_core_set_mag_config(&status.core, status.declination,
                          status.yaw_noise * status.yaw_noise);
  ekf_core_set_wheel_config(&status.core, EKF3_WHEEL_MAX_AGE_US);
  ekf_delay_init(&g_delay, status.horizon_ms);
  ekf_wheel_accel_init(&g_wheel_accel_filter);
  memset(&g_imu_accel_filter, 0, sizeof(g_imu_accel_filter));

  fds[nfds].fd = subscriber;
  fds[nfds].events = POLLIN;
  nfds++;

  if (mag_sub >= 0)
    {
      fds[nfds].fd = mag_sub;
      fds[nfds].events = POLLIN;
      nfds++;
    }

  if (baro_sub >= 0)
    {
      fds[nfds].fd = baro_sub;
      fds[nfds].events = POLLIN;
      nfds++;
    }

  if (extnav_sub >= 0)
    {
      fds[nfds].fd = extnav_sub;
      fds[nfds].events = POLLIN;
      nfds++;
    }

  g_running = true;
  status.running = true;
  status_publish(&status);

  syslog(LOG_INFO,
         "[ekf3] 15-state, horizon %" PRIu32 " ms, sources %u\n",
         status.horizon_ms, status.sources.active_set + 1u);

  while (!g_should_stop)
    {
      struct ekf_imu_sample_s sample;
      uint64_t now;
      uint64_t publication_time;
      uint64_t newest_sample_time;
      uint64_t horizon;
      int ready = poll(fds, nfds, 100);

      if (ready < 0 && errno != EINTR)
        {
          break;
        }

      /* Honour a commanded reset here, on the daemon's own thread, before
       * anything is drained. The ring and the measurement queues go with it:
       * everything buffered was sampled against the trajectory being
       * abandoned, and replaying it afterwards would rebuild the state the
       * reset was asked to discard.
       */

      if (g_should_reset)
        {
          ekf_core_reset(&status.core);
          ekf_delay_init(&g_delay, status.horizon_ms);
          ekf_wheel_accel_init(&g_wheel_accel_filter);
          memset(&g_imu_accel_filter, 0, sizeof(g_imu_accel_filter));
          status.last_wheel_queued_us = 0;
          status.wheel_accel_raw = 0.0f;
          status.wheel_accel_filtered = 0.0f;
          status.imu_accel_filtered = 0.0f;
          status.wheel_slipping = false;
          status.wheel_diag_timestamp = 0;
          status.reset_requests++;
          g_should_reset = false;
          status_publish(&status);
        }

      drain_mag(mag_sub, &status);
      drain_baro(baro_sub, &status);
      drain_extnav(extnav_sub, &status);

      /* One packet in, at most one publication out, so estimator_state keeps
       * the input's rate and every publication carries a distinct sample
       * time. With nothing new there is nothing to say.
       */

      if (!take_imu_sample(subscriber, &status, &newest_sample_time))
        {
          status.imu_overflow = g_delay.imu_overflow_count;
          status.mag_overflow = g_delay.mag_overflow_count;
          status.extnav_overflow = g_delay.extnav_overflow_count;
          status.baro_overflow = g_delay.baro_overflow_count;
          status.wheel_overflow = g_delay.wheel_overflow_count;
          status_publish(&status);
          continue;
        }

      now = ekf_now_us();
      publication_time = monotonic_now_us();
      status.clock_skew_us = (int64_t)now - (int64_t)publication_time;

      /* Read the wheel topic before deciding whether this IMU packet may
       * receive a zero-velocity update. A remembered zero from a dead VESC
       * link is not a velocity measurement.
       */

      drain_wheel(wheel_sub, &status);
      status.wheel_fresh = status.last_wheel_sample_us != 0 &&
                           now >= status.last_wheel_sample_us &&
                           now - status.last_wheel_sample_us <=
                             EKF3_WHEEL_MAX_AGE_US;

      /* The newest IMU sample, not task wake-up time, is the common time
       * reference.  This is the time ArduPilot calls imuSampleTime: it makes
       * the delayed horizon deterministic when the estimator task is
       * scheduled late and removes wall-clock phase from fusion entirely.
       */

      horizon = ekf_delay_horizon_time(&g_delay, newest_sample_time);

      /* Advance the filter to the horizon. Aiding measurements are fused at
       * the point on the trajectory where they were actually taken, which is
       * the whole reason the horizon exists.
       */

      while (ekf_delay_next_imu(&g_delay, horizon, &sample))
        {
          FAR const struct ekf_source_set_s *source =
            &status.sources.set[status.sources.active_set];
          struct ekf_baro_sample_s baro;
          struct ekf_mag_sample_s mag;
          struct ekf_extnav_sample_s ext;
          struct ekf_extnav_sample_s extnav_attempted;
          uint64_t extnav_recall_time;
          bool have_extnav_attempt = false;
          uint32_t extnav_accept_before =
            status.core.extnav_accept_count;
          uint32_t extnav_reject_before =
            status.core.extnav_reject_count;
          uint32_t zupt_accept_before = status.core.zupt_accept_count;
          uint32_t zupt_reject_before = status.core.zupt_reject_count;
          uint32_t gravity_accept_before =
            status.core.gravity_accept_count;
          uint32_t gravity_reject_before =
            status.core.gravity_reject_count;
          int process_result;

          process_result = ekf_core_process(&status.core, &sample);

          if (process_result == EKF_PROCESS_REJECTED)
            {
              publish_diagnostics(diag_publisher, &status, &sample,
                                  NULL, process_result,
                                  extnav_accept_before,
                                  extnav_reject_before,
                                  zupt_accept_before,
                                  zupt_reject_before,
                                  gravity_accept_before,
                                  gravity_reject_before);
              continue;
            }

          if (process_result == EKF_PROCESS_PREDICTED &&
              status.wheel_fusion_selected)
            {
              status.imu_accel_filtered = ekf_wheel_lpf_update(
                &g_imu_accel_filter,
                status.core.last_residual_accel_body[0],
                sample.delta_velocity_dt, status.wheel_accel_tau);
            }

          /* Wheel velocity is recalled to this delayed IMU state, just like
           * every other aiding measurement. VESC speed supplies body X; the
           * z gyro supplies heading propagation and omega-cross-r lever-arm
           * motion. The core additionally fuses the car's lateral body
           * velocity constraint, producing an x/y navigation reference.
           */

          {
            struct ekf_wheel_sample_s wheel_sample;

            status.wheel_diag_timestamp = 0;
            status.wheel_diag_speed_mps = 0.0f;
            status.wheel_diag_accel_mps2 = 0.0f;
            status.wheel_diag_imu_accel_mps2 = 0.0f;

            while (ekf_delay_next_wheel(&g_delay,
                                         sample.timestamp_sample,
                                         EKF3_WHEEL_MAX_AGE_US,
                                         &wheel_sample))
              {
                float stop_mps = fabsf(status.wheel_speed_k) *
                                 status.zupt_threshold_cps;
                bool wheel_stopped = fabsf(wheel_sample.speed_mps) <=
                                     stop_mps;

                status.wheel_diag_timestamp =
                  wheel_sample.timestamp_sample;
                status.wheel_diag_speed_mps = wheel_sample.speed_mps;
                status.wheel_diag_accel_mps2 = wheel_sample.accel_mps2;
                status.wheel_diag_imu_accel_mps2 =
                  status.imu_accel_filtered;

                status.wheel_slipping = ekf_wheel_slipping(
                  wheel_sample.accel_mps2, status.imu_accel_filtered,
                  status.wheel_slip_margin);
                if (sample.delta_angle_dt > 0.0f)
                  {
                    status.wheel_yaw_rate =
                      sample.delta_angle[2] / sample.delta_angle_dt -
                      status.core.gyro_bias[2];
                  }
                else
                  {
                    status.wheel_yaw_rate = 0.0f;
                  }

                if (!status.core.initialized || wheel_stopped)
                  {
                    continue;
                  }

                if (status.wheel_slipping)
                  {
                    status.wheel_slip_block_count++;
                    continue;
                  }

                ekf_core_fuse_wheel_velocity(
                  &status.core, wheel_sample.speed_mps,
                  status.wheel_yaw_rate, status.wheel_position,
                  status.wheel_noise, status.wheel_lateral_noise,
                  status.wheel_gate);
              }
          }

          /* Source selection makes a measurement ELIGIBLE. The health gating
           * inside the fusion decides whether it is USED. A parameter never
           * makes a bad measurement good.
           *
           * Drained either way, so a deselected source cannot fill its queue
           * and report a misleading overflow.
           */

          /* Zero velocity, while the wheels say the vehicle is stopped.
           *
           * Fused per released IMU sample rather than per wheel message so
           * it keeps pulling for as long as the vehicle is stationary - a
           * single update at the moment of stopping would be undone by the
           * next second of accelerometer error.
           */

          status.zupt_imu_stationary =
            ekf_core_zupt_stationary(&status.core,
                                     status.zupt_gravity_limit,
                                     status.zupt_variance_limit,
                                     &status.zupt_gravity_deviation,
                                     &status.zupt_accel_variance);

          status.zupt_dwell_complete =
            status.zupt_stopped && status.zupt_stop_since_us != 0 &&
            sample.timestamp_sample >= status.zupt_stop_since_us &&
            sample.timestamp_sample - status.zupt_stop_since_us >=
              status.zupt_dwell_us;

          if (status.zupt_enabled && status.zupt_stopped &&
              status.wheel_fresh && status.zupt_dwell_complete &&
              status.zupt_imu_stationary)
            {
              ekf_core_fuse_zero_velocity(&status.core, status.zupt_noise,
                                          status.zupt_gate);
            }
          else if (status.zupt_enabled && status.zupt_stopped &&
                   status.wheel_fresh && status.core.initialized)
            {
              status.zupt_motion_block_count++;
            }

          while (ekf_delay_next_baro(&g_delay, sample.timestamp_sample,
                                     EKF3_BARO_MAX_AGE_US, &baro))
            {
              if (source->position_z == EKF_SOURCE_BARO_OR_COMPASS)
                {
                  ekf_core_fuse_baro(&status.core, baro.pressure,
                                     status.alt_noise, status.alt_gate);
                }
            }

          while (ekf_delay_next_mag(&g_delay, sample.timestamp_sample,
                                    EKF3_MAG_MAX_AGE_US, &mag))
            {
              if (!mag.calibrated ||
                  source->yaw != EKF_SOURCE_BARO_OR_COMPASS)
                {
                  continue;
                }

              /* Before alignment completes the field feeds the heading
               * initialisation; afterwards it corrects it. The core decides
               * which, so this cannot get the order wrong.
               */

              if (!status.core.initialized)
                {
                  ekf_core_add_align_mag(&status.core, mag.field);
                  status.mag_align_used++;
                }
              else
                {
                  ekf_core_fuse_mag(&status.core, mag.field,
                                    status.declination,
                                    status.mag_expected,
                                    status.yaw_noise, status.yaw_gate);
                }
            }

          /* External pose is fused on the nearest IMU state.  The delayed
           * trajectory is discrete (400 Hz here), so recalling through half
           * an update interval is equivalent to ArduPilot subtracting half
           * localFilterTimeStep_ms from each external-nav timestamp.  It
           * halves the worst quantisation error without altering the source
           * timestamp itself.
           */

          extnav_recall_time = sample.timestamp_sample +
            (uint64_t)(sample.delta_angle_dt * 500000.0f);

          while (ekf_delay_next_extnav(&g_delay, extnav_recall_time,
                                       EKF3_EXT_MAX_AGE_US, &ext))
            {
              bool want_position =
                source->position_xy == EKF_SOURCE_EXTERNAL_NAV;
              bool want_yaw = source->yaw == EKF_SOURCE_EXTERNAL_NAV;

              if (want_position || want_yaw)
                {
                  extnav_attempted = ext;
                  have_extnav_attempt = true;
                  ekf_core_fuse_extnav(&status.core, &ext,
                                       status.ext_noise, status.ext_gate,
                                       status.ext_yaw_noise,
                                       status.yaw_gate,
                                       want_position, want_yaw);
                }
            }

          publish_diagnostics(diag_publisher, &status, &sample,
                              have_extnav_attempt ? &extnav_attempted : NULL,
                              process_result,
                              extnav_accept_before,
                              extnav_reject_before,
                              zupt_accept_before,
                              zupt_reject_before,
                              gravity_accept_before,
                              gravity_reject_before);
        }

      /* Publish once per PACKET, not once per sample the horizon released.
       *
       * Those are not the same event and do not line up one to one: a sample
       * is released when the horizon time crosses its timestamp, and `now` is
       * read once per iteration, so some iterations release two and some
       * release none. Gating publication on a release dropped the output to
       * about 250 Hz against 400 Hz of input.
       *
       * The packet is the right trigger because it is what moves the newest
       * ring entry - the point the output is re-propagated TO. Each new
       * packet therefore adds one strapdown step to the result and carries a
       * distinct sample time, whether or not the filter itself advanced this
       * iteration.
       */

      drain_monitor_imu(mon_sub, &status);
      publish_output(publisher, &status, now, publication_time);

      status.imu_overflow = g_delay.imu_overflow_count;
      status.mag_overflow = g_delay.mag_overflow_count;
      status.baro_overflow = g_delay.baro_overflow_count;
      status.extnav_overflow = g_delay.extnav_overflow_count;
      status.wheel_overflow = g_delay.wheel_overflow_count;
      status_publish(&status);
    }

  result = EXIT_SUCCESS;

out:
  status.running = false;
  status_publish(&status);

  if (subscriber >= 0)
    {
      orb_unsubscribe(subscriber);
    }

  if (mag_sub >= 0)
    {
      orb_unsubscribe(mag_sub);
    }

  if (baro_sub >= 0)
    {
      orb_unsubscribe(baro_sub);
    }

  if (extnav_sub >= 0)
    {
      orb_unsubscribe(extnav_sub);
    }

  if (mon_sub >= 0)
    {
      orb_unsubscribe(mon_sub);
    }

  if (wheel_sub >= 0)
    {
      orb_unsubscribe(wheel_sub);
    }

  if (publisher >= 0)
    {
      close(publisher);
    }

  if (diag_publisher >= 0)
    {
      close(diag_publisher);
    }

  g_running = false;
  return result;
}

int ekf3_start(void)
{
  int task;
  int wait;

  if (g_running)
    {
      return -EALREADY;
    }

  g_should_stop = false;
  g_should_reset = false;
  task = task_create("ekf3", EKF3_PRIORITY, EKF3_STACK,
                     ekf3_daemon, NULL);

  if (task < 0)
    {
      return -errno;
    }

  for (wait = 0; wait < 100 && !g_running; wait++)
    {
      usleep(10000);
    }

  return g_running ? 0 : -EIO;
}

int ekf3_reset(void)
{
  int wait;

  if (!g_running)
    {
      return -ESRCH;
    }

  g_should_reset = true;

  /* Wait for the daemon to clear the flag, so the caller is told the reset
   * HAPPENED rather than merely that it was asked for.
   */

  for (wait = 0; wait < 100 && g_should_reset; wait++)
    {
      usleep(10000);
    }

  return g_should_reset ? -ETIMEDOUT : 0;
}

int ekf3_stop(void)
{
  int wait;

  if (!g_running)
    {
      return -ENOENT;
    }

  g_should_stop = true;

  for (wait = 0; wait < 100 && g_running; wait++)
    {
      usleep(10000);
    }

  return g_running ? -ETIMEDOUT : 0;
}

void ekf3_status(FAR struct ekf3_status_s *status)
{
  pthread_mutex_lock(&g_lock);
  *status = g_status;
  pthread_mutex_unlock(&g_lock);
}
