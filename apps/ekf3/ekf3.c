/****************************************************************************
 * apps/ekf3/ekf3.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
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
#include "../param/param.h"
#include "../uorb_msgs/uorb_msgs.h"

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

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool g_running;
static volatile bool g_should_stop;
static volatile bool g_should_reset;
static struct ekf3_status_s g_status;

/* ~3 kB. A file-scope static, not a member of g_status and not on the stack:
 * ekf3 runs on 6144 bytes and g_status is copied wholesale under a mutex.
 */

static struct ekf_delay_s g_delay;

static uint64_t now_us(void)
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

static bool take_imu_sample(int sub, FAR struct ekf3_status_s *status)
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

      now = now_us();

      if (now > message.timestamp_sample &&
          now - message.timestamp_sample > EKF3_MAX_INPUT_AGE_US)
        {
          status->stale_count++;
          continue;
        }

      fill_core_sample(&message, &sample);
      ekf_delay_push_imu(&g_delay, &sample);
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

static void publish_output(int publisher, FAR struct ekf3_status_s *status,
                           uint64_t now)
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

  /* Nothing has reached the filter yet. At a 100 ms horizon the first ~40
   * packets are buffered before a single one is released, and publishing
   * here would stamp first_output_us with a zero the reported rate never
   * recovers from.
   */

  if (output.timestamp_sample == 0)
    {
      return;
    }

  fill_output(&status->core, &output, now, &message);

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
  struct pollfd fds[3];
  char source_error[80];
  int nfds = 0;
  int subscriber = -1;
  int mag_sub = -1;
  int baro_sub = -1;
  int publisher = -1;
  int result = EXIT_FAILURE;

  memset(&status, 0, sizeof(status));
  ekf_core_init(&status.core);

  if (ekf_sources_load(&status.sources, source_error,
                       sizeof(source_error)) < 0)
    {
      syslog(LOG_ERR, "[ekf3] source configuration rejected: %s\n",
             source_error);
      status_publish(&status);
      goto out;
    }

  subscriber = orb_subscribe(ORB_ID(vehicle_imu));
  publisher = estimator_state_advertise();

  if (subscriber < 0 || publisher < 0)
    {
      syslog(LOG_ERR,
             "[ekf3] vehicle_imu unavailable; start imu_delta first\n");
      goto out;
    }

  /* The aiding topics are optional. Their absence means no aiding, not a
   * failure to start: attitude from the IMU alone is still a useful output,
   * and it is what this estimator produced before there was any aiding.
   */

  mag_sub = orb_subscribe(ORB_ID(vehicle_mag));
  baro_sub = orb_subscribe(ORB_ID(vehicle_baro));
  status.mag_available = mag_sub >= 0;
  status.baro_available = baro_sub >= 0;

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
  status.declination = param_f32("EK3_MAG_DEC") * 0.017453292519943295f;
  status.yaw_noise = param_f32("EK3_YAW_M_NSE");
  status.yaw_gate = param_f32("EK3_YAW_I_GATE");
  status.mag_expected = param_f32("CAL_MAG0_FIELD");

  /* The core reads no parameters itself, so hand it what alignment needs. */

  ekf_core_set_mag_config(&status.core, status.declination,
                          status.yaw_noise * status.yaw_noise);
  ekf_delay_init(&g_delay, status.horizon_ms);

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
          status.reset_requests++;
          g_should_reset = false;
          status_publish(&status);
        }

      drain_mag(mag_sub, &status);
      drain_baro(baro_sub, &status);

      /* One packet in, at most one publication out, so estimator_state keeps
       * the input's rate and every publication carries a distinct sample
       * time. With nothing new there is nothing to say.
       */

      if (!take_imu_sample(subscriber, &status))
        {
          status.imu_overflow = g_delay.imu_overflow_count;
          status.mag_overflow = g_delay.mag_overflow_count;
          status.baro_overflow = g_delay.baro_overflow_count;
          status_publish(&status);
          continue;
        }

      now = now_us();
      horizon = ekf_delay_horizon_time(&g_delay, now);

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

          if (ekf_core_process(&status.core, &sample) ==
              EKF_PROCESS_REJECTED)
            {
              continue;
            }

          /* Source selection makes a measurement ELIGIBLE. The health gating
           * inside the fusion decides whether it is USED. A parameter never
           * makes a bad measurement good.
           *
           * Drained either way, so a deselected source cannot fill its queue
           * and report a misleading overflow.
           */

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

      publish_output(publisher, &status, now);

      status.imu_overflow = g_delay.imu_overflow_count;
      status.mag_overflow = g_delay.mag_overflow_count;
      status.baro_overflow = g_delay.baro_overflow_count;
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

  if (publisher >= 0)
    {
      close(publisher);
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
