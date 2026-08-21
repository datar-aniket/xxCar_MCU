/****************************************************************************
 * apps/ekf3/ekf3.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
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
#include "../uorb_msgs/uorb_msgs.h"

#define EKF3_PRIORITY          (SCHED_PRIORITY_DEFAULT + 22)
#define EKF3_STACK             6144
#define EKF3_DRAIN_MAX           32
#define EKF3_MAX_INPUT_AGE_US  50000ull

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool g_running;
static volatile bool g_should_stop;
static struct ekf3_status_s g_status;

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

static void fill_output(FAR const struct ekf_core_s *core,
                        uint64_t publication_time,
                        FAR struct estimator_state_s *output)
{
  int axis;

  memset(output, 0, sizeof(*output));
  output->timestamp = publication_time;
  output->timestamp_sample = core->last_timestamp_sample;
  memcpy(output->quaternion, core->quaternion,
         sizeof(output->quaternion));
  memcpy(output->velocity, core->velocity, sizeof(output->velocity));
  memcpy(output->position, core->position, sizeof(output->position));
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

static int ekf3_daemon(int argc, FAR char *argv[])
{
  struct ekf3_status_s status;
  struct pollfd pollfd;
  char source_error[80];
  int subscriber = -1;
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

  pollfd.fd = subscriber;
  pollfd.events = POLLIN;
  g_running = true;
  status.running = true;
  status_publish(&status);

  syslog(LOG_INFO,
         "[ekf3] 15-state prediction 400 Hz, covariance 100 Hz, sources %u\n",
         status.sources.active_set + 1u);

  while (!g_should_stop)
    {
      int ready = poll(&pollfd, 1, 100);

      if (ready < 0 && errno != EINTR)
        {
          break;
        }

      if ((pollfd.revents & POLLIN) != 0)
        {
          int drained = 0;

          while (drained++ < EKF3_DRAIN_MAX)
            {
              struct vehicle_imu_s message;
              struct ekf_imu_sample_s sample;
              struct estimator_state_s output;
              uint64_t publication_time;
              int process_result;

              if (orb_copy(ORB_ID(vehicle_imu), subscriber, &message) < 0)
                {
                  break;
                }

              publication_time = now_us();

              if (publication_time > message.timestamp_sample &&
                  publication_time - message.timestamp_sample >
                  EKF3_MAX_INPUT_AGE_US)
                {
                  status.stale_count++;
                  continue;
                }

              fill_core_sample(&message, &sample);
              process_result = ekf_core_process(&status.core, &sample);

              if (process_result == EKF_PROCESS_REJECTED)
                {
                  continue;
                }

              fill_output(&status.core, publication_time, &output);

              if (estimator_state_publish(publisher, &output) < 0)
                {
                  status.publish_errors++;
                }

              if (status.publish_count == 0)
                {
                  status.first_output_us = message.timestamp_sample;
                }

              status.publish_count++;
              status.last_output_us = message.timestamp_sample;

              if ((status.publish_count & 15u) == 0)
                {
                  status_publish(&status);
                }
            }
        }
    }

  result = EXIT_SUCCESS;

out:
  status.running = false;
  status_publish(&status);

  if (subscriber >= 0)
    {
      orb_unsubscribe(subscriber);
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
