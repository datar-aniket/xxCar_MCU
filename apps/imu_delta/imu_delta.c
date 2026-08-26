/****************************************************************************
 * apps/imu_delta/imu_delta.c
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/uorb.h>
#include <uORB/uORB.h>

#include "imu_delta.h"
#include "imu_integrator.h"
#include "../param/param.h"
#include "../sensors/rotation.h"
#include "../uorb_msgs/uorb_msgs.h"

#define IMU_DELTA_PRIORITY       (SCHED_PRIORITY_DEFAULT + 25)
#define IMU_DELTA_STACK          4096
#define IMU_DELTA_DRAIN_MAX      64
#define IMU_DELTA_QUEUE_SIZE     32
#define ICM_ACCEL_CLIP_M_S2      (0.98f * 16.0f * 9.80665f)
#define ICM_GYRO_CLIP_RAD_S      (0.98f * 2000.0f * 0.017453292519943295f)

struct accel_queue_s
{
  struct sensor_accel sample[IMU_DELTA_QUEUE_SIZE];
  uint16_t head;
  uint16_t count;
};

struct gyro_queue_s
{
  struct sensor_gyro sample[IMU_DELTA_QUEUE_SIZE];
  uint16_t head;
  uint16_t count;
};

struct axis_map_s
{
  int8_t index[3];
  int8_t sign[3];
};

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

/* One set of state per IMU. Instance 0 is the primary (ICM-42688) and feeds
 * the estimator; instance 1 is the secondary (BMI055) and feeds the monitor
 * lane, which exists to disagree with the primary when something is wrong.
 *
 * Two tasks rather than one loop over four descriptors: the pairing and gap
 * logic is per-IMU and already reads as one sensor's story. Interleaving two
 * would make every counter ambiguous.
 */

static volatile bool g_running[IMU_DELTA_INSTANCES];
static volatile bool g_should_stop[IMU_DELTA_INSTANCES];
static struct imu_delta_status_s g_status[IMU_DELTA_INSTANCES];

/* The task's own instance, read from argv. task_create's argv is copied, so
 * a pointer to a local would dangle; the index is passed as a decimal string
 * and parsed back.
 */

static int daemon_instance(int argc, FAR char *argv[])
{
  int instance = 0;

  if (argc > 1 && argv[1] != NULL)
    {
      instance = atoi(argv[1]);
    }

  return (instance >= 0 && instance < IMU_DELTA_INSTANCES) ? instance : 0;
}

static uint64_t now_us(void)
{
  struct timespec timestamp;

  clock_gettime(CLOCK_MONOTONIC, &timestamp);
  return (uint64_t)timestamp.tv_sec * 1000000ull +
         (uint64_t)timestamp.tv_nsec / 1000ull;
}

static void load_cal(FAR const char *prefix, bool accel, FAR float offset[3],
                     FAR float scale[3], FAR bool *calibrated)
{
  static const char axis[3] = {'X', 'Y', 'Z'};
  char name[PARAM_NAME_MAX + 1];
  int index;

  for (index = 0; index < 3; index++)
    {
      offset[index] = 0.0f;
      scale[index] = 1.0f;
    }

  snprintf(name, sizeof(name), "%s_OK", prefix);
  *calibrated = param_i32(name) == 1;

  if (!*calibrated)
    {
      return;
    }

  for (index = 0; index < 3; index++)
    {
      snprintf(name, sizeof(name), "%s_%cOFF", prefix, axis[index]);
      offset[index] = param_f32(name);

      if (accel)
        {
          snprintf(name, sizeof(name), "%s_%cSCL", prefix, axis[index]);
          scale[index] = param_f32(name);
        }
    }
}

static uint8_t clipping_bits(FAR const struct sensor_accel *accel,
                             FAR const struct sensor_gyro *gyro)
{
  const float a[3] = {accel->x, accel->y, accel->z};
  const float g[3] = {gyro->x, gyro->y, gyro->z};
  uint8_t bits = 0;
  int axis;

  for (axis = 0; axis < 3; axis++)
    {
      if (fabsf(a[axis]) >= ICM_ACCEL_CLIP_M_S2)
        {
          bits |= (uint8_t)(1u << axis);
        }

      if (fabsf(g[axis]) >= ICM_GYRO_CLIP_RAD_S)
        {
          bits |= (uint8_t)(1u << (axis + 3));
        }
    }

  return bits;
}

static bool build_axis_map(uint8_t sensor_rotation, uint8_t board_rotation,
                           FAR struct axis_map_s *map)
{
  int source;

  memset(map, 0, sizeof(*map));

  for (source = 0; source < 3; source++)
    {
      float basis[3] = {0.0f, 0.0f, 0.0f};
      int destination;

      basis[source] = 1.0f;

      if (!rotation_apply(sensor_rotation, basis) ||
          !rotation_apply(board_rotation, basis))
        {
          return false;
        }

      for (destination = 0; destination < 3; destination++)
        {
          if (basis[destination] != 0.0f)
            {
              map->index[destination] = (int8_t)source;
              map->sign[destination] = basis[destination] > 0.0f ? 1 : -1;
            }
        }
    }

  return true;
}

static void map_apply(FAR const struct axis_map_s *map, FAR float value[3])
{
  float input[3] = {value[0], value[1], value[2]};

  value[0] = map->sign[0] * input[map->index[0]];
  value[1] = map->sign[1] * input[map->index[1]];
  value[2] = map->sign[2] * input[map->index[2]];
}

static void correct_pair(FAR const struct sensor_accel *accel,
                         FAR const struct sensor_gyro *gyro,
                         FAR const float accel_offset[3],
                         FAR const float accel_scale[3],
                         FAR const float gyro_offset[3],
                         FAR const struct axis_map_s *map,
                         FAR float corrected_accel[3],
                         FAR float corrected_gyro[3])
{
  corrected_accel[0] = (accel->x - accel_offset[0]) * accel_scale[0];
  corrected_accel[1] = (accel->y - accel_offset[1]) * accel_scale[1];
  corrected_accel[2] = (accel->z - accel_offset[2]) * accel_scale[2];
  corrected_gyro[0] = gyro->x - gyro_offset[0];
  corrected_gyro[1] = gyro->y - gyro_offset[1];
  corrected_gyro[2] = gyro->z - gyro_offset[2];

  map_apply(map, corrected_accel);
  map_apply(map, corrected_gyro);
}

static void accel_push(FAR struct accel_queue_s *queue,
                       FAR const struct sensor_accel *sample,
                       FAR uint32_t *overruns)
{
  uint16_t tail;

  if (queue->count == IMU_DELTA_QUEUE_SIZE)
    {
      queue->head = (uint16_t)((queue->head + 1u) % IMU_DELTA_QUEUE_SIZE);
      queue->count--;
      (*overruns)++;
    }

  tail = (uint16_t)((queue->head + queue->count) % IMU_DELTA_QUEUE_SIZE);
  queue->sample[tail] = *sample;
  queue->count++;
}

static void gyro_push(FAR struct gyro_queue_s *queue,
                      FAR const struct sensor_gyro *sample,
                      FAR uint32_t *overruns)
{
  uint16_t tail;

  if (queue->count == IMU_DELTA_QUEUE_SIZE)
    {
      queue->head = (uint16_t)((queue->head + 1u) % IMU_DELTA_QUEUE_SIZE);
      queue->count--;
      (*overruns)++;
    }

  tail = (uint16_t)((queue->head + queue->count) % IMU_DELTA_QUEUE_SIZE);
  queue->sample[tail] = *sample;
  queue->count++;
}

static FAR struct sensor_accel *accel_front(FAR struct accel_queue_s *queue)
{
  return &queue->sample[queue->head];
}

static FAR struct sensor_gyro *gyro_front(FAR struct gyro_queue_s *queue)
{
  return &queue->sample[queue->head];
}

static void accel_pop(FAR struct accel_queue_s *queue)
{
  queue->head = (uint16_t)((queue->head + 1u) % IMU_DELTA_QUEUE_SIZE);
  queue->count--;
}

static void gyro_pop(FAR struct gyro_queue_s *queue)
{
  queue->head = (uint16_t)((queue->head + 1u) % IMU_DELTA_QUEUE_SIZE);
  queue->count--;
}

static void copy_integrator_status(FAR struct imu_delta_status_s *status,
                                   FAR const struct imu_integrator_s *state)
{
  status->resets = state->resets;
  status->gaps = state->gaps;
  status->duplicates = state->duplicates;
  status->backwards = state->backwards;
  status->invalid = state->invalid;
}

static void publish_status(int instance,
                           FAR const struct imu_delta_status_s *local,
                           FAR const struct imu_integrator_s *integrator)
{
  pthread_mutex_lock(&g_lock);
  g_status[instance] = *local;
  copy_integrator_status(&g_status[instance], integrator);
  g_status[instance].running = true;
  pthread_mutex_unlock(&g_lock);
}

static void account_packet(FAR struct imu_delta_status_s *status,
                           FAR const struct imu_delta_output_s *delta)
{
  uint64_t window_us = delta->last_timestamp_us - delta->first_timestamp_us;
  int axis;

  if (status->packets == 0)
    {
      status->first_packet_us = delta->last_timestamp_us;
      status->min_window_us = (uint32_t)window_us;
      status->max_window_us = (uint32_t)window_us;
      status->min_samples = delta->samples;
      status->max_samples = delta->samples;
    }

  status->packets++;
  status->last_packet_us = delta->last_timestamp_us;
  status->total_window_us += window_us;

  if (window_us < status->min_window_us)
    {
      status->min_window_us = (uint32_t)window_us;
    }

  if (window_us > status->max_window_us)
    {
      status->max_window_us = (uint32_t)window_us;
    }

  if (delta->samples < status->min_samples)
    {
      status->min_samples = delta->samples;
    }

  if (delta->samples > status->max_samples)
    {
      status->max_samples = delta->samples;
    }

  if (delta->clipping != 0)
    {
      status->clipped_packets++;
    }

  for (axis = 0; axis < 3; axis++)
    {
      status->total_delta_angle[axis] += delta->delta_angle[axis];
      status->total_delta_velocity[axis] += delta->delta_velocity[axis];
    }
}

static int imu_delta_daemon(int argc, FAR char *argv[])
{
  const int instance = daemon_instance(argc, argv);
  struct accel_queue_s accel_queue;
  struct gyro_queue_s gyro_queue;
  struct imu_integrator_s integrator;
  struct imu_delta_status_s status;
  struct axis_map_s map;
  struct pollfd pollfd[2];
  FAR const struct orb_metadata *accel_meta;
  FAR const struct orb_metadata *gyro_meta;
  float accel_offset[3];
  float accel_scale[3];
  float gyro_offset[3];
  float unused_gyro_scale[3];
  bool accel_calibrated;
  bool gyro_calibrated;
  uint8_t sensor_rot;
  uint8_t board_rot;
  int accel_sub = -1;
  int gyro_sub = -1;
  int publisher = -1;
  int result = EXIT_FAILURE;

  memset(&accel_queue, 0, sizeof(accel_queue));
  memset(&gyro_queue, 0, sizeof(gyro_queue));
  memset(&status, 0, sizeof(status));
  imu_integrator_init(&integrator);

  sensor_rot = (uint8_t)param_i32("SENS_IMU0_ROT");
  board_rot = (uint8_t)param_i32("SENS_BOARD_ROT");

  if (!rotation_supported(sensor_rot) || !rotation_supported(board_rot) ||
      !build_axis_map(sensor_rot, board_rot, &map))
    {
      syslog(LOG_ERR, "[imu-delta] unsupported IMU0/board rotation\n");
      goto out;
    }

  load_cal("CAL_ACC0", true, accel_offset, accel_scale,
           &accel_calibrated);
  load_cal("CAL_GYRO0", false, gyro_offset, unused_gyro_scale,
           &gyro_calibrated);
  status.accel_calibrated = accel_calibrated;
  status.gyro_calibrated = gyro_calibrated;
  status.sensor_rotation = sensor_rot;
  status.board_rotation = board_rot;

  accel_meta = orb_get_meta("sensor_accel");
  /* sensor_gyro and sensor_accel, the RAW driver topics - deliberately not
   * vehicle_gyro/vehicle_accel.
   *
   * Those carry SENS_GYR_LPF and SENS_ACC_LPF, which exist for the rate loop
   * and the companion twist. A low-pass here would add phase lag to the very
   * signal the attitude solution integrates, and the delayed-fusion horizon
   * already handles what such a filter would be there to fix. ArduPilot
   * draws the same line between INS_GYRO_FILTER and the estimator's input.
   *
   * Calibration is applied below, by correct_pair(), so this path is raw
   * CALIBRATED rather than raw uncorrected.
   */

  gyro_meta = orb_get_meta("sensor_gyro");

  if (accel_meta == NULL || gyro_meta == NULL)
    {
      syslog(LOG_ERR, "[imu-delta] sensor metadata unavailable\n");
      goto out;
    }

  /* The driver instance IS the lane: sensor_accel0/sensor_gyro0 are the
   * ICM-42688 and 1 the BMI055.
   */

  accel_sub = orb_subscribe_multi(accel_meta, (unsigned)instance);
  gyro_sub = orb_subscribe_multi(gyro_meta, (unsigned)instance);
  publisher = vehicle_imu_advertise(instance);

  if (accel_sub < 0 || gyro_sub < 0 || publisher < 0)
    {
      syslog(LOG_ERR, "[imu-delta] subscribe/advertise failed (errno %d)\n",
             errno);
      goto out;
    }

  pollfd[0].fd = accel_sub;
  pollfd[0].events = POLLIN;
  pollfd[1].fd = gyro_sub;
  pollfd[1].events = POLLIN;
  g_running[instance] = true;
  status.running = true;
  publish_status(instance, &status, &integrator);

  syslog(LOG_INFO,
         "[imu-delta] ICM42688 2 kHz -> 400 Hz, rotation %s, cal A:%s G:%s\n",
         rotation_name(sensor_rot), accel_calibrated ? "on" : "off",
         gyro_calibrated ? "on" : "off");

  while (!g_should_stop[instance])
    {
      int ready = poll(pollfd, 2, 100);

      if (ready < 0 && errno != EINTR)
        {
          break;
        }

      if ((pollfd[0].revents & POLLIN) != 0)
        {
          struct sensor_accel sample;
          int drained = 0;

          while (drained++ < IMU_DELTA_DRAIN_MAX &&
                 orb_copy(accel_meta, accel_sub, &sample) == 0)
            {
              accel_push(&accel_queue, &sample, &status.queue_overruns);
            }
        }

      if ((pollfd[1].revents & POLLIN) != 0)
        {
          struct sensor_gyro sample;
          int drained = 0;

          while (drained++ < IMU_DELTA_DRAIN_MAX &&
                 orb_copy(gyro_meta, gyro_sub, &sample) == 0)
            {
              gyro_push(&gyro_queue, &sample, &status.queue_overruns);
            }
        }

      while (accel_queue.count > 0 && gyro_queue.count > 0)
        {
          FAR struct sensor_accel *accel = accel_front(&accel_queue);
          FAR struct sensor_gyro *gyro = gyro_front(&gyro_queue);

          if (accel->timestamp < gyro->timestamp)
            {
              accel_pop(&accel_queue);
              status.sync_drops++;
              continue;
            }

          if (gyro->timestamp < accel->timestamp)
            {
              gyro_pop(&gyro_queue);
              status.sync_drops++;
              continue;
            }

          {
            struct imu_delta_output_s delta;
            struct vehicle_imu_s message;
            float corrected_accel[3];
            float corrected_gyro[3];
            uint8_t clipping = clipping_bits(accel, gyro);
            int integrate_result;

            status.paired_samples++;

            correct_pair(accel, gyro, accel_offset, accel_scale,
                         gyro_offset, &map, corrected_accel,
                         corrected_gyro);
            integrate_result = imu_integrator_add(
              &integrator, accel->timestamp, corrected_accel,
              corrected_gyro, clipping, &delta);

            if (integrate_result > 0)
              {
                memset(&message, 0, sizeof(message));
                message.timestamp = now_us();
                message.timestamp_sample = delta.last_timestamp_us;
                message.timestamp_first = delta.first_timestamp_us;
                memcpy(message.delta_angle, delta.delta_angle,
                       sizeof(message.delta_angle));
                memcpy(message.delta_velocity, delta.delta_velocity,
                       sizeof(message.delta_velocity));
                message.delta_angle_dt = delta.delta_angle_dt;
                message.delta_velocity_dt = delta.delta_velocity_dt;
                message.samples = delta.samples;
                message.reset_counter = (uint16_t)integrator.resets;
                message.instance = (uint8_t)instance;
                message.clipping = delta.clipping;
                message.accel_calibrated = accel_calibrated ? 1 : 0;
                message.gyro_calibrated = gyro_calibrated ? 1 : 0;

                if (vehicle_imu_publish(publisher, &message) < 0)
                  {
                    status.publish_errors++;
                  }

                account_packet(&status, &delta);

                if ((status.packets & 15u) == 0)
                  {
                    publish_status(instance, &status, &integrator);
                  }
              }
          }

          accel_pop(&accel_queue);
          gyro_pop(&gyro_queue);
        }
    }

  result = EXIT_SUCCESS;

out:
  status.running = false;
  pthread_mutex_lock(&g_lock);
  g_status[instance] = status;
  copy_integrator_status(&g_status[instance], &integrator);
  g_status[instance].running = false;
  pthread_mutex_unlock(&g_lock);

  if (accel_sub >= 0)
    {
      orb_unsubscribe(accel_sub);
    }

  if (gyro_sub >= 0)
    {
      orb_unsubscribe(gyro_sub);
    }

  if (publisher >= 0)
    {
      close(publisher);
    }

  g_running[instance] = false;
  return result;
}

int imu_delta_start(int instance)
{
  FAR char index[4];
  FAR char *argv[2];
  int task;
  int wait;

  if (instance < 0 || instance >= IMU_DELTA_INSTANCES)
    {
      return -EINVAL;
    }

  if (g_running[instance])
    {
      return -EALREADY;
    }

  snprintf(index, sizeof(index), "%d", instance);
  argv[0] = index;
  argv[1] = NULL;

  g_should_stop[instance] = false;
  task = task_create(instance == 0 ? "imu_delta" : "imu_delta1",
                     IMU_DELTA_PRIORITY, IMU_DELTA_STACK,
                     imu_delta_daemon, argv);

  if (task < 0)
    {
      return -errno;
    }

  for (wait = 0; wait < 100 && !g_running[instance]; wait++)
    {
      usleep(10000);
    }

  return g_running[instance] ? 0 : -EIO;
}

int imu_delta_stop(int instance)
{
  int wait;

  if (instance < 0 || instance >= IMU_DELTA_INSTANCES)
    {
      return -EINVAL;
    }

  if (!g_running[instance])
    {
      return -ENOENT;
    }

  g_should_stop[instance] = true;

  for (wait = 0; wait < 100 && g_running[instance]; wait++)
    {
      usleep(10000);
    }

  return g_running[instance] ? -ETIMEDOUT : 0;
}

void imu_delta_status(int instance, FAR struct imu_delta_status_s *status)
{
  if (status == NULL)
    {
      return;
    }

  if (instance < 0 || instance >= IMU_DELTA_INSTANCES)
    {
      memset(status, 0, sizeof(*status));
      return;
    }

  pthread_mutex_lock(&g_lock);
  *status = g_status[instance];
  pthread_mutex_unlock(&g_lock);
}
