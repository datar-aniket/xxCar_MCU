/****************************************************************************
 * apps/sensors/aux.c
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

#include "aux.h"
#include "mag_frame.h"
#include "../param/param.h"
#include "../uorb_msgs/uorb_msgs.h"

/* Below the IMU path deliberately. These are aiding sensors: a late
 * magnetometer sample costs a little heading accuracy, a late IMU packet
 * costs the strapdown integration itself.
 */

#define AUX_PRIO       (SCHED_PRIORITY_DEFAULT + 8)
#define AUX_STACK      2048
#define AUX_POLL_MS    200

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool g_running;
static volatile bool g_should_stop;
static struct sensors_aux_status_s g_status;

static uint64_t aux_now_us(void)
{
  struct timespec t;

  clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint64_t)t.tv_sec * 1000000ull + (uint64_t)t.tv_nsec / 1000ull;
}

static void status_publish(FAR const struct sensors_aux_status_s *s)
{
  pthread_mutex_lock(&g_lock);
  g_status = *s;
  pthread_mutex_unlock(&g_lock);
}

/* Rate in Hz to the microsecond interval orb_set_interval wants. Clamped so
 * a nonsense parameter cannot divide by zero.
 */

static unsigned rate_to_interval_us(int32_t hz)
{
  if (hz < 1)
    {
      hz = 1;
    }

  return (unsigned)(1000000 / hz);
}

static void handle_mag(FAR const struct orb_metadata *meta, int sub, int pub,
                       FAR const struct mag_frame_s *frame,
                       FAR struct sensors_aux_status_s *s)
{
  struct sensor_mag raw;
  struct vehicle_mag_s out;
  float in[3];
  float body[3];
  bool corrected;

  if (orb_copy(meta, sub, &raw) < 0)
    {
      s->mag_skipped++;
      return;
    }

  in[0] = raw.x;
  in[1] = raw.y;
  in[2] = raw.z;

  corrected = mag_frame_apply(frame, in, body);

  /* mag_frame_apply() returns false both for "not calibrated, passed
   * through" and for "could not be framed at all". The first leaves body
   * usable and worth publishing; the second does not. frame->valid tells
   * them apart.
   */

  if (!corrected && frame->valid)
    {
      s->mag_skipped++;
      return;
    }

  memset(&out, 0, sizeof(out));
  out.timestamp = aux_now_us();
  out.timestamp_sample = raw.timestamp;
  memcpy(out.field, body, sizeof(out.field));
  out.temperature = raw.temperature;
  out.calibrated = corrected ? 1 : 0;
  out.instance = 0;

  if (vehicle_mag_publish(pub, &out) < 0)
    {
      s->mag_skipped++;
      return;
    }

  memcpy(s->mag_field, body, sizeof(s->mag_field));
  s->mag_magnitude = sqrtf(body[0] * body[0] + body[1] * body[1] +
                           body[2] * body[2]);
  s->mag_out++;
}

static void handle_baro(FAR const struct orb_metadata *meta, int sub, int pub,
                        FAR struct sensors_aux_status_s *s)
{
  struct sensor_baro raw;
  struct vehicle_baro_s out;

  if (orb_copy(meta, sub, &raw) < 0)
    {
      s->baro_skipped++;
      return;
    }

  if (!isfinite(raw.pressure) || !isfinite(raw.temperature))
    {
      s->baro_skipped++;
      return;
    }

  memset(&out, 0, sizeof(out));
  out.timestamp = aux_now_us();
  out.timestamp_sample = raw.timestamp;
  out.pressure = raw.pressure;
  out.temperature = raw.temperature;

  if (vehicle_baro_publish(pub, &out) < 0)
    {
      s->baro_skipped++;
      return;
    }

  s->baro_pressure = raw.pressure;
  s->baro_temperature = raw.temperature;
  s->baro_out++;
}

static int aux_daemon(int argc, FAR char *argv[])
{
  struct sensors_aux_status_s status;
  struct mag_frame_s frame;
  struct pollfd fds[2];
  FAR const struct orb_metadata *mag_meta;
  FAR const struct orb_metadata *baro_meta;
  int mag_sub = -1;
  int baro_sub = -1;
  int mag_pub = -1;
  int baro_pub = -1;
  int result = EXIT_FAILURE;

  memset(&status, 0, sizeof(status));

  status.mag_calibrated = mag_frame_load(&frame);
  status.mag_rot = frame.mag_rot;
  status.board_rot = frame.board_rot;
  status.mag_expected = frame.fit.field;
  status.mag_rate_hz = (uint32_t)param_i32("SENS_MAG_RATE");
  status.baro_rate_hz = (uint32_t)param_i32("SENS_BARO_RATE");

  /* The drivers register these through NuttX's sensor framework, so they are
   * not ORB_DECLARE'd here and ORB_ID() cannot name them. Look the metadata
   * up the way cal.c does.
   */

  mag_meta = orb_get_meta("sensor_mag0");
  baro_meta = orb_get_meta("sensor_baro0");

  if (mag_meta == NULL || baro_meta == NULL)
    {
      syslog(LOG_ERR, "[aux] no metadata for %s%s%s; is the driver up?\n",
             mag_meta == NULL ? "sensor_mag0" : "",
             mag_meta == NULL && baro_meta == NULL ? " and " : "",
             baro_meta == NULL ? "sensor_baro0" : "");
      goto out;
    }

  mag_sub = orb_subscribe_multi(mag_meta, 0);
  baro_sub = orb_subscribe_multi(baro_meta, 0);

  if (mag_sub < 0 || baro_sub < 0)
    {
      syslog(LOG_ERR, "[aux] cannot subscribe %s%s%s (errno %d)\n",
             mag_sub < 0 ? "sensor_mag0" : "",
             mag_sub < 0 && baro_sub < 0 ? " and " : "",
             baro_sub < 0 ? "sensor_baro0" : "", errno);
      goto out;
    }

  /* The first code in the tree to honour these two parameters. */

  orb_set_interval(mag_sub,
                   rate_to_interval_us((int32_t)status.mag_rate_hz));
  orb_set_interval(baro_sub,
                   rate_to_interval_us((int32_t)status.baro_rate_hz));

  mag_pub = vehicle_mag_advertise();
  baro_pub = vehicle_baro_advertise();

  /* Name the topic that failed. uorb_msgs.c records that an unnamed "cannot
   * advertise" cost a flash cycle to diagnose.
   */

  if (mag_pub < 0 || baro_pub < 0)
    {
      syslog(LOG_ERR, "[aux] cannot advertise %s%s%s (errno %d)\n",
             mag_pub < 0 ? "vehicle_mag" : "",
             mag_pub < 0 && baro_pub < 0 ? " and " : "",
             baro_pub < 0 ? "vehicle_baro" : "", errno);
      goto out;
    }

  fds[0].fd = mag_sub;
  fds[0].events = POLLIN;
  fds[1].fd = baro_sub;
  fds[1].events = POLLIN;

  g_running = true;
  status.running = true;
  status_publish(&status);

  syslog(LOG_INFO, "[aux] mag %" PRIu32 " Hz (%s), baro %" PRIu32 " Hz\n",
         status.mag_rate_hz,
         status.mag_calibrated ? "calibrated" : "RAW - not calibrated",
         status.baro_rate_hz);

  while (!g_should_stop)
    {
      int ready = poll(fds, 2, AUX_POLL_MS);

      if (ready < 0 && errno != EINTR)
        {
          break;
        }

      if ((fds[0].revents & POLLIN) != 0)
        {
          handle_mag(mag_meta, mag_sub, mag_pub, &frame, &status);
        }

      if ((fds[1].revents & POLLIN) != 0)
        {
          handle_baro(baro_meta, baro_sub, baro_pub, &status);
        }

      status_publish(&status);
    }

  result = EXIT_SUCCESS;

out:
  status.running = false;
  status_publish(&status);

  if (mag_sub >= 0)
    {
      orb_unsubscribe(mag_sub);
    }

  if (baro_sub >= 0)
    {
      orb_unsubscribe(baro_sub);
    }

  if (mag_pub >= 0)
    {
      orb_unadvertise(mag_pub);
    }

  if (baro_pub >= 0)
    {
      orb_unadvertise(baro_pub);
    }

  g_running = false;
  return result;
}

int sensors_aux_start(void)
{
  int pid;
  int spin;

  if (g_running)
    {
      return -EALREADY;
    }

  g_should_stop = false;

  pid = task_create("sensors_aux", AUX_PRIO, AUX_STACK, aux_daemon, NULL);
  if (pid < 0)
    {
      return -errno;
    }

  /* Report what actually happened, for the reason sensors_start() gives: a
   * start that returns OK because a second elapsed, while the task exited on
   * a failed subscription, is a bug worth not repeating.
   */

  for (spin = 0; spin < 100; spin++)
    {
      if (g_running)
        {
          return OK;
        }

      usleep(10000);
    }

  return -EIO;
}

int sensors_aux_stop(void)
{
  int spin;

  if (!g_running)
    {
      return -ESRCH;
    }

  g_should_stop = true;

  for (spin = 0; spin < 100; spin++)
    {
      if (!g_running)
        {
          return OK;
        }

      usleep(10000);
    }

  return -ETIMEDOUT;
}

void sensors_aux_status(FAR struct sensors_aux_status_s *out)
{
  pthread_mutex_lock(&g_lock);
  *out = g_status;
  pthread_mutex_unlock(&g_lock);
}
