/****************************************************************************
 * apps/vesc/vesc.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
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

#include "vesc.h"
#include "../param/param.h"
#include "../uorb_msgs/uorb_msgs.h"

#define VESC_PRIORITY   (SCHED_PRIORITY_DEFAULT + 10)
#define VESC_STACK      2048

/* Poll interval. STATUS_5 arrives at tens of hertz, so 2 ms drains the FIFO
 * far faster than it fills while costing almost nothing.
 *
 * This interval IS the jitter on the telemetry - see the note in fdcan.h
 * about polling being a deliberate trade.
 */

#define VESC_POLL_US    2000

/* Bound on one drain pass. Without it a bus stuck jabbering would keep this
 * loop from ever reaching g_should_stop, and `vesc stop` would hang.
 */

#define VESC_DRAIN_MAX  32

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool g_running;
static volatile bool g_should_stop;
static struct vesc_daemon_status_s g_status;

static uint64_t vesc_now_us(void)
{
  struct timespec t;

  clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint64_t)t.tv_sec * 1000000ull + (uint64_t)t.tv_nsec / 1000ull;
}

static void status_publish(FAR const struct vesc_daemon_status_s *s)
{
  pthread_mutex_lock(&g_lock);
  g_status = *s;
  pthread_mutex_unlock(&g_lock);
}

/* Record that this (packet, controller) pair exists. Linear scan over at
 * most twelve entries, called at frame rate - a hash would be more code for
 * a table that never gets big.
 */

static void vesc_note_seen(FAR struct vesc_daemon_status_s *s,
                           uint8_t packet_id, uint8_t controller_id,
                           uint64_t now)
{
  int i;

  for (i = 0; i < s->nseen; i++)
    {
      if (s->seen[i].packet_id == packet_id &&
          s->seen[i].controller_id == controller_id)
        {
          s->seen[i].count++;
          s->seen[i].last_us = now;
          return;
        }
    }

  if (s->nseen >= VESC_SEEN_MAX)
    {
      return;
    }

  s->seen[s->nseen].packet_id = packet_id;
  s->seen[s->nseen].controller_id = controller_id;
  s->seen[s->nseen].count = 1;
  s->seen[s->nseen].first_us = now;
  s->seen[s->nseen].last_us = now;
  s->nseen++;
}

static void vesc_handle(FAR const struct fdcan_frame_s *frame, int pub,
                        FAR struct vesc_daemon_status_s *s)
{
  uint8_t packet_id = vesc_packet_id(frame->id);
  uint8_t controller_id = vesc_controller_id(frame->id);
  uint64_t now = vesc_now_us();
  struct vesc_status5_s decoded;
  struct vesc_status_s out;

  vesc_note_seen(s, packet_id, controller_id, now);

  if (packet_id != VESC_PACKET_STATUS_5)
    {
      /* Not an error. Counted above so it shows up in discovery, and
       * ignored here because nothing else is decoded yet.
       */

      return;
    }

  if (!vesc_decode_status5(frame->data, frame->dlc, &decoded))
    {
      /* A known packet id with the wrong length is the two ends disagreeing
       * about a format, which is a different thing from a packet id we do
       * not know - and is worth its own counter.
       */

      s->bad_dlc++;
      return;
    }

  memset(&out, 0, sizeof(out));
  out.timestamp = now;
  out.timestamp_sample = now;
  out.tachometer = decoded.tachometer;
  out.current_a = decoded.current_a;
  out.adc_volts = decoded.adc_volts;
  out.controller_id = controller_id;

  if (vesc_status_publish(pub, &out) < 0)
    {
      s->publish_errors++;
      return;
    }

  s->last = decoded;
  s->last_us = now;
  s->decoded++;
}

static int vesc_daemon(int argc, FAR char *argv[])
{
  struct vesc_daemon_status_s status;
  int pub = -1;
  int result = EXIT_FAILURE;
  int ret;

  memset(&status, 0, sizeof(status));
  status.bitrate = (uint32_t)param_i32("VESC_BITRATE");
  status.filter_id = (uint8_t)param_i32("VESC_CAN_ID");

  ret = fdcan_init(status.bitrate);

  if (ret < 0)
    {
      syslog(LOG_ERR, "[vesc] FDCAN1 init failed: %d%s\n", -ret,
             ret == -ENOTSUP ? " (only 1000000 bit/s is implemented)" : "");
      goto out;
    }

  /* fdcan_init leaves the filter accept-any. Narrowing it is a separate
   * call so discovery and normal operation take the same path.
   */

  fdcan_set_filter(status.filter_id);

  pub = vesc_status_advertise();

  if (pub < 0)
    {
      syslog(LOG_ERR, "[vesc] cannot advertise vesc_status (%d)\n", errno);
      goto out;
    }

  g_running = true;
  status.running = true;
  status_publish(&status);

  syslog(LOG_INFO, "[vesc] FDCAN1 at %" PRIu32 " bit/s, filter %s\n",
         status.bitrate,
         status.filter_id == 0 ? "accept-any" : "one id");

  while (!g_should_stop)
    {
      struct fdcan_frame_s frame;
      int drained = 0;

      /* Drain everything pending before sleeping. A burst arrives faster
       * than the poll interval, and leaving frames in the FIFO to collect
       * one per wake-up is how an overrun happens on a bus that is not even
       * busy.
       */

      while (drained++ < VESC_DRAIN_MAX && fdcan_receive(&frame) == OK)
        {
          vesc_handle(&frame, pub, &status);
        }

      fdcan_stats(&status.bus);
      status_publish(&status);
      usleep(VESC_POLL_US);
    }

  result = EXIT_SUCCESS;

out:
  status.running = false;
  status_publish(&status);

  if (pub >= 0)
    {
      orb_unadvertise(pub);
    }

  g_running = false;
  return result;
}

int vesc_start(void)
{
  int task;
  int wait;

  if (g_running)
    {
      return -EALREADY;
    }

  g_should_stop = false;
  task = task_create("vesc", VESC_PRIORITY, VESC_STACK, vesc_daemon, NULL);

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

int vesc_stop(void)
{
  int wait;

  if (!g_running)
    {
      return -ESRCH;
    }

  g_should_stop = true;

  for (wait = 0; wait < 100 && g_running; wait++)
    {
      usleep(10000);
    }

  return g_running ? -ETIMEDOUT : 0;
}

void vesc_status(FAR struct vesc_daemon_status_s *out)
{
  pthread_mutex_lock(&g_lock);
  *out = g_status;
  pthread_mutex_unlock(&g_lock);
}
