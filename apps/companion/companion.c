/****************************************************************************
 * apps/companion/companion.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/uorb.h>
#include <uORB/uORB.h>

#include "companion.h"
#include "../param/param.h"
#include "../serial/serial.h"
#include "../uorb_msgs/uorb_msgs.h"

#define COMP_PRIORITY  (SCHED_PRIORITY_DEFAULT + 12)
#define COMP_STACK     2560
#define COMP_READ_MAX  256

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool g_running;
static volatile bool g_should_stop;
static struct companion_status_s g_status;

/* Board monotonic + this = UTC. Zero until a sync has happened, which is
 * also what "not synced" means at the conversion sites below.
 */

static int64_t g_utc_offset_us;

static uint64_t comp_now_us(void)
{
  struct timespec t;

  clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint64_t)t.tv_sec * 1000000ull + (uint64_t)t.tv_nsec / 1000ull;
}

static void status_publish(FAR const struct companion_status_s *s)
{
  pthread_mutex_lock(&g_lock);
  g_status = *s;
  pthread_mutex_unlock(&g_lock);
}

/* Find the port whose SER_*_FUNC names the companion link.
 *
 * Scanned rather than hardcoded the way cal does it: cal's fixed devpath is
 * a consequence of the GUI always being on USB, and the companion is a real
 * peripheral that could be on any connector.
 */

static int comp_find_port(FAR const struct serial_port_s **out)
{
  FAR const struct serial_port_s *ports = serial_ports();
  int n = serial_port_count();
  int i;

  for (i = 0; i < n; i++)
    {
      if (param_i32(ports[i].func_param) == SER_FUNC_COMPANION)
        {
          *out = &ports[i];
          return i;
        }
    }

  return -ENOENT;
}

static int comp_open(FAR const struct serial_port_s *p, uint32_t baud)
{
  struct termios tio;
  int fd = open(p->devpath, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd < 0)
    {
      return -errno;
    }

  /* Raw mode is REQUIRED, not best-effort.
   *
   * Canonical mode would buffer by line, echo would feed our own frames back
   * to us, and CR/LF translation would corrupt any frame containing 0x0d -
   * which a float payload does routinely. cal_raw_mode() fails the session
   * rather than continuing without it, and so does this: a link that looks
   * open while mangling every frame is worse than one that refused.
   */

  if (tcgetattr(fd, &tio) < 0)
    {
      int err = -errno;

      close(fd);
      return err;
    }

  cfmakeraw(&tio);

  /* Baud only where it means something. The USB CDC port has no baud
   * parameter at all - the host owns the line coding and the device ignores
   * it - and serial_port_s carries NULL there. Reading that NULL as a
   * parameter name is what crashed this daemon on USB.
   */

  if (baud > 0)
    {
      cfsetispeed(&tio, baud);
      cfsetospeed(&tio, baud);
    }

  if (tcsetattr(fd, TCSANOW, &tio) < 0)
    {
      int err = -errno;

      close(fd);
      return err;
    }

  return fd;
}

/* Write a whole frame, or report that it could not be.
 *
 * write() on a non-blocking port may take only part of a frame. Abandoning
 * the rest puts half a frame on the wire: the far end resynchronises on the
 * next sync byte and drops it on CRC, so it is not fatal, but at rate it is
 * constant corruption. Push the remainder instead, and give up only when the
 * far end is genuinely not draining.
 */

static int comp_write_all(int fd, FAR const uint8_t *data, size_t len)
{
  size_t sent = 0;
  int attempts = 0;

  while (sent < len && attempts++ < 8)
    {
      ssize_t n = write(fd, data + sent, len - sent);

      if (n > 0)
        {
          sent += (size_t)n;
          continue;
        }

      if (n < 0 && errno != EAGAIN && errno != EINTR)
        {
          return -errno;
        }

      usleep(1000);
    }

  return sent == len ? OK : -EAGAIN;
}

/* Route one decoded frame. Adding a message means adding a case and a topic;
 * nothing else in this file changes.
 */

static void comp_route(int id, FAR const struct comp_parser_s *parser,
                       int fd, uint64_t rx_us, int pose_pub,
                       FAR struct companion_status_s *s)
{
  if (id == COMP_MSG_EXTERNAL_POSE)
    {
      struct comp_external_pose_s wire;
      struct external_pose_s out;

      memcpy(&wire, parser->payload, sizeof(wire));
      memset(&out, 0, sizeof(out));

      out.timestamp = comp_now_us();

      /* Back to the board's monotonic clock, which is the only timebase the
       * estimator understands. Zero stays zero - it means "not timestamped"
       * and the estimator stamps it on arrival.
       *
       * Unsynced, a supplied UTC cannot be converted at all, so it is turned
       * into that same zero rather than fused as a number a thousand times
       * larger than anything the horizon expects.
       */

      if (wire.timestamp_us == 0 || g_utc_offset_us == 0)
        {
          out.timestamp_sample = 0;

          if (wire.timestamp_us != 0)
            {
              s->rx_unsynced_stamp++;
            }
        }
      else
        {
          out.timestamp_sample =
            (uint64_t)((int64_t)wire.timestamp_us - g_utc_offset_us);
        }
      out.x = wire.x;
      out.y = wire.y;
      out.yaw = wire.yaw;
      memcpy(out.cov, wire.cov, sizeof(out.cov));
      out.flags = wire.flags;
      out.reset_counter = wire.reset_counter;

      if (external_pose_publish(pose_pub, &out) < 0)
        {
          s->rx_publish_errors++;
          return;
        }

      s->rx_pose++;
      s->last_rx_us = out.timestamp;
    }

  else if (id == COMP_MSG_TIMESYNC_START)
    {
      struct comp_timesync_start_s start;

      memcpy(&start, parser->payload, sizeof(start));
      s->timesync_expected = start.count;
      s->timesync_replies = 0;
      s->timesync_synced = false;
    }
  else if (id == COMP_MSG_TIMESYNC_END)
    {
      struct comp_timesync_end_s end;

      memcpy(&end, parser->payload, sizeof(end));
      s->timesync_offset_us = end.utc_offset_us;
      s->timesync_trip_us = end.trip_us;
      s->timesync_samples = end.samples;
      s->timesync_synced = end.samples > 0;

      if (s->timesync_synced)
        {
          struct timespec utc;
          int64_t now_utc = (int64_t)comp_now_us() + end.utc_offset_us;

          g_utc_offset_us = end.utc_offset_us;

          /* Set the wall clock too, so `date`, log filenames and anything
           * else reading CLOCK_REALTIME agree with the companion. This is
           * the only clock that is SET; the monotonic one everything else
           * is timed against is left exactly where it was.
           */

          utc.tv_sec = (time_t)(now_utc / 1000000);
          utc.tv_nsec = (long)((now_utc % 1000000) * 1000);

          if (clock_settime(CLOCK_REALTIME, &utc) == 0)
            {
              s->wall_clock_set = true;
            }
        }
    }
  else if (id == COMP_MSG_TIMESYNC_REQ)
    {
      struct comp_timesync_req_s req;
      struct comp_timesync_rep_s rep;
      uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
      int n;

      memcpy(&req, parser->payload, sizeof(req));

      /* rx_us was taken when the frame COMPLETED, not now. Everything
       * between - routing, packing, the write - is board processing that
       * board_tx_us accounts for separately, so it cannot leak into the
       * offset the companion computes.
       */

      rep.host_tx_us = req.host_tx_us;
      rep.board_rx_us = rx_us;
      rep.board_tx_us = comp_now_us();

      n = comp_encode(COMP_MSG_TIMESYNC_REP, &rep, sizeof(rep), frame,
                      sizeof(frame));

      if (n > 0 && comp_write_all(fd, frame, (size_t)n) == OK)
        {
          s->bytes_out += (uint64_t)n;
          s->tx_frames++;
          s->timesync_replies++;
        }
      else
        {
          s->tx_errors++;
        }
    }

  /* COMP_MSG_CONTROL_TRAJ is reserved. The parser has already counted it as
   * unknown; there is nothing to do here until it is defined.
   */
}

static void comp_transmit(int fd, int est_sub,
                          FAR struct companion_status_s *s)
{
  struct estimator_state_s est;
  struct comp_estimator_pose_s wire;
  uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  int n;

  if (orb_copy(ORB_ID(estimator_state), est_sub, &est) < 0)
    {
      /* No new estimator state since the last read. Counted separately from
       * a write failure: "the companion sees nothing" has two very different
       * causes and the status line has to tell them apart.
       */

      s->tx_no_state++;
      return;
    }

  s->est_seen++;

  memset(&wire, 0, sizeof(wire));
  /* UTC on the wire once synced. Before that the companion gets the board's
   * raw monotonic time, which is all there is to give.
   */

  wire.timestamp_us = g_utc_offset_us != 0 ?
                      (uint64_t)((int64_t)est.timestamp_sample +
                                 g_utc_offset_us) :
                      est.timestamp_sample;
  memcpy(wire.position, est.position, sizeof(wire.position));
  memcpy(wire.quaternion, est.quaternion, sizeof(wire.quaternion));
  memcpy(wire.velocity, est.velocity, sizeof(wire.velocity));
  wire.solution_status = est.solution_status;
  wire.reset_counter = (uint8_t)est.reset_counter;

  n = comp_encode(COMP_MSG_ESTIMATOR_POSE, &wire, sizeof(wire), frame,
                  sizeof(frame));

  if (n < 0)
    {
      s->tx_errors++;
      return;
    }

  if (comp_write_all(fd, frame, (size_t)n) < 0)
    {
      /* A companion that stopped reading backs the port up. Count it and
       * carry on: dropping a pose is correct, blocking the daemon is not.
       */

      s->tx_errors++;
      return;
    }

  s->bytes_out += (uint64_t)n;
  s->tx_frames++;
}

/* Service one connected port until it goes away or we are asked to stop.
 *
 * Returns false when g_should_stop was set, true when the port failed and
 * the caller should go back for another host. Separating the two is the
 * whole point: on a removable port a read error is a cable, not a fault.
 */

static bool comp_service(int fd, FAR struct pollfd *pfd, int est_sub,
                         int pose_pub, FAR uint64_t *next_tx,
                         uint64_t tx_interval,
                         FAR struct companion_status_s *status)
{
  while (!g_should_stop)
    {
      uint8_t buf[COMP_READ_MAX];
      uint64_t now;
      ssize_t got;
      int ready = poll(pfd, 1, 20);

      if (ready < 0 && errno != EINTR)
        {
          return true;
        }

      /* On a removable port these mean the host went away, not a fault. */

      if ((pfd->revents & (POLLHUP | POLLERR)) != 0)
        {
          return true;
        }

      if ((pfd->revents & POLLIN) != 0)
        {
          got = read(fd, buf, sizeof(buf));

          if (got > 0)
            {
              ssize_t i;

              status->bytes_in += (uint64_t)got;

              for (i = 0; i < got; i++)
                {
                  int id = comp_parser_byte(&status->parser, buf[i]);

                  if (id != 0)
                    {
                      /* Stamp the moment the frame COMPLETED. Taking it
                       * later would fold this loop's own work into the
                       * companion's clock offset.
                       */

                      comp_route(id, &status->parser, fd, comp_now_us(),
                                 pose_pub, status);
                    }
                }
            }
          else if (got == 0 || (errno != EAGAIN && errno != EINTR))
            {
              return true;        /* cable pulled */
            }
        }

      now = comp_now_us();

      if (est_sub >= 0 && now >= *next_tx)
        {
          comp_transmit(fd, est_sub, status);

          /* Advance from the deadline, not from now, so the rate does not
           * drift with scheduling. Resynchronise after a long stall rather
           * than bursting to catch up.
           */

          *next_tx += tx_interval;

          if (*next_tx < now)
            {
              *next_tx = now + tx_interval;
            }
        }

      status_publish(status);
    }

  return false;
}

static int companion_daemon(int argc, FAR char *argv[])
{
  struct companion_status_s status;
  FAR const struct serial_port_s *port = NULL;
  struct pollfd pfd;
  uint64_t next_tx;
  uint64_t tx_interval;
  int fd = -1;
  int pose_pub = -1;
  int est_sub = -1;
  int result = EXIT_FAILURE;

  memset(&status, 0, sizeof(status));
  comp_parser_init(&status.parser);

  if (comp_find_port(&port) < 0)
    {
      syslog(LOG_ERR,
             "[companion] no port reserved; set a SER_*_FUNC to %d\n",
             SER_FUNC_COMPANION);
      goto out;
    }

  strncpy(status.port, port->name, sizeof(status.port) - 1);
  /* NULL for the USB CDC port, which has no baud parameter at all. Passing
   * that to param_i32() reaches strcmp(name, NULL) and dereferences it -
   * which is exactly how this daemon crashed on USB.
   */

  status.baud = port->baud_param != NULL ?
                (uint32_t)param_i32(port->baud_param) : 0;
  status.tx_rate_hz = (uint32_t)param_i32("EXT_TX_RATE");
  tx_interval = 1000000ull / (status.tx_rate_hz > 0 ?
                              status.tx_rate_hz : 1);

  pose_pub = external_pose_advertise();

  if (pose_pub < 0)
    {
      syslog(LOG_ERR, "[companion] cannot advertise external_pose (%d)\n",
             errno);
      goto out;
    }

  /* The estimator may not be running. That is not a failure: the link still
   * receives, and starts transmitting when ekf3 comes up.
   */

  est_sub = orb_subscribe(ORB_ID(estimator_state));

  g_running = true;
  status.running = true;
  status_publish(&status);

  if (status.baud > 0)
    {
      syslog(LOG_INFO, "[companion] %s at %" PRIu32 " baud, pose out at "
                       "%" PRIu32 " Hz\n",
             port->name, status.baud, status.tx_rate_hz);
    }
  else
    {
      syslog(LOG_INFO, "[companion] %s (host sets the line coding), pose "
                       "out at %" PRIu32 " Hz\n",
             port->name, status.tx_rate_hz);
    }

  /* Outer loop: acquire the port, service it, and go back for it if it goes
   * away.
   *
   * A removable port - the USB CDC one, and only that one - does not exist
   * until a host attaches, and dies when the cable is pulled. Opening once at
   * boot would fail with ENOTCONN and give up before anyone plugged in, and
   * an unplug would end the daemon for good. serial.c's NSH path makes the
   * same argument and waits the same way.
   */

  while (!g_should_stop)
    {
      bool waiting = false;

      while (!g_should_stop)
        {
          fd = comp_open(port, status.baud);

          if (fd >= 0)
            {
              break;
            }

          /* ONLY "no host attached" is worth waiting on. A missing device or
           * a port that will not go raw is a real fault, and retrying it
           * forever would leave a task that looks alive and does nothing.
           */

          if (!port->removable || (fd != -ENOTCONN && fd != -ENODEV))
            {
              syslog(LOG_ERR, "[companion] cannot open %s: %d\n",
                     port->devpath, -fd);
              goto out;
            }

          if (!waiting)
            {
              syslog(LOG_INFO,
                     "[companion] waiting for a host on %s\n",
                     port->devpath);
              waiting = true;
              status.waiting_for_host = true;
              status_publish(&status);
            }

          usleep(250000);
        }

      if (g_should_stop)
        {
          break;
        }

      status.waiting_for_host = false;
      status.connects++;

      /* A fresh host has seen none of the previous stream, so a half-frame
       * left over from the last connection would be its first bytes.
       */

      comp_parser_init(&status.parser);
      status_publish(&status);

      pfd.fd = fd;
      pfd.events = POLLIN;
      next_tx = comp_now_us();

      if (!comp_service(fd, &pfd, est_sub, pose_pub, &next_tx, tx_interval,
                        &status))
        {
          break;                    /* asked to stop */
        }

      /* The port went away. Close it and go round for the next host. */

      close(fd);
      fd = -1;
      status.disconnects++;
      status_publish(&status);

      if (!port->removable)
        {
          syslog(LOG_ERR, "[companion] %s failed and is not removable\n",
                 port->devpath);
          goto out;
        }

      syslog(LOG_INFO, "[companion] host detached from %s\n",
             port->devpath);
    }

  result = EXIT_SUCCESS;

out:
  status.running = false;
  status_publish(&status);

  if (est_sub >= 0)
    {
      orb_unsubscribe(est_sub);
    }

  if (pose_pub >= 0)
    {
      orb_unadvertise(pose_pub);
    }

  if (fd >= 0)
    {
      close(fd);
    }

  g_running = false;
  return result;
}

int companion_start(void)
{
  int task;
  int wait;

  if (g_running)
    {
      return -EALREADY;
    }

  g_should_stop = false;
  task = task_create("companion", COMP_PRIORITY, COMP_STACK,
                     companion_daemon, NULL);

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

int companion_stop(void)
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

void companion_status(FAR struct companion_status_s *out)
{
  pthread_mutex_lock(&g_lock);
  *out = g_status;
  pthread_mutex_unlock(&g_lock);
}
