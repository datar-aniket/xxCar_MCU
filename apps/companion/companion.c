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

  if (tcgetattr(fd, &tio) == 0)
    {
      /* Raw, both directions. Canonical mode would buffer by line, echo
       * would feed our own frames back to us, and CR/LF translation would
       * corrupt any frame containing 0x0d - which a float payload does
       * routinely. The cal protocol makes the same argument.
       */

      cfmakeraw(&tio);
      cfsetispeed(&tio, baud);
      cfsetospeed(&tio, baud);
      tcsetattr(fd, TCSANOW, &tio);
    }

  return fd;
}

/* Route one decoded frame. Adding a message means adding a case and a topic;
 * nothing else in this file changes.
 */

static void comp_route(int id, FAR const struct comp_parser_s *parser,
                       int pose_pub, FAR struct companion_status_s *s)
{
  if (id == COMP_MSG_EXTERNAL_POSE)
    {
      struct comp_external_pose_s wire;
      struct external_pose_s out;

      memcpy(&wire, parser->payload, sizeof(wire));
      memset(&out, 0, sizeof(out));

      out.timestamp = comp_now_us();
      out.timestamp_sample = wire.timestamp_us;
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

  /* COMP_MSG_CONTROL_TRAJ and the timesync ids are reserved. The parser has
   * already counted them as unknown; there is nothing to do here until they
   * are defined.
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
      return;
    }

  memset(&wire, 0, sizeof(wire));
  wire.timestamp_us = est.timestamp_sample;
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

  if (write(fd, frame, (size_t)n) != n)
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
  status.baud = (uint32_t)param_i32(port->baud_param);
  status.tx_rate_hz = (uint32_t)param_i32("EXT_TX_RATE");
  tx_interval = 1000000ull / (status.tx_rate_hz > 0 ?
                              status.tx_rate_hz : 1);

  fd = comp_open(port, status.baud);

  if (fd < 0)
    {
      syslog(LOG_ERR, "[companion] cannot open %s: %d\n",
             port->devpath, -fd);
      goto out;
    }

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

  pfd.fd = fd;
  pfd.events = POLLIN;
  next_tx = comp_now_us();

  g_running = true;
  status.running = true;
  status_publish(&status);

  syslog(LOG_INFO, "[companion] %s at %" PRIu32 " baud, pose out at "
                   "%" PRIu32 " Hz\n",
         port->name, status.baud, status.tx_rate_hz);

  while (!g_should_stop)
    {
      uint8_t buf[COMP_READ_MAX];
      uint64_t now;
      ssize_t got;
      int ready = poll(&pfd, 1, 20);

      if (ready < 0 && errno != EINTR)
        {
          break;
        }

      if ((pfd.revents & POLLIN) != 0)
        {
          got = read(fd, buf, sizeof(buf));

          if (got > 0)
            {
              ssize_t i;

              status.bytes_in += (uint64_t)got;

              for (i = 0; i < got; i++)
                {
                  int id = comp_parser_byte(&status.parser, buf[i]);

                  if (id != 0)
                    {
                      comp_route(id, &status.parser, pose_pub, &status);
                    }
                }
            }
        }

      now = comp_now_us();

      if (est_sub >= 0 && now >= next_tx)
        {
          comp_transmit(fd, est_sub, &status);

          /* Advance from the deadline, not from now, so the rate does not
           * drift with scheduling. Resynchronise after a long stall rather
           * than bursting to catch up.
           */

          next_tx += tx_interval;

          if (next_tx < now)
            {
              next_tx = now + tx_interval;
            }
        }

      status_publish(&status);
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
