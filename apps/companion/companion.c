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
#include "../../boards/fmuv6c/src/fmuv6c.h"

#define COMP_PRIORITY  (SCHED_PRIORITY_DEFAULT + 12)
#define COMP_STACK     2560
#define COMP_READ_MAX  256

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool g_running;
static volatile bool g_should_stop;
static struct companion_status_s g_status;

/* Board monotonic + this = UTC.
 *
 * g_utc_valid is a separate flag rather than testing the offset against
 * zero: an offset that genuinely IS zero is possible, and would then read as
 * "no reference" for ever.
 */

/* The downlink runs on its own thread so the TIM6 tick reaches the wire
 * without waiting for whatever the receive path happens to be doing. It
 * lives exactly as long as one connection, so the descriptor it writes to
 * cannot be closed underneath it.
 */

static volatile bool g_tx_stop;
static uint64_t g_tx_last_sample;

static int64_t g_utc_offset_us;
static bool    g_utc_valid;

/* Anything before this cannot be a real UTC time on this vehicle, so it is a
 * clock that was never set rather than one that is merely wrong. NuttX reads
 * an unset RTC as its build epoch, which looks like a perfectly plausible
 * date and would make every wire timestamp confidently wrong.
 */

#define COMP_UTC_PLAUSIBLE_S  1700000000ll   /* 2023-11-14 */

static uint64_t comp_now_us(void)
{
  struct timespec t;

  clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint64_t)t.tv_sec * 1000000ull + (uint64_t)t.tv_nsec / 1000ull;
}

/* Take UTC from the battery-backed RTC, if it is holding a time somebody set.
 *
 * This is what makes the sync a ONE-TIME act rather than a per-boot ritual:
 * the RTC keeps running on VBAT, so after a power cycle the board already
 * knows what time it is and the wire conversion works before the companion
 * has said anything.
 *
 * Two limits are worth knowing rather than discovering.
 *
 * It is a SECOND-resolution recovery - the H7's RTC is a calendar, not a
 * tick counter - so the phase within that second is lost. A fresh sync
 * recovers it, and a PPS input would hold it.
 *
 * And this board clocks the RTC from HSE, because it has no 32.768 kHz
 * crystal (PX4's own fmu-v6c config does the same). HSE stops when main
 * power does, so the time survives a warm reboot and NOT a power cycle.
 */

static void comp_seed_utc_from_rtc(FAR struct companion_status_s *s)
{
  struct timespec rt;

  if (clock_gettime(CLOCK_REALTIME, &rt) != 0 ||
      (int64_t)rt.tv_sec < COMP_UTC_PLAUSIBLE_S)
    {
      return;
    }

  g_utc_offset_us = ((int64_t)rt.tv_sec * 1000000ll +
                     (int64_t)rt.tv_nsec / 1000ll) -
                    (int64_t)comp_now_us();
  g_utc_valid = true;
  s->utc_from_rtc = true;

  syslog(LOG_INFO, "[companion] UTC from the RTC; sync to recover the "
                   "sub-second phase\n");
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

      if (wire.timestamp_us == 0 || !g_utc_valid)
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
          g_utc_valid = true;
          s->utc_from_rtc = false;

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

/* Steer the UTC offset so the companion's PPS edge lands on a whole second.
 *
 * The pulse is generated BY the companion, so its rising edge is that
 * machine's own second boundary. Whatever (edge + offset) is short of, or
 * past, an exact second is therefore the error between our idea of UTC and
 * the clock the companion will judge our timestamps against - measured to
 * the microsecond, where a timesync round trip can only manage a millisecond.
 *
 * A positive residual means our UTC runs AHEAD of the companion's, which is
 * the direction that puts a solution time in its future. Nulling it is what
 * makes "never in the future" hold rather than merely being likely.
 */

static void comp_pps_discipline(FAR struct companion_status_s *s)
{
  struct fmuv6c_pps_status_s pps;
  int64_t utc_at_edge;
  int64_t residual;
  uint64_t now;

  fmuv6c_pps_status(&pps);
  s->pps_state = (uint8_t)pps.state;

  /* PPS fixes the PHASE inside a second. It cannot say WHICH second, so
   * without an absolute time from the RTC or a completed sync there is
   * nothing here to correct.
   */

  if (!g_utc_valid || !pps.running ||
      pps.state != FMUV6C_PPS_LOCKED || pps.last_edge_us == 0)
    {
      return;
    }

  if (pps.last_edge_us == s->pps_edge_used)
    {
      return;                     /* already applied to this edge */
    }

  now = fmuv6c_imu_time_now();

  if (now < pps.last_edge_us || now - pps.last_edge_us > 2000000ull)
    {
      return;                     /* stale, or the clock moved under us */
    }

  utc_at_edge = (int64_t)pps.last_edge_us + g_utc_offset_us;
  residual = utc_at_edge % 1000000ll;

  if (residual < 0)
    {
      residual += 1000000ll;
    }

  /* Fold to the nearer boundary: 999.9 ms past a second is 0.1 ms short of
   * the next one, not most of a second late.
   */

  if (residual > 500000ll)
    {
      residual -= 1000000ll;
    }

  g_utc_offset_us -= residual;
  s->pps_edge_used = pps.last_edge_us;
  s->pps_residual_us = (int32_t)residual;
  s->pps_corrections++;
}

static void comp_transmit(int fd, int est_sub,
                          FAR struct companion_status_s *s)
{
  struct estimator_state_s est;
  struct comp_estimator_pose_s wire;
  uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  int64_t now_utc;
  uint32_t gap = 0;
  bool repeat = false;
  bool clamped = false;
  int n;

  if (orb_copy(ORB_ID(estimator_state), est_sub, &est) < 0)
    {
      /* No new estimator state since the last read. Counted separately from
       * a write failure: "the companion sees nothing" has two very different
       * causes and the status line has to tell them apart.
       */

      pthread_mutex_lock(&g_lock);
      s->tx_no_state++;
      pthread_mutex_unlock(&g_lock);
      return;
    }

  /* The tick free-runs against the estimator, so measure the slip rather
   * than assume it away: an unchanged sample time means this tick caught the
   * same solution as the last, and the gap says how many solutions went by
   * between the ones actually sent.
   */

  if (est.timestamp_sample == g_tx_last_sample)
    {
      repeat = true;
    }
  else if (g_tx_last_sample != 0 &&
           est.timestamp_sample > g_tx_last_sample)
    {
      gap = (uint32_t)(est.timestamp_sample - g_tx_last_sample);
    }

  g_tx_last_sample = est.timestamp_sample;

  memset(&wire, 0, sizeof(wire));

  /* UTC on the wire once synced. Before that the companion gets the board's
   * raw monotonic time, which is all there is to give.
   */

  wire.timestamp_us = g_utc_valid ?
                      (uint64_t)((int64_t)est.timestamp_sample +
                                 g_utc_offset_us) :
                      est.timestamp_sample;

  /* A solution cannot be newer than now, so refuse to say that it is.
   *
   * The reference is fmuv6c_imu_time_now() and NOT comp_now_us(). Both are
   * the same epoch, but comp_now_us reads CLOCK_MONOTONIC, which without
   * tickless mode advances in 1 ms steps, while timestamp_sample carries
   * microseconds from TIM5. Comparing against the coarse clock would clamp
   * perfectly good stamps for up to a millisecond after every tick and count
   * a fault that had not happened.
   */

  now_utc = (int64_t)fmuv6c_imu_time_now() +
            (g_utc_valid ? g_utc_offset_us : 0);

  if ((int64_t)wire.timestamp_us > now_utc)
    {
      wire.timestamp_us = (uint64_t)now_utc;
      clamped = true;
    }

  memcpy(wire.position, est.position, sizeof(wire.position));
  memcpy(wire.quaternion, est.quaternion, sizeof(wire.quaternion));
  memcpy(wire.velocity, est.velocity, sizeof(wire.velocity));
  wire.solution_status = est.solution_status;
  wire.reset_counter = (uint8_t)est.reset_counter;

  n = comp_encode(COMP_MSG_ESTIMATOR_POSE, &wire, sizeof(wire), frame,
                  sizeof(frame));

  if (n < 0)
    {
      pthread_mutex_lock(&g_lock);
      s->tx_errors++;
      pthread_mutex_unlock(&g_lock);
      return;
    }

  if (comp_write_all(fd, frame, (size_t)n) < 0)
    {
      /* A companion that stopped reading backs the port up. Count it and
       * carry on: dropping a pose is correct, blocking the downlink is not.
       */

      pthread_mutex_lock(&g_lock);
      s->tx_errors++;
      pthread_mutex_unlock(&g_lock);
      return;
    }

  pthread_mutex_lock(&g_lock);
  s->est_seen++;
  s->bytes_out += (uint64_t)n;
  s->tx_frames++;

  if (repeat)
    {
      s->tx_repeat++;
    }

  if (clamped)
    {
      s->tx_future_clamped++;
    }

  if (gap != 0)
    {
      if (s->tx_gap_min_us == 0 || gap < s->tx_gap_min_us)
        {
          s->tx_gap_min_us = gap;
        }

      if (gap > s->tx_gap_max_us)
        {
          s->tx_gap_max_us = gap;
        }
    }

  pthread_mutex_unlock(&g_lock);
}

/* One connection's downlink.
 *
 * Its own thread so that a tick reaches the wire on the tick, rather than
 * whenever the receive path next comes up for air. It is created after the
 * port opens and joined before it closes, so `fd` cannot go stale under it.
 */

struct comp_tx_args_s
{
  int fd;
  int est_sub;
  FAR struct companion_status_s *status;
};

static FAR void *comp_tx_thread(FAR void *arg)
{
  FAR struct comp_tx_args_s *a = (FAR struct comp_tx_args_s *)arg;

  while (!g_tx_stop && !g_should_stop)
    {
      struct fmuv6c_txtick_status_s tick;

      /* Returns on the tick, or on its timeout if the timer stopped. Either
       * way the loop re-checks the stop flags, which is what lets a
       * disconnect be noticed promptly.
       */

      if (fmuv6c_txtick_wait() < 0)
        {
          continue;
        }

      if (g_tx_stop || g_should_stop)
        {
          break;
        }

      comp_pps_discipline(a->status);

      if (a->est_sub >= 0)
        {
          comp_transmit(a->fd, a->est_sub, a->status);
        }

      fmuv6c_txtick_status(&tick);

      pthread_mutex_lock(&g_lock);
      a->status->tick_ticks = tick.ticks;
      a->status->tick_missed = tick.missed;
      pthread_mutex_unlock(&g_lock);
    }

  return NULL;
}

/* Receive poll timeout. Nothing is timed against it any more - the downlink
 * has its own tick - so it only bounds how quickly a disconnect or a stop
 * request is noticed.
 */

#define COMP_POLL_MS 20

static bool comp_service(int fd, FAR struct pollfd *pfd,
                         int pose_pub,
                         FAR struct companion_status_s *status)
{
  while (!g_should_stop)
    {
      uint8_t buf[COMP_READ_MAX];
      ssize_t got;
      int ready = poll(pfd, 1, COMP_POLL_MS);

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

      /* Transmit is not here any more. It runs on its own thread against the
       * TIM6 tick, so the downlink cadence no longer depends on when this
       * loop happens to come round.
       */

      status_publish(status);
    }

  return false;
}

static int companion_daemon(int argc, FAR char *argv[])
{
  struct companion_status_s status;
  FAR const struct serial_port_s *port = NULL;
  struct pollfd pfd;
  struct comp_tx_args_s tx_args;
  pthread_t tx_thread;
  bool tx_running = false;
  bool keep_going;
  int fd = -1;
  int pose_pub = -1;
  int est_sub = -1;
  int result = EXIT_FAILURE;

  memset(&status, 0, sizeof(status));
  comp_parser_init(&status.parser);
  comp_seed_utc_from_rtc(&status);

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

  /* The tick is the downlink's clock from here on. Failing to start it is
   * fatal to this daemon rather than a quiet fall back to some other
   * cadence: a companion expecting a fixed rate should be told plainly.
   */

  if (fmuv6c_txtick_initialize(status.tx_rate_hz) < 0)
    {
      syslog(LOG_ERR, "[companion] TIM6 tick refused %" PRIu32 " Hz\n",
             status.tx_rate_hz);
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

      /* The downlink thread lives exactly as long as this connection, so the
       * descriptor it writes to is opened before it starts and closed only
       * after it has been joined.
       */

      g_tx_stop = false;
      g_tx_last_sample = 0;
      tx_args.fd = fd;
      tx_args.est_sub = est_sub;
      tx_args.status = &status;
      tx_running = pthread_create(&tx_thread, NULL, comp_tx_thread,
                                  &tx_args) == 0;

      if (!tx_running)
        {
          syslog(LOG_ERR, "[companion] no downlink thread (%d); receive "
                 "only\n", errno);
        }

      keep_going = comp_service(fd, &pfd, pose_pub, &status);

      if (tx_running)
        {
          g_tx_stop = true;
          pthread_join(tx_thread, NULL);
          tx_running = false;
        }

      if (!keep_going)
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

  /* Stop the tick before the thread that waits on it, or the join blocks
   * for a whole timeout period.
   */

  if (tx_running)
    {
      g_tx_stop = true;
      pthread_join(tx_thread, NULL);
    }

  fmuv6c_txtick_uninitialize();

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
