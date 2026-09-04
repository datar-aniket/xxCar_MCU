/****************************************************************************
 * apps/companion/companion.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
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
#include "comp_clock.h"
#include "comp_state.h"
#include "../control_router/control_router.h"
#include "../param/param.h"
#include "../rc_in/rc_in.h"
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

/* Corrected UTC is an affine view of TIM5, not CLOCK_REALTIME and not one
 * mutable offset. The first timesync establishes absolute phase; later
 * observations discipline rate and slew residual phase without a jump.
 */

/* The downlink runs on its own thread so the TIM6 tick reaches the wire
 * without waiting for whatever the receive path happens to be doing. It
 * lives exactly as long as one connection, so the descriptor it writes to
 * cannot be closed underneath it.
 */

static volatile bool g_tx_stop;
static uint64_t g_tx_last_sample;

/* Tachometer differentiator. Advanced when a NEW vesc_status arrives, not on
 * the downlink tick: STATUS_5 comes in at tens of hertz against a 200 Hz
 * downlink, so ticking it here would differentiate an unchanged count most
 * of the time.
 */

/* Scalars turning VESC telemetry into engineering units. Read once at start
 * like every other parameter here.
 */

static float g_torque_k = 1.0f;
static float g_steer_k = 1.0f;
static float g_state_speed_k = 1.0f;
static uint8_t g_steer_source;
static uint8_t g_rc_steering_index;
static uint8_t g_rc_throttle_index;
static uint16_t g_rc_switch_high;

#define COMP_STEER_SOURCE_VESC_ADC 0u
#define COMP_STEER_SOURCE_COMMAND  1u

/* How old an autonomous command may be by the time it lands here.
 *
 * AUTO_CMD_TO_MS, the SAME parameter the control router ages its input
 * against, deliberately reused rather than given a companion-side twin. The
 * two measure different halves of one journey - transport latency here,
 * silence from the companion there - and a second parameter would let an
 * operator set them apart and believe they had one budget.
 */

static uint32_t g_auto_timeout_us = 200000u;

/* How far ahead of its arrival a command's timestamp may be before the
 * offset, rather than the link, is the thing that is wrong. Comfortably
 * above the residual a completed timesync leaves and far below any budget an
 * operator can set.
 */

#define COMP_DIRECT_FUTURE_US  5000

/* The wire enum has to be the topic's enum, or "20 amps" becomes "duty 20"
 * and clamps to full throttle. comp_proto.h cannot include uorb_msgs.h, so
 * the agreement is checked here, where both are visible.
 */

static_assert(COMP_THROTTLE_DUTY == ACTUATOR_MODE_DUTY,
              "wire duty mode must match ACTUATOR_MODE_DUTY");
static_assert(COMP_THROTTLE_CURRENT == ACTUATOR_MODE_CURRENT,
              "wire current mode must match ACTUATOR_MODE_CURRENT");
static_assert(COMP_TRAJ_MAX_HORIZON == CONTROL_TRAJECTORY_MAX_HORIZON,
              "wire and uORB trajectory horizons must match");

static pthread_mutex_t g_clock_lock = PTHREAD_MUTEX_INITIALIZER;
static struct comp_clock_s g_clock;

static bool comp_utc_valid(void)
{
  bool valid;

  pthread_mutex_lock(&g_clock_lock);
  valid = g_clock.valid;
  pthread_mutex_unlock(&g_clock_lock);
  return valid;
}

static bool comp_mono_to_utc(uint64_t mono_us, FAR uint64_t *utc_us)
{
  bool valid;

  pthread_mutex_lock(&g_clock_lock);
  valid = comp_clock_to_utc(&g_clock, mono_us, utc_us);
  pthread_mutex_unlock(&g_clock_lock);
  return valid;
}

static bool comp_utc_to_mono(uint64_t utc_us, FAR uint64_t *mono_us)
{
  bool valid;

  pthread_mutex_lock(&g_clock_lock);
  valid = comp_clock_from_utc(&g_clock, utc_us, mono_us);
  pthread_mutex_unlock(&g_clock_lock);
  return valid;
}

/* Anything before this cannot be a real UTC time on this vehicle, so it is a
 * clock that was never set rather than one that is merely wrong. NuttX reads
 * an unset RTC as its build epoch, which looks like a perfectly plausible
 * date and would make every wire timestamp confidently wrong.
 */

#define COMP_UTC_PLAUSIBLE_S  1700000000ll   /* 2023-11-14 */

/* THE board clock, for everything this link timestamps or converts.
 *
 * fmuv6c_imu_time_now(), not clock_gettime(CLOCK_MONOTONIC), and the
 * distinction is not cosmetic.
 *
 * Every timestamp the affine UTC model converts already lives in this
 * domain: the IMU sample times that reach estimator_state come from
 * fmuv6c_imu_time_now() in icm42688.c, and so does the PPS edge. The offset
 * was being MEASURED against CLOCK_MONOTONIC - by the RTC seed and by the
 * board timestamps in the timesync replies - and then applied to TIM5
 * values.
 *
 * Those are two independent counters. TIM5 free-runs on the APB clock and
 * CLOCK_MONOTONIC advances on the systick; fmuv6c_imu_time_now() re-anchors
 * to the coarse clock only when TIM5 wraps, every 71.6 minutes, so between
 * wraps any rate difference accumulates unopposed. Mixing them imports that
 * divergence straight into the emitted UTC, where it reads as a solution
 * time that leads or lags the host by tens of milliseconds - and it moves,
 * because the divergence grows.
 *
 * One clock, used for every conversion and disciplined by the periodic UTC
 * observations, is the only version of this that cannot drift against
 * itself.
 */

static uint64_t comp_now_us(void)
{
  return fmuv6c_imu_time_now();
}

/* The coarse clock, kept only to measure how far it has drifted from the
 * one above. Nothing converts with it.
 */

static uint64_t comp_coarse_us(void)
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

  {
    uint64_t mono_us = comp_now_us();
    int64_t utc_us = (int64_t)rt.tv_sec * 1000000ll +
                     (int64_t)rt.tv_nsec / 1000ll;

    pthread_mutex_lock(&g_clock_lock);
    s->utc_from_rtc = comp_clock_seed(&g_clock, mono_us, utc_us);
    pthread_mutex_unlock(&g_clock_lock);
  }

  if (!s->utc_from_rtc)
    {
      return;
    }

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
                       int fd, uint64_t rx_us, int pose_pub, int control_pub,
                       int trajectory_pub,
                       FAR struct companion_status_s *s)
{
  if (id == COMP_MSG_EXTERNAL_POSE)
    {
      struct comp_external_pose_s wire;
      struct external_pose_s out;
      bool apply_datum_reset;

      memcpy(&wire, parser->payload, sizeof(wire));
      memset(&out, 0, sizeof(out));

      /* The parser supplied the TIM5 time at which the final frame byte was
       * received. Preserve that receive constraint just as ArduPilot's UART
       * timestamp correction does; stamping after routing adds software
       * scheduling to the measured link age.
       */

      out.timestamp = rx_us;

      /* Convert UTC back to the board's shared TIM5 sample clock, which is
       * the timebase the estimator trajectory uses. Zero stays zero - it
       * means "not timestamped" and the estimator stamps it on arrival.
       *
       * Unsynced, a supplied UTC cannot be converted at all, so it is turned
       * into that same zero rather than fused as a number a thousand times
       * larger than anything the horizon expects.
       */

      if (wire.timestamp_us == 0 ||
          !comp_utc_to_mono(wire.timestamp_us, &out.timestamp_sample))
        {
          out.timestamp_sample = 0;

          if (wire.timestamp_us != 0)
            {
              s->rx_unsynced_stamp++;
            }
        }
      out.x = wire.x;
      out.y = wire.y;
      out.yaw = wire.yaw;
      memcpy(out.cov, wire.cov, sizeof(out.cov));
      out.flags = wire.flags & COMP_POSE_FLAG_VALID;
      out.reset_counter = wire.reset_counter;

      /* DATUM_RESET is separate from the pose so retransmissions can be
       * idempotent, but the operation itself must be atomic with a pose.
       * Mark exactly the next VALID pose; invalid localisation output cannot
       * become a datum, and a publish failure leaves the request armed for
       * the next attempt.
       */

      apply_datum_reset = s->datum_reset_pending &&
        (out.flags & EXTERNAL_POSE_VALID) != 0;

      if (apply_datum_reset)
        {
          out.flags |= EXTERNAL_POSE_RESET_DATUM;
        }

      if (external_pose_publish(pose_pub, &out) < 0)
        {
          s->rx_publish_errors++;
          return;
        }

      if (apply_datum_reset)
        {
          s->datum_reset_pending = false;
          s->datum_reset_dispatched++;
        }

      s->rx_pose++;
      s->last_rx_us = out.timestamp;
    }

  else if (id == COMP_MSG_DATUM_RESET)
    {
      struct comp_datum_reset_s wire;

      memcpy(&wire, parser->payload, sizeof(wire));

      /* The request counter is a transaction id. UART reconnects and host
       * retry timers may deliver a frame twice; only a new counter is allowed
       * to arm another discontinuous datum change.
       */

      if (!s->datum_reset_seen ||
          wire.request_counter != s->last_datum_request)
        {
          s->datum_reset_seen = true;
          s->last_datum_request = wire.request_counter;
          s->datum_reset_pending = true;
          s->rx_datum_reset++;
        }
      else
        {
          s->rx_datum_duplicate++;
        }

      s->last_rx_us = rx_us;
    }

  else if (id == COMP_MSG_CONTROL_TRAJ)
    {
      struct comp_control_trajectory_s wire;
      struct control_trajectory_s out;
      uint64_t current_us;
      uint64_t solution_us;
      int64_t age_us;

      if (!comp_control_trajectory_decode(parser->payload, parser->len,
                                          &wire))
        {
          s->rx_trajectory_invalid++;
          return;
        }

      if (!comp_utc_to_mono(wire.timestamp_us, &current_us) ||
          !comp_utc_to_mono(wire.solution_time_us, &solution_us))
        {
          s->rx_trajectory_stale++;
          return;
        }

      age_us = (int64_t)rx_us - (int64_t)current_us;

      if (age_us < -(int64_t)COMP_DIRECT_FUTURE_US ||
          age_us > (int64_t)g_auto_timeout_us ||
          (solution_us > current_us &&
           solution_us - current_us > COMP_DIRECT_FUTURE_US))
        {
          s->rx_trajectory_stale++;
          return;
        }

      memset(&out, 0, sizeof(out));
      out.timestamp = rx_us;
      out.timestamp_sample = current_us;
      out.solution_time = solution_us;
      out.dt = wire.dt;
      out.horizon = wire.horizon;
      out.control_method = wire.control_method;
      memcpy(out.poses, wire.poses, sizeof(out.poses));
      memcpy(out.controls, wire.controls, sizeof(out.controls));

      if (control_trajectory_publish(trajectory_pub, &out) < 0)
        {
          s->rx_publish_errors++;
          return;
        }

      s->rx_trajectory++;
      s->last_trajectory_horizon = out.horizon;
      s->last_trajectory_dt = out.dt;
      s->last_rx_us = rx_us;
    }

  else if (id == COMP_MSG_DIRECT_CONTROL)
    {
      struct comp_direct_control_s wire;
      struct control_cmd_s out;
      uint64_t sample_us;
      int64_t age_us;

      memcpy(&wire, parser->payload, sizeof(wire));

      /* No usable timestamp, no command.
       *
       * EXTERNAL_POSE treats an unconvertible stamp as "stamp it on arrival"
       * and carries on, because a pose with the wrong age is still mostly
       * right. A command is not: its whole safety argument is that it
       * expires, and an age that cannot be computed cannot expire.
       */

      if (wire.timestamp_us == 0 ||
          !comp_utc_to_mono(wire.timestamp_us, &sample_us))
        {
          s->rx_direct_stale++;
          return;
        }

      /* rx_us is the TIM5 time the frame completed and sample_us is now in
       * that same domain, so this subtraction stays inside one clock. The
       * board-side half of the same 100 ms budget - the companion having
       * stopped sending altogether - is the control router's job, measured
       * in ITS clock, which is why the published timestamp below is zero.
       */

      age_us = (int64_t)rx_us - (int64_t)sample_us;

      /* Ahead of its own arrival is impossible, so any negative age is clock
       * error rather than a fast link. A few milliseconds of it is the
       * timesync residual and is tolerated; more than that means the offset
       * is wrong, and a wrong offset makes every age reading meaningless in
       * the same direction - which would quietly disable the expiry this
       * whole path exists for.
       */

      if (age_us < -(int64_t)COMP_DIRECT_FUTURE_US ||
          age_us > (int64_t)g_auto_timeout_us)
        {
          s->rx_direct_stale++;
          return;
        }

      if (!comp_direct_control_valid(&wire))
        {
          s->rx_direct_invalid++;
          return;
        }

      memset(&out, 0, sizeof(out));

      /* Zero, so the control router stamps it on arrival in its own clock.
       *
       * The router ages this against CLOCK_MONOTONIC while everything here
       * is TIM5. Those are independent counters - see the comment on
       * comp_now_us() - so a TIM5 value put in this field would read as
       * either permanently fresh or permanently stale depending on which way
       * the two had drifted. `vesc set` publishes zero for the same reason.
       */

      out.timestamp = 0;
      out.motor = wire.throttle;
      out.steering = wire.steering;
      out.mode = wire.throttle_type;

      if (control_cmd_publish(control_pub, &out) < 0)
        {
          s->rx_publish_errors++;
          return;
        }

      s->rx_direct++;
      s->last_direct_us = rx_us;
      s->last_direct_age_us = age_us > 0 ? (uint32_t)age_us : 0u;
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
      int sync_result = COMP_CLOCK_SYNC_REJECTED;

      memcpy(&end, parser->payload, sizeof(end));
      s->timesync_offset_us = end.utc_offset_us;
      s->timesync_trip_us = end.trip_us;
      s->timesync_samples = end.samples;

      if (end.samples > 0)
        {
          pthread_mutex_lock(&g_clock_lock);
          sync_result = comp_clock_observe_sync(&g_clock, rx_us,
                                                end.utc_offset_us);

          s->utc_rate_ppb = g_clock.rate_ppb;
          s->utc_base_rate_ppb = g_clock.base_rate_ppb;
          s->timesync_phase_error_us = g_clock.last_phase_error_us;
          s->timesync_updates = g_clock.sync_updates;
          s->timesync_rate_rejected = g_clock.rejected_observations;

          pthread_mutex_unlock(&g_clock_lock);
        }

      s->timesync_synced = sync_result > 0;

      if (s->timesync_synced)
        {
          uint64_t now_utc;

          s->utc_from_rtc = false;

          /* The first authoritative sync establishes absolute UTC and may
           * replace the RTC's second-resolution seed. Later syncs change
           * only the affine rate and are exactly continuous at rx_us.
           * Re-establish the PPS phase after either kind of update so its
           * next edge is compared with the new discipline, not the old one.
           */

          s->pps_phase_valid = false;

          /* Set CLOCK_REALTIME once. It and the hardware RTC are then free
           * running and are never used for wire timestamp conversion. The
           * affine TIM5 clock above is the UTC authority after this point.
           */

          if (sync_result == COMP_CLOCK_SYNC_FIRST &&
              comp_mono_to_utc(rx_us, &now_utc))
            {
              struct timespec utc;

              utc.tv_sec = (time_t)(now_utc / 1000000ull);
              utc.tv_nsec = (long)((now_utc % 1000000ull) * 1000ull);

              if (clock_settime(CLOCK_REALTIME, &utc) == 0)
                {
                  s->wall_clock_set = true;
                }
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

}

/* Steer the affine UTC rate so the companion's PPS edge keeps its established
 * phase (or lands on a whole second when PPS_ABS_PHASE is selected).
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

  if (!comp_utc_valid() || !pps.running ||
      pps.state != FMUV6C_PPS_LOCKED || pps.last_edge_us == 0)
    {
      return;
    }

  if (pps.last_edge_us == s->pps_edge_used)
    {
      return;                     /* already applied to this edge */
    }

  now = comp_now_us();

  if (now < pps.last_edge_us || now - pps.last_edge_us > 2000000ull)
    {
      return;                     /* stale, or the clock moved under us */
    }

  {
    uint64_t converted;

    if (!comp_mono_to_utc(pps.last_edge_us, &converted) ||
        converted > INT64_MAX)
      {
        return;
      }

    utc_at_edge = (int64_t)converted;
  }
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

  s->pps_edge_used = pps.last_edge_us;
  s->pps_residual_us = (int32_t)residual;

  /* THE PULSE MARKS A CONSISTENT INSTANT, NOT NECESSARILY THE UTC SECOND.
   *
   * It comes from a hardware timer on the companion, so its RATE is
   * excellent - microsecond-stable, which is the whole reason to want it.
   * Its PHASE is another matter: a 1 Hz output starts wherever the timer was
   * enabled, so unless something steers it to the system clock its edges sit
   * at a fixed, arbitrary offset from the true second.
   *
   * Forcing that offset to zero - which is what this used to do - drags the
   * board's clock away from UTC by exactly the pulse's phase error and
   * leaves every timestamp wrong by the same amount. Tens of milliseconds of
   * constant offset is entirely normal for a free-running PWM, and it made
   * the link measurably worse than having no PPS at all.
   *
   * So the first edge after a sync ESTABLISHES the phase rather than being
   * corrected to zero. Timesync owns the phase; PPS owns the rate. What is
   * corrected from then on is the DRIFT away from that established phase,
   * which is the clock error PPS is actually good at seeing.
   */

  if (!s->pps_phase_valid)
    {
      s->pps_phase_ref_us = (int32_t)residual;
      s->pps_phase_valid = true;

      /* PPS_ABS_PHASE says the pulse marks the true second, so its phase is
       * authoritative and the disagreement with timesync is timesync's
       * error to give up. Fall through and correct it.
       *
       * Otherwise the phase is merely established here and only drift from
       * it is corrected afterwards.
       */

      if (!s->pps_absolute_phase)
        {
          return;
        }
    }

  if (s->pps_absolute_phase)
    {
      /* The pulse IS the second. Null the residual outright rather than
       * measuring drift against a phase that is defined to be zero.
       */

      s->pps_phase_ref_us = 0;
    }

  residual -= (int64_t)s->pps_phase_ref_us;

  /* The phase reference itself wraps: a drift measured across the second
   * boundary reads as most of a second rather than a few microseconds.
   */

  if (residual > 500000ll)
    {
      residual -= 1000000ll;
    }
  else if (residual < -500000ll)
    {
      residual += 1000000ll;
    }

  s->pps_drift_us = (int32_t)residual;

  if (labs((long)residual) > labs((long)s->pps_worst_us))
    {
      s->pps_worst_us = (int32_t)residual;
    }

  /* Drift this large is not drift. Something moved the clock, or the pulse
   * is not the one we established the phase against; either way the
   * timesync offset is the better answer.
   */

  if (labs((long)residual) > (long)s->pps_max_correction_us)
    {
      s->pps_rejected++;
      return;
    }

  /* Remove the phase error by changing the affine rate over the following
   * second. Re-anchoring preserves the UTC value at `now`, so PPS can never
   * step an outgoing timestamp or move an incoming sample discontinuously
   * across the EKF history.
   */

  pthread_mutex_lock(&g_clock_lock);

  if (comp_clock_adjust_phase(&g_clock, now, -residual, 1000000ull))
    {
      s->utc_rate_ppb = g_clock.rate_ppb;
      s->utc_base_rate_ppb = g_clock.base_rate_ppb;
      s->pps_corrections++;
    }

  pthread_mutex_unlock(&g_clock_lock);
}


static void comp_transmit(int fd, int state_pub, int est_sub, int gyro_sub,
                          int accel_sub, int vesc_sub, int rc_sub,
                          FAR struct companion_status_s *s)
{
  struct estimator_state_s est;
  struct vehicle_gyro_s gyro;
  struct vehicle_accel_s accel;
  struct vesc_status_s vesc;
  struct rc_in_s rc;
  struct control_router_status_s router;
  struct comp_state_inputs_s in;
  struct comp_vehicle_state_s wire;
  struct vehicle_state_tx_s logged;
  uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  int64_t now_utc;
  uint64_t stamp;
  uint64_t accel_sample_time = 0;
  uint32_t gap = 0;
  bool repeat = false;
  bool clamped = false;
  int n;

  memset(&in, 0, sizeof(in));
  in.torque_k = g_torque_k;
  in.steer_k = g_steer_k;
  in.state_speed_k = g_state_speed_k;
  memset(&router, 0, sizeof(router));
  control_router_status(&router);
  in.control_armed = router.armed;
  in.control_auto = router.source == ROUTER_SOURCE_AUTO;
  in.control_current = router.mode == ROUTER_MODE_CURRENT;

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

  in.est_valid = true;
  memcpy(in.position, est.position, sizeof(in.position));
  memcpy(in.quaternion, est.quaternion, sizeof(in.quaternion));
  memcpy(in.velocity_enu, est.velocity, sizeof(in.velocity_enu));
  memcpy(in.accel_bias, est.accel_bias, sizeof(in.accel_bias));
  in.solution_status = est.solution_status;
  in.reset_counter = (uint8_t)est.reset_counter;

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

  if (gyro_sub >= 0 && orb_copy(ORB_ID(vehicle_gyro), gyro_sub, &gyro) >= 0)
    {
      in.gyro_valid = true;
      in.gyro[0] = gyro.x;
      in.gyro[1] = gyro.y;
      in.gyro[2] = gyro.z;
    }

  if (accel_sub >= 0 &&
      orb_copy(ORB_ID(vehicle_accel), accel_sub, &accel) >= 0)
    {
      in.accel_valid = true;
      in.accel[0] = accel.x;
      in.accel[1] = accel.y;
      in.accel[2] = accel.z;
      accel_sample_time = accel.timestamp_sample;
    }

  if (vesc_sub >= 0 && orb_copy(ORB_ID(vesc_status), vesc_sub, &vesc) >= 0)
    {
      in.vesc_valid = true;
      in.current_a = vesc.current_a;

      /* Already differentiated and filtered by the VESC daemon, which sees
       * every STATUS_5 at 400 Hz. Doing it here would be too late: this
       * topic has no queue, so reading it at 200 Hz has already dropped half
       * the samples.
       */

      in.motor_counts_per_s = vesc.speed_cps;

      if (g_steer_source == COMP_STEER_SOURCE_COMMAND)
        {
          /* This is the pulse after the router, VESC mapping, safety state,
           * and clamp. It therefore describes what was actually transmitted,
           * whether the selected control source was RC or autonomous.
           */

          if (vesc.servo_us != 0)
            {
              in.steering_valid = true;
              in.steering_measured = false;
              in.steering_feedback =
                comp_state_servo_feedback(vesc.servo_us);
            }
        }
      else
        {
          in.steering_valid = true;
          in.steering_measured = true;
          in.steering_feedback = vesc.adc_volts;
        }
    }

  if (rc_sub >= 0 && router.rc_valid &&
      orb_copy(ORB_ID(rc_in), rc_sub, &rc) >= 0 &&
      g_rc_steering_index < rc.count && g_rc_throttle_index < rc.count &&
      rc.count > 5u && rc.ok != 0 && rc.failsafe == 0)
    {
      in.rc_valid = true;
      in.rc_steering_pwm = rc.channel[g_rc_steering_index];
      in.rc_throttle_pwm = rc.channel[g_rc_throttle_index];
      in.trigger_high = rc.channel[5] >= g_rc_switch_high;
    }

  /* UTC on the wire once synced. Before that the companion gets the board's
   * raw TIM5 time, which is all there is to give.
   */

  if (!comp_mono_to_utc(est.timestamp_sample, &stamp))
    {
      stamp = est.timestamp_sample;
    }

  /* A solution cannot be newer than now, so refuse to say that it is.
   *
   * comp_now_us() deliberately reads fmuv6c_imu_time_now().  Using the
   * tick-quantized CLOCK_MONOTONIC reference here would clamp perfectly good
   * sample stamps for up to a millisecond after every system tick and count a
   * fault that had not happened.
   */

  {
    uint64_t converted;
    uint64_t now_mono = comp_now_us();

    now_utc = comp_mono_to_utc(now_mono, &converted) ?
              (int64_t)converted : (int64_t)now_mono;
  }

  if ((int64_t)stamp > now_utc)
    {
      stamp = (uint64_t)now_utc;
      clamped = true;
    }

  comp_state_build(&in, stamp, &wire);

  n = comp_encode(COMP_MSG_VEHICLE_STATE, &wire, sizeof(wire), frame,
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

  /* Mirror what was actually delivered, not merely what we intended to
   * deliver.  The first timestamp remains in board monotonic time for ULog
   * joins; every field after it is copied from the successful wire payload.
   * vehicle_accel is logged separately so the gravity-removal input and this
   * output can be compared sample for sample.
   */

  memset(&logged, 0, sizeof(logged));
  logged.timestamp = comp_now_us();
  logged.timestamp_sample = est.timestamp_sample;
  logged.accel_timestamp_sample = accel_sample_time;
  logged.wire_timestamp_us = wire.timestamp_us;
  memcpy(logged.position, wire.position, sizeof(logged.position));
  memcpy(logged.quaternion, wire.quaternion, sizeof(logged.quaternion));
  memcpy(logged.velocity, wire.velocity, sizeof(logged.velocity));
  memcpy(logged.angular_velocity, wire.angular_velocity,
         sizeof(logged.angular_velocity));
  logged.side_slip_rad = wire.side_slip_rad;
  memcpy(logged.accel, wire.accel, sizeof(logged.accel));
  logged.wheel_torque_nm = wire.wheel_torque_nm;
  logged.steering_angle = wire.steering_angle;
  logged.motor_speed_ms = wire.motor_speed_ms;
  logged.solution_status = wire.solution_status;
  logged.reset_counter = wire.reset_counter;
  logged.source_valid = wire.source_valid;
  logged.rc_status = wire.rc_status;
  vehicle_state_tx_publish(state_pub, &logged);

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
  int state_pub;
  int est_sub;
  int gyro_sub;
  int accel_sub;
  int vesc_sub;
  int rc_sub;
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
          comp_transmit(a->fd, a->state_pub, a->est_sub, a->gyro_sub,
                        a->accel_sub,
                        a->vesc_sub, a->rc_sub, a->status);
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

/* Downlink thread stack. comp_transmit's message structs are about 570 bytes
 * on their own; this leaves room for orb_copy and the serial write beneath
 * them.
 */

#define COMP_TX_STACK 3072

static bool comp_service(int fd, FAR struct pollfd *pfd,
                         int pose_pub, int control_pub,
                         int trajectory_pub,
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
                                 pose_pub, control_pub, trajectory_pub,
                                 status);
                    }
                }
            }
          else if (got == 0 || (errno != EAGAIN && errno != EINTR))
            {
              return true;        /* cable pulled */
            }
        }

      status->clock_skew_us = (int64_t)comp_now_us() -
                              (int64_t)comp_coarse_us();

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
  pthread_attr_t tx_attr;
  bool tx_running = false;
  bool keep_going;
  int fd = -1;
  int pose_pub = -1;
  int control_pub = -1;
  int trajectory_pub = -1;
  int state_pub = -1;
  int est_sub = -1;
  int gyro_sub = -1;
  int accel_sub = -1;
  int vesc_sub = -1;
  int rc_sub = -1;
  int result = EXIT_FAILURE;

  memset(&status, 0, sizeof(status));
  comp_parser_init(&status.parser);
  pthread_mutex_lock(&g_clock_lock);
  comp_clock_init(&g_clock);
  pthread_mutex_unlock(&g_clock_lock);
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
  g_torque_k = param_f32("VESC_TORQUE_K");
  g_steer_k = param_f32("VESC_STEER_K");
  g_state_speed_k = param_f32("VESC_STATE_K");
  g_steer_source = (uint8_t)param_i32("STEER_FB_SRC");
  {
    int32_t steering_map = param_i32("RC_MAP_STEERING");
    int32_t throttle_map = param_i32("RC_MAP_THROTTLE");

    g_rc_steering_index = steering_map >= 1 &&
                          steering_map <= RC_IN_MAX_CHANNELS ?
                          (uint8_t)(steering_map - 1) : UINT8_MAX;
    g_rc_throttle_index = throttle_map >= 1 &&
                          throttle_map <= RC_IN_MAX_CHANNELS ?
                          (uint8_t)(throttle_map - 1) : UINT8_MAX;
    g_rc_switch_high = (uint16_t)param_i32("RC_SW_HIGH");
  }
  status.pps_max_correction_us = (uint32_t)param_i32("PPS_MAX_COR_US");
  status.pps_absolute_phase = param_i32("PPS_ABS_PHASE") != 0;
  g_auto_timeout_us = (uint32_t)param_i32("AUTO_CMD_TO_MS") * 1000u;
  status.auto_timeout_us = g_auto_timeout_us;

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

  /* The autonomous command input. Advertised unconditionally: the control
   * router only acts on it when the RC source switch selects AUTO and its own
   * arm sequence has completed, so having the topic available is not the same
   * as handing the vehicle over.
   */

  control_pub = control_cmd_advertise();

  if (control_pub < 0)
    {
      syslog(LOG_ERR, "[companion] cannot advertise control_cmd (%d)\n",
             errno);
      goto out;
    }

  trajectory_pub = control_trajectory_advertise();

  if (trajectory_pub < 0)
    {
      syslog(LOG_ERR,
             "[companion] cannot advertise control_trajectory (%d)\n",
             errno);
      goto out;
    }

  state_pub = vehicle_state_tx_advertise();

  if (state_pub < 0)
    {
      syslog(LOG_ERR, "[companion] cannot advertise vehicle_state_tx (%d)\n",
             errno);
      goto out;
    }

  /* The estimator may not be running. That is not a failure: the link still
   * receives, and starts transmitting when ekf3 comes up.
   */

  est_sub = orb_subscribe(ORB_ID(estimator_state));

  /* The IMU and VESC feeds are equally optional. A missing one leaves its
   * fields zero and its bit clear in source_valid, which the companion can
   * see - rather than the daemon refusing to run because one sensor of four
   * is absent.
   */

  gyro_sub = orb_subscribe(ORB_ID(vehicle_gyro));
  accel_sub = orb_subscribe(ORB_ID(vehicle_accel));
  vesc_sub = orb_subscribe(ORB_ID(vesc_status));
  rc_sub = orb_subscribe(ORB_ID(rc_in));

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
      tx_args.state_pub = state_pub;
      tx_args.est_sub = est_sub;
      tx_args.gyro_sub = gyro_sub;
      tx_args.accel_sub = accel_sub;
      tx_args.vesc_sub = vesc_sub;
      tx_args.rc_sub = rc_sub;
      tx_args.status = &status;
      /* An EXPLICIT stack, not the default.
       *
       * comp_transmit alone puts about 570 bytes of message structs on the
       * stack before calling into orb_copy and the serial write, and this
       * board has already been rebooted once by a stack overflow that
       * produced no assert output at all - the assert handler needs stack
       * too, so it dies as well. Leaving that to whatever
       * PTHREAD_STACK_DEFAULT happens to be is not worth the bytes saved.
       */

      if (pthread_attr_init(&tx_attr) == 0)
        {
          pthread_attr_setstacksize(&tx_attr, COMP_TX_STACK);
          tx_running = pthread_create(&tx_thread, &tx_attr, comp_tx_thread,
                                      &tx_args) == 0;
          pthread_attr_destroy(&tx_attr);
        }
      else
        {
          tx_running = false;
        }

      if (!tx_running)
        {
          syslog(LOG_ERR, "[companion] no downlink thread (%d); receive "
                 "only\n", errno);
        }

      keep_going = comp_service(fd, &pfd, pose_pub, control_pub,
                                trajectory_pub,
                                &status);

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

  if (gyro_sub >= 0)
    {
      orb_unsubscribe(gyro_sub);
    }

  if (accel_sub >= 0)
    {
      orb_unsubscribe(accel_sub);
    }

  if (vesc_sub >= 0)
    {
      orb_unsubscribe(vesc_sub);
    }

  if (rc_sub >= 0)
    {
      orb_unsubscribe(rc_sub);
    }

  if (pose_pub >= 0)
    {
      orb_unadvertise(pose_pub);
    }

  if (control_pub >= 0)
    {
      orb_unadvertise(control_pub);
    }

  if (trajectory_pub >= 0)
    {
      orb_unadvertise(trajectory_pub);
    }

  if (state_pub >= 0)
    {
      orb_unadvertise(state_pub);
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
