/****************************************************************************
 * apps/vesc/vesc.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
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
#include "vesc_cmd.h"
#include "vesc_speed.h"
#include "../param/param.h"
#include "../rc_in/rc_in.h"
#include "../uorb_msgs/uorb_msgs.h"

#define VESC_PRIORITY   (SCHED_PRIORITY_DEFAULT + 10)
#define VESC_STACK      2048

/* Discovery mode has no transmit deadline to wake the task. Keep a bounded
 * wait so `vesc stop` and status refresh remain responsive even on a silent
 * bus. Normal operation wakes at the next transmit deadline instead.
 */

#define VESC_IDLE_WAIT_US  100000

/* Bound on one drain pass. Without it a bus stuck jabbering would keep this
 * loop from ever reaching g_should_stop, and `vesc stop` would hang.
 */

#define VESC_DRAIN_MAX  32

#define VESC_RC_TRIM_CHANNEL  7u
#define VESC_RC_TRIM_INDEX    (VESC_RC_TRIM_CHANNEL - 1u)
#define VESC_RC_PWM_VALID_MIN 750u
#define VESC_RC_PWM_VALID_MAX 2250u

/* The two mode enumerations have to agree. They are declared separately
 * because vesc_cmd.h must compile on a host without uORB, so this is the
 * only place that can check them.
 */

static_assert(VESC_MODE_DUTY == ACTUATOR_MODE_DUTY, "mode mismatch");
static_assert(VESC_MODE_CURRENT == ACTUATOR_MODE_CURRENT, "mode mismatch");

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool g_running;
static volatile bool g_should_stop;

/* Written by vesc_arm() from a caller's thread, read by the daemon. A bool
 * is a single-word store on this target and the daemon only ever reads it;
 * the mutex protects the status copy, not this.
 */

static volatile bool g_armed;

/* Read by vesc_arm(). Set once at start and never changed, but reading it
 * out of g_status would mean reading a structure the daemon rewrites every
 * 2 ms under a mutex, for a value that does not need one.
 */

static uint32_t g_cmd_timeout_ms;

/* Board time of the last decoded STATUS_5, and the watchdog that acts on its
 * absence. Written by the daemon thread only.
 */

static uint64_t g_last_tlm_us;
static volatile bool g_tlm_lost;
static struct vesc_daemon_status_s g_status;

/* Every decoded STATUS_5 reaches this, at the full 400 Hz - which is the
 * only place that rate still exists, since vesc_status has no queue.
 */

static struct vesc_speed_s g_speed;

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
  uint64_t sample_us = frame->ts != 0 ? frame->ts : now;
  struct vesc_status5_s decoded;
  struct vesc_status_s out;

  /* CLOCK_MONOTONIC is tick-quantized on this configuration while the ISR
   * timestamp is not. Do not publish a reception time that appears to occur
   * before its sample by the sub-tick remainder.
   */

  if (now < sample_us)
    {
      now = sample_us;
    }

  vesc_note_seen(s, packet_id, controller_id, sample_us);

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
  out.timestamp_sample = sample_us;
  out.tachometer = decoded.tachometer;
  out.current_a = decoded.current_a;
  out.adc_volts = decoded.adc_volts;
  out.controller_id = controller_id;
  out.servo_us = s->last_servo_us;

  /* Differentiate and filter here, on every frame, before anything
   * downsamples this topic.
   */

  /* sample_us, not now: the ISR timestamp comes from TIM5 in microseconds
   * while CLOCK_MONOTONIC advances in 1 ms steps. Differentiating a 2.5 ms
   * interval against a 1 ms clock reads it as 2 or 3 ms, a 20% error on
   * every single sample.
   */

  out.speed_cps = vesc_speed_update(&g_speed, decoded.tachometer, sample_us);

  if (vesc_status_publish(pub, &out) < 0)
    {
      s->publish_errors++;
      return;
    }

  s->last = decoded;
  s->last_us = sample_us;
  s->decoded++;
}

/* The newest setpoint, kept whole. Its own struct rather than fields on the
 * status so that resolving a command needs no lock: the daemon is the only
 * writer and the only reader.
 */

struct vesc_setpoint_s
{
  bool     valid;
  uint8_t  mode;
  float    motor;
  float    steering;
  uint64_t stamp_us;
};

static struct vesc_setpoint_s g_setpoint;

static void vesc_take_setpoints(int sub, FAR struct vesc_daemon_status_s *s)
{
  bool updated = false;
  int drained = 0;

  if (sub < 0 || orb_check(sub, &updated) < 0 || !updated)
    {
      return;
    }

  while (drained++ < VESC_DRAIN_MAX)
    {
      struct actuator_command_s message;
      uint64_t now;

      if (orb_copy(ORB_ID(actuator_command), sub, &message) < 0)
        {
          return;
        }

      now = vesc_now_us();

      /* A zero or future timestamp is stamped on arrival rather than
       * refused. Age is unsigned, and now - future underflows to something
       * enormous, which would read as permanently stale.
       */

      if (message.timestamp == 0 || message.timestamp > now)
        {
          message.timestamp = now;
        }

      g_setpoint.valid = true;
      g_setpoint.mode = message.mode;
      g_setpoint.motor = message.motor;
      g_setpoint.steering = message.steering;
      g_setpoint.stamp_us = message.timestamp;
      s->setpoints++;

      if (orb_check(sub, &updated) < 0 || !updated)
        {
          return;
        }
    }
}

static void vesc_take_rc(int sub, FAR struct vesc_daemon_status_s *s)
{
  struct rc_in_s rc;
  bool updated = false;
  uint64_t now;

  if (sub < 0 || orb_check(sub, &updated) < 0 || !updated ||
      orb_copy(ORB_ID(rc_in), sub, &rc) < 0)
    {
      return;
    }

  now = vesc_now_us();
  if (rc.timestamp == 0 || rc.timestamp > now)
    {
      rc.timestamp = now;
    }

  s->rc_trim_stamp_us = rc.timestamp;
  s->rc_trim_pwm = rc.count >= VESC_RC_TRIM_CHANNEL ?
                   rc.channel[VESC_RC_TRIM_INDEX] : 0u;
  s->rc_trim_input_valid = rc.ok != 0 && rc.failsafe == 0 &&
                           rc.count >= VESC_RC_TRIM_CHANNEL &&
                           s->rc_trim_pwm >= VESC_RC_PWM_VALID_MIN &&
                           s->rc_trim_pwm <= VESC_RC_PWM_VALID_MAX;
}

/* Disarm when telemetry stops.
 *
 * Checked on the LOOP, not on message arrival - the trigger is an absence,
 * and nothing arrives to notice it. This is deliberately stronger than the
 * command failsafe: a stale setpoint means we stop commanding and hold
 * neutral, but silent telemetry means we cannot see what the motor is doing
 * at all, and staying armed through that is the thing worth refusing.
 *
 * It never re-arms itself. Coming back from a comms failure is a decision
 * for whoever is standing next to the vehicle.
 */

static void vesc_telemetry_watchdog(FAR struct vesc_daemon_status_s *s)
{
  bool lost = vesc_cmd_telemetry_lost(g_last_tlm_us, vesc_now_us(),
                                      s->tlm_timeout_ms);

  s->tlm_lost = lost;

  if (!lost)
    {
      g_tlm_lost = false;
      return;
    }

  if (!g_tlm_lost)
    {
      g_tlm_lost = true;

      if (g_armed)
        {
          g_armed = false;
          s->tlm_disarms++;
          syslog(LOG_ERR, "[vesc] telemetry lost for more than %" PRIu32
                 " ms - DISARMED\n", s->tlm_timeout_ms);
        }
      else
        {
          syslog(LOG_WARNING, "[vesc] telemetry lost for more than %" PRIu32
                 " ms\n", s->tlm_timeout_ms);
        }
    }
}

static void vesc_transmit(FAR struct vesc_daemon_status_s *s)
{
  struct vesc_cmd_out_s cmd;
  struct vesc_limits_s limits;
  struct fdcan_frame_s frame;
  uint64_t now = vesc_now_us();
  uint64_t age = 0;
  bool ok;

  s->rc_trim_active = s->rc_trim_input_valid &&
                      s->rc_trim_stamp_us != 0 &&
                      s->rc_trim_stamp_us <= now &&
                      now - s->rc_trim_stamp_us <=
                      (uint64_t)s->rc_timeout_ms * 1000ull;
  s->rc_trim_us = s->rc_trim_active ?
                  vesc_cmd_rc_trim(s->rc_trim_pwm) : 0;

  /* Apply the live trim after the control router. This deliberately has no
   * dependency on whether the routed command came from RC or Auto.
   */

  limits = s->limits;
  limits.steer_offset = (int16_t)(limits.steer_offset + s->rc_trim_us);

  if (g_setpoint.valid && now > g_setpoint.stamp_us)
    {
      age = now - g_setpoint.stamp_us;
    }

  vesc_cmd_resolve(g_armed, g_setpoint.valid, g_setpoint.mode,
                   g_setpoint.motor, g_setpoint.steering,
                   age, s->cmd_timeout_ms, &limits, &cmd);

  if (cmd.reason < VESC_CMD_NREASON)
    {
      s->reason_count[cmd.reason]++;
    }

  s->last_reason = cmd.reason;
  s->armed = g_armed;

  if (cmd.clamped)
    {
      s->tx_clamped++;
    }

  memset(&frame, 0, sizeof(frame));
  frame.id = vesc_can_id(cmd.packet_id, s->filter_id);
  frame.dlc = VESC_CMD_SERVO_DLC;

  if (cmd.packet_id == VESC_PACKET_SET_CURRENT_SERVO)
    {
      ok = vesc_encode_current_servo(cmd.motor, cmd.servo_us, frame.data);
    }
  else
    {
      ok = vesc_encode_duty_servo(cmd.motor, cmd.servo_us, frame.data);
    }

  /* The encoder refusing means it wrote a zero motor value, which is still
   * safe to send - and sending it is better than skipping, because the VESC
   * would otherwise see a gap it reads as a dropout.
   */

  if (!ok)
    {
      s->tx_clamped++;
    }

  s->last_motor = cmd.motor;
  s->last_servo_us = cmd.servo_us;

  if (fdcan_transmit(&frame) < 0)
    {
      s->tx_errors++;
      return;
    }

  s->tx_sent++;
}

static int vesc_daemon(int argc, FAR char *argv[])
{
  struct vesc_daemon_status_s status;
  int pub = -1;
  int sub = -1;
  int rc_sub = -1;
  int result = EXIT_FAILURE;
  bool can_ready = false;
  uint64_t next_tx_us;
  uint64_t tx_period_us;
  int ret;

  memset(&status, 0, sizeof(status));
  status.bitrate = (uint32_t)param_i32("VESC_BITRATE");
  status.filter_id = (uint8_t)param_i32("VESC_CAN_ID");
  status.tx_rate = (uint32_t)param_i32("VESC_TX_RATE");
  status.tlm_timeout_ms = (uint32_t)param_i32("VESC_TLM_TO_MS");
  status.cmd_timeout_ms = (uint32_t)param_i32("VESC_CMD_TO_MS");
  status.rc_timeout_ms = (uint32_t)param_i32("RC_INPUT_TO_MS");
  g_cmd_timeout_ms = status.cmd_timeout_ms;
  status.limits.cur_max = param_f32("VESC_CUR_MAX");
  status.limits.duty_max = param_f32("VESC_DUTY_MAX");
  status.limits.steer_min = (uint16_t)param_i32("VESC_STEER_MIN");
  status.limits.steer_trim = (uint16_t)param_i32("VESC_STEER_TRIM");
  status.limits.steer_max = (uint16_t)param_i32("VESC_STEER_MAX");
  status.limits.steer_offset = (int16_t)param_i32("VESC_STEER_OFS");
  vesc_speed_init(&g_speed, (float)param_i32("VESC_TLM_HZ"),
                  param_f32("VESC_SPD_LPF"));

  ret = fdcan_init(status.bitrate);

  if (ret < 0)
    {
      syslog(LOG_ERR, "[vesc] FDCAN1 init failed: %d%s\n", -ret,
             ret == -ENOTSUP ? " (only 1000000 bit/s is implemented)" : "");
      goto out;
    }

  can_ready = true;

  /* fdcan_init leaves the filter accept-any. Narrowing it is a separate
   * call so discovery and normal operation take the same path.
   */

  ret = fdcan_set_filter(status.filter_id);

  if (ret < 0)
    {
      syslog(LOG_ERR, "[vesc] cannot set FDCAN1 filter (%d)\n", -ret);
      goto out;
    }

  pub = vesc_status_advertise();

  if (pub < 0)
    {
      syslog(LOG_ERR, "[vesc] cannot advertise vesc_status (%d)\n", errno);
      goto out;
    }

  sub = orb_subscribe(ORB_ID(actuator_command));

  if (sub < 0)
    {
      syslog(LOG_ERR, "[vesc] cannot subscribe actuator_command (%d)\n",
             errno);
      goto out;
    }

  rc_sub = orb_subscribe(ORB_ID(rc_in));

  if (rc_sub < 0)
    {
      syslog(LOG_ERR, "[vesc] cannot subscribe rc_in (%d)\n", errno);
      goto out;
    }

  if (param_i32("RC_MAP_ARM") == VESC_RC_TRIM_CHANNEL)
    {
      syslog(LOG_WARNING,
             "[vesc] RC channel 7 is both steering trim and RC_MAP_ARM\n");
    }

  tx_period_us = 1000000ull / status.tx_rate;
  next_tx_us = vesc_now_us();

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

      /* Drain the complete software ring after every wake. Taking only one
       * frame would leave the remainder queued until another interrupt or
       * transmit deadline, adding latency and eventually overflowing it.
       */

      while (drained++ < VESC_DRAIN_MAX && fdcan_receive(&frame) == OK)
        {
          vesc_handle(&frame, pub, &status);
        }

      vesc_telemetry_watchdog(&status);
      vesc_take_setpoints(sub, &status);
      vesc_take_rc(rc_sub, &status);

      /* Transmit only once the controller id is known. VESC_CAN_ID at 0 is
       * discovery mode, and there the id would be a guess - commanding a
       * node picked at random is worse than not commanding at all.
       */

      if (status.filter_id != 0 && vesc_now_us() >= next_tx_us)
        {
          next_tx_us += tx_period_us;

          /* If the loop fell behind, resynchronise rather than trying to
           * catch up: a burst of stale commands is worse than a late one.
           */

          if (next_tx_us <= vesc_now_us())
            {
              next_tx_us = vesc_now_us() + tx_period_us;
            }

          vesc_transmit(&status);
        }

      fdcan_stats(&status.bus);
      status_publish(&status);

      /* RX wakes this task immediately. With a configured controller the
       * timeout is the time left to the transmit deadline; discovery mode
       * uses only a bounded stop/status watchdog.
       */

      if (status.filter_id != 0)
        {
          uint64_t now = vesc_now_us();
          uint64_t wait_us = next_tx_us > now ? next_tx_us - now : 1u;

          if (wait_us > UINT32_MAX)
            {
              wait_us = UINT32_MAX;
            }

          fdcan_wait((uint32_t)wait_us);
        }
      else
        {
          fdcan_wait(VESC_IDLE_WAIT_US);
        }
    }

  result = EXIT_SUCCESS;

out:
  status.running = false;
  status_publish(&status);

  if (sub >= 0)
    {
      orb_unsubscribe(sub);
    }

  if (rc_sub >= 0)
    {
      orb_unsubscribe(rc_sub);
    }

  if (pub >= 0)
    {
      orb_unadvertise(pub);
    }

  if (can_ready)
    {
      fdcan_deinit();
    }

  g_armed = false;
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
  g_armed = false;
  g_last_tlm_us = 0;
  g_tlm_lost = false;
  memset(&g_setpoint, 0, sizeof(g_setpoint));
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

int vesc_arm(bool armed)
{
  uint64_t now;
  uint64_t age = 0;

  if (!g_running)
    {
      return -ESRCH;
    }

  if (!armed)
    {
      /* Disarming always succeeds. A refusal path here would be a way to
       * fail to make the vehicle safe.
       */

      g_armed = false;
      return 0;
    }

  /* Refuse to arm into a link that is not reporting. This is the same
   * condition the watchdog disarms on, and allowing it back in through the
   * arm command would make the watchdog a suggestion.
   */

  if (g_tlm_lost)
    {
      return -ENOLINK;
    }

  now = vesc_now_us();

  if (g_setpoint.valid && now > g_setpoint.stamp_us)
    {
      age = now - g_setpoint.stamp_us;
    }

  if (!vesc_cmd_may_arm(g_setpoint.valid, g_setpoint.motor, age,
                        g_cmd_timeout_ms))
    {
      return -EPERM;
    }

  g_armed = true;
  return 0;
}

void vesc_status(FAR struct vesc_daemon_status_s *out)
{
  pthread_mutex_lock(&g_lock);
  *out = g_status;
  pthread_mutex_unlock(&g_lock);
}
