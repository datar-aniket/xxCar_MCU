/****************************************************************************
 * apps/vesc/vesc_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `vesc start | stop | status`
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>

#include <uORB/uORB.h>

#include "vesc.h"
#include "../param/param.h"
#include "../uorb_msgs/uorb_msgs.h"

static void usage(void)
{
  printf("Usage: vesc start | stop | status | arm | disarm\n"
         "       vesc set duty|current <motor> <steering> [seconds]\n"
         "\n"
         "  Receives VESC telemetry on FDCAN1 and publishes vesc_status.\n"
         "  Commands the motor and steering from routed actuator_command.\n"
         "\n"
         "  <motor>     duty ratio -1..+1, or amps, per mode\n"
         "  <steering>  normalised -1..+1, POSITIVE IS LEFT\n"
         "  [seconds]   how long to keep publishing; default 2, max 30\n"
         "  'set' publishes control_cmd and therefore requires the router\n"
         "  in AUTO, a healthy RC link, and its arm switch high.\n"
         "\n"
         "  Transmit needs VESC_CAN_ID set to the real node id. At 0 the\n"
         "  link is in discovery mode and sends nothing.\n"
         "\n"
         "  VESC_CAN_ID     0 accepts any controller id (discovery)\n"
         "  VESC_BITRATE    bus bitrate; only 1000000 is implemented\n"
         "  VESC_EN         start at boot\n"
         "  VESC_TLM_TO_MS  telemetry dropout before disarm\n"
         "  VESC_TX_RATE    command frame rate, Hz\n"
         "  VESC_CMD_TO_MS  setpoint age before failsafe neutral\n"
         "  VESC_CUR_MAX    current ceiling, A\n"
         "  VESC_DUTY_MAX   duty ceiling, 0-1\n"
         "  VESC_STEER_*    MIN / TRIM / MAX servo pulse, us\n");
}

static FAR const char *vesc_packet_name(uint8_t id)
{
  switch (id)
    {
      case VESC_PACKET_SET_DUTY:          return "SET_DUTY";
      case VESC_PACKET_SET_CURRENT:       return "SET_CURRENT";
      case VESC_PACKET_PROCESS_SHORT_BUF: return "PROCESS_SHORT_BUFFER";
      case VESC_PACKET_STATUS_5:          return "STATUS_5";
      case VESC_PACKET_SET_CURRENT_SERVO: return "SET_CURRENT_SERVO";
      case VESC_PACKET_SET_DUTY_SERVO:    return "SET_DUTY_SERVO";
      default:                            return "unknown";
    }
}

static void print_status(void)
{
  struct vesc_daemon_status_s s;
  int i;

  vesc_status(&s);

  if (!s.running)
    {
      printf("vesc: stopped\n");
      return;
    }

  printf("vesc: running on FDCAN1 at %" PRIu32 " bit/s, filter %s\n",
         s.bitrate,
         s.filter_id == 0 ? "accept-any" : "one id");

  printf("  bus     rx %" PRIu32 "  hw_lost %" PRIu32
         "  ring_lost %" PRIu32 "  rejected %" PRIu32 "  state %s\n",
         s.bus.rx, s.bus.lost, s.bus.ring_full, s.bus.rejected,
         s.bus.bus_off ? "BUS_OFF" :
         s.bus.error_passive ? "ERROR_PASSIVE" : "ERROR_ACTIVE");

  /* Nothing at all is a distinct condition from frames arriving badly, and
   * saying so beats printing a screen of zeros and leaving the reader to
   * work out which.
   */

  if (s.bus.rx == 0)
    {
      printf("  seen    NOTHING - check wiring, termination and bitrate\n");
    }

  for (i = 0; i < s.nseen; i++)
    {
      FAR const struct vesc_seen_s *e = &s.seen[i];
      double span = (double)(e->last_us - e->first_us) / 1000000.0;

      printf("  seen    packet 0x%02x %-20s id 0x%02x  %" PRIu32
             " frames  %.1f Hz\n",
             e->packet_id, vesc_packet_name(e->packet_id),
             e->controller_id, e->count,
             span > 0.0 ? (double)(e->count - 1) / span : 0.0);
    }

  if (s.decoded > 0)
    {
      printf("  status5 tach %" PRIi32 "  current %.2f A  adc %.3f V\n",
             s.last.tachometer, (double)s.last.current_a,
             (double)s.last.adc_volts);
    }

  printf("  decoded %" PRIu32 "  bad_dlc %" PRIu32 "  publish_err %" PRIu32
         "\n", s.decoded, s.bad_dlc, s.publish_errors);

  printf("  tx      %s  sent %" PRIu32 "  errors %" PRIu32
         "  clamped %" PRIu32 "\n",
         s.armed ? "ARMED" : "disarmed",
         s.tx_sent, s.tx_errors, s.tx_clamped);

  if (s.filter_id == 0)
    {
      printf("  tx      DISABLED - VESC_CAN_ID is 0 (discovery mode)\n");
    }

  printf("  tx      fifo_full %" PRIu32 "  last %s  motor %.3f  servo %u us"
         "\n", s.bus.tx_full, vesc_cmd_reason_name(s.last_reason),
         (double)s.last_motor, (unsigned)s.last_servo_us);

  printf("  telemetry %s  timeout %" PRIu32 " ms  watchdog disarms %"
         PRIu32 "%s\n",
         s.tlm_lost ? "LOST" : "ok", s.tlm_timeout_ms, s.tlm_disarms,
         s.tlm_timeout_ms == 0 ? "   (watchdog DISABLED)" : "");

  printf("  reasons armed %" PRIu32 "  disarmed %" PRIu32
         "  no-setpoint %" PRIu32 "  stale %" PRIu32 "  bad-mode %" PRIu32
         "\n",
         s.reason_count[VESC_CMD_ARMED],
         s.reason_count[VESC_CMD_DISARMED],
         s.reason_count[VESC_CMD_NO_SETPOINT],
         s.reason_count[VESC_CMD_STALE],
         s.reason_count[VESC_CMD_BAD_MODE]);

  printf("  setpts  %" PRIu32 "  limits cur %.1f A  duty %.2f  "
         "steer %u/%u/%u offset %d us\n",
         s.setpoints, (double)s.limits.cur_max, (double)s.limits.duty_max,
         (unsigned)s.limits.steer_min, (unsigned)s.limits.steer_trim,
         (unsigned)s.limits.steer_max, (int)s.limits.steer_offset);

  printf("  trim    RC7 %u us -> %+d us%s  effective offset %+d us\n",
         (unsigned)s.rc_trim_pwm, (int)s.rc_trim_us,
         s.rc_trim_active ? "" : " (inactive)",
         (int)s.limits.steer_offset + (int)s.rc_trim_us);
}

/* Publish a setpoint at the daemon's rate for a bounded time, then stop.
 *
 * It publishes control_cmd, the control router's AUTONOMOUS INPUT, and not
 * actuator_command. That distinction is the whole point: actuator_command is
 * the router's output and the router rewrites it every loop, so a second
 * publisher there would be overwritten within milliseconds and would be
 * writing to the safety boundary's output while it was at it. Going in
 * through control_cmd means the router's arm gate, RC override and staleness
 * policy all still apply to a bench command.
 *
 * Bounded on purpose. A one-shot publish goes stale and could never sweep a
 * servo; an unbounded one would leave a vehicle driving after the command
 * returned. Stopping also demonstrates the failsafe, which is the behaviour
 * most worth seeing on a bench.
 */

static int do_set(int argc, FAR char *argv[])
{
  struct control_cmd_s message;
  int32_t rate;
  double seconds = 2.0;
  int pub;
  int period_us;
  int ticks;
  int i;

  if (argc < 5)
    {
      usage();
      return EXIT_FAILURE;
    }

  memset(&message, 0, sizeof(message));

  if (strcmp(argv[2], "duty") == 0)
    {
      message.mode = ACTUATOR_MODE_DUTY;
    }
  else if (strcmp(argv[2], "current") == 0)
    {
      message.mode = ACTUATOR_MODE_CURRENT;
    }
  else
    {
      printf("vesc: mode must be duty or current\n");
      return EXIT_FAILURE;
    }

  message.motor = strtof(argv[3], NULL);
  message.steering = strtof(argv[4], NULL);

  if (argc >= 6)
    {
      seconds = strtod(argv[5], NULL);
    }

  if (!(seconds > 0.0 && seconds <= 30.0))
    {
      printf("vesc: seconds must be in 0..30\n");
      return EXIT_FAILURE;
    }

  if (!isfinite(message.motor) || !isfinite(message.steering))
    {
      printf("vesc: motor and steering must be finite\n");
      return EXIT_FAILURE;
    }

  rate = param_i32("VESC_TX_RATE");

  if (rate < 1)
    {
      rate = 50;
    }

  pub = control_cmd_advertise();

  if (pub < 0)
    {
      printf("vesc: cannot advertise control_cmd (%d)\n", errno);
      return EXIT_FAILURE;
    }

  period_us = 1000000 / (int)rate;
  ticks = (int)(seconds * (double)rate);

  printf("vesc: publishing control_cmd %s %.3f steering %.3f for %.1f s\n"
         "      the control router must be armed for this to reach the "
         "motor\n",
         argv[2], (double)message.motor, (double)message.steering, seconds);

  for (i = 0; i < ticks; i++)
    {
      message.timestamp = 0;    /* the daemon stamps it on arrival */
      control_cmd_publish(pub, &message);
      usleep(period_us);
    }

  orb_unadvertise(pub);
  printf("vesc: stopped publishing; failsafe takes over\n");
  return EXIT_SUCCESS;
}

int main(int argc, FAR char *argv[])
{
  int ret;

  if (argc < 2)
    {
      usage();
      return EXIT_FAILURE;
    }

  if (strcmp(argv[1], "start") == 0)
    {
      ret = vesc_start();

      if (ret == -EALREADY)
        {
          printf("vesc: already running\n");
          return EXIT_FAILURE;
        }

      if (ret < 0)
        {
          printf("vesc: failed to start (%d) - check the syslog\n", ret);
          return EXIT_FAILURE;
        }

      print_status();
      return EXIT_SUCCESS;
    }

  if (strcmp(argv[1], "stop") == 0)
    {
      ret = vesc_stop();

      if (ret == -ESRCH)
        {
          printf("vesc: not running\n");
          return EXIT_FAILURE;
        }

      printf("vesc: %s\n", ret == OK ? "stopped" : "did not stop");
      return ret == OK ? EXIT_SUCCESS : EXIT_FAILURE;
    }

  if (strcmp(argv[1], "status") == 0)
    {
      print_status();
      return EXIT_SUCCESS;
    }

  if (strcmp(argv[1], "arm") == 0 || strcmp(argv[1], "disarm") == 0)
    {
      bool want = strcmp(argv[1], "arm") == 0;

      ret = vesc_arm(want);

      if (ret == -ESRCH)
        {
          printf("vesc: not running\n");
          return EXIT_FAILURE;
        }

      if (ret == -ENOLINK)
        {
          printf("vesc: refused - telemetry is not arriving. The watchdog\n"
                 "      disarms on this, so arming through it would make the"
                 " watchdog\n      a suggestion. Check the CAN link.\n");
          return EXIT_FAILURE;
        }

      if (ret == -EPERM)
        {
          printf("vesc: refused - a live setpoint is commanding the motor.\n"
                 "      Stop the publisher, or wait %" PRIi32 " ms for it to"
                 " go stale.\n", param_i32("VESC_CMD_TO_MS"));
          return EXIT_FAILURE;
        }

      printf("vesc: %s\n", want ? "ARMED" : "disarmed");
      return EXIT_SUCCESS;
    }

  if (strcmp(argv[1], "set") == 0)
    {
      return do_set(argc, argv);
    }

  usage();
  return EXIT_FAILURE;
}
