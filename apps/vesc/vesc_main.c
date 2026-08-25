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
#include <string.h>

#include "vesc.h"

static void usage(void)
{
  printf("Usage: vesc start | stop | status\n"
         "\n"
         "  Receives VESC telemetry on FDCAN1 and publishes vesc_status.\n"
         "  Receive only - nothing here commands a motor.\n"
         "\n"
         "  VESC_CAN_ID   0 accepts any controller id (discovery)\n"
         "  VESC_BITRATE  bus bitrate; only 1000000 is implemented\n"
         "  VESC_EN       start at boot\n");
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

  printf("  bus     rx %" PRIu32 "  lost %" PRIu32 "  rejected %" PRIu32
         "  state %s\n",
         s.bus.rx, s.bus.lost, s.bus.rejected,
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

  usage();
  return EXIT_FAILURE;
}
