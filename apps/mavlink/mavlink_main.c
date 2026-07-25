/****************************************************************************
 * apps/mavlink/mavlink_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `mav` - inspect and drive the MAVLink daemon.
 *
 *   mav status           what it is doing, and the frame counters
 *   mav start <port>     start it now (normally the serial manager does this)
 *   mav stop
 *
 * Normally you do not start it by hand. Give the port the MAVLink function and
 * it comes up at boot:
 *
 *   param set SER_TEL2_FUNC 2      (2 = MAVLink)
 *   param set SER_TEL2_BAUD 115200
 *   param save && reboot
 *
 * The MTF-02 lands on uORB:
 *   uorb_listener optical_flow
 *   uorb_listener distance_sensor
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>

#include "mavlink.h"
#include "../serial/serial.h"
#include "../param/param.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void mav_usage(void)
{
  printf("Usage: mav <command>\n"
         "  status           counters and identity\n"
         "  start <port>     start on a port now (e.g. TELEM2)\n"
         "  stop\n"
         "\n"
         "Normally the serial manager starts this for you:\n"
         "  param set SER_TEL2_FUNC 2   (2 = MAVLink)\n"
         "  param save && reboot\n"
         "\n"
         "The MTF-02 lands on uORB: 'uorb_listener optical_flow' /\n"
         "'uorb_listener distance_sensor'.\n");
}

static int mav_do_status(void)
{
  struct mavlink_status_s s;

  mavlink_get_status(&s);

  if (!s.running)
    {
      printf("mavlink: not running\n");
      printf("  give a port the MAVLink function (SER_*_FUNC=2), or "
             "'mav start TELEM2'\n");
      return 1;
    }

  printf("mavlink on %s @ %" PRId32 " baud\n", s.devpath, s.baud);
  printf("  identity     sys %u  comp %u\n", s.sysid, s.compid);
  printf("  rx frames    %" PRIu32 "  (dropped %" PRIu32 ")\n",
         s.rx_frames, s.rx_dropped);
  printf("  tx frames    %" PRIu32 "\n", s.tx_frames);
  printf("  optical flow %" PRIu32 "  -> uORB optical_flow\n", s.flow_msgs);
  printf("  distance     %" PRIu32 "  -> uORB distance_sensor\n", s.dist_msgs);
  printf("  params sent  %" PRIu32 "\n", s.param_tx);

  if (s.peer_seen)
    {
      printf("  peer         sysid %u\n", s.peer_sysid);
    }
  else
    {
      printf("  peer         none heard yet\n");
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

  if (argc < 2)
    {
      mav_usage();
      return 1;
    }

  if (strcmp(argv[1], "status") == 0)
    {
      return mav_do_status();
    }

  if (strcmp(argv[1], "stop") == 0)
    {
      mavlink_stop();
      printf("mavlink: stopped\n");
      return 0;
    }

  if (strcmp(argv[1], "start") == 0 && argc == 3)
    {
      int port = serial_find(argv[2]);
      int32_t baud;

      if (port < 0)
        {
          fprintf(stderr, "mav: unknown port '%s' (try: ser status)\n",
                  argv[2]);
          return 1;
        }

      baud = serial_ports()[port].baud_param ?
             param_i32(serial_ports()[port].baud_param) : 0;

      ret = mavlink_start(serial_ports()[port].devpath, baud);

      if (ret == -EALREADY)
        {
          fprintf(stderr, "mav: already running\n");
          return 1;
        }

      if (ret < 0)
        {
          fprintf(stderr, "mav: start failed: %d\n", ret);
          return 1;
        }

      printf("mav: started on %s (%s)\n",
             serial_ports()[port].name, serial_ports()[port].devpath);
      return 0;
    }

  mav_usage();
  return 1;
}
