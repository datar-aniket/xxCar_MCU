/****************************************************************************
 * apps/companion/companion_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `companion start | stop | status`
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "companion.h"
#include "../param/param.h"

static void usage(void)
{
  printf("Usage: companion start | stop | status\n"
         "\n"
         "  Links the companion computer over the port whose SER_*_FUNC is\n"
         "  %d. Routes framed packets to uORB topics by message id and\n"
         "  sends the estimator pose back at EXT_TX_RATE Hz.\n"
         "\n"
         "  The port must be reserved BEFORE boot - a shell started on it\n"
         "  at boot outlives the parameter change:\n"
         "    param set SER_TEL2_FUNC %d\n"
         "    param save\n"
         "    reboot\n", SER_FUNC_COMPANION, SER_FUNC_COMPANION);
}

static void print_status(void)
{
  struct companion_status_s s;

  companion_status(&s);

  if (!s.running)
    {
      printf("companion: stopped\n");
      return;
    }

  if (s.waiting_for_host)
    {
      printf("companion: on %s, WAITING for a host to attach\n", s.port);
      return;
    }

  if (s.baud > 0)
    {
      printf("companion: running on %s at %" PRIu32 " baud\n",
             s.port, s.baud);
    }
  else
    {
      printf("companion: running on %s (the host sets the line coding)\n",
             s.port);
    }
  printf("  in   %" PRIu64 " bytes  frames %" PRIu32
         "  pose %" PRIu32 "\n",
         s.bytes_in, s.parser.frames, s.rx_pose);
  printf("  out  %" PRIu64 " bytes  frames %" PRIu32
         "  at %" PRIu32 " Hz  errors %" PRIu32 "\n",
         s.bytes_out, s.tx_frames, s.tx_rate_hz, s.tx_errors);

  /* Unknown id and bad length are shown apart deliberately: the first is a
   * companion newer than this firmware and is fine, the second is the two
   * ends disagreeing about a format and is not.
   */

  printf("  rx faults  crc %" PRIu32 "  unknown_id %" PRIu32
         "  bad_length %" PRIu32 "  publish %" PRIu32 "\n",
         s.parser.crc_errors, s.parser.unknown_id, s.parser.bad_length,
         s.rx_publish_errors);

  /* Silence on the companion's screen has two very different causes, and
   * without these there is no way to tell them apart: either the estimator
   * is not producing (est 0) or it is and the port will not take the bytes
   * (tx errors climbing).
   */

  if (s.timesync_replies > 0)
    {
      printf("  timesync   replies %" PRIu32 "\n", s.timesync_replies);
    }

  printf("  estimator  states %" PRIu32 "  nothing-new %" PRIu32 "%s\n",
         s.est_seen, s.tx_no_state,
         s.est_seen == 0 ? "   <- ekf3 is not publishing" : "");

  if (s.connects > 1 || s.disconnects > 0)
    {
      printf("  host       connects %" PRIu32 "  disconnects %" PRIu32 "\n",
             s.connects, s.disconnects);
    }
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
      ret = companion_start();

      if (ret == -EALREADY)
        {
          printf("companion: already running\n");
          return EXIT_FAILURE;
        }

      if (ret < 0)
        {
          printf("companion: failed to start (%d) - check the syslog; no\n"
                 "           reserved port is the usual cause\n", ret);
          return EXIT_FAILURE;
        }

      print_status();
      return EXIT_SUCCESS;
    }

  if (strcmp(argv[1], "stop") == 0)
    {
      ret = companion_stop();

      if (ret == -ESRCH)
        {
          printf("companion: not running\n");
          return EXIT_FAILURE;
        }

      printf("companion: %s\n", ret == OK ? "stopped" : "did not stop");
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
