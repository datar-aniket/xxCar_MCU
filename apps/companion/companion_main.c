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

/* Mirrors enum fmuv6c_pps_state_e without including the board header: this
 * file only ever prints the value.
 */

static FAR const char *comp_pps_state_name(uint8_t state)
{
  switch (state)
    {
      case 0:  return "no-signal";
      case 1:  return "acquiring";
      case 2:  return "LOCKED";
      case 3:  return "holdover";
      default: return "unknown";
    }
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

  if (s.utc_from_rtc)
    {
      /* Second resolution: the RTC restores the right SECOND across a
       * reboot, not the right microsecond within it.
       */

      printf("  clock      UTC from the RTC (+/- 1 s) - sync for the "
             "sub-second phase\n");
      printf("             (HSE-clocked: survives a reboot, not a power "
             "cycle)\n");
    }

  if (s.timesync_replies > 0 || s.timesync_expected > 0)
    {
      printf("  timesync   replies %" PRIu32 "/%" PRIu32 "\n",
             s.timesync_replies, s.timesync_expected);

      /* The offset the COMPANION settled on, reported back by it. The board
       * cannot derive this itself - only the companion sees all four
       * timestamps - so without the END packet the board has no idea
       * whether its peer thinks the clocks are aligned.
       */

      if (s.timesync_synced)
        {
          printf("             UTC offset %+.3f ms  trip %.3f ms  "
                 "samples %" PRIu32 "%s\n",
                 (double)s.timesync_offset_us / 1000.0,
                 (double)s.timesync_trip_us / 1000.0,
                 s.timesync_samples,
                 s.wall_clock_set ? "  wall clock set" : "");
        }

      /* A UTC stamp that arrived before a sync could not be converted to the
       * board's monotonic clock, so it was arrival-stamped instead. Worth
       * naming: it means poses were sent before the sync completed.
       */

      if (s.rx_unsynced_stamp > 0)
        {
          printf("             %" PRIu32 " pose(s) stamped before the sync\n",
                 s.rx_unsynced_stamp);
        }
    }

  printf("  estimator  states %" PRIu32 "  nothing-new %" PRIu32 "%s\n",
         s.est_seen, s.tx_no_state,
         s.est_seen == 0 ? "   <- ekf3 is not publishing" : "");

  printf("  tick       TIM6 %" PRIu32 " raised  %" PRIu32 " missed%s\n",
         s.tick_ticks, s.tick_missed,
         s.tick_missed > 0 ? "   <- downlink is behind the tick" : "");

  /* The tick free-runs against the estimator, so the two slip. A repeat is a
   * tick that found the same solution as the last; the gap is how far the
   * sample time moved between the solutions actually sent. At 200 Hz against
   * a 400 Hz estimator, expect a gap near 5000 us straddled by one estimator
   * period either side - that spread IS the slip.
   */

  printf("  downlink   repeats %" PRIu32 "  sample gap %" PRIu32 "-%" PRIu32
         " us\n", s.tx_repeat, s.tx_gap_min_us, s.tx_gap_max_us);

  printf("  pps        %s  corrections %" PRIu32 "  last residual %+"
         PRIi32 " us\n",
         comp_pps_state_name(s.pps_state), s.pps_corrections,
         s.pps_residual_us);

  if (s.pps_corrections == 0)
    {
      printf("             not disciplining: needs PPS locked AND UTC "
             "already known\n");
    }

  /* A solution cannot be newer than now. If this is climbing, the UTC offset
   * is running ahead of the companion's clock and the PPS residual above is
   * the measurement of by how much.
   */

  if (s.tx_future_clamped > 0)
    {
      printf("             %" PRIu32 " stamp(s) clamped back from the "
             "future\n", s.tx_future_clamped);
    }

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
