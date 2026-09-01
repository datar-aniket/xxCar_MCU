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
          printf("             corrected UTC = a*TIM5+b: a-1 %+.3f ppm "
                 "(base %+.3f), phase error %+.3f ms\n",
                 (double)s.utc_rate_ppb / 1000.0,
                 (double)s.utc_base_rate_ppb / 1000.0,
                 (double)s.timesync_phase_error_us / 1000.0);
          printf("             updates %" PRIu32 "  rate/step resets %"
                 PRIu32 "  CLOCK_REALTIME is not used for conversion\n",
                 s.timesync_updates, s.timesync_rate_rejected);
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

  /* The autonomous command input.
   *
   * Shown whenever anything has arrived, and silent otherwise, so a vehicle
   * driven only from the transmitter does not carry a line of zeroes. The
   * age of the last accepted command against the budget is the number that
   * says whether the link is fast enough, and it is measured entirely in the
   * board's TIM5 clock at both ends.
   */

  if (s.rx_direct > 0 || s.rx_direct_stale > 0 || s.rx_direct_invalid > 0)
    {
      printf("  direct     accepted %" PRIu32 "  stale %" PRIu32
             "  invalid %" PRIu32 "\n",
             s.rx_direct, s.rx_direct_stale, s.rx_direct_invalid);

      if (s.rx_direct > 0)
        {
          printf("             last age %.1f ms of %.0f ms budget "
                 "(AUTO_CMD_TO_MS)\n",
                 (double)s.last_direct_age_us / 1000.0,
                 (double)s.auto_timeout_us / 1000.0);
        }

      /* Accepted here means published, not obeyed. The control router still
       * requires the AUTO source switch and its own arm sequence, and says
       * so itself in `control_router status`.
       */

      if (s.rx_direct_stale > 0 && !s.timesync_synced && !s.utc_from_rtc)
        {
          printf("             (no UTC reference yet - run the timesync)\n");
        }
    }

  if (s.rx_trajectory > 0 || s.rx_trajectory_stale > 0 ||
      s.rx_trajectory_invalid > 0)
    {
      printf("  trajectory accepted %" PRIu32 "  stale %" PRIu32
             "  invalid %" PRIu32 "\n",
             s.rx_trajectory, s.rx_trajectory_stale,
             s.rx_trajectory_invalid);

      if (s.rx_trajectory > 0)
        {
          printf("             last horizon %u  dt %.4f s\n",
                 s.last_trajectory_horizon,
                 (double)s.last_trajectory_dt);
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

  /* TIM5 against CLOCK_MONOTONIC. Corrected UTC is based only on the affine
   * TIM5 model; this remains a diagnostic and never enters that conversion.
   */

  printf("  clocks     TIM5 - MONOTONIC %+" PRIi64 " us\n",
         s.clock_skew_us);

  if (s.pps_phase_valid)
    {
      printf("  pps        phase %+.1f ms from the UTC second, drift %+"
             PRIi32 " us\n",
             (double)s.pps_phase_ref_us / 1000.0, s.pps_drift_us);

      /* A large phase offset is NORMAL for a free-running 1 Hz output and
       * is not corrected - saying so stops it being read as an error.
       */

      if (!s.pps_absolute_phase && labs((long)s.pps_phase_ref_us) > 1000)
        {
          printf("             timesync and the pulse disagree by that much,"
                 " and one of them is wrong.\n"
                 "             If the companion's OWN clock is disciplined to"
                 " this same pulse, then\n"
                 "             the pulse defines the second and timesync is"
                 " the weaker measurement -\n"
                 "             set PPS_ABS_PHASE=1 to trust the pulse"
                 " instead. If not, leave it: the\n"
                 "             offset is the pulse's own phase and is"
                 " correctly being ignored.\n");
        }

      if (s.pps_absolute_phase)
        {
          printf("             PPS_ABS_PHASE=1: the pulse is treated as the"
                 " true second.\n");
        }
    }

  if (s.pps_rejected > 0)
    {
      printf("  pps        REFUSED %" PRIu32 " correction(s); worst drift %+"
             PRIi32 " us\n"
             "             drift that large is not drift - something moved "
             "the clock, or the\n"
             "             pulse is not the one the phase was established "
             "against.\n",
             s.pps_rejected, s.pps_worst_us);
    }

  printf("  pps        %s  corrections %" PRIu32 "  last residual %+"
         PRIi32 " us\n",
         comp_pps_state_name(s.pps_state), s.pps_corrections,
         s.pps_residual_us);

  if (s.pps_corrections == 0)
    {
      printf("             not disciplining: needs PPS locked AND UTC "
             "already known\n");
    }

  /* A solution cannot be newer than now. If this is climbing, the affine UTC
   * model is running ahead of the companion's clock and the PPS residual
   * above is the measurement of by how much.
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
