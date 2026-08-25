/****************************************************************************
 * apps/rc/rc_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `rc` - RC receiver on a plain FMU serial port.
 *
 *   rc status            active system RC source and live channels
 *   rc start <port>      start it now (normally the serial manager does this)
 *   rc stop
 *
 * Normally you do not start it by hand. Give the port the RC function and it
 * comes up at boot:
 *
 *   param set SER_TEL2_FUNC 4     (4 = RC)
 *   param set RC_PROT 0           (0 = auto, 1 = SBUS, 2 = CRSF)
 *   param save
 *   reboot
 *
 * A receiver in the RC IN connector belongs to the PX4IO co-processor. Both
 * drivers publish the same `rc_in` uORB topic, and `rc status` reports whichever
 * source is active. `px4io rc` remains available for low-level IO diagnostics:
 *
 *   uorb_listener rc_in
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

#include <nuttx/uorb.h>
#include <uORB/uORB.h>

#include "rc.h"
#include "../serial/serial.h"
#include "../param/param.h"

#define RC_STATUS_WAIT_MS  250

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void rc_usage(void)
{
  printf("Usage: rc <command>\n"
         "  status           active system RC source and live channels\n"
         "  start <port>     start on a port now (e.g. TELEM2)\n"
         "  stop\n"
         "\n"
         "Normally the serial manager starts this for you:\n"
         "  param set SER_TEL2_FUNC 4   (4 = RC)\n"
         "  param set RC_PROT 0         (0 = auto, 1 = SBUS, 2 = CRSF)\n"
         "  param save && reboot\n"
         "\n"
         "RC IN/PX4IO and FMU UART receivers share the 'rc_in' topic.\n"
         "Use `px4io rc` only for low-level PX4IO diagnostics.\n");
}

static FAR const char *rc_protoname(uint8_t proto)
{
  switch (proto)
    {
      case RC_PROTO_SBUS: return "SBUS";
      case RC_PROTO_CRSF: return "CRSF/ELRS";
      default:            return "-";
    }
}

static FAR const char *rc_source_name(uint8_t source)
{
  switch (source)
    {
      case RC_IN_SRC_PX4IO: return "PX4IO";
      case RC_IN_SRC_SBUS:  return "SBUS";
      case RC_IN_SRC_CRSF:  return "CRSF/ELRS";
      case RC_IN_SRC_PPM:   return "PPM";
      default:              return "unknown";
    }
}

static uint64_t rc_now_us(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static bool rc_topic_status(FAR struct rc_in_s *input, FAR uint64_t *age_us,
                            FAR bool *fresh)
{
  uint64_t now;
  uint64_t timeout_us;
  struct pollfd pfd;
  int sub;
  int ret;

  memset(input, 0, sizeof(*input));
  *age_us = UINT64_MAX;
  *fresh = false;
  sub = orb_subscribe(ORB_ID(rc_in));

  if (sub < 0)
    {
      return false;
    }

  /* PX4IO advertises a normal non-persistent topic. A new subscriber cannot
   * rely on copying a frame published before it opened, so wait for the next
   * 50 Hz PX4IO update. Keep the wait bounded for a disconnected receiver.
   */

  pfd.fd = sub;
  pfd.events = POLLIN;
  pfd.revents = 0;
  ret = poll(&pfd, 1, RC_STATUS_WAIT_MS);

  if (ret > 0 && (pfd.revents & POLLIN) != 0)
    {
      ret = orb_copy(ORB_ID(rc_in), sub, input);
    }
  else
    {
      ret = -1;
    }

  orb_unsubscribe(sub);

  if (ret < 0 || input->timestamp == 0 || input->source == RC_IN_SRC_NONE)
    {
      return false;
    }

  now = rc_now_us();
  if (input->timestamp > now)
    {
      return true;
    }

  *age_us = now - input->timestamp;
  timeout_us = (uint64_t)param_i32("RC_INPUT_TO_MS") * 1000ull;
  *fresh = *age_us <= timeout_us;
  return true;
}

static int rc_do_status(void)
{
  struct rc_status_s s;
  struct rc_in_s input;
  uint64_t age_us;
  bool fresh;
  unsigned i;

  rc_get_status(&s);

  /* `rc_in` is the system-level RC source. PX4IO and the direct UART driver
   * both publish it, so prefer it over the private status of the UART driver.
   */

  if (rc_topic_status(&input, &age_us, &fresh))
    {
      bool valid = fresh && input.ok != 0 && input.failsafe == 0;
      unsigned count = input.count <= RC_IN_MAX_CHANNELS ?
                       input.count : RC_IN_MAX_CHANNELS;

      printf("rc: %s  %s%s%s\n", rc_source_name(input.source),
             valid ? "OK" : "NO SIGNAL",
             fresh ? "" : "  STALE",
             input.failsafe ? "  FAILSAFE" : "");

      if (age_us == UINT64_MAX)
        {
          printf("  age       future timestamp\n");
        }
      else
        {
          printf("  age       %.1f ms\n", (double)age_us / 1000.0);
        }

      printf("  frames    %u\n", input.frames);
      printf("  lost      %u\n", input.lost_frames);
      printf("  rssi      %u/255\n", input.rssi);

      if (input.count > RC_IN_MAX_CHANNELS)
        {
          printf("  channels  invalid count %u (showing first %u)\n",
                 input.count, count);
        }

      for (i = 0; i < count; i++)
        {
          printf("  ch%-2u %4u us\n", i + 1, input.channel[i]);
        }

      if (s.running &&
          (input.source == RC_IN_SRC_SBUS ||
           input.source == RC_IN_SRC_CRSF))
        {
          printf("  decoder errors=%" PRIu32 " timeouts=%" PRIu32 "\n",
                 s.errors, s.timeouts);
        }

      return 0;
    }

  if (!s.running)
    {
      printf("rc: not running\n");
      printf("  give a port the RC function (SER_*_FUNC=4), or 'rc start "
             "TELEM2'\n");
      return 1;
    }

  if (!s.locked)
    {
      printf("rc: probing for SBUS / CRSF - nothing decoded yet\n");
      printf("  is the receiver powered and wired to the port's RX pin?\n");
      return 0;
    }

  printf("rc: %s  %s%s\n", rc_protoname(s.proto),
         s.ok ? "OK" : "NO SIGNAL",
         s.failsafe ? "  FAILSAFE" : "");
  printf("  frames    %" PRIu32 "\n", s.frames);
  printf("  errors    %" PRIu32 "\n", s.errors);
  printf("  timeouts  %" PRIu32 "\n", s.timeouts);

  for (i = 0; i < s.last.count; i++)
    {
      printf("  ch%-2u %4u us\n", i + 1, s.last.channel[i]);
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
      rc_usage();
      return 1;
    }

  if (strcmp(argv[1], "status") == 0)
    {
      return rc_do_status();
    }

  if (strcmp(argv[1], "stop") == 0)
    {
      rc_stop();
      printf("rc: stopped\n");
      return 0;
    }

  if (strcmp(argv[1], "start") == 0 && argc == 3)
    {
      int port = serial_find(argv[2]);

      if (port < 0)
        {
          fprintf(stderr, "rc: unknown port '%s' (try: ser status)\n", argv[2]);
          return 1;
        }

      ret = rc_start(serial_ports()[port].devpath, param_i32("RC_PROT"));

      if (ret == -ENOTSUP)
        {
          /* rc_start() already explained why (PPM on a UART). */

          return 1;
        }

      if (ret < 0)
        {
          fprintf(stderr, "rc: start failed: %d\n", ret);
          return 1;
        }

      printf("rc: started on %s (%s); RC on uORB as 'rc_in'\n",
             serial_ports()[port].name, serial_ports()[port].devpath);
      return 0;
    }

  rc_usage();
  return 1;
}
