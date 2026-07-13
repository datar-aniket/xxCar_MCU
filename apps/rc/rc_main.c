/****************************************************************************
 * apps/rc/rc_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `rc` - RC receiver on a plain FMU serial port.
 *
 *   rc status            what it locked on to, and the live channels
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
 * A receiver in the RC IN connector is NOT this - that connector belongs to the
 * PX4IO co-processor. Use `px4io rc` for it. Both publish the same `rc_in` uORB
 * topic, so anything downstream sees one RC source either way:
 *
 *   uorb_listener rc_in
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "rc.h"
#include "../serial/serial.h"
#include "../param/param.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void rc_usage(void)
{
  printf("Usage: rc <command>\n"
         "  status           what it detected, and the live channels\n"
         "  start <port>     start on a port now (e.g. TELEM2)\n"
         "  stop\n"
         "\n"
         "Normally the serial manager starts this for you:\n"
         "  param set SER_TEL2_FUNC 4   (4 = RC)\n"
         "  param set RC_PROT 0         (0 = auto, 1 = SBUS, 2 = CRSF)\n"
         "  param save && reboot\n"
         "\n"
         "A receiver in the RC IN connector belongs to PX4IO, not to this -\n"
         "use `px4io rc`. Both publish the same 'rc_in' uORB topic.\n");
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

static int rc_do_status(void)
{
  struct rc_status_s s;
  unsigned i;

  rc_get_status(&s);

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
