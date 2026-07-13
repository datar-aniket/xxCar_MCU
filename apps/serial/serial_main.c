/****************************************************************************
 * apps/serial/serial_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `ser` - inspect and drive the serial port manager.
 *
 *   ser status            what each connector is set to, and what it maps to
 *   ser start             apply the SER_* parameters now
 *   ser nsh <port|dev>    open a shell right now (does not persist). Takes a
 *                         port name or a device path:
 *                           ser nsh USB
 *                           ser nsh /dev/ttyACM0
 *                           ser nsh TELEM2
 *
 * The USB port only exists while a host is attached, so a shell on it waits for
 * the cable and re-arms when it is pulled.
 *
 * TELEM1 always keeps a shell whatever the parameters say - NSH is the init
 * entrypoint and owns /dev/console. So a params.txt with every port disabled
 * still boots normally and still lets you in.
 *
 * Moving the shell permanently is a parameter change like any other:
 *
 *   param set SER_TEL2_FUNC 1        (1 = NSH)
 *   param set SER_TEL2_BAUD 115200
 *   param save
 *   reboot
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <strings.h>

#include "serial.h"
#include "../param/param.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR const char *serial_funcname(int32_t func)
{
  switch (func)
    {
      case SER_FUNC_DISABLED: return "-";
      case SER_FUNC_NSH:      return "NSH";
      case SER_FUNC_MAVLINK:  return "MAVLink";
      case SER_FUNC_GPS:      return "GPS";
      case SER_FUNC_RC_IN:    return "RC";
      default:                return "?";
    }
}

static void serial_usage(void)
{
  printf("Usage: ser <command>\n"
         "  status           show what each connector is set to\n"
         "  start            apply the SER_* parameters now\n"
         "  nsh <port|dev>   open a shell now, not persistent. Takes a port\n"
         "                   name or a device: 'ser nsh USB', 'ser nsh TELEM2',\n"
         "                   'ser nsh /dev/ttyACM0'\n"
         "\n"
         "To move the shell permanently:\n"
         "  param set SER_TEL2_FUNC 1   (0=off 1=NSH 2=MAVLink 3=GPS 4=RC)\n"
         "  param save\n"
         "  reboot\n");
}

static int serial_do_status(void)
{
  FAR const struct serial_port_s *ports = serial_ports();
  int n = serial_port_count();
  int i;

  printf("%-8s %-13s %-8s %-8s %-9s %s\n",
         "PORT", "DEVICE", "BUS", "FUNCTION", "BAUD", "");

  for (i = 0; i < n; i++)
    {
      int32_t func = param_i32(ports[i].func_param);
      char baud[12];

      /* USB has no baud: the host owns the line coding and the device ignores
       * it. Printing a number there would just be a lie.
       */

      if (ports[i].baud_param != NULL)
        {
          snprintf(baud, sizeof(baud), "%" PRId32,
                   param_i32(ports[i].baud_param));
        }
      else
        {
          strlcpy(baud, "host", sizeof(baud));
        }

      printf("%-8s %-13s %-8s %-8s %-9s %s\n",
             ports[i].name, ports[i].devpath, ports[i].uart,
             serial_funcname(func), baud,
             ports[i].is_console ? "(syslog console)" :
             ports[i].removable  ? "(needs a host attached)" : "");
    }

  /* These two are not assignable, and saying so is more useful than leaving
   * someone to wonder why RC IN is missing from the table.
   */

  printf("\n"
         "RC IN and the 8 PWM rails are NOT FMU ports - they belong to the\n"
         "PX4IO co-processor, reached over USART6 (/dev/ttyS4). See `px4io`.\n");

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  if (argc < 2)
    {
      serial_usage();
      return 1;
    }

  if (strcmp(argv[1], "status") == 0)
    {
      return serial_do_status();
    }

  if (strcmp(argv[1], "start") == 0)
    {
      serial_manager_start();
      return 0;
    }

  if (strcmp(argv[1], "nsh") == 0 && argc == 3)
    {
      int ret;

      /* Accept either a connector name ("USB", "telem2") or a raw device path
       * ("/dev/ttyACM0"), because both are natural things to type.
       */

      FAR const struct serial_port_s *ports = serial_ports();
      int port = -1;
      int i;

      /* Resolve a connector name ("USB", "telem2") OR a device path
       * ("/dev/ttyACM0") to the same port, so that either spelling goes through
       * the one-shell-per-port guard. Two shells on one tty split the input
       * between them and the port stops responding.
       */

      for (i = 0; i < serial_port_count(); i++)
        {
          if (strcasecmp(ports[i].name, argv[2]) == 0 ||
              strcmp(ports[i].devpath, argv[2]) == 0)
            {
              port = i;
              break;
            }
        }

      if (port >= 0)
        {
          ret = serial_start_nsh(port);

          if (ret == -EALREADY)
            {
              fprintf(stderr, "ser: %s already has a shell\n", ports[port].name);
              return 1;
            }

          if (ret < 0)
            {
              fprintf(stderr, "ser: cannot start NSH on %s: %d\n",
                      argv[2], ret);
              return 1;
            }

          printf("ser: NSH started on %s (%s)%s\n",
                 ports[port].name, ports[port].devpath,
                 ports[port].removable ? " (waits for a host to attach)" : "");
          return 0;
        }

      /* Not one of ours - but a device path is still allowed, so a shell can be
       * put on anything that behaves like a tty.
       */

      if (argv[2][0] == '/')
        {
          bool removable = (strstr(argv[2], "ACM") != NULL);

          ret = serial_start_nsh_dev(argv[2], removable);
          if (ret < 0)
            {
              fprintf(stderr, "ser: cannot start NSH on %s: %d\n",
                      argv[2], ret);
              return 1;
            }

          printf("ser: NSH started on %s%s\n", argv[2],
                 removable ? " (waits for a host to attach)" : "");
          return 0;
        }

      fprintf(stderr, "ser: unknown port '%s'\n", argv[2]);
      fprintf(stderr, "  try: ser status\n");
      return 1;
    }

  serial_usage();
  return 1;
}
