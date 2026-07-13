/****************************************************************************
 * apps/serial/serial_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `ser` - inspect and drive the serial port manager.
 *
 *   ser status            what each connector is set to, and what it maps to
 *   ser start             apply the SER_* parameters now
 *   ser nsh <port>        open a shell on a port right now (does not persist)
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
         "  nsh <port>       open a shell on a port now (not persistent)\n"
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

  printf("%-8s %-11s %-8s %-8s %-9s %s\n",
         "PORT", "DEVICE", "UART", "FUNCTION", "BAUD", "");

  for (i = 0; i < n; i++)
    {
      int32_t func = param_i32(ports[i].func_param);
      int32_t baud = param_i32(ports[i].baud_param);

      printf("%-8s %-11s %-8s %-8s %-9" PRId32 " %s\n",
             ports[i].name, ports[i].devpath, ports[i].uart,
             serial_funcname(func), baud,
             ports[i].is_console ? "(syslog console)" : "");
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
      int port = serial_find(argv[2]);
      int ret;

      if (port < 0)
        {
          fprintf(stderr, "ser: unknown port '%s'\n", argv[2]);
          fprintf(stderr, "  try: ser status\n");
          return 1;
        }

      ret = serial_start_nsh(port);
      if (ret < 0)
        {
          fprintf(stderr, "ser: cannot start NSH on %s: %d\n", argv[2], ret);
          return 1;
        }

      printf("ser: NSH started on %s (%s)\n",
             serial_ports()[port].name, serial_ports()[port].devpath);
      return 0;
    }

  serial_usage();
  return 1;
}
