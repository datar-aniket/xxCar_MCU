/****************************************************************************
 * apps/serial/serial.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Serial port manager. See serial.h.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <inttypes.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sched.h>
#include <syslog.h>

#include <nshlib/nshlib.h>

#include "serial.h"
#include "../param/param.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SERIAL_NSH_STACK  4096  /* same as the boot console: `ps` measured the
                                 * NSH task using 2136 bytes, and 2048 was not
                                 * enough (it reset the board).
                                 */
#define SERIAL_NSH_PRIO   SCHED_PRIORITY_DEFAULT

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The FMUv6C connector map.
 *
 * Cross-checked against PX4's boards/px4/fmu-v6c. The ttyS numbering is only
 * true because CONFIG_STM32H7_SERIAL_DISABLE_REORDERING=y - see board.h.
 *
 * USART6 is absent on purpose: it is the PX4IO link, not an assignable port.
 * The RC IN connector is absent for the same reason - it belongs to PX4IO.
 */

static const struct serial_port_s g_ports[] =
{
  { "GPS1",   "/dev/ttyS0", "USART1", "SER_GPS1_FUNC", "SER_GPS1_BAUD", false },
  { "TELEM3", "/dev/ttyS1", "USART2", "SER_TEL3_FUNC", "SER_TEL3_BAUD", false },
  { "DEBUG",  "/dev/ttyS2", "USART3", "SER_DBG_FUNC",  "SER_DBG_BAUD",  false },
  { "TELEM2", "/dev/ttyS3", "UART5",  "SER_TEL2_FUNC", "SER_TEL2_BAUD", false },
  { "TELEM1", "/dev/ttyS5", "UART7",  "SER_TEL1_FUNC", "SER_TEL1_BAUD", true  },
  { "GPS2",   "/dev/ttyS6", "UART8",  "SER_GPS2_FUNC", "SER_GPS2_BAUD", false },
};

#define SERIAL_NPORTS ((int)(sizeof(g_ports) / sizeof(g_ports[0])))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Entry point of an NSH shell bound to a specific UART.
 *
 * argv[1] is the device path. A task starts life with stdin/stdout/stderr
 * pointing at the console it inherited, so to make NSH talk to another UART we
 * open that UART and dup2() it over fds 0, 1 and 2 - NSH itself needs no
 * knowledge of any of this and just reads and writes the standard streams.
 */

static int serial_nsh_task(int argc, FAR char *argv[])
{
  FAR const char *devpath;
  int fd;

  if (argc < 2)
    {
      return EXIT_FAILURE;
    }

  devpath = argv[1];

  fd = open(devpath, O_RDWR);
  if (fd < 0)
    {
      syslog(LOG_ERR, "serial: cannot open %s for NSH: %d\n", devpath, errno);
      return EXIT_FAILURE;
    }

  dup2(fd, 0);
  dup2(fd, 1);
  dup2(fd, 2);

  if (fd > 2)
    {
      close(fd);
    }

  nsh_consolemain(0, NULL);
  return EXIT_SUCCESS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR const struct serial_port_s *serial_ports(void)
{
  return g_ports;
}

int serial_port_count(void)
{
  return SERIAL_NPORTS;
}

int serial_find(FAR const char *name)
{
  int i;

  if (name == NULL)
    {
      return -EINVAL;
    }

  for (i = 0; i < SERIAL_NPORTS; i++)
    {
      if (strcasecmp(g_ports[i].name, name) == 0)
        {
          return i;
        }
    }

  return -ENOENT;
}

int serial_set_baud(FAR const char *devpath, int baud)
{
  struct termios tio;
  int fd;
  int ret = OK;

  fd = open(devpath, O_RDWR | O_NOCTTY);
  if (fd < 0)
    {
      return -errno;
    }

  if (tcgetattr(fd, &tio) < 0)
    {
      ret = -errno;
      goto out;
    }

  /* NuttX's termios takes the B-code, not the raw number, and the H7 driver
   * turns it back into a rate with cfgetispeed(). cfsetspeed() accepts either
   * form, so passing the plain integer works and keeps arbitrary rates (like
   * the 1.5 Mbaud IO link) expressible.
   */

  cfsetspeed(&tio, baud);

  if (tcsetattr(fd, TCSANOW, &tio) < 0)
    {
      ret = -errno;
    }

out:
  close(fd);
  return ret;
}

int serial_start_nsh(int port)
{
  FAR char *argv[2];
  char name[16];
  int pid;

  if (port < 0 || port >= SERIAL_NPORTS)
    {
      return -EINVAL;
    }

  snprintf(name, sizeof(name), "nsh_%s", g_ports[port].name);

  argv[0] = (FAR char *)g_ports[port].devpath;
  argv[1] = NULL;

  pid = task_create(name, SERIAL_NSH_PRIO, SERIAL_NSH_STACK,
                    serial_nsh_task, argv);
  if (pid < 0)
    {
      return -errno;
    }

  return OK;
}

int serial_manager_start(void)
{
  bool have_nsh = false;
  int i;

  param_init();

  for (i = 0; i < SERIAL_NPORTS; i++)
    {
      FAR const struct serial_port_s *p = &g_ports[i];
      int32_t func = param_i32(p->func_param);
      int32_t baud = param_i32(p->baud_param);

      if (func == SER_FUNC_DISABLED)
        {
          continue;
        }

      /* Apply the baud rate before handing the port to anything. */

      if (serial_set_baud(p->devpath, (int)baud) < 0)
        {
          syslog(LOG_ERR, "serial: %s (%s): cannot set %" PRId32 " baud\n",
                 p->name, p->devpath, baud);
          continue;
        }

      /* The boot console already runs an NSH on this port (it is the
       * INIT_ENTRYPOINT), so starting a second one would give two shells
       * fighting over the same UART.
       */

      if (func == SER_FUNC_NSH && p->is_console)
        {
          syslog(LOG_INFO, "serial: %s (%s) NSH @ %" PRId32 " [boot console]\n",
                 p->name, p->devpath, baud);
          have_nsh = true;
          continue;
        }

      /* Warn if a protocol is assigned to the port that carries syslog. It is
       * not fatal - MAVLink and GPS parsers resync on framing - but our own
       * outgoing frames would get log text interleaved into them, and a GCS
       * would drop those. dmesg still has the full log either way.
       */

      if (p->is_console && func != SER_FUNC_NSH)
        {
          syslog(LOG_WARNING,
                 "serial: %s carries the syslog console; log output will "
                 "interleave with this port's protocol\n", p->name);
        }

      switch (func)
        {
          case SER_FUNC_NSH:
            if (serial_start_nsh(i) == OK)
              {
                syslog(LOG_INFO, "serial: %s (%s) NSH @ %" PRId32 "\n",
                       p->name, p->devpath, baud);
                have_nsh = true;
              }
            else
              {
                syslog(LOG_ERR, "serial: %s: NSH failed to start\n", p->name);
              }
            break;

          case SER_FUNC_MAVLINK:
          case SER_FUNC_GPS:
          case SER_FUNC_RC_IN:

            /* Owners for these arrive with the MAVLink, GPS and direct-UART RC
             * drivers. The port is configured and reserved; nothing consumes it
             * yet. Say so plainly rather than pretend it is running.
             */

            syslog(LOG_WARNING,
                   "serial: %s (%s) @ %" PRId32 ": function %" PRId32
                   " not implemented yet\n",
                   p->name, p->devpath, baud, func);
            break;

          default:
            syslog(LOG_ERR, "serial: %s: unknown function %" PRId32 "\n",
                   p->name, func);
            break;
        }
    }

  /* A shell must always exist. A parameter file - which is a text file on a
   * removable card, editable from a host - must never be able to lock you out
   * of the board.
   */

  if (!have_nsh)
    {
      syslog(LOG_WARNING,
             "serial: no port is configured for NSH; keeping the boot "
             "console so the board stays reachable\n");
    }

  return OK;
}
