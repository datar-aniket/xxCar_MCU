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
  /* name      devpath        uart      func            baud             cons   rem */
  { "GPS1",   "/dev/ttyS0",  "USART1", "SER_GPS1_FUNC", "SER_GPS1_BAUD", false, false },
  { "TELEM3", "/dev/ttyS1",  "USART2", "SER_TEL3_FUNC", "SER_TEL3_BAUD", false, false },
  { "DEBUG",  "/dev/ttyS2",  "USART3", "SER_DBG_FUNC",  "SER_DBG_BAUD",  false, false },
  { "TELEM2", "/dev/ttyS3",  "UART5",  "SER_TEL2_FUNC", "SER_TEL2_BAUD", false, false },
  { "TELEM1", "/dev/ttyS5",  "UART7",  "SER_TEL1_FUNC", "SER_TEL1_BAUD", true,  false },
  { "GPS2",   "/dev/ttyS6",  "UART8",  "SER_GPS2_FUNC", "SER_GPS2_BAUD", false, false },

  /* The USB CDC/ACM port. Not a UART, so it differs on two counts:
   *
   *   - no baud rate. The host owns the line coding and the device ignores it,
   *     so there is no SER_USB_BAUD to set - hence NULL.
   *   - removable. It only exists while a host is attached.
   */

  { "USB",    "/dev/ttyACM0", "OTG FS", "SER_USB_FUNC", NULL,            false, true  },
};

#define SERIAL_NPORTS ((int)(sizeof(g_ports) / sizeof(g_ports[0])))


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Entry point of an NSH shell bound to a specific port.
 *
 * argv[1] is the device path; argv[2] is "r" if the port is removable.
 *
 * A task starts life with stdin/stdout/stderr pointing at the console it
 * inherited, so to make NSH talk to some other port we open that port and
 * dup2() it over fds 0, 1 and 2. NSH itself needs no knowledge of any of this
 * and just reads and writes the standard streams.
 *
 * A removable port (USB) needs more than that. It does not exist until a host
 * attaches: open() returns -ENOTCONN until then, so opening once at boot would
 * simply fail and leave no shell even after the cable was plugged in. And when
 * the cable is pulled the port dies under NSH, which returns. So for a removable
 * port this waits for the host, runs the shell, and goes back to waiting when
 * the host disappears - for as long as the board is up.
 */

static int serial_nsh_task(int argc, FAR char *argv[])
{
  FAR const char *devpath;
  bool removable;
  int fd;

  if (argc < 2)
    {
      return EXIT_FAILURE;
    }

  devpath  = argv[1];
  removable = (argc > 2 && argv[2][0] == 'r');

  for (; ; )
    {
      fd = open(devpath, O_RDWR);
      if (fd < 0)
        {
          if (!removable)
            {
              syslog(LOG_ERR, "serial: cannot open %s for NSH: %d\n",
                     devpath, errno);
              return EXIT_FAILURE;
            }

          /* No host attached yet (-ENOTCONN). Keep waiting: the cable may be
           * plugged in at any time, and a shell that gave up at boot would
           * never come back.
           */

          usleep(250000);
          continue;
        }

      dup2(fd, 0);
      dup2(fd, 1);
      dup2(fd, 2);

      if (fd > 2)
        {
          close(fd);
        }

      /* Returns when the port goes away under it - i.e. when the USB cable is
       * unplugged and reads start failing.
       */

      nsh_consolemain(0, NULL);

      if (!removable)
        {
          return EXIT_SUCCESS;
        }

      /* Host gone. Settle, then wait for it to come back. The sleep also stops
       * this becoming a busy loop if the port is openable but immediately
       * unreadable.
       */

      usleep(250000);
    }
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

int serial_start_nsh_dev(FAR const char *devpath, bool removable)
{
  FAR char *argv[3];
  FAR const char *leaf;
  char name[16];
  int pid;

  if (devpath == NULL)
    {
      return -EINVAL;
    }

  /* Name the task after the device, not the path: "nsh_ttyACM0". */

  leaf = strrchr(devpath, '/');
  leaf = leaf ? leaf + 1 : devpath;
  snprintf(name, sizeof(name), "nsh_%s", leaf);

  /* NuttX puts the task name in argv[0] and our arguments after it, so the task
   * sees devpath at argv[1] and the removable flag at argv[2].
   */

  argv[0] = (FAR char *)devpath;
  argv[1] = (FAR char *)(removable ? "r" : "-");
  argv[2] = NULL;

  pid = task_create(name, SERIAL_NSH_PRIO, SERIAL_NSH_STACK,
                    serial_nsh_task, argv);
  if (pid < 0)
    {
      return -errno;
    }

  return OK;
}

int serial_start_nsh(int port)
{
  if (port < 0 || port >= SERIAL_NPORTS)
    {
      return -EINVAL;
    }

  return serial_start_nsh_dev(g_ports[port].devpath, g_ports[port].removable);
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
      int32_t baud = p->baud_param ? param_i32(p->baud_param) : 0;

      if (func == SER_FUNC_DISABLED)
        {
          continue;
        }

      /* Apply the baud rate before handing the port to anything.
       *
       * Skipped for USB: the host owns the line coding on a CDC/ACM port and
       * the device ignores it, so there is nothing to set. Skipped too because
       * the port cannot even be opened until a host attaches - trying would
       * fail with -ENOTCONN and we would refuse to start the shell that is
       * meant to be waiting for exactly that host to turn up.
       */

      if (p->baud_param != NULL &&
          serial_set_baud(p->devpath, (int)baud) < 0)
        {
          syslog(LOG_ERR, "serial: %s (%s): cannot set %" PRId32 " baud\n",
                 p->name, p->devpath, baud);
          continue;
        }

      /* The console port already has a shell: NSH is the init entrypoint and it
       * owns /dev/console. Starting a second one here would give two shells
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
                if (p->removable)
                  {
                    syslog(LOG_INFO,
                           "serial: %s (%s) NSH [waiting for a host]\n",
                           p->name, p->devpath);
                  }
                else
                  {
                    syslog(LOG_INFO, "serial: %s (%s) NSH @ %" PRId32 "\n",
                           p->name, p->devpath, baud);
                  }

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

  /* The boot console (TELEM1) keeps its shell regardless: NSH is the init
   * entrypoint and owns /dev/console. That is deliberate - params.txt is a text
   * file on a removable card, edited from a host, and it must never be able to
   * leave the board with no way in. Other ports ADD shells; they do not take
   * this one away.
   */

  if (!have_nsh)
    {
      syslog(LOG_INFO,
             "serial: no port set to NSH; the boot console still has one\n");
    }

  return OK;
}
