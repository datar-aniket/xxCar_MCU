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
#include <signal.h>

#include <nshlib/nshlib.h>

#include "serial.h"
#include "../param/param.h"

#ifdef CONFIG_XXCAR_RC
#  include "../rc/rc.h"
#endif

#ifdef CONFIG_XXCAR_MAVLINK
#  include "../mavlink/mavlink.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SERIAL_NSH_STACK  4096  /* same as the boot console: `ps` measured the
                                 * NSH task using 2136 bytes, and 2048 was not
                                 * enough (it reset the board).
                                 */
#define SERIAL_NSH_PRIO   SCHED_PRIORITY_DEFAULT

/* Where the shell goes when no port asked for one. The FMU DEBUG connector: it
 * is on every board, always present (USB needs a cable), and it is the port you
 * reach for when something is wrong.
 */

#define SERIAL_RESCUE_PORT "DEBUG"

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

/* Whether the console port asked for a shell. Set by serial_manager_start(),
 * read by the init task (apps/init), which is what actually owns /dev/console.
 */

static bool g_console_nsh;

/* The shell task on each port, or 0 for none.
 *
 * A tty has exactly one useful reader. Put two shells on one port and every
 * character you type goes to whichever happens to wake first, so neither ever
 * assembles a whole line and the port looks dead.
 *
 * This holds the pid rather than a bool because a shell can go away on its own:
 * typing `exit` makes NSH call nsh_exit(), which ends the TASK. A flag would
 * stay set and the port would be locked out for good, so instead the pid is
 * checked for life before it is believed.
 */

static pid_t g_nsh_pid[SERIAL_NPORTS];

/* Is the recorded shell for this port still alive?
 *
 * kill(pid, 0) sends no signal - it only reports whether the process exists.
 */

static bool serial_nsh_alive(int port)
{
  if (g_nsh_pid[port] <= 0)
    {
      return false;
    }

  if (kill(g_nsh_pid[port], 0) == 0)
    {
      return true;
    }

  /* It exited (`exit` at the prompt, most likely). Forget it, so the port can
   * be given a new shell.
   */

  g_nsh_pid[port] = 0;
  return false;
}


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
  struct termios tio;
  bool removable;
  bool waiting = false;
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
          int err = errno;

          /* ONLY "no host attached" is worth waiting on. Everything else -
           * a device that does not exist, a driver that is not there - is a
           * real error, and retrying it forever would hide the problem behind
           * a task that looks alive but silently does nothing. Which is exactly
           * what an earlier version of this loop did.
           */

          if (removable && (err == ENOTCONN || err == ENODEV))
            {
              if (!waiting)
                {
                  syslog(LOG_INFO,
                         "serial: NSH on %s waiting for a host to attach\n",
                         devpath);
                  waiting = true;
                }

              usleep(250000);
              continue;
            }

          syslog(LOG_ERR, "serial: cannot open %s for NSH: errno %d\n",
                 devpath, err);
          return EXIT_FAILURE;
        }

      waiting = false;

      /* Give the port a terminal line discipline.
       *
       * NuttX does this in uart_register() under `if (dev->isconsole)` and
       * nowhere else, so /dev/console works and every other tty is left raw.
       * That is the whole reason a shell on TELEM1 behaves and a shell anywhere
       * else does not. What a raw tty costs, and what fixes it:
       *
       *   ICRNL  - a terminal's Enter sends CR (0x0D). Readline ends a line on
       *            '\n' and nothing else (readline_common.c), so without CR->LF
       *            the line never completes and the shell simply never answers.
       *            This is why `echo ps > /dev/ttyACM0` worked (echo sends LF)
       *            while pressing Enter in a terminal did nothing.
       *
       *   ECHO   - who echoes depends on the editor, so this flag does too.
       *            readline does NOT echo (the driver does, under `tc_lflag &
       *            ECHO`), but CLE redraws the whole line itself on every
       *            keypress - so with CLE the driver must stay quiet or every
       *            character appears twice.
       *
       *   ONLCR  - the shell's *output* is bare '\n'. Without LF->CRLF the
       *            cursor never returns to column 0 and output walks down the
       *            screen.
       *
       *   ISIG   - Ctrl-C. Independent of everything else (uart_check_special).
       *
       * ICANON is deliberately NOT set, even though the console has it.
       *
       * It changes what a read means: with ICANON, uart_read refuses to return
       * partial data and blocks until it sees '\n' (serial.c, `recvd > 0 &&
       * !(dev->tc_lflag & ICANON)`). Readline reads one byte at a time and is
       * supposed to dodge that by clearing ICANON itself - but readline_fd()
       * only does so `if (isatty(infd))`, so it is a dependency, and setting
       * ICANON here broke Enter all over again. Nothing wants it: the echo and
       * the CR/LF mapping above are all independent of it, and readline does its
       * own line editing.
       */

      if (tcgetattr(fd, &tio) == 0)
        {
          tio.c_iflag |= ICRNL;
          tio.c_oflag |= OPOST | ONLCR;
          tio.c_lflag |= ISIG;

#ifdef CONFIG_NSH_CLE
          tio.c_lflag &= ~ECHO;
#else
          tio.c_lflag |= ECHO;
#endif
          tio.c_lflag &= ~ICANON;
          tcsetattr(fd, TCSANOW, &tio);
        }

      /* Point the standard streams at this port. NSH reads fd 0 and writes fd 1
       * directly (nsh_console.c: read(INFD)/write(OUTFD) with INFD=STDIN_FILENO,
       * OUTFD=STDOUT_FILENO), so redirecting the descriptors is all it takes -
       * NSH itself needs to know nothing about which port it is on.
       */

      if (dup2(fd, 0) < 0 || dup2(fd, 1) < 0 || dup2(fd, 2) < 0)
        {
          syslog(LOG_ERR, "serial: %s: cannot redirect stdio: errno %d\n",
                 devpath, errno);
          close(fd);
          return EXIT_FAILURE;
        }

      if (fd > 2)
        {
          close(fd);
        }

      syslog(LOG_INFO, "serial: NSH attached to %s\n", devpath);

      /* Returns when the port goes away under it - i.e. when a USB cable is
       * unplugged and reads start failing.
       */

      nsh_consolemain(0, NULL);

      syslog(LOG_INFO, "serial: NSH on %s ended\n", devpath);

      if (!removable)
        {
          return EXIT_SUCCESS;
        }

      /* Host gone. Settle, then wait for it to come back. The sleep also keeps
       * this from becoming a busy loop if the port opens but is immediately
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

  /* The pid, not OK: the caller tracks it so that a shell which exits can be
   * told apart from one that is still running.
   */

  return pid;
}

int serial_start_nsh(int port)
{
  int ret;

  if (port < 0 || port >= SERIAL_NPORTS)
    {
      return -EINVAL;
    }

  /* One shell per port, but only if the last one is still alive - a shell that
   * was exited must not lock the port out for ever.
   */

  if (serial_nsh_alive(port))
    {
      return -EALREADY;
    }

  ret = serial_start_nsh_dev(g_ports[port].devpath, g_ports[port].removable);
  if (ret > 0)
    {
      g_nsh_pid[port] = (pid_t)ret;
      ret = OK;
    }

  return ret;
}

int serial_manager_start(void)
{
  bool have_nsh = false;
  int ret;
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

      /* The console port's shell is not started here. The init task IS that
       * shell (apps/init owns /dev/console), so spawning another would leave two
       * shells fighting over one UART. Just record that it was asked for; init
       * reads this back through serial_console_wants_nsh().
       */

      if (func == SER_FUNC_NSH && p->is_console)
        {
          syslog(LOG_INFO, "serial: %s (%s) NSH @ %" PRId32 " [boot console]\n",
                 p->name, p->devpath, baud);
          g_console_nsh = true;
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
            ret = serial_start_nsh(i);

            if (ret == -EALREADY)
              {
                syslog(LOG_INFO, "serial: %s already has a shell\n", p->name);
                have_nsh = true;
              }
            else if (ret == OK)
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
                syslog(LOG_ERR, "serial: %s: NSH failed to start: %d\n",
                       p->name, ret);
              }
            break;

          case SER_FUNC_RC_IN:
#ifdef CONFIG_XXCAR_RC
            /* The RC driver owns the line settings from here on: SBUS and CRSF
             * each dictate their own baud, parity, stop bits and polarity, so
             * SER_*_BAUD means nothing on an RC port and is ignored.
             */

            ret = rc_start(p->devpath, param_i32("RC_PROT"));

            if (ret < 0 && ret != -ENOTSUP)
              {
                syslog(LOG_ERR, "serial: %s: RC failed to start: %d\n",
                       p->name, ret);
              }
            else if (ret == OK)
              {
                syslog(LOG_INFO, "serial: %s (%s) RC\n", p->name, p->devpath);
              }
#else
            syslog(LOG_WARNING,
                   "serial: %s: RC requested but XXCAR_RC is not built in\n",
                   p->name);
#endif
            break;

          case SER_FUNC_MAVLINK:
#ifdef CONFIG_XXCAR_MAVLINK
            ret = mavlink_start(p->devpath, baud);

            if (ret < 0 && ret != -EALREADY)
              {
                syslog(LOG_ERR, "serial: %s: MAVLink failed to start: %d\n",
                       p->name, ret);
              }
            else
              {
                syslog(LOG_INFO, "serial: %s (%s) MAVLink @ %" PRId32 "\n",
                       p->name, p->devpath, baud);
              }
#else
            syslog(LOG_WARNING,
                   "serial: %s: MAVLink requested but XXCAR_MAVLINK is not "
                   "built in\n", p->name);
#endif
            break;

          case SER_FUNC_GPS:

            /* The GPS driver is not written yet. The port is configured and
             * reserved; nothing consumes it. Say so plainly.
             */

            syslog(LOG_WARNING,
                   "serial: %s (%s) @ %" PRId32 ": GPS not implemented yet\n",
                   p->name, p->devpath, baud);
            break;

          case SER_FUNC_CAL:

            /* Deliberately starts nothing. The value exists so a port can be
             * marked as belonging to the calibration GUI and, crucially, have
             * NO shell put on it - a shell here would sit blocked in read()
             * and steal the session's input. `cal session` opens it on demand.
             */

            syslog(LOG_INFO,
                   "serial: %s (%s) reserved for calibration - no shell\n",
                   p->name, p->devpath);
            break;

          case SER_FUNC_COMPANION:

            /* The serial manager only reserves the port. The companion daemon
             * opens it later in bring-up when COMP_EN is set. A shell here
             * would sit blocked in read() and steal the link's input.
             */

            syslog(LOG_INFO,
                   "serial: %s (%s) reserved for the companion link - "
                   "no shell\n", p->name, p->devpath);
            break;

          default:
            syslog(LOG_ERR, "serial: %s: unknown function %" PRId32 "\n",
                   p->name, func);
            break;
        }
    }

  /* Nothing asked for a shell. Put one on the FMU DEBUG connector.
   *
   * That port exists on every board, is always present (unlike USB, which needs
   * a cable), and is the connector you would reach for anyway when something has
   * gone wrong. It is the rescue shell: a params.txt that assigns every port to
   * a protocol should not leave the board with no way in, and params.txt is a
   * text file on a removable card that people are meant to hand-edit.
   *
   * If DEBUG is itself carrying a function, leave it alone. Stomping a port the
   * user deliberately configured would be worse than being headless, and they
   * still have the card and the bootloader.
   */

  if (!have_nsh)
    {
      int dbg = serial_find(SERIAL_RESCUE_PORT);

      if (dbg >= 0 &&
          param_i32(g_ports[dbg].func_param) == SER_FUNC_DISABLED)
        {
          int32_t dbgbaud = param_i32(g_ports[dbg].baud_param);

          serial_set_baud(g_ports[dbg].devpath, (int)dbgbaud);

          if (serial_start_nsh(dbg) == OK)
            {
              syslog(LOG_WARNING,
                     "serial: no port set to NSH - rescue shell on %s (%s) "
                     "@ %" PRId32 "\n",
                     g_ports[dbg].name, g_ports[dbg].devpath, dbgbaud);
            }
          else
            {
              syslog(LOG_ERR, "serial: rescue shell on %s failed to start\n",
                     g_ports[dbg].name);
            }
        }
      else
        {
          syslog(LOG_WARNING,
                 "serial: no port set to NSH, and %s is in use - there will be "
                 "NO shell. Recover by editing params.txt on the card.\n",
                 SERIAL_RESCUE_PORT);
        }
    }

  return OK;
}

bool serial_console_wants_nsh(void)
{
  return g_console_nsh;
}
