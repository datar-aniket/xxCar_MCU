/****************************************************************************
 * apps/init/init_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The init task (CONFIG_INIT_ENTRYPOINT).
 *
 * This replaces nsh_main as the entry point for one reason: nsh_main
 * unconditionally takes /dev/console and runs a shell on it. That made TELEM1's
 * shell impossible to turn off, whatever SER_TEL1_FUNC said - if you assigned
 * TELEM1 to MAVLink you got MAVLink *and* a shell fighting over the same UART.
 *
 * So the decision moves here. The console shell runs only if the console's port
 * asked for one. Everything else on every other port was already started by the
 * serial manager during board bring-up, which runs before this task.
 *
 * Nothing else moves. Board initialisation still happens in
 * board_late_initialize() -> stm32_bringup(), driven by the OS and completely
 * independent of which task is the init task; board_app_initialize() is already
 * a no-op under CONFIG_BOARD_LATE_INITIALIZE.
 *
 * A board with no shell at all is a legal configuration and boots normally - it
 * is simply headless. The ways back in are the USB shell (on by default), the
 * microSD card (pull it and edit params.txt), or a reflash.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <syslog.h>
#include <unistd.h>
#include <termios.h>

#include <nshlib/nshlib.h>

#ifdef CONFIG_XXCAR_SERIAL
#  include "../serial/serial.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
#ifdef CONFIG_XXCAR_SERIAL
  /* The serial manager already ran (board bring-up, before this task), so it
   * has read the parameters and started every non-console port. All that is
   * left to decide is whether the console itself hosts a shell.
   */

  if (!serial_console_wants_nsh())
    {
      syslog(LOG_INFO,
             "init: console port is not set to NSH - no shell here\n");

      /* Exiting is the whole point: it leaves /dev/console free for whatever
       * function was assigned to that port. Every service the manager started
       * is its own task and keeps running.
       */

      return 0;
    }
#endif

#ifdef CONFIG_NSH_CLE
  /* Stop the driver echoing on the console.
   *
   * uart_register() turns ECHO on for /dev/console and nothing else, which is
   * right for readline (readline does not echo; the driver does). CLE is the
   * other way round: it redraws the whole line itself on every keypress, so a
   * driver that also echoes puts every character on screen twice.
   *
   * Non-console ports are handled the same way, in apps/serial.
   */

    {
      struct termios tio;

      if (tcgetattr(STDIN_FILENO, &tio) == 0)
        {
          tio.c_lflag &= ~ECHO;
          tcsetattr(STDIN_FILENO, TCSANOW, &tio);
        }
    }
#endif

  /* Becomes the console shell. Does not return. */

  nsh_consolemain(0, NULL);
  return 0;
}
