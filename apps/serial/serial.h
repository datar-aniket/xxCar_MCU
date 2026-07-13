/****************************************************************************
 * apps/serial/serial.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Serial port manager.
 *
 * Decides, at boot, what runs on each FMU serial connector, from parameters
 * rather than from the build. A port's function (NSH, MAVLink, GPS, RC, or
 * nothing) and its baud rate are just values in params.txt, editable from a
 * Linux host over USB mass storage.
 *
 * NSH is one function among several, and it can run on several ports at once -
 * including the USB CDC/ACM port. TELEM1 keeps a shell regardless of what the
 * parameters say, because NSH is the init entrypoint and owns /dev/console:
 * other ports ADD shells, they do not take that one away. That is deliberate -
 * a text file on a removable card must never be able to leave the board with no
 * way in.
 *
 * The RC IN connector is NOT in this list. On the 6C it belongs to the PX4IO
 * co-processor, not the FMU (see apps/px4io/), so it is not a port whose
 * function you can assign. USART6 is likewise absent - it IS the PX4IO link.
 ****************************************************************************/

#ifndef __APPS_SERIAL_SERIAL_H
#define __APPS_SERIAL_SERIAL_H

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* One assignable FMU serial port.
 *
 * The device names are only stable because the defconfig sets
 * CONFIG_STM32H7_SERIAL_DISABLE_REORDERING - without it NuttX renumbers ttySn
 * so the console is ttyS0, and every index here would shift as soon as the set
 * of enabled UARTs changed. See boards/fmuv6c/include/board.h.
 */

struct serial_port_s
{
  FAR const char *name;      /* connector, as silkscreened: "TELEM1" */
  FAR const char *devpath;   /* "/dev/ttyS5" */
  FAR const char *uart;      /* "UART7", for humans */
  FAR const char *func_param;/* "SER_TEL1_FUNC" */
  FAR const char *baud_param;/* "SER_TEL1_BAUD", or NULL if baud is meaningless */
  bool            is_console;/* this port carries /dev/console + syslog */

  /* Removable: the port only exists while a host is attached to it.
   *
   * True for the USB CDC/ACM port and nothing else. It changes two things.
   * First, opening it while no host is attached fails with -ENOTCONN rather
   * than succeeding, so a service cannot simply open it once at boot. Second,
   * the port dies when the cable is pulled and comes back when it is plugged in
   * again. Anything bound to it therefore has to wait for the host and re-arm
   * afterwards, instead of giving up - see serial_start_nsh().
   */

  bool            removable;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* The port table, and its length. */

FAR const struct serial_port_s *serial_ports(void);
int serial_port_count(void);

/* Look a port up by connector name, case-insensitively ("telem1"). <0 if
 * unknown.
 */

int serial_find(FAR const char *name);

/* Apply a port's SER_*_BAUD to the hardware. */

int serial_set_baud(FAR const char *devpath, int baud);

/* Start a detached NSH shell on a port.
 *
 * The port's fd is dup2'd onto stdin/stdout/stderr of a new task, so NSH talks
 * to that UART instead of to the console it inherited.
 */

int serial_start_nsh(int port);

/* Same, addressed by device path rather than by port index, so a shell can be
 * put on anything that behaves like a tty - including a port that is not in the
 * table. `removable` makes it wait for a host and re-arm on unplug (see the
 * `removable` field above); pass false for a plain UART.
 */

int serial_start_nsh_dev(FAR const char *devpath, bool removable);

/* Read every SER_*_FUNC / SER_*_BAUD and start what they ask for.
 *
 * Called once at boot, from board bring-up.
 *
 * The boot console (TELEM1) always keeps its shell - NSH is the init entrypoint
 * and owns /dev/console. Other ports ADD shells; they never take that one away.
 * params.txt is a text file on a removable card, edited from a host, so it must
 * never be able to leave the board with no way in.
 */

int serial_manager_start(void);

#endif /* __APPS_SERIAL_SERIAL_H */
