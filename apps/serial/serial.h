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
 * NSH is deliberately not special. It is one function among several, and it can
 * be moved off TELEM1 onto any other port, or run on several at once. TELEM1
 * merely holds it by default because that is where the boot console lives.
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
  FAR const char *baud_param;/* "SER_TEL1_BAUD" */
  bool            is_console;/* this port carries /dev/console + syslog */
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

/* Read every SER_*_FUNC / SER_*_BAUD and start what they ask for.
 *
 * Called once at boot. Guarantees a shell exists: if no port is configured for
 * NSH (or the configured one fails to open), it falls back to the console -
 * a parameter file must never be able to lock you out of the board.
 */

int serial_manager_start(void);

#endif /* __APPS_SERIAL_SERIAL_H */
