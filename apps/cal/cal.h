/****************************************************************************
 * apps/cal/cal.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Calibration session, spoken over the USB CDC port to a host GUI.
 *
 * Two encodings share the one pipe, and the host tells them apart by the first
 * byte of each message:
 *
 *   '{'   an ASCII JSON line, terminated by '\n' - control and replies
 *   0xA5  a binary sample frame - see CAL_SYNC below
 *
 * Commands are ASCII because they are typed by a human at most once and read by
 * a person forever after; sample data is binary because it is neither. 0xA5 is
 * not printable and cannot be the '{' that starts a JSON line, so a reader can
 * always resynchronise on it.
 *
 * `cal session` runs from the shell on TELEM1/DEBUG and opens /dev/ttyACM0.
 * The shell living on a different port is what makes this safe: nothing else
 * holds the USB port, so there is no handoff and no race for input bytes.
 ****************************************************************************/

#ifndef __APPS_CAL_CAL_H
#define __APPS_CAL_CAL_H

#include <nuttx/config.h>

#include <stdint.h>

/* The port the GUI listens on. Not configurable - it is the only USB serial
 * device on this board.
 */

#define CAL_DEVPATH  "/dev/ttyACM0"

/* Protocol version, reported in the hello event so a mismatched GUI can say so
 * instead of misbehaving.
 */

#define CAL_PROTO_VERSION 1

/* Binary sample frame:
 *
 *   off  size  field
 *   0    1     0xA5      sync
 *   1    1     len       bytes that follow, excluding the CRC = 6 + 4*n
 *   2    1     id        sensor id, as reported by `list`
 *   3    1     seq       per-sensor, wraps at 256 - the GUI detects drops
 *   4    4     t_us      uint32 LE, sample timestamp
 *   8    4*n   values    float32 LE, n as reported by `list`
 *   8+4n 2     crc16     CCITT-FALSE over bytes 1 .. 7+4n
 */

#define CAL_SYNC        0xa5
#define CAL_MAX_VALUES  4        /* widest sensor we stream */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Run a session until the host quits or the cable is pulled. Returns OK, or a
 * negated errno (-ENOTCONN no host, -EBUSY port not reserved).
 */

int cal_session(void);

#endif /* __APPS_CAL_CAL_H */
