/****************************************************************************
 * apps/cal/cal.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Sensor calibration session.
 *
 * `cal session` is run from the shell on TELEM1/DEBUG and opens the USB CDC
 * port, where the calibration GUI is listening. That split is the whole reason
 * this needs no handoff dance: the shell is on a different port, so nothing else
 * ever holds /dev/ttyACM0 and there is no race to lose.
 *
 * The board only ACQUIRES. It detects stillness, averages a segment and reports
 * the vector; the ellipsoid and alignment fits run on the host, where they can
 * be improved without reflashing. Parameters are written once, at the end of a
 * session - the USB port dies on cable pull, and a half-written calibration is
 * worse than none.
 ****************************************************************************/

#ifndef __APPS_CAL_CAL_H
#define __APPS_CAL_CAL_H

#include <nuttx/config.h>

/* The port the GUI listens on. Not configurable: it is the only USB serial
 * device on this board.
 */

#define CAL_DEVPATH "/dev/ttyACM0"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Run a calibration session until the host quits or the cable is pulled.
 * Returns OK, or a negated errno (-ENOTCONN: no host attached; -EBUSY: someone
 * else holds the port).
 */

int cal_session(void);

/* Print the stored calibration for every sensor. No USB involved. */

int cal_print_status(void);

#endif /* __APPS_CAL_CAL_H */
