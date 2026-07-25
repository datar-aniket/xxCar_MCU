/****************************************************************************
 * apps/logger/logger.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * On-request flight logger. Records the uORB topics selected by the LOG_*
 * parameters to a ULog file on the microSD, at their native rate.
 *
 * ULog, not CSV, and that is a deliberate choice about native-rate IMU: the two
 * onboard IMUs stream at 2 kHz, so at full rate the logger sees ~8000
 * samples/second. ULog stores each as its raw ~24-byte record plus a 3-byte
 * header - a few hundred KB/s, which a class-10 card absorbs without noticing.
 * CSV at that rate would spend more CPU turning floats into text than doing the
 * logging, and produce files several times larger. And a .ulg opens directly in
 * pyulog / PlotJuggler / FlightPlot.
 *
 * "On request" means it does not run unless asked. `log start` begins a
 * session; `log stop` ends it; LOG_ENABLE=1 makes it also start at boot. The
 * LOG_IMU / LOG_MAG / LOG_BARO / LOG_RC parameters choose which topics are in
 * the file, and LOG_RATE caps the per-topic rate (0 = every sample, i.e. full
 * native rate).
 ****************************************************************************/

#ifndef __APPS_LOGGER_LOGGER_H
#define __APPS_LOGGER_LOGGER_H

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct logger_status_s
{
  bool     running;
  char     path[48];       /* the .ulg being written */
  uint32_t topics;         /* how many topics are in this session */
  uint32_t samples;        /* records written */
  uint32_t bytes;          /* bytes written to the card */
  uint32_t dropped;        /* samples dropped because the card fell behind */
  int32_t  rate;           /* LOG_RATE this session was started with */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Start a logging session. Reads the LOG_* parameters for what to record and
 * how fast. Returns 0, or a negated errno; -EALREADY if already logging.
 */

int  logger_start(void);
void logger_stop(void);
bool logger_is_running(void);

int  logger_get_status(FAR struct logger_status_s *status);

#endif /* __APPS_LOGGER_LOGGER_H */
