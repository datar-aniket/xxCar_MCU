/****************************************************************************
 * apps/mavlink/mavlink.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MAVLink on a configured FMU serial port.
 *
 * Full MAVLink, not a hand-picked subset: the wire codec is the official
 * generated c_library_v2 (deps/mavlink), so ANY message decodes, not just the
 * ones we act on today. Only messages linked by the code cost flash - the
 * generated packers are all `static inline` - so "full" here does not mean
 * "heavy".
 *
 * What the daemon actually does:
 *   - RX: decode every frame; act on the ones we understand (the MTF-02's
 *     OPTICAL_FLOW_RAD and DISTANCE_SENSOR -> uORB, and the PARAM protocol),
 *     count the rest.
 *   - TX: a 1 Hz HEARTBEAT, and the PARAM protocol so a GCS can read and write
 *     the parameter table.
 *
 * Started by the serial manager on whichever port has SER_*_FUNC = MAVLINK, at
 * that port's SER_*_BAUD. It is a per-port instance; nothing here assumes there
 * is only one serial link in the world, even though today there is one.
 ****************************************************************************/

#ifndef __APPS_MAVLINK_MAVLINK_H
#define __APPS_MAVLINK_MAVLINK_H

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mavlink_status_s
{
  bool     running;
  char     devpath[16];
  int32_t  baud;
  uint8_t  sysid;
  uint8_t  compid;

  uint32_t rx_frames;      /* frames parsed OK */
  uint32_t rx_dropped;     /* parser errors (bad CRC / desync) */
  uint32_t tx_frames;      /* frames we sent */
  uint32_t flow_msgs;      /* OPTICAL_FLOW_RAD routed to uORB */
  uint32_t dist_msgs;      /* DISTANCE_SENSOR routed to uORB */
  uint32_t param_tx;       /* PARAM_VALUE sent */

  bool     peer_seen;      /* have we heard any frame at all? */
  uint8_t  peer_sysid;     /* sysid of the last sender */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Start / stop the daemon on a port. baud <= 0 leaves the port's current baud
 * (the serial manager sets it). Returns 0, or a negated errno; -EALREADY if it
 * is already running.
 */

int  mavlink_start(FAR const char *devpath, int32_t baud);
void mavlink_stop(void);
bool mavlink_is_running(void);

int  mavlink_get_status(FAR struct mavlink_status_s *status);

#endif /* __APPS_MAVLINK_MAVLINK_H */
