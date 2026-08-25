/****************************************************************************
 * apps/companion/companion.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The link to the companion computer.
 *
 * Owns one serial port - whichever SER_*_FUNC names SER_FUNC_COMPANION - and
 * routes framed packets to uORB topics by message id. It knows nothing about
 * navigation: adding a message is a row in the routing table plus a topic,
 * not a new code path, which is what the control trajectory will need.
 ****************************************************************************/

#ifndef __APPS_COMPANION_COMPANION_H
#define __APPS_COMPANION_COMPANION_H

#include <stdbool.h>
#include <stdint.h>

#include "comp_proto.h"

#ifndef FAR
#  define FAR
#endif

struct companion_status_s
{
  bool     running;
  char     port[16];          /* connector name, as silkscreened */
  uint32_t baud;
  uint32_t tx_rate_hz;

  uint64_t bytes_in;
  uint64_t bytes_out;
  uint32_t tx_frames;
  uint32_t tx_errors;

  uint32_t rx_pose;           /* EXTERNAL_POSE routed and published */
  uint32_t rx_publish_errors;
  uint32_t timesync_replies;
  uint32_t timesync_expected;   /* the burst the companion announced */
  uint32_t timesync_samples;    /* what came back, per the companion */
  int64_t  timesync_offset_us;  /* what the companion settled on */
  uint32_t timesync_trip_us;
  bool     timesync_synced;
  bool     wall_clock_set;      /* CLOCK_REALTIME set from companion UTC */
  uint32_t rx_unsynced_stamp;   /* UTC arrived before a sync could use it */
  uint32_t est_seen;          /* estimator states actually read */
  uint32_t tx_no_state;       /* nothing new to send */
  uint32_t connects;          /* hosts attached */
  uint32_t disconnects;
  bool     waiting_for_host;  /* removable port, nobody plugged in */
  uint64_t last_rx_us;        /* board time of the last accepted frame */

  struct comp_parser_s parser;  /* frames, crc_errors, unknown_id, ... */
};

int  companion_start(void);
int  companion_stop(void);
void companion_status(FAR struct companion_status_s *out);

#endif /* __APPS_COMPANION_COMPANION_H */
