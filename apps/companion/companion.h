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
  uint64_t last_rx_us;        /* board time of the last accepted frame */

  struct comp_parser_s parser;  /* frames, crc_errors, unknown_id, ... */
};

int  companion_start(void);
int  companion_stop(void);
void companion_status(FAR struct companion_status_s *out);

#endif /* __APPS_COMPANION_COMPANION_H */
