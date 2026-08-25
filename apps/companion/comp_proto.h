/****************************************************************************
 * apps/companion/comp_proto.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wire format for the companion computer link.
 *
 *   0xFE | id | len | payload[len] | crc16
 *
 * The CRC covers id, len and payload. Anything that is not the sync byte is
 * discarded, so a lost byte costs one message rather than wedging the
 * stream.
 *
 * Inbound ids are low and outbound ids are high. A message sent in the wrong
 * direction then fails to route at all, instead of half-working.
 *
 * No I/O and no uORB in this file. That is what lets the format be tested on
 * the host while it is still being iterated on.
 ****************************************************************************/

#ifndef __APPS_COMPANION_COMP_PROTO_H
#define __APPS_COMPANION_COMP_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

#define COMP_SYNC              0xfe
#define COMP_MAX_PAYLOAD       64
#define COMP_FRAME_OVERHEAD    5      /* sync + id + len + crc16 */

/* Inbound: companion -> board. */

#define COMP_MSG_EXTERNAL_POSE   1
#define COMP_MSG_CONTROL_TRAJ    2    /* reserved, not yet defined */
#define COMP_MSG_TIMESYNC_REQ    3
#define COMP_MSG_TIMESYNC_START  5
#define COMP_MSG_TIMESYNC_END    6

/* Outbound: board -> companion. */

#define COMP_MSG_TIMESYNC_REP    4
#define COMP_MSG_ESTIMATOR_POSE 16

/* Absolute pose in the companion's map frame. Only x, y and yaw are fused;
 * height stays with the barometer.
 *
 * cov is the UPPER TRIANGLE of the 3x3 for (x, y, yaw):
 *   [0] xx  [1] xy  [2] x-yaw  [3] yy  [4] y-yaw  [5] yaw-yaw
 *
 * A zero variance means "no estimate supplied, use the parameter", so the
 * companion side can start by sending zeros and tighten later without a
 * format change.
 */

struct comp_external_pose_s
{
  uint64_t timestamp_us;    /*  0: board timebase, per the operator's sync.
                             *     ZERO means "not timestamped" and the board
                             *     stamps it on arrival - which costs the
                             *     whole link latency as position error, and
                             *     is counted separately so a working
                             *     timesync is never silently replaced. */
  float    x;               /*  8: m, map frame */
  float    y;               /* 12 */
  float    yaw;             /* 16: rad, map frame */
  float    cov[6];          /* 20: see above */
  uint8_t  flags;           /* 44: bit 0 = pose valid */
  uint8_t  reset_counter;   /* 45: the SOURCE's frame-reset generation */
  uint8_t  pad[2];          /* 46 */
};

#define COMP_POSE_FLAG_VALID (1u << 0)

/* The estimator's pose, sent at EXT_TX_RATE.
 *
 * reset_counter matters more than it looks: the datum reset moves position
 * discontinuously, and anything differentiating position on the companion
 * side needs to know that happened rather than seeing a spike.
 */

struct comp_estimator_pose_s
{
  uint64_t timestamp_us;    /*  0: board monotonic */
  float    position[3];     /*  8: m, local ENU */
  float    quaternion[4];   /* 20: w x y z, body to nav */
  float    velocity[3];     /* 36: m/s, local ENU */
  uint8_t  solution_status; /* 48: ESTIMATOR_* validity bits */
  uint8_t  reset_counter;   /* 49: estimator reset generation */
  uint8_t  pad[6];          /* 50: a uint64 first member forces 8-byte
                             *     alignment, so 50 pads to 56 whatever this
                             *     says. Declared, so the wire format is what
                             *     the struct says rather than what the
                             *     compiler decided. */
};

/* Clock synchronisation, request and reply.
 *
 * The ordinary ping-pong: the companion sends its own clock, the board
 * echoes it and adds when it saw the request and when it sent the reply.
 * The companion then has four timestamps and can solve for both the offset
 * and the round trip:
 *
 *   offset     = ((board_rx - host_tx) + (board_tx - host_rx)) / 2
 *   round_trip = (host_rx - host_tx) - (board_tx - board_rx)
 *
 * Both terms matter. Without board_tx the board's own processing delay is
 * indistinguishable from wire latency and lands entirely in the offset.
 *
 * Sending several and keeping the one with the SMALLEST round trip is the
 * standard move: the offset estimate is only as good as the path is
 * symmetric, and the least-delayed exchange is the least asymmetric one.
 */

/* A sync burst is bracketed: START, then `count` exchanges, then END
 * carrying the result the companion settled on.
 *
 * The bracket is not what makes the offset accurate - the exchanges do that.
 * What it buys is that the BOARD knows a sync is in progress and what the
 * companion concluded, so `companion status` can show the agreed offset
 * instead of the board having no idea whether its peer thinks the clocks
 * are aligned.
 */

struct comp_timesync_start_s
{
  uint32_t count;           /* 0: exchanges about to be sent */
  uint32_t pad;             /* 4 */
};

struct comp_timesync_end_s
{
  /* Add to the board's MONOTONIC microseconds to get UTC microseconds since
   * the epoch. The companion computes it because only it sees all four
   * timestamps of an exchange.
   *
   * The board does not adopt UTC as its own timebase, and must not. Every
   * internal timestamp - IMU samples, the delay ring, the fusion horizon,
   * every staleness check - is monotonic, and monotonic is the only clock
   * that cannot step. Moving those to UTC would jump them from microseconds
   * since boot to microseconds since 1970, at which point every buffered
   * sample reads as impossibly old and the filter resets.
   *
   * So UTC is a WIRE format: converted on the way out, converted back on the
   * way in, and never seen by the estimator.
   */

  int64_t  utc_offset_us;   /*  0: board monotonic + this = UTC */
  uint32_t trip_us;         /*  8: the round trip it was measured at */
  uint32_t samples;         /* 12: exchanges that came back */
};

struct comp_timesync_req_s
{
  uint64_t host_tx_us;      /* 0: companion UTC microseconds when asked */
};

struct comp_timesync_rep_s
{
  uint64_t host_tx_us;      /*  0: echoed, so replies cannot be mismatched */
  uint64_t board_rx_us;     /*  8: board clock when the request arrived */
  uint64_t board_tx_us;     /* 16: board clock when this reply was sent */
};

enum comp_parse_state_e
{
  COMP_WAIT_SYNC = 0,
  COMP_WAIT_ID,
  COMP_WAIT_LEN,
  COMP_WAIT_PAYLOAD,
  COMP_WAIT_CRC_LO,
  COMP_WAIT_CRC_HI
};

struct comp_parser_s
{
  uint8_t  state;
  uint8_t  id;
  uint8_t  len;
  uint8_t  fill;
  uint16_t crc_rx;
  uint8_t  payload[COMP_MAX_PAYLOAD];

  uint32_t frames;          /* accepted */
  uint32_t crc_errors;
  uint32_t unknown_id;      /* benign: a companion newer than this firmware */
  uint32_t bad_length;      /* NOT benign: the two ends disagree on a format */
  uint32_t resyncs;
};

/* CRC16-CCITT-FALSE. Same polynomial and seed the cal protocol uses, so a
 * host implementing one has implemented both.
 */

uint16_t comp_crc16(FAR const uint8_t *d, size_t n);
uint16_t comp_crc16_update(uint16_t crc, FAR const uint8_t *d, size_t n);

/* Expected payload length for a known id, or 0 when the id is unknown.
 *
 * Knowing the length is what separates "a companion newer than this
 * firmware", which is fine and ignorable, from "the two ends disagree about
 * a format", which is not.
 */

uint8_t comp_payload_len(uint8_t id);

void comp_parser_init(FAR struct comp_parser_s *p);

/* Feed exactly one byte. Returns the message id when a complete, CRC-valid
 * frame with the right length just completed, and 0 otherwise. The payload is
 * then in p->payload for p->len bytes, valid until the next call.
 *
 * One byte at a time, deliberately. A parser that only works on whole frames
 * passes every other test and fails the first time a UART splits one, which
 * it will.
 */

int comp_parser_byte(FAR struct comp_parser_s *p, uint8_t b);

/* Frame payload into out. Returns the number of bytes written, or a negated
 * errno.
 */

int comp_encode(uint8_t id, FAR const void *payload, uint8_t len,
                FAR uint8_t *out, size_t out_size);

#endif /* __APPS_COMPANION_COMP_PROTO_H */
