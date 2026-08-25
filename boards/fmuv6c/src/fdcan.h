/****************************************************************************
 * boards/fmuv6c/src/fdcan.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal FDCAN1 receiver.
 *
 * Written rather than taken from NuttX because the only H7 CAN driver NuttX
 * ships is SocketCAN, which needs CONFIG_NET - a whole network stack to
 * carry two message types. PX4 reached the same conclusion on the same
 * silicon and drives the peripheral directly.
 *
 * RECEIVE ONLY. There is no transmit path here at all, which is deliberate:
 * a driver that can only listen cannot spin a motor while the bit timing is
 * still being proven.
 *
 * Classic CAN, 8-byte frames. CAN FD would change the message RAM element
 * size from four words to eighteen and invalidate the layout in fdcan.c.
 ****************************************************************************/

#ifndef __BOARDS_FMUV6C_SRC_FDCAN_H
#define __BOARDS_FMUV6C_SRC_FDCAN_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

struct fdcan_frame_s
{
  uint32_t id;              /* 29-bit extended identifier */
  uint8_t  dlc;             /* 0..8 */
  uint8_t  data[8];
};

struct fdcan_stats_s
{
  uint32_t rx;              /* frames handed out */
  uint32_t lost;            /* RX FIFO0 overruns - not drained fast enough */
  uint32_t rejected;        /* non-extended or remote frames */
  uint8_t  last_error;      /* PSR LEC */
  bool     bus_off;
  bool     error_passive;
};

/* Bring up the peripheral. Only 1000000 is supported today; anything else is
 * refused rather than silently mis-timed.
 */

int fdcan_init(uint32_t bitrate);

/* Take one frame. Returns OK, or -EAGAIN when the FIFO is empty.
 *
 * Non-blocking and polled. Much less code than an interrupt path and
 * adequate for telemetry, but it puts the caller's poll interval as jitter
 * on the data - worth revisiting if a control loop ever depends on it.
 */

int fdcan_receive(FAR struct fdcan_frame_s *frame);

/* Narrow the HARDWARE filter to one controller id; 0 accepts any.
 *
 * In hardware rather than in software because an unfiltered 1 Mbit/s bus can
 * deliver over 8000 frames a second, and rejecting them in a task means
 * waking up 8000 times a second to throw work away.
 */

int  fdcan_set_filter(uint8_t controller_id);

void fdcan_stats(FAR struct fdcan_stats_s *out);

#endif /* __BOARDS_FMUV6C_SRC_FDCAN_H */
