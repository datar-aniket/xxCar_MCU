/****************************************************************************
 * boards/fmuv6c/include/fdcan.h
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
 * Receive plus a minimal transmit path. Transmit was deliberately left out
 * of the first version so that a driver still proving its bit timing could
 * not spin a motor; that timing is now confirmed on hardware.
 *
 * Receive is INTERRUPT DRIVEN. There is no DMA option: FDCAN has no DMAMUX
 * request line on this part, and does not need one - the CAN core writes
 * received frames into its own message RAM with no CPU and no DMA channel.
 * The bytes have already moved by the time anything notices. Polling never
 * cost a copy; it cost the delay in finding out.
 *
 * Classic CAN, 8-byte frames. CAN FD would change the message RAM element
 * size from four words to eighteen and invalidate the layout in fdcan.c.
 ****************************************************************************/

#ifndef __BOARDS_FMUV6C_INCLUDE_FDCAN_H
#define __BOARDS_FMUV6C_INCLUDE_FDCAN_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

struct fdcan_frame_s
{
  /* When the interrupt handler took the frame out of the hardware FIFO, on
   * the same epoch as CLOCK_MONOTONIC but with microsecond resolution rather
   * than the 1 ms system tick. Set on receive; ignored on transmit.
   */

  uint64_t ts;

  uint32_t id;              /* 29-bit extended identifier */
  uint8_t  dlc;             /* 0..8 */
  uint8_t  data[8];
};

struct fdcan_stats_s
{
  uint32_t rx;              /* valid frames captured by the ISR */
  uint32_t lost;            /* RX FIFO0 overruns - not drained fast enough */
  uint32_t rejected;        /* non-extended or remote frames */
  uint32_t tx;              /* frames queued for transmission */
  uint32_t tx_full;         /* dropped: Tx FIFO had no free element */
  uint32_t ring_full;       /* dropped: the task did not keep up */
  uint8_t  last_error;      /* PSR LEC */
  bool     bus_off;
  bool     error_passive;
};

/* Bring up the peripheral. Only 1000000 is supported today; anything else is
 * refused rather than silently mis-timed.
 */

int fdcan_init(uint32_t bitrate);

/* Stop receive interrupts and put the peripheral back into INIT mode.
 * Safe to call after a partial initialization failure.
 */

void fdcan_deinit(void);

/* Take one frame from the receive ring. Returns OK, or -EAGAIN when the ring
 * is empty. Never blocks.
 *
 * The frame was moved out of the hardware FIFO and timestamped by the
 * interrupt handler, so `ts` is the arrival time and not the time this was
 * called.
 */

int fdcan_receive(FAR struct fdcan_frame_s *frame);

/* Block until a frame arrives or the timeout expires.
 *
 * Returns OK if woken by an arrival, -ETIMEDOUT on the timeout. Callers with
 * their own deadline - a transmit cadence, say - pass the time remaining and
 * treat both returns the same way: drain, then check the deadline.
 *
 * The timeout is rounded up to whole system ticks, which are 1 ms here.
 * Asking for less does not get less.
 */

int fdcan_wait(uint32_t timeout_us);

/* Queue one frame. Returns OK, -EAGAIN when the Tx FIFO is full, or -EINVAL
 * before init.
 *
 * "Queued", not "sent": this returns as soon as the element is in the FIFO
 * and the hardware has been told to send it.
 *
 * A FULL FIFO IS THE DIAGNOSTIC WORTH KNOWING. Classic CAN needs another
 * node to acknowledge, and the hardware retries a frame that is never
 * acknowledged for as long as it takes. So an absent VESC, or a missing bus
 * terminator, fills all 32 elements and every call after that returns
 * -EAGAIN - in about 80 ms at the default 400 Hz, or 0.6 s at 50. That
 * climbing tx_full is a precise symptom, and it is the first thing to look
 * at when a bench run does nothing.
 */

int fdcan_transmit(FAR const struct fdcan_frame_s *frame);

/* Narrow the HARDWARE filter to one controller id; 0 accepts any.
 *
 * In hardware rather than in software because an unfiltered 1 Mbit/s bus can
 * deliver over 8000 frames a second, and rejecting them in a task means
 * waking up 8000 times a second to throw work away.
 */

int  fdcan_set_filter(uint8_t controller_id);

void fdcan_stats(FAR struct fdcan_stats_s *out);

#endif /* __BOARDS_FMUV6C_INCLUDE_FDCAN_H */
