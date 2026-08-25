/****************************************************************************
 * boards/fmuv6c/src/fdcan_ring.h
 *
 * Index arithmetic for the FDCAN receive ring.
 *
 * Its own header because the ring has exactly one producer - the interrupt
 * handler - and exactly one consumer - the daemon task - and that is what
 * makes it safe without a lock. The ISR only ever advances `head`; the task
 * only ever advances `tail`; each reads the other's index but never writes
 * it. Both are word-sized and aligned, so a read can see the old value or
 * the new one and nothing in between.
 *
 * The arithmetic is separated out because an off-by-one here does not crash.
 * It silently drops a frame, or hands out one that was already consumed, and
 * on a telemetry stream either looks like noise. So it is host-tested.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __BOARDS_FMUV6C_SRC_FDCAN_RING_H
#define __BOARDS_FMUV6C_SRC_FDCAN_RING_H

#include <stdbool.h>
#include <stdint.h>

/* Slots in the ring. MUST be a power of two: the wrap is a mask, not a
 * modulo, because the ISR does this on every frame.
 *
 * One slot is always left empty so that full and empty are distinguishable
 * from the two indices alone - which is what lets the ISR and the task each
 * own one index and never share a lock. Usable capacity is therefore 31.
 *
 * Sized against the 64-element hardware FIFO rather than against the traffic:
 * the ISR drains the FIFO completely on every interrupt, so the ring only has
 * to absorb what arrives between one drain and the task's next wake-up. At
 * tens of frames a second against a task that wakes every millisecond, 31 is
 * already several hundred times more than needed.
 */

#define FDCAN_RING_N      32u
#define FDCAN_RING_MASK   (FDCAN_RING_N - 1u)
#define FDCAN_RING_CAP    (FDCAN_RING_N - 1u)

static inline uint32_t fdcan_ring_next(uint32_t index)
{
  return (index + 1u) & FDCAN_RING_MASK;
}

static inline bool fdcan_ring_empty(uint32_t head, uint32_t tail)
{
  return head == tail;
}

static inline bool fdcan_ring_full(uint32_t head, uint32_t tail)
{
  return fdcan_ring_next(head) == tail;
}

static inline uint32_t fdcan_ring_count(uint32_t head, uint32_t tail)
{
  return (head - tail) & FDCAN_RING_MASK;
}

#endif /* __BOARDS_FMUV6C_SRC_FDCAN_RING_H */
