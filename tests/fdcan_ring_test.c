/****************************************************************************
 * tests/fdcan_ring_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The FDCAN receive ring's index arithmetic.
 *
 * An off-by-one here does not crash. It drops a frame, or re-delivers one
 * already consumed, and on a telemetry stream both look like noise rather
 * than like a bug. The indices are also the whole reason the ISR and the
 * task need no lock between them, so "full" and "empty" have to stay
 * distinguishable at every point in the wrap.
 ****************************************************************************/

#include <assert.h>
#include <stdio.h>

#include "fdcan_ring.h"

static int g_failures;

#define CHECK(cond, ...)                                   \
  do                                                       \
    {                                                      \
      if (!(cond))                                         \
        {                                                  \
          printf("FAIL %s:%d: ", __FILE__, __LINE__);      \
          printf(__VA_ARGS__);                             \
          printf("\n");                                    \
          g_failures++;                                    \
        }                                                  \
    }                                                      \
  while (0)

static void test_power_of_two(void)
{
  /* The wrap is a mask. With a non-power-of-two size the mask silently
   * addresses the wrong slots instead of failing.
   */

  CHECK((FDCAN_RING_N & (FDCAN_RING_N - 1u)) == 0,
        "FDCAN_RING_N %u is not a power of two", FDCAN_RING_N);
  CHECK(FDCAN_RING_MASK == FDCAN_RING_N - 1u, "mask does not match size");
  CHECK(FDCAN_RING_CAP == FDCAN_RING_N - 1u,
        "capacity must be one less than the slot count");
}

static void test_starts_empty(void)
{
  CHECK(fdcan_ring_empty(0, 0), "a fresh ring must read empty");
  CHECK(!fdcan_ring_full(0, 0), "a fresh ring must not read full");
  CHECK(fdcan_ring_count(0, 0) == 0, "a fresh ring must hold nothing");
}

/* Push and pop one at a time all the way round twice, checking the count
 * agrees at every step. Twice, because a wrap bug that only bites after the
 * indices pass FDCAN_RING_N would survive a single lap.
 */

static void test_wraps(void)
{
  uint32_t head = 0;
  uint32_t tail = 0;
  uint32_t i;

  for (i = 0; i < FDCAN_RING_N * 2u + 3u; i++)
    {
      CHECK(fdcan_ring_empty(head, tail), "should be empty at step %u", i);
      head = fdcan_ring_next(head);
      CHECK(fdcan_ring_count(head, tail) == 1,
            "one entry expected at step %u, got %u", i,
            fdcan_ring_count(head, tail));
      CHECK(!fdcan_ring_empty(head, tail), "not empty at step %u", i);
      tail = fdcan_ring_next(tail);
    }

  CHECK(fdcan_ring_empty(head, tail), "must end empty");
}

/* Fill to capacity and confirm the boundary is where it is claimed: the last
 * accepted push leaves the ring full, and full is never mistaken for empty.
 */

static void test_fills_to_capacity(void)
{
  uint32_t head = 0;
  uint32_t tail = 0;
  uint32_t i;

  for (i = 0; i < FDCAN_RING_CAP; i++)
    {
      CHECK(!fdcan_ring_full(head, tail),
            "must accept entry %u of %u", i, FDCAN_RING_CAP);
      head = fdcan_ring_next(head);
    }

  CHECK(fdcan_ring_full(head, tail),
        "must be full after %u entries", FDCAN_RING_CAP);
  CHECK(!fdcan_ring_empty(head, tail), "full must not read as empty");
  CHECK(fdcan_ring_count(head, tail) == FDCAN_RING_CAP,
        "a full ring must count %u, got %u", FDCAN_RING_CAP,
        fdcan_ring_count(head, tail));

  /* Consuming one makes room for exactly one. */

  tail = fdcan_ring_next(tail);
  CHECK(!fdcan_ring_full(head, tail), "one pop must make room");
  head = fdcan_ring_next(head);
  CHECK(fdcan_ring_full(head, tail), "and exactly one");
}

/* The same boundary, reached at every possible starting offset. A mask bug
 * often works from zero and fails from somewhere in the middle.
 */

static void test_capacity_at_every_offset(void)
{
  uint32_t start;

  for (start = 0; start < FDCAN_RING_N; start++)
    {
      uint32_t head = start;
      uint32_t tail = start;
      uint32_t i;

      for (i = 0; i < FDCAN_RING_CAP; i++)
        {
          CHECK(!fdcan_ring_full(head, tail),
                "offset %u: must accept entry %u", start, i);
          head = fdcan_ring_next(head);
        }

      CHECK(fdcan_ring_full(head, tail), "offset %u: must be full", start);
      CHECK(fdcan_ring_count(head, tail) == FDCAN_RING_CAP,
            "offset %u: count wrong", start);
    }
}

int main(void)
{
  test_power_of_two();
  test_starts_empty();
  test_wraps();
  test_fills_to_capacity();
  test_capacity_at_every_offset();

  if (g_failures != 0)
    {
      printf("%d failure(s)\n", g_failures);
      return 1;
    }

  printf("fdcan_ring: wrap and capacity verified at every offset\n");
  return 0;
}
