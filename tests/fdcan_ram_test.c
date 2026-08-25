/****************************************************************************
 * tests/fdcan_ram_test.c
 *
 * The FDCAN message RAM regions are placed by software. Overlap them and
 * the hardware does not complain: filter elements get overwritten by
 * incoming frames, frames get overwritten by filters, and the symptom is
 * intermittent garbage on the bus. None of that is visible at build time,
 * so the placement arithmetic is checked here.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <stdio.h>
#include <stdint.h>

#include "fdcan_ram.h"

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

struct region_s
{
  const char *name;
  uint32_t start;
  uint32_t end;      /* exclusive */
};

static const struct region_s g_regions[] =
{
  {
    "stdfilt", FDCAN_RAM_STDFILT_OFF,
    FDCAN_RAM_STDFILT_OFF + FDCAN_RAM_STDFILT_N * FDCAN_RAM_STDFILT_WORDS
  },
  {
    "extfilt", FDCAN_RAM_EXTFILT_OFF,
    FDCAN_RAM_EXTFILT_OFF + FDCAN_RAM_EXTFILT_N * FDCAN_RAM_EXTFILT_WORDS
  },
  {
    "rxf0", FDCAN_RAM_RXF0_OFF,
    FDCAN_RAM_RXF0_OFF + FDCAN_RAM_RXF0_N * FDCAN_RAM_RXF0_WORDS
  },
  {
    "txf", FDCAN_RAM_TXF_OFF,
    FDCAN_RAM_TXF_OFF + FDCAN_RAM_TXF_N * FDCAN_RAM_TXF_WORDS
  },
};

#define NREGIONS (sizeof(g_regions) / sizeof(g_regions[0]))

static void test_no_overlap(void)
{
  size_t i;
  size_t j;

  for (i = 0; i < NREGIONS; i++)
    {
      for (j = i + 1; j < NREGIONS; j++)
        {
          CHECK(g_regions[i].end <= g_regions[j].start ||
                g_regions[j].end <= g_regions[i].start,
                "%s [%u,%u) overlaps %s [%u,%u)",
                g_regions[i].name, g_regions[i].start, g_regions[i].end,
                g_regions[j].name, g_regions[j].start, g_regions[j].end);
        }
    }
}

static void test_fits_in_our_half(void)
{
  size_t i;
  uint32_t high = 0;

  for (i = 0; i < NREGIONS; i++)
    {
      CHECK(g_regions[i].end <= FDCAN_RAM_FDCAN1_WORDS,
            "%s ends at %u, past FDCAN1's %u words - it is inside FDCAN2's "
            "half", g_regions[i].name, g_regions[i].end,
            FDCAN_RAM_FDCAN1_WORDS);

      if (g_regions[i].end > high)
        {
          high = g_regions[i].end;
        }
    }

  /* FDCAN_RAM_USED drives the clear loop at init. If it is short of the
   * real high-water mark, part of the layout is left holding whatever the
   * last boot put there - and an uninitialised filter element IS a filter.
   */

  CHECK(FDCAN_RAM_USED >= high,
        "FDCAN_RAM_USED is %u but the layout reaches %u; the clear loop "
        "would leave stale words in use", FDCAN_RAM_USED, high);

  CHECK(FDCAN_RAM_FDCAN1_WORDS * 2 == FDCAN_RAM_TOTAL_WORDS,
        "the two halves do not add up to the %u words the chip has",
        FDCAN_RAM_TOTAL_WORDS);
}

static void test_field_widths(void)
{
  /* Each element-count register field is narrow. A count that does not fit
   * is truncated on the way in, so the driver would believe it had a FIFO
   * deeper than the hardware was told about.
   */

  CHECK(FDCAN_RAM_STDFILT_N <= 128u, "SIDFC.LSS holds at most 128");
  CHECK(FDCAN_RAM_EXTFILT_N <= 64u, "XIDFC.LSE holds at most 64");
  CHECK(FDCAN_RAM_RXF0_N <= 64u, "RXF0C.F0S holds at most 64");
  CHECK(FDCAN_RAM_TXF_N <= 32u, "TXBC.TFQS holds at most 32");

  /* Start addresses go into 14-bit word-address fields at bit 2. */

  CHECK(FDCAN_RAM_TXF_OFF < 16384u, "start address exceeds the 14-bit field");
}

int main(void)
{
  test_no_overlap();
  test_fits_in_our_half();
  test_field_widths();

  if (g_failures != 0)
    {
      printf("%d failure(s)\n", g_failures);
      return 1;
    }

  printf("fdcan_ram: all checks passed\n");
  return 0;
}
