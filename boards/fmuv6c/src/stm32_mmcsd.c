/****************************************************************************
 * boards/fmuv6c/src/stm32_mmcsd.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * microSD card on SDMMC2 (4-bit SDIO). Binds the SDMMC peripheral to the
 * NuttX MMC/SD block driver -> /dev/mmcsd0, which stm32_bringup() then mounts
 * as FAT at /fs/microsd (logs, parameters, config).
 *
 * The FMUv6C has NO card-detect line on the microSD slot, so we simply report
 * the card as present after binding. A card that is absent (or pulled) shows
 * up as an I/O error on access rather than a media-change event.
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "stm32_sdmmc.h"
#include "fmuv6c.h"

#ifdef CONFIG_MMCSD_SDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only SDMMC2 is enabled, so the driver numbers its slot 0. */

#define SDIO_SLOTNO 0
#define SDIO_MINOR  0

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct sdio_dev_s *g_sdio_dev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDMMC2 and bind it to the MMC/SD block driver, creating
 *   /dev/mmcsd0.
 *
 ****************************************************************************/

int stm32_sdio_initialize(void)
{
  int ret;

  g_sdio_dev = sdio_initialize(SDIO_SLOTNO);
  if (g_sdio_dev == NULL)
    {
      mcerr("ERROR: Failed to initialize SDMMC2 (slot %d)\n", SDIO_SLOTNO);
      return -ENODEV;
    }

  ret = mmcsd_slotinitialize(SDIO_MINOR, g_sdio_dev);
  if (ret < 0)
    {
      mcerr("ERROR: Failed to bind SDMMC2 to MMC/SD driver: %d\n", ret);
      return ret;
    }

  /* No card-detect line on this board: assume the card is inserted so the
   * MMC/SD driver probes it now.
   */

  sdio_mediachange(g_sdio_dev, true);
  return OK;
}

#endif /* CONFIG_MMCSD_SDIO */
