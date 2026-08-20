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
#include <string.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include <arch/board/board.h>

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

int fmuv6c_sdmmc_get_status(struct fmuv6c_sdmmc_status_s *status,
                            bool reset)
{
  struct stm32_sdmmc_stats_s stats;
  int ret;

  if (status == NULL)
    {
      return -EINVAL;
    }

  if (g_sdio_dev == NULL)
    {
      return -ENODEV;
    }

  ret = stm32_sdmmc_getstats(g_sdio_dev, &stats, reset);
  if (ret < 0)
    {
      return ret;
    }

  memset(status, 0, sizeof(*status));
  status->clkcr = stats.clkcr;
  status->status = stats.status;
  status->idmactrl = stats.idmactrl;
  status->read_transfers = stats.read_transfers;
  status->write_transfers = stats.write_transfers;
  status->read_bytes = stats.read_bytes;
  status->write_bytes = stats.write_bytes;
  status->bounced_reads = stats.bounced_reads;
  status->data_crc_errors = stats.data_crc_errors;
  status->data_timeouts = stats.data_timeouts;
  status->rx_overruns = stats.rx_overruns;
  status->tx_underruns = stats.tx_underruns;
  return OK;
}

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
