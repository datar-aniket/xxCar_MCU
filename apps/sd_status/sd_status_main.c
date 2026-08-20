/****************************************************************************
 * apps/sd_status/sd_status_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#include <arch/board/board.h>

#define SD_STATUS_BLOCKDEV "/dev/mmcsd0"
#define SDMMC_CLKDIV_MASK  0x000003ffu
#define SDMMC_PWRSAV       (1u << 12)
#define SDMMC_WIDBUS_SHIFT 14
#define SDMMC_WIDBUS_MASK  (3u << SDMMC_WIDBUS_SHIFT)
#define SDMMC_NEGEDGE      (1u << 16)
#define SDMMC_IDMAEN       (1u << 0)
#define SDMMC_DPSMACT      (1u << 12)

static const char *sd_status_width(uint32_t clkcr)
{
  switch ((clkcr & SDMMC_WIDBUS_MASK) >> SDMMC_WIDBUS_SHIFT)
    {
      case 0:
        return "1-bit";
      case 1:
        return "4-bit";
      case 2:
        return "8-bit";
      default:
        return "reserved";
    }
}

static void sd_status_geometry(void)
{
  struct geometry geometry;
  uint64_t capacity;
  int fd;

  fd = open(SD_STATUS_BLOCKDEV, O_RDONLY);
  if (fd < 0)
    {
      printf("card: %s open failed: %s\n", SD_STATUS_BLOCKDEV,
             strerror(errno));
      return;
    }

  memset(&geometry, 0, sizeof(geometry));
  if (ioctl(fd, BIOC_GEOMETRY,
            (unsigned long)(uintptr_t)&geometry) < 0)
    {
      printf("card: geometry failed: %s\n", strerror(errno));
      close(fd);
      return;
    }

  capacity = (uint64_t)geometry.geo_nsectors * geometry.geo_sectorsize;
  printf("card: available=%s write=%s sector=%" PRIu32
         " count=%" PRIu64 " capacity=%.1f MiB",
         geometry.geo_available ? "yes" : "no",
         geometry.geo_writeenabled ? "yes" : "no",
         (uint32_t)geometry.geo_sectorsize,
         (uint64_t)geometry.geo_nsectors,
         (double)capacity / (1024.0 * 1024.0));

  if (geometry.geo_model[0] != '\0')
    {
      printf(" model=%s", geometry.geo_model);
    }

  printf("\n");
  close(fd);
}

int main(int argc, char *argv[])
{
  struct fmuv6c_sdmmc_status_s status;
  uint32_t divider;
  uint32_t clock_hz;
  bool reset = false;
  int ch;
  int ret;

  while ((ch = getopt(argc, argv, "r")) != EOF)
    {
      if (ch == 'r')
        {
          reset = true;
        }
      else
        {
          printf("usage: sd_status [-r]\n");
          return EXIT_FAILURE;
        }
    }

  ret = fmuv6c_sdmmc_get_status(&status, reset);
  if (ret < 0)
    {
      printf("sd_status: controller unavailable: %d\n", ret);
      return EXIT_FAILURE;
    }

  divider = status.clkcr & SDMMC_CLKDIV_MASK;
  clock_hz = divider == 0 ? 0 : STM32_PLL1Q_FREQUENCY / (2u * divider);

  printf("SDMMC2: clkcr=%08" PRIx32 " width=%s clock=%" PRIu32
         " Hz divider=%" PRIu32 " edge=%s power-save=%s\n",
         status.clkcr, sd_status_width(status.clkcr), clock_hz, divider,
         (status.clkcr & SDMMC_NEGEDGE) ? "falling" : "rising",
         (status.clkcr & SDMMC_PWRSAV) ? "on" : "off");
  printf("IDMA: enabled=%s data_path_active=%s status=%08" PRIx32 "\n",
         (status.idmactrl & SDMMC_IDMAEN) ? "yes" : "no",
         (status.status & SDMMC_DPSMACT) ? "yes" : "no",
         status.status);
  sd_status_geometry();
  printf("transfers: read=%" PRIu32 " (%" PRIu64 " bytes)"
         " write=%" PRIu32 " (%" PRIu64 " bytes) bounced_read=%" PRIu32
         "\n",
         status.read_transfers, status.read_bytes,
         status.write_transfers, status.write_bytes,
         status.bounced_reads);
  printf("errors: crc=%" PRIu32 " timeout=%" PRIu32
         " rx_overrun=%" PRIu32 " tx_underrun=%" PRIu32 "\n",
         status.data_crc_errors, status.data_timeouts,
         status.rx_overruns, status.tx_underruns);

  if (reset)
    {
      printf("counters reset after snapshot\n");
    }

  return EXIT_SUCCESS;
}
