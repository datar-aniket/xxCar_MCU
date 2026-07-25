/****************************************************************************
 * boards/fmuv6c/src/stm32_reset.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>   /* BOARD_REBOOT_TO_BOOTLOADER */

#include "arm_internal.h"
#include "hardware/stm32_pwr.h"
#include "hardware/stm32_rtcc.h"
#include "hardware/stm32_rcc.h"

#include "fmuv6c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Reboot-into-bootloader handshake.
 *
 * The board keeps PX4's bootloader in the first flash sector, intact. On every
 * reset that bootloader reads RTC backup register 0 (STM32_RTC_BK0R), and if it
 * finds the magic 0xb007b007 it stays in serial-upload mode instead of jumping
 * to our app - so px_uploader can flash without the unplug/replug dance. The
 * value and the register belong to the bootloader (PX4's board_reset.cpp writes
 * the same ones), so both are fixed.
 *
 * BOARD_REBOOT_TO_BOOTLOADER is the `status` our reboot command passes through
 * boardctl(BOARDIOC_RESET, ...); it is defined in board.h so the app and this
 * file share one value. Any other status is an ordinary reset.
 */

#define BOOTLOADER_MAGIC    0xb007b007

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_RESET

/****************************************************************************
 * Name: board_reset
 *
 * Description:
 *   Reset the board. Required by board-level logic when CONFIG_BOARDCTL_RESET
 *   is selected. `status` is board-specific; we use it to select an ordinary
 *   reset (any value) or a reset that leaves the board in the bootloader
 *   (BOARD_REBOOT_TO_BOOTLOADER).
 *
 ****************************************************************************/

int board_reset(int status)
{
  if (status == BOARD_REBOOT_TO_BOOTLOADER)
    {
      /* The RTC backup registers live in the backup domain, behind two gates
       * that this firmware otherwise leaves shut (it does not use the RTC, so
       * CONFIG_STM32H7_PWR - and stm32_pwr_enablebkp - are not even built):
       *
       *   1. the RTC APB clock (RCC_APB4ENR RTCAPBEN) - without it the register
       *      is not clocked and the write is silently dropped;
       *   2. the backup-domain write protection (PWR_CR1 DBP).
       *
       * Both are single bits, so set them directly rather than pull in the PWR
       * driver.
       *
       * The readbacks matter, and getting this wrong is why the first cut of
       * this failed - the magic never stuck and the bootloader booted straight
       * through. On the H7 there is a delay between enabling a peripheral clock
       * and the peripheral actually being clocked; the reference manual's fix is
       * to read the RCC register back before touching the peripheral (NuttX does
       * the same in its clock config). Without that, the write to BK0R below runs
       * before the RTC APB clock is live and is lost. The final readback of BK0R
       * flushes the write across the backup-domain clock boundary before the
       * reset, so it is committed by the time the bootloader looks.
       */

      modifyreg32(STM32_RCC_APB4ENR, 0, RCC_APB4ENR_RTCAPBEN);
      (void)getreg32(STM32_RCC_APB4ENR);     /* let the RTC APB clock come up */

      modifyreg32(STM32_PWR_CR1, 0, PWR_CR1_DBP);
      (void)getreg32(STM32_PWR_CR1);         /* DBP effective before the write */

      putreg32(BOOTLOADER_MAGIC, STM32_RTC_BK0R);
      (void)getreg32(STM32_RTC_BK0R);        /* flush it out before we reset */

      UP_DSB();
    }

  up_systemreset();
  return 0;
}

#endif /* CONFIG_BOARDCTL_RESET */
