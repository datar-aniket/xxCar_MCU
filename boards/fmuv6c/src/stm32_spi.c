/****************************************************************************
 * boards/fmuv6c/src/stm32_spi.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * FMUv6C SPI1 = internal IMU bus (ICM-42688-P + BMI088). Configures the
 * chip-select / DRDY GPIOs and provides the board-level SPI1 select/status
 * hooks the STM32H7 SPI driver calls to assert each sensor's CS.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"

#include "fmuv6c.h"

#if defined(CONFIG_STM32H7_SPI1)

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Configure the SPI1 chip-select and DRDY GPIOs. All chip selects start
 *   de-asserted (high). Called from board bring-up.
 ****************************************************************************/

void stm32_spidev_initialize(void)
{
  stm32_configgpio(GPIO_SPI1_CS_ICM42688);
  stm32_configgpio(GPIO_SPI1_CS_BMI088_ACCEL);
  stm32_configgpio(GPIO_SPI1_CS_BMI088_GYRO);

  stm32_configgpio(GPIO_DRDY_ICM42688);
  stm32_configgpio(GPIO_DRDY_BMI088_ACCEL);
  stm32_configgpio(GPIO_DRDY_BMI088_GYRO);
}

/****************************************************************************
 * Name: stm32_spi1select / stm32_spi1status
 *
 * Description:
 *   The STM32H7 SPI driver calls these to assert/query the chip select for
 *   the device identified by devid. CS is active-low, so we write !selected.
 ****************************************************************************/

void stm32_spi1select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  switch (devid)
    {
      case SPIDEV_IMU(FMUV6C_SPIDEV_ICM42688):
        stm32_gpiowrite(GPIO_SPI1_CS_ICM42688, !selected);
        break;

      case SPIDEV_ACCELEROMETER(FMUV6C_SPIDEV_BMI088_ACCEL):
        stm32_gpiowrite(GPIO_SPI1_CS_BMI088_ACCEL, !selected);
        break;

      case SPIDEV_ACCELEROMETER(FMUV6C_SPIDEV_BMI088_GYRO):
        stm32_gpiowrite(GPIO_SPI1_CS_BMI088_GYRO, !selected);
        break;

      default:
        break;
    }
}

uint8_t stm32_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#endif /* CONFIG_STM32H7_SPI1 */
