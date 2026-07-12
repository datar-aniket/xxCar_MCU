/****************************************************************************
 * boards/fmuv6c/src/bmi055.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BMI055 secondary IMU uorb driver registration.
 ****************************************************************************/

#ifndef __BOARDS_FMUV6C_SRC_BMI055_H
#define __BOARDS_FMUV6C_SRC_BMI055_H

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Name: bmi055_register
 *
 * Description:
 *   Probe and register the BMI055 secondary IMU. The BMI055 is two separate
 *   dies in one package with their own chip selects and interrupt lines:
 *
 *     accel (BMA2x2 class) : CS PC15, INT1 -> PE4 -> sensor_accel<devno>
 *     gyro  (BMG160 class) : CS PC14, INT1 -> PE5 -> sensor_gyro<devno>
 *
 *   Both stream at 2 kHz via their hardware FIFOs with a watermark interrupt,
 *   the same architecture as the primary ICM-42688-P.
 *
 * Input Parameters:
 *   spi   - SPI master both dies are on (SPI1 on the FMUv6C)
 *   devno - sensor device number (1 -> sensor_accel1 + sensor_gyro1)
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 ****************************************************************************/

int bmi055_register(FAR struct spi_dev_s *spi, int devno);

#endif /* __BOARDS_FMUV6C_SRC_BMI055_H */
