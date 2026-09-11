/****************************************************************************
 * boards/fmuv6c/src/icm42688.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ICM-42688-P primary IMU uorb driver registration.
 ****************************************************************************/

#ifndef __BOARDS_FMUV6C_SRC_ICM42688_H
#define __BOARDS_FMUV6C_SRC_ICM42688_H

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Name: icm42688_register
 *
 * Description:
 *   Probe (verify WHO_AM_I) and register an ICM-42688-P as sensor_accel<devno>
 *   and sensor_gyro<devno>. Samples are read from the hardware FIFO. A nonzero
 *   drdy_gpio enables watermark-interrupt draining; zero selects the fixed
 *   polling fallback.
 *
 * Input Parameters:
 *   spi       - SPI master the device is on
 *   devno     - sensor device number (0 -> sensor_accel0 + sensor_gyro0)
 *   devid     - board-specific SPI device identifier
 *   drdy_gpio - data-ready GPIO configuration, or zero if not routed
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 ****************************************************************************/

int icm42688_register(FAR struct spi_dev_s *spi, int devno,
                      uint32_t devid, uint32_t drdy_gpio);

#endif /* __BOARDS_FMUV6C_SRC_ICM42688_H */
