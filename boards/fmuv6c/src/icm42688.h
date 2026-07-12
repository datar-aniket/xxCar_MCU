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
 *   and sensor_gyro<devno>. Polled sampling: one SPI burst per sample read in a
 *   kthread that sleeps the sample interval (>= 1 system tick), no CPU spin.
 *   FIFO + DRDY interrupt streaming at 2 kHz is a later stage.
 *
 * Input Parameters:
 *   spi   - SPI master the device is on (SPI1 on the FMUv6C)
 *   devno - sensor device number (0 -> sensor_accel0 + sensor_gyro0)
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 ****************************************************************************/

int icm42688_register(FAR struct spi_dev_s *spi, int devno);

#endif /* __BOARDS_FMUV6C_SRC_ICM42688_H */
