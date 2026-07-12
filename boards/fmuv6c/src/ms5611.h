/****************************************************************************
 * boards/fmuv6c/src/ms5611.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Lean MS5611 barometer uorb driver registration.
 ****************************************************************************/

#ifndef __BOARDS_FMUV6C_SRC_MS5611_H
#define __BOARDS_FMUV6C_SRC_MS5611_H

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#define MS5611_I2C_ADDR 0x77

/****************************************************************************
 * Name: ms5611_register
 *
 * Description:
 *   Probe (read PROM) and register an MS5611 barometer as sensor_baro<devno>.
 *   Low-CPU: sleep-only conversion waits, temperature decimated.
 *
 * Input Parameters:
 *   i2c   - I2C master the device is on
 *   devno - sensor device number (0 -> sensor_baro0)
 *   addr  - 7-bit I2C address (0x77 on the FMUv6C)
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 ****************************************************************************/

int ms5611_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr);

#endif /* __BOARDS_FMUV6C_SRC_MS5611_H */
