/****************************************************************************
 * boards/fmuv6c/src/ist8310.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IST8310 magnetometer uorb driver registration.
 ****************************************************************************/

#ifndef __BOARDS_FMUV6C_SRC_IST8310_H
#define __BOARDS_FMUV6C_SRC_IST8310_H

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#define IST8310_I2C_ADDR 0x0c

/****************************************************************************
 * Name: ist8310_register
 *
 * Description:
 *   Probe (verify WHO_AM_I) and register an IST8310 magnetometer as
 *   sensor_mag<devno>. Low-CPU: single-shot conversions waited on with
 *   nxsig_usleep only (no CPU spin), pipelined with the sample interval.
 *
 * Input Parameters:
 *   i2c   - I2C master the device is on (I2C4 on the FMUv6C)
 *   devno - sensor device number (0 -> sensor_mag0)
 *   addr  - 7-bit I2C address (0x0c on the FMUv6C)
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 ****************************************************************************/

int ist8310_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr);

#endif /* __BOARDS_FMUV6C_SRC_IST8310_H */
