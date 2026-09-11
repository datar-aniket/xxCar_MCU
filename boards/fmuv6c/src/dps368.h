/****************************************************************************
 * Lean DPS368 barometer uORB driver.
 ****************************************************************************/

#ifndef __BOARDS_FMUV6C_SRC_DPS368_H
#define __BOARDS_FMUV6C_SRC_DPS368_H

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#define DPS368_I2C_ADDR 0x76

int dps368_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr);

#endif
