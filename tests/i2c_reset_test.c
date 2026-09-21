/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define OK 0
#define DEBUGASSERT assert
#define MKI2C_OUTPUT(pin) (pin)
struct i2c_master_s { int unused; };
struct config_s { uint32_t scl_pin, sda_pin; };
struct stm32_i2c_priv_s
{
  int refs, lock;
  uint32_t frequency;
  struct config_s *config;
};
struct stm32_i2c_inst_s { struct i2c_master_s base; struct stm32_i2c_priv_s *priv; };
static bool stuck_sda, stuck_scl;
static unsigned restores, inits, deinits, clocks, locked;
static int init_error;
static int nxmutex_lock(int *m) { (void)m; locked++; return 0; }
static void nxmutex_unlock(int *m) { (void)m; assert(locked == 1); locked--; }
static void stm32_i2c_deinit(struct stm32_i2c_priv_s *p) { (void)p; deinits++; }
static int stm32_i2c_init(struct stm32_i2c_priv_s *p) { (void)p; inits++; return init_error; }
static void stm32_i2c_setclock(struct stm32_i2c_priv_s *p, uint32_t f)
{ assert(p->frequency == f); clocks++; }
static void stm32_configgpio(uint32_t p) { assert(p == 1 || p == 2); }
static void stm32_unconfiggpio(uint32_t p) { assert(p == 1 || p == 2); restores++; }
static void stm32_gpiowrite(uint32_t p, int v) { (void)p; (void)v; }
static bool stm32_gpioread(uint32_t p) { return p == 1 ? !stuck_scl : !stuck_sda; }
static void up_udelay(unsigned us) { assert(us == 10); }
#include "i2c_reset_under_test.h"

int main(void)
{
  struct config_s cfg = {1, 2};
  struct stm32_i2c_priv_s p = {.refs=1, .frequency=400000, .config=&cfg};
  struct stm32_i2c_inst_s bus = {.priv=&p};
  for (unsigned i = 0; i < 4; i++)
    {
      stuck_sda = i != 0;
      stuck_scl = i == 2;
      init_error = i == 3 ? -ENODEV : 0;
      restores = inits = deinits = clocks = 0;
      int result = stm32_i2c_reset(&bus.base);
      assert(result == (i == 0 ? 0 : i == 3 ? -ENODEV : -EIO));
      assert(restores == 2 && inits == 1 && deinits == 1 && locked == 0);
      assert(clocks == (i == 3 ? 0u : 1u));
    }
  puts("I2C reset: normal, stuck SDA/SCL, re-init failure cleanup PASS");
}
