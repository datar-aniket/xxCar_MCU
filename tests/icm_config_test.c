/* SPDX-License-Identifier: Apache-2.0 */
#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include "arm_internal.h"
extern int printf(const char *, ...);
extern int puts(const char *);
extern void exit(int);
#define CHECK(x) do { if (!(x)) { \
  printf("FAIL %d: %s\n", __LINE__, #x); exit(1); } } while (0)
static uint8_t registers[3][128], bank, reg, command;
static int drop_reg = -1;
static unsigned transfers, selected, lock_depth;
static int lock(struct spi_dev_s *d, bool take)
{ (void)d; if (take) lock_depth++; else lock_depth--; return 0; }
static void select_dev(struct spi_dev_s *d, uint32_t id, bool take)
{ (void)d; (void)id; CHECK(lock_depth == 1); selected = take; command = 0; }
static uint32_t freq(struct spi_dev_s *d, uint32_t hz) { (void)d; return hz; }
static void mode(struct spi_dev_s *d, enum spi_mode_e m) { (void)d; (void)m; }
static void bits(struct spi_dev_s *d, int b) { (void)d; CHECK(b == 8); }
static uint32_t send(struct spi_dev_s *d, uint32_t value)
{
  (void)d; CHECK(selected);
  if (!command) { reg = value & 0x7f; command = 1; }
  else if (reg == 0x76) bank = value;
  else if (reg != drop_reg) registers[bank][reg] = value;
  transfers++;
  return 0;
}
static void exchange(struct spi_dev_s *d, const void *tx, void *rx, size_t n)
{
  (void)d; CHECK(tx == NULL && rx != NULL && selected);
  for (size_t i = 0; i < n; i++) ((uint8_t *)rx)[i] = registers[bank][reg++];
}
static int sleep_us(useconds_t us) { (void)us; return 0; }
static const struct spi_ops_s ops = {
  .lock=lock, .select=select_dev, .setfrequency=freq,
  .setmode=mode, .setbits=bits, .send=send, .exchange=exchange
};
#undef enter_critical_section
#undef leave_critical_section
#define enter_critical_section() (0)
#define leave_critical_section(f) ((void)(f))
#define nxsig_usleep(us) sleep_us(us)
#include "../boards/fmuv6c/src/icm42688.c"

int main(void)
{
  struct spi_dev_s spi = {.ops=&ops};
  struct icm42688_dev_s dev = {.spi=&spi};
  registers[0][ICM_REG_WHO_AM_I] = ICM_WHO_AM_I_VAL;
  CHECK(icm42688_configure(&dev) == 0);
  CHECK(bank == 0 && selected == 0 && lock_depth == 0);
  static const uint8_t critical[] = {
    ICM_REG_PWR_MGMT0, ICM_REG_GYRO_CONFIG0, ICM_REG_ACCEL_CONFIG0,
    ICM_REG_TMST_CONFIG, ICM_REG_FIFO_CONFIG1, ICM_REG_FIFO_CONFIG2
  };
  for (unsigned i = 0; i < sizeof(critical); i++)
    {
      drop_reg = critical[i];
      registers[0][drop_reg] = 0;
      CHECK(icm42688_configure(&dev) == -EIO);
      CHECK(bank == 0 && selected == 0 && lock_depth == 0);
      drop_reg = -1;
      CHECK(icm42688_configure(&dev) == 0);
    }
  registers[0][ICM_REG_WHO_AM_I] = 0xff;
  CHECK(icm42688_configure(&dev) == -ENODEV);
  CHECK(transfers > 0);
  puts("ICM42688: production config rejects identity/timing/stream faults PASS");
}
