/* SPDX-License-Identifier: Apache-2.0
 * Production FDCAN TX/recovery code against an asynchronous register model.
 * Not an electrical CAN/controller implementation or hardware qualification.
 */
#include <nuttx/config.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/semaphore.h>
#include <arch/board/board.h>
#include "arm_internal.h"
#include "stm32_gpio.h"
#include "hardware/stm32_fdcan.h"
#include "hardware/stm32_rcc.h"
#include "../boards/fmuv6c/src/fmuv6c.h"
extern int printf(const char *, ...);
extern int puts(const char *);
extern void exit(int);
#define CHECK(x) do { if (!(x)) { \
  printf("FAIL %d: %s\n", __LINE__, #x); exit(1); } } while (0)

static uint32_t regs[128], ram[1280];
static uint64_t now;
static unsigned depth, cancellations, starts, watchdogs;
#define REG(a) regs[((a) - STM32_FDCAN1_BASE) / 4]

static uint32_t *address(uint32_t a)
{
  if (a >= STM32_FDCAN1_BASE && a < STM32_FDCAN1_BASE + sizeof(regs))
    return &REG(a);
  if (a >= STM32_CANRAM_BASE && a < STM32_CANRAM_BASE + sizeof(ram))
    return &ram[(a - STM32_CANRAM_BASE) / 4];
  CHECK(false); /* No access to another controller or shared reset. */
  return NULL;
}

static uint32_t fake_read(uint32_t a) { return *address(a); }
static void fake_write(uint32_t v, uint32_t a)
{
  CHECK(depth > 0);
  if (a == STM32_FDCAN1_TXBAR)
    {
      CHECK(REG(STM32_FDCAN1_TXBRP) == 0); /* One command, no backlog. */
      REG(STM32_FDCAN1_TXBRP) |= v;
      REG(STM32_FDCAN1_TXBTO) &= ~v;
      REG(STM32_FDCAN1_TXBCF) &= ~v;
      starts++;
    }
  else if (a == STM32_FDCAN1_TXBCR)
    {
      CHECK((v & ~REG(STM32_FDCAN1_TXBRP)) == 0);
      cancellations++;
      /* Cancellation is asynchronous: leave pending set. */
    }
  *address(a) = v;
}
static void fake_modify(uint32_t a, uint32_t c, uint32_t s)
{ fake_write((fake_read(a) & ~c) | s, a); }
static irqstate_t fake_enter(void) { return depth++; }
static void fake_leave(irqstate_t f) { CHECK(depth == f + 1); depth = f; }
static int fake_wd(struct wdog_s *w, clock_t d, wdentry_t e, wdparm_t p)
{ (void)w; (void)e; (void)p; CHECK(d > 0); watchdogs++; return 0; }

#undef getreg32
#undef putreg32
#undef enter_critical_section
#undef leave_critical_section
#undef memory_barrier
#undef UNUSED
#define UNUSED(a) ((void)(a))
#define getreg32(a) fake_read(a)
#define putreg32(v, a) fake_write(v, a)
#define modifyreg32(a, c, s) fake_modify(a, c, s)
#define enter_critical_section() fake_enter()
#define leave_critical_section(f) fake_leave(f)
#define memory_barrier() __asm__ volatile ("" ::: "memory")
#define fmuv6c_imu_time_now() (now)
#define wd_start(w,d,e,p) fake_wd(w,d,e,p)
#include "../boards/fmuv6c/src/fdcan.c"

static void service(void)
{
  irqstate_t f = fake_enter();
  fdcan_maintenance(0);
  fake_leave(f);
}

static void finish(bool sent)
{
  uint32_t bits = REG(STM32_FDCAN1_TXBRP);
  if (sent) REG(STM32_FDCAN1_TXBTO) |= bits;
  else REG(STM32_FDCAN1_TXBCF) |= bits;
  REG(STM32_FDCAN1_TXBRP) = 0;
  REG(STM32_FDCAN1_TXFQS) =
    ((starts % 32) << FDCAN_TXFQS_TFQPI_SHIFT);
}

int main(void)
{
  struct fdcan_frame_s frame = {.id = 0x1234, .dlc = 4, .data = {1,2,3,4}};
  struct fdcan_stats_s status;
  CHECK(fdcan_transmit(&frame) == -EINVAL);
  g_ready = true;
  CHECK(fdcan_transmit(&frame) == 0);
  CHECK(g_tx_tracked == 1);
  now = 19999;
  service();
  CHECK(cancellations == 0);
  now = 20000;
  service();
  CHECK(cancellations == 1 && g_stats.tx_expired == 1);
  service();
  CHECK(cancellations == 1); /* Don't count repeated cancellation requests. */
  CHECK(fdcan_transmit(&frame) == -EAGAIN);
  CHECK(starts == 1); /* Cannot overwrite a still-pending message RAM slot. */
  finish(false);
  CHECK(fdcan_transmit(&frame) == 0);
  CHECK(g_stats.tx_cancelled == 1);
  finish(true);
  CHECK(fdcan_tx_idle());
  CHECK(g_stats.tx_completed == 1);

  CHECK(fdcan_transmit(&frame) == 0);
  fdcan_abort_tx();
  finish(true); /* An in-flight frame may finish despite cancellation. */
  service();
  CHECK(g_stats.tx_completed == 2 && g_stats.tx_cancelled == 1);

  CHECK(fdcan_transmit(&frame) == 0);
  REG(STM32_FDCAN1_PSR) = FDCAN_PSR_BO_MASK;
  REG(STM32_FDCAN1_CCCR) = FDCAN_CCCR_INIT;
  service();
  CHECK(g_stats.bus_off_count == 1);
  CHECK(!(REG(STM32_FDCAN1_CCCR) & FDCAN_CCCR_INIT));
  CHECK(fdcan_transmit(&frame) == -ENETDOWN);
  service();
  CHECK(g_stats.bus_off_count == 1); /* One count per episode. */
  finish(false);
  REG(STM32_FDCAN1_PSR) = 0;
  service();
  CHECK(g_stats.recoveries == 1);
  CHECK(fdcan_transmit(&frame) == 0);
  finish(true);
  service();
  for (unsigned i = 0; i < 2000; i++)
    {
      CHECK(fdcan_transmit(&frame) == 0);
      if (i % 3 == 0)
        {
          now += 30000;
          service();
        }
      finish(i % 3 != 0);
      service();
    }
  unsigned before = cancellations;
  fdcan_stats(&status);
  CHECK(cancellations == before); /* Diagnostics never mutate hardware. */
  CHECK(status.ready && status.pending == 0 && depth == 0);
  CHECK(status.tx == status.tx_completed + status.tx_cancelled);
  CHECK(watchdogs > 2000);
  puts("FDCAN: expiry, supersession, cancellation race, bus-off, wrap PASS");
  return 0;
}
