/****************************************************************************
 * boards/fmuv6c/src/fmuv6c_txtick.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Periodic tick for the companion downlink.
 *
 * TIM6 is a BASIC timer: no channels, no GPIO, no alternate function. It
 * exists to raise an update interrupt at a fixed rate and nothing else,
 * which is exactly what a fixed downlink cadence needs and is why it was
 * chosen over one of the general-purpose timers that can still do useful
 * work elsewhere.
 *
 * Claimed directly rather than through NuttX's timer lower-half, matching
 * how TIM5 (the IMU timebase) and TIM3 (the PPS capture) are already taken
 * on this board.
 *
 * This tick FREE-RUNS. It is not locked to the estimator, and at 200 Hz
 * against a 400 Hz estimator the phase between them slips continuously, so
 * the downlink will occasionally repeat a solution or step over two. That is
 * inherent to running the two off separate oscillators; the counters here
 * make it visible rather than letting it hide.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_rcc.h"
#include "hardware/stm32_tim.h"

#include "fmuv6c.h"

#ifdef CONFIG_XXCAR_COMPANION

#ifdef CONFIG_STM32H7_TIM6
#  error "TIM6 is reserved by the FMUv6C companion downlink tick"
#endif

/* Run the counter at 1 MHz so the period is simply the microsecond count,
 * the same convention TIM5 uses. At 1 MHz a 16-bit ARR reaches 65535 us, so
 * the slowest tick this can produce is about 15 Hz.
 */

#define TXTICK_COUNTER_HZ   1000000u
#define TXTICK_MIN_HZ       16u
#define TXTICK_MAX_HZ       1000u

#if (STM32_APB1_TIM6_CLKIN % TXTICK_COUNTER_HZ) != 0
#  error "TIM6 input clock must be an integer multiple of 1 MHz"
#endif

#if (STM32_APB1_TIM6_CLKIN / TXTICK_COUNTER_HZ) > 65536
#  error "TIM6 1 MHz prescaler does not fit in PSC"
#endif

static sem_t g_tick_sem;
static struct fmuv6c_txtick_status_s g_status;
static bool g_initialized;

static int fmuv6c_txtick_isr(int irq, FAR void *context, FAR void *arg)
{
  int value = 0;

  /* UIF is cleared by writing ZERO to it, not one. Writing one is the
   * reflex from the FDCAN interrupt register and would leave the flag set,
   * re-entering this handler forever.
   */

  putreg16(0, STM32_TIM6_SR);

  g_status.ticks++;
  g_status.last_tick_us = fmuv6c_imu_time_now();

  /* Post only when the downlink thread is actually waiting. Otherwise the
   * count builds up while it is busy writing and it then runs several
   * periods back to back, which is the opposite of a fixed cadence.
   */

  if (nxsem_get_value(&g_tick_sem, &value) == OK && value <= 0)
    {
      nxsem_post(&g_tick_sem);
    }
  else
    {
      /* The thread did not consume the previous tick in time. Counting this
       * is the whole point: a downlink that cannot keep up should say so
       * rather than silently running at half rate.
       */

      g_status.missed++;
    }

  UNUSED(irq);
  UNUSED(context);
  UNUSED(arg);
  return OK;
}

int fmuv6c_txtick_initialize(uint32_t rate_hz)
{
  irqstate_t flags;
  uint32_t prescaler;
  uint32_t reload;
  int ret;

  if (rate_hz < TXTICK_MIN_HZ || rate_hz > TXTICK_MAX_HZ)
    {
      return -ERANGE;
    }

  if (g_initialized)
    {
      return -EALREADY;
    }

  reload = TXTICK_COUNTER_HZ / rate_hz;

  if (reload == 0 || reload > 65536u)
    {
      return -ERANGE;
    }

  flags = enter_critical_section();

  modifyreg32(STM32_RCC_APB1LENR, 0, RCC_APB1LENR_TIM6EN);

  putreg16(0, STM32_TIM6_CR1);
  putreg16(0, STM32_TIM6_DIER);

  prescaler = STM32_APB1_TIM6_CLKIN / TXTICK_COUNTER_HZ;
  putreg16((uint16_t)(prescaler - 1u), STM32_TIM6_PSC);
  putreg16((uint16_t)(reload - 1u), STM32_TIM6_ARR);

  /* Generate an update to load PSC and ARR from their shadow registers,
   * then clear the flag that generation just set. Skipping the clear costs
   * one spurious tick immediately after start.
   */

  putreg16(GTIM_EGR_UG, STM32_TIM6_EGR);
  putreg16(0, STM32_TIM6_SR);

  memset(&g_status, 0, sizeof(g_status));
  g_status.rate_hz = rate_hz;
  g_status.period_us = reload;

  nxsem_init(&g_tick_sem, 0, 0);

  leave_critical_section(flags);

  ret = irq_attach(STM32_IRQ_TIM6, fmuv6c_txtick_isr, NULL);

  if (ret < 0)
    {
      modifyreg32(STM32_RCC_APB1LENR, RCC_APB1LENR_TIM6EN, 0);
      return ret;
    }

  g_initialized = true;

  up_enable_irq(STM32_IRQ_TIM6);
  putreg16(GTIM_DIER_UIE, STM32_TIM6_DIER);
  putreg16(GTIM_CR1_CEN, STM32_TIM6_CR1);

  syslog(LOG_INFO, "[companion] TIM6 downlink tick at %" PRIu32 " Hz"
         " (%" PRIu32 " us)\n", rate_hz, reload);
  return OK;
}

int fmuv6c_txtick_uninitialize(void)
{
  if (!g_initialized)
    {
      return -ESRCH;
    }

  putreg16(0, STM32_TIM6_CR1);
  putreg16(0, STM32_TIM6_DIER);
  up_disable_irq(STM32_IRQ_TIM6);
  irq_detach(STM32_IRQ_TIM6);
  modifyreg32(STM32_RCC_APB1LENR, RCC_APB1LENR_TIM6EN, 0);

  g_initialized = false;

  /* Release anyone blocked in fmuv6c_txtick_wait so the downlink thread can
   * observe the stop rather than sleeping until the watchdog notices.
   */

  nxsem_post(&g_tick_sem);
  return OK;
}

int fmuv6c_txtick_wait(void)
{
  if (!g_initialized)
    {
      return -ESRCH;
    }

  /* A generous bound rather than an unbounded wait: if the timer ever stops
   * the caller returns, sees the stop, and reports it.
   */

  return nxsem_tickwait(&g_tick_sem, MSEC2TICK(200));
}

void fmuv6c_txtick_status(FAR struct fmuv6c_txtick_status_s *status)
{
  if (status == NULL)
    {
      return;
    }

  *status = g_status;
  status->running = g_initialized;
}

#endif /* CONFIG_XXCAR_COMPANION */
