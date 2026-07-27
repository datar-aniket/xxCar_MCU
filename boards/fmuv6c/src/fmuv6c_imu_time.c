/****************************************************************************
 * boards/fmuv6c/src/fmuv6c_imu_time.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared high-resolution clock for the FMUv6C onboard IMUs.
 *
 * TIM5 is a 32-bit APB1 timer and is otherwise unused by this board. It runs
 * freely at 1 MHz without channels, DMA, or interrupts. Every IMU DRDY ISR
 * reads the same counter, avoiding the 1 ms quantization of the NuttX system
 * tick without changing the system tick rate.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_rcc.h"
#include "hardware/stm32_tim.h"

#include "fmuv6c.h"

#if defined(CONFIG_SENSORS) && defined(CONFIG_SPI)

#ifdef CONFIG_STM32H7_TIM5
#  error "TIM5 is reserved by the FMUv6C IMU timestamp timebase"
#endif

#define IMU_TIMER_FREQUENCY_HZ 1000000u
#define IMU_TIMER_WRAP_US      (1ull << 32)
#define IMU_TIMER_HALF_WRAP_US (1ull << 31)

#if (STM32_APB1_TIM5_CLKIN % IMU_TIMER_FREQUENCY_HZ) != 0
#  error "TIM5 input clock must be an integer multiple of 1 MHz"
#endif

#if (STM32_APB1_TIM5_CLKIN / IMU_TIMER_FREQUENCY_HZ) > 65536
#  error "TIM5 1 MHz prescaler does not fit in PSC"
#endif

static uint64_t g_imu_timer_epoch_us;
static bool     g_imu_timer_initialized;

static uint64_t fmuv6c_monotonic_us(void)
{
  struct timespec ts;

  clock_systime_timespec(&ts);
  return 1000000ull * (uint64_t)ts.tv_sec +
         (uint64_t)ts.tv_nsec / 1000ull;
}

int fmuv6c_imu_time_initialize(void)
{
  irqstate_t flags;
  uint32_t prescaler;
  uint16_t cr1;

  flags = enter_critical_section();

  if (g_imu_timer_initialized)
    {
      leave_critical_section(flags);
      return OK;
    }

  /* Reserve TIM5 directly rather than registering a NuttX timer lower-half.
   * No GPIO alternate function, DMA request, NVIC line, or update interrupt
   * is enabled by this setup.
   */

  modifyreg32(STM32_RCC_APB1LENR, 0, RCC_APB1LENR_TIM5EN);

  putreg16(0, STM32_TIM5_BASE + STM32_GTIM_CR1_OFFSET);
  putreg16(0, STM32_TIM5_BASE + STM32_GTIM_DIER_OFFSET);
  putreg16(0, STM32_TIM5_BASE + STM32_GTIM_CCER_OFFSET);
  putreg16(0, STM32_TIM5_BASE + STM32_GTIM_SMCR_OFFSET);

  prescaler = STM32_APB1_TIM5_CLKIN / IMU_TIMER_FREQUENCY_HZ;
  putreg16((uint16_t)(prescaler - 1u),
           STM32_TIM5_BASE + STM32_GTIM_PSC_OFFSET);
  putreg32(UINT32_MAX, STM32_TIM5_BASE + STM32_GTIM_ARR_OFFSET);
  putreg32(0, STM32_TIM5_BASE + STM32_GTIM_CNT_OFFSET);
  putreg16(GTIM_EGR_UG, STM32_TIM5_BASE + STM32_GTIM_EGR_OFFSET);
  putreg16(0, STM32_TIM5_BASE + STM32_GTIM_SR_OFFSET);

  /* Establish the NuttX monotonic epoch while interrupts are held, then let
   * the hardware counter advance independently of the scheduler.
   */

  g_imu_timer_epoch_us = fmuv6c_monotonic_us();
  putreg16(GTIM_CR1_CEN, STM32_TIM5_BASE + STM32_GTIM_CR1_OFFSET);
  g_imu_timer_initialized = true;

  cr1 = getreg16(STM32_TIM5_BASE + STM32_GTIM_CR1_OFFSET);
  leave_critical_section(flags);

  if ((cr1 & GTIM_CR1_CEN) == 0)
    {
      return -EIO;
    }

  syslog(LOG_INFO,
         "[sensors] TIM5 reserved as shared 1 MHz IMU timebase"
         " (no IRQ/DMA/GPIO)\n");
  return OK;
}

uint64_t fmuv6c_imu_time_now(void)
{
  uint64_t coarse_us;
  uint64_t elapsed_us;
  uint64_t timer_us;
  uint32_t counter;

  if (!g_imu_timer_initialized)
    {
      return fmuv6c_monotonic_us();
    }

  /* TIM5 wraps every 71.6 minutes. Use the coarse monotonic clock only to
   * select the correct wrap; the low 32 bits, and therefore the timestamp
   * resolution, always come directly from TIM5. This needs no wrap IRQ and
   * remains correct even if every IMU is disabled for several timer wraps.
   */

  counter    = getreg32(STM32_TIM5_BASE + STM32_GTIM_CNT_OFFSET);
  coarse_us  = fmuv6c_monotonic_us();
  elapsed_us = coarse_us - g_imu_timer_epoch_us;
  timer_us   = (elapsed_us & ~(IMU_TIMER_WRAP_US - 1ull)) |
               (uint64_t)counter;

  if (timer_us + IMU_TIMER_HALF_WRAP_US < elapsed_us)
    {
      timer_us += IMU_TIMER_WRAP_US;
    }
  else if (timer_us > elapsed_us + IMU_TIMER_HALF_WRAP_US &&
           timer_us >= IMU_TIMER_WRAP_US)
    {
      timer_us -= IMU_TIMER_WRAP_US;
    }

  return g_imu_timer_epoch_us + timer_us;
}

#endif /* CONFIG_SENSORS && CONFIG_SPI */
