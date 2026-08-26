/****************************************************************************
 * boards/fmuv6c/src/fmuv6c_pps.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware PPS capture for the Jetson companion link.
 *
 * TELEM2 CTS is PC9, alternate function TIM3_CH4.  TIM3 is a free-running
 * 16-bit 1 MHz timer.  The input edge is latched in CCR4 by hardware, so ISR
 * scheduling latency does not become timestamp jitter.  The ISR subtracts
 * the modulo-16-bit CCR-to-CNT delay from the shared TIM5 monotonic timestamp;
 * this is unambiguous provided the ISR runs within 65.536 ms.
 *
 * This module observes PPS only.  It never steps, slews, or otherwise changes
 * CLOCK_MONOTONIC, TIM5, CLOCK_REALTIME, or the companion UTC offset.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "hardware/stm32_tim.h"

#include "fmuv6c.h"

#ifdef CONFIG_XXCAR_PPS

#if !defined(CONFIG_SENSORS) || !defined(CONFIG_SPI)
#  error "PPS capture requires the shared FMUv6C TIM5 timebase"
#endif

#ifdef CONFIG_STM32H7_TIM3
#  error "TIM3 is reserved by the FMUv6C PPS input capture"
#endif

#define PPS_TIMER_HZ             1000000u
#define PPS_NOMINAL_US           1000000u
#define PPS_GOOD_MIN_US           998000u
#define PPS_GOOD_MAX_US          1002000u
#define PPS_GLITCH_MAX_US         800000u
#define PPS_HOLDOVER_US          1500000u
#define PPS_LOCK_INTERVALS             3u

#define PPS_GPIO (GPIO_TIM3_CH4IN_2 | GPIO_PULLDOWN)
#define PPS_SR_FLAGS (GTIM_SR_CC4IF | GTIM_SR_CC4OF)

#if (STM32_APB1_TIM3_CLKIN % PPS_TIMER_HZ) != 0
#  error "TIM3 input clock must be an integer multiple of 1 MHz"
#endif

#if (STM32_APB1_TIM3_CLKIN / PPS_TIMER_HZ) > 65536
#  error "TIM3 PPS prescaler does not fit in PSC"
#endif

static struct fmuv6c_pps_status_s g_pps;

static void pps_process_edge(uint64_t edge_us)
{
  uint64_t delta64;
  uint32_t period;

  g_pps.raw_edges++;

  if (g_pps.last_edge_us == 0)
    {
      g_pps.last_edge_us = edge_us;
      g_pps.accepted_edges++;
      g_pps.state = FMUV6C_PPS_ACQUIRING;
      return;
    }

  delta64 = edge_us - g_pps.last_edge_us;

  /* A short edge is noise/bounce.  Do not move the accepted-edge anchor:
   * the real edge at the next integer second can still be accepted normally.
   */

  if (delta64 < PPS_GLITCH_MAX_US)
    {
      g_pps.glitches++;
      return;
    }

  period = delta64 > UINT32_MAX ? UINT32_MAX : (uint32_t)delta64;
  g_pps.last_edge_us = edge_us;
  g_pps.last_period_us = period;
  g_pps.accepted_edges++;

  if (period >= PPS_GOOD_MIN_US && period <= PPS_GOOD_MAX_US)
    {
      if (g_pps.min_period_us == 0 || period < g_pps.min_period_us)
        {
          g_pps.min_period_us = period;
        }

      if (period > g_pps.max_period_us)
        {
          g_pps.max_period_us = period;
        }

      if (g_pps.good_intervals < PPS_LOCK_INTERVALS)
        {
          g_pps.good_intervals++;
        }

      g_pps.state = g_pps.good_intervals >= PPS_LOCK_INTERVALS ?
                    FMUV6C_PPS_LOCKED : FMUV6C_PPS_ACQUIRING;
      return;
    }

  /* One returning edge after a gap is only a new phase anchor.  It can never
   * lock PPS by itself.  Three subsequent one-second intervals are required.
   */

  g_pps.bad_periods++;
  g_pps.good_intervals = 0;
  g_pps.state = FMUV6C_PPS_ACQUIRING;

  if (period > PPS_GOOD_MAX_US)
    {
      uint32_t intervals = (period + PPS_NOMINAL_US / 2u) / PPS_NOMINAL_US;

      if (intervals > 1u)
        {
          g_pps.missed_pulses += intervals - 1u;
        }
    }
}

static int pps_isr(int irq, FAR void *context, FAR void *arg)
{
  uint16_t captured;
  uint16_t counter;
  uint16_t delay_us;
  uint16_t status;
  uint64_t now_us;

  status = getreg16(STM32_TIM3_BASE + STM32_GTIM_SR_OFFSET);
  if ((status & PPS_SR_FLAGS) == 0)
    {
      return OK;
    }

  /* Read CCR before acknowledging the flag.  CNT-CCR works across a single
   * or multiple 16-bit wrap because unsigned subtraction is modulo 65536.
   */

  captured = getreg16(STM32_TIM3_BASE + STM32_GTIM_CCR4_OFFSET);
  counter = getreg16(STM32_TIM3_BASE + STM32_GTIM_CNT_OFFSET);
  now_us = fmuv6c_imu_time_now();
  delay_us = (uint16_t)(counter - captured);

  /* STM32 timer flags clear when zero is written to the selected bits. */

  putreg16((uint16_t)~PPS_SR_FLAGS,
           STM32_TIM3_BASE + STM32_GTIM_SR_OFFSET);

  if ((status & GTIM_SR_CC4OF) != 0)
    {
      g_pps.overcaptures++;
    }

  pps_process_edge(now_us - (uint64_t)delay_us);
  return OK;
}

int fmuv6c_pps_initialize(void)
{
  irqstate_t flags;
  uint32_t divider;
  int ret;

  flags = enter_critical_section();
  if (g_pps.running)
    {
      leave_critical_section(flags);
      return -EALREADY;
    }

  memset(&g_pps, 0, sizeof(g_pps));
  leave_critical_section(flags);

  ret = irq_attach(STM32_IRQ_TIM3, pps_isr, NULL);
  if (ret < 0)
    {
      return ret;
    }

  ret = stm32_configgpio(PPS_GPIO);
  if (ret < 0)
    {
      irq_detach(STM32_IRQ_TIM3);
      return ret;
    }

  modifyreg32(STM32_RCC_APB1LENR, 0, RCC_APB1LENR_TIM3EN);

  putreg16(0, STM32_TIM3_BASE + STM32_GTIM_CR1_OFFSET);
  putreg16(0, STM32_TIM3_BASE + STM32_GTIM_DIER_OFFSET);
  putreg16(0, STM32_TIM3_BASE + STM32_GTIM_CCER_OFFSET);
  putreg16(0, STM32_TIM3_BASE + STM32_GTIM_SMCR_OFFSET);
  putreg16(0, STM32_TIM3_BASE + STM32_GTIM_CCMR1_OFFSET);
  putreg16(GTIM_CCMR_CCS_CCIN1 << GTIM_CCMR2_CC4S_SHIFT,
           STM32_TIM3_BASE + STM32_GTIM_CCMR2_OFFSET);

  divider = STM32_APB1_TIM3_CLKIN / PPS_TIMER_HZ;
  putreg16((uint16_t)(divider - 1u),
           STM32_TIM3_BASE + STM32_GTIM_PSC_OFFSET);
  putreg16(UINT16_MAX, STM32_TIM3_BASE + STM32_GTIM_ARR_OFFSET);
  putreg16(0, STM32_TIM3_BASE + STM32_GTIM_CNT_OFFSET);
  putreg16(GTIM_EGR_UG, STM32_TIM3_BASE + STM32_GTIM_EGR_OFFSET);
  putreg16(0, STM32_TIM3_BASE + STM32_GTIM_SR_OFFSET);

  /* CC4P/CC4NP remain clear: rising edges only.  The Jetson's 100 ms high
   * pulse therefore produces exactly one capture.
   */

  putreg16(GTIM_CCER_CC4E, STM32_TIM3_BASE + STM32_GTIM_CCER_OFFSET);
  putreg16(GTIM_DIER_CC4IE, STM32_TIM3_BASE + STM32_GTIM_DIER_OFFSET);
  putreg16(GTIM_CR1_CEN, STM32_TIM3_BASE + STM32_GTIM_CR1_OFFSET);

  flags = enter_critical_section();
  g_pps.running = true;
  g_pps.state = FMUV6C_PPS_NO_SIGNAL;
  leave_critical_section(flags);

  up_enable_irq(STM32_IRQ_TIM3);
  return OK;
}

int fmuv6c_pps_uninitialize(void)
{
  irqstate_t flags;

  up_disable_irq(STM32_IRQ_TIM3);
  putreg16(0, STM32_TIM3_BASE + STM32_GTIM_DIER_OFFSET);
  putreg16(0, STM32_TIM3_BASE + STM32_GTIM_CCER_OFFSET);
  putreg16(0, STM32_TIM3_BASE + STM32_GTIM_CR1_OFFSET);
  irq_detach(STM32_IRQ_TIM3);
  stm32_unconfiggpio(PPS_GPIO);
  modifyreg32(STM32_RCC_APB1LENR, RCC_APB1LENR_TIM3EN, 0);

  flags = enter_critical_section();
  g_pps.running = false;
  g_pps.state = FMUV6C_PPS_NO_SIGNAL;
  leave_critical_section(flags);
  return OK;
}

void fmuv6c_pps_status(FAR struct fmuv6c_pps_status_s *status)
{
  irqstate_t flags;
  uint64_t now_us;

  if (status == NULL)
    {
      return;
    }

  now_us = fmuv6c_imu_time_now();
  flags = enter_critical_section();

  if (g_pps.running && g_pps.last_edge_us != 0 &&
      now_us - g_pps.last_edge_us > PPS_HOLDOVER_US &&
      g_pps.state != FMUV6C_PPS_HOLDOVER &&
      g_pps.state != FMUV6C_PPS_NO_SIGNAL)
    {
      /* HOLDOVER specifically means a previously locked source disappeared.
       * An incomplete acquisition that goes silent is simply no signal.
       */

      g_pps.state = g_pps.state == FMUV6C_PPS_LOCKED ?
                    FMUV6C_PPS_HOLDOVER : FMUV6C_PPS_NO_SIGNAL;
      g_pps.good_intervals = 0;
    }

  memcpy(status, &g_pps, sizeof(*status));
  leave_critical_section(flags);
}

#endif /* CONFIG_XXCAR_PPS */
