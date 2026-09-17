/****************************************************************************
 * boards/fmuv6c/src/matekh743_io.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Matek H743-SLIM-V4 direct actuator and RC pulse I/O:
 *   S1 / PA0  -> TIM2_CH1 PWM steering output
 *   R6 / PC7  -> TIM3_CH2 PPM input capture
 *
 * TIM2 is deliberately used for S1 rather than TIM5. TIM5 remains the shared
 * 1 MHz IMU timestamp clock, so steering cannot perturb estimator timing.
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_XXCAR_BOARD_MATEKH743

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "hardware/stm32_tim.h"

#include "fmuv6c.h"

#define S1_PWM_GPIO             (GPIO_TIM2_CH1OUT_1 | GPIO_SPEED_50MHz)
#define S1_PWM_TIMER_HZ         1000000u
#define S1_PWM_MIN_US           900u
#define S1_PWM_MAX_US           2100u
#define S1_PWM_TIMEOUT_MS       200u

#define PPM_GPIO                (GPIO_TIM3_CH2IN_3 | GPIO_PULLDOWN)
#define PPM_TIMER_HZ            1000000u
#define PPM_SYNC_MIN_US         2700u
#define PPM_CHANNEL_MIN_US      750u
#define PPM_CHANNEL_MAX_US      2250u
#define PPM_MIN_CHANNELS        4u

#if (STM32_APB1_TIM2_CLKIN % S1_PWM_TIMER_HZ) != 0
#  error "TIM2 clock must divide exactly to the 1 MHz S1 PWM timebase"
#endif

#if (STM32_APB1_TIM3_CLKIN % PPM_TIMER_HZ) != 0
#  error "TIM3 clock must divide exactly to the 1 MHz PPM timebase"
#endif

static bool          g_s1_running;
static uint16_t      g_s1_neutral_us = 1500u;
static struct wdog_s g_s1_watchdog;

static struct board_rc_ppm_frame_s g_ppm_frame;
static uint16_t g_ppm_work[BOARD_RC_PPM_MAX_CHANNELS];
static uint16_t g_ppm_last_capture;
static uint8_t  g_ppm_count;
static bool     g_ppm_have_edge;
static bool     g_ppm_collecting;
static bool     g_ppm_running;

static void s1_pwm_timeout(wdparm_t arg)
{
  (void)arg;

  if (g_s1_running)
    {
      putreg32(g_s1_neutral_us,
               STM32_TIM2_BASE + STM32_GTIM_CCR1_OFFSET);
    }
}

int board_matek_s1_pwm_start(uint16_t rate_hz, uint16_t neutral_us)
{
  irqstate_t flags;
  uint32_t period_us;
  uint32_t divider;
  int ret;

  if (rate_hz < 25u || rate_hz > 400u ||
      neutral_us < S1_PWM_MIN_US || neutral_us > S1_PWM_MAX_US)
    {
      return -EINVAL;
    }

  period_us = (S1_PWM_TIMER_HZ + rate_hz / 2u) / rate_hz;
  if (period_us <= S1_PWM_MAX_US)
    {
      return -ERANGE;
    }

  flags = enter_critical_section();
  if (g_s1_running)
    {
      leave_critical_section(flags);
      return OK;
    }

  /* S1 cannot simultaneously be the PPS input. Bring-up prevents that
   * combination; configuring AF1 here makes the ownership explicit.
   */

  ret = stm32_configgpio(S1_PWM_GPIO);
  if (ret < 0)
    {
      leave_critical_section(flags);
      return ret;
    }

  modifyreg32(STM32_RCC_APB1LENR, 0, RCC_APB1LENR_TIM2EN);
  putreg16(0, STM32_TIM2_BASE + STM32_GTIM_CR1_OFFSET);
  putreg16(0, STM32_TIM2_BASE + STM32_GTIM_DIER_OFFSET);
  putreg16(0, STM32_TIM2_BASE + STM32_GTIM_CCER_OFFSET);
  putreg16(0, STM32_TIM2_BASE + STM32_GTIM_SMCR_OFFSET);

  divider = STM32_APB1_TIM2_CLKIN / S1_PWM_TIMER_HZ;
  putreg16((uint16_t)(divider - 1u),
           STM32_TIM2_BASE + STM32_GTIM_PSC_OFFSET);
  putreg32(period_us - 1u, STM32_TIM2_BASE + STM32_GTIM_ARR_OFFSET);
  putreg32(neutral_us, STM32_TIM2_BASE + STM32_GTIM_CCR1_OFFSET);

  putreg16((GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC1M_SHIFT) |
           GTIM_CCMR1_OC1PE,
           STM32_TIM2_BASE + STM32_GTIM_CCMR1_OFFSET);
  putreg16(GTIM_CCER_CC1E, STM32_TIM2_BASE + STM32_GTIM_CCER_OFFSET);
  putreg16(GTIM_EGR_UG, STM32_TIM2_BASE + STM32_GTIM_EGR_OFFSET);
  putreg16(0, STM32_TIM2_BASE + STM32_GTIM_SR_OFFSET);
  putreg16(GTIM_CR1_ARPE | GTIM_CR1_CEN,
           STM32_TIM2_BASE + STM32_GTIM_CR1_OFFSET);

  g_s1_neutral_us = neutral_us;
  g_s1_running = true;
  leave_critical_section(flags);
  return OK;
}

int board_matek_s1_pwm_set(uint16_t pulse_us, uint16_t neutral_us)
{
  irqstate_t flags;
  int ret;

  if (pulse_us < S1_PWM_MIN_US || pulse_us > S1_PWM_MAX_US ||
      neutral_us < S1_PWM_MIN_US || neutral_us > S1_PWM_MAX_US)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  if (!g_s1_running)
    {
      leave_critical_section(flags);
      return -ENODEV;
    }

  g_s1_neutral_us = neutral_us;
  putreg32(pulse_us, STM32_TIM2_BASE + STM32_GTIM_CCR1_OFFSET);
  ret = wd_start(&g_s1_watchdog, MSEC2TICK(S1_PWM_TIMEOUT_MS),
                 s1_pwm_timeout, 0);
  leave_critical_section(flags);
  return ret;
}

bool board_matek_s1_pwm_healthy(void)
{
  irqstate_t flags;
  bool healthy;

  flags = enter_critical_section();
  healthy = g_s1_running &&
            (getreg16(STM32_TIM2_BASE + STM32_GTIM_CR1_OFFSET) &
             GTIM_CR1_CEN) != 0 &&
            (getreg16(STM32_TIM2_BASE + STM32_GTIM_CCER_OFFSET) &
             GTIM_CCER_CC1E) != 0;
  leave_critical_section(flags);
  return healthy;
}

static int matek_ppm_isr(int irq, FAR void *context, FAR void *arg)
{
  uint16_t status;
  uint16_t captured;
  uint16_t interval;

  (void)irq;
  (void)context;
  (void)arg;

  status = getreg16(STM32_TIM3_BASE + STM32_GTIM_SR_OFFSET);
  if ((status & (GTIM_SR_CC2IF | GTIM_SR_CC2OF)) == 0)
    {
      return OK;
    }

  captured = getreg16(STM32_TIM3_BASE + STM32_GTIM_CCR2_OFFSET);
  putreg16((uint16_t)~(GTIM_SR_CC2IF | GTIM_SR_CC2OF),
           STM32_TIM3_BASE + STM32_GTIM_SR_OFFSET);

  if ((status & GTIM_SR_CC2OF) != 0)
    {
      g_ppm_frame.errors++;
      g_ppm_collecting = false;
      g_ppm_count = 0;
    }

  if (!g_ppm_have_edge)
    {
      g_ppm_last_capture = captured;
      g_ppm_have_edge = true;
      return OK;
    }

  interval = (uint16_t)(captured - g_ppm_last_capture);
  g_ppm_last_capture = captured;

  if (interval >= PPM_SYNC_MIN_US)
    {
      if (g_ppm_collecting && g_ppm_count >= PPM_MIN_CHANNELS)
        {
          memcpy(g_ppm_frame.channel, g_ppm_work,
                 g_ppm_count * sizeof(uint16_t));
          g_ppm_frame.count = g_ppm_count;
          g_ppm_frame.timestamp_us = fmuv6c_imu_time_now();
          g_ppm_frame.sequence++;
        }
      else if (g_ppm_collecting && g_ppm_count != 0)
        {
          g_ppm_frame.errors++;
        }

      g_ppm_count = 0;
      g_ppm_collecting = true;
    }
  else if (g_ppm_collecting && interval >= PPM_CHANNEL_MIN_US &&
           interval <= PPM_CHANNEL_MAX_US &&
           g_ppm_count < BOARD_RC_PPM_MAX_CHANNELS)
    {
      g_ppm_work[g_ppm_count++] = interval;
    }
  else
    {
      g_ppm_frame.errors++;
      g_ppm_count = 0;
      g_ppm_collecting = false;
    }

  return OK;
}

int board_matek_rc_ppm_start(void)
{
  irqstate_t flags;
  uint32_t divider;
  int ret;

  flags = enter_critical_section();
  if (g_ppm_running)
    {
      leave_critical_section(flags);
      return -EALREADY;
    }

  memset(&g_ppm_frame, 0, sizeof(g_ppm_frame));
  g_ppm_count = 0;
  g_ppm_have_edge = false;
  g_ppm_collecting = false;
  leave_critical_section(flags);

  ret = irq_attach(STM32_IRQ_TIM3, matek_ppm_isr, NULL);
  if (ret < 0)
    {
      return ret;
    }

  ret = stm32_configgpio(PPM_GPIO);
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
  putreg16(GTIM_CCMR_CCS_CCIN1 << GTIM_CCMR1_CC2S_SHIFT,
           STM32_TIM3_BASE + STM32_GTIM_CCMR1_OFFSET);

  divider = STM32_APB1_TIM3_CLKIN / PPM_TIMER_HZ;
  putreg16((uint16_t)(divider - 1u),
           STM32_TIM3_BASE + STM32_GTIM_PSC_OFFSET);
  putreg16(UINT16_MAX, STM32_TIM3_BASE + STM32_GTIM_ARR_OFFSET);
  putreg16(0, STM32_TIM3_BASE + STM32_GTIM_CNT_OFFSET);
  putreg16(GTIM_EGR_UG, STM32_TIM3_BASE + STM32_GTIM_EGR_OFFSET);
  putreg16(0, STM32_TIM3_BASE + STM32_GTIM_SR_OFFSET);
  putreg16(GTIM_CCER_CC2E, STM32_TIM3_BASE + STM32_GTIM_CCER_OFFSET);
  putreg16(GTIM_DIER_CC2IE, STM32_TIM3_BASE + STM32_GTIM_DIER_OFFSET);
  putreg16(GTIM_CR1_CEN, STM32_TIM3_BASE + STM32_GTIM_CR1_OFFSET);

  flags = enter_critical_section();
  g_ppm_running = true;
  leave_critical_section(flags);
  up_enable_irq(STM32_IRQ_TIM3);
  return OK;
}

void board_matek_rc_ppm_stop(void)
{
  irqstate_t flags;

  up_disable_irq(STM32_IRQ_TIM3);
  putreg16(0, STM32_TIM3_BASE + STM32_GTIM_DIER_OFFSET);
  putreg16(0, STM32_TIM3_BASE + STM32_GTIM_CCER_OFFSET);
  putreg16(0, STM32_TIM3_BASE + STM32_GTIM_CR1_OFFSET);
  irq_detach(STM32_IRQ_TIM3);
  stm32_unconfiggpio(PPM_GPIO);
  stm32_configgpio(GPIO_USART6_RX);
  modifyreg32(STM32_RCC_APB1LENR, RCC_APB1LENR_TIM3EN, 0);

  flags = enter_critical_section();
  g_ppm_running = false;
  leave_critical_section(flags);
}

bool board_matek_rc_ppm_latest(FAR struct board_rc_ppm_frame_s *frame)
{
  irqstate_t flags;

  if (frame == NULL)
    {
      return false;
    }

  flags = enter_critical_section();
  memcpy(frame, &g_ppm_frame, sizeof(*frame));
  leave_critical_section(flags);
  return frame->sequence != 0;
}

#endif /* CONFIG_XXCAR_BOARD_MATEKH743 */
