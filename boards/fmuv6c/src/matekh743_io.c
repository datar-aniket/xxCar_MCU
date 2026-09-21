/****************************************************************************
 * boards/fmuv6c/src/matekh743_io.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Matek H743-SLIM-V4 early pin states and RC pulse I/O:
 *   R6 / PC7  -> TIM3_CH2 PPM input capture
 *
 * Steering PWM is implemented in matekh743_pwm.c. TIM5 remains the
 * shared 1 MHz IMU timestamp clock.
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

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "hardware/stm32_tim.h"

#include "fmuv6c.h"
#include "ppm_decode.h"

#define PPM_GPIO                BOARD_MATEK_PPM_GPIO
#define PPM_TIMER_HZ            1000000u
#define PPM_SYNC_MIN_US         2700u
#define PPM_CHANNEL_MIN_US      750u
#define PPM_CHANNEL_MAX_US      2250u
#define PPM_MIN_CHANNELS        4u

#if (STM32_APB1_TIM3_CLKIN % PPM_TIMER_HZ) != 0
#  error "TIM3 clock must divide exactly to the 1 MHz PPM timebase"
#endif

static struct board_rc_ppm_frame_s g_ppm_frame;
static struct ppm_decoder_s g_ppm_decoder;
static uint16_t g_ppm_last_capture;
static uint64_t g_ppm_last_edge_us;
static bool     g_ppm_have_edge;
static bool     g_ppm_running;

/* Called once during early board initialization, before peripheral drivers.
 * Map every servo pad to an inactive GPIO; configuring a map does not emit
 * PWM. stm32_configgpio() loads the output latch before changing GPIO mode.
 */

void board_matek_pins_initialize(void)
{
#define PWM_IDLE(s, timer, channel, complementary, gpio) \
  BOARD_MATEK_OUTPUT_LOW(gpio),
  static const uint32_t pins[] =
  {
    BOARD_MATEK_PWM_PINS(PWM_IDLE)
    BOARD_MATEK_PWM_EXPANSION_PINS(PWM_IDLE)
    BOARD_MATEK_BUZZER_OFF,
    BOARD_MATEK_PPS_IDLE
  };
#undef PWM_IDLE
  unsigned int i;

  for (i = 0; i < sizeof(pins) / sizeof(pins[0]); i++)
    {
      stm32_configgpio(pins[i]);
    }
}

static void ppm_note_invalid(uint64_t timestamp_us)
{
  g_ppm_frame.timestamp_us = timestamp_us;
  g_ppm_frame.attempts++;
  g_ppm_frame.errors++;
  if (g_ppm_frame.invalid_streak < UINT8_MAX)
    {
      g_ppm_frame.invalid_streak++;
    }
}

static int matek_ppm_isr(int irq, FAR void *context, FAR void *arg)
{
  uint16_t status;
  uint16_t captured;
  uint16_t interval;
  uint64_t now_us;
  int result;

  (void)irq;
  (void)context;
  (void)arg;

  status = getreg16(STM32_TIM3_BASE + STM32_GTIM_SR_OFFSET);
  if ((status & (GTIM_SR_CC2IF | GTIM_SR_CC2OF)) == 0)
    {
      return OK;
    }

  captured = getreg16(STM32_TIM3_BASE + STM32_GTIM_CCR2_OFFSET);
  now_us = fmuv6c_imu_time_now();
  putreg16((uint16_t)~(GTIM_SR_CC2IF | GTIM_SR_CC2OF),
           STM32_TIM3_BASE + STM32_GTIM_SR_OFFSET);

  if ((status & GTIM_SR_CC2OF) != 0)
    {
      ppm_note_invalid(fmuv6c_imu_time_now());
      ppm_reset(&g_ppm_decoder, g_ppm_decoder.expected);
      g_ppm_last_capture = captured;
      g_ppm_last_edge_us = now_us;
      g_ppm_have_edge = true;
      return OK;
    }

  if (!g_ppm_have_edge)
    {
      g_ppm_last_capture = captured;
      g_ppm_last_edge_us = now_us;
      g_ppm_have_edge = true;
      return OK;
    }

  interval = (uint16_t)(captured - g_ppm_last_capture);
  g_ppm_last_capture = captured;
  /* A long silence can wrap the 16-bit capture counter into a plausible
   * channel interval. Use the independent timer only to detect that gap.
   */

  if (now_us - g_ppm_last_edge_us >= 30000)
    {
      ppm_reset(&g_ppm_decoder, g_ppm_decoder.expected);
      ppm_note_invalid(now_us);
      interval = PPM_SYNC_MIN_US;
    }
  g_ppm_last_edge_us = now_us;
  result = ppm_interval(&g_ppm_decoder, interval);
  if (result > 0)
    {
      memcpy(g_ppm_frame.channel, g_ppm_decoder.channel,
             g_ppm_decoder.expected * sizeof(uint16_t));
      g_ppm_frame.count = g_ppm_decoder.expected;
      g_ppm_frame.timestamp_us = now_us;
      g_ppm_frame.attempts++;
      g_ppm_frame.sequence++;
      g_ppm_frame.invalid_streak = 0;
    }
  else if (result < 0)
    {
      ppm_note_invalid(now_us);
    }

  return OK;
}

int board_matek_rc_ppm_start(unsigned channels)
{
  irqstate_t flags;
  uint32_t divider;
  int ret;

  if (channels < PPM_MIN_CHANNELS || channels > BOARD_RC_PPM_MAX_CHANNELS)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  if (g_ppm_running)
    {
      leave_critical_section(flags);
      return -EALREADY;
    }

  memset(&g_ppm_frame, 0, sizeof(g_ppm_frame));
  ppm_reset(&g_ppm_decoder, channels);
  g_ppm_have_edge = false;
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
  /* Reject sub-microsecond glitches: fDTS/32, eight stable samples.
   * This filter uses the timer input clock, not the 1 MHz counter prescaler.
   */
  putreg16((GTIM_CCMR_CCS_CCIN1 << GTIM_CCMR1_CC2S_SHIFT) |
           (15u << GTIM_CCMR1_IC2F_SHIFT),
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
