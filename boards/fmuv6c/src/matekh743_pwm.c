/* SPDX-License-Identifier: Apache-2.0
 * S1-S8: independent TIM8/TIM2/TIM4 banks, aligned edges within a bank.
 * Hardware owns every edge: no PWM ISR, DMA, or software counter restart.
 */
#include <nuttx/config.h>
#ifdef CONFIG_XXCAR_BOARD_MATEKH743
#include <errno.h>
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

#define PWM_TIMER_HZ 1000000u
#define PWM_MIN_US 900u
#define PWM_MAX_US 2100u
#define PWM_TIMEOUT_MS 200u
#define PWM_CR1 (GTIM_CR1_ARPE | GTIM_CR1_CEN)

#if STM32_APB1_TIM2_CLKIN % PWM_TIMER_HZ || \
    STM32_APB1_TIM4_CLKIN % PWM_TIMER_HZ || \
    STM32_APB2_TIM8_CLKIN % PWM_TIMER_HZ
#  error "Servo timers require an exact 1 MHz counter"
#endif

struct pwm_bank_s
{
  uint32_t base, enable_reg, reset_reg, enable_bit, reset_bit, clock;
};

static const struct pwm_bank_s g_banks[] =
{
  {STM32_TIM8_BASE, STM32_RCC_APB2ENR, STM32_RCC_APB2RSTR,
   RCC_APB2ENR_TIM8EN, RCC_APB2RSTR_TIM8RST, STM32_APB2_TIM8_CLKIN},
  {STM32_TIM2_BASE, STM32_RCC_APB1LENR, STM32_RCC_APB1LRSTR,
   RCC_APB1LENR_TIM2EN, RCC_APB1LRSTR_TIM2RST, STM32_APB1_TIM2_CLKIN},
  {STM32_TIM4_BASE, STM32_RCC_APB1LENR, STM32_RCC_APB1LRSTR,
   RCC_APB1LENR_TIM4EN, RCC_APB1LRSTR_TIM4RST, STM32_APB1_TIM4_CLKIN}
};

struct pwm_pin_s { uint32_t gpio; uint8_t bank, channel, complementary; };
#define PWM_PIN(s, t, c, n, p) {p, (t == 8 ? 0 : t == 2 ? 1 : 2), c - 1, n},
static const struct pwm_pin_s g_pins[] = {BOARD_MATEK_PWM_PINS(PWM_PIN)};
#undef PWM_PIN

static struct board_matek_pwm_status_s g_status;
static uint16_t g_neutral[BOARD_MATEK_PWM_CHANNELS];
static uint16_t g_ccer[3];
static uint16_t g_ccmr[3][2];
static struct wdog_s g_watchdog;

static bool clock_valid(void)
{
  uint32_t mask = RCC_D2CFGR_D2PPRE1_MASK | RCC_D2CFGR_D2PPRE2_MASK;
  return (getreg32(STM32_RCC_D2CFGR) & mask) ==
    (STM32_RCC_D2CFGR_D2PPRE1 | STM32_RCC_D2CFGR_D2PPRE2);
}

static bool healthy_locked(void)
{
  unsigned b, i;
  if (!g_status.mask || !clock_valid()) return false;
  for (b = 0; b < 3; b++)
    {
      const struct pwm_bank_s *bank = &g_banks[b];
      uint32_t base = bank->base;
      if (!g_ccer[b]) continue;
      if (!(getreg32(bank->enable_reg) & bank->enable_bit) ||
          getreg16(base + STM32_GTIM_CR1_OFFSET) != PWM_CR1 ||
          getreg16(base + STM32_GTIM_CR2_OFFSET) != 0 ||
          getreg16(base + STM32_GTIM_CCER_OFFSET) != g_ccer[b] ||
          getreg16(base + STM32_GTIM_CCMR1_OFFSET) != g_ccmr[b][0] ||
          getreg16(base + STM32_GTIM_CCMR2_OFFSET) != g_ccmr[b][1] ||
          getreg32(base + STM32_GTIM_SMCR_OFFSET) != 0 ||
          getreg16(base + STM32_GTIM_DIER_OFFSET) != 0 ||
          getreg16(base + STM32_GTIM_PSC_OFFSET) != bank->clock / PWM_TIMER_HZ - 1 ||
          getreg32(base + STM32_GTIM_ARR_OFFSET) != g_status.period_us - 1 ||
          (b == 0 && (getreg32(base + STM32_ATIM_BDTR_OFFSET) != ATIM_BDTR_MOE ||
                      getreg16(base + STM32_ATIM_RCR_OFFSET) != 0)))
        return false;
    }

  for (i = 0; i < BOARD_MATEK_PWM_CHANNELS; i++)
    {
      uint32_t gpio = g_pins[i].gpio;
      unsigned pin = gpio & GPIO_PIN_MASK;
      uint32_t port = STM32_GPIOA_BASE +
        0x400u * ((gpio & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
      unsigned shift = (pin % 8) * 4;
      if (!(g_status.mask & (1u << i))) continue;
      if (((getreg32(port + STM32_GPIO_MODER_OFFSET) >> (pin * 2)) & 3) != GPIO_MODER_ALT ||
          ((getreg32(port + (pin < 8 ? STM32_GPIO_AFRL_OFFSET : STM32_GPIO_AFRH_OFFSET)) >> shift) & 15) !=
          ((gpio & GPIO_AF_MASK) >> GPIO_AF_SHIFT)) return false;
    }
  return true;
}

/* UDIS inhibits shadow transfers, NOT counting or edges. Each bank receives
 * the complete old or new vector even on rollover between CCR writes.
 * Banks intentionally need not latch on the same frame. No running UG/CNT.
 */
static void commit(const uint16_t pulse[BOARD_MATEK_PWM_CHANNELS])
{
  unsigned b, i;
  for (b = 0; b < 3; b++)
    {
      uint32_t base = g_banks[b].base;
      if (!g_ccer[b]) continue;
      putreg16(PWM_CR1 | GTIM_CR1_UDIS, base + STM32_GTIM_CR1_OFFSET);
      for (i = 0; i < BOARD_MATEK_PWM_CHANNELS; i++)
        if ((g_status.mask & (1u << i)) && g_pins[i].bank == b)
          putreg32(pulse[i], base + STM32_GTIM_CCR1_OFFSET + 4 * g_pins[i].channel);
      putreg16(PWM_CR1, base + STM32_GTIM_CR1_OFFSET);
    }
}

static void timeout(wdparm_t arg)
{
  irqstate_t flags = enter_critical_section();
  (void)arg;
  if (g_status.mask)
    {
      commit(g_neutral);
      memcpy(g_status.pulse_us, g_neutral, sizeof(g_neutral));
      g_status.timeouts++;
    }
  leave_critical_section(flags);
}

/* Explicit stop/fault rollback may truncate a pulse; ordinary disarming
 * uses neutral via preload. Never touch a timer not owned by this driver.
 */
static void stop_locked(void)
{
  unsigned b, i;
  wd_cancel(&g_watchdog);
  for (b = 0; b < 3; b++)
    if (g_ccer[b])
      {
        putreg16(0, g_banks[b].base + STM32_GTIM_CCER_OFFSET);
        if (b == 0) putreg32(0, g_banks[b].base + STM32_ATIM_BDTR_OFFSET);
        putreg16(0, g_banks[b].base + STM32_GTIM_CR1_OFFSET);
      }
  for (i = 0; i < BOARD_MATEK_PWM_CHANNELS; i++)
    if (g_status.mask & (1u << i))
      stm32_configgpio(BOARD_MATEK_OUTPUT_LOW(g_pins[i].gpio));
  g_status.mask = 0;
  memset(g_ccer, 0, sizeof(g_ccer));
  memset(g_ccmr, 0, sizeof(g_ccmr));
}

void board_matek_pwm_stop(void)
{
  irqstate_t flags = enter_critical_section();
  stop_locked();
  leave_critical_section(flags);
}

int board_matek_pwm_start(uint16_t rate_hz, uint8_t mask,
                          const uint16_t neutral[BOARD_MATEK_PWM_CHANNELS])
{
  irqstate_t flags;
  unsigned b, i;
  uint32_t period;
  int ret = OK;
  if (!mask || !neutral || rate_hz < 25 || rate_hz > 400) return -EINVAL;
  for (i = 0; i < BOARD_MATEK_PWM_CHANNELS; i++)
    if ((mask & (1u << i)) && (neutral[i] < PWM_MIN_US || neutral[i] > PWM_MAX_US))
      return -EINVAL;
  period = (PWM_TIMER_HZ + rate_hz / 2) / rate_hz;
  flags = enter_critical_section();
  if (g_status.mask)
    {
      ret = g_status.mask != mask || g_status.period_us != period ? -EBUSY :
            healthy_locked() ? OK : -EIO;
      leave_critical_section(flags);
      return ret;
    }
  if (!clock_valid())
    {
      leave_critical_section(flags);
      return -EIO;
    }
  g_status.mask = mask;
  g_status.period_us = period;
  memcpy(g_neutral, neutral, sizeof(g_neutral));
  memcpy(g_status.pulse_us, neutral, sizeof(g_neutral));
  for (i = 0; i < BOARD_MATEK_PWM_CHANNELS; i++)
    if (mask & (1u << i))
      {
        unsigned ch = g_pins[i].channel;
        b = g_pins[i].bank;
        ret = stm32_configgpio(BOARD_MATEK_OUTPUT_LOW(g_pins[i].gpio));
        if (ret < 0) goto fail;
        g_ccer[b] |= (g_pins[i].complementary ? 4u : 1u) << (ch * 4);
        g_ccmr[b][ch / 2] |= (GTIM_CCMR_MODE_PWM1 << 4 | 1u << 3) << ((ch % 2) * 8);
      }
  for (b = 0; b < 3; b++)
    {
      const struct pwm_bank_s *bank = &g_banks[b];
      uint32_t base = bank->base;
      if (!g_ccer[b]) continue;
      modifyreg32(bank->enable_reg, 0, bank->enable_bit);
      modifyreg32(bank->reset_reg, 0, bank->reset_bit);
      modifyreg32(bank->reset_reg, bank->reset_bit, 0);
      putreg16(GTIM_CR1_ARPE, base + STM32_GTIM_CR1_OFFSET);
      putreg16(0, base + STM32_GTIM_CR2_OFFSET);
      putreg16(0, base + STM32_GTIM_DIER_OFFSET);
      putreg16(0, base + STM32_GTIM_CCER_OFFSET);
      putreg32(0, base + STM32_GTIM_SMCR_OFFSET);
      putreg16(g_ccmr[b][0], base + STM32_GTIM_CCMR1_OFFSET);
      putreg16(g_ccmr[b][1], base + STM32_GTIM_CCMR2_OFFSET);
      putreg16(bank->clock / PWM_TIMER_HZ - 1, base + STM32_GTIM_PSC_OFFSET);
      putreg32(period - 1, base + STM32_GTIM_ARR_OFFSET);
      if (b == 0)
        {
          /* N-only: E=0, NE=1, NP=0 => OCREF, NOT its inverse (ST AN4013).
           * Zero deadtime/repetition, low idle, no automatic MOE or break.
           */
          putreg16(0, base + STM32_ATIM_RCR_OFFSET);
          putreg32(0, base + STM32_ATIM_BDTR_OFFSET);
        }
      for (i = 0; i < BOARD_MATEK_PWM_CHANNELS; i++)
        if ((mask & (1u << i)) && g_pins[i].bank == b)
          putreg32(neutral[i], base + STM32_GTIM_CCR1_OFFSET + 4 * g_pins[i].channel);
      putreg16(GTIM_EGR_UG, base + STM32_GTIM_EGR_OFFSET);
      /* Park LOW before attaching pins; first tick starts a full pulse. */
      putreg32(period - 1, base + STM32_GTIM_CNT_OFFSET);
      putreg16(0, base + STM32_GTIM_SR_OFFSET);
      putreg16(g_ccer[b], base + STM32_GTIM_CCER_OFFSET);
      if (b == 0) putreg32(ATIM_BDTR_MOE, base + STM32_ATIM_BDTR_OFFSET);
      for (i = 0; i < BOARD_MATEK_PWM_CHANNELS; i++)
        if ((mask & (1u << i)) && g_pins[i].bank == b)
          {
            ret = stm32_configgpio(g_pins[i].gpio);
            if (ret < 0) goto fail;
          }
    }
  for (b = 0; b < 3; b++)
    if (g_ccer[b]) putreg16(PWM_CR1, g_banks[b].base + STM32_GTIM_CR1_OFFSET);
  g_status.last_error = 0;
  leave_critical_section(flags);
  return OK;
fail:
  stop_locked();
  g_status.last_error = ret;
  leave_critical_section(flags);
  return ret;
}

int board_matek_pwm_set(const uint16_t pulse[BOARD_MATEK_PWM_CHANNELS],
                        const uint16_t neutral[BOARD_MATEK_PWM_CHANNELS])
{
  irqstate_t flags = enter_critical_section();
  unsigned i;
  int ret = OK;
  if (!g_status.mask) ret = -ENODEV;
  else if (!pulse || !neutral) ret = -EINVAL;
  else
    for (i = 0; i < BOARD_MATEK_PWM_CHANNELS; i++)
      if ((g_status.mask & (1u << i)) &&
          (pulse[i] < PWM_MIN_US || pulse[i] > PWM_MAX_US ||
           neutral[i] < PWM_MIN_US || neutral[i] > PWM_MAX_US)) ret = -EINVAL;
  if (ret == OK && !healthy_locked())
    {
      stop_locked();
      ret = -EIO;
    }
  if (ret == OK)
    {
      memcpy(g_neutral, neutral, sizeof(g_neutral));
      ret = wd_start(&g_watchdog, MSEC2TICK(PWM_TIMEOUT_MS), timeout, 0);
      commit(ret < 0 ? neutral : pulse);
      memcpy(g_status.pulse_us, ret < 0 ? neutral : pulse, sizeof(g_neutral));
    }
  g_status.last_error = ret;
  leave_critical_section(flags);
  return ret;
}

bool board_matek_steering_pwm_healthy(void)
{
  irqstate_t flags = enter_critical_section();
  bool healthy = healthy_locked();
  leave_critical_section(flags);
  return healthy;
}

void board_matek_pwm_status(struct board_matek_pwm_status_s *status)
{
  irqstate_t flags = enter_critical_section();
  *status = g_status;
  status->healthy = healthy_locked();
  leave_critical_section(flags);
}
#endif
