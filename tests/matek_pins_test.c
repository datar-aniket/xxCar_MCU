/* SPDX-License-Identifier: Apache-2.0
 * Compile-time checks using the actual NuttX GPIO/alternate-function map.
 */

#include <nuttx/config.h>

/* Test the shared header's Pixhawk branch without modifying the live config.
 * This is a header regression check, not a complete Pixhawk firmware build.
 */

#ifdef TEST_PIXHAWK_PINS
#  undef CONFIG_XXCAR_BOARD_MATEKH743
#else
#  ifndef CONFIG_XXCAR_BOARD_MATEKH743
#    error "Configure the Matek target before running the pin checks"
#  endif
#endif

#ifdef TEST_USB_ID_CONFLICT
#  undef CONFIG_OTG_ID_GPIO_DISABLE
#endif

#include "stm32_gpio.h"
#include <arch/board/board.h>

#define PIN(gpio) ((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK))
#define AF(gpio) ((gpio) & GPIO_AF_MASK)
#define CHECK_PIN(gpio, port, pin) \
  _Static_assert(PIN(gpio) == (GPIO_PORT##port | GPIO_PIN##pin), #gpio)

#ifdef TEST_PIXHAWK_PINS

CHECK_PIN(GPIO_USART1_TX, B, 6);
CHECK_PIN(GPIO_USART2_RX, A, 3);
CHECK_PIN(GPIO_UART5_TX, C, 12);
CHECK_PIN(GPIO_UART5_RX, D, 2);
CHECK_PIN(GPIO_I2C4_SCL, D, 12);
CHECK_PIN(GPIO_SPI1_MOSI, A, 7);
CHECK_PIN(GPIO_SDMMC2_CK, D, 6);
#  ifdef BOARD_MATEK_PWM_PINS
#    error "Matek pin map leaked into Pixhawk"
#  endif

#else

CHECK_PIN(GPIO_TIM8_CH2NOUT, B, 0);
CHECK_PIN(GPIO_TIM8_CH3NOUT, B, 1);
CHECK_PIN(BOARD_MATEK_PWM_S3, A, 0);
CHECK_PIN(BOARD_MATEK_PWM_S4, A, 1);
CHECK_PIN(GPIO_TIM2_CH3OUT, A, 2);
CHECK_PIN(GPIO_TIM2_CH4OUT, A, 3);
CHECK_PIN(GPIO_TIM4_CH1OUT, D, 12);
CHECK_PIN(GPIO_TIM4_CH2OUT, D, 13);
CHECK_PIN(GPIO_TIM4_CH3OUT, D, 14);
CHECK_PIN(GPIO_TIM4_CH4OUT, D, 15);
CHECK_PIN(GPIO_TIM15_CH1OUT, E, 5);
CHECK_PIN(GPIO_TIM15_CH2OUT, E, 6);
CHECK_PIN(BOARD_MATEK_PPS_RESERVED, A, 8);
CHECK_PIN(BOARD_MATEK_PPM_GPIO, C, 7);
CHECK_PIN(GPIO_USART1_TX, A, 9);
CHECK_PIN(GPIO_USART1_RX, A, 10);
CHECK_PIN(GPIO_USART2_RX, D, 6);
CHECK_PIN(GPIO_SPI1_MOSI, D, 7);
CHECK_PIN(GPIO_I2C1_SCL, B, 6);

_Static_assert(AF(GPIO_TIM8_CH2NOUT) == GPIO_AF3, "S1 AF");
_Static_assert(AF(GPIO_TIM8_CH3NOUT) == GPIO_AF3, "S2 AF");
_Static_assert(AF(BOARD_MATEK_PWM_S3) == GPIO_AF1, "S3 TIM2 AF");
_Static_assert(AF(BOARD_MATEK_PWM_S4) == GPIO_AF1, "S4 TIM2 AF");
_Static_assert(AF(GPIO_TIM2_CH3OUT) == GPIO_AF1, "S5 TIM2 AF");
_Static_assert(AF(GPIO_TIM2_CH4OUT) == GPIO_AF1, "S6 TIM2 AF");
_Static_assert(AF(GPIO_TIM4_CH1OUT) == GPIO_AF2, "S7 TIM4 AF");
_Static_assert(AF(GPIO_TIM4_CH2OUT) == GPIO_AF2, "S8 TIM4 AF");
_Static_assert(AF(BOARD_MATEK_PPS_RESERVED) == GPIO_AF1, "PPS AF");
_Static_assert(AF(BOARD_MATEK_PPM_GPIO) == GPIO_AF2, "PPM AF");

#define COUNT_PWM(s, timer, channel, comp, gpio) + 1
_Static_assert((0 BOARD_MATEK_PWM_PINS(COUNT_PWM)) == 8, "8 baseline pads");
_Static_assert((0 BOARD_MATEK_PWM_EXPANSION_PINS(COUNT_PWM)) == 4,
               "4 expansion pads");
#undef COUNT_PWM

#define CHECK_BANK(s, timer, channel, comp, gpio) \
  _Static_assert((timer) == ((s) <= 2 ? 8 : (s) <= 6 ? 2 : 4), \
                 "baseline timer bank"); \
  _Static_assert((channel) == ((s) <= 2 ? (s) + 1 : \
                              (s) <= 6 ? (s) - 2 : (s) - 6), \
                 "baseline channel"); \
  _Static_assert((comp) == ((s) <= 2), "complementary output metadata");
BOARD_MATEK_PWM_PINS(CHECK_BANK)
#undef CHECK_BANK

#define CHECK_IDLE(s, timer, channel, comp, gpio) \
  _Static_assert((BOARD_MATEK_OUTPUT_LOW(gpio) & GPIO_MODE_MASK) == \
                 GPIO_OUTPUT, "output idle mode"); \
  _Static_assert((BOARD_MATEK_OUTPUT_LOW(gpio) & \
                 (GPIO_AF_MASK | GPIO_OUTPUT_SET)) == 0, "low, no AF"); \
  _Static_assert(PIN(BOARD_MATEK_OUTPUT_LOW(gpio)) == PIN(gpio), "idle pin");
BOARD_MATEK_PWM_PINS(CHECK_IDLE)
BOARD_MATEK_PWM_EXPANSION_PINS(CHECK_IDLE)
#undef CHECK_IDLE

#define ADC_ENUM(name, channel, gpio, scale) \
  enum { ADC_CHANNEL_##name = channel, ADC_SCALE_##name = scale };
BOARD_MATEK_ADC_PINS(ADC_ENUM)
#undef ADC_ENUM
_Static_assert(ADC_CHANNEL_VBAT == 10 && ADC_SCALE_VBAT == 11, "VBAT");
_Static_assert(ADC_CHANNEL_VB2 == 18 && ADC_SCALE_VB2 == 21, "V4 VB2");
_Static_assert(ADC_CHANNEL_CUR2 == 7 && ADC_CHANNEL_RSSI == 8, "feedback");
_Static_assert(ADC_CHANNEL_AIRS == 4 && ADC_SCALE_AIRS == 2, "AirS");

#if defined(GPIO_TIM1_CH1OUT) || defined(GPIO_TIM5_CH1IN) || \
    defined(GPIO_TIM3_CH3IN) || defined(GPIO_TIM3_CH4IN) || \
    defined(GPIO_I2C4_SCL) || defined(GPIO_OTGFS_ID) || \
    defined(GPIO_ADC12_INP5) || defined(GPIO_ADC12_INP15) || \
    defined(GPIO_ADC12_INP19)
#  error "Conflicting template mappings leaked into Matek"
#endif

/* Duplicate case labels fail compilation: cover active buses, expansion
 * outputs, all ADC pads, IMU selects, status LEDs and reserved PPS/buzzer.
 * R6 PPM is the intentional alternate of USART6 RX and is checked above.
 */

void matek_check_unique_pins(unsigned int pin);
void matek_check_unique_pins(unsigned int pin)
{
  switch (pin)
    {
#define PWM_CASE(s, timer, channel, comp, gpio) case PIN(gpio): break;
      BOARD_MATEK_PWM_PINS(PWM_CASE)
      BOARD_MATEK_PWM_EXPANSION_PINS(PWM_CASE)
#undef PWM_CASE
#define ADC_CASE(name, channel, gpio, scale) case PIN(gpio): break;
      BOARD_MATEK_ADC_PINS(ADC_CASE)
#undef ADC_CASE
#define UNIQUE(gpio) case PIN(gpio): break
      UNIQUE(BOARD_MATEK_PPS_RESERVED);
      UNIQUE(BOARD_MATEK_BUZZER_OFF);
      UNIQUE(GPIO_USART1_TX); UNIQUE(GPIO_USART1_RX);
      UNIQUE(GPIO_USART2_TX); UNIQUE(GPIO_USART2_RX);
      UNIQUE(GPIO_USART3_TX); UNIQUE(GPIO_USART3_RX);
      UNIQUE(GPIO_UART4_TX);  UNIQUE(GPIO_UART4_RX);
      UNIQUE(GPIO_USART6_TX); UNIQUE(GPIO_USART6_RX);
      UNIQUE(GPIO_UART7_TX);  UNIQUE(GPIO_UART7_RX);
      UNIQUE(GPIO_UART8_TX);  UNIQUE(GPIO_UART8_RX);
      UNIQUE(GPIO_I2C1_SCL);  UNIQUE(GPIO_I2C1_SDA);
      UNIQUE(GPIO_I2C2_SCL);  UNIQUE(GPIO_I2C2_SDA);
      UNIQUE(GPIO_SPI1_SCK);  UNIQUE(GPIO_SPI1_MISO); UNIQUE(GPIO_SPI1_MOSI);
      UNIQUE(GPIO_SPI4_SCK);  UNIQUE(GPIO_SPI4_MISO); UNIQUE(GPIO_SPI4_MOSI);
      UNIQUE(GPIO_CAN1_RX);   UNIQUE(GPIO_CAN1_TX);
      UNIQUE(GPIO_OTGFS_DM);  UNIQUE(GPIO_OTGFS_DP);
      UNIQUE(GPIO_SDMMC1_CK); UNIQUE(GPIO_SDMMC1_CMD);
      UNIQUE(GPIO_SDMMC1_D0); UNIQUE(GPIO_SDMMC1_D1);
      UNIQUE(GPIO_SDMMC1_D2); UNIQUE(GPIO_SDMMC1_D3);
      UNIQUE(GPIO_PORTC | GPIO_PIN15); /* IMU1 CS */
      UNIQUE(GPIO_PORTC | GPIO_PIN13); /* IMU2 CS */
      UNIQUE(GPIO_PORTE | GPIO_PIN11); /* legacy CS */
      UNIQUE(GPIO_PORTB | GPIO_PIN2);  /* IMU1 DRDY */
      UNIQUE(GPIO_PORTD | GPIO_PIN3);  /* CAN silent */
      UNIQUE(GPIO_PORTE | GPIO_PIN3);  /* status LED */
      UNIQUE(GPIO_PORTE | GPIO_PIN4);  /* status LED */
#undef UNIQUE
      default: break;
    }
}
#endif
