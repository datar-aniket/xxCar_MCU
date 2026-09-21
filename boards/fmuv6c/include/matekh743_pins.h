/****************************************************************************
 * boards/fmuv6c/include/matekh743_pins.h
 * SPDX-License-Identifier: Apache-2.0
 *
 * H743-SLIM-V4 peripheral allocation. Included by board.h; GPIO encodings
 * come from the STM32H7 headers at the point of use, not from this header.
 *
 * S1-S8 are the baseline servo allocation; S9-S12 are expansion. This map
 * does not itself activate outputs. The PWM driver can own any S1-S8 mask.
 * TIM5 remains pinless. PA8/TIM1 is reserved for the deferred PPS port.
 ****************************************************************************/

#ifndef __BOARDS_FMUV6C_INCLUDE_MATEKH743_PINS_H
#define __BOARDS_FMUV6C_INCLUDE_MATEKH743_PINS_H

/* Reject inherited board features that would steal routed resources. */

#if defined(CONFIG_STM32H7_I2C4) || defined(CONFIG_STM32H7_UART5) || \
    defined(CONFIG_STM32H7_SDMMC2)
#  error "Matek: I2C4/UART5/SDMMC2 conflict with servo, SD or IMU pins"
#endif
#if defined(CONFIG_ARCH_BUTTONS) || defined(CONFIG_DEV_GPIO) || \
    defined(CONFIG_WL_NRF24L01) || defined(CONFIG_STM32H7_ETHMAC)
#  error "Matek: generic button/GPIO/radio/Ethernet board mappings are unsupported"
#endif
#if defined(CONFIG_USBHOST) || defined(CONFIG_STM32H7_HOST)
#  error "Matek: USB host board pins conflict with RC; use USB device mode"
#endif
#if defined(CONFIG_STM32H7_OTGFS) && !defined(CONFIG_OTG_ID_GPIO_DISABLE)
#  error "Matek: enable OTG_ID_GPIO_DISABLE; USB ID would steal USART1 RX PA10"
#endif
#if defined(CONFIG_USBDEV_VBUSSENSING) || defined(CONFIG_STM32H7_OTG_SOFOUTPUT)
#  error "Matek: USB VBUS/SOF pins conflict with USART1 TX / reserved PPS"
#endif
#if defined(CONFIG_STM32H7_TIM1) || defined(CONFIG_STM32H7_TIM2) || \
    defined(CONFIG_STM32H7_TIM3) || defined(CONFIG_STM32H7_TIM5) || \
    defined(CONFIG_STM32H7_TIM6) || defined(CONFIG_STM32H7_TIM4) || \
    defined(CONFIG_STM32H7_TIM8)
#  error "Matek: TIM1 reserved for PPS; TIM2/3/4/5/6/8 have direct board owners"
#endif

/* PWM bank A: TIM8 complementary outputs; do not use TIM3 on these pads.
 * Bank B: TIM2, not TIM5. Bank C: TIM4 on port D, not I2C/UART4 port B.
 * Phase contract: independent banks; aligned rising edges within each bank.
 * PWM1, upcounting, positive pulses. S1/S2 use N-only outputs: CCxE=0,
 * CCxNE=1, CCxNP=0 (not inverted), as in ArduPilot's Matek definition.
 * Do not enable the unused main outputs: that changes N-output behavior.
 */

#if defined(CONFIG_STM32H7_TIM8_CH2OUT) || defined(CONFIG_STM32H7_TIM8_CH3OUT)
#  error "Matek: S1/S2 require TIM8 N-only outputs, not complementary pairs"
#endif
#if (defined(CONFIG_STM32H7_TIM8_CH2NPOL) && CONFIG_STM32H7_TIM8_CH2NPOL != 0) || \
    (defined(CONFIG_STM32H7_TIM8_CH3NPOL) && CONFIG_STM32H7_TIM8_CH3NPOL != 0)
#  error "Matek: S1/S2 N-only servo outputs require non-inverted polarity"
#endif
#if (defined(CONFIG_STM32H7_TIM8_CH2MODE) && CONFIG_STM32H7_TIM8_CH2MODE != 6) || \
    (defined(CONFIG_STM32H7_TIM8_CH3MODE) && CONFIG_STM32H7_TIM8_CH3MODE != 6)
#  error "Matek: S1/S2 servo channels require PWM1 mode"
#endif
#if (defined(CONFIG_STM32H7_TIM8_MODE) && CONFIG_STM32H7_TIM8_MODE != 0) || \
    (defined(CONFIG_STM32H7_TIM8_DEADTIME) && CONFIG_STM32H7_TIM8_DEADTIME != 0) || \
    (defined(CONFIG_STM32H7_TIM8_CH2NIDLE) && CONFIG_STM32H7_TIM8_CH2NIDLE != 0) || \
    (defined(CONFIG_STM32H7_TIM8_CH3NIDLE) && CONFIG_STM32H7_TIM8_CH3NIDLE != 0)
#  error "Matek: S1/S2 require upcounting, zero dead time and low idle levels"
#endif

#define GPIO_TIM8_CH2NOUT (GPIO_TIM8_CH2NOUT_1 | GPIO_SPEED_50MHz)
#define GPIO_TIM8_CH3NOUT (GPIO_TIM8_CH3NOUT_1 | GPIO_SPEED_50MHz)
#define GPIO_TIM2_CH1OUT  (GPIO_TIM2_CH1OUT_1  | GPIO_SPEED_50MHz)
#define GPIO_TIM2_CH2OUT  (GPIO_TIM2_CH2OUT_1  | GPIO_SPEED_50MHz)
#define GPIO_TIM2_CH3OUT  (GPIO_TIM2_CH3OUT_1  | GPIO_SPEED_50MHz)
#define GPIO_TIM2_CH4OUT  (GPIO_TIM2_CH4OUT_1  | GPIO_SPEED_50MHz)
#define GPIO_TIM4_CH1OUT  (GPIO_TIM4_CH1OUT_2  | GPIO_SPEED_50MHz)
#define GPIO_TIM4_CH2OUT  (GPIO_TIM4_CH2OUT_2  | GPIO_SPEED_50MHz)
#define GPIO_TIM4_CH3OUT  (GPIO_TIM4_CH3OUT_2  | GPIO_SPEED_50MHz)
#define GPIO_TIM4_CH4OUT  (GPIO_TIM4_CH4OUT_2  | GPIO_SPEED_50MHz)
#define GPIO_TIM15_CH1OUT (GPIO_TIM15_CH1OUT_2 | GPIO_SPEED_50MHz)
#define GPIO_TIM15_CH2OUT (GPIO_TIM15_CH2OUT_2 | GPIO_SPEED_50MHz)

#define BOARD_MATEK_PWM_S3 GPIO_TIM2_CH1OUT
#define BOARD_MATEK_PWM_S4 GPIO_TIM2_CH2OUT

/* X(physical S number, timer, channel, complementary, GPIO).
 * A timer bank shares its frame rate. Complementary outputs require explicit
 * polarity/idle/MOE configuration in the subsequent driver integration.
 */

#define BOARD_MATEK_PWM_PINS(X) \
  X(1, 8, 2, 1, GPIO_TIM8_CH2NOUT) \
  X(2, 8, 3, 1, GPIO_TIM8_CH3NOUT) \
  X(3, 2, 1, 0, BOARD_MATEK_PWM_S3) \
  X(4, 2, 2, 0, BOARD_MATEK_PWM_S4) \
  X(5, 2, 3, 0, GPIO_TIM2_CH3OUT) \
  X(6, 2, 4, 0, GPIO_TIM2_CH4OUT) \
  X(7, 4, 1, 0, GPIO_TIM4_CH1OUT) \
  X(8, 4, 2, 0, GPIO_TIM4_CH2OUT)

#define BOARD_MATEK_PWM_EXPANSION_PINS(X) \
  X(9,  4, 3, 0, GPIO_TIM4_CH3OUT) \
  X(10, 4, 4, 0, GPIO_TIM4_CH4OUT) \
  X(11, 15, 1, 0, GPIO_TIM15_CH1OUT) \
  X(12, 15, 2, 0, GPIO_TIM15_CH2OUT)

#define BOARD_MATEK_PIN_ID(gpio) ((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK))
#define BOARD_MATEK_OUTPUT_LOW(gpio) \
  (BOARD_MATEK_PIN_ID(gpio) | GPIO_OUTPUT | GPIO_PUSHPULL | \
   GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR)

/* Never mux the buzzer pad to TIM2 CH1: that channel now drives S3. */

#define BOARD_MATEK_BUZZER_OFF \
  BOARD_MATEK_OUTPUT_LOW(GPIO_PORTA | GPIO_PIN15)

/* Reserved only: the TIM1 timestamp/capture implementation comes last.
 * No TIM5 input mapping is exported, and legacy S3 PPS is not built.
 */

#define BOARD_MATEK_PPS_RESERVED (GPIO_TIM1_CH1IN_1 | GPIO_PULLDOWN)
#define BOARD_MATEK_PPS_IDLE \
  (BOARD_MATEK_PIN_ID(BOARD_MATEK_PPS_RESERVED) | GPIO_INPUT | GPIO_PULLDOWN)

/* R6 is the explicit USART6/PPM alternate pad, not a PWM resource. */

#define GPIO_TIM3_CH2IN (GPIO_TIM3_CH2IN_3 | GPIO_PULLDOWN)
#define BOARD_MATEK_PPM_GPIO GPIO_TIM3_CH2IN

/* Physical UART routing; retain existing tty numbering and logical roles. */

#define GPIO_USART1_TX (GPIO_USART1_TX_2 | GPIO_SPEED_100MHz) /* PA9 */
#define GPIO_USART1_RX (GPIO_USART1_RX_2 | GPIO_SPEED_100MHz) /* PA10 */
#define GPIO_USART2_TX (GPIO_USART2_TX_2 | GPIO_SPEED_100MHz) /* PD5 */
#define GPIO_USART2_RX (GPIO_USART2_RX_2 | GPIO_SPEED_100MHz) /* PD6 */
#define GPIO_USART3_TX (GPIO_USART3_TX_3 | GPIO_SPEED_100MHz) /* PD8 */
#define GPIO_USART3_RX (GPIO_USART3_RX_3 | GPIO_SPEED_100MHz) /* PD9 */
#define GPIO_UART4_TX  (GPIO_UART4_TX_3  | GPIO_SPEED_100MHz) /* PB9 */
#define GPIO_UART4_RX  (GPIO_UART4_RX_3  | GPIO_SPEED_100MHz) /* PB8 */
#define GPIO_USART6_TX (GPIO_USART6_TX_1 | GPIO_SPEED_100MHz) /* PC6 */
#define GPIO_USART6_RX (GPIO_USART6_RX_1 | GPIO_SPEED_100MHz) /* PC7 */
#define GPIO_UART7_TX  (GPIO_UART7_TX_3  | GPIO_SPEED_100MHz) /* PE8 */
#define GPIO_UART7_RX  (GPIO_UART7_RX_3  | GPIO_SPEED_100MHz) /* PE7 */
#define GPIO_UART8_TX  (GPIO_UART8_TX_1  | GPIO_SPEED_100MHz) /* PE1 */
#define GPIO_UART8_RX  (GPIO_UART8_RX_1  | GPIO_SPEED_100MHz) /* PE0 */

#define DMAMAP_USART2_RX DMAMAP_DMA12_USART2RX_1
#define DMAMAP_USART3_RX DMAMAP_DMA12_USART3RX_0
#define DMAMAP_USART3_TX DMAMAP_DMA12_USART3TX_1
#define DMAMAP_UART4_RX  DMAMAP_DMA12_UART4RX_1
#define DMAMAP_USART6_RX DMAMAP_DMA12_USART6RX_0
#define DMAMAP_USART6_TX DMAMAP_DMA12_USART6TX_0

/* External I2C1 and onboard barometer I2C2 remain separate buses. */

#define GPIO_I2C1_SCL (GPIO_I2C1_SCL_1 | GPIO_SPEED_50MHz) /* PB6 */
#define GPIO_I2C1_SDA (GPIO_I2C1_SDA_1 | GPIO_SPEED_50MHz) /* PB7 */
#define GPIO_I2C2_SCL (GPIO_I2C2_SCL_1 | GPIO_SPEED_50MHz) /* PB10 */
#define GPIO_I2C2_SDA (GPIO_I2C2_SDA_1 | GPIO_SPEED_50MHz) /* PB11 */

#define GPIO_SPI1_SCK  (GPIO_SPI1_SCK_1  | GPIO_SPEED_50MHz) /* PA5 */
#define GPIO_SPI1_MISO (GPIO_SPI1_MISO_1 | GPIO_SPEED_50MHz) /* PA6 */
#define GPIO_SPI1_MOSI (GPIO_SPI1_MOSI_3 | GPIO_SPEED_50MHz) /* PD7 */
#define GPIO_SPI4_SCK  (GPIO_SPI4_SCK_1  | GPIO_SPEED_50MHz) /* PE12 */
#define GPIO_SPI4_MISO (GPIO_SPI4_MISO_1 | GPIO_SPEED_50MHz) /* PE13 */
#define GPIO_SPI4_MOSI (GPIO_SPI4_MOSI_1 | GPIO_SPEED_50MHz) /* PE14 */

/* Only routed analog inputs, never the inherited PWM/IMU-SPI alternatives.
 * X(name, ADC1 channel, GPIO, board divider multiplier).
 */

#define GPIO_ADC123_INP10 GPIO_ADC123_INP10_0 /* PC0 */
#define GPIO_ADC123_INP11 GPIO_ADC123_INP11_0 /* PC1 */
#define GPIO_ADC12_INP18  GPIO_ADC12_INP18_0  /* PA4 */
#define GPIO_ADC12_INP7   GPIO_ADC12_INP7_0   /* PA7 */
#define GPIO_ADC12_INP4   GPIO_ADC12_INP4_0   /* PC4 */
#define GPIO_ADC12_INP8   GPIO_ADC12_INP8_0   /* PC5 */

#define BOARD_MATEK_ADC_PINS(X) \
  X(VBAT, 10, GPIO_ADC123_INP10, 11) \
  X(CURR, 11, GPIO_ADC123_INP11, 1) \
  X(VB2,  18, GPIO_ADC12_INP18, 21) \
  X(CUR2,  7, GPIO_ADC12_INP7, 1) \
  X(AIRS,  4, GPIO_ADC12_INP4, 2) \
  X(RSSI,  8, GPIO_ADC12_INP8, 1)

#define GPIO_OTGFS_DM (GPIO_OTGFS_DM_0 | GPIO_SPEED_100MHz)
#define GPIO_OTGFS_DP (GPIO_OTGFS_DP_0 | GPIO_SPEED_100MHz)

#endif /* __BOARDS_FMUV6C_INCLUDE_MATEKH743_PINS_H */
