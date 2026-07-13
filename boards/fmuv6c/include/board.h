/****************************************************************************
 * boards/arm/stm32h7/nucleo-h743zi/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32H7_NUCLEO_H743ZI_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32H7_NUCLEO_H743ZI_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdbool.h>
#  include <stdint.h>
#endif

/* Do not include STM32 H7 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The px4_fmu-v6C (Pixhawk 6C / FMUv6C) board provides the following clock
 * sources:
 *
 *   X1: 16 MHz crystal for HSE
 *
 * So we have these clock sources available within the STM32:
 *
 *   HSI: 16 MHz RC factory-trimmed
 *   HSE: 16 MHz crystal
 *
 * NOTE: The clock tree below (PLL1/2/3, HCLK/PCLK, kernel clock sources) is
 * transplanted verbatim from PX4's verified fmu-v6c board.h and is authoritative
 * for this hardware. The peripheral PIN definitions further down are still the
 * nucleo-h743zi placeholders and MUST be replaced with the FMUv6C schematic pins
 * during on-hardware bring-up (USB CDC console works regardless, as it is fixed
 * to OTG_FS PA11/PA12).  TODO(hw): port pin-mux from PX4 fmu-v6c board.h.
 */

#define STM32_BOARD_XTAL        16000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE = 16,000,000
 *
 * PLL_VCOx = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *
 *     1 <= PLLM <= 63
 *     4 <= PLLN <= 512
 *   150 MHz <= PLL_VCOL <= 420MHz
 *   192 MHz <= PLL_VCOH <= 836MHz
 *
 * SYSCLK  = PLL_VCO / PLLP
 * CPUCLK  = SYSCLK / D1CPRE
 * Subject to
 *
 *   PLLP1   = {2, 4, 6, 8, ..., 128}
 *   PLLP2,3 = {2, 3, 4, ..., 128}
 *   CPUCLK <= 480 MHz
 */

#define STM32_BOARD_USEHSE

#define STM32_PLLCFG_PLLSRC      RCC_PLLCKSELR_PLLSRC_HSE

/* PLL1, wide 4 - 8 MHz input, enable DIVP, DIVQ, DIVR
 *
 *   PLL1_VCO = (16,000,000 / 1) * 60 = 960 MHz
 *
 *   PLL1P = PLL1_VCO/2  = 960 MHz / 2   = 480 MHz
 *   PLL1Q = PLL1_VCO/4  = 960 MHz / 4   = 240 MHz
 *   PLL1R = PLL1_VCO/8  = 960 MHz / 8   = 120 MHz
 */

#define STM32_PLLCFG_PLL1CFG    (RCC_PLLCFGR_PLL1VCOSEL_WIDE | \
                                 RCC_PLLCFGR_PLL1RGE_4_8_MHZ | \
                                 RCC_PLLCFGR_DIVP1EN | \
                                 RCC_PLLCFGR_DIVQ1EN | \
                                 RCC_PLLCFGR_DIVR1EN)
#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(1)
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(60)
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(4)
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(8)

#define STM32_VCO1_FREQUENCY     ((STM32_HSE_FREQUENCY / 1) * 60)
#define STM32_PLL1P_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)
#define STM32_PLL1Q_FREQUENCY    (STM32_VCO1_FREQUENCY / 4)
#define STM32_PLL1R_FREQUENCY    (STM32_VCO1_FREQUENCY / 8)

/* PLL2 */

#define STM32_PLLCFG_PLL2CFG     (RCC_PLLCFGR_PLL2VCOSEL_WIDE | \
                                  RCC_PLLCFGR_PLL2RGE_4_8_MHZ | \
                                  RCC_PLLCFGR_DIVP2EN | \
                                  RCC_PLLCFGR_DIVQ2EN | \
                                  RCC_PLLCFGR_DIVR2EN)
#define STM32_PLLCFG_PLL2M       RCC_PLLCKSELR_DIVM2(4)
#define STM32_PLLCFG_PLL2N       RCC_PLL2DIVR_N2(48)
#define STM32_PLLCFG_PLL2P       RCC_PLL2DIVR_P2(2)
#define STM32_PLLCFG_PLL2Q       RCC_PLL2DIVR_Q2(2)
#define STM32_PLLCFG_PLL2R       RCC_PLL2DIVR_R2(2)

#define STM32_VCO2_FREQUENCY     ((STM32_HSE_FREQUENCY / 4) * 48)
#define STM32_PLL2P_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)
#define STM32_PLL2Q_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)
#define STM32_PLL2R_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)

/* PLL3 is DISABLED. FMUv6C/PX4 clock USB from PLL3Q, but NuttX's H7 RCC waits
 * for PLL3RDY in an unbounded loop, so a mis-lock would hang boot before USB
 * ever comes up. We instead clock USB from the internal HSI48 (see USBSRC
 * below + CONFIG_STM32H7_HSI48), which always locks -- nucleo-h743zi does the
 * same. Nothing else on this board uses PLL3. TODO: revisit if a peripheral
 * needs a PLL3 output.
 */

#define STM32_PLLCFG_PLL3CFG 0
#define STM32_PLLCFG_PLL3M   0
#define STM32_PLLCFG_PLL3N   0
#define STM32_PLLCFG_PLL3P   0
#define STM32_PLLCFG_PLL3Q   0
#define STM32_PLLCFG_PLL3R   0

#define STM32_VCO3_FREQUENCY
#define STM32_PLL3P_FREQUENCY
#define STM32_PLL3Q_FREQUENCY
#define STM32_PLL3R_FREQUENCY

/* SYSCLK = PLL1P = 480 MHz
 * CPUCLK = SYSCLK / 1 = 480 MHz
 */

#define STM32_RCC_D1CFGR_D1CPRE  (RCC_D1CFGR_D1CPRE_SYSCLK)
#define STM32_SYSCLK_FREQUENCY   (STM32_PLL1P_FREQUENCY)
#define STM32_CPUCLK_FREQUENCY   (STM32_SYSCLK_FREQUENCY / 1)

/* Configure Clock Assignments */

/* AHB clock (HCLK) is SYSCLK/2 (240 MHz max)
 * HCLK1 = HCLK2 = HCLK3 = HCLK4 = 240 MHz
 */

#define STM32_RCC_D1CFGR_HPRE   RCC_D1CFGR_HPRE_SYSCLKd2        /* HCLK  = SYSCLK / 2 */
#define STM32_ACLK_FREQUENCY    (STM32_CPUCLK_FREQUENCY / 2)    /* ACLK in D1, HCLK3 in D1 */
#define STM32_HCLK_FREQUENCY    (STM32_CPUCLK_FREQUENCY / 2)    /* HCLK in D2, HCLK4 in D3 */
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY            /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/2 (120 MHz) */

#define STM32_RCC_D2CFGR_D2PPRE1  RCC_D2CFGR_D2PPRE1_HCLKd2       /* PCLK1 = HCLK / 2 */
#define STM32_PCLK1_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB2 clock (PCLK2) is HCLK/2 (120 MHz) */

#define STM32_RCC_D2CFGR_D2PPRE2  RCC_D2CFGR_D2PPRE2_HCLKd2       /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB3 clock (PCLK3) is HCLK/2 (120 MHz) */

#define STM32_RCC_D1CFGR_D1PPRE   RCC_D1CFGR_D1PPRE_HCLKd2        /* PCLK3 = HCLK / 2 */
#define STM32_PCLK3_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB4 clock (PCLK4) is HCLK/2 (120 MHz) */

#define STM32_RCC_D3CFGR_D3PPRE   RCC_D3CFGR_D3PPRE_HCLKd2       /* PCLK4 = HCLK / 2 */
#define STM32_PCLK4_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* Timer clock frequencies */

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM15_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM16_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM17_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Kernel Clock Configuration
 *
 * Note: look at Table 54 in ST Manual
 */

/* I2C123 clock source - HSI */

#define STM32_RCC_D2CCIP2R_I2C123SRC RCC_D2CCIP2R_I2C123SEL_HSI

/* I2C4 clock source - HSI */

#define STM32_RCC_D3CCIPR_I2C4SRC    RCC_D3CCIPR_I2C4SEL_HSI

/* SPI123 clock source - PLL2 */

#define STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL2

/* SPI45 clock source - PLL2 */

#define STM32_RCC_D2CCIP1R_SPI45SRC  RCC_D2CCIP1R_SPI45SEL_PLL2

/* SPI6 clock source - PLL2 */

#define STM32_RCC_D3CCIPR_SPI6SRC    RCC_D3CCIPR_SPI6SEL_PLL2

/* USB 1 and 2 clock source - HSI48 (internal 48 MHz RC; see PLL3 note above) */

#define STM32_RCC_D2CCIP2R_USBSRC    RCC_D2CCIP2R_USBSEL_HSI48

/* UART clock selection - reset to default RCC (overwrite any bootloader change) */

#define STM32_RCC_D2CCIP2R_USART234578_SEL RCC_D2CCIP2R_USART234578SEL_RCC
#define STM32_RCC_D2CCIP2R_USART16_SEL     RCC_D2CCIP2R_USART16SEL_RCC

/* ADC 1 2 3 clock source - pll2_pclk */

#define STM32_RCC_D3CCIPR_ADCSRC     RCC_D3CCIPR_ADCSEL_PLL2

/* FDCAN 1 2 clock source - HSE (16 MHz) */

#define STM32_RCC_D2CCIP1R_FDCANSEL  RCC_D2CCIP1R_FDCANSEL_HSE
#define STM32_FDCANCLK               STM32_HSE_FREQUENCY

/* FLASH wait states
 *
 *  ------------ ---------- -----------
 *  Vcore        MAX ACLK   WAIT STATES
 *  ------------ ---------- -----------
 *  1.15-1.26 V     70 MHz    0
 *  (VOS1 level)   140 MHz    1
 *                 210 MHz    2
 *  1.05-1.15 V     55 MHz    0
 *  (VOS2 level)   110 MHz    1
 *                 165 MHz    2
 *                 220 MHz    3
 *  0.95-1.05 V     45 MHz    0
 *  (VOS3 level)    90 MHz    1
 *                 135 MHz    2
 *                 180 MHz    3
 *                 225 MHz    4
 *  ------------ ---------- -----------
 */

#define BOARD_FLASH_WAITSTATES 2

/* SDMMC definitions ********************************************************/

/* Clock dividers use PX4's values for this exact board (fmu-v6c): init well
 * under 400 kHz, transfers at PLL1Q/(2*5). These are more conservative than the
 * generic template values and are hardware-proven on the 6C.
 */

#define STM32_SDMMC_INIT_CLKDIV     (300 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

#if defined(CONFIG_STM32H7_SDMMC_IDMA)
#  define STM32_SDMMC_MMCXFR_CLKDIV (5 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#  define STM32_SDMMC_SDXFR_CLKDIV  (5 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_MMCXFR_CLKDIV (100 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#  define STM32_SDMMC_SDXFR_CLKDIV  (100 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

#define STM32_SDMMC_CLKCR_EDGE      STM32_SDMMC_CLKCR_NEGEDGE

/* SDMMC2 = the microSD slot. There is no card-detect line on the FMUv6C, so
 * the board logic reports the card as always present.
 *
 *   SDMMC2_CK   PD6     SDMMC2_D0   PB14
 *   SDMMC2_CMD  PD7     SDMMC2_D1   PB15
 *                       SDMMC2_D2   PB3
 *                       SDMMC2_D3   PB4
 *
 * With the non-legacy pinmap every SDMMC pin must be selected explicitly here,
 * even the ones that have only a single option.
 */

#define GPIO_SDMMC2_CK    GPIO_SDMMC2_CK_1   /* PD6  */
#define GPIO_SDMMC2_CMD   GPIO_SDMMC2_CMD_1  /* PD7  */
#define GPIO_SDMMC2_D0    GPIO_SDMMC2_D0_0   /* PB14 */
#define GPIO_SDMMC2_D1    GPIO_SDMMC2_D1_0   /* PB15 */
#define GPIO_SDMMC2_D2    GPIO_SDMMC2_D2_2   /* PB3  */
#define GPIO_SDMMC2_D3    GPIO_SDMMC2_D3_0   /* PB4  */

/* Ethernet definitions *****************************************************/

#define GPIO_ETH_RMII_TXD0    (GPIO_ETH_RMII_TXD0_2 | GPIO_SPEED_100MHz)    /* PG13 */
#define GPIO_ETH_RMII_TXD1    (GPIO_ETH_RMII_TXD1_1 | GPIO_SPEED_100MHz)    /* PB13 */
#define GPIO_ETH_RMII_TX_EN   (GPIO_ETH_RMII_TX_EN_2 | GPIO_SPEED_100MHz)   /* PG11 */
#define GPIO_ETH_MDC          (GPIO_ETH_MDC_0 | GPIO_SPEED_100MHz)          /* PC1 */
#define GPIO_ETH_MDIO         (GPIO_ETH_MDIO_0 | GPIO_SPEED_100MHz)         /* PA2 */
#define GPIO_ETH_RMII_RXD0    (GPIO_ETH_RMII_RXD0_0 | GPIO_SPEED_100MHz)    /* PC4 */
#define GPIO_ETH_RMII_RXD1    (GPIO_ETH_RMII_RXD1_0 | GPIO_SPEED_100MHz)    /* PC5 */
#define GPIO_ETH_RMII_CRS_DV  (GPIO_ETH_RMII_CRS_DV_0 | GPIO_SPEED_100MHz)  /* PA7 */
#define GPIO_ETH_RMII_REF_CLK (GPIO_ETH_RMII_REF_CLK_0 | GPIO_SPEED_100MHz) /* PA1 */

/* LED definitions **********************************************************/

/* The Nucleo-144 board has numerous LEDs but only three, LD1 a Green LED,
 * LD2 a Blue LED and LD3 a Red LED, that can be controlled by software.
 * The following definitions assume the default Solder Bridges are installed.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_NLEDS       3

#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_BLUE    BOARD_LED2
#define BOARD_LED_RED     BOARD_LED3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/stm32_leds.c.
 * The LEDs are used to encode OS-related events as follows:
 *
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        Red   Green Blue
 *   ----------------------  --------------------------  ------ ------ ---
 */

#define LED_STARTED        0 /* NuttX has been started   OFF    OFF   OFF  */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  OFF    OFF   ON   */
#define LED_IRQSENABLED    2 /* Interrupts enabled       OFF    ON    OFF  */
#define LED_STACKCREATED   3 /* Idle stack created       OFF    ON    ON   */
#define LED_INIRQ          4 /* In an interrupt          N/C    N/C   GLOW */
#define LED_SIGNAL         5 /* In a signal handler      N/C    GLOW  N/C  */
#define LED_ASSERTION      6 /* An assertion failed      GLOW   N/C   GLOW */
#define LED_PANIC          7 /* The system has crashed   Blink  OFF   N/C  */
#define LED_IDLE           8 /* MCU is is sleep mode     ON     OFF   OFF  */

/* Thus if the Green LED is statically on, NuttX has successfully booted and
 * is, apparently, running normally.  If the Red LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 */

/* Button definitions *******************************************************/

/* The NUCLEO board supports one button:  Pushbutton B1, labeled "User", is
 * connected to GPIO PI11.
 * A high value will be sensed when the button is depressed.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1
#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* ADC */

#define GPIO_ADC12_INP5   GPIO_ADC12_INP5_0                      /* PB1, channel 5 */
#define GPIO_ADC123_INP10 GPIO_ADC123_INP10_0                    /* PC0, channel 10 */
#define GPIO_ADC123_INP12 GPIO_ADC123_INP12_0                    /* PC2, channel 12 */
#define GPIO_ADC12_INP13  GPIO_ADC12_INP13_0                     /* PC3, channel 13 */
#define GPIO_ADC12_INP15  GPIO_ADC12_INP15_0                     /* PA3, channel 15 */
#define GPIO_ADC12_INP18  GPIO_ADC12_INP18_0                     /* PA4, channel 18 */
#define GPIO_ADC12_INP19  GPIO_ADC12_INP19_0                     /* PA5, channel 19 */
#define GPIO_ADC123_INP7  GPIO_ADC12_INP7_0                      /* PA7, channel 7 */
#define GPIO_ADC123_INP11 GPIO_ADC123_INP11_0                    /* PC1, channel 11 */
#define GPIO_ADC2_INP2    GPIO_ADC2_INP2_0                       /* PF13, channel 2 */
#define GPIO_ADC12_INP3   GPIO_ADC12_INP3_0                      /* PA6, channel 3  */
#define GPIO_ADC12_INP14  GPIO_ADC12_INP14_0                     /* PA2, channel 14 */
#define GPIO_ADC12_INP4   GPIO_ADC12_INP4_0                      /* PC4, channel 4  */
#define GPIO_ADC12_INP8   GPIO_ADC12_INP8_0                      /* PC5, channel 8  */

/* Serial ports ************************************************************/

/* The FMUv6C connector <-> UART map. Pin assignments are cross-checked against
 * PX4's boards/px4/fmu-v6c/nuttx-config/include/board.h.
 *
 * CONFIG_STM32H7_SERIAL_DISABLE_REORDERING=y is set in the defconfig, which is
 * what makes the /dev/ttySn numbering below true and stable: without it NuttX
 * renumbers the ports so that the console becomes ttyS0, and every other index
 * shifts depending on which peripherals happen to be enabled. The serial port
 * manager hands these device names out by parameter, so they must not move.
 *
 *   Connector    UART     Pins          Device       Notes
 *   ---------    ------   -----------   ----------   ----------------------
 *   GPS1         USART1   PB6 / PA10    /dev/ttyS0
 *   TELEM3       USART2   PD5 / PA3     /dev/ttyS1
 *   FMU DEBUG    USART3   PD8 / PD9     /dev/ttyS2
 *   TELEM2       UART5    PC12 / PD2    /dev/ttyS3
 *   (internal)   USART6   PC6 / PC7     /dev/ttyS4   PX4IO co-processor link
 *   TELEM1       UART7    PE8 / PE7     /dev/ttyS5   NSH console by default
 *   GPS2         UART8    PE1 / PE0     /dev/ttyS6
 *
 * Note the RC IN connector is NOT on this list: on the 6C it is wired to the
 * PX4IO co-processor, not to the FMU. RC from that connector arrives over the
 * USART6 link as PX4IO registers, not as a raw SBUS/CRSF byte stream. Direct
 * UART RC (SBUS/CRSF) is still possible on any of the connectors above.
 */

/* USART1 = GPS1 */

#define GPIO_USART1_RX    (GPIO_USART1_RX_2 | GPIO_SPEED_100MHz) /* PA10 */
#define GPIO_USART1_TX    (GPIO_USART1_TX_3 | GPIO_SPEED_100MHz) /* PB6 */

/* USART2 = TELEM3 */

#define GPIO_USART2_RX    (GPIO_USART2_RX_1 | GPIO_SPEED_100MHz) /* PA3 */
#define GPIO_USART2_TX    (GPIO_USART2_TX_2 | GPIO_SPEED_100MHz) /* PD5 */

/* USART3 = FMU DEBUG connector */

#define GPIO_USART3_RX    (GPIO_USART3_RX_3 | GPIO_SPEED_100MHz) /* PD9 */
#define GPIO_USART3_TX    (GPIO_USART3_TX_3 | GPIO_SPEED_100MHz) /* PD8 */

#define DMAMAP_USART3_RX DMAMAP_DMA12_USART3RX_0
#define DMAMAP_USART3_TX DMAMAP_DMA12_USART3TX_1

/* UART5 = TELEM2 */

#define GPIO_UART5_RX     (GPIO_UART5_RX_3 | GPIO_SPEED_100MHz) /* PD2 */
#define GPIO_UART5_TX     (GPIO_UART5_TX_3 | GPIO_SPEED_100MHz) /* PC12 */

/* USART6 = link to the PX4IO co-processor (STM32F103).
 *
 * Runs at 1.5 Mbaud, 8N1, full duplex - see apps/px4io/. That rate comes from
 * PX4's PX4IO_SERIAL_BITRATE and is fixed by the firmware already flashed on
 * the IO chip, so it is not configurable.
 *
 * RX DMA matters here. At 1.5 Mbaud a byte lands every 6.7us, and IO's reply to
 * an RC read is a ~52-byte uninterrupted burst. Interrupt-driven, that is one
 * IRQ per FIFO threshold crossing for every exchange; with RX DMA the whole
 * burst is written to memory by the DMA controller and the USART's IDLE-line
 * interrupt raises it in one go when the line goes quiet. That is the same
 * mechanism PX4 uses, and it is what makes a 400 Hz servo update rate cheap
 * rather than a stream of interrupts.
 *
 * Stream assignment matches PX4's boards/px4/fmu-v6c board_dma_map.h: USART6 on
 * DMA1, alongside SPI1 (which takes 2 of DMA1's 8 streams).
 */

#define GPIO_USART6_RX    (GPIO_USART6_RX_1 | GPIO_SPEED_100MHz) /* PC7 */
#define GPIO_USART6_TX    (GPIO_USART6_TX_1 | GPIO_SPEED_100MHz) /* PC6 */

#define DMAMAP_USART6_RX  DMAMAP_DMA12_USART6RX_0  /* DMA1:71 */
#define DMAMAP_USART6_TX  DMAMAP_DMA12_USART6TX_0  /* DMA1:72 */

/* UART7 = TELEM1. The NSH console by default (SER_TEL1_FUNC). No hardware flow
 * control, so a 3-wire (TX/RX/GND) USB-TTL adapter works. TELEM1's flow-control
 * lines are PE9/PE10 and are left unused.
 */

#define GPIO_UART7_RX     (GPIO_UART7_RX_3 | GPIO_SPEED_100MHz) /* PE7 */
#define GPIO_UART7_TX     (GPIO_UART7_TX_3 | GPIO_SPEED_100MHz) /* PE8 */

/* UART8 = GPS2 */

#define GPIO_UART8_RX     (GPIO_UART8_RX_1 | GPIO_SPEED_100MHz) /* PE0 */
#define GPIO_UART8_TX     (GPIO_UART8_TX_1 | GPIO_SPEED_100MHz) /* PE1 */

/* I2C1 - external / expansion bus (PB8/PB7) */

#define GPIO_I2C1_SCL     (GPIO_I2C1_SCL_2 | GPIO_SPEED_50MHz) /* PB8 */
#define GPIO_I2C1_SDA     (GPIO_I2C1_SDA_1 | GPIO_SPEED_50MHz) /* PB7 */

/* I2C2 - external I2C connector (PB10/PB11) */

#define GPIO_I2C2_SCL     (GPIO_I2C2_SCL_1 | GPIO_SPEED_50MHz) /* PB10 */
#define GPIO_I2C2_SDA     (GPIO_I2C2_SDA_1 | GPIO_SPEED_50MHz) /* PB11 */

/* I2C4 - INTERNAL bus: onboard barometer (MS5611) + magnetometer (IST8310)
 * + calibration EEPROM. This is where Task 1 sensor discovery scans.
 */

#define GPIO_I2C4_SCL     (GPIO_I2C4_SCL_1 | GPIO_SPEED_50MHz) /* PD12 */
#define GPIO_I2C4_SDA     (GPIO_I2C4_SDA_1 | GPIO_SPEED_50MHz) /* PD13 */

/* SPI1 - INTERNAL sensor bus: ICM-42688-P + BMI088 IMUs (PA5/PA6/PA7).
 * Chip-select and DRDY GPIOs are in src/fmuv6c.h, driven by stm32_spi.c.
 */

#define GPIO_SPI1_SCK     (GPIO_SPI1_SCK_1  | GPIO_SPEED_50MHz) /* PA5 */
#define GPIO_SPI1_MISO    (GPIO_SPI1_MISO_1 | GPIO_SPEED_50MHz) /* PA6 */
#define GPIO_SPI1_MOSI    (GPIO_SPI1_MOSI_1 | GPIO_SPEED_50MHz) /* PA7 */

/* TIM1 - Advanced Timer 16-bit (4 channels) */
#define GPIO_TIM1_CH1IN   (GPIO_TIM1_CH1IN_2)   /* PE9  */
#define GPIO_TIM1_CH2IN   (GPIO_TIM1_CH2IN_2)   /* PE11 */
#define GPIO_TIM1_CH3IN   (GPIO_TIM1_CH3IN_2)   /* PE13 */
#define GPIO_TIM1_CH4IN   (GPIO_TIM1_CH4IN_2)   /* PE14 */

#define GPIO_TIM1_CH1OUT  (GPIO_TIM1_CH1OUT_2)  /* PE9  - D6 */
#define GPIO_TIM1_CH1NOUT (GPIO_TIM1_CH1NOUT_3) /* PE8  - D42 */
#define GPIO_TIM1_CH2OUT  (GPIO_TIM1_CH2OUT_2)  /* PE11 - D5 */
#define GPIO_TIM1_CH2NOUT (GPIO_TIM1_CH2NOUT_3) /* PE10 - D40 */
#define GPIO_TIM1_CH3OUT  (GPIO_TIM1_CH3OUT_2)  /* PE13 - D3 */
#define GPIO_TIM1_CH3NOUT (GPIO_TIM1_CH3NOUT_3) /* PE12 - D39 */
#define GPIO_TIM1_CH4OUT  (GPIO_TIM1_CH4OUT_2)  /* PE14 - D38 */

/* TIM2 - General Purpose 32-bit Timer (4 channels) */
#define GPIO_TIM2_CH1IN   (GPIO_TIM2_CH1IN_2)   /* PA15 */
#define GPIO_TIM2_CH2IN   (GPIO_TIM2_CH2IN_1)   /* PB3 */
#define GPIO_TIM2_CH3IN   (GPIO_TIM2_CH3IN_1)   /* PB10 */
#define GPIO_TIM2_CH4IN   (GPIO_TIM2_CH4IN_1)   /* PB11 */

/* TIM3 - General Purpose 16-bit Timer (4 channels) */
#define GPIO_TIM3_CH1IN   (GPIO_TIM3_CH1IN_1)   /* PA6 */
#define GPIO_TIM3_CH2IN   (GPIO_TIM3_CH2IN_1)   /* PA7 */
#define GPIO_TIM3_CH3IN   (GPIO_TIM3_CH3IN_1)   /* PB0 */
#define GPIO_TIM3_CH4IN   (GPIO_TIM3_CH4IN_1)   /* PB1 */

/* TIM4 - General Purpose 16-bit Timer (4 channels) */
#define GPIO_TIM4_CH1IN   (GPIO_TIM4_CH1IN_2)   /* PD12 */
#define GPIO_TIM4_CH2IN   (GPIO_TIM4_CH2IN_2)   /* PD13 */
#define GPIO_TIM4_CH3IN   (GPIO_TIM4_CH3IN_2)   /* PD14 */
#define GPIO_TIM4_CH4IN   (GPIO_TIM4_CH4IN_2)   /* PD15 */

/* TIM5 - General Purpose 32-bit Timer (4 channels) */
#define GPIO_TIM5_CH1IN   (GPIO_TIM5_CH1IN_2)
#define GPIO_TIM5_CH2IN   (GPIO_TIM5_CH2IN_2)
#define GPIO_TIM5_CH3IN   (GPIO_TIM5_CH3IN_2)
#define GPIO_TIM5_CH4IN   (GPIO_TIM5_CH4IN_2)

/* TIM6 - Basic 16-bit Timer (0 channels) */

/* TIM7 - Basic 16-bit Timer (0 channels) */

/* TIM8 - Advanced 16-bit Timer (4 channels) */
#define GPIO_TIM8_CH1IN   (GPIO_TIM8_CH1IN_1)
#define GPIO_TIM8_CH2IN   (GPIO_TIM8_CH2IN_1)
#define GPIO_TIM8_CH3IN   (GPIO_TIM8_CH3IN_1)
#define GPIO_TIM8_CH4IN   (GPIO_TIM8_CH4IN_1)

/* TIM12 - General purpose 16-bit Timer (2 channels) */
#define GPIO_TIM12_CH1IN  (GPIO_TIM12_CH1IN_1)
#define GPIO_TIM12_CH2IN  (GPIO_TIM12_CH2IN_1)

/* TIM13 - General purpose 16-bit Timer (1 channels) */
#define GPIO_TIM13_CH1IN  (GPIO_TIM13_CH1IN_1)

/* TIM14 - General purpose 16-bit Timer (1 channels) */
#define GPIO_TIM14_CH1IN  (GPIO_TIM14_CH1IN_1)

/* TIM15 - General purpose 16-bit Timer (2 channels) */
#define GPIO_TIM15_CH1IN  (GPIO_TIM15_CH1IN_1)
#define GPIO_TIM15_CH2IN  (GPIO_TIM15_CH2IN_1)

/* TIM16 - General purpose 16-bit Timer (1 channels) */
#define GPIO_TIM16_CH1IN  (GPIO_TIM16_CH1IN_1)

/* TIM17 - General purpose 16-bit Timer (1 channels) */
#define GPIO_TIM17_CH1IN  (GPIO_TIM17_CH1IN_1)

/* OTGFS */

#define GPIO_OTGFS_DM  (GPIO_OTGFS_DM_0  | GPIO_SPEED_100MHz)
#define GPIO_OTGFS_DP  (GPIO_OTGFS_DP_0  | GPIO_SPEED_100MHz)
#define GPIO_OTGFS_ID  (GPIO_OTGFS_ID_0  | GPIO_SPEED_100MHz)

/* DMA **********************************************************************/

#define DMAMAP_SPI1_RX DMAMAP_DMA12_SPI1RX_0 /* DMA1 - internal IMU bus */
#define DMAMAP_SPI1_TX DMAMAP_DMA12_SPI1TX_0 /* DMA1 - internal IMU bus */

#define DMAMAP_SPI3_RX DMAMAP_DMA12_SPI3RX_0 /* DMA1 */
#define DMAMAP_SPI3_TX DMAMAP_DMA12_SPI3TX_0 /* DMA1 */

#define DMAMAP_USART6_RX DMAMAP_DMA12_USART6RX_1
#define DMAMAP_USART6_TX DMAMAP_DMA12_USART6TX_0

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_USBMSC_COMPOSITE

/****************************************************************************
 * Name: fmuv6c_msc_export / fmuv6c_msc_release / fmuv6c_msc_is_exported
 *
 * Description:
 *   Hand the microSD to the USB host, or take it back. The CDC/ACM serial
 *   function of the composite device is unaffected either way - only the
 *   mass-storage media is toggled.
 *
 *   Exactly one side owns the card at a time: export() unmounts /fs/microsd
 *   locally BEFORE binding the LUN, and release() unbinds the LUN BEFORE
 *   remounting. Declared here (rather than in the private board header) so
 *   NSH applications can drive them; this is a flat build, so apps link
 *   directly against the board code.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure (-ENODEV if USB is not up,
 *   -EBUSY if something still holds /fs/microsd open).
 *
 ****************************************************************************/

int  fmuv6c_msc_export(void);
int  fmuv6c_msc_release(void);
bool fmuv6c_msc_is_exported(void);

#endif /* CONFIG_USBMSC_COMPOSITE */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32H7_NUCLEO_H743ZI_INCLUDE_BOARD_H */
