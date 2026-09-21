#!/usr/bin/env bash
# Compile-time pin checks against the configured target's actual NuttX headers.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NUTTX="$REPO/deps/nuttx"
TEST_OUT="$(mktemp -d)"
trap 'rm -rf "$TEST_OUT"' EXIT

CC="${ARM_CC:-arm-none-eabi-gcc}"
FLAGS=(-std=c11 -Wall -Wextra -Werror -mcpu=cortex-m7 -mthumb
       -I"$NUTTX/include" -I"$NUTTX/arch/arm/src/stm32h7"
       -I"$NUTTX/arch/arm/src/common")
SOURCE="$REPO/tests/matek_pins_test.c"

"$CC" "${FLAGS[@]}" -c "$SOURCE" -o "$TEST_OUT/matek.o"
"$CC" "${FLAGS[@]}" -DTEST_PIXHAWK_PINS -c "$SOURCE" -o "$TEST_OUT/pixhawk.o"

for conflict in CONFIG_STM32H7_I2C4 CONFIG_STM32H7_UART5 \
                CONFIG_STM32H7_SDMMC2 CONFIG_ARCH_BUTTONS CONFIG_DEV_GPIO \
                CONFIG_USBHOST CONFIG_STM32H7_ETHMAC CONFIG_WL_NRF24L01 \
                CONFIG_STM32H7_TIM1 CONFIG_STM32H7_TIM2 CONFIG_STM32H7_TIM3 \
                CONFIG_STM32H7_TIM4 CONFIG_STM32H7_TIM5 CONFIG_STM32H7_TIM6 CONFIG_STM32H7_TIM8 \
                CONFIG_USBDEV_VBUSSENSING CONFIG_STM32H7_OTG_SOFOUTPUT \
                CONFIG_STM32H7_TIM8_CH2OUT CONFIG_STM32H7_TIM8_CH3OUT \
                CONFIG_STM32H7_TIM8_CH2NPOL CONFIG_STM32H7_TIM8_CH3NPOL \
                CONFIG_STM32H7_TIM8_CH2MODE CONFIG_STM32H7_TIM8_CH3MODE \
                CONFIG_STM32H7_TIM8_MODE CONFIG_STM32H7_TIM8_DEADTIME \
                CONFIG_STM32H7_TIM8_CH2NIDLE CONFIG_STM32H7_TIM8_CH3NIDLE \
                TEST_USB_ID_CONFLICT; do
  if "$CC" "${FLAGS[@]}" -D"$conflict"=1 -c "$SOURCE" \
       -o "$TEST_OUT/conflict.o" >"$TEST_OUT/diagnostic" 2>&1; then
    echo "FAIL: $conflict was not rejected" >&2
    exit 1
  fi
  grep -q 'error: #error "Matek:' "$TEST_OUT/diagnostic" || {
    echo "FAIL: unexpected compilation failure for $conflict" >&2
    sed -n '1,12p' "$TEST_OUT/diagnostic" >&2
    exit 1
  }
done
echo 'PASS: Matek pins/AF/ADC/idle states, no overlaps, conflict rejection, Pixhawk header regression'
