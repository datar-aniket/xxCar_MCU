#!/usr/bin/env bash
# Configure + build NuttX for one supported board, then package a .px4 image.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NUTTX="$REPO/deps/nuttx"
BOARD_ARG="${1:-${BOARD:-pixhawk6c}}"
case "${BOARD_ARG,,}" in
  pixhawk6c|pixhawk-6c|fmuv6c)
    BOARD="pixhawk6c"
    BOARD_NAME="fmuv6c"
    BOARD_ID=56
    ;;
  matek|matekh743|matekh743-slim-v4|h743-slim-v4)
    BOARD="matekh743"
    BOARD_NAME="matekh743-slim-v4"
    BOARD_ID=1013
    ;;
  *)
    echo "error: unknown board '$BOARD_ARG' (use pixhawk6c or matekh743)" >&2
    exit 2
    ;;
esac

BOARD_CFG="../../boards/fmuv6c/configs/nsh"   # shared H743 configuration
APPS_DIR="../nuttx-apps"                        # nuttx-apps submodule, relative to $NUTTX
OUT="$REPO/build/$BOARD"
PROTO="$REPO/boards/$([ "$BOARD" = matekh743 ] && echo matekh743 || echo fmuv6c)/firmware.prototype"

[ -d "$NUTTX/arch" ] || { echo "error: submodules missing. run: git submodule update --init --recursive"; exit 1; }
# configure.sh -l needs kconfig-tweak (from kconfig-frontends). kconfiglib alone is
# NOT sufficient because sethost.sh calls kconfig-tweak.
command -v kconfig-tweak >/dev/null 2>&1 || \
  { echo "error: kconfig-tweak not found. install kconfig-frontends (apt install kconfig-frontends)"; exit 1; }
command -v arm-none-eabi-gcc >/dev/null 2>&1 || \
  { echo "error: arm-none-eabi-gcc not found (install the arm-gnu-toolchain, 12.x+ recommended)"; exit 1; }

# We build against a pinned, *unforked* upstream NuttX, so the handful of fixes
# we need in the NuttX tree itself live here as patches rather than as commits in
# the submodule. Anything else would mean a clean clone silently builds different
# firmware than the one on the bench. Applying is idempotent: a patch that is
# already in the tree reverse-applies cleanly, so we skip it.
apply_patches() {
  # $1 = submodule path, $2 = patch directory, $3 = human name
  compgen -G "$2/*.patch" >/dev/null || return 0

  echo ">> applying $3 patches ($(basename "$(dirname "$2")")/$(basename "$2")/)"
  for p in "$2"/*.patch; do
    name="$(basename "$p")"
    if git -C "$1" apply --reverse --check "$p" >/dev/null 2>&1; then
      echo "   [already applied] $name"
    elif git -C "$1" apply "$p" >/dev/null 2>&1; then
      echo "   [applied]         $name"
    else
      echo "error: patch does not apply and is not already applied: $name" >&2
      echo "       the pinned $3 version probably moved. re-roll the patch." >&2
      exit 1
    fi
  done
}

apply_patches "$NUTTX"                "$REPO/patches/nuttx"      "NuttX"
apply_patches "$REPO/deps/nuttx-apps" "$REPO/patches/nuttx-apps" "nuttx-apps"

# Our out-of-tree apps (apps/) are pulled into the NuttX apps tree as a
# top-level directory. nuttx-apps builds any top-level dir that has a Make.defs
# (BUILDIRS := $(dir $(wildcard $(APPDIR)/*/Make.defs))), so a symlink is all
# that is needed. It lives inside the submodule, so recreate it every build.
echo ">> linking out-of-tree apps (apps/ -> nuttx-apps/xxcar)"
ln -sfn ../../apps "$REPO/deps/nuttx-apps/xxcar"

echo ">> configuring NuttX for $BOARD_NAME"
cd "$NUTTX"
CURRENT_BOARD=""
if [ -f .config ]; then
  CURRENT_BOARD="$(sed -n 's/^CONFIG_ARCH_BOARD_CUSTOM_NAME="\(.*\)"/\1/p' .config)"
fi

if [ "${RECONFIGURE:-0}" = "1" ] || [ ! -f .config ] || \
   [ "$CURRENT_BOARD" != "$BOARD_NAME" ]; then
  make distclean >/dev/null 2>&1 || true
  ./tools/configure.sh -l -a "$APPS_DIR" "$BOARD_CFG"

  if [ "$BOARD" = "matekh743" ]; then
    kconfig-tweak --enable CONFIG_XXCAR_BOARD_MATEKH743
    kconfig-tweak --set-str CONFIG_ARCH_BOARD_CUSTOM_NAME "$BOARD_NAME"
    kconfig-tweak --disable CONFIG_STM32H7_I2C4
    kconfig-tweak --enable CONFIG_STM32H7_I2C1
    kconfig-tweak --enable CONFIG_STM32H7_SPI4
    kconfig-tweak --enable CONFIG_STM32H7_SPI4_DMA
    kconfig-tweak --set-val CONFIG_STM32H7_SPI4_DMA_BUFFER 512
    kconfig-tweak --disable CONFIG_STM32H7_UART5
    kconfig-tweak --disable CONFIG_UART5_RXDMA
    kconfig-tweak --enable CONFIG_STM32H7_UART4
    kconfig-tweak --enable CONFIG_UART4_RXDMA
    kconfig-tweak --disable CONFIG_XXCAR_PX4IO
    kconfig-tweak --enable CONFIG_STM32H7_SDMMC1
    kconfig-tweak --disable CONFIG_STM32H7_SDMMC2
    kconfig-tweak --set-val CONFIG_CDCACM_VENDORID 0x1209
    kconfig-tweak --set-val CONFIG_CDCACM_PRODUCTID 0x1013
    kconfig-tweak --set-str CONFIG_CDCACM_VENDORSTR "Matek"
    kconfig-tweak --set-str CONFIG_CDCACM_PRODUCTSTR "MatekH743 xxCar"
    kconfig-tweak --set-val CONFIG_COMPOSITE_VENDORID 0x1209
    kconfig-tweak --set-val CONFIG_COMPOSITE_PRODUCTID 0x1013
    kconfig-tweak --set-str CONFIG_COMPOSITE_VENDORSTR "Matek"
    kconfig-tweak --set-str CONFIG_COMPOSITE_PRODUCTSTR "MatekH743 xxCar"
    # MatekH743 keeps parameters in the final two 128 KiB MCU-flash sectors,
    # matching ArduPilot's STORAGE_FLASH_PAGE=14 layout.  Only the low-level
    # progmem API is needed; the parameter layer supplies its own atomic
    # two-sector journal.
    kconfig-tweak --enable CONFIG_STM32H7_PROGMEM
    make olddefconfig
  fi
fi

echo ">> building"
make -j"$(nproc)"

echo ">> packaging .px4 (board_id $BOARD_ID)"
mkdir -p "$OUT"
python3 "$REPO/tools/px4/px_mkfw.py" --prototype "$PROTO" --image "$NUTTX/nuttx.bin" > "$OUT/xxcar_${BOARD}.px4"
cp -f "$NUTTX/nuttx.bin" "$NUTTX/nuttx" "$OUT/" 2>/dev/null || true
echo ">> done: $OUT/xxcar_${BOARD}.px4"
