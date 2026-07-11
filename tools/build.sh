#!/usr/bin/env bash
# Configure + build barebone NuttX for FMUv6C, then package a flashable .px4 image.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NUTTX="$REPO/deps/nuttx"
BOARD_CFG="../../boards/fmuv6c/configs/nsh"   # relative to $NUTTX
APPS_DIR="../nuttx-apps"                        # nuttx-apps submodule, relative to $NUTTX
OUT="$REPO/build"
PROTO="$REPO/boards/fmuv6c/firmware.prototype"

[ -d "$NUTTX/arch" ] || { echo "error: submodules missing. run: git submodule update --init --recursive"; exit 1; }
# configure.sh -l needs kconfig-tweak (from kconfig-frontends). kconfiglib alone is
# NOT sufficient because sethost.sh calls kconfig-tweak.
command -v kconfig-tweak >/dev/null 2>&1 || \
  { echo "error: kconfig-tweak not found. install kconfig-frontends (apt install kconfig-frontends)"; exit 1; }
command -v arm-none-eabi-gcc >/dev/null 2>&1 || \
  { echo "error: arm-none-eabi-gcc not found (install the arm-gnu-toolchain, 12.x+ recommended)"; exit 1; }

echo ">> configuring NuttX (out-of-tree board: boards/fmuv6c:nsh)"
cd "$NUTTX"
if [ "${RECONFIGURE:-0}" = "1" ] || [ ! -f .config ]; then
  make distclean >/dev/null 2>&1 || true
  ./tools/configure.sh -l -a "$APPS_DIR" "$BOARD_CFG"
fi

echo ">> building"
make -j"$(nproc)"

echo ">> packaging .px4 (board_id 56)"
mkdir -p "$OUT"
python3 "$REPO/tools/px4/px_mkfw.py" --prototype "$PROTO" --image "$NUTTX/nuttx.bin" > "$OUT/xxcar.px4"
cp -f "$NUTTX/nuttx.bin" "$NUTTX/nuttx" "$OUT/" 2>/dev/null || true
echo ">> done: $OUT/xxcar.px4"
