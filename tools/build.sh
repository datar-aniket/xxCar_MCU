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

# We build against a pinned, *unforked* upstream NuttX, so the handful of fixes
# we need in the NuttX tree itself live here as patches rather than as commits in
# the submodule. Anything else would mean a clean clone silently builds different
# firmware than the one on the bench. Applying is idempotent: a patch that is
# already in the tree reverse-applies cleanly, so we skip it.
if compgen -G "$REPO/patches/nuttx/*.patch" >/dev/null; then
  echo ">> applying NuttX patches (patches/nuttx/)"
  for p in "$REPO"/patches/nuttx/*.patch; do
    name="$(basename "$p")"
    if git -C "$NUTTX" apply --reverse --check "$p" >/dev/null 2>&1; then
      echo "   [already applied] $name"
    elif git -C "$NUTTX" apply "$p" >/dev/null 2>&1; then
      echo "   [applied]         $name"
    else
      echo "error: patch does not apply and is not already applied: $name" >&2
      echo "       the pinned NuttX version probably moved. re-roll the patch." >&2
      exit 1
    fi
  done
fi

# Our out-of-tree apps (apps/) are pulled into the NuttX apps tree as a
# top-level directory. nuttx-apps builds any top-level dir that has a Make.defs
# (BUILDIRS := $(dir $(wildcard $(APPDIR)/*/Make.defs))), so a symlink is all
# that is needed. It lives inside the submodule, so recreate it every build.
echo ">> linking out-of-tree apps (apps/ -> nuttx-apps/xxcar)"
ln -sfn ../../apps "$REPO/deps/nuttx-apps/xxcar"

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
