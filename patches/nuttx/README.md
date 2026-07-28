# NuttX patches

`deps/nuttx` is a pinned, **unforked** upstream submodule (`nuttx-12.13.0`). The few
fixes we need *inside* the NuttX tree live here as patches instead of as commits in
the submodule, because a submodule gitlink only records a commit hash: local edits to
the submodule working tree are invisible to git, so a clean clone would silently build
**different firmware** than the board on the bench.

`tools/build.sh` applies every `*.patch` here before configuring. Applying is
idempotent (an already-applied patch reverse-applies cleanly and is skipped), so it is
safe to re-run, and it survives a `git submodule update` reverting the tree.

## Patches

### 0001-cdcacm-do-not-treat-usb-suspend-as-a-disconnect.patch

Without this, opening `/dev/ttyACM0` on the host fails with **errno 107 (ENOTCONN)**.

`cdcacm_suspend()` conflates *bus idle* with *cable unplugged*. A USB SUSPEND only
means the host stopped sending SOFs (bus idle >= 3 ms) - the device is still attached
and still configured. But the handler calls `uart_connected(&priv->serdev, false)`,
which sets `dev->disconnected`, and from then on every `uart_open()` returns
`-ENOTCONN`. Coming back requires a matching RESUME *and* `priv->config` to still be
set (`cdcacm_resume()` checks it); if that resume is missed, the port is wedged for
good.

Nothing is lost by dropping the call: a genuine detach is already handled on its own by
`cdcacm_disconnect()` and `cdcacm_resetconfig()`, both of which call
`uart_connected(false)` themselves.

Only relevant when `CONFIG_SERIAL_REMOVABLE=y`, which we need for the composite
CDC/ACM + MSC device.

Upstreamable as-is.

### 0002-stm32h7-declare-the-serial-rx-tx-dma-kconfig-options.patch

Serial RX/TX DMA on the H7 was implemented but **unreachable**.

`stm32_serial.c` and `stm32_uart.h` already carry the full RX and TX DMA
implementation, keyed off `CONFIG_<port>_RXDMA` / `CONFIG_<port>_TXDMA`, and
`STM32H7_SERIAL_RXDMA_BUFFER_SIZE` already `depends on` those symbols - but nothing
in the H7 Kconfig ever *defined* them, so they could not be selected. `stm32f7` and
`stm32` define exactly these options; this restores them for the H7, one pair per
U[S]ART, with the same `select SERIAL_RXDMA` and DMA-controller dependencies.

RX DMA also turns on the USART IDLE-line interrupt (`up_setup`), which is what hands
a partial burst to the driver as soon as the line goes quiet instead of waiting for
the DMA buffer to half-fill - the behaviour SBUS and CRSF framing needs.

Upstreamable as-is.

### 0003-stm32h7-sdmmc-reject-buffers-the-idma-cannot-use.patch

**This is the fix for the ULog corruption.** Without it, long recordings come back
desynchronised at random sector boundaries and no layer reports an error.

`stm32_dmapreflight()` is the hook where the SDMMC driver tells `mmcsd` "I cannot DMA
to this buffer", so `mmcsd` returns `-EFAULT` and FAT - with `CONFIG_FAT_DIRECT_RETRY`,
which `CONFIG_FAT_DMAMEMORY` selects - repeats the transfer indirectly through its own
aligned sector buffer. Upstream already uses it that way to keep SDMMC1's IDMA out of
SRAM123/SRAM4. Two cases it did not cover are both reachable on this board:

**Unaligned buffers.** The only alignment test upstream sits inside
`#if defined(CONFIG_ARMV7M_DCACHE) && !defined(CONFIG_ARMV7M_DCACHE_WRITETHROUGH)`,
and we run write-through (as every in-tree H7 board does), so it compiled out
entirely. But `SDMMC_IDMABASE0R` requires a word-aligned address regardless of cache
mode: IDMA discards the low two address bits and transfers from the rounded-down
address, **duplicating or losing 1-3 bytes at the sector boundary and reporting no
transfer error**. That is exactly the +/-1-byte phase shift seen in the corrupt logs,
where garbage decoded as valid sensor records two bytes out of frame. The check is now
unconditional.

**DTCM buffers.** DTCM is private to the Cortex-M7 core (MDMA reaches it only through
the CPU's AHBS port); it is not in the address map any other bus master sees, and ST
documents that SDMMC IDMA cannot access TCM
([AN5200](https://www.st.com/resource/en/application_note/an5200-getting-started-with-the-stm32h7-mcu-sdmmc-host-controller-stmicroelectronics.pdf)).
NuttX says the same in `stm32_allocateheap.c`: *"DMA transfers to/from DTCM are
limited."* Yet DTCM stays in the general heap unless `CONFIG_STM32H7_DTCMEXCLUDE` is
set - and **no in-tree STM32H7 board sets it** (0 of 65 defconfigs), because its help
text is about executing ELF modules, not about DMA. So an ordinary `malloc()` can
return a sector-sized, word-aligned buffer at `0x2000xxxx`, FAT can pick it for a
direct multi-sector transfer, and nothing stops it.

Excluding DTCM from the heap would also work, but it costs 128 KB of RAM and leaves
the driver silently wrong for any other caller. Rejecting the buffer is the in-tree
idiom, keeps the memory, and makes the failure explicit.

The DTCM test is written as an **overlap** test (`buffer < DTCM_END &&
buffer + buflen > DTCM_START`) rather than the containment test upstream uses for
SRAM123/SRAM4, which lets a buffer that straddles the boundary through. `DTCM_START`
and `DTCM_END` come from `stm32_dtcm.h`, which `stm32_sdmmc.c` already includes.

Upstreamable as-is; the alignment half is a genuine upstream bug for any
write-through-cache H7 board with an SD card.
