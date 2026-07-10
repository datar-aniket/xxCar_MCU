# Vendored PX4 flashing tools

These files are copied verbatim from
[PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) (`Tools/`) and are licensed
under the **BSD-3-Clause** license (see the header in each file).

| File | Purpose |
|------|---------|
| `px_mkfw.py` | Wraps a raw `nuttx.bin` into a `.px4` image using `boards/fmuv6c/firmware.prototype` (board_id 56). |
| `px4_uploader.py` | Uploads a `.px4` through the PX4 serial bootloader (keeps the bootloader intact). Needs `pyserial`. |

They are vendored (not fetched at build time) so the toolchain is self-contained and pinned.
Refresh them from upstream only when intentionally updating.
