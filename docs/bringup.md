# Bring-up guide (Stage 1)

Barebone NuttX → NSH shell over USB on the Holybro Pixhawk 6C (FMUv6C).

## 1. Host prerequisites

| Tool | Notes |
|------|-------|
| `arm-none-eabi-gcc` | 10.3+ works; **12.x/13.x recommended** for NuttX 12.13. |
| `kconfig-frontends` | **Required** to configure NuttX. `sudo apt install kconfig-frontends` (or build NuttX's `tools/kconfig-frontends`). |
| `make`, `git`, `python3` | standard. |
| `python3` + `pyserial` | for the PX4 uploader: `pip3 install pyserial`. |
| `genromfs` | only if a ROMFS is enabled later; not needed for Stage 1. |

Optional (debug only): `openocd` + `gdb-multiarch` for SWD breakpoints — **never** used to erase flash.

## 2. Get the sources
```bash
git submodule update --init --recursive   # nuttx + nuttx-apps @ nuttx-12.13.0
```

## 3. Build
```bash
tools/build.sh
```
This configures NuttX against the out-of-tree overlay `boards/fmuv6c` (config `nsh`), builds, and
wraps `nuttx.bin` into `build/xxcar.px4` (board_id 56) with `tools/px4/px_mkfw.py`.

Manual equivalent:
```bash
cd deps/nuttx
./tools/configure.sh -l ../../boards/fmuv6c/configs/nsh -a ../../apps
make -j"$(nproc)"
```

## 4. Flash (through the existing bootloader — kept intact)
```bash
tools/flash.sh
```
- The app is linked at `0x08020000`, above the factory PX4 bootloader — the bootloader is **not** erased.
- `flash.sh` polls for the board and prompts you to **unplug/replug**; the bootloader exposes its serial
  port for a few seconds after reset, and `px_uploader.py` catches it, writes only the app region, and reboots.
- Uploader `--port` glob covers `/dev/pixhawk_6c`, `/dev/serial/by-id/*`, `/dev/ttyACM*`.

## 5. Verify
1. On USB connect, `/dev/pixhawk_6c` appears (VID `0x3162` / PID `0x0053`, if00).
2. Open it (any baud, it's CDC): `picocom /dev/pixhawk_6c` → **`nsh>`** prompt.
3. `nsh> ?` lists builtins; `uname -a`, `free`, `ps`, `ls /dev` show `console`, `ttyS*`, `can0`, `i2c*`, `spi*`.
4. `nsh> i2c dev` scans the internal I2C bus → **IST8310** magnetometer answers (sensor-bus init OK).
5. FDCAN loopback on `can0` → TX frame echoes back.
6. Power-cycle without our app: the bootloader port still enumerates (bootloader intact).

## udev (already configured on the dev host)
```
# running app  → /dev/pixhawk_6c
ACTION=="add", SUBSYSTEM=="tty", KERNEL=="ttyACM*", ENV{ID_VENDOR_ID}=="3162", ENV{ID_MODEL_ID}=="0053", ENV{ID_USB_INTERFACE_NUM}=="00", SYMLINK+="pixhawk_6c", MODE="0666", GROUP="dialout"
# bootloader   → also aliased to /dev/pixhawk_6c
ACTION=="add", SUBSYSTEM=="tty", KERNEL=="ttyACM*", ENV{ID_VENDOR_ID}=="3185", ENV{ID_MODEL_ID}=="0038", ENV{ID_USB_INTERFACE_NUM}=="00", SYMLINK+="pixhawk_6c", MODE="0666", GROUP="dialout"
```
