# Bring-up guide (Stage 1)

Barebone NuttX on the Holybro Pixhawk 6C (FMUv6C): **NSH shell on TELEM1**, plus a
**USB CDC-ACM raw data port** (`/dev/ttyACM0`) for the future ROS 2 / DDS link.

## Status — ✅ Stage 1 achieved on hardware
- Builds/links at `0x08020000` (above the intact PX4 bootloader), packaged to `build/xxcar.px4` (board_id 56).
- Flashes through the factory bootloader with `px4_uploader.py`; bootloader kept intact.
- Boots to an interactive **`nsh>` on TELEM1 (UART7) @115200**.
- **USB enumerates** as `/dev/ttyACM0` (VID `0x3162`/PID `0x0053` → `/dev/pixhawk_6c`), auto-connected at
  boot by `stm32_bringup.c`. It is a raw serial data port (no shell), reserved for the Jetson link.

### Known gotchas resolved during bring-up (keep these)
- **USB clock:** sourced from **HSI48**, not PLL3 — NuttX waits for PLL3 lock in an unbounded loop, so a
  PLL3-clocked USB could hang boot. See `board.h`.
- **Interrupt stack:** `CONFIG_ARCH_INTERRUPTSTACK=2048` + `IDLETHREAD_STACKSIZE=3072`. Without a dedicated
  interrupt stack, the OTG IRQ at boot overflowed the tiny idle stack → hardfault.
- **Fault visibility:** `DEBUG_ASSERTIONS` + `DEBUG_HARDFAULT_ALERT` print `file:line` + fault registers on TELEM1.

### Still placeholder (TODO(hw), for later stages)
- Peripheral **pin-mux** in `board.h` is still nucleo-h743zi placeholder except the pins we actually use
  (UART7/TELEM1 PE7/PE8, USB OTG_FS PA11/PA12). I2C/SPI/CAN/sensor pins come from the FMUv6C schematic in Stage 2.

## 1. Host prerequisites

| Tool | Notes |
|------|-------|
| `arm-none-eabi-gcc` | 10.3+ works; **12.x/13.x recommended** for NuttX 12.13. |
| `kconfig-frontends` | **Required** to configure NuttX (`kconfig-tweak`/`kconfig-conf`). `sudo apt install kconfig-frontends`. |
| `make`, `git`, `python3` | standard. |
| `pyserial` | for the uploader: `pip3 install pyserial`. |
| USB-TTL adapter (3.3 V) | for the TELEM1 console (FTDI/CP2102/CH340). |

Optional (debug only): `openocd` + `gdb-multiarch` on the SWD DEBUG port.

## 2. Get the sources
```bash
git submodule update --init --recursive   # nuttx + nuttx-apps @ nuttx-12.13.0
```

## 3. Build
```bash
tools/build.sh          # configure + build + package build/xxcar.px4  (RECONFIGURE=1 to re-run configure)
```
Manual equivalent (from `deps/nuttx`, note `-l` is the Linux-host flag and the board path is positional):
```bash
./tools/configure.sh -l -a ../nuttx-apps ../../boards/fmuv6c/configs/nsh
make -j"$(nproc)"
```

## 4. Flash (through the existing bootloader — kept intact)
```bash
tools/flash.sh          # unplug/replug the Pixhawk USB when prompted, then power-cycle
```
The app is linked at `0x08020000`, so only the app region is written; the factory bootloader is untouched.
A good upload prints `Found board 56,0 … Uploaded in Ns`.

## 5. Wire the TELEM1 console
TELEM1 (6-pin JST-GH) ↔ USB-TTL adapter. **VCC left unconnected.**

| TELEM1 pin | Signal | Adapter |
|---|---|---|
| 2 | UART7_TX | RX |
| 3 | UART7_RX | TX |
| 6 | GND | GND |

> A swapped TX/RX pair is silent — the #1 cause of "no output". If unsure, swap pins 2/3.

## 6. Verify
```bash
picocom -b 115200 /dev/ttyUSB0        # TELEM1 console → nsh>
```
1. `nsh> ?` lists builtins; `uname -a`, `free`, `ps`, `ls /dev` show `console`, `ttyS*`, `ttyACM0`.
2. Host: `lsusb | grep 3162` and `ls /dev/ttyACM* /dev/pixhawk_6c` → the USB data port is present.
3. Data-port loopback: `nsh> echo hello > /dev/ttyACM0`, and on the host read `/dev/ttyACM0` → "hello".
4. Power-cycle without an app present: the bootloader port still enumerates (bootloader intact).

## udev (dev host)
```
# running app  → /dev/pixhawk_6c  (VID 3162 / PID 0053)
ACTION=="add", SUBSYSTEM=="tty", KERNEL=="ttyACM*", ENV{ID_VENDOR_ID}=="3162", ENV{ID_MODEL_ID}=="0053", ENV{ID_USB_INTERFACE_NUM}=="00", SYMLINK+="pixhawk_6c", MODE="0666", GROUP="dialout"
# bootloader   → also aliased to /dev/pixhawk_6c  (VID 3185 / PID 0038)
ACTION=="add", SUBSYSTEM=="tty", KERNEL=="ttyACM*", ENV{ID_VENDOR_ID}=="3185", ENV{ID_MODEL_ID}=="0038", ENV{ID_USB_INTERFACE_NUM}=="00", SYMLINK+="pixhawk_6c", MODE="0666", GROUP="dialout"
```
