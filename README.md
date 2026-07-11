# xxCar_MCU

Low-level firmware stack for the **xxCar** wheeled robot, running on a **Holybro Pixhawk 6C**
(FMUv6C: STM32H743, 480 MHz Cortex-M7 + PX4IO). The MCU is the deterministic bridge between
low-level hardware (VESC over CAN, steering servo, onboard sensors) and a Jetson Orin running ROS 2.

The RTOS is **Apache NuttX**, built as a lean, self-owned image — we reuse PX4's proven FMUv6C
hardware definition as a reference but do **not** build the PX4 flight stack.

## Project stages
| Stage | Goal | Status |
|-------|------|--------|
| **1** | Barebone NuttX booting to an **NSH shell** (on TELEM1) + **USB data port**; verified on hardware | ✅ done |
| 2 | Onboard sensor drivers (ICM-42688-P, BMI088, MS5611, IST8310) + sampling | planned |
| 3 | DDS / ROS 2 pub-sub bridge | planned |
| 4 | Deterministic control loops (odometry, motor/servo, localization) | planned |
| 5 | Safety tasks (RC override, comms-loss failsafe) | planned |

## Layout
```
deps/nuttx, deps/nuttx-apps   git submodules (pinned to nuttx-12.13.0)
boards/fmuv6c/                out-of-tree NuttX board overlay (our hardware definition)
apps/                         out-of-tree custom app dir (empty stub in Stage 1)
tools/                        build.sh, flash.sh, vendored PX4 px_mkfw/px_uploader
docs/bringup.md               toolchain + build + flash instructions
```

## Quick start
See [docs/bringup.md](docs/bringup.md). In short:
```bash
git submodule update --init --recursive
tools/build.sh          # configure + build NuttX, emit build/xxcar.px4
tools/flash.sh          # flash through the existing PX4 bootloader (replug when prompted)
```
Then open the **TELEM1** port via a 3.3 V USB-TTL adapter at `115200` → `nsh>` prompt
(e.g. `picocom -b 115200 /dev/ttyUSB0`). The Pixhawk **USB** enumerates as `/dev/ttyACM0`
(`/dev/pixhawk_6c`) — a raw CDC data port reserved for the Jetson link, not a shell.
See [docs/bringup.md](docs/bringup.md).

## Hardware
- **MCU:** STM32H743VIH6 (2 MB flash, 1 MB RAM). App runs above the stock PX4 bootloader at `0x08020000`.
- **Onboard sensors:** IMU ICM-42688-P + BMI088 (SPI), baro MS5611 (SPI), mag IST8310 (I2C).
- **Flashing:** through the factory PX4 bootloader (kept intact) using `.px4` images (board_id 56).
- **Console:** NSH on TELEM1 (UART7, PE7/PE8) @115200 8N1.
- **USB data port:** CDC-ACM, VID `0x3162` / PID `0x0053`, if00 → `/dev/pixhawk_6c` (raw `/dev/ttyACM0`, no shell).
