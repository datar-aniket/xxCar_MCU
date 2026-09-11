# xxCar_MCU

Low-level firmware stack for the **xxCar** wheeled robot, supporting the **Holybro Pixhawk 6C**
and **MATEKSYS H743-SLIM-V4**. The MCU is the deterministic bridge between low-level hardware
(VESC over CAN, steering servo, onboard sensors) and a Jetson Orin running ROS 2.

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
boards/fmuv6c/                common STM32H743 board layer and board variants
boards/matekh743/             Matek PX4 firmware-package identity
apps/                         out-of-tree custom app dir (empty stub in Stage 1)
tools/                        build.sh, flash.sh, vendored PX4 px_mkfw/px_uploader
docs/bringup.md               toolchain + build + flash instructions
```

## Quick start
See [docs/bringup.md](docs/bringup.md). In short:
```bash
git submodule update --init --recursive
make BOARD=pixhawk6c    # build/pixhawk6c/xxcar_pixhawk6c.px4
make BOARD=matekh743    # build/matekh743/xxcar_matekh743.px4
make flash BOARD=pixhawk6c
make flash BOARD=matekh743
```
See [docs/boards.md](docs/boards.md) before moving a harness between boards; their peripheral
pins and sensor population differ.

## Hardware
- **MCU:** STM32H743VIH6 (2 MB flash, 1 MB RAM), with the application at `0x08020000`.
- **Pixhawk 6C:** ICM-42688-P + Bosch secondary IMU, MS5611, IST8310, PX4IO; package board ID 56.
- **Matek H743-SLIM-V4:** two ICM-42688-P IMUs and DPS368, no onboard compass or PX4IO;
  package board ID 1013.
- **Console:** NSH defaults to UART7 at 115200 and is also available over USB.
