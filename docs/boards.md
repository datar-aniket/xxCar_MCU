# Supported boards

The firmware uses one shared STM32H743 application layer with two compile-time hardware
descriptions. Board selection changes the oscillator/PLL setup, pin mux, buses, sensors,
serial routing, USB identity, and PX4 firmware package ID. It is not just a filename change.

## Build and flash

```sh
make BOARD=pixhawk6c
make BOARD=matekh743

make flash BOARD=pixhawk6c
make flash BOARD=matekh743
```

The direct script equivalents are `tools/build.sh pixhawk6c` and
`tools/build.sh matekh743`. Outputs are kept separately:

- `build/pixhawk6c/xxcar_pixhawk6c.px4` — board ID 56
- `build/matekh743/xxcar_matekh743.px4` — board ID 1013

Do not cross-flash these files. The Matek must have a PX4-protocol-compatible H743-SLIM
bootloader with board ID 1013 and an application start address of `0x08020000`. If the
installed bootloader rejects the `.px4` file, install the matching Matek H743-SLIM
bootloader first; do not force an image with a mismatched board ID.

## Matek H743-SLIM-V4 wiring

The Matek is not connector- or pin-compatible with the Pixhawk 6C. Rebuild the harness using
the Matek pad labels and the following firmware mapping. UART signals are 3.3 V logic; cross
TX to RX and connect a common ground. Check the Jetson carrier's UART voltage before wiring.

| Function | Matek connection | MCU peripheral | Default firmware use |
|---|---|---|---|
| Jetson companion | T4/R4 | UART4 PB9/PB8 | `TELEM2`, 921600 baud |
| NSH console | T7/R7 | UART7 PE8/PE7 | `TELEM1`, 115200 baud |
| RC serial | T6/R6 | USART6 PC6/PC7 | RC auto-detect, 420000 baud |
| VESC | CAN H/L/GND | FDCAN1 PD1/PD0 | 1 Mbit/s |
| PPS input | S1 + GND | TIM5_CH1 PA0 | rising-edge capture |
| External IST8310 | I2C1 SCL/SDA | PB6/PB7 | optional compass |
| microSD | onboard slot | SDMMC1 | logs; optional text parameter mirror |

S1 is reserved for PPS while `PPS_EN=1`; do not also use S1 as a PWM output. The firmware
does not currently expose the Matek S1-S12 PWM outputs because vehicle actuation is sent to
the VESC over CAN. There is no PX4IO co-processor on the Matek, so the Pixhawk RC-IN/PWM
architecture does not carry over; use the dedicated R6/T6 serial RC pads.

Power the board through a regulator/BEC and wiring that meets the Matek power-input
specification. Do not transplant a Pixhawk POWER connector by color or position. For CAN,
avoid tying together independently driven 5 V rails; CAN H, CAN L, and a common ground are
the required signals for an independently powered VESC.

## Board-specific sensor behavior

| Item | Pixhawk 6C | Matek H743-SLIM-V4 |
|---|---|---|
| Crystal | 16 MHz | 8 MHz |
| Primary IMU | ICM-42688-P, SPI1 + DRDY | ICM-42688-P, SPI1 + DRDY |
| Secondary IMU | Bosch BMI055/BMI088 family, SPI1 | ICM-42688-P, SPI4; FIFO polled because DRDY is not routed |
| Barometer | MS5611 | DPS368 on I2C2 address 0x76 |
| Compass | onboard IST8310 | none onboard; optional external IST8310 on I2C1 |
| microSD | SDMMC2 | SDMMC1 |
| IO co-processor | PX4IO | none |
| USB application ID | VID 0x3162 / PID 0x0053 | VID 0x1209 / PID 0x1013 |

Both applications expose two CDC ACM functions inside that one USB composite
device. USB0 has interface ID `00` and defaults to NSH; USB1 has interface ID
`02` and defaults to CAL. Linux's `/dev/ttyACM<N>` numbers are allocation order,
not durable identities, so use `ID_USB_INTERFACE_NUM` in udev rules:

The ready-to-install [`tools/99-v4w.rules`](../tools/99-v4w.rules) assigns:

- interface `00` (USB0/NSH) to `/dev/v4w_consol`
- interface `02` (USB1/CAL or companion) to `/dev/v4w_io`

Install and activate it on the host with:

```sh
sudo install -m 0644 tools/99-v4w.rules /etc/udev/rules.d/99-v4w.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
```

A composite USB device has one VID/PID and serial number. The interface number,
not a second PID, is the standards-compliant per-port identity.

For the Matek, the factory sensor mounting defaults are:

- `SENS_IMU0_ROT=12` (`PITCH_180`)
- `SENS_IMU1_ROT=26` (`PITCH_180_YAW_90`)
- `SENS_BOARD_ROT=0` when the board arrow points forward

Matek parameters are saved in an atomic journal in STM32 internal-flash sectors 14 and 15;
they do not require a microSD card. The firmware image is link- and package-limited below
`0x081c0000`, and the matching ArduPilot bootloader preserves these sectors during normal
PX4-protocol uploads. `param save` optionally mirrors a readable `params.txt` to a mounted SD
card, and a fresh board imports that file once if no internal snapshot exists. A ROM-DFU mass
erase will still erase parameters.

Saved parameters override compiled defaults. On first installation, reset parameters and then
perform fresh accelerometer and gyro calibration. Never reuse the Pixhawk IMU offsets, scales,
or sensor-rotation values. Verify the result before enabling the EKF or controls:

```text
sensor_status
imu_delta status
sensors status
ser status
vesc status
pps status
```

At rest, both accelerometers must report the same gravity direction after rotation and both
`imu_delta` instances must remain near 400 Hz. The second Matek IMU is expected to be drained
in short FIFO batches rather than by a dedicated data-ready interrupt.

Hardware mapping references: [Matek H743-SLIM-V4 product page](https://www.mateksys.com/?portfolio=h743-slim-v4),
[PX4 Matek H743-Slim board configuration](https://github.com/PX4/PX4-Autopilot/tree/main/boards/matek/h743-slim),
and [ArduPilot MatekH743 hardware definition](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef/MatekH743).
