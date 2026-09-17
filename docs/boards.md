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

## Pixhawk 6C front and rear steering outputs

On Pixhawk 6C, steering defaults to the PX4IO PWM rail instead of the VESC CAN
servo command. Connect the front servo to **MAIN 1** (`STEER_IO_CH`) and rear
servo to **MAIN 2** (`STEER_REAR_CH`). Both channels are configurable from 1–8
and must differ. Use an appropriate servo supply and common ground.
The VESC still receives motor-only duty/current commands; the same RC and Auto
steering path, pulse endpoints, trim, and RC channel 7 live trim are retained.

```text
param set STEER_OUT_SRC 1   # 0 restores VESC CAN steering
param set STEER_IO_CH 1     # choose the connected MAIN output, 1 through 8
param set STEER_REAR_CH 2   # rear steering output
param save                  # Pixhawk 6C needs microSD for persistence
reboot
px4io status
vesc status
```

`PX4IO_EN=1` is required. `PX4IO_RATE` controls how often new steering
setpoints can reach the IO chip (50 Hz by default); `PX4IO_PWM_HZ` controls the
PWM frame rate the servo sees (50 Hz by default for analog servos). Both output
selection and front/rear calibration parameters are read when `vesc` starts,
so reboot after changing
them. Without microSD, the parameters remain RAM-only; use `vesc stop` then
`vesc start` to apply them for this session. Test with the wheels clear of the
ground and verify that positive
steering commands turn left before driving. If PX4IO stops responding, the
motor is disarmed; a stopped VESC daemon leaves the IO output at neutral after
its 200 ms steering-command timeout. Steering feedback remains selected
independently by `STEER_FB_SRC`: 0 uses the VESC ADC (only if its feedback
sensor is still wired there), while 1 reports the requested servo pulse as
an estimate, not a measured angle.

RC input has one deterministic owner. With no FMU port assigned function 4,
PX4IO publishes the receiver connected to the Pixhawk RC IN connector. If any
`SER_*_FUNC` is set to 4, that direct UART/PPM driver is the sole `rc_in`
publisher. PX4IO continues servicing MAIN outputs and `px4io rc` diagnostics,
but it no longer publishes its receiver stream. Reboot after changing a port
function so all services consume the same saved assignment.

## Matek H743-SLIM-V4 wiring

The Matek is not connector- or pin-compatible with the Pixhawk 6C. Rebuild the harness using
the Matek pad labels and the following firmware mapping. UART signals are 3.3 V logic; cross
TX to RX and connect a common ground. Check the Jetson carrier's UART voltage before wiring.

| Function | Matek connection | MCU peripheral | Default firmware use |
|---|---|---|---|
| Jetson companion | T4/R4 | UART4 PB9/PB8 | `TELEM2`, 921600 baud |
| NSH console | T7/R7 | UART7 PE8/PE7 | `TELEM1`, 115200 baud |
| RC input | R6 (T6 unused for receivers) | USART6_RX / TIM3_CH2 PC7 | SBUS/CRSF auto-detect; explicit PPM |
| VESC | CAN H/L/GND | FDCAN1 PD1/PD0 | 1 Mbit/s |
| Steering servo | S1 + servo rail/GND | TIM2_CH1 PA0 | 50 Hz PWM, 900–2100 us |
| Optional PPS | S1 + GND | TIM5_CH1 PA0 | only when S1 steering is disabled |
| External IST8310 | I2C1 SCL/SDA | PB6/PB7 | optional compass |
| microSD | onboard slot | SDMMC1 | logs; optional text parameter mirror |

Matek defaults to `STEER_OUT_SRC=1`, which sends steering PWM to S1 and sends
motor-only CAN commands to VESC. `STEER_PWM_HZ` sets the S1 frame rate (default
50 Hz); the same `VESC_STEER_MIN/TRIM/MAX/OFS` mapping and RC channel 7 trim
used on Pixhawk are applied. A 200 ms hardware watchdog returns S1 to neutral
if command updates stop. S1 steering and S1 PPS are physically mutually
exclusive, so `PPS_EN` defaults to 0 on Matek and boot suppresses PPS if board
PWM steering is selected.

R6 is RC input by default (`SER_RC_FUNC=4`). With `RC_PROT=0`, firmware
alternates between SBUS and CRSF settings until valid frames are decoded. PPM
cannot be autodetected as UART data; set `RC_PROT=3` to remux R6/PC7 to
TIM3_CH2 input capture. PPM accepts 4–18 channels with 750–2250 us intervals
and treats an interval of at least 2700 us as frame sync.

```text
param set STEER_OUT_SRC 1
param set STEER_PWM_HZ 50
param set SER_RC_FUNC 4
param set RC_PROT 0       # SBUS/CRSF auto; use 3 for PPM
param save
reboot
vesc status
rc status
```

There is no PX4IO co-processor on Matek. Power the servo rail with a suitable
BEC and connect signal, supply, and ground correctly; the MCU does not provide
servo power from PA0 itself.

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
