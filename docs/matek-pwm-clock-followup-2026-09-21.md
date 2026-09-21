# Matek PWM and clock follow-up — 2026-09-21

## Implemented

- Direct NuttX board PWM backend supports any S1-S8 mask and all eight at once.
  VESC selects its two steering channels through existing `STEER_IO_CH` and
  `STEER_REAR_CH`; other pads stay GPIO-low, not unsolicited servo pulses.
- S1/S2: TIM8 CH2N/CH3N, N-only, PWM1, noninverted, MOE explicit, no deadtime.
  S3-S6: TIM2 CH1-4. S7/S8: TIM4 CH1-2. Independent banks, aligned edges within
  each bank. No TIM3/5/6, PPS, DMA, or PWM interrupt ownership changes.
- 25–400 Hz common requested rate, 1 us counter resolution, rounded fixed
  period (no dithering). Preloaded CCRs + UDIS prevent torn same-bank updates.
  Running updates never reset CNT or generate UG. Startup parks outputs low
  before connecting AF pins. Unrefreshed commands revert to configured neutral
  after 200 ms plus at most one PWM frame and scheduler tick quantization.
  Explicit stop/fault rollback disconnects outputs; it can truncate a pulse.
- Invalid steering configuration no longer terminates the Matek CAN backend.
  Arming still requires healthy steering, CAN, telemetry and a safe setpoint.
  Router status now preserves the backend arm-refusal errno.
- `diag pwm` (also included in `diag status`) passively reports selected pins,
  register health, requested pulse widths, period, timeout count and last error.
  It does not move servos, and is not a physical pulse measurement.

The advanced-timer N-only polarity follows ST's [AN4013 output-control table](https://www.st.com/resource/en/application_note/dm00042534-timers-and-pwm-generation-using-stm32-microcontrollers-stmicroelectronics.pdf):
with main output disabled and only N enabled, N follows OCREF, not its inverse.
No ArduPilot/ChibiOS driver is represented as having been copied into NuttX.

## Live pre-update observations

Board: MatekH743-SLIM-V4, NuttX 12.13.0, build Sep 21 2026 15:32.
USB companion `/dev/v4w_io`, console `/dev/v4w_consol`. Motors disconnected for
subsequent maintenance; no arming or actuator command was sent by the probe.

- USB measurement reproduced increasing sample age: approximately 2.41 ms/s
  during three 15 s windows. Mean ages rose 1157 → 1195 → 1232 ms.
- Raw board-minus-host exchange offsets changed approximately -2610 to -2630
  us/s. Original short capture was -2465 ppm. These are relative measurements,
  not proof of which physical oscillator is inaccurate.
- Console: fitted rate 0 ppm, applied phase slew +200 ppm, repeated rate/step
  resets. The existing +/-1000 ppm bound rejects this sustained rate and
  repeatedly discards the regression: sync cannot catch up.
- EKF had no IMU ring overflow, stale, backwards, gap or publication faults.
  TIM5 minus NuttX MONOTONIC remained within the tick quantization.
- Live RCC: PLLCKSELR=0x00202012 (HSE; M1=1), PLL1DIVR=0x07030277
  (N=120, P=2), PLL1FRACR=0, D1CFGR=0x48, D2CFGR=0x440.
  TIM5 PSC=239, ARR=0xffffffff, CR1=1. These match the nominal 8 MHz HSE /
  480 MHz core / 240 MHz timer / 1 MHz counter configuration. Host NTP status
  reported approximately +9 ppm adjustment, not the measured 2600 ppm.
  Physical HSE/host reference qualification remains outstanding; do not
  claim UTC compensation calibrates PWM widths, CAN timing or sensor dt.

The nominal 8 MHz reference is confirmed by the actual
[ArduPilot MatekH743 hwdef](https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/hwdef/MatekH743/hwdef.dat)
(`OSCILLATOR_HZ 8000000`), not inferred from a different Pixhawk board.

## Clock changes

- Bounded relative-rate range +/-5000 ppm now covers the reproduced rate.
  Console prominently warns above +/-1000 ppm. Phase slew remains +/-200 ppm;
  raw TIM5 and NuttX clocks are never retuned or stepped.
- Added TIMESYNC_END2 (ID 10, 24 bytes): legacy 16-byte END result followed by
  uint64 board midpoint of the selected exchange. Rate regression uses this
  measurement epoch; continuity is enforced at actual END application time.
  Legacy ID 6 remains accepted. Use the updated GUI/probe with new firmware;
  `companion_probe.py --legacy` targets old firmware.
- At least three returned samples and RTT <=20 ms are required. END sample
  cannot be in the future or over 10 s old. Reply TX timestamp is captured
  after acquiring the frame lock. Host request stamping likewise occurs
  after its TX lock. GUI matches replies to the current burst.
- `companion status` reports sample age at enqueue entirely in TIM5 units,
  separating estimator/MCU delay from host UTC/transport effects.
- Headless probe excludes and counts pre-UTC frames, takes the second sync
  after 5 s, then follows a configurable periodic schedule. Example:
  `python3 tools/companion_probe.py --port /dev/v4w_io --duration 120`.
  First-sync phase is provisional until rate is measured and slewed in.

## Automated verification

Production-driver register model: all 255 nonempty channel masks, GPIO failure
rollback, independent timer phase, TIM8 N-only polarity, rollover before/
between/after CCR writes for all banks, watchdog neutral, invalid requests,
rate/mask reconfiguration rejection, randomized updates at 25/50/333/400 Hz.
ASan/UBSan runs and a deliberately missing-UDIS negative control accompany it.
These tests do not establish electrical jitter, servo power integrity or
scope-qualified startup waveforms.

Clock tests include both signs of 2600 ppm, delayed END application, convergence,
inverse conversion, no-jump updates and large host epoch discontinuities.
C/Python wire-layout comparison covers END2. PPS remains deferred.

## Post-upload bench result

Uploaded through the matching board-1013 bootloader with verification; no
force-board option. Build Sep 21 2026 16:18:42. Saved S3/S4 assignments were
not changed. Router remained disarmed; no actuator-control packet or arm
command was issued. `diag pwm`: mask 0x0c, healthy, 20000 us period, no watchdog
expiry. Physical S1/S2/S5-S8 waveforms are not bench-qualified by this check.

120 s USB probe, second sync at 5 s, then every 30 s:

| Window after sync | Mean sample arrival age | Age slope |
| --- | ---: | ---: |
| First (phase only) | 13.97 ms | +2.475 ms/s |
| ~6.6–36.6 s | 18.52 ms | -0.207 ms/s |
| ~37–67 s | 12.17 ms | -0.196 ms/s |
| ~67–97 s | 7.93 ms | -0.081 ms/s |
| ~98–120 s | 6.56 ms | -0.004 ms/s |

Final window: 444 packets, arrival age 3.93–11.02 ms. This is link/sample
latency, **not PWM jitter**. At the third sync, console showed base +2637.853
ppm, applied +2837.853 ppm, no rate/step resets, sample enqueue age 5.05 ms
(max 10.00 ms). The residual phase was being smoothly repaid at 200 ppm.
First initialization is not instantaneous frequency lock.

Both Matek and Pixhawk 6C firmware builds passed. Host PWM/pin/clock/protocol/
router/GUI regressions passed, including the deliberate rollover-race negative
control. Existing unrelated MAVLink `packet_rx_drop_count` uninitialized warning
remains. CAN electrical/ACK tests were not repeated with the ESC unpowered.

### Independent reference check

80 read-only NSH snapshots over 15.14 s compared TIM5 CNT with USB's received
SOF frame counter (unwrapped 11-bit full-speed frame number):

- TIM5: approximately **997351 ticks / host second** (nominal 1000000).
- USB SOF: approximately **999.996 frames / host second** (nominal 1000).
- TIM5: approximately **997.355 ticks / USB frame** (nominal 1000).

This independently localizes the approximately **-2645 ppm** error to the
board timer relative to USB SOF, rather than the GUI, estimator backlog or
host monotonic clock alone. NSH register reads are sequential, not a precision
counter capture. The cause within the physical oscillator/clock path is not
established. A nominal 20 ms PWM frame would be about 20.053 ms at this rate;
scope qualification should check absolute period as well as edge jitter.
Do not remove the warning or assume larger UTC compensation fixed raw timing.

Reproduce with `python3 tools/matek_clock_probe.py --seconds 20` after closing
picocom. It issues only read-only NSH `xd` commands to known H743 registers.
The committed probe was also exercised on the board for 10 s: 53 observations,
approximately 999.990 USB frames/s and 997.352 TIM5 ticks/frame (-2648 ppm).

Final status: five accepted clock updates, zero rate/step resets; base rate
+2638.226 ppm, applied +2633.913 ppm, last sync phase residual -0.131 ms.
Router disarmed, zero arm attempts since upload, PWM healthy, no PWM watchdog
expiry. USB TX error totals also include periods when no host reader is open;
they are not a count of CRC-corrupt packets during the measurement window.
