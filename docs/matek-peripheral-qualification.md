# Matek peripheral hardening — first implementation

NuttX is retained. No firmware has been flashed and no electrical/timing claim
has been verified on hardware. Existing S3/S4 PWM behavior is retained; PPS stays
disabled on Matek pending the final migration.

## Changes to verify

| Component | Implemented behavior | Important boundary |
|---|---|---|
| CAN1 | One outstanding cyclic command, superseded-command cancellation, 20 ms TX lifetime checked by a 10 ms NuttX watchdog; completed/cancelled/expired counters; bus-off recovery without shared-controller reset | Cancellation can lose a race with a frame already transmitting. Software watchdog requires functioning interrupts. This TX policy is for the current single cyclic VESC command, not a generic multi-device queued transport. |
| VESC/router | Bus-off disarms; backend disarm invalidates the router arm latch. Stop cancels old demand and gives neutral up to 50 ms to complete; reports unconfirmed delivery | CAN ACK does not prove the intended ESC applied neutral. Configure the ESC's own command timeout. Recovery does not authorize rearming. |
| SBUS/CRSF | Configuration failures stop startup; held input retains last-good timestamp; explicit receiver failsafe acts immediately; invalid traffic cannot extend the 100 ms freshness limit; idle clears partial frames; SBUS resynchronizes inside rejected candidates; post-starvation backlog is discarded | UART bytes still lack arrival timestamps and parity/framing/overrun metadata. This is **not** ArduPilot-equivalent wire-gap framing. SBUS has no payload CRC. |
| PPM | Exact configured channel count; two complete matching frames qualify acquisition/recovery; missing-edge/shortened frames rejected; long gaps cannot alias through the 16-bit counter; timer input filtering enabled | Set the receiver's actual count. PPM has no CRC and arbitrary noise is not completely detectable. |
| RC smoothing | Optional manual-steering-only low-pass, updated once per fresh sample; resets on safety exits | Off by default. Raw RC, throttle, switches, arming, disarming and failsafe are not smoothed. |
| ICM42688, both instances | All existing configuration checks gate registration; runtime identity/scale/timing/FIFO checks and stall detection; device-local reconfiguration at most once per second; stream transitions owned by the sensor worker | Asynchronous activation failures are visible through health/progress. Lower-half SPI waits can still hang; see remaining work. |
| DPS368/I2C2 | Configuration readback, error/progress counters, rate-limited device recovery; optional bus recovery only on its onboard bus; no twentyfold retry amplification after a transfer timeout | Matek I2C timeout is 50 ms, in both millisecond and tick configuration. External I2C1 is not reset by barometer recovery. |
| NuttX I2C reset | Restores pinmux/controller even when SDA/SCL remains stuck; preserves the failure result | Patch `0012`, applied by the normal build. Restoring software state cannot fix a persistent electrical fault. |

## On-board diagnostics

`diag` only snapshots driver counters. It does not open raw buses, consume the
RC/CAN owner's input, subscribe/activate sensors, reset drivers, send CAN frames,
or move outputs. It also **does not stop background applications**.

```text
diag status
diag test imu0 5
diag test imu1 5
diag test baro 5
diag test can 5
diag test rc 5
diag test all 5
sensor_status -T -t 5000
```

`all` runs sequentially (about 25 seconds at the default five seconds each).
Test windows accept 1–60 seconds. Tests return nonzero for faults, insufficient
progress, or a stopped/absent/uninstrumented driver. PASS means observed progress
without observed errors in that window—not rate, calibration, IRQ latency,
electrical integrity, PWM jitter, or long-run reliability certification.

Progress is sampled every 20 ms. Gap limits are 100 ms for IMUs/RC and 500 ms for
barometer/CAN. A CAN test requires incoming traffic and checks that some queued
TX completes if TX occurred. Discovery-only RX does not qualify the transmit
path. Use `sensor_status -T` for the existing detailed sensor timing audit.
RC expiry is a 100 ms age threshold, not a hard 100 ms response-time guarantee:
the driver's 20 ms poll and downstream scheduling add detection/actuation latency.
Pixhawk's primary ICM is instrumented; its Bosch secondary and MS5611 are not yet
covered by the new driver-health API. Use `sensor_status` for them.

Before a PPM test, set `RC_PPM_CH` to the **actual** receiver output count
(default 8; allowed 4–18), save, and reboot/restart RC. A seven-channel receiver
will deliberately fail an eight-channel configuration. Select `RC_PROT=3` for
PPM, `1` for SBUS, or `2` for CRSF; no automatic PPM detection is introduced.

Optional steering smoothing: `RC_ST_FILT_MS=20` gives a 20 ms time constant;
`0` disables it. Restart the router/reboot after changing it. Qualify raw input
first; filtering is not a remedy for corrupted frames.

## Hardware checks

Use a secured bench setup, wheels lifted and actuators made safe. Disarm before
disconnect/fault tests; a passive diagnostics command does not disarm the car.

1. Capture `diag status`, run each connected component's test, then capture status
   again. Check both IMUs individually and record sensor timing under CAN/RC/USB/SD
   load. Verify that configuration checks do not cause recurring resets.
2. RC: remove transmitter signal and receiver cable separately. Explicit SBUS
   failsafe must not wait for ten packets; corrupt traffic must not keep held
   commands fresh. Recovering reception must require an intentional arm cycle.
   For PPM, inject a missing edge and confirm no channel shifting; verify count
   mismatch remains rejected instead of being relearned.
3. CAN: disconnect/reconnect the peer while safely disarmed. Pending demand must
   not build a replay queue. Inspect completed/cancelled/expired/busy counts.
   Use controlled fault injection to test bus-off: missing ACK alone may remain
   error-passive and does not necessarily produce bus-off. Check recovery counts,
   neutral-stop confirmation, and no automatic rearm.
4. I2C: with suitable current-limited test equipment, hold/release the **onboard**
   bus line and verify recovery after release, without RC/CAN/IMU restart.
   Do not short power rails. A separate external-I2C test device must be tested on
   I2C1; the barometer test does not qualify the external bus.
5. Repeat the separate [PWM scope qualification](matek-pwm-qualification.md)
   under the same load. Software progress tests cannot establish pulse jitter.

## Host coverage and source alignment

Validation: Matek and Pixhawk 6C firmware builds succeeded. RC decoder (including
400,000 deterministic noise bytes), PPM, router/smoothing, CAN driver/RAM/ring,
ICM configuration, I2C cleanup, PWM/pins, VESC command/protocol, parameter-range,
and sensor-timing tests passed. ASan/UBSan were run where provided; LeakSanitizer
was disabled because this environment runs under tracing. The full builds still
report the existing `mavlink.c` potentially-uninitialized parser warning.

Tests execute production RC parsing/link policy, router policy, CAN TX/recovery
code with modeled registers, ICM configuration with modeled SPI, and the exact
NuttX I2C-reset function with modeled GPIO/I2C. PPM tests cover counts 4–18 and
every missing-edge position that merges two 1500 us intervals into a false sync.
The I2C negative control fails against the original upstream cleanup. Existing
PWM register-model and pin/conflict tests remain applicable. No host model
reproduces a real CAN bus, sensor silicon, cache/DMA failure, or IRQ scheduling.

The comparison used actual pinned source, not a presumed board layout:

- [ArduPilot CANFDIface](https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/CANFDIface.cpp): deadline cancellation and software bus-off recovery. Adapted as NuttX-owned logic; ChibiOS HAL was not transplanted.
- [ArduPilot SBUS](https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_RCProtocol/AP_RCProtocol_SBUS.cpp): timestamp-based framing and receiver failsafe handling. The timestamp capability remains a gap here.
- [STM32 RM0433](https://www.st.com.cn/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf): FDCAN request cancellation, completion, and bus-off state semantics.

## Still required, not claimed fixed

1. **SPI lower-half:** bounded DMA and polling waits, DMA-stop completion,
   cache-safe abort cleanup, and error propagation. A worker blocked there cannot
   execute the new sensor recovery logic. This is the next low-level priority.
2. **UART lower-half:** RX error counters/metadata and real arrival/idle timing;
   test framing against byte loss and hardware overrun, not only software noise.
3. **PWM electrical qualification:** the subsequent [S1-S8 driver and clock
   follow-up](matek-pwm-clock-followup-2026-09-21.md) completes the baseline PWM
   backend. Scope/load qualification of all three banks remains required.
4. External I2C-device qualification, ADC acquisition/feedback drivers, broader
   Pixhawk driver-health instrumentation, and cold-start sensor re-probe if a
   sensor failed initial registration. Current recovery covers registered devices.
5. PPS pin migration remains deferred. The follow-up above addresses the
   reproduced clock-rate tracking failure; physical reference qualification
   remains outstanding.
