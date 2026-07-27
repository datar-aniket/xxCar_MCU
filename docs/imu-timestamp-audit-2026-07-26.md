# IMU Synchronization and Timestamp Root-Cause Audit

Date: 2026-07-26
Target: Pixhawk 6C / STM32H743 / NuttX
Sensors: ICM-42688-P and BMI055 accelerometer/gyroscope dies

## Summary

The 1 MHz TIM5 resolution is not the source of the observed millisecond-scale
errors. The latest 61.355-second hardware log proves that the BMI frame-8 and
coalesced-edge fixes worked: all four streams are finite and monotonic, the
ULog has no dropout records, and the former 2-4 ms BMI gyro gaps are gone.

The log also exposes one remaining deterministic ICM failure. ICM interval
discontinuities still occur every 44 samples, or approximately 22 ms. That is
the driver's watchdog schedule, not its configured eight-sample FIFO
watermark. Its period estimator therefore never receives usable PE6 edges and
remains at the nominal 500 us seed.

The source/lifecycle audit found why setting `FIFO_WM_GT_TH` alone was
insufficient. Board probe enabled ICM FIFO packet generation and routed the
latched, active-low threshold interrupt before any application subscribed and
before PE6 EXTI was attached. The FIFO could reach its threshold and hold INT1
low. Attaching a falling-edge callback later cannot observe an edge that
already occurred.

The current correction:

- leaves ICM FIFO packet generation and INT1 routing disabled while idle;
- disables the route and FIFO contents before every restart;
- flushes the FIFO, then arms PE6 EXTI first;
- enables FIFO contents and INT1 routing only after PE6 is ready;
- checks the EXTI setup result and verifies critical ICM registers by reading
  them back;
- retains the validated BMI frame-8 association, first-edge preservation,
  one-second period observations, and four-entry moving average.

Hardware input capture is not required to solve the observed millisecond-scale
gaps. It could later reduce a few microseconds of ISR-entry jitter, but it does
not correct a one-frame association error, a watchdog-driven FIFO, or a FIFO
reset cascade.

## Evidence from the latest log

File: `/home/aniket/Documents/log/log_001_00.ulg`
SHA-256: `47244732b16a13ac822128ed46f9f1b13572787b91d66086e1fff5dac7a839db`
Duration: 61.355 seconds
ULog parser resynchronizations: 0
ULog dropout records: 0

| Stream | Records | Effective rate | Median interval | Maximum interval |
| --- | ---: | ---: | ---: | ---: |
| ICM accel 0 | 122549 | 1997.06 Hz | 500 us | 1024 us |
| ICM gyro 0 | 122549 | 1997.06 Hz | 500 us | 1024 us |
| BMI accel 1 | 125544 | 2045.27 Hz | 489 us | 1466 us |
| BMI gyro 1 | 122483 | 1995.37 Hz | 501 us | 886 us |

### ICM batch signature

The most common ICM run still contains 43 consecutive 500 us intervals,
therefore 44 samples per drain. There are 1129 such runs. Forty-four samples
at 2 kHz span about 22 ms, matching the 20 ms watchdog plus worker/SPI latency.
A functioning eight-frame watermark must instead produce batches close to
eight frames.

The exact 500 us interval is therefore the nominal fallback period. It is not
evidence that the unsynchronized ICM oscillator happens to equal the MCU clock.

### BMI accelerometer

The BMI accelerometer moves from 500.0 us to 497.3, 494.3, and 488.8 us during
the first four one-second observations, then remains near 488.7 us. This
demonstrates that:

- TIM5 is resolving non-integer sensor periods correctly;
- the one-second period estimator works when it receives the correct edge;
- the 1/32 us fixed-point representation is adequate.

### BMI gyroscope

The BMI gyro now has no interval above 1 ms and no repeated XYZ sample. Its
period leaves the 500 us seed and settles near 501.16 us within the first four
seconds, consistent with an independent oscillator at approximately
1995.4 Hz. This closes the earlier frame-9/flush-cascade defect.

There is still a bounded batch-boundary pattern near 454/549 us. It causes no
non-monotonic timestamp or multi-millisecond gap. It will be addressed only
after the ICM watermark fix is confirmed, so one hardware variable is changed
per test.

### Values, temperatures, axes, and phase

- No stream contains NaN or infinity.
- ICM accel/gyro timestamps match record-for-record, as expected for a shared
  FIFO packet, while their measurements are distinct.
- BMI accelerometer exact repeats are 6.46%, with a maximum run of six. This
  is normal stationary quantization for its 12-bit output; BMI gyro has no
  exact repeats.
- ICM die temperature is 36.47-37.62 C, BMI accel is 37.5-38.0 C, and BMI
  gyro is 40.0 C. These are on-die temperatures, not ambient temperature.
- Motion correlation gives the raw-axis mapping
  `ICM x = -BMI y`, `ICM y = BMI x`, `ICM z = BMI z`.
- Gyro correlations are 0.9994-0.9998 and accel correlations are
  0.9925-0.9998.
- ICM timestamps currently trail the matching BMI waveform by approximately
  1.2-1.5 ms. Because ICM is watchdog-anchored in this log, this is not a
  valid measurement of the final sensor-to-sensor synchronization.

## Authoritative sensor behavior

### ICM-42688-P

The TDK datasheet states:

- `FIFO_WM_GT_TH=1` requests a FIFO watermark interrupt on every ODR write
  while `FIFO_COUNT >= FIFO_WM_TH`;
- the watermark is specified in bytes when `FIFO_COUNT_REC=0`;
- INT1 is configured as latched, push-pull, active-low;
- `FIFO_THS_INT_CLEAR=10b` clears the latched threshold interrupt on a FIFO
  data-byte read;
- `INT_ASYNC_RESET` must be cleared for correct interrupt-pin operation.

PX4 uses the same combination:

- `FIFO_WM_GT_TH` set;
- latched active-low INT1;
- clear on FIFO read;
- falling-edge GPIO callback;
- FIFO count checked after the callback;
- one additional sample tolerated as worker latency.

With a latched external pin, repeated internal threshold requests do not imply
that the GPIO produces an unrelated timestamp for every ODR. The pin remains
asserted until the FIFO read clears it. The first falling edge remains the
threshold crossing that must be preserved.

### BMI055

The Bosch BMI055 datasheet states that the FIFO watermark interrupt is asserted
when the FIFO fill level has reached the frame number programmed in:

- accelerometer register `FIFO_CONFIG_0` at `ACC 0x30`;
- gyroscope register `FIFO_CONFIG_0` at `GYR 0x3D`.

The production PX4 BMI055 drivers program the requested number of samples
directly for both dies. If the worker observes `configured_samples + 1`, PX4
leaves one frame for the next transfer because the captured edge still
corresponds to the configured sample count.

There is no basis for adding one only to the BMI gyro watermark/sample index.

## Correct timestamp model

For a watermark of `N` frames, measured period `P`, captured threshold edge
`E`, drain-time snapshot `C`, and FIFO count `M`:

```text
threshold frame absolute index = samples_already_consumed + N - 1
edge-derived first timestamp   = E - N * P
causal first-timestamp limit   = C - M * P
timestamp of frame i           = first + i * P
```

Subtracting `N * P`, rather than `(N - 1) * P`, deliberately places the newest
threshold frame one complete period before the captured/current time. This is
the conservative causality policy requested for the project.

The period estimate is independent of reconstructed output timestamps:

```text
observed period =
    (current captured edge - previous captured edge)
    / (current absolute edge sample - previous absolute edge sample)
```

It is updated only after at least one second of TIM5 time. Four accepted
observations replace the four nominal history entries, so normal acquisition
completes in approximately 4-5 seconds and remains inside the requested
10-second limit.

## MCU timing path

Current path:

```text
sensor FIFO reaches watermark
    -> active-low DRDY/INT edge
    -> STM32 EXTI pending bit
    -> NuttX EXTI dispatcher
    -> sensor ISR reads shared TIM5
    -> high-priority sensor worker drains FIFO
```

TIM5 is:

- 32-bit;
- free-running at 1 MHz;
- shared by all three sensor ISRs;
- not dependent on the 1 ms NuttX scheduler tick;
- extended to 64-bit monotonic time using the coarse clock only to select the
  correct 71.6-minute wrap.

PE4 has a dedicated EXTI4 vector. PE5 and PE6 share EXTI9_5; NuttX services PE5
before PE6 if both are pending in the same entry. This can introduce several
microseconds of relative latency, not the observed 1-4 ms gaps.

### Hardware input capture

The STM32 alternate-function table provides:

- PE5: TIM15_CH1;
- PE6: TIM15_CH2;
- PE4: TIM15_CH1N, which is not a normal capture input.

TIM15 is currently unused, but it is 16-bit and is not the existing TIM5
timebase. A correct capture implementation would need to bridge TIM15 captures
to the TIM5/monotonic epoch and detect capture overruns. It would also leave the
BMI accelerometer on EXTI.

This added complexity does not address the proven root causes. It should only
be considered later if a corrected log shows residual edge jitter large enough
to matter.

## Signal alignment

During both motion sections, the two IMUs agree strongly:

| Signal | Cross-sensor correlation | Approximate ICM lag |
| --- | ---: | ---: |
| Gyroscope axes | 0.9991-0.9995 | 1.25 ms |
| Accelerometer axes | 0.9984-0.9996 | 1.25-1.50 ms |

Raw-axis mapping:

```text
ICM X = -BMI Y
ICM Y =  BMI X
ICM Z =  BMI Z
```

This is a proper 90-degree mounting rotation, not data corruption.

The measured phase offset contains:

- the deliberate one-period timestamp backdating;
- sensor signal-path/filter group delay;
- a small EXTI-entry component.

It must not be interpreted as 1.25 ms of MCU interrupt latency.

## ICM internal FIFO timestamp

The ICM FIFO packet contains a 16-bit timestamp delta because
`TMST_DELTA_EN=1`. PX4 uses this field to derive FIFO `dt`. The local driver
currently uses the more precise long-window TIM5 edge/sample estimate for
published timestamps and validates only the FIFO header.

The internal delta is useful as a future independent diagnostic for:

- missed ODR occurrences;
- malformed FIFO packets;
- disagreement between the sensor oscillator and MCU edge estimator.

It is not an absolute synchronization source because no CLKIN or FSYNC connects
the ICM clock to the MCU/BMI clocks.

## Code corrections

### ICM-42688

- Restored `FIFO_WM_GT_TH`.
- Keep FIFO content generation and INT1 routing off while no subscriber is
  active.
- On start, force INT1 inactive, flush, arm PE6, and only then enable FIFO
  contents and the threshold route.
- Verify critical FIFO/interrupt registers after configuration and stream
  enable.
- Preserve the first unconsumed edge instead of overwriting it.
- Accept one or more coalesced notifications using the preserved first edge.
- Snapshot the 64-bit timestamp and consume its sequence atomically.

### BMI055

- Associate both accel and gyro watermark interrupts with frame 8.
- Remove the incorrect gyro-only frame-9 rule.
- Preserve the first unconsumed edge.
- Accept a coalesced sequence using that first edge.
- Snapshot/consume the timestamp and sequence atomically.

## Verification before hardware

- Firmware compilation: PASS
- `git diff --check`: PASS
- Full host-test/build gate: PASS (10/10 host tests)
- Artifact: `build/xxcar.px4`, 281016 bytes
- Artifact SHA-256:
  `59b0a4c11a1584c6a61a3257c764b649433c64f4657a666ebe353edb7398c7ca`

## Single consolidated hardware test

Only one native-rate test is required for this correction:

1. Flash `build/xxcar.px4`.
2. Log at native rate for 45-60 seconds.
3. Keep the board stationary for the first 12 seconds.
4. Rotate/move it for approximately 15 seconds.
5. Keep it stationary for 8 seconds.
6. Move it again for the remainder.

Pass criteria:

- ULog opens without parser recovery or dropout records.
- ICM accel/gyro counts match.
- ICM batch signature is approximately eight frames, not 44.
- BMI gyro begins leaving the nominal 500 us seed within about two seconds and
  finishes moving-average acquisition within 5-6 seconds.
- All periods remain bounded to 450-550 us.
- No repeated 2-4 ms BMI gyro gaps.
- No continuing FIFO-flush cascade.
- Cross-sensor correlation remains above 0.99 after the board rotation is
  applied.

## Primary references

- [TDK ICM-42688-P datasheet, revision 1.6](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000347-icm-42688-p-v1.6.pdf)
- [PX4 ICM42688P driver](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/imu/invensense/icm42688p)
- [Bosch BMI055 datasheet, revision 1.2](https://datasheet.lcsc.com/lcsc/1811071031_Bosch-Sensortec-BMI055_C189620.pdf)
- [PX4 BMI055 driver](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/imu/bosch/bmi055)
- [ArduPilot Invensense v3 driver](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp)
- [STM32H743 datasheet](https://www.st.com/resource/en/datasheet/stm32h743ag.pdf)
- [STM32H743 reference manual RM0433](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743-753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
