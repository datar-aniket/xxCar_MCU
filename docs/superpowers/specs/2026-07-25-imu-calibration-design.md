# IMU Calibration Framework — Design

Date: 2026-07-25
Branch: `stage4-calibration`
Status: approved, ready for implementation planning

## Goal

Produce IMU calibration good enough that the accelerometers and gyroscopes can
carry localisation and odometry (EKF-class fusion) without injecting avoidable
bias or scale error into the position estimate.

Two IMUs are involved: ICM-42688-P (IMU0, 2 kHz, UI filter at ODR/2) and BMI055
(IMU1, 2 kHz, unfiltered). Both publish raw SI units today and no calibration is
applied anywhere in the system.

## Non-goals

- Magnetometer calibration. Different reference (site-dependent Earth field,
  iron distortion) and it does not serve the odometry goal. Out of stage 4.
- On-board estimation. Fits run on the host (see "Board acquires, host
  estimates").
- A calibrated republishing topic. Calibration produces parameters and a library
  to apply them; whether a daemon republishes calibrated topics is a stage-5 EKF
  decision.

## The central split: characterise vs. calibrate

These have opposite transport requirements and must not share a mechanism.

**Class A — record long, analyse offline.** Allan variance, noise density,
temperature coefficients. Needs hours of untouched static data. Uses the
**existing ULog logger and `sdmsc`** — essentially no new firmware. Host tools
read the `.ulg`.

**Class B — interactive guided procedure.** Multi-position accelerometer, gyro
bias, cross-IMU alignment. Low bandwidth, needs live two-way control. This is
what the `cal` protocol and GUI are for.

## Architecture

```
 TELEM1/DEBUG (FTDI)              USB CDC  /dev/ttyACM0
        │                                  │
   nsh> cal accel  ──────►  apps/cal  ◄────┴────►  GUI (host)
                              │                      │
                       stillness detect         numpy solvers
                       capture + average        visualisation
                       raw stream                    │
                              │                      │
                       imu_cal params  ◄─────────────┘
                              │                (params written back)
                    /fs/microsd/params.txt
```

### Port ownership

The shell lives on TELEM1/DEBUG (already NSH by default). The USB CDC port is
dedicated to the GUI.

- `SER_USB_FUNC` must not be `NSH` (its current default), or NSH sits blocked in
  `read()` on `ttyACM0` and races the GUI for input bytes. Add a `SER_FUNC_CAL`
  enum value so the intent is explicit and survives a params reset.
- `cal` is a plain foreground command run from TELEM1. It opens `/dev/ttyACM0`
  directly. No serial-manager changes, no task killing, no handoff race.
- If the port is already claimed, `cal` must refuse with a message naming
  `SER_USB_FUNC` as the fix — not a bare `-EBUSY`.
- The USB CDC port is `removable`: it dies on cable pull. A yank mid-session
  aborts cleanly. **Parameters are committed once, at the end of a session,
  never incrementally.**

### Board acquires, host estimates

The ellipsoid fit, the 12-parameter least squares, and the Kabsch/SVD solve for
cross-IMU alignment run on the host in numpy. The board detects stillness,
averages a segment, ships the vector, and later accepts and stores results.

Rationale: the GUI is mandatory in this workflow anyway; the fit is where
iteration and visualisation happen; improving the estimator costs no firmware
flash. Writing a 9-DOF ellipsoid fit and an SVD in C on the H7 buys nothing.

Accepted cost: calibration requires the GUI. No on-board fallback in v1.

## Parameter model

Per sensor (`ACC0`, `GYR0`, `ACC1`, `GYR1`): a 3×3 matrix and a 3-vector bias,
stored **pre-inverted** so runtime is a matrix multiply, never a solve:

```
a_true = M · (a_meas − b)          # 9 multiplies, 3 subtracts
```

- 12 floats × 4 sensors = 48 parameters
- IMU1→IMU0 rotation: 3 (rotation vector)
- Per-sensor metadata: calibration date, temperature at calibration, fixture
  mode, fit residual, valid flag

Roughly 70 parameters total. Generated in `param.c` with an X-macro rather than
70 hand-written rows. Each carries bounds that reject garbage on load: bias
±2 m/s², matrix diagonal 1±0.1, off-diagonal ±0.1.

### `imu_cal` library

```c
struct imu_cal_s
{
  float M[9];    /* row-major 3x3, pre-inverted */
  float b[3];
  bool  valid;
};

int  imu_cal_load(enum imu_cal_sensor_e sensor, FAR struct imu_cal_s *out);
void imu_cal_apply(FAR const struct imu_cal_s *cal,
                   FAR const float in[3], FAR float out[3]);
```

Consumed by the EKF in stage 5. **Raw uORB topics stay raw** — this is what
keeps Allan variance and temperature fitting possible indefinitely.

## Protocol

Text commands in, JSON lines out, binary frames for bulk sample streaming.

Framing is disambiguated by first byte:

- `{` → JSON line, terminated by `\n`
- `0x02` → binary frame: `STX | u16 len | payload | crc16`

The control path stays drivable by hand from a plain terminal, which matters
when the GUI misbehaves. The bulk path avoids `snprintf("%f")` at kHz rates,
which is a real CPU cost on the H7 (~130 bytes/sample as JSON vs 24 binary).

| Direction | Messages |
|---|---|
| GUI → board | `hello`, `stream <topic> <rate>`, `stop-stream`, `capture`, `get <param>`, `set <param> <value>`, `commit`, `abort`, `quit` |
| board → GUI | `{"evt":"hello","proto":1,...}`, `{"evt":"still","rms":0.004}`, `{"evt":"moving"}`, `{"evt":"captured","n":7,"a":[...],"t":41.2}`, `{"evt":"error","msg":...}`, `{"evt":"committed"}`, `{"evt":"bye"}`, binary sample frames |

`CDCACM_TXBUFSIZE` should rise from 512 to 4–8 KB for the streaming path.

## Procedures

### Accelerometer — desk mode (default)

The 6C case is *almost* flat, but connectors, draft angle and a tippy 12 mm edge
mean 1–3° of unknown orientation error. Commanded orientations therefore cannot
be trusted, so `a_true` is unknown and the only sound constraint is the norm:

```
‖M⁻¹(a_meas − b)‖ = g
```

This is an ellipsoid fit recovering **9 parameters** — bias, scale,
non-orthogonality. It does **not** recover the 3 rotation DOF, because any
rotation R still satisfies ‖R·x‖ = ‖x‖. Absolute alignment is not in the data.
This is why PX4 and ArduPilot keep "level horizon" separate from accel
calibration, and this design does the same.

Because orientations need not be known, scripted positions are the wrong UX.
**Free tumble**: the operator holds the board still in any new orientation; the
board captures automatically on detecting stillness. The GUI shows a coverage
sphere and flags thin regions. More orientations, better conditioning, no
fixture pretence.

### Accelerometer — jig mode

With a machined cube/jig, commanded orientations *are* trustworthy, so `a_true`
is known per position. Scripted positions, full 12-DOF solve including absolute
alignment to the body frame.

### Gyro bias

Static average with temperature recorded alongside.

### Cross-IMU alignment

Falls out of the same tumble data for free, in **both** modes: the two chips
observe identical physical gravity vectors simultaneously, so their *relative*
rotation is observable without knowing absolute orientation. Kabsch/SVD on the
host.

This matters disproportionately for the odometry goal. Residual mount
misalignment of 1–2° is normal, and a 1° cross-axis error injects a persistent
acceleration bias whenever the vehicle tilts — precisely the error an EKF
converts into position drift.

### Vibration check

Runs before any capture, using the binary stream. Refuses to calibrate on a
vibrating bench, since that silently corrupts the result.

### Stillness detection

Gyro magnitude and accelerometer variance both under threshold for N ms →
capture segment and average.

## Class A: Allan variance and noise density

Procedure uses existing tooling: set `LOG_IMU0`/`LOG_IMU1`, set `LOG_RATE`,
`log start` → wait → `log stop` → `sdmsc on` → pull the `.ulg`.

### Bandwidth analysis

ULog record = 3 (message header) + 2 (msg_id) + 24 (payload) = 29 bytes.

| Run | Samples/s | Rate | 4 h size | Queue headroom |
|---|---|---|---|---|
| Both IMUs @ 2 kHz | 8000 | 232 KB/s | 3.34 GB (near FAT32 4 GB cap) | 16 ms |
| One IMU @ 2 kHz | 4000 | 116 KB/s | 1.67 GB | 16 ms |
| **Both IMUs @ 200 Hz** | 800 | 23 KB/s | **334 MB** | **160 ms** |
| One IMU @ 2 kHz, 20 min | 4000 | 116 KB/s | 139 MB | 16 ms |

The bottleneck is **not** SD bandwidth — 232 KB/s is trivial for SDMMC. It is
the uORB queue depth. `ICM_UORB_NBUFFER = 32` and the drivers push per-sample,
so at 2 kHz there is only 16 ms of headroom before samples are silently
overwritten. SD cards routinely stall 50–200 ms for wear-levelling, which would
blow through a 32-sample queue.

For Allan variance a dropped sample is *corrupting*, not merely lossy: gaps bias
the long-τ region, which is the region of interest.

### Resulting procedure

- **Primary run: 200 Hz for 4 hours**, both IMUs. 10× the stall tolerance, a
  tenth the file size, and still reaches the bias-instability knee.
- **Supplementary: 2 kHz for ~20 minutes**, one IMU at a time, to characterise
  the white-noise region where a stall costs little.

### Required firmware changes

1. Raise `ICM_UORB_NBUFFER` and the BMI055 equivalent (32 → 128+) for full-rate
   runs.
2. **Add queue-overflow detection to the logger.** Its `dropped` counter today
   only counts short writes; uORB overflow is silent. `orb_get_state()`'s
   generation counter makes this cheap — if it advances by more than one between
   copies, samples were lost. A silent gap invalidates a 4-hour run.
3. Host Allan tool must verify timestamp continuity before trusting a dataset.

### Preconditions for a valid run

Constant temperature (warm up ≥30 min first), zero vibration, board untouched.
Temperature drift contaminates the long-τ end badly.

### Outputs

Angle/velocity random walk, bias instability, rate random walk — the numbers
that become EKF process-noise tuning.

## What this fixture set can and cannot achieve

No rate table, no thermal chamber.

| Quantity | Reference | Achievable here |
|---|---|---|
| Accel bias + 3×3 scale/misalignment | gravity (exact, free) | full industrial grade |
| Gyro bias | static | excellent |
| Gyro noise (ARW, bias instability) | time only | full grade (Class A) |
| Cross-IMU alignment | shared gravity vectors | excellent, both modes |
| Gyro **scale factor** | known rotation | ~0.2–0.5% via accel-referenced rotation (S4.4) |
| Temperature coefficients | thermal sweep | self-heating only (S4.5), limited |

Gravity is a perfect 1 g reference, so accelerometer calibration is genuinely
industrial-grade on a desk — limited by fixture squareness, not by method.

Gyro scale is the one quantity that really wants a rate table. Without one, the
accelerometer serves as the angle reference: rotate slowly, take gravity
direction before and after, compare the angle to the integrated gyro. Accuracy
is set by the accel calibration rather than by hand steadiness. Rotation *about*
gravity is invisible to the accelerometer, so three board orientations are
needed to cover all three gyro axes.

BMI055's temperature resolution is 0.5 K, which permanently caps IMU1's
temperature model quality.

## Error handling

- Cable pull mid-session → abort, stored parameters untouched.
- Port already claimed → refuse with a message naming `SER_USB_FUNC`.
- Fit residual above threshold → GUI reports and refuses to commit.
- Vibration above threshold → refuse to capture.
- Parameters out of bounds on load → treated as invalid, calibration marked not
  valid rather than applied.

## Testing

- Host unit tests for the protocol encoder/decoder, following the existing
  `tools/test-*.sh` + `tests/*.c` pattern.
- Synthetic-data tests for the solvers: generate samples from a known M and b,
  verify the fit recovers them.
- Stillness detector tested against recorded `.ulg` segments.
- Round-trip test: write parameters, reload, confirm `imu_cal_apply` reproduces
  expected values.
- On-hardware verification at each increment, per project practice.

## Scope and phasing

**S4.1 + S4.2 — this spec's deliverable.** Parameter model, `imu_cal` library,
`cal` protocol, accelerometer multi-position (both modes), gyro bias, cross-IMU
alignment, vibration check. Independently useful: produces real calibration the
EKF can consume.

### Deliverable boundary: firmware and a reference client, not the graphical GUI

Two host-side pieces are distinct and only one is in scope here:

- **In scope — a headless Python reference client** (`tools/cal_client.py`)
  implementing the protocol and carrying the solvers (ellipsoid fit, 12-DOF
  least squares, Kabsch). This is required regardless: it is what proves the
  protocol end-to-end, and it is where the estimation actually lives. A
  graphical front end is a presentation layer over it.
- **Out of scope — the graphical GUI itself** (coverage sphere, live spectrum,
  orientation view). It consumes the same protocol and imports the same solver
  module. Built separately, once the protocol is proven.

This keeps the firmware increments verifiable without waiting on GUI work, and
means the solvers are unit-testable against synthetic data from day one.

Subsequent, each its own spec:

- **S4.3** — Allan variance / noise-density tooling (mostly host-side) plus the
  three firmware changes listed above.
- **S4.4** — gyro scale and misalignment via accel-referenced rotation.
- **S4.5** — temperature compensation via self-heating sweep.
