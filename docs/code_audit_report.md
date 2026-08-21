# xxCar MCU Firmware Runtime Re-audit

Date: 2026-07-26  
Audited revision: `88d8687` (`stage4-cal-stream`)  
Previous audit baseline: `3c39ade`  
Target: Pixhawk 6C / STM32H7, custom NuttX firmware

## Executive summary

The updated firmware contains two important committed improvements:

1. Sensor workers now run above application tasks, fixing the priority
   inversion that allowed logging, MAVLink, RC, and PX4IO work to starve the
   hardware FIFO-drain threads.
2. FAT now uses a dedicated DMA-capable buffer pool, and the logger's 64 KiB
   buffer is explicitly aligned in AXI SRAM.

Both changes are valid and materially reduce sensor dropout and SD-write risk.
The firmware and all host tests now build and pass.

The SD change does not completely close the original DTCM/SDMMC defect.
FAT's own buffers and the current logger buffer are safe, but an aligned,
sector-sized caller buffer allocated from the general heap can still reside in
DTCM and be sent directly to SDMMC IDMA. The SDMMC preflight routine does not
reject DTCM, so FAT's new indirect retry is not activated for this case.

An uncommitted logger write-recovery helper and host test are also present.
The test passes, but the helper is not listed in the logger Makefile and is not
called by `logger.c`; it therefore has no effect on the firmware yet.

The firmware is improved but should still not be treated as safe for vehicle
motion or unattended calibration. Parameter persistence, RC timeout
publication, PX4IO transport/state handling, calibration validation, and
general DTCM-to-SDMMC protection remain blockers.

## Current status at a glance

| Area | Status | Summary |
| --- | --- | --- |
| Sensor task priority | Resolved | Sensor workers now run above all application tasks |
| Logger buffer alignment | Resolved | Buffer is explicitly aligned in AXI SRAM |
| FAT internal DMA buffers | Resolved | Dedicated aligned DMA pool is initialized before mounting |
| General DTCM/SDMMC safety | Partial | Direct caller buffers in DTCM can still reach IDMA |
| Logger partial-write recovery | In progress | Helper and test exist but are not integrated |
| Logger uORB loss detection | Open | Queue overwrites are still silent |
| Parameter durability | Open blocker | Live parameter file is still truncated in place |
| RC timeout failsafe | Open blocker | No invalid/failsafe uORB message is published on silence |
| PX4IO safety | Open blocker | Short-write, stale PWM, stop/disarm, and reply-validation issues remain |
| Calibration validation | Open | Setter errors and fit residual remain unchecked |
| Sensor driver recovery | Open | MS5611, IST8310, ICM and BMI compatibility findings remain |
| MAVLink robustness | Open | TX, target validation, parser initialization, and reconnect remain |
| Firmware build | Passing with caveat | Build succeeds, but board symlinks still reference another clone |

## Changes reviewed

### Committed changes

- `3b0762e` — `sd: adopt the stock FAT DMA-memory setup, and stop relying on accidental alignment`
- `88d8687` — `sensors: put sampling above every application task`

### Uncommitted files reviewed

- `apps/logger/log_write.c`
- `apps/logger/log_write.h`
- `tests/log_write_test.c`
- `tools/test-log-write.sh`

The existing uncommitted `tools/allan.py` changes were preserved.

## Resolved findings

### 1. Sensor priority inversion

**Previous condition**

Sensor workers ran at `SCHED_PRIORITY_DEFAULT`, normally priority 100.
Application tasks ran above them:

- PX4IO: 110
- RC: 105
- MAVLink: 104
- logger: 102

The ICM-42688 hardware FIFO holds approximately 102 complete 20-byte packets,
or about 51 ms at 2 kHz. A higher-priority task could prevent the sensor worker
from draining it for longer than this limit.

**Current implementation**

`boards/fmuv6c/src/fmuv6c.h:512-538` defines:

```c
#define FMUV6C_SENSOR_PRIO (SCHED_PRIORITY_DEFAULT + 50)
```

The ICM-42688, BMI055, MS5611, and IST8310 workers now use this priority.

The resulting relevant priority order is:

| Priority | Task |
| ---: | --- |
| 224 | HPWORK, including SDMMC completion work |
| 150 | Sensor workers |
| 110 | PX4IO |
| 105 | RC |
| 104 | MAVLink |
| 102 | Logger |
| 100 | NSH, LPWORK and watchdog automonitor |

**Result**

The application-over-sensor priority inversion is resolved. Logger and protocol
work can no longer starve sensor FIFO draining.

### 2. Logger buffer placement and alignment

`apps/logger/logger.c:220-233` now declares the logger buffer with explicit
32-byte alignment:

```c
static uint8_t g_buf[LOG_BUFSIZE] aligned_data(32);
```

ELF inspection places it at `0x24007540`, which is:

- in AXI SRAM;
- accessible by SDMMC IDMA;
- aligned to both 32 and 64 bytes.

The previous dependency on accidental `.bss` layout is resolved.

### 3. FAT internal DMA-buffer allocation

The board configuration now enables:

- `CONFIG_GRAN`
- `CONFIG_FAT_DMAMEMORY`
- the selected `CONFIG_FAT_DIRECT_RETRY`

`boards/fmuv6c/src/stm32_dma_alloc.c` creates a 4096-byte aligned granule pool
for FAT sector buffers. `stm32_bringup()` initializes the pool before SDMMC
initialization and FAT mounting.

ELF inspection confirms:

- `g_dma_heap` at `0x24002ac0`;
- `stm32_dma_alloc_init()` linked;
- `fat_dma_alloc()` linked;
- `fat_dma_free()` linked.

FAT's volume and per-file sector buffers no longer come from the general heap
and therefore cannot unexpectedly land in DTCM.

## Partially resolved findings

### 4. General SDMMC IDMA access to DTCM

**What is now safe**

- FAT internal sector buffers come from the AXI SRAM DMA pool.
- The logger buffer is a static aligned AXI SRAM buffer.
- Small and partial writes normally pass through the FAT internal sector
  buffer.

**What remains unsafe**

`CONFIG_MM_REGIONS=4` still includes DTCM in the general heap, and
`CONFIG_STM32H7_DTCMEXCLUDE` remains disabled.

`deps/nuttx/arch/arm/src/stm32h7/stm32_sdmmc.c:3069-3114` does not reject a
buffer whose address intersects:

```text
0x20000000 - 0x20020000
```

which is the STM32H7 DTCM range.

With write-through DCache enabled, the driver's cache-alignment preflight block
is compiled out. An aligned, sector-sized user buffer can therefore:

1. Be allocated from the DTCM general-heap region.
2. Be selected by FAT for a direct multi-sector transfer.
3. Pass SDMMC preflight.
4. Be written directly to the IDMA base-address register.

ST documents that SDMMC1/2 IDMA cannot access TCM:

<https://www.st.com/resource/en/application_note/an5200-getting-started-with-the-stm32h7-mcu-sdmmc-host-controller-stmicroelectronics.pdf>

`CONFIG_FAT_DIRECT_RETRY` only changes to the safe indirect buffer when the
block driver returns `-EFAULT`. Since DTCM is not rejected during preflight,
that fallback does not activate.

**Recommended correction**

Add an overlap-safe DTCM check to `stm32_dmapreflight()` and return `-EFAULT`,
allowing FAT to retry through its DMA pool. Alternatively, exclude DTCM from
the general heap.

### 5. Logger partial-write recovery

The new `log_write_all()` helper uses `lseek(fd, 0, SEEK_CUR)` to determine the
real file position after a FAT write that may have advanced the file before
returning an error. This is a reasonable response to NuttX FAT returning a
negative error after partially advancing `f_pos`.

The host test covers:

- partial progress followed by an error;
- ordinary short writes;
- a temporary stall followed by recovery;
- permanent failure after partial progress;
- a zero-length write.

All cases pass.

However, the helper is not integrated:

- `apps/logger/Makefile` compiles only `logger.c`;
- `logger.c` does not include `log_write.h`;
- `logger.c` does not call `log_write_all()`;
- the firmware ELF does not contain a `log_write_all` symbol.

The original `log_flush()` remains active.

**Additional issue before integration**

`apps/logger/log_write.c:76-119` checks `errno` after calling `lseek()`.
The write error should be saved immediately:

```c
ssize_t n = io->write(...);
int write_errno = errno;
```

Otherwise the subsequent successful or failed `lseek()` may change `errno`,
causing the helper to return the wrong write error.

A permanent failure after partial progress can still leave the file ending in
the middle of a ULog record. The caller must truncate to the last known complete
record boundary before closing the part.

### 6. Logger sample-loss prevention

The sensor-priority fix prevents lower-priority application tasks from causing
hardware FIFO overflow.

The downstream uORB issue remains:

- `apps/logger/logger.c:606-651` performs only one `orb_copy()` for each ready
  topic per poll cycle;
- it does not drain all queued samples;
- it does not compare uORB generations;
- `g_status.dropped` counts logging/serialization failures rather than uORB
  overwrites.

The logger can therefore report zero drops even after subscriber queue loss.

**Recommended correction**

Drain every available sample after a topic becomes ready and compare subscriber
generation numbers. Report separate hardware FIFO, uORB overwrite,
serialization, and filesystem error counters.

### 7. Build reproducibility

The firmware now builds successfully, but the generated NuttX board symlinks
still resolve to:

```text
/home/aniket/xxCar_Nav/src/xxCar_MCU/boards/fmuv6c/...
```

The repository being audited is:

```text
/home/aniket/Documents/codex/xxCar_Nav/src/xxCar_MCU/...
```

The source and include directories were compared during this re-audit and are
currently byte-identical. The generated artifact therefore contains the
expected board implementation at this moment.

The build remains relocation-unsafe: a future change made in only one clone can
silently build different board code.

## Remaining critical findings

### 8. Parameter saving is still destructive

`apps/param/param.c:503-544` still:

- opens the live parameter file with `"w"`;
- truncates it before proving the new contents can be stored;
- ignores every `fprintf()` result;
- does not explicitly flush or `fsync()`;
- ignores the `fclose()` result;
- reports success based on parameters visited rather than bytes made durable.

**Risk**

SD removal, reset, a full card, DMA failure, or FAT error can leave an empty or
partial `params.txt` while calibration reports success.

**Required correction**

Write and validate a temporary file, flush and synchronize it, close it
successfully, then replace the live file.

### 9. RC timeout is not published to controllers

`apps/rc/rc.c:339-357` detects silence and updates only its private status.
It does not publish a new `rc_in` message with:

- `ok=false`;
- `failsafe=true`;
- a current timestamp;
- safe channel values.

The last valid uORB command can remain visible indefinitely.

Every controller should enforce message freshness, and the RC driver should
publish an explicit invalid transition immediately upon timeout.

### 10. PX4IO short-write handling can falsely report success

`apps/px4io/px4io.c:257-260` returns `-errno` when `write()` does not return the
complete request length.

A short positive write does not have to set `errno`, so the function can return
zero or an unrelated stale error.

The transport needs an exact-write loop with explicit handling for:

- short positive writes;
- `EINTR`;
- `EAGAIN`;
- no progress;
- timeout.

### 11. PX4IO reply validation is incomplete

`apps/px4io/px4io.c:309-323` rejects `CORRUPT` and `ERROR` replies but does not
require the reply code to equal `PKT_CODE_SUCCESS`.

Unknown reply codes can therefore be accepted.

### 12. PX4IO retains stale output state

- `g_pwm` is not cleared when the daemon stops.
- `g_rc_valid` is not cleared on restart.
- `px4io_stop()` does not explicitly disarm or zero outputs.
- Restarting the daemon can resend old PWM values.
- Keepalive failures do not stop or reconnect the daemon.
- `px4io_init()` failure is ignored before setting `g_running=true`.

`apps/px4io/px4io_main.c:339-372` also initializes every PWM channel to zero,
ignores failure when reading current values, then writes the entire block.
A failed read while changing one channel can disable every other channel.

## Remaining high-severity findings

### 13. Logger startup can still falsely report success

`apps/logger/logger.c:750-785` waits for `g_running` but returns `OK`
unconditionally.

If no topics are selected or the log file cannot be opened, the daemon exits
and the caller still receives success after the one-second wait.

Concurrent starts can also create multiple logger tasks before either one sets
`g_running`.

### 14. Calibration does not validate the committed result

`apps/cal/cal.c:1081-1090` ignores all `param_set_f32()` and `param_set_i32()`
return values.

The solver permits scale factors wider than the parameter range. A value can
therefore be clamped by the parameter system while the GUI reports the original
unclamped value.

The calculated fit residual is displayed but has no acceptance threshold.
Calibration can be marked valid despite a poor fit.

The stream protocol also stores nominal sample interval in 16 bits, which
overflows at requested rates below approximately 16 Hz.

### 15. Rate parameters remain disconnected

The following parameters still have no consumer outside their definitions:

- `MAV_RATE`
- `SENS_IMU_RATE`
- `SENS_MAG_RATE`
- `SENS_BARO_RATE`

Values loaded from `/fs/microsd/params.txt` do not alter their respective
subsystems.

### 16. MS5611 recovery and compensation remain incomplete

`boards/fmuv6c/src/ms5611.c` still:

- ignores reset-command failure;
- does not verify PROM CRC;
- validates only C1;
- ignores conversion-start failures while changing its pending state;
- implements first-order compensation only;
- does not restart a conversion after an ADC read error;
- periodically replaces a pressure sample with temperature.

A transient bus error can cause stale ADC data to be processed as the wrong
measurement type. Cold-temperature altitude error also remains.

### 17. ICM-42688 invalid and timing handling remains incomplete

The driver rejects invalid accelerometer values but not the corresponding
invalid gyro values or temperature.

It ignores the FIFO's hardware timestamps and reconstructs time from the task
wake-up time using a fixed 500 µs spacing.

FIFO overflow and framing failures flush the FIFO without incrementing visible
health counters.

### 18. IST8310 can publish stale data

The driver still ignores failures from:

- reset;
- averaging setup;
- pulse-duration setup;
- single-measurement trigger.

It reads and publishes the data registers without checking `STAT1.DRDY`.
A failed trigger can therefore republish the previous magnetic sample with a
new timestamp.

### 19. BMI055/BMI088 detection and runtime support disagree

The board probe accepts a BMI055 or BMI088 accelerometer. The runtime secondary
IMU driver accepts only BMI055.

A later FMUv6C fitted with BMI088 can pass the hardware probe and fail sensor
registration.

### 20. MAVLink transport and connection handling remains incomplete

`apps/mavlink/mavlink.c` still:

- performs one `write()` without retrying short or interrupted writes;
- reads an uninitialized parser-status structure before the first parsed byte;
- does not validate parameter message target system/component;
- ignores termios application failures;
- does not handle HUP/error state;
- does not reconnect after removable USB loss;
- uses a hard-coded heartbeat rate while `MAV_RATE` is unused.

## Remaining medium-severity findings

### 21. Parameter parsing and synchronization

The parameter loader still uses `strtol()` and `strtof()` without checking:

- the end pointer;
- `errno`;
- trailing characters;
- numeric overflow.

Parameter live values and initialization state are not protected by a mutex.
MAVLink, calibration, NSH, and startup code can race during load, save, get,
and set operations.

`serial_manager_start()` still calls `param_init()` and can discard unsaved
live changes.

### 22. USB mass-storage remount recovery

The release path changes the USB configuration to CDC-only before attempting
the SD remount. If mounting fails, later release calls see CDC-only and return
without retrying the mount.

The export rollback path also ignores remount failure.

### 23. Multiple RC publishers

Serial RC and PX4IO both advertise and publish the same `rc_in` topic. If both
are enabled by the SD parameter file, messages from different sources can
interleave, and one source can hide loss of the other.

### 24. Sensor initialization return value

`fmuv6c_sensors_initialize()` overwrites one `ret` variable for every sensor and
returns only the final registration result. Earlier failures can be hidden, and
board bring-up ignores the aggregate result.

### 25. Binary protocols remain allowed on the syslog console

The serial manager warns but permits MAVLink or another protocol on the console.
Syslog output can be inserted between bytes of outgoing binary frames.

### 26. Shared start/stop state remains racy

Logger, MAVLink, RC, PX4IO, and several sensor fields use volatile or ordinary
globals without consistent mutex or atomic state transitions.

Possible results include:

- duplicate daemons;
- stale status;
- interval changes racing workers;
- mixed parameter snapshots;
- start and stop reporting the wrong outcome.

## New observations in the updated code

### 27. FAT mounting continues after DMA allocator initialization failure

`boards/fmuv6c/src/stm32_bringup.c:511-527` logs an error if
`stm32_dma_alloc_init()` fails but continues to initialize and mount the card.

With `CONFIG_FAT_DMAMEMORY`, later FAT allocations call `gran_alloc()` with the
global allocator handle. If initialization failed, that handle is null.

The card mount should be skipped or FAT should be provided with a defined safe
fallback.

### 28. Priority documentation places the watchdog on the wrong queue

The priority comment says HPWORK services the watchdog. NuttX's
`WATCHDOG_AUTOMONITOR_BY_WORKER` implementation queues its feed operation on
LPWORK, which is priority 100 in this configuration.

The chosen sensor priority remains reasonable: a sensor thread that spins
continuously should eventually cause a watchdog reset. The documentation should
nevertheless describe the actual scheduling arrangement.

## Verification results

### Host tests

All ten available host tests passed:

- Allan GUI
- Allan analysis
- accelerometer calibration
- calibration GUI
- logger partial-write helper
- MAVLink
- parameter ranges
- RC decoding
- ULog parser
- ULog splitting

### Firmware build

`tools/verify.sh` completed successfully:

- firmware build exited zero;
- `build/xxcar.px4` was rebuilt;
- artifact size: 277,816 bytes;
- `cal_session` linked;
- `cal_main` linked;
- `param_init` linked;
- `serial_manager_start` linked;
- calibration style gate passed.

The linked ELF size is:

| Segment | Bytes |
| --- | ---: |
| Text | 434,132 |
| Data | 4,968 |
| BSS | 109,176 |
| Total | 548,276 |

### Static analysis

`cppcheck` was run across the application and FMUv6C board sources with
warning, performance, and portability checks. It produced no diagnostics.

### ELF and source-path checks

- The FAT DMA allocator and logger buffer are in AXI SRAM.
- The FAT allocator functions are linked.
- The uncommitted `log_write_all()` helper is not linked.
- The external board-symlink target currently matches the audited board source
  byte for byte.

## Recommended remediation order

### Before moving actuators

1. Publish RC timeout/failsafe state and enforce controller message freshness.
2. Fix PX4IO exact-write and strict reply validation.
3. Explicitly disarm and clear output state on stop, startup failure, and
   restart.
4. Abort CLI PWM changes when the existing output block cannot be read.
5. Select one authoritative RC source.

### Before trusting parameters or calibration

1. Make parameter saving transactional.
2. Add parameter locking and strict numeric parsing.
3. Check every calibration parameter setter.
4. Reject excessive fit residual, offsets, or scales.
5. Mark calibration valid only after durable persistence succeeds.

### Before long recordings

1. Reject DTCM in SDMMC preflight or remove it from the general heap.
2. Integrate and correct `log_write_all()`.
3. Recover to a complete ULog record boundary after permanent partial writes.
4. Fix logger startup state reporting.
5. Drain uORB queues and detect generation loss.
6. Exercise the logger under injected short writes and 200–500 ms SD stalls.

### Before navigation development

1. Repair MS5611 CRC, compensation, and error-state recovery.
2. Use ICM FIFO timestamps and validate gyro/temperature samples.
3. Check IST8310 configuration, trigger, and DRDY.
4. Resolve BMI055/BMI088 runtime selection.
5. Connect or remove unused rate parameters.
6. Add observable sensor error, overflow, recovery, and effective-rate
   counters.

## Information required for configuration-specific review

1. The exact `/fs/microsd/params.txt` used on the board.
2. Pixhawk 6C hardware revision.
3. Whether the secondary accelerometer identifies as BMI055 or BMI088.
4. Which physical port carries the boot/syslog console.
5. Whether RC should come from PX4IO, a direct serial receiver, or an
   arbitration layer.

These inputs determine whether the dual-RC publisher, console/MAVLink
interleaving, and secondary-IMU compatibility risks are active on the current
vehicle.
