# Matek low-level audit and ArduPilot reuse plan

> Decision update: NuttX is retained, as requested. The ChibiOS migration recommendation and migration work packages below are superseded by [the NuttX resource plan](matek-nuttx-resource-plan-2026-09-21.md). The source findings remain relevant; this document preserves the original audit evidence.

## Decision in brief

The active source configuration is **Matek H743-SLIM-V4**. There is no demonstrated CPU, RAM, or DMA-stream capacity shortage for the stated workload. The more consequential gaps are error handling, clock discipline, resource ownership, and reproducibility. More peripherals are not required to trigger these failures.

For the stated objective—reuse proven ArduPilot infrastructure—my preferred destination is a **small xxCar application on ArduPilot's native AP_HAL_ChibiOS**, with its board generator, pinned ChibiOS, bus drivers and RC protocol library retained together. Keep the custom companion protocol, VESC protocol, control policy and estimator behind a platform boundary. Do not translate individual ChibiOS register drivers into NuttX and expect to retain their validation history. This is a migration proposal, not a claim that NuttX itself is unsuitable, nor authorization to replace the working firmware.

First fix/test the safety-critical defects below on the current branch. Prove the replacement HAL on the exact small workload before migrating the estimator or changing Pixhawk 6C.

## Evidence and limits

- Local HEAD: `369e98237cadec72056f24bb86a281093c224310`, **including the existing dirty working tree**. NuttX: `b9b5d9afbf5a88f3c52830d4ce88ae038be0fd35` plus local patches. Existing edits were preserved.
- Retrieved ArduPilot source: `663fc09bb25b84a756d8585e4f6d9266a0ab53ae`; its actual ChibiOS submodule: `9aebaf4a40442277d01bd9dd38c13713fc45ef41`. Ran its Matek hardware-definition generator successfully, including DMA-map and linker-script generation. This is a reproducible source comparison, **not a release qualification or firmware build**.
- Manufacturer identifies this V4 as dual ICM42688P plus DPS368; do not substitute older H743 variant specifications. [Matek source][matek]
- Reviewed local PX4 FMUv6C board definitions at `72ab92881ae4a7fb41d13d4a40f978f881811bc7` as a second reference.
- No connected-board measurements, live parameter dump, bootloader readback, bus waveform, stack high-water measurement or new target build was performed. The flashed image and stored parameters are not established by `.config`. Findings below distinguish source defects from unproven explanations of the reported incidents.

## Power-on through runtime: comparison and disposition

| Stage | Current stack | ArduPilot reference and recommended disposition |
| --- | --- | --- |
| 1. Build identity | Shared FMUv6C defconfig, Matek overrides in `tools/build.sh:93`. Overrides run only on reconfigure/board change. Same-board incremental builds can retain settings after the script/defconfig changes. | Generate each board from one definition into an independent build directory. Embed firmware, configuration, patch and board-revision hashes. Reconfigure when inputs change. ArduPilot's `hwdef.dat`/generator is the model, not another handwritten pin table. |
| 2. Bootloader handoff | Application starts at `0x08020000`; current startup patch sets VTOR early. The installed bootloader is unknown. | Matek bootloader definition reserves 128 KiB and deasserts sensor CS pins. `jump_to_app()` disables peripheral clocks/interrupts and sets VTOR. Verify the actual bootloader before assigning startup symptoms to it. [Bootloader][boot] |
| 3. Clock/power reset | NuttX `rcc_reset()` requests HSI and switches CFGR without the explicit HSIRDY/SWS waits present upstream in ChibiOS. Current APB1-mask correction is already applied. | ChibiOS explicitly waits for HSI and its selection, programs full divider/kernel-source registers, voltage/flash settings, then switches clock. Adopt the complete sequence as a unit if migrating; if retaining NuttX, audit its transition against ST and test cold/warm/debugger resets. Missing waits are a robustness gap, not a reproduced root cause. [Clock initialization][chclock] |
| 4. Clocks used | Matek HSE 8 MHz; CPU 480, HCLK 240, APB 120 MHz; APB timers 240 MHz. SPI kernel PLL2 is **96 MHz**; USB PLL3Q 48 MHz; FDCAN HSE 8 MHz. | Default AP Matek profile uses CPU 400 MHz, FDCAN 80 MHz, generated clock checks and explicit kernel sources. Different does not mean faulty: changing CPU/CAN clocks alone is not a demonstrated fix. Start a native-AP prototype with its unmodified clock profile. [MCU configuration][mcuconf] |
| 5. Safe pins/cache/RAM | Early board init does configure SPI CS. Cache is write-through. AXI, SRAM123 and SRAM4 feed the general heap; DTCM is excluded. | AP initializes generated GPIO states very early, configures MPU/cache attributes, and allocates by memory capability. Preserve safe CS/output states through all reset paths; distinguish CPU, DMA1/2 and SDMMC1 memory. [Early initialization][boardinit], [allocator][allocator] |
| 6. OS and timebase | 1 ms SysTick maintains OS time; separate 1 MHz TIM5 timestamps sensors/link; TIM6 schedules downlink. | AP Matek uses TIM12 for a 1 MHz ChibiOS system timebase and `AP_HAL::micros64()`. Give all sample timestamps and deadline/age checks one monotonic clock. UTC is a separate mapping; never discipline the scheduler with UTC. [System time][systime] |
| 7. Storage and parameters | SD/USB initialize before sensors/apps. Matek flash journal uses sectors 14/15 and commit-last records. Parameter save has no transaction-wide writer lock or armed-state erase gate. | Keep the journal's crash protection. Serialize saves, snapshot values, and defer erase to disarmed maintenance. AP storage has an explicit disarmed erase gate and background servicing. Flash layout compatibility does **not** make parameter formats compatible. [Storage][storage] |
| 8. Buses and sensors | SPI1/SPI4 DMA, IMU primary DRDY and secondary 4 ms FIFO polling. I2C timeout is 500 ms; `CONFIG_I2C_RESET` is off. | Keep polling for a sensor without verified DRDY wiring. AP provides per-bus ownership/callbacks, transfer timeouts, I2C clear/retry, and ongoing checked-register surveillance. Bring those contracts with the drivers. [SPI][spi], [I2C][i2c], [IMU][imu] |
| 9. Services | Board bringup directly calls parameter/serial/PPS/companion/VESC/router/sensor/estimator APIs. A required boot failure becomes a log summary, not a shared readiness gate. | Separate board initialization, driver readiness and application startup. Critical clock/resource faults must inhibit dependent outputs; optional SD/barometer faults should degrade only dependent functionality. Do not indiscriminately require an estimator for manual control. |
| 10. Runtime supervision | Short critical sections, task mutexes, IRQ-posted semaphores; no priority inheritance or selected peripheral IRQ priorities. Automatic watchdog feeding is not tied to application progress. | Use bounded ISR work, PI-capable bus mutexes, explicit priority/lock rules, and a supervisor that checks relevant progress before feeding. AP differentiates IRQ/task priorities and feeds its watchdog through main-loop progress. Copy the policy, not numeric priorities between different OSes. [HAL lifecycle][hal], [scheduler][scheduler] |

Local startup evidence: `deps/nuttx/arch/arm/src/stm32h7/stm32_start.c:185`, `stm32h7x3xx_rcc.c:185`; `boards/fmuv6c/src/stm32_boot.c:52`, `stm32_bringup.c:410`; `boards/fmuv6c/include/board.h`.

## Matek allocation: what actually competes

| Resource | Current allocation | AP Matek allocation / implication |
| --- | --- | --- |
| SPI1 / primary IMU | PA5/PA6/PD7, CS PC15, DRDY PB2; RX+TX DMA1 | Same SPI/CS wiring; generated dedicated IMU DMA streams. |
| SPI4 / secondary IMU | PE12/PE13/PE14, CS PC13; RX+TX DMA2, polling | Same SPI/CS wiring for this sensor position. AP's historical `icm42605` device name is not evidence that V4 contains that older chip. |
| CAN1 | RX PD0, TX PD1, silent PD3 low; FDCAN message RAM, **no DMA stream** | Same pins; AP uses `CANFDIface.cpp` on H7, not the bxCAN implementation. |
| R6 / PC7 | USART6 inverted 100 kbaud 8E2 for SBUS, **or** TIM3 CH2 for PPM; USART6 DMA disabled | AP default is TIM3 pulse capture, capable of decoding serial RC too; alternative UART6 configuration is NODMA. DMA-off here is not inherently wrong. |
| S3 / PA0; S4 / PA1 | TIM2 CH1/CH2 steering; no output DMA required | AP uses TIM5 CH1/CH2. Both AF choices are electrically legitimate; changing the timer number is not itself a fix. |
| PPS / PA0 | TIM5 CH1 on **the same S3 pad** | Physical conflict with steering regardless of the different timer instances. Reserve another verified capture pad or move the two PWM outputs; reject conflicts at every start/stop entry point. |
| Time / TX tick | TIM5 base; TIM6 update IRQ | AP system time uses TIM12, leaving TIM5 available for outputs. Preserve that plan in a native AP port; add PPS only after a complete pad/channel claim check. |
| I2C | I2C1 PB6/PB7 external; I2C2 PB10/PB11 onboard barometer | Same physical buses. No requirement to add DMA to these small transactions. |
| SD / USB | SDMMC1 PC8–PC12/PD2, IDMA; USB FS PA11/PA12, two CDC functions plus optional MSC | Same physical interfaces; storage DMA has stricter RAM reachability than ordinary DMA1/2. |
| UARTs | ttyS0 USART1 PA9/10; S1 USART2 PD5/6; S2 USART3 PD8/9; S3 UART4 PB9/8; S4 USART6 PC6/7; S5 UART7 PE8/7; S6 UART8 PE1/0 | Pins agree with AP. Logical GPS/TELEM aliases differ: expose physical UART/pad names in the board manifest. USART2 and UART4 RX use DMA2; other enabled UARTs do not use RX DMA. |

For this enabled configuration, the two SPI buses and two DMA RX UARTs require **2 DMA1 + 4 DMA2 streams**, out of eight per controller. NuttX dynamically chooses streams through DMAMUX; board macros choose the controller. I found no stream-capacity conflict. AP's generated full-featured board instead dedicates DMA1 streams 0/1 to SPI1 and 2/3 to SPI4 and explicitly marks shared streams for other functions. Do not import that complexity unless needed. [Board definition][hwdef], [DMA resolver][dma]

ADC1/2/3, Ethernet, SPI2/3 and SDMMC2 are disabled in the active Matek configuration. Importantly, stock FDCAN, PWM/capture and TIM2/3/5/6 driver options are also disabled **while the custom board code operates those peripherals directly**. Kconfig alone therefore cannot serve as the resource inventory or prevent a future second driver from claiming them. The generated manifest must include these direct-register owners.

## Defects to address, in order

### 1. CAN failure recovery and stale actuation — confirmed gaps

`boards/fmuv6c/src/fdcan.c:101,672,749` enables receive/loss IRQs, but has no bus-off recovery, transmit deadline, cancellation or completion tracking. When its 32-entry TX FIFO fills, it drops the new command while older commands remain pending. Reconnection can therefore transmit obsolete intent; one device is sufficient. `apps/vesc/vesc.c:819` queues neutral on stop, then deinitializes without confirming transmission.

Use AP's H7 CAN interface, including `select()` housekeeping: it cancels expired mailboxes and clears INIT following bus-off detection. Keep the VESC protocol separate from the controller driver. Add latest-command replacement/deadlines and explicit neutral-completion/timeout handling; verify the VESC's independent command timeout. A single-owner TX path means the present absence of a TX mutex is **not** evidence of the reported failure. [H7 CAN driver][can]

### 2. SBUS transport integrity and failsafe — confirmed gaps; incident cause unproven

The active NuttX byte path gets USART error status but discards it in `drivers/serial/serial_io.c:267`. The RC decoder sees bytes without parity/framing/overrun metadata. `apps/rc/rc_decode.c:174` has no elapsed-time input or frame-gap reset; a partial packet can survive a long gap. SBUS has no payload CRC to compensate. Forced-mode setup also ignores `rc_configure_port()` failure (`apps/rc/rc.c:373`).

AP's UART path discards parity-error reads; its SBUS parser uses frame gaps. Reuse the protocol backend with a transport that preserves timing and rejects errored receive spans. Do **not** merely enable USART error IRQs: this local ISR has no explicit parity-error clear branch, which must be audited too. Keep the selected protocol fixed; autodetection is not needed for this workload. [UART][uart], [SBUS][sbus]

Separately, `rc.c:452` combines receiver failsafe and frame-lost into the same ten-invalid-frame hold policy, refreshes the held command's timestamp, and suppresses failsafe until the threshold. Separate last-good age, frame loss and explicit receiver failsafe. Do not present held channels as newly measured channels.

### 3. PPM channel identity — deterministic failure mechanism

`matekh743_io.c:234` accepts any frame with at least four channels; there is no expected-count lock and no digital input filter. A lost physical edge can merge two 1500 us intervals into a false 3000 us sync. Five remaining channels can then be published as channels 1–5. The code detects hardware overcapture, but a missing/noisy wire edge need not set that flag.

For a fixed receiver, require the configured channel count, reject partial frames, reacquire across consecutive consistent frames, and validate input filtering/polarity with a trace. AP's two-edge capture has filtering and overcapture-to-parser reset. **Its PPM backend also uses a 2700 us gap and minimum count**, so decoder substitution alone does not eliminate this ambiguity. PPM cannot provide CRC-level integrity. [Capture][capture], [PPM decoder][ppm]

### 4. Time synchronization — reproduced control-loop defect

`comp_clock.c:388` adds phase-derived correction to the **already corrected rate**. `companion.c:920` repeats this every PPS. I compiled the actual clock implementation with ideal one-second observations and an initial 100 ppm rate error. Phase error repeats:

```text
seconds:       1    2    3     4     5    6    7 ...
error (us):  100  100    0  -100  -100    0  100 ...
```

It does not settle even without jitter. Existing `test-comp-clock.sh` passes: it lacks this closed-loop test. This demonstrates an undamped loop, **not that every observed accumulating drift has this cause**, particularly if PPS is disabled.

Other source-level biases remain: `TIMESYNC_END` carries no selected-observation epoch but is fitted at END reception (`companion.c:649`); reply TX time is sampled before acquiring the shared TX lock (`:713`, `:370`); that lock covers bounded write/retry sleeps. Host `UtcClock` anchors wall time once, so later wall-clock steps are not adopted. Linux monotonic frequency slewing should not be confused with those steps.

Use one monotonic clock, a separately estimated oscillator rate, and a **bounded, damped** UTC phase correction that is not repeatedly accumulated into the base rate. Carry observation epoch/session identity/uncertainty; test loss, outliers, oscillator error and simultaneous serial/PPS updates. AP's RTC is chiefly a UTC offset/source-policy service, **not a drop-in precision PPS/PTP servo**. Do not promise that importing it solves this requirement. [AP RTC][rtc]

TIM5 and SysTick are different counters but derive from the same HSE/PLL tree here, not independent crystals. Persistent relative drift therefore needs investigation of clock configuration, lost tick accounting and timestamp conversion; it is not an inevitable oscillator mismatch. Keep a measured TIM5-minus-OS-time trace alongside PPS period and UTC residual. The HSE-clocked RTC cannot maintain elapsed time through loss of main power.

### 5. Ownership and boot readiness — confirmed structural gaps

The S3/PPS conflict is checked only by boot policy (`stm32_bringup.c:879`). Runtime `pps start` calls the GPIO remux directly; PWM start does not reject active PPS. `board_matek_steering_pwm_healthy()` checks timer enable bits, not the pad's actual AF ownership. Thus health can remain true while S3 has become an input.

Use generated static exclusivity checks plus runtime claims at the hardware boundary. Apply this to pads, timer channels/base frequency, serial reader, DMA streams and storage ownership. Expose readiness explicitly instead of relying on the boot summary. Resource ownership must not depend on which CLI/application called first.

### 6. SPI/DMA failure containment — confirmed gap

NuttX `stm32_spi.c:1129,1167` waits indefinitely on DMA completion semaphores. A missing completion can strand a sensor worker while the automatic watchdog continues to be fed elsewhere. Current ICM configuration checks timing registers, but a failed `config_verified` only changes logging; only `stream_verified` prevents registration (`icm42688.c:449–479`). There is no equivalent ongoing checked-register surveillance in its normal read loop.

AP SPI has a finite transfer timeout and abort; its Invensense driver checks registers during normal operation. Reuse both layers, or implement deadline/abort/reprobe semantics together on NuttX. Do not claim changing task priority fixes a permanently blocked DMA wait. [SPI][spi], [IMU][imu]

### 7. Memory/USB/I2C/storage — bounded secondary work

The existing Matek ELF has 5,168 bytes data and 139,248 bytes BSS; this is **an existing artifact**, not a verified build of today's dirty source. Its layout leaves roughly 720 KiB across the configured heap regions before runtime allocations. The 4 KiB FAT DMA pool is a separate limit; DTCM's 128 KiB is excluded, not exhausted. Obtain heap minima and stack/IRQ high-water marks before resizing.

The SDMMC preflight patches now reject unreachable/unaligned buffers, and FAT has a safe retry pool. However USB MSC allocates its sector buffer from the general heap (`usbmsc.c:1564`) and accesses the block driver directly. On Matek, an SRAM123/SRAM4 allocation is rejected by SDMMC1 preflight with no FAT retry. This is an **allocation-dependent MSC I/O failure**, not proof of current memory corruption. Allocate that buffer from AXI-capable memory or use a block-layer bounce buffer. ST documents SDMMC1's restricted bus reachability. [ST manual][st]

USB export correctly unmounts before handing over the disk; companion already has a reconnect loop. Retain those improvements. Serialize composite/storage transitions and allow MSC as a disarmed maintenance operation. AP even offers a dedicated mass-storage boot path; live composite switching is not necessary for basic control reliability.

I2C reset is disabled and no board sensor recovery calls it. Add bounded bus clear/reinitialization if keeping NuttX; a 500 ms timeout should not block unrelated buses/control. Keep flash saves serialized and disarmed. I found **no blanket IRQ-off erase loop** in the current H743 flash implementation, so flash erase cannot responsibly be asserted as the explanation for the reported clock drift.

## OS/interrupt budget and migration boundary

Actual daemon priorities, not CLI Kconfig priorities: HPWORK 224; sensor workers 150; IMU integration 125; EKF 122; sensor aggregation 120; router 115; companion 112; VESC 110; RC 105; logger 102. Higher NuttX numbers win. Hardware IRQ preemption is a different axis: `CONFIG_ARCH_IRQPRIO` is off. The ISR stack is 2 KiB; this has not been proven adequate by runtime measurement.

RC/SPI/CAN tasks mostly block or wait; there is no measured utilization evidence justifying a blanket priority increase. Enable latency/critical-section measurement and distinguish hardware FIFO overrun, software-ring overrun, missed deadlines and stale publications. Size buffers from measured blackout intervals. PI helps bus locks; it does not fix IRQ masking, stale queues or an unstable clock servo. Never hold a critical section across allocation, I/O, formatting or sleeping.

For the replacement, use `AP_HAL_MAIN()`/a custom target as the small application entry point. Reuse `AP_HAL_ChibiOS`, generated Matek board definitions, `CANFDIface`, UART/SPI/I2C/RCOutput and `AP_RCProtocol`; integrate AP's sensor backend with its real `AP_InertialSensor`/parameter dependencies. AP_Periph is a useful build example, but already brings a substantial dependency set and DroneCAN assumptions—it is not automatically the smallest xxCar solution. Preserve license notices when reusing upstream code.

NuttX tasks, file descriptors, uORB, shell and logger are not drop-in ChibiOS APIs. Define narrow application interfaces for monotonic time, bounded queues, bus access and actuator publication; migrate one producer/consumer chain at a time. Retaining NuttX is a valid alternative if these dependencies dominate, but then prefer its/PX4's native infrastructure and describe the result as a maintained port, not unchanged ArduPilot HAL reuse.

Pixhawk 6C is a regression reference, not evidence that the Matek port is safe: its USART6 serves a separate IOMCU, whereas Matek puts RC capture and direct PWM responsibility on the H743; it also uses a different HSE and SDMMC instance. [AP Pixhawk6C definition][pixhawk]

## Work packages and acceptance gates

1. **Reproducible board baseline:** independent clean board configs; archived live parameters, bootloader ID, clock/GPIO/DMA register dump; cold/warm reset tests with peripherals powered before/after MCU. Unchanged configuration must yield identical allocations.
2. **Contain actuation faults:** CAN bus-off/reconnect and expired-TX tests; final-neutral completion/timeout; runtime PWM/PPS conflict must return an error without changing registers. Start with outputs unloaded.
3. **Make RC integrity observable:** fixed SBUS configuration, injected parity/drop/gap tests; explicit failsafe and last-good timestamps. PPM missing/extra-edge tests must never publish shifted channels.
4. **Repair and qualify time:** closed-loop tests, shared time API, observation-epoch protocol; long PPS/reference captures including over 72 minutes, source loss/reacquisition and host clock changes. Set accuracy/holdover requirements before selecting servo gains.
5. **Prove native AP HAL reuse:** boot, one CAN device, fixed UART RC, two IMUs, two PWM outputs—no estimator initially. Add USB/logging stress and forced SPI/I2C failures. Compare against the same tests on current NuttX and Pixhawk 6C.
6. **Migrate application logic only after the HAL passes:** retain companion/VESC wire formats and safety contracts; adapt message transport, then estimator/logger. No simultaneous rewrite of drivers and estimator.

Executed checks: existing clock (normal + UBSan), FDCAN RAM/ring, RC decode, control-router and parameter-file save tests passed. These do not validate hardware timing, Matek flash-journal concurrency, PPM capture or CAN recovery. Added no firmware changes; the PPS diagnostic harness/executable and retrieved source are under `/tmp/xxcar-*`.

PPS reproduction command, while the temporary harness remains available:

```sh
cc -std=c11 -Wall -Wextra -Werror -DFAR= -Iapps/companion \
  /tmp/xxcar-audit-pps.c apps/companion/comp_clock.c -o /tmp/xxcar-audit-pps
/tmp/xxcar-audit-pps
```

The harness establishes UTC, sets an initial `rate_ppb = 100000`, and once per ideal second applies `comp_clock_adjust_phase(clock, now, -phase_error, 1000000)`. It isolates the repeated correction law; it does not simulate physical PPS capture or the full concurrent companion application.

## Pinned upstream references

[matek]: https://www.mateksys.com/?portfolio=h743-slim-v4
[hwdef]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/hwdef/MatekH743/hwdef.dat
[boot]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/Tools/AP_Bootloader/bl_protocol.cpp
[chclock]: https://github.com/ArduPilot/ChibiOS/blob/9aebaf4a40442277d01bd9dd38c13713fc45ef41/os/hal/ports/STM32/STM32H7xx/hal_lld.c
[boardinit]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/hwdef/common/board.c
[mcuconf]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/hwdef/common/stm32h7_mcuconf.h
[allocator]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/hwdef/common/malloc.c
[dma]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/hwdef/scripts/dma_resolver.py
[systime]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/system.cpp
[hal]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/HAL_ChibiOS_Class.cpp
[scheduler]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/Scheduler.h
[storage]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/Storage.cpp
[can]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/CANFDIface.cpp
[uart]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/UARTDriver.cpp
[sbus]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_RCProtocol/AP_RCProtocol_SBUS.cpp
[capture]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/SoftSigReaderInt.cpp
[ppm]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_RCProtocol/AP_RCProtocol_PPMSum.cpp
[spi]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/SPIDevice.cpp
[i2c]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/I2CDevice.cpp
[imu]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp
[rtc]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_RTC/AP_RTC.cpp
[pixhawk]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6C/hwdef.dat
[st]: https://www.st.com/resource/en/reference_manual/dm00314099.pdf
