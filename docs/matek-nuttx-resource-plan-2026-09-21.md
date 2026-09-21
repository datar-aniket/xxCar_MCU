# Matek H743-SLIM-V4: NuttX resource and isolation plan

## Decision

Keep NuttX, its POSIX interfaces, and the application architecture. Use PX4/NuttX as the native platform reference and selectively port ArduPilot's device/protocol logic and failure-handling contracts. A translated ChibiOS driver is a maintained port, not an unchanged, already-qualified driver.

**Revised baseline, per the user's pin constraint: eight servo outputs on S1–S8; PPS moves to LED/PA8; S9–S12 remain available for PWM expansion.** S1–S4 remain servo outputs and current S3/S4 steering wiring is preserved. TIM5 remains the pinless 32-bit timestamp clock. RC, CAN, external I2C, all seven UARTs, ADC, USB and SD remain available. Trade-offs: surrender the addressable-LED pad and disable TIM2-based buzzer tones. The onboard status LEDs remain available.

Implementation has begun with the pin-definition stage: `matekh743_pins.h` selects the PWM/ADC/UART/I2C/SPI mappings, rejects conflicting template features, and supplies early servo-low/buzzer-off states. Existing S3/S4 steering uses that map. The additional PWM channels and ADC acquisition are **not enabled yet**. Matek's legacy S3 PPS driver is excluded; start/stop reports unsupported while the PA8/TIM1 port is deferred. The remaining sections describe the target design, not completed hardware qualification. Source identities and the earlier fault analysis are in [the audit](matek-ardupilot-low-level-audit-2026-09-21.md). Physical board revision, installed bootloader and live saved parameters still need confirmation.

PWM follow-up: the user selected independent timer banks with aligned rising edges within each bank. S3/S4 startup and paired shadow updates are hardened in `matekh743_pwm.c`, with a register-model regression test. See [PWM qualification](matek-pwm-qualification.md) for guarantees, pending channels and the required bench measurements; electrical jitter has not been measured.

## 1. Preferred timer/output assignment

| Logical use | Physical pad / MCU pin | Peripheral / AF | Ownership |
| --- | --- | --- | --- |
| Servo 1 | S1 / PB0 | TIM8 CH2N / AF3 | PWM bank A |
| Servo 2 | S2 / PB1 | TIM8 CH3N / AF3 | PWM bank A |
| Servo 3, front steering | S3 / PA0 | TIM2 CH1 / AF1 | PWM bank B |
| Servo 4, rear steering | S4 / PA1 | TIM2 CH2 / AF1 | PWM bank B |
| Servo 5 | S5 / PA2 | TIM2 CH3 / AF1 | PWM bank B |
| Servo 6 | S6 / PA3 | TIM2 CH4 / AF1 | PWM bank B |
| Servo 7 | S7 / PD12 | TIM4 CH1 / AF2 | PWM bank C |
| Servo 8 | S8 / PD13 | TIM4 CH2 / AF2 | PWM bank C |
| Expansion 9/10 | S9/S10 / PD14/PD15 | TIM4 CH3/4 / AF2 | Same PWM bank C |
| Expansion 11/12 | S11/S12 / PE5/PE6 | TIM15 CH1/2 / AF4 | Additional PWM bank D |
| PPS | LED / PA8 | TIM1 CH1 / AF1 | Dedicated PPS timer; reserve all TIM1 |
| Monotonic timestamps | No additional pad | TIM5, 32-bit, 1 MHz | Timestamp service owns entire timer base |
| Optional PPM | R6 / PC7 | TIM3 CH2 / AF2 | Exclusive alternative to USART6 RX |
| Existing downlink tick | No pad | TIM6 | Downlink scheduling only |
| Buzzer | PA15 | Safe inactive GPIO, no timer AF | Disable TIM2 tone generation |

S1/S2 and S7–S12 match [ArduPilot's Matek definition][ap-hwdef]. For S3–S6, use the MCU's valid TIM2 alternate functions instead of ArduPilot's TIM5 selection, preserving the existing NuttX timestamp clock. These alternatives are confirmed in both upstream MCU maps. Each bank shares its frame frequency, but has independent pulse widths. This means eight servo channels with **three frequency groups**, not eight independent-frequency generators. Start with a conservative common servo-compatible rate; change a bank's rate only when every attached servo supports it. Plain servo PWM requires no DMA.

NuttX already implements TIM8 complementary outputs and TIM15 multichannel PWM. Complementary outputs are not plug-and-play normal outputs: explicitly configure their polarity, idle state and main-output enable, and scope startup/shutdown to prove short positive servo pulses rather than their inverses. Do not configure the unused TIM8 main-output pins. A timer-bank failure can affect all channels in that bank; move the two steering outputs to different banks if independent timer-bank fault containment becomes a requirement.

PPS owns TIM1, independently of all PWM banks and TIM5. Its initialization/recovery must never reset or retune TIM5, TIM2, TIM4 or TIM8. An invalid/noisy PPS must be quarantined without stopping monotonic time or servo outputs. Reject overcapture and implausible intervals, bound ISR work, and disable a storming capture channel pending controlled recovery. PA8's capture alternate function and physical LED pad are source-confirmed; verify the actual board's input path/protection and use appropriate PPS logic levels before committing the wiring.

### PPS implementation requirement

TIM1 is 16-bit: at 1 MHz it wraps every 65.536 ms. This is not a configuration-only PPS move. Implement overflow extension with capture/update-race handling, correlate that counter to the common TIM5 epoch, and detect missed-overflow/latency violations. Use TIM1's actual APB2 timer clock and capture/update IRQs, not TIM5's APB1 constants or vector. NuttX's generic capture frequency/duty interface alone is not the required timestamp API.

The existing non-Matek `pps_isr()` subtracts 16-bit CNT-CCR from a later TIM5 reading. Contrary to its comment, modulo subtraction does not recover arbitrary multi-wrap capture-to-ISR delays; it needs a proven latency bound below one wrap, and the sequential counter reads introduce timestamp bias. Do not blindly transplant that branch. Test both sides of overflow, delayed ISR service, overcapture, missing pulses and PPS restart while PWM continues.

S9/S10 extend TIM4 and S11/S12 add TIM15: **up to 12 PWM outputs plus PPS**, subject to integration and qualification. If the LED pad cannot accept PPS on the actual board, use S11/PE5 TIM15 CH1 instead and reserve the whole TIM15 for PPS; S1–S8 remain unchanged, but S11/S12 PWM expansion is lost. No change to S1–S4 is required in either case.

## 2. Serial and bus assignment

Keep the already-correct Matek physical UART mappings. Proposed functions below preserve the existing companion and console defaults; GPS/flow/spare are proposed roles, not verified live settings.

| Role | UART / device | TX / RX | Notes |
| --- | --- | --- | --- |
| Fixed SBUS RC | USART6 / ttyS4 | PC6 / PC7, T6/R6 | Inverted 100 kbaud 8E2; no autodetection; PPM is an explicit mutually exclusive mode |
| Optical flow | USART1 / ttyS0 | PA9 / PA10, T1/R1 | Driver/baud depend on the actual flow device; current alias is GPS1 |
| GPS | USART2 / ttyS1 | PD5 / PD6, T2/R2 | RX DMA already enabled; current alias is TELEM3 |
| Spare | USART3 / ttyS2 | PD8 / PD9, T3/R3 | Currently the fallback DEBUG shell; explicitly preserve/relocate rescue-console policy |
| Companion | UART4 / ttyS3 | PB9 / PB8, T4/R4 | Keep current TELEM2 assignment and RX DMA |
| Console/debug | UART7 / ttyS5 | PE8 / PE7, T7/R7 | Keep current TELEM1 console; PE9/PE10 remain optional RTS/CTS |
| Additional spare | UART8 / ttyS6 | PE1 / PE0, T8/R8 | Current alias GPS2 |

There is ample UART capacity; adding RC DMA is not the prerequisite for reliable SBUS. ArduPilot also marks its alternative USART6 configuration NODMA. Preserve receive-error information and frame timing instead. Optical-flow serial support is a separate driver/protocol task; a free UART alone does not implement it.

| Function | Fixed resource | Isolation requirement |
| --- | --- | --- |
| External I2C | I2C1, PB6 SCL / PB7 SDA | Bounded timeout, bus clear/reprobe; separate worker/lock from onboard barometer |
| Onboard barometer | I2C2, PB10 / PB11 | Keep expansion devices off this bus when fault isolation is important |
| CAN | FDCAN1, PD0 RX / PD1 TX; PD3 transceiver silent control | Dedicated controller owner and recovery; no DMA1/2 stream |
| Primary IMU | SPI1 PA5/PA6/PD7; CS PC15, DRDY PB2 | Bounded DMA transaction; dedicated bus lock |
| Secondary IMU | SPI4 PE12/PE13/PE14; V4 CS PC13 | Bounded transaction; keep legacy CS PE11 inactive |
| USB device | PA11 / PA12 | Keep CDC; maintenance-only MSC must not block control |
| microSD | SDMMC1 PC8–PC12 / PD2 | Correct DMA memory; logger/storage errors must not stop control |

These preserve the board's actual routed connections; arbitrary MCU alternate functions cannot move an onboard CAN transceiver or IMU. SPI3 breakout PB3/PB4/PB5 with external CS PD4/PE2 also remains physically free; it is currently disabled and is not included in the active DMA budget. SPI2/OSD is likewise not needed for this baseline.

## 3. ADC allocation

Use one ADC1 scan owner for these six channels. Populate only the measurements actually needed; use software-triggered scans initially, without borrowing a PWM/PPS timer.

| Signal | MCU / ADC1 channel | Recommended use / scaling |
| --- | --- | --- |
| Board battery sense | PC0 / 10 | Battery voltage, existing divider factor 11 |
| Curr connector signal | PC1 / 11 | Optional external current sensor; board has no built-in current sensor |
| VB2 | PA4 / 18 | Additional battery/servo-rail voltage; V4 divider factor **21**, not 11 |
| Cur2 | PA7 / 7 | Servo feedback 1, conditioned 0–3.3 V |
| RSSI | PC5 / 8 | Servo feedback 2, conditioned 0–3.3 V; forego analog RSSI |
| AirS | PC4 / 4 | Additional feedback; built-in factor-2 divider accommodates a 0–5 V signal |

The V4 manufacturer specifies VB2 sense up to 69 V and AirS up to 6.6 V; these are **sense-input limits, not the board power-input rating**. Verify actual hardware and calibrate before use. In particular, the generic upstream Matek hwdef has `HAL_BATT2_VOLT_SCALE 11.0`; blindly importing it is wrong for the V4 divider. [Manufacturer electrical data][matek]

Feedback requires an actual analog feedback wire/potentiometer; the ordinary servo PWM lead does not return position. Add appropriate input protection/division/buffering and grounding for the selected sensor voltage and cable. Configure acquisition time for the divider/source impedance; preserve raw readings, calibrated scale and measurement age. DMA, if used for scan batches, needs DMA-safe memory. ADC1/2 share hardware resources: one ADC1 service is simpler than competing independent clients.

Neither board `stm32_adc.c` nor board `stm32_pwm.c` exists, although `boards/fmuv6c/src/Makefile` conditionally names both. ADC is disabled; current steering PWM is custom direct-register code. Enabling CONFIG_ADC or CONFIG_PWM alone is therefore **not a completed integration**. Add board setup/registration against NuttX's existing H7 lower halves and one actuator service for the PWM banks.

## 4. Conflicts the board definition must reject

| Conflict | Status / disposition |
| --- | --- |
| S3 steering TIM2 versus PPS TIM5 | Present source-level conflict; boot guard exists but runtime starts can bypass it. Move PPS to PA8/TIM1; remove S3 from PPS capability in this profile. |
| R6 USART6 versus TIM3 capture | Intentional alternate use. Claim the pad and stop the other owner before switching; preferably select once at boot. |
| S1/S2 on TIM3 instead of TIM8 | Valid alternative pinmux but conflicts with PPM's timer base. Use ArduPilot's TIM8 mapping. |
| S3–S6 TIM2 PWM versus buzzer | Same timer period; PA15 also duplicates S3's CH1 output. Disable tone generation and keep PA15 out of timer AF. |
| TIM4 alternatives PB6–PB9 | Collide with external I2C1/UART4. Use PD12–PD15 only. |
| TIM15 template inputs PA2/PA3 | Not the selected S11/S12 pads. Explicitly select PE5/PE6 for PWM. |
| Generic TIM1 mappings PE8–PE14 | Can collide with UART7 and SPI4. PPS must configure only PA8 capture; reject unrelated template channels and LED output. |
| Generic user button PC13 | Actually V4 secondary IMU CS. Button support is currently disabled; never enable this template definition on Matek. |
| Template I2C4 PD12/PD13 | Steals S7/S8; currently disabled. Reject on this board. |
| USB-host template PC6/PC7 | Collides with RC UART. Retain device-only USB unless separately designed. |
| USB device OTG-ID PA10 | Confirmed initialization conflict with USART1 RX in the original active config. Pin-stage fix enables `CONFIG_OTG_ID_GPIO_DISABLE` for Matek, including incremental builds; compile-time guard rejects its absence. |
| FMUv6C UART5 PC12/PD2 or PPS PC9 | Collides with Matek SDMMC1; keep board-specific implementations separated. |
| FMUv6C USART1 TX PB6 / USART2 RX PA3 / SPI1 MOSI PA7 | Steals Matek I2C1 / expansion PWM / feedback ADC. Retain Matek's PA9 / PD6 / PD7 overrides. |
| Generic ADC lists PB1, PA2/3, PA5/6 | Can steal PWM and IMU SPI pins. Generate only the six actual board ADC channels. |

Disabled template definitions are **latent hazards, not evidence those collisions are happening today**. Likewise, all timer Kconfig options being off does not mean timers are unused: current direct-register owners operate TIM2/3/5/6 outside the native drivers. Include them in allocation checks. Do not copy ArduPilot's TIM12 OS tick into NuttX: it is unnecessary and TIM12/13/14 have shared interrupt vectors with TIM8 functions on this MCU.

## 5. Boot, OS and memory contracts

1. **Build identity:** separate per-board definitions/configurations; generate GPIO, clock, timer, DMA, ADC scale and serial-role tables from one source. Embed config/patch hashes. Compile-time checks cover pins, whole-timer ownership, shared IRQs, clock domains and legal board features—not pins alone.
2. **Reset and clocks:** verify bootloader/VTOR handoff; safe inactive CS/output/transceiver states; complete clock transitions and verify actual kernel clocks. Keep NuttX SysTick. Do not change clocks simply to match ArduPilot numbers.
3. **Memory and DMA:** create explicit DMA-capable pools, alignment/cache-coherency rules and an AXI-compatible SDMMC1 buffer path. Do not assume the general heap is valid for every DMA engine. No allocation or blocking work in peripheral ISRs.
4. **Drivers:** claim resources, initialize with bounded deadlines, then publish ready/degraded/failed states. One owner per bus/controller and PWM timer bank; start/stop/recovery touches only owned resources. PPS is a client of the time service.
5. **Services:** start fixed RC, CAN, sensing and actuator services from explicit readiness dependencies. Optional GPS, PPS, barometer, external I2C, SD or USB failures must not prevent unrelated services from starting. Gate actuation on required command/safety inputs, not the presence of every optional sensor.
6. **Runtime:** per-bus PI-capable mutexes, explicit lock order, bounded queues and finite transfers. Avoid putting potentially blocking peripheral operations on a shared high-priority worker. Specify IRQ/task priorities after measuring worst-case latency; higher priority is not a substitute for bounded work. Bound fault-generated IRQ load too.
7. **Supervision:** feed watchdog based on required-service progress; expose stale data, overruns, recovery counts, maximum ISR latency and stack/heap watermarks. Flash erase/save and MSC are controlled maintenance operations.

Current active DMA demand is two SPI1 streams on DMA1 and two SPI4 plus USART2/UART4 RX on DMA2: **2/8 + 4/8 streams**. Adding one ADC1 DMA stream on DMA1 gives **3/8 + 4/8**. Ordinary PWM, PPS and FDCAN consume none of those streams. SDMMC1 uses its own IDMA. This is a capacity estimate, not a fixed stream allocation: NuttX allocates DMAMUX streams dynamically, and future devices need admission checks. Stream sharing is unnecessary for the present workload.

NuttX here is a flat, shared-memory RTOS configuration: separate tasks do not provide process-level memory isolation. Nor can pin allocation isolate a brownout. Use a separately rated servo supply with appropriate protection and common signal ground; do not assume the FC's 5 V rail can power eight servos. MCU reset, shared rails and shared timer-bank failures remain common failure domains.

## 6. What to reuse, and what pin changes cannot fix

| Area | NuttX-compatible action aligned with the references |
| --- | --- |
| CAN | Adapt ArduPilot H7 `CANFDIface` bus-off recovery, completion and expired-TX cancellation semantics to the NuttX controller owner. Latest actuation commands must supersede stale queued commands. Keep VESC protocol separate; test its independent timeout. Not a wholesale ChibiOS driver copy. |
| SBUS | Reuse the AP protocol parser where practical, with NuttX transport preserving frame gaps/errors. Fix setup-error handling and explicit failsafe versus frame-loss/last-good age. Fixed protocol, no autodetection needed. |
| PPM | Filter input, reset on overcapture, require expected channel count and stable reacquisition. AP's minimum-count/gap decoder alone does not prevent every missing-edge channel shift. Prefer serial RC when practical. |
| SPI/I2C/sensors | Retain native NuttX controller drivers but add finite DMA waits, local abort/recovery and checked sensor registers, as AP does. External I2C recovery must not reset the barometer bus or hold a global lock. |
| Time | Retain a monotonic hardware clock, separate UTC mapping and source quality. Fix the reproduced undamped rate/phase loop and observation-epoch/TX-lock timestamp biases. AP_RTC is not a drop-in precision PPS servo. Moving a PPS wire does not repair this algorithm. |
| PWM/ADC | Reuse NuttX H7 lower halves; board registration and actuator/analog services supply safe defaults, timeout policy, scaling and ownership. ArduPilot provides the bank/resource and ADC accumulation model, not binary-compatible drivers. |

RC failure should invalidate commands and invoke the chosen safety policy; it must **not tear down CAN**. CAN should remain alive for neutral commands, recovery and diagnostics. Conversely, losing CAN cannot freeze RC decoding. That intentional safety dependency is different from accidental driver coupling.

## 7. Implementation order and acceptance

1. Introduce the board manifest and reject conflicting profiles; split generic FMUv6C template resources from Matek. Preserve Pixhawk 6C's independent configuration.
2. Implement PWM on S1–S8, retaining S3/S4 steering; move PPS wiring/code to PA8/TIM1 and leave TIM5 exclusively owned by the time service. Keep output parameter/label mappings explicit. With actuators disconnected, scope every output through boot/start/stop/reset and PPS/RC fault injection. Prove period/polarity, capture rollover handling and no unintended pulses.
3. Repair CAN recovery/stale-TX and RC framing/failsafe; disconnect/reconnect the CAN device, inject RC noise/missing edges, and verify neither driver stops the other.
4. Repair clock estimation and test PPS loss/noise/reacquisition, serial-sync interaction and operation beyond the 32-bit timer wrap. Track TIM5-versus-OS time and UTC residual separately.
5. Add ADC and external I2C recovery; calibrate known voltages, short/stall the expansion bus through a safe fixture, and verify IMU/CAN/RC/PWM continue within measured timing budgets.
6. Stress USB/SD logging and sensor DMA aborts together; measure worst-case blackout, queue ages, memory/stack headroom and watchdog behavior. Repeat on Pixhawk 6C before accepting shared-code changes.

Source checks performed for this plan: independently checked the baseline and revised pin allocations against ArduPilot's STM32H743 alternate-function/ADC map, including TIM2 on PA0–PA3 and TIM1 capture on PA8; no duplicate pins in the proposed active/expansion assignment. Checked PPM alternatives and NuttX TIM8 complementary/TIM15 support. Pin-stage verification adds actual-NuttX-header compile tests, negative configuration tests and a Matek firmware build. Incremental-link inspection exposed a retained legacy PPS archive member; board archive generation now recreates the archive so removed drivers cannot remain selected. No flash or bench qualification has been performed.

## Sources

- [Matek V4 physical routing and electrical data][matek]. Important: board-specific divider values take precedence over a generic family default.
- [Pinned ArduPilot Matek hwdef][ap-hwdef] and [MCU alternate-function/ADC map][ap-map].
- Local `boards/fmuv6c/src/matekh743_io.c`, `fmuv6c_pps.c`, `fmuv6c_imu_time.c`, `fmuv6c.h`, `Makefile`; `boards/fmuv6c/include/board.h`; `apps/serial/serial.c`; `apps/param/param.c`; active `deps/nuttx/.config`.
- NuttX `arch/arm/src/stm32h7/{stm32_pwm.c,stm32_pwm.h,stm32_adc.c,stm32_capture.h,Kconfig}` and `hardware/stm32h7x3xx_pinmap.h`; revision recorded in the earlier audit.
- Pinned AP `AP_HAL_ChibiOS/{CANFDIface.cpp,UARTDriver.cpp,SPIDevice.cpp,I2CDevice.cpp,AnalogIn.cpp}`, `AP_RCProtocol` and `AP_RTC`; detailed references and reproduced defects in the earlier audit.

[matek]: https://www.mateksys.com/?portfolio=h743-slim-v4
[ap-hwdef]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/hwdef/MatekH743/hwdef.dat
[ap-map]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/hwdef/scripts/STM32H743xx.py
