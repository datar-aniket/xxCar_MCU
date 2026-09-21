# Matek PWM phase and jitter qualification

## Contract and current scope

User-selected policy: **independent banks, aligned rising edges within each
bank**. Positive servo pulses, idle low; no timer synchronization links between
banks. Changing one channel must not reset its bank counter, change its period,
or change another channel's pulse width. Channels in a bank share frame rate.

The original two-channel baseline below is superseded by the
[S1-S8 implementation and bench follow-up](matek-pwm-clock-followup-2026-09-21.md).
All eight baseline channels are implemented; expansion S9-S12 and PPS remain
deferred. No claim of measured electrical jitter is made.

## Implemented for S3/S4

- Hardware upcounting PWM1, non-inverted outputs, exact integer 1 MHz timer
  division. The frame period is rounded once to an integer microsecond, not
  alternated between adjacent periods. Absolute timing accuracy still depends
  on the board clock; resolution is not accuracy.
- Dedicated TIM2 reset/configuration while the pads are low. Load shadows
  while disconnected, park CNT at ARR (inactive PWM level), connect both pads,
  then start the counter. The first wrap starts a complete pulse on both pins.
- CCR preloads and a short UDIS-protected paired write. CPU interrupt masking
  alone does not prevent hardware rollover between writes. With UDIS, a frame
  uses the old pair or the new pair, never a mixture. A coincident boundary can
  defer command application by one frame; it does not stretch the pulse.
- No per-frame IRQ, DMA or software edge scheduling. Runtime updates do not
  write CNT/ARR, force UG, or stop/start CEN. Timer hardware continues during
  application/IRQ load; command-arrival latency is a separate measurement.
- A running bank rejects rate changes with `-EBUSY`. Health checks include
  pin alternate functions, polarity/preload, period/prescaler, enable state
  and the APB1 prescaler. These are diagnostics, not detection of every
  possible clock/electrical fault.
- Watchdog neutral uses the same paired preload path. Failure to schedule a
  watchdog queues neutral rather than a new non-neutral command. The NuttX
  watchdog is software: include its dispatch latency and the next frame in
  the timeout budget. It cannot guarantee neutral during an interrupt/CPU
  stall. Independent actuator-side failsafes remain necessary where required.

This follows ST's documented preload/update mechanism, not a claim that
ArduPilot's complete RCOutput driver was transplanted. [ST RM0433][rm],
[timer application note][an]

## S1/S2 complementary-output requirement for the next driver

Use TIM8 CH2N/CH3N with PWM1 and only the N outputs enabled: main `CCxE=0`,
`CCxNE=1`, `CCxNP=0`. In this configuration the N output follows OCxREF; it
is not the inverted member of an enabled output pair. Enabling both main and
N outputs changes that behavior. The board header rejects main-output,
inverted-polarity and non-PWM1 configurations for these channels.

This matches the generated ArduPilot Matek `PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH`
selection and its ChibiOS CCER programming. Before activation, also configure
MOE, low idle/off state, zero unwanted dead time, update/repetition settings,
and the same disconnected startup/paired-update sequence. Do not assume
native lower-half availability proves these integration details. [ST AN4013,
section 4.1][an], [ArduPilot Matek hardware definition][ap]

## Automated evidence

`bash tools/test-matek-pwm.sh` executes the actual production driver with
mocked MMIO/GPIO/critical-section/watchdog boundaries and real NuttX register
definitions. Its counter continues while interrupts are masked.

- Startup from inherited timer state; failures at each GPIO setup step.
- Complete positive pulses and aligned rising edges; unchanged frame periods.
- Rollover before, between and after paired writes, including watchdog writes.
- 2,000 randomized command updates across 25, 50, 333 and 400 Hz; modeled pulse
  widths exactly equal the active CCR values and periods remain constant.
- Invalid inputs, unexpected pin mux/polarity, watchdog-start failure and
  runtime rate-change rejection.
- ASan/UBSan run and a negative control which removes effective UDIS gating;
  the rollover test must fail for that unsafe variant.

`bash tools/test-matek-pins.sh` checks allocation, alternate functions and
negative configuration guards. The Matek target build checks actual embedded
compilation. The model is deliberately limited: it does not simulate PLL
noise, electrical rise times, complete silicon behavior or real IRQ latency.

## Required bench acceptance (not yet executed)

Use a logic analyzer/oscilloscope with resolution comfortably below 1 us;
capture at the board pads first, with servos disconnected and common ground.

1. **Boot/restart:** repeated cold/warm reset and service start. No spurious
   high pulse in application startup; first commanded pulse has full width.
   Record any bootloader-time activity separately (application code cannot
   control the pad before handoff).
2. **Fixed commands:** capture 900, 1500 and 2100 us pulses at supported rates
   for at least 10 minutes. Record min/max width, frame period and within-bank
   rising-edge skew. Proposed acceptance target: <=1 us peak-to-peak width
   jitter and <=1 us within-bank skew, accounting for instrument uncertainty.
   Report constant oscillator scale error separately from jitter.
3. **Concurrent load:** repeat with both IMUs, RC, CAN, companion traffic,
   USB and SD logging; exercise reconnect/fault paths. No runt, extra or
   missing output pulses. Report maxima, not just averages.
4. **Command steps:** change both steering widths around frame boundaries;
   verify coherent old/new pairs and unchanged phase/period. Distinguish
   intentional width changes from fixed-command jitter.
5. **Timeout/failure:** stop command updates; measure last command to first
   complete neutral frame. At 50 Hz expect the 200 ms software timeout plus
   dispatch latency and up to a 20 ms frame (plus the short commit interval).
   Test watchdog setup failure separately. Do not claim a hard latency bound
   without measuring/limiting worst-case IRQ masking.
6. **Powered actuators:** only after unloaded checks pass, repeat with a
   correctly rated servo supply, protection and shared signal ground. Check
   voltage dips, ground bounce and signal thresholds under mechanical load.

Repeat these gates for TIM8/TIM4 when the eight-channel driver is integrated.
Cross-bank phase offset is intentionally unspecified, not a failure.

[rm]: https://www.st.com.cn/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
[an]: https://www.st.com/content/ccc/resource/technical/document/application_note/54/0f/67/eb/47/34/45/40/DM00042534.pdf/files/DM00042534.pdf/jcr:content/translations/en.DM00042534.pdf
[ap]: https://github.com/ArduPilot/ardupilot/blob/663fc09bb25b84a756d8585e4f6d9266a0ab53ae/libraries/AP_HAL_ChibiOS/hwdef/MatekH743/hwdef.dat
