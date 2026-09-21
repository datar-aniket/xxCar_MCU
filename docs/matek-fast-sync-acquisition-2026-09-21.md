# Fast initial UTC acquisition

The previous implementation immediately established first-sync UTC phase,
but waited 5 s to measure rate, then slewed the accumulated phase at only
200 ppm (0.2 ms/s). With this board's approximately 2640 ppm rate error,
about 14 ms accumulated before rate acquisition: repayment took about 70 s.
That was an inappropriate steady-state policy during initial acquisition.

Changes:

- Initial phase observation remains immediate and explicitly provisional.
- First plausible rate fit within a 10 s interval applies rate **and phase**
  together. This one-time wire-clock step is bounded to +/-50 ms and works
  for either sign of drift. No MCU timer or internal estimator clock changes.
- Subsequent syncs keep the continuous, bounded-slew policy. Late/large first
  corrections also use slew; START/reconnect cannot repeatedly phase-step an
  already acquired clock. Status exposes acquisition and the applied step.
- Phase slew expires after its correction interval (at most 30 s). Both UTC
  conversion and its inverse use the same continuous two-segment mapping.
  A short acquisition correction cannot accidentally continue through the
  following 30 s tracking interval, or keep moving UTC during holdover.
- GUI/probe take six startup bursts with 1.2 s gaps, then use the normal 30 s
  cadence. This avoids letting a noisy initial two-point rate run for 30 s.
- Arrival age still includes the approximately 5–10 ms sensor/estimator/link
  delay measured previously. Correct synchronization cannot promise every
  individual arriving sample is younger than 10 ms.

Regression coverage: immediate phase alignment for both +/-2600 ppm at the
first rate correction, UTC inverse, no repeated phase steps, late acquisition,
host discontinuities, bounds/overflow checks, GUI burst cadence, selected
exchange epoch and timestamp capture under the TX lock.

The raw board-clock error reported in the preceding follow-up is unchanged;
this improves UTC acquisition, not physical PWM/oscillator calibration.

## Separating startup effects on this board

The connected board currently uses a 100 Hz downlink and 30 ms EKF horizon
(different from the earlier 20 Hz/zero-horizon test). During cold-boot EKF
alignment, `ekf_core_output_predict()` returns the delayed sample without
replaying the horizon because the state is not initialized. Once initialized,
it predicts forward and publishes a current sample. Observed arrival age was
35–45 ms during that alignment, then approximately 7 ms. This is real sample
age and must not be erased by shifting UTC or relabeling sample timestamps.

The headless probe now requests one reply after END before starting each
measurement window. This ordering barrier drains queued pre-sync/RTC-seeded
frames from the measurement window. It proves END was processed; status is
still required to confirm the clock accepted the observation.

## Verified on the connected Matek

Final firmware uploaded and bootloader verification passed. Fresh board-clock
acquisition was measured after EKF alignment, to isolate clock convergence
from the 30 ms estimator startup delay. No saved parameters were changed and
no arm/actuator command was issued.

At 100 Hz downlink:

| Measurement window | Mean arrival age | Range |
| --- | ---: | ---: |
| After first sync (~1.2 s from probe start) | 8.45 ms | 4.91–12.47 ms |
| After first rate correction (~2.8 s) | 6.82 ms | 4.26–10.26 ms |
| Next three acquisition windows | 6.84 / 6.84 / 6.99 ms | approximately 4.4–11 ms |
| ~9–39 s (after the rapid burst sequence) | 7.33 ms | 4.43–11.83 ms |
| ~40–45 s | 7.63 ms | 5.11–11.45 ms |

The initial window contains unknown-rate drift; the first rate correction
immediately removes its accumulated phase. No minute-long repayment is needed.
Individual arrival ages can still exceed 10 ms because arrival age includes
sampling, estimator and transport delay, not only clock error.

Final status: seven accepted updates, zero rate/step resets, startup phase
correction +4.416 ms, last phase residual +0.839 ms. After the finite slew
expired, applied rate and base rate both read +2645.070 ppm. Router remained
disarmed with zero arm attempts. TIM5 physical frequency error is unchanged.

Both firmware targets build, and clock (including UBSan), C/Python protocol,
GUI scheduling/TX-lock, and GUI control-safety regressions pass. The existing
unrelated MAVLink uninitialized-counter compiler warning remains.
