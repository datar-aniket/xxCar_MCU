# EKF replay and health logging

With `LOG_EKF=1`, each new ULog part now contains the complete parameter set
and all estimator-facing topics. The high-rate `estimator_diag` remains the
record of individual propagation/fusion decisions. `estimator_health` adds a
10 Hz summary of timing ages, output replay depth, covariance integrity,
overflows, reset/rejection counts, and independent-IMU monitor disagreement.

Every rollover part is self-contained. Parameter differences between parts
are treated as a reproducibility warning rather than silently combining runs
made with different tuning.

## Generate a report

```sh
python3 tools/ekf_replay.py log_012_00.ulg \
  --report log_012_report.json
```

For a split session, pass all parts. Their command-line order does not affect
the result:

```sh
python3 tools/ekf_replay.py log_012_02.ulg log_012_00.ulg log_012_01.ulg \
  --report log_012_report.json \
  --events log_012_inputs.jsonl
```

The JSONL stream is deterministically ordered by sample timestamp, topic
priority, log part, and original row. It contains the acquired EKF inputs,
not scheduler arrival order. This makes it the stable input artifact for a
host-side C-core runner and lets two firmware revisions be compared against
the same measurements.

`--strict` exits nonzero if core topics are missing, rollover parameters
differ, covariance-health flags fail, a queue overflow is recorded, or an EKF
publication error is present. Older logs remain analyzable, but warn that they
have no embedded parameter snapshot or the new health topic.

## Important health fields

- `imu_age_us`: TIM5 now minus the newest IMU sample. This exposes scheduling
  delay without contaminating the propagation time base.
- `output_age_us`: TIM5 now minus the state output horizon.
- `extnav_source_age_us`: receive timestamp minus the corrected source
  timestamp; use its distribution to tune external-nav delay and jitter.
- `output_replay_samples`: number of IMU deltas replayed from the delayed EKF
  horizon to the current output.
- `covariance_diag_min`, `covariance_diag_max`, and
  `covariance_asymmetry_max`: direct checks of the full 15-state covariance.
- `extnav_reject_run` and `extnav_test_ratio`: distinguish an isolated noisy
  pose from persistent disagreement or a frame/extrinsic error.

The generated report separately measures stationary residual acceleration and
velocity only where both wheel-stop and IMU-stationary gates are true. That
prevents braking and wheel slip from being mislabeled as bench noise.
