#!/usr/bin/env python3
"""Host-side checks for the tachometer-to-velocity scale fit.

A scale factor fitted through wheelspin, a scrubbing turn, or a stretch where
the estimator did not know its own speed is worse than no scale factor,
because it looks calibrated. Most of what is tested here is the refusal to
produce one.
"""

import pathlib
import queue
import re
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))

import comp_link  # noqa: E402
import wheel_cal  # noqa: E402


FAILURES = []


def check(condition, message):
    if not condition:
        FAILURES.append(message)


def sample(vx=2.0, cps=2000.0, yaw_rate=0.0,
           source=wheel_cal.SRC_ESTIMATOR | wheel_cal.SRC_VESC,
           status=wheel_cal.SOLUTION_POSITION_HORIZ):
    return {
        "velocity": (vx, 0.0, 0.0),
        "angular_velocity": (0.0, 0.0, yaw_rate),
        "motor_speed_ms": cps,
        "source_valid": source,
        "solution_status": status,
    }


def test_recovers_a_known_scale():
    true_k = 0.00125
    pairs = [(c, true_k * c) for c in range(200, 4000, 5)]
    result = wheel_cal.fit(pairs)

    check(abs(result.k - true_k) < 1e-9,
          f"exact data must recover the scale, got {result.k}")
    check(result.r_squared > 0.999, "exact data must fit perfectly")


def test_survives_noise():
    true_k = 0.002
    rng = 12345
    pairs = []

    for c in range(200, 4000, 5):
        # deterministic pseudo-noise, no numpy dependency
        rng = (1103515245 * rng + 12345) % (1 << 31)
        noise = (rng / (1 << 31) - 0.5) * 0.2
        pairs.append((c, true_k * c + noise))

    result = wheel_cal.fit(pairs)
    check(abs(result.k - true_k) < 2e-5,
          f"noisy data must stay near the scale, got {result.k}")


def test_fit_is_through_the_origin():
    """A standing offset must not be absorbed by a fitted intercept.

    Zero counts is zero speed - a wheel cannot report motion while stopped -
    so the model has no intercept to give away. Given one, a dataset offset
    from the origin trades slope against it and lands on neither, and the
    give-away is that the recovered slope comes back CLEAN. So this asserts
    the through-origin value exactly rather than that the answer is off:
    "off" is satisfied by rounding error, and was.
    """
    true_k = 0.001
    offset = 0.5
    pairs = [(float(c), true_k * c + offset) for c in range(1000, 5000, 5)]

    sum_cv = sum(c * v for c, v in pairs)
    sum_cc = sum(c * c for c, _ in pairs)
    expected_k = sum_cv / sum_cc

    result = wheel_cal.fit(pairs)

    check(abs(result.k - expected_k) < 1e-12,
          f"the fit must be through the origin: {result.k} vs {expected_k}")
    check(result.k > true_k + 1e-6,
          "an offset dataset must show the offset in the slope")
    check(result.residual_rms_ms > 0.05,
          "an offset dataset must leave a visible residual")


def test_r_squared_is_measured_about_zero():
    """About zero, matching the model - not about the mean.

    A dataset clustered away from zero scores well against its own mean
    however wrong the slope is, so an R^2 about the mean would report a
    healthy fit for a scale factor that is plainly off.
    """
    true_k = 0.001
    pairs = [(float(c), true_k * c + 0.5) for c in range(1000, 5000, 5)]

    sum_cv = sum(c * v for c, v in pairs)
    sum_cc = sum(c * c for c, _ in pairs)
    k = sum_cv / sum_cc

    residual_sq = sum((v - k * c) ** 2 for c, v in pairs)
    about_zero = 1.0 - residual_sq / sum(v * v for _, v in pairs)

    mean_v = sum(v for _, v in pairs) / len(pairs)
    about_mean = 1.0 - residual_sq / sum((v - mean_v) ** 2 for _, v in pairs)

    # If these agreed the test would prove nothing.
    check(abs(about_zero - about_mean) > 0.01,
          "the two conventions must differ on this dataset")

    result = wheel_cal.fit(pairs)
    check(abs(result.r_squared - about_zero) < 1e-9,
          f"R^2 must be about zero: {result.r_squared} vs {about_zero}")


def test_refuses_a_narrow_speed_range():
    pairs = [(2000.0 + i * 0.01, 2.0) for i in range(500)]

    try:
        wheel_cal.fit(pairs)
        FAILURES.append("a single-speed dataset must be refused")
    except wheel_cal.FitError as error:
        check("range" in error.reason, f"unexpected reason: {error.reason}")


def test_refuses_too_few_samples():
    pairs = [(c, 0.001 * c) for c in range(200, 400, 5)]

    try:
        wheel_cal.fit(pairs)
        FAILURES.append("a short dataset must be refused")
    except wheel_cal.FitError as error:
        check("samples" in error.reason, f"unexpected reason: {error.reason}")


def test_refuses_a_dead_tachometer():
    """Moving, with the wheel reading zero throughout.

    The span guard does not cover this - the vehicle did cover a range of
    speeds - and without its own guard the fit divides by zero. A dead
    tachometer on a live VESC reports exactly this: the source flag is set,
    the counts are not.
    """
    pairs = [(0.0, 0.5 + i * 0.005) for i in range(500)]

    try:
        wheel_cal.fit(pairs)
        FAILURES.append("a dataset with no tachometer motion must be refused")
    except ZeroDivisionError:
        FAILURES.append("a dead tachometer must be refused, not divided by")
    except wheel_cal.FitError as error:
        check("moved" in error.reason, f"unexpected reason: {error.reason}")


def test_refuses_a_stationary_dataset():
    pairs = [(0.0, 0.0)] * 500

    try:
        wheel_cal.fit(pairs)
        FAILURES.append("a stationary dataset must be refused")
    except wheel_cal.FitError as error:
        check("moved" in error.reason or "range" in error.reason,
              f"unexpected reason: {error.reason}")


def test_sample_rejection_rules():
    check(wheel_cal.usable(sample()), "a clean sample must be used")

    check(not wheel_cal.usable(sample(source=wheel_cal.SRC_ESTIMATOR)),
          "no VESC data means no wheel reading to fit")

    check(not wheel_cal.usable(sample(source=wheel_cal.SRC_VESC)),
          "no estimator means no velocity reference")

    check(not wheel_cal.usable(sample(status=0)),
          "without POSITION_HORIZ the estimator does not know its own speed")

    check(not wheel_cal.usable(sample(yaw_rate=1.0)),
          "a turning vehicle scrubs its wheels")

    check(not wheel_cal.usable(sample(vx=0.05)),
          "at a crawl both signals are mostly noise")

    # The boundaries themselves, so a threshold change is deliberate.
    check(wheel_cal.usable(sample(yaw_rate=wheel_cal.MAX_YAW_RATE - 0.01)),
          "just inside the yaw limit must be accepted")
    check(not wheel_cal.usable(sample(yaw_rate=wheel_cal.MAX_YAW_RATE + 0.01)),
          "just outside the yaw limit must be refused")


def encode_state(sample):
    """Frame a sample the way the board would, so collect() is tested against
    real bytes rather than the dict it happens to want."""
    payload = comp_link.VEHICLE_STATE.pack(
        1000, 0.0, 0.0, 0.0,               # timestamp, position
        1.0, 0.0, 0.0, 0.0,                # quaternion
        *sample["velocity"],
        *sample["angular_velocity"],
        float("nan"),                      # side slip
        0.0, 0.0, 0.0,                     # accel
        0.0,                               # wheel torque
        0.0,                               # steering angle
        sample["motor_speed_ms"],
        sample["solution_status"],
        0,                                 # reset counter
        sample["source_valid"],
        0)                                  # packed RC/control status
    return ("frame", (comp_link.MSG_VEHICLE_STATE, payload, 0))


def test_collect_counts_and_filters():
    events = queue.Queue()

    for _ in range(10):
        events.put(encode_state(sample(vx=2.0, cps=1600.0)))

    for _ in range(4):
        events.put(encode_state(sample(yaw_rate=1.0)))

    # Traffic collect() must ignore rather than choke on.
    events.put(("frame", (comp_link.MSG_TIMESYNC_REQ, b"\x00" * 8, 0)))
    events.put(("error", "link lost"))

    pairs, seen, used = wheel_cal.collect(events, 0.5)

    check(seen == 14, f"every state frame must be counted, got {seen}")
    check(used == 10, f"only clean samples may be used, got {used}")
    check(all(abs(c - 1600.0) < 1e-3 for c, _ in pairs),
          "collect must pair the tachometer rate with the speed")
    check(all(abs(v - 2.0) < 1e-6 for _, v in pairs),
          "collect must take the forward component of body velocity")


def test_solution_bit_matches_the_firmware():
    """The one constant still restated rather than imported.

    If it drifts, every sample stops qualifying and the tool reports an empty
    run - which reads as a bad drive rather than a stale constant.
    """
    header = (REPO / "apps" / "ekf3" / "ekf_core.h").read_text()
    match = re.search(r"EKF_SOLUTION_POSITION_HORIZ\s+\(1u\s*<<\s*(\d+)\)",
                      header)

    check(match is not None,
          "EKF_SOLUTION_POSITION_HORIZ not found in ekf_core.h")

    if match:
        check(wheel_cal.SOLUTION_POSITION_HORIZ == 1 << int(match.group(1)),
              "SOLUTION_POSITION_HORIZ has drifted from the firmware")


def main():
    for name, function in sorted(globals().items()):
        if name.startswith("test_") and callable(function):
            function()

    if FAILURES:
        for failure in FAILURES:
            print(f"FAIL {failure}")

        print(f"{len(FAILURES)} failure(s)")
        return 1

    print("wheel_cal: scale recovery, origin constraint and rejections - OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
