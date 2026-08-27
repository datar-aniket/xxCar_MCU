#!/usr/bin/env python3
"""Host-side estimation of VESC_SPEED_K, the tachometer-to-velocity scale.

The fit itself is a through-origin least squares of the estimator's forward
speed against the tachometer rate.  Through the origin because zero counts
MUST mean zero velocity: a fitted intercept would be a standing speed offset
at a standstill, which is not a thing a wheel can measure, and allowing one
lets a biased sample set trade slope against it and land on neither.

    K = sum(v * c) / sum(c * c)

Everything else here is about which samples are allowed into that sum.  A
scale factor fitted through wheelspin, a scrubbing turn, or a stretch where
the estimator itself did not know how fast it was going is worse than no
scale factor, because it looks calibrated.

The estimator's velocity is an acceptable reference here only because it is
not derived from the wheels.  The zero-velocity update is deliberately
K-independent - it asserts zero, never a speed - so nothing in this fit is
circular today.  If wheel speed is ever fused as a velocity measurement, this
tool must take its reference from the external fix directly instead.
"""

from dataclasses import dataclass
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import comp_link  # noqa: E402

# Taken from the codec rather than restated here. A second copy of these bits
# would drift silently: every sample would simply stop qualifying, and the
# tool would report an empty run rather than a wrong one.
SRC_ESTIMATOR = comp_link.SRC_ESTIMATOR
SRC_VESC = comp_link.SRC_VESC
MSG_VEHICLE_STATE = comp_link.MSG_VEHICLE_STATE
decode_vehicle_state = comp_link.decode_vehicle_state

# EKF_SOLUTION_POSITION_HORIZ from apps/ekf3/ekf_core.h.
SOLUTION_POSITION_HORIZ = 1 << 5

# Below this the estimator's own speed is mostly noise and the tachometer is
# mostly quantisation, so the pair says nothing about their ratio.
MIN_SPEED_MS = 0.3

# A turning vehicle scrubs its wheels, and a differential drives them at
# different rates than the body travels.  Samples taken while turning hard
# measure the turn, not the scale.
MAX_YAW_RATE = 0.35

# Enough spread that the fit is a line and not a point.  Without this a
# dataset taken entirely at one speed produces a confident K that describes
# only that speed.
MIN_SPEED_SPAN_MS = 1.0
MIN_SAMPLES = 200


class FitError(ValueError):
    """The dataset cannot produce a scale factor worth storing."""

    def __init__(self, reason: str, **metrics):
        super().__init__(reason)
        self.reason = reason
        self.metrics = metrics


@dataclass
class WheelFit:
    k: float
    samples: int
    r_squared: float
    speed_span_ms: float
    residual_rms_ms: float

    def summary(self) -> str:
        return (f"VESC_SPEED_K = {self.k:.6g}   "
                f"({self.samples} samples, R^2 {self.r_squared:.4f}, "
                f"span {self.speed_span_ms:.2f} m/s, "
                f"residual {self.residual_rms_ms:.3f} m/s)")


def usable(sample: dict) -> bool:
    """Is this sample allowed into the fit?

    Each rejection below is a way of measuring something other than the scale
    factor, and every one of them still produces a number.
    """
    if not sample["source_valid"] & SRC_VESC:
        return False

    if not sample["source_valid"] & SRC_ESTIMATOR:
        return False

    # The estimator has to know its own speed, or it is not a reference.
    if not sample["solution_status"] & SOLUTION_POSITION_HORIZ:
        return False

    if abs(sample["angular_velocity"][2]) > MAX_YAW_RATE:
        return False

    if abs(sample["velocity"][0]) < MIN_SPEED_MS:
        return False

    return True


def fit(pairs) -> WheelFit:
    """pairs: iterable of (tachometer_rate, forward_speed_ms).

    The rate is whatever VESC_SPEED_K was set to at capture time multiplied
    into it, so calibrate with that parameter at 1.0 and the rate is raw
    counts per second.  Calibrating against an already-scaled reading gives a
    CORRECTION to the existing K, not the K itself, which is a good way to
    apply the same factor twice.
    """
    rows = [(float(c), float(v)) for c, v in pairs]

    if len(rows) < MIN_SAMPLES:
        raise FitError("not enough usable samples", samples=len(rows),
                       needed=MIN_SAMPLES)

    speeds = [v for _, v in rows]
    span = max(speeds) - min(speeds)

    if span < MIN_SPEED_SPAN_MS:
        raise FitError("speed range too narrow to fit a slope",
                       span=span, needed=MIN_SPEED_SPAN_MS)

    sum_cv = sum(c * v for c, v in rows)
    sum_cc = sum(c * c for c, _ in rows)

    if sum_cc <= 0.0:
        raise FitError("tachometer never moved", samples=len(rows))

    k = sum_cv / sum_cc

    # R^2 about zero, matching the through-origin model. Using the mean here
    # would flatter the fit: a dataset clustered away from zero scores well
    # against its own mean no matter how wrong the slope is.
    residual_sq = sum((v - k * c) ** 2 for c, v in rows)
    total_sq = sum(v * v for _, v in rows)
    r_squared = 1.0 - residual_sq / total_sq if total_sq > 0.0 else 0.0

    return WheelFit(k=k, samples=len(rows), r_squared=r_squared,
                    speed_span_ms=span,
                    residual_rms_ms=math.sqrt(residual_sq / len(rows)))


def collect(events, seconds: float, progress=None):
    """Gather usable samples from a comp_link event queue.

    `events` is the queue a comp_link.Link posts to: ("frame", (id, body,
    rx_us)) tuples with the occasional ("error", text). Taking the queue
    rather than the Link keeps this testable without a serial port.

    Returns (pairs, seen, used). Both counts are worth reporting: a run where
    almost nothing was used is a run that measured the rejection rules, and
    the operator needs to see that rather than be handed a fit over the
    handful of samples that got through.
    """
    import queue as queue_mod
    import time

    pairs = []
    seen = 0
    deadline = time.monotonic() + seconds

    while True:
        remaining = deadline - time.monotonic()

        if remaining <= 0.0:
            break

        try:
            kind, data = events.get(timeout=min(remaining, 0.2))
        except queue_mod.Empty:
            continue

        if kind != "frame":
            continue

        msg_id, body, _rx = data

        if msg_id != MSG_VEHICLE_STATE:
            continue

        sample = decode_vehicle_state(body)
        seen += 1

        if usable(sample):
            pairs.append((sample["motor_speed_ms"], sample["velocity"][0]))

        if progress is not None and seen % 200 == 0:
            progress(seen, len(pairs), remaining)

    return pairs, seen, len(pairs)


def main(argv=None):
    import argparse
    import queue

    parser = argparse.ArgumentParser(
        description="Estimate VESC_SPEED_K from a drive.")
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--seconds", type=float, default=60.0)
    args = parser.parse_args(argv)

    print("Set VESC_SPEED_K to 1.0 and reboot before running this, or the")
    print("result is a correction to the existing value rather than the")
    print("value itself.\n")
    print(f"Drive smoothly over a range of speeds for {args.seconds:.0f} s.")
    print("Straight lines only - turning scrubs the wheels and those samples")
    print("are discarded anyway.\n")

    events = queue.Queue()
    link = comp_link.Link(args.port, args.baud, events)
    link.start()

    def progress(seen, used, remaining):
        print(f"  {remaining:5.1f}s left   {seen} frames, {used} usable",
              end="\r", flush=True)

    try:
        pairs, seen, used = collect(events, args.seconds, progress)
    finally:
        link.close()

    print()

    if not pairs:
        print(f"No usable samples out of {seen} state frames. Check that")
        print("`ekf3 status` reports a healthy external fix and that")
        print("`vesc status` shows telemetry arriving.")
        return 1

    try:
        result = fit(pairs)
    except FitError as error:
        print(f"Fit refused: {error.reason}")

        for name, value in error.metrics.items():
            print(f"  {name}: {value}")

        print(f"  {used} usable of {seen} state frames")
        return 1

    print(result.summary())
    print()

    if result.r_squared < 0.9:
        print("R^2 is low. The wheel and the estimator disagree about shape,")
        print("not just scale - suspect wheelspin, a slipping belt, or an")
        print("external fix that was not tracking well during the run.\n")

    print(f"  param set VESC_SPEED_K {result.k:.6g}")
    print("  param save")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
