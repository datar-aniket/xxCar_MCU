#!/usr/bin/env python3
"""Headless clock/latency probe for the xxCar companion serial link."""

import argparse
import queue
import statistics
import time

import comp_link


def wait_reply(events, utc, host_tx_us, timeout_s=0.25):
    deadline = time.monotonic() + timeout_s
    vehicle_ages = []

    while time.monotonic() < deadline:
        try:
            kind, payload = events.get(timeout=deadline - time.monotonic())
        except queue.Empty:
            break

        if kind != "frame":
            continue

        msg_id, body, rx_us = payload
        if msg_id == comp_link.MSG_TIMESYNC_REP:
            reply = comp_link.decode_timesync_rep(body)
            if reply["host_tx_us"] == host_tx_us:
                offset, trip = comp_link.timesync_solve(
                    reply, utc.to_utc(rx_us))
                if 0 <= trip <= 20000:
                    return (offset, trip, (reply['board_rx_us'] +
                                           reply['board_tx_us']) // 2), vehicle_ages
        elif msg_id == comp_link.MSG_VEHICLE_STATE:
            state = comp_link.decode_vehicle_state(body)
            vehicle_ages.append(
                (utc.to_utc(rx_us) - state["timestamp_us"]) / 1000.0)

    return None, vehicle_ages


def synchronize(link, events, utc, count=10, legacy=False):
    samples = []
    ages = []
    # One extra exchange after END is an ordering barrier. Otherwise the
    # first measurement window can include queued, pre-sync RTC timestamps.
    link.send(comp_link.encode_timesync_start(count + 1))

    for _ in range(count):
        host_tx_us = link.send_timesync(utc)
        if host_tx_us is None:
            continue
        result, observed_ages = wait_reply(events, utc, host_tx_us)
        ages.extend(observed_ages)
        if result is not None:
            samples.append(result)
        time.sleep(0.04)

    if len(samples) < 3:
        link.send(comp_link.encode_timesync_end(0, 0, 0))
        raise RuntimeError("fewer than three usable TIMESYNC replies")

    offset, trip, sample_us = min(samples, key=lambda sample: sample[1])
    link.send(comp_link.encode_timesync_end(-offset, trip, len(samples),
                                            None if legacy else sample_us))
    barrier_tx = link.send_timesync(utc)
    if barrier_tx is None:
        raise RuntimeError('post-END barrier write failed')
    barrier, transitional_ages = wait_reply(events, utc, barrier_tx)
    ages.extend(transitional_ages)
    if barrier is None:
        raise RuntimeError('no post-END barrier reply')
    # A reply proves END was processed, not that a malformed END was accepted;
    # companion status still provides the authoritative acquisition counters.
    return offset, trip, len(samples), ages, sample_us


def collect_ages(events, utc, seconds):
    deadline = time.monotonic() + seconds
    values = []
    times = []
    unsynced = 0

    while time.monotonic() < deadline:
        try:
            kind, payload = events.get(timeout=deadline - time.monotonic())
        except queue.Empty:
            break

        if kind != "frame":
            continue

        msg_id, body, rx_us = payload
        if msg_id != comp_link.MSG_VEHICLE_STATE:
            continue

        state = comp_link.decode_vehicle_state(body)
        # Exclude pre-sync frames, not delayed UTC frames. Report exclusions
        # so an entirely unsynchronized stream cannot look like a pass.
        if state['timestamp_us'] < 1000000000000000:
            unsynced += 1
            continue
        values.append((utc.to_utc(rx_us) - state["timestamp_us"]) / 1000.0)
        times.append(rx_us / 1.0e6)

    if len(values) < 2:
        return None

    x_mean = statistics.fmean(times)
    y_mean = statistics.fmean(values)
    denominator = sum((x - x_mean) ** 2 for x in times)
    slope = (sum((x - x_mean) * (y - y_mean)
                 for x, y in zip(times, values)) / denominator
             if denominator > 0.0 else 0.0)
    return {
        "count": len(values),
        "min_ms": min(values),
        "mean_ms": y_mean,
        "max_ms": max(values),
        "slope_ms_s": slope,
        "pre_sync_discarded": unsynced,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/v4w_io")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--duration", type=float, default=120)
    parser.add_argument("--interval", type=float, default=30)
    parser.add_argument("--legacy", action="store_true",
                        help="use END v1 for older firmware")
    args = parser.parse_args()
    if args.duration < 5 or args.interval < 2:
        parser.error('duration >= 5 and interval >= 2 required')

    events = queue.Queue()
    utc = comp_link.UtcClock()
    link = comp_link.Link(args.port, args.baud, events)
    link.start()

    try:
        started = time.monotonic()
        previous = None
        completed = 0
        while time.monotonic() - started < args.duration:
            current = synchronize(link, events, utc, legacy=args.legacy)
            completed += 1
            at = time.monotonic()
            print(f't={at-started:.1f}s offset={current[0]} us '
                  f'trip={current[1]} us samples={current[2]}', flush=True)
            if previous:
                # Use selected exchange epochs, not END arrival. Different
                # min-RTT samples within a burst otherwise fake rate noise.
                offset_delta = current[0] - previous[0]
                board_delta = current[4] - previous[1]
                host_delta = board_delta - offset_delta
                rate = offset_delta * 1e6 / host_delta
                print(f'raw board-minus-host rate={rate:+.3f} ppm', flush=True)
            window = min(comp_link.TIMESYNC_ACQUIRE_INTERVAL_S
                         if completed < comp_link.TIMESYNC_ACQUIRE_BURSTS
                         else args.interval,
                         max(.1, args.duration - (at - started)))
            previous = current[0], current[4]
            result = collect_ages(events, utc, window)
            if result is None:
                print('No UTC vehicle-state frames (sync not accepted?)', flush=True)
            else:
                print(f"n={result['count']} age "
                      f"min/mean/max={result['min_ms']:+.3f}/"
                      f"{result['mean_ms']:+.3f}/"
                      f"{result['max_ms']:+.3f} ms "
                      f"slope={result['slope_ms_s']:+.3f} ms/s "
                      f"pre_sync_discarded={result['pre_sync_discarded']}", flush=True)
    finally:
        link.close()
        link.join(timeout=1.0)


if __name__ == "__main__":
    main()
