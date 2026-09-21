#!/usr/bin/env python3
"""Passive MatekH743 clock comparison via NSH xd (read-only register dumps).

Close picocom first. Does not arm, start/stop drivers, or change parameters.
USB received SOF frame numbers provide a reference independent of MCU HSE.
Sequential NSH reads introduce jitter: use regression, not individual samples.
This is a screening measurement, not a calibrated frequency counter.
"""
import argparse
import re
import statistics
import time

import serial


def read_register(port, address):
    port.write(f"xd {address:#x} 4\n".encode())
    data = b""
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        data += port.read(4096)
        found = re.search(rb"0000: ((?:[0-9a-fA-F]{2} ){3}[0-9a-fA-F]{2})", data)
        if found:
            return int.from_bytes(bytes.fromhex(found[1].decode()), "little")
    raise RuntimeError(f"No NSH register reply for {address:#x}; close other console users")


def slope(x, y):
    mx, my = statistics.fmean(x), statistics.fmean(y)
    denominator = sum((v - mx) ** 2 for v in x)
    if denominator == 0:
        raise RuntimeError("Reference counter is not advancing")
    return sum((a - mx) * (b - my) for a, b in zip(x, y)) / denominator


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/v4w_consol')
    parser.add_argument('--seconds', type=float, default=20)
    args = parser.parse_args()
    if not 10 <= args.seconds <= 120:
        parser.error('--seconds must be 10..120')
    with serial.Serial(args.port, 115200, timeout=.02, exclusive=True) as port:
        port.reset_input_buffer()
        # STM32H743 RCC and TIM5; never write memory-mapped registers.
        for label, address in [('PLLCKSELR', 0x58024428), ('PLL1DIVR', 0x58024430),
                               ('PLL1FRACR', 0x58024434), ('D1CFGR', 0x58024418),
                               ('D2CFGR', 0x5802441c), ('TIM5 PSC', 0x40000c28)]:
            print(f'{label}={read_register(port, address):08x}', flush=True)
        rows = []
        previous = None
        frames = ticks = 0
        started = time.monotonic()
        while time.monotonic() - started < args.seconds:
            before = time.monotonic()
            counter = read_register(port, 0x40000c24)
            dsts = read_register(port, 0x40080808)
            after = time.monotonic()
            frame = (dsts >> 8) & 0x7ff  # full-speed USB 11-bit SOF number
            if dsts & 1:
                raise RuntimeError('USB suspended; no valid SOF reference')
            if previous is not None:
                if after - previous[2] >= 1:
                    raise RuntimeError('Console delay too large for unambiguous SOF unwrapping')
                frames += (frame - previous[0]) % 2048
                ticks += (counter - previous[1]) % (1 << 32)
            previous = frame, counter, after
            rows.append(((before + after) / 2, ticks, frames))
            time.sleep(.15)
        host, timer, sof = zip(*rows)
        timer_host = slope(host, timer)
        sof_host = slope(host, sof)
        timer_sof = slope(sof, timer)
        print(f'{len(rows)} observations; {host[-1] - host[0]:.1f} s')
        print(f'TIM5 ticks / host second: {timer_host:.3f} (nominal 1000000)')
        print(f'USB frames / host second: {sof_host:.3f} (nominal 1000)')
        print(f'TIM5 ticks / USB frame: {timer_sof:.3f} (nominal 1000)')
        print(f'TIM5 relative to USB SOF: {(timer_sof / 1000 - 1) * 1e6:+.0f} ppm')
        print('Sequential-read screening only; confirm physical timing with a scope.')


if __name__ == '__main__':
    main()
