"""Prove a split ULog part is independently readable.

The 100 MB rollover writes a fresh header, format section and subscription
list at the head of every part. If that prologue were wrong - or omitted, as it
would be if the byte stream were simply cut - every part after the first would
be unopenable, and an eight hour run would yield 100 MB of usable data and
several gigabytes of rubble. Nothing on the board would report a problem.

So this builds parts with the exact layout apps/logger/logger.c writes, with
different start timestamps and different data per part, and requires pyulog to
open each one on its own.

Layout, from logger.c:
    magic 8B | start_us u64 | 'B' flag-bits (40 zero bytes)
    then messages, each: u16 payload_len | u8 type | payload
      'F'  "name:field;field;"
      'A'  multi_id u8 | msg_id u16 | name
      'D'  msg_id u16 | record
"""

import struct
import sys
import tempfile
from pathlib import Path

MAGIC = bytes([0x55, 0x4C, 0x6F, 0x67, 0x01, 0x12, 0x35, 0x01])

ACCEL_FMT = "sensor_accel:uint64_t timestamp;float x;float y;float z;float temperature;"
GYRO_FMT = "sensor_gyro:uint64_t timestamp;float x;float y;float z;float temperature;"


def msg(mtype: str, payload: bytes) -> bytes:
    return struct.pack("<HB", len(payload), ord(mtype)) + payload


def add_sub(multi_id: int, msg_id: int, name: str) -> bytes:
    return msg("A", struct.pack("<BH", multi_id, msg_id) + name.encode())


def build_part(start_us: int, nrec: int, base: float) -> bytes:
    """One complete part: prologue, then interleaved accel/gyro records."""
    out = bytearray()
    out += MAGIC + struct.pack("<Q", start_us)
    out += msg("B", bytes(40))

    # definition section - repeated in EVERY part, which is the point
    out += msg("F", ACCEL_FMT.encode())
    out += msg("F", GYRO_FMT.encode())
    out += add_sub(0, 0, "sensor_accel")
    out += add_sub(0, 1, "sensor_gyro")

    for k in range(nrec):
        t = start_us + k * 500
        acc = struct.pack("<Qffff", t, base + k, 0.5, 9.81, 41.0)
        gyr = struct.pack("<Qffff", t, 0.01, -0.02, base * 0.1, 41.0)
        out += msg("D", struct.pack("<H", 0) + acc)
        out += msg("D", struct.pack("<H", 1) + gyr)
    return bytes(out)


def main() -> int:
    try:
        from pyulog import ULog
    except ImportError:
        print("ulog_split: pyulog not installed - skipped")
        return 0

    fails = []
    tmp = Path(tempfile.mkdtemp())

    # Two parts of one session, as the rollover produces them: different start
    # timestamps, different data, each with its own prologue.
    parts = [
        ("log_007_00.ulg", build_part(1_000_000, 40, 100.0)),
        ("log_007_01.ulg", build_part(9_500_000, 25, 500.0)),
    ]

    for name, blob in parts:
        (tmp / name).write_bytes(blob)

    for name, blob in parts:
        path = tmp / name
        try:
            u = ULog(str(path))
        except Exception as exc:                       # noqa: BLE001
            fails.append(f"{name}: pyulog could not open it ({exc})")
            continue

        got = sorted(d.name for d in u.data_list)
        if got != ["sensor_accel", "sensor_gyro"]:
            fails.append(f"{name}: topics {got}")
            continue

        by = {d.name: d for d in u.data_list}
        n = len(by["sensor_accel"].data["x"])
        first = by["sensor_accel"].data["x"][0]
        print(f"  {name}: {len(blob):6d} B, {n:3d} accel records, "
              f"x[0]={first:.1f}, start={u.start_timestamp}")

    if len(parts) == 2:
        a = ULog(str(tmp / parts[0][0]))
        b = ULog(str(tmp / parts[1][0]))
        na = len({d.name: d for d in a.data_list}["sensor_accel"].data["x"])
        nb = len({d.name: d for d in b.data_list}["sensor_accel"].data["x"])
        xa = {d.name: d for d in a.data_list}["sensor_accel"].data["x"][0]
        xb = {d.name: d for d in b.data_list}["sensor_accel"].data["x"][0]
        if na == nb:
            fails.append("both parts decoded the same record count - "
                         "suspicious, they were built with 40 and 25")
        if xa == xb:
            fails.append("both parts decoded identical data")
        if a.start_timestamp == b.start_timestamp:
            fails.append("both parts share a start timestamp")

    # A part WITHOUT its prologue must fail - proving the test would notice if
    # the rollover ever stopped writing one.
    headless = tmp / "headless.ulg"
    body = build_part(1_000_000, 10, 1.0)
    headless.write_bytes(body[16:])            # strip magic + start timestamp
    try:
        ULog(str(headless))
        fails.append("a prologue-less file parsed - this test cannot detect "
                     "the failure it exists for")
    except Exception:                           # noqa: BLE001
        print("  headless.ulg: correctly rejected")

    for f in fails:
        print(f"FAIL {f}")
    if fails:
        return 1
    print("ulog_split: every part opens on its own - OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
