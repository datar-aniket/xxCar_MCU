#!/usr/bin/env python3
"""Companion-link codec and serial reader.

Mirrors apps/companion/comp_proto.c. The wire format is

    0xFE | id | len | payload[len] | crc16

with the CRC over id, len and payload.

This is a SECOND implementation of a format that already exists in C, which
is a thing worth being nervous about: a CRC or a struct layout that disagrees
produces frames the board silently drops as corrupt, and the only symptom is
a counter climbing on one end with no indication of which end is wrong. So
tools/test-comp-proto.sh compiles the C encoder and compares its bytes
against this one for the same payload.

Nothing here imports tkinter, so it runs headless.
"""

import math
import queue
import struct
import threading
import time

import serial

SYNC = 0xFE
MAX_PAYLOAD = 64
FRAME_OVERHEAD = 5

MSG_EXTERNAL_POSE = 1
MSG_CONTROL_TRAJ = 2       # reserved
MSG_TIMESYNC_REQ = 3
MSG_TIMESYNC_REP = 4
MSG_TIMESYNC_START = 5
MSG_TIMESYNC_END = 6
MSG_ESTIMATOR_POSE = 16

POSE_FLAG_VALID = 1 << 0

# struct comp_external_pose_s - 48 bytes
EXTERNAL_POSE = struct.Struct("<Q3f6fBB2x")

# struct comp_estimator_pose_s - 56 bytes
ESTIMATOR_POSE = struct.Struct("<Q3f4f3fBB6x")

assert EXTERNAL_POSE.size == 48, EXTERNAL_POSE.size
assert ESTIMATOR_POSE.size == 56, ESTIMATOR_POSE.size

# struct comp_timesync_req_s / _rep_s
TIMESYNC_REQ = struct.Struct("<Q")
TIMESYNC_REP = struct.Struct("<QQQ")
TIMESYNC_START = struct.Struct("<II")
TIMESYNC_END = struct.Struct("<qII")

assert TIMESYNC_REQ.size == 8, TIMESYNC_REQ.size
assert TIMESYNC_REP.size == 24, TIMESYNC_REP.size
assert TIMESYNC_START.size == 8, TIMESYNC_START.size
assert TIMESYNC_END.size == 16, TIMESYNC_END.size

PAYLOAD_LEN = {
    MSG_EXTERNAL_POSE: EXTERNAL_POSE.size,
    MSG_ESTIMATOR_POSE: ESTIMATOR_POSE.size,
    MSG_TIMESYNC_REQ: TIMESYNC_REQ.size,
    MSG_TIMESYNC_REP: TIMESYNC_REP.size,
    MSG_TIMESYNC_START: TIMESYNC_START.size,
    MSG_TIMESYNC_END: TIMESYNC_END.size,
}

_CRC_TAB = (0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
            0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF)


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC16-CCITT-FALSE, nibble table - the same one comp_proto.c uses."""
    for b in data:
        hi = ((crc >> 12) ^ (b >> 4)) & 0xF
        crc = ((crc << 4) ^ _CRC_TAB[hi]) & 0xFFFF
        hi = ((crc >> 12) ^ (b & 0xF)) & 0xF
        crc = ((crc << 4) ^ _CRC_TAB[hi]) & 0xFFFF
    return crc


def encode(msg_id: int, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload {len(payload)} exceeds {MAX_PAYLOAD}")
    head = bytes((msg_id, len(payload)))
    crc = crc16(head + payload)
    return bytes((SYNC,)) + head + payload + struct.pack("<H", crc)


def encode_external_pose(x, y, yaw, cov=None, valid=True, reset_counter=0,
                         timestamp_us=0) -> bytes:
    """Frame an EXTERNAL_POSE.

    A zero covariance means "no estimate supplied"; the board falls back to
    EK3_EXT_M_NSE, which is a FLOOR rather than a default - a reported sigma
    below it is raised to it.

    A zero timestamp means "not timestamped": the board stamps it on arrival
    rather than measuring its age against a clock the source does not share.
    That costs the link latency as position error, so send a real one once
    there is a timesync.
    """
    cov = tuple(cov) if cov else (0.0,) * 6
    body = EXTERNAL_POSE.pack(int(timestamp_us), float(x), float(y),
                              float(yaw), *[float(c) for c in cov],
                              POSE_FLAG_VALID if valid else 0,
                              int(reset_counter) & 0xFF)
    return encode(MSG_EXTERNAL_POSE, body)


def host_now_us() -> int:
    """Host MONOTONIC microseconds - the timebase the exchange is measured in.

    Not wall time. A round trip measured against a clock NTP can step is
    meaningless the moment it steps, and the step would land in the offset
    rather than showing up as an outlier the min-RTT pick could discard.
    """
    return int(time.monotonic() * 1e6)


class UtcClock:
    """UTC microseconds, advanced by the MONOTONIC clock.

    The board is told UTC, but the burst that measures the link must not be
    vulnerable to the wall clock stepping halfway through it. So UTC is
    sampled ONCE and carried forward by the monotonic clock: absolute like
    wall time, and as steady as monotonic between samples.
    """

    def __init__(self):
        self.utc_base_us = int(time.time() * 1e6)
        self.mono_base_us = host_now_us()

    def now_us(self) -> int:
        return self.utc_base_us + (host_now_us() - self.mono_base_us)

    def to_utc(self, mono_us: int) -> int:
        return self.utc_base_us + (mono_us - self.mono_base_us)


def encode_timesync_req(host_tx_us: int) -> bytes:
    return encode(MSG_TIMESYNC_REQ, TIMESYNC_REQ.pack(int(host_tx_us)))


def encode_timesync_start(count: int) -> bytes:
    return encode(MSG_TIMESYNC_START, TIMESYNC_START.pack(int(count), 0))


def encode_timesync_end(utc_offset_us: int, trip_us: int,
                        samples: int) -> bytes:
    """utc_offset_us: add to the board's MONOTONIC time to get UTC."""
    return encode(MSG_TIMESYNC_END,
                  TIMESYNC_END.pack(int(utc_offset_us), int(trip_us),
                                    int(samples)))


def decode_timesync_rep(payload: bytes) -> dict:
    host_tx, board_rx, board_tx = TIMESYNC_REP.unpack(payload)
    return {"host_tx_us": host_tx, "board_rx_us": board_rx,
            "board_tx_us": board_tx}


def timesync_solve(rep: dict, host_rx_us: int):
    """(offset, round_trip) in microseconds, from one exchange.

    offset is what to ADD to a host timestamp to express it in board time;
    negate it to get what the board adds to its own clock to reach ours.

    Subtracting the board's own processing time is what makes this better
    than a naive round-trip halving: without board_tx the board's delay is
    indistinguishable from wire latency and lands entirely in the offset.
    """
    offset = ((rep["board_rx_us"] - rep["host_tx_us"]) +
              (rep["board_tx_us"] - host_rx_us)) // 2
    trip = ((host_rx_us - rep["host_tx_us"]) -
            (rep["board_tx_us"] - rep["board_rx_us"]))
    return offset, trip


def decode_estimator_pose(payload: bytes) -> dict:
    f = ESTIMATOR_POSE.unpack(payload)
    return {
        "timestamp_us": f[0],
        "position": f[1:4],
        "quaternion": f[4:8],
        "velocity": f[8:11],
        "solution_status": f[11],
        "reset_counter": f[12],
    }


def quaternion_to_euler(q):
    """Roll, pitch, yaw in radians, from a w/x/y/z body-to-ENU quaternion.

    Transcribed from ekf_core_euler() in apps/ekf3/ekf_core.c rather than
    written afresh. This project is FLU body / ENU navigation, where yaw is
    counter-clockwise from EAST and roll runs the opposite sense to the FRD
    convention most reference code assumes - so a formula copied from
    anywhere else would disagree by a sign or ninety degrees and still look
    plausible.
    """
    w, x, y, z = q
    pitch_sine = 2.0 * (w * y - z * x)
    pitch_sine = max(-1.0, min(1.0, pitch_sine))

    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(pitch_sine)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


SOLUTION_BITS = (
    (1 << 0, "ATT"),
    (1 << 1, "YAW_REL"),
    (1 << 2, "YAW_ABS"),
    (1 << 3, "VELXY"),
    (1 << 4, "VELZ"),
    (1 << 5, "POSXY"),
    (1 << 6, "POSZ"),
)


def solution_names(status: int) -> str:
    names = [n for bit, n in SOLUTION_BITS if status & bit]
    return " ".join(names) if names else "NONE"


class Parser:
    """Byte-fed frame parser, mirroring the C state machine.

    One byte at a time because a UART splits frames, and a parser that only
    works on whole reads fails the first time it happens.
    """

    WAIT_SYNC, WAIT_ID, WAIT_LEN, WAIT_PAYLOAD, WAIT_CRC_LO, WAIT_CRC_HI = \
        range(6)

    def __init__(self):
        self.state = self.WAIT_SYNC
        self.id = 0
        self.len = 0
        self.payload = bytearray()
        self.crc_rx = 0
        self.frames = 0
        self.crc_errors = 0
        self.unknown_id = 0
        self.bad_length = 0

    def feed(self, b: int):
        """Return (id, payload) when a valid frame completes, else None."""
        if self.state == self.WAIT_SYNC:
            if b == SYNC:
                self.state = self.WAIT_ID
            return None

        if self.state == self.WAIT_ID:
            self.id = b
            self.state = self.WAIT_LEN
            return None

        if self.state == self.WAIT_LEN:
            if b > MAX_PAYLOAD:
                self.bad_length += 1
                self.state = self.WAIT_SYNC
                return None
            self.len = b
            self.payload = bytearray()
            self.state = self.WAIT_PAYLOAD if b else self.WAIT_CRC_LO
            return None

        if self.state == self.WAIT_PAYLOAD:
            # The length field is authoritative: a 0xFE inside a float must
            # not restart the parse.
            self.payload.append(b)
            if len(self.payload) >= self.len:
                self.state = self.WAIT_CRC_LO
            return None

        if self.state == self.WAIT_CRC_LO:
            self.crc_rx = b
            self.state = self.WAIT_CRC_HI
            return None

        self.crc_rx |= b << 8
        self.state = self.WAIT_SYNC

        want = crc16(bytes((self.id, self.len)) + bytes(self.payload))
        if want != self.crc_rx:
            self.crc_errors += 1
            return None

        expect = PAYLOAD_LEN.get(self.id)
        if expect is None:
            self.unknown_id += 1
            return None
        if expect != self.len:
            self.bad_length += 1
            return None

        self.frames += 1
        return self.id, bytes(self.payload)


class Link(threading.Thread):
    """Reads the port on its own thread and posts frames to a queue."""

    def __init__(self, port, baud, out: queue.Queue):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.out = out
        self.stop_flag = threading.Event()
        # read() runs on this thread and send() is called from Tk's. pyserial
        # does not promise those are safe together.
        self._tx_lock = threading.Lock()
        self.parser = Parser()
        self.bytes_in = 0
        self.bytes_out = 0
        self.tx_frames = 0

    def send(self, frame: bytes):
        try:
            with self._tx_lock:
                self.ser.write(frame)
            self.bytes_out += len(frame)
            self.tx_frames += 1
        except Exception as exc:
            self.out.put(("error", f"write failed: {exc}"))

    def close(self):
        self.stop_flag.set()

    def run(self):
        while not self.stop_flag.is_set():
            try:
                n = max(1, self.ser.in_waiting)
                chunk = self.ser.read(n)
            except Exception as exc:
                self.out.put(("error", f"link lost: {exc}"))
                return
            if chunk:
                # Stamp arrival HERE, on the reading thread.
                #
                # Taking it where the frame is consumed instead puts the
                # consumer's scheduling between the wire and the timestamp.
                # The Tk pump runs on a 100 ms timer, so a round trip
                # measured there reads as 5-20 ms of jitter that is entirely
                # the timer, and no amount of work on the board can improve
                # a number measured that late.
                rx_us = host_now_us()
                self.bytes_in += len(chunk)
                for b in chunk:
                    got = self.parser.feed(b)
                    if got is not None:
                        self.out.put(("frame", (got[0], got[1], rx_us)))
        try:
            self.ser.close()
        except Exception:
            pass
