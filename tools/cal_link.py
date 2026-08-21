#!/usr/bin/env python3
"""Serial link to the board: JSON lines and binary sample frames.

Extracted from cal_gui.py so the alignment driver can share exactly this
decoder rather than carry a second copy. A frame decoder that exists twice is
a frame decoder that will disagree with itself the first time the wire format
moves, and the format is already versioned only by both ends agreeing.

Nothing here imports tkinter, which is what lets it run headless.
"""

import json
import queue
import struct
import threading

import serial

SYNC = 0xA5
ENC_I16, ENC_F32 = 0, 1

# sync, len, sensor id, seq, t0, dt, count, values, encoding
HDR = struct.Struct("<BHBBIHBBB")
HDR_LEN = HDR.size                      # 14


def crc16(data: bytes) -> int:
    """CRC16-CCITT-FALSE, cross-checked against the board's cal_crc16()."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 \
                else (crc << 1) & 0xFFFF
    return crc


class Link(threading.Thread):
    """Reads the port, demuxes JSON lines from binary frames, posts to a queue.

    Its own thread, so a stalled or chatty board can never freeze the UI -
    tkinter drains the queue on a timer instead. Frames are decoded here rather
    than on the UI thread so a 2 kHz stream costs the UI only the plotting.
    """

    def __init__(self, port, baud, out: queue.Queue):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.out = out
        self.stop_flag = threading.Event()
        self.scales: dict[int, float] = {}
        self._buf = bytearray()

    def send(self, line: str):
        try:
            self.ser.write((line + "\n").encode())
            self.out.put(("tx", line))
        except Exception as exc:
            self.out.put(("error", f"write failed: {exc}"))

    def close(self):
        self.stop_flag.set()

    def run(self):
        while not self.stop_flag.is_set():
            try:
                n = max(1, self.ser.in_waiting)
                chunk = self.ser.read(n)
            except Exception as exc:                    # cable pulled
                self.out.put(("error", f"link lost: {exc}"))
                return
            if chunk:
                self._buf.extend(chunk)
                self._drain()
        try:
            self.ser.close()
        except Exception:
            pass

    def _drain(self):
        """Pull every complete message out of the buffer.

        Resynchronisation matters: if a byte is lost the stream must recover on
        its own rather than wedge. Anything that is neither '{' nor the sync
        byte is discarded, so a corrupt frame costs one message.
        """
        buf = self._buf
        while buf:
            b = buf[0]

            if b == SYNC:
                if len(buf) < 3:
                    return
                ln = struct.unpack_from("<H", buf, 1)[0]
                total = ln + 5                      # sync + len + body + crc
                if len(buf) < total:
                    return
                frame = bytes(buf[:total])
                if crc16(frame[1:-2]) == struct.unpack("<H", frame[-2:])[0]:
                    self._emit(frame)
                    del buf[:total]
                else:
                    self.out.put(("error", "bad CRC, resyncing"))
                    del buf[:1]
                continue

            if b == 0x7B:                            # '{'
                nl = buf.find(b"\n")
                if nl < 0:
                    return
                line = bytes(buf[:nl]).decode(errors="replace")
                del buf[: nl + 1]
                try:
                    self.out.put(("json", json.loads(line)))
                except json.JSONDecodeError:
                    self.out.put(("error", f"bad JSON: {line[:60]}"))
                continue

            del buf[:1]                              # noise; skip it

    def _emit(self, frame: bytes):
        _, _ln, sid, seq, t0, dt, count, nvals, enc = HDR.unpack_from(frame, 0)
        body = frame[HDR_LEN:-2]
        n = count * nvals
        if enc == ENC_I16:
            raw = struct.unpack_from(f"<{n}h", body, 0)
            sc = self.scales.get(sid, 1.0)
            flat = [r * sc for r in raw]
        else:
            flat = list(struct.unpack_from(f"<{n}f", body, 0))
        rows = [flat[k * nvals:(k + 1) * nvals] for k in range(count)]
        self.out.put(("batch", (sid, seq, t0, dt, rows)))
