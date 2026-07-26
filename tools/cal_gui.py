#!/usr/bin/env python3
"""xxCar calibration GUI.

Talks to `cal session` on the board's USB CDC port. Two encodings share the one
pipe and are told apart by the first byte of each message:

    '{'   an ASCII JSON line, terminated by '\\n'  - control and replies
    0xA5  a binary sample frame                   - see FRAME below

Only stdlib plus pyserial: tkinter draws the plots on a Canvas rather than
pulling in matplotlib, which keeps the install to one package and the redraw
fast enough for a live strip chart.

Run:  python3 tools/cal_gui.py
"""

import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    raise SystemExit("pyserial missing:  pip install pyserial")

import json

SYNC = 0xA5
PROTO = 1

# Binary sample frame, little-endian:
#   0xA5 | len | id | seq | t_us(u32) | values(f32 * n) | crc16
FRAME_HDR = struct.Struct("<BBBBI")     # sync, len, id, seq, t_us


def crc16(data: bytes) -> int:
    """CRC16-CCITT-FALSE, matching the board."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Link(threading.Thread):
    """Reads the port, demuxes JSON lines from binary frames, posts to a queue.

    Runs on its own thread so a stalled or chatty board can never freeze the
    UI - tkinter drains the queue on a timer instead.
    """

    def __init__(self, port, baud, out: queue.Queue):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.out = out
        self.stop_flag = threading.Event()
        self._buf = bytearray()

    def send(self, line: str):
        self.ser.write((line + "\n").encode())
        self.out.put(("tx", line))

    def close(self):
        self.stop_flag.set()

    def run(self):
        while not self.stop_flag.is_set():
            try:
                chunk = self.ser.read(4096)
            except Exception as exc:                       # cable pulled
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

        Resynchronisation matters here: if a byte is lost the stream must
        recover on its own rather than wedge. Anything that is neither '{' nor
        the sync byte is discarded, so a corrupt frame costs one message.
        """
        while True:
            if not self._buf:
                return

            b = self._buf[0]

            if b == SYNC:
                if len(self._buf) < 2:
                    return
                ln = self._buf[1]
                total = 2 + ln + 2                  # sync+len + payload + crc
                if len(self._buf) < total:
                    return
                frame = bytes(self._buf[:total])
                got = struct.unpack("<H", frame[-2:])[0]
                if crc16(frame[1:-2]) == got:
                    self._emit_sample(frame)
                    del self._buf[:total]
                else:
                    self.out.put(("error", "bad CRC, resyncing"))
                    del self._buf[:1]               # drop one, try again
                continue

            if b == 0x7B:                           # '{'
                nl = self._buf.find(b"\n")
                if nl < 0:
                    return
                line = bytes(self._buf[:nl]).decode(errors="replace")
                del self._buf[: nl + 1]
                try:
                    self.out.put(("json", json.loads(line)))
                except json.JSONDecodeError:
                    self.out.put(("error", f"bad JSON: {line[:60]}"))
                continue

            del self._buf[:1]                       # noise; skip it

    def _emit_sample(self, frame: bytes):
        _, ln, sid, seq, t_us = FRAME_HDR.unpack(frame[:8])
        nvals = (ln - 6) // 4
        vals = struct.unpack(f"<{nvals}f", frame[8:8 + 4 * nvals])
        self.out.put(("sample", (sid, seq, t_us, vals)))


class Strip(tk.Canvas):
    """A live strip chart: one trace per axis, newest on the right."""

    COLOURS = ["#e6194b", "#3cb44b", "#4363d8", "#f58231",
               "#911eb4", "#42d4f4", "#f032e6", "#bfef45"]
    KEEP = 600

    def __init__(self, master, **kw):
        super().__init__(master, bg="#14161a", highlightthickness=0, **kw)
        self.labels: list[str] = []
        self.series: list[list[float]] = []
        self.bind("<Configure>", lambda _e: self.redraw())

    def configure_series(self, labels):
        self.labels = list(labels)
        self.series = [[] for _ in labels]
        self.redraw()

    def push(self, values):
        for i, v in enumerate(values):
            if i < len(self.series):
                s = self.series[i]
                s.append(v)
                if len(s) > self.KEEP:
                    del s[: len(s) - self.KEEP]

    def redraw(self):
        self.delete("all")
        w = self.winfo_width()
        h = self.winfo_height()
        if w < 20 or h < 20 or not self.series:
            return

        flat = [v for s in self.series for v in s]
        if not flat:
            self.create_text(w // 2, h // 2, fill="#556",
                             text="waiting for samples…")
            return

        lo, hi = min(flat), max(flat)
        if hi - lo < 1e-9:                 # a flat trace still needs a band
            lo, hi = lo - 1.0, hi + 1.0
        pad = (hi - lo) * 0.1
        lo, hi = lo - pad, hi + pad

        def y_of(v):
            return h - (v - lo) / (hi - lo) * h

        for gy in (0.0, 0.25, 0.5, 0.75, 1.0):
            y = gy * h
            self.create_line(0, y, w, y, fill="#22262d")
        self.create_text(4, 8, anchor="w", fill="#7a8290",
                         text=f"{hi:.4g}", font=("TkFixedFont", 8))
        self.create_text(4, h - 8, anchor="w", fill="#7a8290",
                         text=f"{lo:.4g}", font=("TkFixedFont", 8))

        n = max(len(s) for s in self.series)
        step = w / max(n - 1, 1)
        for i, s in enumerate(self.series):
            if len(s) < 2:
                continue
            pts = []
            off = n - len(s)
            for j, v in enumerate(s):
                pts += [(off + j) * step, y_of(v)]
            self.create_line(*pts, fill=self.COLOURS[i % len(self.COLOURS)],
                             width=1.4)

        for i, lab in enumerate(self.labels):
            last = self.series[i][-1] if self.series[i] else 0.0
            self.create_text(w - 6, 10 + i * 14, anchor="e",
                             fill=self.COLOURS[i % len(self.COLOURS)],
                             text=f"{lab} {last: .4f}",
                             font=("TkFixedFont", 9))


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("xxCar calibration")
        self.geometry("980x640")
        self.configure(bg="#0f1115")

        self.q: queue.Queue = queue.Queue()
        self.link: Link | None = None
        self.sensors: dict[int, dict] = {}
        self.active_id: int | None = None
        self.last_seq: dict[int, int] = {}
        self.drops = 0

        self._build()
        self.after(40, self._pump)
        self.protocol("WM_DELETE_WINDOW", self._quit)

    # ---- layout ---------------------------------------------------------

    def _build(self):
        bar = ttk.Frame(self, padding=6)
        bar.pack(fill="x")

        ttk.Label(bar, text="Port").pack(side="left")
        self.port_cb = ttk.Combobox(bar, width=22, state="readonly")
        self.port_cb.pack(side="left", padx=4)
        ttk.Button(bar, text="Rescan", command=self._scan).pack(side="left")
        self.open_btn = ttk.Button(bar, text="Open", command=self._toggle)
        self.open_btn.pack(side="left", padx=6)

        self.status = ttk.Label(bar, text="closed")
        self.status.pack(side="left", padx=10)

        body = ttk.Panedwindow(self, orient="horizontal")
        body.pack(fill="both", expand=True, padx=6, pady=(0, 6))

        left = ttk.Frame(body)
        ttk.Label(left, text="Sensors").pack(anchor="w")
        self.tree = ttk.Treeview(left, columns=("rate",), show="tree headings",
                                 height=12)
        self.tree.heading("#0", text="sensor")
        self.tree.heading("rate", text="Hz")
        self.tree.column("#0", width=150)
        self.tree.column("rate", width=60, anchor="e")
        self.tree.pack(fill="both", expand=True)
        self.tree.bind("<<TreeviewSelect>>", self._select)
        body.add(left, weight=1)

        right = ttk.Frame(body)
        self.plot = Strip(right)
        self.plot.pack(fill="both", expand=True)
        self.plot_info = ttk.Label(right, text="select a sensor")
        self.plot_info.pack(anchor="w")
        body.add(right, weight=3)

        ttk.Label(self, text="link").pack(anchor="w", padx=6)
        self.log = tk.Text(self, height=8, bg="#0b0d10", fg="#9aa4b2",
                           insertbackground="#9aa4b2", font=("TkFixedFont", 9))
        self.log.pack(fill="x", padx=6, pady=(0, 6))

        self._scan()

    # ---- link -----------------------------------------------------------

    def _scan(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_cb["values"] = ports
        pref = [p for p in ports if "ACM" in p]
        if pref:
            self.port_cb.set(pref[0])
        elif ports:
            self.port_cb.set(ports[0])

    def _toggle(self):
        if self.link:
            self._close()
        else:
            self._open()

    def _open(self):
        port = self.port_cb.get()
        if not port:
            self._say("no port selected")
            return
        try:
            self.link = Link(port, 115200, self.q)
        except Exception as exc:
            self._say(f"open failed: {exc}")
            return
        self.link.start()
        self.open_btn.config(text="Close")
        self.status.config(text=f"open {port}")

        # Opening the tool IS the handshake: identify, then ask what there is.
        self.link.send("hello")
        self.after(150, lambda: self.link and self.link.send("list"))

    def _close(self):
        if self.link:
            try:
                self.link.send("quit")
                time.sleep(0.1)
            except Exception:
                pass
            self.link.close()
            self.link = None
        self.open_btn.config(text="Open")
        self.status.config(text="closed")

    def _quit(self):
        self._close()
        self.destroy()

    # ---- events ---------------------------------------------------------

    def _say(self, msg):
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        if float(self.log.index("end")) > 400:
            self.log.delete("1.0", "200.0")

    def _pump(self):
        redraw = False
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "json":
                    self._on_json(payload)
                elif kind == "sample":
                    self._on_sample(payload)
                    redraw = True
                elif kind == "tx":
                    self._say(f"> {payload}")
                else:
                    self._say(f"! {payload}")
        except queue.Empty:
            pass
        if redraw:
            self.plot.redraw()
        self.after(40, self._pump)

    def _on_json(self, msg):
        evt = msg.get("evt")
        if evt == "sensor":
            self.sensors[msg["id"]] = msg
            self._add_row(msg)
            return
        self._say(f"< {json.dumps(msg)}")
        if evt == "hello" and msg.get("proto") != PROTO:
            self._say(f"! proto mismatch: board {msg.get('proto')}, gui {PROTO}")

    def _add_row(self, s):
        iid = str(s["id"])
        label = s["name"] if s["present"] else f"{s['name']}  (absent)"
        rate = s.get("rate", 0)
        if self.tree.exists(iid):
            self.tree.item(iid, text=label, values=(rate,))
        else:
            self.tree.insert("", "end", iid=iid, text=label, values=(rate,))

    def _select(self, _evt):
        sel = self.tree.selection()
        if not sel or not self.link:
            return
        sid = int(sel[0])
        s = self.sensors.get(sid)
        if not s:
            return
        if not s["present"]:
            self._say(f"! {s['name']} is not publishing - nothing to plot")
            return
        self.active_id = sid
        self.drops = 0
        self.plot.configure_series(s["labels"])
        self.plot_info.config(text=f"{s['name']}  [{s['unit']}]")
        self.link.send("stop")
        self.link.send(f"stream {s['name']} 50")

    def _on_sample(self, payload):
        sid, seq, _t_us, vals = payload
        if sid != self.active_id:
            return
        prev = self.last_seq.get(sid)
        if prev is not None and (prev + 1) & 0xFF != seq:
            self.drops += 1
            self.plot_info.config(
                text=f"{self.sensors[sid]['name']}  "
                     f"[{self.sensors[sid]['unit']}]   dropped {self.drops}")
        self.last_seq[sid] = seq
        self.plot.push(vals)


if __name__ == "__main__":
    App().mainloop()
