#!/usr/bin/env python3
"""xxCar calibration GUI.

Talks to `cal session` on the board's USB CDC port. Two encodings share the one
pipe and are told apart by the first byte of each message:

    '{'   an ASCII JSON line, terminated by '\\n'  - control and replies
    0xA5  a binary sample frame                   - see FRAME_HDR

Only stdlib plus pyserial: tkinter draws the plot on a Canvas rather than
pulling in matplotlib, which keeps the install to one package and the redraw
fast enough for a live strip chart.

Run:  python3 tools/cal_gui.py
"""

import json
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

SYNC = 0xA5
PROTO = 1

# 0xA5 | len | id | seq | t_us(u32) | values(f32 * n) | crc16, little-endian
FRAME_HDR = struct.Struct("<BBBBI")

# ---- palette ---------------------------------------------------------------

BG = "#0f1115"
PANEL = "#161a21"
PLOT_BG = "#0b0d11"
GRID = "#1e232c"
FG = "#c8d0da"
MUTED = "#6b7688"
ACCENT = "#4c9aff"
TRACE = ["#ff5c7a", "#3ddc84", "#4c9aff", "#ffb74d",
         "#c77dff", "#4dd0e1", "#f06292", "#aed581"]


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
    tkinter drains the queue on a timer instead.
    """

    def __init__(self, port, baud, out: queue.Queue):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.out = out
        self.stop_flag = threading.Event()
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
                chunk = self.ser.read(4096)
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
        while True:
            if not self._buf:
                return
            b = self._buf[0]

            if b == SYNC:
                if len(self._buf) < 2:
                    return
                total = 2 + self._buf[1] + 2       # sync+len, payload, crc
                if len(self._buf) < total:
                    return
                frame = bytes(self._buf[:total])
                if crc16(frame[1:-2]) == struct.unpack("<H", frame[-2:])[0]:
                    self._emit_sample(frame)
                    del self._buf[:total]
                else:
                    self.out.put(("error", "bad CRC, resyncing"))
                    del self._buf[:1]
                continue

            if b == 0x7B:                            # '{'
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

            del self._buf[:1]                        # noise; skip it

    def _emit_sample(self, frame: bytes):
        _, ln, sid, seq, t_us = FRAME_HDR.unpack(frame[:8])
        n = (ln - 6) // 4
        vals = struct.unpack(f"<{n}f", frame[8:8 + 4 * n])
        self.out.put(("sample", (sid, seq, t_us, vals)))


class Strip(tk.Canvas):
    """Live strip chart. Newest sample on the right, time increasing rightward.

    Holds (t, value) rather than bare values so the x axis is real seconds and
    the window is a duration, not a sample count - otherwise changing the rate
    would silently change how much history you are looking at.
    """

    def __init__(self, master, **kw):
        super().__init__(master, bg=PLOT_BG, highlightthickness=0, **kw)
        self.labels: list[str] = []
        self.visible: list[bool] = []
        self.data: list[list[tuple[float, float]]] = []
        self.window_s = 10.0
        self.width_px = 1.6
        self.ylim = None              # None = autoscale, else (lo, hi)
        self.unit = ""
        self.bind("<Configure>", lambda _e: self.redraw())

    def set_series(self, labels):
        self.labels = list(labels)
        self.visible = [True] * len(labels)
        self.data = [[] for _ in labels]
        self.redraw()

    def clear(self):
        self.data = [[] for _ in self.labels]
        self.redraw()

    def push(self, t, values):
        cutoff = t - self.window_s * 1.2      # a little slack beyond the view
        for i, v in enumerate(values):
            if i >= len(self.data):
                break
            d = self.data[i]
            d.append((t, v))
            if d and d[0][0] < cutoff:
                keep = 0
                for keep, (ts, _) in enumerate(d):
                    if ts >= cutoff:
                        break
                del d[:keep]

    def _shown(self):
        return [i for i, ok in enumerate(self.visible)
                if ok and i < len(self.data) and self.data[i]]

    def redraw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 40 or h < 40:
            return

        pad_l, pad_r, pad_t, pad_b = 62, 12, 10, 22
        pw, ph = w - pad_l - pad_r, h - pad_t - pad_b
        if pw < 10 or ph < 10:
            return

        self.create_rectangle(pad_l, pad_t, pad_l + pw, pad_t + ph,
                              outline=GRID)

        shown = self._shown()
        if not shown:
            self.create_text(w // 2, h // 2, fill=MUTED,
                             text="no axis selected"
                                  if self.labels else "waiting for data…")
            return

        t_now = max(self.data[i][-1][0] for i in shown)
        t0 = t_now - self.window_s

        if self.ylim:
            lo, hi = self.ylim
        else:
            vals = [v for i in shown for (ts, v) in self.data[i] if ts >= t0]
            if not vals:
                return
            lo, hi = min(vals), max(vals)
            if hi - lo < 1e-12:          # a flat trace still needs a band
                lo, hi = lo - 1.0, hi + 1.0
            pad = (hi - lo) * 0.12
            lo, hi = lo - pad, hi + pad
        if hi - lo < 1e-12:
            hi = lo + 1.0

        def yp(v):
            return pad_t + ph - (v - lo) / (hi - lo) * ph

        def xp(t):
            return pad_l + (t - t0) / self.window_s * pw

        for k in range(5):
            frac = k / 4
            y = pad_t + frac * ph
            self.create_line(pad_l, y, pad_l + pw, y, fill=GRID)
            self.create_text(pad_l - 6, y, anchor="e", fill=MUTED,
                             text=f"{hi - frac * (hi - lo):.4g}",
                             font=("TkFixedFont", 8))
        for k in range(5):
            x = pad_l + k / 4 * pw
            self.create_line(x, pad_t, x, pad_t + ph, fill=GRID)
            self.create_text(x, pad_t + ph + 11, fill=MUTED,
                             text=f"-{self.window_s * (1 - k / 4):.1f}s",
                             font=("TkFixedFont", 8))

        if self.unit:
            self.create_text(pad_l - 6, pad_t - 2, anchor="se", fill=MUTED,
                             text=self.unit, font=("TkFixedFont", 8))

        for i in shown:
            pts = []
            for (t, v) in self.data[i]:
                if t < t0:
                    continue
                pts += [xp(t), max(pad_t, min(pad_t + ph, yp(v)))]
            if len(pts) >= 4:
                self.create_line(*pts, fill=TRACE[i % len(TRACE)],
                                 width=self.width_px, capstyle="round",
                                 joinstyle="round")

        for row, i in enumerate(shown):
            last = self.data[i][-1][1]
            self.create_text(pad_l + pw - 8, pad_t + 10 + row * 15, anchor="e",
                             fill=TRACE[i % len(TRACE)],
                             text=f"{self.labels[i]}  {last: .5g}",
                             font=("TkFixedFont", 9, "bold"))


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("xxCar — sensor calibration")
        self.geometry("1180x740")
        self.minsize(900, 560)
        self.configure(bg=BG)

        self.q: queue.Queue = queue.Queue()
        self.link: Link | None = None
        self.sensors: dict[int, dict] = {}
        self.active: int | None = None
        self.last_seq: dict[int, int] = {}
        self.drops = 0
        self.rx = 0
        self._rx_mark = (time.time(), 0)
        self.rx_hz = 0.0
        self.t_start = time.time()
        self.axis_vars: list[tk.BooleanVar] = []
        self.paused = False

        self._style()
        self._build()
        self.after(40, self._pump)
        self.after(500, self._tick_stats)
        self.protocol("WM_DELETE_WINDOW", self._quit)

    # ---- chrome ---------------------------------------------------------

    def _style(self):
        s = ttk.Style(self)
        try:
            s.theme_use("clam")
        except tk.TclError:
            pass
        s.configure(".", background=BG, foreground=FG,
                    fieldbackground=PANEL, bordercolor=GRID)
        s.configure("TFrame", background=BG)
        s.configure("Card.TFrame", background=PANEL)
        s.configure("TLabel", background=BG, foreground=FG)
        s.configure("Card.TLabel", background=PANEL, foreground=FG)
        s.configure("Muted.TLabel", background=PANEL, foreground=MUTED)
        s.configure("Head.TLabel", background=PANEL, foreground=ACCENT,
                    font=("TkDefaultFont", 9, "bold"))
        s.configure("TButton", background=PANEL, foreground=FG,
                    borderwidth=0, padding=6)
        s.map("TButton", background=[("active", "#232a35")])
        s.configure("Go.TButton", background=ACCENT, foreground="#08111f",
                    font=("TkDefaultFont", 9, "bold"))
        s.map("Go.TButton", background=[("active", "#6fb0ff")])
        s.configure("TCheckbutton", background=PANEL, foreground=FG)
        s.map("TCheckbutton", background=[("active", PANEL)])
        s.configure("Treeview", background=PANEL, fieldbackground=PANEL,
                    foreground=FG, borderwidth=0, rowheight=23)
        s.configure("Treeview.Heading", background="#1d2430", foreground=MUTED,
                    borderwidth=0)
        s.map("Treeview", background=[("selected", "#22406b")],
              foreground=[("selected", "#ffffff")])
        s.configure("TCombobox", fieldbackground=PANEL, background=PANEL)
        s.configure("TScale", background=PANEL)
        s.configure("TLabelframe", background=PANEL, foreground=MUTED,
                    bordercolor=GRID)
        s.configure("TLabelframe.Label", background=PANEL, foreground=MUTED)

    def _build(self):
        # -- top bar
        bar = ttk.Frame(self, style="Card.TFrame", padding=(10, 8))
        bar.pack(fill="x", padx=8, pady=(8, 0))

        ttk.Label(bar, text="PORT", style="Muted.TLabel").pack(side="left")
        self.port_cb = ttk.Combobox(bar, width=20, state="readonly")
        self.port_cb.pack(side="left", padx=(6, 4))
        ttk.Button(bar, text="⟳", width=3,
                   command=self._scan).pack(side="left")
        self.open_btn = ttk.Button(bar, text="Open", style="Go.TButton",
                                   command=self._toggle)
        self.open_btn.pack(side="left", padx=8)

        self.dot = tk.Canvas(bar, width=10, height=10, bg=PANEL,
                             highlightthickness=0)
        self.dot.pack(side="left")
        self._dot("#555")
        self.status = ttk.Label(bar, text="disconnected", style="Muted.TLabel")
        self.status.pack(side="left", padx=6)

        self.stats = ttk.Label(bar, text="", style="Muted.TLabel",
                               font=("TkFixedFont", 9))
        self.stats.pack(side="right")

        # -- body
        body = ttk.Frame(self)
        body.pack(fill="both", expand=True, padx=8, pady=8)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        left = ttk.Frame(body, style="Card.TFrame", padding=8)
        left.grid(row=0, column=0, sticky="ns", padx=(0, 8))
        self._build_sidebar(left)

        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky="nsew")
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)

        self._build_controls(right)
        plot_wrap = ttk.Frame(right, style="Card.TFrame", padding=6)
        plot_wrap.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        self.plot = Strip(plot_wrap)
        self.plot.pack(fill="both", expand=True)

        # -- log
        logf = ttk.Frame(self, style="Card.TFrame", padding=(8, 6))
        logf.pack(fill="x", padx=8, pady=(0, 8))
        head = ttk.Frame(logf, style="Card.TFrame")
        head.pack(fill="x")
        ttk.Label(head, text="LINK", style="Head.TLabel").pack(side="left")
        ttk.Button(head, text="clear",
                   command=lambda: self.log.delete("1.0", "end")
                   ).pack(side="right")
        self.log = tk.Text(logf, height=7, bg=PLOT_BG, fg=MUTED,
                           insertbackground=FG, relief="flat",
                           font=("TkFixedFont", 9))
        self.log.pack(fill="x", pady=(4, 0))
        self.log.tag_config("tx", foreground=ACCENT)
        self.log.tag_config("err", foreground="#ff5c7a")

        self._scan()

    def _build_sidebar(self, p):
        ttk.Label(p, text="SENSORS", style="Head.TLabel").pack(anchor="w")
        self.tree = ttk.Treeview(p, columns=("hz",), show="tree headings",
                                 height=10, selectmode="browse")
        self.tree.heading("#0", text="sensor")
        self.tree.heading("hz", text="Hz")
        self.tree.column("#0", width=160)
        self.tree.column("hz", width=52, anchor="e")
        self.tree.pack(fill="x", pady=(4, 2))
        self.tree.tag_configure("absent", foreground="#4a525f")
        self.tree.bind("<<TreeviewSelect>>", self._select)
        ttk.Button(p, text="Refresh list",
                   command=lambda: self.link and self.link.send("list")
                   ).pack(fill="x", pady=(2, 10))

        self.axis_box = ttk.Labelframe(p, text=" axes ", padding=6)
        self.axis_box.pack(fill="x")
        self.axis_hint = ttk.Label(self.axis_box, text="select a sensor",
                                   style="Muted.TLabel")
        self.axis_hint.pack(anchor="w")

    def _build_controls(self, p):
        c = ttk.Frame(p, style="Card.TFrame", padding=8)
        c.grid(row=0, column=0, sticky="ew")

        # rate
        ttk.Label(c, text="RATE", style="Head.TLabel").grid(row=0, column=0,
                                                            sticky="w")
        self.rate_var = tk.IntVar(value=50)
        rate_cb = ttk.Combobox(c, width=6, state="readonly",
                               textvariable=self.rate_var,
                               values=(1, 5, 10, 20, 50, 100, 200))
        rate_cb.grid(row=1, column=0, padx=(0, 4))
        rate_cb.bind("<<ComboboxSelected>>", lambda _e: self._restream())
        ttk.Label(c, text="Hz", style="Muted.TLabel").grid(row=1, column=1,
                                                           sticky="w")

        # window
        ttk.Label(c, text="WINDOW", style="Head.TLabel").grid(
            row=0, column=2, sticky="w", padx=(18, 0))
        self.win_var = tk.DoubleVar(value=10.0)
        win_cb = ttk.Combobox(c, width=6, state="readonly",
                              textvariable=self.win_var,
                              values=(2, 5, 10, 20, 30, 60))
        win_cb.grid(row=1, column=2, padx=(18, 4))
        win_cb.bind("<<ComboboxSelected>>", lambda _e: self._apply_plot())
        ttk.Label(c, text="s", style="Muted.TLabel").grid(row=1, column=3,
                                                          sticky="w")

        # line width
        ttk.Label(c, text="LINE", style="Head.TLabel").grid(
            row=0, column=4, sticky="w", padx=(18, 0))
        self.lw_var = tk.DoubleVar(value=1.6)
        lw = ttk.Scale(c, from_=0.6, to=4.0, variable=self.lw_var,
                       command=lambda _v: self._apply_plot(), length=90)
        lw.grid(row=1, column=4, padx=(18, 4))

        # y limits
        ttk.Label(c, text="Y AXIS", style="Head.TLabel").grid(
            row=0, column=5, sticky="w", padx=(18, 0))
        yf = ttk.Frame(c, style="Card.TFrame")
        yf.grid(row=1, column=5, sticky="w", padx=(18, 0))
        self.auto_y = tk.BooleanVar(value=True)
        ttk.Checkbutton(yf, text="auto", variable=self.auto_y,
                        command=self._apply_plot).pack(side="left")
        self.ymin = ttk.Entry(yf, width=8)
        self.ymax = ttk.Entry(yf, width=8)
        self.ymin.pack(side="left", padx=(6, 2))
        self.ymax.pack(side="left", padx=2)
        for e in (self.ymin, self.ymax):
            e.bind("<Return>", lambda _e: self._apply_plot())
        ttk.Button(yf, text="fit", width=4,
                   command=self._fit_y).pack(side="left", padx=(4, 0))

        # actions
        af = ttk.Frame(c, style="Card.TFrame")
        af.grid(row=1, column=6, sticky="e", padx=(18, 0))
        c.columnconfigure(6, weight=1)
        self.pause_btn = ttk.Button(af, text="Pause", width=7,
                                    command=self._pause)
        self.pause_btn.pack(side="left", padx=2)
        ttk.Button(af, text="Clear", width=7,
                   command=lambda: self.plot.clear()).pack(side="left", padx=2)

    # ---- helpers --------------------------------------------------------

    def _dot(self, colour):
        self.dot.delete("all")
        self.dot.create_oval(1, 1, 9, 9, fill=colour, outline="")

    def _say(self, msg, tag=None):
        self.log.insert("end", msg + "\n", tag or ())
        self.log.see("end")
        if int(self.log.index("end").split(".")[0]) > 500:
            self.log.delete("1.0", "250.0")

    def _apply_plot(self):
        self.plot.window_s = float(self.win_var.get())
        self.plot.width_px = float(self.lw_var.get())
        if self.auto_y.get():
            self.plot.ylim = None
        else:
            try:
                self.plot.ylim = (float(self.ymin.get()), float(self.ymax.get()))
            except ValueError:
                self.plot.ylim = None      # unparseable: fall back to auto
        self.plot.redraw()

    def _fit_y(self):
        """Freeze the y axis at what is currently on screen.

        Handy for a sensor like the baro, where pressure and temperature differ
        by two orders of magnitude and autoscaling one flattens the other.
        """
        shown = self.plot._shown()
        if not shown:
            return
        vals = [v for i in shown for (_t, v) in self.plot.data[i]]
        if not vals:
            return
        lo, hi = min(vals), max(vals)
        pad = (hi - lo) * 0.1 or 1.0
        self.ymin.delete(0, "end")
        self.ymin.insert(0, f"{lo - pad:.6g}")
        self.ymax.delete(0, "end")
        self.ymax.insert(0, f"{hi + pad:.6g}")
        self.auto_y.set(False)
        self._apply_plot()

    def _pause(self):
        self.paused = not self.paused
        self.pause_btn.config(text="Resume" if self.paused else "Pause")

    # ---- link -----------------------------------------------------------

    def _scan(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_cb["values"] = ports
        acm = [p for p in ports if "ACM" in p]
        if acm:
            self.port_cb.set(acm[0])
        elif ports:
            self.port_cb.set(ports[0])

    def _toggle(self):
        self._close() if self.link else self._open()

    def _open(self):
        port = self.port_cb.get()
        if not port:
            self._say("! no port selected", "err")
            return
        try:
            self.link = Link(port, 115200, self.q)
        except Exception as exc:
            self._say(f"! open failed: {exc}", "err")
            return
        self.link.start()
        self.open_btn.config(text="Close")
        self.status.config(text=port)
        self._dot("#3ddc84")
        self.t_start = time.time()
        # Opening IS the handshake: identify, then ask what there is.
        self.link.send("hello")
        self.after(150, lambda: self.link and self.link.send("list"))

    def _close(self):
        if self.link:
            try:
                self.link.send("stop")
                self.link.send("quit")
                time.sleep(0.12)
            except Exception:
                pass
            self.link.close()
            self.link = None
        self.open_btn.config(text="Open")
        self.status.config(text="disconnected")
        self._dot("#555")
        self.active = None

    def _quit(self):
        self._close()
        self.destroy()

    # ---- sensors --------------------------------------------------------

    def _add_row(self, s):
        iid = str(s["id"])
        tags = () if s["present"] else ("absent",)
        text = s["name"] if s["present"] else f"{s['name']}  ·  absent"
        hz = s.get("rate", 0) if s["present"] else ""
        if self.tree.exists(iid):
            self.tree.item(iid, text=text, values=(hz,), tags=tags)
        else:
            self.tree.insert("", "end", iid=iid, text=text, values=(hz,),
                             tags=tags)

    def _build_axes(self, s):
        for w in self.axis_box.winfo_children():
            w.destroy()
        self.axis_vars = []
        for i, lab in enumerate(s["labels"]):
            v = tk.BooleanVar(value=True)
            self.axis_vars.append(v)
            row = ttk.Frame(self.axis_box, style="Card.TFrame")
            row.pack(fill="x", pady=1)
            sw = tk.Canvas(row, width=12, height=12, bg=PANEL,
                           highlightthickness=0)
            sw.create_rectangle(2, 5, 12, 8, fill=TRACE[i % len(TRACE)],
                                outline="")
            sw.pack(side="left")
            ttk.Checkbutton(row, text=lab, variable=v,
                            command=self._axes_changed).pack(side="left")

    def _axes_changed(self):
        self.plot.visible = [v.get() for v in self.axis_vars]
        self.plot.redraw()

    def _select(self, _evt):
        sel = self.tree.selection()
        if not sel or not self.link:
            return
        s = self.sensors.get(int(sel[0]))
        if not s:
            return
        if not s["present"]:
            self._say(f"! {s['name']} is not publishing — nothing to plot",
                      "err")
            return
        self.active = s["id"]
        self.drops = 0
        self.rx = 0
        self.plot.set_series(s["labels"])
        self.plot.unit = s["unit"]
        self._build_axes(s)
        self._apply_plot()
        self._restream()

    def _restream(self):
        if not self.link or self.active is None:
            return
        s = self.sensors[self.active]
        self.link.send("stop")
        self.link.send(f"stream {s['name']} {int(self.rate_var.get())}")

    # ---- pump -----------------------------------------------------------

    def _pump(self):
        redraw = False
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "json":
                    self._on_json(payload)
                elif kind == "sample":
                    if self._on_sample(payload):
                        redraw = True
                elif kind == "tx":
                    self._say(f"> {payload}", "tx")
                else:
                    self._say(f"! {payload}", "err")
                    if "link lost" in str(payload):
                        self._close()
        except queue.Empty:
            pass
        if redraw and not self.paused:
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
            self._say(f"! protocol mismatch: board {msg.get('proto')}, "
                      f"gui {PROTO}", "err")

    def _on_sample(self, payload):
        sid, seq, _t_us, vals = payload
        if sid != self.active:
            return False
        prev = self.last_seq.get(sid)
        if prev is not None and (prev + 1) & 0xFF != seq:
            self.drops += 1
        self.last_seq[sid] = seq
        self.rx += 1
        if self.paused:
            return False
        self.plot.push(time.time() - self.t_start, vals)
        return True

    def _tick_stats(self):
        now, then = time.time(), self._rx_mark[0]
        if now > then:
            self.rx_hz = (self.rx - self._rx_mark[1]) / (now - then)
        self._rx_mark = (now, self.rx)
        self.stats.config(
            text=f"rx {self.rx:>7d}   {self.rx_hz:5.1f} Hz   drops {self.drops}")
        self.after(500, self._tick_stats)


if __name__ == "__main__":
    App().mainloop()
