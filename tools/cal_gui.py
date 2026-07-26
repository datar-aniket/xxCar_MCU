#!/usr/bin/env python3
"""xxCar calibration GUI.

Talks to `cal session` on the board's USB CDC port. Two encodings share the one
pipe and are told apart by the first byte of each message:

    '{'   an ASCII JSON line, terminated by '\\n'  - control and replies
    0xA5  a binary sample frame                   - see Link._emit

Only stdlib plus pyserial: tkinter draws the plot on a Canvas rather than
pulling in matplotlib, which keeps the install to one package and lets the
renderer decimate exactly how a dense time series needs.

Run:  python3 tools/cal_gui.py
"""

import json
import queue
import struct
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    raise SystemExit("pyserial missing:  pip install pyserial")

SYNC = 0xA5
PROTO = 2
ENC_I16, ENC_F32 = 0, 1

# sync | len(u16) | id | seq | t0_us(u32) | dt_us(u16) | count | nvals | enc
HDR = struct.Struct("<BHBBIHBBB")
HDR_LEN = HDR.size                      # 14

BG, PANEL, PLOT_BG = "#0f1115", "#171b22", "#0b0d11"
GRID, GRID_HI = "#212734", "#2c3444"
FG, MUTED, ACCENT = "#cdd6e0", "#7b8798", "#4c9aff"
TRACE = ["#ff6b81", "#3ddc84", "#57a6ff", "#ffb74d",
         "#c77dff", "#4dd0e1", "#f06292", "#aed581"]

RATES = (1, 5, 10, 20, 50, 100, 200, 400, 500, 1000, 2000)
WINDOWS = (1, 2, 5, 10, 20, 30, 60)


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


class Strip(tk.Canvas):
    """Live strip chart.

    Times and values are kept in parallel flat lists rather than tuples: at
    2 kHz a ten second window is 20 000 samples per axis, and the difference
    between a list of floats and a list of pairs is real memory and real
    allocation churn.

    Rendering decimates with a min/max envelope, one vertical span per pixel
    column. Drawing 20 000 points into a 900 pixel canvas is both slow and a
    lie - a stride would drop exactly the noise spikes you are looking for,
    while min/max keeps the envelope honest.
    """

    def __init__(self, master, **kw):
        super().__init__(master, bg=PLOT_BG, highlightthickness=0, **kw)
        self.labels: list[str] = []
        self.visible: list[bool] = []
        self.t: list[float] = []
        self.v: list[list[float]] = []
        self.window_s = 10.0
        self.width_px = 1.5
        self.ylim = None                # None = autoscale
        self.unit = ""
        self.font = ("TkFixedFont", 8)
        self.bind("<Configure>", lambda _e: self.redraw())

    def set_series(self, labels):
        self.labels = list(labels)
        self.visible = [True] * len(labels)
        self.clear()

    def clear(self):
        self.t = []
        self.v = [[] for _ in self.labels]
        self.redraw()

    def push_many(self, times, rows):
        """Append a decoded batch. rows[k] is the value list for times[k]."""
        if not self.v:
            return
        self.t.extend(times)
        for i, col in enumerate(self.v):
            col.extend(r[i] if i < len(r) else 0.0 for r in rows)

        cutoff = self.t[-1] - self.window_s * 1.15
        if self.t[0] < cutoff:
            # one bisect beats scanning; the list is sorted by construction
            lo, hi = 0, len(self.t)
            while lo < hi:
                mid = (lo + hi) // 2
                if self.t[mid] < cutoff:
                    lo = mid + 1
                else:
                    hi = mid
            if lo:
                del self.t[:lo]
                for col in self.v:
                    del col[:lo]

    def shown(self):
        return [i for i, ok in enumerate(self.visible)
                if ok and i < len(self.v) and self.v[i]]

    def redraw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 60 or h < 50:
            return

        pl, pr, pt, pb = 64, 14, 12, 24
        pw, ph = w - pl - pr, h - pt - pb
        if pw < 20 or ph < 20:
            return
        self.create_rectangle(pl, pt, pl + pw, pt + ph, outline=GRID_HI)

        shown = self.shown()
        if not shown or not self.t:
            self.create_text(w // 2, h // 2, fill=MUTED, font=self.font,
                             text="no axis selected" if self.labels
                             else "waiting for data…")
            return

        t_end = self.t[-1]
        t0 = t_end - self.window_s

        # first index inside the window
        lo, hi = 0, len(self.t)
        while lo < hi:
            mid = (lo + hi) // 2
            if self.t[mid] < t0:
                lo = mid + 1
            else:
                hi = mid
        i0 = lo
        npts = len(self.t) - i0
        if npts < 1:
            return

        if self.ylim:
            ylo, yhi = self.ylim
        else:
            ylo, yhi = None, None
            for i in shown:
                col = self.v[i]
                for k in range(i0, len(col)):
                    x = col[k]
                    if ylo is None or x < ylo:
                        ylo = x
                    if yhi is None or x > yhi:
                        yhi = x
            if ylo is None:
                return
            if yhi - ylo < 1e-12:
                ylo, yhi = ylo - 1.0, yhi + 1.0
            pad = (yhi - ylo) * 0.12
            ylo, yhi = ylo - pad, yhi + pad
        if yhi - ylo < 1e-12:
            yhi = ylo + 1.0
        span = yhi - ylo

        for k in range(5):
            f = k / 4
            y = pt + f * ph
            self.create_line(pl, y, pl + pw, y, fill=GRID)
            self.create_text(pl - 7, y, anchor="e", fill=MUTED, font=self.font,
                             text=f"{yhi - f * span:.4g}")
            x = pl + f * pw
            self.create_line(x, pt, x, pt + ph, fill=GRID)
            self.create_text(x, pt + ph + 12, fill=MUTED, font=self.font,
                             text=f"−{self.window_s * (1 - f):.1f}s")
        if self.unit:
            self.create_text(pl - 7, pt - 3, anchor="se", fill=MUTED,
                             font=self.font, text=self.unit)

        def ypix(val):
            y = pt + ph - (val - ylo) / span * ph
            return pt if y < pt else (pt + ph if y > pt + ph else y)

        cols = max(1, min(pw, 1400))
        per = npts / cols

        for i in shown:
            col = self.v[i]
            colour = TRACE[i % len(TRACE)]
            if per <= 1.5:
                pts = []
                sx = pw / max(npts - 1, 1)
                for k in range(i0, len(col)):
                    pts.append(pl + (k - i0) * sx)
                    pts.append(ypix(col[k]))
                if len(pts) >= 4:
                    self.create_line(*pts, fill=colour, width=self.width_px,
                                     capstyle="round", joinstyle="round")
            else:
                # min/max envelope: one vertical span per pixel column
                for c in range(cols):
                    a = i0 + int(c * per)
                    b = i0 + int((c + 1) * per)
                    if b <= a:
                        b = a + 1
                    if b > len(col):
                        b = len(col)
                    if a >= b:
                        break
                    seg = col[a:b]
                    x = pl + c * (pw / cols)
                    self.create_line(x, ypix(min(seg)), x, ypix(max(seg)),
                                     fill=colour, width=self.width_px)

        for row, i in enumerate(shown):
            self.create_text(pl + pw - 8, pt + 11 + row * 15, anchor="e",
                             fill=TRACE[i % len(TRACE)],
                             font=("TkFixedFont", 9, "bold"),
                             text=f"{self.labels[i]}  {self.v[i][-1]: .5g}")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("xxCar — sensor calibration")
        self.geometry("1240x780")
        self.minsize(880, 540)
        self.configure(bg=BG)

        self.q: queue.Queue = queue.Queue()
        self.link: Link | None = None
        self.sensors: dict[int, dict] = {}
        self.active: int | None = None
        self.last_seq: dict[int, int] = {}
        self.drops = self.rx = 0
        self._mark = (time.time(), 0)
        self.t_start = time.time()
        self.axis_vars: list[tk.BooleanVar] = []
        self.paused = False

        self._fonts()
        self._style()
        self._build()
        self.after(50, self._pump)
        self.after(500, self._stats)
        self.protocol("WM_DELETE_WINDOW", self._quit)

    # ---- chrome ---------------------------------------------------------

    def _fonts(self):
        """Pin the named fonts.

        Left to the system these inherit the desktop's scaling, which on a
        HiDPI setup renders the sidebar text far larger than its column and
        clips it. Fixing the sizes here makes the layout the same everywhere.
        """
        for name, size in (("TkDefaultFont", 9), ("TkTextFont", 9),
                           ("TkFixedFont", 9), ("TkMenuFont", 9),
                           ("TkHeadingFont", 9)):
            try:
                tkfont.nametofont(name).configure(size=size)
            except tk.TclError:
                pass

    def _style(self):
        s = ttk.Style(self)
        try:
            s.theme_use("clam")
        except tk.TclError:
            pass
        s.configure(".", background=BG, foreground=FG,
                    fieldbackground=PANEL, bordercolor=GRID_HI,
                    lightcolor=PANEL, darkcolor=PANEL)
        s.configure("TFrame", background=BG)
        s.configure("Card.TFrame", background=PANEL)
        s.configure("TLabel", background=BG, foreground=FG)
        s.configure("Card.TLabel", background=PANEL, foreground=FG)
        s.configure("Muted.TLabel", background=PANEL, foreground=MUTED)
        s.configure("Head.TLabel", background=PANEL, foreground=MUTED,
                    font=("TkDefaultFont", 8, "bold"))
        s.configure("TButton", background="#222835", foreground=FG,
                    borderwidth=0, padding=(8, 4))
        s.map("TButton", background=[("active", "#2c3444")])
        s.configure("Go.TButton", background=ACCENT, foreground="#08111f",
                    font=("TkDefaultFont", 9, "bold"), padding=(14, 4))
        s.map("Go.TButton", background=[("active", "#6fb0ff")])
        s.configure("TCheckbutton", background=PANEL, foreground=FG)
        s.map("TCheckbutton", background=[("active", PANEL)])
        s.configure("Treeview", background=PANEL, fieldbackground=PANEL,
                    foreground=FG, borderwidth=0, rowheight=24)
        s.configure("Treeview.Heading", background="#222835",
                    foreground=MUTED, borderwidth=0, padding=(4, 3))
        s.map("Treeview", background=[("selected", "#274a7d")],
              foreground=[("selected", "#ffffff")])
        s.configure("TCombobox", fieldbackground=PANEL, background=PANEL,
                    foreground=FG, arrowcolor=FG, padding=(4, 2))
        s.configure("TEntry", fieldbackground=PANEL, foreground=FG,
                    padding=(4, 2))
        s.configure("TLabelframe", background=PANEL, bordercolor=GRID_HI)
        s.configure("TLabelframe.Label", background=PANEL, foreground=MUTED,
                    font=("TkDefaultFont", 8, "bold"))
        s.configure("TPanedwindow", background=BG)
        s.configure("Sash", sashthickness=6, gripcount=0)

    def _build(self):
        bar = ttk.Frame(self, style="Card.TFrame", padding=(10, 7))
        bar.pack(fill="x", padx=8, pady=(8, 0))
        ttk.Label(bar, text="PORT", style="Head.TLabel").pack(side="left")
        self.port_cb = ttk.Combobox(bar, width=18, state="readonly")
        self.port_cb.pack(side="left", padx=(6, 3))
        ttk.Button(bar, text="⟳", width=2,
                   command=self._scan).pack(side="left")
        self.open_btn = ttk.Button(bar, text="Open", style="Go.TButton",
                                   command=self._toggle)
        self.open_btn.pack(side="left", padx=10)
        self.dot = tk.Canvas(bar, width=9, height=9, bg=PANEL,
                             highlightthickness=0)
        self.dot.pack(side="left")
        self._dot("#4a525f")
        self.status = ttk.Label(bar, text="disconnected", style="Muted.TLabel")
        self.status.pack(side="left", padx=6)
        self.stats = ttk.Label(bar, text="", style="Muted.TLabel",
                               font=("TkFixedFont", 9))
        self.stats.pack(side="right")

        # a real paned window, so the sidebar can be dragged to fit its content
        self.pane = ttk.Panedwindow(self, orient="horizontal")
        self.pane.pack(fill="both", expand=True, padx=8, pady=8)

        side = ttk.Frame(self.pane, style="Card.TFrame", padding=8)
        self._sidebar(side)
        self.pane.add(side, weight=0)

        right = ttk.Frame(self.pane)
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)
        self._controls(right)
        wrap = ttk.Frame(right, style="Card.TFrame", padding=6)
        wrap.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        self.plot = Strip(wrap)
        self.plot.pack(fill="both", expand=True)
        self.pane.add(right, weight=1)

        logf = ttk.Frame(self, style="Card.TFrame", padding=(8, 6))
        logf.pack(fill="x", padx=8, pady=(0, 8))
        head = ttk.Frame(logf, style="Card.TFrame")
        head.pack(fill="x")
        ttk.Label(head, text="LINK", style="Head.TLabel").pack(side="left")
        ttk.Button(head, text="clear",
                   command=lambda: self.log.delete("1.0", "end")
                   ).pack(side="right")
        self.log = tk.Text(logf, height=6, bg=PLOT_BG, fg=MUTED, relief="flat",
                           insertbackground=FG, font=("TkFixedFont", 9))
        self.log.pack(fill="x", pady=(4, 0))
        self.log.tag_config("tx", foreground=ACCENT)
        self.log.tag_config("err", foreground="#ff6b81")
        self._scan()

    def _sidebar(self, p):
        ttk.Label(p, text="SENSORS", style="Head.TLabel").pack(anchor="w")
        self.tree = ttk.Treeview(p, columns=("hz",), show="tree headings",
                                 height=9, selectmode="browse")
        self.tree.heading("#0", text="sensor", anchor="w")
        self.tree.heading("hz", text="Hz", anchor="e")
        # stretch so dragging the sash actually widens the name column
        self.tree.column("#0", width=140, minwidth=90, stretch=True)
        self.tree.column("hz", width=64, minwidth=48, anchor="e",
                         stretch=False)
        self.tree.pack(fill="both", expand=True, pady=(4, 2))
        self.tree.tag_configure("absent", foreground="#4a525f")
        self.tree.bind("<<TreeviewSelect>>", self._select)
        ttk.Button(p, text="Refresh list",
                   command=lambda: self.link and self.link.send("list")
                   ).pack(fill="x", pady=(2, 10))
        self.axis_box = ttk.Labelframe(p, text=" axes ", padding=6)
        self.axis_box.pack(fill="x")
        ttk.Label(self.axis_box, text="select a sensor",
                  style="Muted.TLabel").pack(anchor="w")

    def _controls(self, p):
        c = ttk.Frame(p, style="Card.TFrame", padding=(10, 8))
        c.grid(row=0, column=0, sticky="ew")

        def group(col, title):
            ttk.Label(c, text=title, style="Head.TLabel").grid(
                row=0, column=col, sticky="w", padx=(0 if col == 0 else 16, 0))
            f = ttk.Frame(c, style="Card.TFrame")
            f.grid(row=1, column=col, sticky="w",
                   padx=(0 if col == 0 else 16, 0), pady=(3, 0))
            return f

        g = group(0, "RATE")
        self.rate_var = tk.IntVar(value=50)
        cb = ttk.Combobox(g, width=6, state="readonly", values=RATES,
                          textvariable=self.rate_var)
        cb.pack(side="left")
        cb.bind("<<ComboboxSelected>>", lambda _e: self._restream())
        ttk.Label(g, text="Hz", style="Muted.TLabel").pack(side="left",
                                                           padx=(4, 0))

        g = group(1, "WINDOW")
        self.win_var = tk.DoubleVar(value=10.0)
        cb = ttk.Combobox(g, width=5, state="readonly", values=WINDOWS,
                          textvariable=self.win_var)
        cb.pack(side="left")
        cb.bind("<<ComboboxSelected>>", lambda _e: self._apply())
        ttk.Label(g, text="s", style="Muted.TLabel").pack(side="left",
                                                          padx=(4, 0))

        g = group(2, "LINE")
        self.lw_var = tk.DoubleVar(value=1.5)
        ttk.Scale(g, from_=0.6, to=4.0, variable=self.lw_var, length=80,
                  command=lambda _v: self._apply()).pack(side="left")

        g = group(3, "Y AXIS")
        self.auto_y = tk.BooleanVar(value=True)
        ttk.Checkbutton(g, text="auto", variable=self.auto_y,
                        command=self._apply).pack(side="left")
        self.ymin = ttk.Entry(g, width=9)
        self.ymax = ttk.Entry(g, width=9)
        self.ymin.pack(side="left", padx=(6, 3))
        self.ymax.pack(side="left", padx=3)
        for e in (self.ymin, self.ymax):
            e.bind("<Return>", lambda _e: self._apply())
        ttk.Button(g, text="fit", width=4,
                   command=self._fit_y).pack(side="left", padx=(4, 0))

        act = ttk.Frame(c, style="Card.TFrame")
        act.grid(row=1, column=9, sticky="e", pady=(3, 0))
        c.columnconfigure(9, weight=1)
        self.pause_btn = ttk.Button(act, text="Pause", width=7,
                                    command=self._pause)
        self.pause_btn.pack(side="left", padx=3)
        ttk.Button(act, text="Clear", width=7,
                   command=lambda: self.plot.clear()).pack(side="left", padx=3)

    # ---- helpers --------------------------------------------------------

    def _dot(self, colour):
        self.dot.delete("all")
        self.dot.create_oval(0, 0, 9, 9, fill=colour, outline="")

    def _say(self, msg, tag=None):
        self.log.insert("end", msg + "\n", tag or ())
        self.log.see("end")
        if int(self.log.index("end").split(".")[0]) > 400:
            self.log.delete("1.0", "200.0")

    def _apply(self):
        self.plot.window_s = float(self.win_var.get())
        self.plot.width_px = float(self.lw_var.get())
        if self.auto_y.get():
            self.plot.ylim = None
        else:
            try:
                self.plot.ylim = (float(self.ymin.get()),
                                  float(self.ymax.get()))
            except ValueError:
                self.plot.ylim = None       # unparseable: fall back to auto
        self.plot.redraw()

    def _fit_y(self):
        """Freeze the y axis at what is currently on screen.

        Needed whenever one sensor's values differ by orders of magnitude - a
        baro reporting 1013 hPa beside 40 degC autoscales to a range that
        flattens both traces into straight lines.
        """
        shown = self.plot.shown()
        if not shown or not self.plot.t:
            return
        vals = [x for i in shown for x in self.plot.v[i]]
        lo, hi = min(vals), max(vals)
        pad = (hi - lo) * 0.1 or 1.0
        for e, val in ((self.ymin, lo - pad), (self.ymax, hi + pad)):
            e.delete(0, "end")
            e.insert(0, f"{val:.6g}")
        self.auto_y.set(False)
        self._apply()

    def _pause(self):
        self.paused = not self.paused
        self.pause_btn.config(text="Resume" if self.paused else "Pause")

    # ---- link -----------------------------------------------------------

    def _scan(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_cb["values"] = ports
        acm = [p for p in ports if "ACM" in p]
        self.port_cb.set(acm[0] if acm else (ports[0] if ports else ""))

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
        self._dot("#4a525f")
        self.active = None

    def _quit(self):
        self._close()
        self.destroy()

    # ---- sensors --------------------------------------------------------

    def _row(self, s):
        iid = str(s["id"])
        tags = () if s["present"] else ("absent",)
        text = s["name"] if s["present"] else f"{s['name']}  · absent"
        hz = s.get("rate", 0) if s["present"] else ""
        if self.tree.exists(iid):
            self.tree.item(iid, text=text, values=(hz,), tags=tags)
        else:
            self.tree.insert("", "end", iid=iid, text=text, values=(hz,),
                             tags=tags)

    def _axes(self, s):
        for w in self.axis_box.winfo_children():
            w.destroy()
        self.axis_vars = []
        for i, lab in enumerate(s["labels"]):
            var = tk.BooleanVar(value=True)
            self.axis_vars.append(var)
            row = ttk.Frame(self.axis_box, style="Card.TFrame")
            row.pack(fill="x", pady=1)
            sw = tk.Canvas(row, width=14, height=12, bg=PANEL,
                           highlightthickness=0)
            sw.create_rectangle(1, 5, 13, 8, fill=TRACE[i % len(TRACE)],
                                outline="")
            sw.pack(side="left")
            ttk.Checkbutton(row, text=lab, variable=var,
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
        self.drops = self.rx = 0
        self.plot.set_series(s["labels"])
        self.plot.unit = s["unit"]
        self._axes(s)
        self._apply()
        self._restream()

    def _restream(self):
        if not self.link or self.active is None:
            return
        s = self.sensors[self.active]
        self.plot.clear()
        self.link.send("stop")
        self.link.send(f"stream {s['name']} {int(self.rate_var.get())}")

    # ---- pump -----------------------------------------------------------

    def _pump(self):
        redraw = False
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "batch":
                    redraw |= self._on_batch(payload)
                elif kind == "json":
                    self._on_json(payload)
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
        self.after(50, self._pump)

    def _on_json(self, msg):
        evt = msg.get("evt")
        if evt == "sensor":
            self.sensors[msg["id"]] = msg
            if self.link:
                self.link.scales[msg["id"]] = msg.get("scale", 1.0) or 1.0
            self._row(msg)
            return
        self._say(f"< {json.dumps(msg)}")
        if evt == "hello" and msg.get("proto") != PROTO:
            self._say(f"! protocol mismatch: board {msg.get('proto')}, "
                      f"gui {PROTO}", "err")

    def _on_batch(self, payload):
        sid, seq, t0, dt, rows = payload
        if sid != self.active:
            return False
        prev = self.last_seq.get(sid)
        if prev is not None and (prev + 1) & 0xFF != seq:
            self.drops += 1
        self.last_seq[sid] = seq
        self.rx += len(rows)
        if self.paused:
            return False
        base = time.time() - self.t_start
        step = (dt or 1000) / 1e6
        n = len(rows)
        # anchor the batch so its LAST sample is now; t0 is board time
        times = [base - (n - 1 - k) * step for k in range(n)]
        self.plot.push_many(times, rows)
        return True

    def _stats(self):
        now, then = time.time(), self._mark[0]
        hz = (self.rx - self._mark[1]) / (now - then) if now > then else 0.0
        self._mark = (now, self.rx)
        self.stats.config(text=f"rx {self.rx:>8d}   {hz:7.1f} Hz   "
                               f"drops {self.drops}")
        self.after(500, self._stats)


if __name__ == "__main__":
    App().mainloop()
