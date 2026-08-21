#!/usr/bin/env python3
"""xxCar calibration GUI.

Talks to `cal session` on the board's USB CDC port. Two encodings share the one
pipe and are told apart by the first byte of each message:

    '{'   an ASCII JSON line, terminated by '\\n'  - control and replies
    0xA5  a binary sample frame                   - see Link._emit

NumPy performs the full ellipsoid fit on the host. tkinter draws both plots on
Canvas widgets, avoiding a heavyweight plotting dependency.

Run:  python3 tools/cal_gui.py
"""

import json
import math
import os
import queue
import struct
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk

try:
    import numpy as np
except ImportError:
    raise SystemExit("numpy missing:  pip install numpy")

sys_path = os.path.dirname(os.path.abspath(__file__))
if sys_path not in sys.path:
    sys.path.insert(0, sys_path)
from cal_link import ENC_F32, ENC_I16, HDR, HDR_LEN, SYNC, Link, crc16
from mag_cal import (FitError, MAX_SAMPLES, MIN_SAMPLES, apply_calibration,
                     fit_ellipsoid, validate_corrected)

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    raise SystemExit("pyserial missing:  pip install pyserial")

PROTO = 5

# sync | len(u16) | id | seq | t0_us(u32) | dt_us(u16) | count | nvals | enc

BG, PANEL, PLOT_BG = "#0f1115", "#171b22", "#0b0d11"
GRID, GRID_HI = "#212734", "#2c3444"
FG, MUTED, ACCENT = "#cdd6e0", "#7b8798", "#4c9aff"
TRACE = ["#ff6b81", "#3ddc84", "#57a6ff", "#ffb74d",
         "#c77dff", "#4dd0e1", "#f06292", "#aed581"]

RATES = (1, 5, 10, 20, 50, 100, 200, 400, 500, 1000, 2000)
WINDOWS = (1, 2, 5, 10, 20, 30, 60)

# Index matches the board's cal_accel.h: axis * 2, +1 when that axis reads
# negative. Wording is what the operator does, not what the maths calls it.
POSITIONS = ("X up  (left side down)", "X down (left side up)",
             "Y up  (nose down)",      "Y down (nose up)",
             "Z up  (board level)",    "Z down (upside down)")


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


class MagSphere(tk.Canvas):
    """Interactive orthographic view of tumble coverage and fit residuals."""

    def __init__(self, master, **kw):
        super().__init__(master, bg=PLOT_BG, highlightthickness=0, **kw)
        self.points = np.empty((0, 3), dtype=np.float64)
        self.accepted = None
        self.error = None
        self.coverage = 0.0
        self.yaw, self.pitch = -0.65, 0.45
        self.drag = None
        self.bind("<Configure>", lambda _e: self.redraw())
        self.bind("<ButtonPress-1>", self._press)
        self.bind("<B1-Motion>", self._motion)

    def clear(self):
        self.points = np.empty((0, 3), dtype=np.float64)
        self.accepted = self.error = None
        self.coverage = 0.0
        self.redraw()

    def set_points(self, points, coverage=0.0, accepted=None, error=None):
        p = np.asarray(points, dtype=np.float64)
        self.points = p if p.ndim == 2 and p.shape[1] == 3 else \
            np.empty((0, 3), dtype=np.float64)
        self.coverage = float(coverage)
        self.accepted = None if accepted is None else np.asarray(accepted)
        self.error = None if error is None else np.asarray(error)
        self.redraw()

    def _press(self, event):
        self.drag = (event.x, event.y, self.yaw, self.pitch)

    def _motion(self, event):
        if self.drag:
            x, y, yaw, pitch = self.drag
            self.yaw = yaw + (event.x - x) * 0.012
            self.pitch = max(-1.45, min(1.45,
                             pitch + (event.y - y) * 0.012))
            self.redraw()

    def redraw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        radius = max(10, min(w, h) * 0.39)
        cx, cy = w * 0.5, h * 0.52
        self.create_oval(cx-radius, cy-radius, cx+radius, cy+radius,
                         outline=GRID_HI, width=2)
        self.create_oval(cx-radius, cy-radius*0.28, cx+radius, cy+radius*0.28,
                         outline=GRID)
        self.create_line(cx-radius, cy, cx+radius, cy, fill=GRID)
        self.create_line(cx, cy-radius, cx, cy+radius, fill=GRID)
        self.create_text(10, 10, anchor="nw", fill=MUTED,
                         font=("TkFixedFont", 9),
                         text=f"3D coverage {self.coverage*100:.1f}%\n"
                              "drag to rotate")
        if not len(self.points):
            self.create_text(cx, cy, fill=MUTED, text="start a mag tumble")
            return

        p = self.points.copy()
        # Before fitting, estimate the ellipsoid centre from extrema and show
        # directions. After fitting the points are already centred/corrected.
        if self.accepted is None:
            p -= (np.min(p, axis=0) + np.max(p, axis=0)) * 0.5
        norm = np.linalg.norm(p, axis=1)
        good = np.isfinite(norm) & (norm > 1e-9)
        p[good] /= norm[good, None]
        p = p[good]
        accepted = None if self.accepted is None else self.accepted[good]
        error = None if self.error is None else np.abs(self.error[good])

        cyaw, syaw = math.cos(self.yaw), math.sin(self.yaw)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        rz = np.array(((cyaw, -syaw, 0), (syaw, cyaw, 0), (0, 0, 1)))
        rx = np.array(((1, 0, 0), (0, cp, -sp), (0, sp, cp)))
        p = p @ (rx @ rz).T
        for k in np.argsort(p[:, 2]):
            if accepted is not None and not accepted[k]:
                colour = "#ff5d73"
            elif error is not None:
                ratio = min(1.0, error[k] / 0.03)
                colour = "#3ddc84" if ratio < 0.33 else \
                         ("#ffb74d" if ratio < 0.75 else "#ff6b81")
            else:
                colour = "#57a6ff"
            size = 1.2 + 1.2 * (p[k, 2] + 1.0)
            x, y = cx + radius*p[k, 0], cy - radius*p[k, 1]
            self.create_rectangle(x-size, y-size, x+size, y+size,
                                  fill=colour, outline="")


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
        self.cal_have: set[int] = set()

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
        ttk.Button(bar, text="Allan variance…",
                   command=self._open_allan).pack(side="right", padx=12)

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
        wrap.rowconfigure(0, weight=1)
        wrap.columnconfigure(0, weight=2)
        wrap.columnconfigure(1, weight=1)
        self.plot = Strip(wrap)
        self.plot.grid(row=0, column=0, sticky="nsew")
        self.mag_sphere = MagSphere(wrap, width=360)
        self.mag_sphere.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
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

        # ---- six-position accel calibration
        cb = ttk.Labelframe(p, text=" 6-position accel ", padding=6)
        cb.pack(fill="x", pady=(10, 0))
        self.cal_box = cb
        self.cal_btn = ttk.Button(cb, text="Start calibration",
                                  command=self._cal_start)
        self.cal_btn.pack(fill="x")
        self.cal_hint = ttk.Label(cb, text="select an accelerometer",
                                  style="Muted.TLabel", wraplength=190,
                                  justify="left")
        self.cal_hint.pack(anchor="w", pady=(6, 4))
        self.pos_labels: list[ttk.Label] = []
        for name in POSITIONS:
            lab = ttk.Label(cb, text=f"○  {name}", style="Muted.TLabel")
            lab.pack(anchor="w")
            self.pos_labels.append(lab)
        row = ttk.Frame(cb, style="Card.TFrame")
        row.pack(fill="x", pady=(6, 0))
        self.cap_btn = ttk.Button(row, text="Capture", command=self._cal_cap,
                                  state="disabled")
        self.cap_btn.pack(side="left", expand=True, fill="x", padx=(0, 3))
        self.save_btn = ttk.Button(row, text="Save", command=self._cal_save,
                                   state="disabled")
        self.save_btn.pack(side="left", expand=True, fill="x", padx=(3, 0))

        # ---- gyro zero-rate bias
        # Its own box, and deliberately not a step of the wizard above: there
        # is nothing to reposition between, so folding it into a six-position
        # checklist would imply an order that does not exist.
        gb = ttk.Labelframe(p, text=" gyro bias ", padding=6)
        gb.pack(fill="x", pady=(10, 0))
        self.gyro_btn = ttk.Button(gb, text="Measure gyro bias",
                                   command=self._gyro_start)
        self.gyro_btn.pack(fill="x")
        self.gyro_hint = ttk.Label(gb, text="select a gyroscope",
                                   style="Muted.TLabel", wraplength=190,
                                   justify="left")
        self.gyro_hint.pack(anchor="w", pady=(6, 0))
        # "not steady" is emitted by both the accel capture and the gyro
        # average, so the message alone cannot say which panel it belongs to.
        self.gyro_busy = False

        # ---- full 3D magnetometer ellipsoid
        mb = ttk.Labelframe(p, text=" 3D magnetometer ", padding=6)
        mb.pack(fill="x", pady=(10, 0))
        self.mag_btn = ttk.Button(mb, text="Start tumble",
                                  command=self._mag_start)
        self.mag_btn.pack(fill="x")
        self.mag_progress = ttk.Progressbar(mb, maximum=MAX_SAMPLES, value=0)
        self.mag_progress.pack(fill="x", pady=(6, 3))
        self.mag_hint = ttk.Label(
            mb, text="select mag0", style="Muted.TLabel", wraplength=190,
            justify="left")
        self.mag_hint.pack(anchor="w")
        row = ttk.Frame(mb, style="Card.TFrame")
        row.pack(fill="x", pady=(6, 0))
        self.mag_fit_btn = ttk.Button(row, text="Fit", command=self._mag_fit,
                                      state="disabled")
        self.mag_fit_btn.pack(side="left", expand=True, fill="x",
                              padx=(0, 3))
        self.mag_save_btn = ttk.Button(row, text="Commit",
                                       command=self._mag_save,
                                       state="disabled")
        self.mag_save_btn.pack(side="left", expand=True, fill="x",
                               padx=(3, 0))
        self.mag_active = False
        self.mag_phase = "idle"
        self.mag_samples = []
        self.mag_validation = []
        self.mag_fit = None
        self.mag_ignore_abort_ack = False

        # ---- Allan-variance recording
        rb = ttk.Labelframe(p, text=" record to SD ", padding=6)
        rb.pack(fill="x", pady=(10, 0))
        ttk.Label(rb, text="both IMUs, raw, for Allan variance",
                  style="Muted.TLabel", wraplength=190,
                  justify="left").pack(anchor="w")
        rr = ttk.Frame(rb, style="Card.TFrame")
        rr.pack(fill="x", pady=(4, 0))
        ttk.Label(rr, text="rate", style="Muted.TLabel").pack(side="left")
        self.rec_hz = tk.IntVar(value=200)
        rcb = ttk.Combobox(rr, width=6, state="readonly",
                           values=(50, 100, 200, 500, 1000, 0),
                           textvariable=self.rec_hz)
        rcb.pack(side="left", padx=(5, 0))
        rcb.bind("<<ComboboxSelected>>", lambda _e: self._rec_estimate())
        ttk.Label(rr, text="Hz (0 = native)",
                  style="Muted.TLabel").pack(side="left", padx=(4, 0))
        self.rec_est = ttk.Label(rb, text="", style="Muted.TLabel",
                                 wraplength=190, justify="left")
        self.rec_est.pack(anchor="w", pady=(3, 0))
        row = ttk.Frame(rb, style="Card.TFrame")
        row.pack(fill="x", pady=(6, 0))
        self.rec_btn = ttk.Button(row, text="Start", command=self._rec_toggle)
        self.rec_btn.pack(side="left", expand=True, fill="x", padx=(0, 3))
        ttk.Button(row, text="Status",
                   command=lambda: self.link and self.link.send("record")
                   ).pack(side="left", expand=True, fill="x", padx=(3, 0))
        self.rec_lab = ttk.Label(rb, text="idle", style="Muted.TLabel",
                                 wraplength=190, justify="left")
        self.rec_lab.pack(anchor="w", pady=(5, 0))
        self.recording = False
        self._rec_estimate()

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
        self.cal_on = tk.BooleanVar(value=False)
        ttk.Checkbutton(g, text="calibrated", variable=self.cal_on,
                        command=self._cal_toggle).pack(side="left",
                                                       padx=(10, 0))

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

    def _open_allan(self):
        """Offline analysis in its own window.

        Deliberately usable with no board attached - the logs are on disk and
        the numbers are worth having whether or not the vehicle is plugged in.
        Saving to params is the only part that needs the link.
        """
        try:
            from cal_allan_win import AllanWindow
        except ImportError as exc:
            self._say(f"! Allan window unavailable: {exc}", "err")
            return
        if getattr(self, "_allan", None) and self._allan.winfo_exists():
            self._allan.lift()
            return
        self._allan = AllanWindow(self, on_save=self._save_allan_params)

    def _save_allan_params(self, params: dict) -> bool:
        """Write the noise coefficients to the board, then persist once.

        Returns False rather than raising when nothing is connected, so the
        JSON report still counts as a successful save.
        """
        if not self.link:
            return False
        for name, value in params.items():
            self.link.send(f"set {name} {value:.9g}")
        self.link.send("commit")
        self._say(f"> wrote {len(params)} Allan params and committed", "tx")
        return True

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

    def _axes(self, labels):
        for w in self.axis_box.winfo_children():
            w.destroy()
        self.axis_vars = []
        for i, lab in enumerate(labels):
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
        labels = list(s["labels"])
        if s["name"].startswith("mag"):
            labels.append("|B|")
        self.plot.set_series(labels)
        self.plot.unit = s["unit"]
        self._axes(labels)
        self._apply()
        self._restream()

    def _cal_toggle(self):
        if self.link:
            self.link.send("calib " + ("on" if self.cal_on.get() else "off"))

    def _cal_start(self):
        if not self.link or self.active is None:
            return
        s = self.sensors[self.active]
        if not s["name"].startswith("accel"):
            self._say("! pick an accelerometer first", "err")
            return
        self.cal_have = set()
        for lab, name in zip(self.pos_labels, POSITIONS):
            lab.config(text=f"○  {name}", style="Muted.TLabel")
        self.link.send(f"cal6 start {s['name']}")
        self.cap_btn.config(state="normal")
        self.save_btn.config(state="disabled")
        self.cal_hint.config(
            text=f"calibrating {s['name']} — place the board in any of the six "
                 "orientations, hold it still, then Capture. Order does not "
                 "matter; the board works out which position it is in.")

    def _cal_cap(self):
        if self.link:
            self.cal_hint.config(text="hold still…")
            self.cap_btn.config(state="disabled")
            self.link.send("cal6 capture")
            # the board blocks for up to 4 s while it averages
            self.after(5000, lambda: self.cap_btn.config(state="normal"))

    def _cal_save(self):
        if self.link:
            self.link.send("cal6 save")

    def _gyro_start(self):
        if not self.link or self.active is None:
            return
        s = self.sensors[self.active]
        if not s["name"].startswith("gyro"):
            self._say("! pick a gyroscope first", "err")
            self.gyro_hint.config(text="select a gyroscope in the list above.")
            return
        self.gyro_busy = True
        self.gyro_btn.config(state="disabled")
        self.gyro_hint.config(
            text=f"measuring {s['name']} — put the board down and do not "
                 "touch it for about four seconds.")
        self.link.send(f"gyro {s['name']}")
        # the board blocks for the whole averaging window
        self.after(6000, lambda: self.gyro_btn.config(state="normal"))

    def _mag_start(self):
        if not self.link or self.active is None:
            return
        s = self.sensors[self.active]
        if not s["name"].startswith("mag"):
            self._say("! pick a magnetometer first", "err")
            self.mag_hint.config(text="select mag0 in the sensor list.")
            return
        if self.mag_active:
            self.mag_phase = "aborting"
            self.link.send("mag abort")
            return
        self.mag_active = True
        self.mag_phase = "collect"
        self.mag_samples = []
        self.mag_validation = []
        self.mag_fit = None
        self.mag_sphere.clear()
        self.mag_progress["value"] = 0
        self.mag_progress["maximum"] = MAX_SAMPLES
        self.mag_btn.config(text="Abort")
        self.mag_fit_btn.config(state="disabled")
        self.mag_save_btn.config(state="disabled")
        self.mag_hint.config(
            text=f"Collecting 0/{MIN_SAMPLES}: slowly rotate the complete "
                 "vehicle through every face, edge, and corner.")
        # Fitting requires raw samples even if calibrated preview was selected.
        self.cal_on.set(False)
        self.mag_ignore_abort_ack = True
        self.link.send("mag abort")
        self.link.send("calib off")

    def _mag_fit(self):
        if not self.link or self.mag_phase != "collect":
            return
        self.mag_hint.config(text="fitting full 3D ellipsoid on host…")
        self.mag_fit_btn.config(state="disabled")
        try:
            self.mag_fit = fit_ellipsoid(np.asarray(self.mag_samples))
        except FitError as exc:
            self.mag_hint.config(text=f"✗ {exc.reason}. Continue tumbling; "
                                      "nothing was staged or saved.")
            self.mag_fit_btn.config(state="normal")
            return

        fit = self.mag_fit
        self.mag_sphere.set_points(fit.corrected, fit.coverage,
                                   fit.accepted, fit.radial_error)
        self.mag_phase = "staging"
        self.mag_validation = []
        values = " ".join(f"{v:.9g}" for v in fit.stage_values())
        self.mag_hint.config(
            text=f"fit: {fit.coverage*100:.1f}% coverage, {fit.used} used, "
                 f"{fit.rejected} rejected, RMS {fit.rms:.4f} G. "
                 "Board is checking the candidate…")
        self.link.send("mag stage " + values)

    def _mag_save(self):
        if self.link and self.mag_phase == "ready":
            self.mag_save_btn.config(state="disabled")
            self.link.send("mag commit")

    @staticmethod
    def _mag_coverage(samples):
        if len(samples) < 8:
            return 0.0
        p = np.asarray(samples, dtype=np.float64)
        p -= (np.min(p, axis=0) + np.max(p, axis=0)) * 0.5
        norm = np.linalg.norm(p, axis=1)
        good = norm > 1e-9
        p = p[good] / norm[good, None]
        az = np.clip(np.floor((np.arctan2(p[:, 1], p[:, 0]) + np.pi) *
                             24 / (2*np.pi)).astype(int), 0, 23)
        zb = np.clip(np.floor((p[:, 2] + 1.0) * 6).astype(int), 0, 11)
        return len(np.unique(zb * 24 + az)) / (24 * 12)

    def _rec_estimate(self):
        """Say up front what the run will cost, and when FAT32 stops it.

        A 4 GB per-file ceiling at 227 KB/s is 5.1 hours - an overnight run at
        the native rate ends before morning with nothing to say so.
        """
        hz = int(self.rec_hz.get()) or 2000
        bps = 4 * hz * 29                      # 4 topics x (3 hdr + 2 id + 24)
        cap_h = 4 * 1024 ** 3 / bps / 3600
        self.rec_est.config(
            text=f"{bps/1024:.0f} KB/s · {bps*8*3600/1e9:.2f} GB per 8 h · "
                 f"4 GB file cap at {cap_h:.1f} h")

    def _rec_toggle(self):
        if not self.link:
            return
        if self.recording:
            self.link.send("record stop")
        else:
            self.link.send(f"record start {int(self.rec_hz.get())}")

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
        if evt == "cal6":
            self._cal_progress(msg)
            return
        if evt == "record":
            self._rec_status(msg)
            return
        if evt == "ok":
            what = msg.get("what", "")
            if what == "cal6 save":
                self.cal_hint.config(
                    text=f"saved. residual {msg['residual']:.4f} m/s² "
                         f"(lower is better; under ~0.05 is good)")
                self._say(f"< {json.dumps(msg)}")
                self.cal_on.set(True)
                self._cal_toggle()
                return
            if what == "gyro start":
                self.gyro_hint.config(
                    text=f"averaging {msg.get('name','?')} for "
                         f"{msg.get('secs', 4)} s — hands off.")
                self._say(f"< {json.dumps(msg)}")
                return
            if what == "gyro save":
                self.gyro_busy = False
                b = msg.get("bias", [0, 0, 0])
                self.gyro_hint.config(
                    text=f"saved. bias {b[0]:+.5f} {b[1]:+.5f} {b[2]:+.5f} "
                         f"rad/s from {msg.get('n', 0)} samples.")
                self.gyro_btn.config(state="normal")
                self._say(f"< {json.dumps(msg)}")
                self.cal_on.set(True)
                self._cal_toggle()
                return
            if what == "mag stage":
                self.mag_phase = "validate"
                self.mag_validation = []
                self.mag_progress["maximum"] = 500
                self.mag_progress["value"] = 0
                self.mag_hint.config(
                    text="Candidate passed the board safety checks. Rotate "
                         "again while 500 fresh corrected samples are "
                         "validated; Commit stays locked until they pass.")
                self._say(f"< {json.dumps(msg)}")
                return
            if what == "mag commit":
                self.mag_active = False
                self.mag_phase = "idle"
                self.mag_btn.config(text="Start tumble")
                self.mag_fit_btn.config(state="disabled")
                self.mag_save_btn.config(state="disabled")
                self.mag_hint.config(
                    text=f"committed. field {msg.get('field', 0):.3f} G; "
                         "the validated correction is now persistent.")
                self.cal_on.set(True)
                self._say(f"< {json.dumps(msg)}")
                return
            if what == "mag abort":
                # Starting a new host collection sends a defensive board abort
                # first; do not let its acknowledgement cancel that collection.
                if self.mag_ignore_abort_ack:
                    self.mag_ignore_abort_ack = False
                    return
                self.mag_active = False
                self.mag_phase = "idle"
                self.mag_btn.config(text="Start tumble")
                self.mag_fit_btn.config(state="disabled")
                self.mag_save_btn.config(state="disabled")
                self.mag_hint.config(text="calibration aborted; nothing saved.")
                return
            if what == "record":
                self.recording = True
                self.rec_btn.config(text="Stop")
                self.rec_lab.config(text=f"recording → {msg.get('path','?')}")
            elif what == "record stop":
                self.recording = False
                self.rec_btn.config(text="Start")
                self.rec_lab.config(
                    text=f"stopped. {msg.get('samples',0)} samples, "
                         f"{msg.get('bytes',0)/1e6:.1f} MB, "
                         f"dropped {msg.get('dropped',0)}")
        if evt == "error" and self.mag_active:
            self.mag_hint.config(
                text=f"✗ {msg.get('msg', 'mag calibration failed')} — "
                     "nothing was saved.")
            self.mag_phase = "collect" if self.mag_samples else "idle"
            self.mag_fit_btn.config(state="normal"
                                    if len(self.mag_samples) >= MIN_SAMPLES
                                    else "disabled")
            self.mag_save_btn.config(state="disabled")
            self._say(f"< {json.dumps(msg)}")
            return
        if evt == "error" and self.gyro_busy:
            m = msg.get("msg", "")
            self.gyro_busy = False
            self.gyro_btn.config(state="normal")
            if m == "not steady":
                sd = msg.get("sd", [0, 0, 0])
                self.gyro_hint.config(
                    text="✗ the board moved — worst axis "
                         f"{max(abs(v) for v in sd):.4f} rad/s, limit "
                         f"{msg.get('limit', 0):.3f}.  Put it down and retry.")
            elif m == "still turning":
                # Steady AND rotating: the standard deviation cannot see this,
                # which is exactly why the board checks the magnitude too.
                b = msg.get("bias", [0, 0, 0])
                self.gyro_hint.config(
                    text="✗ the board is rotating, not still — "
                         f"{max(abs(v) for v in b):.3f} rad/s.  "
                         "Nothing was saved.")
            elif m == "too few samples":
                self.gyro_hint.config(
                    text=f"✗ only {msg.get('n', 0)} samples, need "
                         f"{msg.get('need', 0)} — is the gyro streaming?")
            else:
                self.gyro_hint.config(text=f"✗ {m} — nothing was saved.")
            self._say(f"< {json.dumps(msg)}")
            return
        if evt == "error":
            m = msg.get("msg", "")
            if "steady" in m or "square" in m:
                self.cal_hint.config(text=f"✗ {m} — reposition and retry")
            elif m == "fit rejected":
                # A refused save has to be as visible as an accepted one.
                # Left in the log line alone, the operator sees the six ticks
                # still lit, no error on the panel, and concludes it saved.
                self.cal_hint.config(
                    text=f"✗ fit rejected — residual {msg.get('residual', 0):.3f}"
                         f" m/s² exceeds {msg.get('limit', 0):.2f}."
                         "  Redo the six positions on a flat surface.")
                self.save_btn.config(state="normal")
            elif m == "out of range":
                self.cal_hint.config(
                    text=f"✗ {msg.get('param', '?')} = "
                         f"{msg.get('value', 0):.5f} is outside its allowed "
                         "range — nothing was saved.")
                self.save_btn.config(state="normal")
        self._say(f"< {json.dumps(msg)}")
        if evt == "hello" and msg.get("proto") != PROTO:
            self._say(f"! protocol mismatch: board {msg.get('proto')}, "
                      f"gui {PROTO}", "err")

    def _cal_progress(self, msg):
        pos = msg["pos"]
        self.cal_have.add(pos)
        self.pos_labels[pos].config(text=f"●  {POSITIONS[pos]}",
                                    style="Card.TLabel")
        left = msg["need"] - msg["have"]
        self.cal_hint.config(
            text=f"captured {POSITIONS[pos]} — {msg['have']}/{msg['need']} done"
                 + ("" if left else ".  Ready to Save."))
        if not left:
            self.save_btn.config(state="normal")
        self._say(f"< cal6 pos {pos} a={[round(v,3) for v in msg['a']]}")

    def _rec_status(self, msg):
        self.recording = bool(msg.get("running"))
        self.rec_btn.config(text="Stop" if self.recording else "Start")
        if self.recording:
            self.rec_lab.config(
                text=f"{msg.get('path','?')}\n{msg.get('samples',0)} samples, "
                     f"{msg.get('bytes',0)/1e6:.1f} MB, "
                     f"dropped {msg.get('dropped',0)}")
        else:
            self.rec_lab.config(text="idle")

    def _on_batch(self, payload):
        sid, seq, t0, dt, rows = payload
        if sid != self.active:
            return False
        sensor = self.sensors.get(sid)
        if sensor and sensor["name"].startswith("mag"):
            xyz = [list(row[:3]) for row in rows]
            if self.mag_phase == "collect":
                room = MAX_SAMPLES - len(self.mag_samples)
                if room > 0:
                    self.mag_samples.extend(xyz[:room])
                count = len(self.mag_samples)
                self.mag_progress["value"] = count
                self.mag_fit_btn.config(
                    state="normal" if count >= MIN_SAMPLES else "disabled")
                if count == MAX_SAMPLES or count % 250 < len(rows):
                    coverage = self._mag_coverage(self.mag_samples)
                    self.mag_sphere.set_points(self.mag_samples, coverage)
                    self.mag_hint.config(
                        text=f"{count}/{MIN_SAMPLES} samples "
                             f"({coverage*100:.1f}% spherical coverage). "
                             + ("Fit is available; keep moving toward 6000 "
                                "if the sphere still has holes."
                                if count >= MIN_SAMPLES else
                                "Cover every face, edge, and corner."))
            elif self.mag_phase == "validate" and self.mag_fit is not None:
                room = 500 - len(self.mag_validation)
                if room > 0:
                    self.mag_validation.extend(xyz[:room])
                    self.mag_progress["value"] = len(self.mag_validation)
                if len(self.mag_validation) == 500:
                    quality = validate_corrected(self.mag_validation,
                                                 self.mag_fit.field)
                    limit = min(0.030, 0.07 * self.mag_fit.field)
                    passed = (quality["rms"] <= limit and
                              quality["maximum"] <= 0.100)
                    self.mag_phase = "ready" if passed else "failed"
                    self.mag_save_btn.config(state="normal" if passed
                                              else "disabled")
                    self.mag_hint.config(
                        text=("Preview passed" if passed else
                              "✗ Preview failed") +
                             f": fresh RMS {quality['rms']:.4f} G "
                             f"(limit {limit:.4f}), max "
                             f"{quality['maximum']:.4f} G (limit 0.1000). " +
                             ("Review the sphere, then Commit."
                              if passed else
                              "Abort and recollect away from magnetic "
                              "interference; nothing was saved."))
            rows = [list(row) + [math.sqrt(sum(v * v for v in row[:3]))]
                    for row in rows]
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
