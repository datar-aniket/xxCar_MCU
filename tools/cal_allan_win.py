"""Allan variance window for the xxCar calibration GUI.

Opened from the main window, runs standalone against a folder of .ulg parts,
and hands the resulting noise coefficients back to be written to the board.

Kept in its own module because it shares nothing with the live-streaming GUI
but the styling: the analysis is offline, and mixing it into the session window
would put a multi-minute compute on the same event loop that has to keep a
2 kHz stream drawn.
"""

from __future__ import annotations

import json
import math
import queue
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, ttk

sys.path.insert(0, str(Path(__file__).resolve().parent))

import allan  # noqa: E402

# Validated for the dark plot surface #0b0d11 with the dataviz six checks:
# all pairs clear the normal-vision floor (worst 23.0) and the CVD target
# (worst 15.6 protan/deutan, 15.5 tritan), L inside the dark band, contrast
# 4.2:1 or better. Fixed order - x, y, z - never cycled.
AXIS_COLOURS = ("#1ba79f", "#855fd2", "#b16512")
AXIS_NAMES = ("x", "y", "z")

BG, PANEL, PLOT_BG = "#0f1115", "#171b22", "#0b0d11"
GRID, GRID_HI = "#212734", "#2c3444"
FG, MUTED, ACCENT = "#cdd6e0", "#7b8798", "#4c9aff"
WARN = "#e0a030"

# The four channels, and the param prefix each maps to on the board.
PARAM_PREFIX = {
    "accel0": "IMU0_ACC", "gyro0": "IMU0_GYR",
    "accel1": "IMU1_ACC", "gyro1": "IMU1_GYR",
}


class LogLog(tk.Canvas):
    """Log-log Allan deviation plot for one sensor: three axes, one panel.

    Small multiples rather than all twelve curves on one pair of axes - six
    or twelve overlapping traces cannot be told apart no matter how the hues
    are chosen, and the shape of each curve is the thing being read.
    """

    def __init__(self, master, **kw):
        super().__init__(master, bg=PLOT_BG, highlightthickness=0, **kw)
        self.title = ""
        self.unit = ""
        self.curves: list = []          # (tau, adev) per axis
        self.marks: list = []           # (tau_B, adev_min) per axis
        self.bind("<Configure>", lambda _e: self.redraw())

    def set_data(self, title, unit, results):
        self.title = title
        self.unit = unit
        self.curves = [(r.tau, r.adev) for r in results
                       if r.tau is not None and len(r.tau)]
        self.marks = [(r.tau_B, min(r.adev)) for r in results
                      if r.tau is not None and len(r.tau)]
        self.redraw()

    def redraw(self):
        import math
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 80 or h < 70:
            return
        pl, pr, pt, pb = 62, 12, 24, 30
        pw, ph = w - pl - pr, h - pt - pb
        if pw < 30 or ph < 30:
            return

        self.create_text(pl, 12, anchor="w", fill=FG, text=self.title,
                         font=("TkDefaultFont", 9, "bold"))
        if not self.curves:
            self.create_text(w // 2, h // 2, fill=MUTED, font=("TkFixedFont", 8),
                             text="no data")
            return

        xs = [t for tau, _ in self.curves for t in tau if t > 0]
        ys = [a for _, adev in self.curves for a in adev if a > 0]
        if not xs or not ys:
            return
        lx0, lx1 = math.log10(min(xs)), math.log10(max(xs))
        ly0, ly1 = math.log10(min(ys)), math.log10(max(ys))
        if lx1 - lx0 < 1e-9:
            lx1 = lx0 + 1
        pad = (ly1 - ly0) * 0.08 or 0.5
        ly0, ly1 = ly0 - pad, ly1 + pad

        def px(t):
            return pl + (math.log10(t) - lx0) / (lx1 - lx0) * pw

        def py(a):
            return pt + ph - (math.log10(a) - ly0) / (ly1 - ly0) * ph

        # decade gridlines - recessive, and labelled only on the decade
        for d in range(int(math.floor(lx0)), int(math.ceil(lx1)) + 1):
            if not (lx0 <= d <= lx1):
                continue
            x = px(10.0 ** d)
            self.create_line(x, pt, x, pt + ph, fill=GRID)
            self.create_text(x, pt + ph + 12, fill=MUTED,
                             font=("TkFixedFont", 8),
                             text=f"1e{d}" if d else "1")
        for d in range(int(math.floor(ly0)), int(math.ceil(ly1)) + 1):
            if not (ly0 <= d <= ly1):
                continue
            y = py(10.0 ** d)
            self.create_line(pl, y, pl + pw, y, fill=GRID)
            self.create_text(pl - 6, y, anchor="e", fill=MUTED,
                             font=("TkFixedFont", 8), text=f"1e{d}")

        self.create_rectangle(pl, pt, pl + pw, pt + ph, outline=GRID_HI)
        self.create_text(pl + pw // 2, h - 6, fill=MUTED,
                         font=("TkFixedFont", 8), text="averaging time τ (s)")
        self.create_text(pl - 6, pt - 6, anchor="se", fill=MUTED,
                         font=("TkFixedFont", 8), text=f"σ ({self.unit})")

        for i, (tau, adev) in enumerate(self.curves):
            pts = []
            for t, a in zip(tau, adev):
                if t > 0 and a > 0:
                    pts += [px(t), py(a)]
            if len(pts) >= 4:
                self.create_line(*pts, fill=AXIS_COLOURS[i], width=2,
                                 capstyle="round", joinstyle="round")

        # mark the curve minimum: it is where bias instability is read, and
        # seeing it lets you judge whether the run was long enough to reach it
        for i, (tb, amin) in enumerate(self.marks):
            if tb > 0 and amin > 0:
                x, y = px(tb), py(amin)
                self.create_oval(x - 3, y - 3, x + 3, y + 3,
                                 outline=AXIS_COLOURS[i], width=2,
                                 fill=PLOT_BG)

        # direct labels - three series, so no legend box is needed
        for i, name in enumerate(AXIS_NAMES):
            self.create_text(pl + pw - 8, pt + 11 + i * 14, anchor="e",
                             fill=AXIS_COLOURS[i], text=name,
                             font=("TkFixedFont", 9, "bold"))


class AllanWindow(tk.Toplevel):
    def __init__(self, master, on_save=None):
        super().__init__(master)
        self.title("xxCar — Allan variance")
        self.geometry("1180x820")
        self.configure(bg=BG)
        self.on_save = on_save

        self.folder: Path | None = None
        self.sessions: dict = {}
        self.series: dict = {}
        self.results: dict = {}
        self.summary: dict = {}
        self.q: queue.Queue = queue.Queue()
        self.busy = False

        self._build()
        self.after(80, self._pump)

    # ---- layout ---------------------------------------------------------

    def _build(self):
        bar = ttk.Frame(self, style="Card.TFrame", padding=(10, 8))
        bar.pack(fill="x", padx=8, pady=(8, 0))
        ttk.Button(bar, text="Open log folder…",
                   command=self._pick).pack(side="left")
        self.folder_lab = ttk.Label(bar, text="no folder", style="Muted.TLabel")
        self.folder_lab.pack(side="left", padx=10)

        ttk.Label(bar, text="session", style="Head.TLabel").pack(side="left",
                                                                 padx=(16, 4))
        self.sess_cb = ttk.Combobox(bar, width=8, state="readonly")
        self.sess_cb.pack(side="left")
        self.sess_cb.bind("<<ComboboxSelected>>", lambda _e: self._load())

        ctl = ttk.Frame(self, style="Card.TFrame", padding=(10, 8))
        ctl.pack(fill="x", padx=8, pady=(8, 0))

        ttk.Label(ctl, text="TRIM", style="Head.TLabel").grid(row=0, column=0,
                                                              sticky="w")
        ttk.Label(ctl, text="from start", style="Muted.TLabel").grid(
            row=1, column=0, sticky="w")
        self.head = tk.DoubleVar(value=60.0)
        ttk.Spinbox(ctl, from_=0, to=3600, increment=30, width=8,
                    textvariable=self.head).grid(row=1, column=1, padx=(6, 2))
        ttk.Label(ctl, text="s", style="Muted.TLabel").grid(row=1, column=2)

        ttk.Label(ctl, text="from end", style="Muted.TLabel").grid(
            row=1, column=3, sticky="w", padx=(16, 0))
        self.tail = tk.DoubleVar(value=60.0)
        ttk.Spinbox(ctl, from_=0, to=3600, increment=30, width=8,
                    textvariable=self.tail).grid(row=1, column=4, padx=(6, 2))
        ttk.Label(ctl, text="s", style="Muted.TLabel").grid(row=1, column=5)

        ttk.Label(ctl,
                  text="handling and settling at each end are not the noise "
                       "being measured, and they land at long τ where they "
                       "do the most damage",
                  style="Muted.TLabel", wraplength=420,
                  justify="left").grid(row=0, column=6, rowspan=2,
                                       sticky="w", padx=(20, 0))

        self.run_btn = ttk.Button(ctl, text="Compute", style="Go.TButton",
                                  command=self._compute, state="disabled")
        self.run_btn.grid(row=1, column=7, padx=(20, 4))
        ctl.columnconfigure(7, weight=1)

        self.save_btn = ttk.Button(ctl, text="Save to board", command=self._save,
                                   state="disabled")
        self.save_btn.grid(row=1, column=8, padx=4)
        ttk.Button(ctl, text="Close", command=self.destroy).grid(row=1, column=9)

        # dataset health, above the plots: whether the numbers can be believed
        self.health = ttk.Label(self, text="", style="Muted.TLabel",
                                justify="left", wraplength=1100)
        self.health.pack(fill="x", padx=14, pady=(8, 0))

        grid = ttk.Frame(self, style="Card.TFrame", padding=6)
        grid.pack(fill="both", expand=True, padx=8, pady=8)
        self.plots = {}
        for i, name in enumerate(("accel0", "gyro0", "accel1", "gyro1")):
            p = LogLog(grid)
            p.grid(row=i // 2, column=i % 2, sticky="nsew", padx=4, pady=4)
            self.plots[name] = p
        grid.rowconfigure(0, weight=1)
        grid.rowconfigure(1, weight=1)
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        tblf = ttk.Frame(self, style="Card.TFrame", padding=(8, 6))
        tblf.pack(fill="x", padx=8, pady=(0, 8))
        cols = ("N", "Nq", "B", "Bq", "K", "tauB")
        self.tbl = ttk.Treeview(tblf, columns=cols, show="tree headings",
                                height=13)
        self.tbl.heading("#0", text="channel")
        for c, t in zip(cols, ("N (SI)", "N (quoted)", "B (SI)", "B (quoted)",
                               "K (SI)", "τ at min")):
            self.tbl.heading(c, text=t)
            self.tbl.column(c, width=130, anchor="e")
        self.tbl.column("#0", width=110)
        self.tbl.pack(fill="x")

    # ---- loading --------------------------------------------------------

    def _pick(self):
        d = filedialog.askdirectory(title="Folder containing log_NNN_PP.ulg")
        if not d:
            return
        self.folder = Path(d)
        self.sessions = allan.find_sessions(self.folder)
        if not self.sessions:
            self.folder_lab.config(text=f"{d} — no .ulg parts found")
            self.sess_cb["values"] = []
            return
        vals = [f"{s} ({len(p)}p)" for s, p in self.sessions.items()]
        self.sess_cb["values"] = vals
        self.sess_cb.set(vals[-1])
        self.folder_lab.config(
            text=f"{d} — {len(self.sessions)} session(s)")
        self._load()

    def _load(self):
        if not self.sess_cb.get():
            return
        sess = int(self.sess_cb.get().split()[0])
        paths = self.sessions[sess]
        self.health.config(text=f"loading {len(paths)} part(s)…")
        self.run_btn.config(state="disabled")
        self.busy = True

        def work():
            try:
                s = allan.load_session(
                    paths, progress=lambda i, n, nm:
                    self.q.put(("progress", f"loading {i+1}/{n}: {nm}")))
                self.q.put(("loaded", s))
            except Exception as exc:                      # noqa: BLE001
                self.q.put(("error", f"load failed: {exc}"))

        threading.Thread(target=work, daemon=True).start()

    def _describe(self):
        """Say whether the dataset is trustworthy before its numbers are."""
        bits = []
        bad = False
        for name in ("accel0", "gyro0", "accel1", "gyro1"):
            s = self.series.get(name)
            if s is None:
                continue
            hrs = s.duration / 3600.0
            g = len(s.gaps)
            lost = sum(d for _, d in s.gaps)
            frag = f"{name}: {s.fs:.0f} Hz, {hrs:.2f} h, {s.xyz.shape[1]:,} samples"
            if g:
                frag += f", {g} gap(s) totalling {lost:.1f} s"
                bad = True
            bits.append(frag)
        txt = "   |   ".join(bits)
        if bad:
            txt += ("\n⚠  gaps break the uniform sampling Allan variance "
                    "assumes, and bias the long-τ end where bias "
                    "instability is read — treat B and K with suspicion.")
        # a run too short to reach the curve minimum cannot give a real B
        short = [n for n, s in self.series.items() if s.duration < 1800]
        if short:
            txt += (f"\n⚠  {', '.join(short)} shorter than 30 min: the "
                    "curve may not reach its minimum, so B and K are "
                    "extrapolations rather than measurements.")
        self.health.config(text=txt, foreground=WARN if bad or short else MUTED)

    # ---- compute --------------------------------------------------------

    def _compute(self):
        if self.busy or not self.series:
            return
        self.busy = True
        self.run_btn.config(state="disabled")
        self.save_btn.config(state="disabled")
        head, tail = float(self.head.get()), float(self.tail.get())

        def work():
            try:
                out = {}
                for name, s in self.series.items():
                    self.q.put(("progress", f"computing {name}…"))
                    out[name] = allan.analyse(allan.trim(s, head, tail))
                self.q.put(("done", out))
            except Exception as exc:                      # noqa: BLE001
                self.q.put(("error", f"compute failed: {exc}"))

        threading.Thread(target=work, daemon=True).start()

    def _show(self, results):
        self.results = results
        for i in self.tbl.get_children():
            self.tbl.delete(i)
        self.summary = {}

        for name in ("accel0", "gyro0", "accel1", "gyro1"):
            res = results.get(name)
            if not res:
                continue
            s = self.series[name]
            self.plots[name].set_data(f"{name}   ({s.fs:.0f} Hz, "
                                      f"{s.duration/3600:.2f} h)", s.unit, res)
            self.summary[name] = allan.summarise(name, s.unit, res)
            parent = self.tbl.insert("", "end", text=name, values=("",) * 6)
            for ax, r in zip(AXIS_NAMES, res):
                d = self.summary[name]["axes"][ax]
                if s.unit == "rad/s":
                    nq = f"{d['N_deg_sqrt_hr']:.4f} °/√h"
                    bq = f"{d['B_deg_per_hr']:.2f} °/h"
                else:
                    nq = f"{d['N_ug_sqrt_hz']:.1f} µg/√Hz"
                    bq = f"{d['B_ug']:.1f} µg"
                self.tbl.insert(parent, "end", text=f"  {ax}",
                                values=(f"{r.N:.4e}", nq, f"{r.B:.4e}", bq,
                                        f"{r.K:.4e}", f"{r.tau_B:.1f} s"))
            self.tbl.item(parent, open=True)
        self.save_btn.config(state="normal")

    # ---- save -----------------------------------------------------------

    def _save(self):
        """Write a JSON report beside the logs, then hand params to the board.

        The report goes first and unconditionally: it is the durable artefact,
        and it should survive the board being unplugged or the write failing.
        """
        if not self.summary:
            return
        report = {
            "folder": str(self.folder),
            "session": self.sess_cb.get(),
            "trim_head_s": float(self.head.get()),
            "trim_tail_s": float(self.tail.get()),
            "channels": self.summary,
            "diagnostics": {
                n: {"rate_hz": s.fs, "duration_s": s.duration,
                    "samples": int(s.xyz.shape[1]),
                    "gaps": [{"at_s": a, "len_s": d} for a, d in s.gaps]}
                for n, s in self.series.items()
            },
        }
        out = self.folder / "allan_report.json"
        out.write_text(json.dumps(report, indent=2))

        # Mean across the three axes: the EKF takes one number per sensor, and
        # a per-axis spread large enough to matter is a sign the run was bad
        # rather than something to encode.
        #
        # A coefficient the run could not measure is NaN, not zero, and must
        # not be written. A run that never reaches its curve minimum has no
        # rate random walk to report - the board rejects a non-finite float
        # too, but sending one and relying on that would leave the operator
        # thinking it was saved.
        params = {}
        skipped = []
        for name, pfx in PARAM_PREFIX.items():
            res = self.results.get(name)
            if not res:
                continue
            for key, vals in ((f"{pfx}_ND", [r.N for r in res]),
                              (f"{pfx}_RW", [r.K for r in res]),
                              (f"{pfx}_BI", [r.B for r in res])):
                if all(math.isfinite(v) for v in vals):
                    params[key] = sum(vals) / 3.0
                else:
                    skipped.append(key)

        sent = self.on_save(params) if self.on_save else False
        txt = f"report written to {out}"
        txt += (f"   |   {len(params)} params sent to the board" if sent
                else "   |   board not connected — params not written")
        if skipped:
            txt += ("\n⚠  not measurable from this run, left unset: "
                    + ", ".join(skipped)
                    + ".  Rate random walk needs a run long enough to pass "
                      "the curve minimum — roughly 3x the τ shown at the "
                      "marked point.")
        self.health.config(text=txt,
                           foreground=WARN if skipped else MUTED)

    # ---- pump -----------------------------------------------------------

    def _pump(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "progress":
                    self.health.config(text=payload, foreground=MUTED)
                elif kind == "loaded":
                    self.series = payload
                    self.busy = False
                    self._describe()
                    self.run_btn.config(
                        state="normal" if payload else "disabled")
                elif kind == "done":
                    self.busy = False
                    self._show(payload)
                    self.run_btn.config(state="normal")
                    self._describe()
                elif kind == "error":
                    self.busy = False
                    self.run_btn.config(state="normal")
                    self.health.config(text=payload, foreground="#ff6b81")
        except queue.Empty:
            pass
        self.after(80, self._pump)
