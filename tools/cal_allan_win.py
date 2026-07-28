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
import time
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
        self.curves: list = []          # (axis, tau, adev)
        self.marks: dict = {}           # axis -> (tau_B, adev_min)
        self._plot_box = None
        self._hover_points: list = []   # (px, py, axis, tau, adev)
        self.bind("<Configure>", lambda _e: self.redraw())
        self.bind("<Motion>", self._motion)
        self.bind("<Leave>", lambda _e: self.delete("hover"))

    def set_data(self, title, unit, results):
        self.title = title
        self.unit = unit
        self.curves = [(i, r.tau, r.adev) for i, r in enumerate(results)
                       if r.tau is not None and len(r.tau)]
        self.marks = {
            i: (r.tau_B, min(r.adev))
            for i, r in enumerate(results)
            if r.tau is not None and len(r.tau) and
            math.isfinite(r.tau_B)
        }
        self.redraw()

    @staticmethod
    def _time_label(value):
        if value < 1e-3:
            return f"{value * 1e6:g} µs"
        if value < 1.0:
            return f"{value * 1e3:g} ms"
        if value < 60.0:
            return f"{value:g} s"
        if value < 3600.0:
            return f"{value / 60.0:g} min"
        return f"{value / 3600.0:g} h"

    def _motion(self, event):
        """Show exact curve values without putting text inside the plot."""
        self.delete("hover")
        if not self._plot_box or not self._hover_points:
            return
        left, top, right, bottom = self._plot_box
        if not (left <= event.x <= right and top <= event.y <= bottom):
            return

        point = min(self._hover_points,
                    key=lambda p: (p[0] - event.x) ** 2 +
                                  (p[1] - event.y) ** 2)
        if (point[0] - event.x) ** 2 + (point[1] - event.y) ** 2 > 18 ** 2:
            return

        x, y, axis, tau, adev = point
        self.create_line(x, top, x, bottom, fill=GRID_HI, dash=(2, 4),
                         tags="hover")
        self.create_oval(x - 4, y - 4, x + 4, y + 4,
                         outline=AXIS_COLOURS[axis], width=2,
                         fill=PLOT_BG, tags="hover")
        label = (f"{AXIS_NAMES[axis].upper()}   "
                 f"τ {self._time_label(tau)}   "
                 f"σ {adev:.3e} {self.unit}")
        tx = min(event.x + 14, self.winfo_width() - 12)
        anchor = "ne" if tx > self.winfo_width() - 230 else "nw"
        text_id = self.create_text(
            tx, max(top + 8, event.y - 12), anchor=anchor, text=label,
            fill=FG, font=("TkFixedFont", 8), tags="hover")
        box = self.bbox(text_id)
        if box:
            rect = self.create_rectangle(
                box[0] - 6, box[1] - 4, box[2] + 6, box[3] + 4,
                fill=PANEL, outline=GRID_HI, tags="hover")
            self.tag_lower(rect, text_id)

    def redraw(self):
        self.delete("all")
        self._hover_points = []
        self._plot_box = None
        w, h = self.winfo_width(), self.winfo_height()
        if w < 260 or h < 180:
            return

        # Header and legend have their own band above the plotting rectangle.
        # No text is placed against the border, which avoids the old legend
        # and title bleeding into the top grid line.

        pl, pr, pt, pb = 76, 18, 42, 42
        pw, ph = w - pl - pr, h - pt - pb
        if pw < 120 or ph < 80:
            return

        self.create_text(pl, 16, anchor="w", fill=FG, text=self.title,
                         font=("TkDefaultFont", 10, "bold"))

        legend_x = w - 150
        for i, name in enumerate(AXIS_NAMES):
            x = legend_x + i * 47
            self.create_line(x, 16, x + 13, 16, fill=AXIS_COLOURS[i],
                             width=3, capstyle="round")
            self.create_text(x + 18, 16, anchor="w", fill=FG, text=name.upper(),
                             font=("TkFixedFont", 8, "bold"))

        if not self.curves:
            self.create_rectangle(pl, pt, pl + pw, pt + ph, outline=GRID_HI)
            self.create_text(pl + pw // 2, pt + ph // 2, fill=MUTED,
                             font=("TkDefaultFont", 9),
                             text="Load a session and compute Allan deviation")
            return

        xs = [t for _, tau, _ in self.curves for t in tau if t > 0]
        ys = [a for _, _, adev in self.curves for a in adev if a > 0]
        if not xs or not ys:
            return

        lx0, lx1 = math.log10(min(xs)), math.log10(max(xs))
        ly0, ly1 = math.log10(min(ys)), math.log10(max(ys))
        if lx1 - lx0 < 1e-9:
            lx1 = lx0 + 1
        pad = (ly1 - ly0) * 0.10 or 0.5
        ly0, ly1 = ly0 - pad, ly1 + pad

        def px(t):
            return pl + (math.log10(t) - lx0) / (lx1 - lx0) * pw

        def py(a):
            return pt + ph - (math.log10(a) - ly0) / (ly1 - ly0) * ph

        self._plot_box = (pl, pt, pl + pw, pt + ph)

        # Minor 2x/5x lines make a log plot readable without making it busy.
        for d in range(int(math.floor(lx0)) - 1,
                       int(math.ceil(lx1)) + 1):
            for multiple in (2.0, 5.0):
                value = multiple * 10.0 ** d
                lv = math.log10(value)
                if lx0 < lv < lx1:
                    x = px(value)
                    self.create_line(x, pt, x, pt + ph, fill=GRID)

        for d in range(int(math.floor(lx0)), int(math.ceil(lx1)) + 1):
            if not (lx0 <= d <= lx1):
                continue
            value = 10.0 ** d
            x = px(value)
            self.create_line(x, pt, x, pt + ph, fill=GRID_HI)
            self.create_text(x, pt + ph + 14, fill=MUTED,
                             font=("TkFixedFont", 8),
                             text=self._time_label(value))

        for d in range(int(math.floor(ly0)), int(math.ceil(ly1)) + 1):
            if not (ly0 <= d <= ly1):
                continue
            value = 10.0 ** d
            y = py(value)
            self.create_line(pl, y, pl + pw, y, fill=GRID_HI)
            self.create_text(pl - 10, y, anchor="e", fill=MUTED,
                             font=("TkFixedFont", 8), text=f"1e{d}")

        self.create_rectangle(pl, pt, pl + pw, pt + ph, outline=GRID_HI)
        self.create_text(pl + pw // 2, h - 8, fill=MUTED,
                         font=("TkFixedFont", 8), text="averaging time τ (s)")
        self.create_text(15, pt + ph // 2, fill=MUTED, angle=90,
                         font=("TkFixedFont", 8),
                         text=f"Allan deviation σ  ({self.unit})")

        for axis, tau, adev in self.curves:
            pts = []
            for t, a in zip(tau, adev):
                if t > 0 and a > 0:
                    x, y = px(t), py(a)
                    pts += [x, y]
                    self._hover_points.append((x, y, axis, float(t),
                                               float(a)))
            if len(pts) >= 4:
                self.create_line(*pts, fill=AXIS_COLOURS[axis], width=2,
                                 capstyle="round", joinstyle="round")

        # Only a measured knee is marked. An endpoint minimum deliberately has
        # no marker, matching the "not measured" result in the table.

        for axis, (tb, amin) in self.marks.items():
            if tb > 0 and amin > 0:
                x, y = px(tb), py(amin)
                self.create_line(x, y + 7, x, pt + ph, fill=AXIS_COLOURS[axis],
                                 dash=(2, 4))
                self.create_oval(x - 4, y - 4, x + 4, y + 4,
                                 outline=AXIS_COLOURS[axis], width=2,
                                 fill=PLOT_BG)


class AllanWindow(tk.Toplevel):
    def __init__(self, master, on_save=None):
        super().__init__(master)
        self.title("xxCar — Allan variance")
        self.geometry("1280x900")
        self.minsize(1000, 720)
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

        work = ttk.Frame(self, style="Card.TFrame", padding=(10, 6))
        work.pack(fill="x", padx=8, pady=(8, 0))
        self.work_text = tk.StringVar(value="Ready")
        ttk.Label(work, textvariable=self.work_text,
                  style="Muted.TLabel", width=42).pack(side="left")
        self.work_progress = ttk.Progressbar(
            work, mode="determinate", maximum=100)
        self.work_progress.pack(side="left", fill="x", expand=True, padx=(8, 0))

        # dataset health, above the plots: whether the numbers can be believed
        self.health = ttk.Label(self, text="", style="Muted.TLabel",
                                justify="left", wraplength=1220)
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

    def _work_status(self, text, current=0, total=0):
        self.work_text.set(text)
        self.work_progress["value"] = (
            100.0 * current / total if total > 0 else 0.0)

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
        if self.busy or not self.sess_cb.get():
            return
        sess = int(self.sess_cb.get().split()[0])
        paths = self.sessions[sess]
        load_workers = min(2, len(paths))
        self._work_status(
            f"Loading {len(paths)} parts with {load_workers} worker"
            f"{'s' if load_workers != 1 else ''}…",
                          0, len(paths))
        self.series = {}
        self.results = {}
        self.summary = {}
        for plot in self.plots.values():
            plot.set_data("", "", [])
        for item in self.tbl.get_children():
            self.tbl.delete(item)
        self.health.config(text="Validating framing and timestamps…",
                           foreground=MUTED)
        self.run_btn.config(state="disabled")
        self.save_btn.config(state="disabled")
        self.sess_cb.config(state="disabled")
        self.busy = True

        def work():
            started = time.perf_counter()
            try:
                s = allan.load_session(
                    paths, progress=lambda i, n, nm:
                    self.q.put(("progress",
                                (f"Loaded {i+1}/{n}: {nm}", i + 1, n))),
                    workers=load_workers)
                self.q.put(("loaded", (s, time.perf_counter() - started)))
            except Exception as exc:                      # noqa: BLE001
                self.q.put(("error", ("load", f"load failed: {exc}")))

        threading.Thread(target=work, daemon=True).start()

    def _describe(self):
        """Say whether the dataset is trustworthy before its numbers are."""
        bits = []
        bad = False
        has_gaps = False
        has_damage = False
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
                has_gaps = True
            damage = s.resyncs + s.rejected + s.dropped
            if damage:
                frag += (f", {damage} damaged record(s) rejected"
                         f" [{s.resyncs} resync, {s.rejected} framing, "
                         f"{s.dropped} payload]")
                bad = True
                has_damage = True
            bits.append(frag)
        txt = "   |   ".join(bits)
        if has_gaps:
            txt += ("\n⚠  gaps break the uniform sampling Allan variance "
                    "assumes, and bias the long-τ end where bias "
                    "instability is read — treat B and K with suspicion.")
        if has_damage:
            txt += ("\n⚠  damaged records were excluded before calculation; "
                    "inspect the sensor/data path even if their fraction is "
                    "too small to move the Allan curve.")
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
        names = list(self.series)
        self._work_status(f"Computing {len(names)} sensors with 2 workers…",
                          0, len(names))

        def work():
            started = time.perf_counter()
            try:
                out = allan.analyse_many(
                    self.series, head, tail, workers=2,
                    progress=lambda i, n, name:
                    self.q.put((
                        "progress",
                        (f"Computed {name} ({i+1}/{n})", i + 1, n))))
                self.q.put(("done", (out, time.perf_counter() - started)))
            except Exception as exc:                      # noqa: BLE001
                self.q.put(("error", ("compute",
                                      f"compute failed: {exc}")))

        threading.Thread(target=work, daemon=True).start()

    def _show(self, results):
        self.results = results
        for i in self.tbl.get_children():
            self.tbl.delete(i)
        self.summary = {}
        unmeasured = []

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
                    bq = (f"{d['B_deg_per_hr']:.2f} °/h"
                          if math.isfinite(r.B) else "not measured")
                else:
                    nq = f"{d['N_ug_sqrt_hz']:.1f} µg/√Hz"
                    bq = (f"{d['B_ug']:.1f} µg"
                          if math.isfinite(r.B) else "not measured")

                b_si = f"{r.B:.4e}" if math.isfinite(r.B) else "—"
                k_si = f"{r.K:.4e}" if math.isfinite(r.K) else "—"
                tau_b = f"{r.tau_B:.1f} s" if math.isfinite(r.tau_B) else "—"
                self.tbl.insert(parent, "end", text=f"  {ax}",
                                values=(f"{r.N:.4e}", nq, b_si, bq,
                                        k_si, tau_b))
                if not math.isfinite(r.B):
                    unmeasured.append(f"{name}.{ax} B")
                if not math.isfinite(r.K):
                    unmeasured.append(f"{name}.{ax} K")
            self.tbl.item(parent, open=True)

        if unmeasured:
            self.health.config(
                text=self.health.cget("text") +
                "\n⚠  not measurable from the available right-hand "
                "long-τ curve: " + ", ".join(unmeasured),
                foreground=WARN)

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
                    "gaps": [{"at_s": a, "len_s": d} for a, d in s.gaps],
                    "resyncs": s.resyncs,
                    "framing_rejected": s.rejected,
                    "payload_rejected": s.dropped}
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
                    text, current, total = payload
                    self._work_status(text, current, total)
                elif kind == "loaded":
                    self.series, elapsed = payload
                    self.busy = False
                    self._describe()
                    self._work_status(
                        f"Ready — {len(self.series)} sensors loaded in "
                        f"{elapsed:.1f} s", 1, 1)
                    self.sess_cb.config(state="readonly")
                    self.run_btn.config(
                        state="normal" if self.series else "disabled")
                elif kind == "done":
                    results, elapsed = payload
                    self.busy = False
                    self._describe()
                    self._show(results)
                    self._work_status(
                        f"Ready — Allan curves computed in {elapsed:.1f} s",
                        1, 1)
                    self.run_btn.config(state="normal")
                    self.sess_cb.config(state="readonly")
                elif kind == "error":
                    stage, message = payload
                    self.busy = False
                    self.run_btn.config(
                        state="normal"
                        if stage == "compute" and self.series else "disabled")
                    self.sess_cb.config(state="readonly")
                    self._work_status("Failed", 0, 1)
                    self.health.config(text=message, foreground="#ff6b81")
        except queue.Empty:
            pass
        self.after(80, self._pump)
