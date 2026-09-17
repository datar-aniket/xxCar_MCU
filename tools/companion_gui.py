#!/usr/bin/env python3
"""Bench tool for the companion link.

Sends an EXTERNAL_POSE with hand-entered x, y and yaw, sends DIRECT_CONTROL
from a pair of sliders, and shows the estimator pose coming back as six
numbers. The link-test panel measures end-to-end echo RTT and verified
full-duplex bandwidth without relying on UTC or board clock synchronisation.

For testing the link and the fusion, not for flying anything: the pose it
sends is whatever you typed, which is exactly what makes it useful for
checking gating, the datum reset and the noise floor.

The DIRECT_CONTROL half MOVES THE VEHICLE. It is still the board that decides
whether to obey - the control router needs the RC source switch in AUTO and
its own arm sequence completed - but bench-test it with the wheels off the
ground first.

    tools/companion_gui.py
    tools/companion_gui.py --port /dev/ttyUSB0 --baud 921600
"""

import argparse
import datetime
import math
import pathlib
import queue
import statistics
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    raise SystemExit("pyserial missing:  pip install pyserial")

import comp_link
from comp_link import (Link, decode_vehicle_state, encode_direct_control,
                       encode_external_pose)
from comp_link import (UtcClock, decode_timesync_rep, encode_timesync_end,
                       encode_timesync_req, encode_timesync_start,
                       host_now_us, quaternion_to_euler, solution_names,
                       timesync_solve)

BG, PANEL = "#0f1115", "#171b22"
FG, MUTED, ACCENT = "#cdd6e0", "#7b8798", "#4c9aff"
GOOD, BAD = "#3ddc84", "#ff6b81"

DEG = 180.0 / math.pi


class App(tk.Tk):
    def __init__(self, port=None, baud=921600):
        super().__init__()
        self.title("companion link")
        self.configure(bg=BG)
        self.geometry("800x960")

        self.q = queue.Queue()
        self.link = None
        self.reset_counter = 0
        self.last_pose_us = 0.0

        # None until a sync has been done. Until then poses go out with a
        # zero timestamp, which the board reads as "stamp it on arrival".
        self.clock_offset_us = None      # host mono -> board mono
        self.clock_trip_us = None
        self.utc = UtcClock()
        self._sync_samples = []
        self._sync_left = 0

        # DIRECT_CONTROL repeats on its own timer rather than riding the
        # 100 ms receive pump. The board expires a command after
        # AUTO_CMD_TO_MS, 100 ms by default, so a sender running at exactly
        # that period spends half its life on the wrong side of the deadline.
        self._drive_job = None

        # Link diagnostics are matched by sequence and verified byte for
        # byte. The serial reader calls _observe_frame directly so a
        # bandwidth sender can refill its bounded window without waiting for
        # Tk's display timer.
        self._diag_cv = threading.Condition()
        self._diag_mode = None
        self._diag_pending = {}
        self._diag_corrupt = 0
        self._diag_seq = 0
        self._diag_generation = 0
        self._lat_job = None
        self._lat_samples = []
        self._lat_sent = 0
        self._lat_target = 0
        self._bw_stop = threading.Event()

        self._build(port, baud)
        self.protocol("WM_DELETE_WINDOW", self._close_window)
        self.after(50, self._drain)

    # ---- layout ---------------------------------------------------------

    def _label(self, parent, text, colour=MUTED, size=10, bold=False):
        return tk.Label(parent, text=text, bg=parent["bg"], fg=colour,
                        font=("TkDefaultFont", size,
                              "bold" if bold else "normal"))

    def _build(self, port, baud):
        top = tk.Frame(self, bg=BG)
        top.pack(fill="x", padx=12, pady=10)

        self._label(top, "port", size=10).pack(side="left")
        self.port_var = tk.StringVar(value=port or "")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var,
                                     width=22, values=self._ports())
        self.port_box.pack(side="left", padx=6)

        self._label(top, "baud").pack(side="left", padx=(10, 0))
        self.baud_var = tk.StringVar(value=str(baud))
        tk.Entry(top, textvariable=self.baud_var, width=8, bg=PANEL, fg=FG,
                 insertbackground=FG, relief="flat").pack(side="left", padx=6)

        self.open_btn = tk.Button(top, text="open", command=self._toggle,
                                  bg=PANEL, fg=FG, relief="flat", padx=14)
        self.open_btn.pack(side="left", padx=10)

        self.state_lbl = self._label(top, "closed", BAD, bold=True)
        self.state_lbl.pack(side="left")

        # ---- send -------------------------------------------------------

        send = tk.LabelFrame(self, text=" send EXTERNAL_POSE ", bg=PANEL,
                             fg=MUTED, relief="flat", padx=12, pady=10)
        send.pack(fill="x", padx=12, pady=6)

        self.entries = {}
        for i, (key, text) in enumerate((("x", "x  (m, east)"),
                                         ("y", "y  (m, north)"),
                                         ("yaw", "yaw  (deg, CCW from east)"))):
            self._label(send, text).grid(row=0, column=i * 2, sticky="e",
                                         padx=(0 if i == 0 else 14, 6))
            var = tk.StringVar(value="0.0")
            tk.Entry(send, textvariable=var, width=10, bg=BG, fg=FG,
                     insertbackground=FG, relief="flat").grid(row=0,
                                                              column=i * 2 + 1)
            self.entries[key] = var

        # Per-channel measurement noise. Sigma rather than variance because
        # that is the unit a person can judge - "10 cm" means something,
        # "0.01 m^2" needs a moment - and the wire carries variance, so the
        # square happens here.
        self.sigmas = {}
        for i, (key, text) in enumerate((("sx", "sigma x  (m)"),
                                         ("sy", "sigma y  (m)"),
                                         ("syaw", "sigma yaw  (deg)"))):
            self._label(send, text).grid(row=1, column=i * 2, sticky="e",
                                         padx=(0 if i == 0 else 14, 6),
                                         pady=(8, 0))
            var = tk.StringVar(value="0")
            tk.Entry(send, textvariable=var, width=10, bg=BG, fg=FG,
                     insertbackground=FG, relief="flat").grid(
                         row=1, column=i * 2 + 1, pady=(8, 0))
            self.sigmas[key] = var

        self._label(send,
                    "0 = no estimate, board uses EK3_EXT_*_NSE. That "
                    "parameter is a FLOOR: a smaller sigma is raised to it.",
                    size=8).grid(row=2, column=0, columnspan=6, sticky="w",
                                 pady=(4, 0))

        row = tk.Frame(send, bg=PANEL)
        row.grid(row=3, column=0, columnspan=6, sticky="w", pady=(10, 0))

        self.valid_var = tk.BooleanVar(value=True)
        tk.Checkbutton(row, text="valid", variable=self.valid_var, bg=PANEL,
                       fg=FG, selectcolor=BG, activebackground=PANEL,
                       activeforeground=FG).pack(side="left")

        tk.Button(row, text="send once", command=self._send, bg=ACCENT,
                  fg="#08111f", relief="flat", padx=16).pack(side="left",
                                                             padx=10)

        self.stream_var = tk.BooleanVar(value=False)
        tk.Checkbutton(row, text="stream 10 Hz", variable=self.stream_var,
                       bg=PANEL, fg=FG, selectcolor=BG,
                       activebackground=PANEL,
                       activeforeground=FG).pack(side="left", padx=6)

        # Bumping this is the only way to exercise the source-reset re-datum.
        tk.Button(row, text="bump reset_counter", command=self._bump,
                  bg=PANEL, fg=FG, relief="flat",
                  padx=10).pack(side="left", padx=10)
        self.reset_lbl = self._label(row, "reset_counter 0")
        self.reset_lbl.pack(side="left")

        clock = tk.Frame(send, bg=PANEL)
        clock.grid(row=4, column=0, columnspan=6, sticky="w", pady=(10, 0))

        tk.Button(clock, text="sync clock", command=self._sync, bg=PANEL,
                  fg=FG, relief="flat", padx=12).pack(side="left")
        self.clock_lbl = self._label(clock,
                                     "unsynced - poses arrive-stamped", BAD)
        self.clock_lbl.pack(side="left", padx=10)

        # ---- drive ------------------------------------------------------

        drive = tk.LabelFrame(self, text=" send DIRECT_CONTROL ", bg=PANEL,
                              fg=MUTED, relief="flat", padx=12, pady=10)
        drive.pack(fill="x", padx=12, pady=6)

        self.steer_var = tk.DoubleVar(value=0.0)
        self.rear_steer_var = tk.DoubleVar(value=0.0)
        self.throttle_var = tk.DoubleVar(value=0.0)
        self.throttle_mode = tk.IntVar(value=comp_link.THROTTLE_DUTY)

        self._label(drive, "steering    left +").grid(row=0, column=0,
                                                      sticky="e", padx=(0, 8))
        self.steer_scale = tk.Scale(
            drive, from_=-1.0, to=1.0, resolution=0.01,
            orient="horizontal", variable=self.steer_var, length=380,
            bg=PANEL, fg=FG, troughcolor=BG, highlightthickness=0,
            activebackground=ACCENT)
        self.steer_scale.grid(row=0, column=1, sticky="w")

        tk.Button(drive, text="centre", command=self._centre_steering,
                  bg=PANEL, fg=FG, relief="flat",
                  padx=10).grid(row=0, column=2, padx=8)

        self._label(drive, "rear steering left +").grid(
            row=1, column=0, sticky="e", padx=(0, 8))
        self.rear_steer_scale = tk.Scale(
            drive, from_=-1.0, to=1.0, resolution=0.01,
            orient="horizontal", variable=self.rear_steer_var, length=380,
            bg=PANEL, fg=FG, troughcolor=BG, highlightthickness=0,
            activebackground=ACCENT)
        self.rear_steer_scale.grid(row=1, column=1, sticky="w")

        self._label(drive, "throttle").grid(row=2, column=0, sticky="e",
                                            padx=(0, 8))
        self.throttle_scale = tk.Scale(
            drive, from_=-1.0, to=1.0, resolution=0.01,
            orient="horizontal", variable=self.throttle_var, length=380,
            bg=PANEL, fg=FG, troughcolor=BG, highlightthickness=0,
            activebackground=ACCENT)
        self.throttle_scale.grid(row=2, column=1, sticky="w")

        # Springs back the moment the mouse is let go, like a transmitter
        # stick. Steering deliberately does NOT: a car holds its lock, and
        # having to re-aim after every nudge would make the panel useless
        # for checking a steering trim.
        self.throttle_scale.bind("<ButtonRelease-1>",
                                 lambda _e: self.throttle_var.set(0.0))

        modes = tk.Frame(drive, bg=PANEL)
        modes.grid(row=2, column=2, padx=8)

        for text, value in (("duty", comp_link.THROTTLE_DUTY),
                            ("amps", comp_link.THROTTLE_CURRENT)):
            tk.Radiobutton(modes, text=text, value=value,
                           variable=self.throttle_mode,
                           command=self._throttle_mode_changed, bg=PANEL,
                           fg=FG, selectcolor=BG, activebackground=PANEL,
                           activeforeground=FG).pack(anchor="w")

        self._label(drive,
                    "VESC_DUTY_MAX and VESC_CUR_MAX still apply on the "
                    "board, and are lower than the wire range.",
                    size=8).grid(row=3, column=0, columnspan=3, sticky="w",
                                 pady=(4, 0))

        drow = tk.Frame(drive, bg=PANEL)
        drow.grid(row=4, column=0, columnspan=3, sticky="w", pady=(10, 0))

        tk.Button(drow, text="send once", command=self._send_drive,
                  bg=ACCENT, fg="#08111f", relief="flat",
                  padx=16).pack(side="left")

        self.drive_stream_var = tk.BooleanVar(value=False)
        tk.Checkbutton(drow, text="stream 20 Hz",
                       variable=self.drive_stream_var,
                       command=self._drive_stream_toggled, bg=PANEL, fg=FG,
                       selectcolor=BG, activebackground=PANEL,
                       activeforeground=FG).pack(side="left", padx=10)

        # Wide, red, and it does the stopping itself rather than just going
        # quiet: silence works - the router neutrals after AUTO_CMD_TO_MS -
        # but waiting out a timeout is not what anybody reaching for a stop
        # button has in mind.
        tk.Button(drow, text="STOP", command=self._stop_drive, bg=BAD,
                  fg="#1a0508", relief="flat", padx=24,
                  font=("TkDefaultFont", 10, "bold")).pack(side="left",
                                                           padx=16)

        self.drive_lbl_tx = self._label(drow, "idle")
        self.drive_lbl_tx.pack(side="left", padx=6)

        # ---- link diagnostics ------------------------------------------

        diag = tk.LabelFrame(self, text=" link test ", bg=PANEL, fg=MUTED,
                             relief="flat", padx=12, pady=8)
        diag.pack(fill="x", padx=12, pady=6)

        self.lat_count_var = tk.StringVar(value="40")
        self.bw_duration_var = tk.StringVar(value="3.0")
        self._label(diag, "latency packets").grid(row=0, column=0,
                                                   sticky="e")
        tk.Entry(diag, textvariable=self.lat_count_var, width=6, bg=BG,
                 fg=FG, insertbackground=FG, relief="flat").grid(
                     row=0, column=1, padx=6)
        tk.Button(diag, text="test latency", command=self._start_latency,
                  bg=ACCENT, fg="#08111f", relief="flat", padx=12).grid(
                      row=0, column=2, padx=6)

        self._label(diag, "bandwidth seconds").grid(row=0, column=3,
                                                     sticky="e", padx=(18, 0))
        tk.Entry(diag, textvariable=self.bw_duration_var, width=6, bg=BG,
                 fg=FG, insertbackground=FG, relief="flat").grid(
                     row=0, column=4, padx=6)
        tk.Button(diag, text="test bandwidth", command=self._start_bandwidth,
                  bg=ACCENT, fg="#08111f", relief="flat", padx=12).grid(
                      row=0, column=5, padx=6)
        tk.Button(diag, text="stop", command=self._cancel_diagnostics,
                  bg=PANEL, fg=FG, relief="flat", padx=10).grid(
                      row=0, column=6, padx=(6, 0))

        self.diag_lbl = self._label(
            diag, "idle - tests use a board echo; clock sync is not required")
        self.diag_lbl.grid(row=1, column=0, columnspan=7, sticky="w",
                           pady=(7, 0))
        self._label(
            diag,
            "Bandwidth test saturates the companion link; stop the vehicle "
            "and other high-rate traffic first.", BAD, size=8).grid(
                row=2, column=0, columnspan=7, sticky="w", pady=(3, 0))

        # ---- receive ----------------------------------------------------

        recv = tk.LabelFrame(self, text=" estimator pose ", bg=PANEL,
                             fg=MUTED, relief="flat", padx=12, pady=10)
        recv.pack(fill="both", expand=True, padx=12, pady=6)

        self.pose_vals = {}
        names = (("x", "m"), ("y", "m"), ("z", "m"),
                 ("roll", "deg"), ("pitch", "deg"), ("yaw", "deg"))
        for i, (name, unit) in enumerate(names):
            cell = tk.Frame(recv, bg=PANEL)
            cell.grid(row=i // 3, column=i % 3, padx=18, pady=8, sticky="w")
            self._label(cell, f"{name}  ({unit})").pack(anchor="w")
            val = self._label(cell, "--", FG, size=22, bold=True)
            val.pack(anchor="w")
            self.pose_vals[name] = val

        self.sol_lbl = self._label(recv, "solution --", MUTED)
        self.sol_lbl.grid(row=2, column=0, columnspan=3, sticky="w",
                          padx=18, pady=(6, 0))
        self.est_reset_lbl = self._label(recv, "reset_counter --")
        self.est_reset_lbl.grid(row=3, column=0, columnspan=3, sticky="w",
                                padx=18)

        self.twist_lbl = self._label(recv, "v_body --    w --    slip --",
                                     MUTED)
        self.twist_lbl.grid(row=4, column=0, columnspan=3, sticky="w",
                            padx=18)
        self.drive_lbl = self._label(recv, "torque --    steer --    speed --",
                                     MUTED)
        self.drive_lbl.grid(row=5, column=0, columnspan=3, sticky="w",
                            padx=18)

        self.time_lbl = self._label(recv, "solution time --")
        self.time_lbl.grid(row=6, column=0, columnspan=3, sticky="w",
                           padx=18, pady=(4, 0))

        # ---- counters ---------------------------------------------------

        self.stats_lbl = self._label(self, "", MUTED)
        self.stats_lbl.pack(fill="x", padx=14, pady=(0, 10))

    def _ports(self):
        return [p.device for p in list_ports.comports()]

    # ---- link -----------------------------------------------------------

    def _toggle(self):
        if self.link:
            # Stop driving BEFORE the port goes away. Leaving the repeat
            # running would spend every tick failing to send, and the panel
            # would still be showing the last command it managed.
            self._stop_drive()
            self._cancel_diagnostics()
            self.link.close()
            self.link = None
            self.open_btn.configure(text="open")
            self.state_lbl.configure(text="closed", fg=BAD)
            return

        try:
            self.link = Link(self.port_var.get(), int(self.baud_var.get()),
                             self.q)
            self.link.frame_observer = self._observe_frame
            self.link.start()
        except Exception as exc:
            self.state_lbl.configure(text=str(exc)[:44], fg=BAD)
            return

        self.open_btn.configure(text="close")
        self.state_lbl.configure(text="open", fg=GOOD)

    def _close_window(self):
        self._stop_drive()
        self._cancel_diagnostics()
        if self.link:
            self.link.close()
        self.destroy()

    def _bump(self):
        self.reset_counter = (self.reset_counter + 1) & 0xFF
        self.reset_lbl.configure(text=f"reset_counter {self.reset_counter}")

    def _sync(self):
        """Ten exchanges, then keep the least-delayed one.

        Not an average. The offset is only as good as the path is
        symmetric, and averaging lets one badly queued exchange drag the
        estimate; the shortest round trip is the one with least room for
        asymmetry to hide in.
        """
        if not self.link:
            return

        self._sync_samples = []
        self._sync_left = 10

        # Bracket the burst so the board knows one is running and, at the
        # end, what we concluded - it cannot work the offset out itself,
        # only this side sees all four timestamps.
        self.link.send(encode_timesync_start(10))
        self.clock_lbl.configure(text="syncing...", fg=MUTED)
        self._sync_step()

    def _sync_step(self):
        if not self.link or self._sync_left <= 0:
            self._sync_finish()
            return

        self._sync_left -= 1
        self.link.send(encode_timesync_req(self.utc.now_us()))
        self.after(40, self._sync_step)

    def _sync_finish(self):
        if not self._sync_samples:
            self.clock_lbl.configure(text="sync failed - no reply", fg=BAD)
            if self.link:
                self.link.send(encode_timesync_end(0, 0, 0))
            return

        offset, trip = min(self._sync_samples, key=lambda s: s[1])
        self.clock_offset_us = offset
        self.clock_trip_us = trip

        # solve() gives "add to UTC to get board monotonic". The board needs
        # the inverse: what to add to ITS clock to reach UTC.
        if self.link:
            self.link.send(encode_timesync_end(-offset, trip,
                                               len(self._sync_samples)))
        self.clock_lbl.configure(
            text=(f"synced to UTC: board is {-offset / 1000.0:+.2f} ms from "
                  f"UTC, round trip {trip / 1000.0:.2f} ms "
                  f"({len(self._sync_samples)}/10)"),
            fg=GOOD)

    def _send(self):
        if not self.link:
            return

        try:
            x = float(self.entries["x"].get())
            y = float(self.entries["y"].get())
            yaw = float(self.entries["yaw"].get()) / DEG
        except ValueError:
            self.state_lbl.configure(text="bad number", fg=BAD)
            return

        try:
            sx = float(self.sigmas["sx"].get())
            sy = float(self.sigmas["sy"].get())
            syaw = float(self.sigmas["syaw"].get()) / DEG
        except ValueError:
            self.state_lbl.configure(text="bad sigma", fg=BAD)
            return

        # The wire carries VARIANCE; the fields are sigma because that is
        # what a person can judge. Off-diagonals stay zero: the board fuses
        # the diagonal only, as ArduPilot does.
        cov = (sx * sx, 0.0, 0.0, sy * sy, 0.0, syaw * syaw)

        # UTC once synced; the board converts it back to its own monotonic
        # clock on arrival. Unsynced, send zero - "stamp it on arrival" -
        # rather than a UTC the board has no offset to interpret.
        stamp = self.utc.now_us() if self.clock_offset_us is not None else 0

        self.link.send(encode_external_pose(
            x, y, yaw, cov=cov, valid=self.valid_var.get(),
            reset_counter=self.reset_counter, timestamp_us=stamp))

    # ---- drive ----------------------------------------------------------

    def _centre_steering(self):
        self.steer_var.set(0.0)
        self.rear_steer_var.set(0.0)

    def _throttle_mode_changed(self):
        """Rescale the slider, and zero it on the way.

        Carrying the number across would reinterpret it: 12 amps becomes a
        duty of 12, which the board rejects, and a duty of 0.8 becomes 0.8 A,
        which it accepts and which does nothing. Neither is what the person
        who clicked the radio button meant.
        """
        current = self.throttle_mode.get() == comp_link.THROTTLE_CURRENT
        limit = (comp_link.DIRECT_CURRENT_MAX if current
                 else comp_link.DIRECT_DUTY_MAX)

        self.throttle_var.set(0.0)
        self.throttle_scale.configure(from_=-limit, to=limit,
                                      resolution=0.5 if current else 0.01)

    def _drive_stream_toggled(self):
        if self.drive_stream_var.get():
            self._drive_tick()
        elif self._drive_job is not None:
            self.after_cancel(self._drive_job)
            self._drive_job = None

    def _drive_tick(self):
        self._send_drive()

        if self.drive_stream_var.get():
            self._drive_job = self.after(50, self._drive_tick)
        else:
            self._drive_job = None

    def _stop_drive(self):
        """Zero the sliders and say so on the wire, now.

        Going quiet would also stop the vehicle, but only after the board's
        AUTO_CMD_TO_MS expires. An explicit zero arrives in one frame time.
        """
        self.drive_stream_var.set(False)
        self._drive_stream_toggled()
        self.throttle_var.set(0.0)
        self.steer_var.set(0.0)
        self.rear_steer_var.set(0.0)

        if self.link:
            self._send_drive()

    def _send_drive(self):
        if not self.link:
            return

        # The board rejects a command it cannot age, so an unsynced clock
        # means every frame sent from here is counted and dropped. Say that
        # instead of letting the panel look like it is driving.
        if self.clock_offset_us is None:
            self.drive_lbl_tx.configure(text="sync the clock first", fg=BAD)
            return

        try:
            frame = encode_direct_control(
                steering=self.steer_var.get(),
                delta_rear=self.rear_steer_var.get(),
                throttle=self.throttle_var.get(),
                throttle_type=self.throttle_mode.get(),
                timestamp_us=self.utc.now_us())
        except ValueError as exc:
            self.drive_lbl_tx.configure(text=str(exc)[:40], fg=BAD)
            return

        self.link.send(frame)

        current = self.throttle_mode.get() == comp_link.THROTTLE_CURRENT
        self.drive_lbl_tx.configure(
            text=(f"front {self.steer_var.get():+.2f} rear "
                  f"{self.rear_steer_var.get():+.2f}   "
                  f"throttle {self.throttle_var.get():+.2f}"
                  f"{' A' if current else ''}"),
            fg=GOOD if self.throttle_var.get() == 0.0 else ACCENT)

    # ---- latency / bandwidth ------------------------------------------

    def _next_diag_seq(self):
        self._diag_seq = (self._diag_seq + 1) & 0xFFFFFFFF
        return self._diag_seq

    def _begin_diagnostic(self, mode):
        if not self.link:
            self.diag_lbl.configure(text="open a port first", fg=BAD)
            return False
        self._cancel_diagnostics(show=False)
        with self._diag_cv:
            self._diag_generation += 1
            self._diag_mode = mode
            self._diag_pending.clear()
            self._diag_corrupt = 0
            return self._diag_generation

    def _observe_frame(self, msg_id, body, rx_us):
        """Runs on the serial-reader thread; never touches Tk widgets."""
        if msg_id != comp_link.MSG_LINK_TEST_REP:
            return

        try:
            decoded = comp_link.decode_link_test(body)
        except ValueError:
            with self._diag_cv:
                self._diag_corrupt += 1
                self._diag_cv.notify_all()
            return

        with self._diag_cv:
            item = self._diag_pending.pop(decoded["sequence"], None)
            if item is None:
                return
            if item["body"] != body or item["kind"] != decoded["kind"]:
                self._diag_corrupt += 1
            elif decoded["kind"] == comp_link.LINK_TEST_LATENCY:
                self._lat_samples.append(
                    max(0, rx_us - decoded["timestamp_us"]) / 1000.0)
            else:
                item["rx_us"] = rx_us
                self._bw_received.append(item)
            self._diag_cv.notify_all()

    def _start_latency(self):
        try:
            count = int(self.lat_count_var.get())
            if not 1 <= count <= 1000:
                raise ValueError
        except ValueError:
            self.diag_lbl.configure(text="latency count must be 1..1000",
                                    fg=BAD)
            return
        if self._begin_diagnostic("latency") is False:
            return

        self._lat_samples = []
        self._lat_sent = 0
        self._lat_target = count
        self.diag_lbl.configure(text=f"latency: 0/{count}", fg=MUTED)
        self._latency_tick()

    def _latency_tick(self):
        if self._diag_mode != "latency" or not self.link:
            return
        if self._lat_sent >= self._lat_target:
            # Allow the last echo to drain. An absent reply becomes loss.
            self._lat_job = self.after(750, self._finish_latency)
            return

        sequence = self._next_diag_seq()
        frame = comp_link.encode_link_test(
            sequence, comp_link.LINK_TEST_LATENCY,
            comp_link.LINK_TEST_HEADER.size)
        body = frame[3:-2]
        with self._diag_cv:
            self._diag_pending[sequence] = {
                "body": body, "kind": comp_link.LINK_TEST_LATENCY}
        if not self.link.send(frame):
            with self._diag_cv:
                self._diag_pending.pop(sequence, None)
        self._lat_sent += 1
        self.diag_lbl.configure(
            text=f"latency: sent {self._lat_sent}/{self._lat_target}",
            fg=MUTED)
        self._lat_job = self.after(25, self._latency_tick)

    def _finish_latency(self):
        with self._diag_cv:
            samples = list(self._lat_samples)
            corrupt = self._diag_corrupt
            lost = max(0, self._lat_sent - len(samples) - corrupt)
            self._diag_pending.clear()
            self._diag_mode = None
        self._lat_job = None

        if not samples:
            self.diag_lbl.configure(
                text=f"latency: no replies, lost {lost}, corrupt {corrupt}",
                fg=BAD)
            return
        ordered = sorted(samples)
        p95 = ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)]
        self.diag_lbl.configure(
            text=(f"RTT ms  min {ordered[0]:.3f}  median "
                  f"{statistics.median(ordered):.3f}  mean "
                  f"{statistics.fmean(ordered):.3f}  p95 {p95:.3f}  "
                  f"max {ordered[-1]:.3f}   replies {len(samples)}/"
                  f"{self._lat_sent}, corrupt {corrupt}"),
            fg=GOOD if not lost and not corrupt else BAD)

    def _start_bandwidth(self):
        try:
            duration = float(self.bw_duration_var.get())
            if not 0.25 <= duration <= 60.0 or not math.isfinite(duration):
                raise ValueError
        except ValueError:
            self.diag_lbl.configure(text="duration must be 0.25..60 seconds",
                                    fg=BAD)
            return
        generation = self._begin_diagnostic("bandwidth")
        if generation is False:
            return

        # This test intentionally fills the transport. Do not compete with
        # actuator commands, even on a bench where AUTO is not armed.
        self._stop_drive()
        self._bw_stop.clear()
        self._bw_received = []
        self.diag_lbl.configure(text="bandwidth: running...", fg=MUTED)
        threading.Thread(target=self._bandwidth_worker,
                         args=(duration, generation),
                         daemon=True).start()

    def _bandwidth_worker(self, duration, generation):
        link = self.link
        start_us = host_now_us()
        deadline = time.monotonic() + duration
        sent = 0
        wire_sent = 0

        # A bounded pipeline avoids measuring how fast pyserial can enqueue
        # into an unbounded OS buffer. Replies wake this thread directly from
        # the reader, so the Tk refresh period is not part of the result.
        while (time.monotonic() < deadline and not self._bw_stop.is_set()
               and link is not None):
            with self._diag_cv:
                while (len(self._diag_pending) >= 32 and
                       time.monotonic() < deadline and
                       not self._bw_stop.is_set() and
                       generation == self._diag_generation):
                    self._diag_cv.wait(timeout=0.05)
                if (self._bw_stop.is_set() or
                    generation != self._diag_generation or
                    time.monotonic() >= deadline):
                    break
                sequence = self._next_diag_seq()
                frame = comp_link.encode_link_test(
                    sequence, comp_link.LINK_TEST_BANDWIDTH,
                    comp_link.MAX_PAYLOAD)
                self._diag_pending[sequence] = {
                    "body": frame[3:-2],
                    "kind": comp_link.LINK_TEST_BANDWIDTH,
                    "wire_bytes": len(frame)}

            if not link.send(frame):
                with self._diag_cv:
                    self._diag_pending.pop(sequence, None)
                break
            sent += 1
            wire_sent += len(frame)

        send_end_us = host_now_us()
        # Drain one pipeline window after sending finishes.
        drain_deadline = time.monotonic() + 1.0
        with self._diag_cv:
            while (self._diag_pending and time.monotonic() < drain_deadline
                   and not self._bw_stop.is_set()
                   and generation == self._diag_generation):
                self._diag_cv.wait(timeout=0.05)
            if generation != self._diag_generation:
                return
            received = list(self._bw_received)
            lost = len(self._diag_pending)
            corrupt = self._diag_corrupt
            self._diag_pending.clear()
            self._diag_mode = None

        last_rx_us = max((x["rx_us"] for x in received), default=send_end_us)
        elapsed_s = max(1e-6, (max(send_end_us, last_rx_us) - start_us) / 1e6)
        good_wire = sum(x["wire_bytes"] for x in received)
        self.q.put(("bandwidth_done", {
            "sent": sent, "received": len(received), "lost": lost,
            "corrupt": corrupt, "elapsed_s": elapsed_s,
            "offered_kbps": wire_sent * 8.0 / max(
                1e-6, (send_end_us - start_us) / 1e6) / 1000.0,
            "each_kbps": good_wire * 8.0 / elapsed_s / 1000.0,
        }))

    def _show_bandwidth(self, result):
        each = result["each_kbps"]
        self.diag_lbl.configure(
            text=(f"verified {each:.1f} kbit/s each direction, "
                  f"{2 * each:.1f} kbit/s aggregate   host TX "
                  f"{result['offered_kbps']:.1f} kbit/s   frames "
                  f"{result['received']}/{result['sent']}, lost "
                  f"{result['lost']}, corrupt {result['corrupt']}   "
                  f"{result['elapsed_s']:.2f}s"),
            fg=(GOOD if not result["lost"] and not result["corrupt"]
                else BAD))

    def _cancel_diagnostics(self, show=True):
        self._bw_stop.set()
        if self._lat_job is not None:
            try:
                self.after_cancel(self._lat_job)
            except tk.TclError:
                pass
            self._lat_job = None
        with self._diag_cv:
            was_running = self._diag_mode is not None
            self._diag_generation += 1
            self._diag_mode = None
            self._diag_pending.clear()
            self._diag_cv.notify_all()
        if show and was_running:
            self.diag_lbl.configure(text="link test stopped", fg=MUTED)

    # ---- pump -----------------------------------------------------------


    def _drain(self):
        try:
            self._pump()
        except Exception as exc:                       # noqa: BLE001
            # Never let one bad frame kill the pump. An exception escaping
            # here means the after() below never runs, the GUI stops sending
            # AND receiving, and nothing says why - which is indistinguishable
            # from the link having died.
            self.state_lbl.configure(text=f"pump: {exc}"[:44], fg=BAD)

        self.after(100, self._drain)

    def _pump(self):
        now = time.time()

        while True:
            try:
                kind, payload = self.q.get_nowait()
            except queue.Empty:
                break

            if kind == "frame":
                msg_id, body, rx_us = payload
                if msg_id == comp_link.MSG_VEHICLE_STATE:
                    self._show(decode_vehicle_state(body), rx_us)
                    self.last_pose_us = now
                elif msg_id == comp_link.MSG_TIMESYNC_REP:
                    rep = decode_timesync_rep(body)
                    # rx_us came off the reading thread, not from here, and
                    # is converted to the same UTC basis the request was
                    # sent in so both sides of the solve agree.
                    self._sync_samples.append(
                        timesync_solve(rep, self.utc.to_utc(rx_us)))
                # LINK_TEST_REP was already timestamped and consumed by the
                # reader-thread observer. Do not measure it again here.
            elif kind == "bandwidth_done":
                self._show_bandwidth(payload)
            elif kind == "error":
                self.state_lbl.configure(text=str(payload)[:44], fg=BAD)

        if self.stream_var.get() and self.link:
            self._send()

        self._stats(now)

    def _show(self, pose, rx_us=None):
        roll, pitch, yaw = quaternion_to_euler(pose["quaternion"])
        for name, value in (("x", pose["position"][0]),
                            ("y", pose["position"][1]),
                            ("z", pose["position"][2]),
                            ("roll", roll * DEG),
                            ("pitch", pitch * DEG),
                            ("yaw", yaw * DEG)):
            self.pose_vals[name].configure(text=f"{value:+8.3f}")

        self.sol_lbl.configure(
            text="solution " + solution_names(pose["solution_status"]))

        # Which inputs were fresh. Without this a stopped VESC and a
        # stationary vehicle look identical - both report zero torque.
        missing = [name for bit, name in (
            (comp_link.SRC_ESTIMATOR, "ekf"),
            (comp_link.SRC_GYRO, "gyro"),
            (comp_link.SRC_ACCEL, "accel"),
            (comp_link.SRC_VESC, "vesc"))
            if not pose["source_valid"] & bit]
        self.est_reset_lbl.configure(
            text=(f"reset_counter {pose['reset_counter']}"
                  + ("   MISSING: " + " ".join(missing) if missing else "")),
            fg=BAD if missing else MUTED)

        # Body-frame twist and the VESC-derived channels.
        vx, vy, vz = pose["velocity"]
        wx, wy, wz = pose["angular_velocity"]
        slip = pose["side_slip_rad"]
        self.twist_lbl.configure(
            text=(f"v_body {vx:+6.2f} {vy:+6.2f} {vz:+6.2f} m/s    "
                  f"w {wx:+6.2f} {wy:+6.2f} {wz:+6.2f} rad/s    "
                  f"slip {'--' if math.isnan(slip) else f'{slip*DEG:+.1f}'}"))
        self.drive_lbl.configure(
            text=(f"torque {pose['wheel_torque_nm']:+7.3f} Nm    "
                  f"steer F/R {pose['steering_angle']:+7.3f}/"
                  f"{pose['steering_angle_rear']:+7.3f}    "
                  f"motor rate {pose['motor_speed_ms']:+10.3f} state-units"))

        # The solution's own timestamp, and how stale it is by the time it
        # got here. Age needs the offset - it is the difference between two
        # different clocks - so it only means anything once synced.
        stamp_us = pose["timestamp_us"]

        if self.clock_offset_us is not None and rx_us is not None:
            # Both are UTC now, so the age is a plain subtraction rather
            # than an offset correction.
            age_ms = (self.utc.to_utc(rx_us) - stamp_us) / 1000.0
            shown = datetime.datetime.fromtimestamp(
                stamp_us / 1e6,
                datetime.timezone.utc).strftime("%H:%M:%S.%f")[:-3]
            self.time_lbl.configure(
                text=(f"solution time {shown} UTC   "
                      f"age at arrival {age_ms:+.2f} ms"),
                fg=BAD if abs(age_ms) > 100.0 else MUTED)
        else:
            self.time_lbl.configure(
                text=(f"solution time {stamp_us / 1e6:12.6f} s board "
                      "monotonic   (sync for UTC and age)"), fg=MUTED)

    def _stats(self, now):
        if not self.link:
            self.stats_lbl.configure(text="")
            return

        p = self.link.parser
        age = now - self.last_pose_us if self.last_pose_us else None
        stale = "  STALE" if age is not None and age > 1.0 else ""

        self.stats_lbl.configure(
            text=(f"in {self.link.bytes_in}B  frames {p.frames}   "
                  f"out {self.link.bytes_out}B  frames {self.link.tx_frames}"
                  f"   faults: crc {p.crc_errors}  unknown_id "
                  f"{p.unknown_id}  bad_length {p.bad_length}{stale}"),
            fg=BAD if (p.crc_errors or p.bad_length) else MUTED)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=921600)
    args = ap.parse_args()
    App(args.port, args.baud).mainloop()


if __name__ == "__main__":
    main()
