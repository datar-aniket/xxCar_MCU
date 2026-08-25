#!/usr/bin/env python3
"""Bench tool for the companion link.

Sends an EXTERNAL_POSE with hand-entered x, y and yaw, and shows the
estimator pose coming back as six numbers.

For testing the link and the fusion, not for flying anything: the pose it
sends is whatever you typed, which is exactly what makes it useful for
checking gating, the datum reset and the noise floor.

    tools/companion_gui.py
    tools/companion_gui.py --port /dev/ttyUSB0 --baud 921600
"""

import argparse
import datetime
import math
import pathlib
import queue
import sys
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
from comp_link import Link, decode_estimator_pose, encode_external_pose
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
        self.geometry("760x560")

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

        self._build(port, baud)
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
        self.est_reset_lbl = self._label(recv, "estimator reset_counter --")
        self.est_reset_lbl.grid(row=3, column=0, columnspan=3, sticky="w",
                                padx=18)

        self.time_lbl = self._label(recv, "solution time --")
        self.time_lbl.grid(row=4, column=0, columnspan=3, sticky="w",
                           padx=18, pady=(4, 0))

        # ---- counters ---------------------------------------------------

        self.stats_lbl = self._label(self, "", MUTED)
        self.stats_lbl.pack(fill="x", padx=14, pady=(0, 10))

    def _ports(self):
        return [p.device for p in list_ports.comports()]

    # ---- link -----------------------------------------------------------

    def _toggle(self):
        if self.link:
            self.link.close()
            self.link = None
            self.open_btn.configure(text="open")
            self.state_lbl.configure(text="closed", fg=BAD)
            return

        try:
            self.link = Link(self.port_var.get(), int(self.baud_var.get()),
                             self.q)
            self.link.start()
        except Exception as exc:
            self.state_lbl.configure(text=str(exc)[:44], fg=BAD)
            return

        self.open_btn.configure(text="close")
        self.state_lbl.configure(text="open", fg=GOOD)

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
                if msg_id == comp_link.MSG_ESTIMATOR_POSE:
                    self._show(decode_estimator_pose(body), rx_us)
                    self.last_pose_us = now
                elif msg_id == comp_link.MSG_TIMESYNC_REP:
                    rep = decode_timesync_rep(body)
                    # rx_us came off the reading thread, not from here, and
                    # is converted to the same UTC basis the request was
                    # sent in so both sides of the solve agree.
                    self._sync_samples.append(
                        timesync_solve(rep, self.utc.to_utc(rx_us)))
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
        self.est_reset_lbl.configure(
            text=f"estimator reset_counter {pose['reset_counter']}")

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
