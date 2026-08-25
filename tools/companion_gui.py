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
from comp_link import quaternion_to_euler, solution_names

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

        row = tk.Frame(send, bg=PANEL)
        row.grid(row=1, column=0, columnspan=6, sticky="w", pady=(10, 0))

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

        # The board timestamps against its own clock; with no timesync yet,
        # zero is honest about that rather than inventing a time.
        self.link.send(encode_external_pose(
            x, y, yaw, valid=self.valid_var.get(),
            reset_counter=self.reset_counter, timestamp_us=0))

    # ---- pump -----------------------------------------------------------

    def _drain(self):
        now = time.time()

        while True:
            try:
                kind, payload = self.q.get_nowait()
            except queue.Empty:
                break

            if kind == "frame":
                msg_id, body = payload
                if msg_id == comp_link.MSG_ESTIMATOR_POSE:
                    self._show(decode_estimator_pose(body))
                    self.last_pose_us = now
            elif kind == "error":
                self.state_lbl.configure(text=str(payload)[:44], fg=BAD)

        if self.stream_var.get() and self.link:
            self._send()

        self._stats(now)
        self.after(100, self._drain)

    def _show(self, pose):
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
