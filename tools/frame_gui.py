#!/usr/bin/env python3
"""Visualize xxCar sensor frames and estimate mocap extrinsics from a ULog."""

from __future__ import annotations

import json
import math
import subprocess
import sys
import tkinter as tk
import tkinter.font as tkfont
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import numpy as np

TOOL_DIR = Path(__file__).resolve().parent
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

from extrinsic_cal import CalibrationError, calibrate_log
from rotation_table import ROTATIONS

BG, PANEL, PLOT = "#0f1115", "#171b22", "#0b0d11"
FG, MUTED, GRID = "#cdd6e0", "#7b8798", "#2c3444"
AXIS = ("#ff5c70", "#46d37b", "#559cff")


def euler_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr,
                      cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr,
                      sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def parameter_frames(parameters, estimate=None):
    """Return display frames as (name, origin body XYZ, axes-to-body)."""
    def value(name, default=0.0):
        return float(parameters.get(name, default))

    frames = [("BODY FLU", np.zeros(3), np.eye(3))]
    for label, stem, rotation_name in (
            ("IMU0", "SENS_IMU0_POS", "SENS_IMU0_ROT"),
            ("IMU1", "SENS_IMU1_POS", "SENS_IMU1_ROT"),
            ("MAG0", "SENS_MAG0_POS", "SENS_MAG0_ROT")):
        origin = np.array([value(f"{stem}_{axis}") for axis in "XYZ"])
        rotation_index = int(value(rotation_name, 0))
        rotation = np.asarray(ROTATIONS.get(rotation_index, np.eye(3)))
        frames.append((label, origin, rotation))

    ext = dict(parameters)
    if estimate:
        ext.update(estimate)
    position = np.array([value("EK3_EXT_POS_X"), value("EK3_EXT_POS_Y"),
                         value("EK3_EXT_POS_Z")])
    if estimate:
        position[:2] = [float(ext["EK3_EXT_POS_X"]),
                        float(ext["EK3_EXT_POS_Y"])]
    rpy = np.radians([value("EK3_EXT_ROLL"), value("EK3_EXT_PITCH"),
                      float(ext.get("EK3_EXT_YAW", value("EK3_EXT_YAW")))])
    marker_to_body = euler_matrix(*rpy)
    body_to_marker = marker_to_body.T
    marker_origin_body = -body_to_marker @ position
    frames.append(("MOCAP", marker_origin_body, body_to_marker))
    return frames


class FrameCanvas(tk.Canvas):
    def __init__(self, master, **kwargs):
        super().__init__(master, bg=PLOT, highlightthickness=0, **kwargs)
        self.frames = parameter_frames({})
        self.azimuth = math.radians(-40)
        self.elevation = math.radians(24)
        self.bind("<Configure>", lambda _event: self.redraw())

    def set_view(self, azimuth_deg, elevation_deg):
        self.azimuth = math.radians(float(azimuth_deg))
        self.elevation = math.radians(float(elevation_deg))
        self.redraw()

    def set_frames(self, frames):
        self.frames = frames
        self.redraw()

    def project(self, point, scale):
        ca, sa = math.cos(self.azimuth), math.sin(self.azimuth)
        ce, se = math.cos(self.elevation), math.sin(self.elevation)
        x, y, z = point
        horizontal = ca * x - sa * y
        depth = sa * x + ca * y
        vertical = ce * z - se * depth
        return self.winfo_width() * 0.5 + scale * horizontal, \
            self.winfo_height() * 0.55 - scale * vertical

    def redraw(self):
        self.delete("all")
        width, height = self.winfo_width(), self.winfo_height()
        if width < 100 or height < 100:
            return
        furthest = max((np.linalg.norm(origin) for _, origin, _ in self.frames),
                       default=0.2)
        extent = max(0.35, furthest + 0.25)
        scale = min(width, height) * 0.38 / extent
        for grid_value in np.linspace(-extent, extent, 9):
            for start, end in (((-extent, grid_value, 0),
                                (extent, grid_value, 0)),
                               ((grid_value, -extent, 0),
                                (grid_value, extent, 0))):
                self.create_line(*self.project(np.array(start), scale),
                                 *self.project(np.array(end), scale), fill=GRID)
        axis_length = min(0.20, extent * 0.35)
        for frame_index, (name, origin, rotation) in enumerate(self.frames):
            ox, oy = self.project(origin, scale)
            self.create_oval(ox - 3, oy - 3, ox + 3, oy + 3,
                             fill=FG, outline="")
            for axis in range(3):
                endpoint = origin + rotation[:, axis] * axis_length
                ex, ey = self.project(endpoint, scale)
                self.create_line(ox, oy, ex, ey, fill=AXIS[axis], width=2,
                                 arrow="last")
                if frame_index == 0:
                    self.create_text(ex, ey, text="XYZ"[axis], fill=AXIS[axis],
                                     font=("TkFixedFont", 9, "bold"))
            self.create_text(ox + 7, oy - 8, text=name, fill=FG, anchor="sw",
                             font=("TkFixedFont", 9))


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("xxCar — frame calibration")
        self.geometry("1180x760")
        self.configure(bg=BG)
        self.result = None
        self.path = None
        for name, size in (("TkDefaultFont", 9), ("TkFixedFont", 9)):
            try:
                tkfont.nametofont(name).configure(size=size)
            except tk.TclError:
                pass
        self._style()
        self._build()

    def _style(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", background=BG, foreground=FG,
                        fieldbackground=PANEL, bordercolor=GRID)
        style.configure("TFrame", background=BG)
        style.configure("Card.TFrame", background=PANEL)
        style.configure("TLabel", background=BG, foreground=FG)
        style.configure("Card.TLabel", background=PANEL, foreground=FG)
        style.configure("TButton", background="#222835", foreground=FG)

    def _build(self):
        bar = ttk.Frame(self, style="Card.TFrame", padding=8)
        bar.pack(fill="x", padx=8, pady=8)
        ttk.Button(bar, text="Open ULog…", command=self.open_log).pack(side="left")
        ttk.Button(bar, text="Export result…", command=self.export,
                   state="normal").pack(side="left", padx=8)
        ttk.Button(bar, text="Sensor intrinsic calibration…",
                   command=self.open_intrinsics).pack(side="left")
        self.file_label = ttk.Label(bar, text="no log loaded",
                                    style="Card.TLabel")
        self.file_label.pack(side="left", padx=8)

        pane = ttk.Panedwindow(self, orient="horizontal")
        pane.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        left = ttk.Frame(pane, style="Card.TFrame", padding=8)
        self.output = tk.Text(left, width=48, bg=PLOT, fg=FG, relief="flat",
                              font=("TkFixedFont", 9), wrap="word")
        self.output.pack(fill="both", expand=True)
        pane.add(left, weight=0)
        right = ttk.Frame(pane, style="Card.TFrame", padding=8)
        right.rowconfigure(0, weight=1)
        right.columnconfigure(0, weight=1)
        self.canvas = FrameCanvas(right)
        self.canvas.grid(row=0, column=0, columnspan=2, sticky="nsew")
        ttk.Label(right, text="azimuth", style="Card.TLabel").grid(row=1, column=0)
        azimuth = tk.Scale(right, from_=-180, to=180, orient="horizontal",
                           bg=PANEL, fg=FG, highlightthickness=0, resolution=1)
        azimuth.set(-40)
        azimuth.grid(row=2, column=0, sticky="ew")
        ttk.Label(right, text="elevation", style="Card.TLabel").grid(row=1, column=1)
        elevation = tk.Scale(right, from_=-80, to=80, orient="horizontal",
                             bg=PANEL, fg=FG, highlightthickness=0, resolution=1)
        elevation.set(24)
        elevation.grid(row=2, column=1, sticky="ew")
        update = lambda _value=None: self.canvas.set_view(azimuth.get(), elevation.get())
        azimuth.configure(command=update)
        elevation.configure(command=update)
        pane.add(right, weight=1)

    def open_intrinsics(self):
        """Open the live six-face, gyro, and magnetometer wizard."""
        try:
            subprocess.Popen([sys.executable, str(TOOL_DIR / "cal_gui.py")],
                             start_new_session=True)
        except OSError as exc:
            messagebox.showerror("Cannot start calibration", str(exc))

    def open_log(self):
        selected = filedialog.askopenfilename(filetypes=[("ULog", "*.ulg"),
                                                         ("All files", "*")])
        if not selected:
            return
        self.path = Path(selected)
        self.file_label.configure(text=self.path.name)
        self.output.delete("1.0", "end")
        self.output.insert("end", "Analyzing motion and observability…\n")
        self.update_idletasks()
        try:
            self.result = calibrate_log(self.path)
        except (CalibrationError, OSError, ValueError) as exc:
            self.output.insert("end", f"\nREFUSED: {exc}\n")
            messagebox.showerror("Calibration refused", str(exc))
            return
        rec = self.result["recommended_parameters"]
        yaw = self.result["yaw"]
        lever = self.result["lever_arm"]
        timing = self.result["timing"]
        warm = self.result["warmup"]
        self.output.delete("1.0", "end")
        self.output.insert("end", "PLANAR MOCAP → IMU CALIBRATION\n\n")
        self.output.insert("end", f"time correlation   {timing['correlation']:.3f}\n")
        self.output.insert("end", f"IMU - pose time    {timing['imu_minus_pose_s']*1000:+.2f} ms\n")
        self.output.insert("end", f"timing uncertainty ±{timing['peak_uncertainty_s']*1000:.2f} ms "
                                  f"at {timing['pose_rate_hz']:.1f} Hz\n")
        self.output.insert("end", f"yaw                {yaw['yaw_deg']:+.3f} deg\n")
        self.output.insert("end", f"yaw RMS            {math.degrees(yaw['rms_rad']):.3f} deg\n")
        self.output.insert("end", f"marker→IMU X/Y     {lever['position_x_m']:+.4f}, "
                                  f"{lever['position_y_m']:+.4f} m\n")
        self.output.insert("end", f"lever residual     {lever['rms_mps2']:.4f} m/s²\n")
        self.output.insert("end", f"condition number   {lever['condition']:.2f}\n\n")
        self.output.insert("end", "UNOBSERVABLE FROM PLANAR PACKET\n"
                                  "  roll, pitch and Z translation\n\n")
        stable = warm.get("thermal_stable")
        self.output.insert("end", "WARM-UP / DATA QUALITY\n")
        self.output.insert("end", f"  thermal stability: "
                                  f"{'PASS' if stable else 'NOT SETTLED' if stable is False else 'UNKNOWN'}\n")
        if "temperature_slope_c_per_min" in warm:
            self.output.insert("end", f"  final temperature slope: "
                                      f"{warm['temperature_slope_c_per_min']:+.3f} °C/min\n")
        self.output.insert("end", f"  initial stationary: "
                                  f"{warm['initial_stationary_s']:.1f} s "
                                  f"{'PASS' if warm['stationary_alignment_pass'] else 'NEEDS 10 s'}\n")
        self.output.insert("end", f"  driven distance: {warm['translation_distance_m']:.1f} m "
                                  f"{'PASS' if warm['translation_excitation_pass'] else 'NEEDS 5 m'}\n")
        self.output.insert("end", f"  left/right turn: {warm['left_turn_deg']:.0f}/"
                                  f"{warm['right_turn_deg']:.0f} deg "
                                  f"{'PASS' if warm['bidirectional_turn_excitation_pass'] else 'NEEDS BOTH DIRECTIONS'}\n")
        quality = self.result["quality"]
        self.output.insert("end", f"\nFIT QUALITY: "
                                  f"{'PASS' if quality['spatial_pass'] else 'RECORD BETTER MOTION'}\n")
        self.output.insert("end", "\nPARAMETER PREVIEW\n")
        for name, value in rec.items():
            passed = quality["time_pass" if name == "EK3_EXT_DLY_MS"
                             else "spatial_pass"]
            suffix = "" if passed else "  [CANDIDATE ONLY — WITHHELD]"
            self.output.insert("end", f"  {name} {value:.7g}{suffix}\n")
        self.canvas.set_frames(parameter_frames(self.result["logged_parameters"], rec))

    def export(self):
        if not self.result:
            messagebox.showinfo("No result", "Open and solve a ULog first.")
            return
        selected = filedialog.asksaveasfilename(defaultextension=".json",
                                                 filetypes=[("JSON", "*.json")])
        if selected:
            Path(selected).write_text(json.dumps(self.result, indent=2,
                                                 sort_keys=True) + "\n")


if __name__ == "__main__":
    App().mainloop()
