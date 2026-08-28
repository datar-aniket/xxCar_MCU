#!/usr/bin/env python3
"""Drives the companion GUI's DIRECT_CONTROL panel and inspects the wire.

This panel moves a vehicle, and its failure modes are quiet ones. A command
sent before the clocks are related is dropped by the board and counted, not
reported back, so a panel that looks like it is driving while nothing happens
is the expected symptom rather than an unusual one. Switching the throttle
mode without zeroing the slider reinterprets the number that is already
there. And a stream left running against a closed port keeps showing the last
command it managed to send.

None of those raise, so each one is asserted here against the bytes the panel
actually put on the wire.

No hardware and no serial port: the link is a stub that records frames.
"""

import importlib.util
import pathlib
import sys
import tkinter as tk

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))

import comp_link  # noqa: E402


def load_gui():
    spec = importlib.util.spec_from_file_location(
        "companion_gui", str(REPO / "tools" / "companion_gui.py"))
    module = importlib.util.module_from_spec(spec)
    sys.modules["companion_gui"] = module
    spec.loader.exec_module(module)
    return module


class FakeLink:
    """Records frames instead of writing them to a port."""

    def __init__(self):
        self.frames = []
        self.closed = False
        self.parser = comp_link.Parser()
        self.bytes_in = 0
        self.bytes_out = 0
        self.tx_frames = 0

    def send(self, frame):
        self.frames.append(frame)
        self.bytes_out += len(frame)
        self.tx_frames += 1

    def close(self):
        self.closed = True


def decode_last(link):
    """Parse the newest frame back, the way the board would."""
    parser = comp_link.Parser()
    got = None

    for byte in link.frames[-1]:
        result = parser.feed(byte)

        if result is not None:
            got = result

    assert got is not None, "the panel emitted a frame the parser rejects"
    msg_id, body = got
    assert msg_id == comp_link.MSG_DIRECT_CONTROL, msg_id

    stamp, steering, throttle, mode = comp_link.DIRECT_CONTROL.unpack(body)
    return {"timestamp_us": stamp, "steering": steering,
            "throttle": throttle, "throttle_type": mode}


def main():
    gui = load_gui()

    try:
        app = gui.App()
    except tk.TclError as exc:
        print(f"companion_gui: no display, skipped ({exc})")
        return 0

    app.withdraw()
    failures = []

    def check(condition, what):
        if not condition:
            failures.append(what)
            print(f"FAIL {what}")

    # ---- unsynced ------------------------------------------------------
    #
    # The board rejects a command it cannot age, so this must not go out at
    # all. Sending it anyway is the case that looks like driving and is not.

    link = FakeLink()
    app.link = link
    app.clock_offset_us = None
    app.throttle_var.set(0.5)
    app._send_drive()

    check(link.frames == [], "nothing may be sent before a clock sync")
    check("sync" in app.drive_lbl_tx.cget("text"),
          "an unsynced panel must say why it is not sending")

    # ---- synced --------------------------------------------------------

    app.clock_offset_us = 0
    app.steer_var.set(-0.25)
    app.throttle_var.set(0.4)
    app.throttle_mode.set(comp_link.THROTTLE_DUTY)
    app._send_drive()

    check(len(link.frames) == 1, "a synced panel must send")

    sent = decode_last(link)
    check(abs(sent["steering"] + 0.25) < 1e-6,
          f"steering must reach the wire, got {sent['steering']}")
    check(abs(sent["throttle"] - 0.4) < 1e-6,
          f"throttle must reach the wire, got {sent['throttle']}")
    check(sent["throttle_type"] == comp_link.THROTTLE_DUTY,
          "the selected mode must reach the wire")
    check(sent["timestamp_us"] > 1_600_000_000_000_000,
          "the wire stamp must be real UTC microseconds")

    # ---- mode switch ---------------------------------------------------
    #
    # 0.4 duty carried into current mode would be 0.4 A - accepted, and
    # nothing like what the person clicking the radio button asked for.

    # Exactly what the radio button does: set the variable, then call the
    # command bound to it.
    app.throttle_mode.set(comp_link.THROTTLE_CURRENT)
    app._throttle_mode_changed()

    check(app.throttle_var.get() == 0.0,
          "changing throttle mode must zero the slider")
    check(float(app.throttle_scale.cget("to")) ==
          comp_link.DIRECT_CURRENT_MAX,
          "the slider must rescale to the current-mode range")

    app.throttle_var.set(30.0)
    app._send_drive()
    sent = decode_last(link)

    check(sent["throttle_type"] == comp_link.THROTTLE_CURRENT,
          "amps mode must reach the wire as amps")
    check(abs(sent["throttle"] - 30.0) < 1e-6,
          "a value legal in amps must not be rejected as duty")

    # ---- out of range --------------------------------------------------
    #
    # The slider is the guard: tk.Scale clamps its variable to the range set
    # for the mode, so the panel physically cannot ask for 60 A. Worth
    # pinning, because it is the reason the encoder's refusal path is not
    # reachable from the UI.

    app.throttle_var.set(60.0)
    check(app.throttle_var.get() == comp_link.DIRECT_CURRENT_MAX,
          f"the slider must clamp, got {app.throttle_var.get()}")

    app.throttle_var.set(-60.0)
    check(app.throttle_var.get() == -comp_link.DIRECT_CURRENT_MAX,
          "the slider must clamp on the negative side too")

    # And if one ever did get through - a future edit reading a text entry
    # instead - the panel must report it rather than raise inside the pump.

    detached = tk.DoubleVar(value=999.0)
    attached, app.throttle_var = app.throttle_var, detached
    before = len(link.frames)
    app._send_drive()
    app.throttle_var = attached

    check(len(link.frames) == before,
          "an out-of-range command must not be sent")
    check(app.drive_lbl_tx.cget("fg") == gui.BAD,
          "a refused command must be visible on the panel")

    # ---- stop ----------------------------------------------------------
    #
    # Silence would also stop the vehicle, but only once AUTO_CMD_TO_MS
    # expires on the board. Stop means now.

    app.throttle_var.set(20.0)
    app.steer_var.set(0.8)
    app.drive_stream_var.set(True)
    before = len(link.frames)
    app._stop_drive()

    check(len(link.frames) == before + 1, "STOP must send, not just go quiet")

    sent = decode_last(link)
    check(sent["throttle"] == 0.0, "STOP must put zero throttle on the wire")
    check(sent["steering"] == 0.0, "STOP must centre the steering")
    check(not app.drive_stream_var.get(), "STOP must end the stream")
    check(app._drive_job is None, "STOP must cancel the repeat timer")

    # ---- closing the port ----------------------------------------------

    app.throttle_var.set(0.2)
    app.throttle_mode.set(comp_link.THROTTLE_DUTY)
    app.drive_stream_var.set(True)
    app._drive_stream_toggled()
    check(app._drive_job is not None, "the stream must arm a repeat timer")

    app._toggle()

    check(link.closed, "the port must be closed")
    check(app._drive_job is None,
          "closing the port must cancel the repeat timer")
    check(not app.drive_stream_var.get(),
          "closing the port must clear the stream checkbox")
    check(decode_last(link)["throttle"] == 0.0,
          "the last frame before closing must be a zero throttle")

    app.destroy()

    if failures:
        print(f"{len(failures)} failure(s)")
        return 1

    print("companion_gui: DIRECT_CONTROL panel, mode switch and STOP - OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
