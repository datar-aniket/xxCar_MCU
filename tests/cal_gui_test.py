"""Drives the GUI's event handlers with the exact JSON the board emits.

The wizard is a state machine spread across callbacks, and its failure modes
are quiet: a checklist that advances on a REJECTED capture, or a Save button
enabled before all six positions are in, both produce a calibration that looks
finished and is wrong. Neither shows up as an exception.

No hardware and no serial port - only the decode and UI-state paths.

Note on ttk: `widget.cget("state")` returns a Tcl_Obj, so comparing it to the
string "normal" is always False. `instate(["disabled"])` is the real check.
Getting that wrong made an earlier version of this file report a bug that did
not exist.
"""

import importlib.util
import json
import struct
import sys
import tkinter as tk

REPO = __file__.rsplit("/tests/", 1)[0]


def load_gui():
    spec = importlib.util.spec_from_file_location("g", f"{REPO}/tools/cal_gui.py")
    g = importlib.util.module_from_spec(spec)
    sys.modules["g"] = g
    spec.loader.exec_module(g)
    return g


def build_frame(g, sid, seq, t0, dt, rows, enc, scale=1.0):
    """Assemble a frame byte-for-byte the way apps/cal/cal.c does."""
    n = len(rows) * len(rows[0])
    if enc == g.ENC_I16:
        flat = []
        for r in rows:
            for v in r:
                q = v / scale
                flat.append(32767 if q >= 32767 else
                            -32768 if q <= -32768 else int(round(q)))
        data = struct.pack(f"<{n}h", *flat)
    else:
        data = struct.pack(f"<{n}f", *[v for r in rows for v in r])
    ln = 11 + len(data)
    body = struct.pack("<HBBIHBBB", ln, sid, seq, t0, dt,
                       len(rows), len(rows[0]), enc) + data
    return b"\xa5" + body + struct.pack("<H", g.crc16(body))


def main():
    g = load_gui()
    try:
        app = g.App()
    except tk.TclError as exc:
        print(f"cal_gui: no display, skipped ({exc})")
        return 0
    app.withdraw()
    fails = []

    def check(cond, what):
        if not cond:
            fails.append(what)
            print(f"FAIL {what}")

    # ---- sensor list -----------------------------------------------------
    for i, nm in enumerate(("accel0", "gyro0")):
        app._on_json({"evt": "sensor", "id": i, "name": nm, "n": 3, "enc": 0,
                      "scale": 0.00478, "labels": ["x", "y", "z"],
                      "unit": "m/s^2", "present": True, "rate": 2000})
    app._on_json({"evt": "sensor", "id": 5, "name": "baro0", "n": 2, "enc": 1,
                  "scale": 0.0, "labels": ["pressure", "temperature"],
                  "unit": "hPa", "present": False, "rate": 0})
    check(set(app.tree.get_children()) == {"0", "1", "5"}, "three rows listed")
    check("absent" in app.tree.item("5", "text"), "absent sensor marked")

    # ---- six-position wizard --------------------------------------------
    # out of order on purpose: the board classifies, the operator does not
    order = (4, 5, 0, 1, 2, 3)

    for k, pos in enumerate(order[:2]):
        app._on_json({"evt": "cal6", "pos": pos, "have": k + 1, "need": 6,
                      "a": [0, 0, 9.81], "n": 200})

    # A refused capture must not advance the checklist. This has to be tested
    # PART WAY THROUGH: once all six are in, re-adding a position leaves the
    # count unchanged and the check passes against a broken handler. An earlier
    # version of this test did exactly that and let the mutation through.
    before = set(app.cal_have)
    app._on_json({"evt": "error", "msg": "not square to an axis",
                  "a": [5.0, 5.0, 4.0]})
    check(set(app.cal_have) == before, "rejected capture did not advance")
    check("reposition" in app.cal_hint.cget("text"), "rejection explained")
    check(app.save_btn.instate(["disabled"]), "Save disabled after rejection")

    for k, pos in enumerate(order[2:], start=2):
        app._on_json({"evt": "cal6", "pos": pos, "have": k + 1, "need": 6,
                      "a": [0, 0, 9.81], "n": 200})
        if k < 5:
            check(app.save_btn.instate(["disabled"]),
                  f"Save still disabled at {k + 1}/6")

    check(not app.save_btn.instate(["disabled"]), "Save enabled at 6/6")
    check(len(app.cal_have) == 6, "all six positions marked")

    # ---- a refused save must look refused ---------------------------------
    # The board can decline to store a result after all six positions are in:
    # a residual over the limit, or a value the parameter range will not take.
    # Both arrive as "error" events while the six ticks are still lit, so if
    # the panel says nothing the operator reads it as saved. This is tested
    # BEFORE the successful save below, so a handler that only ever appends
    # cannot pass by leaving an earlier success message on screen.
    app._on_json({"evt": "error", "msg": "fit rejected",
                  "residual": 1.42, "limit": 0.5})
    check("rejected" in app.cal_hint.cget("text"), "fit rejection not shown")
    check("1.42" in app.cal_hint.cget("text"), "measured residual not shown")
    check(app.cal_on.get() is not True,
          "calibrated stream enabled by a rejected fit")
    check(not app.save_btn.instate(["disabled"]),
          "Save left disabled after a rejection - retry is impossible")

    app._on_json({"evt": "error", "msg": "out of range",
                  "param": "CAL_ACC0_XSCL", "value": 1.7315})
    check("CAL_ACC0_XSCL" in app.cal_hint.cget("text"),
          "out-of-range parameter not named")
    check("1.73" in app.cal_hint.cget("text"),
          "the value that would not fit was not shown")

    app._on_json({"evt": "ok", "what": "cal6 save", "off": [0.1, -0.2, 0.05],
                  "scl": [1.01, 0.99, 1.0], "residual": 0.0123})
    check("0.0123" in app.cal_hint.cget("text"), "residual shown after save")
    check(app.cal_on.get() is True, "calibrated stream enabled after save")

    # ---- recording -------------------------------------------------------
    app._on_json({"evt": "ok", "what": "record",
                  "path": "/fs/microsd/log/log_007.ulg", "topics": 4})
    check(app.recording and str(app.rec_btn.cget("text")) == "Stop",
          "record start flips the button")
    app._on_json({"evt": "record", "running": True, "path": "log_007.ulg",
                  "samples": 1234567, "bytes": 35800000, "dropped": 0})
    check("35.8 MB" in app.rec_lab.cget("text"), "record status renders size")
    app._on_json({"evt": "ok", "what": "record stop", "path": "x",
                  "samples": 1234567, "bytes": 35800000, "dropped": 0})
    check(not app.recording and str(app.rec_btn.cget("text")) == "Start",
          "record stop flips back")

    # ---- binary decode, both encodings ------------------------------------
    SC = 16.0 * 9.80665 / 32767.0
    app.link = type("L", (), {"scales": {0: SC}, "send": lambda *a: None})()
    dec = g.Link.__dict__["_emit"]

    class Cap:
        def __init__(self):
            self.got = []
            self.scales = {0: SC}

        def put(self, x):
            self.got.append(x)

    cap = Cap()
    holder = type("H", (), {"out": cap, "scales": {0: SC}})()
    dec(holder, build_frame(g, 0, 1, 100, 500,
                            [[9.81, -0.02, 0.31]] * 50, g.ENC_I16, SC))
    kind, (sid, seq, t0, dt, rows) = cap.got[0]
    check(kind == "batch" and len(rows) == 50, "50-sample i16 batch decoded")
    check(abs(rows[0][0] - 9.81) <= SC, "i16 within one LSB")

    cap2 = Cap()
    holder2 = type("H", (), {"out": cap2, "scales": {}})()
    dec(holder2, build_frame(g, 5, 2, 0, 20000,
                             [[1013.25, 41.5]], g.ENC_F32))
    check(cap2.got[0][1][4][0] == [1013.25, 41.5], "f32 baro frame decoded")

    app.destroy()
    if fails:
        print(f"cal_gui: {len(fails)} failure(s)")
        return 1
    print("cal_gui: wizard, recording and frame decode verified - OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
