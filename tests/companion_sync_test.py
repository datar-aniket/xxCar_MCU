#!/usr/bin/env python3
"""Headless regression for startup synchronization scheduling and wire epoch."""
import pathlib
import sys
from types import SimpleNamespace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'tools'))
import comp_link
import companion_gui


frames, delays = [], []
app = SimpleNamespace(
    link=SimpleNamespace(send=frames.append),
    _sync_left=0, _sync_completed=0, _sync_samples=[],
    clock_lbl=SimpleNamespace(configure=lambda **kw: None),
    _sync_timer=lambda: None,
    after=lambda delay, callback: delays.append(delay),
)
for i in range(comp_link.TIMESYNC_ACQUIRE_BURSTS):
    app._sync_samples = [(-1700000000000000, 400, 1234567),
                         (-1700000000000100, 200, 2345678),
                         (-1700000000000300, 600, 3456789)]
    companion_gui.App._sync_finish(app)
    assert app._sync_completed == i + 1
    expected = (int(comp_link.TIMESYNC_ACQUIRE_INTERVAL_S * 1000)
                if i + 1 < comp_link.TIMESYNC_ACQUIRE_BURSTS else 30000)
    assert delays[-1] == expected
    assert frames[-1] == comp_link.encode_timesync_end(
        1700000000000100, 200, 3, 2345678)

# Failed acquisition must retry promptly and not count toward the six bursts.
app._sync_samples = []
companion_gui.App._sync_finish(app)
assert delays[-1] == int(comp_link.TIMESYNC_ACQUIRE_INTERVAL_S * 1000)
assert app._sync_completed == comp_link.TIMESYNC_ACQUIRE_BURSTS

# Request timestamp construction must occur after the transmit lock.
state = {'locked': False}


class Lock:
    def __enter__(self):
        state['locked'] = True

    def __exit__(self, *args):
        state['locked'] = False


class Clock:
    def now_us(self):
        assert state['locked']
        return 123456789


class Wire:
    def write(self, frame):
        assert state['locked']
        assert frame == comp_link.encode_timesync_req(123456789)
        return len(frame)


link = object.__new__(comp_link.Link)
link._tx_lock = Lock()
link.ser = Wire()
link.out = SimpleNamespace(put=lambda event: (_ for _ in ()).throw(AssertionError(event)))
link.bytes_out = link.tx_frames = 0
assert link.send_timesync(Clock()) == 123456789
assert link.tx_frames == 1
print('companion sync: fast acquisition cadence, selected epoch, locked TX timestamp - OK')
