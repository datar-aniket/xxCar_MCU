# NuttX patches

`deps/nuttx` is a pinned, **unforked** upstream submodule (`nuttx-12.13.0`). The few
fixes we need *inside* the NuttX tree live here as patches instead of as commits in
the submodule, because a submodule gitlink only records a commit hash: local edits to
the submodule working tree are invisible to git, so a clean clone would silently build
**different firmware** than the board on the bench.

`tools/build.sh` applies every `*.patch` here before configuring. Applying is
idempotent (an already-applied patch reverse-applies cleanly and is skipped), so it is
safe to re-run, and it survives a `git submodule update` reverting the tree.

## Patches

### 0001-cdcacm-do-not-treat-usb-suspend-as-a-disconnect.patch

Without this, opening `/dev/ttyACM0` on the host fails with **errno 107 (ENOTCONN)**.

`cdcacm_suspend()` conflates *bus idle* with *cable unplugged*. A USB SUSPEND only
means the host stopped sending SOFs (bus idle >= 3 ms) - the device is still attached
and still configured. But the handler calls `uart_connected(&priv->serdev, false)`,
which sets `dev->disconnected`, and from then on every `uart_open()` returns
`-ENOTCONN`. Coming back requires a matching RESUME *and* `priv->config` to still be
set (`cdcacm_resume()` checks it); if that resume is missed, the port is wedged for
good.

Nothing is lost by dropping the call: a genuine detach is already handled on its own by
`cdcacm_disconnect()` and `cdcacm_resetconfig()`, both of which call
`uart_connected(false)` themselves.

Only relevant when `CONFIG_SERIAL_REMOVABLE=y`, which we need for the composite
CDC/ACM + MSC device.

Upstreamable as-is.
