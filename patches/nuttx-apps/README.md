# nuttx-apps patches

Same rationale as `patches/nuttx/`: `deps/nuttx-apps` is a pinned, **unforked**
upstream submodule, so fixes we need *inside* it live here as patches rather than
as commits in the submodule. A submodule gitlink records only a commit hash, so a
local edit there would be invisible to git and a clean clone would silently build
different firmware.

`tools/build.sh` applies these before configuring, idempotently.

## Patches

### 0001-cle-add-tab-completion.patch

NuttX gives you two shell line editors and neither one is complete:

  * **readline** has TAB completion and history, but cannot move the cursor. It
    decodes exactly two escape sequences - 'A' and 'B', up and down - and
    swallows the rest, so LEFT/RIGHT/HOME/END do nothing and fixing a typo means
    retyping the line.
  * **CLE** decodes the full VT100 set (left, right, home, end, delete) and has
    its own history - but has no TAB completion at all.

This adds TAB completion to CLE, so one editor does all three.

It reuses the command list NSH already exposes to readline: NSH registers a
count/getname vtable for tab completion, and this adds the CLE-side equivalent
(`cle_extmatch()`), so TAB completes NSH's own `ls`/`ps`/`cat` as well as the
registered builtin apps. One match completes outright; several are listed and the
line is extended by whatever prefix they all agree on, so a TAB is never wasted.

Upstreamable.
