# Custom applications (reserved)

This directory is a placeholder for xxCar's own NuttX applications (control loops,
DDS bridge, safety tasks) added in later stages.

> **Note:** This is *not* the NuttX apps tree. The build uses the `deps/nuttx-apps`
> submodule as the apps directory (`configure.sh -a ../nuttx-apps`). Custom apps here
> will be wired in later via NuttX's external-apps mechanism.
