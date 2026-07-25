############################################################################
# apps/mavlink/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

# The generated MAVLink codec (deps/mavlink), header-only. APPDIR is the
# nuttx-apps submodule, so deps/mavlink sits at $(APPDIR)/../mavlink. Include the
# `common` dialect directory so `#include <mavlink.h>` resolves; common pulls in
# minimal and the standard message set.
CFLAGS += ${INCDIR_PREFIX}$(APPDIR)/../mavlink/common

PROGNAME  = $(CONFIG_XXCAR_MAVLINK_PROGNAME)
PRIORITY  = $(CONFIG_XXCAR_MAVLINK_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_MAVLINK_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_MAVLINK)

# mavlink.c is the daemon; mavlink_main.c is the `mav` command.
CSRCS    += mavlink.c
MAINSRC   = mavlink_main.c

include $(APPDIR)/Application.mk
