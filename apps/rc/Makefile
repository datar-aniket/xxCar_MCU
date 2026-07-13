############################################################################
# apps/rc/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = $(CONFIG_XXCAR_RC_PROGNAME)
PRIORITY  = $(CONFIG_XXCAR_RC_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_RC_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_RC)

# rc.c is the driver (port setup, autodetect, publish); rc_decode.c is the
# SBUS/CRSF frame decoding; rc_main.c is the `rc` command.
CSRCS    += rc.c rc_decode.c
MAINSRC   = rc_main.c

include $(APPDIR)/Application.mk
