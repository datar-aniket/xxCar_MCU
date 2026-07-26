############################################################################
# apps/cal/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = $(CONFIG_XXCAR_CAL_PROGNAME)
PRIORITY  = $(CONFIG_XXCAR_CAL_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_CAL_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_CAL)

# cal.c is the session, cal_proto.c the wire format, cal_still.c the
# stillness gate; cal_main.c is the command.
CSRCS    += cal.c cal_proto.c cal_still.c
MAINSRC   = cal_main.c

include $(APPDIR)/Application.mk
