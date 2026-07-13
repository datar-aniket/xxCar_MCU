############################################################################
# apps/param/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = $(CONFIG_XXCAR_PARAM_PROGNAME)
PRIORITY  = $(CONFIG_XXCAR_PARAM_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_PARAM_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_PARAM)

# param.c is the library other xxCar apps (mavlink, rc, logger, calibration)
# link against; param_main.c is the NSH command.
CSRCS    += param.c
MAINSRC   = param_main.c

include $(APPDIR)/Application.mk
