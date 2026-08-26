############################################################################
# apps/pps/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = $(CONFIG_XXCAR_PPS_PROGNAME)
PRIORITY  = $(CONFIG_XXCAR_PPS_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_PPS_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_PPS)

CSRCS   = pps.c
MAINSRC = pps_main.c

include $(APPDIR)/Application.mk
