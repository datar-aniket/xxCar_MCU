############################################################################
# apps/ekf3/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = $(CONFIG_XXCAR_EKF3_PROGNAME)
PRIORITY  = $(CONFIG_XXCAR_EKF3_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_EKF3_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_EKF3)

CSRCS   = ekf3.c ekf_core.c ekf_sources.c
MAINSRC = ekf3_main.c

include $(APPDIR)/Application.mk
