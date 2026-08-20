############################################################################
# apps/cpu_status/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = $(CONFIG_XXCAR_CPU_STATUS_PROGNAME) top
PRIORITY  = $(CONFIG_XXCAR_CPU_STATUS_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_CPU_STATUS_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_CPU_STATUS)

MAINSRC   = cpu_status_main.c top_main.c
CSRCS     = cpu_audit.c cpu_runtime.c

include $(APPDIR)/Application.mk
