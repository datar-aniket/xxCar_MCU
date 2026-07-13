############################################################################
# apps/init/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

# PROGNAME "xxcar_init" makes the entry symbol xxcar_init_main(), which is what
# CONFIG_INIT_ENTRYPOINT must name (exactly as "nsh" yields nsh_main).
PROGNAME  = $(CONFIG_XXCAR_INIT_PROGNAME)
PRIORITY  = $(CONFIG_XXCAR_INIT_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_INIT_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_INIT)

MAINSRC   = init_main.c

include $(APPDIR)/Application.mk
