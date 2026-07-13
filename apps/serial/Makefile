############################################################################
# apps/serial/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = $(CONFIG_XXCAR_SERIAL_PROGNAME)
PRIORITY  = $(CONFIG_XXCAR_SERIAL_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_SERIAL_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_SERIAL)

# serial.c is the manager library (the port table, boot-time start-up, and
# NSH-on-any-port); serial_main.c is the `ser` command.
CSRCS    += serial.c
MAINSRC   = serial_main.c

include $(APPDIR)/Application.mk
