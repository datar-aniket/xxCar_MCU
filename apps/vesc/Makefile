############################################################################
# apps/vesc/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = $(CONFIG_XXCAR_VESC_PROGNAME)
PRIORITY  = $(CONFIG_XXCAR_VESC_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_VESC_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_VESC)

# vesc_proto.c is the wire format, with no I/O so it can be host-tested;
# vesc.c is the daemon; vesc_main.c is the command.
CSRCS   = vesc.c vesc_proto.c
MAINSRC = vesc_main.c

include $(APPDIR)/Application.mk
