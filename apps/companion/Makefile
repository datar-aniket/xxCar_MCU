############################################################################
# apps/companion/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = $(CONFIG_XXCAR_COMPANION_PROGNAME)
PRIORITY  = $(CONFIG_XXCAR_COMPANION_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_COMPANION_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_COMPANION)

# comp_proto.c is the wire format, with no I/O so it can be host-tested;
# companion.c is the daemon; companion_main.c is the command.
CSRCS   = companion.c comp_proto.c comp_state.c
MAINSRC = companion_main.c

include $(APPDIR)/Application.mk
