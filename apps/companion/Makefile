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

# comp_proto.c is the wire format, comp_clock.c the affine UTC discipline,
# and comp_state.c the outbound frame assembly. All three have no device I/O
# so their contracts can be host-tested; companion.c is the daemon and
# companion_main.c is the command.
CSRCS   = companion.c comp_clock.c comp_proto.c comp_state.c
MAINSRC = companion_main.c

include $(APPDIR)/Application.mk
