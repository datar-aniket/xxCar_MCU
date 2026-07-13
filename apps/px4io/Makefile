############################################################################
# apps/px4io/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = $(CONFIG_XXCAR_PX4IO_PROGNAME)
PRIORITY  = $(CONFIG_XXCAR_PX4IO_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_PX4IO_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_PX4IO)

# px4io.c is the client library (RC input, PWM output) that the serial port
# manager and the RC/actuator code link against; px4io_main.c is the NSH
# command that drives it by hand.
CSRCS    += px4io.c
MAINSRC   = px4io_main.c

include $(APPDIR)/Application.mk
