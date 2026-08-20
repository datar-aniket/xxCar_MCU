############################################################################
# apps/sensors/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = $(CONFIG_XXCAR_SENSORS_PROGNAME)
PRIORITY  = $(CONFIG_XXCAR_SENSORS_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_SENSORS_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_SENSORS)

# sensors.c is the daemon; rotation.c is the shared rotation table;
# dsp_filter.c is the native-rate three-axis FPU biquad;
# sensors_main.c is the `sensors` command.
CSRCS    += sensors.c rotation.c dsp_filter.c
MAINSRC   = sensors_main.c

include $(APPDIR)/Application.mk
