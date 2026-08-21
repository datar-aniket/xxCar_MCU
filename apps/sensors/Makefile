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
# mag_frame.c loads the stored magnetometer calibration and rotates the
# corrected field into the vehicle frame (the correction itself is
# apps/cal/cal_mag.c, which is not duplicated here);
# sensors_main.c is the `sensors` command.
CSRCS    += sensors.c rotation.c dsp_filter.c mag_frame.c aux.c
MAINSRC   = sensors_main.c

include $(APPDIR)/Application.mk
