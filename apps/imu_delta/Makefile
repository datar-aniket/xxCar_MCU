############################################################################
# apps/imu_delta/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = $(CONFIG_XXCAR_IMU_DELTA_PROGNAME)
PRIORITY  = $(CONFIG_XXCAR_IMU_DELTA_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_IMU_DELTA_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_IMU_DELTA)

CSRCS   = imu_delta.c imu_integrator.c imu_lpf.c imu_resampler.c
MAINSRC = imu_delta_main.c

include $(APPDIR)/Application.mk
