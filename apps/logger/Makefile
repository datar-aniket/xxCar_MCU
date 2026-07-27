############################################################################
# apps/logger/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = $(CONFIG_XXCAR_LOGGER_PROGNAME)
PRIORITY  = $(CONFIG_XXCAR_LOGGER_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_LOGGER_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_LOGGER)

# logger.c is the daemon + ULog writer; logger_main.c is the `log` command.
CSRCS    += logger.c log_write.c
MAINSRC   = logger_main.c

include $(APPDIR)/Application.mk
