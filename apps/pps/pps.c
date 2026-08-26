/****************************************************************************
 * apps/pps/pps.c
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include "pps.h"
#include "../../boards/fmuv6c/src/fmuv6c.h"

int pps_start(void)
{
  return fmuv6c_pps_initialize();
}

int pps_stop(void)
{
  return fmuv6c_pps_uninitialize();
}
