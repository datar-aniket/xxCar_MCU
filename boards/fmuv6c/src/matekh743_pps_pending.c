/****************************************************************************
 * boards/fmuv6c/src/matekh743_pps_pending.c
 * SPDX-License-Identifier: Apache-2.0
 *
 * PPS migration is deliberately last. Do not link the legacy S3 capture
 * driver on Matek: that pad is now unconditionally reserved for servos.
 * Keep the service API available without touching GPIO, timer or IRQ state.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <string.h>

#include "fmuv6c.h"

int fmuv6c_pps_initialize(void)
{
  return -ENOTSUP;
}

int fmuv6c_pps_uninitialize(void)
{
  return -ENOTSUP;
}

void fmuv6c_pps_status(FAR struct fmuv6c_pps_status_s *status)
{
  if (status != NULL)
    {
      memset(status, 0, sizeof(*status));
      status->state = FMUV6C_PPS_NO_SIGNAL;
    }
}
