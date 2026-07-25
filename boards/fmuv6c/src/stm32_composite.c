/****************************************************************************
 * boards/fmuv6c/src/stm32_composite.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB device with two selectable function sets:
 *
 *   configid 0 (default) : CDC/ACM only          - board owns the microSD,
 *                                                  mounted at /fs/microsd
 *   configid 1           : CDC/ACM + Mass Storage - host owns the microSD
 *
 * The CDC/ACM serial data port exists in BOTH, so the host always has the
 * serial port. sdmsc on/off swaps the function set, which re-enumerates USB.
 *
 * Why not simply keep MSC enumerated all the time and just bind/unbind the LUN
 * (which would avoid re-enumeration)? Because NuttX's SCSI layer mishandles an
 * unbound LUN: usbmsc_cmdreadcapacity10() does not check lun->inode and returns
 * "last LBA = nsectors - 1", which for an unbound LUN (nsectors == 0) underflows
 * to 0xFFFFFFFF. Linux then believes it has a >2TB disk, escalates to READ(16)
 * (which NuttX does not implement), and floods dmesg with I/O errors on a
 * phantom drive. So the mass-storage function is only present while we are
 * actually exporting the card.
 *
 * Ordering is the safety property: exactly one side owns the card at a time.
 * export() unmounts /fs/microsd BEFORE the MSC function appears (the LUN is
 * bound in board_mscclassobject()); release() drops the MSC function BEFORE
 * remounting. NuttX and the host must never write the same FAT.
 ****************************************************************************/

#include <nuttx/config.h>

#include <unistd.h>

#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/composite.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbmsc.h>

#include "fmuv6c.h"

#ifdef CONFIG_USBDEV_COMPOSITE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FMUV6C_CONFIGID_CDC     0   /* CDC/ACM only            */
#define FMUV6C_CONFIGID_CDC_MSC 1   /* CDC/ACM + mass storage  */

#define COMPOSITE_MAXDEV        2   /* CDC + MSC */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Handle of the currently registered composite device, and which function set
 * it is running.
 */

static FAR void *g_composite;
static int       g_configid = FMUV6C_CONFIGID_CDC;

#ifdef CONFIG_USBMSC_COMPOSITE
static FAR void *g_mschandle;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_USBMSC_COMPOSITE

/* Instantiate the mass-storage class and bind the LUN to the microSD. This
 * only ever runs while building the CDC+MSC function set, i.e. when the card
 * has already been unmounted locally by fmuv6c_msc_export().
 */

static int board_mscclassobject(int minor,
                                FAR struct usbdev_devinfo_s *devinfo,
                                FAR struct usbdevclass_driver_s **classdev)
{
  int ret;

  ret = usbmsc_configure(1, &g_mschandle);
  if (ret < 0)
    {
      uerr("ERROR: usbmsc_configure failed: %d\n", ret);
      return ret;
    }

  ret = usbmsc_bindlun(g_mschandle, FMUV6C_MICROSD_BLOCKDEV, 0, 0, 0, false);
  if (ret < 0)
    {
      uerr("ERROR: usbmsc_bindlun(%s) failed: %d\n",
           FMUV6C_MICROSD_BLOCKDEV, ret);
      usbmsc_uninitialize(g_mschandle);
      g_mschandle = NULL;
      return ret;
    }

  ret = usbmsc_classobject(g_mschandle, devinfo, classdev);
  if (ret < 0)
    {
      uerr("ERROR: usbmsc_classobject failed: %d\n", ret);
      usbmsc_uninitialize(g_mschandle);
      g_mschandle = NULL;
    }

  return ret;
}

static void board_mscuninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  if (g_mschandle != NULL)
    {
      usbmsc_uninitialize(g_mschandle);
    }

  g_mschandle = NULL;
}
#endif /* CONFIG_USBMSC_COMPOSITE */

/* Build the function set for the requested configid. */

static FAR void *board_composite_build(int configid)
{
  struct composite_devdesc_s dev[COMPOSITE_MAXDEV];
  int ifnobase = 0;
  int strbase  = COMPOSITE_NSTRIDS;
  int dev_idx  = 0;
  int epin     = 1;
  int epout    = 1;

#ifdef CONFIG_CDCACM_COMPOSITE
  /* CDC/ACM: present in every configuration, so the serial port never goes
   * away.
   */

  cdcacm_get_composite_devdesc(&dev[dev_idx]);

  dev[dev_idx].classobject      = cdcacm_classobject;
  dev[dev_idx].uninitialize     = cdcacm_uninitialize;
  dev[dev_idx].devinfo.ifnobase = ifnobase;
  dev[dev_idx].minor            = 0;
  dev[dev_idx].devinfo.strbase  = strbase;

  dev[dev_idx].devinfo.epno[CDCACM_EP_INTIN_IDX]   = epin++;
  dev[dev_idx].devinfo.epno[CDCACM_EP_BULKIN_IDX]  = epin++;
  dev[dev_idx].devinfo.epno[CDCACM_EP_BULKOUT_IDX] = epout++;

  ifnobase += dev[dev_idx].devinfo.ninterfaces;
  strbase  += dev[dev_idx].devinfo.nstrings;
  dev_idx  += 1;
#endif

#ifdef CONFIG_USBMSC_COMPOSITE
  if (configid == FMUV6C_CONFIGID_CDC_MSC)
    {
      /* Mass storage: only while the card is handed to the host */

      usbmsc_get_composite_devdesc(&dev[dev_idx]);

      dev[dev_idx].classobject      = board_mscclassobject;
      dev[dev_idx].uninitialize     = board_mscuninitialize;
      dev[dev_idx].devinfo.ifnobase = ifnobase;
      dev[dev_idx].minor            = 0;
      dev[dev_idx].devinfo.strbase  = strbase;

      dev[dev_idx].devinfo.epno[USBMSC_EP_BULKIN_IDX]  = epin++;
      dev[dev_idx].devinfo.epno[USBMSC_EP_BULKOUT_IDX] = epout++;

      ifnobase += dev[dev_idx].devinfo.ninterfaces;
      strbase  += dev[dev_idx].devinfo.nstrings;
      dev_idx  += 1;
    }
#endif

  return composite_initialize(composite_getdevdescs(), dev, dev_idx);
}

/* Tear down the current USB device and bring it back up with a new function
 * set. The host sees a disconnect/reconnect.
 */

static int board_composite_switch(int configid)
{
  FAR void *handle;

  if (g_composite != NULL)
    {
      composite_uninitialize(g_composite);
      g_composite = NULL;
    }

  handle = board_composite_build(configid);
  if (handle == NULL)
    {
      uerr("ERROR: composite_initialize(configid=%d) failed\n", configid);

      /* Try to get back to the plain CDC device so we are not left with no
       * USB at all.
       */

      if (configid != FMUV6C_CONFIGID_CDC)
        {
          g_composite = board_composite_build(FMUV6C_CONFIGID_CDC);
          g_configid  = FMUV6C_CONFIGID_CDC;
        }

      return -EIO;
    }

  g_composite = handle;
  g_configid  = configid;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_composite_initialize(int port)
{
  return OK;
}

FAR void *board_composite_connect(int port, int configid)
{
  if (configid != FMUV6C_CONFIGID_CDC &&
      configid != FMUV6C_CONFIGID_CDC_MSC)
    {
      return NULL;
    }

  g_composite = board_composite_build(configid);
  if (g_composite != NULL)
    {
      g_configid = configid;
    }

  return g_composite;
}

#ifdef CONFIG_USBMSC_COMPOSITE

/****************************************************************************
 * Name: fmuv6c_msc_export
 *
 * Description:
 *   Hand the microSD to the USB host. Unmounts /fs/microsd FIRST, then brings
 *   up the CDC+MSC function set (which binds the LUN). USB re-enumerates.
 *
 ****************************************************************************/

int fmuv6c_msc_export(void)
{
  int ret;

  if (g_configid == FMUV6C_CONFIGID_CDC_MSC)
    {
      return OK;                  /* already handed over */
    }

  /* Push everything to the card before letting go of it. nx_umount2 flushes the
   * FAT itself, but sync() first is cheap insurance: it commits every dirty
   * buffer on every mount, so nothing a program wrote is left in RAM when the
   * host takes over. Without a flush the host reads stale blocks and a file that
   * `ls` shows on the board is simply absent over USB.
   */

  sync();

  /* Drop our mount. -ENOENT/-EINVAL just means it was not mounted (e.g. no
   * card), which is fine - carry on.
   */

  ret = nx_umount2(FMUV6C_MICROSD_MOUNTPOINT, 0);
  if (ret < 0 && ret != -ENOENT && ret != -EINVAL)
    {
      uerr("ERROR: cannot unmount %s (busy?): %d\n",
           FMUV6C_MICROSD_MOUNTPOINT, ret);
      return ret;
    }

  ret = board_composite_switch(FMUV6C_CONFIGID_CDC_MSC);
  if (ret < 0)
    {
      /* Take the card back so we are not left with nothing */

      nx_mount(FMUV6C_MICROSD_BLOCKDEV, FMUV6C_MICROSD_MOUNTPOINT,
               "vfat", 0, NULL);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: fmuv6c_msc_release
 *
 * Description:
 *   Take the microSD back from the host: drop the mass-storage function, then
 *   remount /fs/microsd. USB re-enumerates as CDC-only.
 *
 ****************************************************************************/

int fmuv6c_msc_release(void)
{
  int ret;

  if (g_configid != FMUV6C_CONFIGID_CDC_MSC)
    {
      return OK;
    }

  ret = board_composite_switch(FMUV6C_CONFIGID_CDC);
  if (ret < 0)
    {
      return ret;
    }

  ret = nx_mount(FMUV6C_MICROSD_BLOCKDEV, FMUV6C_MICROSD_MOUNTPOINT,
                 "vfat", 0, NULL);
  if (ret < 0)
    {
      uerr("ERROR: failed to remount %s: %d\n",
           FMUV6C_MICROSD_MOUNTPOINT, ret);
    }

  return ret;
}

bool fmuv6c_msc_is_exported(void)
{
  return g_configid == FMUV6C_CONFIGID_CDC_MSC;
}

#endif /* CONFIG_USBMSC_COMPOSITE */
#endif /* CONFIG_USBDEV_COMPOSITE */
