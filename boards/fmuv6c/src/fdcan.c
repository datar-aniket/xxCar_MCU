/****************************************************************************
 * boards/fmuv6c/src/fdcan.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32_gpio.h"
#include "hardware/stm32_fdcan.h"
#include "hardware/stm32_rcc.h"

#include <arch/board/fdcan.h>
#include "fdcan_ram.h"

/* NuttX's stm32_fdcan.h defines NTSEG1, NBRP and NSJW but NOT NTSEG2.
 * Reference manual: NBTP bits [6:0]. Without this the field would land at
 * whatever shift a typo chose, and 15 tq instead of 16 is a bit rate 7% out
 * - close enough to sometimes work, which is the worst kind of wrong.
 */

#define FDCAN_NBTP_NTSEG2_SHIFT   (0U)

/* Bit timing for 1 Mbit/s from the 16 MHz HSE kernel clock.
 *
 *   tq per bit  = 16 MHz / 1 Mbit/s = 16
 *   prescaler   = 1                       -> NBRP  = 0
 *   bs1_bs2_sum = 15
 *   bs1 = (7 * 15 - 1) / 8 = 13           -> NTSEG1 = 12
 *   bs2 = 15 - 13         =  2            -> NTSEG2 = 1
 *   sjw = 1                               -> NSJW   = 0
 *
 * Registers hold each value MINUS ONE, which is why these look off by one.
 * Sample point is (1 + 13) / 16 = 87.5%, the CiA recommendation, and PX4's
 * generic solver produces exactly these values for this clock and rate.
 */

#define FDCAN_BITRATE_SUPPORTED   1000000u
#define FDCAN_NBRP_1M             0u
#define FDCAN_NTSEG1_1M           12u
#define FDCAN_NTSEG2_1M           1u
#define FDCAN_NSJW_1M             0u

#define FDCAN_RAM_WORD(n)         (STM32_CANRAM_BASE + ((n) * 4u))

/* Extended filter element, two words:
 *   word 0: EFEC[31:29] | EFID1[28:0]
 *   word 1: EFT[31:30]  | EFID2[28:0]
 */

#define FDCAN_EFEC_STORE_FIFO0    (1u << 29)
#define FDCAN_EFT_CLASSIC         (2u << 30)

/* RX element word 0 flags. */

#define FDCAN_RX_XTD              (1u << 30)
#define FDCAN_RX_RTR              (1u << 29)

/* Tx element word 0 flags. Same bit positions as the Rx element, which is
 * not a coincidence - the two share a layout.
 */

#define FDCAN_TX_XTD              (1u << 30)

/* Tx element word 1: DLC occupies bits [19:16]. FDF and BRS stay clear;
 * setting either would make this a CAN FD frame, which the four-word element
 * size cannot hold.
 */

#define FDCAN_TX_DLC_SHIFT        (16u)

/* GFC: reject everything that does not match, including remote frames.
 * ANFE/ANFS 3 = reject non-matching.
 */

#define FDCAN_GFC_REJECT_ALL      ((3u << FDCAN_GFC_ANFE_SHIFT) | \
                                   (3u << FDCAN_GFC_ANFS_SHIFT) | \
                                   FDCAN_GFC_RRFE | FDCAN_GFC_RRFS)

/* ANFE 0 = accept non-matching extended into FIFO0. Standard frames are
 * still rejected - every frame in this protocol is extended.
 */

#define FDCAN_GFC_ACCEPT_EXT      ((0u << FDCAN_GFC_ANFE_SHIFT) | \
                                   (3u << FDCAN_GFC_ANFS_SHIFT) | \
                                   FDCAN_GFC_RRFE | FDCAN_GFC_RRFS)

static struct fdcan_stats_s g_stats;
static bool g_ready;

static void fdcan_ram_clear(void)
{
  uint32_t i;

  /* Message RAM is NOT cleared by a peripheral reset and comes up holding
   * whatever was there before. An uninitialised filter element is a filter,
   * and it will happily accept or reject traffic nobody asked it to.
   */

  for (i = 0; i < FDCAN_RAM_USED; i++)
    {
      putreg32(0, FDCAN_RAM_WORD(i));
    }
}

static int fdcan_enter_config(void)
{
  uint32_t regval;
  int guard;

  /* INIT is not immediate. Configuring before it latches silently does
   * nothing, which is the classic way to get a peripheral that looks
   * configured and is not.
   */

  regval = getreg32(STM32_FDCAN1_CCCR);
  regval |= FDCAN_CCCR_INIT;
  putreg32(regval, STM32_FDCAN1_CCCR);

  for (guard = 0; guard < 100000; guard++)
    {
      if ((getreg32(STM32_FDCAN1_CCCR) & FDCAN_CCCR_INIT) != 0)
        {
          break;
        }
    }

  if ((getreg32(STM32_FDCAN1_CCCR) & FDCAN_CCCR_INIT) == 0)
    {
      return -ETIMEDOUT;
    }

  regval = getreg32(STM32_FDCAN1_CCCR);
  regval |= FDCAN_CCCR_CCE;
  putreg32(regval, STM32_FDCAN1_CCCR);
  return OK;
}

static void fdcan_leave_config(void)
{
  uint32_t regval;

  regval = getreg32(STM32_FDCAN1_CCCR);
  regval &= ~FDCAN_CCCR_INIT;
  putreg32(regval, STM32_FDCAN1_CCCR);
}

/* The register half of setting a filter. GFC and XIDFC are write-protected
 * outside INIT + CCE: the caller MUST already hold configuration mode or
 * these writes are discarded without any error to notice.
 */

static void fdcan_write_filter(uint8_t controller_id)
{
  if (controller_id == 0)
    {
      /* Discovery: take every extended frame, whatever its id. */

      putreg32(0, STM32_FDCAN1_XIDFC);
      putreg32(FDCAN_GFC_ACCEPT_EXT, STM32_FDCAN1_GFC);
      return;
    }

  /* One classic filter: match the low byte, ignore the packet id above it,
   * so every packet type from this one node arrives and nothing else does.
   */

  putreg32(FDCAN_EFEC_STORE_FIFO0 | controller_id,
           FDCAN_RAM_WORD(FDCAN_RAM_EXTFILT_OFF));
  putreg32(FDCAN_EFT_CLASSIC | 0xffu,
           FDCAN_RAM_WORD(FDCAN_RAM_EXTFILT_OFF + 1));

  putreg32((1u << FDCAN_XIDFC_LSE_SHIFT) |
           (FDCAN_RAM_EXTFILT_OFF << FDCAN_XIDFC_FLESA_SHIFT),
           STM32_FDCAN1_XIDFC);
  putreg32(FDCAN_GFC_REJECT_ALL, STM32_FDCAN1_GFC);
}

int fdcan_set_filter(uint8_t controller_id)
{
  int ret;

  if (!g_ready)
    {
      return -EINVAL;
    }

  /* Narrowing the filter after discovery means going back into
   * configuration mode. The bus is off air for the few microseconds this
   * takes, so frames in flight are lost - which is why it happens once,
   * when the node id is learnt, and not per frame.
   */

  ret = fdcan_enter_config();
  if (ret < 0)
    {
      return ret;
    }

  fdcan_write_filter(controller_id);
  fdcan_leave_config();
  return OK;
}

int fdcan_init(uint32_t bitrate)
{
  uint32_t regval;
  int ret;

  /* Only the timing that has been derived and checked. A different bitrate
   * would need different NBTP values, and quietly using 1 Mbit/s timing for
   * a 500 kbit/s bus produces a link that half works.
   */

  if (bitrate != FDCAN_BITRATE_SUPPORTED)
    {
      return -ENOTSUP;
    }

  regval = getreg32(STM32_RCC_APB1HENR);
  regval |= RCC_APB1HENR_FDCANEN;
  putreg32(regval, STM32_RCC_APB1HENR);

  stm32_configgpio(GPIO_CAN1_RX);
  stm32_configgpio(GPIO_CAN1_TX);

  ret = fdcan_enter_config();
  if (ret < 0)
    {
      return ret;
    }

  fdcan_ram_clear();

  putreg32((FDCAN_NSJW_1M   << FDCAN_NBTP_NSJW_SHIFT)   |
           (FDCAN_NBRP_1M   << FDCAN_NBTP_NBRP_SHIFT)   |
           (FDCAN_NTSEG1_1M << FDCAN_NBTP_NTSEG1_SHIFT) |
           (FDCAN_NTSEG2_1M << FDCAN_NBTP_NTSEG2_SHIFT),
           STM32_FDCAN1_NBTP);

  /* The start-address fields sit at bit 2, which turns a word offset into
   * the byte address the hardware wants. Writing a byte offset here would
   * place every region four times too far out - into FDCAN2's half.
   */

  putreg32((FDCAN_RAM_STDFILT_N << FDCAN_SIDFC_LSS_SHIFT) |
           (FDCAN_RAM_STDFILT_OFF << FDCAN_SIDFC_FLSSA_SHIFT),
           STM32_FDCAN1_SIDFC);

  putreg32((FDCAN_RAM_RXF0_N << FDCAN_RXF0C_F0S_SHIFT) |
           (FDCAN_RAM_RXF0_OFF << FDCAN_RXF0C_F0SA_SHIFT),
           STM32_FDCAN1_RXF0C);

  putreg32((FDCAN_RAM_TXF_N << FDCAN_TXBC_TFQS_SHIFT) |
           (FDCAN_RAM_TXF_OFF << FDCAN_TXBC_TBSA_SHIFT),
           STM32_FDCAN1_TXBC);

  /* RXESC 0 in every field: 8-byte data. Explicit rather than relying on the
   * reset value, because it is what makes the element four words - and the
   * whole RAM table above assumes four.
   */

  putreg32(0, STM32_FDCAN1_RXESC);

  /* TXESC 0: 8-byte data, matching RXESC. Explicit for the same reason - the
   * reset value happens to be what the four-word element in fdcan_ram.h
   * assumes, and leaning on a reset value for a number the RAM layout
   * depends on is how the layout quietly stops being true.
   */

  putreg32(0, STM32_FDCAN1_TXESC);

  fdcan_write_filter(0);

  fdcan_leave_config();

  memset(&g_stats, 0, sizeof(g_stats));
  g_ready = true;
  return OK;
}

int fdcan_receive(FAR struct fdcan_frame_s *frame)
{
  uint32_t status;
  uint32_t index;
  uint32_t element;
  uint32_t w0;
  uint32_t w1;
  uint32_t i;

  if (!g_ready || frame == NULL)
    {
      return -EINVAL;
    }

  status = getreg32(STM32_FDCAN1_RXF0S);

  /* F0FL is the fill level. Everything else in this register is about
   * WHERE, not whether.
   */

  if ((status & FDCAN_RXF0S_F0FL_MASK) == 0)
    {
      return -EAGAIN;
    }

  index = (status & FDCAN_RXF0S_F0GI_MASK) >> FDCAN_RXF0S_F0GI_SHIFT;
  element = FDCAN_RAM_RXF0_OFF + (index * 4u);

  w0 = getreg32(FDCAN_RAM_WORD(element));
  w1 = getreg32(FDCAN_RAM_WORD(element + 1));

  frame->id = w0 & 0x1fffffffu;
  frame->dlc = (uint8_t)((w1 >> 16) & 0xfu);

  if (frame->dlc > 8)
    {
      frame->dlc = 8;
    }

  for (i = 0; i < frame->dlc; i++)
    {
      uint32_t word = getreg32(FDCAN_RAM_WORD(element + 2 + (i / 4u)));

      frame->data[i] = (uint8_t)((word >> ((i % 4u) * 8u)) & 0xffu);
    }

  /* Acknowledge BEFORE deciding whether we want it: the FIFO slot has to be
   * released either way, and returning early without acknowledging wedges
   * the FIFO one frame at a time until it overruns.
   */

  putreg32(index, STM32_FDCAN1_RXF0A);

  /* Every frame in this protocol is an extended DATA frame. A standard or
   * remote frame here is somebody else's traffic.
   */

  if ((w0 & FDCAN_RX_XTD) == 0 || (w0 & FDCAN_RX_RTR) != 0)
    {
      g_stats.rejected++;
      return -EAGAIN;
    }

  g_stats.rx++;
  return OK;
}

int fdcan_transmit(FAR const struct fdcan_frame_s *frame)
{
  uint32_t status;
  uint32_t index;
  uint32_t element;
  uint32_t word;
  uint32_t i;

  if (!g_ready || frame == NULL || frame->dlc > 8)
    {
      return -EINVAL;
    }

  status = getreg32(STM32_FDCAN1_TXFQS);

  if ((status & FDCAN_TXFQS_TFQF) != 0)
    {
      /* Nothing on the bus is acknowledging, so the hardware is still
       * retrying frames queued up to 0.6 s ago. Dropping this one is right:
       * at 50 Hz the next carries fresher intent than anything stuck in the
       * queue.
       */

      g_stats.tx_full++;
      return -EAGAIN;
    }

  index = (status & FDCAN_TXFQS_TFQPI_MASK) >> FDCAN_TXFQS_TFQPI_SHIFT;
  element = FDCAN_RAM_TXF_OFF + (index * 4u);

  putreg32(FDCAN_TX_XTD | (frame->id & 0x1fffffffu),
           FDCAN_RAM_WORD(element));
  putreg32((uint32_t)frame->dlc << FDCAN_TX_DLC_SHIFT,
           FDCAN_RAM_WORD(element + 1));

  /* Data words are packed least significant byte first WITHIN each word,
   * while the CAN payload itself is big-endian. Those are two different
   * things: the byte order on the wire was already decided by the encoder,
   * and this only moves bytes into the message RAM in the order the
   * peripheral reads them out.
   */

  for (i = 0; i < 8u; i += 4u)
    {
      word = 0;

      if (i + 0u < frame->dlc)
        {
          word |= (uint32_t)frame->data[i + 0u];
        }

      if (i + 1u < frame->dlc)
        {
          word |= (uint32_t)frame->data[i + 1u] << 8;
        }

      if (i + 2u < frame->dlc)
        {
          word |= (uint32_t)frame->data[i + 2u] << 16;
        }

      if (i + 3u < frame->dlc)
        {
          word |= (uint32_t)frame->data[i + 3u] << 24;
        }

      putreg32(word, FDCAN_RAM_WORD(element + 2 + (i / 4u)));
    }

  /* Add request: one bit per element index. */

  putreg32(1u << index, STM32_FDCAN1_TXBAR);

  g_stats.tx++;
  return OK;
}

void fdcan_stats(FAR struct fdcan_stats_s *out)
{
  uint32_t psr;

  if (out == NULL)
    {
      return;
    }

  psr = getreg32(STM32_FDCAN1_PSR);
  g_stats.last_error = (uint8_t)(psr & FDCAN_PSR_LEC_MASK);
  g_stats.bus_off = (psr & FDCAN_PSR_BO_MASK) != 0;
  g_stats.error_passive = (psr & FDCAN_PSR_EP_MASK) != 0;

  if ((getreg32(STM32_FDCAN1_RXF0S) & FDCAN_RXF0S_RF0L) != 0)
    {
      g_stats.lost++;
    }

  *out = g_stats;
}
