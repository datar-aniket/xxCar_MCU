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

#include <nuttx/clock.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "stm32_gpio.h"
#include "hardware/stm32_fdcan.h"
#include "hardware/stm32_rcc.h"

#include <arch/board/fdcan.h>
#include "fdcan_ram.h"
#include "fdcan_ring.h"
#include "fmuv6c.h"

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

/* The interrupt flags this driver acts on. RF0N is a new message in Rx
 * FIFO 0; RF0L is one the hardware had to discard because the FIFO was
 * already full. Both are write-1-to-clear in IR.
 */

#define FDCAN_IRQ_MASK            (FDCAN_IR_RF0N | FDCAN_IR_RF0L)

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
static bool g_irq_attached;
static bool g_sem_initialized;

/* The receive ring. `head` belongs to the interrupt handler and `tail` to
 * the task; neither writes the other's. See fdcan_ring.h for why that is
 * enough, and why one slot stays empty.
 */

static struct fdcan_frame_s g_ring[FDCAN_RING_N];
static volatile uint32_t g_head;
static volatile uint32_t g_tail;

/* Posted once per interrupt batch, not once per frame. The task drains the
 * whole ring on each wake, so a post per frame would only make it loop
 * through an empty ring counting down a semaphore.
 */

static sem_t g_rx_sem;

static int fdcan_isr(int irq, FAR void *context, FAR void *arg);

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

  if (g_ready)
    {
      return -EALREADY;
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

  /* Route both flags to interrupt line 0 (ILS = 0 selects line 0 for every
   * source) and enable that line. Enabling a source in IE without ILE is the
   * quiet failure here: the flag sets in IR, nothing reaches the NVIC, and
   * the driver looks like a bus with no traffic on it.
   */

  putreg32(FDCAN_IRQ_MASK, STM32_FDCAN1_IE);
  putreg32(0, STM32_FDCAN1_ILS);
  putreg32(FDCAN_ILE_EINT0, STM32_FDCAN1_ILE);

  /* IR is write-one-to-clear and can retain a pending receive indication
   * across a daemon restart. Clear it before the NVIC line is enabled.
   */

  putreg32(UINT32_MAX, STM32_FDCAN1_IR);

  memset(&g_stats, 0, sizeof(g_stats));
  g_head = 0;
  g_tail = 0;

  /* Matches the pattern the IMU drivers already use on this board: a plain
   * counting semaphore, posted from interrupt context, waited on with
   * nxsem_tickwait.
   */

  ret = nxsem_init(&g_rx_sem, 0, 0);

  if (ret < 0)
    {
      putreg32(0, STM32_FDCAN1_IE);
      putreg32(0, STM32_FDCAN1_ILE);
      return ret;
    }

  g_sem_initialized = true;

  ret = irq_attach(STM32_IRQ_FDCAN1_0, fdcan_isr, NULL);

  if (ret < 0)
    {
      nxsem_destroy(&g_rx_sem);
      g_sem_initialized = false;
      putreg32(0, STM32_FDCAN1_IE);
      putreg32(0, STM32_FDCAN1_ILE);
      return ret;
    }

  g_irq_attached = true;

  /* Ready BEFORE the first interrupt can arrive: the handler fills the ring
   * unconditionally, but fdcan_receive refuses to hand anything out until
   * this is set, and leaving INIT is what starts the traffic.
   */

  g_ready = true;

  fdcan_leave_config();
  up_enable_irq(STM32_IRQ_FDCAN1_0);
  return OK;
}

void fdcan_deinit(void)
{
  up_disable_irq(STM32_IRQ_FDCAN1_0);
  putreg32(0, STM32_FDCAN1_IE);
  putreg32(0, STM32_FDCAN1_ILE);
  g_ready = false;

  if (g_irq_attached)
    {
      irq_detach(STM32_IRQ_FDCAN1_0);
      g_irq_attached = false;
    }

  if (g_sem_initialized)
    {
      nxsem_destroy(&g_rx_sem);
      g_sem_initialized = false;
    }

  /* INIT stops both reception and command transmission. Interrupts and
   * public access are already disabled if the transition itself fails.
   */

  if (fdcan_enter_config() == OK)
    {
      putreg32(UINT32_MAX, STM32_FDCAN1_IR);
    }

  g_head = 0;
  g_tail = 0;
}

/* Move one element out of Rx FIFO 0 into `frame`, and release the slot.
 *
 * Shared by the interrupt handler; split out because acknowledging the FIFO
 * has to happen on every path, including the ones that discard the frame.
 */

enum fdcan_take_e
{
  FDCAN_TAKE_EMPTY = 0,
  FDCAN_TAKE_ACCEPTED,
  FDCAN_TAKE_REJECTED
};

static enum fdcan_take_e fdcan_fifo_take(FAR struct fdcan_frame_s *frame)
{
  uint32_t status;
  uint32_t index;
  uint32_t element;
  uint32_t w0;
  uint32_t w1;
  uint32_t i;

  status = getreg32(STM32_FDCAN1_RXF0S);

  if ((status & FDCAN_RXF0S_F0FL_MASK) == 0)
    {
      return FDCAN_TAKE_EMPTY;
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
      return FDCAN_TAKE_REJECTED;
    }

  return FDCAN_TAKE_ACCEPTED;
}

static int fdcan_isr(int irq, FAR void *context, FAR void *arg)
{
  uint32_t pending;
  bool arrived = false;
  int guard;

  pending = getreg32(STM32_FDCAN1_IR) & FDCAN_IRQ_MASK;

  /* Clear BEFORE draining. A frame that lands while this handler is running
   * sets RF0N again, and clearing afterwards would wipe that notification
   * along with the one being serviced - leaving a frame in the FIFO with
   * nothing scheduled to come back for it.
   */

  putreg32(pending, STM32_FDCAN1_IR);

  if ((pending & FDCAN_IR_RF0L) != 0)
    {
      g_stats.lost++;
    }

  /* Bounded by the hardware FIFO depth, which is the most that can be
   * waiting. It cannot spin: a frame takes at least ~50 us on the wire at
   * 1 Mbit/s and about a microsecond to move out of the FIFO here, so the
   * drain always wins. The bound only guarantees that if the assumption is
   * ever wrong, this leaves interrupt context anyway.
   */

  for (guard = 0; guard < (int)FDCAN_RAM_RXF0_N; guard++)
    {
      struct fdcan_frame_s frame;
      enum fdcan_take_e taken;

      taken = fdcan_fifo_take(&frame);

      if (taken == FDCAN_TAKE_EMPTY)
        {
          break;
        }

      if (taken == FDCAN_TAKE_REJECTED)
        {
          continue;
        }

      if (fdcan_ring_full(g_head, g_tail))
        {
          /* The task has not kept up. Drop the newest rather than overwrite
           * the oldest: the tail is the task's index and moving it from here
           * is the one thing that would need a lock.
           */

          g_stats.ring_full++;
          continue;
        }

      frame.ts = fmuv6c_imu_time_now();
      g_ring[g_head] = frame;

      /* Publish the slot only after its complete frame is visible. The task
       * performs the matching acquire barrier after observing head.
       */

      memory_barrier();
      g_head = fdcan_ring_next(g_head);
      g_stats.rx++;
      arrived = true;
    }

  if (arrived)
    {
      int value = 0;

      /* Only post when the task is actually waiting. Otherwise the count
       * builds up while it is busy and it spins through an empty ring that
       * many times before blocking again.
       */

      if (nxsem_get_value(&g_rx_sem, &value) == OK && value <= 0)
        {
          nxsem_post(&g_rx_sem);
        }
    }

  UNUSED(irq);
  UNUSED(context);
  UNUSED(arg);
  return OK;
}

int fdcan_wait(uint32_t timeout_us)
{
  sclock_t ticks;
  uint64_t rounded;

  if (!g_ready)
    {
      return -EINVAL;
    }

  if (!fdcan_ring_empty(g_head, g_tail))
    {
      return OK;
    }

  /* Round up. USEC2TICK truncates, and a timeout that rounds to zero would
   * turn this into a spin.
   */

  rounded = (uint64_t)timeout_us + USEC_PER_TICK - 1u;
  ticks = (sclock_t)(rounded / USEC_PER_TICK);

  if (ticks < 1)
    {
      ticks = 1;
    }

  return nxsem_tickwait(&g_rx_sem, ticks);
}

int fdcan_receive(FAR struct fdcan_frame_s *frame)
{
  if (!g_ready || frame == NULL)
    {
      return -EINVAL;
    }

  if (fdcan_ring_empty(g_head, g_tail))
    {
      return -EAGAIN;
    }

  memory_barrier();
  *frame = g_ring[g_tail];
  g_tail = fdcan_ring_next(g_tail);
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
  irqstate_t flags;
  uint32_t psr;

  if (out == NULL)
    {
      return;
    }

  psr = getreg32(STM32_FDCAN1_PSR);
  flags = enter_critical_section();
  g_stats.last_error = (uint8_t)(psr & FDCAN_PSR_LEC_MASK);
  g_stats.bus_off = (psr & FDCAN_PSR_BO_MASK) != 0;
  g_stats.error_passive = (psr & FDCAN_PSR_EP_MASK) != 0;

  *out = g_stats;
  leave_critical_section(flags);
}
