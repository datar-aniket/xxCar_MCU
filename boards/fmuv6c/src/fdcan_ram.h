/****************************************************************************
 * boards/fmuv6c/src/fdcan_ram.h
 *
 * FDCAN message RAM layout, in WORDS from the start of the shared RAM.
 *
 * The 2560 words are shared between FDCAN1 and FDCAN2 and every region's
 * start address is assigned by software. Two regions that overlap corrupt
 * each other's frames and present as bus errors - not as a build failure -
 * so the arithmetic lives here on its own, where a host test can check it.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __BOARDS_FMUV6C_SRC_FDCAN_RAM_H
#define __BOARDS_FMUV6C_SRC_FDCAN_RAM_H

/* Total message RAM, and the half this board gives FDCAN1. Following PX4's
 * allocation, which splits it evenly so FDCAN2 can be brought up later
 * without moving anything.
 */

#define FDCAN_RAM_TOTAL_WORDS     2560u
#define FDCAN_RAM_FDCAN1_WORDS    1280u

/* Element sizes. RXESC is programmed to 0 (8-byte data), which is what
 * makes an Rx element four words. Change that and every offset below moves.
 */

#define FDCAN_RAM_STDFILT_WORDS   1u
#define FDCAN_RAM_EXTFILT_WORDS   2u
#define FDCAN_RAM_RXF0_WORDS      4u
#define FDCAN_RAM_TXF_WORDS       4u

#define FDCAN_RAM_STDFILT_OFF     0u
#define FDCAN_RAM_STDFILT_N       128u
#define FDCAN_RAM_EXTFILT_OFF     128u
#define FDCAN_RAM_EXTFILT_N       64u
#define FDCAN_RAM_RXF0_OFF        256u
#define FDCAN_RAM_RXF0_N          64u

/* THE TX FIFO IS RESERVED THOUGH NOTHING TRANSMITS YET. Laying it out later
 * would move nothing above it, but laying it out BEFORE the Rx FIFO would -
 * and shifting a live region's start address is a silent corruption rather
 * than a build error. Reserve it now, use it when control lands.
 */

#define FDCAN_RAM_TXF_OFF         512u
#define FDCAN_RAM_TXF_N           32u

#define FDCAN_RAM_USED            640u

#endif /* __BOARDS_FMUV6C_SRC_FDCAN_RAM_H */
