/******************************************************************************
 * mt6260/mt6260_memorymap.h
 *
 *   Copyright (C) 2007, 2009-2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#ifndef __MT6260_MEMORYMAP_H
#define __MT6260_MEMORYMAP_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>
#include "arm.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Mapped base of all registers ***********************************************/

/* MT6260 Physical Memory Map, where:
 *
 *   CW = cachable with write buffering
 *   -W = Write buffering only
 *   -- = Neither
 *
 * NOTE:
 * 1. Most MT6260 memory sections can be programmed to lie at different
 *    locations in the memory map. Therefore, much of the MT6260 physical
 *    memory map is really board-specific and, as such, really belongs
 *    in the configs/<board>/include/board.h file rather than here.
 *
 *    To handle all cases, this file defines a "default" physical memory map,
 *    but section address for most regions can be overriden if the same
 *    setting is defined in the board.h file (These defaults correspond to
 *    the product Neuros OSD memory configuration).
 *
 * 2. The MT6260 only has a single control line for external peripherals.
 *    To support more than one peripheral, most hardware will use external
 *    memory decode logic, so that physical memory regions is in the
 *    board-specific files.
 */

/* Section/Region Name             Phys Address    Size  TLB Enty        CW */
#define   MT6260_PSRAM_ADDR           0x00000000 /*   1MB   1 large page  CW */

/* Sizes of sections/regions */

/* Section / Region Name             Size */
#define MT6260_PERIPHERALS_NSECTIONS  1          /*   1MB   1 section     -- */
#define   MT6260_PSRAM_SIZE           (8*1024*1024)

/* 16Kb of memory is reserved at the beginning of PSRAM to hold the
 * page table for the virtual mappings.  A portion of this table is
 * not accessible in the virtual address space (for normal operation).
 * We will reuse this memory for coarse page tables as follows:
 * FIXME!  Where does that 0x00000800 come from.  I can't remember
 * and it does not feel right!
 */
#define PGTABLE_BASE_PADDR          MT6260_PSRAM_ADDR + 1024
#define PGTABLE_PSRAM_PADDR         PGTABLE_BASE_PADDR
#define PGTABLE_L2_COARSE_PBASE     (PGTABLE_BASE_PADDR+0x00000800)
#define PGTABLE_L2_FINE_PBASE       (PGTABLE_BASE_PADDR+0x00001000)
#define PGTABLE_L2_END_PADDR        (PGTABLE_BASE_PADDR+PGTABLE_SIZE)

#define PGTABLE_BASE_VADDR          MT6260_PSRAM_ADDR + 1024
#define PGTABLE_PSRAM_VADDR         PGTABLE_BASE_VADDR
#define PGTABLE_L2_COARSE_VBASE     (PGTABLE_BASE_VADDR+0x00000800)
#define PGTABLE_L2_FINE_VBASE       (PGTABLE_BASE_VADDR+0x00001000)
#define PGTABLE_L2_END_VADDR        (PGTABLE_BASE_VADDR+PGTABLE_SIZE)

/* Page table sizes */

#define PGTABLE_L2_COARSE_ALLOC     (PGTABLE_L2_END_VADDR-PGTABLE_L2_COARSE_VBA
#define PGTABLE_COARSE_TABLE_SIZE   (4*256)
#define PGTABLE_NCOARSE_TABLES      (PGTABLE_L2_COARSE_ALLOC / PGTABLE_COARSE_T

#define PGTABLE_L2_FINE_ALLOC       (PGTABLE_L2_END_VADDR-PGTABLE_L2_FINE_VBASE
#define PGTABLE_FINE_TABLE_SIZE     (4*1024)
#define PGTABLE_NFINE_TABLES        (PGTABLE_L2_FINE_ALLOC / PGTABLE_FINE_TABLE

/* MT6260 Virtual Memory Map */

#if CONFIG_RAM_VSTART != 0x00000000
# error "Invalid setting for CONFIG_RAM_VSTART"
#endif

/* The NuttX entry point starts at an offset from the virtual beginning of DRAM.
 * This offset reserves space for the MMU page cache.
 */

#define NUTTX_START_VADDR            (MT6260_PSRAM_ADDR+1024+PGTABLE_SIZE)
#define NUTTX_START_PADDR            (MT6260_PSRAM_ADDR+1024+PGTABLE_SIZE)

/* This is the base address of the interrupt vectors on the ARM926 */

#define VECTOR_BASE                 MT6260_VECTOR_VADDR

/* MT6260 Peripheral Registers */

#define MT6260_IRQ_REGISTER_BASE    0xa0060000
#define MT6260_UART0_REGISTER_BASE  0xa0080000
#define MT6260_UART1_REGISTER_BASE  0xa0090000

/******************************************************************************
 * Inline Functions
 ******************************************************************************/

#ifndef __ASSEMBLY__

#endif

#endif  /* __MT6260_MEMORYMAP_H */
