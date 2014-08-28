/******************************************************************************
 * mt6260/mt6260_emif.h
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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

#ifndef __MT6260_MT6260_EMIF_H
#define __MT6260_MT6260_EMIF_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* External Memory Interface (EMIF) Registers */

#define MT6260_EMIF_CS0CTRL1     (MT6260_PERIPHERALS_VADDR + 0x0A00) /* CS0 Control Register #1 */
#define MT6260_EMIF_CS0CTRL2     (MT6260_PERIPHERALS_VADDR + 0x0A02) /* CS0 Control Register #2 */
#define MT6260_EMIF_CS0CTRL3     (MT6260_PERIPHERALS_VADDR + 0x0A04) /* CS0 Control Register #3 */
#define MT6260_EMIF_CS1CTRL1A    (MT6260_PERIPHERALS_VADDR + 0x0A06) /* CS1 Control Register #1A */
#define MT6260_EMIF_CS1CTRL1B    (MT6260_PERIPHERALS_VADDR + 0x0A08) /* CS1 Control Register #1B */
#define MT6260_EMIF_CS2CTRL2     (MT6260_PERIPHERALS_VADDR + 0x0A0A) /* CS1 Control Register #2 */
#define MT6260_EMIF_CS2CTRL1     (MT6260_PERIPHERALS_VADDR + 0x0A0C) /* CS2 Control Register #1 */
#define MT6260_EMIF_CS1CTRL2     (MT6260_PERIPHERALS_VADDR + 0x0A0E) /* CS2 Control Register #2 */
#define MT6260_EMIF_CS3CTRL1     (MT6260_PERIPHERALS_VADDR + 0x0A10) /* CS3 Control Register #1 */
#define MT6260_EMIF_CS3CTRL2     (MT6260_PERIPHERALS_VADDR + 0x0A12) /* CS3 Control Register #2 */
#define MT6260_EMIF_CS4CTRL1     (MT6260_PERIPHERALS_VADDR + 0x0A14) /* CS4 Control Register #1 */
#define MT6260_EMIF_CS4CTRL2     (MT6260_PERIPHERALS_VADDR + 0x0A16) /* CS4 Control Register #2 */
#define MT6260_EMIF_BUSCTRL      (MT6260_PERIPHERALS_VADDR + 0x0A18) /* Bus Control Register */
#define MT6260_EMIF_BUSRLS       (MT6260_PERIPHERALS_VADDR + 0x0A1A) /* Bus Release Control Register */
#define MT6260_EMIF_CFCTRL1      (MT6260_PERIPHERALS_VADDR + 0x0A1C) /* CFC ControlRegister #1 */
#define MT6260_EMIF_CFCTRL2      (MT6260_PERIPHERALS_VADDR + 0x0A1E) /* CFC ControlRegister#2 */
#define MT6260_EMIF_SMCTRL       (MT6260_PERIPHERALS_VADDR + 0x0A20) /* SmartMedia Control Register */
#define MT6260_EMIF_BUSINTEN     (MT6260_PERIPHERALS_VADDR + 0x0A22) /* Bus Interrupt Enable Register */
#define MT6260_EMIF_BUSSTS       (MT6260_PERIPHERALS_VADDR + 0x0A24) /* Bus Status Register */
#define MT6260_EMIF_BUSWAITMD    (MT6260_PERIPHERALS_VADDR + 0x0A26) /* Bus Wait Mode Register */
#define MT6260_EMIF_ECC1CP       (MT6260_PERIPHERALS_VADDR + 0x0A28) /* ECC Area 1 CP Register */
#define MT6260_EMIF_ECC1LP       (MT6260_PERIPHERALS_VADDR + 0x0A2A) /* ECC Area 1 LP Register */
#define MT6260_EMIF_ECC2CP       (MT6260_PERIPHERALS_VADDR + 0x0A2C) /* ECC Area 2 CP Register */
#define MT6260_EMIF_ECC2LP       (MT6260_PERIPHERALS_VADDR + 0x0A2E) /* ECC Area 2 LP Register */
#define MT6260_EMIF_ECC3CP       (MT6260_PERIPHERALS_VADDR + 0x0A30) /* ECC Area 3 CP Register */
#define MT6260_EMIF_ECC3LP       (MT6260_PERIPHERALS_VADDR + 0x0A32) /* ECC Area 3 LP Register */
#define MT6260_EMIF_ECC4CP       (MT6260_PERIPHERALS_VADDR + 0x0A34) /* ECC Area 4 CP Register */
#define MT6260_EMIF_ECC4LP       (MT6260_PERIPHERALS_VADDR + 0x0A36) /* ECC Area 4 LP Register */
#define MT6260_EMIF_ECC5CP       (MT6260_PERIPHERALS_VADDR + 0x0A38) /* ECC Area 5 CP Register */
#define MT6260_EMIF_ECC5LP       (MT6260_PERIPHERALS_VADDR + 0x0A3A) /* ECC Area 5 LP Register */
#define MT6260_EMIF_ECC6CP       (MT6260_PERIPHERALS_VADDR + 0x0A3C) /* ECC Area 6 CP Register */
#define MT6260_EMIF_ECC6LP       (MT6260_PERIPHERALS_VADDR + 0x0A3E) /* ECC Area 6 LP Register */
#define MT6260_EMIF_ECC7CP       (MT6260_PERIPHERALS_VADDR + 0x0A40) /* ECC Area 7 CP Register */
#define MT6260_EMIF_ECC7LP       (MT6260_PERIPHERALS_VADDR + 0x0A42) /* ECC Area 7 LP Register */
#define MT6260_EMIF_ECC8CP       (MT6260_PERIPHERALS_VADDR + 0x0A44) /* ECC Area 8 CP Register */
#define MT6260_EMIF_ECC8LP       (MT6260_PERIPHERALS_VADDR + 0x0A46) /* ECC Area 8 LP Register */
#define MT6260_EMIF_ECCCLR       (MT6260_PERIPHERALS_VADDR + 0x0A48) /* ECC Clear Register */
#define MT6260_EMIF_PAGESZ       (MT6260_PERIPHERALS_VADDR + 0x0A4A) /* SmartMedia Page Size Register */
#define MT6260_EMIF_PRIORCTL     (MT6260_PERIPHERALS_VADDR + 0x0A4C) /* Priority control for DMA */
#define MT6260_EMIF_IMGDSPDEST   (MT6260_PERIPHERALS_VADDR + 0x0A4E) /* DSP/IMGBUF DMA destination */
#define MT6260_EMIF_IMGDSPADDH   (MT6260_PERIPHERALS_VADDR + 0x0A50) /* DSP/IMGBUF high address */
#define MT6260_EMIF_IMGDSPADDL   (MT6260_PERIPHERALS_VADDR + 0x0A52) /* DSP/IMGBUG low address */
#define MT6260_EMIF_AHBADDH      (MT6260_PERIPHERALS_VADDR + 0x0A54) /* AHB high address */
#define MT6260_EMIF_AHBADDL      (MT6260_PERIPHERALS_VADDR + 0x0A56) /* AHB low address */
#define MT6260_EMIF_MTCADDH      (MT6260_PERIPHERALS_VADDR + 0x0A58) /* MTC high address */
#define MT6260_EMIF_MTCADDL      (MT6260_PERIPHERALS_VADDR + 0x0A5A) /* MTC low address */
#define MT6260_EMIF_DMASIZE      (MT6260_PERIPHERALS_VADDR + 0x0A5C) /* DMA Transfer Size Register */
#define MT6260_EMIF_DMAMTCSEL    (MT6260_PERIPHERALS_VADDR + 0x0A5E) /* DMA Device Select Register */
#define MT6260_EMIF_DMACTL       (MT6260_PERIPHERALS_VADDR + 0x0A60) /* DMA Control Register */
#define MT6260_EMIF_TEST         (MT6260_PERIPHERALS_VADDR + 0x0A62) /* Test Register.Do not use */

/******************************************************************************
 * Inline Functions
 ******************************************************************************/

#ifndef __ASSEMBLY__

#endif

#endif  /* __MT6260_MT6260_EMIF_H */
