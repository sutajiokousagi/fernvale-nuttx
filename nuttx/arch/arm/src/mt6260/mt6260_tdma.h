/******************************************************************************
 * mt6260/mt6260_tdma.h
 *
 *   Copyright (C) 2014 Sean Cross. All rights reserved.
 *   Author: Sean Cross <xobs@kosagi.com>
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

#ifndef __MT6260_TDMA_H
#define __MT6260_TDMA_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* TDMA definitions ***********************************************************/

/* TDMA Registers (offsets from the register base) */

#define TDMA_TQCNT            0x0000 /* Read Quarter Bit Counter Register */
#define TDMA_WRAP             0x0004 /* Latched Qbit Counter Reset Position Register */
#define TDMA_WRAPIMD          0x0008 /* Direct Qbit Counter Reset Position Register */
#define TDMA_EVTVAL           0x000c /* Event Latch Position Register */
#define TDMA_DTIRQ            0x0010 /* DSP Software Control Register */
#define TDMA_CTIRQ1           0x0014 /* MCU Software Control Register 1 */
#define TDMA_CTIRQ2           0x0018 /* MCU Software Control Register 2 */
#define TDMA_AFC0             0x0020 /* AFC 0 Control Register */
#define TDMA_AFC1             0x0024 /* AFC 1 Control Register */
#define TDMA_AFC2             0x0028 /* AFC 2 Control Register */
#define TDMA_AFC3             0x002c /* AFC 3 Control Register */

#define TDMA_EVTENA0          0x0150 /* Event Enable Register 0 */
#define TDMA_EVTENA1          0x0154 /* Event Enable Register 1 */
#define TDMA_EVTENA2          0x0158 /* Event Enable Register 2 */
#define TDMA_EVTENA3          0x015c /* Event Enable Register 3 */
#define TDMA_EVTENA4          0x0160 /* Event Enable Register 4 */
#define TDMA_EVTENA5          0x0164 /* Event Enable Register 5 */
#define TDMA_WRAPOFS          0x0170 /* Qbit Timer Offset Control Register */
#define TDMA_REGBIAS          0x0174 /* Qbit Timer Biasing Control Register */
#define TDMA_DTXCON           0x0180 /* DTX Control Register */
#define TDMA_RXCON            0x0184 /* Receive Interrupt Control Register */
#define TDMA_BDLCON           0x0188 /* Baseband Downlink Control Register */
#define TDMA_BULCON1          0x018c /* Baseband Uplink Control Register 1 */
#define TDMA_BULCON2          0x0190 /* Baseband Uplink Control Register 2 */

#define TDMA_EVTENA0_DTIRQ    (1 << 0)  /* Enable DT IRQ */
#define TDMA_EVTENA0_CTIRQ1   (1 << 1)  /* Enable CT1 IRQ */
#define TDMA_EVTENA0_CTIRQ2   (1 << 2)  /* Enable CT2 IRQ */
#define TDMA_EVTENA0_BDL0     (1 << 6)  /* Enable BDL0 ON and OFF IRQ */
#define TDMA_EVTENA0_BDL1     (1 << 7)  /* Enable BDL1 ON and OFF IRQ */
#define TDMA_EVTENA0_BDL2     (1 << 8)  /* Enable BDL2 ON and OFF IRQ */
#define TDMA_EVTENA0_BDL3     (1 << 9)  /* Enable BDL3 ON and OFF IRQ */
#define TDMA_EVTENA0_BDL4     (1 << 10) /* Enable BDL4 ON and OFF IRQ */
#define TDMA_EVTENA0_BDL5     (1 << 11) /* Enable BDL5 ON and OFF IRQ */
#define TDMA_EVTENA0_AFC0     (1 << 12) /* Enable AFC0 */
#define TDMA_EVTENA0_AFC1     (1 << 13) /* Enable AFC1 */
#define TDMA_EVTENA0_AFC2     (1 << 14) /* Enable AFC2 */
#define TDMA_EVTENA0_AFC3     (1 << 15) /* Enable AFC3 */

#define TDMA_EVTENA1_APC0     (1 << 0)  /* Enable APC0 */
#define TDMA_EVTENA1_APC1     (1 << 1)  /* Enable APC1 */
#define TDMA_EVTENA1_APC2     (1 << 2)  /* Enable APC2 */
#define TDMA_EVTENA1_APC3     (1 << 3)  /* Enable APC3 */
#define TDMA_EVTENA1_APC4     (1 << 4)  /* Enable APC4 */
#define TDMA_EVTENA1_APC5     (1 << 5)  /* Enable APC5 */
#define TDMA_EVTENA1_APC6     (1 << 6)  /* Enable APC6 */
#define TDMA_EVTENA1_BUL0     (1 << 7)  /* Enable BUL0 ON and OFF */
#define TDMA_EVTENA1_BUL1     (1 << 8)  /* Enable BUL1 ON and OFF */
#define TDMA_EVTENA1_BUL2     (1 << 9)  /* Enable BUL2 ON and OFF */
#define TDMA_EVTENA1_BUL3     (1 << 10) /* Enable BUL3 ON and OFF */

/******************************************************************************
 * Inline Functions
 ******************************************************************************/

#endif  /* __MT6260_TDMA_H */
