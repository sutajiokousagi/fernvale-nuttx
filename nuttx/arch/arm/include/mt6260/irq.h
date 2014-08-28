/****************************************************************************
 * arch/arm/include/mt6260/irq.h
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_MT6260_IRQ_H
#define __ARCH_ARM_INCLUDE_MT6260_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* MT6260 Interrupts */

#define MT6260_IRQ_TDMA      0 /* IRQ0:  */
#define MT6260_IRQ_CTIRQ1    1 /* IRQ1:  */
#define MT6260_IRQ_CTIRQ2    2 /* IRQ2:  */
#define MT6260_IRQ_GPI       3 /* IRQ3:  */
#define MT6260_IRQ_GPII      4 /* IRQ4:  */
#define MT6260_IRQ_HIF       5 /* IRQ5:  */
#define MT6260_IRQ_SW_LISR1  6 /* IRQ6:  */
#define MT6260_IRQ_SW_LISR2  7 /* IRQ7:  */
#define MT6260_IRQ_MD_SLEEP  8 /* IRQ8:  */
/* Reserved 9 */
#define MT6260_IRQ_DIE2DIE  10 /* IRQ10: */
#define MT6260_IRQ_OSTIMER  11 /* IRQ11: */
#define MT6260_IRQ_TOPSM    12 /* IRQ12: */
#define MT6260_IRQ_SIM      13 /* IRQ13: */
#define MT6260_IRQ_DMA      14 /* IRQ14: */
#define MT6260_IRQ_UART0    15 /* IRQ15: */
#define MT6260_IRQ_KPAD     16 /* IRQ16: */
#define MT6260_IRQ_UART1    17 /* IRQ17: */
#define MT6260_IRQ_GPT      18 /* IRQ18: */
#define MT6260_IRQ_EIT      19 /* IRQ19: */
#define MT6260_IRQ_USB      20 /* IRQ20: */
#define MT6260_IRQ_MSDC     21 /* IRQ21: */
#define MT6260_IRQ_SFI      22 /* IRQ22: */
#define MT6260_IRQ_LCD      23 /* IRQ23: */
/* Reserved 24 */
#define MT6260_IRQ_WDT      25 /* IRQ25: */
#define MT6260_IRQ_DSP2CPU  26 /* IRQ26: */
#define MT6260_IRQ_RESZ     27 /* IRQ27: */
#define MT6260_IRQ_G2D      28 /* IRQ28: */
#define MT6260_IRQ_MSDC_CD  29 /* IRQ29: */
#define MT6260_IRQ_SPI      30 /* IRQ30: */
#define MT6260_IRQ_I2C      31 /* IRQ31: */
#define MT6260_IRQ_IRDEBUG  32 /* IRQ32: */
#define MT6260_IRQ_SIM2     33 /* IRQ33: */
#define MT6260_IRQ_CAMERA   34 /* IRQ34: */
#define MT6260_IRQ_TIMCON   35 /* IRQ35: */
#define MT6260_IRQ_ROT_DMA  36 /* IRQ36: */
#define MT6260_IRQ_PAD2CAM  37 /* IRQ37: */
#define MT6260_IRQ_BTIF     38 /* IRQ38: */
/* Reserved 39 */
#define MT6260_IRQ_NFI      40 /* IRQ40: */
/* Reserved 41 */
#define MT6260_IRQ_FM       42 /* IRQ42: */
#define MT6260_IRQ_MSDC2    43 /* IRQ43: */
#define MT6260_IRQ_MSDC2_CD 44 /* IRQ44: */
/* Reserved 45 */
/* Reserved 46 */
/* Reserved 47 */

#define MT6260_IRQ_SYSTIMER MT6260_IRQ_TMR0
#define NR_IRQS             48

#define MT6260_INTC_MASK0       (MT6260_IRQ_REGISTER_BASE + 0x00)
#define MT6260_INTC_MASK1       (MT6260_IRQ_REGISTER_BASE + 0x00)
#define MT6260_INTC_MASK0_SET   (MT6260_IRQ_REGISTER_BASE + 0x20)
#define MT6260_INTC_MASK1_SET   (MT6260_IRQ_REGISTER_BASE + 0x24)
#define MT6260_INTC_MASK0_CLR   (MT6260_IRQ_REGISTER_BASE + 0x40)
#define MT6260_INTC_MASK1_CLR   (MT6260_IRQ_REGISTER_BASE + 0x40)

#define MT6260_INTC_ISTATUS0    (MT6260_IRQ_REGISTER_BASE + 0x120)
#define MT6260_INTC_ISTATUS1    (MT6260_IRQ_REGISTER_BASE + 0x124)
#define MT6260_INTC_ISTATUS_BIN (MT6260_IRQ_REGISTER_BASE + 0x1a0)

#define MT6260_INTC_ISTATUS_BIN_NOIRQ (1 << 8)

#define MT6260_INTC_FSTATUS0    (MT6260_IRQ_REGISTER_BASE + 0x140)
#define MT6260_INTC_FSTATUS1    (MT6260_IRQ_REGISTER_BASE + 0x144)
#define MT6260_INTC_FSTATUS_BIN (MT6260_IRQ_REGISTER_BASE + 0x1a4)

#define MT6260_INTC_ACK0        (MT6260_IRQ_REGISTER_BASE + 0x160)
#define MT6260_INTC_ACK1        (MT6260_IRQ_REGISTER_BASE + 0x164)
#define MT6260_INTC_ACK_BIN     (MT6260_IRQ_REGISTER_BASE + 0x1a8)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_MT6260_IRQ_H */

