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

#define MT6260_IRQ_UART0    15 /* IRQ15: UART 0 */
#define MT6260_IRQ_UART1    17 /* IRQ17: UART 1 */
#define MT6260_IRQ_GPT      23 /* IRQ23: General Purpose Timer */

#define MT6260_IRQ_SYSTIMER MT6260_IRQ_GPT
#define NR_IRQS             48

#define MT6260_INTC_MASK0       (MT6260_IRQ_REGISTER_BASE + 0x00)
#define MT6260_INTC_MASK1       (MT6260_IRQ_REGISTER_BASE + 0x04)
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

#define MT6260_INTC_LEVEL0      (MT6260_IRQ_REGISTER_BASE + 0x60)
#define MT6260_INTC_LEVEL1      (MT6260_IRQ_REGISTER_BASE + 0x64)
#define MT6260_INTC_LEVEL0_SET  (MT6260_IRQ_REGISTER_BASE + 0x80)
#define MT6260_INTC_LEVEL1_SET  (MT6260_IRQ_REGISTER_BASE + 0x84)
#define MT6260_INTC_LEVEL0_CLR  (MT6260_IRQ_REGISTER_BASE + 0xa0)
#define MT6260_INTC_LEVEL1_CLR  (MT6260_IRQ_REGISTER_BASE + 0xa4)

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

  uint32_t up_irq_count(int irq);
  const char *up_irq_name(int irq);
  void up_irq_edge(int irq);
  void up_irq_level(int irq);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_MT6260_IRQ_H */

