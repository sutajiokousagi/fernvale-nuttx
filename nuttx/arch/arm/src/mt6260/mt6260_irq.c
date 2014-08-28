/************************************************************************
 * arch/arm/src/mt6260/mt6260_irq.c
 *
 *   Copyright (C) 2007, 2009, 2012 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/irq.h>

#include "arm.h"
#include "chip.h"

#include "up_arch.h"
#include "up_internal.h"

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/************************************************************************
 * Public Data
 ************************************************************************/

volatile uint32_t *current_regs;

/************************************************************************
 * Private Data
 ************************************************************************/

/* The value of _svectors is defined in ld.script.  It could be hard-
 * coded because we know that correct IRAM area is 0xffc00000.
 */

extern int _svectors; /* Type does not matter */

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: up_irqinitialize
 ************************************************************************/

void up_irqinitialize(void)
{
  /* Clear, disable and configure all interrupts. */

  /* XXX Set IRQ ordering here */

  putreg32(0xffffffff, MT6260_INTC_MASK0);      /* Mask all IRQs/FIQs */
  putreg32(0xffffffff, MT6260_INTC_MASK1);

  putreg32(0xffffffff, MT6260_INTC_ACK0);  /* Clear all pending IRQs */
  putreg32(0xffffffff, MT6260_INTC_ACK1);

  /* XXX Enable Die2Die interrupts here */

  /* currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  irqrestore(SVC_MODE | PSR_F_BIT);
#endif
}

/************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ************************************************************************/

void up_disable_irq(int irq)
{
  /* Disable the interrupt by setting the corresponding bit in
   * the IRQ mask register.
   */

  if (irq < 32)
    {
      /* IRQs0-31 are controlled by the IRQ0 mask register
       * Set the associated bit to disable the interrupt
       */

      putreg32((1 << irq), MT6260_INTC_MASK0_SET);
    }
  else
    {
      /* IRQs16-31 are controlled by the IRQ1 mask register
       * Set the associated bit to disable the interrupt
       */

      putreg32((1 << (irq-32)), MT6260_INTC_MASK1_SET);
    }
}

/************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ************************************************************************/

void up_enable_irq(int irq)
{
  /* Enable the interrupt by setting the corresponding bit in
   * the IRQ enable register.
   */

  if (irq < 32)
    {
      /* IRQs0-32 are controlled by the IRQ0 mask register
       * Clear the associated bit to enable the interrupt
       */

      putreg32((1 << irq), MT6260_INTC_MASK0_CLR);
    }
  else
    {
      /* IRQs32-47 are controlled by the IRQ1 mask register
       * Clear the associated bit to enable the interrupt
       */

      putreg32((1 << (irq-32)), MT6260_INTC_MASK1_CLR);
    }
}

/************************************************************************
 * Name: up_maskack_irq
 *
 * Description:
 *   Mask the IRQ and acknowledge it
 *
 ************************************************************************/

void up_maskack_irq(int irq)
{
  /* Disable the interrupt by clearing the corresponding bit in
   * the IRQ enable register.  And acknowlege it by setting the
   * corresponding bit in the IRQ status register.
   */

  if (irq < 32)
    {
      /* IRQs0-31 are controlled by the IRQ0 mask register
       * Set the associated mask bit to disable the interrupt
       * Set the associated status bit to clear the interrupt
       */

      putreg32((1 << irq), MT6260_INTC_MASK0_SET);
      putreg32((1 << irq), MT6260_INTC_ACK0);
    }
  else
    {
      /* IRQs32-47 are controlled by the IRQ1 mask register
       * Set the associated mask bit to disable the interrupt
       * Set the associated status bit to clear the interrupt
       */

      putreg32((1 << irq), MT6260_INTC_MASK1_SET);
      putreg32((1 << (irq-32)), MT6260_INTC_ACK1);
    }
}
