/****************************************************************************
 * arch/arm/src/mt6260/mt6260_timerisr.c
 * arch/arm/src/chip/mt6260_timerisr.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>
#include <nuttx/arch.h>

#include "chip.h"
#include "up_arch.h"
#include "clock/clock.h"
#include "up_internal.h"
#include "mt6260_gpt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MT6260 Timers
 *
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

int up_timerisr(int irq, uint32_t *regs)
{
   /* Process timer interrupt */
  uint32_t gpt_num = gpt_read(GPT_STATUS);

  if (gpt_num == 1)
    sched_process_timer();
   return 0;
}

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  
  /* Start the timer */

  gpt_write(8192 / CLK_TCK, GPT_DAT1);
  gpt_write(GPT_PRESCALE1_16KHZ, GPT_PRESCALE1);

  /* Force readback, which seems necessary to get it activated */
  gpt_read(GPT_PRESCALE1);
  gpt_write(GPT_CON1_MODE_FREERUN | GPT_CON1_EN, GPT_CON1);
  gpt_read(GPT_CON1);
  gpt_write(GPT_CON1_MODE_FREERUN | GPT_CON1_EN, GPT_CON1);
  gpt_read(GPT_CON1);


  /* Attach and enable the timer interrupt */

  up_irq_level(MT6260_IRQ_SYSTIMER);
  irq_attach(MT6260_IRQ_SYSTIMER, (xcpt_t)up_timerisr);
  up_enable_irq(MT6260_IRQ_SYSTIMER);
}
