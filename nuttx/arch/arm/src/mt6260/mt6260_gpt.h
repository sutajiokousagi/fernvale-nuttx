/******************************************************************************
 * mt6260/mt6260_gpt.h
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

#ifndef __MT6260_GPT_H
#define __MT6260_GPT_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* GPT definitions ***********************************************************/

/* GPT Registers (offsets from the register base) */

#define GPT_CON1              0x0000 /* GPT 1 Control Register */
#define GPT_DAT1              0x0004 /* GPT 1 Time Out Interval Register */
#define GPT_CON2              0x0008 /* GPT 2 Control Register */
#define GPT_DAT2              0x000c /* GPT 2 Time Out Interval Register */
#define GPT_STATUS            0x0010 /* GPT Status Register */
#define GPT_PRESCALE1         0x0014 /* GPT 1 Prescale value */
#define GPT_PRESCALE2         0x0018 /* GPT 2 Prescale value */
#define GPT_CON3              0x001c /* GPT 3 Control Register */
#define GPT_DAT3              0x0020 /* GPT 3 Time Out Interval Register */
#define GPT_PRESCALE3         0x0014 /* GPT 3 Prescale value */
#define GPT_CON4              0x0028 /* GPT 4 Control Register */
#define GPT_DAT4              0x002c /* GPT 4 Time Out Interval Register */

#define GPT_CON1_EN           (1 << 15) /* Enable GPT1 */
#define GPT_CON1_MODE         (1 << 14) /* One-shot / Free-run mode */
#define GPT_CON1_MODE_ONESHOT  (0 << 14) /* One-shot mode */
#define GPT_CON1_MODE_FREERUN  (1 << 14) /* Free-run mode */
#define GPT_CON2_EN           (1 << 15) /* Enable GPT2 */
#define GPT_CON2_MODE         (1 << 14) /* One-shot / Free-run mode */
#define GPT_CON2_MODE_ONESHOT  (0 << 14) /* One-shot mode */
#define GPT_CON2_MODE_FREERUN  (1 << 14) /* Free-run mode */
#define GPT_CON3_EN           (1 << 0) /* Enable GPT3 */
#define GPT_CON4_EN           (1 << 0) /* Enable GPT4 */
#define GPT_CON4_LOCK         (1 << 1) /* Permanently lock GPT4 on */
#define GPT_PRESCALE1_16KHZ   (1 << 0)
#define GPT_PRESCALE1_8KHZ    (1 << 1)
#define GPT_PRESCALE1_4KHZ    (1 << 2)
#define GPT_PRESCALE1_2KHZ    (1 << 3)
#define GPT_PRESCALE1_1KHZ    (1 << 4)
#define GPT_PRESCALE1_500HZ   (1 << 5)
#define GPT_PRESCALE1_250HZ   (1 << 6)
#define GPT_PRESCALE1_125HZ   (1 << 7)
#define GPT_PRESCALE2_16KHZ   (1 << 0)
#define GPT_PRESCALE2_8KHZ    (1 << 1)
#define GPT_PRESCALE2_4KHZ    (1 << 2)
#define GPT_PRESCALE2_2KHZ    (1 << 3)
#define GPT_PRESCALE2_1KHZ    (1 << 4)
#define GPT_PRESCALE2_500HZ   (1 << 5)
#define GPT_PRESCALE2_250HZ   (1 << 6)
#define GPT_PRESCALE2_125HZ   (1 << 7)
#define GPT_PRESCALE3_16KHZ   (1 << 0)
#define GPT_PRESCALE3_8KHZ    (1 << 1)
#define GPT_PRESCALE3_4KHZ    (1 << 2)
#define GPT_PRESCALE3_2KHZ    (1 << 3)
#define GPT_PRESCALE3_1KHZ    (1 << 4)
#define GPT_PRESCALE3_500HZ   (1 << 5)
#define GPT_PRESCALE3_250HZ   (1 << 6)
#define GPT_PRESCALE3_125HZ   (1 << 7)

#define GPT_STATUS_GPT1       (1 << 0) /* GPT1 has fired */
#define GPT_STATUS_GPT2       (1 << 1) /* GPT2 has fired */

/******************************************************************************
 * Inline Functions
 ******************************************************************************/

static uint32_t gpt_read(uint32_t reg)
{
    return getreg32(MT6260_GPT_REGISTER_BASE + reg);
}

static void gpt_write(uint32_t val, uint32_t reg)
{
    putreg32(val, MT6260_GPT_REGISTER_BASE + reg);
}

#endif  /* __MT6260_GPT_H */
