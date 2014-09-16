/****************************************************************************
 * arch/arm/src/arm/up_head.S
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm.h"
#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/* The ARM9 L1 page table can be placed at the beginning or at the end of
 * the RAM space.  This decision is based on the placement of the vector
 * area: If the vectors are place in low memory at address 0x0000 0000, then
 * the page table is placed in high memory; if the vectors are placed in
 * high memory at address 0xfff0 0000, then the page table is locating at
 * the beginning of RAM.
 *
 * For the special case where (1) the program executes out of RAM, and (2)
 * the page is located at the beginning of RAM (i.e., the high vector case),
 * then the following macro can easily find the physical address of the
 * section that includes the first part of the text region:  Since the page
 * table is closely related to the NuttX base address in this case, we can
 * convert the page table base address to the base address of the section
 * containing both.
 */

/* This macro will modify r0, r1, r2 and r14 */

#ifdef CONFIG_DEBUG
	.macro	showprogress, code
	mov		r0, #\code
	bl		up_lowputc
	.endm
#else
	.macro	showprogress, code
	.endm
#endif

/****************************************************************************
 * Name: _start (called from __start, which is the vector table)
 ****************************************************************************/

/* We assume the bootloader has already initialized most of the h/w for us
 * and that only leaves us having to do some os specific things below.
 */
	.text
	.global	_start
	.type	_start, #function
_start:

  showprogress 'A'
	mov		r0, #(SVC_MODE | PSR_I_BIT | PSR_F_BIT)
	msr		cpsr_c, r0
  showprogress 'B'

  b     .Lvstart

/****************************************************************************
 * PC_Relative Data
 ****************************************************************************/

	/* Most addresses are all virtual address */
	
	.type	.LCvstart, %object
.LCvstart:
	.long	.Lvstart

	.size	_start, .-_start

/****************************************************************************
 * Name: .Lvstart
 ***************************************************************************/

/* The following is executed after the MMU has been enabled. This uses
 * absolute addresses; this is not position independent.
 */
	.align	5
	.local	.Lvstart
	.type	.Lvstart, %function
.Lvstart:

  showprogress 'E'

	/* Zero BSS and set up the stack pointer */

  showprogress 'J'
	adr		r0, .Linitparms
	ldmia	r0, {r0, r1, sp}

	/* Clear the frame pointer and .bss */

	mov		fp, #0
  showprogress 'K'

.Lbssinit:	
	cmp		r0, r1					/* Clear up to _bss_end_ */
	strcc	fp, [r0],#4
	bcc		.Lbssinit

	/* If the .data section is in a separate, unitialized address space,
	 * then we will also need to copy the initial values of of the .data
	 * section from the .text region into that .data region.  This would
	 * be the case if we are executing from FLASH and the .data section
	 * lies in a different physical address region OR if we are support
	 * on-demand paging and the .data section lies in a different virtual
	 * address region.
	 */

  showprogress 'L'

	/* Perform early C-level, platform-specific initialization */

	bl		up_boot

  showprogress 'M'
#ifdef CONFIG_DEBUG_STACK
	/* Write a known value to the IDLE thread stack to support stack
	 * monitoring logic
	 */

	adr		r3, .Lstkinit
	ldmia	r3, {r0, r1, r2}	/* R0 = start of IDLE stack; R1 = Size of tack; R2 = coloration */

2:								/* Top of the loop */
	sub		r1, r1, #1			/* R1 = Number of words remaining */
	cmp		r1, #0				/* Check (nwords == 0) */
	str		r2, [r0], #4		/* Save stack color word, increment stack address */
	bne		2b					/* Bottom of the loop */

#endif

  showprogress 'N'
	/* Finally branch to the OS entry point */

	mov		lr, #0				/* LR = return address (none) */
	b		os_start			/* Branch to os_start */

	/* Text-section constants:
	 *
	 *   _sbss is the start of the BSS region (see ld.script)
	 *   _ebss is the end of the BSS regsion (see ld.script)
	 *
	 * The idle task stack starts at the end of BSS and is of size
	 * CONFIG_IDLETHREAD_STACKSIZE.  The heap continues from there until the
	 * end of memory.  See g_idle_topstack below.
	 */

.global bssStart
bssStart:
.Linitparms:
	.long	_sbss
.global bssEnd
bssEnd:
	.long	_ebss
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE-4

#ifdef CONFIG_DEBUG_STACK
	.type	.Lstkinit, %object
.Lstkinit:
	.long	_ebss					/* Beginning of the IDLE stack, then words of IDLE stack */
	.long	(CONFIG_IDLETHREAD_STACKSIZE >> 2)
	.long	STACK_COLOR				/* Stack coloration word */
	.size	.Lstkinit, . -.Lstkinit
#endif
	.size	.Lvstart, .-.Lvstart

	 /* Data section variables */

	/* This global variable is unsigned long g_idle_topstack and is
	 * exported from here only because of its coupling to .Linitparms
	 * above.
	 */

	.data
	.align	4
	.globl	g_idle_topstack
	.type	g_idle_topstack, object
g_idle_topstack:
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
	.size	g_idle_topstack, .-g_idle_topstack
	.end
